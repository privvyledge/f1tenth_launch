#!/usr/bin/env python3
"""Loaders for a map set: the .pgm/.yaml occupancy grid and the .pcd cloud.

Split out of the retired `map_cloud_align.py` (deleted 2026-08-30). That file
also carried a one-way alignment score -- median distance from each occupied
grid cell to the nearest cloud point -- which is BIASED and cannot tell
alignment from density: the cloud is far denser than the grid and does not
cover its footprint, so any pose that packs grid cells into the dense region
scores well. It reported "aligned at zero shift" for three weeks on a map set
that was yawed 25.33 deg, and a spurious +31 deg optimum under a yaw sweep
(bug-265). The loaders here were the only honest part of it.

Score alignment with `map_cloud_align2.py`, which is symmetric.
"""
import re
import struct

import numpy as np
import yaml


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    if not data.startswith(b'P5'):
        raise ValueError('only binary P5 PGM is supported: %s' % path)
    # header: magic, width, height, maxval -- comments start with '#'
    tokens, i = [], 2
    while len(tokens) < 3:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while data[i:i + 1] not in (b'\n', b''):
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace():
            j += 1
        tokens.append(int(data[i:j]))
        i = j
    i += 1  # single whitespace after maxval
    w, h, maxval = tokens
    img = np.frombuffer(data[i:i + w * h], dtype=np.uint8).reshape(h, w)
    return img, maxval


def grid_occupied_xy(pgm_path, yaml_path):
    meta = yaml.safe_load(open(yaml_path))
    img, maxval = read_pgm(pgm_path)
    h, w = img.shape
    res = float(meta['resolution'])
    ox, oy, _ = (list(meta['origin']) + [0, 0, 0])[:3]
    occ_thresh = float(meta.get('occupied_thresh', 0.65))
    negate = int(meta.get('negate', 0))
    p = img.astype(np.float64) / maxval
    # nav2 map_server trinary: occupancy = 1 - p for negate=0
    occ = p if negate else (1.0 - p)
    rows, cols = np.nonzero(occ >= occ_thresh)
    # row 0 of the image is the TOP row = max y
    x = ox + (cols + 0.5) * res
    y = oy + (h - rows - 0.5) * res
    return np.column_stack([x, y]), meta


def read_pcd_xyz(path):
    with open(path, 'rb') as f:
        raw = f.read()
    hdr_end = raw.index(b'DATA')
    nl = raw.index(b'\n', hdr_end)
    header = raw[:nl].decode('ascii', 'replace')
    body = raw[nl + 1:]
    fields = re.search(r'^FIELDS (.*)$', header, re.M).group(1).split()
    sizes = [int(v) for v in re.search(r'^SIZE (.*)$', header, re.M).group(1).split()]
    counts_m = re.search(r'^COUNT (.*)$', header, re.M)
    counts = [int(v) for v in counts_m.group(1).split()] if counts_m else [1] * len(fields)
    npts = int(re.search(r'^POINTS (\d+)$', header, re.M).group(1))
    fmt = re.search(r'^DATA (\S+)$', header, re.M).group(1)
    if fmt != 'binary':
        raise ValueError('only DATA binary is supported (got %s)' % fmt)
    stride = sum(s * c for s, c in zip(sizes, counts))
    off, offsets = 0, {}
    for name, s, c in zip(fields, sizes, counts):
        offsets[name] = off
        off += s * c
    arr = np.frombuffer(body[:npts * stride], dtype=np.uint8).reshape(npts, stride)
    out = []
    for name in ('x', 'y', 'z'):
        o = offsets[name]
        out.append(arr[:, o:o + 4].copy().view(np.float32).ravel().astype(np.float64))
    xyz = np.column_stack(out)
    return xyz[np.isfinite(xyz).all(axis=1)]
