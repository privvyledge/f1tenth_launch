# Video assets

**Video files are never committed** (plan §7.8; `.gitignore` keeps only this file).

Canonical locations:

| Where | Path |
|---|---|
| Robot (capture host) | `gosling1:/mnt/shared_dir/videos/<date>/` |
| OneDrive copy | `.../Autonole/f1tenth_launch/presentation_video/<date>/` |
| Local, for a build | drop the files here as `assets/video/<ASSET-ID>.mp4` |

Slides embed video with a relative path, so an HTML export plays it when this
folder is populated and the PDF export shows the poster frame:

```html
<video src="../assets/video/VID-LOC-LOOP.mp4" controls width="900"></video>
```

Every ID here must appear in `../../ASSETS.md`. Until the file exists, the slide
carries a placeholder card instead — see the conventions in any brief.

2024 clips that may be reused **with a visible "2024 stack" label** live in the
repo already: `docs/2d_mapping.mp4`, `docs/3d_mapping.mp4`,
`docs/perception_and_gotogoal_planning.mp4`.
