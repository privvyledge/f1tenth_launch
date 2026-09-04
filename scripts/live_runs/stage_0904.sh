# stage_0904.sh -- bring a container to git HEAD as of 2026-09-04.
# Supersedes stage_0902.sh. 203 entries vs 200: adds scripts/live_runs/stage_0902.sh,
# scripts/live_runs/live_rates.py and scripts/analysis/plot_rates.py.
# Tarball md5 a7fc0807faee8ced4c06c90f5451917f.
#
# THIS STAGE IS MEANT TO BE COMMITTED TO AN IMAGE, which the previous ones were
# not. Two consequences:
#   * Do not leave host-specific state in the container. The tarballs it reads
#     live on the bind mount (/mnt/shared_dir), not in the image, so nothing to
#     clean; but if you add a step, keep it that way.
#   * Verify BEFORE `docker commit`, not after. A committed image with a bad
#     tree is much more expensive to walk back than a re-run of this script.
#
# What is new since 0902:
#   * The display check. librealsense opens a GL window, and a RealSense that
#     cannot reach an AUTHORIZED display fails to start while every other check
#     in the suite passes -- VESC, LiDAR, both EKFs, all four TF edges and AMCL
#     lifecycle all report ok with camera and VSLAM dead. Measured on four
#     launches 2026-09-03. The old prose blamed lost X11 bind mounts; the
#     observed failure had the mounts perfectly correct and Xorg still refused,
#     because it authenticates against the display manager's cookie, which no
#     ssh session can read. require_gl_display() in 00_env.sh probes with
#     xdpyinfo, which tells the two cases apart. Wired into 10_, 20_, 21_, 71_.
#   * 71_mpc_stack.sh accepts trailing name:=value tokens and passes them to
#     bringup, so a one-off does not need the file edited (which would lose the
#     health checks and the stop_launch_tree teardown).
#   * live_rates.py: achieved topic rates on a running stack, metered through
#     each image stream's paired camera_info so measuring does not perturb.
#   * bag_stats.py gains a pure-Python mcap fallback for ROS-less workstations.
# The map set STILL ships inside the tarball and nothing is copied over it --
# keep it that way; grid, cloud and AMCL seed must move in lockstep.
# NO NETWORK, as 0831: the imu_pipeline fork comes from a staged tarball (bug-263).
# NOT set -u: /opt/ros/humble/setup.bash reads an unbound var (bug-250)
#
# The verify block below uses cnt()/has(), NOT bare `grep -c`. This matters:
# `grep -c` exits 1 when it finds zero matches, and several of these checks PASS
# on zero -- "the sysid topic set contains no image topics" is the whole point
# of bug-272. Under `set -e` a passing check therefore killed the script.
# stage_0902.sh has that bug and has been aborting at its own `bug-272 no
# images` line, never reaching "staged (0902)."; everything before it passed, so
# nobody noticed. Do not reintroduce a bare `grep -c` here.
set -eo pipefail

# cnt <pattern> <file...> -- match count, 0 instead of a set -e abort.
cnt() { local pat="$1"; shift; grep -c "$pat" "$@" 2>/dev/null || true; }
# has <pattern> <file> -- print the matched text, or MISSING. Never fails.
has() { grep -o "$1" "$2" 2>/dev/null | head -1 || true; }

TGZ=/mnt/shared_dir/f1tenth_stage_20260904.tgz
IMU=/mnt/shared_dir/imu_pipeline_58d227e.tgz
W=/workspaces/f1tenth
S=$W/src/f1tenth_launch
I=$W/install/f1tenth_launch/share/f1tenth_launch

echo "=== 1. package tarball (carries data/maps/20260805 -- do NOT copy over it) ==="
cd $S && tar xzf $TGZ
rm -f $S/scripts/analysis/map_cloud_align.py $S/scripts/analysis/__pycache__/map_cloud_align*.pyc

echo "=== 2. imu_processors from the staged fork tarball (no network) ==="
cd $W/src
rm -rf imu_pipeline imu_pipeline_stage
tar xzf $IMU
mv imu_pipeline_stage imu_pipeline
git config --global --add safe.directory $W/src/imu_pipeline || true
git -C imu_pipeline log --oneline -1

echo "=== 3. build ==="
source /opt/ros/humble/setup.bash
cd $W
colcon build --symlink-install --packages-select imu_processors f1tenth_launch \
  --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -6

echo "=== 4. verify (config/launch from the INSTALLED tree; scripts/ is not installed, so from src) ==="
echo -n "  amcl seed yaw      : "; { v=$(has "yaw: -1.4748" $I/config/localization/localizer_amcl.yaml); echo "${v:-MISSING}"; }
echo -n "  bug-248 constant_dt: "; { v=$(has "imu_filter_constant_dt': '0.005'" $I/launch/sensors/realsense_d435i.launch.py); echo "${v:-MISSING}"; }
echo -n "  bug-244 imu1 accel : "; { v=$(has "ax, ay, az DISABLED" $I/config/localization/ekf_odom.yaml); echo "${v:-MISSING}"; }
echo -n "  max_steering       : "; { v=$(has "max_steering', default=0.314" $I/launch/bringup.launch.py); echo "${v:-MISSING}"; }
echo -n "  nav2 steer limit   : "; { v=$(has "max_steering_angle': 0.314" $I/launch/vehicle/vehicle.launch.py); echo "${v:-MISSING}"; }
echo -n "  stationary_timeout : "; { v=$(has "stationary_timeout: 0.5" $I/config/filters/imu_bias_remover.yaml); echo "${v:-MISSING}"; }
echo -n "  fork param in .so  : "; { n=$(strings $W/install/imu_processors/lib/libimu_bias_remover.so 2>/dev/null | grep -c "^stationary_timeout$" || true); echo "${n:-0}"; }
echo -n "  remove_imu_bias    : "; { v=$(has "'remove_imu_bias': '[A-Za-z]*'" $I/launch/sensors/realsense_d435i.launch.py); echo "${v:-MISSING}"; }
echo -n "  container executor : "; { v=$(has "container_multi_threaded', default='False'" $I/launch/bringup.launch.py); echo "${v:-MISSING}"; }
echo -n "  no MTE flag        : "; cnt "use_multi_threaded_executor" $I/launch/bringup.launch.py
echo -n "  movement_time_allow: "; { v=$(has "movement_time_allowance: 10.0" $I/config/nav2_params.yaml); echo "${v:-MISSING}"; }
echo -n "  bug-260 tmr gate   : "; { v=$(has "IfCondition(launch_localization_string)" $I/launch/teleop.launch.py); echo "${v:-MISSING}"; }
echo -n "  bug-261 pcloud QoS : "; cnt "ParticleCloud" $I/config/f1tenth.rviz
echo -n "  installed maps     : "; ls $I/data/maps/20260805/ | tr "\n" " "; echo
echo -n "  bug-265 cloud pts  : "; { v=$(grep -a -m1 "^POINTS" $I/data/maps/20260805/cloud_voxel_0p05.pcd || true); echo "${v:-MISSING}"; }
echo -n "  bug-266 cloud rgb  : "; { v=$(grep -a -m1 "^FIELDS" $I/data/maps/20260805/cloud_voxel_0p05.pcd || true); echo "${v:-MISSING}"; }
echo -n "  one-way metric gone: "; test ! -e $I/scripts/analysis/map_cloud_align.py && echo "deleted" || echo "STILL PRESENT"
echo -n "  bug-269 teardown   : "; { v=$(has "stop_launch_tree()" $S/scripts/live_runs/00_env.sh); echo "${v:-MISSING}"; }
echo -n "  velox1 DDS profile : "; { v=$(has "static|lo|velox1" $S/scripts/live_runs/00_env.sh); echo "${v:-MISSING}"; }
echo -n "  bug-265 preflight  : "; { v=$(has "NOT in the grid frame" $S/scripts/live_runs/10_preflight.sh); echo "${v:-MISSING}"; }
echo -n "  exec bit (bug-268) : "; test -x $S/scripts/live_runs/71_mpc_stack.sh && echo "71_mpc_stack.sh executable" || echo "STILL NOT EXECUTABLE"
echo -n "  stage_0902 tracked : "; test -f $S/scripts/live_runs/stage_0902.sh && echo present || echo MISSING
echo -n "  bug-272 sysid set  : "; cnt "^    sysid)" $S/scripts/live_runs/topic_sets.sh
echo -n "  bug-272 no images  : "; { n=$(awk '/^    sysid\)/,/^      ;;/' $S/scripts/live_runs/topic_sets.sh | grep -c 'TOPICS_CAMERA"' || true); echo "${n:-0}"; }
echo -n "  bug-272 env override: "; cnt "BAG_TOPIC_SET" $S/scripts/live_runs/25_drive_session.sh
echo "--- new in 0904 ---"
echo -n "  display check fn   : "; { v=$(has "^require_gl_display()" $S/scripts/live_runs/00_env.sh); echo "${v:-MISSING}"; }
echo -n "  display in 10_/20_/21_/71_ : "; { grep -l "require_gl_display" \
  $S/scripts/live_runs/10_preflight.sh $S/scripts/live_runs/20_sensor_bag.sh \
  $S/scripts/live_runs/21_detection_dataset_bag.sh $S/scripts/live_runs/71_mpc_stack.sh 2>/dev/null || true; } | wc -l
echo -n "  EXPECT_CAMERA gate : "; cnt "EXPECT_CAMERA" $S/scripts/live_runs/10_preflight.sh
echo -n "  71_ extra args     : "; cnt "EXTRA_LAUNCH_ARGS" $S/scripts/live_runs/71_mpc_stack.sh
echo -n "  live_rates.py exec : "; test -x $S/scripts/live_runs/live_rates.py && echo executable || echo "NOT EXECUTABLE"
echo -n "  plot_rates.py exec : "; test -x $S/scripts/analysis/plot_rates.py && echo executable || echo "NOT EXECUTABLE"
echo -n "  bag_stats mcap fb  : "; cnt "read_timestamps_mcap" $S/scripts/analysis/bag_stats.py
echo "staged (0904)."
