# stage_0902.sh -- bring a container to git HEAD as of 2026-09-01 night (a2eb260).
# Supersedes stage_0901.sh. 200 entries vs 199: adds scripts/live_runs/stage_0901.sh,
# now tracked. Tarball md5 29ccc04e42e0f5a3d58ab469e44c8669.
# Difference from 0901 is one commit, a2eb260, and it matters for every driven bag:
#   * bug-272. Recording the `drive` topic set writes ~125 MB/s and starves
#     librealsense's USB thread. ONLY infra1/infra2 stop -- colour, depth, the
#     pointcloud and camera/imu keep full rate -- and Isaac VSLAM consumes the IR
#     stereo pair, so it dies in lockstep: 107 VSLAM messages over 214 s against
#     the EKF's 6415. It is SILENT: visual_slam_node never crashes and reads 30 Hz
#     before and after, so a parked rate check passes while ekf_odom loses odom1
#     for the whole drive. topic_sets.sh gains TOPICS_CAMERA_IMU and a `sysid`
#     bundle (no image streams); 25_drive_session.sh gains a BAG_TOPIC_SET env
#     override, defaulting to `drive` so old bags stay interchangeable.
#     Measured: BAG_TOPIC_SET=sysid gives 47 MB instead of 26 GiB with VSLAM at
#     30.02 Hz. Verify VSLAM survived a bag by MESSAGE COUNT in `ros2 bag info`,
#     never by topic rate.
# The map set STILL ships inside the tarball and nothing is copied over it --
# keep it that way; grid, cloud and AMCL seed must move in lockstep.
# NO NETWORK, as 0831: the imu_pipeline fork comes from a staged tarball (bug-263).
# NOT set -u: /opt/ros/humble/setup.bash reads an unbound var (bug-250)
set -eo pipefail

TGZ=/mnt/shared_dir/f1tenth_stage_20260902.tgz
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
echo -n "  amcl seed yaw      : "; grep -o "yaw: -1.4748" $I/config/localization/localizer_amcl.yaml || echo MISSING
echo -n "  bug-248 constant_dt: "; grep -o "imu_filter_constant_dt': '0.005'" $I/launch/sensors/realsense_d435i.launch.py || echo MISSING
echo -n "  bug-244 imu1 accel : "; grep -o "ax, ay, az DISABLED" $I/config/localization/ekf_odom.yaml || echo MISSING
echo -n "  max_steering       : "; grep -o "max_steering', default=0.314" $I/launch/bringup.launch.py || echo MISSING
echo -n "  nav2 steer limit   : "; grep -o "max_steering_angle': 0.314" $I/launch/vehicle/vehicle.launch.py || echo MISSING
echo -n "  stationary_timeout : "; grep -o "stationary_timeout: 0.5" $I/config/filters/imu_bias_remover.yaml || echo MISSING
echo -n "  fork param in .so  : "; strings $W/install/imu_processors/lib/libimu_bias_remover.so | grep -c "^stationary_timeout$"
echo -n "  remove_imu_bias    : "; grep -o "'remove_imu_bias': '[A-Za-z]*'" $I/launch/sensors/realsense_d435i.launch.py
echo -n "  container executor : "; grep -o "container_multi_threaded', default='False'" $I/launch/bringup.launch.py || echo MISSING
echo -n "  no MTE flag        : "; grep -c "use_multi_threaded_executor" $I/launch/bringup.launch.py
echo -n "  movement_time_allow: "; grep -o "movement_time_allowance: 10.0" $I/config/nav2_params.yaml || echo MISSING
echo -n "  bug-260 tmr gate   : "; grep -o "IfCondition(launch_localization_string)" $I/launch/teleop.launch.py || echo MISSING
echo -n "  bug-261 pcloud QoS : "; grep -c "ParticleCloud" $I/config/f1tenth.rviz
echo -n "  installed maps     : "; ls $I/data/maps/20260805/ | tr "\n" " "; echo
echo -n "  bug-265 cloud pts  : "; grep -a -m1 "^POINTS" $I/data/maps/20260805/cloud_voxel_0p05.pcd
echo -n "  bug-266 cloud rgb  : "; grep -a -m1 "^FIELDS" $I/data/maps/20260805/cloud_voxel_0p05.pcd
echo -n "  one-way metric gone: "; test ! -e $I/scripts/analysis/map_cloud_align.py && echo "deleted" || echo "STILL PRESENT"
echo -n "  bug-269 teardown   : "; grep -o "stop_launch_tree()" $S/scripts/live_runs/00_env.sh | head -1 || echo MISSING
echo -n "  velox1 DDS profile : "; grep -o "static|lo|velox1" $S/scripts/live_runs/00_env.sh | head -1 || echo MISSING
echo -n "  bug-265 preflight  : "; grep -o "NOT in the grid frame" $S/scripts/live_runs/10_preflight.sh | head -1 || echo MISSING
echo -n "  exec bit (bug-268) : "; test -x $S/scripts/live_runs/71_mpc_stack.sh && echo "71_mpc_stack.sh executable" || echo "STILL NOT EXECUTABLE"
echo -n "  stage_0831 tracked : "; test -f $S/scripts/live_runs/stage_0831.sh && echo present || echo MISSING
echo -n "  bug-272 sysid set  : "; grep -c "^    sysid)" $S/scripts/live_runs/topic_sets.sh
echo -n "  bug-272 no images  : "; awk '/^    sysid\)/,/^      ;;/' $S/scripts/live_runs/topic_sets.sh | grep -c 'TOPICS_CAMERA"'
echo -n "  bug-272 env override: "; grep -c "BAG_TOPIC_SET" $S/scripts/live_runs/25_drive_session.sh
echo "staged (0902)."
