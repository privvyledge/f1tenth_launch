# stage_0901.sh -- bring a container to git HEAD as of 2026-09-01 evening (f50e12d).
# Supersedes stage_0831.sh. Same path set (199 entries vs 198: adds
# scripts/live_runs/stage_0831.sh, now tracked; the map_cloud_align pyc that 0831
# deleted by hand is simply gone from the archive). Tarball md5 5a7cc874a49bfd8e6d62f970e69d6b12.
# Differences, all from today's four commits:
#   * 172dd48 exec bit restored on nine live_runs scripts. 71_mpc_stack.sh and
#     friends were mode 100644 despite having shebangs, so running one directly
#     gave "Permission denied". git archive preserves the mode, so unpacking fixes
#     it -- check 17 below asserts that.
#   * e25ebff stop_launch_tree() in 00_env.sh. Ctrl-C used to leave joy_node and
#     vesc_driver_node running (blocked in a device read, they never reach their
#     SIGINT handler), and `wait` hung forever. Orphans make the NEXT run look
#     like the launch file is duplicating subsystems.
#   * e25ebff DDS_PROFILE=velox1 selecting cyclonedds_velox1.xml, for runs where
#     velox1 (192.168.2.13, wired) publishes drive commands. NOTE DDS_PROFILE=lo
#     binds loopback only, so velox1 is invisible with NO error.
#   * e25ebff 10_preflight.sh fingerprints the cloud (bug-265 frame, bug-266 rgb).
#   * f50e12d stage_0831.sh is now tracked in the repo.
# The map set STILL ships inside the tarball and nothing is copied over it --
# keep it that way; grid, cloud and AMCL seed must move in lockstep.
# NO NETWORK, as 0831: the imu_pipeline fork comes from a staged tarball (bug-263).
# NOT set -u: /opt/ros/humble/setup.bash reads an unbound var (bug-250)
set -eo pipefail

TGZ=/mnt/shared_dir/f1tenth_stage_20260901.tgz
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
echo "staged (0901)."
