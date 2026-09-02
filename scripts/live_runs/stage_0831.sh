# stage_0831.sh -- bring a container to git HEAD as of 2026-08-30 evening (a8e027f).
# Supersedes stage_0830b.sh. Differences:
#   * package tarball is f1tenth_stage_20260831.tgz (198 entries, git archive HEAD),
#     adding a51bdc6 (bug-265: the cloud is re-expressed in the grid's frame) and
#     a8e027f (bug-265: map_cloud_align.py, the one-way metric, deleted).
#   * THE MAP SET NOW SHIPS INSIDE THE TARBALL. 0830b copied it out of
#     /mnt/shared_dir/maps/20260805/ AFTER unpacking, so the SSD copy silently won
#     over whatever git HEAD held -- exactly the drift that would have reverted the
#     bug-265 cloud fix. data/maps/20260805 is now in the git archive path set and
#     nothing is copied over it. Keep it that way: the grid, the cloud and the AMCL
#     seed must move in lockstep, and one git commit is the only thing that
#     guarantees it.
#   * fifteenth and sixteenth checks: the cloud is the de-rotated one (96376 points)
#     and it still carries colour (FIELDS x y z rgb -- bug-266).
#   * NO NETWORK, as 0830b: the imu_pipeline fork comes from a staged tarball
#     because cloning it fails on the lab network (bug-263).
# NOT set -u: /opt/ros/humble/setup.bash reads an unbound var (bug-250)
set -eo pipefail

TGZ=/mnt/shared_dir/f1tenth_stage_20260831.tgz
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

echo "=== 4. verify in the INSTALLED tree ==="
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
echo "staged (0831)."
