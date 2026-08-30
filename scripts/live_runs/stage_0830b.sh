# stage_0830b.sh -- bring a container to git HEAD as of 2026-08-30 (9f97980).
# Supersedes stage_0830.sh. Differences:
#   * package tarball is f1tenth_stage_20260830b.tgz (189 entries, git archive HEAD),
#     which adds 7 commits on top of 0830's e07c2e1 -- notably 07c5149, the
#     f1tenth.rviz ParticleCloud display (Best Effort; bug-261), and 9f97980,
#     the stagecheck guards.
#   * NO NETWORK. stage_0830.sh's step 2 cloned privvyledge/imu_pipeline from
#     github and the lab network fails that clone (TCP 443 connects, the transfer
#     then times out at ~130 s). The same tree is staged as a tarball instead:
#     imu_pipeline_58d227e.tgz == humble-devel @ 58d227e, the commit that adds
#     ImuBiasRemover's stationary_timeout (bug-251).
# NOT set -u: /opt/ros/humble/setup.bash reads an unbound var (bug-250)
set -eo pipefail

TGZ=/mnt/shared_dir/f1tenth_stage_20260830b.tgz
IMU=/mnt/shared_dir/imu_pipeline_58d227e.tgz
MAPS=/mnt/shared_dir/maps/20260805
W=/workspaces/f1tenth
S=$W/src/f1tenth_launch
I=$W/install/f1tenth_launch/share/f1tenth_launch

echo "=== 1. package tarball ==="
cd $S && tar xzf $TGZ
mkdir -p $S/data/maps/20260805
cp $MAPS/cloud_voxel_0p05.pcd $MAPS/rtabmap_2d_final.pgm $MAPS/rtabmap_2d_final.yaml $S/data/maps/20260805/

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
echo -n "  src maps           : "; ls $S/data/maps/20260805/ | tr "\n" " "; echo
echo "staged (0830b)."
