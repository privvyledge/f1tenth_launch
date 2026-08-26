#!/usr/bin/env bash
# Offline verification of imu_processors::ImuBiasRemover against real RealSense
# gyro data, with a synthetic velocity source. Run INSIDE the container.
# NOT set -u: /opt/ros/humble/setup.bash reads an unbound AMENT_TRACE_SETUP_FILES (bug-250)
set -o pipefail

BAG="${1:-/mnt/shared_dir/claude_bringup_0826/run18_constdt}"
OUT="${2:-/mnt/shared_dir/biastest_0826}"
CSV="$OUT/bias_test.csv"
CFG=/workspaces/f1tenth/install/f1tenth_launch/share/f1tenth_launch/config/filters/imu_bias_remover.yaml

export ROS_DOMAIN_ID=7      # keep off the default domain other agents use
source /opt/ros/humble/setup.bash
source /workspaces/f1tenth/install/setup.bash
mkdir -p "$OUT"

echo "=== node under test ==="
setsid ros2 run imu_processors imu_bias_remover_node --ros-args \
    --params-file "$CFG" \
    -r imu:=/test/imu -r imu_biased:=/test/imu_biased \
    -r bias:=/test/bias -r odom:=/test/odom \
    > "$OUT/node.log" 2>&1 &
NODE=$!
sleep 5
kill -0 $NODE 2>/dev/null || { echo "node died:"; cat "$OUT/node.log"; exit 1; }
echo "node pid $NODE"

echo "=== driver (phases A/B/C/D, 70 s) ==="
setsid python3 "$OUT/bias_test_driver.py" "$CSV" > "$OUT/driver.log" 2>&1 &
DRV=$!
sleep 2

echo "=== bag play: real /camera/imu ==="
setsid ros2 bag play "$BAG" --topics /camera/imu > "$OUT/play.log" 2>&1 &
PLAY=$!

# driver self-terminates at 70 s
for i in $(seq 1 100); do kill -0 $DRV 2>/dev/null || break; sleep 2; done
kill -0 $DRV 2>/dev/null && { kill -INT $DRV; sleep 5; }

kill -INT -- -"$PLAY" 2>/dev/null; sleep 2
kill -INT -- -"$NODE" 2>/dev/null; sleep 3
kill -9 -- -"$PLAY" -- -"$NODE" 2>/dev/null

echo "=== driver log ==="; cat "$OUT/driver.log"
echo "=== node log ==="; grep -vE "ddsi|^$" "$OUT/node.log" | head -10
echo "=== survivors ==="; ps -eo pid,args --no-headers | grep "[i]mu_bias_remover\|[b]ag play" || echo "  none"
