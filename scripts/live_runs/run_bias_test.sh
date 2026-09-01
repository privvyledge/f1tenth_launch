#!/usr/bin/env bash
# Offline verification of imu_processors::ImuBiasRemover against real RealSense
# gyro data, with a synthetic velocity source. Run INSIDE the container.
# NOT set -u: /opt/ros/humble/setup.bash reads an unbound AMENT_TRACE_SETUP_FILES (bug-250)
set -o pipefail

BAG="${1:-/mnt/shared_dir/claude_bringup_0826/run18_constdt}"
OUT="${2:-/mnt/shared_dir/biastest_0826}"
# Seconds a velocity source may go silent before its "stationary" verdict is
# dropped. 0.0 (default) disables the check, which is upstream's behaviour and
# reproduces the 2026-08-26 hazard measurement. Any positive value requires the
# privvyledge/imu_pipeline fork. Stock imu_processors does not declare this
# parameter and SILENTLY IGNORES it -- verified 2026-08-26, the node starts
# normally and logs nothing -- so a stock build scored against a positive
# timeout looks configured and is not. Confirm with
#   ros2 param get /imu_bias_remover stationary_timeout
# which errors on a stock build and returns the value on the fork.
TIMEOUT="${3:-0.0}"
CSV="$OUT/bias_test.csv"
CFG=/workspaces/f1tenth/install/f1tenth_launch/share/f1tenth_launch/config/filters/imu_bias_remover.yaml

export ROS_DOMAIN_ID=7      # keep off the default domain other agents use
source /opt/ros/humble/setup.bash
source /workspaces/f1tenth/install/setup.bash
mkdir -p "$OUT"

echo "=== node under test (stationary_timeout=$TIMEOUT) ==="
setsid ros2 run imu_processors imu_bias_remover_node --ros-args \
    --params-file "$CFG" \
    -p stationary_timeout:="$TIMEOUT" \
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
