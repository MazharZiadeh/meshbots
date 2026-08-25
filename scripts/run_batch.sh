#!/usr/bin/env bash
# Monte Carlo evaluation: N full delivery missions with different noise seeds.
#
#   ./scripts/run_batch.sh [N_RUNS] [OUT_DIR] [SECONDS_PER_RUN]
#
# Each run gets its own seed and log directory. Afterwards:
#   ros2 run meshbots batch_metrics --dir <OUT_DIR> --plot batch.png
# (no `set -u`: ROS setup.bash trips over unbound variables)
N=${1:-6}
BASE=${2:-/tmp/meshbots_batch}
DURATION=${3:-280}

WS="$(cd "$(dirname "$0")/.." && pwd)/ros2_ws"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

mkdir -p "$BASE"
for i in $(seq 1 "$N"); do
  d="$BASE/run_$i"
  rm -rf "$d"; mkdir -p "$d"
  echo "=== run $i/$N (seed $i) -> $d ==="
  timeout "$DURATION" ros2 launch meshbots sim.launch.py \
    gui:=false rviz:=false seed:="$i" eval_dir:="$d" \
    > "$d/launch.log" 2>&1
  # Unique targets delivered (log lines repeat per redundant broadcast).
  grep -o "DELIVERED target [0-9]*" "$d/launch.log" | sort -u | wc -l \
    > "$d/deliveries.txt"
  echo "    delivered: $(cat "$d/deliveries.txt")/3"
  pkill -f "gz sim" 2>/dev/null
  sleep 3
done
echo "batch complete: $BASE"
