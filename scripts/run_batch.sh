#!/usr/bin/env bash
# Monte Carlo evaluation: N full delivery missions with different noise seeds.
#
#   ./scripts/run_batch.sh [N_RUNS] [OUT_DIR] [SECONDS_PER_RUN] [EXTRA_ARGS...]
#   e.g. ./scripts/run_batch.sh 8 results/informative 280 formation:=informative
#
# Each run gets its own seed and log directory. Afterwards:
#   ros2 run meshbots batch_metrics --dir <OUT_DIR> --plot batch.png
#   ros2 run meshbots batch_metrics --compare <DIR_A> <DIR_B>
# Default output lives under the repo's results/ (not /tmp: a reboot wipes it).
# (no `set -u`: ROS setup.bash trips over unbound variables)
N=${1:-6}
BASE=${2:-$(cd "$(dirname "$0")/.." && pwd)/results/batch}
DURATION=${3:-280}
shift 3 2>/dev/null
EXTRA="$@"

WS="$(cd "$(dirname "$0")/.." && pwd)/ros2_ws"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

mkdir -p "$BASE"
for i in $(seq 1 "$N"); do
  d="$BASE/run_$i"
  rm -rf "$d"; mkdir -p "$d"
  echo "=== run $i/$N (seed $i) -> $d ==="
  timeout "$DURATION" ros2 launch meshbots sim.launch.py \
    gui:=false rviz:=false seed:="$i" eval_dir:="$d" $EXTRA \
    > "$d/launch.log" 2>&1
  # Unique targets delivered (log lines repeat per redundant broadcast).
  grep -o "DELIVERED target [0-9]*" "$d/launch.log" | sort -u | wc -l \
    > "$d/deliveries.txt"
  echo "    delivered: $(cat "$d/deliveries.txt")/3"
  pkill -f "gz sim" 2>/dev/null
  sleep 3
done
echo "batch complete: $BASE"
