#!/usr/bin/env bash
# Hand-off: wait for campaign 3, merge the rf-realism branch, rebuild, smoke
# test the new channel knobs, then run the sensitivity campaign.
cd "$(dirname "$0")/.."
R=$PWD/results
while ! grep -q CAMPAIGN3_DONE "$R/campaign3.log"; do sleep 20; done
sleep 5
echo "=== campaign 3 done, merging rf-realism ==="
git merge -q --no-edit rf-realism || { echo MERGE_FAILED; exit 1; }
source /opt/ros/jazzy/setup.bash
(cd ros2_ws && colcon build --symlink-install --packages-select meshbots) \
  > "$R/build_c4.log" 2>&1 || { echo BUILD_FAILED; exit 1; }
source ros2_ws/install/setup.bash
mkdir -p "$R/smoke_c4"
timeout 70 ros2 launch meshbots sim.launch.py gui:=false rviz:=false seed:=3 \
  eval_dir:="$R/smoke_c4" rf_sigma_db:=4.0 rf_fading_db:=3.0 rf_offset_db:=3.0 \
  rf_n_exp:=2.8 > "$R/smoke_c4/launch.log" 2>&1
pkill -f "gz sim"; sleep 3
if grep -qi traceback "$R/smoke_c4/launch.log"; then echo SMOKE_FAILED; exit 1; fi
if ! grep -q "device offsets" "$R/smoke_c4/launch.log"; then echo SMOKE_NO_OFFSETS; exit 1; fi
echo "=== smoke ok, starting campaign 4 ==="
./scripts/run_sensitivity.sh
