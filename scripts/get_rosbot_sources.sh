#!/usr/bin/env bash
# Vendored ROSbot chassis sources (not tracked in this repo).
cd "$(dirname "$0")/../ros2_ws/src"
git clone --depth 1 -b jazzy https://github.com/husarion/rosbot_ros.git
git clone --depth 1 https://github.com/husarion/husarion_gz_worlds.git
for p in rosbot_moveit rosbot_hardware_interfaces rosbot rosbot_bringup husarion_asset_server; do touch "rosbot_ros/$p/COLCON_IGNORE"; done
