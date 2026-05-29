#!/bin/bash
set -e

source /environment.sh || echo "[Launcher] Warning: /environment.sh not found"

dt-launchfile-init

#──────────────────────────────────────────────
# STEP 4: run your nodes
#──────────────────────────────────────────────
rosrun gesture_control tcp_server.py &
rosrun gesture_control motor_controller_node.py &
dt-exec roslaunch fpd_control fpd_lane_controller_node.launch veh:=$VEHICLE_NAME

#──────────────────────────────────────────────
# STEP 5: keep alive
#──────────────────────────────────────────────
dt-launchfile-join
