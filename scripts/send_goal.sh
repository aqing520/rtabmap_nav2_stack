#!/usr/bin/env bash
# Usage: ./send_goal.sh <x> <y> [yaw_deg]
#   x, y   : target position in map frame (metres)
#   yaw_deg: optional heading in degrees (default 0)
#
# Example: ./send_goal.sh 2.0 1.0
#          ./send_goal.sh 2.0 1.0 90

set -e

X=${1:?Usage: $0 <x> <y> [yaw_deg]}
Y=${2:?Usage: $0 <x> <y> [yaw_deg]}
YAW_DEG=${3:-0}

# Convert yaw from degrees to quaternion (rotation around Z only)
# qz = sin(yaw/2), qw = cos(yaw/2)
YAW_RAD=$(python3 -c "import math; print(math.radians($YAW_DEG))")
QZ=$(python3 -c "import math; print(math.sin(math.radians($YAW_DEG)/2))")
QW=$(python3 -c "import math; print(math.cos(math.radians($YAW_DEG)/2))")

echo "Sending goal: x=$X y=$Y yaw=${YAW_DEG}deg  (qz=$QZ qw=$QW)"

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}}}}"
