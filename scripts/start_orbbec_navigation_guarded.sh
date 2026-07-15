#!/usr/bin/env bash
# Start Orbbec + Livox navigation with RTAB-Map startup localization gating.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

DATABASE_PATH="${DATABASE_PATH:-$WS_DIR/rtabmap_orbbec.db}"
SENSOR_PROFILE="${SENSOR_PROFILE:-lidar_rgbd}"
ENABLE_RVIZ="${ENABLE_RVIZ:-true}"
ALLOW_LAST_POSE_FALLBACK="${ALLOW_LAST_POSE_FALLBACK:-false}"

# ROS/colcon setup files may reference optional variables before defining
# them, which is incompatible with `set -u`. Temporarily disable nounset while
# sourcing the environment, then restore it for the rest of this script.
set +u
source /opt/ros/humble/setup.bash

if [[ -n "${CUDA_ROBOTICS_SETUP:-}" ]]; then
  source "$CUDA_ROBOTICS_SETUP"
elif [[ -f "$HOME/cuda_robotics_ws/install/setup.bash" ]]; then
  source "$HOME/cuda_robotics_ws/install/setup.bash"
fi

source "$WS_DIR/install/setup.bash"
set -u

if ! ros2 pkg prefix cuda_mppi_controller >/dev/null 2>&1; then
  echo "[ERROR] cuda_mppi_controller is not available in the sourced environment." >&2
  echo "        Set CUDA_ROBOTICS_SETUP=/path/to/cuda_ws/install/setup.bash." >&2
  exit 2
fi

echo "============================================================"
echo "  Orbbec navigation with RTAB-Map startup localization guard"
echo "  database                 : $DATABASE_PATH"
echo "  sensor profile           : $SENSOR_PROFILE"
echo "  allow last-pose fallback : $ALLOW_LAST_POSE_FALLBACK"
echo "============================================================"

exec ros2 launch robot_bringup bringup_orbbec.launch.py \
  mode:=navigation \
  sensor_profile:="$SENSOR_PROFILE" \
  start_livox:=true \
  start_camera:=true \
  use_sim_time:=false \
  database_path:="$DATABASE_PATH" \
  enable_rviz:="$ENABLE_RVIZ" \
  enable_startup_localization_guard:=true \
  allow_last_pose_fallback:="$ALLOW_LAST_POSE_FALLBACK" \
  "$@"
