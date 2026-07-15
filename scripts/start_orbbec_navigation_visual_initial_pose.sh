#!/usr/bin/env bash
# Start Orbbec navigation using the independent visual_initial_pose package.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

DATABASE_PATH="${DATABASE_PATH:-$WS_DIR/rtabmap_orbbec.db}"
SENSOR_PROFILE="${SENSOR_PROFILE:-lidar_rgbd}"
ENABLE_RVIZ="${ENABLE_RVIZ:-true}"
ENABLE_RTABMAP_VIZ="${ENABLE_RTABMAP_VIZ:-false}"
ALLOW_LAST_POSE_FALLBACK="${ALLOW_LAST_POSE_FALLBACK:-false}"

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
  echo "[ERROR] cuda_mppi_controller is not available." >&2
  exit 2
fi

if [[ ! -f "$DATABASE_PATH" ]]; then
  echo "[ERROR] RTAB-Map database not found: $DATABASE_PATH" >&2
  exit 3
fi

echo "============================================================"
echo "  Orbbec navigation with independent visual initial pose"
echo "  database                 : $DATABASE_PATH"
echo "  sensor profile           : $SENSOR_PROFILE"
echo "  allow last-pose fallback : $ALLOW_LAST_POSE_FALLBACK"
echo "============================================================"

exec ros2 launch visual_initial_pose \
  orbbec_navigation_visual_initial_pose.launch.py \
  database_path:="$DATABASE_PATH" \
  sensor_profile:="$SENSOR_PROFILE" \
  enable_rviz:="$ENABLE_RVIZ" \
  enable_rtabmap_viz:="$ENABLE_RTABMAP_VIZ" \
  allow_last_pose_fallback:="$ALLOW_LAST_POSE_FALLBACK" \
  "$@"
