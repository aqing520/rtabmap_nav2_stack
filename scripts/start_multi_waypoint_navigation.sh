#!/usr/bin/env bash
# Start navigation with RViz clicked-point waypoint collection enabled.
#
# Usage:
#   scripts/start_multi_waypoint_navigation.sh
#   scripts/start_multi_waypoint_navigation.sh --map site_workspace --db maps/site_workspace/rtabmap.db --rviz

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

MAP_ID="${MAP_ID:-site_workspace}"
MAP_FRAME_ID="${MAP_FRAME_ID:-map}"
DATABASE_PATH="${DATABASE_PATH:-$WS_DIR/rtabmap_orbbec.db}"
ENABLE_RVIZ="${ENABLE_RVIZ:-true}"
MODE="${MODE:-navigation}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --map)
            MAP_ID="$2"
            DATABASE_PATH="$WS_DIR/maps/${MAP_ID}/rtabmap.db"
            shift 2
            ;;
        --db)
            DATABASE_PATH="$2"
            shift 2
            ;;
        --frame)
            MAP_FRAME_ID="$2"
            shift 2
            ;;
        --mode)
            MODE="$2"
            shift 2
            ;;
        --rviz)
            ENABLE_RVIZ="true"
            shift
            ;;
        --no-rviz)
            ENABLE_RVIZ="false"
            shift
            ;;
        -h|--help)
            sed -n '1,8p' "$0"
            exit 0
            ;;
        *)
            echo "[WARN] Unknown arg: $1"
            shift
            ;;
    esac
done

source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

echo "=============================================="
echo "  Multi-Waypoint Navigation"
echo "  workspace : $WS_DIR"
echo "  map_id    : $MAP_ID"
echo "  map_frame : $MAP_FRAME_ID"
echo "  database  : $DATABASE_PATH"
echo "  rviz      : $ENABLE_RVIZ"
echo "=============================================="

exec ros2 launch robot_bringup bringup.launch.py \
    mode:="$MODE" \
    database_path:="$DATABASE_PATH" \
    enable_rviz:="$ENABLE_RVIZ" \
    start_multi_waypoint_routes:=true \
    waypoint_map_id:="$MAP_ID" \
    waypoint_map_frame_id:="$MAP_FRAME_ID"
