#!/bin/bash
# start_with_scan_context_localization.sh
#
# 启动流程:
#   Phase 1: bringup（Livox + FAST-LIO + RTAB-Map定位 + Nav2 autostart=false）
#   Phase 2: Scan Context 全局重定位 → 发布 /initialpose
#   Phase 3: 激活 Nav2 lifecycle 节点

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

DATABASE_PATH="${DATABASE_PATH:-/data/maps/site_a/rtabmap.db}"
BRINGUP_MODE="${BRINGUP_MODE:-navigation}"
ENABLE_RVIZ="${ENABLE_RVIZ:-false}"

BRINGUP_WAIT_SEC=10
INITIALPOSE_TIMEOUT_SEC=300

LOG_DIR="/tmp/nav_logs"
mkdir -p "$LOG_DIR"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --db)
            DATABASE_PATH="$2"
            shift 2
            ;;
        --mode)
            BRINGUP_MODE="$2"
            shift 2
            ;;
        --rviz)
            ENABLE_RVIZ="true"
            shift
            ;;
        *)
            echo "[WARN] Unknown arg: $1"
            shift
            ;;
    esac
done

. /opt/ros/humble/setup.bash
. "$WS_DIR/install/setup.bash"

echo "=============================================="
echo "  Scan Context Global Localization Bringup"
echo "  workspace : $WS_DIR"
echo "  database  : $DATABASE_PATH"
echo "  mode      : $BRINGUP_MODE"
echo "  rviz      : $ENABLE_RVIZ"
echo "=============================================="

BRINGUP_PGID=""
SCAN_PID=""

cleanup() {
    echo ""
    echo "[INFO] Shutting down ..."

    if [[ -n "$SCAN_PID" ]]; then
        kill "$SCAN_PID" 2>/dev/null || true
    fi

    if [[ -n "$BRINGUP_PGID" ]]; then
        kill -- -"$BRINGUP_PGID" 2>/dev/null || true
    fi

    sleep 2

    pkill -9 -f "scan_context_relocalization_node" 2>/dev/null || true
    pkill -9 -f "fastlio_mapping" 2>/dev/null || true
    pkill -9 -f "livox_ros_driver2_node" 2>/dev/null || true
    pkill -9 -f "rtabmap$" 2>/dev/null || true
    pkill -9 -f "hdl_global_localization_node" 2>/dev/null || true

    echo "[INFO] All processes stopped. Logs: $LOG_DIR"
}
trap cleanup EXIT INT TERM

echo ""
echo "【Phase 1】Bringup，Nav2 autostart=false"

setsid ros2 launch robot_bringup global_localization_bringup.launch.py \
    mode:="$BRINGUP_MODE" \
    database_path:="$DATABASE_PATH" \
    enable_rviz:="$ENABLE_RVIZ" \
    > "$LOG_DIR/bringup.log" 2>&1 &

BRINGUP_PID=$!
BRINGUP_PGID=$BRINGUP_PID

echo "  PID=$BRINGUP_PID  PGID=$BRINGUP_PGID"
echo "  log=$LOG_DIR/bringup.log"

echo "  Waiting ${BRINGUP_WAIT_SEC}s for FAST-LIO and RTAB-Map ..."
sleep "$BRINGUP_WAIT_SEC"

echo ""
echo "【Phase 2】Scan Context global localization"

ros2 run scan_context_relocalization scan_context_relocalization_node \
    > "$LOG_DIR/scan_context.log" 2>&1 &

SCAN_PID=$!

echo "  Scan Context PID=$SCAN_PID"
echo "  log=$LOG_DIR/scan_context.log"
echo "  Waiting for /initialpose ..."

if timeout "$INITIALPOSE_TIMEOUT_SEC" bash -c '
. /opt/ros/humble/setup.bash
. "'"$WS_DIR"'/install/setup.bash"

ros2 topic echo /initialpose --once \
> "'"$LOG_DIR"'/initialpose.log"
'
then
    echo "[OK] /initialpose received"

    sleep 3

    kill "$SCAN_PID" 2>/dev/null || true
    SCAN_PID=""

    echo ""
    echo "【Phase 3】Activating Nav2 ..."

    ros2 service call /lifecycle_manager_navigation/manage_nodes \
        nav2_msgs/srv/ManageLifecycleNodes "{command: 0}" 2>/dev/null \
        && echo "  Nav2 activated" \
        || echo "  [WARN] Nav2 activation failed — may already be active"

    echo ""
    echo "[OK] System running. Press Ctrl+C to stop everything."

    wait "$BRINGUP_PID"

else
    echo ""
    echo "[WARN] Scan Context timeout after ${INITIALPOSE_TIMEOUT_SEC}s"
    echo "[WARN] No /initialpose received"
    echo "[WARN] Bringup is still running. Check logs:"
    echo "       $LOG_DIR/scan_context.log"
    echo "       $LOG_DIR/bringup.log"
    echo ""
    echo "[INFO] Press Ctrl+C to stop everything."

    wait "$BRINGUP_PID"
fi
