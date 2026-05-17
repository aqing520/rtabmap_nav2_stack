#!/bin/bash
# start_with_global_localization.sh
#
# 启动流程:
#   Phase 1: bringup（Livox + FAST-LIO + RTAB-Map定位 + Nav2 autostart=false）
#   Phase 2: 全局定位 → 发布 /initialpose（TRANSIENT_LOCAL留存）
#   Phase 3: 激活 Nav2 lifecycle 节点
#
# RTAB-Map 随 bringup 立即启动，以 --RGBD/StartAtOrigin true 从原点发布
# map→odom TF；/initialpose 到达后 RTAB-Map 重置到正确位置，Nav2 此后激活。
#
# 用法:
#   ./scripts/start_with_global_localization.sh
#   ./scripts/start_with_global_localization.sh --db /data/maps/site_a/rtabmap.db
#   ./scripts/start_with_global_localization.sh --engine BBS

set -eo pipefail

# ── 可配置参数 ────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

DATABASE_PATH="${DATABASE_PATH:-/data/maps/site_a/rtabmap.db}"
ENGINE="${ENGINE:-FPFH_RANSAC}"
BRINGUP_MODE="${BRINGUP_MODE:-navigation}"
ENABLE_RVIZ="${ENABLE_RVIZ:-false}"

BRINGUP_WAIT_SEC=10   # 等 FAST-LIO 发布第一帧扫描

LOG_DIR="/tmp/nav_logs"
mkdir -p "$LOG_DIR"
# ─────────────────────────────────────────────────────────────

while [[ $# -gt 0 ]]; do
    case "$1" in
        --db)     DATABASE_PATH="$2"; shift 2 ;;
        --engine) ENGINE="$2";        shift 2 ;;
        --mode)   BRINGUP_MODE="$2";  shift 2 ;;
        --rviz)   ENABLE_RVIZ="true"; shift   ;;
        *) echo "[WARN] Unknown arg: $1"; shift  ;;
    esac
done

. /opt/ros/humble/setup.bash
. "$WS_DIR/install/setup.bash"

echo "=============================================="
echo "  Global Localization Bringup"
echo "  workspace : $WS_DIR"
echo "  database  : $DATABASE_PATH"
echo "  engine    : $ENGINE"
echo "  mode      : $BRINGUP_MODE"
echo "=============================================="

BRINGUP_PGID=""

cleanup() {
    echo ""
    echo "[INFO] Shutting down ..."
    # kill 整个进程组（hdl_node / rtabmap / fastlio / livox 全包含）
    [[ -n "$BRINGUP_PGID" ]] && kill -- -"$BRINGUP_PGID" 2>/dev/null
    sleep 2
    # 兜底：强制清理可能残留的节点
    pkill -9 -f "fastlio_mapping"              2>/dev/null || true
    pkill -9 -f "livox_ros_driver2_node"       2>/dev/null || true
    pkill -9 -f "rtabmap$"                     2>/dev/null || true
    pkill -9 -f "hdl_global_localization_node" 2>/dev/null || true
    echo "[INFO] All processes stopped. Logs: $LOG_DIR"
}
trap cleanup EXIT INT TERM

# ─────────────────────────────────────────────────────────────
# Phase 1: 完整 bringup，Nav2 先不自动激活（autostart=false）
# ─────────────────────────────────────────────────────────────
echo ""
echo "【Phase 1】Bringup + HDL node (Nav2 autostart=false)"
# setsid 让整个 launch 组（含 hdl_node、rtabmap、fastlio、livox）
# 在独立进程组里运行，cleanup 时 kill -- -PGID 一次性全部清理
setsid ros2 launch robot_bringup global_localization_bringup.launch.py \
    mode:="$BRINGUP_MODE" \
    database_path:="$DATABASE_PATH" \
    enable_rviz:="$ENABLE_RVIZ" \
    > "$LOG_DIR/bringup.log" 2>&1 &
BRINGUP_PID=$!
BRINGUP_PGID=$BRINGUP_PID   # setsid 后 PGID == PID
echo "  PID=$BRINGUP_PID  PGID=$BRINGUP_PGID  log=$LOG_DIR/bringup.log"

echo "  Waiting ${BRINGUP_WAIT_SEC}s for FAST-LIO ..."
sleep "$BRINGUP_WAIT_SEC"

# ─────────────────────────────────────────────────────────────
# Phase 2: 全局定位 → /initialpose（TRANSIENT_LOCAL）
# ─────────────────────────────────────────────────────────────
echo ""
echo "【Phase 2】Global localization (engine=$ENGINE)"
python3 "$SCRIPT_DIR/global_localization_node.py" --engine "$ENGINE"
echo "  /initialpose published"

# ─────────────────────────────────────────────────────────────
# Phase 3: 激活 Nav2（lifecycle manager）
# ─────────────────────────────────────────────────────────────
echo ""
echo "【Phase 3】Activating Nav2 ..."
ros2 service call /lifecycle_manager_navigation/manage_nodes \
    nav2_msgs/srv/ManageLifecycleNodes "{command: 0}" 2>/dev/null \
    && echo "  Nav2 activated" \
    || echo "  [WARN] Nav2 activation failed — may already be active"

echo ""
echo "[OK] System running. Press Ctrl+C to stop everything."
wait "$BRINGUP_PID"
