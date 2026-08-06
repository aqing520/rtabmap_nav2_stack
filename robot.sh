#!/usr/bin/env bash
# Pure-LiDAR mapping/navigation/relocalization entry point.
#
# Usage:
#   ./robot.sh map
#   ./robot.sh nav
#   ./robot.sh rel
#
# Extra arguments are forwarded to the ROS 2 launch file:
#   ./robot.sh map rviz:=false
#   ./robot.sh nav enable_collision_monitor:=false
#   ./robot.sh rel enable_rviz:=false

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR"
DB_DIR="$WS_DIR/db"
DATABASE_PATH="$DB_DIR/rtabmap.db"
PCD_DIR="$DB_DIR/pcd"
ROS_LOG_DIR="$WS_DIR/roslog"
export ROS_LOG_DIR
LOG_FILTER="$WS_DIR/scripts/robot_log_filter.py"
RELOCALIZATION_CLIENT="$WS_DIR/scripts/global_localization_node.py"

RELOCALIZATION_ENGINE="${RELOCALIZATION_ENGINE:-FPFH_RANSAC}"
RELOCALIZATION_MIN_INLIER="${RELOCALIZATION_MIN_INLIER:-0.98}"
RELOCALIZATION_MAX_ERROR="${RELOCALIZATION_MAX_ERROR:-inf}"
RELOCALIZATION_MAX_RETRIES="${RELOCALIZATION_MAX_RETRIES:-3}"
RELOCALIZATION_SCAN_TIMEOUT="${RELOCALIZATION_SCAN_TIMEOUT:-30.0}"
RELOCALIZATION_SUBSCRIBER_TIMEOUT="${RELOCALIZATION_SUBSCRIBER_TIMEOUT:-15.0}"
RELOCALIZATION_CONFIRMATION_TIMEOUT="${RELOCALIZATION_CONFIRMATION_TIMEOUT:-30.0}"
RELOCALIZATION_LINEAR_TOLERANCE="${RELOCALIZATION_LINEAR_TOLERANCE:-1.0}"
RELOCALIZATION_YAW_TOLERANCE_DEG="${RELOCALIZATION_YAW_TOLERANCE_DEG:-30.0}"
RELOCALIZATION_MAX_LINEAR_VARIANCE="${RELOCALIZATION_MAX_LINEAR_VARIANCE:-1.0}"
RELOCALIZATION_MAX_YAW_VARIANCE="${RELOCALIZATION_MAX_YAW_VARIANCE:-1.0}"
MANUAL_INITIALPOSE_TIMEOUT="${MANUAL_INITIALPOSE_TIMEOUT:-86400.0}"
MANUAL_INITIALPOSE_FALLBACK="${MANUAL_INITIALPOSE_FALLBACK:-true}"
ALLOW_STALE_PCD="${ALLOW_STALE_PCD:-false}"
PCD_PATH="${PCD_PATH:-}"

usage() {
    cat <<EOF
用法:
  ./robot.sh map    纯激光雷达建图
  ./robot.sh nav    使用 db/rtabmap.db 启动基础导航
  ./robot.sh rel    先做纯点云全局重定位，成功后再激活导航

固定配置:
  传感器      : Livox MID360
  相机/视觉   : 关闭
  GPS         : 关闭
  全局重定位  : 仅 rel 模式启用
  数据库      : $DATABASE_PATH
  点云地图    : $PCD_DIR 下最新的 PCD
  ROS日志     : $ROS_LOG_DIR
  Nav2 控制器 : CUDA MPPI

可在模式后追加 launch 参数，例如:
  ./robot.sh map rviz:=false
  ./robot.sh nav enable_collision_monitor:=false
  ./robot.sh rel enable_rviz:=false

rel 模式可通过环境变量覆盖:
  PCD_PATH=/path/to/map.pcd
  RELOCALIZATION_ENGINE=FPFH_RANSAC
  RELOCALIZATION_MIN_INLIER=0.98
  RELOCALIZATION_MAX_ERROR=inf
  RELOCALIZATION_MAX_RETRIES=3
  RELOCALIZATION_SCAN_TIMEOUT=30.0
  RELOCALIZATION_SUBSCRIBER_TIMEOUT=15.0
  RELOCALIZATION_CONFIRMATION_TIMEOUT=30.0
  RELOCALIZATION_LINEAR_TOLERANCE=1.0
  RELOCALIZATION_YAW_TOLERANCE_DEG=30.0
  RELOCALIZATION_MAX_LINEAR_VARIANCE=1.0
  RELOCALIZATION_MAX_YAW_VARIANCE=1.0
  MANUAL_INITIALPOSE_TIMEOUT=86400.0
  MANUAL_INITIALPOSE_FALLBACK=true
  ALLOW_STALE_PCD=false
EOF
}

die() {
    echo "[ERROR] $*" >&2
    exit 1
}

if [[ $# -lt 1 ]]; then
    usage
    exit 1
fi

MODE="$1"
shift

case "$MODE" in
    map|nav|rel)
        ;;
    -h|--help|help)
        usage
        exit 0
        ;;
    *)
        usage >&2
        die "未知模式 '$MODE'，只能传 map、nav 或 rel。"
        ;;
esac

[[ -f /opt/ros/humble/setup.bash ]] \
    || die "未找到 /opt/ros/humble/setup.bash"
[[ -f "$WS_DIR/install/setup.bash" ]] \
    || die "项目尚未编译，请先在项目根目录执行编译。"
[[ -f "$LOG_FILTER" ]] \
    || die "未找到终端日志过滤器：$LOG_FILTER"
if [[ "$MODE" == "rel" ]]; then
    [[ -f "$RELOCALIZATION_CLIENT" ]] \
        || die "未找到点云重定位客户端：$RELOCALIZATION_CLIENT"
fi

mkdir -p "$DB_DIR" "$PCD_DIR" "$ROS_LOG_DIR"

# ROS/colcon setup files can reference variables before defining them.
set +u
source /opt/ros/humble/setup.bash

if [[ "$MODE" == "nav" || "$MODE" == "rel" ]]; then
    if [[ -n "${CUDA_ROBOTICS_SETUP:-}" ]]; then
        [[ -f "$CUDA_ROBOTICS_SETUP" ]] \
            || die "CUDA_ROBOTICS_SETUP 指向的文件不存在：$CUDA_ROBOTICS_SETUP"
        source "$CUDA_ROBOTICS_SETUP"
    elif [[ -f "$HOME/cuda_robotics_ws/install/setup.bash" ]]; then
        source "$HOME/cuda_robotics_ws/install/setup.bash"
    else
        die "未找到 CUDA MPPI 工作空间。请设置 CUDA_ROBOTICS_SETUP=/path/to/install/setup.bash"
    fi
fi

source "$WS_DIR/install/setup.bash"
set -u

TERMINAL_LOG="$ROS_LOG_DIR/terminal_${MODE}_$(date '+%Y-%m-%d_%H-%M-%S').log"

{
    echo "============================================================"
    echo "  RTAB-Map Nav2 纯激光管理脚本"
    echo "  模式     : $MODE"
    echo "  工作空间 : $WS_DIR"
    echo "  数据库   : $DATABASE_PATH"
    if [[ "$MODE" == "rel" ]]; then
        echo "  重定位   : HDL $RELOCALIZATION_ENGINE"
    fi
    echo "  ROS日志  : $ROS_LOG_DIR"
    echo "  完整输出 : $TERMINAL_LOG"
    echo "============================================================"
} | tee "$TERMINAL_LOG"

run_with_filtered_terminal() {
    local launch_status

    # ros2 launch receives Ctrl+C normally. The shell, tee and filter stay
    # alive long enough to record and summarize the complete shutdown output.
    trap ':' INT
    set +e
    "$@" > >(
        tee -ia "$TERMINAL_LOG" \
            | python3 -u "$LOG_FILTER" --mode "$MODE"
    ) 2>&1
    launch_status="$?"
    set -e
    trap - INT

    exit "$launch_status"
}

if [[ "$MODE" == "map" ]]; then
    echo "[INFO] 启动纯激光建图；已有数据库将由 RTAB-Map 清空后重建。"
    run_with_filtered_terminal ros2 launch robot_bringup fastlio_mapping.launch.py \
        rviz:=true \
        rtabmap_viz:=false \
        "$@" \
        start_livox:=true \
        start_camera:=false \
        sensor_profile:=lidar_only \
        use_sim_time:=false \
        database_path:="$DATABASE_PATH" \
        delete_db_on_start:=true
fi

[[ -s "$DATABASE_PATH" ]] \
    || die "导航数据库不存在或为空：$DATABASE_PATH。请先执行 ./robot.sh map"

ros2 pkg prefix cuda_mppi_controller >/dev/null 2>&1 \
    || die "当前环境中找不到 cuda_mppi_controller，请检查 CUDA MPPI 工作空间。"

if [[ "$MODE" == "nav" ]]; then
    echo "[INFO] 启动基础导航，不启用相机、视觉定位或全局重定位。"
    run_with_filtered_terminal ros2 launch robot_bringup bringup.launch.py \
        autostart:=true \
        enable_rviz:=true \
        enable_collision_monitor:=true \
        "$@" \
        mode:=navigation \
        nav2_controller:=cuda_mppi \
        sensor_profile:=lidar_only \
        start_livox:=true \
        start_camera:=false \
        enable_gps:=false \
        use_sim_time:=false \
        database_path:="$DATABASE_PATH" \
        use_edited_map:=false \
        start_multi_waypoint_routes:=false
fi

latest_pcd() {
    find "$PCD_DIR" -maxdepth 1 -type f -name '*.pcd' -printf '%T@ %p\n' \
        | sort -nr \
        | head -n 1 \
        | cut -d' ' -f2-
}

if [[ -z "$PCD_PATH" ]]; then
    PCD_PATH="$(latest_pcd)"
fi

[[ -n "$PCD_PATH" && -s "$PCD_PATH" ]] \
    || die "没有可用的重定位 PCD。请先把当前地图导出到：$PCD_DIR"

PCD_PATH="$(realpath "$PCD_PATH")"
if [[ "$ALLOW_STALE_PCD" != "true" && "$PCD_PATH" -ot "$DATABASE_PATH" ]]; then
    die "PCD 比数据库旧，拒绝使用不匹配地图：$PCD_PATH。请重新导出 PCD；仅调试时可设置 ALLOW_STALE_PCD=true"
fi

echo "[INFO] 启动纯点云重定位；Nav2 将保持 inactive，直到重定位成功。"
echo "[INFO] PCD地图：$PCD_PATH"

RELOCALIZATION_LAUNCH_PID=""
cleanup_relocalization() {
    if [[ -n "$RELOCALIZATION_LAUNCH_PID" ]] &&
       kill -0 "$RELOCALIZATION_LAUNCH_PID" 2>/dev/null; then
        echo "[INFO] 正在停止重定位导航栈..."
        kill -INT -- "-$RELOCALIZATION_LAUNCH_PID" 2>/dev/null || true
        for _ in {1..20}; do
            kill -0 "$RELOCALIZATION_LAUNCH_PID" 2>/dev/null || break
            sleep 0.25
        done
        if kill -0 "$RELOCALIZATION_LAUNCH_PID" 2>/dev/null; then
            kill -TERM -- "-$RELOCALIZATION_LAUNCH_PID" 2>/dev/null || true
        fi
        wait "$RELOCALIZATION_LAUNCH_PID" 2>/dev/null || true
    fi
}
trap 'cleanup_relocalization; exit 130' INT TERM
trap cleanup_relocalization EXIT

setsid ros2 launch robot_bringup global_localization_bringup.launch.py \
    database_path:="$DATABASE_PATH" \
    enable_rviz:=true \
    enable_collision_monitor:=true \
    use_edited_map:=false \
    "$@" \
    > >(
        tee -ia "$TERMINAL_LOG" \
            | python3 -u "$LOG_FILTER" --mode rel
    ) 2>&1 &
RELOCALIZATION_LAUNCH_PID=$!

set +e
python3 "$RELOCALIZATION_CLIENT" "$PCD_PATH" \
    --engine "$RELOCALIZATION_ENGINE" \
    --min-inlier "$RELOCALIZATION_MIN_INLIER" \
    --max-error "$RELOCALIZATION_MAX_ERROR" \
    --max-retries "$RELOCALIZATION_MAX_RETRIES" \
    --scan-timeout "$RELOCALIZATION_SCAN_TIMEOUT" \
    --subscriber-timeout "$RELOCALIZATION_SUBSCRIBER_TIMEOUT" \
    --confirmation-timeout "$RELOCALIZATION_CONFIRMATION_TIMEOUT" \
    --linear-tolerance "$RELOCALIZATION_LINEAR_TOLERANCE" \
    --yaw-tolerance-deg "$RELOCALIZATION_YAW_TOLERANCE_DEG" \
    --max-linear-variance "$RELOCALIZATION_MAX_LINEAR_VARIANCE" \
    --max-yaw-variance "$RELOCALIZATION_MAX_YAW_VARIANCE" \
    2>&1 | tee -a "$TERMINAL_LOG"
RELOCALIZATION_STATUS="${PIPESTATUS[0]}"
set -e

RELOCALIZATION_SOURCE="自动点云重定位"

wait_for_manual_initialpose() {
    cat <<'EOF' | tee -a "$TERMINAL_LOG"
[WARN] 自动点云重定位失败，Nav2 继续保持 inactive。
[ACTION] 请在 RViz 中选择 “2D Pose Estimate”，在地图上设置机器人位置和朝向。
[ACTION] 系统正在等待新的 /initialpose；RTAB-Map 和 TF 均确认后才会激活导航。
[INFO]  按 Ctrl+C 可放弃人工定位并停止系统。
EOF

    set +e
    python3 "$RELOCALIZATION_CLIENT" \
        --wait-manual \
        --manual-pose-timeout "$MANUAL_INITIALPOSE_TIMEOUT" \
        --confirmation-timeout "$RELOCALIZATION_CONFIRMATION_TIMEOUT" \
        --linear-tolerance "$RELOCALIZATION_LINEAR_TOLERANCE" \
        --yaw-tolerance-deg "$RELOCALIZATION_YAW_TOLERANCE_DEG" \
        --max-linear-variance "$RELOCALIZATION_MAX_LINEAR_VARIANCE" \
        --max-yaw-variance "$RELOCALIZATION_MAX_YAW_VARIANCE" \
        2>&1 | tee -a "$TERMINAL_LOG"
    local status="${PIPESTATUS[0]}"
    set -e

    return "$status"
}

if [[ "$RELOCALIZATION_STATUS" -ne 0 ]]; then
    if [[ "$MANUAL_INITIALPOSE_FALLBACK" != "true" ]]; then
        die "点云重定位失败，Nav2 保持 inactive。"
    fi
    wait_for_manual_initialpose \
        || die "人工 /initialpose 未被 RTAB-Map/TF 确认，Nav2 保持 inactive。"
    RELOCALIZATION_SOURCE="人工初始位姿"
fi

activate_lifecycle_manager() {
    local service_name="$1"
    local label="$2"
    local output=""

    set +e
    output="$(timeout 30s ros2 service call \
        "$service_name" \
        nav2_msgs/srv/ManageLifecycleNodes \
        "{command: 0}" 2>&1)"
    local status="$?"
    set -e
    printf '%s\n' "$output" >> "$TERMINAL_LOG"

    if [[ "$status" -ne 0 ]] ||
       ! grep -Eq 'success[=:][[:space:]]*(True|true)' <<< "$output"; then
        echo "[ERROR] $label 激活失败：$service_name" >&2
        return 1
    fi
    echo "[OK] $label 已激活"
}

activate_lifecycle_manager \
    /lifecycle_manager_collision_monitor/manage_nodes \
    "Collision Monitor" \
    || die "安全节点未激活，Nav2 不会启动。"

activate_lifecycle_manager \
    /lifecycle_manager_navigation/manage_nodes \
    "Nav2 导航节点" \
    || die "Nav2 激活失败。"

echo "[READY] $RELOCALIZATION_SOURCE 已接受，导航系统已激活。按 Ctrl+C 停止。"

set +e
wait "$RELOCALIZATION_LAUNCH_PID"
LAUNCH_STATUS="$?"
set -e
RELOCALIZATION_LAUNCH_PID=""
trap - EXIT INT TERM
exit "$LAUNCH_STATUS"
