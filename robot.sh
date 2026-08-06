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
PCD_EXPORTER="$WS_DIR/scripts/extract_pcd_from_db.py"
STARTUP_CHECKER="$WS_DIR/scripts/robot_startup_check.py"

ROBOT_STARTUP_CHECKS="${ROBOT_STARTUP_CHECKS:-true}"
ROBOT_ROS2_CLEANUP_WAIT="${ROBOT_ROS2_CLEANUP_WAIT:-1.0}"
ROBOT_SENSOR_CHECK_TIMEOUT="${ROBOT_SENSOR_CHECK_TIMEOUT:-30.0}"
ROBOT_SENSOR_MAX_AGE="${ROBOT_SENSOR_MAX_AGE:-3.0}"
ROBOT_SENSOR_MAX_FUTURE="${ROBOT_SENSOR_MAX_FUTURE:-0.2}"
ROBOT_SENSOR_REQUIRED_SAMPLES="${ROBOT_SENSOR_REQUIRED_SAMPLES:-5}"

RELOCALIZATION_ENGINE="${RELOCALIZATION_ENGINE:-FPFH_RANSAC}"
RELOCALIZATION_MIN_INLIER="${RELOCALIZATION_MIN_INLIER:-0.98}"
RELOCALIZATION_MAX_ERROR="${RELOCALIZATION_MAX_ERROR:-inf}"
RELOCALIZATION_MAX_RETRIES="${RELOCALIZATION_MAX_RETRIES:-1}"
RELOCALIZATION_SCAN_TIMEOUT="${RELOCALIZATION_SCAN_TIMEOUT:-30.0}"
RELOCALIZATION_SUBSCRIBER_TIMEOUT="${RELOCALIZATION_SUBSCRIBER_TIMEOUT:-15.0}"
RELOCALIZATION_EXPORT_PCD="${RELOCALIZATION_EXPORT_PCD:-true}"
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
  启动清理     : 启动前执行 pkill -f ros2
  数据检查     : nav/rel 检查点云、里程计和 TF 新鲜度

可在模式后追加 launch 参数，例如:
  ./robot.sh map rviz:=false
  ./robot.sh nav enable_collision_monitor:=false
  ./robot.sh rel enable_rviz:=false

rel 模式可通过环境变量覆盖:
  PCD_PATH=/path/to/map.pcd
  RELOCALIZATION_ENGINE=FPFH_RANSAC
  RELOCALIZATION_MIN_INLIER=0.98
  RELOCALIZATION_MAX_ERROR=inf
  RELOCALIZATION_MAX_RETRIES=1
  RELOCALIZATION_SCAN_TIMEOUT=30.0
  RELOCALIZATION_SUBSCRIBER_TIMEOUT=15.0
  RELOCALIZATION_EXPORT_PCD=true
  MANUAL_INITIALPOSE_TIMEOUT=86400.0
  MANUAL_INITIALPOSE_FALLBACK=true
  ALLOW_STALE_PCD=false

启动清理和数据检查可通过环境变量覆盖:
  ROBOT_STARTUP_CHECKS=true
  ROBOT_ROS2_CLEANUP_WAIT=1.0
  ROBOT_SENSOR_CHECK_TIMEOUT=30.0
  ROBOT_SENSOR_MAX_AGE=3.0
  ROBOT_SENSOR_MAX_FUTURE=0.2
  ROBOT_SENSOR_REQUIRED_SAMPLES=5
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
if [[ "$ROBOT_STARTUP_CHECKS" == "true" &&
      ( "$MODE" == "nav" || "$MODE" == "rel" ) ]]; then
    [[ -f "$STARTUP_CHECKER" ]] \
        || die "未找到启动检查脚本：$STARTUP_CHECKER"
fi
if [[ "$MODE" == "rel" ]]; then
    [[ -f "$RELOCALIZATION_CLIENT" ]] \
        || die "未找到点云重定位客户端：$RELOCALIZATION_CLIENT"
    [[ -f "$PCD_EXPORTER" ]] \
        || die "未找到数据库点云导出脚本：$PCD_EXPORTER"
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

run_logged_check() {
    local status

    set +e
    "$@" 2>&1 | tee -a "$TERMINAL_LOG"
    status="${PIPESTATUS[0]}"
    set -e
    return "$status"
}

cleanup_previous_ros2() {
    echo "[INFO] 启动前清理旧 ROS 2 进程：pkill -f ros2" \
        | tee -a "$TERMINAL_LOG"
    pkill -f ros2 >/dev/null 2>&1 || true
    sleep "$ROBOT_ROS2_CLEANUP_WAIT"
    echo "[OK] 旧 ROS 2 启动进程清理命令已执行。" \
        | tee -a "$TERMINAL_LOG"
}

run_sensor_startup_check() {
    if [[ "$ROBOT_STARTUP_CHECKS" != "true" ]]; then
        return 0
    fi

    echo "[INFO] 检查 FAST-LIO 点云、里程计和 odom→base_footprint TF..." \
        | tee -a "$TERMINAL_LOG"
    run_logged_check python3 -u "$STARTUP_CHECKER" \
        --sensors \
        --timeout "$ROBOT_SENSOR_CHECK_TIMEOUT" \
        --max-age "$ROBOT_SENSOR_MAX_AGE" \
        --max-future "$ROBOT_SENSOR_MAX_FUTURE" \
        --required-samples "$ROBOT_SENSOR_REQUIRED_SAMPLES"
}

launch_arg_value() {
    local key="$1"
    local default_value="$2"
    shift 2

    local value="$default_value"
    local arg
    for arg in "$@"; do
        if [[ "$arg" == "$key:="* ]]; then
            value="${arg#"$key:="}"
        fi
    done
    printf '%s\n' "$value"
}

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

STACK_LAUNCH_PID=""

start_background_stack() {
    local filter_mode="$1"
    shift

    setsid "$@" \
        > >(
            tee -ia "$TERMINAL_LOG" \
                | python3 -u "$LOG_FILTER" --mode "$filter_mode"
        ) 2>&1 &
    STACK_LAUNCH_PID=$!
}

cleanup_stack() {
    if [[ -n "$STACK_LAUNCH_PID" ]] &&
       kill -0 "$STACK_LAUNCH_PID" 2>/dev/null; then
        echo "[INFO] 正在停止本次启动的机器人栈..."
        kill -INT -- "-$STACK_LAUNCH_PID" 2>/dev/null || true
        for _ in {1..20}; do
            kill -0 "$STACK_LAUNCH_PID" 2>/dev/null || break
            sleep 0.25
        done
        if kill -0 "$STACK_LAUNCH_PID" 2>/dev/null; then
            kill -TERM -- "-$STACK_LAUNCH_PID" 2>/dev/null || true
        fi
        wait "$STACK_LAUNCH_PID" 2>/dev/null || true
    fi
}

install_stack_cleanup_traps() {
    trap 'cleanup_stack; exit 130' INT TERM
    trap cleanup_stack EXIT
}

finish_with_stack() {
    local ready_message="$1"
    local launch_status

    echo "$ready_message"
    set +e
    wait "$STACK_LAUNCH_PID"
    launch_status="$?"
    set -e
    STACK_LAUNCH_PID=""
    trap - EXIT INT TERM
    exit "$launch_status"
}

cleanup_previous_ros2

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
    echo "[INFO] 启动基础导航；Nav2 将保持 inactive，直到传感器检查通过。"
    COLLISION_MONITOR_ENABLED="$(
        launch_arg_value enable_collision_monitor true "$@"
    )"
    install_stack_cleanup_traps
    start_background_stack nav \
        ros2 launch robot_bringup bringup.launch.py \
        enable_rviz:=true \
        enable_collision_monitor:=true \
        "$@" \
        autostart:=false \
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

    run_sensor_startup_check \
        || die "传感器数据未达到启动条件，Nav2 保持 inactive。"
    kill -0 "$STACK_LAUNCH_PID" 2>/dev/null \
        || die "导航 launch 已提前退出，请查看：$TERMINAL_LOG"

    if [[ "$COLLISION_MONITOR_ENABLED" == "true" ]]; then
        activate_lifecycle_manager \
            /lifecycle_manager_collision_monitor/manage_nodes \
            "Collision Monitor" \
            || die "安全节点未激活，Nav2 不会启动。"
    else
        echo "[WARN] Collision Monitor 已按 launch 参数禁用。"
    fi

    activate_lifecycle_manager \
        /lifecycle_manager_navigation/manage_nodes \
        "Nav2 导航节点" \
        || die "Nav2 激活失败。"

    finish_with_stack \
        "[READY] 传感器启动检查通过，导航系统已激活。按 Ctrl+C 停止。"
fi

latest_pcd() {
    find "$PCD_DIR" -maxdepth 1 -type f -name '*.pcd' -printf '%T@ %p\n' \
        | sort -nr \
        | head -n 1 \
        | cut -d' ' -f2-
}

export_relocalization_pcd() {
    echo "[INFO] 正在从当前 RTAB-Map 数据库导出重定位 PCD..." \
        | tee -a "$TERMINAL_LOG"
    run_logged_check python3 -u "$PCD_EXPORTER" \
        "$DATABASE_PATH" \
        "$PCD_DIR" \
        || die "从数据库导出重定位 PCD 失败，未启动重定位。"
}

if [[ -z "$PCD_PATH" ]]; then
    if [[ "$RELOCALIZATION_EXPORT_PCD" == "true" ]]; then
        export_relocalization_pcd
    else
        echo "[WARN] RELOCALIZATION_EXPORT_PCD=false，使用已有最新 PCD。" \
            | tee -a "$TERMINAL_LOG"
    fi
    PCD_PATH="$(latest_pcd)"
else
    echo "[INFO] 已显式指定 PCD_PATH，跳过数据库自动导出。" \
        | tee -a "$TERMINAL_LOG"
fi

[[ -n "$PCD_PATH" && -s "$PCD_PATH" ]] \
    || die "没有可用的重定位 PCD。请先把当前地图导出到：$PCD_DIR"

PCD_PATH="$(realpath "$PCD_PATH")"
if [[ "$ALLOW_STALE_PCD" != "true" && "$PCD_PATH" -ot "$DATABASE_PATH" ]]; then
    die "PCD 比数据库旧，拒绝使用不匹配地图：$PCD_PATH。请重新导出 PCD；仅调试时可设置 ALLOW_STALE_PCD=true"
fi

echo "[INFO] 启动纯点云重定位；Nav2 将保持 inactive，直到重定位成功。"
echo "[INFO] PCD地图：$PCD_PATH"

COLLISION_MONITOR_ENABLED="$(
    launch_arg_value enable_collision_monitor true "$@"
)"
install_stack_cleanup_traps
start_background_stack rel \
    ros2 launch robot_bringup global_localization_bringup.launch.py \
        database_path:="$DATABASE_PATH" \
        enable_rviz:=true \
        enable_collision_monitor:=true \
        use_edited_map:=false \
        "$@"

run_sensor_startup_check \
    || die "传感器数据未达到启动条件，Nav2 保持 inactive，未执行重定位。"
kill -0 "$STACK_LAUNCH_PID" 2>/dev/null \
    || die "重定位 launch 已提前退出，请查看：$TERMINAL_LOG"

set +e
python3 "$RELOCALIZATION_CLIENT" "$PCD_PATH" \
    --engine "$RELOCALIZATION_ENGINE" \
    --min-inlier "$RELOCALIZATION_MIN_INLIER" \
    --max-error "$RELOCALIZATION_MAX_ERROR" \
    --max-retries "$RELOCALIZATION_MAX_RETRIES" \
    --scan-timeout "$RELOCALIZATION_SCAN_TIMEOUT" \
    --max-scan-age "$ROBOT_SENSOR_MAX_AGE" \
    --max-future-skew "$ROBOT_SENSOR_MAX_FUTURE" \
    --subscriber-timeout "$RELOCALIZATION_SUBSCRIBER_TIMEOUT" \
    2>&1 | tee -a "$TERMINAL_LOG"
RELOCALIZATION_STATUS="${PIPESTATUS[0]}"
set -e

RELOCALIZATION_SOURCE="自动点云重定位"

wait_for_manual_initialpose() {
    cat <<'EOF' | tee -a "$TERMINAL_LOG"
[WARN] 自动点云重定位失败，Nav2 继续保持 inactive。
[ACTION] 请在 RViz 中选择 “2D Pose Estimate”，在地图上设置机器人位置和朝向。
[ACTION] 系统正在等待新的 /initialpose；收到后将直接激活导航。
[INFO]  按 Ctrl+C 可放弃人工定位并停止系统。
EOF

    set +e
    python3 "$RELOCALIZATION_CLIENT" \
        --wait-manual \
        --manual-pose-timeout "$MANUAL_INITIALPOSE_TIMEOUT" \
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
        || die "未收到新的人工 /initialpose，Nav2 保持 inactive。"
    RELOCALIZATION_SOURCE="人工初始位姿"
fi

if [[ "$COLLISION_MONITOR_ENABLED" == "true" ]]; then
    activate_lifecycle_manager \
        /lifecycle_manager_collision_monitor/manage_nodes \
        "Collision Monitor" \
        || die "安全节点未激活，Nav2 不会启动。"
else
    echo "[WARN] Collision Monitor 已按 launch 参数禁用。"
fi

activate_lifecycle_manager \
    /lifecycle_manager_navigation/manage_nodes \
    "Nav2 导航节点" \
    || die "Nav2 激活失败。"

finish_with_stack \
    "[READY] $RELOCALIZATION_SOURCE 已接受，导航系统已激活。按 Ctrl+C 停止。"
