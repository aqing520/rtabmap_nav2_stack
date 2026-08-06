#!/usr/bin/env bash
# Pure-LiDAR mapping/navigation entry point.
#
# Usage:
#   ./robot.sh map
#   ./robot.sh nav
#
# Extra arguments are forwarded to the ROS 2 launch file:
#   ./robot.sh map rviz:=false
#   ./robot.sh nav enable_collision_monitor:=false

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR"
DB_DIR="$WS_DIR/db"
DATABASE_PATH="$DB_DIR/rtabmap.db"
ROS_LOG_DIR="$WS_DIR/roslog"
export ROS_LOG_DIR
LOG_FILTER="$WS_DIR/scripts/robot_log_filter.py"

usage() {
    cat <<EOF
用法:
  ./robot.sh map    纯激光雷达建图
  ./robot.sh nav    使用 db/rtabmap.db 启动基础导航

固定配置:
  传感器      : Livox MID360
  相机/视觉   : 关闭
  GPS         : 关闭
  全局重定位  : 关闭
  数据库      : $DATABASE_PATH
  ROS日志     : $ROS_LOG_DIR
  Nav2 控制器 : CUDA MPPI

可在模式后追加 launch 参数，例如:
  ./robot.sh map rviz:=false
  ./robot.sh nav enable_collision_monitor:=false
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
    map|nav)
        ;;
    -h|--help|help)
        usage
        exit 0
        ;;
    *)
        usage >&2
        die "未知模式 '$MODE'，只能传 map 或 nav。"
        ;;
esac

[[ -f /opt/ros/humble/setup.bash ]] \
    || die "未找到 /opt/ros/humble/setup.bash"
[[ -f "$WS_DIR/install/setup.bash" ]] \
    || die "项目尚未编译，请先在项目根目录执行编译。"
[[ -f "$LOG_FILTER" ]] \
    || die "未找到终端日志过滤器：$LOG_FILTER"

mkdir -p "$DB_DIR" "$ROS_LOG_DIR"

# ROS/colcon setup files can reference variables before defining them.
set +u
source /opt/ros/humble/setup.bash

if [[ "$MODE" == "nav" ]]; then
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
