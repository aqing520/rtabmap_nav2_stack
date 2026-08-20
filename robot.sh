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
#   ./robot.sh nav enable_rviz:=false
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
STRUCTURAL_EXPORTER="$WS_DIR/scripts/export_structural_map_from_db.py"
CACHE_BUILDER="$WS_DIR/scripts/build_hdl_map_cache.py"
CACHE_ROOT="$PCD_DIR/cache/fpfh_ransac"
STARTUP_CHECKER="$WS_DIR/scripts/robot_startup_check.py"

ROBOT_STARTUP_CHECKS="${ROBOT_STARTUP_CHECKS:-true}"
ROBOT_ROS2_CLEANUP_WAIT="${ROBOT_ROS2_CLEANUP_WAIT:-1.0}"
ROBOT_LIVOX_RESTART="${ROBOT_LIVOX_RESTART:-true}"
ROBOT_SENSOR_CHECK_TIMEOUT="${ROBOT_SENSOR_CHECK_TIMEOUT:-30.0}"
ROBOT_SENSOR_MAX_AGE="${ROBOT_SENSOR_MAX_AGE:-3.0}"
ROBOT_SENSOR_READY_MAX_AGE="${ROBOT_SENSOR_READY_MAX_AGE:-0.3}"
ROBOT_SENSOR_MAX_FUTURE="${ROBOT_SENSOR_MAX_FUTURE:-0.2}"
ROBOT_SENSOR_MAX_CATCHUP_AGE_STEP="${ROBOT_SENSOR_MAX_CATCHUP_AGE_STEP:-0.03}"
ROBOT_SENSOR_REQUIRED_SAMPLES="${ROBOT_SENSOR_REQUIRED_SAMPLES:-5}"

# 当前工作空间的 FAST-LIO 和 Livox 二进制绝对路径。
# 仅匹配这些路径下的进程，避免误杀其它工作空间的节点。
FAST_LIO_BIN="$WS_DIR/install/fast_lio/lib/fast_lio/fastlio_mapping"
LIVOX_DRIVER_BIN="$WS_DIR/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node"

RELOCALIZATION_ENGINE="${RELOCALIZATION_ENGINE:-FPFH_RANSAC}"
RELOCALIZATION_PROFILE="${RELOCALIZATION_PROFILE:-legacy_fpfh_v1}"
CACHE_PROFILE="${CACHE_PROFILE:-$RELOCALIZATION_PROFILE}"
RELOCALIZATION_MIN_INLIER="${RELOCALIZATION_MIN_INLIER:-0.98}"
RELOCALIZATION_MAX_ERROR="${RELOCALIZATION_MAX_ERROR:-inf}"
RELOCALIZATION_MAX_RETRIES="${RELOCALIZATION_MAX_RETRIES:-1}"
RELOCALIZATION_SCAN_TIMEOUT="${RELOCALIZATION_SCAN_TIMEOUT:-30.0}"
RELOCALIZATION_SUBSCRIBER_TIMEOUT="${RELOCALIZATION_SUBSCRIBER_TIMEOUT:-15.0}"
RELOCALIZATION_EXPORT_PCD="${RELOCALIZATION_EXPORT_PCD:-false}"
RELOCALIZATION_ALLOW_ONLINE_MAP_SETUP="${RELOCALIZATION_ALLOW_ONLINE_MAP_SETUP:-false}"
FORCE_REBUILD_CACHE="${FORCE_REBUILD_CACHE:-false}"
RELOCALIZATION_QUERY_V2="${RELOCALIZATION_QUERY_V2:-false}"
RELOCALIZATION_QUERY_ACCUMULATION_SEC="${RELOCALIZATION_QUERY_ACCUMULATION_SEC:-0.0}"
RELOCALIZATION_QUERY_MIN_FRAMES="${RELOCALIZATION_QUERY_MIN_FRAMES:-5}"
RELOCALIZATION_QUERY_MAX_FRAMES="${RELOCALIZATION_QUERY_MAX_FRAMES:-30}"
RELOCALIZATION_QUERY_MAX_POINTS="${RELOCALIZATION_QUERY_MAX_POINTS:-30000}"
RELOCALIZATION_QUERY_SURFACE_VOXEL="${RELOCALIZATION_QUERY_SURFACE_VOXEL:-0.10}"
RELOCALIZATION_ODOM_SYNC_TOLERANCE="${RELOCALIZATION_ODOM_SYNC_TOLERANCE:-0.02}"
RELOCALIZATION_DIAGNOSTIC_CANDIDATES="${RELOCALIZATION_DIAGNOSTIC_CANDIDATES:-20}"
RELOCALIZATION_CAPTURE_QUERY_BAG="${RELOCALIZATION_CAPTURE_QUERY_BAG:-false}"
RELOCALIZATION_SAVE_DIAGNOSTICS="${RELOCALIZATION_SAVE_DIAGNOSTICS:-true}"
MANUAL_INITIALPOSE_TIMEOUT="${MANUAL_INITIALPOSE_TIMEOUT:-86400.0}"
MANUAL_INITIALPOSE_FALLBACK="${MANUAL_INITIALPOSE_FALLBACK:-true}"
ALLOW_STALE_PCD="${ALLOW_STALE_PCD:-false}"
PCD_PATH="${PCD_PATH:-}"

usage() {
    cat <<EOF
用法:
  ./robot.sh map    纯激光雷达建图
  ./robot.sh nav    使用 db/rtabmap.db 启动基础导航
  ./robot.sh cache  离线导出PCD并生成FPFH缓存
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
  ./robot.sh nav enable_rviz:=false
  ./robot.sh rel enable_rviz:=false

rel 模式可通过环境变量覆盖:
  PCD_PATH=/path/to/map.pcd
  RELOCALIZATION_ENGINE=FPFH_RANSAC
  RELOCALIZATION_PROFILE=legacy_fpfh_v1
  CACHE_PROFILE=legacy_fpfh_v1
  RELOCALIZATION_MIN_INLIER=0.98
  RELOCALIZATION_MAX_ERROR=inf
  RELOCALIZATION_MAX_RETRIES=1
  RELOCALIZATION_SCAN_TIMEOUT=30.0
  RELOCALIZATION_SUBSCRIBER_TIMEOUT=15.0
  RELOCALIZATION_EXPORT_PCD=false
  RELOCALIZATION_ALLOW_ONLINE_MAP_SETUP=false
  RELOCALIZATION_QUERY_V2=false
  RELOCALIZATION_QUERY_ACCUMULATION_SEC=0.0
  RELOCALIZATION_CAPTURE_QUERY_BAG=false
  RELOCALIZATION_SAVE_DIAGNOSTICS=true
  MANUAL_INITIALPOSE_TIMEOUT=86400.0
  MANUAL_INITIALPOSE_FALLBACK=true
  ALLOW_STALE_PCD=false

启动清理和数据检查可通过环境变量覆盖:
  ROBOT_STARTUP_CHECKS=true
  ROBOT_ROS2_CLEANUP_WAIT=1.0
  ROBOT_LIVOX_RESTART=true
  ROBOT_SENSOR_CHECK_TIMEOUT=30.0
  ROBOT_SENSOR_MAX_AGE=3.0
  ROBOT_SENSOR_READY_MAX_AGE=0.3
  ROBOT_SENSOR_MAX_FUTURE=0.2
  ROBOT_SENSOR_MAX_CATCHUP_AGE_STEP=0.03
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
    map|nav|cache|rel)
        ;;
    -h|--help|help)
        usage
        exit 0
        ;;
    *)
        usage >&2
        die "未知模式 '$MODE'，只能传 map、nav、cache 或 rel。"
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
if [[ "$MODE" == "rel" || "$MODE" == "cache" ]]; then
    [[ -f "$RELOCALIZATION_CLIENT" ]] \
        || die "未找到点云重定位客户端：$RELOCALIZATION_CLIENT"
    [[ -f "$PCD_EXPORTER" ]] \
        || die "未找到数据库点云导出脚本：$PCD_EXPORTER"
    [[ -f "$CACHE_BUILDER" ]] \
        || die "未找到HDL缓存构建脚本：$CACHE_BUILDER"
    [[ -f "$STRUCTURAL_EXPORTER" ]] \
        || die "未找到结构地图导出脚本：$STRUCTURAL_EXPORTER"
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
        echo "  profile  : $RELOCALIZATION_PROFILE"
    elif [[ "$MODE" == "cache" ]]; then
        echo "  缓存目录 : $CACHE_ROOT"
        echo "  profile  : $CACHE_PROFILE"
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

stop_workspace_node() {
    local executable="$1"
    local label="$2"

    [[ -x "$executable" ]] || return 0
    pgrep -f "$executable" >/dev/null 2>&1 || return 0

    echo "[INFO] 停止旧 $label：pkill -f $(basename "$executable")" \
        | tee -a "$TERMINAL_LOG"
    pkill -INT -f "$executable" >/dev/null 2>&1 || true
    for _ in {1..12}; do
        pgrep -f "$executable" >/dev/null 2>&1 || return 0
        sleep 0.25
    done
    pkill -KILL -f "$executable" >/dev/null 2>&1 || true
}

cleanup_previous_ros2() {
    echo "[INFO] 启动前清理旧 ROS 2 进程：pkill -f ros2" \
        | tee -a "$TERMINAL_LOG"
    pkill -f ros2 >/dev/null 2>&1 || true

    # ros2 launch 被停止后，子进程可能被 PID 1 收养；其命令行不含 "ros2"。
    # FAST-LIO 是 /Odometry 和 /cloud_registered_body 的唯一权威发布者，
    # 因此必须确保启动前没有旧实例保留在本工作空间中。
    stop_workspace_node "$FAST_LIO_BIN" "FAST-LIO"

    # Livox 驱动同样不含 "ros2"；保留该开关以兼容已有启动习惯。
    if [[ "$ROBOT_LIVOX_RESTART" == "true" ]]; then
        stop_workspace_node "$LIVOX_DRIVER_BIN" "Livox 驱动"
    fi

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
        --ready-max-age "$ROBOT_SENSOR_READY_MAX_AGE" \
        --max-future "$ROBOT_SENSOR_MAX_FUTURE" \
        --max-catchup-age-step "$ROBOT_SENSOR_MAX_CATCHUP_AGE_STEP" \
        --required-samples "$ROBOT_SENSOR_REQUIRED_SAMPLES"
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

validate_fpfh_profile() {
    case "$1" in
        legacy_fpfh_v1|garage_structural_v1)
            ;;
        *)
            die "不支持的FPFH profile：$1（仅支持 legacy_fpfh_v1 或 garage_structural_v1）"
            ;;
    esac
}

if [[ "$MODE" == "cache" ]]; then
    validate_fpfh_profile "$CACHE_PROFILE"
    CACHE_PCD_PATH="$PCD_PATH"
    if [[ "$CACHE_PROFILE" == "garage_structural_v1" ]]; then
        [[ -s "$DATABASE_PATH" ]] \
            || die "数据库不存在或为空：$DATABASE_PATH。请先执行 ./robot.sh map"
        if [[ -n "$CACHE_PCD_PATH" ]]; then
            echo "[WARN] garage_structural_v1 使用数据库和 Admin.opt_* 作为缓存源；忽略 PCD_PATH=$CACHE_PCD_PATH" \
                | tee -a "$TERMINAL_LOG"
        fi
        STRUCTURAL_BUILD_DIR="$PCD_DIR/structural_build/$CACHE_PROFILE"
        mkdir -p "$STRUCTURAL_BUILD_DIR"
        echo "[INFO] 从当前RTAB-Map数据库导出结构地图（需要 Admin.opt_ids/opt_poses）..." \
            | tee -a "$TERMINAL_LOG"
        STRUCTURAL_EXPORT_OUTPUT="$(
            python3 -u "$STRUCTURAL_EXPORTER" \
                "$DATABASE_PATH" "$STRUCTURAL_BUILD_DIR" \
                --profile "$CACHE_PROFILE" 2>&1
        )" || {
            printf '%s\n' "$STRUCTURAL_EXPORT_OUTPUT" | tee -a "$TERMINAL_LOG"
            die "结构地图导出失败。"
        }
        printf '%s\n' "$STRUCTURAL_EXPORT_OUTPUT" | tee -a "$TERMINAL_LOG"
        STRUCTURAL_SURFACE="$(
            sed -n 's/^STRUCTURAL_SURFACE: //p' <<<"$STRUCTURAL_EXPORT_OUTPUT" | tail -n 1
        )"
        STRUCTURAL_KEYPOINTS="$(
            sed -n 's/^STRUCTURAL_KEYPOINTS: //p' <<<"$STRUCTURAL_EXPORT_OUTPUT" | tail -n 1
        )"
        [[ -n "$STRUCTURAL_SURFACE" && -s "$STRUCTURAL_SURFACE" ]] \
            || die "结构surface PCD无效。"
        [[ -n "$STRUCTURAL_KEYPOINTS" && -s "$STRUCTURAL_KEYPOINTS" ]] \
            || die "结构keypoints PCD无效。"
        CACHE_PCD_PATH="$(realpath "$DATABASE_PATH")"
    elif [[ -z "$CACHE_PCD_PATH" ]]; then
        [[ -s "$DATABASE_PATH" ]] \
            || die "数据库不存在或为空：$DATABASE_PATH。请先执行 ./robot.sh map"
        echo "[INFO] 从当前RTAB-Map数据库导出离线缓存源PCD..." \
            | tee -a "$TERMINAL_LOG"
        CACHE_EXPORT_OUTPUT="$(
            python3 -u "$PCD_EXPORTER" "$DATABASE_PATH" "$PCD_DIR" 2>&1
        )" || {
            printf '%s\n' "$CACHE_EXPORT_OUTPUT" | tee -a "$TERMINAL_LOG"
            die "缓存源PCD导出失败。"
        }
        printf '%s\n' "$CACHE_EXPORT_OUTPUT" | tee -a "$TERMINAL_LOG"
        CACHE_PCD_PATH="$(
            sed -n 's/^  PCD: //p' <<<"$CACHE_EXPORT_OUTPUT" | tail -n 1
        )"
    fi

    [[ -n "$CACHE_PCD_PATH" && -s "$CACHE_PCD_PATH" ]] \
        || die "没有可用于缓存构建的地图源文件。"
    CACHE_PCD_PATH="$(realpath "$CACHE_PCD_PATH")"

    CACHE_ARGS=(
        "$CACHE_PCD_PATH"
        --cache-root "$CACHE_ROOT"
        --profile "$CACHE_PROFILE"
    )
    if [[ "$CACHE_PROFILE" == "garage_structural_v1" ]]; then
        CACHE_ARGS+=(
            --structural-surface "$STRUCTURAL_SURFACE"
            --structural-keypoints "$STRUCTURAL_KEYPOINTS"
        )
    fi
    if [[ "$FORCE_REBUILD_CACHE" == "true" ]]; then
        CACHE_ARGS+=(--force)
    fi
    echo "[INFO] 离线构建FPFH缓存：$CACHE_PCD_PATH" \
        | tee -a "$TERMINAL_LOG"
    python3 -u "$CACHE_BUILDER" "${CACHE_ARGS[@]}" \
        2>&1 | tee -a "$TERMINAL_LOG" \
        || die "FPFH缓存构建或校验失败。"
    echo "[READY] 离线FPFH缓存已生成；未启动任何传感器或导航节点。" \
        | tee -a "$TERMINAL_LOG"
    exit 0
fi

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
    install_stack_cleanup_traps
    start_background_stack nav \
        ros2 launch robot_bringup bringup.launch.py \
        enable_rviz:=true \
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

active_cache_source_for_profile() {
    local profile="$1"
    local active_json="$CACHE_ROOT/active.json"

    [[ -s "$active_json" ]] || return 1
    python3 - "$active_json" "$profile" <<'PY'
import json
import os
import sys

active_json, requested_profile = sys.argv[1:3]
try:
    with open(active_json, "r", encoding="utf-8") as stream:
        active = json.load(stream)
except Exception:
    sys.exit(1)

if active.get("profile_name") != requested_profile:
    sys.exit(2)

source_pcd = active.get("source_pcd") or ""
cache_directory = active.get("cache_directory") or ""
if not source_pcd or not os.path.isfile(source_pcd):
    sys.exit(3)
if cache_directory and not os.path.isdir(cache_directory):
    sys.exit(4)

print(os.path.realpath(source_pcd))
PY
}

export_relocalization_pcd() {
    echo "[INFO] 正在从当前 RTAB-Map 数据库导出重定位 PCD..." \
        | tee -a "$TERMINAL_LOG"
    run_logged_check python3 -u "$PCD_EXPORTER" \
        "$DATABASE_PATH" \
        "$PCD_DIR" \
        || die "从数据库导出重定位 PCD 失败，未启动重定位。"
}

RELOCALIZATION_CACHE_DIR=""
RELOCALIZATION_CACHE_SOURCE_PATH=""
RELOCALIZATION_MAP_INPUT_PATH=""
if [[ "$RELOCALIZATION_ENGINE" == "FPFH_RANSAC" ]]; then
    validate_fpfh_profile "$RELOCALIZATION_PROFILE"
    if [[ "$RELOCALIZATION_PROFILE" == "garage_structural_v1" ]]; then
        if [[ "$RELOCALIZATION_ALLOW_ONLINE_MAP_SETUP" == "true" ]]; then
            die "garage_structural_v1 不支持在线地图FPFH计算；请先执行 CACHE_PROFILE=garage_structural_v1 ./robot.sh cache"
        fi
        RELOCALIZATION_CACHE_SOURCE_PATH="$(realpath "$DATABASE_PATH")"
        RELOCALIZATION_MAP_INPUT_PATH="$RELOCALIZATION_CACHE_SOURCE_PATH"
        if [[ -n "$PCD_PATH" ]]; then
            echo "[WARN] garage_structural_v1 使用数据库作为缓存源；忽略重定位 PCD_PATH=$PCD_PATH" \
                | tee -a "$TERMINAL_LOG"
        fi
    else
        if [[ -z "$PCD_PATH" ]]; then
            if [[ "$RELOCALIZATION_EXPORT_PCD" == "true" ]]; then
                export_relocalization_pcd
                PCD_PATH="$(latest_pcd)"
            elif ACTIVE_CACHE_SOURCE="$(
                active_cache_source_for_profile "$RELOCALIZATION_PROFILE"
            )"; then
                PCD_PATH="$ACTIVE_CACHE_SOURCE"
                echo "[INFO] 使用 active FPFH 缓存记录中的源地图：$PCD_PATH" \
                    | tee -a "$TERMINAL_LOG"
            else
                echo "[WARN] 未找到可用 active FPFH 缓存记录，回退使用已有最新 PCD。" \
                    | tee -a "$TERMINAL_LOG"
                PCD_PATH="$(latest_pcd)"
            fi
        else
            echo "[INFO] 已显式指定 PCD_PATH，跳过数据库自动导出。" \
                | tee -a "$TERMINAL_LOG"
        fi

        [[ -n "$PCD_PATH" && -s "$PCD_PATH" ]] \
            || die "没有可用的重定位 PCD。请先把当前地图导出到：$PCD_DIR"

        PCD_PATH="$(realpath "$PCD_PATH")"
        if [[ "$ALLOW_STALE_PCD" != "true" && "$PCD_PATH" -ot "$DATABASE_PATH" ]]; then
            echo "[WARN] PCD 比数据库旧：$PCD_PATH；rel 阶段将继续按 source hash 校验并加载离线 FPFH 缓存，不再强制重建缓存。" \
                | tee -a "$TERMINAL_LOG"
        fi
        RELOCALIZATION_CACHE_SOURCE_PATH="$PCD_PATH"
        RELOCALIZATION_MAP_INPUT_PATH="$PCD_PATH"
    fi

    set +e
    CACHE_LOOKUP_OUTPUT="$(
        python3 -u "$CACHE_BUILDER" "$RELOCALIZATION_CACHE_SOURCE_PATH" \
            --cache-root "$CACHE_ROOT" \
            --profile "$RELOCALIZATION_PROFILE" \
            --locate-only 2>&1
    )"
    CACHE_LOOKUP_STATUS="$?"
    set -e
    if [[ "$CACHE_LOOKUP_STATUS" -eq 0 ]]; then
        RELOCALIZATION_CACHE_DIR="$(
            sed -n 's/^CACHE_DIR: //p' <<<"$CACHE_LOOKUP_OUTPUT" | tail -n 1
        )"
        echo "[INFO] FPFH缓存命中：$RELOCALIZATION_CACHE_DIR" \
            | tee -a "$TERMINAL_LOG"
    elif [[ "$RELOCALIZATION_ALLOW_ONLINE_MAP_SETUP" == "true" ]]; then
        printf '%s\n' "$CACHE_LOOKUP_OUTPUT" | tee -a "$TERMINAL_LOG"
        echo "[WARN] 缓存不可用；已显式允许在线地图FPFH计算。" \
            | tee -a "$TERMINAL_LOG"
    else
        printf '%s\n' "$CACHE_LOOKUP_OUTPUT" | tee -a "$TERMINAL_LOG"
        if [[ "$RELOCALIZATION_PROFILE" == "garage_structural_v1" ]]; then
            die "FPFH缓存缺失或失效。请先执行：CACHE_PROFILE=garage_structural_v1 ./robot.sh cache"
        fi
        die "FPFH缓存缺失或失效。请先执行：PCD_PATH='$PCD_PATH' ./robot.sh cache"
    fi
else
    if [[ -z "$PCD_PATH" ]]; then
        PCD_PATH="$(latest_pcd)"
    fi
    [[ -n "$PCD_PATH" && -s "$PCD_PATH" ]] \
        || die "没有可用的重定位 PCD。请先把当前地图导出到：$PCD_DIR"
    RELOCALIZATION_MAP_INPUT_PATH="$(realpath "$PCD_PATH")"
fi

echo "[INFO] 启动纯点云重定位；Nav2 将保持 inactive，直到重定位成功。"
echo "[INFO] 地图源：$RELOCALIZATION_MAP_INPUT_PATH"
if [[ -n "$RELOCALIZATION_CACHE_SOURCE_PATH" ]]; then
    echo "[INFO] 缓存源：$RELOCALIZATION_CACHE_SOURCE_PATH"
fi

if [[ "$RELOCALIZATION_ENGINE" == "FPFH_RANSAC" ]]; then
    export HDL_FPFH_PROFILE="$RELOCALIZATION_PROFILE"
fi
install_stack_cleanup_traps
start_background_stack rel \
    ros2 launch robot_bringup global_localization_bringup.launch.py \
        database_path:="$DATABASE_PATH" \
        enable_rviz:=true \
        use_edited_map:=false \
        "$@"

run_sensor_startup_check \
    || die "传感器数据未达到启动条件，Nav2 保持 inactive，未执行重定位。"
kill -0 "$STACK_LAUNCH_PID" 2>/dev/null \
    || die "重定位 launch 已提前退出，请查看：$TERMINAL_LOG"

set +e
RELOCALIZATION_CLIENT_ARGS=(
    "$RELOCALIZATION_MAP_INPUT_PATH"
    --engine "$RELOCALIZATION_ENGINE" \
    --min-inlier "$RELOCALIZATION_MIN_INLIER" \
    --max-error "$RELOCALIZATION_MAX_ERROR" \
    --max-retries "$RELOCALIZATION_MAX_RETRIES" \
    --scan-timeout "$RELOCALIZATION_SCAN_TIMEOUT" \
    --max-scan-age "$ROBOT_SENSOR_MAX_AGE" \
    --max-future-skew "$ROBOT_SENSOR_MAX_FUTURE" \
    --subscriber-timeout "$RELOCALIZATION_SUBSCRIBER_TIMEOUT" \
    --diagnostic-candidates "$RELOCALIZATION_DIAGNOSTIC_CANDIDATES" \
    --query-accumulation-sec "$RELOCALIZATION_QUERY_ACCUMULATION_SEC" \
    --query-min-frames "$RELOCALIZATION_QUERY_MIN_FRAMES" \
    --query-max-frames "$RELOCALIZATION_QUERY_MAX_FRAMES" \
    --query-max-points "$RELOCALIZATION_QUERY_MAX_POINTS" \
    --query-surface-voxel "$RELOCALIZATION_QUERY_SURFACE_VOXEL" \
    --odom-sync-tolerance "$RELOCALIZATION_ODOM_SYNC_TOLERANCE"
)
if [[ -n "$RELOCALIZATION_CACHE_DIR" ]]; then
    RELOCALIZATION_CLIENT_ARGS+=(--cache-dir "$RELOCALIZATION_CACHE_DIR")
fi
if [[ -n "$RELOCALIZATION_CACHE_SOURCE_PATH" ]]; then
    RELOCALIZATION_CLIENT_ARGS+=(
        --cache-source-path "$RELOCALIZATION_CACHE_SOURCE_PATH"
    )
fi
if [[ "$RELOCALIZATION_ALLOW_ONLINE_MAP_SETUP" == "true" ]]; then
    RELOCALIZATION_CLIENT_ARGS+=(--allow-online-map-setup)
fi
if [[ "$RELOCALIZATION_QUERY_V2" == "true" ]]; then
    RELOCALIZATION_CLIENT_ARGS+=(--query-v2)
fi
if [[ "$RELOCALIZATION_CAPTURE_QUERY_BAG" == "true" ]]; then
    RELOCALIZATION_CLIENT_ARGS+=(--capture-query-bag)
fi
if [[ "$RELOCALIZATION_SAVE_DIAGNOSTICS" != "true" ]]; then
    RELOCALIZATION_CLIENT_ARGS+=(--no-save-diagnostics)
fi
python3 "$RELOCALIZATION_CLIENT" "${RELOCALIZATION_CLIENT_ARGS[@]}" \
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

activate_lifecycle_manager \
    /lifecycle_manager_navigation/manage_nodes \
    "Nav2 导航节点" \
    || die "Nav2 激活失败。"

finish_with_stack \
    "[READY] $RELOCALIZATION_SOURCE 已接受，导航系统已激活。按 Ctrl+C 停止。"
