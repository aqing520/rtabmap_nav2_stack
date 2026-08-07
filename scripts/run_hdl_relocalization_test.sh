#!/usr/bin/env bash
# Run one HDL pure-point-cloud relocalization test and write report-ready data.
#
# This script intentionally stops after the HDL pose is obtained. It launches
# robot_bringup in localization mode, so Nav2 and Collision Monitor are not
# started or activated.

set -uo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
DB_PATH="$WS_DIR/db/rtabmap.db"
PCD_DIR="$WS_DIR/db/pcd"
EXPORTER="$SCRIPT_DIR/extract_pcd_from_db.py"
STARTUP_CHECKER="$SCRIPT_DIR/robot_startup_check.py"
LOCALIZATION_CLIENT="$SCRIPT_DIR/global_localization_node.py"
RESULT_ROOT="$WS_DIR/test_results/hdl_relocalization"
SUMMARY_CSV="$RESULT_ROOT/hdl_relocalization_results.csv"

SCENE=""
POINT_ID=""
TRIAL_ID=""
REF_X=""
REF_Y=""
REF_YAW_DEG=""
NOTES=""
ENGINE="${RELOCALIZATION_ENGINE:-FPFH_RANSAC}"
MIN_INLIER="${RELOCALIZATION_MIN_INLIER:-0.98}"
MAX_ERROR="${RELOCALIZATION_MAX_ERROR:-inf}"
MAX_RETRIES="${RELOCALIZATION_MAX_RETRIES:-1}"
SCAN_TIMEOUT="${RELOCALIZATION_SCAN_TIMEOUT:-30.0}"
SUBSCRIBER_TIMEOUT="${RELOCALIZATION_SUBSCRIBER_TIMEOUT:-15.0}"
SENSOR_TIMEOUT="${ROBOT_SENSOR_CHECK_TIMEOUT:-30.0}"
MAX_SCAN_AGE="${ROBOT_SENSOR_MAX_AGE:-3.0}"
MAX_FUTURE_SKEW="${ROBOT_SENSOR_MAX_FUTURE:-0.2}"
REQUIRED_SAMPLES="${ROBOT_SENSOR_REQUIRED_SAMPLES:-5}"
CLEANUP_OLD_ROS2="${HDL_TEST_CLEANUP_ROS2:-true}"
MAX_POS_ERROR=""
MAX_YAW_ERROR=""

STACK_PID=""
RUN_DIR=""
RAW_LOG=""
PCD_PATH=""
PCD_POINTS=""
EXPORT_STATUS=99
SENSOR_STATUS=99
CLIENT_STATUS=99
FAILURE_REASON=""
START_NS=""
EXPORT_START_NS=""
EXPORT_END_NS=""
SENSOR_START_NS=""
SENSOR_END_NS=""
CLIENT_START_NS=""
CLIENT_END_NS=""
END_NS=""
FINALIZED=false

usage() {
    cat <<'EOF'
用途:
  只执行一次 HDL 纯点云重定位测试，不启动 Nav2，不测试导航。
  每次运行都会从当前 db/rtabmap.db 导出新的 PCD，并生成 CSV、JSON 和原始日志。

用法:
  bash scripts/run_hdl_relocalization_test.sh \
    --scene workshop --point W1 \
    --ref-x 1.20 --ref-y 2.30 --ref-yaw 90 \
    --notes "靠近工作台"

  bash scripts/run_hdl_relocalization_test.sh \
    --scene garage --point G3 \
    --ref-x -4.50 --ref-y 12.80 --ref-yaw 180

必填参数:
  --scene NAME       workshop/工作室 或 garage/地下车库
  --point ID         测试点编号，例如 W1、G3

建议填写:
  --ref-x M          参考位置 x，单位 m
  --ref-y M          参考位置 y，单位 m
  --ref-yaw DEG      参考航向角，单位 °；默认只记录，不参与通过判定
  --trial ID         试验编号；默认使用时间戳
  --notes TEXT       现场条件、遮挡、车辆状态等说明

可选参数:
  --engine NAME      FPFH_RANSAC 或 BBS，默认 FPFH_RANSAC
  --min-inlier N     默认 0.98
  --max-error N      默认 inf
  --max-retries N    默认 1
  --max-pos-error M  平面位置误差验收半径；默认 1.5 m
  --max-yaw-error D  可选航向误差验收线；默认不启用，航向只记录
  --no-cleanup       不执行启动前 pkill -f ros2
  -h, --help         显示帮助

输出:
  test_results/hdl_relocalization/hdl_relocalization_results.csv
  test_results/hdl_relocalization/<场景>_<点位>_<时间>/result.json
  test_results/hdl_relocalization/<场景>_<点位>_<时间>/terminal.log
EOF
}

normalize_scene() {
    case "$1" in
        workshop|studio|工作室)
            printf 'workshop\n'
            ;;
        garage|underground_garage|地下车库|车库)
            printf 'garage\n'
            ;;
        *)
            return 1
            ;;
    esac
}

sanitize_label() {
    printf '%s' "$1" | sed 's/[^A-Za-z0-9_.-]/_/g'
}

now_ns() {
    date +%s%N
}

log() {
    local message="$*"
    if [[ -n "$RAW_LOG" ]]; then
        printf '%s\n' "$message" | tee -a "$RAW_LOG"
    else
        printf '%s\n' "$message"
    fi
}

cleanup_stack() {
    if [[ -n "$STACK_PID" ]] && kill -0 "$STACK_PID" 2>/dev/null; then
        log "[TEST] 正在停止本次纯重定位节点..."
        kill -INT -- "-$STACK_PID" 2>/dev/null || true
        for _ in {1..20}; do
            kill -0 "$STACK_PID" 2>/dev/null || break
            sleep 0.25
        done
        if kill -0 "$STACK_PID" 2>/dev/null; then
            kill -TERM -- "-$STACK_PID" 2>/dev/null || true
        fi
        wait "$STACK_PID" 2>/dev/null || true
    fi
    STACK_PID=""
}

write_result() {
    if [[ "$FINALIZED" == "true" || -z "$RUN_DIR" ]]; then
        return
    fi
    FINALIZED=true
    END_NS="${END_NS:-$(now_ns)}"

    export TEST_WS_DIR="$WS_DIR"
    export TEST_RUN_DIR="$RUN_DIR"
    export TEST_RAW_LOG="$RAW_LOG"
    export TEST_SUMMARY_CSV="$SUMMARY_CSV"
    export TEST_SCENE="$SCENE"
    export TEST_POINT_ID="$POINT_ID"
    export TEST_TRIAL_ID="$TRIAL_ID"
    export TEST_REF_X="$REF_X"
    export TEST_REF_Y="$REF_Y"
    export TEST_REF_YAW_DEG="$REF_YAW_DEG"
    export TEST_NOTES="$NOTES"
    export TEST_ENGINE="$ENGINE"
    export TEST_MIN_INLIER="$MIN_INLIER"
    export TEST_MAX_ERROR="$MAX_ERROR"
    export TEST_MAX_RETRIES="$MAX_RETRIES"
    export TEST_MAX_SCAN_AGE="$MAX_SCAN_AGE"
    export TEST_MAX_FUTURE_SKEW="$MAX_FUTURE_SKEW"
    export TEST_REQUIRED_SAMPLES="$REQUIRED_SAMPLES"
    export TEST_MAX_POS_ERROR="$MAX_POS_ERROR"
    export TEST_MAX_YAW_ERROR="$MAX_YAW_ERROR"
    export TEST_DB_PATH="$DB_PATH"
    export TEST_PCD_PATH="$PCD_PATH"
    export TEST_PCD_POINTS="$PCD_POINTS"
    export TEST_EXPORT_STATUS="$EXPORT_STATUS"
    export TEST_SENSOR_STATUS="$SENSOR_STATUS"
    export TEST_CLIENT_STATUS="$CLIENT_STATUS"
    export TEST_FAILURE_REASON="$FAILURE_REASON"
    export TEST_START_NS="$START_NS"
    export TEST_EXPORT_START_NS="$EXPORT_START_NS"
    export TEST_EXPORT_END_NS="$EXPORT_END_NS"
    export TEST_SENSOR_START_NS="$SENSOR_START_NS"
    export TEST_SENSOR_END_NS="$SENSOR_END_NS"
    export TEST_CLIENT_START_NS="$CLIENT_START_NS"
    export TEST_CLIENT_END_NS="$CLIENT_END_NS"
    export TEST_END_NS="$END_NS"

    python3 - <<'PY'
import csv
import fcntl
import hashlib
import json
import math
import os
import platform
import re
import subprocess
from datetime import datetime
from pathlib import Path


def env(name, default=""):
    return os.environ.get(name, default)


def number(value):
    if value in ("", None):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def integer(value):
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def seconds(start_name, end_name):
    start = integer(env(start_name))
    end = integer(env(end_name))
    if start is None or end is None or end < start:
        return None
    return round((end - start) / 1.0e9, 3)


def sha256(path_text):
    path = Path(path_text)
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def file_info(path_text):
    path = Path(path_text)
    if not path.is_file():
        return {"path": path_text or None, "size_bytes": None,
                "mtime": None, "sha256": None}
    stat = path.stat()
    return {
        "path": str(path.resolve()),
        "size_bytes": stat.st_size,
        "mtime": datetime.fromtimestamp(stat.st_mtime).astimezone().isoformat(),
        "sha256": sha256(path),
    }


def ros_stamp(line):
    match = re.search(r"\[INFO\]\s+\[(\d+\.\d+)\]", line)
    return float(match.group(1)) if match else None


raw_path = Path(env("TEST_RAW_LOG"))
text = raw_path.read_text(errors="replace") if raw_path.is_file() else ""
text = re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", text).replace("\r", "")
lines = text.splitlines()

pose_match = None
pose_re = re.compile(
    r"x=([-+]?\d+(?:\.\d+)?)\s+y=([-+]?\d+(?:\.\d+)?)\s+"
    r"yaw=([-+]?\d+(?:\.\d+)?)°\s+inlier=([-+]?\d+(?:\.\d+)?)\s+"
    r"err=([-+]?\d+(?:\.\d+)?)"
)
for line in lines:
    candidate = pose_re.search(line)
    if candidate:
        pose_match = candidate

scan_points = None
fresh_stamp = None
result_stamp = None
sending_map_stamp = None
map_loaded_stamp = None
for line in lines:
    if "Sending map (" in line:
        match = re.search(r"Sending map \((\d+) pts\)", line)
        if match:
            map_points_sent = int(match.group(1))
        sending_map_stamp = ros_stamp(line) or sending_map_stamp
    if "Map loaded. Discarded scans" in line:
        map_loaded_stamp = ros_stamp(line) or map_loaded_stamp
    if "Fresh scan received" in line:
        match = re.search(r"Fresh scan received \((\d+) pts\)", line)
        if match:
            scan_points = int(match.group(1))
        fresh_stamp = ros_stamp(line) or fresh_stamp
    if pose_re.search(line):
        result_stamp = ros_stamp(line) or result_stamp

map_points_sent = locals().get("map_points_sent")
result_x = float(pose_match.group(1)) if pose_match else None
result_y = float(pose_match.group(2)) if pose_match else None
result_yaw = float(pose_match.group(3)) if pose_match else None
inlier = float(pose_match.group(4)) if pose_match else None
matching_error = float(pose_match.group(5)) if pose_match else None

ref_x = number(env("TEST_REF_X"))
ref_y = number(env("TEST_REF_Y"))
ref_yaw = number(env("TEST_REF_YAW_DEG"))
max_pos_error = number(env("TEST_MAX_POS_ERROR"))
max_yaw_error = number(env("TEST_MAX_YAW_ERROR"))

position_error = None
if None not in (result_x, result_y, ref_x, ref_y):
    position_error = round(math.hypot(result_x - ref_x, result_y - ref_y), 3)

yaw_error = None
if result_yaw is not None and ref_yaw is not None:
    yaw_error = round(abs((result_yaw - ref_yaw + 180.0) % 360.0 - 180.0), 2)

client_status = integer(env("TEST_CLIENT_STATUS"))
match_success = client_status == 0 and pose_match is not None
has_ground_truth = None not in (ref_x, ref_y)
yaw_used_for_acceptance = max_yaw_error is not None
if not match_success:
    report_status = "FAIL_MATCH"
    accuracy_pass = False if has_ground_truth else None
elif not has_ground_truth:
    report_status = "MATCH_SUCCESS_NO_GROUND_TRUTH"
    accuracy_pass = None
else:
    accuracy_pass = (
        position_error is not None
        and max_pos_error is not None
        and position_error <= max_pos_error
        and (
            not yaw_used_for_acceptance
            or (
                yaw_error is not None
                and yaw_error <= max_yaw_error
            )
        )
    )
    report_status = "PASS" if accuracy_pass else "FAIL_ACCURACY"

try:
    git_commit = subprocess.check_output(
        ["git", "-C", env("TEST_WS_DIR"), "rev-parse", "HEAD"],
        text=True,
        stderr=subprocess.DEVNULL,
    ).strip()
except Exception:
    git_commit = None

try:
    ros_distro = subprocess.check_output(
        ["bash", "-c", "source /opt/ros/humble/setup.bash && printf %s \"$ROS_DISTRO\""],
        text=True,
        stderr=subprocess.DEVNULL,
    ).strip()
except Exception:
    ros_distro = None

timing = {
    "pcd_export_sec": seconds("TEST_EXPORT_START_NS", "TEST_EXPORT_END_NS"),
    "sensor_gate_sec": seconds("TEST_SENSOR_START_NS", "TEST_SENSOR_END_NS"),
    "localization_client_sec": seconds("TEST_CLIENT_START_NS", "TEST_CLIENT_END_NS"),
    "total_sec": seconds("TEST_START_NS", "TEST_END_NS"),
    "hdl_map_setup_sec": (
        round(map_loaded_stamp - sending_map_stamp, 3)
        if map_loaded_stamp is not None and sending_map_stamp is not None
        else None
    ),
    "hdl_query_sec": (
        round(result_stamp - fresh_stamp, 3)
        if result_stamp is not None and fresh_stamp is not None
        else None
    ),
}

result = {
    "schema_version": 1,
    "test": {
        "scene": env("TEST_SCENE"),
        "point_id": env("TEST_POINT_ID"),
        "trial_id": env("TEST_TRIAL_ID"),
        "notes": env("TEST_NOTES") or None,
        "started_at": (
            datetime.fromtimestamp(
                int(env("TEST_START_NS")) / 1.0e9
            ).astimezone().isoformat()
            if env("TEST_START_NS") else None
        ),
        "finished_at": (
            datetime.fromtimestamp(
                int(env("TEST_END_NS")) / 1.0e9
            ).astimezone().isoformat()
            if env("TEST_END_NS") else None
        ),
    },
    "reference_pose": {
        "x_m": ref_x,
        "y_m": ref_y,
        "yaw_deg": ref_yaw,
    },
    "hdl_result": {
        "match_success": match_success,
        "x_m": result_x,
        "y_m": result_y,
        "yaw_deg": result_yaw,
        "inlier_fraction": inlier,
        "matching_error": matching_error,
        "fresh_scan_points": scan_points,
        "map_points_sent_to_hdl": map_points_sent,
    },
    "accuracy": {
        "position_error_m": position_error,
        "yaw_error_deg": yaw_error,
        "max_position_error_m": max_pos_error,
        "max_yaw_error_deg": max_yaw_error,
        "yaw_used_for_acceptance": yaw_used_for_acceptance,
        "acceptance_rule": (
            "position_radius_and_yaw"
            if yaw_used_for_acceptance
            else "position_radius_only"
        ),
        "pass": accuracy_pass,
        "report_status": report_status,
    },
    "timing": timing,
    "configuration": {
        "engine": env("TEST_ENGINE"),
        "min_inlier": number(env("TEST_MIN_INLIER")),
        "max_error": env("TEST_MAX_ERROR"),
        "max_retries": integer(env("TEST_MAX_RETRIES")),
        "max_scan_age_sec": number(env("TEST_MAX_SCAN_AGE")),
        "max_future_skew_sec": number(env("TEST_MAX_FUTURE_SKEW")),
        "required_fresh_samples": integer(env("TEST_REQUIRED_SAMPLES")),
    },
    "phase_status": {
        "pcd_export_exit_code": integer(env("TEST_EXPORT_STATUS")),
        "sensor_gate_exit_code": integer(env("TEST_SENSOR_STATUS")),
        "localization_client_exit_code": client_status,
        "failure_reason": env("TEST_FAILURE_REASON") or None,
    },
    "database": file_info(env("TEST_DB_PATH")),
    "pcd": {
        **file_info(env("TEST_PCD_PATH")),
        "exported_points": integer(env("TEST_PCD_POINTS")),
    },
    "environment": {
        "git_commit": git_commit,
        "ros_distro": ros_distro,
        "hostname": platform.node(),
        "kernel": platform.release(),
        "python": platform.python_version(),
    },
    "evidence": {
        "terminal_log": str(raw_path.resolve()) if raw_path.exists() else None,
    },
}

run_dir = Path(env("TEST_RUN_DIR"))
json_path = run_dir / "result.json"
json_path.write_text(
    json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
    encoding="utf-8",
)

csv_path = Path(env("TEST_SUMMARY_CSV"))
csv_path.parent.mkdir(parents=True, exist_ok=True)
row = {
    "started_at": result["test"]["started_at"],
    "scene": result["test"]["scene"],
    "point_id": result["test"]["point_id"],
    "trial_id": result["test"]["trial_id"],
    "ref_x_m": ref_x,
    "ref_y_m": ref_y,
    "ref_yaw_deg": ref_yaw,
    "result_x_m": result_x,
    "result_y_m": result_y,
    "result_yaw_deg": result_yaw,
    "position_error_m": position_error,
    "yaw_error_deg": yaw_error,
    "max_position_error_m": max_pos_error,
    "max_yaw_error_deg": max_yaw_error,
    "yaw_used_for_acceptance": yaw_used_for_acceptance,
    "match_success": match_success,
    "accuracy_pass": accuracy_pass,
    "report_status": report_status,
    "inlier_fraction": inlier,
    "matching_error": matching_error,
    "fresh_scan_points": scan_points,
    "map_points_sent": map_points_sent,
    "pcd_exported_points": integer(env("TEST_PCD_POINTS")),
    "pcd_export_sec": timing["pcd_export_sec"],
    "sensor_gate_sec": timing["sensor_gate_sec"],
    "hdl_map_setup_sec": timing["hdl_map_setup_sec"],
    "hdl_query_sec": timing["hdl_query_sec"],
    "localization_client_sec": timing["localization_client_sec"],
    "total_sec": timing["total_sec"],
    "engine": env("TEST_ENGINE"),
    "min_inlier": number(env("TEST_MIN_INLIER")),
    "max_error": env("TEST_MAX_ERROR"),
    "max_scan_age_sec": number(env("TEST_MAX_SCAN_AGE")),
    "db_sha256": result["database"]["sha256"],
    "pcd_path": result["pcd"]["path"],
    "pcd_sha256": result["pcd"]["sha256"],
    "git_commit": git_commit,
    "failure_reason": env("TEST_FAILURE_REASON") or None,
    "notes": env("TEST_NOTES") or None,
    "result_json": str(json_path.resolve()),
    "terminal_log": result["evidence"]["terminal_log"],
}
fieldnames = list(row.keys())
with csv_path.open("a+", newline="", encoding="utf-8-sig") as stream:
    fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
    stream.seek(0, os.SEEK_END)
    writer = csv.DictWriter(stream, fieldnames=fieldnames)
    if stream.tell() == 0:
        writer.writeheader()
    writer.writerow(row)
    stream.flush()
    fcntl.flock(stream.fileno(), fcntl.LOCK_UN)

print("")
print("================ HDL 重定位测试结果 ================")
print(f"场景/点位       : {result['test']['scene']} / {result['test']['point_id']}")
print(f"匹配状态        : {'成功' if match_success else '失败'}")
if match_success:
    print(
        "输出位置        : "
        f"x={result_x:.3f} m, y={result_y:.3f} m, yaw={result_yaw:.1f}°"
    )
    print(f"inlier / error  : {inlier:.3f} / {matching_error:.3f}")
if position_error is not None:
    print(f"平面位置误差    : {position_error:.3f} m")
if yaw_error is not None:
    print(f"航向误差（记录）: {yaw_error:.2f}°")
if max_pos_error is not None:
    yaw_rule = (
        f"航向≤{max_yaw_error:.2f}°"
        if yaw_used_for_acceptance
        else "航向仅记录"
    )
    print(f"验收规则        : 半径≤{max_pos_error:.3f} m；{yaw_rule}")
print(f"地图建立耗时    : {timing['hdl_map_setup_sec']} s")
print(f"HDL 查询耗时    : {timing['hdl_query_sec']} s")
print(f"总耗时          : {timing['total_sec']} s")
print(f"报告状态        : {report_status}")
print(f"CSV             : {csv_path.resolve()}")
print(f"JSON            : {json_path.resolve()}")
print(f"完整日志        : {raw_path.resolve()}")
print("====================================================")
PY
}

finish() {
    local exit_code="$1"
    cleanup_stack
    write_result
    exit "$exit_code"
}

on_signal() {
    FAILURE_REASON="用户中断"
    CLIENT_STATUS=130
    CLIENT_END_NS="$(now_ns)"
    finish 130
}

trap on_signal INT TERM

while [[ $# -gt 0 ]]; do
    case "$1" in
        --scene)
            [[ $# -ge 2 ]] || { echo "[ERROR] --scene 缺少参数" >&2; exit 2; }
            SCENE="$2"; shift 2
            ;;
        --point)
            [[ $# -ge 2 ]] || { echo "[ERROR] --point 缺少参数" >&2; exit 2; }
            POINT_ID="$2"; shift 2
            ;;
        --trial)
            [[ $# -ge 2 ]] || { echo "[ERROR] --trial 缺少参数" >&2; exit 2; }
            TRIAL_ID="$2"; shift 2
            ;;
        --ref-x)
            [[ $# -ge 2 ]] || { echo "[ERROR] --ref-x 缺少参数" >&2; exit 2; }
            REF_X="$2"; shift 2
            ;;
        --ref-y)
            [[ $# -ge 2 ]] || { echo "[ERROR] --ref-y 缺少参数" >&2; exit 2; }
            REF_Y="$2"; shift 2
            ;;
        --ref-yaw)
            [[ $# -ge 2 ]] || { echo "[ERROR] --ref-yaw 缺少参数" >&2; exit 2; }
            REF_YAW_DEG="$2"; shift 2
            ;;
        --notes)
            [[ $# -ge 2 ]] || { echo "[ERROR] --notes 缺少参数" >&2; exit 2; }
            NOTES="$2"; shift 2
            ;;
        --engine)
            [[ $# -ge 2 ]] || { echo "[ERROR] --engine 缺少参数" >&2; exit 2; }
            ENGINE="$2"; shift 2
            ;;
        --min-inlier)
            [[ $# -ge 2 ]] || { echo "[ERROR] --min-inlier 缺少参数" >&2; exit 2; }
            MIN_INLIER="$2"; shift 2
            ;;
        --max-error)
            [[ $# -ge 2 ]] || { echo "[ERROR] --max-error 缺少参数" >&2; exit 2; }
            MAX_ERROR="$2"; shift 2
            ;;
        --max-retries)
            [[ $# -ge 2 ]] || { echo "[ERROR] --max-retries 缺少参数" >&2; exit 2; }
            MAX_RETRIES="$2"; shift 2
            ;;
        --max-pos-error)
            [[ $# -ge 2 ]] || { echo "[ERROR] --max-pos-error 缺少参数" >&2; exit 2; }
            MAX_POS_ERROR="$2"; shift 2
            ;;
        --max-yaw-error)
            [[ $# -ge 2 ]] || { echo "[ERROR] --max-yaw-error 缺少参数" >&2; exit 2; }
            MAX_YAW_ERROR="$2"; shift 2
            ;;
        --no-cleanup)
            CLEANUP_OLD_ROS2=false; shift
            ;;
        -h|--help)
            usage; exit 0
            ;;
        *)
            echo "[ERROR] 未知参数：$1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

[[ -n "$SCENE" ]] || { echo "[ERROR] 必须指定 --scene" >&2; usage >&2; exit 2; }
SCENE="$(normalize_scene "$SCENE")" \
    || { echo "[ERROR] --scene 只能是 workshop/工作室 或 garage/地下车库" >&2; exit 2; }
[[ -n "$POINT_ID" ]] || { echo "[ERROR] 必须指定 --point" >&2; usage >&2; exit 2; }
[[ "$ENGINE" == "FPFH_RANSAC" || "$ENGINE" == "BBS" ]] \
    || { echo "[ERROR] --engine 只能是 FPFH_RANSAC 或 BBS" >&2; exit 2; }

if [[ -z "$MAX_POS_ERROR" ]]; then
    MAX_POS_ERROR="1.5"
fi

if [[ -z "$TRIAL_ID" ]]; then
    TRIAL_ID="$(date '+%Y%m%d_%H%M%S')"
fi

SCENE_SAFE="$(sanitize_label "$SCENE")"
POINT_SAFE="$(sanitize_label "$POINT_ID")"
TRIAL_SAFE="$(sanitize_label "$TRIAL_ID")"
RUN_DIR="$RESULT_ROOT/${SCENE_SAFE}_${POINT_SAFE}_${TRIAL_SAFE}"
if [[ -e "$RUN_DIR" ]]; then
    RUN_DIR="${RUN_DIR}_$(date '+%N')"
fi
mkdir -p "$RUN_DIR/ros" "$PCD_DIR" "$RESULT_ROOT"
RAW_LOG="$RUN_DIR/terminal.log"
export ROS_LOG_DIR="$RUN_DIR/ros"
START_NS="$(now_ns)"

log "============================================================"
log "  HDL 纯点云重定位测试"
log "  场景/点位 : $SCENE / $POINT_ID"
log "  试验编号  : $TRIAL_ID"
log "  数据库    : $DB_PATH"
log "  导航测试  : 禁用（mode=localization，不启动 Nav2）"
log "  结果目录  : $RUN_DIR"
log "============================================================"

for required in \
    /opt/ros/humble/setup.bash \
    "$WS_DIR/install/setup.bash" \
    "$EXPORTER" \
    "$STARTUP_CHECKER" \
    "$LOCALIZATION_CLIENT"; do
    if [[ ! -f "$required" ]]; then
        FAILURE_REASON="缺少文件：$required"
        log "[ERROR] $FAILURE_REASON"
        finish 1
    fi
done
if [[ ! -s "$DB_PATH" ]]; then
    FAILURE_REASON="数据库不存在或为空：$DB_PATH"
    log "[ERROR] $FAILURE_REASON"
    finish 1
fi

set +u
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"
set -u

if [[ "$CLEANUP_OLD_ROS2" == "true" ]]; then
    log "[TEST] 启动前清理旧 ROS 2 进程：pkill -f ros2"
    pkill -f ros2 >/dev/null 2>&1 || true
    sleep 1.0
else
    log "[WARN] 已使用 --no-cleanup；重复 publisher 可能导致测试失败。"
fi

log "[TEST] 阶段1：从当前数据库导出本次测试专用 PCD..."
EXPORT_START_NS="$(now_ns)"
set +e
EXPORT_OUTPUT="$(python3 -u "$EXPORTER" "$DB_PATH" "$PCD_DIR" 2>&1)"
EXPORT_STATUS="$?"
set -e
EXPORT_END_NS="$(now_ns)"
printf '%s\n' "$EXPORT_OUTPUT" | tee -a "$RAW_LOG"
if [[ "$EXPORT_STATUS" -ne 0 ]]; then
    FAILURE_REASON="PCD 导出失败"
    finish 1
fi
PCD_PATH="$(
    sed -n 's/^  PCD: //p' <<<"$EXPORT_OUTPUT" | tail -n 1
)"
PCD_POINTS="$(
    sed -n 's/^\[OK\] Wrote \([0-9][0-9]*\) points to .*/\1/p' \
        <<<"$EXPORT_OUTPUT" | tail -n 1
)"
if [[ -z "$PCD_PATH" || ! -s "$PCD_PATH" ]]; then
    FAILURE_REASON="导出脚本未生成有效 PCD"
    log "[ERROR] $FAILURE_REASON"
    finish 1
fi
PCD_PATH="$(realpath "$PCD_PATH")"
log "[OK] 本次测试 PCD：$PCD_PATH（${PCD_POINTS:-未知} 点）"

log "[TEST] 阶段2：启动 Livox、FAST-LIO、RTAB-Map 和 HDL；不启动 Nav2..."
setsid ros2 launch robot_bringup global_localization_bringup.launch.py \
    mode:=localization \
    database_path:="$DB_PATH" \
    enable_rviz:=false \
    enable_collision_monitor:=false \
    use_edited_map:=false \
    > >(tee -a "$RAW_LOG") 2>&1 &
STACK_PID="$!"

SENSOR_START_NS="$(now_ns)"
set +e
python3 -u "$STARTUP_CHECKER" \
    --sensors \
    --timeout "$SENSOR_TIMEOUT" \
    --max-age "$MAX_SCAN_AGE" \
    --max-future "$MAX_FUTURE_SKEW" \
    --required-samples "$REQUIRED_SAMPLES" \
    2>&1 | tee -a "$RAW_LOG"
SENSOR_STATUS="${PIPESTATUS[0]}"
set -e
SENSOR_END_NS="$(now_ns)"
if [[ "$SENSOR_STATUS" -ne 0 ]]; then
    FAILURE_REASON="点云、里程计或 TF 启动门控失败"
    finish 1
fi
if ! kill -0 "$STACK_PID" 2>/dev/null; then
    FAILURE_REASON="纯重定位 launch 提前退出"
    log "[ERROR] $FAILURE_REASON"
    finish 1
fi

log "[TEST] 阶段3：执行 HDL 匹配并记录位置和耗时..."
CLIENT_START_NS="$(now_ns)"
set +e
python3 -u "$LOCALIZATION_CLIENT" "$PCD_PATH" \
    --engine "$ENGINE" \
    --min-inlier "$MIN_INLIER" \
    --max-error "$MAX_ERROR" \
    --max-retries "$MAX_RETRIES" \
    --scan-timeout "$SCAN_TIMEOUT" \
    --max-scan-age "$MAX_SCAN_AGE" \
    --max-future-skew "$MAX_FUTURE_SKEW" \
    --subscriber-timeout "$SUBSCRIBER_TIMEOUT" \
    2>&1 | tee -a "$RAW_LOG"
CLIENT_STATUS="${PIPESTATUS[0]}"
set -e
CLIENT_END_NS="$(now_ns)"
END_NS="$CLIENT_END_NS"

if [[ "$CLIENT_STATUS" -ne 0 ]]; then
    FAILURE_REASON="HDL 未得到满足门限的位姿"
    finish 1
fi

log "[OK] HDL 位姿已得到；测试到此结束，不启动或激活任何导航节点。"
finish 0
