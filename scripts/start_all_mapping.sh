#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ===== User config (override with env vars) =====
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"
LIDAR_TOPIC="${LIDAR_TOPIC:-/sensors/lidar/points_deskewed}"
ODOM_TOPIC="${ODOM_TOPIC:-/odometry/local}"
FRAME_ID="${FRAME_ID:-auto}"
ODOM_FRAME_ID="${ODOM_FRAME_ID:-odom}"
RVIZ="${RVIZ:-false}"
RTABMAP_VIZ="${RTABMAP_VIZ:-true}"
WAIT_TIMEOUT_SEC="${WAIT_TIMEOUT_SEC:-40}"
CHECK_INTERVAL_SEC="${CHECK_INTERVAL_SEC:-1}"

# Comma-separated fallback candidates for lidar point cloud topic
LIDAR_TOPIC_CANDIDATES="${LIDAR_TOPIC_CANDIDATES:-/sensors/lidar/points_deskewed,/livox/lidar,/livox/pointcloud2,/livox/point_cloud,/livox/points,/points_raw,/pointcloud}"

# Upstream startup toggles
START_LIDAR="${START_LIDAR:-1}"
START_ODOM="${START_ODOM:-0}"

# Upstream startup commands (edit for your robot)
# If LIDAR_CMD is empty, script auto-selects a Livox launch.
LIDAR_CMD="${LIDAR_CMD:-}"
LIDAR_CMD_POINTCLOUD2="${LIDAR_CMD_POINTCLOUD2:-}"
LIDAR_CMD_CUSTOM="${LIDAR_CMD_CUSTOM:-ros2 launch livox_ros_driver2 msg_MID360_launch.py}"
ODOM_CMD="${ODOM_CMD:-}"
# 1=prefer PointCloud2 launch for RTAB-Map compatibility
LIVOX_FORCE_POINTCLOUD2="${LIVOX_FORCE_POINTCLOUD2:-1}"
# 1=if CustomMsg is detected at runtime, auto-restart lidar with PointCloud2 launch once
LIVOX_AUTO_RESTART_TO_POINTCLOUD2="${LIVOX_AUTO_RESTART_TO_POINTCLOUD2:-1}"
# 1=if PointCloud2 topic has publishers but echo check fails (QoS/mixed-type), allow pass
LIDAR_ALLOW_PUBLISHER_ONLY="${LIDAR_ALLOW_PUBLISHER_ONLY:-1}"
# 1=before starting lidar, kill stale livox_ros_driver2/livox_lidar_publisher processes
KILL_STALE_LIVOX_PROCESSES="${KILL_STALE_LIVOX_PROCESSES:-1}"
# Maximum allowed PointCloud2 publishers on selected lidar topic
MAX_LIDAR_PUBLISHERS="${MAX_LIDAR_PUBLISHERS:-1}"
LIVOX_FRAME_ID="${LIVOX_FRAME_ID:-livox_frame}"
LIVOX_PUBLISH_FREQ="${LIVOX_PUBLISH_FREQ:-10.0}"
LIVOX_CMDLINE_BD_CODE="${LIVOX_CMDLINE_BD_CODE:-livox0000000001}"

# Optional extra setup files (comma-separated), e.g.
# EXTRA_SETUP_FILES="/home/wheeltec/install/setup.bash,/home/wheeltec/ws_xxx/install/setup.bash"
EXTRA_SETUP_FILES="${EXTRA_SETUP_FILES:-}"
AUTO_SOURCE_LIVOX_SETUP="${AUTO_SOURCE_LIVOX_SETUP:-1}"
LIVOX_SETUP_SEARCH_ROOTS="${LIVOX_SETUP_SEARCH_ROOTS:-/home/$USER,/root,$ROOT_DIR/..}"

# Livox runtime shared library checks
LIVOX_REQUIRED_LIB="${LIVOX_REQUIRED_LIB:-liblivox_lidar_sdk_shared.so}"
LIVOX_LIB_HINTS="${LIVOX_LIB_HINTS:-$ROOT_DIR/install/livox_sdk2/lib,$ROOT_DIR/install/lib,/home/$USER/install/livox_sdk2/lib,/home/$USER/install/lib,/usr/local/lib,/usr/lib/aarch64-linux-gnu,/usr/lib/x86_64-linux-gnu,/usr/lib}"

# RTAB-Map startup script and mode passed through
RTABMAP_SCRIPT="${RTABMAP_SCRIPT:-$ROOT_DIR/scripts/start_rtabmap_mapping.sh}"
USE_ICP_MODE="${USE_ICP_MODE:-auto}"

LOG_DIR="${LOG_DIR:-$ROOT_DIR/logs/bringup_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$LOG_DIR"
LIDAR_PID_FILE="$LOG_DIR/lidar.pid"
LIDAR_MONITOR_ENABLE="${LIDAR_MONITOR_ENABLE:-1}"
LIDAR_MONITOR_INTERVAL_SEC="${LIDAR_MONITOR_INTERVAL_SEC:-2}"
LIDAR_STALL_TIMEOUT_SEC="${LIDAR_STALL_TIMEOUT_SEC:-5}"
LIDAR_MAX_AUTO_RESTARTS="${LIDAR_MAX_AUTO_RESTARTS:-3}"
DEFAULT_DESKEWED_LIDAR_TOPIC="/sensors/lidar/points_deskewed"
DEFAULT_RAW_LIVOX_TOPIC="/livox/lidar"

# setup.bash may reference unset vars when nounset is enabled
set +u
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
source "$ROOT_DIR/install/setup.bash"
set -u

pids=()
names=()
LIDAR_PID=""
LIDAR_RESTARTED_TO_POINTCLOUD2="0"
SOURCED_SETUP_FILES=()

source_setup_file() {
  local sf="$1"
  [[ -n "$sf" ]] || return 1
  [[ -f "$sf" ]] || return 1

  local prev
  for prev in "${SOURCED_SETUP_FILES[@]:-}"; do
    [[ "$prev" == "$sf" ]] && return 0
  done

  echo "[INFO] Source setup: $sf"
  set +u
  source "$sf"
  set -u
  SOURCED_SETUP_FILES+=("$sf")
}

source_extra_setups() {
  [[ -n "$EXTRA_SETUP_FILES" ]] || return 0

  local sf
  IFS=',' read -r -a setup_arr <<< "$EXTRA_SETUP_FILES"
  for sf in "${setup_arr[@]}"; do
    [[ -z "$sf" ]] && continue
    if [[ -f "$sf" ]]; then
      source_setup_file "$sf"
    else
      echo "[WARN] Extra setup file not found: $sf"
    fi
  done
}

source_extra_setups

cleanup() {
  local code=$?
  if [[ ${#pids[@]} -gt 0 ]]; then
    echo "[INFO] Stopping background processes..."
    for i in "${!pids[@]}"; do
      if kill -0 "${pids[$i]}" >/dev/null 2>&1; then
        echo "[INFO] Kill ${names[$i]} (pid=${pids[$i]})"
        kill "${pids[$i]}" >/dev/null 2>&1 || true
      fi
    done
    wait || true
  fi
  if [[ -f "$LIDAR_PID_FILE" ]]; then
    local pid_from_file
    pid_from_file="$(cat "$LIDAR_PID_FILE" 2>/dev/null || true)"
    if [[ -n "$pid_from_file" ]] && kill -0 "$pid_from_file" >/dev/null 2>&1; then
      echo "[INFO] Kill lidar from pid file (pid=$pid_from_file)"
      kill "$pid_from_file" >/dev/null 2>&1 || true
    fi
    rm -f "$LIDAR_PID_FILE"
  fi
  exit "$code"
}
trap cleanup EXIT INT TERM

start_bg() {
  local name="$1"
  local cmd="$2"
  local log_file="$LOG_DIR/${name}.log"

  echo "[STEP] Starting $name"
  echo "[INFO] cmd: $cmd"
  echo "[INFO] log: $log_file"

  bash -lc "$cmd" >"$log_file" 2>&1 &
  local pid=$!
  pids+=("$pid")
  names+=("$name")
  if [[ "$name" == "lidar" ]]; then
    LIDAR_PID="$pid"
    echo "$pid" > "$LIDAR_PID_FILE"
  fi
  echo "[INFO] $name pid=$pid"
}

get_lidar_pid() {
  if [[ -f "$LIDAR_PID_FILE" ]]; then
    cat "$LIDAR_PID_FILE" 2>/dev/null || true
    return 0
  fi
  printf '%s\n' "$LIDAR_PID"
}

stop_lidar_process() {
  local pid
  pid="$(get_lidar_pid)"
  if [[ -n "$pid" ]] && kill -0 "$pid" >/dev/null 2>&1; then
    echo "[INFO] Stopping lidar pid=$pid"
    kill "$pid" >/dev/null 2>&1 || true
    sleep 1
  fi
}

cleanup_stale_livox_processes() {
  [[ "$KILL_STALE_LIVOX_PROCESSES" == "1" ]] || return 0

  if ! pgrep -f "livox_ros_driver2|livox_lidar_publisher" >/dev/null 2>&1; then
    return 0
  fi

  echo "[WARN] Found stale Livox processes, cleaning them before startup..."
  pgrep -af "livox_ros_driver2|livox_lidar_publisher" || true
  pkill -f livox_lidar_publisher >/dev/null 2>&1 || true
  pkill -f livox_ros_driver2 >/dev/null 2>&1 || true
  sleep 2
}

prepend_ld_path() {
  local dir="$1"
  [[ -z "$dir" ]] && return 0
  if [[ ":${LD_LIBRARY_PATH:-}:" != *":$dir:"* ]]; then
    export LD_LIBRARY_PATH="$dir:${LD_LIBRARY_PATH:-}"
  fi
}

ensure_shared_lib_visible() {
  local lib_name="$1"

  if ldconfig -p 2>/dev/null | grep -q "$lib_name"; then
    return 0
  fi

  IFS=',' read -r -a hint_arr <<< "$LIVOX_LIB_HINTS"
  local d
  for d in "${hint_arr[@]}"; do
    [[ -z "$d" ]] && continue
    if [[ -f "$d/$lib_name" ]]; then
      prepend_ld_path "$d"
      echo "[INFO] Found $lib_name in hint dir: $d"
      return 0
    fi
  done

  local found
  found="$(find "$ROOT_DIR" "/home/$USER" /usr/local /usr -type f -name "$lib_name" 2>/dev/null | head -n 1 || true)"
  if [[ -n "$found" ]]; then
    local lib_dir
    lib_dir="$(dirname "$found")"
    prepend_ld_path "$lib_dir"
    echo "[INFO] Found $lib_name by search: $found"
    return 0
  fi

  echo "[ERROR] Missing required shared library: $lib_name" >&2
  echo "[HINT] Build/install Livox SDK or export LD_LIBRARY_PATH to the SDK lib dir." >&2
  echo "[HINT] You can set LIVOX_LIB_HINTS to include custom directories." >&2
  return 1
}

topic_exists_in_text() {
  local topic="$1"
  local topics_text="$2"
  grep -Fxq "$topic" <<< "$topics_text"
}

topic_info_verbose() {
  local topic="$1"
  ros2 topic info "$topic" -v 2>/dev/null | tr -d '\r' || true
}

topic_supports_type() {
  local topic="$1"
  local wanted_type="$2"
  topic_info_verbose "$topic" | grep -Fq "Topic type: $wanted_type"
}

count_topic_publishers_of_type() {
  local topic="$1"
  local wanted_type="${2:-}"
  local info count
  info="$(topic_info_verbose "$topic")"
  [[ -n "$info" ]] || { echo 0; return 0; }

  if [[ -z "$wanted_type" ]]; then
    count="$(awk -F': ' '/Publisher count/ {print $2; exit}' <<< "$info" || true)"
    [[ "${count:-0}" =~ ^[0-9]+$ ]] || count=0
    echo "$count"
    return 0
  fi

  awk -v t="$wanted_type" '
    BEGIN { c=0; in_pub=0; cur_type="" }
    /^Topic type:/ {
      cur_type=$0
      sub(/^Topic type: /, "", cur_type)
    }
    /^Endpoint type: PUBLISHER$/ {
      if (cur_type == t) c++
    }
    END { print c }
  ' <<< "$info"
}

has_topic_publishers() {
  local topic="$1"
  local count
  count="$(count_topic_publishers_of_type "$topic")"
  [[ "${count:-0}" =~ ^[0-9]+$ ]] || count=0
  (( count > 0 ))
}

has_topic_publishers_of_type() {
  local topic="$1"
  local wanted_type="$2"
  local count
  count="$(count_topic_publishers_of_type "$topic" "$wanted_type")"
  [[ "${count:-0}" =~ ^[0-9]+$ ]] || count=0
  (( count > 0 ))
}

ensure_single_lidar_publisher() {
  local topic="$1"
  local count
  count="$(count_topic_publishers_of_type "$topic" "sensor_msgs/msg/PointCloud2")"
  [[ "${count:-0}" =~ ^[0-9]+$ ]] || count=0

  if (( MAX_LIDAR_PUBLISHERS > 0 && count > MAX_LIDAR_PUBLISHERS )); then
    echo "[ERROR] Lidar topic has too many PointCloud2 publishers: $topic (count=$count, limit=$MAX_LIDAR_PUBLISHERS)" >&2
    echo "[HINT] Kill duplicate Livox processes, then relaunch with a single driver instance." >&2
    echo "[HINT] Example:" >&2
    echo "[HINT]   pkill -f livox_ros_driver2" >&2
    echo "[HINT]   pkill -f livox_lidar_publisher" >&2
    echo "[HINT]   ros2 launch livox_ros_driver2 rviz_MID360_launch.py" >&2
    topic_info_verbose "$topic" >&2 || true
    return 1
  fi

  echo "[INFO] Lidar publisher count on $topic: $count"
}

has_topic_data() {
  local topic="$1"
  local msg_type="${2:-}"

  if [[ -n "$msg_type" ]]; then
    timeout 3s ros2 topic echo "$topic" "$msg_type" --once >/dev/null 2>&1 && return 0
    timeout 3s ros2 topic echo "$topic" "$msg_type" --once --qos-reliability best_effort --qos-durability volatile >/dev/null 2>&1 && return 0
    return 1
  fi

  timeout 3s ros2 topic echo "$topic" --once >/dev/null 2>&1 && return 0
  timeout 3s ros2 topic echo "$topic" --once --qos-reliability best_effort --qos-durability volatile >/dev/null 2>&1 && return 0
  return 1
}

print_lidar_related_topics() {
  echo "[INFO] Available topics (with type, filtered by lidar/point/cloud):"
  local topics
  topics="$(ros2 topic list -t 2>/dev/null || true)"
  if [[ -z "$topics" ]]; then
    echo "[WARN] ros2 topic list returned no topics."
    echo "[INFO] Available nodes:"
    ros2 node list 2>/dev/null || true
    return 0
  fi
  grep -Ei 'lidar|point|cloud|livox' <<< "$topics" || true
}

adjust_lidar_topic_defaults() {
  [[ "$START_ODOM" == "1" ]] && return 0
  [[ "$LIDAR_TOPIC" == "$DEFAULT_DESKEWED_LIDAR_TOPIC" ]] || return 0

  echo "[WARN] START_ODOM=0, but preferred lidar topic is the deskewed cloud: $LIDAR_TOPIC"
  echo "[WARN] That topic is usually produced by an upstream odom/deskew pipeline."

  local t
  IFS=',' read -r -a candidate_arr <<< "$LIDAR_TOPIC_CANDIDATES"
  for t in "${candidate_arr[@]}"; do
    [[ -z "$t" ]] && continue
    [[ "$t" == "$DEFAULT_DESKEWED_LIDAR_TOPIC" ]] && continue
    LIDAR_TOPIC="$t"
    echo "[INFO] Auto-switch preferred lidar topic to raw PointCloud2 candidate: $LIDAR_TOPIC"
    return 0
  done

  LIDAR_TOPIC="$DEFAULT_RAW_LIVOX_TOPIC"
  echo "[INFO] Auto-switch preferred lidar topic to fallback raw Livox topic: $LIDAR_TOPIC"
}

find_livox_custom_topic() {
  local topics
  topics="$(ros2 topic list -t 2>/dev/null || true)"
  awk '{print $1" "$2}' <<< "$topics" | grep -E " livox_ros_driver2/msg/CustomMsg$" | head -n 1 | awk '{print $1}' || true
}

get_livox_pkg_prefix() {
  ros2 pkg prefix livox_ros_driver2 2>/dev/null | tr -d '\r' || true
}

can_setup_file_provide_livox_pkg() {
  local sf="$1"
  [[ -f "$sf" ]] || return 1

  bash -lc "set +u; source '/opt/ros/${ROS_DISTRO_NAME}/setup.bash' >/dev/null 2>&1; source '$sf' >/dev/null 2>&1; ros2 pkg prefix livox_ros_driver2 >/dev/null 2>&1"
}

auto_source_livox_setup() {
  [[ "$AUTO_SOURCE_LIVOX_SETUP" == "1" ]] || return 1

  local roots=()
  local root
  IFS=',' read -r -a roots <<< "$LIVOX_SETUP_SEARCH_ROOTS"

  local candidates=()
  while IFS= read -r root; do
    [[ -d "$root" ]] || continue
    while IFS= read -r sf; do
      [[ -n "$sf" ]] && candidates+=("$sf")
    done < <(find "$root" -maxdepth 5 \( -path '*/install/setup.bash' -o -path '*/devel/setup.bash' \) 2>/dev/null | sort)
  done < <(printf '%s\n' "${roots[@]}")

  local sf
  for sf in "${candidates[@]}"; do
    if can_setup_file_provide_livox_pkg "$sf"; then
      echo "[INFO] Auto-detected livox_ros_driver2 overlay: $sf"
      source_setup_file "$sf"
      return 0
    fi
  done

  return 1
}

ensure_livox_pkg_available() {
  [[ "$START_LIDAR" == "1" ]] || return 0
  if [[ -n "$LIDAR_CMD" && "$LIDAR_CMD" != *"livox_ros_driver2"* ]]; then
    return 0
  fi

  local prefix
  prefix="$(get_livox_pkg_prefix)"
  if [[ -n "$prefix" ]]; then
    echo "[INFO] livox_ros_driver2 prefix: $prefix"
    return 0
  fi

  if auto_source_livox_setup; then
    prefix="$(get_livox_pkg_prefix)"
    if [[ -n "$prefix" ]]; then
      echo "[INFO] livox_ros_driver2 prefix: $prefix"
      return 0
    fi
  fi

  echo "[ERROR] livox_ros_driver2 package not found in current ROS environment." >&2
  echo "[HINT] Source the workspace containing livox_ros_driver2 before running, or set EXTRA_SETUP_FILES." >&2
  echo "[HINT] Example:" >&2
  echo "[HINT]   EXTRA_SETUP_FILES=/path/to/ws/install/setup.bash bash scripts/start_all_mapping.sh" >&2
  echo "[HINT] Search manually:" >&2
  echo "[HINT]   find /home/$USER /root -maxdepth 5 \\( -path '*/install/setup.bash' -o -path '*/devel/setup.bash' \\)" >&2
  return 1
}

build_livox_pointcloud2_cmd() {
  local prefix config_path
  prefix="$(get_livox_pkg_prefix)"
  [[ -n "$prefix" ]] || return 1
  config_path="$prefix/share/livox_ros_driver2/config/MID360_config.json"
  [[ -f "$config_path" ]] || return 1

  printf '%s\n' "ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args -r __node:=livox_lidar_publisher -p xfer_format:=0 -p multi_topic:=0 -p data_src:=0 -p publish_freq:=${LIVOX_PUBLISH_FREQ} -p output_data_type:=0 -p frame_id:=${LIVOX_FRAME_ID} -p user_config_path:=${config_path} -p cmdline_input_bd_code:=${LIVOX_CMDLINE_BD_CODE}"
}

livox_launch_exists() {
  local launch_file="$1"
  local prefix
  prefix="$(get_livox_pkg_prefix)"
  [[ -n "$prefix" ]] || return 1
  find "$prefix/share/livox_ros_driver2" -maxdepth 4 \( -type f -o -type l \) -name "$launch_file" 2>/dev/null | head -n1 | grep -q .
}

lidar_cmd_is_custom_launch() {
  [[ "$LIDAR_CMD" == *"msg_MID360_launch.py"* || "$LIDAR_CMD" == *"msg_HAP_launch.py"* ]]
}

lidar_cmd_is_pointcloud2_launch() {
  [[ "$LIDAR_CMD" == *"rviz_MID360_launch.py"* || "$LIDAR_CMD" == *"rviz_HAP_launch.py"* ]]
}

restart_lidar_with_pointcloud2() {
  if [[ "$START_LIDAR" != "1" ]]; then
    return 1
  fi
  if [[ "$LIDAR_RESTARTED_TO_POINTCLOUD2" == "1" ]]; then
    return 1
  fi
  if ! livox_launch_exists "rviz_MID360_launch.py"; then
    return 1
  fi

  echo "[WARN] CustomMsg detected, auto-restarting lidar with PointCloud2 launch."
  LIDAR_RESTARTED_TO_POINTCLOUD2="1"
  LIDAR_CMD="$LIDAR_CMD_POINTCLOUD2"

  stop_lidar_process
  start_bg "lidar" "$LIDAR_CMD"
  return 0
}

monitor_lidar_stream() {
  [[ "$LIDAR_MONITOR_ENABLE" == "1" ]] || return 0
  [[ "$START_LIDAR" == "1" ]] || return 0
  [[ "$LIDAR_CMD" == *"livox_ros_driver2"* ]] || return 0

  local last_ok=$SECONDS
  local restart_count=0

  echo "[INFO] Lidar stream monitor enabled: interval=${LIDAR_MONITOR_INTERVAL_SEC}s stall=${LIDAR_STALL_TIMEOUT_SEC}s max_restart=${LIDAR_MAX_AUTO_RESTARTS}"

  while true; do
    sleep "$LIDAR_MONITOR_INTERVAL_SEC"

    if has_topic_data "$LIDAR_TOPIC" "sensor_msgs/msg/PointCloud2"; then
      last_ok=$SECONDS
      continue
    fi

    if (( SECONDS - last_ok < LIDAR_STALL_TIMEOUT_SEC )); then
      continue
    fi

    if (( restart_count >= LIDAR_MAX_AUTO_RESTARTS )); then
      echo "[ERROR] Lidar stream stalled and auto-restart limit reached ($LIDAR_MAX_AUTO_RESTARTS)."
      [[ -f "$LOG_DIR/lidar.log" ]] && tail -n 80 "$LOG_DIR/lidar.log" || true
      return 1
    fi

    restart_count=$((restart_count + 1))
    echo "[WARN] Lidar topic stalled for ${LIDAR_STALL_TIMEOUT_SEC}s, auto-restarting lidar (${restart_count}/${LIDAR_MAX_AUTO_RESTARTS})..."
    [[ -f "$LOG_DIR/lidar.log" ]] && tail -n 80 "$LOG_DIR/lidar.log" || true
    stop_lidar_process
    start_bg "lidar" "$LIDAR_CMD"
    select_lidar_topic "$WAIT_TIMEOUT_SEC" || true
    ensure_single_lidar_publisher "$LIDAR_TOPIC" || true
    last_ok=$SECONDS
  done
}

select_default_lidar_cmd() {
  if [[ -n "$LIDAR_CMD" ]]; then
    return 0
  fi
  if [[ "$LIVOX_FORCE_POINTCLOUD2" == "1" ]] && livox_launch_exists "rviz_MID360_launch.py"; then
    if [[ -n "$LIDAR_CMD_POINTCLOUD2" ]]; then
      LIDAR_CMD="$LIDAR_CMD_POINTCLOUD2"
    else
      LIDAR_CMD="$(build_livox_pointcloud2_cmd || true)"
      [[ -n "$LIDAR_CMD" ]] || LIDAR_CMD="ros2 launch livox_ros_driver2 rviz_MID360_launch.py"
    fi
    return 0
  fi
  if livox_launch_exists "msg_MID360_launch.py"; then
    LIDAR_CMD="$LIDAR_CMD_CUSTOM"
    return 0
  fi
  LIDAR_CMD="$LIDAR_CMD_CUSTOM"
}

select_lidar_topic() {
  local timeout_sec="$1"
  local deadline=$((SECONDS + timeout_sec))
  local next_report=$SECONDS

  IFS=',' read -r -a candidate_arr <<< "$LIDAR_TOPIC_CANDIDATES"

  echo "[STEP] Waiting lidar topic data (timeout=${timeout_sec}s)"
  echo "[INFO] Preferred topic: $LIDAR_TOPIC"
  echo "[INFO] Candidates     : $LIDAR_TOPIC_CANDIDATES"

  while (( SECONDS < deadline )); do
    if [[ -n "$LIDAR_PID" ]] && ! kill -0 "$LIDAR_PID" >/dev/null 2>&1; then
      echo "[ERROR] lidar process exited early (pid=$LIDAR_PID)."
      [[ -f "$LOG_DIR/lidar.log" ]] && tail -n 120 "$LOG_DIR/lidar.log" || true
      return 1
    fi

    local topics
    topics="$(ros2 topic list 2>/dev/null || true)"

    # 1) preferred topic first (must include PointCloud2 type)
    if topic_exists_in_text "$LIDAR_TOPIC" "$topics"; then
      if topic_supports_type "$LIDAR_TOPIC" "sensor_msgs/msg/PointCloud2"; then
        if has_topic_data "$LIDAR_TOPIC" "sensor_msgs/msg/PointCloud2"; then
          echo "[OK] Lidar ready on preferred topic: $LIDAR_TOPIC"
          return 0
        fi
        if [[ "$LIDAR_ALLOW_PUBLISHER_ONLY" == "1" ]] && has_topic_publishers_of_type "$LIDAR_TOPIC" "sensor_msgs/msg/PointCloud2"; then
          echo "[WARN] Lidar topic has PointCloud2 publishers but echo check timed out. Continue anyway: $LIDAR_TOPIC"
          return 0
        fi
      fi
    fi

    # 2) fallback candidates (must include PointCloud2 type)
    for t in "${candidate_arr[@]}"; do
      [[ -z "$t" ]] && continue
      [[ "$t" == "$LIDAR_TOPIC" ]] && continue
      if ! topic_exists_in_text "$t" "$topics"; then
        continue
      fi
      if ! topic_supports_type "$t" "sensor_msgs/msg/PointCloud2"; then
        continue
      fi
      if has_topic_data "$t" "sensor_msgs/msg/PointCloud2"; then
        echo "[WARN] Preferred topic has no data, fallback to: $t"
        LIDAR_TOPIC="$t"
        return 0
      fi
      if [[ "$LIDAR_ALLOW_PUBLISHER_ONLY" == "1" ]] && has_topic_publishers_of_type "$t" "sensor_msgs/msg/PointCloud2"; then
        echo "[WARN] Fallback topic has PointCloud2 publishers but echo check timed out, use it anyway: $t"
        LIDAR_TOPIC="$t"
        return 0
      fi
    done

    if (( SECONDS >= next_report )); then
      local elapsed=$((timeout_sec - (deadline - SECONDS)))
      echo "[INFO] waiting... ${elapsed}s/${timeout_sec}s"
      print_lidar_related_topics

      local custom_topic
      custom_topic="$(find_livox_custom_topic)"
      if [[ -n "$custom_topic" ]]; then
        echo "[WARN] Detected Livox CustomMsg topic: $custom_topic"
        echo "[HINT] RTAB-Map scan_cloud needs sensor_msgs/msg/PointCloud2."
        if topic_supports_type "$custom_topic" "sensor_msgs/msg/PointCloud2"; then
          echo "[HINT] Same topic also provides PointCloud2; mixed-type topic detected."
          echo "[HINT] Script will accept it if PointCloud2 publishers exist."
        elif [[ "$LIVOX_AUTO_RESTART_TO_POINTCLOUD2" == "1" ]] && ! lidar_cmd_is_pointcloud2_launch; then
          if restart_lidar_with_pointcloud2; then
            deadline=$((SECONDS + timeout_sec))
            echo "[INFO] Restarted lidar command: $LIDAR_CMD"
            echo "[INFO] Retry waiting PointCloud2 topic..."
          else
            echo "[HINT] Auto-restart unavailable, run PointCloud2 launch manually:"
            echo "[HINT]   ros2 launch livox_ros_driver2 rviz_MID360_launch.py"
          fi
        else
          echo "[HINT] Use PointCloud2 launch: LIDAR_CMD='ros2 launch livox_ros_driver2 rviz_MID360_launch.py'"
        fi
      fi
      next_report=$((SECONDS + 5))
    fi

    sleep "$CHECK_INTERVAL_SEC"
  done

  echo "[ERROR] Lidar topic timeout after ${timeout_sec}s"
  print_lidar_related_topics
  if [[ -f "$LOG_DIR/lidar.log" ]]; then
    echo "[INFO] Last 120 lines of lidar.log:"
    tail -n 120 "$LOG_DIR/lidar.log" || true
  fi
  return 1
}

wait_topic() {
  local topic="$1"
  local timeout_sec="$2"
  local deadline=$((SECONDS + timeout_sec))

  echo "[STEP] Waiting topic: $topic (timeout=${timeout_sec}s)"
  until has_topic_data "$topic"; do
    if (( SECONDS >= deadline )); then
      echo "[ERROR] Topic timeout: $topic"
      return 1
    fi
    sleep "$CHECK_INTERVAL_SEC"
  done
  echo "[OK] Topic ready: $topic"
}

echo "[INFO] Root           : $ROOT_DIR"
echo "[INFO] ROS distro     : $ROS_DISTRO_NAME"
echo "[INFO] Lidar topic    : $LIDAR_TOPIC"
echo "[INFO] Odom topic     : $ODOM_TOPIC"
echo "[INFO] Frame ID       : $FRAME_ID"
echo "[INFO] Odom frame     : $ODOM_FRAME_ID"
echo "[INFO] RVIZ           : $RVIZ"
echo "[INFO] RTABMAP_VIZ    : $RTABMAP_VIZ"
echo "[INFO] USE_ICP_MODE   : $USE_ICP_MODE"
echo "[INFO] Log dir        : $LOG_DIR"

adjust_lidar_topic_defaults
echo "[INFO] Effective lidar topic: $LIDAR_TOPIC"

if [[ "$START_LIDAR" == "1" ]]; then
  ensure_livox_pkg_available
  select_default_lidar_cmd
  if [[ -z "$LIDAR_CMD" ]]; then
    echo "[ERROR] START_LIDAR=1 but LIDAR_CMD is empty"
    exit 31
  fi

  if [[ "$LIVOX_FORCE_POINTCLOUD2" == "1" ]] && lidar_cmd_is_custom_launch && livox_launch_exists "rviz_MID360_launch.py"; then
    echo "[WARN] LIDAR_CMD is custom-msg launch, auto-switch to PointCloud2 launch for RTAB-Map compatibility."
    LIDAR_CMD="$LIDAR_CMD_POINTCLOUD2"
  fi

  if [[ "$LIDAR_CMD" == *"livox_ros_driver2"* ]]; then
    cleanup_stale_livox_processes
    ensure_shared_lib_visible "$LIVOX_REQUIRED_LIB"
    echo "[INFO] LD_LIBRARY_PATH (for livox) = ${LD_LIBRARY_PATH:-<empty>}"
  fi

  start_bg "lidar" "$LIDAR_CMD"
else
  echo "[INFO] Skip lidar startup (START_LIDAR=$START_LIDAR)"
fi

if [[ "$START_ODOM" == "1" ]]; then
  if [[ -z "$ODOM_CMD" ]]; then
    echo "[ERROR] START_ODOM=1 but ODOM_CMD is empty"
    exit 32
  fi
  start_bg "odom" "$ODOM_CMD"
else
  echo "[INFO] Skip odom startup (START_ODOM=$START_ODOM)"
fi

# Must have lidar data before launching RTAB-Map (with fallback topic probing)
select_lidar_topic "$WAIT_TIMEOUT_SEC"
ensure_single_lidar_publisher "$LIDAR_TOPIC"
monitor_lidar_stream &
pids+=("$!")
names+=("lidar_monitor")

# If user explicitly requires external odom, verify it now
if [[ "$USE_ICP_MODE" == "off" ]]; then
  wait_topic "$ODOM_TOPIC" "$WAIT_TIMEOUT_SEC"
fi

if [[ ! -f "$RTABMAP_SCRIPT" ]]; then
  echo "[ERROR] RTABMAP_SCRIPT not found: $RTABMAP_SCRIPT"
  exit 33
fi

if [[ ! -r "$RTABMAP_SCRIPT" ]]; then
  echo "[ERROR] RTABMAP_SCRIPT not readable: $RTABMAP_SCRIPT"
  exit 34
fi

echo "[STEP] Starting RTAB-Map"
export LIDAR_TOPIC ODOM_TOPIC FRAME_ID ODOM_FRAME_ID RVIZ RTABMAP_VIZ USE_ICP_MODE
bash "$RTABMAP_SCRIPT"
