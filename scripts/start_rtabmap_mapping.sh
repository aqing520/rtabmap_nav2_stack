#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# User-tunable parameters (can be overridden by env vars)
NAMESPACE="${NAMESPACE:-rtabmap}"
LOCALIZATION="${LOCALIZATION:-false}"
FRAME_ID="${FRAME_ID:-auto}"
LIDAR_TOPIC="${LIDAR_TOPIC:-/sensors/lidar/points_deskewed}"
ODOM_TOPIC="${ODOM_TOPIC:-/odometry/local}"
ODOM_FRAME_ID="${ODOM_FRAME_ID:-odom}"

# ICP mode: auto | on | off
USE_ICP_MODE="${USE_ICP_MODE:-auto}"

APPROX_SYNC_MAX_INTERVAL="${APPROX_SYNC_MAX_INTERVAL:-0.05}"
TOPIC_QUEUE_SIZE="${TOPIC_QUEUE_SIZE:-10}"
SYNC_QUEUE_SIZE="${SYNC_QUEUE_SIZE:-10}"
QOS="${QOS:-2}"

# Backward-compatible shared ICP defaults. Specific odom/map values below may override them.
ICP_VOXEL_SIZE="${ICP_VOXEL_SIZE:-0.15}"
ICP_DOWNSAMPLING_STEP="${ICP_DOWNSAMPLING_STEP:-1}"
ICP_MAX_TRANSLATION="${ICP_MAX_TRANSLATION:-1.5}"
ICP_MAX_ROTATION="${ICP_MAX_ROTATION:-0.7}"
ICP_MAX_CORRESPONDENCE_DISTANCE="${ICP_MAX_CORRESPONDENCE_DISTANCE:-0.8}"
ICP_CORRESPONDENCE_RATIO="${ICP_CORRESPONDENCE_RATIO:-0.05}"
ICP_POINT_TO_PLANE="${ICP_POINT_TO_PLANE:-true}"
ICP_SCAN_NORMAL_K="${ICP_SCAN_NORMAL_K:-20}"
ICP_POINT_TO_PLANE_MIN_COMPLEXITY="${ICP_POINT_TO_PLANE_MIN_COMPLEXITY:-0.04}"

# ICP odometry defaults: keep local registration stable.
ODOM_ICP_VOXEL_SIZE="${ODOM_ICP_VOXEL_SIZE:-$ICP_VOXEL_SIZE}"
ODOM_ICP_DOWNSAMPLING_STEP="${ODOM_ICP_DOWNSAMPLING_STEP:-$ICP_DOWNSAMPLING_STEP}"
ODOM_ICP_MAX_TRANSLATION="${ODOM_ICP_MAX_TRANSLATION:-$ICP_MAX_TRANSLATION}"
ODOM_ICP_MAX_ROTATION="${ODOM_ICP_MAX_ROTATION:-$ICP_MAX_ROTATION}"
ODOM_ICP_MAX_CORRESPONDENCE_DISTANCE="${ODOM_ICP_MAX_CORRESPONDENCE_DISTANCE:-$ICP_MAX_CORRESPONDENCE_DISTANCE}"
ODOM_ICP_CORRESPONDENCE_RATIO="${ODOM_ICP_CORRESPONDENCE_RATIO:-$ICP_CORRESPONDENCE_RATIO}"
ODOM_ICP_POINT_TO_PLANE="${ODOM_ICP_POINT_TO_PLANE:-$ICP_POINT_TO_PLANE}"
ODOM_ICP_SCAN_NORMAL_K="${ODOM_ICP_SCAN_NORMAL_K:-$ICP_SCAN_NORMAL_K}"
ODOM_ICP_POINT_TO_PLANE_MIN_COMPLEXITY="${ODOM_ICP_POINT_TO_PLANE_MIN_COMPLEXITY:-$ICP_POINT_TO_PLANE_MIN_COMPLEXITY}"

# RTAB-Map defaults: preserve more structure for map constraints than odometry does.
MAP_ICP_VOXEL_SIZE="${MAP_ICP_VOXEL_SIZE:-0.12}"
MAP_SCAN_VOXEL_SIZE="${MAP_SCAN_VOXEL_SIZE:-$MAP_ICP_VOXEL_SIZE}"
MAP_ICP_DOWNSAMPLING_STEP="${MAP_ICP_DOWNSAMPLING_STEP:-$ICP_DOWNSAMPLING_STEP}"
MAP_ICP_MAX_TRANSLATION="${MAP_ICP_MAX_TRANSLATION:-$ICP_MAX_TRANSLATION}"
MAP_ICP_MAX_ROTATION="${MAP_ICP_MAX_ROTATION:-$ICP_MAX_ROTATION}"
MAP_ICP_MAX_CORRESPONDENCE_DISTANCE="${MAP_ICP_MAX_CORRESPONDENCE_DISTANCE:-$ICP_MAX_CORRESPONDENCE_DISTANCE}"
MAP_ICP_CORRESPONDENCE_RATIO="${MAP_ICP_CORRESPONDENCE_RATIO:-$ICP_CORRESPONDENCE_RATIO}"
MAP_ICP_POINT_TO_PLANE="${MAP_ICP_POINT_TO_PLANE:-$ICP_POINT_TO_PLANE}"
MAP_ICP_SCAN_NORMAL_K="${MAP_ICP_SCAN_NORMAL_K:-15}"
MAP_SCAN_NORMAL_K="${MAP_SCAN_NORMAL_K:-$MAP_ICP_SCAN_NORMAL_K}"
MAP_ICP_POINT_TO_PLANE_MIN_COMPLEXITY="${MAP_ICP_POINT_TO_PLANE_MIN_COMPLEXITY:-$ICP_POINT_TO_PLANE_MIN_COMPLEXITY}"
MAP_REG_STRATEGY="${MAP_REG_STRATEGY:-1}"
MAP_PROXIMITY_BY_SPACE="${MAP_PROXIMITY_BY_SPACE:-true}"
MAP_NOT_LINKED_NODES_KEPT="${MAP_NOT_LINKED_NODES_KEPT:-false}"

ODOM_RESET_COUNTDOWN="${ODOM_RESET_COUNTDOWN:-1}"
ODOM_SCAN_SUBTRACT_RADIUS="${ODOM_SCAN_SUBTRACT_RADIUS:-$ODOM_ICP_VOXEL_SIZE}"
ODOM_ARGS_EXTRA="${ODOM_ARGS_EXTRA:-}"
RTABMAP_ARGS_EXTRA="${RTABMAP_ARGS_EXTRA:-}"

RVIZ="${RVIZ:-false}"
RTABMAP_VIZ="${RTABMAP_VIZ:-true}"
ODOM_ALWAYS_PROCESS_MOST_RECENT_FRAME="${ODOM_ALWAYS_PROCESS_MOST_RECENT_FRAME:-true}"

# Wait timeout for topic self-check (seconds)
WAIT_TIMEOUT_SEC="${WAIT_TIMEOUT_SEC:-5}"

# setup.bash may reference unset vars when nounset is enabled
set +u
source /opt/ros/humble/setup.bash
source "$ROOT_DIR/install/setup.bash"
set -u

has_topic_data() {
  local topic="$1"
  # If topic has multiple types (e.g. livox publishes both CustomMsg and PointCloud2),
  # ros2 topic echo fails. Fall back to publisher count check.
  local type_count
  type_count="$(ros2 topic type "$topic" 2>/dev/null | wc -l)"
  if [[ "$type_count" -gt 1 ]]; then
    local pub_count
    pub_count="$(ros2 topic info "$topic" 2>/dev/null | grep 'Publisher count' | awk '{print $NF}')"
    [[ "${pub_count:-0}" -gt 0 ]]
    return $?
  fi
  timeout "${WAIT_TIMEOUT_SEC}s" ros2 topic echo --once "$topic" >/dev/null 2>&1
}

is_valid_frame_name() {
  local value="$1"
  [[ -n "$value" && "$value" != /* && "$value" =~ ^[A-Za-z0-9_./-]+$ ]]
}

is_valid_number() {
  local value="$1"
  [[ -n "$value" && "$value" =~ ^[-+]?[0-9]*([.][0-9]+)?([eE][-+]?[0-9]+)?$ ]]
}

number_abs_lt() {
  local value="$1"
  local threshold="$2"
  awk -v v="$value" -v t="$threshold" 'BEGIN { if (v < 0) v = -v; exit !(v < t) }'
}

get_topic_value() {
  local topic="$1"
  local field="$2"
  local value
  value="$(
    timeout "${WAIT_TIMEOUT_SEC}s" ros2 topic echo "$topic" --field "$field" --once 2>&1 \
      | tr -d '\r' \
      | sed -e '/^---$/d' -e '/^$/d' -e 's/^ *//' -e 's/ *$//' \
      | while IFS= read -r line; do
          case "$line" in
            WARNING:*|ERROR:*|*does\ not\ appear\ to\ be\ published\ yet*|*unknown\ topic*|*Could\ not\ determine\ the\ type*)
              continue
              ;;
          esac
          printf '%s\n' "$line"
          break
        done
  )"
  printf '%s\n' "$value"
}

get_topic_field() {
  local topic="$1"
  local field="$2"
  local value
  value="$(get_topic_value "$topic" "$field")"
  if is_valid_frame_name "$value"; then
    printf '%s\n' "$value"
  fi
}

collect_tf_frames() {
  {
    timeout "${WAIT_TIMEOUT_SEC}s" ros2 topic echo /tf_static --once 2>/dev/null || true
    timeout "${WAIT_TIMEOUT_SEC}s" ros2 topic echo /tf --once 2>/dev/null || true
  } | awk -F': ' '
      /^[[:space:]]*frame_id: / {print $2}
      /^[[:space:]]*child_frame_id: / {print $2}
    ' \
    | sed -e 's/^"//' -e 's/"$//' -e '/^$/d' \
    | sort -u
}

TF_FRAMES_CACHE=""
TF_FRAMES_CACHE_READY="0"

load_tf_frames_cache() {
  if [[ "$TF_FRAMES_CACHE_READY" == "1" ]]; then
    return 0
  fi
  TF_FRAMES_CACHE="$(collect_tf_frames || true)"
  TF_FRAMES_CACHE_READY="1"
}

frame_exists_in_tf() {
  local frame="$1"
  load_tf_frames_cache
  grep -Fxq "$frame" <<< "$TF_FRAMES_CACHE"
}

ODOM_HAS_DATA="0"
ODOM_USABLE="0"
ODOM_HEADER_FRAME=""
ODOM_CHILD_FRAME=""
ODOM_ORIENTATION_W=""

probe_odom_topic() {
  ODOM_HAS_DATA="0"
  ODOM_USABLE="0"
  ODOM_HEADER_FRAME=""
  ODOM_CHILD_FRAME=""
  ODOM_ORIENTATION_W=""

  if ! has_topic_data "$ODOM_TOPIC"; then
    return 1
  fi

  ODOM_HAS_DATA="1"
  ODOM_HEADER_FRAME="$(get_topic_field "$ODOM_TOPIC" "header.frame_id")"
  ODOM_CHILD_FRAME="$(get_topic_field "$ODOM_TOPIC" "child_frame_id")"
  ODOM_ORIENTATION_W="$(get_topic_value "$ODOM_TOPIC" "pose.pose.orientation.w")"

  if [[ -z "$ODOM_HEADER_FRAME" ]]; then
    echo "[WARN] Odom header.frame_id is empty or invalid: $ODOM_TOPIC"
    return 1
  fi
  if [[ "$ODOM_HEADER_FRAME" == "$ODOM_TOPIC" ]]; then
    echo "[WARN] Odom header.frame_id is using the topic name instead of a TF frame: $ODOM_HEADER_FRAME"
    return 1
  fi
  if [[ -z "$ODOM_CHILD_FRAME" ]]; then
    echo "[WARN] Odom child_frame_id is empty or invalid: $ODOM_TOPIC"
    return 1
  fi
  if is_valid_number "$ODOM_ORIENTATION_W" && number_abs_lt "$ODOM_ORIENTATION_W" "1e-6"; then
    echo "[WARN] Odom orientation.w is 0, treat external odom as invalid: $ODOM_TOPIC"
    return 1
  fi

  ODOM_USABLE="1"
  echo "[INFO] External odom looks usable: header.frame_id=$ODOM_HEADER_FRAME child_frame_id=$ODOM_CHILD_FRAME"
  return 0
}

resolve_frame_id() {
  local lidar_frame
  lidar_frame="$(get_topic_field "$LIDAR_TOPIC" "header.frame_id")"

  if [[ "$FRAME_ID" == "auto" || -z "$FRAME_ID" ]]; then
    if [[ "$ODOM_USABLE" == "1" ]]; then
      FRAME_ID="$ODOM_CHILD_FRAME"
      echo "[INFO] Auto frame_id from odom child_frame_id: $FRAME_ID"
    elif frame_exists_in_tf "base_link"; then
      FRAME_ID="base_link"
      echo "[INFO] Auto frame_id from TF tree: $FRAME_ID"
    elif frame_exists_in_tf "base_footprint"; then
      FRAME_ID="base_footprint"
      echo "[INFO] Auto frame_id from TF tree: $FRAME_ID"
    elif [[ -n "$lidar_frame" ]]; then
      FRAME_ID="$lidar_frame"
      echo "[WARN] No robot base frame found in TF, fallback frame_id to lidar frame: $FRAME_ID"
    else
      echo "[ERROR] Cannot resolve FRAME_ID automatically." >&2
      echo "[HINT] Set FRAME_ID explicitly, e.g. FRAME_ID=base_link or FRAME_ID=livox_frame" >&2
      exit 24
    fi
  fi

  if [[ -n "$lidar_frame" ]]; then
    echo "[INFO] Lidar frame      : $lidar_frame"
    if [[ "$FRAME_ID" != "$lidar_frame" ]] && ! frame_exists_in_tf "$FRAME_ID"; then
      echo "[ERROR] FRAME_ID '$FRAME_ID' not found in TF tree, but lidar messages are in '$lidar_frame'." >&2
      echo "[HINT] Publish a static TF between '$FRAME_ID' and '$lidar_frame', or run with FRAME_ID=$lidar_frame" >&2
      echo "[HINT] Example check: ros2 topic echo $LIDAR_TOPIC --field header.frame_id --once" >&2
      exit 25
    fi
  else
    echo "[WARN] Cannot read lidar frame_id from topic: $LIDAR_TOPIC"
  fi

  if [[ "$ODOM_HAS_DATA" != "1" ]]; then
    echo "[INFO] Odom topic has no data yet, skip child_frame_id auto-detect: $ODOM_TOPIC"
  elif [[ "$ODOM_USABLE" != "1" ]]; then
    echo "[WARN] Odom topic has data, but it is not usable as external odometry: $ODOM_TOPIC"
  fi
}

echo "[INFO] Root            : $ROOT_DIR"
echo "[INFO] Namespace       : $NAMESPACE"
echo "[INFO] Localization    : $LOCALIZATION"
echo "[INFO] Frame ID        : $FRAME_ID"
echo "[INFO] Lidar topic     : $LIDAR_TOPIC"
echo "[INFO] Odom topic      : $ODOM_TOPIC"
echo "[INFO] Odom frame      : $ODOM_FRAME_ID"
echo "[INFO] ICP mode        : $USE_ICP_MODE"
echo "[INFO] Sync max interval: $APPROX_SYNC_MAX_INTERVAL"
echo "[INFO] Topic queue     : $TOPIC_QUEUE_SIZE"
echo "[INFO] Sync queue      : $SYNC_QUEUE_SIZE"
echo "[INFO] Odom ICP voxel  : $ODOM_ICP_VOXEL_SIZE"
echo "[INFO] Odom ICP corr r : $ODOM_ICP_CORRESPONDENCE_RATIO"
echo "[INFO] Odom ICP max d  : $ODOM_ICP_MAX_CORRESPONDENCE_DISTANCE"
echo "[INFO] Map ICP voxel   : $MAP_ICP_VOXEL_SIZE"
echo "[INFO] Map scan voxel  : $MAP_SCAN_VOXEL_SIZE"
echo "[INFO] Map scan normal : $MAP_SCAN_NORMAL_K"
echo "[INFO] Odom reset cd   : $ODOM_RESET_COUNTDOWN"
echo "[INFO] Odom subtract r : $ODOM_SCAN_SUBTRACT_RADIUS"
echo "[INFO] Odom recent-only : $ODOM_ALWAYS_PROCESS_MOST_RECENT_FRAME"
echo "[INFO] RVIZ           : $RVIZ"
echo "[INFO] RTABMAP_VIZ    : $RTABMAP_VIZ"

if ! has_topic_data "$LIDAR_TOPIC"; then
  echo "[ERROR] No data on lidar topic: $LIDAR_TOPIC" >&2
  echo "[HINT] Start your lidar/LIO pipeline first, then rerun this script." >&2
  exit 21
fi

if ! is_valid_frame_name "$ODOM_FRAME_ID"; then
  echo "[ERROR] Invalid ODOM_FRAME_ID: $ODOM_FRAME_ID" >&2
  echo "[HINT] Use a TF frame name like odom, not a topic name like /odometry/local" >&2
  exit 26
fi

probe_odom_topic || true
resolve_frame_id
echo "[INFO] Resolved frame  : $FRAME_ID"

ODOM_ARGS=(
  "--Icp/VoxelSize" "$ODOM_ICP_VOXEL_SIZE"
  "--Icp/DownsamplingStep" "$ODOM_ICP_DOWNSAMPLING_STEP"
  "--Icp/MaxTranslation" "$ODOM_ICP_MAX_TRANSLATION"
  "--Icp/MaxRotation" "$ODOM_ICP_MAX_ROTATION"
  "--Icp/MaxCorrespondenceDistance" "$ODOM_ICP_MAX_CORRESPONDENCE_DISTANCE"
  "--Icp/CorrespondenceRatio" "$ODOM_ICP_CORRESPONDENCE_RATIO"
  "--Icp/PointToPlane" "$ODOM_ICP_POINT_TO_PLANE"
  "--Icp/PointToPlaneK" "$ODOM_ICP_SCAN_NORMAL_K"
  "--Icp/PointToPlaneMinComplexity" "$ODOM_ICP_POINT_TO_PLANE_MIN_COMPLEXITY"
  "--Odom/ResetCountdown" "$ODOM_RESET_COUNTDOWN"
  "--OdomF2M/ScanSubtractRadius" "$ODOM_SCAN_SUBTRACT_RADIUS"
)

if [[ -n "$ODOM_ARGS_EXTRA" ]]; then
  # Allow extra odometry args while keeping the Livox-tuned defaults first.
  ODOM_ARGS+=( $ODOM_ARGS_EXTRA )
fi

RTABMAP_ARGS=(
  "--Reg/Strategy" "$MAP_REG_STRATEGY"
  "--RGBD/ProximityBySpace" "$MAP_PROXIMITY_BY_SPACE"
  "--Mem/NotLinkedNodesKept" "$MAP_NOT_LINKED_NODES_KEPT"
  "--Icp/VoxelSize" "$MAP_ICP_VOXEL_SIZE"
  "--Icp/DownsamplingStep" "$MAP_ICP_DOWNSAMPLING_STEP"
  "--Icp/MaxTranslation" "$MAP_ICP_MAX_TRANSLATION"
  "--Icp/MaxRotation" "$MAP_ICP_MAX_ROTATION"
  "--Icp/MaxCorrespondenceDistance" "$MAP_ICP_MAX_CORRESPONDENCE_DISTANCE"
  "--Icp/CorrespondenceRatio" "$MAP_ICP_CORRESPONDENCE_RATIO"
  "--Icp/PointToPlane" "$MAP_ICP_POINT_TO_PLANE"
  "--Icp/PointToPlaneK" "$MAP_ICP_SCAN_NORMAL_K"
  "--Icp/PointToPlaneMinComplexity" "$MAP_ICP_POINT_TO_PLANE_MIN_COMPLEXITY"
)

if [[ -n "$RTABMAP_ARGS_EXTRA" ]]; then
  RTABMAP_ARGS+=( $RTABMAP_ARGS_EXTRA )
fi

USE_ICP="false"
case "$USE_ICP_MODE" in
  on)
    USE_ICP="true"
    ;;
  off)
    USE_ICP="false"
    if [[ "$ODOM_USABLE" != "1" ]]; then
      echo "[ERROR] USE_ICP_MODE=off but external odom is missing or invalid: $ODOM_TOPIC" >&2
      echo "[HINT] Start a valid external odometry first, or set USE_ICP_MODE=auto/on." >&2
      exit 22
    fi
    ;;
  auto)
    if [[ "$ODOM_USABLE" == "1" ]]; then
      USE_ICP="false"
      echo "[INFO] External odometry detected, keep icp_odometry:=false"
    else
      USE_ICP="true"
      echo "[WARN] External odometry is unavailable or invalid, auto switch to icp_odometry:=true"
    fi
    ;;
  *)
    echo "[ERROR] Invalid USE_ICP_MODE=$USE_ICP_MODE (expected: auto|on|off)" >&2
    exit 23
    ;;
esac

launch_args=(
  "localization:=$LOCALIZATION"
  "visual_odometry:=false"
  "icp_odometry:=$USE_ICP"
  "subscribe_scan:=false"
  "subscribe_scan_cloud:=true"
  "scan_cloud_topic:=$LIDAR_TOPIC"
  "scan_voxel_size:=$MAP_SCAN_VOXEL_SIZE"
  "scan_normal_k:=$MAP_SCAN_NORMAL_K"
  "odom_topic:=$ODOM_TOPIC"
  "vo_frame_id:=$ODOM_FRAME_ID"
  "frame_id:=$FRAME_ID"
  "rgbd_sync:=false"
  "subscribe_rgbd:=false"
  "subscribe_rgb:=false"
  "depth:=false"
  "stereo:=false"
  "approx_sync:=true"
  "approx_sync_max_interval:=$APPROX_SYNC_MAX_INTERVAL"
  "topic_queue_size:=$TOPIC_QUEUE_SIZE"
  "sync_queue_size:=$SYNC_QUEUE_SIZE"
  "odom_always_process_most_recent_frame:=$ODOM_ALWAYS_PROCESS_MOST_RECENT_FRAME"
  "qos:=$QOS"
  "rviz:=$RVIZ"
  "rtabmap_viz:=$RTABMAP_VIZ"
  "rtabmap_args:=${RTABMAP_ARGS[*]}"
  "odom_args:=${ODOM_ARGS[*]}"
)

if [[ -n "$NAMESPACE" ]]; then
  launch_args+=("namespace:=$NAMESPACE")
fi

echo "[INFO] rtabmap_args    : ${RTABMAP_ARGS[*]}"
echo "[INFO] odom_args       : ${ODOM_ARGS[*]}"
echo "[INFO] Launching rtabmap..."
ros2 launch rtabmap_launch rtabmap.launch.py "${launch_args[@]}"
