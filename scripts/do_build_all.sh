#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"
JOBS="${JOBS:-8}"
WORKERS="${WORKERS:-4}"
HEAVY_JOBS="${HEAVY_JOBS:-1}"
CLEAN_BUILD="${CLEAN_BUILD:-0}"
INSTALL_DEPS="${INSTALL_DEPS:-0}"
BUILD_TYPE="${BUILD_TYPE:-Release}"

if [[ $# -ge 1 ]]; then
  JOBS="$1"
  WORKERS="$JOBS"
fi

if [[ ! -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]]; then
  echo "[ERROR] ROS distro not found: /opt/ros/${ROS_DISTRO_NAME}/setup.bash" >&2
  exit 1
fi

detect_opencv_dir() {
  local candidates=()
  local pc_dir=""

  if [[ -n "${RTABMAP_OPENCV_DIR:-}" ]]; then
    candidates+=("${RTABMAP_OPENCV_DIR}")
  fi
  if [[ -n "${OpenCV_DIR:-}" ]]; then
    candidates+=("${OpenCV_DIR}")
  fi

  if command -v pkg-config >/dev/null 2>&1; then
    pc_dir="$(pkg-config --variable=pcfiledir opencv4 2>/dev/null || true)"
    if [[ -n "$pc_dir" ]]; then
      candidates+=("${pc_dir%/pkgconfig}/cmake/opencv4")
    fi
  fi

  candidates+=(
    "/usr/lib/aarch64-linux-gnu/cmake/opencv4"
    "/usr/lib/x86_64-linux-gnu/cmake/opencv4"
    "/usr/local/lib/cmake/opencv4"
    "/usr/local/lib64/cmake/opencv4"
    "/usr/lib/cmake/opencv4"
  )

  local d
  for d in "${candidates[@]}"; do
    if [[ -n "$d" && -f "$d/OpenCVConfig.cmake" ]]; then
      echo "$d"
      return 0
    fi
  done

  local found
  found="$(find /usr /usr/local -type f -path '*/cmake/opencv4/OpenCVConfig.cmake' 2>/dev/null | head -n 1 || true)"
  if [[ -n "$found" ]]; then
    dirname "$found"
    return 0
  fi

  return 1
}

cd "$ROOT_DIR"

echo "[INFO] Root        : $ROOT_DIR"
echo "[INFO] ROS distro  : $ROS_DISTRO_NAME"
echo "[INFO] Jobs        : $JOBS"
echo "[INFO] Workers     : $WORKERS"
echo "[INFO] Heavy jobs  : $HEAVY_JOBS"
echo "[INFO] Build type  : $BUILD_TYPE"

# ROS setup scripts may reference unset vars when nounset is enabled.
set +u
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
set -u

# Avoid stale pkg-config overrides from previous sessions.
unset PKG_CONFIG_PATH || true
unset PKG_CONFIG_LIBDIR || true

export RTABMAP_OPENCV_DIR="$(detect_opencv_dir || true)"
if [[ -z "$RTABMAP_OPENCV_DIR" ]]; then
  echo "[ERROR] OpenCVConfig.cmake not found. Set RTABMAP_OPENCV_DIR manually." >&2
  exit 11
fi
export OpenCV_DIR="$RTABMAP_OPENCV_DIR"
echo "[INFO] OpenCV_DIR   : $OpenCV_DIR"

if [[ "$INSTALL_DEPS" == "1" ]]; then
  echo "[STEP] Installing dependencies with rosdep"
  rosdep update
  rosdep install --from-paths src --ignore-src -r -y
fi

if [[ "$CLEAN_BUILD" == "1" ]]; then
  echo "[STEP] Cleaning build/install/log and third_party RTABMap cache"
  rm -rf build install log
  rm -rf third_party/rtabmap-0.23.4/build_local third_party/rtabmap-0.23.4/install
fi

echo "[STEP] Building RTABMap 0.23.4 (third_party)"
JOBS="$JOBS" RTABMAP_OPENCV_DIR="$RTABMAP_OPENCV_DIR" bash "$ROOT_DIR/scripts/build_rtabmap_0234.sh"

# Make local RTABMap discoverable for all downstream find_package(RTABMap 0.23.4)
source "$ROOT_DIR/scripts/use_rtabmap_0234_env.sh"

if [[ -z "${RTABMap_DIR:-}" || ! -f "${RTABMap_DIR}/RTABMapConfig.cmake" ]]; then
  echo "[ERROR] Invalid RTABMap_DIR: ${RTABMap_DIR:-<empty>}" >&2
  exit 2
fi
RTABMAP_CONFIG_VERSION_FILE="${RTABMap_DIR}/RTABMapConfigVersion.cmake"
if [[ ! -f "$RTABMAP_CONFIG_VERSION_FILE" ]]; then
  echo "[ERROR] Version file not found: $RTABMAP_CONFIG_VERSION_FILE" >&2
  exit 3
fi
if ! grep -Eq 'PACKAGE_VERSION[[:space:]]+"0\.23\.4"' "$RTABMAP_CONFIG_VERSION_FILE"; then
  echo "[ERROR] RTABMap version is not 0.23.4: $RTABMAP_CONFIG_VERSION_FILE" >&2
  exit 4
fi
echo "[INFO] Using RTABMap_DIR: $RTABMap_DIR"

OVERRIDES=(
  rtabmap_conversions rtabmap_costmap_plugins rtabmap_demos rtabmap_examples
  rtabmap_launch rtabmap_msgs rtabmap_odom rtabmap_python rtabmap_ros
  rtabmap_rviz_plugins rtabmap_slam rtabmap_sync rtabmap_util rtabmap_viz
)

BASE_PACKAGES=(
  livox_ros_driver2 rtabmap_msgs rtabmap_costmap_plugins rtabmap_python rtabmap_conversions
)

HEAVY_PACKAGES=(
  fast_lio rtabmap_sync rtabmap_viz rtabmap_rviz_plugins
)

REST_PACKAGES=(
  rtabmap_util rtabmap_odom rtabmap_slam rtabmap_launch rtabmap_examples rtabmap_demos rtabmap_ros
  rtsp_camera_bridge bt468_rtk_driver robot_bringup
)

CMAKE_ARGS=(
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
  -DOpenCV_DIR="$OpenCV_DIR"
)

COMMON_ARGS=(
  --symlink-install
  --cmake-clean-cache
  --allow-overriding "${OVERRIDES[@]}"
)

echo "[STEP] Build stage 1/3: base packages (parallel)"
export MAKEFLAGS="-j${JOBS} -l${JOBS}"
export CMAKE_BUILD_PARALLEL_LEVEL="$JOBS"
colcon build \
  --executor parallel \
  --parallel-workers "$WORKERS" \
  "${COMMON_ARGS[@]}" \
  --cmake-args "${CMAKE_ARGS[@]}" \
  --packages-select "${BASE_PACKAGES[@]}"

echo "[STEP] Build stage 2/3: heavy packages (low parallel)"
export MAKEFLAGS="-j${HEAVY_JOBS} -l${HEAVY_JOBS}"
export CMAKE_BUILD_PARALLEL_LEVEL="$HEAVY_JOBS"
colcon build \
  --executor sequential \
  --parallel-workers 1 \
  "${COMMON_ARGS[@]}" \
  --cmake-args "${CMAKE_ARGS[@]}" \
  --packages-select "${HEAVY_PACKAGES[@]}"

echo "[STEP] Build stage 3/3: remaining packages (parallel)"
export MAKEFLAGS="-j${JOBS} -l${JOBS}"
export CMAKE_BUILD_PARALLEL_LEVEL="$JOBS"
colcon build \
  --executor parallel \
  --parallel-workers "$WORKERS" \
  "${COMMON_ARGS[@]}" \
  --cmake-args "${CMAKE_ARGS[@]}" \
  --packages-select "${REST_PACKAGES[@]}"

echo "[DONE] Build finished."
echo "[NEXT] source \"$ROOT_DIR/install/setup.bash\""
