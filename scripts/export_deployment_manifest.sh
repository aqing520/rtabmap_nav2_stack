#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WHEELTEC_WS="${WHEELTEC_WS:-$HOME/wheeltec_ros2}"

section() {
  printf '\n[%s]\n' "$1"
}

package_info() {
  local package="$1"
  local prefix version

  prefix="$(ros2 pkg prefix "$package" 2>/dev/null || true)"
  version="$(ros2 pkg xml "$package" 2>/dev/null | sed -n 's:.*<version>\([^<]*\)</version>.*:\1:p' | head -n 1)"
  printf '%-32s version=%-16s prefix=%s\n' "$package" "${version:-missing}" "${prefix:-missing}"
}

section system
if [[ -r /etc/os-release ]]; then
  . /etc/os-release
  printf 'os=%s\n' "${PRETTY_NAME:-unknown}"
fi
printf 'kernel=%s\n' "$(uname -r)"
printf 'architecture=%s\n' "$(dpkg --print-architecture 2>/dev/null || uname -m)"
dpkg-query -W -f='jetpack=${Version}\n' nvidia-jetpack 2>/dev/null || true
dpkg-query -W -f='l4t=${Version}\n' nvidia-l4t-core 2>/dev/null || true

section toolchain
cmake --version 2>/dev/null | head -n 1 || true
gcc --version 2>/dev/null | head -n 1 || true
g++ --version 2>/dev/null | head -n 1 || true
python3 --version 2>/dev/null || true
nvcc --version 2>/dev/null | tail -n 1 || true
pkg-config --modversion opencv4 2>/dev/null | sed 's/^/opencv=/' || true
pkg-config --modversion pcl_common 2>/dev/null | sed 's/^/pcl=/' || true

set +u
source /opt/ros/humble/setup.bash
if [[ -f "$WHEELTEC_WS/install/setup.bash" ]]; then
  source "$WHEELTEC_WS/install/setup.bash"
fi
if [[ -f "$WS_DIR/scripts/use_rtabmap_0234_env.sh" ]]; then
  source "$WS_DIR/scripts/use_rtabmap_0234_env.sh" >/dev/null
fi
if [[ -f "$WS_DIR/install/setup.bash" ]]; then
  source "$WS_DIR/install/setup.bash"
fi
set -u

section ros_packages
printf 'ROS_DISTRO=%s\n' "${ROS_DISTRO:-missing}"
for package in \
  rviz2 nav2_bringup nav2_planner nav2_controller nav2_bt_navigator \
  nav2_costmap_2d nav2_mppi_controller nav2_map_server \
  rtabmap_ros rtabmap_slam robot_bringup fast_lio livox_ros_driver2; do
  package_info "$package"
done

section rtabmap_runtime
rtabmap_core_lib="$WS_DIR/third_party/rtabmap-0.23.4/install/lib/librtabmap_core.so.0.23.4"
if [[ -f "$rtabmap_core_lib" ]]; then
  printf 'project_core=%s\n' "$rtabmap_core_lib"
else
  echo 'project_core=missing'
fi

# `rtabmap` is the Qt GUI executable and aborts on headless deployments when
# no display server is available. Use the console executable for a safe
# system-runtime inventory instead.
if command -v rtabmap-console >/dev/null 2>&1; then
  rtabmap-console --version 2>/dev/null | sed -n '1,8p'
else
  echo 'rtabmap-console=missing'
fi

section selected_debian_packages
for package in \
  ros-humble-rviz2 ros-humble-navigation2 ros-humble-robot-localization \
  ros-humble-cv-bridge ros-humble-pcl-ros \
  libopencv-dev libpcl-dev libeigen3-dev libceres-dev libboost-dev; do
  dpkg-query -W -f='${Package}\t${Version}\n' "$package" 2>/dev/null || printf '%s\tmissing\n' "$package"
done

section project_git
git -C "$WS_DIR" status --short --branch 2>/dev/null || true
git -C "$WS_DIR" rev-parse HEAD 2>/dev/null || true

section critical_config_sha256
config_files=(
  src/robot_bringup/config/nav2_navigation.rviz
  src/robot_bringup/config/nav2_common.yaml
  src/robot_bringup/config/navigate_to_pose_clear_costmaps_on_goal_start.xml
  src/FAST_LIO_ROS2/config/mid360.yaml
  src/livox_ros_driver2/config/MID360_config.json
)
for relative_path in "${config_files[@]}"; do
  if [[ -f "$WS_DIR/$relative_path" ]]; then
    (cd "$WS_DIR" && sha256sum "$relative_path")
  else
    printf 'missing  %s\n' "$relative_path"
  fi
done

section map_files
{
  find "$WS_DIR" -maxdepth 3 -type f \
    \( -name '*.db' -o -name '*.yaml' -o -name '*.pgm' \) \
    -printf '%p\t%s bytes\n' 2>/dev/null
  find "$WS_DIR/cloud_map" -maxdepth 1 -type f -name '*.pcd' \
    -printf '%p\t%s bytes\n' 2>/dev/null
} | sort || true

section external_wheeltec_workspace
printf 'path=%s\n' "$WHEELTEC_WS"
if [[ -d "$WHEELTEC_WS/src" ]]; then
  find "$WHEELTEC_WS/src" -name package.xml -print0 2>/dev/null \
    | sort -z \
    | xargs -0 sha256sum 2>/dev/null || true
else
  echo 'status=missing'
fi
