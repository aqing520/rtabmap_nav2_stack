#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RTABMAP_INSTALL_DIR="$ROOT_DIR/third_party/rtabmap-0.23.4/install"

if [[ ! -d "$RTABMAP_INSTALL_DIR" ]]; then
  echo "[ERROR] Install dir not found: $RTABMAP_INSTALL_DIR" >&2
  echo "[HINT] Build first: $ROOT_DIR/scripts/build_rtabmap_0234.sh" >&2
  return 1 2>/dev/null || exit 1
fi

RTABMAP_CONFIG=$(find "$RTABMAP_INSTALL_DIR" -name RTABMapConfig.cmake | head -n 1 || true)
if [[ -z "$RTABMAP_CONFIG" ]]; then
  echo "[ERROR] RTABMapConfig.cmake not found under $RTABMAP_INSTALL_DIR" >&2
  return 1 2>/dev/null || exit 1
fi

export RTABMap_DIR="$(dirname "$RTABMAP_CONFIG")"
export CMAKE_PREFIX_PATH="$RTABMAP_INSTALL_DIR:${CMAKE_PREFIX_PATH:-}"

echo "[OK] RTABMap_DIR=$RTABMap_DIR"
echo "[OK] CMAKE_PREFIX_PATH prepended with $RTABMAP_INSTALL_DIR"
