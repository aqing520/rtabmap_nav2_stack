#!/usr/bin/env bash

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RTABMAP_INSTALL_DIR="$ROOT_DIR/third_party/rtabmap-0.23.4/install"

prepend_unique_path() {
  local var_name="$1"
  local new_path="$2"
  local current_value="${!var_name:-}"

  case ":$current_value:" in
    *":$new_path:"*) ;;
    *)
      if [[ -n "$current_value" ]]; then
        export "$var_name=$new_path:$current_value"
      else
        export "$var_name=$new_path"
      fi
      ;;
  esac
}

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
prepend_unique_path CMAKE_PREFIX_PATH "$RTABMAP_INSTALL_DIR"
prepend_unique_path LD_LIBRARY_PATH "$RTABMAP_INSTALL_DIR/lib"
prepend_unique_path PATH "$RTABMAP_INSTALL_DIR/bin"

echo "[OK] RTABMap_DIR=$RTABMap_DIR"
echo "[OK] CMAKE_PREFIX_PATH prepended with $RTABMAP_INSTALL_DIR"
echo "[OK] LD_LIBRARY_PATH prepended with $RTABMAP_INSTALL_DIR/lib"
