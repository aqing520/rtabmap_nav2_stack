#!/usr/bin/env bash
# Export a Nav2 PGM/YAML map directly from an RTAB-Map database.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

DATABASE_PATH="${1:-${WORKSPACE_ROOT}/rtabmap_orbbec.db}"
OUTPUT_PREFIX="${2:-${WORKSPACE_ROOT}/pgm_map/map}"
MAP_MODE="${MAP_MODE:-trinary}"
MAP_OCCUPIED_THRESH="${MAP_OCCUPIED_THRESH:-0.65}"
MAP_FREE_THRESH="${MAP_FREE_THRESH:-0.196}"
ALLOW_STALE_DATABASE_EXPORT="${ALLOW_STALE_DATABASE_EXPORT:-false}"

usage() {
  cat <<EOF
Usage:
  bash scripts/export_rtabmap_map_offline.sh [database.db] [output_prefix]

Defaults:
  database.db   ${WORKSPACE_ROOT}/rtabmap_orbbec.db
  output_prefix ${WORKSPACE_ROOT}/pgm_map/map

The output prefix must not include .pgm or .yaml.

Optional environment variables:
  MAP_MODE=trinary
  MAP_OCCUPIED_THRESH=0.65
  MAP_FREE_THRESH=0.196
  ALLOW_STALE_DATABASE_EXPORT=false

MAP_FREE_THRESH=0.196 preserves RTAB-Map's gray unknown cells as unknown.
Set ALLOW_STALE_DATABASE_EXPORT=true only when intentionally exporting a
database other than the one used by the latest mapping session.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if (( $# > 2 )); then
  usage >&2
  exit 2
fi

if [[ "${OUTPUT_PREFIX}" == *.pgm || "${OUTPUT_PREFIX}" == *.yaml ]]; then
  echo "[db-map-export] ERROR: output must be a prefix without .pgm/.yaml:" >&2
  echo "                ${OUTPUT_PREFIX}" >&2
  exit 2
fi

if [[ ! -s "${DATABASE_PATH}" ]]; then
  echo "[db-map-export] ERROR: database does not exist or is empty:" >&2
  echo "                ${DATABASE_PATH}" >&2
  exit 3
fi

DATABASE_PATH="$(realpath -- "${DATABASE_PATH}")"

# Mapping launch files record the database selected for the latest mapping
# session. Refuse to export a different, older database by accident.
MAPPING_MARKER="${HOME}/.ros/last_rtabmap_mapping_session"
if [[ -f "${MAPPING_MARKER}" ]]; then
  LAST_MAPPING_DATABASE="$(
    sed -n 's/^database_path=//p' "${MAPPING_MARKER}" | head -n 1
  )"
  LAST_MAPPING_STARTED="$(
    sed -n 's/^session_started_epoch=//p' "${MAPPING_MARKER}" | head -n 1
  )"
  LAST_MAPPING_LAUNCH="$(
    sed -n 's/^launch_file=//p' "${MAPPING_MARKER}" | head -n 1
  )"

  if [[ -n "${LAST_MAPPING_DATABASE}" ]]; then
    LAST_MAPPING_DATABASE="$(realpath -m -- "${LAST_MAPPING_DATABASE}")"
  fi

  if [[
    "${ALLOW_STALE_DATABASE_EXPORT}" != "true"
    && -n "${LAST_MAPPING_DATABASE}"
    && "${DATABASE_PATH}" != "${LAST_MAPPING_DATABASE}"
  ]]; then
    echo "[db-map-export] ERROR: requested database was not used by the latest mapping session." >&2
    echo "                requested: ${DATABASE_PATH}" >&2
    echo "                latest:    ${LAST_MAPPING_DATABASE}" >&2
    echo "                launch:    ${LAST_MAPPING_LAUNCH:-unknown}" >&2
    echo "                This would export an older/different map." >&2
    echo "                Re-run mapping with the requested database path." >&2
    exit 8
  fi

  if [[
    "${ALLOW_STALE_DATABASE_EXPORT}" != "true"
    && "${DATABASE_PATH}" == "${LAST_MAPPING_DATABASE}"
    && "${LAST_MAPPING_STARTED}" =~ ^[0-9]+$
  ]]; then
    DATABASE_MTIME="$(stat -c '%Y' -- "${DATABASE_PATH}")"
    if (( DATABASE_MTIME < LAST_MAPPING_STARTED )); then
      echo "[db-map-export] ERROR: database was not updated by the latest mapping session." >&2
      echo "                database: ${DATABASE_PATH}" >&2
      echo "                Stop mapping with Ctrl+C and wait for RTAB-Map to close cleanly." >&2
      exit 9
    fi
  fi
fi

if command -v lsof >/dev/null 2>&1 &&
   lsof -- "${DATABASE_PATH}" >/dev/null 2>&1; then
  echo "[db-map-export] ERROR: database is still open by a running process:" >&2
  echo "                ${DATABASE_PATH}" >&2
  echo "                Stop mapping/navigation and wait for RTAB-Map to exit before exporting." >&2
  lsof -- "${DATABASE_PATH}" >&2 || true
  exit 10
fi

# Load the ROS and workspace environments so the matching RTAB-Map binary and
# libraries are used even when this script is called from a clean terminal.
set +u
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [[ ! -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  echo "[db-map-export] ERROR: workspace has not been built:" >&2
  echo "                ${WORKSPACE_ROOT}/install/setup.bash not found" >&2
  exit 4
fi
# shellcheck disable=SC1091
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u

if ! command -v rtabmap-export >/dev/null 2>&1; then
  echo "[db-map-export] ERROR: rtabmap-export was not found." >&2
  exit 5
fi

OUTPUT_DIR="$(dirname -- "${OUTPUT_PREFIX}")"
OUTPUT_NAME="$(basename -- "${OUTPUT_PREFIX}")"
mkdir -p "${OUTPUT_DIR}"
OUTPUT_DIR="$(cd -- "${OUTPUT_DIR}" && pwd)"

# Export to a temporary directory first. Existing usable map files are only
# replaced after both new files have been generated and validated.
TMP_DIR="$(mktemp -d "${OUTPUT_DIR}/.rtabmap_map_export.XXXXXX")"
cleanup() {
  rm -rf -- "${TMP_DIR}"
}
trap cleanup EXIT

echo "[db-map-export] database: ${DATABASE_PATH}"
echo "[db-map-export] database modified: $(date -d "@$(stat -c '%Y' -- "${DATABASE_PATH}")" '+%F %T %z')"
echo "[db-map-export] output:   ${OUTPUT_DIR}/${OUTPUT_NAME}.{pgm,yaml}"

rtabmap-export \
  --map \
  --opt 2 \
  --output "${OUTPUT_NAME}" \
  --output_dir "${TMP_DIR}" \
  "${DATABASE_PATH}"

TMP_PGM="${TMP_DIR}/${OUTPUT_NAME}.pgm"
TMP_YAML="${TMP_DIR}/${OUTPUT_NAME}.yaml"

if [[ ! -s "${TMP_PGM}" || ! -s "${TMP_YAML}" ]]; then
  echo "[db-map-export] ERROR: rtabmap-export did not generate a valid PGM/YAML pair." >&2
  exit 6
fi

if ! grep -q '^image:' "${TMP_YAML}" ||
   ! grep -q '^resolution:' "${TMP_YAML}" ||
   ! grep -q '^origin:' "${TMP_YAML}"; then
  echo "[db-map-export] ERROR: generated YAML is missing required map fields." >&2
  exit 7
fi

# RTAB-Map encodes unknown PGM cells as gray value 205, corresponding to an
# occupancy value of about 0.196078. Keeping free_thresh at 0.196 prevents
# those unknown cells from being interpreted as free. The thresholds remain
# configurable through the environment variables above.
NORMALIZED_YAML="${TMP_YAML}.normalized"
awk \
  -v mode="${MAP_MODE}" \
  -v occupied="${MAP_OCCUPIED_THRESH}" \
  -v free="${MAP_FREE_THRESH}" \
  '
    /^mode:/ {next}
    /^occupied_thresh:/ {next}
    /^free_thresh:/ {next}
    /^image:/ {
      print
      print "mode: " mode
      next
    }
    {print}
    END {
      print "occupied_thresh: " occupied
      print "free_thresh: " free
    }
  ' "${TMP_YAML}" > "${NORMALIZED_YAML}"
mv -f -- "${NORMALIZED_YAML}" "${TMP_YAML}"

mv -f -- "${TMP_PGM}" "${OUTPUT_DIR}/${OUTPUT_NAME}.pgm"
mv -f -- "${TMP_YAML}" "${OUTPUT_DIR}/${OUTPUT_NAME}.yaml"

cat > "${OUTPUT_DIR}/.${OUTPUT_NAME}_export_source" <<EOF
database_path=${DATABASE_PATH}
database_mtime=$(stat -c '%Y' -- "${DATABASE_PATH}")
exported_epoch=$(date +%s)
EOF

echo "[db-map-export] SUCCESS"
echo "[db-map-export] mode=${MAP_MODE}, occupied=${MAP_OCCUPIED_THRESH}, free=${MAP_FREE_THRESH}"
ls -lh \
  "${OUTPUT_DIR}/${OUTPUT_NAME}.pgm" \
  "${OUTPUT_DIR}/${OUTPUT_NAME}.yaml"
