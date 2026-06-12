#!/usr/bin/env bash
# Usage: ./send_goal.sh <x> <y> [yaw_deg]
#   x, y   : target position in map frame (metres)
#   yaw_deg: optional heading in degrees (default 0)
#
# Example: ./send_goal.sh 2.0 1.0
#          ./send_goal.sh 2.0 1.0 90

set -e

X=${1:?Usage: $0 <x> <y> [yaw_deg]}
Y=${2:?Usage: $0 <x> <y> [yaw_deg]}
YAW_DEG=${3:-0}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$SCRIPT_DIR/send_goal.py" "$X" "$Y" "$YAW_DEG"
