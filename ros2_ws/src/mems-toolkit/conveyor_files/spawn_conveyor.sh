#!/usr/bin/env bash
set -euo pipefail

# Spawns the conveyor belt model into the running gz-sim world.
#
# The belt is positioned so its right end (x=0.45) aligns with the robot's
# pickup point and its left end (x=1.55) is the block input end.
# Belt top surface is at TABLE_TOP_Z (world frame).
#
# Usage:
#   bash spawn_conveyor.sh
#
# Override defaults via env:
#   WORLD=empty TABLE_TOP_Z=0.30 bash spawn_conveyor.sh

WORLD="${WORLD:-empty}"
TABLE_TOP_Z="${TABLE_TOP_Z:-0.30}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BELT_SDF="${SCRIPT_DIR}/conveyor_belt.sdf"

need() { command -v "$1" >/dev/null 2>&1 || { echo "ERROR: $1 not found"; exit 1; }; }
need gz

# Auto-detect world name if default doesn't match
if ! gz service -l 2>/dev/null | grep -q "/world/${WORLD}/create"; then
  DETECTED=$(gz service -l 2>/dev/null | sed -n 's#.*/world/\([^/]*\)/create.*#\1#p' | head -n1 || true)
  [[ -n "$DETECTED" ]] && WORLD="$DETECTED"
fi
echo "[conveyor] Using world: ${WORLD}"

# Belt geometry (must match conveyor_belt.sdf)
# Belt is oriented along Y axis — surface moves in Y, not X.
# This prevents the belt surface from sliding into the robot body.
BELT_LENGTH=2.2   # Y extent of belt
BELT_THICK=0.02   # frame thickness
SURF_THICK=0.005  # surface thickness above frame

# Belt at x=0.45 (robot pickup X), centered in Y at y=1.1 (spans y=0.0 to y=2.2)
BELT_CENTER_X=0.45
BELT_CENTER_Y=1.1

# Belt center Z: frame top at TABLE_TOP_Z, frame center below by BELT_THICK/2
BELT_CENTER_Z=$(python3 -c "print(${TABLE_TOP_Z} - ${BELT_THICK}/2.0)")

echo "[conveyor] Belt center: x=${BELT_CENTER_X}, y=${BELT_CENTER_Y}, z=${BELT_CENTER_Z}"
echo "[conveyor] Pickup end:  y=0.0  (robot picks here at x=0.45, y=0.0)"
echo "[conveyor] Input end:   y=2.2  (blocks spawn here)"

# Choose CLI flag style
USE_LONG_FLAGS=false
if gz service --help 2>/dev/null | grep -q -- '--reqtype'; then
  USE_LONG_FLAGS=true
fi

# Remove any existing conveyor belt
remove_model() {
  local name="$1"
  local payload="name: \"${name}\"\ntype: MODEL"
  if $USE_LONG_FLAGS; then
    gz service -s "/world/${WORLD}/remove" \
      --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 3000 \
      --req "$(printf "%b" "$payload")" >/dev/null 2>&1 || true
  else
    gz service -s "/world/${WORLD}/remove" \
      -m gz.msgs.Entity -r gz.msgs.Boolean \
      -p "$(printf "%b" "$payload")" >/dev/null 2>&1 || true
  fi
}
remove_model "conveyor_belt"
sleep 0.3

# Spawn the conveyor belt
PAYLOAD=$(cat <<PBUF
sdf_filename: "${BELT_SDF}"
name: "conveyor_belt"
pose { position { x: ${BELT_CENTER_X} y: ${BELT_CENTER_Y} z: ${BELT_CENTER_Z} } }
allow_renaming: false
PBUF
)

if $USE_LONG_FLAGS; then
  gz service -s "/world/${WORLD}/create" \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout 5000 \
    --req "$PAYLOAD"
else
  gz service -s "/world/${WORLD}/create" \
    -m gz.msgs.EntityFactory \
    -r gz.msgs.Boolean \
    -p "$PAYLOAD"
fi

echo "[conveyor] Conveyor belt spawned. Start belt via ROS2:"
echo "  ros2 topic pub --once /conveyor_belt/cmd_vel std_msgs/msg/Float64 '{data: -0.1}'"
echo "  (negative = toward robot, 0.0 = stop)"
