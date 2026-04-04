#!/usr/bin/env bash
set -euo pipefail

# ---------- Tunables (override via env) ----------
WORLD="${WORLD:-empty}"
TABLE_TOP_Z="${TABLE_TOP_Z:-0.30}"
TABLE_SIZE_X="${TABLE_SIZE_X:-1.0}"
TABLE_SIZE_Y="${TABLE_SIZE_Y:-1.0}"
TABLE_THICK="${TABLE_THICK:-0.05}"

X="${X:-0.45}"
BLOCK_DROP="${BLOCK_DROP:-0.01}"

BLOCK_MASS="${BLOCK_MASS:-0.02}"
BLOCK_DENSITY="${BLOCK_DENSITY:-}"

BLOCK_MU="${BLOCK_MU:-2.0}"
BLOCK_MU2="${BLOCK_MU2:-2.0}"
BLOCK_TORSION="${BLOCK_TORSION:-0.5}"
BLOCK_SLIP1="${BLOCK_SLIP1:-0.0}"
BLOCK_SLIP2="${BLOCK_SLIP2:-0.0}"
BLOCK_KP="${BLOCK_KP:-1e5}"
BLOCK_KD="${BLOCK_KD:-10.0}"
BLOCK_MIN_DEPTH="${BLOCK_MIN_DEPTH:-1e-5}"
BLOCK_MAX_VEL="${BLOCK_MAX_VEL:-0.10}"

ALLOW_RENAME="${ALLOW_RENAME:-false}"
SWEEP_SUFFIXES="${SWEEP_SUFFIXES:-true}"
MAX_SUFFIX="${MAX_SUFFIX:-9}"
WORLD_SDF="${WORLD_SDF:-/tmp/lab06_empty_usercmd.world.sdf}"

# ---------- Block sizes (meters) ----------
# Five different sizes, lined up next to each other along the y-axis.
# Edit these to change how big each block is.
BLOCK_SIZE_1="0.04"   # red    — small
BLOCK_SIZE_2="0.04"   # blue   — medium-small
BLOCK_SIZE_3="0.05"   # green  — medium
BLOCK_SIZE_4="0.06"   # yellow — medium-large
BLOCK_SIZE_5="0.07"   # orange — large

# Gap between block centers in meters.
# 0.08 = blocks sit close together without overlapping.
# Increase this to add space between them.
BLOCK_SPACING="0.08"
# -------------------------------------------------

need() { command -v "$1" >/dev/null 2>&1 || { echo "ERROR: $1 not found"; exit 1; }; }
need gz
need python3

# ---------- ensure world ----------
ensure_world() {
  if ! gz service -l | grep -q "/world/"; then
    cat > "$WORLD_SDF" <<'SDF'
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="empty">
    <gravity>0 0 -9.81</gravity>
    <plugin name="scene_broadcaster" filename="gz-sim-scene-broadcaster-system"/>
    <plugin name="user_commands" filename="gz-sim-user-commands-system"/>
  </world>
</sdf>
SDF
    echo "[spawn] Launching headless world: $WORLD_SDF"
    nohup gz sim -r "$WORLD_SDF" >/tmp/lab06_world.log 2>&1 &
    sleep 0.5
  fi

  if ! gz service -l | grep -q "/world/${WORLD}/create"; then
    DETECTED=$(gz service -l | sed -n 's#.*/world/\([^/]*\)/create.*#\1#p' | head -n1 || true)
    [[ -n "$DETECTED" ]] && WORLD="$DETECTED"
  fi
  echo "[spawn] Using world: ${WORLD}"

  echo -n "[spawn] Waiting for /world/${WORLD}/set_pose ... "
  for _ in {1..60}; do
    if gz service -l | grep -q "/world/${WORLD}/set_pose"; then
      echo "OK"
      return
    fi
    sleep 0.25
  done
  echo "TIMEOUT"
  echo "ERROR: /world/${WORLD}/set_pose not available." >&2
  exit 1
}

# choose CLI flag style (long/short)
USE_LONG_FLAGS=false
if gz service --help 2>/dev/null | grep -q -- '--reqtype'; then
  USE_LONG_FLAGS=true
fi

# ---------- service helpers ----------
call_create() {
  local sdf_path="$1" name="$2" x="$3" y="$4" z="$5"
  local payload
  payload=$(cat <<PBUF
sdf_filename: "$sdf_path"
name: "$name"
pose { position { x: $x y: $y z: $z } }
allow_renaming: ${ALLOW_RENAME}
PBUF
)
  if $USE_LONG_FLAGS; then
    gz service -s "/world/${WORLD}/create" \
      --reqtype gz.msgs.EntityFactory \
      --reptype gz.msgs.Boolean \
      --timeout 3000 \
      --req "$payload" >/dev/null
  else
    gz service -s "/world/${WORLD}/create" \
      -m gz.msgs.EntityFactory \
      -r gz.msgs.Boolean \
      -p "$payload" >/dev/null
  fi
}

remove_model_once() {
  local name="$1"
  local payload_sym=$'name: "'"$name"$'"\n'"type: MODEL"
  local payload_num=$'name: "'"$name"$'"\n'"type: 2"
  if $USE_LONG_FLAGS; then
    gz service -s "/world/${WORLD}/remove" \
      --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 3000 \
      --req "$payload_sym" >/dev/null 2>&1 \
    || gz service -s "/world/${WORLD}/remove" \
      --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 3000 \
      --req "$payload_num" >/dev/null 2>&1 || true
  else
    gz service -s "/world/${WORLD}/remove" \
      -m gz.msgs.Entity -r gz.msgs.Boolean -p "$payload_sym" >/dev/null 2>&1 \
    || gz service -s "/world/${WORLD}/remove" \
      -m gz.msgs.Entity -r gz.msgs.Boolean -p "$payload_num" >/dev/null 2>&1 || true
  fi
}

remove_model() {
  local base="$1"
  remove_model_once "$base"
  if [[ "$SWEEP_SUFFIXES" == "true" ]]; then
    for i in $(seq 1 "$MAX_SUFFIX"); do
      remove_model_once "${base}_${i}"
    done
  fi
}

# ---------- write SDFs ----------
SDF_DIR=/tmp/lab06_spawn
mkdir -p "$SDF_DIR"

# ---------- write table SDF ----------
TABLE_SDF="${SDF_DIR}/table_${TABLE_SIZE_X}x${TABLE_SIZE_Y}x${TABLE_THICK}.sdf"
cat > "$TABLE_SDF" <<EOF
<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="lab06_table">
    <static>true</static>
    <link name="top">
      <collision name="c">
        <geometry><box><size>${TABLE_SIZE_X} ${TABLE_SIZE_Y} ${TABLE_THICK}</size></box></geometry>
      </collision>
      <visual name="v">
        <geometry><box><size>${TABLE_SIZE_X} ${TABLE_SIZE_Y} ${TABLE_THICK}</size></box></geometry>
      </visual>
    </link>
  </model>
</sdf>
EOF

# ---------- write a single block SDF ----------
# Usage: write_cube_sdf <output_path> <size_m> <r> <g> <b>
# Mass and inertia are computed inside python using only the size argument,
# so no variable passing issues between bash and python.
write_cube_sdf() {
  local path="$1"
  local size="$2"
  local r="$3" g="$4" b="$5"

  # Compute mass — uses BLOCK_MASS / BLOCK_DENSITY env vars + the size argument
  local mass
  mass=$(python3 -c "
import os
a = float('${size}')
dens = os.environ.get('BLOCK_DENSITY', '').strip()
if dens:
    m = float(dens) * (a ** 3)
else:
    m = float(os.environ.get('BLOCK_MASS', '0.02'))
print(f'{m:.8f}')
")

  # Compute inertia — uses the mass we just computed and the size argument
  local inertia
  inertia=$(python3 -c "
a = float('${size}')
m = float('${mass}')
I = (m * (a ** 2)) / 6.0
print(f'{I:.8e}')
")

  cat > "$path" <<EOF
<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="cube_${size}m">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>${mass}</mass>
        <inertia>
          <ixx>${inertia}</ixx>
          <iyy>${inertia}</iyy>
          <izz>${inertia}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="c">
        <geometry><box><size>${size} ${size} ${size}</size></box></geometry>
        <surface>
          <friction>
            <ode>
              <mu>${BLOCK_MU}</mu>
              <mu2>${BLOCK_MU2}</mu2>
              <slip1>${BLOCK_SLIP1}</slip1>
              <slip2>${BLOCK_SLIP2}</slip2>
            </ode>
            <torsional>
              <coefficient>${BLOCK_TORSION}</coefficient>
              <use_patch_radius>1</use_patch_radius>
              <patch_radius>0.02</patch_radius>
              <surface_radius>0.02</surface_radius>
            </torsional>
          </friction>
          <contact>
            <ode>
              <kp>${BLOCK_KP}</kp>
              <kd>${BLOCK_KD}</kd>
              <max_vel>${BLOCK_MAX_VEL}</max_vel>
              <min_depth>${BLOCK_MIN_DEPTH}</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>
      <visual name="v">
        <geometry><box><size>${size} ${size} ${size}</size></box></geometry>
        <material>
          <ambient>${r} ${g} ${b} 1</ambient>
          <diffuse>${r} ${g} ${b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
EOF
}

# ---------- generate all 5 block SDFs ----------
#                                    path                         size           R     G     B
CUBE_1_SDF="${SDF_DIR}/cube_1.sdf"; write_cube_sdf "$CUBE_1_SDF" "$BLOCK_SIZE_1" 0.85 0.10 0.10  # red
CUBE_2_SDF="${SDF_DIR}/cube_2.sdf"; write_cube_sdf "$CUBE_2_SDF" "$BLOCK_SIZE_2" 0.10 0.10 0.85  # blue
CUBE_3_SDF="${SDF_DIR}/cube_3.sdf"; write_cube_sdf "$CUBE_3_SDF" "$BLOCK_SIZE_3" 0.10 0.75 0.10  # green
CUBE_4_SDF="${SDF_DIR}/cube_4.sdf"; write_cube_sdf "$CUBE_4_SDF" "$BLOCK_SIZE_4" 0.90 0.85 0.10  # yellow
CUBE_5_SDF="${SDF_DIR}/cube_5.sdf"; write_cube_sdf "$CUBE_5_SDF" "$BLOCK_SIZE_5" 0.80 0.40 0.00  # orange

# ---------- run ----------
ensure_world

echo "[spawn] Cleaning up previous lab06 entities..."
remove_model lab06_table
remove_model lab06_block_1
remove_model lab06_block_2
remove_model lab06_block_3
remove_model lab06_block_4
remove_model lab06_block_5

# ---------- spawn table ----------
TABLE_CENTER_X="${TABLE_CENTER_X:-0.0}"
TABLE_CENTER_Y="${TABLE_CENTER_Y:-0.0}"
TABLE_CENTER_Z=$(python3 -c "print(${TABLE_TOP_Z} - ${TABLE_THICK}/2.0)")
call_create "$TABLE_SDF" "lab06_table" "$TABLE_CENTER_X" "$TABLE_CENTER_Y" "$TABLE_CENTER_Z"
echo "[spawn] Spawned table; top @ z=${TABLE_TOP_Z}"

# ---------- spawn 5 blocks side by side along the y-axis ----------
# y positions are centered around 0: -2s, -1s, 0, +1s, +2s
# where s = BLOCK_SPACING

spawn_block() {
  local sdf="$1" name="$2" size="$3" y="$4"
  local z
  z=$(python3 -c "print(${TABLE_TOP_Z} + float('${size}')/2.0 + ${BLOCK_DROP})")
  call_create "$sdf" "$name" "$X" "$y" "$z"
  echo "[spawn] Spawned $name (size=${size}m) at x=${X}, y=${y}, z=${z}"
}

Y1=$(python3 -c "print(f'{-2 * ${BLOCK_SPACING}:.4f}')")
Y2=$(python3 -c "print(f'{-1 * ${BLOCK_SPACING}:.4f}')")
Y3="0.0000"
Y4=$(python3 -c "print(f'{ 1 * ${BLOCK_SPACING}:.4f}')")
Y5=$(python3 -c "print(f'{ 2 * ${BLOCK_SPACING}:.4f}')")

spawn_block "$CUBE_1_SDF" "lab06_block_1" "$BLOCK_SIZE_1" "$Y1"   # red
spawn_block "$CUBE_2_SDF" "lab06_block_2" "$BLOCK_SIZE_2" "$Y2"   # blue
spawn_block "$CUBE_3_SDF" "lab06_block_3" "$BLOCK_SIZE_3" "$Y3"   # green
spawn_block "$CUBE_4_SDF" "lab06_block_4" "$BLOCK_SIZE_4" "$Y4"   # yellow
spawn_block "$CUBE_5_SDF" "lab06_block_5" "$BLOCK_SIZE_5" "$Y5"   # orange

echo "[spawn] Friction(mu,mu2,torsion)=(${BLOCK_MU},${BLOCK_MU2},${BLOCK_TORSION})"
echo "[spawn] Contact kp=${BLOCK_KP}, kd=${BLOCK_KD}"
echo "[spawn] Done."