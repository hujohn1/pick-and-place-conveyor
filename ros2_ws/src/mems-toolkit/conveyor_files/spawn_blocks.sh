#!/usr/bin/env bash
set -euo pipefail

# ---------- Tunables (override via env) ----------
WORLD="${WORLD:-empty}"
TABLE_TOP_Z="${TABLE_TOP_Z:-0.30}"
TABLE_SIZE_X="${TABLE_SIZE_X:-1.0}"
TABLE_SIZE_Y="${TABLE_SIZE_Y:-1.0}"
TABLE_THICK="${TABLE_THICK:-0.05}"

X="${X:-0.45}"
SPACING="${SPACING:-0.08}"
BLOCK_SIZE="${BLOCK_SIZE:-0.04}"
BLOCK_DROP="${BLOCK_DROP:-0.01}"

# Mass OR density (set one). Defaults to 20 g if neither is set.
BLOCK_MASS="${BLOCK_MASS:-0.02}"       # kg
BLOCK_DENSITY="${BLOCK_DENSITY:-}"     # kg/m^3; if set, overrides BLOCK_MASS

# >>> More realistic friction and contact parameters <<<
BLOCK_MU="${BLOCK_MU:-2.0}"             # High, but not extreme
BLOCK_MU2="${BLOCK_MU2:-2.0}"            # High, but not extreme
BLOCK_TORSION="${BLOCK_TORSION:-0.5}"    # Torsional friction
BLOCK_SLIP1="${BLOCK_SLIP1:-0.0}"        # Slip
BLOCK_SLIP2="${BLOCK_SLIP2:-0.0}"        # Slip
BLOCK_KP="${BLOCK_KP:-1e5}"              # Reduced contact stiffness (100,000)
BLOCK_KD="${BLOCK_KD:-10.0}"             # Increased damping for stability
BLOCK_MIN_DEPTH="${BLOCK_MIN_DEPTH:-1e-5}"
BLOCK_MAX_VEL="${BLOCK_MAX_VEL:-0.10}"

ALLOW_RENAME="${ALLOW_RENAME:-false}"
SWEEP_SUFFIXES="${SWEEP_SUFFIXES:-true}"
MAX_SUFFIX="${MAX_SUFFIX:-9}"
WORLD_SDF="${WORLD_SDF:-/tmp/lab06_empty_usercmd.world.sdf}"
# -------------------------------------------------

need() { command -v "$1" >/dev/null 2>&1 || { echo "ERROR: $1 not found"; exit 1; }; }
need gz
need python3

# ---------- ensure a world that exposes /world/<WORLD>/set_pose ----------
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
  echo "ERROR: /world/${WORLD}/set_pose not available. Check that UserCommands is loaded." >&2
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

export BLOCK_SIZE BLOCK_MASS BLOCK_DENSITY
BLOCK_MASS_COMPUTED=$(python3 - <<'PY'
import os
a=float(os.environ.get("BLOCK_SIZE","0.04"))
dens=os.environ.get("BLOCK_DENSITY","").strip()
if dens:
    m=float(dens)*(a**3)
else:
    m=float(os.environ.get("BLOCK_MASS","0.02"))
print(f"{m:.8f}")
PY
)
export BLOCK_MASS_COMPUTED
BLOCK_INERTIA=$(python3 - <<'PY'
import os
a=float(os.environ.get("BLOCK_SIZE","0.04"))
m=float(os.environ.get("BLOCK_MASS_COMPUTED","0.02"))
I=(m*(a**2))/6.0
print(f"{I:.8e}")
PY
)
export BLOCK_INERTIA

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

write_cube_sdf() {
  local path="$1" r="$2" g="$3" b="$4"
  cat > "$path" <<EOF
<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="cube_40mm">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>${BLOCK_MASS_COMPUTED}</mass>
        <inertia>
          <ixx>${BLOCK_INERTIA}</ixx>
          <iyy>${BLOCK_INERTIA}</iyy>
          <izz>${BLOCK_INERTIA}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="c">
        <geometry><box><size>${BLOCK_SIZE} ${BLOCK_SIZE} ${BLOCK_SIZE}</size></box></geometry>
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
        <geometry><box><size>${BLOCK_SIZE} ${BLOCK_SIZE} ${BLOCK_SIZE}</size></box></geometry>
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

CUBE_RED_SDF="${SDF_DIR}/cube_red.sdf";     write_cube_sdf "$CUBE_RED_SDF"     0.85 0.10 0.10
CUBE_BLUE_SDF="${SDF_DIR}/cube_blue.sdf";   write_cube_sdf "$CUBE_BLUE_SDF"    0.10 0.10 0.85
CUBE_YELL_SDF="${SDF_DIR}/cube_yellow.sdf"; write_cube_sdf "$CUBE_YELL_SDF"    0.90 0.85 0.10

# ---------- run ----------
ensure_world

echo "[spawn] Cleaning up previous lab06 entities…"
remove_model lab06_table
remove_model lab06_block_red


TABLE_CENTER_X="${TABLE_CENTER_X:-0.0}"
TABLE_CENTER_Y="${TABLE_CENTER_Y:-0.0}"
TABLE_CENTER_Z=$(python3 - <<PY
top=${TABLE_TOP_Z}; th=${TABLE_THICK}
print(top - th/2.0)
PY
)
call_create "$TABLE_SDF" "lab06_table" "$TABLE_CENTER_X" "$TABLE_CENTER_Y" "$TABLE_CENTER_Z"
echo "[spawn] Spawned table; top @ z=${TABLE_TOP_Z}"

BLOCK_Z=$(python3 - <<PY
top=${TABLE_TOP_Z}; sz=${BLOCK_SIZE}; drop=${BLOCK_DROP}
print(top + sz/2.0 + drop)
PY
)
call_create "$CUBE_RED_SDF"   "lab06_block_red"    "$X"  "0.0"        "$BLOCK_Z"

echo "[spawn] Cubes: size=${BLOCK_SIZE} m, mass=${BLOCK_MASS_COMPUTED} kg"
echo "[spawn] Friction(mu,mu2,torsion)=(${BLOCK_MU},${BLOCK_MU2},${BLOCK_TORSION}), slip=(${BLOCK_SLIP1},${BLOCK_SLIP2})"
echo "[spawn] Contact kp=${BLOCK_KP}, kd=${BLOCK_KD}, min_depth=${BLOCK_MIN_DEPTH}, max_vel=${BLOCK_MAX_VEL}"
echo "[spawn] Spawned cubes at x=${X}, y=±${SPACING},0, z=${BLOCK_Z} in world '${WORLD}'."
echo "[spawn] Done."








