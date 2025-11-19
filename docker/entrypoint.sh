#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /usr/share/gazebo-11/setup.sh

if [ -f /workspaces/app_ws/install/setup.bash ]; then
  source /workspaces/app_ws/install/setup.bash
fi

MODEL_DIRS=$(bash -lc 'shopt -s nullglob; arr=(); while IFS= read -r -d "" f; do arr+=("$(dirname "$f")"); done < <(find /workspaces/app_ws/src -type f -name model.config -print0); IFS=":"; echo "${arr[*]}"')
export GAZEBO_MODEL_PATH="${MODEL_DIRS}${MODEL_DIRS:+:}${GAZEBO_MODEL_PATH}"
export GAZEBO_RESOURCE_PATH="/workspaces/app_ws/src:${GAZEBO_RESOURCE_PATH}"
export GAZEBO_PLUGIN_PATH="/workspaces/app_ws/install/lib:${GAZEBO_PLUGIN_PATH}"
export GAZEBO_PLUGIN_PATH="/opt/ros/humble/lib:${GAZEBO_PLUGIN_PATH}"

export QT_X11_NO_MITSHM=1
: "${XDG_RUNTIME_DIR:=/tmp/runtime-root}"
mkdir -p "$XDG_RUNTIME_DIR"

if [ -n "${ROS_LAUNCH}" ]; then
  echo "[ENTRYPOINT] Launching: ${ROS_LAUNCH}"
  exec bash -c "source /opt/ros/humble/setup.bash && source /usr/share/gazebo-11/setup.sh && ${ROS_LAUNCH}"
fi

exec bash