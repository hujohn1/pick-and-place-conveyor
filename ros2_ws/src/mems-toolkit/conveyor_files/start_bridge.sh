#!/usr/bin/env bash
# Starts the ros_gz_bridge for conveyor belt control.
# Run this in a separate terminal after launching the robot simulation.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ros2 run ros_gz_bridge parameter_bridge \
  --ros-args -p config_file:="${SCRIPT_DIR}/bridge.yaml"
