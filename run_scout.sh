#!/usr/bin/env bash
set -e

workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export SCOUT_WORKSPACE="${workspace_dir}"
source /opt/ros/humble/setup.bash

if [[ ! -f "$workspace_dir/install/setup.bash" ]]; then
  echo "Workspace is not built: run colcon build --symlink-install first." >&2
  exit 1
fi
source "$workspace_dir/install/setup.bash"
export PYTHONPATH="$workspace_dir/src/NeuPAN:$workspace_dir/.deps/python${PYTHONPATH:+:$PYTHONPATH}"

if ! can_details=$(ip -details link show can0 2>/dev/null); then
  echo "CAN interface can0 was not found." >&2
  exit 1
fi
if [[ "$can_details" != *"bitrate 500000"* ]]; then
  sudo ip link set can0 down
  sudo ip link set can0 type can bitrate 500000
  can_details=$(ip -details link show can0)
fi
if [[ "$can_details" != *",UP,"* ]]; then
  sudo ip link set can0 up
fi

exec ros2 launch neupan_ros2 scout.launch.py "$@"
