#!/usr/bin/env bash
set -euo pipefail

if (($# == 0)); then
    echo "Usage: $0 <natural-language navigation instruction>" >&2
    exit 2
fi
instruction="$*"
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export SCOUT_WORKSPACE="${workspace_dir}"
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY="${SCOUT_ROS_LOCALHOST_ONLY:-1}"

set +u
source /opt/ros/humble/setup.bash
source "${workspace_dir}/install/setup.bash"
source "${workspace_dir}/groundingdinoVLM/install/setup.bash"
source "${workspace_dir}/llmModule/install/setup.bash"
set -u

exec ros2 run obstacle_traversal publish_instruction "$instruction"
