#!/usr/bin/env bash

set -euo pipefail
umask 077

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export SCOUT_WORKSPACE="${workspace_dir}"
export PYTHONPATH="${workspace_dir}/src/NeuPAN:${workspace_dir}/.deps/python${PYTHONPATH:+:${PYTHONPATH}}"
if [[ "${SCOUT_PUBLIC_BACKEND:-mock}" != "real" ]]; then
    echo "The standalone semantic-navigation provider is private. Use run_traversal_system.sh for the public mock demo." >&2
    exit 2
fi
echo "Configure the private adapter module:factory variables before real mode." >&2
if [[ -z "${ISAACSIM_PYTHON_EXE:-}" && -n "${ISAACSIM_PATH:-}" ]]; then
    export ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"
fi
if [[ -z "${ISAACSIM_PYTHON_EXE:-}" ]]; then
    echo "Set ISAACSIM_PYTHON_EXE (or ISAACSIM_PATH) for real semantic navigation." >&2
    exit 2
fi
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY="${SCOUT_ROS_LOCALHOST_ONLY:-1}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-${workspace_dir}/runtime/ros_logs}"
mkdir -p -- "${ROS_LOG_DIR}"
chmod 700 -- "${ROS_LOG_DIR}"

set +u
source /opt/ros/humble/setup.bash
source "${workspace_dir}/install/setup.bash"
source "${workspace_dir}/groundingdinoVLM/install/local_setup.bash"
source "${workspace_dir}/llmModule/setup_scout_llm.sh"
set -u

exec ros2 launch llm_module4 scout_semantic_navigation.launch.py \
    use_sim_time:=true use_pushability_mapping:=false "$@"
