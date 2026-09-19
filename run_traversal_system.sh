#!/usr/bin/env bash

set -euo pipefail
umask 077

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export SCOUT_WORKSPACE="${workspace_dir}"
export PYTHONPATH="${workspace_dir}/src/NeuPAN:${workspace_dir}/.deps/python${PYTHONPATH:+:${PYTHONPATH}}"
export SCOUT_PUBLIC_BACKEND="${SCOUT_PUBLIC_BACKEND:-mock}"
if [[ "${SCOUT_PUBLIC_BACKEND}" == "real" ]]; then
    echo "Real mode requires separately installed adapters; configure their module:factory variables." >&2
    launch_name="real_isaac_traversal.launch.py"
else
    launch_name="public_isaac_traversal.launch.py"
fi
if [[ -z "${ISAACSIM_PYTHON_EXE:-}" && -n "${ISAACSIM_PATH:-}" ]]; then
    export ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"
fi
if [[ -z "${ISAACSIM_PYTHON_EXE:-}" ]]; then
    echo "Set ISAACSIM_PYTHON_EXE (or ISAACSIM_PATH) to the Isaac Sim python.sh executable." >&2
    exit 2
fi
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY="${SCOUT_ROS_LOCALHOST_ONLY:-1}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-${workspace_dir}/runtime/ros_logs}"
mkdir -p -- "${ROS_LOG_DIR}"
chmod 700 -- "${ROS_LOG_DIR}"

# Generated ROS setup files read optional variables and require nounset off.
set +u
source /opt/ros/humble/setup.bash
[[ -f "${workspace_dir}/install/setup.bash" ]] && source "${workspace_dir}/install/setup.bash"
[[ -f "${workspace_dir}/groundingdinoVLM/install/local_setup.bash" ]] && source "${workspace_dir}/groundingdinoVLM/install/local_setup.bash"
[[ "${SCOUT_PUBLIC_BACKEND}" == "real" && -f "${workspace_dir}/llmModule/setup_scout_llm.sh" ]] && source "${workspace_dir}/llmModule/setup_scout_llm.sh"
[[ "${SCOUT_PUBLIC_BACKEND}" == "real" && -f "${workspace_dir}/llmdecision/install/setup.bash" ]] && source "${workspace_dir}/llmdecision/install/setup.bash"
set -u

exec ros2 launch obstacle_traversal "${launch_name}" \
    workspace_dir:="${workspace_dir}" "$@"
