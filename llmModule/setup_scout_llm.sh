#!/usr/bin/env bash

# Source this file before building or running the Scout semantic chain.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo "Source this script: source ${BASH_SOURCE[0]}" >&2
    exit 1
fi

_SCOUT_LLM_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
_SCOUT_WORKSPACE_ROOT="$(dirname -- "${_SCOUT_LLM_ROOT}")"

if [[ -f "${_SCOUT_LLM_ROOT}/.venv/bin/activate" ]]; then
    # Keep ROS binary extensions from the system installation while isolating
    # the pure-Python semantic-navigation dependencies.
    source "${_SCOUT_LLM_ROOT}/.venv/bin/activate"
fi
source /opt/ros/humble/setup.bash

export SCOUT_LLM_ROOT="${_SCOUT_LLM_ROOT}"
export SCOUT_WORKSPACE_ROOT="${_SCOUT_WORKSPACE_ROOT}"
export PYTHONNOUSERSITE=1
if [[ ! -f "${_SCOUT_LLM_ROOT}/.venv/bin/activate" ]]; then
    # Compatibility for workspaces prepared before the venv migration.
    export PYTHONPATH="${_SCOUT_LLM_ROOT}/.python_deps${PYTHONPATH:+:${PYTHONPATH}}"
fi

if [[ -f "${_SCOUT_LLM_ROOT}/install/setup.bash" ]]; then
    source "${_SCOUT_LLM_ROOT}/install/setup.bash"
fi

unset _SCOUT_LLM_ROOT _SCOUT_WORKSPACE_ROOT
