#!/usr/bin/env bash

# Source this after preparing public dependencies and building the workspace.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo "Source this script: source ${BASH_SOURCE[0]}" >&2
    exit 1
fi

_GDV_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f "${_GDV_ROOT}/.venv/bin/activate" ]]; then
    source "${_GDV_ROOT}/.venv/bin/activate"
fi
if [[ -f "${_GDV_ROOT}/install/local_setup.bash" ]]; then
    source "${_GDV_ROOT}/install/local_setup.bash"
fi
export PYTHONNOUSERSITE=1
unset _GDV_ROOT
