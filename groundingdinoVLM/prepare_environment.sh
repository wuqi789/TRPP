#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${ROOT_DIR}/.venv"

if (($#)); then
    echo "Usage: $0" >&2
    echo "The public repository prepares mock dependencies only." >&2
    exit 2
fi

set +u
source /opt/ros/humble/setup.bash
set -u

if [[ ! -d "${VENV_DIR}" ]]; then
    python3 -m venv --system-site-packages "${VENV_DIR}"
fi

"${VENV_DIR}/bin/python" -m pip install --upgrade "pip<27" "setuptools<80" wheel
"${VENV_DIR}/bin/python" -m pip install -r "${ROOT_DIR}/requirements-mock.txt"

echo "Public mock environment ready. External providers manage their own dependencies."
