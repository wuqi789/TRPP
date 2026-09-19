#!/usr/bin/env bash
set -euo pipefail

LLM_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${LLM_ROOT}/.venv"
LEGACY_DEPENDENCY_DIR="${LLM_ROOT}/.python_deps"

if /usr/bin/python3 -c 'import ensurepip' >/dev/null 2>&1; then
    /usr/bin/python3 -m venv --system-site-packages "${VENV_DIR}"
else
    # Minimal ROS installations often omit python3-venv/ensurepip. The venv
    # interpreter and isolation still work; install into its own site-packages
    # with the system pip instead of requiring a system package change.
    /usr/bin/python3 -m venv --without-pip --system-site-packages "${VENV_DIR}"
fi
VENV_SITE_PACKAGES="$(
    "${VENV_DIR}/bin/python" -c \
        'import sysconfig; print(sysconfig.get_paths()["purelib"])'
)"
if [[ -d "${LEGACY_DEPENDENCY_DIR}" ]]; then
    cp -a "${LEGACY_DEPENDENCY_DIR}/." "${VENV_SITE_PACKAGES}/"
    echo "Migrated cached dependencies from ${LEGACY_DEPENDENCY_DIR}"
else
    /usr/bin/python3 -m pip install \
        --disable-pip-version-check \
        --no-deps \
        --upgrade \
        --target "${VENV_SITE_PACKAGES}" \
        "networkx==3.4.2" \
        "typing-extensions==4.15.0" \
        "pytest==8.3.5" \
        "iniconfig" \
        "pluggy" \
        "tomli" \
        "exceptiongroup"
fi

echo "Scout llmModule Python environment prepared in ${VENV_DIR}"
echo "Activate it through: source ${LLM_ROOT}/setup_scout_llm.sh"

unset VENV_SITE_PACKAGES
