#!/usr/bin/env bash
set -euo pipefail
umask 077

WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_DIR="${SOURCE_DIR:-}"
DEST_DIR="${DEST_DIR:-${WORKSPACE}/llmdecision}"

if [[ -z "${SOURCE_DIR}" || ! -d "${SOURCE_DIR}" ]]; then
  echo "ERROR: set SOURCE_DIR to an external llmdecision source directory" >&2
  exit 2
fi
if [[ -e "${DEST_DIR}" ]]; then
  echo "ERROR: destination already exists; refusing to overwrite the independent copy" >&2
  exit 3
fi

copy_complete=false
cleanup_incomplete_copy() {
  if [[ "${copy_complete}" != true && -d "${DEST_DIR}" ]]; then
    find "${DEST_DIR}" -depth -delete
  fi
}
trap cleanup_incomplete_copy EXIT INT TERM

mkdir -p "${DEST_DIR}"
chmod 700 "${DEST_DIR}"
rsync -a \
  --exclude='__pycache__/' \
  --exclude='.pytest_cache/' \
  --exclude='*.pyc' \
  --exclude='examples/output_history/' \
  --exclude='examples/batch_decisions/' \
  --exclude='examples/images/' \
  --exclude='runtime/' \
  --exclude='examples/batch_output.json' \
  --exclude='examples/output_example.json' \
  --exclude='examples/output_examples.json' \
  --exclude='*.stdout' \
  --exclude='*.log' \
  --exclude='.env' \
  --exclude='.env.*' \
  --exclude='*.pem' \
  --exclude='*.key' \
  --exclude='*.token' \
  --exclude='credentials.*' \
  --exclude='secrets.*' \
  --exclude='.ssh/' \
  "${SOURCE_DIR}/" "${DEST_DIR}/"

printf '%s\n' \
  '# Imported llmdecision source' \
  '' \
  '- Source: external SOURCE_DIR (path intentionally omitted)' \
  '- Import mode: regular-file copy; no symbolic links' \
  '- Runtime dependency on the source tree: none' \
  '- Historical output, raw provider responses, caches, and generated results: excluded' \
  > "${DEST_DIR}/UPSTREAM_COPY.md"

if find "${DEST_DIR}" -type l -print -quit | grep -q .; then
  echo "ERROR: copied tree unexpectedly contains symbolic links" >&2
  exit 4
fi

if rg -n --hidden --glob '!UPSTREAM_COPY.md' \
  '(BEGIN [A-Z ]*PRIVATE KEY|/home/[^/ ]+/|Authorization:[[:space:]]*Bearer)' \
  "${DEST_DIR}"; then
  echo "ERROR: imported tree failed the credential/path safety scan" >&2
  exit 5
fi

copy_complete=true
trap - EXIT INT TERM
echo "Prepared independent llmdecision copy at ${DEST_DIR}"
