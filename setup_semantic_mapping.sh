#!/usr/bin/env bash
set -euo pipefail

cat >&2 <<'EOF'
Semantic mapping assets are not provisioned by the public repository.
The default Isaac demo uses its published scene and local route fixtures.
For a deployment that needs semantic segmentation, install a licensed
provider-owned adapter and configure it outside this workspace.
EOF
exit 2
