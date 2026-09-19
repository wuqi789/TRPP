# Security Policy

## Public Release Security Boundary

The default `SCOUT_PUBLIC_BACKEND=mock` runs entirely locally and must not read API keys, SSH
passwords, or physical-hardware credentials. Cloud models, SSH inference, real arm control, and
physical sensors are external adapter concerns and are not included in the public release.

## Credentials

- Do not store credentials in source files, YAML, README files, shell arguments, screenshots, or
  issues.
- Production adapters must read credentials only from the process environment or the deployment
  platform's secret manager.
- Do not collect passwords through interactive prompts; input can enter terminal recordings or
  automation logs.
- Never log environment variable values, Authorization headers, request bodies, remote stderr, or
  complete provider responses.
- If a secret is exposed, revoke and rotate it immediately, then clean the Git history; deleting
  only the current file is not sufficient remediation.

Environment variable names are public contracts and may remain in the repository; their values are
not repository configuration. Never commit real values in `.env` files.

## Data and Logs

Camera images, point clouds, rosbags, model inputs/outputs, and route diagnostics may contain
interior layouts, people, or equipment information. Write them to a permission-controlled
directory outside the repository by default; `runtime/` may be used for debugging and is ignored.
The startup script uses `umask 077` and writes ROS logs to `runtime/ros_logs/` with mode `0700`.
Use the shortest necessary retention period and remove EXIF, hostnames, absolute paths, timestamps,
network addresses, and personal information before sharing.

Do not commit pickle files, NumPy dumps, SQLite databases, or other opaque serialized files as
example data. They may contain raw scene/model data, and pickle-like formats can execute code when
deserialized. Public tests must use minimal, human-reviewable JSON/YAML fixtures; keep real data in
controlled storage outside the repository.

ROS 2 logs normally reside in the user's ROS log directory. When publishing a bug report, include
only the necessary errors and check for:

- usernames, hostnames, and absolute workspace paths;
- images, point clouds, poses, and entity names in topic data;
- provider URLs, hardware IPs, serial numbers, MAC addresses, and SSH fingerprints;
- any token, cookie, header, or environment variable value.

## Build and Model Files

`build/`, `install/`, `log/`, caches, and Python virtual environments contain local paths and must
not be included in a release. Except for runtime-required upstream checkpoints explicitly listed in
`THIRD_PARTY_NOTICES.md`, users must obtain model weights locally under their applicable licenses.
Rebuild after moving to another host; do not copy colcon artifacts.

Local development-tool metadata such as `.codex/` and `.agents/` is not source code and must not
enter a public release even when it currently contains no credentials. Nested `.git`/`.hg`/`.svn`
directories and symlinks that point outside the workspace are likewise forbidden, preventing
historical credentials, remote URLs, or local files from being carried into the release.

## Network and Hardware

The public default launch sets `ROS_LOCALHOST_ONLY=1` to reduce discovery with physical robots or
other nodes on the local network. For cross-host ROS 2 communication, the deployer must configure
DDS security, network isolation, and access control. Keep physical LiDAR IPs, CAN devices, and SSH
hosts only in uncommitted local configuration.

## Pre-release Checks

At minimum, run:

```bash
./scripts/security_scan.sh
```

Classify every match manually. An interface's `password_env` or a test placeholder is not a
credential, but real values, absolute local paths, and generated logs must be removed. The script
does not determine third-party asset redistribution rights; verify asset licenses manually according
to `THIRD_PARTY_NOTICES.md`.

`run_real_traversal_acceptance.sh` records images, point clouds, poses, and model outputs and starts
only when `SCOUT_ALLOW_SENSITIVE_RECORDING=1` is explicitly set. This switch confirms the risk; it
does not automatically anonymize, encrypt, or delete data on expiry.

## Vulnerability Reports

Do not attach keys, logs, rosbags, or interior data to public issues. Use a private channel
published by the repository maintainers to provide a minimal reproduction, including the affected
module, version, and whether physical hardware is involved.
