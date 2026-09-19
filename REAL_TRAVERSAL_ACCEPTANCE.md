# Real Backend Acceptance Boundary

> Status: external adapter. This document defines interface and security requirements; it does not
> mean that the public repository contains private models, cloud providers, SSH inference flows,
> model weights, or real arm-control implementations.

The only out-of-the-box acceptance for the public release is the Isaac living-room fixed-route
curtain traversal, using:

```bash
./run_traversal_system.sh headless:=true use_rviz:=false
```

The real backend entry point `SCOUT_PUBLIC_BACKEND=real` is for private adapter integration only. It
checks required environment variables before startup and fails immediately when they are missing;
it never reads credentials through terminal prompts. Real acceptance also requires the deployer to
provide:

- license-compliant perception and semantic models with their runtime assets;
- a cloud or local VLM/LLM adapter;
- a remote execution environment with a verified host key (when SSH is used);
- Piper drivers, force limits, emergency stop, collision protection, and reset procedures;
- data retention, access control, log-redaction, and credential-rotation procedures.

## Preserved Contracts

Real adapters must preserve the following actions:

- `/groundingdino_vlm/verify_target`
- `/llmdecision/assess_pushability`
- `/piper/probe_pushability`
- `/obstacle_traversal/approach_obstacle`

They must also preserve the types, QoS, timeouts, and failure semantics of readiness/status,
`/piper/yolo_*`, `/isaac_joint_states`, `/isaac_joint_command`, and traversal authorization topics.

## Acceptance Data

`run_real_traversal_acceptance.sh` produces rosbags, JSONL, and logs that may contain images, scene
structure, poses, model responses, and local machine information. The default output directory is
ignored by Git. Production deployments should redirect output to a permission-controlled directory
outside the repository and configure automatic expiry cleanup. Publish only summary metrics and
redacted evidence.

The script requires `SCOUT_ALLOW_SENSITIVE_RECORDING=1` explicitly and refuses to start recording
otherwise. Also set its second argument to a controlled directory outside the repository; this
confirmation switch does not mean that the data is already redacted.

## Decision Rules

Real-backend acceptance cannot be replaced by public mock results. At minimum, prove separately that
perception input comes from the current camera frame, every action uses the same `request_id`, the
mechanical probe completes and resets safely, filtering authorization applies only to the target
scan cluster, the full obstacle is retained on rejection, and Isaac/physical odometry actually
crosses the target. Any missing provider, timeout, parse failure, or uncertain arm state must fail
closed.

Environment variable names in real mode are part of the interface and may remain in source; values
must not enter the repository, command-line arguments, logs, rosbags, or acceptance reports. See
`SECURITY.md` for the complete rules.
