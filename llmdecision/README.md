# Obstacle Pushability Assessment

> Public status: input/output schemas, the ROS action, provider abstractions, and offline mock tests
> remain available. The default curtain demo uses `scout_public_mock` and does not start an external
> LLM/VLM provider from this directory.

## Formal Responsibility

This module combines structured scene descriptions, vehicle state, mechanical interaction state, and
optional knowledge sources. It returns an obstacle class, `pushability_probability`, a
`push`/`avoid`/`stop` recommendation, uncertainty, and risk flags. The result is decision evidence;
it is not a velocity, arm, or scan-filter authorization command.

ROS integration is exposed through `/llmdecision/assess_pushability`. The public mock uses the same
action type and returns a deterministic pass result only to validate the cross-module data flow.

## Directory Boundaries

- `api/`: strict input and output schemas
- `core/`: decision fusion, confidence, and result storage
- `llm/`: provider abstraction and offline mock
- `vlm/`: visual-description provider abstraction
- `rag/`: local example knowledge base
- `llmdecision_ros/`: ROS 2 action adapter
- `config/`: credential-free example configuration
- `examples/`: sanitized JSON fixtures only; no real images or run records

## Formal Replacement and Host Adaptation

Implement a formal provider in `llm/` or `vlm/`, or supply an adapter from a private package. Keep
the schema, action name, `request_id`, cancellation, timeout, and conservative failure semantics.
Public code reads only `SCOUT_DECISION_ADAPTER` and `SCOUT_PERCEPTION_ADAPTER` values in
`module:factory` form; the adapter owns models, transport, credentials, certificates, retries, and
private paths.

Never put credentials in README files, YAML, shell arguments, or command history. Inject them from
the deployment secret manager. Error logs must not contain headers, response bodies, remote stderr,
or environment variable values.

## Runtime Data

Default output is written to the ignored `llmdecision/runtime/` directory:

- `latest.json`: latest completion record
- `history/`: request history
- `batch_*`, `latency_*`: batch and latency test output

Runtime files use restricted permissions. Public code does not store raw provider responses, prompts,
tracebacks, or internal paths. Real images and scene descriptions may still be sensitive; production
deployments should redirect storage outside the repository and configure expiry cleanup.

The batch tool looks for `examples/images/` by default, but the public repository intentionally does
not provide images there. Use an owned, sanitized local set or `--images-dir` pointing outside the
repository.

## Offline Tests

```bash
cd "$SCOUT_WORKSPACE/llmdecision"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q tests
```

Standalone formal execution is not part of the public demo. Read the root `SECURITY.md` before
enabling an external provider.
