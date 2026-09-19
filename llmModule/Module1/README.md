# Module 1: Language Intent Parsing

> Public status: interfaces, JSON validation, provider abstraction, and offline tests remain; the
> default curtain demo does not start this module.

Module 1 converts the natural-language request on `/user_instruction` into a structured navigation
intent. It owns the language boundary and never generates velocity, trajectories, or control commands.

## Interface

- Input: `/user_instruction` (`std_msgs/msg/String`)
- Output: `/navigation_intent`, or the equivalent canonical intent in an integrated chain
- Fields: target, reference entities, spatial relations, constraints, and policy

Model output must pass strict schema validation. Timeout, authentication failure, invalid JSON, or
unknown fields must never be reported as success.

## Formal Replacement and Host Adaptation

`llm_agent/llm_interface.py` is the provider boundary. A formal implementation may use a local model
or an external service while preserving parser and failure semantics. The deployment environment
supplies the endpoint, model name, CA, proxy, and credentials. Public configuration contains only
invalid example endpoints and environment-variable names; real values do not belong in `config/*.yaml`.

## Tests

```bash
cd "$SCOUT_WORKSPACE/llmModule/Module1"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q test/test_parser.py
```

Offline tests use explicit placeholders, access no network, and never print credential variables.
