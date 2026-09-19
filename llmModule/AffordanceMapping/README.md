# Pushability Semantic Mapping

> Public status: messages, fusion boundaries, and the reference node remain; the default curtain demo
> does not start an external VLM and the pushability action returns a deterministic local mock result.

Formal `llm_semantic_affordance` should create session-level pushability instances from sparse RGB-D
keyframes, semantic classes, and TF, then publish `/semantic_mapping/pushability_map`, markers,
overlays, and readiness. It provides planning metadata only; it does not authorize mechanical pushing
and does not replace collision detection.

A formal provider must validate structured output, associate it with the current frame and coordinate
frame, and fail conservatively on image, depth, TF, network, or parsing errors. Camera frames, model
responses, and instance snapshots should not be persisted by default. Diagnostics belong outside the
repository in a restricted directory.

Host adaptations include camera topics, calibration, TF, keyframe thresholds, model endpoints, and
credentials. Inject credentials through the deployment environment; never put them in configuration,
logs, or archived model requests.
