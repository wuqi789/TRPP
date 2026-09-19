# Public module boundary

This directory documents the public ROS contracts and deterministic local
fallbacks. Historical implementation notes, provider names, prompts, model
revisions and private inference details are intentionally omitted from the
public workspace.

The released modules retain message/action types, validation failure semantics,
route hand-off interfaces and the local NeuPAN/control integration. Optional
semantic, visual or decision backends are loaded only through an external
`module:factory` adapter selected by the deployment environment.
