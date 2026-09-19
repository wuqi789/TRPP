# Scout Mini core-framework data-flow figure

> This figure describes the intended full system. In the public repository's
> default launch, private perception/reasoning providers and arm motion are
> replaced by deterministic local mocks; Isaac Sim, TF, LiDAR, NeuPAN, and base
> motion remain executable.

The figure source is `render_figure.py`. Rendered deliverables are generated as SVG, PDF,
and 300-dpi PNG so that the vector version can be used in the manuscript and the raster version can
be used for review.

## Figure caption (English)

**Data flow of the proposed LLM-guided semantic navigation and event-triggered obstacle traversal
framework.** A natural-language instruction is converted into a constrained semantic intent,
grounded against a static scene graph, and verified by deterministic entity, topology, geometry,
and planner checks. Only a verified request reaches the Module 4 execution bridge, where
model-proposed coarse waypoints are sanitized and dispatched through a FIFO interface to NeuPAN.
During execution, a synchronized LiDAR--RGB event coordinator identifies obstacles intersecting the
current path corridor and invokes two independent provider-neutral gates in parallel: target
verification and pushability assessment. A tracked obstacle is granted a time-bounded
scan-filtering and low-speed traversal authorization only when both gates pass;
rejection, timeout, provider failure, or watchdog expiry retains/restores the full scan and yields a
conservative avoidance or stop response.

## Figure Caption (English)

**Data flow of the proposed semantic navigation and event-triggered obstacle traversal framework.**
A natural-language instruction is converted into a structured semantic intent, grounded against a
static scene graph, and checked by deterministic validation. Only a verified request reaches the
Module 4 execution bridge, which sanitizes route candidates and submits them to NeuPAN through a FIFO
interface. During execution, a synchronized LiDAR-RGB event coordinator detects obstacles in the
current path corridor and invokes target verification and pushability assessment in parallel. A
tracked obstacle receives time-bounded scan filtering and low-speed traversal authorization only when
both gates pass. Rejection, timeout, provider failure, or watchdog expiry retains or restores the
full scan and produces a conservative avoidance or stop response.

## Rendering

```bash
python3 render_figure.py
```

The diagram describes the formal authorization architecture and does not claim
that the public mock performs physical pushing. Public-demo acceptance is limited
to actual Scout motion through the living-room curtain route in Isaac Sim.
