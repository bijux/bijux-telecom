---
title: This Package Does Not Own
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# This Package Does Not Own

`bijux-gnss-infra` does not own:

- receiver stage orchestration, tracking sessions, or runtime ports
- signal processing, sample math, or DSP primitives
- orbit parsing, corrections, or navigation estimators
- operator command parsing, report rendering, or command vocabulary
- shared semantic record meaning that belongs in `bijux-gnss-core`

The crate owns repository-facing infrastructure over those surfaces. If the
proposal sounds like product behavior with a file nearby, infra is probably the
wrong owner.
