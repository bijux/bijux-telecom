---
title: This Package Does Not Own
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# This Package Does Not Own

`bijux-gnss-core` does not own:

- signal catalogs, code families, raw-IQ contracts, or DSP primitives
- orbit products, message parsers, correction models, or navigation estimators
- receiver runtime orchestration, ports, or staged execution policy
- dataset registry mechanics, run directories, sweep expansion, or provenance
  hashing
- command-line workflows, operator reports, or top-level user-facing policy

The crate owns shared meaning that those packages depend on. If the proposed
work sounds more like behavior than cross-package meaning, the burden is on the
change to prove that core is the right owner.
