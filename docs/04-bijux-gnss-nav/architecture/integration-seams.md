---
title: Integration Seams
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-18
---

# Integration Seams

This page names the places where `bijux-gnss-nav` intentionally meets
neighboring crates without surrendering ownership.

## Seam Map

```mermaid
flowchart LR
    products["external nav<br/>products"]
    nav["nav seams"]
    estimators["estimators"]
    consumers["receiver infra<br/>command"]

    products --> nav --> estimators --> consumers
```

## Seam Families

| seam | owned purpose | should not carry |
| --- | --- | --- |
| `src/api.rs` | curated downstream surface for navigation science | private module layout |
| `EphemerisProvider` | orbit and ephemeris access without one storage shape | repository path discovery |
| `ProductsProvider` | precise product access for PPP and correction logic | CLI product policy wording |
| bias providers | code and phase bias lookup with source evidence | receiver scheduling state |
| EKF traits | reusable measurement-model integration | command workflow orchestration |
| `NavigationEngine` and `PositionRuntime` | runtime-neutral positioning boundary | sample-source ownership |
| PPP and RTK policy surfaces | precise-input availability and ambiguity/fix evidence | infra run layout |

## Seam Use Standard

- Use `api.rs` before reaching for an internal module path.
- Pass scientific inputs and outputs through typed seams.
- Keep repository paths, operator flags, and receiver scheduling state outside
  nav.
- Add provider traits only when multiple callers need the same scientific
  boundary.
- Keep refusal, support, and provenance evidence attached at the seam.

## First Proof Check

Inspect `crates/bijux-gnss-nav/src/api.rs`,
`crates/bijux-gnss-nav/src/orbits/ephemeris.rs`,
`crates/bijux-gnss-nav/src/formats/precise_products/mod.rs`,
`crates/bijux-gnss-nav/src/corrections/biases.rs`,
`crates/bijux-gnss-nav/src/estimation/ekf/traits.rs`,
`crates/bijux-gnss-nav/src/estimation/position/`, and
`crates/bijux-gnss-nav/docs/BOUNDARY.md`.
