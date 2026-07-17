# Pipeline

`bijux-gnss-receiver` owns runtime composition across receiver stages.

## Stage families

`src/pipeline/` currently owns:

- acquisition and acquisition assistance
- tracking and tracking-state reporting
- observation construction and observation validation
- optional navigation and navigation filtering when the `nav` feature is enabled
- pipeline step reporting through `StepReport` and `StepStats`

## Boundary rules

- Stage ordering, handoff, and runtime state transitions belong here.
- Signal math reused across contexts belongs in `bijux-gnss-signal`.
- Navigation-domain solver science belongs in `bijux-gnss-nav`.
- Repository manifests, reports, and persisted directories belong in `bijux-gnss-infra`.

This crate is where those lower-level surfaces are composed into a receiver, not where they are
re-owned.
