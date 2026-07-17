# Boundary

Owner: repository-facing GNSS infrastructure and run-layout mechanics

## Scope

`bijux-gnss-infra` owns:
- dataset registry and raw-IQ metadata resolution
- run directory identity, manifests, reports, and history
- artifact explanation and validation
- experiment sweep expansion and profile overrides
- validation-reference adapters and infrastructure-friendly API composition

## What this crate must not own

- signal-processing implementations
- navigation-solvers or orbit/atmosphere models
- receiver channel orchestration
- operator command parsing and rendering

## Effect model

This crate is allowed to touch repository-facing concerns: filesystem paths, manifests, reports,
dataset configs, and artifact payloads. Those effects are its reason to exist, but they must remain
typed and explicit.

## Dependency rule

This crate may aggregate lower-level product APIs for infrastructure convenience, but it should not
become a catch-all home for unrelated helpers that merely lacked a better place.
