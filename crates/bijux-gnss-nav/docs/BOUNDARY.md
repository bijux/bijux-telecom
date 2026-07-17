# Boundary

Owner: navigation-domain science, corrections, and estimation

## Scope

`bijux-gnss-nav` owns:
- broadcast and precise orbit state
- navigation message and reference-product parsing
- atmospheric, bias, and signal-combination corrections
- position, integrity, PPP, and RTK estimation logic
- navigation-time and rollover utilities
- supporting physical models needed by those computations

## What this crate must not own

- receiver sample scheduling and channel orchestration
- repository run layout, manifests, and dataset registry logic
- operator CLI behavior
- generic infrastructure wrappers over persisted artifacts

## Dependency rule

This crate should stay above `core` and `signal`, but below `receiver`, `infra`, and CLI surfaces.
If a helper needs runtime orchestration or filesystem ownership, it belongs elsewhere.

## Effect model

This crate is primarily computational. Parsing navigation files and precise products is allowed
because those formats are part of the domain boundary, but repository workflow policy is not.
