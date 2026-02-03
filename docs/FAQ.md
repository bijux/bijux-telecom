# FAQ

## What time system does the receiver use?
The internal timeline is receiver time with explicit GPS/UTC conversions defined in `bijux-gnss-core::time`. See `docs/CONCEPTS.md` and `docs/GLOSSARY.md`.

## Which reference frame is used?
WGS‑84 ECEF is the default. Earth rotation during signal transit is modeled explicitly in nav corrections.

## How are ionosphere/troposphere handled?
Broadcast models are supported, with scaffolding for estimable states and dual-frequency combinations. See `docs/NUMERICS.md`.

## What does “deterministic” mean?
Deterministic mode fixes seeds, disables nondeterministic parallelism, and enforces stable ordering to make artifacts reproducible.

## Does the receiver support RTK or PPP?
The architecture supports both. RTK has a baseline solver and ambiguity scaffolding; PPP has a stateful filter and precise products scaffolding.
