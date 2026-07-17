# Boundary

Owner: reusable GNSS signal definitions and DSP primitives

## Scope

`bijux-gnss-signal` owns:
- signal catalogs and wavelength helpers
- spreading-code and secondary-code generation
- local-code sampling and code-phase math
- replica generation and carrier/code wipeoff helpers
- front-end quality and spectrum helpers
- tracking-loop primitives and related uncertainty helpers
- raw-IQ metadata and sample conversion utilities
- signal-layer observation compatibility checks

## What this crate must not own

- filesystem-backed sample ingestion
- receiver scheduling, channel orchestration, or artifact persistence
- orbit, atmospheric, PPP, or RTK estimation
- CLI commands or maintainer workflows

## Allowed dependencies

- `bijux-gnss-core` for foundational contracts
- numeric and FFT libraries needed for DSP primitives

## Effect model

This crate should be computationally pure from a product perspective. It may transform samples and
analyze in-memory data, but it should not decide run layouts, datasets, or repository workflows.

## Change standard

Signal definitions and DSP helpers are reused across the workspace. Changes here must preserve
clear ownership and deterministic behavior, especially when they affect code generation, timing, or
physical interpretation.
