# Support Matrix

`bijux-gnss-core` owns the typed support-matrix contract that other crates can exchange and persist.

## Support-matrix responsibilities

The support-matrix surface currently owns:

- `SupportStatus`
- `SignalStageSupport`
- `SignalSupportRow`
- `SupportMatrix`

## Why this lives in core

Support-matrix state is shared meaning about which signals and stages are supported, planned,
deprecated, or unavailable. That is not receiver-only runtime state and not CLI-only reporting
state; it is a cross-crate contract.

## Boundary rule

This crate owns the support-matrix shape. The receiver may populate it, infrastructure may persist
it, and the CLI may render it, but the typed contract itself belongs here.
