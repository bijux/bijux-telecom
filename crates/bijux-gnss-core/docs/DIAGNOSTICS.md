# Diagnostics

`bijux-gnss-core` owns the shared diagnostic language used across the GNSS stack.

## Diagnostic responsibilities

The diagnostics surface currently owns:

- `DiagnosticSeverity`
- `DiagnosticEvent`
- `DiagnosticSummary` and `DiagnosticSummaryEntry`
- stable diagnostic-code metadata through `DiagnosticCode` and `DIAGNOSTIC_CODES`
- diagnostic aggregation via `aggregate_diagnostics`

## Why this belongs here

Diagnostics are cross-crate contracts, not local logging details. Receiver, navigation,
infrastructure, and CLI layers need one shared way to describe machine-readable failure and warning
semantics without inventing incompatible code tables.

## Boundary rule

This crate owns diagnostic meaning and aggregation shape. It does not own runtime log routing or
operator-facing presentation. Those belong in higher layers.
