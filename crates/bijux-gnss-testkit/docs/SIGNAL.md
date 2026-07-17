# Signal

`bijux-gnss-testkit` owns deterministic signal-oriented fixtures that help validate acquisition and
sample-processing behavior.

## Signal-test responsibilities

The signal test surface currently owns:

- acquisition expectation helpers
- deterministic signal-synthesis helpers
- clipped quantized-IQ generation used by integration tests

## Boundary rule

This crate owns signal test truth, not canonical signal-processing production behavior. The
production signal layer remains owned by `bijux-gnss-signal`.
