# Traits

`bijux-gnss-signal` exposes a small set of reusable trait seams for sample and correlation flows.

## Public trait surface

The intentionally public traits are:

- `SignalSource`
- `SampleSource`
- `Correlator`
- `SampleSink`

## Why these traits live here

These traits sit close to the signal and sample contracts they operate on. They are lightweight
integration seams used by higher-level crates and tests without forcing those crates to own signal
abstractions themselves.

## Boundary rule

These traits should stay minimal and signal-adjacent. Runtime orchestration, scheduling policy, and
artifact persistence do not belong in them.
