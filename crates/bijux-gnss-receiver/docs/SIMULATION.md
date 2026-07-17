# Simulation

`bijux-gnss-receiver` owns receiver-boundary simulation helpers used by tests, demos, and
validation.

## Simulation responsibilities

The simulation surface currently owns:

- synthetic receiver execution under `src/sim/`
- the curated `sim` re-export exposed when `nav` is enabled

## Boundary rule

Simulation here is about exercising the receiver boundary. It should not become a generic test-data
bucket or a replacement for the dedicated truth ownership in `bijux-gnss-testkit`.
