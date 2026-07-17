# Experiments

`bijux-gnss-infra` owns repository-facing experiment and sweep description for batch runs.

## Experiment responsibilities

The experiment surface currently owns:

- `ExperimentSpec`
- `SweepParameter`
- sweep parsing through `parse_sweep`
- Cartesian expansion through `expand_sweep`

## Why this lives here

Experiment batching is a repository-facing concern: it describes how one maintained configuration
becomes a set of reproducible run variants. It is not low-level receiver science and not just CLI
argument handling.

## Boundary rule

This crate owns typed experiment description and expansion. It does not own the scientific meaning
of the underlying receiver parameters.
