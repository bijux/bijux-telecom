# Antenna

`bijux-gnss-testkit` owns deterministic antenna-effect fixtures used by higher-level GNSS tests.

## Antenna responsibilities

The antenna surface currently owns:

- PPP antenna-effect cases
- RTK antenna-effect cases
- deterministic synthesis helpers used to generate those cases

## Boundary rule

This crate owns antenna-effect truth for tests. Production antenna modeling that feeds navigation
solutions belongs in `bijux-gnss-nav`.
