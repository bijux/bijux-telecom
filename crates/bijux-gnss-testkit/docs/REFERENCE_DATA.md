# Reference Data

`bijux-gnss-testkit` owns checked-in reference evidence that multiple GNSS crates rely on in tests.

## Reference-data responsibilities

`src/reference_data/` currently owns:

- station truth inputs and derived records
- troposphere-elevation reference data
- PPP convergence reference data
- checked-in coordinate and related truth records used across tests

## Boundary rule

These assets are shared evidence, not local scratch files for one test. If a dataset exists only to
help a single test file and has no cross-crate truth value, it probably should not live in this
crate.
