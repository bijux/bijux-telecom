# Tests

`bijux-gnss-nav` has one of the largest verification surfaces in the workspace because it combines
format parsing, physical models, corrections, and estimators.

## Major test families

- decoder and parser tests for GPS, Galileo, BeiDou, GLONASS, RINEX, and precise products
- orbit and clock reference-accuracy tests
- correction and dual-frequency compatibility tests
- SPP, PPP, integrity, and RTK solution tests
- public-data and station-truth integration tests
- guardrail, fault-injection, and long-run stability tests

## Support fixtures

- `tests/data/` contains checked-in public-data references and precise-product fixtures
- `tests/support/` contains shared truth and reference helpers
- `tests/golden_lnav_fixture.rs` and related files lock decoder fixture behavior

## Verification

Useful commands from the repository root:

```sh
cargo test -p bijux-gnss-nav --test integration_position
cargo test -p bijux-gnss-nav --test integration_precise_products
cargo test -p bijux-gnss-nav --test integration_rtk_baseline_accuracy
```
