# bijux-gnss-testkit

`bijux-gnss-testkit` owns shared GNSS test truth, fixtures, and independent reference models.

## Scope

This crate owns:

- deterministic fixture loading across crates
- checked-in reference datasets used as shared test evidence
- independent reference models used to compute expected behavior
- truth generation for acquisition, antenna, observation, and position validation

This crate does not own production receiver orchestration, navigation solver implementations,
repository persistence rules, or throwaway test-only wrappers around the same helper being tested.

## Public surface

`lib.rs` exposes a direct module surface for shared test support. The public boundary is organized
around durable test roles such as fixtures, reference data, antenna truth, and signal synthesis
rather than around one-off test files.

## Source map

- `src/fixtures.rs` owns deterministic typed fixture loading.
- `src/reference_data/` owns checked-in public truth inputs and derived records.
- `src/reference_models/` owns private independent scientific models.
- `src/position_truth/`, `src/antenna/`, and `src/signal/` own reusable truth-generation helpers.

## Documentation

The crate root intentionally contains only this README. Durable crate documentation lives under
`docs/`:

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- [docs/BOUNDARY.md](docs/BOUNDARY.md)
- [docs/CONTRACTS.md](docs/CONTRACTS.md)
- [docs/FIXTURES.md](docs/FIXTURES.md)
- [docs/PUBLIC_API.md](docs/PUBLIC_API.md)
- [docs/TESTS.md](docs/TESTS.md)
- [docs/TRUTH_MODELS.md](docs/TRUTH_MODELS.md)

## Verification

Run from the repository root when changing this crate:

```sh
cargo test -p bijux-gnss-testkit --test scientific_independence
cargo test -p bijux-gnss-testkit --test integration_guardrails
```

Repository-wide expectations are documented in [../../README.md](../../README.md).
