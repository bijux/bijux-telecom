# bijux-gnss-testkit

`bijux-gnss-testkit` owns shared GNSS test truth, fixtures, and independent
reference models. It exists so product crates can test against evidence that is
not just a wrapper around the implementation under test.

Start here when a test needs reusable truth data, fixture loading, independent
reference-model calculations, or deterministic signal/observation synthesis. Do
not start here for production receiver orchestration, navigation solver
implementation, repository persistence rules, or one-off wrappers.

## Reader Route

| question | go next |
| --- | --- |
| Which fixture or reference dataset is shared? | [docs/FIXTURES.md](docs/FIXTURES.md), [docs/REFERENCE_DATA.md](docs/REFERENCE_DATA.md) |
| Which independent model computes expected behavior? | [docs/TRUTH_MODELS.md](docs/TRUTH_MODELS.md), `src/reference_models/` |
| Which antenna or signal truth helper is available? | [docs/ANTENNA.md](docs/ANTENNA.md), [docs/SIGNAL.md](docs/SIGNAL.md) |
| Which Rust API is public? | [API.md](API.md), `src/lib.rs` |
| How is independence protected? | [docs/INDEPENDENCE.md](docs/INDEPENDENCE.md), `tests/scientific_independence.rs` |
| What changed in this package? | [CHANGELOG.md](CHANGELOG.md) |

## Owned Boundary

- deterministic fixture loading across crates
- checked-in reference datasets used as shared test evidence
- independent reference models used to compute expected behavior
- truth generation for acquisition, antenna, observation, and position tests

This crate does not own production receiver orchestration, navigation solver
implementations, repository persistence rules, or throwaway test-only wrappers
around the same helper being tested.

```mermaid
flowchart TB
    fixture["fixture or reference data"]
    model["independent model"]
    truth["test truth"]
    product["product crate test"]

    fixture --> model
    model --> truth
    truth --> product
```

## Source Map

- `src/fixtures.rs` owns deterministic typed fixture loading.
- `src/reference_data/` owns checked-in public truth inputs and derived records.
- `src/reference_models/` owns private independent scientific models.
- `src/position_truth/`, `src/antenna/`, and `src/signal/` own reusable
  truth-generation helpers.

## Documentation Map

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- [API.md](API.md)
- [docs/ANTENNA.md](docs/ANTENNA.md)
- [docs/BOUNDARY.md](docs/BOUNDARY.md)
- [docs/CONTRACTS.md](docs/CONTRACTS.md)
- [docs/FIXTURES.md](docs/FIXTURES.md)
- [docs/SIGNAL.md](docs/SIGNAL.md)
- [docs/PUBLIC_API.md](docs/PUBLIC_API.md)
- [docs/REFERENCE_DATA.md](docs/REFERENCE_DATA.md)
- [docs/TESTS.md](docs/TESTS.md)
- [docs/TRUTH_MODELS.md](docs/TRUTH_MODELS.md)
- [docs/INDEPENDENCE.md](docs/INDEPENDENCE.md)

## Verification Focus

Use independence tests when changing shared truth:

```sh
cargo test -p bijux-gnss-testkit --test scientific_independence
cargo test -p bijux-gnss-testkit --test integration_guardrails
```

Repository-wide lanes and package routing are documented in
[../../README.md](../../README.md).
