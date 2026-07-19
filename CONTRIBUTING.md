# Contributing to bijux-gnss

This document is the operational entry point for repository contributors. It
describes where changes belong, which checks provide evidence, and what a
reviewable pull request must contain.

## Prerequisites

- Rust 1.88.0, managed by `rust-toolchain.toml`
- GNU Make
- Bash
- Python 3 for documentation and repository contract checks

Some targets also require tools such as `cargo-nextest`, `cargo-deny`,
`cargo-audit`, or `uv`. Run `make doctor` and `make help` to inspect the
current environment and command surface.

## Repository Boundaries

Choose the narrowest owner that can express the change honestly:

- `crates/bijux-gnss-core` owns shared identities, units, observations,
  diagnostics, and artifact envelopes.
- `crates/bijux-gnss-signal` owns signal definitions, codes, samples, and DSP.
- `crates/bijux-gnss-nav` owns navigation products, corrections, positioning,
  and integrity.
- `crates/bijux-gnss-receiver` owns acquisition, tracking, receiver execution,
  and receiver evidence.
- `crates/bijux-gnss-infra` owns datasets, provenance, run layout, and
  repository-side persistence.
- `crates/bijux-gnss` owns the public facade and `bijux gnss` command.
- `crates/bijux-gnss-dev`, `crates/bijux-gnss-policies`, and
  `crates/bijux-gnss-testkit` own maintainer automation, repository policy, and
  independent test support.
- `configs/`, `datasets/`, `makes/`, and `docs/` own repository-wide contracts,
  checked-in evidence inputs, automation, and the published handbook.

Do not hand-edit synchronized content under `.bijux/shared/` or managed
`.github/` files. Changes to shared standards belong in `bijux-std` first and
must be consumed from an accepted exact upstream commit.

## Local Workflow

1. Read `README.md` and the handbook page for the affected crate.
2. Make the smallest change that preserves crate ownership and dependency
   direction.
3. Add focused tests or evidence for changed behavior.
4. Update public docs, contracts, fixtures, or metadata when their claims
   change.
5. Run the narrowest relevant checks, then the repository gate appropriate to
   the change.
6. Commit one coherent intent with a durable Conventional Commit message.

Generated outputs, caches, reports, and local run products belong under
`artifacts/`. Cargo follows this rule through `.cargo/config.toml`.

## Validation

The principal repository targets are:

| Command | Evidence |
| --- | --- |
| `make fmt` | Rust formatting is current. |
| `make lint` | Clippy passes across all workspace targets and features with warnings denied. |
| `make test` | The fast nextest lane passes with governed slow tests excluded. |
| `make test-slow` | The governed slow-test lane passes. |
| `make test-all` | All nextest cases, including ignored tests, pass without fast-lane filtering. |
| `make docs-check` | Documentation contracts, generated shell content, links, and strict rendering pass. |
| `make security` | Dependency policy and advisory checks pass. |
| `make release-check` | Public crate metadata and publication contracts pass. |
| `make ci` | The canonical pull-request gate passes. |

Focused commands are useful during development. For example:

```bash
cargo test --locked -p bijux-gnss-nav
cargo test --locked -p bijux-gnss-dev \
  --test integration_nextest_suite_selection
make docs-check
```

Report the exact commands you ran. A focused command proves only its selected
surface.

## Scientific and Operational Claims

Tests and documentation must distinguish deterministic software behavior from
scientific performance evidence.

- Use checked-in truth, provenance, tolerances, and reproducible configuration
  for accuracy or integrity claims.
- Do not present a synthetic fixture as live-sky validation.
- Preserve explicit degraded, unavailable, and refused outcomes.
- Keep generated evidence under `artifacts/`; commit only governed fixtures and
  repository-owned reference data.
- Update the owning handbook and crate documentation when an interface,
  support matrix, report, or artifact contract changes.

## Commits and Pull Requests

Use scoped Conventional Commits:

```text
<type>(<scope>): <lowercase imperative summary>
```

Examples:

- `fix(ci): exclude governed slow tests from the fast lane`
- `docs(governance): define private vulnerability reporting`
- `test(receiver): cover acquisition refusal evidence`

Names and commit subjects must describe durable ownership and intent. Do not
use delivery sequence labels, temporary planning language, or generic subjects
that require the surrounding conversation to make sense.

Before requesting review:

- keep unrelated changes out of the branch;
- explain user-facing, scientific, compatibility, and release effects;
- state which checks passed and which were not run;
- update `CHANGELOG.md` when the change is notable to users or maintainers; and
- verify generated and synchronized content through its owning command rather
  than editing around a failing contract.

Open a regular GitHub issue for non-sensitive questions. Report
vulnerabilities through the private channels in [SECURITY.md](SECURITY.md).
