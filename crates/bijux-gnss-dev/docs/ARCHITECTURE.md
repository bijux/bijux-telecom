# Architecture

`bijux-gnss-dev` is a single-binary maintainer tooling crate. Its job is not to expose a reusable
Rust library; its job is to keep repository governance and benchmark hygiene explicit and typed.

## Source map

- `src/main.rs` defines the command-line parser, subcommand inventory, and all execution paths.

The crate is intentionally compact today because every command operates on repository files and
shared workspace conventions. If it grows, it should split by owned workflow, not by temporary task
history.

## Command families

- `AuditAllowlist` validates the shape and freshness of `audit-allowlist.toml`.
- `DenyPolicyDeviations` validates governance rules in `configs/rust/deny.deviations.toml`.
- `AuditIgnoreArgs` converts the reviewed allowlist into `cargo audit --ignore ...` arguments.
- `BenchCompare` runs workspace benchmarks, records outputs under `artifacts/`, and compares the
  results against the checked-in baseline.

## Test map

- `tests/integration_guardrails.rs` ensures the crate still satisfies workspace guardrails.

## Design constraints

- Commands must stay repository-owned and deterministic enough for CI and local maintainer use.
- Generated outputs belong under `artifacts/` or another explicitly governed repository location.
- If a command starts owning product behavior rather than maintainer behavior, it belongs in another
  crate.
