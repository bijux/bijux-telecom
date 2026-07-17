# Tests

## Entry points

- `tests/integration_dep_rules.rs` verifies the workspace dependency DAG and selected feature
  constraints.
- `tests/integration_no_cross_layer_imports.rs` blocks imports that break layer boundaries.
- `tests/integration_no_anyhow_eyre.rs` keeps generic error crates confined to the allowed command
  crates.
- `tests/integration_no_ad_hoc_warnings.rs` blocks unmanaged warning/error logging in core runtime
  crates.
- `tests/integration_policy_snapshot.rs` locks the serialized default guardrail configuration.
- `tests/integration_workspace.rs` checks workspace-wide crate and topology expectations.
- `tests/integration_guardrails.rs` verifies this crate can run its own shared guardrails.

## Snapshot fixture

- `tests/snapshots/guardrail_default.json` is the stable serialized policy snapshot used by the
  default-policy test.

## What these tests are protecting

- architecture decisions remain executable, not aspirational
- new crates do not quietly break the dependency DAG
- policy drift becomes visible in review instead of appearing months later as architectural debt
- guardrail configuration stays aligned with the checks wired through `check(crate_root, config)`

## Verification

Useful commands from the repository root:

```sh
cargo test -p bijux-gnss-policies --test integration_dep_rules
cargo test -p bijux-gnss-policies --test integration_workspace
cargo test -p bijux-gnss-policies --test integration_policy_snapshot
```
