# Architecture

`bijux-gnss-policies` is the repository guardrail crate. It turns architectural expectations into
code and makes those expectations executable in CI and local development.

## Source map

- `src/api.rs` is the only curated public surface.
- `src/guardrails/` owns the rule engine:
  - `api_surface.rs` checks public-export discipline
  - `config.rs` defines configurable guardrail limits and toggles
  - `content_policy.rs` checks textual policy constraints
  - `source_tree.rs` checks topology and layout rules
  - `error.rs` defines crate-local failure reporting
- `src/bin/purity_report.rs` generates a workspace report over dependency and public-item patterns.

## Test map

- `tests/integration_dep_rules.rs` locks workspace dependency direction and feature boundaries.
- `tests/integration_no_cross_layer_imports.rs` blocks imports that violate layering.
- `tests/integration_no_anyhow_eyre.rs` constrains error-handling dependencies to the command
  crates.
- `tests/integration_no_ad_hoc_warnings.rs` blocks unmanaged warning/error logging in core runtime
  crates.
- `tests/integration_policy_snapshot.rs` locks the default guardrail configuration snapshot.
- `tests/integration_workspace.rs` checks workspace-wide structural expectations.
- `tests/integration_guardrails.rs` validates the crate against its own shared rules.
- `tests/snapshots/guardrail_default.json` is the stable policy snapshot fixture.

## Dependency direction

This crate sits beside the product crates, not above them at runtime. Product crates use it for
tests and guardrails, but it must not become a dumping ground for product helpers.

## Design constraints

- policy code must stay explicit and reviewable
- repository rules should live in named checks instead of scattered grep fragments
- new guardrails should fail with enough context that maintainers can act on them quickly
