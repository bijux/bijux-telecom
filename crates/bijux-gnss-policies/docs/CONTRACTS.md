# Contracts

`bijux-gnss-policies` owns executable repository-policy contracts.

## Guardrail configuration contract

`GuardrailConfig` defines the per-crate policy knobs used by the rule engine. It is the stable way
to declare what a crate is allowed to look like without hard-coding every rule in every test.

## Guardrail execution contract

`check(crate_root, config)` is the crate’s central contract:
- inspect a crate root
- evaluate the configured rules
- return a typed failure if the crate violates policy

That contract keeps policy invocation consistent across crates.

## Repository rule contracts

The test suite and guardrail modules together own contracts for:
- allowed dependency edges between workspace crates
- restrictions on `anyhow` and `eyre` usage
- restrictions on cross-layer imports
- restrictions on unmanaged ad hoc warning/error logging
- requirements for crate-local guardrail tests and workspace structure

The active guardrail families and their responsibilities are enumerated in
[GUARDRAILS.md](GUARDRAILS.md).

## Reporting contract

`src/bin/purity_report.rs` owns a read-only reporting workflow over workspace crate purity
characteristics. It is a maintainer report, not a product feature.

That reporting surface is detailed in [REPORTING.md](REPORTING.md), and the serialized policy
snapshot surface is detailed in [SNAPSHOTS.md](SNAPSHOTS.md).
