# Guardrails

This crate exists so structural rules stay executable instead of decaying into unwritten norms.

## Guardrail families

`src/guardrails/mod.rs` wires together four main rule families:

- API-surface checks in `api_surface.rs`
- source-tree checks in `source_tree.rs`
- textual and content checks in `content_policy.rs`
- configuration and error reporting in `config.rs` and `error.rs`

## What the rule engine checks

The central `check(crate_root, config)` flow currently enforces:

- source depth and layout limits
- module-directory shape and empty-module rejection
- public-item discipline, including `api.rs` placement rules
- forbidden filenames and excessive `pub use` patterns
- content rules such as `expect` restrictions, purity-zone checks, and forbidden staged-string
  patterns

This is intentionally opinionated. The crate is supposed to say "no" to structural drift.

## Public API discipline

The API-surface rules matter because the repository wants curated downstream entrypoints instead of
accidental public exposure from implementation files. Crates that opt in can require public items
to stay behind `api.rs`.

## Reporting surface

`src/bin/purity_report.rs` is not a mutating fixer. It is a read-only reporter that summarizes:

- dependency counts
- selected heavy dependencies
- public-item distribution between `api.rs` and non-API files
- feature inventory

Maintainers use it to see structural drift, but policy enforcement still belongs in tests and
guardrail calls.
