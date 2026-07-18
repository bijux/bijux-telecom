# Changelog

Changes to the `bijux-gnss` facade crate and `bijux` binary are recorded here.
Workspace-wide release notes live in [../../CHANGELOG.md](../../CHANGELOG.md).

## Unreleased

### Added

- Package changelog entrypoint for command, facade, reporting, validation, and
  workflow changes.

### Changed

- CLI documentation now separates operator invocation, lower-crate execution,
  report contracts, and artifact handoff.

## What Belongs Here

- CLI command names, arguments, reports, and operator workflow behavior.
- Facade exports from `src/lib.rs`.
- Command-level validation, routing, and artifact handoff behavior.
- Breaking changes in command output or public facade imports.

## What Belongs Elsewhere

- Receiver-stage internals belong to `bijux-gnss-receiver`.
- Signal math and raw-IQ contracts belong to `bijux-gnss-signal`.
- Navigation science belongs to `bijux-gnss-nav`.
- Repository persistence contracts belong to `bijux-gnss-infra`.

## Entry Rules

- Explain the operator or downstream Rust-user impact.
- Name the owning lower crate when this package only routes the change.
- Do not record private implementation reshuffles unless they change a command,
  report, facade import, or workflow contract.
