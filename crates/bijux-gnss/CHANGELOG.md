# Changelog

Changes to the `bijux-gnss` facade crate and `bijux` binary are recorded here.
Workspace-wide release notes live in the [workspace changelog](../../CHANGELOG.md).

## Unreleased

### Added

- The crate is prepared for publication on crates.io with complete package
  metadata, an Apache-2.0 license, API documentation, and links to its GHCR
  source bundle and handbook.
- Installing the crate with Cargo provides the `bijux` command. Adding it as a
  dependency provides the `bijux_gnss` Rust facade.
- The [command guide](docs/COMMANDS.md), [execution guide](docs/EXECUTION.md),
  and [reporting guide](docs/REPORTING.md) define the operator-visible command
  path from parsed arguments to evidence artifacts.

### Changed

- Command documentation now distinguishes facade-owned arguments and reports
  from work delegated to receiver, navigation, signal, and infrastructure
  crates.
- The [public API guide](docs/PUBLIC_API.md) identifies stable facade exports
  without presenting lower-crate implementation details as facade contracts.
- Repository policy and scientific test support remain path-based development
  dependencies and are excluded from the registry package.

## Release Impact

| change | reader impact |
| --- | --- |
| command name, argument, default, or exit behavior | operators may need to change automation |
| report field, schema, or artifact location | report consumers may need to change parsers |
| facade export or feature default | Rust consumers may need to change imports or features |
| delegated lower-crate behavior | the owning package changelog defines compatibility |

The [workspace release handbook](../../docs/bijux-gnss-dev/operations/release-and-versioning.md)
defines versioning and publication order. Entries here describe only the facade
and command behavior included in that release.

## What Belongs Here

- CLI command names, arguments, reports, and operator workflow behavior.
- [Public facade exports](src/lib.rs).
- Command-level validation, routing, and artifact handoff behavior.
- Breaking changes in command output or public facade imports.

## What Belongs Elsewhere

- Receiver-stage internals belong in the
  [receiver runtime history](../bijux-gnss-receiver/CHANGELOG.md).
- Signal math and raw-IQ contracts belong in the
  [signal behavior history](../bijux-gnss-signal/CHANGELOG.md).
- Navigation science belongs in the
  [navigation science history](../bijux-gnss-nav/CHANGELOG.md).
- Repository persistence contracts belong in the
  [infrastructure history](../bijux-gnss-infra/CHANGELOG.md).

## Entry Rules

- Explain the operator or downstream Rust-user impact.
- Name the owning lower crate when this package only routes the change.
- State whether command automation, report consumers, or Rust imports must
  change.
- Omit private implementation changes unless they alter a command, report,
  facade import, feature, or workflow contract.
