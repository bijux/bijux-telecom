# Changelog

Changes to `bijux-gnss-core` shared contracts are recorded here. Workspace-wide
release notes live in the [workspace changelog](../../CHANGELOG.md).

## Unreleased

### Added

- The crate is prepared for crates.io publication with complete package
  metadata, an Apache-2.0 license, API documentation, and release-channel links.
- The [contract guide](docs/CONTRACTS.md) and
  [serialization guide](docs/SERIALIZATION.md) identify the shared records and
  persisted meanings that downstream crates may rely on.
- The [release guide](../../docs/02-bijux-gnss-core/operations/release-and-versioning.md)
  maps public exports, validation changes, serialized data, units, time
  systems, and diagnostics to explicit compatibility decisions.

### Changed

- The [public API guide](docs/PUBLIC_API.md) now presents the crate as the
  curated shared-vocabulary boundary rather than an inventory of exported
  symbols.
- The [invariant guide](docs/INVARIANTS.md) connects public exports, artifact
  coherence, timekeeping, and dependency direction to their enforcing tests.
- Repository policy remains a path-based development dependency and is
  excluded from the registry package.

## Compatibility Notes

- Adding a Rust export can be source-compatible while still changing serialized
  data, exhaustive matches, or downstream validation behavior.
- A changed unit, coordinate frame, time-system interpretation, or artifact
  field meaning is a breaking contract change even when the Rust type is
  unchanged.
- New diagnostics may be additive; changing an existing code's condition or
  severity changes machine-readable behavior.
- Tightened validation must identify which previously accepted state is now
  rejected and why that state was invalid.

## What Belongs Here

- Public shared types exchanged across GNSS crates.
- Unit, time, coordinate, identity, observation, and solution semantics.
- Versioned artifact envelopes and validation rules.
- Diagnostic code and severity taxonomy changes.

## What Belongs Elsewhere

- Runtime orchestration belongs to `bijux-gnss-receiver`.
- DSP implementations belong to `bijux-gnss-signal`.
- Navigation estimators belong to `bijux-gnss-nav`.
- Filesystem layout and run persistence belong to `bijux-gnss-infra`.

## Entry Rules

- Treat public type and serialized-shape changes as compatibility-sensitive.
- Record unit, time-system, and schema implications explicitly.
- Name the affected artifact version, diagnostic code, or invariant when one is
  involved.
- Do not describe downstream convenience imports as core ownership.
