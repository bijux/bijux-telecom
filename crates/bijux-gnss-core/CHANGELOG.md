# Changelog

Changes to `bijux-gnss-core` shared contracts are recorded here. Workspace-wide
release notes live in the [workspace changelog](../../CHANGELOG.md).

## Unreleased

### Added

- Package changelog entrypoint for identifiers, units, time systems,
  observation records, diagnostics, support matrices, and artifact envelopes.

### Changed

- API documentation now explains core as the shared vocabulary boundary instead
  of listing exported symbols without reader guidance.

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
- Do not describe downstream convenience imports as core ownership.
