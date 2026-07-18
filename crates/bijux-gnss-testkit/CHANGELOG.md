# Changelog

Changes to `bijux-gnss-testkit` shared test evidence are recorded here.
Workspace-wide release notes live in the [workspace changelog](../../CHANGELOG.md).

## Unreleased

### Added

- Package changelog entrypoint for fixtures, reference data, independent
  reference models, public testkit API, and deterministic truth generation.

### Changed

- API, independence, and reference-data documentation now describe provenance,
  shared consumers, and independence expectations.

## What Belongs Here

- Shared fixtures and checked-in reference datasets used by multiple crates.
- Independent antenna, atmosphere, coordinate, broadcast, RTK, and signal truth
  models used by tests.
- Synthetic observation and signal inputs that must remain independent from the
  production code being tested.
- Public helper modules exported by `src/lib.rs`.

## What Belongs Elsewhere

- Production receiver execution belongs to `bijux-gnss-receiver`.
- Navigation solver implementation belongs to `bijux-gnss-nav`.
- Canonical signal DSP belongs to `bijux-gnss-signal`.
- Repository persistence policy belongs to `bijux-gnss-infra`.

## Entry Rules

- Record provenance, unit, fixture, and consumer impact.
- Explain independence implications when a helper changes.
- Do not use this changelog for one-off test setup that is not shared evidence.
