# Changelog

Changes to `bijux-gnss-testkit` shared test evidence are recorded here.
Workspace-wide release notes live in [../../CHANGELOG.md](../../CHANGELOG.md).

## Unreleased

### Added

- Changelog entry point for fixtures, reference data, independent reference
  models, and deterministic test-truth generation.

### Owned Change Types

- Shared fixtures and checked-in reference datasets used by multiple crates.
- Independent antenna, atmosphere, coordinate, broadcast, RTK, and signal truth
  models used for tests.
- Synthetic observation and signal inputs that must remain independent from the
  production code being tested.

### Not Owned Here

- Production receiver execution, navigation solver implementation, canonical
  signal DSP, and repository persistence policy belong to other crates.
