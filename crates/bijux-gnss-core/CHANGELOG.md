# Changelog

Changes to `bijux-gnss-core` shared contracts are recorded here. Workspace-wide
release notes live in [../../CHANGELOG.md](../../CHANGELOG.md).

## Unreleased

### Added

- Changelog entry point for core identifiers, units, records, diagnostics, and
  artifact contracts.

### Owned Change Types

- Public shared types exchanged across GNSS crates.
- Unit, time, coordinate, identity, observation, and solution semantics.
- Versioned artifact envelopes and validation rules.

### Not Owned Here

- Runtime orchestration, DSP implementations, navigation estimators, filesystem
  layout, and operator workflow policy belong outside core.
