# Changelog

Changes to `bijux-gnss-receiver` runtime behavior are recorded here.
Workspace-wide release notes live in [../../CHANGELOG.md](../../CHANGELOG.md).

## Unreleased

### Added

- Changelog entry point for receiver configuration, acquisition, tracking,
  observation generation, optional navigation execution, and receiver artifacts.

### Owned Change Types

- Receiver runtime configuration, channel state, diagnostics, and reports.
- Acquisition, tracking, observation, and navigation-stage orchestration.
- Synthetic receiver runs and runtime-side reference validation.

### Not Owned Here

- Canonical signal definitions, repository persistence, standalone navigation
  science, and CLI report formatting belong outside the receiver crate.
