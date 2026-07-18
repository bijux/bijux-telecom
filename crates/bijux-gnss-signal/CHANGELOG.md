# Changelog

Changes to `bijux-gnss-signal` signal definitions and DSP primitives are
recorded here. Workspace-wide release notes live in
[../../CHANGELOG.md](../../CHANGELOG.md).

## Unreleased

### Added

- Changelog entry point for signal catalogs, spreading codes, sample contracts,
  replicas, NCOs, spectra, and tracking-loop primitives.

### Owned Change Types

- Supported signal registry entries and physical signal metadata.
- Code-family generation, secondary-code handling, and replica construction.
- Raw sample conversion, quantization vocabulary, and reusable DSP helpers.

### Not Owned Here

- Receiver scheduling, dataset sidecar discovery, persisted artifacts,
  navigation estimation, and operator command behavior belong elsewhere.
