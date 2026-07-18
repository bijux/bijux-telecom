# Changelog

Changes to `bijux-gnss-signal` signal definitions and DSP primitives are
recorded here. Workspace-wide release notes live in the
[workspace changelog](../../CHANGELOG.md).

## Unreleased

### Added

- Package changelog entrypoint for signal catalogs, spreading codes, raw sample
  contracts, replicas, NCOs, spectra, and tracking-loop primitives.

### Changed

- Public API and trait documentation now separate reusable signal math from
  receiver orchestration and repository persistence.

## What Belongs Here

- Supported signal registry entries and physical signal metadata.
- Code-family generation, secondary-code handling, and replica construction.
- Raw sample conversion, quantization vocabulary, and reusable DSP helpers.
- Tracking-loop primitives that are not receiver channel lifecycle policy.

## What Belongs Elsewhere

- Receiver scheduling and lock lifecycle belong to `bijux-gnss-receiver`.
- Dataset sidecar discovery belongs to `bijux-gnss-infra`.
- Persisted artifact envelopes belong to `bijux-gnss-core` and infra.
- Navigation estimation belongs to `bijux-gnss-nav`.

## Entry Rules

- Include units, constellation/signal identity, and compatibility impact.
- Record public constants, helper semantics, and sample-format changes.
- Do not claim receiver performance from signal-layer metadata alone.
