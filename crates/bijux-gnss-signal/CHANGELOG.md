# Changelog

Changes to `bijux-gnss-signal` signal definitions and DSP primitives are
recorded here. Workspace-wide release notes live in the
[workspace changelog](../../CHANGELOG.md).

## Unreleased

### Added

- The crate is prepared for crates.io publication with complete package
  metadata, an Apache-2.0 license, API documentation, and release-channel links.
- The [signal catalog](docs/CATALOG.md), [code-family guide](docs/CODE_FAMILIES.md),
  [DSP guide](docs/DSP.md), and [raw-IQ guide](docs/RAW_IQ.md) now define the
  reusable facts and behavior consumed by receiver and validation code.
- The [release guide](../../docs/bijux-gnss-signal/operations/release-and-versioning.md)
  connects registry, code, sampling, DSP, and raw-IQ changes to reference and
  continuity evidence.

### Changed

- The [public API guide](docs/PUBLIC_API.md) and [trait guide](docs/TRAITS.md)
  separate reusable signal math from receiver orchestration and repository
  persistence.
- Signal documentation now states the units, normalization, wrapping, and
  component-role assumptions that downstream processing must preserve.
- Repository policy remains a path-based development dependency and is
  excluded from the registry package.

## Compatibility Notes

- Adding a catalog entry is not behavior-neutral if it changes default signal
  selection, component roles, or supported-capability claims.
- Any chip polarity, PRN assignment, secondary-code timing, phase origin, or
  wrapping change can alter acquisition and tracking results.
- Sample conversion and raw-IQ metadata changes affect persisted capture
  interpretation even when public type names remain stable.
- DSP changes must report units and normalization assumptions; numerical output
  changes are part of the public behavior of these primitives.

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
- Name the reference, continuity proof, or downstream receiver behavior used to
  assess the change.
- Do not claim receiver performance from signal-layer metadata alone.
