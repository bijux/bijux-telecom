# Changelog

Changes to `bijux-gnss-infra` repository infrastructure are recorded here.
Workspace-wide release notes live in the [workspace changelog](../../CHANGELOG.md).

## Unreleased

### Added

- The crate is prepared for crates.io publication with complete package
  metadata, an Apache-2.0 license, API documentation, and release-channel links.
- The [dataset guide](docs/DATASETS.md) and [run-layout guide](docs/RUN_LAYOUT.md)
  define how typed input provenance becomes deterministic manifests, reports,
  histories, and artifact locations.
- The [override guide](docs/OVERRIDES.md), [experiment guide](docs/EXPERIMENTS.md),
  and [validation guide](docs/VALIDATION.md) explain reproducible configuration
  expansion and infrastructure-side evidence.

### Changed

- The [public API guide](docs/PUBLIC_API.md) separates durable repository
  contracts from receiver science and operator presentation.
- Infrastructure documentation now makes provenance, typed mutation,
  deterministic run identity, persistence ownership, and inspection behavior
  explicit.
- Repository policy remains a path-based development dependency and is
  excluded from the registry package.

## Compatibility Notes

- Dataset sidecar, station-coordinate, and capture-metadata interpretation
  changes affect every run that resolves the same dataset.
- Run identity, directory shape, manifest fields, reports, and history entries
  are durable contracts for future readers and automation.
- Hashing changes affect provenance comparisons and cache identity even when
  stored payload bytes do not change.
- Override and experiment expansion changes affect reproducibility because the
  same requested sweep may produce a different set of receiver profiles.
- Artifact-inspection output is reader-visible behavior; report fields and
  validation summaries need compatibility treatment.

The [infrastructure release guide](../../docs/03-bijux-gnss-infra/operations/release-and-versioning.md)
defines release treatment for persisted footprints and metadata meaning.

## What Belongs Here

- Dataset metadata and repository-side discovery.
- Persisted run footprints, manifests, run reports, and provenance fields.
- Experiment sweep expansion and receiver-profile overrides.
- Artifact inspection, artifact explanation, and validation adapters.

## What Belongs Elsewhere

- Receiver execution science belongs to `bijux-gnss-receiver`.
- Signal generation belongs to `bijux-gnss-signal`.
- Navigation estimation belongs to `bijux-gnss-nav`.
- Operator presentation belongs to `bijux-gnss`.

## Entry Rules

- Record output paths, manifest fields, and provenance behavior explicitly.
- Include dataset and artifact compatibility impact.
- State how existing runs, sidecars, manifests, histories, or automation read
  after the change.
- Do not bury repository contract changes under command-only wording.
