# Changelog

Changes to `bijux-gnss-infra` repository infrastructure are recorded here.
Workspace-wide release notes live in the [workspace changelog](../../CHANGELOG.md).

## Unreleased

### Added

- Package changelog entrypoint for dataset registry, run layout, artifact
  inspection, hashing, overrides, experiment sweeps, and reference-validation
  adapters.

### Changed

- API, override, and test documentation now explain repository evidence flow,
  typed mutation, provenance, and package-boundary tests.

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
- Do not bury repository contract changes under command-only wording.
