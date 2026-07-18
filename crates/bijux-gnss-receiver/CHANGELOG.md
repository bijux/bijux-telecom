# Changelog

Changes to `bijux-gnss-receiver` runtime behavior are recorded here.
Workspace-wide release notes live in the [workspace changelog](../../CHANGELOG.md).

## Unreleased

### Added

- The crate is prepared for crates.io publication with complete package
  metadata, an Apache-2.0 license, API documentation, and release-channel links.
- The [pipeline guide](docs/PIPELINE.md), [runtime guide](docs/RUNTIME.md), and
  [artifact guide](docs/ARTIFACTS.md) now trace a receiver run from validated
  configuration through acquisition, tracking, observations, optional
  navigation, diagnostics, and typed evidence.
- The [reference-validation guide](docs/REFERENCE_VALIDATION.md) distinguishes
  receiver claims from signal references, navigation truth, and
  infrastructure persistence.

### Changed

- The [port guide](docs/PORTS.md) makes clock, sample-source, and artifact-sink
  side effects explicit instead of hiding them inside stage logic.
- Runtime documentation now states defaults, validation, degraded and refused
  states, support evidence, and feature-gated navigation handoff.
- Repository policy and scientific test support remain path-based development
  dependencies and are excluded from the registry package.

## Compatibility Notes

- A configuration default or validation change alters receiver behavior even
  when an existing configuration still parses.
- Acquisition acceptance, tracking thresholds, channel transitions, CN0,
  uncertainty, and refusal rules are observable runtime contracts.
- A receiver artifact change must identify whether only in-memory evidence
  changes or persisted core and infrastructure contracts also change.
- Default features include navigation. A feature-default or navigation-handoff
  change must describe behavior with and without navigation enabled.

The [receiver release guide](../../docs/bijux-gnss-receiver/operations/release-and-versioning.md)
defines the proof route for runtime, artifact, port, and validation changes.

## What Belongs Here

- Receiver runtime configuration, defaults, diagnostics, and reports.
- Acquisition, tracking, observation, and navigation-stage orchestration.
- Channel state, lock lifecycle, CN0, uncertainty, and refusal evidence.
- Synthetic receiver runs and runtime-side reference validation.

## What Belongs Elsewhere

- Canonical signal definitions belong to `bijux-gnss-signal`.
- Repository persistence belongs to `bijux-gnss-infra`.
- Standalone navigation science belongs to `bijux-gnss-nav`.
- CLI report wording belongs to `bijux-gnss`.

## Entry Rules

- Record runtime behavior changes in terms of observable receiver evidence.
- Mention feature gates when navigation behavior is affected.
- State the old and new default, threshold, transition, report, or refusal
  behavior and name the affected signal or stage.
- Do not hide changed defaults inside vague implementation wording.
