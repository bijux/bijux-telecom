# Changelog

Changes to `bijux-gnss-receiver` runtime behavior are recorded here.
Workspace-wide release notes live in [../../CHANGELOG.md](../../CHANGELOG.md).

## Unreleased

### Added

- Package changelog entrypoint for receiver configuration, acquisition,
  tracking, observation generation, optional navigation execution, diagnostics,
  and receiver artifacts.

### Changed

- Runtime and port documentation now makes side effects, stage ownership, and
  feature-gated navigation handoff explicit.

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
- Do not hide changed defaults inside vague implementation wording.
