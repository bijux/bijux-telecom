# Changelog

Changes to `bijux-gnss-nav` navigation science are recorded here.
Workspace-wide release notes live in the [workspace changelog](../../CHANGELOG.md).

## Unreleased

### Added

- The crate is prepared for crates.io publication with complete package
  metadata, an Apache-2.0 license, API documentation, and release-channel links.
- The [format guide](docs/FORMATS.md), [correction guide](docs/CORRECTIONS.md),
  [orbit guide](docs/ORBITS.md), and [estimation guide](docs/ESTIMATION.md)
  connect external products to solution, degraded, and refusal evidence.
- The [release guide](../../docs/bijux-gnss-nav/operations/release-and-versioning.md)
  defines the scientific claim and proof required for parser, model, orbit,
  estimator, integrity, and tolerance changes.

### Changed

- Navigation documentation now states product provenance, time-system, frame,
  unit, prerequisite, and refusal assumptions instead of describing successful
  solution paths alone.
- The [public API guide](docs/PUBLIC_API.md) distinguishes durable navigation
  contracts from solver-local implementation details.
- Repository policy and scientific test support remain path-based development
  dependencies and are excluded from the registry package.

## Compatibility Notes

- Parsing more input is additive only when previously valid products keep the
  same interpretation and malformed input is not silently accepted.
- A correction, orbit, clock, time, weighting, or covariance change can alter
  positions and integrity claims without changing a public Rust type.
- Refusal and downgrade behavior is public scientific behavior. A solver must
  not replace a refused claim with a plausible-looking unsupported solution.
- A changed tolerance needs a physical or reference-based reason; matching a
  new implementation output is not sufficient evidence.

## What Belongs Here

- Navigation products derived from SP3, CLK, RINEX, ANTEX, and broadcast data.
- Position estimation, residuals, protection levels, RTK, PPP, and RAIM.
- Atmospheric, antenna, tide, bias, group-delay, orbit, and carrier-combination
  corrections.
- Navigation time-system and week-rollover behavior.

## What Belongs Elsewhere

- Raw-IQ ingestion and acquisition/tracking runtime belong to
  `bijux-gnss-receiver`.
- Signal-code production belongs to `bijux-gnss-signal`.
- Operator command surfaces belong to `bijux-gnss`.
- Dataset discovery and run layout belong to `bijux-gnss-infra`.

## Entry Rules

- Record input product, time-system, and refusal-path impact.
- Mention whether a change affects SPP, RTK, PPP, or integrity surfaces.
- State the frame, units, reference source, and expected numerical movement when
  a scientific result changes.
- Do not describe solver changes as successful claims unless evidence and
  refusal behavior changed accordingly.
