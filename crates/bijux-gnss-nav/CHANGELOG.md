# Changelog

Changes to `bijux-gnss-nav` navigation science are recorded here.
Workspace-wide release notes live in [../../CHANGELOG.md](../../CHANGELOG.md).

## Unreleased

### Added

- Package changelog entrypoint for navigation-product parsing, correction
  models, estimators, orbit propagation, time interpretation, RTK, PPP, and
  integrity evidence.

### Changed

- API, correction, and orbit documentation now describe model input contracts,
  refusal behavior, and estimator-ready evidence.

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
- Do not describe solver changes as successful claims unless evidence and
  refusal behavior changed accordingly.
