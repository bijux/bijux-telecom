# Changelog
<a id="top"></a>

All notable changes to **bijux-gnss-core** are documented in this file.
Workspace-wide release notes live in the
[Bijux GNSS changelog](../../CHANGELOG.md).
This package adheres to [Semantic Versioning](https://semver.org) and the
[Keep a Changelog](https://keepachangelog.com/en/1.0.0/) format.

---

## 0.1.0 - 2026-07-19

### Added

- Defined canonical constellation, satellite, signal, channel, dataset, run,
  and artifact identities for values exchanged across GNSS package boundaries.
- Added strongly typed units, coordinate representations, geodetic conversion,
  physical conventions, time scales, epochs, durations, and week-rollover
  context.
- Introduced validated workspace configuration contracts with schema versions,
  structured validation reports, typed failures, and stable error categories.
- Modeled acquisition candidates, tracking epochs and transitions, observation
  epochs, navigation inputs, differencing records, and stage-specific evidence
  as shared data contracts.
- Added observation-quality, uncertainty, covariance, code-carrier divergence,
  receiver-clock, cycle-slip, carrier-arc, and rejection evidence used by
  receiver and navigation consumers.
- Established machine-readable diagnostics with stable codes, severities,
  events, summaries, refusal reasons, and deterministic reporting semantics.
- Defined navigation solution, RTK, PPP, residual, integrity, quality, and
  degraded-or-refused outcome records without owning estimation algorithms.
- Added version-one artifact envelopes and payloads for acquisition, tracking,
  observations, navigation, and support matrices with explicit validation and
  serialization behavior.
- Introduced support-matrix and capability records that distinguish catalog
  availability from acquisition, tracking, observation, and navigation support.
- Published a curated `api` module for the shared contract families, backed by
  statistics and geodesy helpers while preserving a dependency direction that
  keeps runtime, signal, navigation, and persistence policy out of core.
