# Changelog
<a id="top"></a>

All notable changes to **bijux-gnss-infra** are documented in this file.
Workspace-wide release notes live in the
[Bijux GNSS changelog](../../CHANGELOG.md).
This package adheres to [Semantic Versioning](https://semver.org) and the
[Keep a Changelog](https://keepachangelog.com/en/1.0.0/) format.

---

## 0.1.0 - Unreleased

### Added

- Added a typed dataset registry for named captures, source locations, station
  coordinates, sample metadata, and recorded or synthetic provenance.
- Added raw-IQ sidecar loading and strict coordinate, sample-rate, intermediate
  frequency, capture-time, byte-offset, and sample-format parsing without
  guessing absent metadata.
- Implemented deterministic content and configuration hashing with source
  revision, dirty-state, CPU-feature, and input provenance records.
- Added stable run identity and directory derivation from declared dataset,
  receiver profile, configuration, variation, and execution context.
- Added run-directory persistence for manifests, reports, artifacts, provenance,
  configuration snapshots, and append-only history with explicit record
  schemas.
- Implemented typed receiver-profile overrides that preserve field ownership,
  validate mutations, and reject malformed or unsupported values.
- Added experiment specifications and deterministic Cartesian sweep expansion
  for controlled parameter studies with stable variation identities.
- Added artifact classification, summaries, inspection, explanation, and
  validation adapters for persisted acquisition, tracking, observation,
  navigation, and support evidence.
- Added run preparation and reference-validation workflows that connect
  resolved inputs and repository context to receiver execution without
  redefining receiver or navigation science.
- Published a curated infrastructure `api` with feature forwarding for
  navigation, precise products, and tracing, plus package metadata and
  build-time assets suitable for the first crates.io release.
