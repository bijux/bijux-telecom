# Changelog
<a id="top"></a>

All notable changes to **bijux-gnss-dev** are documented in this file.
Workspace-wide release notes live in the
[Bijux GNSS changelog](../../CHANGELOG.md).
This repository-only package follows the workspace
[Semantic Versioning](https://semver.org) and
[Keep a Changelog](https://keepachangelog.com/en/1.0.0/) conventions.

---

## 0.1.0 - Unreleased

### Added

- Added the repository-only `bijux-gnss-dev` binary as a narrow maintainer
  control plane that resolves governed inputs from an explicit workspace root
  and remains outside the public GNSS runtime.
- Added audit-allowlist validation for advisory identifiers, rationales, owners,
  review links, expiry dates, duplicate records, and malformed current entries.
- Added deterministic Cargo audit ignore-argument derivation that accepts valid
  current and preserved identifiers, then sorts and deduplicates them without
  presenting derivation as policy approval.
- Added dependency-policy deviation validation for stable identities, owners,
  reasons, shared-standards review links, expiry dates, and duplicate records.
- Added benchmark execution for receiver correlation, acquisition, and tracking
  paths together with navigation EKF updates through explicit Cargo benchmark
  selection.
- Added normalized benchmark snapshots, captured raw output, baseline
  comparison, configurable regression ratios, and strict-mode failure when a
  maintained baseline is exceeded.
- Defined honest benchmark behavior when no baseline exists: execution and
  current-snapshot generation succeed, while regression acceptance is reported
  as unavailable rather than implied.
- Added repository test evidence that validates the governed slow-test roster,
  resolves every listed case, enforces deterministic ordering, and proves that
  fast and slow nextest expressions are exact complements.
- Added typed command parsing, actionable failures, deterministic output, and
  artifact-contained benchmark logs suitable for local and CI consumption.
- Documented maintainer command, governance-file, output, benchmark, audit,
  architecture, and workflow contracts while keeping this package unpublished
  and free of product-facing GNSS commands.
