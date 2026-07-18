# Changelog
<a id="top"></a>

All notable changes to **bijux-gnss-policies** are documented in this file.
Workspace-wide release notes live in the
[Bijux GNSS changelog](../../CHANGELOG.md).
This repository-only package follows the workspace
[Semantic Versioning](https://semver.org) and
[Keep a Changelog](https://keepachangelog.com/en/1.0.0/) conventions.

---

## 0.1.0 - Unreleased

### Added

- Added a typed `GuardrailConfig` and read-only `check` entrypoint that turns
  repository architecture expectations into deterministic pass or typed
  `GuardrailError` results.
- Added source discovery and topology checks for maximum depth, directory
  fan-out, file size, empty modules, module-directory shape, and reviewable
  crate structure.
- Added durable source naming checks that reject forbidden filenames and
  structural patterns instead of allowing ambiguous ownership to accumulate.
- Added public-surface density limits for public items and re-exports so API
  growth remains deliberate and large facade files require explicit review.
- Added optional `api.rs` placement enforcement and API-purity checks that keep
  curated exports separate from implementation details and side effects.
- Added content policy for panic and `expect` use with narrow, configured
  exceptions and file-specific violation evidence.
- Added source-text policy that rejects staged delivery identifiers and other
  sequencing-shaped strings from durable product code.
- Added configurable purity zones that prevent selected crate paths from
  acquiring forbidden dependencies, imports, or side-effect patterns.
- Added deterministic policy snapshots and focused tests for configuration
  defaults, serialization, rule behavior, allowed exceptions, and actionable
  failure messages.
- Added the read-only `purity_report` binary for crate discovery, dependency
  counts, heavy-dependency visibility, public-item distribution, and feature
  inventories without mutating source or treating observations as policy.
