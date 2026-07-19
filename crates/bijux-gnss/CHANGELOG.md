# Changelog
<a id="top"></a>

All notable changes to **bijux-gnss** are documented in this file.
Workspace-wide release notes live in the
[Bijux GNSS changelog](../../CHANGELOG.md).
This package adheres to [Semantic Versioning](https://semver.org) and the
[Keep a Changelog](https://keepachangelog.com/en/1.0.0/) format.

---

## 0.1.0 - 2026-07-19

### Added

- Added the `bijux_gnss` Rust facade with stable module routes for core, signal,
  receiver, and feature-gated navigation APIs while preserving ownership in the
  underlying crates.
- Added the `bijux` binary with the `gnss` command namespace, structured
  argument parsing, typed execution context, explicit failures, and operator
  help for the supported workflow surface.
- Added dataset inspection and ingestion commands that resolve registered
  inputs, preserve raw-IQ and station metadata, and report provenance without
  claiming that receiver processing occurred.
- Added acquisition, tracking, complete receiver-run, replay, and diagnosis
  commands that delegate scientific behavior to the receiver, navigation,
  signal, and infrastructure crates.
- Added table and JSON reports with deterministic fields, explicit stage
  outcomes, diagnostics, artifact locations, and refusal evidence suitable for
  both interactive use and automation.
- Added governed output handling for manifests, reports, validation evidence,
  exported records, and optional bitmap plots under caller-selected artifact
  locations.
- Added JSON Schema generation and validation for command-facing records behind
  the `schema-validate` feature.
- Added synthetic IQ and navigation export and validation workflows that expose
  declared truth, comparison tolerances, and evidence strength without
  presenting synthetic results as live-sky performance.
- Added feature controls for CLI support, navigation, precise products, tracing,
  schema validation, and plots, with library-only builds available by disabling
  default features.
- Prepared the Apache-2.0 package for crates.io and docs.rs with the `bijux`
  executable, complete metadata, curated documentation, and release-channel
  links for the first public version.
