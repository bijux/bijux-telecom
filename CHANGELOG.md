# Changelog
<a id="top"></a>

All notable changes to **Bijux GNSS** are documented in this file.
This project adheres to [Semantic Versioning](https://semver.org) and the
[Keep a Changelog](https://keepachangelog.com/en/1.0.0/) format.

---

## Unreleased

### Added

- Added repository-specific contribution guidance and coordinated private
  vulnerability reporting policy.

### Fixed

- Preserved the canonical MkDocs hub registry order in desktop and mobile
  navigation.
- Excluded both root-level and nested `slow__` tests from the fast nextest lane
  used by pull-request CI.

## 0.1.0 - 2026-07-19

### Added

- Established a Rust workspace with durable ownership boundaries for shared
  contracts, signals and DSP, receiver execution, navigation science, dataset
  infrastructure, operator workflows, scientific test support, repository
  policy, and maintainer automation.
- Implemented multi-constellation GNSS processing across GPS, Galileo,
  GLONASS, and BeiDou, including signal definitions, spreading codes,
  acquisition, tracking, observation production, navigation-message handling,
  and constellation-specific timing and orbit behavior.
- Delivered positioning and integrity capabilities spanning broadcast and
  precise products, atmospheric and antenna corrections, weighted and filtered
  solutions, RAIM, RTK ambiguity resolution, PPP state estimation, uncertainty,
  and explicit refusal evidence.
- Added the `bijux gnss` command and `bijux-gnss` Rust facade for inspecting,
  ingesting, acquiring, tracking, running, replaying, diagnosing, exporting,
  and validating GNSS workflows through typed reports and artifacts.
- Made experiments reproducible through registered datasets, raw-IQ sidecars,
  provenance hashing, typed receiver-profile overrides, deterministic sweep
  expansion, stable run identities, manifests, reports, histories, and artifact
  inspection.
- Built scientific validation around checked-in references, independent truth
  models, deterministic synthetic signals and observations, property tests,
  reference comparisons, performance benchmarks, and evidence-strength
  boundaries that prevent unsupported claims.
- Defined curated public APIs, typed units and identities, versioned artifact
  envelopes, diagnostics, support matrices, feature-gated capabilities, and
  dependency-direction guardrails across the workspace.
- Added repository-wide formatting, linting, audit, test, benchmark, Rustdoc,
  strict MkDocs, artifact-hygiene, standards, packaging, and release-readiness
  lanes with generated outputs contained under `artifacts/`.
- Prepared six Apache-2.0 public crates for crates.io and docs.rs, kept three
  support crates repository-only, and added managed publication workflows for
  crate packages, source bundles, checksums, GHCR, and GitHub Releases.
- Renamed the repository and every maintained product-facing reference from
  `bijux-telecom` to `bijux-gnss`, then established the canonical
  `bijux.io/bijux-gnss/` handbook with package-owned navigation and synchronized
  Bijux governance.

### Security

- Updated `time` to 0.3.47 across the workspace and fuzz lockfiles to address
  `RUSTSEC-2026-0009` (`CVE-2026-25727`), which could allow malicious RFC 2822
  input to exhaust the parser stack.
