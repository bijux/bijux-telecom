---
title: Repository Fit
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Repository Fit

`bijux-gnss-dev` sits beside the product crates, not above them as an umbrella
owner and not below them as a shared scientific dependency.

## Upstream Inputs

- reviewed governance files such as `audit-allowlist.toml` and
  `configs/rust/deny.deviations.toml`
- repository benchmark baselines and current-run evidence files
- general-purpose libraries for CLI parsing, regex matching, and TOML parsing

## Downstream Consumers

- maintainers running repository checks locally
- CI or Make workflows that need one reviewed maintainer command surface
- reviewers who need stable evidence locations and typed diagnostics

## Why This Placement Matters

Without this crate, governance logic would either leak into product command
crates or hide inside shell snippets that have no stable ownership, no typed
validation, and no clear review boundary.

## First Proof Check

Inspect `crates/bijux-gnss-dev/docs/ARCHITECTURE.md`,
`crates/bijux-gnss-dev/docs/PUBLIC_API.md`, and
`crates/bijux-gnss-dev/docs/TESTS.md`. Then inspect
`crates/bijux-gnss-dev/src/main.rs` and the integration tests to confirm this
crate still sits beside the product path as a repository-maintenance owner
rather than as a hidden umbrella over product behavior.
