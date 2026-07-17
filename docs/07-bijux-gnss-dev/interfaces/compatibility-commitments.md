---
title: Compatibility Commitments
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Compatibility Commitments

`bijux-gnss-dev` is small in code and important in workflow impact.

## Commitments

- keep the binary command inventory explicit and reviewable
- keep governed input and output locations documented
- treat command meaning changes as maintainer workflow changes, not as casual
  CLI cleanup
- preserve the binary-only boundary unless a stronger architectural reason
  appears

## Explicit Non-Commitments

- internal helper layout inside `main.rs` is not a caller promise
- the crate does not promise a reusable Rust API
- product crates are not expected to integrate with this crate as a library
