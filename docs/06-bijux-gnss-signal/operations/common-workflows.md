---
title: Common Workflows
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Common Workflows

## Typical Change Families

- add or extend a signal family in `codes/`
- adjust a reusable DSP primitive in `dsp/`
- extend raw-IQ or sample contracts
- strengthen or fix signal-layer validation logic
- refresh reference catalogs or independent support fixtures

## Workflow Rule

Choose the workflow based on the owning surface. A code-family change is not
reviewed the same way as a metadata-contract change, even if both are in the
same crate.

## First Proof Check

Use `crates/bijux-gnss-signal/docs/CODE_FAMILIES.md`,
`crates/bijux-gnss-signal/docs/DSP.md`,
`crates/bijux-gnss-signal/docs/RAW_IQ.md`, and
`crates/bijux-gnss-signal/docs/VALIDATION.md` as the workflow map. Then choose
the matching proof family in `crates/bijux-gnss-signal/tests/` before deciding
whether the work is really one change family or several independent ones.
