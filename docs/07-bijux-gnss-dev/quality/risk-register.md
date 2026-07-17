---
title: Risk Register
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Risk Register

## Main Risks

- the dev binary may accumulate unrelated automation
- governed-file rules may drift away from their documented contract
- benchmark evidence may spread into unmanaged locations
- maintainers may treat command success as proof of broader repository health
  than the command actually covers

## Mitigations

- keep the refusal ledger current
- review new reads and writes as boundary changes
- require docs updates when governed inputs or outputs change
- state verification scope honestly when heavy benchmark execution is skipped

## First Proof Check

Inspect `crates/bijux-gnss-dev/docs/BOUNDARY.md`,
`crates/bijux-gnss-dev/docs/WORKFLOWS.md`,
`crates/bijux-gnss-dev/docs/OUTPUTS.md`, and
`crates/bijux-gnss-dev/tests/integration_guardrails.rs`. Then inspect
`crates/bijux-gnss-dev/src/main.rs` to confirm the highest-risk workflow drift
still has active proof.
