---
title: Local Development
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Local Development

Local signal development should make one physical or computational claim
clearer. A small source edit can affect acquisition search, tracking handoff,
navigation combinations, and artifact metadata downstream, so local work starts
from the claim and not from the file that happened to be easy to edit.

## Local Navigation

| question | open first |
| --- | --- |
| "Is this reusable signal behavior?" | `crates/bijux-gnss-signal/docs/BOUNDARY.md` |
| "Which source family owns it?" | `crates/bijux-gnss-signal/docs/ARCHITECTURE.md` |
| "Is this public API?" | `crates/bijux-gnss-signal/docs/PUBLIC_API.md` and `src/api.rs` |
| "Which test proves it?" | `crates/bijux-gnss-signal/docs/TESTS.md` |
| "Does this touch raw samples or metadata?" | `crates/bijux-gnss-signal/docs/RAW_IQ.md` and `SAMPLES.md` |
| "Does this change reusable DSP behavior?" | `crates/bijux-gnss-signal/docs/DSP.md` |

## Local Loop

1. Write the claim in signal terms: catalog, code, DSP, raw-IQ, sample, trait,
   or validation behavior.
2. Inspect the owning source module and adjacent tests before editing.
3. Keep unrelated signal families out of the same change.
4. Run the focused signal proof first.
5. Run downstream receiver or command tests only when the public claim crosses
   the signal boundary.

## Reader-Facing Standard

A local change is not ready if the only explanation is "the tests pass." A
reader should be able to see which signal fact changed, which physical
assumption backs it, and which proof would catch drift in that fact.
