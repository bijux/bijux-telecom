---
title: Code Navigation
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Code Navigation

Use this route when reading the crate for the first time or reviewing a change.

## Recommended Path

1. Start at `src/lib.rs` to confirm the crate boundary and public facade.
2. Read `src/api.rs` to understand what downstream crates are supposed to use.
3. Read `src/catalog.rs` to see supported signal identity and physical meaning.
4. Move into `src/codes/` or `src/dsp/` depending on whether the change is
   about canonical code behavior or reusable processing math.
5. Read `src/raw_iq.rs`, `src/samples.rs`, and `src/obs_validation.rs` when the
   change touches capture metadata, conversion, or observation compatibility.
6. Finish in the matching proof family: registry tests for catalog truth,
   reference tests for code truth, continuity tests for long-duration or
   chunked behavior, raw-IQ and sample-conversion tests for sample contracts,
   and property tests for structural rules.

## Practical Shortcut

If the change touches code generation, read the matching integration reference
tests before reviewing the implementation. In this crate, reference proof often
states the contract more clearly than the helper signature alone. If the change
touches reusable DSP state such as NCOs, replicas, or loop tracking, read the
long-duration continuity tests before trusting a local helper rename or math
cleanup.

## First Proof Check

Open the [signal architecture guide](../../../crates/bijux-gnss-signal/docs/ARCHITECTURE.md)
and [signal test guide](../../../crates/bijux-gnss-signal/docs/TESTS.md) first.
Then follow this route through the crate boundary, curated API, catalog source,
code-family source, DSP source, and the matching signal integration test
family.
