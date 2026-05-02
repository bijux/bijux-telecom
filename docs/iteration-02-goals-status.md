# Iteration 02 Goals Status Log

## Active Iteration Goals

Selected goals for this branch (`feat/deep-foundation`) are:

- Goal 25 — Support-status reporting for each constellation-band-code combination
  - Status: done
  - Note: added support matrix artifact generation in receiver engine with explicit supported/planned/unsupported rows and reasons.
- Goal 26 — Acquisition threshold provenance
  - Status: done
  - Note: acquisition results now persist coherent/noncoherent settings, doppler bounds/step, and ratio thresholds used for each candidate.
- Goal 27 — Deterministic acquisition fixtures
  - Status: done
  - Note: added deterministic acquisition fixture corpus and regression tests for repeatable hypotheses and candidate outputs.
- Goal 28 — False-acquisition and missed-acquisition edge-case corpora
  - Status: done
  - Note: added zero-signal and low-CN0 corpora fixtures with explicit non-acceptance expectations.
- Goal 29 — Acquisition explain artifacts
  - Status: done
  - Note: acquisition now emits explain artifacts with ranked candidates, thresholds hit, and selected rationale.
- Goal 36 — Stable channel identity and tracking provenance
  - Status: done
  - Note: tracking epochs now include stable channel id/uid fields and acquisition-to-tracking provenance strings.
- Goal 39 — Explain transitions into degraded/lost
  - Status: done
  - Note: tracking now records transition artifacts with from/to state, reason, and lock quality for each state change.
- Goal 44 — Explicit handling for missing, weak, or inconsistent observables
  - Status: done
  - Note: observation pipeline now classifies per-satellite status and rejection reasons for missing, weak, and inconsistent observables.
- Goal 46 — Observation artifact identities and stable epoch manifests
  - Status: done
  - Note: observation epochs now emit deterministic artifact identities and stable manifest payloads.
- Goal 49 — Explain artifacts for epoch acceptance/rejection
  - Status: done
  - Note: observation decision artifacts now capture epoch-level acceptance/rejection decision and reasons.

## Completed in this iteration plan

- Completed goals: 25, 26, 27, 28, 29, 36, 39, 44, 46, 49.
- Verification basis: workspace `cargo check --workspace` plus targeted iteration-02 acquisition fixture/corpus tests.
