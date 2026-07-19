---
title: Change Sequence
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-18
---

# Change Sequence

A safe infra change starts with the repository contract, not the source file.
Use this path for dataset identity, run placement, persisted records, artifact
inspection, overrides, sweeps, provenance, or reference adapters.

## Before Editing

Write one sentence in this form:

> A `<reader or caller>` will observe `<changed repository behavior>` when
> `<declared input or state>` occurs.

If the sentence describes command wording, receiver execution, signal behavior,
or navigation science, route it through the
[ownership decision](../foundation/ownership-boundary.md) before editing infra.

## Safe Change Path

1. Identify every writer and reader of the changed state. Include older
   persisted records and feature-disabled callers where relevant.
2. Open the owning contract from the map below and state the invariant that
   changes or remains fixed.
3. Decide compatibility before changing fields, paths, schema versions, public
   exports, precedence, or refusal behavior.
4. Change the narrowest owner. Keep command policy and producer meaning in their
   packages.
5. Add or revise focused evidence for accepted, refused, and contradictory
   inputs.
6. Update the reader guide and operator-facing guidance in the same change when
   observable behavior moves.
7. Review the final diff from the caller's perspective: can someone understand
   the new state without knowing the implementation layout?
8. Record exact automated checks and any behavior supported only by source or
   contract review.

## Contract And Evidence Map

| changed family | contract to read | evidence to select |
| --- | --- | --- |
| registry, sidecar, capture provenance | [Dataset guide](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-infra/docs/DATASETS.md) | dataset and raw-IQ module tests |
| run identity, layout, manifest, report, history | [Run layout guide](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-infra/docs/RUN_LAYOUT.md) | named persistence review and focused tests added for changed behavior |
| artifact explanation or validation | [Validation guide](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-infra/docs/VALIDATION.md) | artifact inspection module tests |
| override or sweep behavior | [Override guide](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-infra/docs/OVERRIDES.md) and [experiment guide](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-infra/docs/EXPERIMENTS.md) | override integration and module tests |
| configuration or repository provenance | [Hashing guide](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-infra/docs/HASHING.md) | focused hash or front-end provenance tests |
| reference alignment or lower-owner validation bridge | [Validation guide](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-infra/docs/VALIDATION.md) | named adapter review plus producer-package proof |
| public imports or feature gates | [Public API contract](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-infra/docs/PUBLIC_API.md) | curated export review and boundary guardrail |

Use [Test Strategy](../quality/test-strategy.md) to calibrate the proof claim and
[Definition of Done](../quality/definition-of-done.md) for the compatibility and
handoff record. If no dedicated test protects the changed behavior, state that
gap explicitly and add focused proof when the behavior is stable enough to
assert.
