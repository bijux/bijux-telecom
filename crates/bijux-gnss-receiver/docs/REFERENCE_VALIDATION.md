# Reference Validation

`bijux-gnss-receiver` owns runtime-side comparison helpers between receiver outputs and reference
trajectories.

## Reference-validation responsibilities

The receiver-side validation surface currently owns:

- `ReferenceAlign`
- `ValidationReferenceEpoch`
- `ReferenceCompareStats`
- `SolutionConsistencyReport`
- alignment and comparison helpers in `src/reference_validation.rs`

## Boundary rule

This crate owns runtime-side reference alignment and comparison logic. Repository-facing artifact
inspection and persisted validation workflows belong in `bijux-gnss-infra`.
