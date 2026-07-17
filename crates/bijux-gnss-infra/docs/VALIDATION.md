# Validation

`bijux-gnss-infra` owns infrastructure-side validation over persisted artifacts and reference
comparison entrypoints.

## Validation families

The crate currently owns:

- artifact explanation and validation in `src/artifact_inspection/`
- repository-facing validation adapters in `src/validate_reference.rs`
- infrastructure-friendly re-export points for receiver and optional nav validation helpers

## Why this is distinct from receiver validation

The receiver crate owns runtime-side validation capabilities during and immediately after execution.
This crate owns the repository-facing side: loading, interrogating, and comparing persisted
artifacts once they exist in an infrastructure context.

## Boundary rule

If a validation flow is fundamentally about persisted artifacts, manifests, sidecars, or
repository-facing interpretation, it belongs here. If it is about runtime stage behavior, it
belongs in `bijux-gnss-receiver`.
