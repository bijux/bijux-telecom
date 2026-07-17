# Datasets

`bijux-gnss-infra` is where repository-side dataset handling becomes typed instead of ad hoc.

## Dataset-owned responsibilities

The dataset layer owns:

- `DatasetRegistry` and `DatasetEntry` as the registry contract
- recorded-capture provenance attached to dataset entries
- `parse_ecef` for infrastructure-side coordinate parsing
- raw-IQ sidecar loading through `load_raw_iq_metadata`
- dataset-aware metadata lookup through `resolve_raw_iq_metadata`

## Why this lives here

Dataset resolution is not receiver science and it is not CLI UX. It is infrastructure state:
repository files, sidecars, coordinates, and capture provenance that need to be interpreted the
same way everywhere.

## Boundary rules

- Signal-specific metadata types remain owned by `bijux-gnss-signal`.
- Dataset-driven receiver execution remains owned by `bijux-gnss-receiver`.
- Human-facing dataset command semantics remain owned by `bijux-gnss`.

This crate owns the repository-facing glue between those surfaces.
