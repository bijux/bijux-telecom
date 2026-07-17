# Public API

`bijux-gnss-infra` publishes one curated downstream surface through `bijux_gnss_infra::api`.

## Infrastructure-owned API families

### Artifact inspection

The API exposes:
- `artifact_validate`
- `artifact_explain`
- `ArtifactValidationResult`
- `ArtifactExplainResult`

This family is owned directly by `bijux-gnss-infra`.

### Run layout and dataset helpers

The API exposes:
- run-directory, manifest, report, and history helpers
- dataset-registry and raw-IQ metadata loading helpers
- experiment, sweep, override, and hashing helpers

These families are also owned directly by `bijux-gnss-infra`.

## Re-export families

The API also re-exports selected `receiver`, `core`, `signal`, and optional `nav` surfaces for
infrastructure convenience. Those re-exports do not transfer domain ownership into this crate; they
exist so infrastructure consumers can work through one repository-facing API boundary.

## Extension rule

Add a new export here only if it:
1. is infrastructure-owned, or
2. materially improves the infrastructure boundary by presenting a curated repository-facing surface

Do not use `api.rs` as a license to dump unrelated helpers into this crate.
