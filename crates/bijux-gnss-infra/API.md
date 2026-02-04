# bijux-gnss-infra API

Stable public API surface exposed via `crates/bijux-gnss-infra/src/api.rs`.

Artifacts
- `artifact_validate`, `artifact_explain`.
- `ArtifactValidationResult`, `ArtifactExplainResult`.

Run layout
- `RunContextArgs`, `RunDirLayout`, `RunManifest`, `RunReport`.
- `run_dir`, `artifacts_dir`, `artifact_header`, `write_manifest`, `write_run_report`.

Datasets
- `DatasetRegistry`, `DatasetEntry`, `parse_ecef`.

Experiments
- `ExperimentSpec`, `SweepParameter`.
- `parse_sweep`, `expand_sweep`.

Overrides
- `CommonOverrides`, `apply_overrides`, `apply_common_overrides`, `apply_sweep_value`.

Hashing
- `hash_config`, `git_hash`, `git_dirty`, `cpu_features`.

Re-exports
- `receiver`, `core`, `signal` (and `nav` when feature `nav` is enabled).
