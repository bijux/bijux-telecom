# Run Layout

`bijux-gnss-infra` owns the repository contract for persisted GNSS execution state.

## Run-layout responsibilities

The run-layout layer owns:

- `RunContextArgs` and `RunDirectoryLayout`
- `run_dir` and `artifacts_dir` path resolution
- `RunManifest`, `RunReport`, and `RunHistoryEntry`
- manifest and report persistence through `write_manifest` and `write_run_report`
- history append semantics through `append_run_history_entry`
- infrastructure-side artifact header construction

## Why this is a dedicated boundary

Run naming, directory shape, manifest persistence, and history append behavior are repository
concerns. If these rules lived inside CLI commands or receiver stages, every caller would risk
forking the repository footprint.

## Change discipline

Changes here affect durability:

- manifests and reports must remain understandable after the command that produced them is gone
- path resolution must stay deterministic for the same run context
- history appends must remain stable enough for downstream indexing and audit workflows

When changing this surface, treat the repository’s persisted execution footprint as a contract, not
as an implementation detail.
