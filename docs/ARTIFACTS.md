# Artifacts

This document defines the on-disk artifacts emitted by the GNSS pipeline, their purpose,
schema versioning rules, and invariants. Every artifact includes an `ArtifactHeaderV1` and is
versioned.

## Versioning Policy

- Writers **only** emit the latest version.
- Readers **must** support the last `N` versions (currently `N = 1`, version `v1`).
- All JSON/JSONL artifacts are versioned. Unversioned payloads are considered legacy-only.

## Artifact Types

Each artifact includes:
- `header`: `ArtifactHeaderV1 { schema_version, created_at_unix_ms, git_sha, config_hash, dataset_id, toolchain, features, deterministic, git_dirty }`
- payload field (named per artifact, e.g. `epoch` or `result`)

Example header:

```json
{
  "schema_version": 1,
  "created_at_unix_ms": 1700000000000,
  "git_sha": "abc123",
  "git_dirty": false,
  "config_hash": "sha256:...",
  "dataset_id": "synthetic_basic",
  "toolchain": "rustc 1.78.0",
  "features": ["trace-dump"],
  "deterministic": true
}
```

### obs (ObsEpochV1)
Purpose: observation epochs from tracking.
Files: `obs.jsonl`
Units: meters, seconds, hertz, cycles.
Invariants:
- epochs are monotonic
- no NaNs/Inf in measurements

### track (TrackEpochV1)
Purpose: per-epoch tracking outputs.
Files: `track.jsonl`
Units: hertz, seconds, cycles.
Invariants:
- carrier/code NCO values finite
- prompt I/Q finite

### acq (AcqResultV1)
Purpose: acquisition hits/candidates.
Files: `acq.jsonl`
Units: hertz, chips, seconds.
Invariants:
- carrier/code estimates finite

### eph (GpsEphemerisV1)
Purpose: decoded ephemerides.
Files: `ephemeris.json`
Invariants:
- ephemeris fields present for decoded PRNs

### pvt (NavSolutionEpochV1)
Purpose: position/clock solutions.
Files: `pvt.jsonl`
Units: meters, seconds.
Invariants:
- receiver state finite
- residual RMS finite
- assumptions include explicit `time_system`, `reference_frame`, and `clock_model`

### rtk (Rtk*V1)
Purpose: RTK SD/DD, baseline, fix audit, precision.
Files: `rtk_*.jsonl`
Invariants:
- SD/DD time tags monotonic
- baseline solution finite when emitted

### ppp (PppSolutionEpochV1)
Purpose: PPP filter solutions and diagnostics.
Files: `ppp.jsonl`
Invariants:
- state vector finite
- covariance positive semi-definite (within tolerance)

## Compatibility

The CLI `bijux gnss artifact validate` enforces schema version compatibility and invariants.
Use `bijux gnss artifact convert --to vX` to migrate artifacts (scaffolded).

## Run Manifest

Every command writes `manifest.json` into the run output directory. The manifest includes:
- git SHA + dirty flag
- config hash and snapshot
- dataset id
- enabled features
- replay scope metadata (`deterministic`, `resume`, output selection)
- front-end provenance (`sample_rate_hz`, `intermediate_freq_hz`, `quantization_bits`, normalization/calibration source)

## Validation Evidence Bundle

Validation workflows emit an evidence bundle:
- file: `artifacts/validation_evidence_bundle.json`
- purpose: summarize physical signal context and numerical solution behavior in one machine-readable report
- sections:
  - `physical`: constellation counts, CN0 summary, lock-quality ratio
  - `numerical`: PDOP/residual summary, error RMS, refusal-class counts
  - `diagnostics`: advisory vs enforced-refusal partition

## Reproducibility Verification

Diagnostics workflows can verify run reproducibility artifacts:
- command: `bijux gnss diagnostics verify-repro --run-dir <RUN_DIR>`
- output: run fingerprint plus SHA-256 hashes for manifest/report/artifact bundles
- use case: auditability checks before replay, review, or archival
