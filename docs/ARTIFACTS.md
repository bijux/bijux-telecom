# Artifacts

This document defines the on-disk artifacts emitted by the GNSS pipeline, their purpose, and
schema versioning rules. Every artifact includes an `ArtifactHeader` and is versioned.

## Versioning Policy

- Writers **only** emit the latest version.
- Readers **must** support the last `N` versions (currently `N = 1`, version `v1`).
- All JSON/JSONL artifacts are versioned. Unversioned payloads are considered legacy-only.

## Artifact Types

Each artifact includes:
- `header`: `ArtifactHeaderV1 { schema_version, created_at_unix_ms, git_sha, config_hash, dataset_id, toolchain, features, deterministic, git_dirty }`
- payload field (named per artifact, e.g. `epoch` or `result`)

### obs (ObsEpochV1)
Purpose: observation epochs from tracking.
Files: `obs.jsonl`

### track (TrackEpochV1)
Purpose: per-epoch tracking outputs.
Files: `track.jsonl`

### acq (AcqResultV1)
Purpose: acquisition hits/candidates.
Files: `acq.jsonl`

### eph (GpsEphemerisV1)
Purpose: decoded ephemerides.
Files: `ephemeris.json`

### pvt (NavSolutionEpochV1)
Purpose: position/clock solutions.
Files: `pvt.jsonl`

### rtk (Rtk*V1)
Purpose: RTK SD/DD, baseline, fix audit, precision.
Files: `rtk_*.jsonl`

### ppp (PppSolutionEpochV1)
Purpose: PPP filter solutions and diagnostics.
Files: `ppp.jsonl`

## Compatibility

The CLI `bijux gnss artifact validate` enforces schema version compatibility and invariants.
Use `bijux gnss artifact convert --to vX` to migrate artifacts (scaffolded).
