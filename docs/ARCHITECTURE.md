# bijux-gnss Architecture

## Dependency Direction
```
CLI → receiver → {signal, nav, core}
signal → core
nav → core
core (no dependencies on other workspace crates)
```
Rules:
- `bijux-gnss-cli` must not be depended on by any other crate.
- `bijux-gnss-nav` must not depend on receiver or CLI.
- `bijux-gnss-signal` depends only on core.
- `bijux-gnss-receiver` orchestrates and may depend on signal + nav + core.

## Crate Responsibilities
- `bijux-gnss-core`: time, identities, observables schema, shared primitives.
- `bijux-gnss-signal`: DSP primitives (NCO, mixing, correlators, FFT helpers, code generators).
- `bijux-gnss-receiver`: pipeline orchestration (ingest → acquire → track → observations). No nav math.
- `bijux-gnss-nav`: ephemeris, satellite state, corrections, solvers, filters (EKF/PPP/RTK nav-side).
- `bijux-gnss-cli`: wiring + UX only.

## Pipeline Stages
1. Ingest: load IQ samples, validate metadata, frame samples.
2. Acquisition: coarse search over PRN and Doppler, return candidates.
3. Tracking: carrier + code tracking loops, produce per-epoch prompt I/Q.
4. Observations: convert tracking to observables (pseudorange, carrier phase, Doppler, C/N0).
5. Navigation: ephemeris decode, sat state, corrections, estimation (EKF/PPP/RTK).
6. Output: structured artifacts (JSON/JSONL/CSV) and reports.

## Contracts
- SamplesFrame: timed complex samples and metadata.
- AcqRequest/AcqResult: acquisition inputs/outputs.
- TrackEpoch: per-epoch tracking data.
- ObservationsEpoch: per-epoch observables with metadata.
- NavSolutionEpoch: per-epoch solution + residuals.

## Data Structures
- Identities: `SatId`, `SigId`, `SignalBand`, `SignalCode`.
- Time: `SampleTime`, `Epoch`, `GpsTime`, `UtcTime`, `TaiTime`, `ReceiverTime`.
- Observables: `ObsEpoch`, `ObsSatellite`, `LockFlags`, `ObsMetadata`.

## Invariants
- Time is monotonic per stream.
- Observations carry units and provenance.
- No domain logic in CLI; no navigation logic in receiver.
