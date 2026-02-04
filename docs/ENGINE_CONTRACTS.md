# Engine Contracts

These are the shared types exchanged across layers. They live in `bijux-gnss-core` and are the only types allowed across engine boundaries.

## Samples and Observations
- `SamplesFrame`: time-tagged IQ samples
- `ObsEpoch`: per-epoch observables (pseudorange, phase, doppler, CN0)
- `LockFlags`: lock quality
- `SolutionStatus`: validity state (Invalid, Degraded, Coarse, Converged, Float, Fixed)

## Navigation
- `NavSolutionEpoch` (`NavEpoch` alias): position/clock solution with residuals

## Diagnostics
- `DiagnosticEvent`: severity + code + message
- Diagnostic codes registry: `core::diagnostic::codes`

## Artifacts
- `ArtifactHeaderV1`: schema version + provenance
- Versioned wrappers: `ObsEpochV1`, `TrackEpochV1`, `NavSolutionEpochV1`, `AcqResultV1`

## Invariants
- Epochs are monotonic in `t_rx_s`
- No NaNs/Inf in artifacts
- `SolutionStatus` determines `valid` flag
