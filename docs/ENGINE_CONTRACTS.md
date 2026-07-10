# Engine Contracts

These are the shared types exchanged across layers. They live in `bijux-gnss-core` and are the only types allowed across engine boundaries.

## Samples and Observations
- `SamplesFrame`: time-tagged IQ samples
- `AcqSearchSummary`: per-PRN acquisition decision counts for a configured search set
- `ObsEpoch`: per-epoch observables (pseudorange, phase, doppler, CN0) plus anchored GPS receive
  time when dataset timing is known
- `ObsSignalTiming`: per-satellite signal travel time and GPS transmit time derived from the
  receive-time anchor and pseudorange, emitted only when the tracked signal delay is resolved
- `SignalDelayAlignment`: resolved whole-code-period alignment carried from acquisition/tracking
  when absolute pseudorange is known
- `TrackingUncertainty`: per-epoch code phase, carrier phase, Doppler, and C/N0 uncertainty
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
- Acquisition returns one selected outcome per searched PRN, even when the frame is too short for a usable search
- Epochs are monotonic in `t_rx_s`
- No NaNs/Inf in artifacts
- `SolutionStatus` determines `valid` flag
