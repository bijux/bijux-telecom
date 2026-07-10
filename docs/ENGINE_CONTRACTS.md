# Engine Contracts

These are the shared types exchanged across layers. They live in `bijux-gnss-core` and are the only types allowed across engine boundaries.

## Samples and Observations
- `SamplesFrame`: time-tagged IQ samples
- `AcqSearchSummary`: per-PRN acquisition decision counts for a configured search set
- `ObsEpoch`: per-epoch observables (pseudorange, phase, doppler, CN0) plus anchored GPS receive
  time when dataset timing is known
- `ObsMetadata`: observation provenance, timing source, pseudorange model, explicit Doppler model,
  carrier-phase arc continuity contract, and per-row observation lock state
- `ObsSignalTiming`: per-satellite signal travel time and GPS transmit time derived from the
  receive-time anchor and pseudorange, emitted only when the tracked signal delay is resolved
- `SignalDelayAlignment`: resolved whole-code-period alignment carried from acquisition/tracking
  when absolute pseudorange is known
- `TrackingUncertainty`: per-epoch tracking-derived code phase, carrier phase, Doppler, and C/N0
  uncertainty
- `LockFlags`: raw code, carrier, bit, and cycle-slip booleans
- `ObsMetadata.lock_quality`: derived observation lock-quality scalar used by the observation
  uncertainty model
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
- Every `ObsSatellite` row carries a finite C/N0 value within shared convention bounds, including
  degraded and rejected observation rows
- Every `ObsSatellite` row carries `observation_lock_state`; current contract values are
  `acquired`, `pull_in`, `locked`, `degraded`, `lost`, `reacquired`, `cycle_slip`, and `inactive`
- `SolutionStatus` determines `valid` flag
