# bijux-gnss Architecture

This document describes the current receiver pipeline, crate boundaries, and the key data structures used across the workspace. The intent is to keep this concise and stable as the implementation grows.

## Pipeline Overview

1. **Sample Ingest**
   - Sources provide interleaved `i16` IQ and are converted into the canonical internal type: complex `f32` samples.
   - Streaming is framed into fixed-size epochs (typically 1 ms) with explicit time stamps.

2. **Front-End Processing**
   - Optional frequency translation to baseband (IF removal).
   - Epoch framing and normalization.

3. **Acquisition**
   - Coarse search over Doppler bins and code phase.
   - FFT-based circular correlation against local PRN code replicas.
   - Output: candidate satellites with carrier estimates, code phase, and detection metrics.

4. **Tracking**
   - Per-channel state machine (Idle → Acquired → PullIn → Tracking → Lost).
   - Correlator taps (Early/Prompt/Late) computed each 1 ms epoch.
   - Loop filters (DLL/PLL/FLL) will be layered on top of the tap outputs.

5. **Navigation**
   - Decode navigation bits and solve position/clock states.
   - This is currently scaffolding only.

## Crate Responsibilities

- `bijux-gnss-receiver`
  - Core receiver pipeline and algorithms: acquisition, tracking, navigation scaffolding.
  - Canonical data model (`Sample`, `SamplesFrame`, `Epoch`, `AcquisitionResult`, `TrackingEpoch`).
  - Sample source adapters and test harness utilities.

- `gnss-signal`
  - Shared DSP utilities (NCO, math helpers).
  - Kept small and dependency-light for reuse.

- `gnss-nav`
  - Navigation domain structs (ephemeris, positioning solvers).

- `bijux-gnss-cli`
  - CLI wrapper for testing and quick experiments.

  - Shared GNSS constants.

## Workspace Structure

- `domain/`
  - GNSS signal domain references (e.g., `domain/gps_l1ca/`).
- `configs/`
  - Receiver profiles in TOML, validated by `bijux gnss validate-config`.
- `datasets/`
  - Dataset registry and optional data payloads.

## Key Data Structures

- `Sample` (`Complex<f32>`)
  - Canonical processing type. All ingest paths normalize `i16` IQ to this format using a fixed scale.

- `SamplesFrame`
  - A time-stamped batch of samples: `{ t0, dt_s, iq }`.
  - `t0` is the sample index mapped to absolute time; `dt_s` is the sample period.

- `SampleClock`
  - Maps sample counters to seconds and epoch indices.
  - Defines the **1 ms epoch** as a fixed 0.001-second interval.

- `AcquisitionResult`
  - `{ prn, carrier_freq_hz, code_phase_samples, peak, mean, ratios }`.
  - Encodes detection metrics for ranking candidates.

- `ChannelState` and `Channel`
  - Channel state machine: `Idle → Acquired → PullIn → Tracking → Lost`.
  - Used by tracking to manage lifecycle and lock recovery.

- `TrackingEpoch`
  - Contains correlator tap outputs (Early/Prompt/Late) for each 1 ms epoch.

## Interfaces

- `SampleSource`
  - Streaming interface for sources such as files, SDRs, or memory buffers.
  - Produces typed `SamplesFrame` values rather than raw `i16` slices.

## Logging and Observability

- Feature-gated structured logging via `tracing` in the receiver crate.
- Logs include per-channel state transitions, acquisition hits, and lock status changes.

## Configuration Notes

- `navigation.weighting` controls measurement weighting for PVT:
  - `enabled`: toggles weighting on/off (off uses uniform weights).
  - `min_elev_deg`: minimum elevation for weighting (degrees).
  - `elev_exponent`: exponent applied to normalized elevation weighting.
  - `cn0_ref_dbhz`: reference CN0 used to normalize CN0 weighting.
  - `min_weight`: floor for the combined weight.
