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
- observation metadata may carry source `tracking_uncertainty` so downstream weighting can use
  tracking-derived uncertainty instead of CN0-only heuristics

### track (TrackEpochV1)
Purpose: per-epoch tracking outputs.
Files: `track.jsonl`
Units: hertz, seconds, cycles.
Invariants:
- carrier/code NCO values finite
- prompt I/Q finite
- when present, `tracking_uncertainty` carries non-negative per-epoch uncertainty for code phase
  (samples), carrier phase (cycles), Doppler (Hz), and C/N0 (dB-Hz)

### acq (AcqResultV1)
Purpose: acquisition hits/candidates.
Files: `acq.jsonl`
Units: hertz, chips, seconds.
Invariants:
- carrier/code estimates finite
- `acq.jsonl` stores retained acquisition candidates; when top-N retention is greater than 1, multiple rows may exist for one searched PRN
- `acquire_report.json.primary_results` stores exactly one selected outcome per searched PRN
- `acquire_report.json.search_summary` counts selected PRN decisions (`accepted`, `ambiguous`, `rejected`, `deferred`)
- when the input frame is too short for the requested coherent/noncoherent window, acquisition still emits one deferred outcome per searched PRN instead of omitting them

When sub-bin Doppler refinement is available, `acq.jsonl.payload.carrier_hz` carries the refined
carrier estimate that downstream tracking will consume, and
`acq.jsonl.payload.doppler_refinement` preserves the coarse selected bin plus the bounded
refinement offset:
- `coarse_carrier_hz`
- `offset_hz`
- `offset_bins`
- `left_peak_mean_ratio`, `center_peak_mean_ratio`, `right_peak_mean_ratio`

### eph (GpsEphemerisV1)
Purpose: decoded ephemerides.
Files: `ephemeris.json`
Invariants:
- ephemeris fields present for decoded PRNs

### nav decode report
Purpose: navigation-bit demodulation and LNAV subframe alignment from tracking prompt history.
Files: `nav_decode_report.json`
Units: milliseconds, prompt indices, bits.
Invariants:
- `bit_start_ms` is the selected 20 ms GPS L1 C/A bit boundary offset within the prompt history
- `aligned_subframes[*].start_prompt_index` equals `bit_start_ms + start_bit_index * 20`
- `aligned_subframes[*].end_prompt_index_exclusive` equals
  `bit_start_ms + end_bit_index_exclusive * 20`
- `aligned_subframes[*].start_bit_index` values are monotonic
- `preamble_hits` equals `aligned_subframes.len()`

The report includes:
- `sat`
- `bit_start_ms`
- `bit_signs`
- `aligned_subframes[*].start_bit_index`
- `aligned_subframes[*].end_bit_index_exclusive`
- `aligned_subframes[*].start_prompt_index`
- `aligned_subframes[*].end_prompt_index_exclusive`
- `aligned_subframes[*].inverted`
- `aligned_subframes[*].word_count`
- `aligned_subframes[*].parity_ok_count`
- `decoded_subframes[*].alignment`
- `decoded_subframes[*].tlm.preamble`
- `decoded_subframes[*].tlm.parity_ok`
- `decoded_subframes[*].how.tow_count`
- `decoded_subframes[*].how.tow_start_s`
- `decoded_subframes[*].how.alert`
- `decoded_subframes[*].how.anti_spoof`
- `decoded_subframes[*].how.subframe_id`
- `decoded_subframes[*].how.parity_ok`
- `decoded_subframes[*].clock`
  Present when `decoded_subframes[*].how.subframe_id == 1`.
- `decoded_subframes[*].clock.week`
- `decoded_subframes[*].clock.sv_health`
- `decoded_subframes[*].clock.iodc`
- `decoded_subframes[*].clock.toc_s`
- `decoded_subframes[*].clock.af0`
- `decoded_subframes[*].clock.af1`
- `decoded_subframes[*].clock.af2`
- `decoded_subframes[*].clock.tgd`
- `decoded_subframes[*].orbit_subframe_2`
  Present when `decoded_subframes[*].how.subframe_id == 2`.
- `decoded_subframes[*].orbit_subframe_2.iode`
- `decoded_subframes[*].orbit_subframe_2.crs`
- `decoded_subframes[*].orbit_subframe_2.delta_n`
- `decoded_subframes[*].orbit_subframe_2.m0`
- `decoded_subframes[*].orbit_subframe_2.cuc`
- `decoded_subframes[*].orbit_subframe_2.e`
- `decoded_subframes[*].orbit_subframe_2.cus`
- `decoded_subframes[*].orbit_subframe_2.sqrt_a`
- `decoded_subframes[*].orbit_subframe_2.toe_s`
- `decoded_subframes[*].orbit_subframe_3`
  Present when `decoded_subframes[*].how.subframe_id == 3`.
- `decoded_subframes[*].orbit_subframe_3.iode`
- `decoded_subframes[*].orbit_subframe_3.cic`
- `decoded_subframes[*].orbit_subframe_3.omega0`
- `decoded_subframes[*].orbit_subframe_3.cis`
- `decoded_subframes[*].orbit_subframe_3.i0`
- `decoded_subframes[*].orbit_subframe_3.crc`
- `decoded_subframes[*].orbit_subframe_3.w`
- `decoded_subframes[*].orbit_subframe_3.omegadot`
- `decoded_subframes[*].orbit_subframe_3.idot`
- `decoded_subframes[*].parity.word_count`
- `decoded_subframes[*].parity.passed_word_count`
- `decoded_subframes[*].parity.failed_word_indexes`
- `decoded_subframes[*].word_parity_ok`
- `parity_word_count`
- `parity_failed_words`
- `preamble_hits`
- `parity_pass_rate`
- `ephemerides`

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
- dataset metadata when a registered dataset is used, including raw IQ format and capture timestamp
- front-end provenance (`sample_rate_hz`, `intermediate_freq_hz`, `quantization_bits`, normalization/calibration source)

## Signal Quality Report

Raw-IQ workflows emit a standalone signal-quality artifact:
- file: `signal_quality_report.json`
- workflows: `inspect`, `acquire`, `track`, `run`
- purpose: record operator-facing RF quality for the analyzed input window

The report includes:
- sample format, sample rate, IF, and capture start time
- analyzed sample count and usable duration
- DC offset and centered front-end metrics
- I/Q power imbalance and quadrature warning state
- clipping percentage and precision-claim refusal state
- RMS, centered RMS, and estimated noise floor in dB

Raw-IQ acquisition and tracking command reports also include a `doppler_search` section that
records the effective `max_search_hz`, `bin_width_hz`, `bin_count`, and `intermediate_freq_hz`
used for that run. This field reflects the final receiver profile after any explicit CLI override.

Raw-IQ acquisition reports also include a `code_phase_search` section that records the effective
sample-domain search contract:
- `start_sample`
- `step_samples`
- `bin_count`
- `period_samples`
- `mode`

For GPS L1 C/A acquisition, the current contract is a full-code search over the entire sampled code
period: `start_sample = 0`, `step_samples = 1`, `bin_count = period_samples`, `mode = "full_code"`.
The same values are preserved per candidate in `acq.jsonl.payload.assumptions` so downstream tools
can validate the report against the retained acquisition evidence.

When an acquisition row has sub-bin Doppler refinement, `acquire_report.json.results[*]` and
`acquire_report.json.primary_results[*]` include:
- `carrier_hz`: refined carrier estimate
- `coarse_carrier_hz`: selected Doppler-bin carrier
- `doppler_refinement_hz`: refined-minus-coarse carrier offset
- `doppler_refinement_bins`: the same offset expressed in Doppler bins

## Synthetic IQ Export Bundle

The synthetic IQ export workflow emits a deterministic raw-capture bundle for a scenario:
- command: `bijux gnss export-synthetic-iq --scenario <SCENARIO>`
- files:
  - `artifacts/<scenario_id>.iq16`
  - `artifacts/<scenario_id>.sidecar.toml`
  - `artifacts/<scenario_id>.truth.json`
  - `artifacts/<scenario_id>.scenario.toml`
- truth schema: `schemas/synthetic_iq_truth.schema.json`

The truth artifact records the emitted PRNs, Doppler, code phase, C/N0, and navigation-bit state
for each synthetic satellite. It also records the exported signal amplitude plus bundle-level noise
statistics (`noise_std_per_component`, `noise_power_per_complex_sample`) so receiver-side C/N0
validation can reproduce the injected calibration conditions. Given the same scenario, seed, and
capture start time, the emitted bundle is deterministic. When navigation data modulation is
enabled, the truth bundle records both the nav-bit mode and the exact sample-aligned 20 ms bit
segments so downstream validation can distinguish deterministic bit flips from carrier events.

## Synthetic IQ Validation Report

The synthetic IQ validation workflow checks receiver measurements against injected truth:
- command: `bijux gnss validate-synthetic-iq --file <IQ16> --sidecar <SIDECAR> --truth <TRUTH>`
- files:
  - `validate_synthetic_iq_report.json`
  - `manifest.json`
- purpose: compare measured prompt-power C/N0 against the injected synthetic truth for each
  tracked satellite and compare acquisition-reported code phase against clean regenerated truth for
  each injected satellite

The bundled `synthetic_iq_cn0_reference.toml` scenario is a single-satellite calibration fixture.
That scope is intentional: it validates C/N0 recovery without overstating multi-satellite
navigation claims. Long-duration sampled-code phase stability is validated separately against
the shared 60-second phase model in signal and synthetic receiver tests.

`validate_synthetic_iq_report.json` includes two explicit validation sections:
- `validation`: truth-guided C/N0 checks
- `acquisition_code_phase_validation`: truth-guided acquisition code-phase checks
- `acquisition_doppler_validation`: truth-guided acquisition Doppler checks

Current defaults:
- `tolerance_db_hz = 4.0`
- `acquisition_code_phase_tolerance_samples = 2`
- `acquisition_doppler_tolerance_bins = 1`

The acquisition code-phase validation regenerates each satellite as a clean single-satellite signal
from the truth bundle, centers acquisition on the injected carrier, and then checks the wrapped
sample error of the reported code phase. This keeps the code-phase accuracy check focused on
acquisition phase recovery rather than Doppler-bin quantization.

The acquisition Doppler validation also regenerates each satellite as a clean single-satellite
signal, but it keeps the configured acquisition Doppler grid intact and checks the selected
IF-relative Doppler estimate against the injected truth. The report preserves both absolute
`doppler_error_hz` and normalized `doppler_error_bins` so downstream tools can reason about the
effective bin accuracy directly.

## Validation Evidence Bundle

Validation workflows emit an evidence bundle:
- file: `artifacts/validation_evidence_bundle.json`
- purpose: summarize physical signal context and numerical solution behavior in one machine-readable report
- sections:
  - `physical`: constellation counts, CN0 summary, lock-quality ratio
  - `numerical`: PDOP/residual summary, error RMS, refusal-class counts
  - `diagnostics`: advisory vs enforced-refusal partition
  - `claim_evidence_guard`: policy thresholds, pass/fail, and exact violations when claims are not evidence-supported

## Reproducibility Verification

Diagnostics workflows can verify run reproducibility artifacts:
- command: `bijux gnss diagnostics verify-repro --run-dir <RUN_DIR>`
- output: run fingerprint plus SHA-256 hashes for manifest/report/artifact bundles
- use case: auditability checks before replay, review, or archival

## Diagnostics Command Reports

Diagnostics workflows emit schema-versioned command reports under command-specific artifact directories:
- `artifacts/diagnostics_operator_map/report.json`
- `artifacts/diagnostics_workflow/report.json`
- `artifacts/diagnostics_summarize/report.json`
- `artifacts/diagnostics_explain/report.json`
- `artifacts/diagnostics_verify_repro/report.json`
- `artifacts/diagnostics_compare/report.json`
- `artifacts/diagnostics_replay_audit/report.json`
- `artifacts/diagnostics_advanced_gate/report.json`
- `artifacts/diagnostics_artifact_inventory/report.json`
- `artifacts/diagnostics_debug_plan/report.json`
- `artifacts/diagnostics_benchmark_summary/report.json`
- `artifacts/diagnostics_medium_gate/report.json`
- `artifacts/diagnostics_operator_status/report.json`
- `artifacts/diagnostics_channel_summary/report.json`
- `artifacts/diagnostics_export_bundle/report.json`
- `artifacts/diagnostics_machine_catalog/report.json`

These reports are validated against corresponding schemas in `schemas/` and include evidence-bound claim interpretation fields (`claim_level`, `interpretation`) to prevent overstatement.
