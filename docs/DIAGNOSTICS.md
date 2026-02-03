# Diagnostics

This document defines canonical diagnostic codes and how they are used.

## Principles

- Every pipeline stage emits `DiagnosticEvent` with a stable `code`.
- Codes are human-readable and stable across releases.
- Severity is one of `Info`, `Warning`, `Error`.
- CLI tooling can gate on severity (`--fail-on WARN` / `--fail-on ERROR`).

## Code Registry

| Code | Severity | Meaning | Mitigation |
| --- | --- | --- | --- |
| `GNSS_NUMERIC_ACQ_INVALID` | Error | Acquisition results contain NaN/Inf | Inspect input IQ scaling and acquisition parameters. |
| `GNSS_NUMERIC_TRACK_INVALID` | Error | Tracking epoch contains NaN/Inf | Inspect loop configuration and correlator outputs. |
| `GNSS_NUMERIC_OBS_INVALID` | Error | Observation contains NaN/Inf | Check tracking outputs and observables conversion. |
| `GNSS_NUMERIC_PVT_INVALID` | Error | Navigation solution contains NaN/Inf | Inspect measurement inputs and solver configuration. |
| `GNSS_NUMERIC_T_RX_INVALID` | Error | Receiver time tag is not finite | Check sample clock and epoch timing logic. |
| `GNSS_EPOCH_ALIGN_FAIL` | Warning | Base/rover epoch alignment failed or had gaps | Check dataset timing and alignment tolerance. |
| `NAV_EPHEMERIS_GAP` | Warning | Ephemeris coverage gap detected | Provide a dataset with complete ephemeris coverage. |
| `TRACK_LOSS_OF_LOCK` | Warning | Tracking reported loss of lock | Inspect signal strength and tracking loop parameters. |

## Stage Attribution

When available, diagnostic events include context entries:

- `stage`: pipeline stage name (e.g., `acquisition`, `tracking`, `nav`).
- `epoch`: epoch index for time-aligned summaries.

The diagnostic aggregator groups by `code` and records counts and first/last epochs.
