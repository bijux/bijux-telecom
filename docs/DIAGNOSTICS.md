# Diagnostics

This document defines structured diagnostic events and error codes emitted by the GNSS
pipeline. Diagnostic codes are stable identifiers for operators and tooling.

## Event Structure

All events conform to `bijux_gnss_core::DiagnosticEvent`:

- `severity`: `Info | Warning | Error`
- `code`: stable machine-readable identifier
- `message`: human-readable summary
- `context`: optional key/value metadata

## Codes

### Configuration
- `CONFIG_SCHEMA_MISMATCH`: schema version does not match supported versions.
- `CONFIG_INVALID_FIELD`: a config field failed validation.
- `CONFIG_MISSING_REQUIRED`: a required config section is missing.

### Input / IO
- `IO_DATASET_NOT_FOUND`: dataset path or ID not found.
- `IO_DATASET_READ_FAIL`: dataset could not be opened or read.
- `IO_SCHEMA_INVALID`: file did not match required schema.

### Signal / Acquisition
- `ACQ_SEARCH_EMPTY`: acquisition search produced no candidates.
- `ACQ_PEAK_BELOW_THRESHOLD`: best acquisition metric below threshold.

### Tracking
- `TRACK_LOSS_OF_LOCK`: lock detector failed for a channel.
- `TRACK_CYCLE_SLIP`: cycle slip detected for a channel.
- `TRACK_EPOCH_GAP`: tracking epochs are not contiguous or exceeded gap limit.

### Navigation / PVT
- `NAV_EPHEMERIS_GAP`: ephemeris provider has no valid data for time.
- `NAV_PVT_DIVERGED`: PVT solver failed to converge.
- `NAV_GEOMETRY_WEAK`: PDOP/condition number above configured threshold.
- `NAV_OUTLIER_REJECTED`: measurement rejected during residual gating.

### RTK / PPP
- `RTK_EPOCH_ALIGN_FAIL`: base/rover epochs could not be aligned.
- `RTK_AMB_RESET`: ambiguity reset due to slip or ref change.
- `PPP_PRODUCT_GAP`: precise products missing for epoch.
- `PPP_FILTER_RESET`: PPP filter reset due to integrity failure.
