# Conventions

This document defines the scientific and engineering conventions used across `bijux-gnss`.

## Sign Conventions
- **Doppler**: positive Doppler means increasing carrier frequency at the receiver.
  Receiver outputs report Doppler relative to the configured IF; absolute in-band carrier
  frequency remains an internal acquisition/tracking quantity. Acquisition searches carrier bins
  at `intermediate_freq_hz + doppler_hz`, so zero-IF and high-IF captures share the same
  IF-relative Doppler convention.
- **Carrier phase**: phase is reported in cycles; increasing phase corresponds to increasing range.

## Reference Frames
- **ECEF (WGS‑84)**: default Earth-centered Earth-fixed coordinates.
- **ENU**: local tangent frame for baselines; derived from ECEF using WGS‑84.

## Time Scales
- **GPS Time**: continuous, no leap seconds.
- **UTC**: includes leap seconds; GPS ↔ UTC handled via leap second table.

## Artifact Units (Public Fields)
- Distances: **meters** (`Meters`)
- Time tags: **seconds** (`Seconds`) or GPS time structs
- Doppler: **hertz** (`Hertz`)
- Carrier phase: **cycles** (`Cycles`)
- Code phase: **chips** (`Chips`)

See `docs/ARTIFACTS.md` for artifact schemas and invariants.
