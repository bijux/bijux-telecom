# GPS L1 C/A

This document captures the repository-owned reference points for the GPS L1 C/A signal surface.

## Signal Shape
- Carrier: L1 at 1575.42 MHz
- Code family: C/A
- Chip rate: 1.023 MHz
- Code length: 1023 chips
- Code period: 1 ms

## Repository Ownership
- Code generation and correlation helpers live in `crates/bijux-gnss-signal/src/codes/ca_code.rs`.
- Signal sampling and replica construction live in `crates/bijux-gnss-signal/src/dsp/`.
- Receiver acquisition and tracking defaults live in configuration profiles under `configs/`.
- Navigation decoding and orbit usage for GPS L1 C/A live in `crates/bijux-gnss-nav/`.

## Related Surfaces
- `docs/CONCEPTS.md`
- `docs/EXTENDING_SIGNAL.md`
- `docs/EXTENDING_NAV.md`
