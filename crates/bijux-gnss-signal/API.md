# bijux-gnss-signal API

Stable public API surface exposed via `crates/bijux-gnss-signal/src/api.rs`.

Signal generation + helpers
- `generate_ca_code`: C/A code generator.
- `samples_per_code`: helper for samples per code period.

Types
- `Prn`: PRN identifier.
- `SamplesFrame`: sample buffer type.

Interfaces
- `SignalSource`: streaming source trait for samples.
