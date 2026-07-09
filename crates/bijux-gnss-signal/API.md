# bijux-gnss-signal API

Stable public API surface exposed via `crates/bijux-gnss-signal/src/api.rs`.

Signal generation + helpers
- `generate_ca_code`: C/A code generator.
- `samples_per_code`: helper for samples per code period.
- `sample_code`: arbitrary-rate sampled spreading-code helper.
- `sample_ca_code`: arbitrary-rate GPS L1 C/A sampled-code helper.
- `advance_code_phase_chips`: wrapped chip-phase advance helper for chunked generation.
- `advance_code_phase_seconds`: wrapped chip-phase advance helper for elapsed-time validation.
- `wipeoff_carrier`: absolute-time-aware carrier wipeoff helper.
- `code_value_at_phase`: chip lookup at a wrapped chip phase.

Types
- `Prn`: PRN identifier.
- `SamplesFrame`: sample buffer type.

Interfaces
- `SignalSource`: streaming source trait for samples.
