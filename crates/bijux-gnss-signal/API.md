# bijux-gnss-signal API

Stable public API surface exposed via `crates/bijux-gnss-signal/src/api.rs`.

Signal generation + helpers
- `generate_ca_code`: C/A code generator.
- `code_sample_position_at_index`: absolute sample-index helper for chunk-stable code state.
- `samples_per_code`: helper for samples per code period.
- `sample_code`: arbitrary-rate sampled spreading-code helper.
- `sample_ca_code`: arbitrary-rate GPS L1 C/A sampled-code helper.
- `advance_code_phase_chips`: wrapped chip-phase advance helper for chunked generation.
- `advance_code_phase_seconds`: wrapped chip-phase advance helper for elapsed-time validation.
- `sample_modulated_replica_at_sample_index`: absolute sample-index replica helper.
- `wipeoff_carrier`: absolute-time-aware carrier wipeoff helper.
- `code_value_at_phase`: chip lookup at a wrapped chip phase.
- `LocalCodeModel::sample_block`: chunk-stable local-code block sampler.
- `ReplicaCodeModel::sample_block`: chunk-stable replica block sampler.

Types
- `CodeSamplePosition`: absolute sample-index code timing state.
- `Prn`: PRN identifier.
- `SamplesFrame`: sample buffer type.

Interfaces
- `SignalSource`: streaming source trait for samples.
