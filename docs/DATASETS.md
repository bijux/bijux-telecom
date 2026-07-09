# Datasets

Datasets are registered in `datasets/registry.toml` and referenced by dataset id in config or CLI.

## Adding a Dataset
1. Place IQ data and sidecar metadata under `datasets/<name>/`.
2. Add a registry entry with:
   - `id`
   - `path`
   - `format`
   - `sample_rate_hz`
   - `intermediate_freq_hz`
   - `capture_start_utc`
   - `sidecar` when extra ingest metadata such as `offset_bytes` or notes is needed
3. Add any expected sanity outputs for reproducible tests.

## Metadata Requirements
Sidecars must specify:
- `format`
- `sample_rate_hz`
- `intermediate_freq_hz`
- `capture_start_utc`

Optional sidecar fields include:
- `offset_bytes`
- `quantization_bits`
- `notes`

See `datasets/registry.toml` for a complete example.
