# Datasets

Datasets are registered in `datasets/registry.toml` and referenced by dataset id in config or CLI.

## Adding a Dataset
1. Place IQ data and sidecar metadata under `datasets/<name>/`.
2. Add a registry entry with:
   - `id`
   - `path`
   - `sample_rate_hz`
   - `if_hz`
   - `quantization`
   - `signals` (e.g., `gps_l1ca`)
3. Add any expected sanity outputs for reproducible tests.

## Metadata Requirements
Sidecars must specify:
- sampling rate
- intermediate frequency
- quantization format
- start time (if known)

See `datasets/registry.toml` for a complete example.
