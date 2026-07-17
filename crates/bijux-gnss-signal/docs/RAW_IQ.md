# Raw IQ

`bijux-gnss-signal` owns the typed raw-IQ metadata contract shared across ingest, runtime, and
validation surfaces.

## Raw-IQ responsibilities

The raw-IQ surface currently owns:

- `IqSampleFormat`
- `IqQuantization`
- `RawIqMetadata`

## Boundary rule

This crate owns the metadata shape and quantization vocabulary. Repository sidecar discovery and
file-path policy belong in infrastructure and CLI layers.
