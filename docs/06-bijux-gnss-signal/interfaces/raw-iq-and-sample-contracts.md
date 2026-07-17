---
title: Raw IQ And Sample Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Raw IQ And Sample Contracts

`RawIqMetadata`, `IqSampleFormat`, `IqQuantization`, and the sample-conversion
helpers are public contracts shared by ingest, validation, runtime, and tests.

## Metadata Contract

`RawIqMetadata` defines:

- encoded on-disk format
- sample rate and intermediate frequency
- capture start time
- optional offset, quantization, and notes

This is the meaning contract for raw capture description. It is not a file-path
or sidecar-discovery policy.

## Conversion Contract

The published conversion helpers normalize interleaved IQ into complex samples
and support controlled quantization for storage profiles. They define how
encoded sample representations are interpreted at the signal boundary.

The owned helper family is concrete:

- `iq_i8_to_samples`
- `iq_i16_to_samples`
- `iq_f32_to_samples`
- `encode_quantized_samples`
- `quantize_samples_for_storage`

## Stability Rule

Changes here should be treated as cross-crate contract changes, even when the
implementation is mechanically simple.

## Protecting Proof

- `crates/bijux-gnss-signal/docs/RAW_IQ.md`
- `crates/bijux-gnss-signal/docs/SAMPLES.md`
- `crates/bijux-gnss-signal/src/raw_iq.rs`
- `crates/bijux-gnss-signal/src/samples.rs`
- `crates/bijux-gnss-signal/tests/integration_raw_iq_metadata.rs`
- `crates/bijux-gnss-signal/tests/integration_iq_sample_conversion.rs`
