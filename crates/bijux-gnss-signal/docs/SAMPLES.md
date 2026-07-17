# Samples

`bijux-gnss-signal` owns reusable sample conversion and controlled storage quantization helpers.

## Sample responsibilities

The sample surface currently owns:

- `iq_i8_to_samples`
- `iq_i16_to_samples`
- `iq_f32_to_samples`
- `encode_quantized_samples`
- `quantize_samples_for_storage`

## Boundary rule

These helpers translate between sample encodings and normalized complex samples. They do not own
capture-file reading, dataset resolution, or receiver-stage scheduling.
