# Public API

`bijux-gnss-signal` publishes one curated downstream surface through `bijux_gnss_signal::api`.

## Public API families

### Catalog and physical helpers

The catalog family exposes signal lookup, wavelength conversion, ionosphere-scaling helpers, and
default acquisition signal selection.

### Code families

The API exposes dedicated helpers for:
- GPS L1 C/A, L2C, and L5
- Galileo E1 and E5
- BeiDou B1I, B2I, and D1
- GLONASS L1

These exports include code generators, samplers, assignments, and family constants.

### DSP families

The DSP family exposes:
- front-end FIR and transfer-response helpers
- local-code and code-phase helpers
- NCO primitives
- quality and spectrum analysis helpers
- replica-generation helpers
- tracking-loop primitives and uncertainty helpers

### Sample and validation families

The API exposes:
- raw-IQ metadata contracts
- sample conversion helpers
- dual-frequency and inter-frequency observation-validation reports
- lightweight stream/source/sink traits used by tests and higher-level crates

## Public trait surface

The intentionally public traits are:
- `SignalSource`
- `SampleSource`
- `Correlator`
- `SampleSink`

These traits are the reusable integration seam. Receiver orchestration still belongs in
`bijux-gnss-receiver`.

Their role and boundary are documented in the [trait guide](TRAITS.md).
