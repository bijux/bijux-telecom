# bijux-gnss-receiver API

Stable public API surface exposed via `crates/bijux-gnss-receiver/src/api.rs`.

Config + errors
- `ReceiverConfig`: on-disk receiver configuration schema (serde + schema validation).
- `ReceiverRuntimeConfig`: derived runtime configuration used by engines.
- `ReceiverError`: receiver pipeline error wrapper.

I/O + ports
- `FileSamples`: file-backed sample source.
- `MemorySamples`: in-memory sample source.
- `SampleSourceError`: sample-source error type.
- `SignalSource`: trait for streaming samples.
- `SampleSource`, `ArtifactSink`, `Clock`, `SystemClock`: purity-boundary traits for I/O.

Engines + helpers
- `Receiver`: top-level receiver entrypoint (constructed with `ReceiverRuntimeConfig`).
- `ReceiverEngine`: trait for a receiver pipeline runner.
- `RunArtifacts`: artifacts captured during a run.
- `AcquisitionEngine`: acquisition engine implementation.
- `TrackingEngine`: tracking engine implementation.
- `TrackingResult`, `Channel`, `ChannelState`, `ChannelEvent`, `CorrelatorOutput`.
- `observations_from_tracking`, `observations_from_tracking_results`.

Navigation (feature-gated: `nav`)
- `NavigationEngine`, `Navigation`, `EkfState`.
- RTK helpers and types: ambiguity management, differencing, baseline metrics, and RTK artifact types.
- Validation report helpers: `build_validation_report`, `ValidationReport`, `ConvergenceReport`, etc.

Re-exports
- `core`: bijux-gnss-core public API module.
- `signal`: bijux-gnss-signal public API module.
- `nav`: bijux-gnss-nav public API module (feature-gated).
