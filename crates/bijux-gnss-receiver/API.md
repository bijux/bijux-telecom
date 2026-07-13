# bijux-gnss-receiver API

Stable public API surface exposed via `crates/bijux-gnss-receiver/src/api.rs`.

Config + errors
- `ReceiverConfig`: on-disk receiver configuration schema (serde + schema validation).
- `ReceiverPipelineConfig`: derived pipeline configuration used by engines.
- `ReceiverRuntimeConfig`: runtime options for side-effectful outputs (run id, trace dir, etc).
- `ReceiverError`: receiver pipeline error wrapper.

I/O + ports
- `FileSamples`: file-backed sample source.
- `MemorySamples`: in-memory sample source.
- `SampleSourceError`: sample-source error type.
- `SignalSource`: trait for streaming samples.
- `SampleSource`, `ArtifactSink`, `Clock`, `SystemClock`: purity-boundary traits for I/O.

Engines + helpers
- `Receiver`: top-level receiver entrypoint (constructed with `ReceiverPipelineConfig` + `ReceiverRuntimeConfig`).
- `ReceiverEngine`: trait for a receiver pipeline runner.
- `RunArtifacts`: artifacts captured during a run, including observation residual reports.
- `AcquisitionEngine`: acquisition engine implementation.
- `TrackingEngine`: tracking engine implementation.
- `TrackingResult`, `Channel`, `ChannelState`, `ChannelEvent`, `CorrelatorOutput`.
- `observations_from_tracking`, `observations_from_tracking_results`.
- `observation_artifacts_from_tracking_results`, `observation_residuals_from_tracking_results`.
- `ObservationPipelineArtifacts`, `ObservationResidualEpochReport`, `ObservationResidualSatellite`,
  `ObservationResidualValue`.

Navigation (feature-gated: `nav`)
- `NavigationEngine`, `Navigation`, `EkfState`.
- `Navigation` is a receiver-runtime adapter over the nav-owned `PositionRuntime`.
- Direct typed navigation science APIs remain in `bijux_gnss_receiver::nav`, including
  `PositionRuntime`, `PositionRuntimeConfig`, `PositionBroadcastNavigation`, and related solver
  and correction types.
- `NavigationFilter`: receiver-configured compatibility surface over the nav-owned broadcast filter.
- RTK helpers and artifact types are re-exported from `bijux-gnss-nav`.
- Validation report helpers: `build_validation_report`, `ValidationReport`, `ConvergenceReport`, etc.

Re-exports
- `core`: bijux-gnss-core public API module.
- `signal`: bijux-gnss-signal public API module.
- `nav`: bijux-gnss-nav public API module (feature-gated).
