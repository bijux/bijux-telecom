# Public API

`bijux-gnss-receiver` publishes one curated downstream surface through `bijux_gnss_receiver::api`.

## Receiver-owned API families

### Runtime and configuration

The API exposes:
- receiver configuration and derived pipeline configuration
- runtime configuration, trace, metric, and sink interfaces
- clock, sample-source, and artifact-sink boundary traits

### Stage engines

The API exposes the high-level stage entrypoints:
- `AcquisitionEngine`
- `TrackingEngine`
- observation construction and carrier-smoothed validation helpers
- `Navigation` and `NavigationFilter` when `nav` is enabled
- the top-level `Receiver` and `ReceiverEngine` boundary

### Runtime artifacts and validation

The API exposes:
- `RunArtifacts`
- observation residual and measurement-quality reports
- reference-validation, covariance-realism, and validation-report helpers under `nav`

## Re-export families

`api.rs` also re-exports curated surfaces from:
- `bijux-gnss-core` as `core`
- `bijux-gnss-signal` as `signal`
- `bijux-gnss-nav` as `nav` when enabled

Those re-exports are for integration convenience. They do not shift ownership of those families
into the receiver crate.
