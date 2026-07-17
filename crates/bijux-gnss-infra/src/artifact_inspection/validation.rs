//! Artifact validation helpers.

use std::path::Path;

use bijux_gnss_receiver::api::core::{
    AcqResultV1, ArtifactHeaderV1, ArtifactPayloadValidate, ArtifactReadPolicy, ArtifactValidate,
    DiagnosticEvent, DiagnosticSeverity, InputError, NavSolutionEpochV1, ObsEpochV1, TrackEpochV1,
};

use super::artifact_type::ArtifactKind;
use super::ArtifactValidationResult;

/// Validate an artifact file and return diagnostics.
pub fn artifact_validate(
    path: &Path,
    kind: Option<&str>,
    strict: bool,
) -> Result<ArtifactValidationResult, InputError> {
    let data = std::fs::read_to_string(path).map_err(map_err)?;
    if strict && data.trim().is_empty() {
        return Err(InputError { message: format!("artifact is empty: {}", path.display()) });
    }
    let kind = ArtifactKind::detect(kind, path)
        .ok_or_else(|| InputError { message: "unsupported artifact type".to_string() })?;
    let diagnostics = match kind {
        ArtifactKind::Acq => validate_acq_artifact(&data)?,
        ArtifactKind::Track => validate_track_artifact(&data)?,
        ArtifactKind::Obs => validate_obs_artifact(&data)?,
        ArtifactKind::Pvt => validate_nav_artifact(&data)?,
    };

    Ok(ArtifactValidationResult { kind: kind.as_str().to_string(), diagnostics })
}

fn validate_obs_artifact(data: &str) -> Result<Vec<DiagnosticEvent>, InputError> {
    let mut epochs = Vec::new();
    let mut events = Vec::new();
    let mut last_t_rx_s: Option<bijux_gnss_receiver::api::core::Seconds> = None;
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let wrapped: ObsEpochV1 = serde_json::from_str(line).map_err(map_err)?;
        validate_schema_version(wrapped.header.schema_version, "obs")?;
        if let Some(prev) = last_t_rx_s {
            if wrapped.payload.t_rx_s.0 < prev.0 {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "GNSS_OBS_TIME_NON_MONOTONIC",
                    "obs t_rx_s is not monotonic",
                ));
            }
        }
        last_t_rx_s = Some(wrapped.payload.t_rx_s);
        events.extend(wrapped.validate());
        epochs.push(wrapped.payload);
    }
    if let Err(err) = bijux_gnss_receiver::api::signal::validate_obs_epochs(&epochs) {
        events.push(DiagnosticEvent::new(
            DiagnosticSeverity::Error,
            "GNSS_OBS_VALIDATE_FAILED",
            format!("obs epoch validation failed: {err}"),
        ));
    }
    Ok(events)
}

fn validate_acq_artifact(data: &str) -> Result<Vec<DiagnosticEvent>, InputError> {
    validate_wrapped_payloads::<AcqResultV1>(data, "acq")
}

fn validate_track_artifact(data: &str) -> Result<Vec<DiagnosticEvent>, InputError> {
    let mut last_sample_index: Option<u64> = None;
    let mut events = Vec::new();
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let wrapped: TrackEpochV1 = serde_json::from_str(line).map_err(map_err)?;
        validate_schema_version(wrapped.header.schema_version, "track")?;
        if let Some(prev) = last_sample_index {
            if wrapped.payload.sample_index < prev {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "GNSS_TRACK_SAMPLE_NON_MONOTONIC",
                    "track sample index is not monotonic",
                ));
            }
        }
        last_sample_index = Some(wrapped.payload.sample_index);
        events.extend(wrapped.payload.validate_payload());
    }
    Ok(events)
}

fn validate_nav_artifact(data: &str) -> Result<Vec<DiagnosticEvent>, InputError> {
    let mut last_t_rx_s: Option<bijux_gnss_receiver::api::core::Seconds> = None;
    let mut events = Vec::new();
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let wrapped: NavSolutionEpochV1 = serde_json::from_str(line).map_err(map_err)?;
        validate_schema_version(wrapped.header.schema_version, "pvt")?;
        if let Some(prev) = last_t_rx_s {
            if wrapped.payload.t_rx_s.0 < prev.0 {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "GNSS_NAV_TIME_NON_MONOTONIC",
                    "nav t_rx_s is not monotonic",
                ));
            }
        }
        last_t_rx_s = Some(wrapped.payload.t_rx_s);
        events.extend(wrapped.payload.validate_payload());
    }
    Ok(events)
}

fn validate_wrapped_payloads<T>(data: &str, kind: &str) -> Result<Vec<DiagnosticEvent>, InputError>
where
    T: serde::de::DeserializeOwned + WrappedPayloadValidate,
{
    let mut events = Vec::new();
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let wrapped: T = serde_json::from_str(line).map_err(map_err)?;
        validate_schema_version(wrapped.header().schema_version, kind)?;
        events.extend(wrapped.validate_payload());
    }
    Ok(events)
}

fn validate_schema_version(schema_version: u32, kind: &str) -> Result<(), InputError> {
    if schema_version != ArtifactReadPolicy::LATEST {
        return Err(InputError {
            message: format!("unsupported {kind} schema_version {schema_version}"),
        });
    }
    Ok(())
}

trait WrappedPayloadValidate {
    fn header(&self) -> &ArtifactHeaderV1;
    fn validate_payload(&self) -> Vec<DiagnosticEvent>;
}

impl WrappedPayloadValidate for AcqResultV1 {
    fn header(&self) -> &ArtifactHeaderV1 {
        &self.header
    }

    fn validate_payload(&self) -> Vec<DiagnosticEvent> {
        self.payload.validate_payload()
    }
}

impl WrappedPayloadValidate for TrackEpochV1 {
    fn header(&self) -> &ArtifactHeaderV1 {
        &self.header
    }

    fn validate_payload(&self) -> Vec<DiagnosticEvent> {
        self.payload.validate_payload()
    }
}

impl WrappedPayloadValidate for NavSolutionEpochV1 {
    fn header(&self) -> &ArtifactHeaderV1 {
        &self.header
    }

    fn validate_payload(&self) -> Vec<DiagnosticEvent> {
        self.payload.validate_payload()
    }
}

fn map_err(err: impl std::fmt::Display) -> InputError {
    InputError { message: err.to_string() }
}
