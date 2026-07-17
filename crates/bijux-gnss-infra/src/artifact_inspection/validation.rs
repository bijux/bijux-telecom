//! Artifact validation helpers.

use std::path::Path;

use bijux_gnss_receiver::api::core::{
    AcqResultV1, ArtifactHeaderV1, ArtifactPayloadValidate, DiagnosticEvent, InputError,
    NavSolutionEpochV1, TrackEpochV1,
};

use super::artifact_type::ArtifactKind;
use super::ArtifactValidationResult;

mod acquisition;
mod observation;
mod navigation;
mod schema_policy;
mod tracking;

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
        ArtifactKind::Acq => acquisition::validate_acq_artifact(&data)?,
        ArtifactKind::Track => tracking::validate_track_artifact(&data)?,
        ArtifactKind::Obs => observation::validate_obs_artifact(&data)?,
        ArtifactKind::Pvt => navigation::validate_nav_artifact(&data)?,
    };

    Ok(ArtifactValidationResult { kind: kind.as_str().to_string(), diagnostics })
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
        schema_policy::validate_schema_version(wrapped.header().schema_version, kind)?;
        events.extend(wrapped.validate_payload());
    }
    Ok(events)
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
