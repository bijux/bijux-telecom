//! Artifact validation and explanation helpers.

use std::path::Path;

use crate::errors::{InfraError, InfraResult};
use bijux_gnss_core::{
    aggregate_diagnostics, ArtifactHeaderV1, ArtifactReadPolicy, ArtifactValidate, DiagnosticEvent,
    DiagnosticSeverity, ObsEpochV1, TrackEpochV1,
};
use bijux_gnss_nav::GpsEphemerisV1;

/// Result of validating an artifact.
#[derive(Debug, Clone)]
pub struct ArtifactValidationResult {
    /// Artifact kind.
    pub kind: String,
    /// Diagnostics produced by validation.
    pub diagnostics: Vec<DiagnosticEvent>,
}

/// Result of explaining an artifact.
#[derive(Debug, Clone)]
pub struct ArtifactExplainResult {
    /// Artifact kind.
    pub kind: String,
    /// Artifact header.
    pub header: ArtifactHeaderV1,
    /// Entry count (if applicable).
    pub entries: usize,
    /// Diagnostics summary.
    pub diagnostics_total: usize,
    /// Diagnostics error count.
    pub diagnostics_error: usize,
    /// Diagnostics warning count.
    pub diagnostics_warn: usize,
}

/// Validate an artifact file and return diagnostics.
pub fn artifact_validate(
    path: &Path,
    kind: Option<&str>,
    strict: bool,
) -> InfraResult<ArtifactValidationResult> {
    let data = std::fs::read_to_string(path)?;
    if strict && data.trim().is_empty() {
        return Err(InfraError::InvalidInput(format!(
            "artifact is empty: {}",
            path.display()
        )));
    }
    let kind = kind
        .map(|k| k.to_lowercase())
        .or_else(|| detect_kind_from_path(path))
        .unwrap_or_else(|| "unknown".to_string());

    let diagnostics = match kind.as_str() {
        "obs" => validate_obs_artifact(&data)?,
        "track" => validate_track_artifact(&data)?,
        "acq" => validate_acq_artifact(&data)?,
        "eph" | "ephemeris" => validate_ephemeris_artifact(&data)?,
        "pvt" | "nav" => validate_pvt_artifact(&data)?,
        "ppp" => validate_ppp_artifact(&data)?,
        "rtk" => validate_rtk_artifact(&data)?,
        _ => {
            return Err(InfraError::InvalidInput(
                "unknown artifact kind; pass --kind".to_string(),
            ))
        }
    };

    Ok(ArtifactValidationResult { kind, diagnostics })
}

/// Explain an artifact file and return header + stats.
pub fn artifact_explain(path: &Path) -> InfraResult<ArtifactExplainResult> {
    let data = std::fs::read_to_string(path)?;
    let mut header: Option<ArtifactHeaderV1> = None;
    let mut kind = detect_kind_from_path(path).unwrap_or_else(|| "unknown".to_string());
    let mut entries = 0usize;

    if data.trim_start().starts_with('{') && !data.contains('\n') {
        if let Ok(wrapped) = serde_json::from_str::<GpsEphemerisV1>(&data) {
            header = Some(wrapped.header);
            kind = "ephemeris".to_string();
            entries = wrapped.payload.len();
        } else if let Ok(wrapped) = serde_json::from_str::<serde_json::Value>(&data) {
            if let Some(value) = wrapped.get("header") {
                header = Some(serde_json::from_value(value.clone())?);
            }
        }
    } else {
        for line in data.lines() {
            if line.trim().is_empty() {
                continue;
            }
            if header.is_none() {
                let value: serde_json::Value = serde_json::from_str(line)?;
                if let Some(value) = value.get("header") {
                    header = Some(serde_json::from_value(value.clone())?);
                }
            }
            entries += 1;
        }
    }

    let header = header
        .ok_or_else(|| InfraError::InvalidInput("artifact header not found".to_string()))?;
    let diagnostics = artifact_validate(path, Some(&kind), false)?.diagnostics;
    let summary = aggregate_diagnostics(&diagnostics);
    let mut error_count = 0usize;
    let mut warn_count = 0usize;
    for entry in &summary.entries {
        match entry.severity {
            DiagnosticSeverity::Error => error_count += entry.count,
            DiagnosticSeverity::Warning => warn_count += entry.count,
            DiagnosticSeverity::Info => {}
        }
    }

    Ok(ArtifactExplainResult {
        kind,
        header,
        entries,
        diagnostics_total: summary.total,
        diagnostics_error: error_count,
        diagnostics_warn: warn_count,
    })
}

fn validate_obs_artifact(data: &str) -> InfraResult<Vec<DiagnosticEvent>> {
    let mut epochs = Vec::new();
    let mut events = Vec::new();
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let wrapped: ObsEpochV1 = serde_json::from_str(line)?;
        if !ArtifactReadPolicy::is_supported(wrapped.header.schema_version) {
            return Err(InfraError::InvalidInput(format!(
                "unsupported obs schema_version {}",
                wrapped.header.schema_version
            )));
        }
        events.extend(wrapped.validate());
        epochs.push(wrapped.payload);
    }
    if let Err(err) = bijux_gnss_core::validate_obs_epochs(&epochs) {
        events.push(DiagnosticEvent::new(
            DiagnosticSeverity::Error,
            "GNSS_OBS_VALIDATE_FAILED",
            format!("obs epoch validation failed: {err}"),
        ));
    }
    Ok(events)
}

fn validate_track_artifact(data: &str) -> InfraResult<Vec<DiagnosticEvent>> {
    let mut events = Vec::new();
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let wrapped: TrackEpochV1 = serde_json::from_str(line)?;
        if !ArtifactReadPolicy::is_supported(wrapped.header.schema_version) {
            return Err(InfraError::InvalidInput(format!(
                "unsupported track schema_version {}",
                wrapped.header.schema_version
            )));
        }
        events.extend(wrapped.validate());
    }
    Ok(events)
}

fn validate_acq_artifact(data: &str) -> InfraResult<Vec<DiagnosticEvent>> {
    let mut events = Vec::new();
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let wrapped: bijux_gnss_core::AcqResultV1 = serde_json::from_str(line)?;
        if !ArtifactReadPolicy::is_supported(wrapped.header.schema_version) {
            return Err(InfraError::InvalidInput(format!(
                "unsupported acq schema_version {}",
                wrapped.header.schema_version
            )));
        }
        events.extend(wrapped.validate());
    }
    Ok(events)
}

fn validate_ephemeris_artifact(data: &str) -> InfraResult<Vec<DiagnosticEvent>> {
    let wrapped: GpsEphemerisV1 = serde_json::from_str(data)?;
    if !ArtifactReadPolicy::is_supported(wrapped.header.schema_version) {
        return Err(InfraError::InvalidInput(format!(
            "unsupported ephemeris schema_version {}",
            wrapped.header.schema_version
        )));
    }
    Ok(Vec::new())
}

fn validate_pvt_artifact(data: &str) -> InfraResult<Vec<DiagnosticEvent>> {
    let mut events = Vec::new();
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let wrapped: bijux_gnss_core::NavSolutionEpochV1 = serde_json::from_str(line)?;
        if !ArtifactReadPolicy::is_supported(wrapped.header.schema_version) {
            return Err(InfraError::InvalidInput(format!(
                "unsupported pvt schema_version {}",
                wrapped.header.schema_version
            )));
        }
        events.extend(wrapped.validate());
    }
    Ok(events)
}

fn validate_ppp_artifact(data: &str) -> InfraResult<Vec<DiagnosticEvent>> {
    let mut events = Vec::new();
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let wrapped: bijux_gnss_nav::PppEpochV1 = serde_json::from_str(line)?;
        if !ArtifactReadPolicy::is_supported(wrapped.header.schema_version) {
            return Err(InfraError::InvalidInput(format!(
                "unsupported ppp schema_version {}",
                wrapped.header.schema_version
            )));
        }
        events.extend(wrapped.validate());
    }
    Ok(events)
}

fn validate_rtk_artifact(data: &str) -> InfraResult<Vec<DiagnosticEvent>> {
    let mut events = Vec::new();
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let value: serde_json::Value = serde_json::from_str(line)?;
        let header: ArtifactHeaderV1 = serde_json::from_value(
            value
                .get("header")
                .cloned()
                .ok_or_else(|| InfraError::InvalidInput("missing header".to_string()))?,
        )?;
        if !ArtifactReadPolicy::is_supported(header.schema_version) {
            return Err(InfraError::InvalidInput(format!(
                "unsupported rtk schema_version {}",
                header.schema_version
            )));
        }
        if let Ok(wrapped) = serde_json::from_value::<
            bijux_gnss_core::rtk::RtkSdEpochV1<bijux_gnss_receiver::rtk::SdObservation>,
        >(value.clone())
        {
            events.extend(wrapped.validate());
            continue;
        }
        if let Ok(wrapped) = serde_json::from_value::<
            bijux_gnss_core::rtk::RtkDdEpochV1<bijux_gnss_receiver::rtk::DdObservation>,
        >(value.clone())
        {
            events.extend(wrapped.validate());
            continue;
        }
        if let Ok(wrapped) = serde_json::from_value::<
            bijux_gnss_core::rtk::RtkBaselineEpochV1<
                bijux_gnss_receiver::rtk::BaselineSolution,
            >,
        >(value.clone())
        {
            events.extend(wrapped.validate());
            continue;
        }
        if let Ok(wrapped) = serde_json::from_value::<
            bijux_gnss_core::rtk::RtkBaselineQualityV1<
                bijux_gnss_receiver::rtk::RtkBaselineQuality,
            >,
        >(value.clone())
        {
            events.extend(wrapped.validate());
            continue;
        }
        if let Ok(wrapped) = serde_json::from_value::<
            bijux_gnss_core::rtk::RtkPrecisionV1<
                bijux_gnss_receiver::rtk::RtkPrecision,
            >,
        >(value.clone())
        {
            events.extend(wrapped.validate());
            continue;
        }
        if let Ok(wrapped) = serde_json::from_value::<
            bijux_gnss_core::rtk::RtkFixAuditV1<
                bijux_gnss_receiver::rtk::FixAuditEvent,
            >,
        >(value)
        {
            events.extend(wrapped.validate());
            continue;
        }
    }
    Ok(events)
}

fn detect_kind_from_path(path: &Path) -> Option<String> {
    let name = path.file_name()?.to_string_lossy().to_lowercase();
    if name.contains("obs") {
        return Some("obs".to_string());
    }
    if name.contains("track") {
        return Some("track".to_string());
    }
    if name.contains("acq") {
        return Some("acq".to_string());
    }
    if name.contains("eph") {
        return Some("eph".to_string());
    }
    if name.contains("pvt") || name.contains("nav") {
        return Some("pvt".to_string());
    }
    if name.contains("ppp") {
        return Some("ppp".to_string());
    }
    if name.contains("rtk") {
        return Some("rtk".to_string());
    }
    None
}
