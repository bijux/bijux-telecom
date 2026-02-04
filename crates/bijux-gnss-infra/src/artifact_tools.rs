//! Artifact validation and explanation helpers.

use std::path::Path;

use bijux_gnss_receiver::api::core::{
    aggregate_diagnostics, ArtifactHeaderV1, ArtifactReadPolicy, ArtifactValidate, DiagnosticEvent,
    DiagnosticSeverity, InputError, ObsEpochV1,
};

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
) -> Result<ArtifactValidationResult, InputError> {
    let data = std::fs::read_to_string(path).map_err(map_err)?;
    if strict && data.trim().is_empty() {
        return Err(InputError {
            message: format!("artifact is empty: {}", path.display()),
        });
    }
    let kind = kind
        .map(|k| k.to_lowercase())
        .or_else(|| detect_kind_from_path(path))
        .unwrap_or_else(|| "unknown".to_string());

    let diagnostics = match kind.as_str() {
        "obs" => validate_obs_artifact(&data)?,
        "unknown" => {
            return Err(InputError {
                message: "unsupported artifact type".to_string(),
            })
        }
        _ => {
            return Err(InputError {
                message: "unsupported artifact type".to_string(),
            })
        }
    };

    Ok(ArtifactValidationResult { kind, diagnostics })
}

/// Explain an artifact file and return header + stats.
pub fn artifact_explain(path: &Path) -> Result<ArtifactExplainResult, InputError> {
    let data = std::fs::read_to_string(path).map_err(map_err)?;
    let mut header: Option<ArtifactHeaderV1> = None;
    let kind = detect_kind_from_path(path).unwrap_or_else(|| "unknown".to_string());
    let mut entries = 0usize;

    if kind != "obs" {
        return Err(InputError {
            message: "unsupported artifact type".to_string(),
        });
    }

    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let wrapped: ObsEpochV1 = serde_json::from_str(line).map_err(map_err)?;
        if header.is_none() {
            header = Some(wrapped.header.clone());
        }
        entries += 1;
    }

    let header = header.ok_or_else(|| InputError {
        message: "artifact header not found".to_string(),
    })?;
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

fn validate_obs_artifact(data: &str) -> Result<Vec<DiagnosticEvent>, InputError> {
    let mut epochs = Vec::new();
    let mut events = Vec::new();
    let mut last_t_rx_s: Option<bijux_gnss_receiver::api::core::Seconds> = None;
    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        let wrapped: ObsEpochV1 = serde_json::from_str(line).map_err(map_err)?;
        if wrapped.header.schema_version != ArtifactReadPolicy::LATEST {
            return Err(InputError {
                message: format!(
                    "unsupported obs schema_version {}",
                    wrapped.header.schema_version
                ),
            });
        }
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
    if let Err(err) = bijux_gnss_receiver::api::core::validate_obs_epochs(&epochs) {
        events.push(DiagnosticEvent::new(
            DiagnosticSeverity::Error,
            "GNSS_OBS_VALIDATE_FAILED",
            format!("obs epoch validation failed: {err}"),
        ));
    }
    Ok(events)
}

fn map_err(err: impl std::fmt::Display) -> InputError {
    InputError {
        message: err.to_string(),
    }
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
