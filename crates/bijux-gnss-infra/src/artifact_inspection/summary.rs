//! Artifact explanation helpers.

use std::path::Path;

use bijux_gnss_receiver::api::core::{
    aggregate_diagnostics, AcqResultV1, ArtifactHeaderV1, DiagnosticSeverity, InputError,
    NavSolutionEpochV1, ObsEpochV1, TrackEpochV1,
};

use super::artifact_type::ArtifactKind;
use super::{artifact_validate, ArtifactExplainResult};

/// Explain an artifact file and return header and diagnostics summary information.
pub fn artifact_explain(path: &Path) -> Result<ArtifactExplainResult, InputError> {
    let data = std::fs::read_to_string(path).map_err(map_err)?;
    let kind = ArtifactKind::from_path(path)
        .ok_or_else(|| InputError { message: "unsupported artifact type".to_string() })?;
    let mut header: Option<ArtifactHeaderV1> = None;
    let mut entries = 0usize;

    for line in data.lines() {
        if line.trim().is_empty() {
            continue;
        }
        if header.is_none() {
            header = Some(match kind {
                ArtifactKind::Acq => {
                    serde_json::from_str::<AcqResultV1>(line).map_err(map_err)?.header
                }
                ArtifactKind::Track => {
                    serde_json::from_str::<TrackEpochV1>(line).map_err(map_err)?.header
                }
                ArtifactKind::Obs => {
                    serde_json::from_str::<ObsEpochV1>(line).map_err(map_err)?.header
                }
                ArtifactKind::Pvt => {
                    serde_json::from_str::<NavSolutionEpochV1>(line).map_err(map_err)?.header
                }
            });
        }
        entries += 1;
    }

    let header =
        header.ok_or_else(|| InputError { message: "artifact header not found".to_string() })?;
    let diagnostics = artifact_validate(path, Some(kind.as_str()), false)?.diagnostics;
    let summary = aggregate_diagnostics(&diagnostics);
    let mut diagnostics_error = 0usize;
    let mut diagnostics_warn = 0usize;
    for entry in &summary.entries {
        match entry.severity {
            DiagnosticSeverity::Error => diagnostics_error += entry.count,
            DiagnosticSeverity::Warning => diagnostics_warn += entry.count,
            DiagnosticSeverity::Info => {}
        }
    }

    Ok(ArtifactExplainResult {
        kind: kind.as_str().to_string(),
        header,
        entries,
        diagnostics_total: summary.total,
        diagnostics_error,
        diagnostics_warn,
    })
}

fn map_err(err: impl std::fmt::Display) -> InputError {
    InputError { message: err.to_string() }
}
