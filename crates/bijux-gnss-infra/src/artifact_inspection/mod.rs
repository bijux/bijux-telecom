//! Artifact validation and explanation helpers.

mod explanation;
mod kind;
mod validation;

#[cfg(test)]
mod tests;

use bijux_gnss_receiver::api::core::{ArtifactHeaderV1, DiagnosticEvent, InputError};

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

/// Explain an artifact file and summarize its diagnostics.
pub fn artifact_explain(path: &std::path::Path) -> Result<ArtifactExplainResult, InputError> {
    explanation::artifact_explain(path)
}

/// Validate an artifact file and return diagnostics.
pub fn artifact_validate(
    path: &std::path::Path,
    kind: Option<&str>,
    strict: bool,
) -> Result<ArtifactValidationResult, InputError> {
    validation::artifact_validate(path, kind, strict)
}
