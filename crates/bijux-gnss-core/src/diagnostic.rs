#![allow(missing_docs)]
use serde::{Deserialize, Serialize};

/// Severity of a diagnostic event.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum DiagnosticSeverity {
    /// Informational event.
    Info,
    /// Warning-level event.
    Warning,
    /// Error-level event.
    Error,
}

/// Structured diagnostic event emitted by pipeline stages.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DiagnosticEvent {
    /// Severity of the event.
    pub severity: DiagnosticSeverity,
    /// Stable machine-readable code.
    pub code: String,
    /// Human-readable message.
    pub message: String,
    /// Optional structured context.
    pub context: Vec<(String, String)>,
}

impl DiagnosticEvent {
    /// Create a new diagnostic event.
    pub fn new(
        severity: DiagnosticSeverity,
        code: impl Into<String>,
        message: impl Into<String>,
    ) -> Self {
        Self {
            severity,
            code: code.into(),
            message: message.into(),
            context: Vec::new(),
        }
    }

    /// Attach a context key/value pair.
    pub fn with_context(mut self, key: impl Into<String>, value: impl Into<String>) -> Self {
        self.context.push((key.into(), value.into()));
        self
    }
}
