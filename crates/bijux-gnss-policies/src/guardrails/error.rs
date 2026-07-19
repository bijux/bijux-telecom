//! Error ownership for guardrail policy evaluation.

/// Guardrail error type.
#[derive(Debug)]
pub enum GuardrailError {
    /// IO errors.
    Io(std::io::Error),
    /// Walkdir errors.
    Walk(walkdir::Error),
    /// Regex errors.
    Regex(regex::Error),
    /// Guardrail violation.
    Violation(String),
}

impl std::fmt::Display for GuardrailError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            GuardrailError::Io(err) => write!(f, "io error: {err}"),
            GuardrailError::Walk(err) => write!(f, "walkdir error: {err}"),
            GuardrailError::Regex(err) => write!(f, "regex error: {err}"),
            GuardrailError::Violation(msg) => write!(f, "{msg}"),
        }
    }
}

impl std::error::Error for GuardrailError {}

impl From<std::io::Error> for GuardrailError {
    fn from(err: std::io::Error) -> Self {
        GuardrailError::Io(err)
    }
}

impl From<walkdir::Error> for GuardrailError {
    fn from(err: walkdir::Error) -> Self {
        GuardrailError::Walk(err)
    }
}

impl From<regex::Error> for GuardrailError {
    fn from(err: regex::Error) -> Self {
        GuardrailError::Regex(err)
    }
}

/// Result type for guardrail checks.
pub type Result<T> = std::result::Result<T, GuardrailError>;
