//! Error types for infrastructure utilities.

use thiserror::Error;

/// Result type for infra operations.
pub type InfraResult<T> = Result<T, InfraError>;

/// Infra error taxonomy.
#[derive(Debug, Error)]
pub enum InfraError {
    /// IO errors.
    #[error("io error: {0}")]
    Io(#[from] std::io::Error),
    /// TOML decode errors.
    #[error("toml error: {0}")]
    Toml(#[from] toml::de::Error),
    /// TOML encode errors.
    #[error("toml error: {0}")]
    TomlSer(#[from] toml::ser::Error),
    /// JSON errors.
    #[error("json error: {0}")]
    Json(#[from] serde_json::Error),
    /// Invalid input parameters.
    #[error("invalid input: {0}")]
    InvalidInput(String),
}
