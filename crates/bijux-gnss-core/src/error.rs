#![allow(missing_docs)]
use thiserror::Error;

/// Invalid or missing input data.
#[derive(Debug, Error, Clone)]
#[error("{message}")]
pub struct InputError {
    /// Human-readable error message.
    pub message: String,
}

/// Invalid configuration or schema mismatch.
#[derive(Debug, Error, Clone)]
#[error("{message}")]
pub struct ConfigError {
    /// Human-readable error message.
    pub message: String,
}

/// Failed I/O operation.
#[derive(Debug, Error, Clone)]
#[error("{message}")]
pub struct IoError {
    /// Human-readable error message.
    pub message: String,
}

/// Failed to parse an input format.
#[derive(Debug, Error, Clone)]
#[error("{message}")]
pub struct ParseError {
    /// Human-readable error message.
    pub message: String,
}

/// Signal processing error.
#[derive(Debug, Error, Clone)]
#[error("{message}")]
pub struct SignalError {
    /// Human-readable error message.
    pub message: String,
}

/// Acquisition error.
#[derive(Debug, Error, Clone)]
#[error("{message}")]
pub struct AcqError {
    /// Human-readable error message.
    pub message: String,
}

/// Tracking error.
#[derive(Debug, Error, Clone)]
#[error("{message}")]
pub struct TrackError {
    /// Human-readable error message.
    pub message: String,
}

/// Navigation error.
#[derive(Debug, Error, Clone)]
#[error("{message}")]
pub struct NavError {
    /// Human-readable error message.
    pub message: String,
}

/// Invariant violation.
#[derive(Debug, Error, Clone)]
#[error("{message}")]
pub struct InvariantError {
    /// Human-readable error message.
    pub message: String,
}
