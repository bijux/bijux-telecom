//! Error types for bijux-gnss-signal.

/// Errors emitted by signal utilities.
#[derive(Debug, Clone, thiserror::Error, PartialEq, Eq)]
pub enum SignalError {
    /// Unsupported or out-of-range PRN.
    #[error("unsupported PRN {0}")]
    UnsupportedPrn(u8),
    /// Sample rate must be finite and strictly positive.
    #[error("invalid sample rate")]
    InvalidSampleRate,
    /// Code rate must be finite and strictly positive.
    #[error("invalid code rate")]
    InvalidCodeRate,
    /// Code phase must be finite.
    #[error("invalid code phase")]
    InvalidCodePhase,
    /// Elapsed duration must be finite and non-negative.
    #[error("invalid elapsed duration")]
    InvalidElapsedDuration,
    /// Code sequence must contain at least one chip.
    #[error("empty code sequence")]
    EmptyCodeSequence,
    /// Periodic correlation requires equal-length sequences.
    #[error("periodic correlation length mismatch: left={left}, right={right}")]
    CorrelationLengthMismatch {
        /// Left-hand sequence length.
        left: usize,
        /// Right-hand sequence length.
        right: usize,
    },
}
