//! Error types for bijux-gnss-signal.

/// Errors emitted by signal utilities.
#[derive(Debug, Clone, thiserror::Error, PartialEq, Eq)]
pub enum SignalError {
    /// Unsupported or out-of-range PRN.
    #[error("unsupported PRN {0}")]
    UnsupportedPrn(u8),
    /// Periodic correlation requires equal-length sequences.
    #[error("periodic correlation length mismatch: left={left}, right={right}")]
    CorrelationLengthMismatch {
        /// Left-hand sequence length.
        left: usize,
        /// Right-hand sequence length.
        right: usize,
    },
}
