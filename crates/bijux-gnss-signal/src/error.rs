//! Error types for bijux-gnss-signal.

use bijux_gnss_core::api::{Constellation, SatId, SignalBand, SignalCode};

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
    /// Carrier frequency must be finite.
    #[error("invalid carrier frequency")]
    InvalidCarrierFrequency,
    /// GLONASS acquisition and synthesis require a frequency channel.
    #[error("missing GLONASS frequency channel for {0:?}")]
    MissingGlonassFrequencyChannel(SatId),
    /// Signal metadata exists but does not expose a usable default component.
    #[error("unsupported signal definition for {constellation:?} {signal_band:?} {signal_code:?}")]
    UnsupportedSignalDefinition {
        /// Constellation carried by the incomplete signal definition.
        constellation: Constellation,
        /// Signal band carried by the incomplete signal definition.
        signal_band: SignalBand,
        /// Signal code carried by the incomplete signal definition.
        signal_code: SignalCode,
    },
    /// Code phase must be finite.
    #[error("invalid code phase")]
    InvalidCodePhase,
    /// Navigation symbol streams must contain at least one symbol.
    #[error("empty navigation symbol stream")]
    EmptyNavigationSymbolStream,
    /// Navigation symbols must use bipolar levels.
    #[error("invalid navigation symbol {0}")]
    InvalidNavigationSymbol(i8),
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
