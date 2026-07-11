//! Public API for bijux-gnss-signal.

/// Spreading code generators.
pub use crate::codes::ca_code::{
    ca_code_assignment, ca_code_assignments, ca_code_autocorrelation_summary,
    ca_code_cross_correlation_summary, ca_code_periodic_autocorrelation,
    ca_code_periodic_cross_correlation, generate_ca_code, generate_ca_code_chips,
    periodic_correlation, CaCodeAssignment, CaCodeAutocorrelationSummary,
    CaCodeCrossCorrelationSummary, Prn, CA_CODE_PERIOD_CHIPS,
};
pub use crate::codes::galileo_e1::{
    boc_subcarrier_value, galileo_e1_cboc_value, galileo_e1c_secondary_chip,
    galileo_e1c_secondary_code, generate_galileo_e1_primary_code, generate_galileo_e1b_code,
    generate_galileo_e1c_code, sample_boc_code, sample_galileo_e1_boc11_code,
    sample_galileo_e1_cboc, GalileoE1Channel, GALILEO_E1_CBOC_ALPHA, GALILEO_E1_CBOC_BETA,
    GALILEO_E1_CODE_RATE_HZ, GALILEO_E1_PRIMARY_CODE_CHIPS, GALILEO_E1_PRIMARY_PERIOD_MS,
    GALILEO_E1_SECONDARY_CODE_CHIPS,
};
/// Numerically controlled oscillator helper.
pub use crate::dsp::nco::Nco;
/// Front-end quality metrics derived from complex I/Q samples.
pub use crate::dsp::quality::{
    estimate_iq_noise_floor_db, estimate_iq_noise_floor_db_from_metrics,
    measure_iq_front_end_metrics, measure_raw_iq_front_end_metrics, remove_dc_offset_in_place,
    IqFrontEndAnalyzer, IqFrontEndMetrics,
};
/// Signal processing utilities.
pub use crate::dsp::signal::{
    advance_code_phase_chips, advance_code_phase_seconds, code_value_at_phase, sample_ca_code,
    sample_code, samples_per_code, wipeoff_carrier,
};
/// Tracking helpers.
pub use crate::dsp::tracking::{
    adaptive_bandwidth, carrier_frequency_error_hz_from_phase_delta, code_at, discriminators,
    estimate_cn0_dbhz, first_order_angular_loop_coefficients, first_order_loop_coefficients,
    phase_lock_loop_coefficients, FirstOrderLoopCoefficients, PhaseLockLoopCoefficients,
};
/// Error types.
pub use crate::error::SignalError;
/// Raw IQ metadata contracts.
pub use crate::raw_iq::{IqSampleFormat, RawIqMetadata};
/// Sample conversion helpers.
pub use crate::samples::{iq_f32_to_samples, iq_i16_to_samples, iq_i8_to_samples};

/// Streaming signal source interface.
pub trait SignalSource {
    /// Error type returned by the source.
    type Error;

    /// Sample rate in Hz.
    fn sample_rate_hz(&self) -> f64;

    /// Fetch the next sample frame.
    fn next_frame(
        &mut self,
        frame_len: usize,
    ) -> Result<Option<bijux_gnss_core::api::SamplesFrame>, Self::Error>;

    /// Whether the source has reached end-of-stream.
    fn is_done(&self) -> bool;
}

/// Minimal sample source interface.
pub trait SampleSource {
    /// Error type returned by the source.
    type Error;

    /// Fetch the next sample frame.
    fn next_samples(
        &mut self,
        frame_len: usize,
    ) -> Result<Option<bijux_gnss_core::api::SamplesFrame>, Self::Error>;
}

/// Correlator interface for tracking tests.
pub trait Correlator {
    /// Correlate a sample frame with a local code.
    fn correlate(
        &self,
        frame: &bijux_gnss_core::api::SamplesFrame,
        prn: u8,
        carrier_hz: f64,
        code_phase_samples: f64,
        early_late_spacing_chips: f64,
    ) -> (num_complex::Complex<f32>, num_complex::Complex<f32>, num_complex::Complex<f32>);
}

/// Sink interface for writing sample frames.
pub trait SampleSink {
    /// Error type returned by the sink.
    type Error;

    /// Push a sample frame into the sink.
    fn push_frame(&mut self, frame: &bijux_gnss_core::api::SamplesFrame)
        -> Result<(), Self::Error>;
}
