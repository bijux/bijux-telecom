//! Public API for bijux-gnss-signal.

/// Spreading code generators.
pub use crate::codes::ca_code::{generate_ca_code, Prn};
/// Numerically controlled oscillator helper.
pub use crate::dsp::nco::Nco;
/// Signal processing utilities.
pub use crate::dsp::signal::samples_per_code;
/// Tracking helpers.
pub use crate::dsp::tracking::{adaptive_bandwidth, code_at, discriminators, estimate_cn0_dbhz};
/// Sample conversion helpers.
pub use crate::samples::iq_i16_to_samples;

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
    ) -> Result<Option<bijux_gnss_core::SamplesFrame>, Self::Error>;

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
    ) -> Result<Option<bijux_gnss_core::SamplesFrame>, Self::Error>;
}

/// Correlator interface for tracking tests.
pub trait Correlator {
    /// Correlate a sample frame with a local code.
    fn correlate(
        &self,
        frame: &bijux_gnss_core::SamplesFrame,
        prn: u8,
        carrier_hz: f64,
        code_phase_samples: f64,
        early_late_spacing_chips: f64,
    ) -> (
        num_complex::Complex<f32>,
        num_complex::Complex<f32>,
        num_complex::Complex<f32>,
    );
}

/// Sink interface for writing sample frames.
pub trait SampleSink {
    /// Error type returned by the sink.
    type Error;

    /// Push a sample frame into the sink.
    fn push_frame(&mut self, frame: &bijux_gnss_core::SamplesFrame) -> Result<(), Self::Error>;
}
