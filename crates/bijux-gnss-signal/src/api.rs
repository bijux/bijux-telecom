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
    /// Fetch next sample frame.
    fn next_frame(&mut self) -> Option<bijux_gnss_core::SamplesFrame>;
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
