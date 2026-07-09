//! Doppler and carrier-frequency conversion helpers.
//!
//! Acquisition and tracking work with an absolute carrier frequency inside the
//! sampled band, while downstream measurement artifacts report Doppler relative
//! to the configured intermediate frequency. These helpers keep that
//! sign-sensitive conversion explicit in one place.

/// Convert a Doppler offset into an absolute carrier frequency inside the
/// sampled band.
pub fn carrier_hz_from_doppler_hz(intermediate_freq_hz: f64, doppler_hz: f64) -> f64 {
    intermediate_freq_hz + doppler_hz
}

/// Convert an absolute carrier frequency inside the sampled band into a
/// Doppler offset relative to the configured intermediate frequency.
pub fn doppler_hz_from_carrier_hz(intermediate_freq_hz: f64, carrier_hz: f64) -> f64 {
    carrier_hz - intermediate_freq_hz
}

#[cfg(test)]
mod tests {
    use super::{carrier_hz_from_doppler_hz, doppler_hz_from_carrier_hz};

    #[test]
    fn positive_doppler_maps_above_intermediate_frequency() {
        let intermediate_freq_hz = 4_130_000.0;
        let doppler_hz = 750.0;
        let carrier_hz = carrier_hz_from_doppler_hz(intermediate_freq_hz, doppler_hz);

        assert_eq!(carrier_hz, 4_130_750.0);
        assert_eq!(doppler_hz_from_carrier_hz(intermediate_freq_hz, carrier_hz), doppler_hz);
    }

    #[test]
    fn negative_doppler_maps_below_intermediate_frequency() {
        let intermediate_freq_hz = 4_130_000.0;
        let doppler_hz = -1_250.0;
        let carrier_hz = carrier_hz_from_doppler_hz(intermediate_freq_hz, doppler_hz);

        assert_eq!(carrier_hz, 4_128_750.0);
        assert_eq!(doppler_hz_from_carrier_hz(intermediate_freq_hz, carrier_hz), doppler_hz);
    }
}
