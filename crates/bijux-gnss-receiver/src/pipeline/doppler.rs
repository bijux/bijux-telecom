//! Doppler and carrier-frequency conversion helpers.
//!
//! Synthetic signal generation, acquisition, and tracking all model the carrier
//! as an absolute in-band frequency. Public Doppler outputs, however, remain an
//! IF-relative quantity. These helpers keep that sign-sensitive conversion
//! explicit in one place so the signal model and receiver pipeline use the same
//! convention.

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

    #[test]
    fn zero_if_carrier_matches_doppler_bin() {
        let intermediate_freq_hz = 0.0;
        let doppler_hz = 750.0;
        let carrier_hz = carrier_hz_from_doppler_hz(intermediate_freq_hz, doppler_hz);

        assert_eq!(carrier_hz, doppler_hz);
        assert_eq!(doppler_hz_from_carrier_hz(intermediate_freq_hz, carrier_hz), doppler_hz);
    }
}
