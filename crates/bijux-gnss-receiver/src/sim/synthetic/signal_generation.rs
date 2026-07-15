include!("signal_generation/capture_generation.rs");

include!("signal_generation/signal_frames.rs");
include!("signal_generation/streaming_source.rs");

include!("signal_generation/satellite_state.rs");

include!("signal_generation/frame_regeneration.rs");

include!("signal_generation/navigation_symbols.rs");

include!("signal_generation/receiver_oscillator.rs");

include!("signal_generation/navigation_segments.rs");

include!("signal_generation/signal_identity.rs");

#[derive(Debug, Clone)]
struct XorShift64 {
    state: u64,
}

impl XorShift64 {
    fn new(seed: u64) -> Self {
        let seed = if seed == 0 { 0xDEADBEEFCAFEBABE } else { seed };
        Self { state: seed }
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.state = x;
        x
    }

    fn next_f32(&mut self) -> f32 {
        let val = (self.next_u64() >> 40) as u32;
        val as f32 / (u32::MAX as f32)
    }

    fn next_gaussian(&mut self) -> f32 {
        let u1 = self.next_f32().max(1e-12);
        let u2 = self.next_f32();
        let r = (-2.0 * u1.ln()).sqrt();
        let theta = TAU * u2;
        r * theta.cos()
    }
}

#[cfg(test)]
mod signal_generation_tests {
    use super::{synthetic_replica_model, SyntheticNavigationData, SyntheticSignalParams};
    use bijux_gnss_core::api::{Constellation, SatId, SignalBand, SignalCode};
    use bijux_gnss_signal::api::ReplicaCodeModel;

    fn synthetic_signal_params(
        sat: SatId,
        signal_band: SignalBand,
        signal_code: SignalCode,
    ) -> SyntheticSignalParams {
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band,
            signal_code,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 60.0,
            navigation_data: SyntheticNavigationData::ConstantPositive,
        }
    }

    #[test]
    fn synthetic_replica_model_emits_composite_galileo_e5a_signal() {
        let model = synthetic_replica_model(&synthetic_signal_params(
            SatId { constellation: Constellation::Galileo, prn: 11 },
            SignalBand::E5,
            SignalCode::E5a,
        ));

        assert!(matches!(model, ReplicaCodeModel::GalileoE5aQpsk { .. }), "{model:?}");
    }

    #[test]
    fn synthetic_replica_model_emits_composite_galileo_e5b_signal() {
        let model = synthetic_replica_model(&synthetic_signal_params(
            SatId { constellation: Constellation::Galileo, prn: 11 },
            SignalBand::E5,
            SignalCode::E5b,
        ));

        assert!(matches!(model, ReplicaCodeModel::GalileoE5bQpsk { .. }), "{model:?}");
    }
}
