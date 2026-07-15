include!("signal_generation/capture_generation.rs");

include!("signal_generation/signal_frames.rs");
include!("signal_generation/streaming_source.rs");

include!("signal_generation/satellite_state.rs");

include!("signal_generation/frame_regeneration.rs");

include!("signal_generation/navigation_symbols.rs");

include!("signal_generation/receiver_oscillator.rs");

include!("signal_generation/navigation_segments.rs");

fn resolved_signal_code(
    sat: SatId,
    signal_band: SignalBand,
    signal_code: bijux_gnss_core::api::SignalCode,
) -> bijux_gnss_core::api::SignalCode {
    if signal_code != bijux_gnss_core::api::SignalCode::Unknown {
        return signal_code;
    }
    match (sat.constellation, signal_band) {
        (bijux_gnss_core::api::Constellation::Gps, SignalBand::L1) => {
            bijux_gnss_core::api::SignalCode::Ca
        }
        (bijux_gnss_core::api::Constellation::Gps, SignalBand::L2) => {
            bijux_gnss_core::api::SignalCode::L2C
        }
        (bijux_gnss_core::api::Constellation::Gps, SignalBand::L5) => {
            bijux_gnss_core::api::SignalCode::L5I
        }
        (bijux_gnss_core::api::Constellation::Galileo, SignalBand::E1) => {
            bijux_gnss_core::api::SignalCode::E1B
        }
        (bijux_gnss_core::api::Constellation::Galileo, SignalBand::E5) => {
            bijux_gnss_core::api::SignalCode::E5a
        }
        (bijux_gnss_core::api::Constellation::Beidou, SignalBand::B1) => {
            bijux_gnss_core::api::SignalCode::B1I
        }
        (bijux_gnss_core::api::Constellation::Beidou, SignalBand::B2) => {
            bijux_gnss_core::api::SignalCode::B2I
        }
        _ => bijux_gnss_core::api::SignalCode::Unknown,
    }
}

#[cfg(test)]
fn nav_bit_mode_from_flip(data_bit_flip: bool) -> SyntheticNavBitMode {
    if data_bit_flip {
        SyntheticNavBitMode::AlternatingGpsLnav20ms
    } else {
        SyntheticNavBitMode::ConstantPositive
    }
}

#[cfg(test)]
fn navigation_data_for_nav_mode(nav_bit_mode: SyntheticNavBitMode) -> SyntheticNavigationData {
    match nav_bit_mode {
        SyntheticNavBitMode::ConstantPositive => SyntheticNavigationData::ConstantPositive,
        SyntheticNavBitMode::ConstantNegative => SyntheticNavigationData::ConstantNegative,
        SyntheticNavBitMode::GlonassL1FixedDataString => SyntheticNavigationData::ConstantPositive,
        SyntheticNavBitMode::GlonassL1AlternatingDataString => {
            SyntheticNavigationData::AlternatingStartPositive
        }
        SyntheticNavBitMode::GlonassL1CustomDataString => {
            SyntheticNavigationData::GlonassL1String {
                raw_data_bits: glonass_l1_raw_data_bits(
                    &SyntheticNavigationData::AlternatingStartPositive,
                )
                .to_vec(),
            }
        }
        SyntheticNavBitMode::AlternatingGpsLnav20ms
        | SyntheticNavBitMode::AlternatingGalileoInav4ms
        | SyntheticNavBitMode::AlternatingGpsL5I10ms => {
            SyntheticNavigationData::AlternatingStartPositive
        }
        SyntheticNavBitMode::NativeSymbolSequence => {
            SyntheticNavigationData::SymbolSequence(vec![1, -1])
        }
        SyntheticNavBitMode::GpsL5QNh20 => SyntheticNavigationData::ConstantPositive,
    }
}

fn glonass_l1_raw_data_bits(
    navigation_data: &SyntheticNavigationData,
) -> [i8; bijux_gnss_signal::api::GLONASS_L1_STRING_DATA_BITS] {
    let len = bijux_gnss_signal::api::GLONASS_L1_STRING_DATA_BITS;
    match navigation_data {
        SyntheticNavigationData::ConstantPositive => {
            [1; bijux_gnss_signal::api::GLONASS_L1_STRING_DATA_BITS]
        }
        SyntheticNavigationData::AlternatingStartPositive => {
            let mut raw_data_bits = [1; bijux_gnss_signal::api::GLONASS_L1_STRING_DATA_BITS];
            for (index, bit) in raw_data_bits.iter_mut().enumerate().skip(1) {
                *bit = if index % 2 == 0 { 1 } else { -1 };
            }
            raw_data_bits
        }
        SyntheticNavigationData::GlonassL1String { raw_data_bits } => {
            assert!(
                raw_data_bits.len() == len,
                "synthetic glonass l1 raw data string must contain {len} bits"
            );
            let mut validated = [1; bijux_gnss_signal::api::GLONASS_L1_STRING_DATA_BITS];
            for (slot, bit) in validated.iter_mut().zip(raw_data_bits.iter().copied()) {
                *slot = validate_navigation_symbol(bit);
            }
            validated
        }
        _ => {
            panic!("glonass l1 string modulation requires constant positive, alternating start positive, or an explicit raw data string")
        }
    }
}

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
