#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SatId};
use bijux_gnss_receiver::api::{
    doppler_hz_from_carrier_hz,
    sim::{generate_l1_ca, SyntheticSignalParams},
    AcquisitionEngine, ReceiverPipelineConfig,
};
use bijux_gnss_signal::api::samples_per_code;

const DOPPLER_BIN_HZ: f64 = 250.0;
const COHERENT_MS: u32 = 10;

#[test]
fn acquisition_recovers_positive_and_negative_doppler_sign_at_zero_if() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 12,
        ..ReceiverPipelineConfig::default()
    };
    let samples_per_code =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let runtime = bijux_gnss_receiver::api::ReceiverRuntime::default();
    let acquisition =
        AcquisitionEngine::new(config.clone(), runtime).with_doppler(2_000, DOPPLER_BIN_HZ as i32);

    for doppler_hz in [750.0, -750.0] {
        let frame = generate_l1_ca(
            &config,
            SyntheticSignalParams {
                sat,
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                doppler_hz,
                code_phase_chips: 321.0,
                carrier_phase_rad: 0.5,
                cn0_db_hz: 60.0,
                navigation_data: false.into(),
            },
            0xD077E200 + doppler_hz.abs() as u64,
            COHERENT_MS as f64 * samples_per_code as f64 / config.sampling_freq_hz,
        );
        let result =
            acquisition.run_fft_topn(&frame, &[sat], 1, COHERENT_MS, 1).remove(0).remove(0);
        let recovered_doppler_hz =
            doppler_hz_from_carrier_hz(config.intermediate_freq_hz, result.carrier_hz.0);

        assert_eq!(
            recovered_doppler_hz.signum(),
            doppler_hz.signum(),
            "doppler sign mismatch for injected doppler {doppler_hz}"
        );
        assert!(
            (recovered_doppler_hz - doppler_hz).abs() <= DOPPLER_BIN_HZ,
            "doppler recovery error too large: injected={doppler_hz}, recovered={recovered_doppler_hz}"
        );
    }
}
