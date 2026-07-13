#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SatId};
use bijux_gnss_receiver::api::{
    carrier_hz_from_doppler_hz, doppler_hz_from_carrier_hz,
    sim::{generate_l1_ca, SyntheticSignalParams},
    AcquisitionEngine, ReceiverPipelineConfig, ReceiverRuntime,
};
use bijux_gnss_signal::api::samples_per_code;

const DOPPLER_BIN_HZ: i32 = 250;
const COHERENT_MS: u32 = 10;

#[test]
fn acquisition_selects_expected_high_if_doppler_bins() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 1_250_000.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 12,
        ..ReceiverPipelineConfig::default()
    };
    let samples_per_code =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let sat = SatId { constellation: Constellation::Gps, prn: 8 };
    let acquisition = AcquisitionEngine::new(config.clone(), ReceiverRuntime::default())
        .with_doppler(2_000, DOPPLER_BIN_HZ);

    for doppler_hz in [1_000.0, -1_500.0] {
        let frame = generate_l1_ca(
            &config,
            SyntheticSignalParams {
                sat,
                glonass_frequency_channel: None,
                signal_band: bijux_gnss_core::api::SignalBand::L1,
                doppler_hz,
                code_phase_chips: 200.375,
                carrier_phase_rad: 0.2,
                cn0_db_hz: 60.0,
                data_bit_flip: false,
            },
            0x1F000000 + doppler_hz.abs() as u64,
            COHERENT_MS as f64 * samples_per_code as f64 / config.sampling_freq_hz,
        );
        let result =
            acquisition.run_fft_topn(&frame, &[sat], 1, COHERENT_MS, 1).remove(0).remove(0);
        let expected_carrier_hz =
            carrier_hz_from_doppler_hz(config.intermediate_freq_hz, doppler_hz);
        let coarse_carrier_hz = result
            .doppler_refinement
            .as_ref()
            .map(|refinement| refinement.coarse_carrier_hz.0)
            .unwrap_or(result.carrier_hz.0);
        let recovered_doppler_hz =
            doppler_hz_from_carrier_hz(config.intermediate_freq_hz, result.carrier_hz.0);

        assert_eq!(
            coarse_carrier_hz, expected_carrier_hz,
            "wrong carrier bin selected for injected doppler {doppler_hz}"
        );
        assert!(
            (recovered_doppler_hz - doppler_hz).abs() <= DOPPLER_BIN_HZ as f64,
            "refined Doppler drifted away from the injected truth: injected={doppler_hz}, recovered={recovered_doppler_hz}"
        );
    }
}
