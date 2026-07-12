#![allow(missing_docs)]

use bijux_gnss_core::api::{
    AcqRequest, Constellation, SignalBand, BEIDOU_B1_CARRIER_HZ, GPS_L1_CA_CARRIER_HZ,
};
use bijux_gnss_receiver::api::{
    sim::{
        expected_acquisition_code_phase_samples, generate_l1_ca,
        wrapped_code_phase_error_samples_f64, SyntheticSignalParams,
    },
    AcquisitionEngine, ReceiverPipelineConfig, ReceiverRuntime,
};
use bijux_gnss_signal::api::samples_per_code;

fn beidou_b1i_config() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: GPS_L1_CA_CARRIER_HZ.value() - BEIDOU_B1_CARRIER_HZ.value(),
        code_freq_basis_hz: 2_046_000.0,
        code_length: 2046,
        acquisition_doppler_search_hz: 0,
        acquisition_doppler_step_hz: 250,
        acquisition_integration_ms: 1,
        acquisition_noncoherent: 1,
        channels: 1,
        ..ReceiverPipelineConfig::default()
    }
}

fn beidou_b1i_signal(
    sat: bijux_gnss_core::api::SatId,
    code_phase_chips: f64,
) -> SyntheticSignalParams {
    SyntheticSignalParams {
        sat,
        glonass_frequency_channel: None,
        doppler_hz: 0.0,
        code_phase_chips,
        carrier_phase_rad: 0.25,
        cn0_db_hz: 60.0,
        data_bit_flip: false,
    }
}

#[test]
fn acquisition_engine_detects_beidou_b1i_requests_across_prns() {
    let prns = [6_u8, 11, 24, 37];

    for (seed_offset, prn) in prns.into_iter().enumerate() {
        let config = beidou_b1i_config();
        let sat = bijux_gnss_core::api::SatId { constellation: Constellation::Beidou, prn };
        let samples_per_code = samples_per_code(
            config.sampling_freq_hz,
            config.code_freq_basis_hz,
            config.code_length,
        );
        let code_phase_chips = 173.125 + prn as f64 * 0.5;
        let frame = generate_l1_ca(
            &config,
            beidou_b1i_signal(sat, code_phase_chips),
            0xB1D0_0000_u64 + seed_offset as u64,
            samples_per_code as f64 / config.sampling_freq_hz,
        );
        let expected_code_phase_samples =
            expected_acquisition_code_phase_samples(&config, &frame, code_phase_chips) as f64;
        let acquisition = AcquisitionEngine::new(config.clone(), ReceiverRuntime::default());
        let request = AcqRequest {
            sat,
            glonass_frequency_channel: None,
            doppler_search_hz: 0,
            doppler_step_hz: 250,
            coherent_ms: 1,
            noncoherent: 1,
        };
        let result = acquisition
            .run_fft_for_requests(&frame, &[request])
            .into_iter()
            .next()
            .expect("BeiDou B1I acquisition result");
        let code_phase_error_samples = wrapped_code_phase_error_samples_f64(
            result.code_phase_samples as f64,
            expected_code_phase_samples,
            samples_per_code,
        );

        assert_eq!(result.sat, sat, "{result:?}");
        assert_eq!(result.signal_band, SignalBand::B1, "{result:?}");
        assert!(
            matches!(
                result.hypothesis,
                bijux_gnss_core::api::AcqHypothesis::Accepted
                    | bijux_gnss_core::api::AcqHypothesis::Ambiguous
            ),
            "{result:?}"
        );
        assert!(result.carrier_hz.0.abs() <= f64::EPSILON, "{result:?}");
        assert!(result.peak_mean_ratio > 10.0, "{result:?}");
        assert!(result.peak_second_ratio > 1.2, "{result:?}");
        assert!(code_phase_error_samples <= 0.5, "{result:?}");
    }
}
