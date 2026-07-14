#![allow(missing_docs)]

use bijux_gnss_core::api::{
    glonass_slot_sat, AcqRequest, Constellation, GlonassFrequencyChannel, GlonassSlot, SignalBand,
    GPS_L1_CA_CARRIER_HZ,
};
use bijux_gnss_receiver::api::{
    sim::{
        expected_acquisition_code_phase_samples, generate_l1_ca,
        wrapped_code_phase_error_samples_f64, SyntheticSignalParams,
    },
    AcquisitionEngine, ReceiverPipelineConfig, ReceiverRuntime,
};
use bijux_gnss_signal::api::{glonass_l1_carrier_hz, samples_per_code};

fn glonass_channel_config(channel: GlonassFrequencyChannel) -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 2_044_000.0,
        intermediate_freq_hz: GPS_L1_CA_CARRIER_HZ.value() - glonass_l1_carrier_hz(channel).value(),
        code_freq_basis_hz: 511_000.0,
        code_length: 511,
        acquisition_doppler_search_hz: 0,
        acquisition_doppler_step_hz: 250,
        acquisition_integration_ms: 1,
        acquisition_noncoherent: 1,
        channels: 1,
        ..ReceiverPipelineConfig::default()
    }
}

fn glonass_channel_signal(
    sat: bijux_gnss_core::api::SatId,
    channel: GlonassFrequencyChannel,
) -> SyntheticSignalParams {
    SyntheticSignalParams {
        sat,
        glonass_frequency_channel: Some(channel),
        signal_band: bijux_gnss_core::api::SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        doppler_hz: 0.0,
        code_phase_chips: 147.25,
        carrier_phase_rad: 0.5,
        cn0_db_hz: 60.0,
        navigation_data: false.into(),
    }
}

#[test]
fn acquisition_engine_detects_glonass_l1_requests_across_frequency_channels() {
    let slot = GlonassSlot::new(8).expect("slot 8 must be valid");
    let sat = glonass_slot_sat(slot);
    let channels = [-7, -4, 6, 13]
        .into_iter()
        .map(|value| GlonassFrequencyChannel::new(value).expect("channel must be valid"))
        .collect::<Vec<_>>();

    for (seed_offset, channel) in channels.into_iter().enumerate() {
        let config = glonass_channel_config(channel);
        let samples_per_code = samples_per_code(
            config.sampling_freq_hz,
            config.code_freq_basis_hz,
            config.code_length,
        );
        let frame = generate_l1_ca(
            &config,
            glonass_channel_signal(sat, channel),
            0x9100_0000_u64 + seed_offset as u64,
            samples_per_code as f64 / config.sampling_freq_hz,
        );
        let expected_code_phase_samples =
            expected_acquisition_code_phase_samples(&config, &frame, 147.25) as f64;
        let acquisition = AcquisitionEngine::new(config.clone(), ReceiverRuntime::default());
        let request = AcqRequest {
            sat,
            glonass_frequency_channel: Some(channel),
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_center_hz: 0.0,
            expected_line_of_sight_doppler_hz: None,
            doppler_search_hz: 0,
            doppler_step_hz: 250,
            coherent_ms: 1,
            noncoherent: 1,
        };
        let result = acquisition
            .run_fft_for_requests(&frame, &[request])
            .into_iter()
            .next()
            .expect("GLONASS acquisition result");
        let code_phase_error_samples = wrapped_code_phase_error_samples_f64(
            result.resolved_code_phase_samples(),
            expected_code_phase_samples,
            samples_per_code,
        );

        assert_eq!(result.sat.constellation, Constellation::Glonass, "{result:?}");
        assert_eq!(result.sat, sat, "{result:?}");
        assert_eq!(result.signal_band, SignalBand::L1, "{result:?}");
        assert_eq!(result.glonass_frequency_channel, Some(channel), "{result:?}");
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
