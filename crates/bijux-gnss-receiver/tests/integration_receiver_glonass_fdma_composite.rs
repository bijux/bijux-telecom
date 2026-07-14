#![allow(missing_docs)]

use bijux_gnss_core::api::{
    glonass_slot_sat, AcqRequest, Constellation, GlonassFrequencyChannel, GlonassSlot, SignalBand,
    SignalCode, GPS_L1_CA_CARRIER_HZ,
};
use bijux_gnss_receiver::api::{
    sim::{SyntheticScenario, SyntheticSignalParams, SyntheticSignalSource},
    Receiver, ReceiverPipelineConfig, ReceiverRuntime,
};
use bijux_gnss_signal::api::glonass_l1_carrier_hz;

fn glonass_composite_config(reference_channel: GlonassFrequencyChannel) -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 8_176_000.0,
        intermediate_freq_hz: GPS_L1_CA_CARRIER_HZ.value()
            - glonass_l1_carrier_hz(reference_channel).value(),
        code_freq_basis_hz: 511_000.0,
        code_length: 511,
        acquisition_doppler_search_hz: 0,
        acquisition_doppler_step_hz: 250,
        acquisition_integration_ms: 1,
        acquisition_noncoherent: 1,
        channels: 4,
        tracking_budget_ms: 100.0,
        tracking_over_budget_action: "continue".to_string(),
        ..ReceiverPipelineConfig::default()
    }
}

fn glonass_composite_scenario(
    lower_sat: bijux_gnss_core::api::SatId,
    lower_channel: GlonassFrequencyChannel,
    upper_sat: bijux_gnss_core::api::SatId,
    upper_channel: GlonassFrequencyChannel,
    reference_channel: GlonassFrequencyChannel,
) -> SyntheticScenario {
    SyntheticScenario {
        sample_rate_hz: 8_176_000.0,
        intermediate_freq_hz: GPS_L1_CA_CARRIER_HZ.value()
            - glonass_l1_carrier_hz(reference_channel).value(),
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.020,
        seed: 0x6100_2590,
        satellites: vec![
            SyntheticSignalParams {
                sat: lower_sat,
                glonass_frequency_channel: Some(lower_channel),
                signal_band: SignalBand::L1,
                signal_code: SignalCode::Unknown,
                doppler_hz: 0.0,
                code_phase_chips: 147.25,
                carrier_phase_rad: 0.1,
                cn0_db_hz: 60.0,
                navigation_data: true.into(),
            },
            SyntheticSignalParams {
                sat: upper_sat,
                glonass_frequency_channel: Some(upper_channel),
                signal_band: SignalBand::L1,
                signal_code: SignalCode::Unknown,
                doppler_hz: 0.0,
                code_phase_chips: 311.5,
                carrier_phase_rad: -0.2,
                cn0_db_hz: 59.0,
                navigation_data: false.into(),
            },
        ],
        ephemerides: Vec::new(),
        id: "receiver-glonass-fdma-composite".to_string(),
    }
}

#[test]
fn receiver_run_keeps_two_glonass_fdma_slots_distinct_in_one_capture() {
    let lower_slot = GlonassSlot::new(8).expect("slot 8 must be valid");
    let upper_slot = GlonassSlot::new(12).expect("slot 12 must be valid");
    let lower_sat = glonass_slot_sat(lower_slot);
    let upper_sat = glonass_slot_sat(upper_slot);
    let lower_channel = GlonassFrequencyChannel::new(-4).expect("channel -4 must be valid");
    let upper_channel = GlonassFrequencyChannel::new(2).expect("channel 2 must be valid");
    let reference_channel = GlonassFrequencyChannel::new(-1).expect("channel -1 must be valid");
    let config = glonass_composite_config(reference_channel);
    let scenario = glonass_composite_scenario(
        lower_sat,
        lower_channel,
        upper_sat,
        upper_channel,
        reference_channel,
    );
    let requests = [
        AcqRequest {
            sat: lower_sat,
            glonass_frequency_channel: Some(lower_channel),
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Unknown,
            doppler_center_hz: 0.0,
            doppler_rate_center_hz_per_s: 0.0,
            doppler_rate_search_hz_per_s: 0,
            doppler_rate_step_hz_per_s: 250,
            expected_line_of_sight_doppler_hz: None,
            assistance_bounds: None,
            doppler_search_hz: 0,
            doppler_step_hz: 250,
            coherent_ms: 1,
            noncoherent: 1,
        },
        AcqRequest {
            sat: upper_sat,
            glonass_frequency_channel: Some(upper_channel),
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Unknown,
            doppler_center_hz: 0.0,
            doppler_rate_center_hz_per_s: 0.0,
            doppler_rate_search_hz_per_s: 0,
            doppler_rate_step_hz_per_s: 250,
            expected_line_of_sight_doppler_hz: None,
            assistance_bounds: None,
            doppler_search_hz: 0,
            doppler_step_hz: 250,
            coherent_ms: 1,
            noncoherent: 1,
        },
    ];
    let mut source = SyntheticSignalSource::new_signal_only(&config, &scenario);
    let receiver = Receiver::new(config, ReceiverRuntime::default());

    let artifacts = receiver
        .run_with_acquisition_requests(&mut source, &requests)
        .expect("receiver run with explicit GLONASS requests");

    for (sat, channel) in [(lower_sat, lower_channel), (upper_sat, upper_channel)] {
        let acquisition = artifacts
            .acquisitions
            .iter()
            .find(|result| result.sat == sat)
            .expect("GLONASS acquisition result");
        assert_eq!(acquisition.signal_band, SignalBand::L1, "{acquisition:?}");
        assert_eq!(acquisition.signal_code, SignalCode::Unknown, "{acquisition:?}");
        assert_eq!(acquisition.glonass_frequency_channel, Some(channel), "{acquisition:?}");
        assert!(
            matches!(
                acquisition.hypothesis,
                bijux_gnss_core::api::AcqHypothesis::Accepted
                    | bijux_gnss_core::api::AcqHypothesis::Ambiguous
            ),
            "{acquisition:?}"
        );

        let tracking = artifacts
            .tracking
            .iter()
            .find(|result| result.sat == sat)
            .expect("GLONASS tracking result");
        assert!(!tracking.epochs.is_empty(), "{tracking:?}");
        assert!(
            tracking.epochs.iter().all(|epoch| epoch.glonass_frequency_channel == Some(channel)),
            "{tracking:?}"
        );

        let observation_epoch = artifacts.observations.first().expect("GLONASS observation epoch");
        let observation = observation_epoch
            .sats
            .iter()
            .find(|row| row.signal_id.sat == sat)
            .expect("GLONASS observation row");
        assert_eq!(observation.signal_id.band, SignalBand::L1, "{observation:?}");
        assert_eq!(observation.signal_id.code, SignalCode::Unknown, "{observation:?}");
        assert_eq!(
            observation.metadata.signal.constellation,
            Constellation::Glonass,
            "{observation:?}"
        );
    }

    let lower_tracking = artifacts
        .tracking
        .iter()
        .find(|result| result.sat == lower_sat)
        .expect("lower tracking result");
    let upper_tracking = artifacts
        .tracking
        .iter()
        .find(|result| result.sat == upper_sat)
        .expect("upper tracking result");
    assert_ne!(
        lower_tracking.epochs[0].glonass_frequency_channel,
        upper_tracking.epochs[0].glonass_frequency_channel
    );
}
