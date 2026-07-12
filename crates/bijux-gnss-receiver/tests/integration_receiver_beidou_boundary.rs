#![allow(missing_docs)]

use bijux_gnss_core::api::{
    Constellation, SatId, SignalBand, SignalCode, SupportStatus, BEIDOU_B1_CARRIER_HZ,
    GPS_L1_CA_CARRIER_HZ,
};
use bijux_gnss_receiver::api::{
    sim::{SyntheticScenario, SyntheticSignalParams, SyntheticSignalSource},
    Receiver, ReceiverPipelineConfig, ReceiverRuntime, TrackingChannelState,
};

fn beidou_b1_config() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: GPS_L1_CA_CARRIER_HZ.value() - BEIDOU_B1_CARRIER_HZ.value(),
        code_freq_basis_hz: 2_046_000.0,
        code_length: 2046,
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

fn beidou_b1_scenario(sat: SatId) -> SyntheticScenario {
    SyntheticScenario {
        sample_rate_hz: 4_092_000.0,
        intermediate_freq_hz: GPS_L1_CA_CARRIER_HZ.value() - BEIDOU_B1_CARRIER_HZ.value(),
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.020,
        seed: 0xB1D0_1000,
        satellites: vec![SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            doppler_hz: 0.0,
            code_phase_chips: 321.375,
            carrier_phase_rad: 0.25,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
        }],
        ephemerides: Vec::new(),
        id: "receiver-beidou-b1-boundary".to_string(),
    }
}

#[test]
fn receiver_promotes_beidou_b1i_into_tracking_and_observations() {
    let config = beidou_b1_config();
    let sat = SatId { constellation: Constellation::Beidou, prn: 11 };
    let scenario = beidou_b1_scenario(sat);
    let mut source = SyntheticSignalSource::new_signal_only(&config, &scenario);
    let receiver = Receiver::new(config, ReceiverRuntime::default());

    let artifacts = receiver.run(&mut source).expect("receiver run");
    let acquisition = artifacts
        .acquisitions
        .iter()
        .find(|result| result.sat == sat)
        .expect("BeiDou acquisition result");
    let track = artifacts
        .tracking
        .iter()
        .find(|result| result.sat == sat)
        .expect("BeiDou tracking result");
    let report = artifacts
        .channel_state_reports
        .iter()
        .find(|report| report.sat == sat)
        .expect("BeiDou tracking state report");
    let observation_epoch = artifacts.observations.first().expect("BeiDou observation epoch");
    let observation = observation_epoch
        .sats
        .iter()
        .find(|row| row.signal_id.sat == sat)
        .expect("BeiDou observation row");

    assert_eq!(acquisition.signal_band, SignalBand::B1, "{acquisition:?}");
    assert!(
        matches!(
            acquisition.hypothesis,
            bijux_gnss_core::api::AcqHypothesis::Accepted
                | bijux_gnss_core::api::AcqHypothesis::Ambiguous
        ),
        "{acquisition:?}"
    );
    assert!(!track.epochs.is_empty(), "{artifacts:?}");
    assert!(
        track
            .epochs
            .iter()
            .any(|epoch| epoch.lock && epoch.dll_lock && epoch.pll_lock && epoch.fll_lock),
        "{track:?}",
    );
    assert_eq!(report.final_state, TrackingChannelState::Locked, "{report:?}");
    assert_eq!(observation.signal_id.band, SignalBand::B1, "{observation:?}");
    assert_eq!(observation.signal_id.code, SignalCode::B1I, "{observation:?}");
    assert_eq!(observation.metadata.signal.code, SignalCode::B1I, "{observation:?}");
}

#[test]
fn support_matrix_describes_beidou_b1i_as_observation_ready() {
    let config = beidou_b1_config();
    let sat = SatId { constellation: Constellation::Beidou, prn: 11 };
    let scenario = beidou_b1_scenario(sat);
    let mut source = SyntheticSignalSource::new_signal_only(&config, &scenario);
    let receiver = Receiver::new(config, ReceiverRuntime::default());

    let artifacts = receiver.run(&mut source).expect("receiver run");
    let support_matrix = artifacts.support_matrix.expect("support matrix");
    let row = support_matrix
        .rows
        .iter()
        .find(|row| {
            row.constellation == Constellation::Beidou
                && row.band == SignalBand::B1
                && row.code == SignalCode::B1I
        })
        .expect("BeiDou B1I support row");

    assert!(matches!(row.status, SupportStatus::Planned), "{row:?}");
    assert!(
        row.reason.contains("receiver acquisition, tracking, and observations support this signal path"),
        "{row:?}"
    );
    assert!(row.reason.contains("navigation"), "{row:?}");
}
