#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SatId, SignalBand, SignalCode, SupportStatus};
use bijux_gnss_receiver::api::{
    sim::{SyntheticScenario, SyntheticSignalParams, SyntheticSignalSource},
    Receiver, ReceiverPipelineConfig, ReceiverRuntime, TrackingChannelState,
};

fn galileo_e1_config() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 4092,
        acquisition_doppler_search_hz: 0,
        acquisition_doppler_step_hz: 500,
        acquisition_integration_ms: 20,
        acquisition_noncoherent: 1,
        channels: 4,
        tracking_budget_ms: 100.0,
        tracking_over_budget_action: "continue".to_string(),
        ..ReceiverPipelineConfig::default()
    }
}

fn galileo_e1_scenario(sat: SatId) -> SyntheticScenario {
    SyntheticScenario {
        sample_rate_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.080,
        seed: 0x6A11_E100,
        satellites: vec![SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            doppler_hz: 0.0,
            code_phase_chips: 321.0,
            carrier_phase_rad: 0.25,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
        }],
        ephemerides: Vec::new(),
        id: "receiver-galileo-e1-boundary".to_string(),
    }
}

#[test]
fn receiver_promotes_galileo_e1_acquisitions_into_tracking() {
    let config = galileo_e1_config();
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    let scenario = galileo_e1_scenario(sat);
    let mut source = SyntheticSignalSource::new_signal_only(&config, &scenario);
    let receiver = Receiver::new(config, ReceiverRuntime::default());

    let artifacts = receiver.run(&mut source).expect("receiver run");
    let sat_result = artifacts
        .acquisitions
        .iter()
        .find(|result| result.sat == sat)
        .expect("Galileo acquisition result");
    let track = artifacts
        .tracking
        .iter()
        .find(|result| result.sat == sat)
        .expect("Galileo tracking result");
    let report = artifacts
        .channel_state_reports
        .iter()
        .find(|report| report.sat == sat)
        .expect("Galileo tracking state report");

    assert_eq!(sat_result.signal_band, SignalBand::E1, "{sat_result:?}");
    assert_eq!(sat_result.hypothesis.to_string(), "accepted", "{sat_result:?}");
    assert!(!track.epochs.is_empty(), "{artifacts:?}");
    assert!(
        track
            .epochs
            .iter()
            .any(|epoch| epoch.lock && epoch.dll_lock && epoch.pll_lock && epoch.fll_lock),
        "{track:?}",
    );
    assert_eq!(report.final_state, TrackingChannelState::Locked, "{report:?}");
}

#[test]
fn support_matrix_describes_galileo_e1_as_tracking_ready() {
    let config = galileo_e1_config();
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    let scenario = galileo_e1_scenario(sat);
    let mut source = SyntheticSignalSource::new_signal_only(&config, &scenario);
    let receiver = Receiver::new(config, ReceiverRuntime::default());

    let artifacts = receiver.run(&mut source).expect("receiver run");
    let support_matrix = artifacts.support_matrix.expect("support matrix");
    let row = support_matrix
        .rows
        .iter()
        .find(|row| {
            row.constellation == Constellation::Galileo
                && row.band == SignalBand::E1
                && row.code == SignalCode::E1B
        })
        .expect("Galileo E1-B support row");

    assert!(matches!(row.status, SupportStatus::Planned), "{row:?}");
    assert!(
        row.reason.contains("receiver acquisition and tracking support this signal path"),
        "{row:?}"
    );
    assert!(row.reason.contains("observations"), "{row:?}");
    assert!(row.reason.contains("navigation"), "{row:?}");
}
