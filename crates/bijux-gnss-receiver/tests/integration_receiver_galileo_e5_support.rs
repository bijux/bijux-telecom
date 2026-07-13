#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SatId, SignalBand, SignalCode, SupportStatus};
use bijux_gnss_receiver::api::{
    sim::{SyntheticScenario, SyntheticSignalParams, SyntheticSignalSource},
    Receiver, ReceiverPipelineConfig, ReceiverRuntime,
};

fn galileo_e5a_config() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
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

fn galileo_e5a_scenario(sat: SatId) -> SyntheticScenario {
    SyntheticScenario {
        sample_rate_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.030,
        seed: 0x6A11_E500,
        satellites: vec![SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::E5,
            signal_code: bijux_gnss_core::api::SignalCode::E5a,
            doppler_hz: 0.0,
            code_phase_chips: 2_048.375,
            carrier_phase_rad: 0.25,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
        }],
        ephemerides: Vec::new(),
        id: "receiver-galileo-e5-support".to_string(),
    }
}

#[test]
fn support_matrix_lists_galileo_e5_with_explicit_tracking_scope() {
    let config = galileo_e5a_config();
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    let scenario = galileo_e5a_scenario(sat);
    let mut source = SyntheticSignalSource::new_signal_only(&config, &scenario);
    let receiver = Receiver::new(config, ReceiverRuntime::default());

    let artifacts = receiver.run(&mut source).expect("receiver run");
    let acquisition = artifacts
        .acquisitions
        .iter()
        .find(|result| result.sat == sat)
        .expect("Galileo E5 acquisition result");
    assert_eq!(acquisition.signal_band, SignalBand::E5, "{acquisition:?}");
    assert_eq!(acquisition.signal_code, SignalCode::E5a, "{acquisition:?}");
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
        .expect("Galileo E5 tracking result");
    assert!(!tracking.epochs.is_empty(), "{tracking:?}");
    assert!(
        tracking.epochs.iter().all(|epoch| epoch.signal_code == SignalCode::E5a),
        "{tracking:?}"
    );
    assert!(tracking.epochs.iter().any(|epoch| epoch.lock_state == "tracking"), "{tracking:?}");

    let observation_epoch = artifacts.observations.first().expect("Galileo E5 observation epoch");
    let observation = observation_epoch
        .sats
        .iter()
        .find(|row| row.signal_id.sat == sat)
        .expect("Galileo E5 observation row");
    assert_eq!(observation.signal_id.band, SignalBand::E5, "{observation:?}");
    assert_eq!(observation.signal_id.code, SignalCode::E5a, "{observation:?}");
    assert_eq!(observation.metadata.signal.code, SignalCode::E5a, "{observation:?}");

    let support_matrix = artifacts.support_matrix.expect("support matrix");
    let row = support_matrix
        .rows
        .iter()
        .find(|row| {
            row.constellation == Constellation::Galileo
                && row.band == SignalBand::E5
                && row.code == SignalCode::E5a
        })
        .expect("Galileo E5 support row");

    assert!(matches!(row.status, SupportStatus::Planned), "{row:?}");
    assert!(matches!(row.stage_support.acquisition, SupportStatus::Supported), "{row:?}");
    assert!(matches!(row.stage_support.tracking, SupportStatus::Supported), "{row:?}");
    assert!(matches!(row.stage_support.data_decoding, SupportStatus::Planned), "{row:?}");
    assert!(matches!(row.stage_support.observations, SupportStatus::Supported), "{row:?}");
    assert!(matches!(row.stage_support.positioning, SupportStatus::Planned), "{row:?}");
    assert!(row.requirements.is_empty(), "{row:?}");
}
