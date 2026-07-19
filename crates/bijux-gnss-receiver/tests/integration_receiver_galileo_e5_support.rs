#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SatId, SignalBand, SignalCode, SupportStatus};
use bijux_gnss_receiver::api::{
    sim::{SyntheticScenario, SyntheticSignalParams, SyntheticSignalSource},
    Receiver, ReceiverPipelineConfig, ReceiverRuntime,
};

fn galileo_e5_config() -> ReceiverPipelineConfig {
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

fn galileo_e5_scenario(
    sat: SatId,
    signal_code: SignalCode,
    data_bit_flip: bool,
) -> SyntheticScenario {
    SyntheticScenario {
        sample_rate_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.060,
        seed: 0x6A11_E500,
        satellites: vec![SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::E5,
            signal_code,
            doppler_hz: 750.0,
            code_phase_chips: 2_048.375,
            carrier_phase_rad: 0.25,
            cn0_db_hz: 60.0,
            navigation_data: data_bit_flip.into(),
        }],
        ephemerides: Vec::new(),
        id: "receiver-galileo-e5-support".to_string(),
    }
}

#[test]
#[expect(non_snake_case, reason = "slow__ is the governed nextest namespace marker")]
fn slow__support_matrix_lists_galileo_e5_with_explicit_tracking_scope() {
    let config = galileo_e5_config();
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    let scenario = galileo_e5_scenario(sat, SignalCode::E5a, false);
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

#[test]
#[expect(non_snake_case, reason = "slow__ is the governed nextest namespace marker")]
fn slow__receiver_runs_explicit_galileo_e5b_signal_through_observations() {
    let config = galileo_e5_config();
    let sat = SatId { constellation: Constellation::Galileo, prn: 11 };
    let scenario = galileo_e5_scenario(sat, SignalCode::E5b, true);
    let mut source = SyntheticSignalSource::new_signal_only(&config, &scenario);
    let receiver = Receiver::new(config, ReceiverRuntime::default());

    let artifacts = receiver.run(&mut source).expect("receiver run");
    let acquisition = artifacts
        .acquisitions
        .iter()
        .find(|result| result.sat == sat)
        .expect("Galileo E5b acquisition result");
    assert_eq!(acquisition.signal_band, SignalBand::E5, "{acquisition:?}");
    assert_eq!(acquisition.signal_code, SignalCode::E5b, "{acquisition:?}");
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
        .expect("Galileo E5b tracking result");
    assert!(!tracking.epochs.is_empty(), "{tracking:?}");
    assert!(
        tracking.epochs.iter().all(|epoch| epoch.signal_code == SignalCode::E5b),
        "{tracking:?}"
    );

    let support_matrix = artifacts.support_matrix.expect("support matrix");
    let row = support_matrix
        .rows
        .iter()
        .find(|row| {
            row.constellation == Constellation::Galileo
                && row.band == SignalBand::E5
                && row.code == SignalCode::E5b
        })
        .expect("Galileo E5b support row");

    assert!(matches!(row.status, SupportStatus::Planned), "{row:?}");
    assert!(matches!(row.stage_support.acquisition, SupportStatus::Supported), "{row:?}");
    assert!(matches!(row.stage_support.tracking, SupportStatus::Supported), "{row:?}");
    assert!(matches!(row.stage_support.data_decoding, SupportStatus::Planned), "{row:?}");
    assert!(matches!(row.stage_support.observations, SupportStatus::Supported), "{row:?}");
    assert!(matches!(row.stage_support.positioning, SupportStatus::Planned), "{row:?}");
    assert!(row.requirements.is_empty(), "{row:?}");
}
