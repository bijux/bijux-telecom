#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SatId, SignalBand, SignalCode, SupportStatus};
use bijux_gnss_receiver::api::{
    sim::{SyntheticScenario, SyntheticSignalParams, SyntheticSignalSource},
    Receiver, ReceiverPipelineConfig, ReceiverRuntime,
};

fn gps_l1_config() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
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

fn gps_l1_scenario(sat: SatId) -> SyntheticScenario {
    SyntheticScenario {
        sample_rate_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.020,
        seed: 0x12C0_1000,
        satellites: vec![SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            doppler_hz: 0.0,
            code_phase_chips: 321.375,
            carrier_phase_rad: 0.25,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
        }],
        ephemerides: Vec::new(),
        id: "receiver-gps-l2c-support".to_string(),
    }
}

#[test]
fn support_matrix_lists_gps_l2c_with_explicit_tracking_scope() {
    let config = gps_l1_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 11 };
    let scenario = gps_l1_scenario(sat);
    let mut source = SyntheticSignalSource::new_signal_only(&config, &scenario);
    let receiver = Receiver::new(config, ReceiverRuntime::default());

    let artifacts = receiver.run(&mut source).expect("receiver run");
    let support_matrix = artifacts.support_matrix.expect("support matrix");
    let row = support_matrix
        .rows
        .iter()
        .find(|row| {
            row.constellation == Constellation::Gps
                && row.band == SignalBand::L2
                && row.code == SignalCode::L2C
        })
        .expect("GPS L2C support row");

    assert!(matches!(row.status, SupportStatus::Planned), "{row:?}");
    assert!(matches!(row.stage_support.acquisition, SupportStatus::Planned), "{row:?}");
    assert!(matches!(row.stage_support.tracking, SupportStatus::Supported), "{row:?}");
    assert!(matches!(row.stage_support.data_decoding, SupportStatus::Planned), "{row:?}");
    assert!(matches!(row.stage_support.observations, SupportStatus::Supported), "{row:?}");
    assert!(matches!(row.stage_support.positioning, SupportStatus::Planned), "{row:?}");
    assert!(row.requirements.is_empty(), "{row:?}");
}

#[test]
fn support_matrix_lists_gps_l1_ca_as_full_execution_path() {
    let config = gps_l1_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 11 };
    let scenario = gps_l1_scenario(sat);
    let mut source = SyntheticSignalSource::new_signal_only(&config, &scenario);
    let receiver = Receiver::new(config, ReceiverRuntime::default());

    let artifacts = receiver.run(&mut source).expect("receiver run");
    let support_matrix = artifacts.support_matrix.expect("support matrix");
    let row = support_matrix
        .rows
        .iter()
        .find(|row| {
            row.constellation == Constellation::Gps
                && row.band == SignalBand::L1
                && row.code == SignalCode::Ca
        })
        .expect("GPS L1 C/A support row");

    assert!(matches!(row.status, SupportStatus::Supported), "{row:?}");
    assert!(matches!(row.stage_support.acquisition, SupportStatus::Supported), "{row:?}");
    assert!(matches!(row.stage_support.tracking, SupportStatus::Supported), "{row:?}");
    assert!(matches!(row.stage_support.data_decoding, SupportStatus::Supported), "{row:?}");
    assert!(matches!(row.stage_support.observations, SupportStatus::Supported), "{row:?}");
    assert!(matches!(row.stage_support.positioning, SupportStatus::Supported), "{row:?}");
    assert!(row.requirements.is_empty(), "{row:?}");
}
