#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SatId, SignalBand, SignalCode, SupportStatus};
use bijux_gnss_receiver::api::{
    sim::{SyntheticScenario, SyntheticSignalParams, SyntheticSignalSource},
    Receiver, ReceiverPipelineConfig, ReceiverRuntime,
};

fn beidou_b1_config() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 2_046_000.0,
        code_length: 2046,
        acquisition_doppler_search_hz: 0,
        acquisition_doppler_step_hz: 500,
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
        intermediate_freq_hz: 0.0,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.020,
        seed: 0xB2D0_1000,
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
        id: "receiver-beidou-b2-support".to_string(),
    }
}

#[test]
fn support_matrix_lists_beidou_b2_with_explicit_tracking_scope() {
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
                && row.band == SignalBand::B2
                && row.code == SignalCode::B2I
        })
        .expect("BeiDou B2I support row");

    assert!(matches!(row.status, SupportStatus::Planned), "{row:?}");
    assert!(matches!(row.stage_support.acquisition, SupportStatus::Planned), "{row:?}");
    assert!(matches!(row.stage_support.tracking, SupportStatus::Planned), "{row:?}");
    assert!(matches!(row.stage_support.data_decoding, SupportStatus::Planned), "{row:?}");
    assert!(matches!(row.stage_support.observations, SupportStatus::Supported), "{row:?}");
    assert!(matches!(row.stage_support.positioning, SupportStatus::Planned), "{row:?}");
    assert!(row.requirements.iter().any(|value| value == "tracked_epoch_input"), "{row:?}");
}
