#![allow(missing_docs)]

use bijux_gnss_core::api::{
    Constellation, SatId, SignalBand, SignalCode, SignalStageSupport, SupportStatus,
};
use bijux_gnss_receiver::api::{
    sim::{SyntheticScenario, SyntheticSignalParams, SyntheticSignalSource},
    Receiver, ReceiverPipelineConfig, ReceiverRuntime,
};
use bijux_gnss_signal::api::registered_signal_registry_entries;

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
        seed: 0x2440_1000,
        satellites: vec![SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Ca,
            doppler_hz: 0.0,
            code_phase_chips: 321.375,
            carrier_phase_rad: 0.25,
            cn0_db_hz: 60.0,
            navigation_data: false.into(),
        }],
        ephemerides: Vec::new(),
        id: "receiver-support-matrix-inventory".to_string(),
    }
}

fn support_matrix_artifact() -> bijux_gnss_core::api::SupportMatrix {
    let config = gps_l1_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 11 };
    let scenario = gps_l1_scenario(sat);
    let mut source = SyntheticSignalSource::new_signal_only(&config, &scenario);
    let receiver = Receiver::new(config, ReceiverRuntime::default());
    let artifacts = receiver.run(&mut source).expect("receiver run");
    artifacts.support_matrix.expect("support matrix")
}

fn positioning_status() -> SupportStatus {
    if cfg!(feature = "nav") {
        SupportStatus::Supported
    } else {
        SupportStatus::Planned
    }
}

fn find_row(
    matrix: &bijux_gnss_core::api::SupportMatrix,
    constellation: Constellation,
    band: SignalBand,
    code: SignalCode,
) -> &bijux_gnss_core::api::SignalSupportRow {
    matrix
        .rows
        .iter()
        .find(|row| row.constellation == constellation && row.band == band && row.code == code)
        .expect("support matrix row")
}

#[test]
fn support_matrix_artifact_covers_registered_signal_inventory() {
    let matrix = support_matrix_artifact();
    let registered = registered_signal_registry_entries();

    assert_eq!(matrix.rows.len(), registered.len());
    for entry in registered {
        let matches = matrix
            .rows
            .iter()
            .filter(|row| {
                row.constellation == entry.spec.constellation
                    && row.band == entry.spec.band
                    && row.code == entry.spec.code
            })
            .count();
        assert_eq!(matches, 1, "{entry:?}");
    }
}

#[test]
fn support_matrix_artifact_reports_registered_signal_stage_boundaries() {
    let matrix = support_matrix_artifact();
    let navigation_ready = positioning_status();

    let cases = [
        (
            Constellation::Gps,
            SignalBand::L1,
            SignalCode::Ca,
            SignalStageSupport {
                acquisition: SupportStatus::Supported,
                tracking: SupportStatus::Supported,
                data_decoding: SupportStatus::Supported,
                observations: SupportStatus::Supported,
                positioning: navigation_ready,
            },
            Vec::<&str>::new(),
            if cfg!(feature = "nav") { SupportStatus::Supported } else { SupportStatus::Planned },
        ),
        (
            Constellation::Gps,
            SignalBand::L2,
            SignalCode::L2C,
            SignalStageSupport {
                acquisition: SupportStatus::Planned,
                tracking: SupportStatus::Supported,
                data_decoding: SupportStatus::Planned,
                observations: SupportStatus::Supported,
                positioning: SupportStatus::Planned,
            },
            Vec::<&str>::new(),
            SupportStatus::Planned,
        ),
        (
            Constellation::Gps,
            SignalBand::L2,
            SignalCode::Py,
            SignalStageSupport {
                acquisition: SupportStatus::Planned,
                tracking: SupportStatus::Planned,
                data_decoding: SupportStatus::Planned,
                observations: SupportStatus::Planned,
                positioning: SupportStatus::Planned,
            },
            Vec::<&str>::new(),
            SupportStatus::Planned,
        ),
        (
            Constellation::Gps,
            SignalBand::L5,
            SignalCode::L5I,
            SignalStageSupport {
                acquisition: SupportStatus::Supported,
                tracking: SupportStatus::Supported,
                data_decoding: SupportStatus::Planned,
                observations: SupportStatus::Supported,
                positioning: SupportStatus::Planned,
            },
            Vec::<&str>::new(),
            SupportStatus::Planned,
        ),
        (
            Constellation::Gps,
            SignalBand::L5,
            SignalCode::L5Q,
            SignalStageSupport {
                acquisition: SupportStatus::Supported,
                tracking: SupportStatus::Supported,
                data_decoding: SupportStatus::Planned,
                observations: SupportStatus::Supported,
                positioning: SupportStatus::Planned,
            },
            Vec::<&str>::new(),
            SupportStatus::Planned,
        ),
        (
            Constellation::Galileo,
            SignalBand::E1,
            SignalCode::E1B,
            SignalStageSupport {
                acquisition: SupportStatus::Supported,
                tracking: SupportStatus::Supported,
                data_decoding: SupportStatus::Planned,
                observations: SupportStatus::Supported,
                positioning: navigation_ready,
            },
            Vec::<&str>::new(),
            SupportStatus::Planned,
        ),
        (
            Constellation::Galileo,
            SignalBand::E1,
            SignalCode::E1C,
            SignalStageSupport {
                acquisition: SupportStatus::Planned,
                tracking: SupportStatus::Planned,
                data_decoding: SupportStatus::Planned,
                observations: SupportStatus::Planned,
                positioning: SupportStatus::Planned,
            },
            Vec::<&str>::new(),
            SupportStatus::Planned,
        ),
        (
            Constellation::Galileo,
            SignalBand::E5,
            SignalCode::E5a,
            SignalStageSupport {
                acquisition: SupportStatus::Supported,
                tracking: SupportStatus::Supported,
                data_decoding: SupportStatus::Planned,
                observations: SupportStatus::Supported,
                positioning: SupportStatus::Planned,
            },
            Vec::<&str>::new(),
            SupportStatus::Planned,
        ),
        (
            Constellation::Galileo,
            SignalBand::E5,
            SignalCode::E5b,
            SignalStageSupport {
                acquisition: SupportStatus::Supported,
                tracking: SupportStatus::Supported,
                data_decoding: SupportStatus::Planned,
                observations: SupportStatus::Supported,
                positioning: SupportStatus::Planned,
            },
            Vec::<&str>::new(),
            SupportStatus::Planned,
        ),
        (
            Constellation::Glonass,
            SignalBand::L1,
            SignalCode::Unknown,
            SignalStageSupport {
                acquisition: SupportStatus::Supported,
                tracking: SupportStatus::Supported,
                data_decoding: SupportStatus::Planned,
                observations: SupportStatus::Supported,
                positioning: navigation_ready,
            },
            vec!["glonass_frequency_channel_available"],
            SupportStatus::Planned,
        ),
        (
            Constellation::Beidou,
            SignalBand::B1,
            SignalCode::B1I,
            SignalStageSupport {
                acquisition: SupportStatus::Supported,
                tracking: SupportStatus::Supported,
                data_decoding: SupportStatus::Planned,
                observations: SupportStatus::Supported,
                positioning: navigation_ready,
            },
            Vec::<&str>::new(),
            SupportStatus::Planned,
        ),
        (
            Constellation::Beidou,
            SignalBand::B2,
            SignalCode::B2I,
            SignalStageSupport {
                acquisition: SupportStatus::Supported,
                tracking: SupportStatus::Supported,
                data_decoding: SupportStatus::Planned,
                observations: SupportStatus::Supported,
                positioning: SupportStatus::Planned,
            },
            Vec::<&str>::new(),
            SupportStatus::Planned,
        ),
    ];

    for (constellation, band, code, stage_support, requirements, status) in cases {
        let row = find_row(&matrix, constellation, band, code);

        assert_eq!(row.stage_support, stage_support, "{row:?}");
        assert_eq!(
            row.requirements,
            requirements.into_iter().map(str::to_string).collect::<Vec<_>>(),
            "{row:?}"
        );
        assert_eq!(row.status, status, "{row:?}");
    }
}
