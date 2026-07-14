#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::SignalBand;
use bijux_gnss_core::api::{Constellation, SatId, SignalCode};
use bijux_gnss_receiver::api::{
    sim::{
        build_iq16_capture_bundle, generate_l1_ca_multi,
        validate_truth_guided_composite_component_recovery, SyntheticScenario,
        SyntheticSignalParams,
    },
    ReceiverPipelineConfig,
};

use support::composite_component_recovery::{
    build_noisy_capture, load_composite_component_recovery_fixture, scaled_capture_frame,
};

const NOISY_POWER_TOLERANCE_DB: f64 = 0.75;
const NOISY_PHASE_TOLERANCE_RAD: f64 = 0.20;
const MAX_RECOVERED_POWER_ERROR_DB: f64 = 0.50;
const MAX_RECOVERED_PHASE_ERROR_RAD: f64 = 0.08;

#[test]
fn composite_component_recovery_matches_multiconstellation_fixture() {
    let fixture = load_composite_component_recovery_fixture("composite_component_recovery.toml");
    let (scaled_frame, truth) = build_noisy_capture(&fixture);

    let report = validate_truth_guided_composite_component_recovery(
        &fixture.config,
        &scaled_frame,
        &truth,
        NOISY_POWER_TOLERANCE_DB,
        NOISY_PHASE_TOLERANCE_RAD,
    );

    assert!(report.pass, "{report:?}");
    assert_eq!(report.scenario_id, "composite_component_recovery");
    assert_eq!(report.solver_status, "ok");
    assert_eq!(report.sample_count, scaled_frame.len());
    assert_eq!(report.satellites.len(), 3, "{report:?}");
    assert!(report.residual_rms.is_finite(), "{report:?}");
    assert!(report.residual_rms > 0.0, "{report:?}");
    assert!(report.satellites.iter().all(|row| row.pass), "{report:?}");
    assert!(report.satellites.iter().any(|row| row.signal_band == SignalBand::L1));
    assert!(report.satellites.iter().any(|row| row.signal_band == SignalBand::E1));
    assert!(report.satellites.iter().any(|row| row.signal_band == SignalBand::B2));
}

#[test]
fn composite_component_recovery_keeps_noisy_power_and_phase_errors_small() {
    let fixture = load_composite_component_recovery_fixture("composite_component_recovery.toml");
    let (scaled_frame, truth) = build_noisy_capture(&fixture);

    let report = validate_truth_guided_composite_component_recovery(
        &fixture.config,
        &scaled_frame,
        &truth,
        NOISY_POWER_TOLERANCE_DB,
        NOISY_PHASE_TOLERANCE_RAD,
    );

    assert!(report.pass, "{report:?}");
    for row in &report.satellites {
        assert!(row.power_error_db.abs() <= MAX_RECOVERED_POWER_ERROR_DB, "{row:?}");
        assert!(row.phase_error_rad.abs() <= MAX_RECOVERED_PHASE_ERROR_RAD, "{row:?}");
    }
}

#[test]
fn composite_component_recovery_separates_same_satellite_mixed_band_rows() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 10_230_000.0,
        code_length: 10_230,
        channels: 4,
        ..ReceiverPipelineConfig::default()
    };
    let duplicated_sat = SatId { constellation: Constellation::Gps, prn: 3 };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 75.0,
        duration_s: 0.08,
        seed: 0x2640_0003,
        satellites: vec![
            SyntheticSignalParams {
                sat: duplicated_sat,
                glonass_frequency_channel: None,
                signal_band: SignalBand::L1,
                signal_code: SignalCode::Ca,
                doppler_hz: 420.0,
                code_phase_chips: 155.25,
                carrier_phase_rad: 0.25,
                cn0_db_hz: 58.0,
                navigation_data: true.into(),
            },
            SyntheticSignalParams {
                sat: duplicated_sat,
                glonass_frequency_channel: None,
                signal_band: SignalBand::L5,
                signal_code: SignalCode::L5I,
                doppler_hz: 420.0,
                code_phase_chips: 612.5,
                carrier_phase_rad: -0.40,
                cn0_db_hz: 56.0,
                navigation_data: false.into(),
            },
            SyntheticSignalParams {
                sat: SatId { constellation: Constellation::Galileo, prn: 11 },
                glonass_frequency_channel: None,
                signal_band: SignalBand::E1,
                signal_code: SignalCode::E1B,
                doppler_hz: -310.0,
                code_phase_chips: 418.75,
                carrier_phase_rad: 0.90,
                cn0_db_hz: 54.0,
                navigation_data: true.into(),
            },
        ],
        ephemerides: Vec::new(),
        id: "mixed_band_component_recovery".to_string(),
    };
    let frame = generate_l1_ca_multi(&config, &scenario);
    let bundle = build_iq16_capture_bundle(
        &scenario.id,
        &scenario,
        &frame,
        "2026-07-13T00:00:00Z",
        Some("mixed-band composite component recovery".to_string()),
    );
    let scaled_frame = scaled_capture_frame(&frame, bundle.truth.output_scale_applied);

    let report = validate_truth_guided_composite_component_recovery(
        &config,
        &scaled_frame,
        &bundle.truth,
        0.60,
        0.10,
    );

    assert!(report.pass, "{report:?}");
    let duplicated_rows =
        report.satellites.iter().filter(|row| row.sat == duplicated_sat).collect::<Vec<_>>();
    assert_eq!(duplicated_rows.len(), 2, "{report:?}");
    assert!(duplicated_rows.iter().all(|row| row.pass), "{duplicated_rows:?}");
    assert!(duplicated_rows
        .iter()
        .any(|row| row.signal_band == SignalBand::L1 && row.signal_code == SignalCode::Ca));
    assert!(duplicated_rows
        .iter()
        .any(|row| row.signal_band == SignalBand::L5 && row.signal_code == SignalCode::L5I));
}
