#![allow(missing_docs)]

use bijux_gnss_core::api::{
    Constellation, GlonassFrequencyChannel, SamplesFrame, SatId, SignalBand, SignalCode,
};
use bijux_gnss_receiver::api::{
    sim::{
        build_iq16_capture_bundle, generate_l1_ca_multi, truth_guided_receiver_accuracy_budgets,
        validate_acquisition_accuracy_budget, validate_truth_guided_acquisition_table,
        SyntheticAcquisitionTruthTableReport, SyntheticAcquisitionTruthTableSatellite,
        SyntheticIqTruthBundle, SyntheticScenario, SyntheticSignalParams,
    },
    ReceiverPipelineConfig,
};

#[test]
#[expect(non_snake_case, reason = "slow__ is the governed nextest namespace marker")]
fn slow__acquisition_accuracy_budget_resolves_thresholds_by_signal_family() {
    let budgets = truth_guided_receiver_accuracy_budgets();
    let cases = vec![
        (
            "gps-l1-ca",
            signal_truth_fixture(
                "acquisition-budget-gps-l1-ca",
                ReceiverPipelineConfig {
                    sampling_freq_hz: 2_046_000.0,
                    intermediate_freq_hz: 0.0,
                    code_freq_basis_hz: 1_023_000.0,
                    code_length: 1023,
                    acquisition_doppler_search_hz: 2_000,
                    acquisition_doppler_step_hz: 500,
                    ..ReceiverPipelineConfig::default()
                },
                SyntheticScenario {
                    sample_rate_hz: 2_046_000.0,
                    intermediate_freq_hz: 0.0,
                    receiver_clock_frequency_bias_hz: 0.0,
                    duration_s: 0.060,
                    seed: 0xA511_0001,
                    satellites: vec![SyntheticSignalParams {
                        sat: SatId { constellation: Constellation::Gps, prn: 3 },
                        glonass_frequency_channel: None,
                        signal_band: SignalBand::L1,
                        signal_code: SignalCode::Ca,
                        doppler_hz: 750.0,
                        code_phase_chips: 200.25,
                        carrier_phase_rad: 0.125,
                        cn0_db_hz: 60.0,
                        navigation_data: false.into(),
                    }],
                    ephemerides: Vec::new(),
                    id: "acquisition-budget-gps-l1-ca".to_string(),
                },
            ),
            500.0,
            2usize,
            1.0,
        ),
        (
            "galileo-e1b",
            signal_truth_fixture(
                "acquisition-budget-galileo-e1b",
                ReceiverPipelineConfig {
                    sampling_freq_hz: 4_092_000.0,
                    intermediate_freq_hz: 0.0,
                    code_freq_basis_hz: 1_023_000.0,
                    code_length: 4092,
                    acquisition_doppler_search_hz: 2_000,
                    acquisition_doppler_step_hz: 500,
                    acquisition_integration_ms: 20,
                    ..ReceiverPipelineConfig::default()
                },
                SyntheticScenario {
                    sample_rate_hz: 4_092_000.0,
                    intermediate_freq_hz: 0.0,
                    receiver_clock_frequency_bias_hz: 0.0,
                    duration_s: 0.080,
                    seed: 0xA511_0002,
                    satellites: vec![SyntheticSignalParams {
                        sat: SatId { constellation: Constellation::Galileo, prn: 11 },
                        glonass_frequency_channel: None,
                        signal_band: SignalBand::E1,
                        signal_code: SignalCode::E1B,
                        doppler_hz: 0.0,
                        code_phase_chips: 321.0,
                        carrier_phase_rad: 0.25,
                        cn0_db_hz: 60.0,
                        navigation_data: false.into(),
                    }],
                    ephemerides: Vec::new(),
                    id: "acquisition-budget-galileo-e1b".to_string(),
                },
            ),
            500.0,
            1usize,
            0.25,
        ),
        (
            "gps-l5q",
            signal_truth_fixture(
                "acquisition-budget-gps-l5q",
                ReceiverPipelineConfig {
                    sampling_freq_hz: 10_230_000.0,
                    intermediate_freq_hz: 0.0,
                    code_freq_basis_hz: 10_230_000.0,
                    code_length: 10_230,
                    acquisition_doppler_search_hz: 2_000,
                    acquisition_doppler_step_hz: 250,
                    ..ReceiverPipelineConfig::default()
                },
                SyntheticScenario {
                    sample_rate_hz: 10_230_000.0,
                    intermediate_freq_hz: 0.0,
                    receiver_clock_frequency_bias_hz: 0.0,
                    duration_s: 0.060,
                    seed: 0xA511_0003,
                    satellites: vec![SyntheticSignalParams {
                        sat: SatId { constellation: Constellation::Gps, prn: 18 },
                        glonass_frequency_channel: None,
                        signal_band: SignalBand::L5,
                        signal_code: SignalCode::L5Q,
                        doppler_hz: 750.0,
                        code_phase_chips: 2_048.25,
                        carrier_phase_rad: 0.4,
                        cn0_db_hz: 60.0,
                        navigation_data: false.into(),
                    }],
                    ephemerides: Vec::new(),
                    id: "acquisition-budget-gps-l5q".to_string(),
                },
            ),
            250.0,
            1usize,
            1.0,
        ),
        (
            "beidou-b2i",
            signal_truth_fixture(
                "acquisition-budget-beidou-b2i",
                ReceiverPipelineConfig {
                    sampling_freq_hz: 4_092_000.0,
                    intermediate_freq_hz: 0.0,
                    code_freq_basis_hz: 2_046_000.0,
                    code_length: 2046,
                    acquisition_doppler_search_hz: 2_000,
                    acquisition_doppler_step_hz: 250,
                    ..ReceiverPipelineConfig::default()
                },
                SyntheticScenario {
                    sample_rate_hz: 4_092_000.0,
                    intermediate_freq_hz: 0.0,
                    receiver_clock_frequency_bias_hz: 0.0,
                    duration_s: 0.060,
                    seed: 0xA511_0004,
                    satellites: vec![SyntheticSignalParams {
                        sat: SatId { constellation: Constellation::Beidou, prn: 11 },
                        glonass_frequency_channel: None,
                        signal_band: SignalBand::B2,
                        signal_code: SignalCode::B2I,
                        doppler_hz: 750.0,
                        code_phase_chips: 321.375,
                        carrier_phase_rad: 0.25,
                        cn0_db_hz: 60.0,
                        navigation_data: false.into(),
                    }],
                    ephemerides: Vec::new(),
                    id: "acquisition-budget-beidou-b2i".to_string(),
                },
            ),
            250.0,
            2usize,
            1.0,
        ),
        (
            "glonass-l1",
            signal_truth_fixture(
                "acquisition-budget-glonass-l1",
                ReceiverPipelineConfig {
                    sampling_freq_hz: 2_044_000.0,
                    intermediate_freq_hz: 0.0,
                    code_freq_basis_hz: 511_000.0,
                    code_length: 511,
                    acquisition_doppler_search_hz: 2_000,
                    acquisition_doppler_step_hz: 500,
                    ..ReceiverPipelineConfig::default()
                },
                SyntheticScenario {
                    sample_rate_hz: 2_044_000.0,
                    intermediate_freq_hz: 0.0,
                    receiver_clock_frequency_bias_hz: 0.0,
                    duration_s: 0.060,
                    seed: 0xA511_0005,
                    satellites: vec![SyntheticSignalParams {
                        sat: SatId { constellation: Constellation::Glonass, prn: 8 },
                        glonass_frequency_channel: Some(
                            GlonassFrequencyChannel::new(-4).expect("channel -4 must be valid"),
                        ),
                        signal_band: SignalBand::L1,
                        signal_code: SignalCode::Unknown,
                        doppler_hz: 500.0,
                        code_phase_chips: 128.5,
                        carrier_phase_rad: 0.15,
                        cn0_db_hz: 60.0,
                        navigation_data: false.into(),
                    }],
                    ephemerides: Vec::new(),
                    id: "acquisition-budget-glonass-l1".to_string(),
                },
            ),
            500.0,
            4usize,
            1.0,
        ),
        (
            "beidou-b1i",
            signal_truth_fixture(
                "acquisition-budget-beidou-b1i",
                ReceiverPipelineConfig {
                    sampling_freq_hz: 4_092_000.0,
                    intermediate_freq_hz: 0.0,
                    code_freq_basis_hz: 2_046_000.0,
                    code_length: 2046,
                    acquisition_doppler_search_hz: 2_000,
                    acquisition_doppler_step_hz: 250,
                    ..ReceiverPipelineConfig::default()
                },
                SyntheticScenario {
                    sample_rate_hz: 4_092_000.0,
                    intermediate_freq_hz: 0.0,
                    receiver_clock_frequency_bias_hz: 0.0,
                    duration_s: 0.060,
                    seed: 0xA511_0006,
                    satellites: vec![SyntheticSignalParams {
                        sat: SatId { constellation: Constellation::Beidou, prn: 19 },
                        glonass_frequency_channel: None,
                        signal_band: SignalBand::B1,
                        signal_code: SignalCode::B1I,
                        doppler_hz: -250.0,
                        code_phase_chips: 512.25,
                        carrier_phase_rad: 0.35,
                        cn0_db_hz: 60.0,
                        navigation_data: false.into(),
                    }],
                    ephemerides: Vec::new(),
                    id: "acquisition-budget-beidou-b1i".to_string(),
                },
            ),
            250.0,
            2usize,
            1.0,
        ),
    ];

    for (
        label,
        fixture,
        expected_doppler_hz,
        expected_code_phase_samples,
        expected_code_phase_chips,
    ) in cases
    {
        let truth_table = validate_truth_guided_acquisition_table(
            &fixture.config,
            &fixture.frame,
            &fixture.truth,
            2,
            4,
        );
        let report = validate_acquisition_accuracy_budget(&truth_table, budgets.acquisition);
        let satellite = report.satellites.first().expect(label);

        assert!(report.pass, "{label}: {report:#?}");
        assert_eq!(report.satellite_count, 1, "{label}: {report:#?}");
        assert_eq!(satellite.max_doppler_error_hz, expected_doppler_hz, "{label}: {satellite:#?}");
        assert_eq!(
            satellite.max_code_phase_error_samples, expected_code_phase_samples,
            "{label}: {satellite:#?}"
        );
        assert!(
            (satellite.max_code_phase_error_chips - expected_code_phase_chips).abs()
                <= f64::EPSILON,
            "{label}: {satellite:#?}"
        );
    }
}

#[test]
fn acquisition_accuracy_budget_rejects_split_spectrum_code_phase_error_inside_legacy_budget() {
    let budgets = truth_guided_receiver_accuracy_budgets();
    let report = synthetic_truth_table_report(SyntheticTruthTableReportRequest {
        sat: SatId { constellation: Constellation::Galileo, prn: 11 },
        glonass_frequency_channel: None,
        signal_band: SignalBand::E1,
        signal_code: SignalCode::E1B,
        sample_rate_hz: 4_092_000.0,
        doppler_step_hz: 500,
        doppler_error_hz: 400.0,
        code_phase_error_samples: 2,
    });

    let accuracy = validate_acquisition_accuracy_budget(&report, budgets.acquisition);
    let satellite = accuracy.satellites.first().expect("Galileo E1B budget row");

    assert!(!accuracy.pass, "{accuracy:#?}");
    assert_eq!(satellite.max_doppler_error_hz, 500.0, "{satellite:#?}");
    assert_eq!(satellite.max_code_phase_error_samples, 1, "{satellite:#?}");
    assert_eq!(satellite.code_phase_error_samples, 2, "{satellite:#?}");
    assert!(!satellite.pass, "{satellite:#?}");
}

#[test]
fn acquisition_accuracy_budget_rejects_high_rate_doppler_error_inside_legacy_budget() {
    let budgets = truth_guided_receiver_accuracy_budgets();
    let report = synthetic_truth_table_report(SyntheticTruthTableReportRequest {
        sat: SatId { constellation: Constellation::Gps, prn: 18 },
        glonass_frequency_channel: None,
        signal_band: SignalBand::L5,
        signal_code: SignalCode::L5Q,
        sample_rate_hz: 10_230_000.0,
        doppler_step_hz: 250,
        doppler_error_hz: 300.0,
        code_phase_error_samples: 1,
    });

    let accuracy = validate_acquisition_accuracy_budget(&report, budgets.acquisition);
    let satellite = accuracy.satellites.first().expect("GPS L5Q budget row");

    assert!(!accuracy.pass, "{accuracy:#?}");
    assert_eq!(satellite.max_doppler_error_hz, 250.0, "{satellite:#?}");
    assert_eq!(satellite.max_code_phase_error_samples, 1, "{satellite:#?}");
    assert_eq!(satellite.doppler_error_hz, 300.0, "{satellite:#?}");
    assert!(!satellite.pass, "{satellite:#?}");
}

struct TruthTableFixture {
    config: ReceiverPipelineConfig,
    frame: SamplesFrame,
    truth: SyntheticIqTruthBundle,
}

struct SyntheticTruthTableReportRequest {
    sat: SatId,
    glonass_frequency_channel: Option<bijux_gnss_core::api::GlonassFrequencyChannel>,
    signal_band: SignalBand,
    signal_code: SignalCode,
    sample_rate_hz: f64,
    doppler_step_hz: i32,
    doppler_error_hz: f64,
    code_phase_error_samples: usize,
}

fn signal_truth_fixture(
    scenario_id: &str,
    config: ReceiverPipelineConfig,
    scenario: SyntheticScenario,
) -> TruthTableFixture {
    let frame = generate_l1_ca_multi(&config, &scenario);
    let bundle = build_iq16_capture_bundle(
        scenario_id,
        &scenario,
        &frame,
        "2026-07-14T00:00:00Z",
        Some(format!("integration {scenario_id}")),
    );
    let scaled_frame = SamplesFrame::new(
        frame.t0,
        frame.dt_s,
        frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
    );

    TruthTableFixture { config, frame: scaled_frame, truth: bundle.truth }
}

fn synthetic_truth_table_report(
    request: SyntheticTruthTableReportRequest,
) -> SyntheticAcquisitionTruthTableReport {
    let SyntheticTruthTableReportRequest {
        sat,
        glonass_frequency_channel,
        signal_band,
        signal_code,
        sample_rate_hz,
        doppler_step_hz,
        doppler_error_hz,
        code_phase_error_samples,
    } = request;
    SyntheticAcquisitionTruthTableReport {
        scenario_id: format!("synthetic-budget-{signal_band:?}-{signal_code:?}"),
        doppler_tolerance_bins: 1,
        doppler_tolerance_hz: f64::from(doppler_step_hz),
        code_phase_tolerance_samples: 4,
        sample_rate_hz,
        period_samples: sample_rate_hz.round() as usize,
        doppler_step_hz,
        pass: true,
        satellites: vec![SyntheticAcquisitionTruthTableSatellite {
            sat,
            glonass_frequency_channel,
            signal_band,
            signal_code,
            injected_doppler_hz: 750.0,
            expected_measured_doppler_hz: 750.0,
            measured_doppler_hz: 750.0 + doppler_error_hz,
            doppler_error_hz,
            doppler_error_bins: doppler_error_hz / f64::from(doppler_step_hz.max(1)),
            injected_code_phase_chips: 200.0,
            expected_code_phase_samples: 100,
            measured_code_phase_samples: 100 + code_phase_error_samples,
            code_phase_error_samples,
            peak_mean_ratio: 12.0,
            hypothesis: "accepted".to_string(),
            doppler_pass: true,
            code_phase_pass: true,
            pass: true,
        }],
    }
}
