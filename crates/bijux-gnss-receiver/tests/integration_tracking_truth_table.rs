#![allow(missing_docs)]

use std::fs;
use std::path::Path;

use bijux_gnss_core::api::SamplesFrame;
use bijux_gnss_receiver::api::{
    sim::{
        build_iq16_capture_bundle, generate_l1_ca_multi, validate_truth_guided_tracking_table,
        SyntheticIqTruthBundle, SyntheticScenario,
    },
    ReceiverPipelineConfig,
};

const TRACKING_CARRIER_TOLERANCE_HZ: f64 = 10.0;
const TRACKING_DOPPLER_TOLERANCE_HZ: f64 = 10.0;
const TRACKING_CODE_PHASE_TOLERANCE_SAMPLES: f64 = 1.0;
const TRACKING_CN0_TOLERANCE_DB_HZ: f64 = 8.0;

#[test]
fn tracking_truth_table_records_errors_and_lock_state_per_epoch() {
    let fixture = build_truth_table_fixture("synthetic_iq_acquisition_reference_low_rate.toml");
    let report = validate_truth_guided_tracking_table(
        &fixture.config,
        &fixture.frame,
        &fixture.truth,
        TRACKING_CARRIER_TOLERANCE_HZ,
        TRACKING_DOPPLER_TOLERANCE_HZ,
        TRACKING_CODE_PHASE_TOLERANCE_SAMPLES,
        TRACKING_CN0_TOLERANCE_DB_HZ,
    );

    assert!(report.pass, "{report:?}");
    assert_eq!(report.scenario_id, "synthetic_iq_acquisition_reference_low_rate");
    assert_eq!(report.sample_rate_hz, fixture.config.sampling_freq_hz);
    assert_eq!(report.period_samples, 2046);
    assert_eq!(report.satellites.len(), 2);

    for satellite in &report.satellites {
        assert!(satellite.pass, "{satellite:?}");
        assert!(!satellite.epochs.is_empty(), "{satellite:?}");
        assert_eq!(satellite.epoch_count, satellite.epochs.len(), "{satellite:?}");
        assert!(satellite.stable_epoch_count > 0, "{satellite:?}");
        assert!(satellite.first_stable_epoch_index.is_some(), "{satellite:?}");
        assert!(
            satellite.epochs.iter().any(|epoch| !epoch.stable_tracking_epoch),
            "{satellite:?}"
        );

        for epoch in &satellite.epochs {
            assert!(!epoch.lock_state.is_empty(), "{epoch:?}");
            assert!(epoch.expected_carrier_hz.is_finite(), "{epoch:?}");
            assert!(epoch.expected_doppler_hz.is_finite(), "{epoch:?}");
            assert!(epoch.expected_code_phase_samples.is_finite(), "{epoch:?}");
            assert!(epoch.expected_cn0_db_hz.is_finite(), "{epoch:?}");
            assert!(epoch.measured_carrier_hz.is_finite(), "{epoch:?}");
            assert!(epoch.measured_doppler_hz.is_finite(), "{epoch:?}");
            assert!(epoch.measured_code_phase_samples.is_finite(), "{epoch:?}");
            assert!(epoch.measured_cn0_dbhz.is_finite(), "{epoch:?}");

            if epoch.stable_tracking_epoch {
                assert!(epoch.pass, "{epoch:?}");
                assert!(epoch.lock, "{epoch:?}");
                assert!(epoch.pll_lock, "{epoch:?}");
                assert!(epoch.fll_lock, "{epoch:?}");
                assert_eq!(epoch.lock_state, "tracking", "{epoch:?}");
                assert!(
                    epoch.carrier_error_hz <= TRACKING_CARRIER_TOLERANCE_HZ + f64::EPSILON,
                    "{epoch:?}"
                );
                assert!(
                    epoch.doppler_error_hz <= TRACKING_DOPPLER_TOLERANCE_HZ + f64::EPSILON,
                    "{epoch:?}"
                );
                assert!(
                    epoch.code_phase_error_samples
                        <= TRACKING_CODE_PHASE_TOLERANCE_SAMPLES + f64::EPSILON,
                    "{epoch:?}"
                );
                assert!(
                    epoch.cn0_error_db <= TRACKING_CN0_TOLERANCE_DB_HZ + f64::EPSILON,
                    "{epoch:?}"
                );
            }
        }
    }

    let prn3 = report
        .satellites
        .iter()
        .find(|satellite| satellite.sat.prn == 3)
        .expect("PRN 3 row");
    assert_eq!(prn3.injected_doppler_hz, 750.0);
    assert_eq!(prn3.injected_code_phase_chips, 200.25);
    assert_eq!(prn3.injected_cn0_db_hz, 58.0);

    let prn7 = report
        .satellites
        .iter()
        .find(|satellite| satellite.sat.prn == 7)
        .expect("PRN 7 row");
    assert_eq!(prn7.injected_doppler_hz, -1_000.0);
    assert_eq!(prn7.injected_code_phase_chips, 321.5);
    assert_eq!(prn7.injected_cn0_db_hz, 52.0);
}

struct TruthTableFixture {
    config: ReceiverPipelineConfig,
    frame: SamplesFrame,
    truth: SyntheticIqTruthBundle,
}

fn build_truth_table_fixture(scenario_file: &str) -> TruthTableFixture {
    let scenario = load_scenario(scenario_file);
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: scenario.sample_rate_hz,
        intermediate_freq_hz: scenario.intermediate_freq_hz,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        ..ReceiverPipelineConfig::default()
    };
    let frame = generate_l1_ca_multi(&config, &scenario);
    let bundle = build_iq16_capture_bundle(
        &scenario.id,
        &scenario,
        &frame,
        "2026-07-11T00:00:00Z",
        Some("integration tracking truth table".to_string()),
    );
    let scaled_frame = SamplesFrame::new(
        frame.t0,
        frame.dt_s,
        frame.iq.iter().map(|sample| *sample * bundle.truth.output_scale_applied).collect(),
    );

    TruthTableFixture { config, frame: scaled_frame, truth: bundle.truth }
}

fn load_scenario(scenario_file: &str) -> SyntheticScenario {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join(format!("../../configs/scenarios/{scenario_file}"));
    let contents = fs::read_to_string(path).expect("scenario file");
    toml::from_str(&contents).expect("valid scenario")
}
