#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SatId, SignalBand, SignalCode};
use bijux_gnss_receiver::api::{
    signal::IqQuantization,
    sim::{
        generate_l1_ca_multi, measure_truth_guided_quantization_loss,
        truth_guided_quantization_reference_sweep, write_truth_guided_quantization_loss_artifact,
        SyntheticScenario, SyntheticSignalParams,
    },
    ReceiverPipelineConfig,
};
use std::fs;
use std::path::{Path, PathBuf};
use std::time::{SystemTime, UNIX_EPOCH};

fn temp_file_path(name: &str) -> PathBuf {
    let nanos = SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos();
    std::env::temp_dir().join(format!("bijux_{name}_{}_{}.json", std::process::id(), nanos))
}

#[test]
fn quantization_loss_report_measures_lower_precision_against_float_reference() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 18.0,
        fll_bw_hz: 12.0,
        acquisition_integration_ms: 10,
        acquisition_noncoherent: 2,
        ..ReceiverPipelineConfig::default()
    };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.05,
        seed: 29,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 7 },
            glonass_frequency_channel: None,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Unknown,
            doppler_hz: -1000.0,
            code_phase_chips: 321.0,
            carrier_phase_rad: 0.2,
            cn0_db_hz: 48.0,
            navigation_data: false.into(),
        }],
        ephemerides: Vec::new(),
        id: "quantization-loss-sweep".to_string(),
    };

    // Pre-generate the frame once so the test fails early if the scenario becomes invalid.
    let _frame = generate_l1_ca_multi(&config, &scenario);
    let report = measure_truth_guided_quantization_loss(
        &config,
        &scenario,
        truth_guided_quantization_reference_sweep(),
        "2026-07-13T00:00:00Z",
    );

    assert_eq!(report.scenario_id, scenario.id);
    assert_eq!(report.reference_quantization, IqQuantization::Float32);
    assert_eq!(report.points.len(), 6);
    assert!(report.coherent_samples_per_epoch > 0);
    assert!(report.coherent_integration_s > 0.0);

    let float32 = &report.points[0];
    let signed16 = &report.points[1];
    let signed8 = &report.points[2];
    let signed4 = &report.points[3];
    let signed2 = &report.points[4];
    let bipolar1 = &report.points[5];

    assert_eq!(float32.quantization, IqQuantization::Float32);
    assert_eq!(float32.satellites.len(), 1);
    assert!(
        float32.satellites[0]
            .acquisition_correlation_loss_db
            .expect("float32 acquisition loss")
            .abs()
            < 1e-6
    );
    assert!(float32.satellites[0].cn0_loss_db_hz.abs() < 1e-6);

    let finite_acquisition_losses = [
        &signed16.satellites[0],
        &signed8.satellites[0],
        &signed4.satellites[0],
        &signed2.satellites[0],
    ];

    for row in finite_acquisition_losses {
        assert_eq!(row.sat, scenario.satellites[0].sat, "{row:?}");
        assert!(row.reference_acquisition_peak > 0.0, "{row:?}");
        assert!(row.quantized_acquisition_peak > 0.0, "{row:?}");
        assert!(row.reference_mean_cn0_db_hz.is_finite(), "{row:?}");
        assert!(row.quantized_mean_cn0_db_hz.is_finite(), "{row:?}");
    }
    assert!(bipolar1.satellites[0].reference_acquisition_peak > 0.0, "{bipolar1:?}");
    assert!(bipolar1.satellites[0].reference_mean_cn0_db_hz.is_finite(), "{bipolar1:?}");
    assert!(bipolar1.satellites[0].quantized_mean_cn0_db_hz.is_finite(), "{bipolar1:?}");

    assert!(
        signed16.satellites[0].acquisition_correlation_loss_db.expect("signed16 acquisition loss")
            < 0.1,
        "{signed16:?}"
    );
    assert!(signed16.satellites[0].cn0_loss_db_hz < 0.1, "{signed16:?}");
    assert!(
        signed8.satellites[0].acquisition_correlation_loss_db.expect("signed8 acquisition loss")
            > 0.0,
        "{signed8:?}"
    );
    assert!(
        signed2.satellites[0].acquisition_correlation_loss_db.expect("signed2 acquisition loss")
            > 1.0,
        "{signed2:?}"
    );
    assert_eq!(bipolar1.satellites[0].acquisition_correlation_loss_db, None, "{bipolar1:?}");
    assert_eq!(signed8.satellites[0].cn0_loss_db_hz, 0.0, "{signed8:?}");
    assert_eq!(signed4.satellites[0].cn0_loss_db_hz, 0.0, "{signed4:?}");
    assert_eq!(signed2.satellites[0].cn0_loss_db_hz, 0.0, "{signed2:?}");
    assert_eq!(bipolar1.satellites[0].cn0_loss_db_hz, 0.0, "{bipolar1:?}");
}

#[test]
fn quantization_loss_artifact_round_trips_as_json() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        acquisition_integration_ms: 10,
        acquisition_noncoherent: 2,
        ..ReceiverPipelineConfig::default()
    };
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s: 0.05,
        seed: 31,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 11 },
            glonass_frequency_channel: None,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Unknown,
            doppler_hz: 750.0,
            code_phase_chips: 128.5,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 46.0,
            navigation_data: false.into(),
        }],
        ephemerides: Vec::new(),
        id: "quantization-loss-artifact".to_string(),
    };
    let report = measure_truth_guided_quantization_loss(
        &config,
        &scenario,
        truth_guided_quantization_reference_sweep(),
        "2026-07-13T00:00:00Z",
    );
    let artifact_path = temp_file_path("synthetic_quantization_loss_artifact");

    write_truth_guided_quantization_loss_artifact(&artifact_path, &report)
        .expect("write quantization artifact");
    let round_trip: serde_json::Value = serde_json::from_str(
        &fs::read_to_string(&artifact_path).expect("read quantization artifact"),
    )
    .expect("parse quantization artifact");

    assert_eq!(round_trip["scenario_id"], "quantization-loss-artifact");
    assert_eq!(round_trip["reference_quantization"], "float32");
    assert_eq!(
        round_trip["points"].as_array().map(|points| points.len()),
        Some(truth_guided_quantization_reference_sweep().len())
    );
    assert_eq!(round_trip["points"][0]["quantization"], "float32");
    assert_eq!(round_trip["points"][1]["quantization"], "signed16_bit");

    fs::remove_file(Path::new(&artifact_path)).expect("remove quantization artifact");
}
