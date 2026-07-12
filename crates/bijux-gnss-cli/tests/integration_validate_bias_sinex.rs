#![allow(missing_docs)]

use bijux_gnss_infra::api::core::{
    signal_spec_gps_l1_ca, signal_spec_gps_l2_py, Constellation, Cycles, Hertz, LockFlags, Meters,
    ObsEpoch, ObsMetadata, ObsSatellite, ObservationEpochDecision, ObservationStatus, ReceiverRole,
    ReceiverSampleTrace, SatId, SigId, SignalBand, SignalCode, Seconds,
};
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::{SystemTime, UNIX_EPOCH};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(Path::parent)
        .expect("workspace root")
        .to_path_buf()
}

fn run_bijux(args: &[&str], cwd: &Path) -> std::process::Output {
    Command::new(env!("CARGO_BIN_EXE_bijux"))
        .args(args)
        .current_dir(cwd)
        .output()
        .expect("run bijux")
}

fn temp_dir_path(name: &str) -> PathBuf {
    let nanos = SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos();
    std::env::temp_dir().join(format!("bijux_{}_{}_{}", name, std::process::id(), nanos))
}

fn dual_frequency_epoch() -> ObsEpoch {
    let sat = SatId { constellation: Constellation::Gps, prn: 23 };
    let l1 = signal_spec_gps_l1_ca();
    let l2 = signal_spec_gps_l2_py();
    let base_range_m = 20_200_000.0;
    let iono_l1_m = 4.0;
    let l1_bias_m = 3.0e-9 * SPEED_OF_LIGHT_MPS;
    let l2_bias_m = 9.0e-9 * SPEED_OF_LIGHT_MPS;
    let l1_hz = l1.carrier_hz.value();
    let l2_hz = l2.carrier_hz.value();
    let l2_iono_m = iono_l1_m * (l1_hz * l1_hz) / (l2_hz * l2_hz);

    ObsEpoch {
        t_rx_s: Seconds(0.0),
        source_time: ReceiverSampleTrace::from_sample_index(0, 1_000.0),
        gps_week: None,
        tow_s: None,
        epoch_idx: 0,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        role: ReceiverRole::Rover,
        sats: vec![
            ObsSatellite {
                signal_id: SigId { sat, band: SignalBand::L1, code: SignalCode::Ca },
                pseudorange_m: Meters(base_range_m + iono_l1_m + l1_bias_m),
                pseudorange_var_m2: 1.0,
                carrier_phase_cycles: Cycles(0.0),
                carrier_phase_var_cycles2: 0.01,
                doppler_hz: Hertz(0.0),
                doppler_var_hz2: 1.0,
                cn0_dbhz: 45.0,
                lock_flags: LockFlags {
                    code_lock: true,
                    carrier_lock: true,
                    bit_lock: false,
                    cycle_slip: false,
                },
                multipath_suspect: false,
                observation_status: ObservationStatus::Accepted,
                observation_reject_reasons: Vec::new(),
                elevation_deg: None,
                azimuth_deg: None,
                weight: None,
                timing: None,
                error_model: None,
                metadata: ObsMetadata {
                    tracking_mode: "test".to_string(),
                    integration_ms: 1,
                    lock_quality: 45.0,
                    smoothing_window: 0,
                    smoothing_age: 0,
                    smoothing_resets: 0,
                    signal: l1,
                    ..ObsMetadata::default()
                },
            },
            ObsSatellite {
                signal_id: SigId { sat, band: SignalBand::L2, code: SignalCode::Py },
                pseudorange_m: Meters(base_range_m + l2_iono_m + l2_bias_m),
                pseudorange_var_m2: 1.0,
                carrier_phase_cycles: Cycles(0.0),
                carrier_phase_var_cycles2: 0.01,
                doppler_hz: Hertz(0.0),
                doppler_var_hz2: 1.0,
                cn0_dbhz: 45.0,
                lock_flags: LockFlags {
                    code_lock: true,
                    carrier_lock: true,
                    bit_lock: false,
                    cycle_slip: false,
                },
                multipath_suspect: false,
                observation_status: ObservationStatus::Accepted,
                observation_reject_reasons: Vec::new(),
                elevation_deg: None,
                azimuth_deg: None,
                weight: None,
                timing: None,
                error_model: None,
                metadata: ObsMetadata {
                    tracking_mode: "test".to_string(),
                    integration_ms: 1,
                    lock_quality: 45.0,
                    smoothing_window: 0,
                    smoothing_age: 0,
                    smoothing_resets: 0,
                    signal: l2,
                    ..ObsMetadata::default()
                },
            },
        ],
        decision: ObservationEpochDecision::Accepted,
        decision_reason: Some("accepted_observables_present".to_string()),
        manifest: None,
    }
}

#[test]
fn validate_emits_bias_corrected_iono_free_code_artifact() {
    let repo = repo_root();
    let out_dir = temp_dir_path("validate_bias_sinex");
    fs::create_dir_all(&out_dir).expect("create output dir");

    let obs_path = out_dir.join("obs.jsonl");
    let reference_path = out_dir.join("reference.jsonl");
    fs::write(
        &obs_path,
        format!("{}\n", serde_json::to_string(&dual_frequency_epoch()).expect("serialize obs epoch")),
    )
    .expect("write obs fixture");
    fs::write(&reference_path, "").expect("write empty reference file");

    let nav_path = repo.join("crates/bijux-gnss-nav/tests/data/noaa_brdc0640_20100305.nav");
    let bias_path = repo.join("crates/bijux-gnss-nav/tests/data/gps_l1_l2_absolute_biases.bia");
    let output = run_bijux(
        &[
            "gnss",
            "validate",
            "--file",
            obs_path.to_str().expect("obs path"),
            "--eph",
            nav_path.to_str().expect("nav path"),
            "--reference",
            reference_path.to_str().expect("reference path"),
            "--prn",
            "23",
            "--bias-sinex",
            bias_path.to_str().expect("bias path"),
            "--unregistered-dataset",
            "--out",
            out_dir.to_str().expect("out dir"),
        ],
        &repo,
    );

    assert!(
        output.status.success(),
        "validate failed: {}",
        String::from_utf8_lossy(&output.stderr)
    );

    let artifact_path = out_dir.join("artifacts/iono_free_code.jsonl");
    let text = fs::read_to_string(&artifact_path).expect("read iono-free code artifact");
    let rows: Vec<serde_json::Value> = text
        .lines()
        .map(|line| serde_json::from_str(line).expect("parse iono-free code row"))
        .collect();

    assert_eq!(rows.len(), 1);
    assert_eq!(rows[0]["status"], "ok");
    assert!(rows[0]["code_bias_m"].as_f64().expect("code bias").abs() > 0.1);
    assert!(
        (rows[0]["corrected_code_m"].as_f64().expect("corrected code") - 20_200_000.0).abs()
            < 1.0e-6
    );
    assert!(out_dir.join("artifacts/validation_report.json").exists());
    assert!(out_dir.join("artifacts/validation_evidence_bundle.json").exists());

    fs::remove_dir_all(&out_dir).expect("remove output dir");
}
