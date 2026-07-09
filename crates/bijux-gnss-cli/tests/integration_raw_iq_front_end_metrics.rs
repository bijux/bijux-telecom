#![allow(missing_docs)]

use bijux_gnss_infra::api::core::{Constellation, SatId};
use bijux_gnss_infra::api::receiver::{
    sim::{generate_l1_ca, SyntheticSignalParams},
    ReceiverConfig,
};
use serde_json::Value;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::{SystemTime, UNIX_EPOCH};

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

fn write_constant_iq8_capture(path: &Path, i: i8, q: i8, sample_count: usize) {
    let mut bytes = Vec::with_capacity(sample_count * 2);
    for _ in 0..sample_count {
        bytes.push(i as u8);
        bytes.push(q as u8);
    }
    fs::write(path, bytes).expect("write iq capture");
}

fn write_raw_iq_sidecar(path: &Path) {
    write_raw_iq_sidecar_with_sample_rate(path, 5_000_000.0);
}

fn write_raw_iq_sidecar_with_sample_rate(path: &Path, sample_rate_hz: f64) {
    fs::write(
        path,
        format!(
            r#"
format = "iq8"
sample_rate_hz = {sample_rate_hz}
intermediate_freq_hz = 0.0
capture_start_utc = "2026-07-09T00:00:00Z"
quantization_bits = 8
"#
        ),
    )
    .expect("write sidecar");
}

fn load_json(path: &Path) -> Value {
    serde_json::from_str(&fs::read_to_string(path).expect("read json")).expect("parse json")
}

fn write_receiver_config(path: &Path, configure: impl FnOnce(&mut ReceiverConfig)) {
    let mut profile = ReceiverConfig::default();
    configure(&mut profile);
    fs::write(path, toml::to_string_pretty(&profile).expect("serialize config"))
        .expect("write config");
}

fn assert_constant_iq_metrics(metrics: &Value, sample_count: u64) {
    assert_eq!(metrics.get("sample_count").and_then(Value::as_u64), Some(sample_count));
    assert_eq!(metrics.get("i_mean").and_then(Value::as_f64), Some(0.25));
    assert_eq!(metrics.get("q_mean").and_then(Value::as_f64), Some(0.0));
    assert_eq!(metrics.get("i_power").and_then(Value::as_f64), Some(0.0625));
    assert_eq!(metrics.get("q_power").and_then(Value::as_f64), Some(0.0));
    assert!(metrics
        .get("iq_power_ratio")
        .and_then(Value::as_f64)
        .expect("iq_power_ratio")
        > 1.0e10);
    assert_eq!(
        metrics.get("power_imbalance_warning").and_then(Value::as_bool),
        Some(true)
    );
    assert_eq!(metrics.get("rms").and_then(Value::as_f64), Some(0.25));
    assert_eq!(metrics.get("dc_imbalance").and_then(Value::as_f64), Some(1.0));
}

fn write_biased_synthetic_iq8_capture(path: &Path, i_bias: f32, q_bias: f32) {
    let mut profile = ReceiverConfig::default();
    profile.sample_rate_hz = 4_092_000.0;
    profile.intermediate_freq_hz = 0.0;
    let pipeline = profile.to_pipeline_config();
    let frame = generate_l1_ca(
        &pipeline,
        SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 1 },
            doppler_hz: 500.0,
            code_phase_chips: 200.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 25.0,
            data_bit_flip: false,
        },
        0xC0FF_EE11,
        1_023.0 / profile.code_freq_basis_hz,
    );
    let peak_component = frame
        .iq
        .iter()
        .flat_map(|sample| [sample.re.abs(), sample.im.abs()])
        .fold(0.0_f32, f32::max)
        .max(1.0);
    let amplitude_scale = 0.45 / peak_component;

    let mut raw = Vec::with_capacity(frame.iq.len() * 2);
    for sample in &frame.iq {
        let i = ((sample.re * amplitude_scale + i_bias).clamp(-1.0, 127.0 / 128.0) * 128.0)
            .round() as i8;
        let q = ((sample.im * amplitude_scale + q_bias).clamp(-1.0, 127.0 / 128.0) * 128.0)
            .round() as i8;
        raw.push(i as u8);
        raw.push(q as u8);
    }
    fs::write(path, raw).expect("write biased iq8 capture");
}

#[test]
fn acquire_reports_front_end_metrics_from_acquisition_window() {
    let temp = temp_dir_path("acquire_front_end_metrics");
    fs::create_dir_all(&temp).expect("create temp dir");

    let iq_path = temp.join("demo.iq8");
    write_constant_iq8_capture(&iq_path, 32, 0, 5_000);
    let sidecar_path = temp.join("demo.sidecar.toml");
    write_raw_iq_sidecar(&sidecar_path);

    let out_dir = temp.join("acquire-out");
    let output = run_bijux(
        &[
            "gnss",
            "acquire",
            "--unregistered-dataset",
            "--file",
            iq_path.to_str().expect("iq path"),
            "--sidecar",
            sidecar_path.to_str().expect("sidecar path"),
            "--prn",
            "1",
            "--report",
            "json",
            "--out",
            out_dir.to_str().expect("out dir"),
        ],
        &repo_root(),
    );

    assert!(output.status.success(), "acquire failed: {}", String::from_utf8_lossy(&output.stderr));
    let report = load_json(&out_dir.join("acquire_report.json"));
    let metrics = report
        .get("front_end_metrics")
        .expect("front_end_metrics present");
    assert_constant_iq_metrics(metrics, 5_000);

    fs::remove_dir_all(&temp).expect("remove temp dir");
}

#[test]
fn track_reports_front_end_metrics_from_acquisition_window() {
    let temp = temp_dir_path("track_front_end_metrics");
    fs::create_dir_all(&temp).expect("create temp dir");

    let iq_path = temp.join("demo.iq8");
    write_constant_iq8_capture(&iq_path, 32, 0, 5_000);
    let sidecar_path = temp.join("demo.sidecar.toml");
    write_raw_iq_sidecar(&sidecar_path);

    let out_dir = temp.join("track-out");
    let output = run_bijux(
        &[
            "gnss",
            "track",
            "--unregistered-dataset",
            "--file",
            iq_path.to_str().expect("iq path"),
            "--sidecar",
            sidecar_path.to_str().expect("sidecar path"),
            "--prn",
            "1",
            "--report",
            "json",
            "--out",
            out_dir.to_str().expect("out dir"),
        ],
        &repo_root(),
    );

    assert!(output.status.success(), "track failed: {}", String::from_utf8_lossy(&output.stderr));
    let report = load_json(&out_dir.join("track_report.json"));
    let metrics = report
        .get("front_end_metrics")
        .expect("front_end_metrics present");
    assert_constant_iq_metrics(metrics, 5_000);

    fs::remove_dir_all(&temp).expect("remove temp dir");
}

#[test]
fn inspect_reports_front_end_metrics_from_requested_samples() {
    let temp = temp_dir_path("inspect_front_end_metrics");
    fs::create_dir_all(&temp).expect("create temp dir");

    let iq_path = temp.join("demo.iq8");
    write_constant_iq8_capture(&iq_path, 32, 0, 4);
    let sidecar_path = temp.join("demo.sidecar.toml");
    write_raw_iq_sidecar(&sidecar_path);

    let out_dir = temp.join("inspect-out");
    let output = run_bijux(
        &[
            "gnss",
            "inspect",
            "--unregistered-dataset",
            "--file",
            iq_path.to_str().expect("iq path"),
            "--sidecar",
            sidecar_path.to_str().expect("sidecar path"),
            "--max-samples",
            "4",
            "--report",
            "json",
            "--out",
            out_dir.to_str().expect("out dir"),
        ],
        &repo_root(),
    );

    assert!(output.status.success(), "inspect failed: {}", String::from_utf8_lossy(&output.stderr));
    let report = load_json(&out_dir.join("inspect_report.json"));
    let metrics = report
        .get("front_end_metrics")
        .expect("front_end_metrics present");
    assert_constant_iq_metrics(metrics, 4);
    assert_eq!(report.get("total_samples").and_then(Value::as_u64), Some(4));

    fs::remove_dir_all(&temp).expect("remove temp dir");
}

#[test]
fn run_reports_front_end_metrics_for_stream_start() {
    let temp = temp_dir_path("run_front_end_metrics");
    fs::create_dir_all(&temp).expect("create temp dir");

    let iq_path = temp.join("demo.iq8");
    write_constant_iq8_capture(&iq_path, 32, 0, 5_000);
    let sidecar_path = temp.join("demo.sidecar.toml");
    write_raw_iq_sidecar(&sidecar_path);

    let out_dir = temp.join("run-out");
    let output = run_bijux(
        &[
            "gnss",
            "run",
            "--unregistered-dataset",
            "--file",
            iq_path.to_str().expect("iq path"),
            "--sidecar",
            sidecar_path.to_str().expect("sidecar path"),
            "--report",
            "json",
            "--out",
            out_dir.to_str().expect("out dir"),
        ],
        &repo_root(),
    );

    assert!(output.status.success(), "run failed: {}", String::from_utf8_lossy(&output.stderr));
    let report = load_json(&out_dir.join("run_report.json"));
    let metrics = report
        .get("front_end_metrics")
        .expect("front_end_metrics present");
    assert_constant_iq_metrics(metrics, 5_000);
    assert_eq!(report.get("epochs").and_then(Value::as_u64), Some(1));

    fs::remove_dir_all(&temp).expect("remove temp dir");
}

#[test]
fn acquire_config_dc_removal_preserves_or_improves_biased_peak_quality() {
    let temp = temp_dir_path("acquire_config_dc_removal");
    fs::create_dir_all(&temp).expect("create temp dir");

    let iq_path = temp.join("biased.iq8");
    write_biased_synthetic_iq8_capture(&iq_path, 0.35, -0.20);
    let sidecar_path = temp.join("biased.sidecar.toml");
    write_raw_iq_sidecar_with_sample_rate(&sidecar_path, 4_092_000.0);

    let disabled_config = temp.join("receiver-disabled.toml");
    write_receiver_config(&disabled_config, |profile| {
        profile.sample_rate_hz = 4_092_000.0;
        profile.acquisition.doppler_search_hz = 1_000;
        profile.acquisition.doppler_step_hz = 250;
        profile.acquisition.peak_mean_threshold = 1.5;
        profile.acquisition.peak_second_threshold = 1.1;
        profile.front_end.remove_dc_offset = false;
    });

    let enabled_config = temp.join("receiver-enabled.toml");
    write_receiver_config(&enabled_config, |profile| {
        profile.sample_rate_hz = 4_092_000.0;
        profile.acquisition.doppler_search_hz = 1_000;
        profile.acquisition.doppler_step_hz = 250;
        profile.acquisition.peak_mean_threshold = 1.5;
        profile.acquisition.peak_second_threshold = 1.1;
        profile.front_end.remove_dc_offset = true;
    });

    let disabled_out = temp.join("acquire-disabled");
    let disabled = run_bijux(
        &[
            "gnss",
            "acquire",
            "--config",
            disabled_config.to_str().expect("config path"),
            "--unregistered-dataset",
            "--file",
            iq_path.to_str().expect("iq path"),
            "--sidecar",
            sidecar_path.to_str().expect("sidecar path"),
            "--prn",
            "1",
            "--report",
            "json",
            "--out",
            disabled_out.to_str().expect("out dir"),
        ],
        &repo_root(),
    );
    assert!(disabled.status.success(), "acquire without dc removal failed: {}", String::from_utf8_lossy(&disabled.stderr));

    let enabled_out = temp.join("acquire-enabled");
    let enabled = run_bijux(
        &[
            "gnss",
            "acquire",
            "--config",
            enabled_config.to_str().expect("config path"),
            "--unregistered-dataset",
            "--file",
            iq_path.to_str().expect("iq path"),
            "--sidecar",
            sidecar_path.to_str().expect("sidecar path"),
            "--prn",
            "1",
            "--report",
            "json",
            "--out",
            enabled_out.to_str().expect("out dir"),
        ],
        &repo_root(),
    );
    assert!(enabled.status.success(), "acquire with dc removal failed: {}", String::from_utf8_lossy(&enabled.stderr));

    let disabled_report = load_json(&disabled_out.join("acquire_report.json"));
    let enabled_report = load_json(&enabled_out.join("acquire_report.json"));
    let disabled_peak = disabled_report
        .get("results")
        .and_then(Value::as_array)
        .and_then(|rows| rows.first())
        .and_then(|row| row.get("peak_mean_ratio"))
        .and_then(Value::as_f64)
        .expect("disabled peak mean ratio");
    let enabled_peak = enabled_report
        .get("results")
        .and_then(Value::as_array)
        .and_then(|rows| rows.first())
        .and_then(|row| row.get("peak_mean_ratio"))
        .and_then(Value::as_f64)
        .expect("enabled peak mean ratio");

    assert!(
        enabled_peak + f64::EPSILON >= disabled_peak,
        "dc removal reduced CLI acquisition peak quality: before={} after={}",
        disabled_peak,
        enabled_peak
    );

    fs::remove_dir_all(&temp).expect("remove temp dir");
}
