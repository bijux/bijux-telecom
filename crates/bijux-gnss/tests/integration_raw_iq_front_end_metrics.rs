#![allow(missing_docs)]

use bijux_gnss_infra::api::core::{Constellation, SatId, SignalBand, SignalCode};
use bijux_gnss_infra::api::receiver::{
    sim::{generate_l1_ca, SyntheticSignalParams},
    ReceiverConfig,
};
use bijux_gnss_testkit::signal::synthesis::{
    generate_clipped_iq16_le_bytes, generate_clipped_iq8_bytes, generate_quadrature_skew_carrier,
};
use serde_json::Value;
use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::{SystemTime, UNIX_EPOCH};

const QUADRATURE_FIXTURE_TOLERANCE_DEG: f64 = 0.25;
const CLIPPING_REFUSAL_TOLERANCE_PCT: f64 = 1e-9;

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
    write_raw_iq_sidecar_with_format_sample_rate_and_if(path, "iq8", sample_rate_hz, 0.0);
}

fn write_raw_iq_sidecar_with_format_and_sample_rate(
    path: &Path,
    format: &str,
    sample_rate_hz: f64,
) {
    write_raw_iq_sidecar_with_format_sample_rate_and_if(path, format, sample_rate_hz, 0.0);
}

fn write_raw_iq_sidecar_with_format_sample_rate_and_if(
    path: &Path,
    format: &str,
    sample_rate_hz: f64,
    intermediate_freq_hz: f64,
) {
    let quantization_bits = match format {
        "iq8" => 8,
        "iq16_le" => 16,
        "cf32_le" => 32,
        other => panic!("unsupported raw iq format for sidecar helper: {other}"),
    };
    fs::write(
        path,
        format!(
            r#"
format = "{format}"
sample_rate_hz = {sample_rate_hz}
intermediate_freq_hz = {intermediate_freq_hz}
capture_start_utc = "2026-07-09T00:00:00Z"
quantization_bits = {quantization_bits}
"#
        ),
    )
    .expect("write sidecar");
}

fn load_json(path: &Path) -> Value {
    serde_json::from_str(&fs::read_to_string(path).expect("read json")).expect("parse json")
}

fn assert_signal_quality_report_shape(
    signal_quality: &Value,
    expected_sample_rate_hz: f64,
    expected_intermediate_frequency_hz: f64,
    expected_analyzed_samples: u64,
) {
    assert_eq!(signal_quality.get("format").and_then(Value::as_str), Some("Iq8"));
    assert_eq!(
        signal_quality.get("sample_rate_hz").and_then(Value::as_f64),
        Some(expected_sample_rate_hz)
    );
    assert_eq!(
        signal_quality.get("intermediate_freq_hz").and_then(Value::as_f64),
        Some(expected_intermediate_frequency_hz)
    );
    assert_eq!(
        signal_quality.get("analyzed_samples").and_then(Value::as_u64),
        Some(expected_analyzed_samples)
    );
    let usable_duration_s =
        signal_quality.get("usable_duration_s").and_then(Value::as_f64).expect("usable_duration_s");
    assert!(usable_duration_s > 0.0, "usable_duration_s={usable_duration_s}");
    assert!(
        signal_quality
            .get("estimated_noise_floor_db")
            .and_then(Value::as_f64)
            .is_some_and(f64::is_finite),
        "signal_quality={signal_quality}"
    );
    assert_dc_only_metrics(
        signal_quality.get("front_end_metrics").expect("front_end_metrics present"),
        expected_analyzed_samples,
    );
}

fn write_receiver_config(path: &Path, configure: impl FnOnce(&mut ReceiverConfig)) {
    let mut profile = ReceiverConfig::default();
    configure(&mut profile);
    fs::write(path, toml::to_string_pretty(&profile).expect("serialize config"))
        .expect("write config");
}

fn assert_dc_only_metrics(metrics: &Value, sample_count: u64) {
    assert_eq!(metrics.get("sample_count").and_then(Value::as_u64), Some(sample_count));
    assert_eq!(metrics.get("i_mean").and_then(Value::as_f64), Some(0.25));
    assert_eq!(metrics.get("q_mean").and_then(Value::as_f64), Some(0.0));
    assert_eq!(metrics.get("i_power").and_then(Value::as_f64), Some(0.0625));
    assert_eq!(metrics.get("q_power").and_then(Value::as_f64), Some(0.0));
    assert!(
        metrics.get("iq_power_ratio").and_then(Value::as_f64).expect("iq_power_ratio") > 1.0e10
    );
    assert_eq!(metrics.get("power_imbalance_warning").and_then(Value::as_bool), Some(true));
    assert_eq!(metrics.get("quadrature_error_deg").and_then(Value::as_f64), None);
    assert_eq!(metrics.get("quadrature_error_warning").and_then(Value::as_bool), Some(false));
    assert_eq!(metrics.get("clipping_pct").and_then(Value::as_f64), Some(0.0));
    assert_eq!(metrics.get("clipping_warning").and_then(Value::as_bool), Some(false));
    assert_eq!(metrics.get("centered_rms").and_then(Value::as_f64), Some(0.0));
    assert_eq!(metrics.get("zero_signal_detected").and_then(Value::as_bool), Some(true));
    assert!(metrics
        .get("zero_signal_reason")
        .and_then(Value::as_str)
        .expect("zero_signal_reason")
        .contains("no varying signal energy"));
    assert_eq!(metrics.get("precision_claims_allowed").and_then(Value::as_bool), Some(false));
    assert!(metrics
        .get("precision_claims_refused_reason")
        .and_then(Value::as_str)
        .expect("precision_claims_refused_reason")
        .contains("no varying signal energy"));
    assert_eq!(metrics.get("rms").and_then(Value::as_f64), Some(0.25));
    assert_eq!(metrics.get("dc_imbalance").and_then(Value::as_f64), Some(1.0));
}

fn assert_all_zero_metrics(metrics: &Value, sample_count: u64) {
    assert_eq!(metrics.get("sample_count").and_then(Value::as_u64), Some(sample_count));
    assert_eq!(metrics.get("i_mean").and_then(Value::as_f64), Some(0.0));
    assert_eq!(metrics.get("q_mean").and_then(Value::as_f64), Some(0.0));
    assert_eq!(metrics.get("i_power").and_then(Value::as_f64), Some(0.0));
    assert_eq!(metrics.get("q_power").and_then(Value::as_f64), Some(0.0));
    assert_eq!(metrics.get("iq_power_ratio").and_then(Value::as_f64), Some(1.0));
    assert_eq!(metrics.get("power_imbalance_warning").and_then(Value::as_bool), Some(false));
    assert_eq!(metrics.get("quadrature_error_deg").and_then(Value::as_f64), None);
    assert_eq!(metrics.get("quadrature_error_warning").and_then(Value::as_bool), Some(false));
    assert_eq!(metrics.get("clipping_pct").and_then(Value::as_f64), Some(0.0));
    assert_eq!(metrics.get("clipping_warning").and_then(Value::as_bool), Some(false));
    assert_eq!(metrics.get("centered_rms").and_then(Value::as_f64), Some(0.0));
    assert_eq!(metrics.get("zero_signal_detected").and_then(Value::as_bool), Some(true));
    assert!(metrics
        .get("zero_signal_reason")
        .and_then(Value::as_str)
        .expect("zero_signal_reason")
        .contains("no varying signal energy"));
    assert_eq!(metrics.get("precision_claims_allowed").and_then(Value::as_bool), Some(false));
    assert!(metrics
        .get("precision_claims_refused_reason")
        .and_then(Value::as_str)
        .expect("precision_claims_refused_reason")
        .contains("no varying signal energy"));
    assert_eq!(metrics.get("rms").and_then(Value::as_f64), Some(0.0));
    assert_eq!(metrics.get("dc_imbalance").and_then(Value::as_f64), Some(0.0));
}

fn assert_precision_refusal_metrics(
    metrics: &Value,
    sample_count: u64,
    expected_clipping_pct: f64,
) {
    assert_eq!(metrics.get("sample_count").and_then(Value::as_u64), Some(sample_count));
    let clipping_pct = metrics.get("clipping_pct").and_then(Value::as_f64).expect("clipping_pct");
    assert!(
        (clipping_pct - expected_clipping_pct).abs() <= CLIPPING_REFUSAL_TOLERANCE_PCT,
        "measured={clipping_pct} expected={expected_clipping_pct}"
    );
    assert_eq!(metrics.get("clipping_warning").and_then(Value::as_bool), Some(true));
    assert_eq!(metrics.get("precision_claims_allowed").and_then(Value::as_bool), Some(false));
    assert!(metrics
        .get("precision_claims_refused_reason")
        .and_then(Value::as_str)
        .expect("precision_claims_refused_reason")
        .contains("front-end clipping"));
}

fn write_biased_synthetic_iq8_capture(path: &Path, i_bias: f32, q_bias: f32) {
    let profile = ReceiverConfig {
        sample_rate_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        ..ReceiverConfig::default()
    };
    let pipeline = profile.to_pipeline_config();
    let frame = generate_l1_ca(
        &pipeline,
        SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 1 },
            glonass_frequency_channel: None,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Ca,
            doppler_hz: 500.0,
            code_phase_chips: 200.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 25.0,
            navigation_data: false.into(),
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
        let i = ((sample.re * amplitude_scale + i_bias).clamp(-1.0, 127.0 / 128.0) * 128.0).round()
            as i8;
        let q = ((sample.im * amplitude_scale + q_bias).clamp(-1.0, 127.0 / 128.0) * 128.0).round()
            as i8;
        raw.push(i as u8);
        raw.push(q as u8);
    }
    fs::write(path, raw).expect("write biased iq8 capture");
}

fn write_synthetic_iq8_capture_with_signal_if_and_duration_s(
    path: &Path,
    sample_rate_hz: f64,
    signal_intermediate_freq_hz: f64,
    duration_s: f64,
) {
    let profile = ReceiverConfig {
        sample_rate_hz,
        intermediate_freq_hz: signal_intermediate_freq_hz,
        ..ReceiverConfig::default()
    };
    let pipeline = profile.to_pipeline_config();
    let frame = generate_l1_ca(
        &pipeline,
        SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 11 },
            glonass_frequency_channel: None,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Ca,
            doppler_hz: 0.0,
            code_phase_chips: 210.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 48.0,
            navigation_data: false.into(),
        },
        4_277_009_102,
        duration_s,
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
        let i = ((sample.re * amplitude_scale).clamp(-1.0, 127.0 / 128.0) * 128.0).round() as i8;
        let q = ((sample.im * amplitude_scale).clamp(-1.0, 127.0 / 128.0) * 128.0).round() as i8;
        raw.push(i as u8);
        raw.push(q as u8);
    }
    fs::write(path, raw).expect("write wrong-if iq8 capture");
}

fn write_synthetic_cf32_capture_with_signal_if(
    path: &Path,
    sample_rate_hz: f64,
    signal_intermediate_freq_hz: f64,
    duration_s: f64,
) {
    let profile = ReceiverConfig {
        sample_rate_hz,
        intermediate_freq_hz: signal_intermediate_freq_hz,
        ..ReceiverConfig::default()
    };
    let pipeline = profile.to_pipeline_config();
    let frame = generate_l1_ca(
        &pipeline,
        SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 11 },
            glonass_frequency_channel: None,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Ca,
            doppler_hz: 0.0,
            code_phase_chips: 210.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 48.0,
            navigation_data: false.into(),
        },
        4_277_009_102,
        duration_s,
    );
    let mut file = fs::File::create(path).expect("create cf32 capture");
    for sample in frame.iq {
        file.write_all(&sample.re.to_le_bytes()).expect("write I sample");
        file.write_all(&sample.im.to_le_bytes()).expect("write Q sample");
    }
}

fn write_phase_skewed_cf32_capture(path: &Path, phase_error_deg: f32) {
    let samples = generate_quadrature_skew_carrier(4096, 8.0, phase_error_deg);
    let mut file = fs::File::create(path).expect("create cf32 capture");
    for sample in samples {
        file.write_all(&sample.re.to_le_bytes()).expect("write I sample");
        file.write_all(&sample.im.to_le_bytes()).expect("write Q sample");
    }
}

fn write_clipped_iq8_capture(path: &Path, sample_count: usize, clipped_sample_count: usize) {
    fs::write(path, generate_clipped_iq8_bytes(sample_count, clipped_sample_count))
        .expect("write clipped iq8 capture");
}

fn write_clipped_iq16_capture(path: &Path, sample_count: usize, clipped_sample_count: usize) {
    fs::write(path, generate_clipped_iq16_le_bytes(sample_count, clipped_sample_count))
        .expect("write clipped iq16 capture");
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
    let metrics = report.get("front_end_metrics").expect("front_end_metrics present");
    assert_dc_only_metrics(metrics, 5_000);
    let signal_quality = report.get("signal_quality").expect("signal_quality present");
    assert_eq!(signal_quality.get("analyzed_samples").and_then(Value::as_u64), Some(5_000));
    assert_eq!(signal_quality.get("front_end_metrics"), report.get("front_end_metrics"));
    let standalone_signal_quality = load_json(&out_dir.join("signal_quality_report.json"));
    assert_eq!(standalone_signal_quality, *signal_quality);

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
    let metrics = report.get("front_end_metrics").expect("front_end_metrics present");
    assert_dc_only_metrics(metrics, 5_000);
    let signal_quality = report.get("signal_quality").expect("signal_quality present");
    assert_eq!(signal_quality.get("analyzed_samples").and_then(Value::as_u64), Some(5_000));
    assert_eq!(signal_quality.get("front_end_metrics"), report.get("front_end_metrics"));
    let standalone_signal_quality = load_json(&out_dir.join("signal_quality_report.json"));
    assert_eq!(standalone_signal_quality, *signal_quality);

    fs::remove_dir_all(&temp).expect("remove temp dir");
}

#[test]
fn track_report_uses_epoch_tracking_state_for_clean_capture() {
    let temp = temp_dir_path("track_epoch_tracking_state");
    fs::create_dir_all(&temp).expect("create temp dir");

    let iq_path = temp.join("clean.iq8");
    write_synthetic_iq8_capture_with_signal_if_and_duration_s(&iq_path, 4_000_000.0, 0.0, 0.012);
    let sidecar_path = temp.join("clean.sidecar.toml");
    write_raw_iq_sidecar_with_format_sample_rate_and_if(&sidecar_path, "iq8", 5_000_000.0, 0.0);

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
            "11",
            "--doppler-search-hz",
            "1500",
            "--doppler-step-hz",
            "250",
            "--report",
            "json",
            "--out",
            out_dir.to_str().expect("out dir"),
        ],
        &repo_root(),
    );

    assert!(output.status.success(), "track failed: {}", String::from_utf8_lossy(&output.stderr));
    let report = load_json(&out_dir.join("track_report.json"));
    let rows = report.get("epochs").and_then(Value::as_array).expect("track epoch rows");
    assert!(rows.len() >= 2, "rows={rows:?}");

    let first_code_phase =
        rows[0].get("code_phase_samples").and_then(Value::as_f64).expect("first code phase");
    let first_carrier_hz =
        rows[0].get("carrier_hz").and_then(Value::as_f64).expect("first carrier");
    assert!(
        rows.iter().skip(1).any(|row| {
            row.get("code_phase_samples")
                .and_then(Value::as_f64)
                .is_some_and(|value| (value - first_code_phase).abs() > 1.0e-6)
        }),
        "rows={rows:?}"
    );
    assert!(
        rows.iter().skip(1).any(|row| {
            row.get("carrier_hz")
                .and_then(Value::as_f64)
                .is_some_and(|value| (value - first_carrier_hz).abs() > 1.0e-6)
        }),
        "rows={rows:?}"
    );

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
    let metrics = report.get("front_end_metrics").expect("front_end_metrics present");
    assert_dc_only_metrics(metrics, 4);
    assert_eq!(report.get("total_samples").and_then(Value::as_u64), Some(4));
    let usable_duration_s =
        report.get("usable_duration_s").and_then(Value::as_f64).expect("usable_duration_s");
    assert!(
        (usable_duration_s - 4.0 / 5_000_000.0).abs() <= 1e-12,
        "usable_duration_s={usable_duration_s}"
    );
    let signal_quality = report.get("signal_quality").expect("signal_quality present");
    assert_eq!(signal_quality.get("format").and_then(Value::as_str), Some("Iq8"));
    assert_eq!(signal_quality.get("sample_rate_hz").and_then(Value::as_f64), Some(5_000_000.0));
    assert_eq!(signal_quality.get("intermediate_freq_hz").and_then(Value::as_f64), Some(0.0));
    assert_eq!(signal_quality.get("analyzed_samples").and_then(Value::as_u64), Some(4));
    assert_eq!(signal_quality.get("front_end_metrics"), report.get("front_end_metrics"));
    let standalone_signal_quality = load_json(&out_dir.join("signal_quality_report.json"));
    assert_eq!(standalone_signal_quality, *signal_quality);

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
    let metrics = report.get("front_end_metrics").expect("front_end_metrics present");
    assert_dc_only_metrics(metrics, 5_000);
    assert_eq!(report.get("epochs").and_then(Value::as_u64), Some(1));
    let signal_quality = report.get("signal_quality").expect("signal_quality present");
    assert_eq!(signal_quality.get("analyzed_samples").and_then(Value::as_u64), Some(5_000));
    assert_eq!(signal_quality.get("front_end_metrics"), report.get("front_end_metrics"));
    let standalone_signal_quality = load_json(&out_dir.join("signal_quality_report.json"));
    assert_eq!(standalone_signal_quality, *signal_quality);

    fs::remove_dir_all(&temp).expect("remove temp dir");
}

#[test]
fn run_reports_streamed_tracking_progress_for_clean_capture() {
    let temp = temp_dir_path("run_streamed_tracking_progress");
    fs::create_dir_all(&temp).expect("create temp dir");

    let iq_path = temp.join("demo.iq8");
    write_synthetic_iq8_capture_with_signal_if_and_duration_s(&iq_path, 5_000_000.0, 0.0, 0.060);
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
    assert_eq!(report.get("epochs").and_then(Value::as_u64), Some(60));
    assert_eq!(report.get("processed_input_samples").and_then(Value::as_u64), Some(300_000));
    assert!(
        report.get("tracked_channels").and_then(Value::as_u64).unwrap_or(0) >= 1,
        "tracked_channels={report}",
    );
    assert!(
        report.get("observation_epochs").and_then(Value::as_u64).unwrap_or(0) >= 1,
        "observation_epochs={report}",
    );
    assert!(
        report.get("acquisitions").and_then(Value::as_u64).unwrap_or(0) >= 1,
        "acquisitions={report}",
    );
    let signal_quality = report.get("signal_quality").expect("signal_quality present");
    assert_eq!(signal_quality.get("analyzed_samples").and_then(Value::as_u64), Some(300_000));
    let standalone_signal_quality = load_json(&out_dir.join("signal_quality_report.json"));
    assert_eq!(standalone_signal_quality, *signal_quality);

    fs::remove_dir_all(&temp).expect("remove temp dir");
}

#[test]
fn raw_iq_workflows_emit_required_signal_quality_fields() {
    let temp = temp_dir_path("raw_iq_signal_quality_fields");
    fs::create_dir_all(&temp).expect("create temp dir");

    let iq_path = temp.join("demo.iq8");
    write_constant_iq8_capture(&iq_path, 32, 0, 5_000);
    let sidecar_path = temp.join("demo.sidecar.toml");
    write_raw_iq_sidecar(&sidecar_path);

    let inspect_out = temp.join("inspect-out");
    let inspect = run_bijux(
        &[
            "gnss",
            "inspect",
            "--unregistered-dataset",
            "--file",
            iq_path.to_str().expect("iq path"),
            "--sidecar",
            sidecar_path.to_str().expect("sidecar path"),
            "--max-samples",
            "5000",
            "--report",
            "json",
            "--out",
            inspect_out.to_str().expect("inspect out"),
        ],
        &repo_root(),
    );
    assert!(
        inspect.status.success(),
        "inspect failed: {}",
        String::from_utf8_lossy(&inspect.stderr)
    );

    let acquire_out = temp.join("acquire-out");
    let acquire = run_bijux(
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
            acquire_out.to_str().expect("acquire out"),
        ],
        &repo_root(),
    );
    assert!(
        acquire.status.success(),
        "acquire failed: {}",
        String::from_utf8_lossy(&acquire.stderr)
    );

    let track_out = temp.join("track-out");
    let track = run_bijux(
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
            track_out.to_str().expect("track out"),
        ],
        &repo_root(),
    );
    assert!(track.status.success(), "track failed: {}", String::from_utf8_lossy(&track.stderr));

    let run_out = temp.join("run-out");
    let run = run_bijux(
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
            run_out.to_str().expect("run out"),
        ],
        &repo_root(),
    );
    assert!(run.status.success(), "run failed: {}", String::from_utf8_lossy(&run.stderr));

    for out_dir in [&inspect_out, &acquire_out, &track_out, &run_out] {
        let signal_quality = load_json(&out_dir.join("signal_quality_report.json"));
        assert_signal_quality_report_shape(&signal_quality, 5_000_000.0, 0.0, 5_000);
    }

    fs::remove_dir_all(&temp).expect("remove temp dir");
}

#[test]
fn inspect_reports_quadrature_error_from_synthetic_fixture() {
    let expected_error_deg = 12.5_f64;
    let temp = temp_dir_path("inspect_quadrature_error");
    fs::create_dir_all(&temp).expect("create temp dir");

    let iq_path = temp.join("quadrature.cf32");
    write_phase_skewed_cf32_capture(&iq_path, expected_error_deg as f32);
    let sidecar_path = temp.join("quadrature.sidecar.toml");
    write_raw_iq_sidecar_with_format_and_sample_rate(&sidecar_path, "cf32_le", 4_096_000.0);

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
            "--report",
            "json",
            "--out",
            out_dir.to_str().expect("out dir"),
        ],
        &repo_root(),
    );

    assert!(output.status.success(), "inspect failed: {}", String::from_utf8_lossy(&output.stderr));
    let report = load_json(&out_dir.join("inspect_report.json"));
    let metrics = report.get("front_end_metrics").expect("front_end_metrics present");
    let measured_error_deg =
        metrics.get("quadrature_error_deg").and_then(Value::as_f64).expect("quadrature_error_deg");

    assert!(
        (measured_error_deg - expected_error_deg).abs() <= QUADRATURE_FIXTURE_TOLERANCE_DEG,
        "measured={measured_error_deg} expected={expected_error_deg}"
    );
    assert_eq!(metrics.get("quadrature_error_warning").and_then(Value::as_bool), Some(true));
    assert_eq!(metrics.get("power_imbalance_warning").and_then(Value::as_bool), Some(false));
    assert_eq!(metrics.get("clipping_pct").and_then(Value::as_f64), None);
    assert_eq!(metrics.get("clipping_warning").and_then(Value::as_bool), Some(false));
    assert_eq!(metrics.get("precision_claims_allowed").and_then(Value::as_bool), Some(true));
    assert_eq!(metrics.get("precision_claims_refused_reason"), Some(&Value::Null));

    fs::remove_dir_all(&temp).expect("remove temp dir");
}

#[test]
fn raw_iq_commands_refuse_precision_claims_for_clipped_iq8_capture() {
    let temp = temp_dir_path("clipped_iq8_precision_refusal");
    fs::create_dir_all(&temp).expect("create temp dir");

    let iq_path = temp.join("clipped.iq8");
    write_clipped_iq8_capture(&iq_path, 5_000, 100);
    let sidecar_path = temp.join("clipped.sidecar.toml");
    write_raw_iq_sidecar(&sidecar_path);

    let acquire_out = temp.join("acquire-out");
    let acquire = run_bijux(
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
            acquire_out.to_str().expect("out dir"),
        ],
        &repo_root(),
    );
    assert!(
        acquire.status.success(),
        "acquire failed: {}",
        String::from_utf8_lossy(&acquire.stderr)
    );
    let acquire_report = load_json(&acquire_out.join("acquire_report.json"));
    assert_precision_refusal_metrics(
        acquire_report.get("front_end_metrics").expect("front_end_metrics present"),
        5_000,
        2.0,
    );

    let track_out = temp.join("track-out");
    let track = run_bijux(
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
            track_out.to_str().expect("out dir"),
        ],
        &repo_root(),
    );
    assert!(track.status.success(), "track failed: {}", String::from_utf8_lossy(&track.stderr));
    let track_report = load_json(&track_out.join("track_report.json"));
    assert_precision_refusal_metrics(
        track_report.get("front_end_metrics").expect("front_end_metrics present"),
        5_000,
        2.0,
    );

    let inspect_out = temp.join("inspect-out");
    let inspect = run_bijux(
        &[
            "gnss",
            "inspect",
            "--unregistered-dataset",
            "--file",
            iq_path.to_str().expect("iq path"),
            "--sidecar",
            sidecar_path.to_str().expect("sidecar path"),
            "--max-samples",
            "5000",
            "--report",
            "json",
            "--out",
            inspect_out.to_str().expect("out dir"),
        ],
        &repo_root(),
    );
    assert!(
        inspect.status.success(),
        "inspect failed: {}",
        String::from_utf8_lossy(&inspect.stderr)
    );
    let inspect_report = load_json(&inspect_out.join("inspect_report.json"));
    assert_precision_refusal_metrics(
        inspect_report.get("front_end_metrics").expect("front_end_metrics present"),
        5_000,
        2.0,
    );

    let run_out = temp.join("run-out");
    let run = run_bijux(
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
            run_out.to_str().expect("out dir"),
        ],
        &repo_root(),
    );
    assert!(run.status.success(), "run failed: {}", String::from_utf8_lossy(&run.stderr));
    let run_report = load_json(&run_out.join("run_report.json"));
    assert_precision_refusal_metrics(
        run_report.get("front_end_metrics").expect("front_end_metrics present"),
        5_000,
        2.0,
    );

    fs::remove_dir_all(&temp).expect("remove temp dir");
}

#[test]
fn inspect_reports_clipping_for_signed_16bit_capture() {
    let temp = temp_dir_path("inspect_iq16_clipping");
    fs::create_dir_all(&temp).expect("create temp dir");

    let iq_path = temp.join("clipped.iq16");
    write_clipped_iq16_capture(&iq_path, 128, 8);
    let sidecar_path = temp.join("clipped.sidecar.toml");
    write_raw_iq_sidecar_with_format_and_sample_rate(&sidecar_path, "iq16_le", 5_000_000.0);

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
            "--report",
            "json",
            "--out",
            out_dir.to_str().expect("out dir"),
        ],
        &repo_root(),
    );

    assert!(output.status.success(), "inspect failed: {}", String::from_utf8_lossy(&output.stderr));
    let report = load_json(&out_dir.join("inspect_report.json"));
    let metrics = report.get("front_end_metrics").expect("front_end_metrics present");
    assert_precision_refusal_metrics(metrics, 128, 6.25);

    fs::remove_dir_all(&temp).expect("remove temp dir");
}

#[test]
fn acquire_reports_signal_outside_search_range_for_wrong_if_capture() {
    let temp = temp_dir_path("acquire_wrong_if");
    fs::create_dir_all(&temp).expect("create temp dir");

    let iq_path = temp.join("wrong-if.cf32");
    write_synthetic_cf32_capture_with_signal_if(&iq_path, 4_092_000.0, 1_252_000.0, 0.010);
    let sidecar_path = temp.join("wrong-if.sidecar.toml");
    write_raw_iq_sidecar_with_format_sample_rate_and_if(
        &sidecar_path,
        "cf32_le",
        4_092_000.0,
        1_250_000.0,
    );

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
            "11",
            "--top",
            "8",
            "--doppler-search-hz",
            "1500",
            "--doppler-step-hz",
            "250",
            "--report",
            "json",
            "--out",
            out_dir.to_str().expect("out dir"),
        ],
        &repo_root(),
    );

    assert!(output.status.success(), "acquire failed: {}", String::from_utf8_lossy(&output.stderr));
    let report = load_json(&out_dir.join("acquire_report.json"));
    let row = report
        .get("results")
        .and_then(Value::as_array)
        .and_then(|rows| rows.first())
        .expect("acquire result row");
    assert_eq!(row.get("hypothesis").and_then(Value::as_str), Some("rejected"));
    assert!(row
        .get("selection_reason")
        .and_then(Value::as_str)
        .expect("selection_reason")
        .contains("signal_outside_search_range"));

    fs::remove_dir_all(&temp).expect("remove temp dir");
}

#[test]
fn acquire_table_reports_search_range_rejection_for_wrong_if_capture() {
    let temp = temp_dir_path("acquire_wrong_if_table");
    fs::create_dir_all(&temp).expect("create temp dir");

    let iq_path = temp.join("wrong-if.cf32");
    write_synthetic_cf32_capture_with_signal_if(&iq_path, 4_092_000.0, 1_252_000.0, 0.010);
    let sidecar_path = temp.join("wrong-if.sidecar.toml");
    write_raw_iq_sidecar_with_format_sample_rate_and_if(
        &sidecar_path,
        "cf32_le",
        4_092_000.0,
        1_250_000.0,
    );

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
            "11",
            "--top",
            "8",
            "--doppler-search-hz",
            "1500",
            "--doppler-step-hz",
            "250",
            "--report",
            "table",
        ],
        &repo_root(),
    );

    assert!(output.status.success(), "acquire failed: {}", String::from_utf8_lossy(&output.stderr));
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(stdout.contains("Hypothesis"), "stdout={stdout}");
    assert!(stdout.contains("Reason"), "stdout={stdout}");
    assert!(stdout.contains("rejected"), "stdout={stdout}");
    assert!(stdout.contains("signal_outside_search_range"), "stdout={stdout}");

    fs::remove_dir_all(&temp).expect("remove temp dir");
}

#[test]
fn track_reports_sample_rate_mismatch_for_wrong_sample_rate_capture() {
    let temp = temp_dir_path("track_wrong_sample_rate");
    fs::create_dir_all(&temp).expect("create temp dir");

    let iq_path = temp.join("wrong-sample-rate.iq8");
    write_synthetic_iq8_capture_with_signal_if_and_duration_s(&iq_path, 5_000_000.0, 0.0, 0.012);
    let sidecar_path = temp.join("wrong-sample-rate.sidecar.toml");
    write_raw_iq_sidecar_with_format_sample_rate_and_if(&sidecar_path, "iq8", 5_050_000.0, 0.0);

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
            "11",
            "--doppler-search-hz",
            "1500",
            "--doppler-step-hz",
            "250",
            "--report",
            "json",
            "--out",
            out_dir.to_str().expect("out dir"),
        ],
        &repo_root(),
    );

    assert!(output.status.success(), "track failed: {}", String::from_utf8_lossy(&output.stderr));
    let report = load_json(&out_dir.join("track_report.json"));
    let rows = report.get("epochs").and_then(Value::as_array).expect("track epoch rows");
    assert!(
        rows.iter().any(|row| {
            row.get("lock_state_reason")
                .and_then(Value::as_str)
                .is_some_and(|reason| reason == "sample_rate_mismatch")
        }),
        "rows={rows:?}"
    );

    fs::remove_dir_all(&temp).expect("remove temp dir");
}

#[test]
fn raw_iq_commands_flag_all_zero_input_as_zero_signal() {
    let temp = temp_dir_path("zero_signal_raw_iq");
    fs::create_dir_all(&temp).expect("create temp dir");

    let iq_path = temp.join("zeros.iq8");
    write_constant_iq8_capture(&iq_path, 0, 0, 5_000);
    let sidecar_path = temp.join("zeros.sidecar.toml");
    write_raw_iq_sidecar(&sidecar_path);

    let acquire_out = temp.join("acquire-out");
    let acquire = run_bijux(
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
            acquire_out.to_str().expect("out dir"),
        ],
        &repo_root(),
    );
    assert!(
        acquire.status.success(),
        "acquire failed: {}",
        String::from_utf8_lossy(&acquire.stderr)
    );
    let acquire_report = load_json(&acquire_out.join("acquire_report.json"));
    assert_all_zero_metrics(
        acquire_report.get("front_end_metrics").expect("front_end_metrics present"),
        5_000,
    );
    let acquire_results =
        acquire_report.get("results").and_then(Value::as_array).expect("acquire results");
    assert_eq!(acquire_results.len(), 1);
    assert_eq!(acquire_results[0].get("peak").and_then(Value::as_f64), Some(0.0));
    assert_eq!(acquire_results[0].get("peak_mean_ratio").and_then(Value::as_f64), Some(0.0));
    assert_eq!(acquire_results[0].get("peak_second_ratio").and_then(Value::as_f64), Some(0.0));

    let track_out = temp.join("track-out");
    let track = run_bijux(
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
            track_out.to_str().expect("out dir"),
        ],
        &repo_root(),
    );
    assert!(track.status.success(), "track failed: {}", String::from_utf8_lossy(&track.stderr));
    let track_report = load_json(&track_out.join("track_report.json"));
    assert_all_zero_metrics(
        track_report.get("front_end_metrics").expect("front_end_metrics present"),
        5_000,
    );
    assert_eq!(track_report.get("epochs").and_then(Value::as_array).map(Vec::len), Some(0));

    let inspect_out = temp.join("inspect-out");
    let inspect = run_bijux(
        &[
            "gnss",
            "inspect",
            "--unregistered-dataset",
            "--file",
            iq_path.to_str().expect("iq path"),
            "--sidecar",
            sidecar_path.to_str().expect("sidecar path"),
            "--max-samples",
            "5000",
            "--report",
            "json",
            "--out",
            inspect_out.to_str().expect("out dir"),
        ],
        &repo_root(),
    );
    assert!(
        inspect.status.success(),
        "inspect failed: {}",
        String::from_utf8_lossy(&inspect.stderr)
    );
    let inspect_report = load_json(&inspect_out.join("inspect_report.json"));
    assert_all_zero_metrics(
        inspect_report.get("front_end_metrics").expect("front_end_metrics present"),
        5_000,
    );

    let run_out = temp.join("run-out");
    let run = run_bijux(
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
            run_out.to_str().expect("out dir"),
        ],
        &repo_root(),
    );
    assert!(run.status.success(), "run failed: {}", String::from_utf8_lossy(&run.stderr));
    let run_report = load_json(&run_out.join("run_report.json"));
    assert_all_zero_metrics(
        run_report.get("front_end_metrics").expect("front_end_metrics present"),
        5_000,
    );
    assert_eq!(run_report.get("epochs").and_then(Value::as_u64), Some(1));

    fs::remove_dir_all(&temp).expect("remove temp dir");
}

#[test]
fn inspect_table_reports_zero_signal_diagnostics_for_all_zero_capture() {
    let temp = temp_dir_path("inspect_zero_signal_table");
    fs::create_dir_all(&temp).expect("create temp dir");

    let iq_path = temp.join("zeros.iq8");
    write_constant_iq8_capture(&iq_path, 0, 0, 4);
    let sidecar_path = temp.join("zeros.sidecar.toml");
    write_raw_iq_sidecar(&sidecar_path);

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
            "table",
        ],
        &repo_root(),
    );

    assert!(output.status.success(), "inspect failed: {}", String::from_utf8_lossy(&output.stderr));
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(stdout.contains("ZeroSignalDetected"), "stdout={stdout}");
    assert!(stdout.contains("ZeroSignalReason"), "stdout={stdout}");
    assert!(stdout.contains("PrecisionRefusal"), "stdout={stdout}");
    assert!(stdout.contains("no varying signal energy"), "stdout={stdout}");

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
    assert!(
        disabled.status.success(),
        "acquire without dc removal failed: {}",
        String::from_utf8_lossy(&disabled.stderr)
    );

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
    assert!(
        enabled.status.success(),
        "acquire with dc removal failed: {}",
        String::from_utf8_lossy(&enabled.stderr)
    );

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
        "dc removal reduced CLI acquisition peak quality: before={disabled_peak} after={enabled_peak}"
    );

    fs::remove_dir_all(&temp).expect("remove temp dir");
}
