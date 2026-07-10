#![allow(missing_docs)]

use bijux_gnss_infra::api::core::{
    ArtifactHeaderV1, Chips, Constellation, Cycles, Epoch, Hertz, ReceiverSampleTrace, SatId,
    TrackEpoch, TrackEpochV1,
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

fn artifact_header() -> ArtifactHeaderV1 {
    ArtifactHeaderV1 {
        schema_version: 1,
        producer: "bijux-gnss-cli-test".to_string(),
        producer_version: "0.1.0".to_string(),
        created_at_unix_ms: 1,
        git_sha: "test".to_string(),
        config_hash: "fixture".to_string(),
        dataset_id: None,
        toolchain: "rustc test".to_string(),
        features: Vec::new(),
        deterministic: true,
        git_dirty: false,
    }
}

fn set_bits(data: &mut u32, start: usize, len: usize, value: u32) {
    let shift = 24 - (start - 1) - len;
    let mask = ((1_u32 << len) - 1) << shift;
    *data &= !mask;
    *data |= (value << shift) & mask;
}

fn encode_word(data: u32, prev_d29: u8, prev_d30: u8) -> [u8; 30] {
    let mut bits = [0_u8; 30];
    for (i, bit) in bits.iter_mut().enumerate().take(24) {
        let shift = 23 - i;
        *bit = ((data >> shift) & 1) as u8;
    }
    let parity = compute_parity(&bits[..24], prev_d29, prev_d30);
    bits[24] = parity.0;
    bits[25] = parity.1;
    bits[26] = parity.2;
    bits[27] = parity.3;
    bits[28] = parity.4;
    bits[29] = parity.5;
    if prev_d30 == 1 {
        for bit in &mut bits {
            *bit = 1 - *bit;
        }
    }
    bits
}

fn compute_parity(data_bits: &[u8], d29_star: u8, d30_star: u8) -> (u8, u8, u8, u8, u8, u8) {
    let d = |i: usize| -> u8 { data_bits[i - 1] };
    let p1 = d(1)
        ^ d(2)
        ^ d(3)
        ^ d(5)
        ^ d(6)
        ^ d(10)
        ^ d(11)
        ^ d(12)
        ^ d(13)
        ^ d(14)
        ^ d(15)
        ^ d(17)
        ^ d(18)
        ^ d(20)
        ^ d(23)
        ^ d(24)
        ^ d29_star;
    let p2 = d(2)
        ^ d(3)
        ^ d(4)
        ^ d(6)
        ^ d(7)
        ^ d(11)
        ^ d(12)
        ^ d(13)
        ^ d(14)
        ^ d(15)
        ^ d(16)
        ^ d(18)
        ^ d(19)
        ^ d(21)
        ^ d(24)
        ^ d29_star
        ^ d30_star;
    let p3 = d(1)
        ^ d(3)
        ^ d(4)
        ^ d(5)
        ^ d(7)
        ^ d(8)
        ^ d(12)
        ^ d(13)
        ^ d(14)
        ^ d(15)
        ^ d(16)
        ^ d(17)
        ^ d(19)
        ^ d(20)
        ^ d(22)
        ^ d29_star
        ^ d30_star;
    let p4 = d(1)
        ^ d(2)
        ^ d(4)
        ^ d(5)
        ^ d(6)
        ^ d(8)
        ^ d(9)
        ^ d(13)
        ^ d(14)
        ^ d(15)
        ^ d(16)
        ^ d(17)
        ^ d(18)
        ^ d(20)
        ^ d(21)
        ^ d(23)
        ^ d30_star;
    let p5 = d(1)
        ^ d(2)
        ^ d(3)
        ^ d(5)
        ^ d(6)
        ^ d(7)
        ^ d(9)
        ^ d(10)
        ^ d(14)
        ^ d(15)
        ^ d(16)
        ^ d(17)
        ^ d(18)
        ^ d(19)
        ^ d(21)
        ^ d(22)
        ^ d(24)
        ^ d29_star;
    let p6 = d(2)
        ^ d(3)
        ^ d(4)
        ^ d(6)
        ^ d(7)
        ^ d(8)
        ^ d(10)
        ^ d(11)
        ^ d(15)
        ^ d(16)
        ^ d(17)
        ^ d(18)
        ^ d(19)
        ^ d(20)
        ^ d(22)
        ^ d(23)
        ^ d(24)
        ^ d30_star;
    (p1, p2, p3, p4, p5, p6)
}

fn encode_subframe(subframe_id: u8, tow_count: u32) -> Vec<i8> {
    encode_subframe_with_how(subframe_id, tow_count, false, false)
}

fn encode_subframe_with_how(
    subframe_id: u8,
    tow_count: u32,
    alert: bool,
    anti_spoof: bool,
) -> Vec<i8> {
    let mut tlm = 0_u32;
    set_bits(&mut tlm, 1, 8, 0x8B);

    let mut how = 0_u32;
    set_bits(&mut how, 1, 17, tow_count);
    set_bits(&mut how, 18, 1, u32::from(alert));
    set_bits(&mut how, 19, 1, u32::from(anti_spoof));
    set_bits(&mut how, 20, 3, subframe_id as u32);

    let mut words = vec![tlm, how];
    for offset in 0..8_u32 {
        words.push(0x012345 + offset * 0x010101);
    }

    let mut prev_d29 = 0_u8;
    let mut prev_d30 = 0_u8;
    let mut bits = Vec::with_capacity(300);
    for data in words {
        let encoded = encode_word(data, prev_d29, prev_d30);
        prev_d29 = encoded[28];
        prev_d30 = encoded[29];
        bits.extend(encoded.into_iter().map(|bit| if bit == 1 { 1 } else { -1 }));
    }
    bits
}

fn write_track_artifact_from_bits(path: &Path, sat: SatId, prompt_offset_ms: usize, bits: &[i8]) {
    let mut rows = Vec::with_capacity(prompt_offset_ms + bits.len() * 20);
    let sample_rate_hz = 4_092_000.0;
    for epoch_idx in 0..prompt_offset_ms {
        let sample_index = epoch_idx as u64 * 4_092;
        rows.push(
            serde_json::to_string(&TrackEpochV1 {
                header: artifact_header(),
                payload: TrackEpoch {
                    epoch: Epoch { index: epoch_idx as u64 },
                    sample_index,
                    source_time: ReceiverSampleTrace::from_sample_index(sample_index, sample_rate_hz),
                    sat,
                    prompt_i: 0.25,
                    prompt_q: 0.0,
                    early_i: 0.0,
                    early_q: 0.0,
                    late_i: 0.0,
                    late_q: 0.0,
                    carrier_hz: Hertz(120.0),
                    carrier_phase_cycles: Cycles(0.0),
                    code_rate_hz: Hertz(1_023_000.0),
                    code_phase_samples: Chips(144.0),
                    lock: true,
                    cn0_dbhz: 48.0,
                    pll_lock: true,
                    dll_lock: true,
                    fll_lock: true,
                    cycle_slip: false,
                    nav_bit_lock: true,
                    navigation_bit_sign: None,
                    dll_err: 0.0,
                    pll_err: 0.0,
                    fll_err: 0.0,
                    anti_false_lock: false,
                    cycle_slip_reason: None,
                    lock_state: "tracking".to_string(),
                    lock_state_reason: None,
                    channel_id: Some(0),
                    channel_uid: "Gps-12-ch00".to_string(),
                    tracking_provenance: "fixture".to_string(),
                    tracking_assumptions: None,
                    tracking_uncertainty: None,
                    processing_ms: None,
                },
            })
            .expect("serialize prompt offset row"),
        );
    }

    for (bit_index, bit) in bits.iter().enumerate() {
        for prompt_index in 0..20_usize {
            let epoch_idx = prompt_offset_ms + bit_index * 20 + prompt_index;
            let sample_index = epoch_idx as u64 * 4_092;
            rows.push(
                serde_json::to_string(&TrackEpochV1 {
                    header: artifact_header(),
                    payload: TrackEpoch {
                        epoch: Epoch { index: epoch_idx as u64 },
                        sample_index,
                        source_time: ReceiverSampleTrace::from_sample_index(
                            sample_index,
                            sample_rate_hz,
                        ),
                        sat,
                        prompt_i: *bit as f32,
                        prompt_q: 0.0,
                        early_i: 0.0,
                        early_q: 0.0,
                        late_i: 0.0,
                        late_q: 0.0,
                        carrier_hz: Hertz(120.0),
                        carrier_phase_cycles: Cycles(0.0),
                        code_rate_hz: Hertz(1_023_000.0),
                        code_phase_samples: Chips(144.0),
                        lock: true,
                        cn0_dbhz: 48.0,
                        pll_lock: true,
                        dll_lock: true,
                        fll_lock: true,
                        cycle_slip: false,
                        nav_bit_lock: true,
                        navigation_bit_sign: Some(*bit),
                        dll_err: 0.0,
                        pll_err: 0.0,
                        fll_err: 0.0,
                        anti_false_lock: false,
                        cycle_slip_reason: None,
                        lock_state: "tracking".to_string(),
                        lock_state_reason: None,
                        channel_id: Some(0),
                        channel_uid: "Gps-12-ch00".to_string(),
                        tracking_provenance: "fixture".to_string(),
                        tracking_assumptions: None,
                        tracking_uncertainty: None,
                        processing_ms: None,
                    },
                })
                .expect("serialize navigation bit row"),
            );
        }
    }

    fs::write(path, rows.join("\n")).expect("write wrapped track artifact");
}

#[test]
fn nav_decode_recovers_navigation_bit_signs_from_track_artifact() {
    let repo = repo_root();
    let temp = temp_dir_path("nav_decode_bits");
    fs::create_dir_all(&temp).expect("create temp dir");
    let scenario_path = temp.join("nav_decode_bits.toml");
    fs::write(
        &scenario_path,
        r#"id = "nav_decode_bits"
sample_rate_hz = 4092000.0
intermediate_freq_hz = 0.0
duration_s = 0.080
seed = 24072026

[[satellites]]
sat = { constellation = "Gps", prn = 12 }
doppler_hz = 120.0
code_phase_chips = 144.375
carrier_phase_rad = 0.3
cn0_db_hz = 52.0
data_bit_flip = true
"#,
    )
    .expect("write scenario");

    let export_dir = temp.join("export");
    fs::create_dir_all(&export_dir).expect("create export dir");
    let export_output = run_bijux(
        &[
            "gnss",
            "export-synthetic-iq",
            "--scenario",
            scenario_path.to_str().expect("scenario path"),
            "--report",
            "json",
            "--out",
            export_dir.to_str().expect("export dir"),
        ],
        &repo,
    );
    assert!(
        export_output.status.success(),
        "export-synthetic-iq failed: {}",
        String::from_utf8_lossy(&export_output.stderr)
    );

    let artifact_dir = export_dir.join("artifacts");
    let iq_path = artifact_dir.join("nav_decode_bits.iq16");
    let sidecar_path = artifact_dir.join("nav_decode_bits.sidecar.toml");

    let track_dir = temp.join("track");
    fs::create_dir_all(&track_dir).expect("create track dir");
    let track_output = run_bijux(
        &[
            "gnss",
            "track",
            "--unregistered-dataset",
            "--file",
            iq_path.to_str().expect("iq path"),
            "--sidecar",
            sidecar_path.to_str().expect("sidecar path"),
            "--config",
            "configs/receiver_low_rate.toml",
            "--prn",
            "12",
            "--report",
            "json",
            "--out",
            track_dir.to_str().expect("track dir"),
        ],
        &repo,
    );
    assert!(
        track_output.status.success(),
        "track failed: {}",
        String::from_utf8_lossy(&track_output.stderr)
    );

    let nav_dir = temp.join("nav");
    fs::create_dir_all(&nav_dir).expect("create nav dir");
    let track_artifact = track_dir.join("artifacts").join("track.jsonl");
    let nav_output = run_bijux(
        &[
            "gnss",
            "nav",
            "decode",
            "--unregistered-dataset",
            "--track",
            track_artifact.to_str().expect("track artifact"),
            "--prn",
            "12",
            "--config",
            "configs/receiver_low_rate.toml",
            "--report",
            "json",
            "--out",
            nav_dir.to_str().expect("nav dir"),
        ],
        &repo,
    );
    assert!(
        nav_output.status.success(),
        "nav decode failed: {}",
        String::from_utf8_lossy(&nav_output.stderr)
    );

    let report: Value = serde_json::from_str(
        &fs::read_to_string(nav_dir.join("nav_decode_report.json")).expect("read nav report"),
    )
    .expect("parse nav report");
    let bit_start_ms = report["bit_start_ms"].as_u64().expect("bit_start_ms");
    let bit_signs = report["bit_signs"].as_array().expect("bit_signs");

    assert_eq!(report["sat"]["prn"], 12);
    assert!(bit_start_ms < 20, "bit_start_ms={bit_start_ms}");
    assert!(bit_signs.len() >= 3, "report={report}");
    assert!(
        bit_signs.iter().any(|sign| sign.as_i64() == Some(1))
            && bit_signs.iter().any(|sign| sign.as_i64() == Some(-1)),
        "recovered navigation bits did not contain both polarities in the alternating-bit scenario: report={report}"
    );

    fs::remove_dir_all(&temp).expect("remove temp dir");
}

#[test]
fn nav_decode_reports_aligned_lnav_subframes_from_wrapped_track_artifact() {
    let repo = repo_root();
    let temp = temp_dir_path("nav_decode_aligned_subframes");
    fs::create_dir_all(&temp).expect("create temp dir");
    let sat = SatId { constellation: Constellation::Gps, prn: 12 };
    let mut bits = encode_subframe(1, 1);
    bits.extend(encode_subframe_with_how(2, 2, true, true));

    let track_path = temp.join("track.jsonl");
    write_track_artifact_from_bits(&track_path, sat, 7, &bits);

    let nav_dir = temp.join("nav");
    fs::create_dir_all(&nav_dir).expect("create nav dir");
    let nav_output = run_bijux(
        &[
            "gnss",
            "nav",
            "decode",
            "--unregistered-dataset",
            "--track",
            track_path.to_str().expect("track path"),
            "--prn",
            "12",
            "--config",
            "configs/receiver_low_rate.toml",
            "--report",
            "json",
            "--out",
            nav_dir.to_str().expect("nav dir"),
        ],
        &repo,
    );
    assert!(
        nav_output.status.success(),
        "nav decode failed: {}",
        String::from_utf8_lossy(&nav_output.stderr)
    );

    let report: Value = serde_json::from_str(
        &fs::read_to_string(nav_dir.join("nav_decode_report.json")).expect("read nav report"),
    )
    .expect("parse nav report");
    let aligned_subframes = report["aligned_subframes"].as_array().expect("aligned_subframes");
    let decoded_subframes = report["decoded_subframes"].as_array().expect("decoded_subframes");

    assert_eq!(report["bit_start_ms"], 7);
    assert_eq!(report["preamble_hits"], 2);
    assert_eq!(aligned_subframes.len(), 2, "report={report}");
    assert_eq!(decoded_subframes.len(), 2, "report={report}");
    assert_eq!(aligned_subframes[0]["start_bit_index"], 0);
    assert_eq!(aligned_subframes[0]["start_prompt_index"], 7);
    assert_eq!(aligned_subframes[0]["word_count"], 10);
    assert_eq!(aligned_subframes[1]["start_bit_index"], 300);
    assert_eq!(aligned_subframes[1]["start_prompt_index"], 6007);
    assert_eq!(aligned_subframes[1]["end_prompt_index_exclusive"], 12007);
    assert_eq!(decoded_subframes[0]["tlm"]["preamble"], 139);
    assert_eq!(decoded_subframes[0]["how"]["tow_count"], 1);
    assert_eq!(decoded_subframes[0]["how"]["subframe_id"], 1);
    assert_eq!(decoded_subframes[0]["how"]["alert"], false);
    assert_eq!(decoded_subframes[0]["how"]["anti_spoof"], false);
    assert_eq!(decoded_subframes[1]["how"]["tow_count"], 2);
    assert_eq!(decoded_subframes[1]["how"]["subframe_id"], 2);
    assert_eq!(decoded_subframes[1]["how"]["alert"], true);
    assert_eq!(decoded_subframes[1]["how"]["anti_spoof"], true);
    assert_eq!(decoded_subframes[1]["word_parity_ok"].as_array().map(|items| items.len()), Some(10));

    fs::remove_dir_all(&temp).expect("remove temp dir");
}
