use std::fs;
use std::path::Path;

use bijux_gnss_core::api::{AcqHypothesis, SampleTime, SamplesFrame, SatId, Seconds};
use bijux_gnss_receiver::api::{
    sim::{generate_l1_ca, SyntheticSignalParams},
    AcquisitionEngine, ReceiverPipelineConfig, ReceiverRuntime,
};
use bijux_gnss_signal::api::samples_per_code;
use num_complex::Complex;
use serde::Deserialize;

#[derive(Debug, Deserialize)]
struct AcqFixture {
    id: String,
    kind: String,
    sat: SatId,
    doppler_hz: f64,
    code_phase_chips: f64,
    cn0_db_hz: f32,
    seed: u64,
    search_hz: i32,
    step_hz: i32,
    coherent_ms: u32,
    noncoherent: u32,
    expected_hypothesis: Option<String>,
    expect_not_accepted: Option<bool>,
}

#[test]
fn acquisition_fixtures_are_deterministic_with_expected_hypotheses() {
    let fixtures = load_fixture_paths();
    for path in fixtures {
        let fixture = load_fixture(&path);
        let config = ReceiverPipelineConfig::default();
        let frame = frame_from_fixture(&config, &fixture);

        let acquisition = AcquisitionEngine::new(config.clone(), ReceiverRuntime::default())
            .with_doppler(fixture.search_hz, fixture.step_hz);
        let run_a = acquisition.run_fft_topn_with_explain(
            &frame,
            &[fixture.sat],
            4,
            fixture.coherent_ms,
            fixture.noncoherent,
        );
        let run_b = acquisition.run_fft_topn_with_explain(
            &frame,
            &[fixture.sat],
            4,
            fixture.coherent_ms,
            fixture.noncoherent,
        );

        let json_a = serde_json::to_string(&run_a.results).expect("serialize results a");
        let json_b = serde_json::to_string(&run_b.results).expect("serialize results b");
        assert_eq!(json_a, json_b, "fixture {} must be deterministic", fixture.id);

        let explain_a = serde_json::to_string(&run_a.explains).expect("serialize explain a");
        let explain_b = serde_json::to_string(&run_b.explains).expect("serialize explain b");
        assert_eq!(explain_a, explain_b, "fixture {} explain must be deterministic", fixture.id);

        let best = run_a
            .results
            .first()
            .and_then(|candidates| candidates.first())
            .expect("at least one acquisition candidate");

        if let Some(expected) = &fixture.expected_hypothesis {
            let expected = parse_hypothesis(expected);
            assert_eq!(
                best.hypothesis.to_string(),
                expected.to_string(),
                "fixture {} hypothesis mismatch",
                fixture.id
            );
        }
        if fixture.expect_not_accepted.unwrap_or(false) {
            assert_ne!(
                best.hypothesis.to_string(),
                AcqHypothesis::Accepted.to_string(),
                "fixture {} must not produce accepted hypothesis",
                fixture.id
            );
        }
    }
}

#[test]
fn acquisition_explain_artifact_contains_ranked_rationale() {
    let fixture = load_fixture(&fixture_root().join("fixture_synthetic_clean.json"));
    let config = ReceiverPipelineConfig::default();
    let frame = frame_from_fixture(&config, &fixture);
    let acquisition = AcquisitionEngine::new(config, ReceiverRuntime::default())
        .with_doppler(fixture.search_hz, fixture.step_hz);
    let run = acquisition.run_fft_topn_with_explain(
        &frame,
        &[fixture.sat],
        4,
        fixture.coherent_ms,
        fixture.noncoherent,
    );

    assert_eq!(run.explains.len(), 1);
    let explain = &run.explains[0];
    assert_eq!(explain.sat, fixture.sat);
    assert!(explain.candidate_count >= 1);
    assert!(!explain.selected_reason.is_empty());
    assert!(
        explain
            .candidates
            .iter()
            .any(|candidate| candidate.rank == 1 && !candidate.reason.is_empty()),
        "rank-1 candidate must include explanation reason"
    );
}

fn frame_from_fixture(config: &ReceiverPipelineConfig, fixture: &AcqFixture) -> SamplesFrame {
    let ms = (fixture.coherent_ms * fixture.noncoherent).max(1) as f64;
    let duration_s = ms / 1000.0;
    match fixture.kind.as_str() {
        "synthetic" => generate_l1_ca(
            config,
            SyntheticSignalParams {
                sat: fixture.sat,
                doppler_hz: fixture.doppler_hz,
                code_phase_chips: fixture.code_phase_chips,
                carrier_phase_rad: 0.0,
                cn0_db_hz: fixture.cn0_db_hz,
                data_bit_flip: false,
            },
            fixture.seed,
            duration_s,
        ),
        "zeros" => {
            let samples = samples_per_code(
                config.sampling_freq_hz,
                config.code_freq_basis_hz,
                config.code_length,
            ) * (fixture.coherent_ms * fixture.noncoherent).max(1) as usize;
            SamplesFrame::new(
                SampleTime { sample_index: 0, sample_rate_hz: config.sampling_freq_hz },
                Seconds(1.0 / config.sampling_freq_hz),
                vec![Complex::new(0.0, 0.0); samples],
            )
        }
        other => panic!("unsupported fixture kind: {other}"),
    }
}

fn parse_hypothesis(value: &str) -> AcqHypothesis {
    match value {
        "accepted" => AcqHypothesis::Accepted,
        "ambiguous" => AcqHypothesis::Ambiguous,
        "rejected" => AcqHypothesis::Rejected,
        "deferred" => AcqHypothesis::Deferred,
        _ => panic!("unsupported hypothesis {value}"),
    }
}

fn fixture_root() -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data/acquisition")
}

fn load_fixture_paths() -> Vec<std::path::PathBuf> {
    let mut paths = fs::read_dir(fixture_root())
        .expect("read fixture directory")
        .filter_map(|entry| entry.ok().map(|e| e.path()))
        .filter(|path| path.extension().is_some_and(|ext| ext == "json"))
        .collect::<Vec<_>>();
    paths.sort();
    paths
}

fn load_fixture(path: &Path) -> AcqFixture {
    let raw = fs::read_to_string(path).expect("read fixture file");
    serde_json::from_str(&raw).expect("parse fixture")
}
