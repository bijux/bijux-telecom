use std::fs;
use std::path::Path;

use bijux_gnss_core::api::{AcqHypothesis, SampleTime, SamplesFrame, SatId, Seconds};
use bijux_gnss_receiver::api::{
    sim::{generate_l1_ca, generate_l1_ca_multi, SyntheticScenario, SyntheticSignalParams},
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
    signal_sat: Option<SatId>,
    requested_sats: Option<Vec<SatId>>,
    signals: Option<Vec<FixtureSignal>>,
    doppler_hz: f64,
    signal_intermediate_freq_hz: Option<f64>,
    code_phase_chips: f64,
    cn0_db_hz: f32,
    seed: u64,
    search_hz: i32,
    step_hz: i32,
    coherent_ms: u32,
    noncoherent: u32,
    acquisition_peak_mean_threshold: Option<f32>,
    expected_hypothesis: Option<String>,
    expected_selected_reason: Option<String>,
    expect_not_accepted: Option<bool>,
}

#[derive(Debug, Deserialize)]
struct FixtureSignal {
    sat: SatId,
    doppler_hz: f64,
    code_phase_chips: f64,
    cn0_db_hz: f32,
}

#[test]
fn acquisition_fixtures_are_deterministic_with_expected_hypotheses() {
    let fixtures = load_fixture_paths();
    for path in fixtures {
        let fixture = load_fixture(&path);
        let mut config = ReceiverPipelineConfig::default();
        if let Some(acquisition_peak_mean_threshold) = fixture.acquisition_peak_mean_threshold {
            config.acquisition_peak_mean_threshold = acquisition_peak_mean_threshold;
        }
        let frame = frame_from_fixture(&config, &fixture);
        let requested_sats = requested_sats(&fixture);

        let acquisition = AcquisitionEngine::new(config.clone(), ReceiverRuntime::default())
            .with_doppler(fixture.search_hz, fixture.step_hz);
        let run_a = acquisition.run_fft_topn_with_explain(
            &frame,
            &requested_sats,
            4,
            fixture.coherent_ms,
            fixture.noncoherent,
        );
        let run_b = acquisition.run_fft_topn_with_explain(
            &frame,
            &requested_sats,
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

        let best = result_for_sat(&run_a.results, fixture.sat)
            .first()
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
        if let Some(expected_selected_reason) = &fixture.expected_selected_reason {
            let explain =
                explain_for_sat(&run_a.explains, fixture.sat).expect("acquisition explain for sat");
            assert_eq!(
                explain.selected_reason, *expected_selected_reason,
                "fixture {} selected reason mismatch",
                fixture.id
            );
        }
    }
}

#[test]
fn acquisition_explain_artifact_contains_ranked_rationale() {
    let fixture = load_fixture(&fixture_root().join("fixture_synthetic_clean.json"));
    let mut config = ReceiverPipelineConfig::default();
    if let Some(acquisition_peak_mean_threshold) = fixture.acquisition_peak_mean_threshold {
        config.acquisition_peak_mean_threshold = acquisition_peak_mean_threshold;
    }
    let frame = frame_from_fixture(&config, &fixture);
    let requested_sats = requested_sats(&fixture);
    let acquisition = AcquisitionEngine::new(config, ReceiverRuntime::default())
        .with_doppler(fixture.search_hz, fixture.step_hz);
    let run = acquisition.run_fft_topn_with_explain(
        &frame,
        &requested_sats,
        4,
        fixture.coherent_ms,
        fixture.noncoherent,
    );

    let explain = explain_for_sat(&run.explains, fixture.sat).expect("acquisition explain for sat");
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
        "synthetic" => {
            let mut signal_config = config.clone();
            if let Some(signal_intermediate_freq_hz) = fixture.signal_intermediate_freq_hz {
                signal_config.intermediate_freq_hz = signal_intermediate_freq_hz;
            }
            generate_l1_ca(
                &signal_config,
                SyntheticSignalParams {
                    sat: signal_sat(fixture),
                    glonass_frequency_channel: None,
                    signal_band: bijux_gnss_core::api::SignalBand::L1,
                    signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                    doppler_hz: fixture.doppler_hz,
                    code_phase_chips: fixture.code_phase_chips,
                    carrier_phase_rad: 0.0,
                    cn0_db_hz: fixture.cn0_db_hz,
                    navigation_data: false.into(),
                },
                fixture.seed,
                duration_s,
            )
        }
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
        "noise_only" => generate_l1_ca_multi(
            config,
            &SyntheticScenario {
                sample_rate_hz: config.sampling_freq_hz,
                intermediate_freq_hz: config.intermediate_freq_hz,
                receiver_clock_frequency_bias_hz: 0.0,
                duration_s,
                seed: fixture.seed,
                satellites: Vec::new(),
                ephemerides: Vec::new(),
                id: fixture.id.clone(),
            },
        ),
        "synthetic_multi" => generate_l1_ca_multi(
            config,
            &SyntheticScenario {
                sample_rate_hz: config.sampling_freq_hz,
                intermediate_freq_hz: fixture
                    .signal_intermediate_freq_hz
                    .unwrap_or(config.intermediate_freq_hz),
                receiver_clock_frequency_bias_hz: 0.0,
                duration_s,
                seed: fixture.seed,
                satellites: fixture
                    .signals()
                    .iter()
                    .map(|signal| SyntheticSignalParams {
                        sat: signal.sat,
                        glonass_frequency_channel: None,
                        signal_band: bijux_gnss_core::api::SignalBand::L1,
                        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
                        doppler_hz: signal.doppler_hz,
                        code_phase_chips: signal.code_phase_chips,
                        carrier_phase_rad: 0.0,
                        cn0_db_hz: signal.cn0_db_hz,
                        navigation_data: false.into(),
                    })
                    .collect(),
                ephemerides: Vec::new(),
                id: fixture.id.clone(),
            },
        ),
        other => panic!("unsupported fixture kind: {other}"),
    }
}

fn signal_sat(fixture: &AcqFixture) -> SatId {
    fixture.signal_sat.unwrap_or(fixture.sat)
}

fn requested_sats(fixture: &AcqFixture) -> Vec<SatId> {
    fixture.requested_sats.clone().unwrap_or_else(|| vec![fixture.sat])
}

impl AcqFixture {
    fn signals(&self) -> &[FixtureSignal] {
        self.signals.as_deref().expect("synthetic_multi fixtures must define signals")
    }
}

fn result_for_sat(
    results: &[Vec<bijux_gnss_core::api::AcqResult>],
    sat: SatId,
) -> &Vec<bijux_gnss_core::api::AcqResult> {
    results
        .iter()
        .find(|candidates| candidates.first().is_some_and(|candidate| candidate.sat == sat))
        .expect("result for requested satellite")
}

fn explain_for_sat(
    explains: &[bijux_gnss_core::api::AcqExplain],
    sat: SatId,
) -> Option<&bijux_gnss_core::api::AcqExplain> {
    explains.iter().find(|explain| explain.sat == sat)
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
        .filter(|path| {
            path.extension().is_some_and(|ext| ext == "json")
                && path
                    .file_stem()
                    .and_then(|stem| stem.to_str())
                    .is_some_and(|stem| stem.starts_with("fixture_") || stem.starts_with("corpus_"))
        })
        .collect::<Vec<_>>();
    paths.sort();
    paths
}

fn load_fixture(path: &Path) -> AcqFixture {
    let raw = fs::read_to_string(path).expect("read fixture file");
    serde_json::from_str(&raw).expect("parse fixture")
}
