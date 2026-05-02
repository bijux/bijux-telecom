use std::collections::BTreeMap;
use std::fs;
use std::path::Path;

use bijux_gnss_core::api::{
    Chips, Constellation, Epoch, Hertz, ObservationEpochDecision, SatId, TrackEpoch,
};
use bijux_gnss_receiver::api::{
    observations_from_tracking_results, ReceiverPipelineConfig, TrackingResult,
};
use serde::Deserialize;

#[derive(Debug, Deserialize)]
struct ObservationFixture {
    id: String,
    rows: Vec<ObservationRowFixture>,
    expect: ObservationExpectFixture,
}

#[derive(Debug, Deserialize)]
struct ObservationRowFixture {
    sat_prn: u8,
    epoch_idx: u64,
    sample_index: u64,
    lock: bool,
    cn0_dbhz: f64,
    code_phase_samples: f64,
    prompt_i: f32,
    prompt_q: f32,
    carrier_hz: f64,
    lock_state: String,
}

#[derive(Debug, Deserialize)]
struct ObservationExpectFixture {
    decisions: Vec<ExpectedDecisionFixture>,
    support_classes: Vec<String>,
    uncertainty_classes: Vec<String>,
    min_rejected: usize,
    multipath_any: bool,
}

#[derive(Debug, Deserialize)]
struct ExpectedDecisionFixture {
    epoch_idx: u64,
    decision: String,
}

#[test]
fn observation_regression_fixtures_cover_low_cn0_partial_visibility_and_multipath_like() {
    let config = ReceiverPipelineConfig::default();
    for path in load_fixture_paths() {
        let fixture = load_fixture(&path);
        let tracks = tracks_from_fixture_rows(&fixture.rows, &config);
        let report = observations_from_tracking_results(&config, &tracks, 10);

        let by_epoch = report
            .output
            .iter()
            .map(|epoch| (epoch.epoch_idx, epoch.decision))
            .collect::<BTreeMap<_, _>>();
        for expected in &fixture.expect.decisions {
            let actual = by_epoch.get(&expected.epoch_idx).copied().expect("expected epoch");
            assert_eq!(
                actual,
                parse_decision(&expected.decision),
                "fixture {} decision mismatch at epoch {}",
                fixture.id,
                expected.epoch_idx
            );
        }

        let support = report
            .output
            .iter()
            .flat_map(|epoch| epoch.sats.iter())
            .map(|sat| sat.metadata.observation_support_class.clone())
            .collect::<Vec<_>>();
        for expected in &fixture.expect.support_classes {
            assert!(
                support.iter().any(|value| value == expected),
                "fixture {} missing support class {}",
                fixture.id,
                expected
            );
        }

        let uncertainty = report
            .output
            .iter()
            .flat_map(|epoch| epoch.sats.iter())
            .map(|sat| sat.metadata.observation_uncertainty_class.clone())
            .collect::<Vec<_>>();
        for expected in &fixture.expect.uncertainty_classes {
            assert!(
                uncertainty.iter().any(|value| value == expected),
                "fixture {} missing uncertainty class {}",
                fixture.id,
                expected
            );
        }

        let rejected = report
            .output
            .iter()
            .flat_map(|epoch| epoch.sats.iter())
            .filter(|sat| !sat.observation_reject_reasons.is_empty())
            .count();
        assert!(
            rejected >= fixture.expect.min_rejected,
            "fixture {} rejected count {} is below {}",
            fixture.id,
            rejected,
            fixture.expect.min_rejected
        );

        let multipath_any = report
            .output
            .iter()
            .flat_map(|epoch| epoch.sats.iter())
            .any(|sat| sat.multipath_suspect);
        assert_eq!(
            multipath_any, fixture.expect.multipath_any,
            "fixture {} multipath expectation mismatch",
            fixture.id
        );
    }
}

fn tracks_from_fixture_rows(
    rows: &[ObservationRowFixture],
    config: &ReceiverPipelineConfig,
) -> Vec<TrackingResult> {
    let mut by_sat: BTreeMap<u8, Vec<TrackEpoch>> = BTreeMap::new();
    for row in rows {
        let sat = SatId { constellation: Constellation::Gps, prn: row.sat_prn };
        by_sat.entry(row.sat_prn).or_default().push(TrackEpoch {
            epoch: Epoch { index: row.epoch_idx },
            sample_index: row.sample_index,
            sat,
            prompt_i: row.prompt_i,
            prompt_q: row.prompt_q,
            carrier_hz: Hertz(row.carrier_hz),
            code_rate_hz: Hertz(config.code_freq_basis_hz),
            code_phase_samples: Chips(row.code_phase_samples),
            lock: row.lock,
            cn0_dbhz: row.cn0_dbhz,
            pll_lock: row.lock,
            dll_lock: row.lock,
            fll_lock: row.lock,
            cycle_slip: false,
            nav_bit_lock: false,
            dll_err: 0.0,
            pll_err: 0.0,
            fll_err: 0.0,
            anti_false_lock: false,
            cycle_slip_reason: None,
            lock_state: row.lock_state.clone(),
            lock_state_reason: None,
            ..TrackEpoch::default()
        });
    }
    by_sat
        .into_iter()
        .map(|(prn, mut epochs)| {
            epochs.sort_by_key(|epoch| epoch.epoch.index);
            let sat = SatId { constellation: Constellation::Gps, prn };
            TrackingResult {
                sat,
                carrier_hz: 0.0,
                code_phase_samples: 0.0,
                acquisition_hypothesis: "accepted".to_string(),
                acquisition_score: 1.0,
                acquisition_code_phase_samples: 0,
                acquisition_carrier_hz: 0.0,
                acq_to_track_state: "tracking".to_string(),
                epochs,
                transitions: Vec::new(),
            }
        })
        .collect()
}

fn parse_decision(value: &str) -> ObservationEpochDecision {
    match value {
        "accepted" => ObservationEpochDecision::Accepted,
        "rejected" => ObservationEpochDecision::Rejected,
        _ => panic!("unsupported decision {value}"),
    }
}

fn fixture_root() -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data/observations")
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

fn load_fixture(path: &Path) -> ObservationFixture {
    let raw = fs::read_to_string(path).expect("read fixture file");
    serde_json::from_str(&raw).expect("parse fixture")
}
