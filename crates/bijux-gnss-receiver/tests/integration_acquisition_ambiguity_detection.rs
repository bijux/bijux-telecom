#![allow(missing_docs)]

use bijux_gnss_core::api::{AcqSearchSummary, Constellation, SatId};
use bijux_gnss_receiver::api::{
    sim::{generate_l1_ca_multi, SyntheticScenario, SyntheticSignalParams},
    AcquisitionEngine, ReceiverPipelineConfig, ReceiverRuntime,
};

#[test]
fn acquisition_ambiguity_harness_preserves_ranked_candidates() {
    let sat = gps_l1_ca_satellite();
    let config = ambiguity_profile();
    let frame = competing_peak_frame(
        &config,
        0.001,
        [competing_signal(sat, 0.0, 300.0, 44.0), competing_signal(sat, 250.0, 300.0, 44.0)],
        0x2407_1997,
    );
    let run = AcquisitionEngine::new(config.clone(), ReceiverRuntime::default())
        .with_doppler(config.acquisition_doppler_search_hz, config.acquisition_doppler_step_hz)
        .run_fft_topn_with_explain(&frame, &[sat], 4, 1, 1);

    assert_eq!(run.results.len(), 1);
    assert_eq!(run.explains.len(), 1);
    assert!(!run.results[0].is_empty());
    assert!(run.results[0].len() <= 4);
    assert!(!run.explains[0].selected_reason.is_empty());
    assert!(run.explains[0].candidate_count >= 1);
}

#[test]
fn acquisition_preserves_candidate_rank_metadata_for_ranked_rows() {
    let sat = gps_l1_ca_satellite();
    let config = ambiguity_profile();
    let frame = competing_peak_frame(
        &config,
        0.001,
        [competing_signal(sat, 0.0, 300.0, 44.0), competing_signal(sat, 250.0, 300.0, 44.0)],
        0x2407_1996,
    );
    let run = AcquisitionEngine::new(config.clone(), ReceiverRuntime::default())
        .with_doppler(config.acquisition_doppler_search_hz, config.acquisition_doppler_step_hz)
        .run_fft_topn_with_explain(&frame, &[sat], 4, 1, 1);

    let candidates = &run.results[0];
    assert!(candidates.len() >= 2, "{run:?}");
    assert_eq!(candidates[0].candidate_rank, 1, "{run:?}");
    assert!(candidates[0].is_primary_candidate, "{run:?}");
    assert_eq!(candidates[1].candidate_rank, 2, "{run:?}");
    assert!(!candidates[1].is_primary_candidate, "{run:?}");
    assert!(
        candidates.windows(2).all(|pair| pair[0].candidate_rank + 1 == pair[1].candidate_rank),
        "{run:?}"
    );

    let explain = &run.explains[0];
    assert_eq!(explain.candidates[0].rank, candidates[0].candidate_rank, "{run:?}");
    assert_eq!(explain.candidates[1].rank, candidates[1].candidate_rank, "{run:?}");
}

#[test]
fn acquisition_marks_comparable_competing_peaks_as_ambiguous() {
    let sat = gps_l1_ca_satellite();
    let config = ambiguity_profile();
    let frame = competing_peak_frame(
        &config,
        0.001,
        [competing_signal(sat, 0.0, 300.0, 44.0), competing_signal(sat, 0.0, 650.0, 44.0)],
        0x2407_1998,
    );
    let run = AcquisitionEngine::new(config.clone(), ReceiverRuntime::default())
        .with_doppler(config.acquisition_doppler_search_hz, config.acquisition_doppler_step_hz)
        .run_fft_topn_with_explain(&frame, &[sat], 4, 1, 1);
    let best = &run.results[0][0];
    let second = run.results[0].get(1).expect("runner-up candidate");
    let competing_peak_ratio = best.peak_mean_ratio / second.peak_mean_ratio;

    assert_eq!(best.hypothesis.to_string(), "ambiguous", "{run:?}");
    assert!(
        best.peak_second_ratio < config.acquisition_peak_second_threshold
            || competing_peak_ratio < config.acquisition_peak_second_threshold,
        "{run:?}"
    );
}

#[test]
fn acquisition_preserves_runner_up_as_ranked_alternative() {
    let sat = gps_l1_ca_satellite();
    let config = ambiguity_profile();
    let frame = competing_peak_frame(
        &config,
        0.001,
        [competing_signal(sat, 0.0, 300.0, 44.0), competing_signal(sat, 250.0, 300.0, 44.0)],
        0x2407_2001,
    );
    let run = AcquisitionEngine::new(config.clone(), ReceiverRuntime::default())
        .with_doppler(config.acquisition_doppler_search_hz, config.acquisition_doppler_step_hz)
        .run_fft_topn_with_explain(&frame, &[sat], 4, 1, 1);

    let primary = &run.results[0][0];
    let alternative = run.results[0].get(1).expect("runner-up candidate");

    assert_eq!(primary.hypothesis.to_string(), "ambiguous", "{run:?}");
    assert_eq!(alternative.hypothesis.to_string(), "rejected", "{run:?}");
    assert_eq!(alternative.score, 0.0, "{run:?}");
    assert!(
        alternative
            .explain_selection_reason
            .as_deref()
            .is_some_and(|reason| reason.starts_with("ranked_alternative:")),
        "{run:?}"
    );
}

#[test]
fn acquisition_explainability_reports_multipath_suspicion_for_delayed_secondary_peaks() {
    let sat = gps_l1_ca_satellite();
    let config = ambiguity_profile();
    let frame = competing_peak_frame(
        &config,
        0.001,
        [competing_signal(sat, 0.0, 300.0, 44.0), competing_signal(sat, 0.0, 650.0, 44.0)],
        0x2407_1999,
    );
    let run = AcquisitionEngine::new(config.clone(), ReceiverRuntime::default())
        .with_doppler(config.acquisition_doppler_search_hz, config.acquisition_doppler_step_hz)
        .run_fft_topn_with_explain(&frame, &[sat], 4, 1, 1);
    let explain = &run.explains[0];
    let selected = explain.candidates.first().expect("selected candidate");

    assert_eq!(explain.selected_reason, "multipath_suspect", "{run:?}");
    assert_eq!(selected.hypothesis.to_string(), "ambiguous", "{run:?}");
    assert!(selected.reason.contains("delayed secondary peak"), "{run:?}");
}

#[test]
fn acquisition_search_summary_counts_ambiguous_outcomes() {
    let sat = gps_l1_ca_satellite();
    let config = ambiguity_profile();
    let frame = competing_peak_frame(
        &config,
        0.001,
        [competing_signal(sat, 0.0, 300.0, 44.0), competing_signal(sat, 0.0, 650.0, 44.0)],
        0x2407_2000,
    );
    let results = AcquisitionEngine::new(config.clone(), ReceiverRuntime::default())
        .with_doppler(config.acquisition_doppler_search_hz, config.acquisition_doppler_step_hz)
        .run_fft(&frame, &[sat]);
    let summary = AcqSearchSummary::from_results(&results);

    assert_eq!(results.len(), 1);
    assert_eq!(results[0].hypothesis.to_string(), "ambiguous", "{results:?}");
    assert_eq!(summary.searched_satellites, 1);
    assert_eq!(summary.ambiguous, 1);
    assert_eq!(summary.accepted, 0);
    assert_eq!(summary.rejected, 0);
    assert_eq!(summary.deferred, 0);
}

fn ambiguity_profile() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        acquisition_doppler_search_hz: 500,
        acquisition_doppler_step_hz: 250,
        ..ReceiverPipelineConfig::default()
    }
}

fn competing_peak_frame(
    config: &ReceiverPipelineConfig,
    duration_s: f64,
    signals: [SyntheticSignalParams; 2],
    seed: u64,
) -> bijux_gnss_core::api::SamplesFrame {
    generate_l1_ca_multi(
        config,
        &SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            receiver_clock_frequency_bias_hz: 0.0,
            duration_s,
            seed,
            satellites: signals.into_iter().collect(),
            ephemerides: Vec::new(),
            id: "ambiguity_detection".to_string(),
        },
    )
}

fn competing_signal(
    sat: SatId,
    doppler_hz: f64,
    code_phase_chips: f64,
    cn0_db_hz: f32,
) -> SyntheticSignalParams {
    SyntheticSignalParams {
        sat,
        glonass_frequency_channel: None,
        doppler_hz,
        code_phase_chips,
        carrier_phase_rad: 0.0,
        cn0_db_hz,
        data_bit_flip: false,
    }
}

fn gps_l1_ca_satellite() -> SatId {
    SatId { constellation: Constellation::Gps, prn: 7 }
}
