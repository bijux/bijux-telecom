use super::*;
use crate::engine::receiver_config::AcquisitionThresholdMode;
use crate::engine::runtime::ReceiverRuntime;
use crate::sim::synthetic::{
    generate_l1_ca, SyntheticScenario, SyntheticSignalParams, SyntheticSignalSource,
};
use bijux_gnss_core::api::{
    AcqAssistanceBounds, AcqComponentCombinationMode, AcqComponentProvenance,
    AcqComponentStatistic, Constellation, GlonassFrequencyChannel, ReceiverSampleTrace, SampleTime,
    SamplesFrame, SatId, Seconds, SignalBand, SignalComponentRole, GPS_L1_CA_CARRIER_HZ,
};
use bijux_gnss_signal::api::{
    glonass_l1_carrier_hz, sample_glonass_l1_st_code, samples_per_code, shared_path_doppler_hz,
    signal_spec_gps_l1_ca, signal_spec_gps_l5_i, SignalSource,
};

#[path = "tests/code_fft_cache.rs"]
mod code_fft_cache;
#[path = "tests/coherent_accumulation.rs"]
mod coherent_accumulation;
#[path = "tests/decision_policy.rs"]
mod decision_policy;
#[path = "tests/galileo_e1_acquisition.rs"]
mod galileo_e1_acquisition;
#[path = "tests/insufficient_frames.rs"]
mod insufficient_frames;
#[path = "tests/integration_guards.rs"]
mod integration_guards;
#[path = "tests/refinement_diagnostics.rs"]
mod refinement_diagnostics;
#[path = "tests/related_signal_follow_up.rs"]
mod related_signal_follow_up;
#[path = "tests/request_execution.rs"]
mod request_execution;
#[path = "tests/search_window_diagnostics.rs"]
mod search_window_diagnostics;

fn acquisition_component_plan_for_signal(
    sat: SatId,
    signal_band: SignalBand,
    signal_code: SignalCode,
    coherent_ms: u32,
) -> AcquisitionComponentPlan {
    acquisition_strategies_for_signal(sat, signal_band, signal_code, None, coherent_ms)
        .expect("acquisition strategies")
        .into_iter()
        .next()
        .and_then(|strategy| strategy.components.into_iter().next())
        .expect("primary acquisition component")
}

fn alternating_frame(sample_rate_hz: f64, sample_count: usize) -> SamplesFrame {
    SamplesFrame::new(
        SampleTime { sample_index: 0, sample_rate_hz },
        Seconds(1.0 / sample_rate_hz),
        (0..sample_count)
            .map(|idx| if idx % 2 == 0 { Complex::new(1.0, 0.0) } else { Complex::new(-1.0, 0.0) })
            .collect(),
    )
}

fn signal_only_frame(
    config: &ReceiverPipelineConfig,
    scenario: &SyntheticScenario,
    frame_len: usize,
) -> SamplesFrame {
    let mut source = SyntheticSignalSource::new_signal_only(config, scenario);
    source.next_frame(frame_len).expect("signal-only frame").expect("acquisition frame")
}

fn resolved_thresholds(config: &ReceiverPipelineConfig) -> ResolvedAcquisitionThresholds {
    ResolvedAcquisitionThresholds {
        peak_mean_threshold: config.acquisition_peak_mean_threshold,
        peak_second_threshold: config.acquisition_peak_second_threshold,
        provenance: AcqThresholdProvenance {
            mode: "fixed_ratio".to_string(),
            coherent_ms: config.acquisition_integration_ms,
            noncoherent: config.acquisition_noncoherent,
            doppler_search_hz: config.acquisition_doppler_search_hz,
            doppler_step_hz: config.acquisition_doppler_step_hz,
            doppler_rate_search_hz_per_s: config.acquisition_doppler_rate_search_hz_per_s,
            doppler_rate_step_hz_per_s: config.acquisition_doppler_rate_step_hz_per_s,
            peak_mean_threshold: config.acquisition_peak_mean_threshold,
            peak_second_threshold: config.acquisition_peak_second_threshold,
            false_alarm_probability: None,
            calibration_trial_count: None,
            calibration_confidence_level: None,
            calibration_false_alarm_rate: None,
            calibration_false_alarm_interval_low: None,
            calibration_false_alarm_interval_high: None,
        },
    }
}

#[test]
fn acquisition_stability_keys_are_sorted() {
    let sat = SatId { constellation: bijux_gnss_core::api::Constellation::Gps, prn: 1 };
    let mut rows = vec![
        AcqResult {
            sat,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Unknown,
            glonass_frequency_channel: None,
            source_time: ReceiverSampleTrace::default(),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: Hertz(100.0),
            doppler_rate_hz_per_s: 0.0,
            carrier_hz: Hertz(100.0),
            code_phase_samples: 10,
            peak: 10.0,
            second_peak: 2.0,
            mean: 1.0,
            peak_mean_ratio: 10.0,
            peak_second_ratio: 5.0,
            cn0_proxy: 10.0,
            score: 2.0,
            hypothesis: AcqHypothesis::Accepted,
            assumptions: None,
            evidence: Vec::new(),
            threshold_provenance: None,
            explain_selection_reason: None,
            doppler_refinement: None,
            code_phase_refinement: None,
            signal_delay_alignment: None,
            uncertainty: None,
        },
        AcqResult {
            sat,
            signal_band: SignalBand::L1,
            signal_code: SignalCode::Unknown,
            glonass_frequency_channel: None,
            source_time: ReceiverSampleTrace::default(),
            candidate_rank: 1,
            is_primary_candidate: true,
            doppler_hz: Hertz(50.0),
            doppler_rate_hz_per_s: 0.0,
            carrier_hz: Hertz(50.0),
            code_phase_samples: 20,
            peak: 10.0,
            second_peak: 2.0,
            mean: 1.0,
            peak_mean_ratio: 10.0,
            peak_second_ratio: 5.0,
            cn0_proxy: 10.0,
            score: 2.0,
            hypothesis: AcqHypothesis::Accepted,
            assumptions: None,
            evidence: Vec::new(),
            threshold_provenance: None,
            explain_selection_reason: None,
            doppler_refinement: None,
            code_phase_refinement: None,
            signal_delay_alignment: None,
            uncertainty: None,
        },
    ];
    rows.sort_by(|a, b| {
        let primary =
            b.peak_mean_ratio.partial_cmp(&a.peak_mean_ratio).unwrap_or(std::cmp::Ordering::Equal);
        if primary == std::cmp::Ordering::Equal {
            return acq_result_stability_key(a).cmp(&acq_result_stability_key(b));
        }
        primary
    });
    let keys = stable_acq_result_keys(&rows);
    assert!(keys.windows(2).all(|window| window[0] <= window[1]));
}

fn candidate_for_search_window_test(
    sat: SatId,
    carrier_hz: f64,
    peak_mean_ratio: f32,
) -> AcqResult {
    AcqResult {
        sat,
        signal_band: SignalBand::L1,
        signal_code: SignalCode::Unknown,
        glonass_frequency_channel: None,
        source_time: ReceiverSampleTrace::default(),
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(carrier_hz),
        doppler_rate_hz_per_s: 0.0,
        carrier_hz: Hertz(carrier_hz),
        code_phase_samples: 0,
        peak: peak_mean_ratio,
        second_peak: 1.0,
        mean: 1.0,
        peak_mean_ratio,
        peak_second_ratio: peak_mean_ratio,
        cn0_proxy: peak_mean_ratio,
        score: 0.0,
        hypothesis: AcqHypothesis::Deferred,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: None,
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    }
}
