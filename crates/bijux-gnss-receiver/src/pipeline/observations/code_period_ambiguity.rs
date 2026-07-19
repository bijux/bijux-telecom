use std::collections::HashMap;

use bijux_gnss_core::api::{
    DiagnosticEvent, DiagnosticSeverity, GpsTime, Meters, ObsEpoch, ObsSignalTiming,
    ObservationStatus, Seconds, SigId, TrackEpoch,
};

use crate::engine::receiver_config::ReceiverPipelineConfig;
use crate::pipeline::observations::labels::{observation_status_label, observation_support_label};
use crate::pipeline::observations::pseudorange_timing::code_phase_timing_from_tracking_epoch;
use crate::pipeline::observations::signal_model::observation_signal_model;

use super::status::push_observation_reject_reason;
use super::{observation_signal_key, observation_snapshot_key};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
pub(super) const CODE_PERIOD_AMBIGUITY_SEARCH_PERIODS: i64 = 2;
pub(super) const CODE_PERIOD_AMBIGUITY_EPS_S: f64 = 2.5e-7;
const CODE_PERIOD_AMBIGUITY_MAX_COHERENCE_FRACTION: f64 = 0.20;
const CODE_PERIOD_AMBIGUITY_MAX_CLOCK_BIAS_FRACTION: f64 = 0.55;
pub(super) const CODE_PERIOD_AMBIGUITY_NON_UNIQUE: &str =
    "integer_code_period_ambiguity_non_unique";
const CODE_PERIOD_AMBIGUITY_INCOHERENT: &str = "integer_code_period_ambiguity_incoherent";
const CODE_PERIOD_AMBIGUITY_UNAVAILABLE: &str = "integer_code_period_ambiguity_unavailable";

#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct CodePeriodAmbiguityInput {
    pub(super) signal_id: SigId,
    pub(super) receive_gps_time: GpsTime,
    pub(super) decoded_transmit_gps_time: GpsTime,
    pub(super) code_period_s: f64,
    pub(super) code_delay_s: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct CodePeriodCandidate {
    integer_code_periods: u64,
    signal_travel_time_s: f64,
    residual_s: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct CodePeriodAmbiguitySatellite {
    pub(super) signal_id: SigId,
    pub(super) integer_code_periods: u64,
    pub(super) signal_travel_time_s: Seconds,
    pub(super) transmit_gps_time: GpsTime,
}

#[derive(Debug, Clone, PartialEq)]
pub(super) struct CodePeriodAmbiguitySolution {
    pub(super) common_receiver_clock_bias_s: Seconds,
    pub(super) satellites: Vec<CodePeriodAmbiguitySatellite>,
}

#[derive(Debug, Clone, PartialEq)]
struct ScoredCodePeriodAmbiguitySolution {
    score_s: f64,
    non_unique_candidate_selection: bool,
    solution: CodePeriodAmbiguitySolution,
}

pub(super) fn resolve_integer_code_period_ambiguities(
    inputs: &[CodePeriodAmbiguityInput],
) -> Result<CodePeriodAmbiguitySolution, &'static str> {
    if inputs.is_empty() {
        return Err(CODE_PERIOD_AMBIGUITY_UNAVAILABLE);
    }

    let mut candidate_sets = Vec::with_capacity(inputs.len());
    let mut anchors = vec![0.0_f64];
    let mut minimum_code_period_s = f64::INFINITY;

    for input in sorted_code_period_ambiguity_inputs(inputs) {
        let candidates = code_period_candidates(input);
        if candidates.is_empty() {
            return Err(CODE_PERIOD_AMBIGUITY_UNAVAILABLE);
        }
        minimum_code_period_s = minimum_code_period_s.min(input.code_period_s);
        anchors.extend(candidates.iter().map(|candidate| candidate.residual_s));
        candidate_sets.push((*input, candidates));
    }

    if !minimum_code_period_s.is_finite() || minimum_code_period_s <= 0.0 {
        return Err(CODE_PERIOD_AMBIGUITY_UNAVAILABLE);
    }

    anchors.sort_by(|a, b| a.total_cmp(b));
    anchors.dedup_by(|a, b| (*a - *b).abs() <= CODE_PERIOD_AMBIGUITY_EPS_S);

    let max_coherence_s = minimum_code_period_s * CODE_PERIOD_AMBIGUITY_MAX_COHERENCE_FRACTION;
    let max_clock_bias_s = minimum_code_period_s * CODE_PERIOD_AMBIGUITY_MAX_CLOCK_BIAS_FRACTION;
    let mut scored_solutions = Vec::new();

    for anchor_s in anchors {
        if let Some(solution) =
            scored_code_period_ambiguity_solution(&candidate_sets, anchor_s, max_coherence_s)
        {
            if solution.solution.common_receiver_clock_bias_s.0.abs() <= max_clock_bias_s {
                scored_solutions.push(solution);
            }
        }
    }

    if scored_solutions.is_empty() {
        return Err(CODE_PERIOD_AMBIGUITY_INCOHERENT);
    }

    scored_solutions.sort_by(|a, b| a.score_s.total_cmp(&b.score_s));
    let best = scored_solutions.remove(0);
    if best.non_unique_candidate_selection {
        return Err(CODE_PERIOD_AMBIGUITY_NON_UNIQUE);
    }
    if scored_solutions.iter().any(|candidate| {
        (candidate.score_s - best.score_s).abs() <= CODE_PERIOD_AMBIGUITY_EPS_S
            && code_period_solution_key(&candidate.solution)
                != code_period_solution_key(&best.solution)
    }) {
        return Err(CODE_PERIOD_AMBIGUITY_NON_UNIQUE);
    }

    Ok(best.solution)
}

fn sorted_code_period_ambiguity_inputs(
    inputs: &[CodePeriodAmbiguityInput],
) -> Vec<&CodePeriodAmbiguityInput> {
    let mut sorted = inputs.iter().collect::<Vec<_>>();
    sorted.sort_by_key(|input| observation_signal_key(input.signal_id));
    sorted
}

fn code_period_candidates(input: &CodePeriodAmbiguityInput) -> Vec<CodePeriodCandidate> {
    if !input.code_period_s.is_finite()
        || !input.code_delay_s.is_finite()
        || input.code_period_s <= 0.0
        || input.code_delay_s < 0.0
        || input.code_delay_s >= input.code_period_s
        || !input.receive_gps_time.tow_s.is_finite()
        || !input.decoded_transmit_gps_time.tow_s.is_finite()
    {
        return Vec::new();
    }

    let decoded_travel_time_s =
        input.receive_gps_time.to_seconds() - input.decoded_transmit_gps_time.to_seconds();
    if !decoded_travel_time_s.is_finite() || decoded_travel_time_s <= 0.0 {
        return Vec::new();
    }

    let raw_periods = (decoded_travel_time_s - input.code_delay_s) / input.code_period_s;
    if !raw_periods.is_finite() {
        return Vec::new();
    }

    let center = raw_periods.floor() as i64;
    let mut candidates = Vec::new();
    for offset in -CODE_PERIOD_AMBIGUITY_SEARCH_PERIODS..=CODE_PERIOD_AMBIGUITY_SEARCH_PERIODS {
        let candidate_periods = center + offset;
        if candidate_periods < 0 {
            continue;
        }
        let signal_travel_time_s =
            candidate_periods as f64 * input.code_period_s + input.code_delay_s;
        let residual_s = decoded_travel_time_s - signal_travel_time_s;
        if signal_travel_time_s.is_finite() && signal_travel_time_s > 0.0 && residual_s.is_finite()
        {
            candidates.push(CodePeriodCandidate {
                integer_code_periods: candidate_periods as u64,
                signal_travel_time_s,
                residual_s,
            });
        }
    }
    candidates.sort_by_key(|candidate| candidate.integer_code_periods);
    candidates.dedup_by_key(|candidate| candidate.integer_code_periods);
    candidates
}

fn scored_code_period_ambiguity_solution(
    candidate_sets: &[(CodePeriodAmbiguityInput, Vec<CodePeriodCandidate>)],
    anchor_s: f64,
    max_coherence_s: f64,
) -> Option<ScoredCodePeriodAmbiguitySolution> {
    let mut chosen = Vec::with_capacity(candidate_sets.len());
    let mut non_unique_candidate_selection = false;

    for (input, candidates) in candidate_sets {
        let best_distance_s = candidates
            .iter()
            .map(|candidate| (candidate.residual_s - anchor_s).abs())
            .min_by(|a, b| a.total_cmp(b))?;
        let nearest = candidates
            .iter()
            .filter(|candidate| {
                ((candidate.residual_s - anchor_s).abs() - best_distance_s).abs()
                    <= CODE_PERIOD_AMBIGUITY_EPS_S
            })
            .collect::<Vec<_>>();
        if nearest.len() > 1 {
            non_unique_candidate_selection = true;
        }
        chosen.push((*input, *nearest[0]));
    }

    let common_receiver_clock_bias_s =
        chosen.iter().map(|(_, candidate)| candidate.residual_s).sum::<f64>() / chosen.len() as f64;
    let max_residual_spread_s = chosen
        .iter()
        .map(|(_, candidate)| (candidate.residual_s - common_receiver_clock_bias_s).abs())
        .fold(0.0_f64, f64::max);
    if max_residual_spread_s > max_coherence_s {
        return None;
    }

    let satellites = chosen
        .into_iter()
        .map(|(input, candidate)| {
            let observed_signal_travel_time_s =
                candidate.signal_travel_time_s + common_receiver_clock_bias_s;
            CodePeriodAmbiguitySatellite {
                signal_id: input.signal_id,
                integer_code_periods: candidate.integer_code_periods,
                signal_travel_time_s: Seconds(observed_signal_travel_time_s),
                transmit_gps_time: input
                    .receive_gps_time
                    .offset_seconds(-observed_signal_travel_time_s),
            }
        })
        .collect::<Vec<_>>();

    Some(ScoredCodePeriodAmbiguitySolution {
        score_s: max_residual_spread_s + common_receiver_clock_bias_s.abs(),
        non_unique_candidate_selection,
        solution: CodePeriodAmbiguitySolution {
            common_receiver_clock_bias_s: Seconds(common_receiver_clock_bias_s),
            satellites,
        },
    })
}

fn code_period_solution_key(solution: &CodePeriodAmbiguitySolution) -> Vec<(String, u64)> {
    solution
        .satellites
        .iter()
        .map(|satellite| {
            (observation_signal_key(satellite.signal_id), satellite.integer_code_periods)
        })
        .collect()
}

pub(super) fn code_period_ambiguity_input_from_tracking_epoch(
    config: &ReceiverPipelineConfig,
    capture_start_gps_time: Option<GpsTime>,
    epoch: &TrackEpoch,
) -> Option<CodePeriodAmbiguityInput> {
    let transmit_time = epoch.transmit_time.as_ref()?;
    let receive_gps_time = capture_start_gps_time?.offset_seconds(
        epoch.sample_index as f64 / config.sampling_freq_hz + config.receiver_clock_bias_s,
    );
    let signal_model = observation_signal_model(config, epoch);
    let code_phase_timing = code_phase_timing_from_tracking_epoch(
        epoch,
        signal_model.samples_per_chip,
        signal_model.signal.code_rate_hz,
        signal_model.code_length as f64,
    )?;

    Some(CodePeriodAmbiguityInput {
        signal_id: signal_model.signal_id,
        receive_gps_time,
        decoded_transmit_gps_time: transmit_time.transmit_gps_time,
        code_period_s: code_phase_timing.code_period_s,
        code_delay_s: code_phase_timing.code_delay_s,
    })
}

pub(super) fn apply_grouped_integer_code_period_ambiguities(
    epoch: &mut ObsEpoch,
    ambiguity_inputs: &HashMap<String, CodePeriodAmbiguityInput>,
    diagnostics: &mut Vec<DiagnosticEvent>,
) {
    let inputs = epoch
        .sats
        .iter()
        .filter(|sat| sat.metadata.pseudorange_model == "decoded_transmit_time_code_phase")
        .filter_map(|sat| {
            ambiguity_inputs.get(&observation_snapshot_key(epoch.epoch_idx, sat.signal_id)).copied()
        })
        .collect::<Vec<_>>();
    if inputs.is_empty() {
        return;
    }

    match resolve_integer_code_period_ambiguities(&inputs) {
        Ok(solution) => apply_code_period_ambiguity_solution(epoch, solution),
        Err(reason) => {
            mark_code_period_ambiguity_refusal(epoch, &inputs, reason);
            diagnostics.push(code_period_ambiguity_diagnostic(epoch, &inputs, reason));
        }
    }
}

fn apply_code_period_ambiguity_solution(
    epoch: &mut ObsEpoch,
    solution: CodePeriodAmbiguitySolution,
) {
    for resolved in solution.satellites {
        if let Some(sat) = epoch.sats.iter_mut().find(|sat| sat.signal_id == resolved.signal_id) {
            sat.pseudorange_m = Meters(resolved.signal_travel_time_s.0 * SPEED_OF_LIGHT_MPS);
            sat.timing = Some(ObsSignalTiming {
                signal_travel_time_s: resolved.signal_travel_time_s,
                transmit_gps_time: resolved.transmit_gps_time,
            });
            sat.metadata.pseudorange_integer_code_periods = Some(resolved.integer_code_periods);
            sat.metadata.observation_support_class =
                observation_support_label(sat.observation_status, true).to_string();
        }
    }
}

fn mark_code_period_ambiguity_refusal(
    epoch: &mut ObsEpoch,
    inputs: &[CodePeriodAmbiguityInput],
    reason: &str,
) {
    for input in inputs {
        if let Some(sat) = epoch.sats.iter_mut().find(|sat| sat.signal_id == input.signal_id) {
            sat.observation_status = ObservationStatus::Inconsistent;
            push_observation_reject_reason(&mut sat.observation_reject_reasons, reason);
            sat.metadata.observation_status =
                observation_status_label(sat.observation_status).to_string();
            sat.metadata.observation_reject_reasons = sat.observation_reject_reasons.clone();
            sat.metadata.observation_support_class =
                observation_support_label(sat.observation_status, false).to_string();
        }
    }
}

fn code_period_ambiguity_diagnostic(
    epoch: &ObsEpoch,
    inputs: &[CodePeriodAmbiguityInput],
    reason: &str,
) -> DiagnosticEvent {
    let signal_count = inputs.len();
    let mut signals =
        inputs.iter().map(|input| observation_signal_key(input.signal_id)).collect::<Vec<_>>();
    signals.sort();
    DiagnosticEvent::new(
        DiagnosticSeverity::Warning,
        "OBS_INTEGER_CODE_PERIOD_AMBIGUITY_REFUSED",
        format!(
            "refused integer code-period ambiguity resolution for {signal_count} decoded-time signals"
        ),
    )
    .with_context("epoch", epoch.epoch_idx.to_string())
    .with_context("reason", reason)
    .with_context("signals", signals.join(","))
    .with_context("stage", "observations")
}
