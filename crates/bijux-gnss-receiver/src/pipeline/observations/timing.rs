use bijux_gnss_core::api::{
    ConventionsConfig, DiagnosticEvent, DiagnosticSeverity, ObsEpoch, ObsSatellite,
    ObservationEpochDecision, ObservationStatus, ReceiverSampleTrace, Seconds, SignalBand,
    TrackEpoch,
};
use bijux_gnss_signal::api::samples_per_code;

use crate::engine::receiver_config::ReceiverPipelineConfig;

use super::signal_model::ObservationSignalModel;
use super::status::push_observation_reject_reason;

pub(super) fn observation_timing_interval_diagnostic(
    config: &ReceiverPipelineConfig,
    previous: &ObsEpoch,
    current: &ObsEpoch,
) -> Option<DiagnosticEvent> {
    let expected_sample_interval = observation_interval_samples(config);
    let actual_sample_interval =
        current.source_time.sample_index.saturating_sub(previous.source_time.sample_index);
    let actual_interval_s = current.t_rx_s.0 - previous.t_rx_s.0;
    let expected_interval_s = expected_sample_interval as f64 / config.sampling_freq_hz;
    let tolerance_s = receiver_time_tolerance_s(config.sampling_freq_hz);

    if actual_sample_interval == expected_sample_interval
        && (actual_interval_s - expected_interval_s).abs() <= tolerance_s
    {
        return None;
    }

    Some(
        DiagnosticEvent::new(
            DiagnosticSeverity::Error,
            "GNSS_OBS_TIME_INTERVAL_INVALID",
            format!(
                "observation epoch spacing {} samples ({actual_interval_s:.9}s) does not match configured interval {} samples ({expected_interval_s:.9}s)",
                actual_sample_interval, expected_sample_interval
            ),
        )
        .with_context("epoch", current.epoch_idx.to_string())
        .with_context("previous_epoch", previous.epoch_idx.to_string())
        .with_context("stage", "observations"),
    )
}

pub(super) fn normalize_observation_cn0_dbhz(cn0_dbhz: f64) -> f64 {
    if !cn0_dbhz.is_finite() {
        return cn0_dbhz;
    }

    let conventions = ConventionsConfig::default();
    cn0_dbhz.clamp(conventions.min_cn0_dbhz, conventions.max_cn0_dbhz)
}

pub(super) fn observation_interval_samples(config: &ReceiverPipelineConfig) -> u64 {
    let samples_per_code =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let integration_ms = config.tracking_params(SignalBand::L1).integration_ms.max(1) as u64;
    samples_per_code as u64 * integration_ms
}

pub(super) fn observation_epoch_interval_samples(
    config: &ReceiverPipelineConfig,
    signal_model: &ObservationSignalModel,
    signal_band: SignalBand,
) -> u64 {
    let samples_per_code =
        (signal_model.samples_per_chip * signal_model.code_length as f64).round() as u64;
    let integration_ms = config.tracking_params(signal_band).integration_ms.max(1) as u64;
    samples_per_code.saturating_mul(integration_ms)
}

pub(super) fn reject_epoch_for_invalid_timing(epoch: &mut ObsEpoch) {
    epoch.valid = false;
    if epoch.decision == ObservationEpochDecision::Accepted {
        epoch.decision = ObservationEpochDecision::Rejected;
        epoch.decision_reason = Some("invalid_observation_timing".to_string());
    }
}

pub(super) fn observation_epoch_id(epoch_idx: u64, sample_index: u64) -> String {
    format!("epoch-{epoch_idx:010}-sample-{sample_index:012}")
}

pub(super) fn grouped_epoch_time_mismatch_reasons(
    grouped: &ObsEpoch,
    incoming: &ObsEpoch,
) -> Option<Vec<&'static str>> {
    if grouped.sats.is_empty() {
        return None;
    }

    let mut reasons = Vec::new();
    let tolerance_s = receiver_time_tolerance_s(
        grouped.source_time.sample_rate_hz.max(incoming.source_time.sample_rate_hz),
    );

    if (grouped.t_rx_s.0 - incoming.t_rx_s.0).abs() > tolerance_s {
        reasons.push("receiver_time_mismatch");
    }
    if grouped.source_time.sample_index != incoming.source_time.sample_index
        || (grouped.source_time.sample_rate_hz - incoming.source_time.sample_rate_hz).abs()
            > f64::EPSILON
    {
        reasons.push("receiver_sample_trace_mismatch");
    }
    if grouped.gps_week != incoming.gps_week
        || !optional_seconds_match(grouped.tow_s, incoming.tow_s, tolerance_s)
    {
        reasons.push("gps_receive_time_mismatch");
    }

    (!reasons.is_empty()).then_some(reasons)
}

fn receiver_time_tolerance_s(sample_rate_hz: f64) -> f64 {
    if sample_rate_hz.is_finite() && sample_rate_hz > 0.0 {
        (0.5 / sample_rate_hz).max(1.0e-12)
    } else {
        1.0e-12
    }
}

fn optional_seconds_match(lhs: Option<Seconds>, rhs: Option<Seconds>, tolerance_s: f64) -> bool {
    match (lhs, rhs) {
        (Some(lhs), Some(rhs)) => (lhs.0 - rhs.0).abs() <= tolerance_s,
        (None, None) => true,
        _ => false,
    }
}

pub(super) fn mark_grouped_epoch_time_mismatch(sat: &mut ObsSatellite, reasons: &[&str]) {
    sat.observation_status = ObservationStatus::Inconsistent;
    for reason in reasons {
        push_observation_reject_reason(&mut sat.observation_reject_reasons, reason);
    }
}

pub(super) fn grouped_epoch_time_mismatch_diagnostic(
    grouped: &ObsEpoch,
    incoming_epoch_idx: u64,
    incoming_source_time: ReceiverSampleTrace,
    incoming_t_rx_s: Seconds,
    sat: &ObsSatellite,
) -> DiagnosticEvent {
    DiagnosticEvent::new(
        DiagnosticSeverity::Error,
        "OBS_GROUPED_RECEIVER_TIME_MISMATCH",
        format!(
            "satellite {} sample trace {}@{}s does not match grouped epoch {}@{}s",
            sat.signal_id.sat.prn,
            incoming_source_time.sample_index,
            incoming_t_rx_s.0,
            grouped.source_time.sample_index,
            grouped.t_rx_s.0
        ),
    )
    .with_context("epoch", incoming_epoch_idx.to_string())
    .with_context("stage", "observations")
}

pub(super) fn tracking_time_tag(
    epoch: &TrackEpoch,
    last_locked_sample: &mut Option<u64>,
) -> (String, u64) {
    if epoch.lock || epoch.lock_state == "tracking" || epoch.lock_state == "acquired" {
        *last_locked_sample = Some(epoch.sample_index);
        return ("tracking".to_string(), epoch.sample_index);
    }
    match (epoch.lock, *last_locked_sample) {
        (false, Some(last)) => ("tracking_last_locked".to_string(), last),
        (false, None) => ("tracking_unlocked".to_string(), epoch.sample_index),
        _ => ("tracking".to_string(), epoch.sample_index),
    }
}
