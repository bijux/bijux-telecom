use super::validate_receiver_sample_trace;
use crate::artifact::{ArtifactPayloadValidate, ArtifactV1};
use crate::api::{DiagnosticEvent, DiagnosticSeverity, TrackEpoch, TrackTransition};

/// Tracking epoch artifact v1.
pub type TrackEpochV1 = ArtifactV1<TrackEpoch>;

impl ArtifactPayloadValidate for TrackEpoch {
    fn validate_payload(&self) -> Vec<DiagnosticEvent> {
        let mut events = Vec::new();
        if !self.carrier_hz.0.is_finite() || !self.code_rate_hz.0.is_finite() {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "GNSS_NUMERIC_TRACK_INVALID",
                "tracking epoch contains NaN/Inf",
            ));
        }
        if self.navigation_bit_sign.is_some_and(|sign| sign != -1 && sign != 1) {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "GNSS_TRACK_NAVIGATION_BIT_SIGN_INVALID",
                "tracking epoch navigation bit sign must be -1 or 1",
            ));
        }
        if let Some(uncertainty) = &self.tracking_uncertainty {
            if !uncertainty.code_phase_samples.is_finite()
                || !uncertainty.carrier_phase_cycles.is_finite()
                || !uncertainty.doppler_hz.is_finite()
                || !uncertainty.cn0_dbhz.is_finite()
                || uncertainty.code_phase_samples < 0.0
                || uncertainty.carrier_phase_cycles < 0.0
                || uncertainty.doppler_hz < 0.0
                || uncertainty.cn0_dbhz < 0.0
            {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "GNSS_NUMERIC_TRACK_UNCERTAINTY_INVALID",
                    "tracking uncertainty contains invalid values",
                ));
            }
        }
        events.extend(validate_receiver_sample_trace(
            self.source_time,
            "tracking",
            None,
            Some(self.sample_index),
        ));
        events
    }
}

/// Track transition artifact v1.
pub type TrackTransitionV1 = ArtifactV1<TrackTransition>;

impl ArtifactPayloadValidate for TrackTransition {
    fn validate_payload(&self) -> Vec<DiagnosticEvent> {
        let mut events = Vec::new();
        if self.sample_index == 0 && self.epoch_idx == 0 {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Warning,
                "GNSS_TRACK_TRANSITION_UNINITIALIZED",
                "transition uses zero epoch/sample index",
            ));
        }
        events
    }
}
