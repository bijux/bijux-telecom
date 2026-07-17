use super::validate_receiver_sample_trace;
use crate::artifact::{ArtifactPayloadValidate, ArtifactV1};
use crate::api::{
    ArtifactValidate, Constellation, DiagnosticEvent, DiagnosticSeverity, ObsDecisionArtifact,
    ObsEpoch, SigId, SignalBand, SignalCode,
};

/// Observation epoch artifact v1.
pub type ObsEpochV1 = ArtifactV1<ObsEpoch>;

impl ArtifactValidate for ObsEpochV1 {
    fn validate(&self) -> Vec<DiagnosticEvent> {
        let mut events = self.payload.validate_payload();
        for sat in &self.payload.sats {
            if !is_valid_signal_id(&sat.signal_id) {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "GNSS_OBS_ID_INVALID",
                    "obs signal_id is invalid",
                ));
            }
        }
        events
    }
}

impl ArtifactPayloadValidate for ObsEpoch {
    fn validate_payload(&self) -> Vec<DiagnosticEvent> {
        let mut events = self.validate_physics();
        if !self.t_rx_s.0.is_finite() {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "GNSS_NUMERIC_T_RX_INVALID",
                "t_rx_s is not finite",
            ));
        }
        events.extend(validate_receiver_sample_trace(
            self.source_time,
            "observation",
            Some(self.t_rx_s),
            None,
        ));
        if let Some(manifest) = &self.manifest {
            events.extend(validate_receiver_sample_trace(
                manifest.source_time,
                "observation manifest",
                Some(self.t_rx_s),
                Some(manifest.source_sample_index),
            ));
            if manifest.source_time.sample_index != self.source_time.sample_index {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "GNSS_OBS_SOURCE_TIME_MISMATCH",
                    "observation manifest source trace does not match epoch source trace",
                ));
            }
        }
        events
    }
}

/// Observation decision artifact v1.
pub type ObsDecisionV1 = ArtifactV1<ObsDecisionArtifact>;

impl ArtifactPayloadValidate for ObsDecisionArtifact {
    fn validate_payload(&self) -> Vec<DiagnosticEvent> {
        let mut events = Vec::new();
        if self.artifact_id.is_empty() {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "GNSS_OBS_DECISION_INVALID",
                "observation decision missing artifact id",
            ));
        }
        events
    }
}

fn is_valid_signal_id(id: &SigId) -> bool {
    if id.sat.prn == 0 {
        return false;
    }
    !matches!(id.sat.constellation, Constellation::Unknown)
        && !matches!(id.band, SignalBand::Unknown)
        && !matches!(id.code, SignalCode::Unknown)
}
