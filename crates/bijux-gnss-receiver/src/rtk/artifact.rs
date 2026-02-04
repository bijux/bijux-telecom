//! Versioned artifact wrappers for RTK outputs.

use bijux_gnss_core::{ArtifactPayloadValidate, ArtifactV1, DiagnosticEvent, DiagnosticSeverity};
use crate::rtk_internal::ambiguity::FixAuditEvent;
use crate::rtk_internal::core::{DdObservation, SdObservation};
use crate::rtk_internal::metrics::{BaselineSolution, RtkBaselineQuality, RtkPrecision};

pub type RtkSdEpochV1 = ArtifactV1<SdObservation>;
pub type RtkDdEpochV1 = ArtifactV1<DdObservation>;
pub type RtkBaselineEpochV1 = ArtifactV1<BaselineSolution>;
pub type RtkBaselineQualityV1 = ArtifactV1<RtkBaselineQuality>;
pub type RtkFixAuditV1 = ArtifactV1<FixAuditEvent>;
pub type RtkPrecisionV1 = ArtifactV1<RtkPrecision>;

fn finite_or_event(value: f64, code: &str) -> Option<DiagnosticEvent> {
    if value.is_finite() {
        None
    } else {
        Some(DiagnosticEvent::new(
            DiagnosticSeverity::Error,
            code,
            "rtk artifact contains NaN/Inf",
        ))
    }
}

impl ArtifactPayloadValidate for SdObservation {
    fn validate_payload(&self) -> Vec<DiagnosticEvent> {
        let mut events = Vec::new();
        if let Some(event) = finite_or_event(self.code_m, "RTK_SD_CODE_INVALID") {
            events.push(event);
        }
        if let Some(event) = finite_or_event(self.phase_cycles, "RTK_SD_PHASE_INVALID") {
            events.push(event);
        }
        if let Some(event) = finite_or_event(self.doppler_hz, "RTK_SD_DOPPLER_INVALID") {
            events.push(event);
        }
        events
    }
}

impl ArtifactPayloadValidate for DdObservation {
    fn validate_payload(&self) -> Vec<DiagnosticEvent> {
        let mut events = Vec::new();
        if let Some(event) = finite_or_event(self.code_m, "RTK_DD_CODE_INVALID") {
            events.push(event);
        }
        if let Some(event) = finite_or_event(self.phase_cycles, "RTK_DD_PHASE_INVALID") {
            events.push(event);
        }
        if let Some(event) = finite_or_event(self.doppler_hz, "RTK_DD_DOPPLER_INVALID") {
            events.push(event);
        }
        events
    }
}

impl ArtifactPayloadValidate for BaselineSolution {
    fn validate_payload(&self) -> Vec<DiagnosticEvent> {
        let mut events = Vec::new();
        for &value in &self.enu_m {
            if let Some(event) = finite_or_event(value, "RTK_BASELINE_INVALID") {
                events.push(event);
            }
        }
        events
    }
}

impl ArtifactPayloadValidate for RtkBaselineQuality {
    fn validate_payload(&self) -> Vec<DiagnosticEvent> {
        let mut events = Vec::new();
        for value in [
            self.sigma_e,
            self.sigma_n,
            self.sigma_u,
            self.residual_rms_m,
            self.predicted_rms_m,
        ] {
            if let Some(event) = finite_or_event(value, "RTK_QUALITY_INVALID") {
                events.push(event);
            }
        }
        events
    }
}

impl ArtifactPayloadValidate for FixAuditEvent {
    fn validate_payload(&self) -> Vec<DiagnosticEvent> {
        Vec::new()
    }
}

impl ArtifactPayloadValidate for RtkPrecision {
    fn validate_payload(&self) -> Vec<DiagnosticEvent> {
        let mut events = Vec::new();
        if let Some(ratio) = self.ratio {
            if let Some(event) = finite_or_event(ratio, "RTK_PRECISION_INVALID") {
                events.push(event);
            }
        }
        events
    }
}
