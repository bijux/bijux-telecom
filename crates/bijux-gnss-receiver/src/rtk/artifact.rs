//! Versioned artifact wrappers for RTK outputs.

use bijux_gnss_core::{ArtifactHeaderV1, ArtifactValidate, DiagnosticEvent, DiagnosticSeverity};
use serde::{Deserialize, Serialize};

use crate::rtk_internal::ambiguity::FixAuditEvent;
use crate::rtk_internal::core::{DdObservation, SdObservation};
use crate::rtk_internal::metrics::{BaselineSolution, RtkBaselineQuality, RtkPrecision};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RtkSdEpochV1 {
    pub header: ArtifactHeaderV1,
    pub payload: SdObservation,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RtkDdEpochV1 {
    pub header: ArtifactHeaderV1,
    pub payload: DdObservation,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RtkBaselineEpochV1 {
    pub header: ArtifactHeaderV1,
    pub payload: BaselineSolution,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RtkBaselineQualityV1 {
    pub header: ArtifactHeaderV1,
    pub payload: RtkBaselineQuality,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RtkFixAuditV1 {
    pub header: ArtifactHeaderV1,
    pub payload: FixAuditEvent,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RtkPrecisionV1 {
    pub header: ArtifactHeaderV1,
    pub payload: RtkPrecision,
}

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

impl ArtifactValidate for RtkSdEpochV1 {
    fn validate(&self) -> Vec<DiagnosticEvent> {
        let mut events = Vec::new();
        if let Some(event) = finite_or_event(self.payload.code_m, "RTK_SD_CODE_INVALID") {
            events.push(event);
        }
        if let Some(event) = finite_or_event(self.payload.phase_cycles, "RTK_SD_PHASE_INVALID") {
            events.push(event);
        }
        if let Some(event) = finite_or_event(self.payload.doppler_hz, "RTK_SD_DOPPLER_INVALID") {
            events.push(event);
        }
        events
    }
}

impl ArtifactValidate for RtkDdEpochV1 {
    fn validate(&self) -> Vec<DiagnosticEvent> {
        let mut events = Vec::new();
        if let Some(event) = finite_or_event(self.payload.code_m, "RTK_DD_CODE_INVALID") {
            events.push(event);
        }
        if let Some(event) = finite_or_event(self.payload.phase_cycles, "RTK_DD_PHASE_INVALID") {
            events.push(event);
        }
        if let Some(event) = finite_or_event(self.payload.doppler_hz, "RTK_DD_DOPPLER_INVALID") {
            events.push(event);
        }
        events
    }
}

impl ArtifactValidate for RtkBaselineEpochV1 {
    fn validate(&self) -> Vec<DiagnosticEvent> {
        let mut events = Vec::new();
        for &value in &self.payload.enu_m {
            if let Some(event) = finite_or_event(value, "RTK_BASELINE_INVALID") {
                events.push(event);
            }
        }
        events
    }
}

impl ArtifactValidate for RtkBaselineQualityV1 {
    fn validate(&self) -> Vec<DiagnosticEvent> {
        let mut events = Vec::new();
        for value in [
            self.payload.sigma_e,
            self.payload.sigma_n,
            self.payload.sigma_u,
            self.payload.residual_rms_m,
            self.payload.predicted_rms_m,
        ] {
            if let Some(event) = finite_or_event(value, "RTK_QUALITY_INVALID") {
                events.push(event);
            }
        }
        events
    }
}

impl ArtifactValidate for RtkFixAuditV1 {
    fn validate(&self) -> Vec<DiagnosticEvent> {
        Vec::new()
    }
}

impl ArtifactValidate for RtkPrecisionV1 {
    fn validate(&self) -> Vec<DiagnosticEvent> {
        let mut events = Vec::new();
        if let Some(ratio) = self.payload.ratio {
            if let Some(event) = finite_or_event(ratio, "RTK_PRECISION_INVALID") {
                events.push(event);
            }
        }
        events
    }
}
