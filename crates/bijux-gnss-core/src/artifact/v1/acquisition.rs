use super::validate_receiver_sample_trace;
use crate::artifact::{ArtifactPayloadValidate, ArtifactV1};
use crate::api::{DiagnosticEvent, DiagnosticSeverity};

use crate::api::{AcqExplain, AcqResult};

/// Acquisition result artifact v1.
pub type AcqResultV1 = ArtifactV1<AcqResult>;

impl ArtifactPayloadValidate for AcqResult {
    fn validate_payload(&self) -> Vec<DiagnosticEvent> {
        let mut events = Vec::new();
        if !self.doppler_hz.0.is_finite() || !self.carrier_hz.0.is_finite() {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "GNSS_NUMERIC_ACQ_INVALID",
                "acquisition result contains NaN/Inf",
            ));
        }
        if self.signal_band == crate::api::SignalBand::Unknown {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "GNSS_ACQ_SIGNAL_BAND_INVALID",
                "acquisition result must declare an explicit signal band",
            ));
        }
        if let Some(refinement) = &self.doppler_refinement {
            if !refinement.coarse_carrier_hz.0.is_finite()
                || !refinement.offset_hz.is_finite()
                || !refinement.offset_bins.is_finite()
                || !refinement.left_peak_mean_ratio.is_finite()
                || !refinement.center_peak_mean_ratio.is_finite()
                || !refinement.right_peak_mean_ratio.is_finite()
            {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "GNSS_NUMERIC_ACQ_REFINEMENT_INVALID",
                    "acquisition refinement contains NaN/Inf",
                ));
            }
        }
        if let Some(refinement) = &self.code_phase_refinement {
            if !refinement.offset_samples.is_finite()
                || !refinement.refined_code_phase_samples.is_finite()
                || !refinement.left_correlation_norm.is_finite()
                || !refinement.center_correlation_norm.is_finite()
                || !refinement.right_correlation_norm.is_finite()
            {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "GNSS_NUMERIC_ACQ_CODE_PHASE_REFINEMENT_INVALID",
                    "acquisition code-phase refinement contains NaN/Inf",
                ));
            }
        }
        if let Some(uncertainty) = &self.uncertainty {
            if !uncertainty.doppler_hz.is_finite()
                || !uncertainty.code_phase_samples.is_finite()
                || uncertainty.doppler_hz < 0.0
                || uncertainty.code_phase_samples < 0.0
            {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "GNSS_NUMERIC_ACQ_UNCERTAINTY_INVALID",
                    "acquisition uncertainty contains invalid values",
                ));
            }
        }
        if self.candidate_rank == 0 {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "GNSS_ACQ_CANDIDATE_RANK_INVALID",
                "acquisition candidate rank must be at least 1",
            ));
        }
        if self.is_primary_candidate && self.candidate_rank != 1 {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "GNSS_ACQ_PRIMARY_CANDIDATE_RANK_INVALID",
                "primary acquisition candidate must use rank 1",
            ));
        }
        if !self.is_primary_candidate && self.candidate_rank == 1 {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "GNSS_ACQ_ALTERNATIVE_CANDIDATE_RANK_INVALID",
                "non-primary acquisition candidates must use rank 2 or greater",
            ));
        }
        if let Some(provenance) =
            self.evidence.iter().find_map(|evidence| evidence.component_provenance.as_ref())
        {
            if provenance.components.is_empty() {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "GNSS_ACQ_COMPONENT_PROVENANCE_EMPTY",
                    "acquisition component provenance must include at least one component",
                ));
            }
            for component in &provenance.components {
                if !component.peak.is_finite()
                    || !component.second_peak.is_finite()
                    || !component.mean.is_finite()
                    || !component.peak_mean_ratio.is_finite()
                    || !component.peak_second_ratio.is_finite()
                {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Error,
                        "GNSS_NUMERIC_ACQ_COMPONENT_INVALID",
                        "acquisition component provenance contains NaN/Inf",
                    ));
                    break;
                }
            }
        }
        events.extend(validate_receiver_sample_trace(
            self.source_time,
            "acquisition",
            None,
            None,
        ));
        events
    }
}

/// Acquisition explain artifact v1.
pub type AcqExplainV1 = ArtifactV1<AcqExplain>;

impl ArtifactPayloadValidate for AcqExplain {
    fn validate_payload(&self) -> Vec<DiagnosticEvent> {
        let mut events = Vec::new();
        if self.sat.prn == 0 {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "GNSS_ACQ_EXPLAIN_INVALID",
                "explain record has invalid sat",
            ));
        }
        events
    }
}
