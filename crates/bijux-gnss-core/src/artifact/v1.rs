//! Versioned v1 artifact contracts.

use super::{ArtifactPayloadValidate, ArtifactV1};
use crate::api::{DiagnosticEvent, DiagnosticSeverity};

mod receiver_trace;
pub mod acquisition;
pub mod observation;
pub mod support_matrix;
pub mod tracking;

use receiver_trace::validate_receiver_sample_trace;

pub mod nav {
    use super::*;
    use crate::api::{NavSolutionEpoch, SolutionStatus};

    /// Navigation solution artifact v1.
    pub type NavSolutionEpochV1 = ArtifactV1<NavSolutionEpoch>;

    impl ArtifactPayloadValidate for NavSolutionEpoch {
        fn validate_payload(&self) -> Vec<DiagnosticEvent> {
            let mut events = Vec::new();
            if !self.ecef_x_m.0.is_finite()
                || !self.ecef_y_m.0.is_finite()
                || !self.ecef_z_m.0.is_finite()
            {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "GNSS_NUMERIC_PVT_INVALID",
                    "PVT position contains NaN/Inf",
                ));
            }
            if let Some(covariance) = self.position_covariance_ecef_m2 {
                if !covariance.iter().flat_map(|row| row.iter()).all(|value| value.is_finite()) {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Error,
                        "GNSS_NAV_POSITION_COVARIANCE_INVALID",
                        "nav solution position covariance contains NaN/Inf",
                    ));
                }
                if covariance[0][0] < 0.0 || covariance[1][1] < 0.0 || covariance[2][2] < 0.0 {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Error,
                        "GNSS_NAV_POSITION_COVARIANCE_NEGATIVE_VARIANCE",
                        "nav solution position covariance diagonal must be non-negative",
                    ));
                }
            } else if self.valid {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Warning,
                    "GNSS_NAV_POSITION_COVARIANCE_MISSING",
                    "valid nav solution should carry an ECEF position covariance matrix",
                ));
            }
            let enu_sigmas = [self.sigma_e_m, self.sigma_n_m, self.sigma_u_m];
            if enu_sigmas.iter().flatten().any(|value| !value.0.is_finite()) {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "GNSS_NAV_POSITION_SIGMA_ENU_INVALID",
                    "nav solution ENU position standard deviations contain NaN/Inf",
                ));
            }
            if enu_sigmas.iter().flatten().any(|value| value.0 < 0.0) {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "GNSS_NAV_POSITION_SIGMA_ENU_NEGATIVE",
                    "nav solution ENU position standard deviations must be non-negative",
                ));
            }
            if self.valid && enu_sigmas.iter().any(|value| value.is_none()) {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Warning,
                    "GNSS_NAV_POSITION_SIGMA_ENU_MISSING",
                    "valid nav solution should carry east, north, and up position standard deviations",
                ));
            }
            let horizontal_error_ellipse = [
                self.horizontal_error_ellipse_major_axis_m,
                self.horizontal_error_ellipse_minor_axis_m,
            ];
            if horizontal_error_ellipse.iter().flatten().any(|value| !value.0.is_finite())
                || self
                    .horizontal_error_ellipse_azimuth_deg
                    .is_some_and(|value| !value.is_finite())
            {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "GNSS_NAV_HORIZONTAL_ERROR_ELLIPSE_INVALID",
                    "nav solution horizontal error ellipse contains NaN/Inf",
                ));
            }
            if horizontal_error_ellipse.iter().flatten().any(|value| value.0 < 0.0) {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "GNSS_NAV_HORIZONTAL_ERROR_ELLIPSE_NEGATIVE_AXIS",
                    "nav solution horizontal error ellipse axes must be non-negative",
                ));
            }
            if self
                .horizontal_error_ellipse_azimuth_deg
                .is_some_and(|value| !(0.0..180.0).contains(&value))
            {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "GNSS_NAV_HORIZONTAL_ERROR_ELLIPSE_AZIMUTH_RANGE",
                    "nav solution horizontal error ellipse azimuth must be in [0, 180)",
                ));
            }
            if self.valid
                && (self.horizontal_error_ellipse_major_axis_m.is_none()
                    || self.horizontal_error_ellipse_minor_axis_m.is_none()
                    || self.horizontal_error_ellipse_azimuth_deg.is_none())
            {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Warning,
                    "GNSS_NAV_HORIZONTAL_ERROR_ELLIPSE_MISSING",
                    "valid nav solution should carry horizontal error ellipse axes and azimuth",
                ));
            }
            if !self.clock_bias_s.0.is_finite() || !self.clock_bias_m.0.is_finite() {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "GNSS_NAV_CLOCK_BIAS_INVALID",
                    "nav solution clock bias contains NaN/Inf",
                ));
            } else {
                let expected_clock_bias_m = self.clock_bias_s.0 * 299_792_458.0;
                if (self.clock_bias_m.0 - expected_clock_bias_m).abs() > 1.0e-6 {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Error,
                        "GNSS_NAV_CLOCK_BIAS_UNITS_INCONSISTENT",
                        "nav solution clock bias meters do not match clock bias seconds",
                    ));
                }
            }
            if !self.pdop.is_finite()
                || self.hdop.is_some_and(|value| !value.is_finite())
                || self.vdop.is_some_and(|value| !value.is_finite())
                || self.gdop.is_some_and(|value| !value.is_finite())
                || self.tdop.is_some_and(|value| !value.is_finite())
            {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "GNSS_NAV_DOPS_INVALID",
                    "nav solution DOP values contain NaN/Inf",
                ));
            }
            if self.pre_fit_residual_rms_m.is_some_and(|value| !value.0.is_finite())
                || self.post_fit_residual_rms_m.is_some_and(|value| !value.0.is_finite())
            {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "GNSS_NAV_RESIDUAL_RMS_INVALID",
                    "nav solution residual RMS values contain NaN/Inf",
                ));
            }
            if self.pre_fit_residual_rms_m.is_some() ^ self.post_fit_residual_rms_m.is_some() {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Warning,
                    "GNSS_NAV_RESIDUAL_RMS_INCOMPLETE",
                    "nav solution should either provide both residual RMS values or neither",
                ));
            }
            if let Some(post_fit_residual_rms_m) = self.post_fit_residual_rms_m {
                if (post_fit_residual_rms_m.0 - self.rms_m.0).abs() > 1.0e-9 {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Warning,
                        "GNSS_NAV_POST_FIT_RMS_MISMATCH",
                        "nav solution post-fit residual RMS does not match rms_m",
                    ));
                }
            }
            if self.model_version == 0 {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "GNSS_NAV_MODEL_VERSION_INVALID",
                    "nav solution model_version must be non-zero",
                ));
            }
            if self.sat_count != self.used_sat_count + self.rejected_sat_count {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "GNSS_NAV_SAT_COUNTS_INVALID",
                    "sat_count does not match used_sat_count + rejected_sat_count",
                ));
            }
            if self.valid != self.status.is_valid() {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Warning,
                    "GNSS_NAV_STATUS_VALID_MISMATCH",
                    "nav solution valid flag does not match status validity",
                ));
            }
            if self.artifact_id.is_empty() {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Warning,
                    "GNSS_NAV_ARTIFACT_ID_MISSING",
                    "nav solution artifact_id is empty",
                ));
            }
            if self.source_observation_epoch_id.is_empty() {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Warning,
                    "GNSS_NAV_SOURCE_EPOCH_ID_MISSING",
                    "nav solution source_observation_epoch_id is empty",
                ));
            }
            events.extend(validate_receiver_sample_trace(
                self.source_time,
                "navigation",
                Some(self.t_rx_s),
                None,
            ));
            if self.refusal_class.is_some() && self.explain_decision.is_empty() {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Warning,
                    "GNSS_NAV_EXPLAIN_DECISION_MISSING",
                    "nav solution refusal_class requires explain_decision",
                ));
            }
            if self.stability_signature.is_empty() {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Warning,
                    "GNSS_NAV_STABILITY_SIGNATURE_MISSING",
                    "nav solution stability signature is empty",
                ));
            }
            if self.stability_signature_version == 0 {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Warning,
                    "GNSS_NAV_STABILITY_SIGNATURE_VERSION_INVALID",
                    "nav solution stability signature version is zero",
                ));
            }
            if matches!(
                self.status,
                SolutionStatus::Unavailable
                    | SolutionStatus::Refused
                    | SolutionStatus::IntegrityFailed
                    | SolutionStatus::Diverged
            ) && self.refusal_class.is_none()
            {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Warning,
                    "GNSS_NAV_REFUSAL_CLASS_MISSING",
                    "non-usable nav solution should carry a refusal_class",
                ));
            }
            if !self.status.is_valid()
                && (self.integrity_hpl_m.is_some() || self.integrity_vpl_m.is_some())
            {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Warning,
                    "GNSS_NAV_INTEGRITY_CLAIMS_INVALID",
                    "non-usable nav solution should not carry integrity protection levels",
                ));
            }
            if !self.residuals.is_empty() && self.post_fit_residual_rms_m.is_none() {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Warning,
                    "GNSS_NAV_POST_FIT_RMS_MISSING",
                    "nav solution with residuals should carry post-fit residual RMS",
                ));
            }
            if !self.residuals.is_empty() && self.constellation_residual_rms.is_empty() {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Warning,
                    "GNSS_NAV_CONSTELLATION_RMS_MISSING",
                    "nav solution with residuals should carry per-constellation residual RMS",
                ));
            }
            let accepted_residual_counts = self
                .residuals
                .iter()
                .filter(|residual| !residual.rejected)
                .fold(std::collections::BTreeMap::new(), |mut counts, residual| {
                    *counts.entry(residual.sat.constellation).or_insert(0usize) += 1;
                    counts
                });
            let mut summarized_post_fit_count = 0usize;
            let mut seen_constellations = std::collections::BTreeSet::new();
            for summary in &self.constellation_residual_rms {
                if !seen_constellations.insert(summary.constellation) {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Error,
                        "GNSS_NAV_CONSTELLATION_RMS_DUPLICATE",
                        "nav solution duplicates a constellation residual RMS entry",
                    ));
                }
                if summary.pre_fit_rms_m.is_some_and(|value| !value.0.is_finite())
                    || summary.post_fit_rms_m.is_some_and(|value| !value.0.is_finite())
                {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Error,
                        "GNSS_NAV_CONSTELLATION_RMS_INVALID",
                        "nav solution constellation residual RMS contains NaN/Inf",
                    ));
                }
                if summary.pre_fit_rms_m.is_some() && summary.pre_fit_sat_count == 0 {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Error,
                        "GNSS_NAV_CONSTELLATION_PRE_FIT_COUNT_INVALID",
                        "nav solution constellation pre-fit RMS requires a non-zero satellite count",
                    ));
                }
                if summary.post_fit_rms_m.is_some() && summary.post_fit_sat_count == 0 {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Error,
                        "GNSS_NAV_CONSTELLATION_POST_FIT_COUNT_INVALID",
                        "nav solution constellation post-fit RMS requires a non-zero satellite count",
                    ));
                }
                if summary.post_fit_sat_count > 0
                    && accepted_residual_counts
                        .get(&summary.constellation)
                        .copied()
                        .unwrap_or(0)
                        != summary.post_fit_sat_count
                {
                    events.push(DiagnosticEvent::new(
                        DiagnosticSeverity::Error,
                        "GNSS_NAV_CONSTELLATION_POST_FIT_COUNT_MISMATCH",
                        "nav solution constellation post-fit satellite counts do not match accepted residuals",
                    ));
                }
                summarized_post_fit_count += summary.post_fit_sat_count;
            }
            if !self.constellation_residual_rms.is_empty()
                && summarized_post_fit_count != accepted_residual_counts.values().sum::<usize>()
            {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "GNSS_NAV_CONSTELLATION_POST_FIT_TOTAL_MISMATCH",
                    "nav solution constellation residual RMS totals do not match accepted residuals",
                ));
            }
            events
        }
    }
}

pub mod rtk {
    //! RTK artifact aliases.
    use super::*;

    /// Single-difference artifact v1.
    pub type RtkSdEpochV1<T> = ArtifactV1<T>;
    /// Double-difference artifact v1.
    pub type RtkDdEpochV1<T> = ArtifactV1<T>;
    /// Baseline solution artifact v1.
    pub type RtkBaselineEpochV1<T> = ArtifactV1<T>;
    /// Baseline quality artifact v1.
    pub type RtkBaselineQualityV1<T> = ArtifactV1<T>;
    /// Fix audit artifact v1.
    pub type RtkFixAuditV1<T> = ArtifactV1<T>;
    /// Precision artifact v1.
    pub type RtkPrecisionV1<T> = ArtifactV1<T>;
}

pub mod ppp {
    //! PPP artifact aliases.
    use super::*;

    /// PPP solution artifact v1.
    pub type PppEpochV1<T> = ArtifactV1<T>;
}
