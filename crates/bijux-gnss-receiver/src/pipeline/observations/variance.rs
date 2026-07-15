use bijux_gnss_core::api::{
    MeasurementErrorModel, Meters, ObsSatellite, ObservationStatus, TrackEpoch,
};

use crate::pipeline::observations::receiver_clock::{
    receiver_clock_error_m, ObservationReceiverClock,
};

use super::status::push_observation_reject_reason;

#[derive(Debug, Clone)]
pub(super) struct ObservationVarianceEvidence {
    pub(super) pseudorange_var_m2: f64,
    pub(super) carrier_phase_var_cycles2: f64,
    pub(super) doppler_var_hz2: f64,
    pub(super) pseudorange_error_m: MeasurementErrorModel,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) enum ObservationVarianceFailure {
    Missing(Vec<&'static str>),
    Invalid(Vec<&'static str>),
}

#[derive(Debug, Clone)]
struct PseudorangeUncertaintyModel {
    sigma_m: f64,
    pseudorange_error_m: MeasurementErrorModel,
}

pub(super) fn finite_value(value: Option<f64>) -> Option<f64> {
    value.filter(|candidate| candidate.is_finite())
}

pub(super) fn finite_sigma(value: Option<f64>) -> Option<f64> {
    value.filter(|candidate| positive_finite_sigma(*candidate))
}

fn sigma_from_variance(variance: f64) -> Option<f64> {
    finite_sigma((variance.is_finite() && variance > 0.0).then(|| variance.sqrt()))
}

pub(super) fn evidence_sigma_from_variance(sat: &ObsSatellite, variance: f64) -> Option<f64> {
    has_variance_evidence(sat).then(|| sigma_from_variance(variance)).flatten()
}

pub(super) fn has_variance_evidence(sat: &ObsSatellite) -> bool {
    sat.error_model.is_some()
        && positive_finite_sigma(sat.pseudorange_var_m2.sqrt())
        && positive_finite_sigma(sat.carrier_phase_var_cycles2.sqrt())
        && positive_finite_sigma(sat.doppler_var_hz2.sqrt())
}

pub(super) fn observation_variance_evidence(
    epoch: &TrackEpoch,
    meters_per_sample: f64,
    receiver_clock: &ObservationReceiverClock,
) -> Result<ObservationVarianceEvidence, ObservationVarianceFailure> {
    let Some(uncertainty) = epoch.tracking_uncertainty.as_ref() else {
        return Err(ObservationVarianceFailure::Missing(vec!["missing_tracking_uncertainty"]));
    };

    let mut invalid = Vec::new();
    if !positive_finite_sigma(uncertainty.code_phase_samples) {
        invalid.push("invalid_tracking_uncertainty_code_phase");
    }
    if !positive_finite_sigma(uncertainty.carrier_phase_cycles) {
        invalid.push("invalid_tracking_uncertainty_carrier_phase");
    }
    if !positive_finite_sigma(uncertainty.doppler_hz) {
        invalid.push("invalid_tracking_uncertainty_doppler");
    }
    if !positive_finite_sigma(uncertainty.cn0_dbhz) {
        invalid.push("invalid_tracking_uncertainty_cn0");
    }
    if !invalid.is_empty() {
        return Err(ObservationVarianceFailure::Invalid(invalid));
    }

    let pseudorange_uncertainty = pseudorange_uncertainty_model(
        uncertainty.code_phase_samples,
        meters_per_sample,
        receiver_clock,
    );

    Ok(ObservationVarianceEvidence {
        pseudorange_var_m2: pseudorange_uncertainty.sigma_m.powi(2),
        carrier_phase_var_cycles2: uncertainty.carrier_phase_cycles.powi(2),
        doppler_var_hz2: uncertainty.doppler_hz.powi(2),
        pseudorange_error_m: pseudorange_uncertainty.pseudorange_error_m,
    })
}

fn pseudorange_uncertainty_model(
    code_phase_sigma_samples: f64,
    meters_per_sample: f64,
    receiver_clock: &ObservationReceiverClock,
) -> PseudorangeUncertaintyModel {
    let tracking_jitter_m = code_phase_sigma_samples * meters_per_sample;

    PseudorangeUncertaintyModel {
        sigma_m: tracking_jitter_m,
        pseudorange_error_m: MeasurementErrorModel {
            thermal_noise_m: Meters(0.0),
            tracking_jitter_m: Meters(tracking_jitter_m),
            multipath_proxy_m: Meters(0.0),
            clock_error_m: receiver_clock_error_m(receiver_clock),
        },
    }
}

fn positive_finite_sigma(value: f64) -> bool {
    value.is_finite() && value > 0.0
}

pub(super) fn apply_variance_evidence_status(
    status: &mut ObservationStatus,
    reasons: &mut Vec<String>,
    failure: Option<&ObservationVarianceFailure>,
) {
    let Some(failure) = failure else {
        return;
    };
    match failure {
        ObservationVarianceFailure::Missing(missing_reasons) => {
            if *status == ObservationStatus::Accepted || *status == ObservationStatus::Weak {
                *status = ObservationStatus::Missing;
            }
            for reason in missing_reasons {
                push_observation_reject_reason(reasons, reason);
            }
        }
        ObservationVarianceFailure::Invalid(invalid_reasons) => {
            *status = ObservationStatus::Inconsistent;
            for reason in invalid_reasons {
                push_observation_reject_reason(reasons, reason);
            }
        }
    }
}

pub(super) fn observation_error_model(
    sat: &ObsSatellite,
    multipath_proxy_m: f64,
) -> Option<MeasurementErrorModel> {
    let model = sat.error_model.as_ref()?;

    Some(MeasurementErrorModel {
        thermal_noise_m: model.thermal_noise_m,
        tracking_jitter_m: model.tracking_jitter_m,
        multipath_proxy_m: Meters(multipath_proxy_m),
        clock_error_m: model.clock_error_m,
    })
}
