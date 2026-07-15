use bijux_gnss_core::api::{
    ObservationStatus, ObservationSupportClass, ObservationUncertaintyClass,
};

use super::CarrierPhaseContinuity;

pub(super) fn observation_status_label(status: ObservationStatus) -> &'static str {
    match status {
        ObservationStatus::Accepted => "accepted",
        ObservationStatus::Missing => "missing",
        ObservationStatus::Weak => "weak",
        ObservationStatus::Inconsistent => "inconsistent",
        ObservationStatus::Rejected => "rejected",
    }
}

pub(super) fn observation_support_label(
    status: ObservationStatus,
    alignment_resolved: bool,
) -> &'static str {
    match status {
        ObservationStatus::Accepted if alignment_resolved => {
            support_class_label(ObservationSupportClass::Supported)
        }
        ObservationStatus::Accepted => support_class_label(ObservationSupportClass::Degraded),
        ObservationStatus::Weak | ObservationStatus::Missing => {
            support_class_label(ObservationSupportClass::Degraded)
        }
        ObservationStatus::Inconsistent | ObservationStatus::Rejected => {
            support_class_label(ObservationSupportClass::Unsupported)
        }
    }
}

pub(super) fn doppler_model_label() -> &'static str {
    bijux_gnss_core::api::OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET
}

pub(super) fn pseudorange_model_has_resolved_alignment(model: &str) -> bool {
    matches!(model, "tracked_code_phase_alignment" | "decoded_transmit_time_code_phase")
}

pub(super) fn carrier_phase_continuity_label(value: CarrierPhaseContinuity) -> &'static str {
    match value {
        CarrierPhaseContinuity::Unusable => "unusable",
        CarrierPhaseContinuity::ArcStart => "arc_start",
        CarrierPhaseContinuity::Continuous => "continuous",
        CarrierPhaseContinuity::Coasted => "coasted",
        CarrierPhaseContinuity::ResetAfterCycleSlip => "reset_after_cycle_slip",
        CarrierPhaseContinuity::ResetAfterUnlock => "reset_after_unlock",
        CarrierPhaseContinuity::ResetAfterReacquisition => "reset_after_reacquisition",
        CarrierPhaseContinuity::ResetAfterDiscontinuity => "reset_after_discontinuity",
    }
}

fn support_class_label(value: ObservationSupportClass) -> &'static str {
    match value {
        ObservationSupportClass::Supported => "supported",
        ObservationSupportClass::Degraded => "degraded",
        ObservationSupportClass::Unsupported => "unsupported",
    }
}

pub(super) fn observation_uncertainty_label(
    cn0_dbhz: f64,
    has_variance_evidence: bool,
) -> &'static str {
    let class = if !has_variance_evidence || !cn0_dbhz.is_finite() {
        ObservationUncertaintyClass::Unknown
    } else if cn0_dbhz >= 45.0 {
        ObservationUncertaintyClass::Low
    } else if cn0_dbhz >= 35.0 {
        ObservationUncertaintyClass::Medium
    } else {
        ObservationUncertaintyClass::High
    };
    match class {
        ObservationUncertaintyClass::Low => "low",
        ObservationUncertaintyClass::Medium => "medium",
        ObservationUncertaintyClass::High => "high",
        ObservationUncertaintyClass::Unknown => "unknown",
    }
}
