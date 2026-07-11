use std::collections::BTreeMap;

use bijux_gnss_core::api::{
    AmbiguityId, ArtifactPayloadValidate, Constellation, DiagnosticEvent, DiagnosticSeverity,
    ObsSignalTiming, SigId,
};
use serde::{Deserialize, Serialize};

use super::single_difference::RtkSingleDifferenceObservation;

/// RTK double-difference observation formed against a reference satellite.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RtkDoubleDifferenceObservation {
    /// Signal identity for the non-reference satellite.
    pub sig: SigId,
    /// Signal identity for the chosen reference satellite.
    pub ref_sig: SigId,
    /// Base-station pseudorange used to reconstruct the non-reference satellite state.
    pub signal_pseudorange_m: f64,
    /// Optional base-station timing used to reconstruct the non-reference satellite state.
    pub signal_timing: Option<ObsSignalTiming>,
    /// Base-station pseudorange used to reconstruct the reference satellite state.
    pub ref_signal_pseudorange_m: f64,
    /// Optional base-station timing used to reconstruct the reference satellite state.
    pub ref_signal_timing: Option<ObsSignalTiming>,
    /// Double-difference code observation in meters.
    pub code_m: f64,
    /// Double-difference carrier phase observation in cycles.
    pub phase_cycles: f64,
    /// Double-difference Doppler observation in hertz.
    pub doppler_hz: f64,
    /// Propagated double-difference code variance in square meters.
    pub code_variance_m2: f64,
    /// Propagated double-difference carrier variance in square cycles.
    pub phase_variance_cycles2: f64,
    /// Ambiguity identifiers canceled by the double difference.
    pub canceled: Vec<AmbiguityId>,
}

impl ArtifactPayloadValidate for RtkDoubleDifferenceObservation {
    fn validate_payload(&self) -> Vec<DiagnosticEvent> {
        let mut events = Vec::new();
        if !self.signal_pseudorange_m.is_finite()
            || !self.ref_signal_pseudorange_m.is_finite()
            || !self.code_m.is_finite()
            || !self.phase_cycles.is_finite()
            || !self.doppler_hz.is_finite()
        {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_DD_NUMERIC_INVALID",
                "double-difference observation contains NaN/Inf",
            ));
        }
        if timing_is_invalid(self.signal_timing) || timing_is_invalid(self.ref_signal_timing) {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_DD_TIMING_INVALID",
                "double-difference timing contains NaN/Inf",
            ));
        }
        if self.code_variance_m2 < 0.0 || self.phase_variance_cycles2 < 0.0 {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_DD_VARIANCE_INVALID",
                "double-difference variance is negative",
            ));
        }
        events
    }
}

/// Build RTK double-difference observations from single differences and a reference signal.
pub fn rtk_double_differences_from_single_differences(
    observations: &[RtkSingleDifferenceObservation],
    ref_sig: SigId,
) -> Vec<RtkDoubleDifferenceObservation> {
    let Some(reference) = observations.iter().find(|observation| observation.sig == ref_sig) else {
        return Vec::new();
    };

    let mut out = Vec::new();
    for observation in observations {
        if observation.sig == ref_sig {
            continue;
        }
        if observation.sig.sat.constellation != reference.sig.sat.constellation {
            continue;
        }
        out.push(RtkDoubleDifferenceObservation {
            sig: observation.sig,
            ref_sig: reference.sig,
            signal_pseudorange_m: observation.base_pseudorange_m,
            signal_timing: observation.base_signal_timing,
            ref_signal_pseudorange_m: reference.base_pseudorange_m,
            ref_signal_timing: reference.base_signal_timing,
            code_m: observation.code_m - reference.code_m,
            phase_cycles: observation.phase_cycles - reference.phase_cycles,
            doppler_hz: observation.doppler_hz - reference.doppler_hz,
            code_variance_m2: observation.code_variance_m2 + reference.code_variance_m2,
            phase_variance_cycles2: observation.phase_variance_cycles2
                + reference.phase_variance_cycles2,
            canceled: vec![
                observation.ambiguity_rover.clone(),
                observation.ambiguity_base.clone(),
                reference.ambiguity_rover.clone(),
                reference.ambiguity_base.clone(),
            ],
        });
    }
    out.sort_by_key(|observation| observation.sig);
    out
}

/// Build RTK double differences independently per constellation.
pub fn rtk_double_differences_by_constellation(
    observations: &[RtkSingleDifferenceObservation],
    references: &BTreeMap<Constellation, SigId>,
) -> Vec<RtkDoubleDifferenceObservation> {
    let mut out = Vec::new();
    for (constellation, ref_sig) in references {
        let subset: Vec<RtkSingleDifferenceObservation> = observations
            .iter()
            .filter(|observation| observation.sig.sat.constellation == *constellation)
            .cloned()
            .collect();
        out.extend(rtk_double_differences_from_single_differences(&subset, *ref_sig));
    }
    out.sort_by_key(|observation| (observation.ref_sig, observation.sig));
    out
}

fn timing_is_invalid(timing: Option<ObsSignalTiming>) -> bool {
    let Some(timing) = timing else {
        return false;
    };
    !timing.signal_travel_time_s.0.is_finite() || !timing.transmit_gps_time.tow_s.is_finite()
}
