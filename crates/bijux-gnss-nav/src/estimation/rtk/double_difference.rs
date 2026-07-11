use std::collections::BTreeMap;

use bijux_gnss_core::api::{
    AmbiguityId, ArtifactPayloadValidate, Constellation, DiagnosticEvent, DiagnosticSeverity,
    ObsSignalTiming, SigId,
};
use serde::{Deserialize, Serialize};

use super::single_difference::RtkSingleDifferenceObservation;
use crate::orbits::gps::{sat_state_gps_l1ca_from_observation, GpsEphemeris};

/// RTK double-difference observation formed against a reference satellite.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RtkDoubleDifferenceObservation {
    /// Signal identity for the non-reference satellite.
    pub sig: SigId,
    /// Signal identity for the chosen reference satellite.
    pub ref_sig: SigId,
    /// Lowest contributing C/N0 across all four observations in the double difference.
    pub min_cn0_dbhz: f64,
    /// Whether any contributing observation was marked multipath-suspect.
    pub multipath_suspect: bool,
    /// Rover pseudorange for the non-reference satellite.
    pub rover_signal_pseudorange_m: f64,
    /// Optional rover timing for the non-reference satellite.
    pub rover_signal_timing: Option<ObsSignalTiming>,
    /// Base-station pseudorange for the non-reference satellite.
    pub base_signal_pseudorange_m: f64,
    /// Optional base-station timing for the non-reference satellite.
    pub base_signal_timing: Option<ObsSignalTiming>,
    /// Rover pseudorange for the reference satellite.
    pub rover_ref_pseudorange_m: f64,
    /// Optional rover timing for the reference satellite.
    pub rover_ref_signal_timing: Option<ObsSignalTiming>,
    /// Base-station pseudorange for the reference satellite.
    pub base_ref_pseudorange_m: f64,
    /// Optional base-station timing for the reference satellite.
    pub base_ref_signal_timing: Option<ObsSignalTiming>,
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

/// Residual summary for a batch of RTK double-difference code observations.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RtkDoubleDifferenceResidualMetrics {
    /// Root-mean-square code residual against the provided baseline truth.
    pub residual_rms_m: f64,
    /// Root-mean-square modeled code variance carried by the observations.
    pub predicted_rms_m: f64,
    /// Number of observations contributing to the summary.
    pub used_observations: usize,
}

impl ArtifactPayloadValidate for RtkDoubleDifferenceObservation {
    fn validate_payload(&self) -> Vec<DiagnosticEvent> {
        let mut events = Vec::new();
        if !self.rover_signal_pseudorange_m.is_finite()
            || !self.base_signal_pseudorange_m.is_finite()
            || !self.rover_ref_pseudorange_m.is_finite()
            || !self.base_ref_pseudorange_m.is_finite()
            || !self.code_m.is_finite()
            || !self.phase_cycles.is_finite()
            || !self.doppler_hz.is_finite()
            || !self.min_cn0_dbhz.is_finite()
        {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_DD_NUMERIC_INVALID",
                "double-difference observation contains NaN/Inf",
            ));
        }
        if timing_is_invalid(self.rover_signal_timing)
            || timing_is_invalid(self.base_signal_timing)
            || timing_is_invalid(self.rover_ref_signal_timing)
            || timing_is_invalid(self.base_ref_signal_timing)
        {
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
            min_cn0_dbhz: observation.min_cn0_dbhz.min(reference.min_cn0_dbhz),
            multipath_suspect: observation.multipath_suspect || reference.multipath_suspect,
            rover_signal_pseudorange_m: observation.rover_pseudorange_m,
            rover_signal_timing: observation.rover_signal_timing,
            base_signal_pseudorange_m: observation.base_pseudorange_m,
            base_signal_timing: observation.base_signal_timing,
            rover_ref_pseudorange_m: reference.rover_pseudorange_m,
            rover_ref_signal_timing: reference.rover_signal_timing,
            base_ref_pseudorange_m: reference.base_pseudorange_m,
            base_ref_signal_timing: reference.base_signal_timing,
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

/// Evaluate double-difference code residuals against a known rover-base baseline.
pub fn rtk_double_difference_residual_metrics(
    observations: &[RtkDoubleDifferenceObservation],
    base_ecef_m: [f64; 3],
    rover_enu_m: [f64; 3],
    ephemerides: &[GpsEphemeris],
    receive_time_s: f64,
) -> Option<RtkDoubleDifferenceResidualMetrics> {
    if observations.is_empty() {
        return None;
    }

    let rover_ecef_m = enu_to_ecef(base_ecef_m, rover_enu_m);
    let mut residuals_m = Vec::new();
    let mut predicted_variances_m2 = Vec::new();
    for observation in observations {
        let signal_ephemeris =
            ephemerides.iter().find(|candidate| candidate.sat == observation.sig.sat)?;
        let reference_ephemeris =
            ephemerides.iter().find(|candidate| candidate.sat == observation.ref_sig.sat)?;
        let rover_signal_satellite = sat_state_gps_l1ca_from_observation(
            signal_ephemeris,
            receive_time_s,
            observation.rover_signal_pseudorange_m,
            observation.rover_signal_timing,
        );
        let base_signal_satellite = sat_state_gps_l1ca_from_observation(
            signal_ephemeris,
            receive_time_s,
            observation.base_signal_pseudorange_m,
            observation.base_signal_timing,
        );
        let rover_reference_satellite = sat_state_gps_l1ca_from_observation(
            reference_ephemeris,
            receive_time_s,
            observation.rover_ref_pseudorange_m,
            observation.rover_ref_signal_timing,
        );
        let base_reference_satellite = sat_state_gps_l1ca_from_observation(
            reference_ephemeris,
            receive_time_s,
            observation.base_ref_pseudorange_m,
            observation.base_ref_signal_timing,
        );
        let modeled_code_m = (modeled_pseudorange_m(
            rover_ecef_m,
            [rover_signal_satellite.x_m, rover_signal_satellite.y_m, rover_signal_satellite.z_m],
            rover_signal_satellite.clock_correction.bias_s,
        ) - modeled_pseudorange_m(
            base_ecef_m,
            [base_signal_satellite.x_m, base_signal_satellite.y_m, base_signal_satellite.z_m],
            base_signal_satellite.clock_correction.bias_s,
        )) - (modeled_pseudorange_m(
            rover_ecef_m,
            [
                rover_reference_satellite.x_m,
                rover_reference_satellite.y_m,
                rover_reference_satellite.z_m,
            ],
            rover_reference_satellite.clock_correction.bias_s,
        ) - modeled_pseudorange_m(
            base_ecef_m,
            [
                base_reference_satellite.x_m,
                base_reference_satellite.y_m,
                base_reference_satellite.z_m,
            ],
            base_reference_satellite.clock_correction.bias_s,
        ));
        residuals_m.push(observation.code_m - modeled_code_m);
        predicted_variances_m2.push(observation.code_variance_m2.max(1.0e-6));
    }

    let residual_rms_m = (residuals_m.iter().map(|residual| residual * residual).sum::<f64>()
        / residuals_m.len() as f64)
        .sqrt();
    let predicted_rms_m =
        (predicted_variances_m2.iter().sum::<f64>() / predicted_variances_m2.len() as f64).sqrt();
    Some(RtkDoubleDifferenceResidualMetrics {
        residual_rms_m,
        predicted_rms_m,
        used_observations: residuals_m.len(),
    })
}

fn timing_is_invalid(timing: Option<ObsSignalTiming>) -> bool {
    let Some(timing) = timing else {
        return false;
    };
    !timing.signal_travel_time_s.0.is_finite() || !timing.transmit_gps_time.tow_s.is_finite()
}

fn enu_to_ecef(base_ecef_m: [f64; 3], enu_m: [f64; 3]) -> [f64; 3] {
    let (lat_deg, lon_deg, _alt_m) = crate::estimation::position::solver::ecef_to_geodetic(
        base_ecef_m[0],
        base_ecef_m[1],
        base_ecef_m[2],
    );
    let (sin_lat, cos_lat) = lat_deg.to_radians().sin_cos();
    let (sin_lon, cos_lon) = lon_deg.to_radians().sin_cos();
    let east_m = enu_m[0];
    let north_m = enu_m[1];
    let up_m = enu_m[2];
    let dx = -sin_lon * east_m - sin_lat * cos_lon * north_m + cos_lat * cos_lon * up_m;
    let dy = cos_lon * east_m - sin_lat * sin_lon * north_m + cos_lat * sin_lon * up_m;
    let dz = cos_lat * north_m + sin_lat * up_m;
    [base_ecef_m[0] + dx, base_ecef_m[1] + dy, base_ecef_m[2] + dz]
}

fn modeled_pseudorange_m(
    receiver_ecef_m: [f64; 3],
    sat_ecef_m: [f64; 3],
    sat_clock_bias_s: f64,
) -> f64 {
    geometric_range_m(receiver_ecef_m, sat_ecef_m) - sat_clock_bias_s * 299_792_458.0
}

fn geometric_range_m(receiver_ecef_m: [f64; 3], sat_ecef_m: [f64; 3]) -> f64 {
    let dx = receiver_ecef_m[0] - sat_ecef_m[0];
    let dy = receiver_ecef_m[1] - sat_ecef_m[1];
    let dz = receiver_ecef_m[2] - sat_ecef_m[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}
