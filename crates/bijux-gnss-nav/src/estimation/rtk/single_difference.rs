use std::collections::BTreeMap;

use bijux_gnss_core::api::{
    AmbiguityId, ArtifactPayloadValidate, Constellation, DiagnosticEvent, DiagnosticSeverity,
    ObsEpoch, ObsSatellite, ObsSignalTiming, ObservationStatus, SigId,
};
use serde::{Deserialize, Serialize};

/// RTK single-difference observation formed as rover minus base for one signal.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RtkSingleDifferenceObservation {
    /// Signal identity shared by the base and rover observations.
    pub sig: SigId,
    /// Rover pseudorange used to form the single difference.
    pub rover_pseudorange_m: f64,
    /// Optional rover transmit-time timing carried from the observation.
    pub rover_signal_timing: Option<ObsSignalTiming>,
    /// Base pseudorange used to form the single difference.
    pub base_pseudorange_m: f64,
    /// Optional base transmit-time timing carried from the observation.
    pub base_signal_timing: Option<ObsSignalTiming>,
    /// Single-difference code observation in meters.
    pub code_m: f64,
    /// Single-difference carrier phase observation in cycles.
    pub phase_cycles: f64,
    /// Single-difference Doppler observation in hertz.
    pub doppler_hz: f64,
    /// Propagated single-difference code variance in square meters.
    pub code_variance_m2: f64,
    /// Propagated single-difference carrier variance in square cycles.
    pub phase_variance_cycles2: f64,
    /// Rover ambiguity identifier for the contributing carrier observation.
    pub ambiguity_rover: AmbiguityId,
    /// Base ambiguity identifier for the contributing carrier observation.
    pub ambiguity_base: AmbiguityId,
}

impl ArtifactPayloadValidate for RtkSingleDifferenceObservation {
    fn validate_payload(&self) -> Vec<DiagnosticEvent> {
        let mut events = Vec::new();
        if !self.rover_pseudorange_m.is_finite()
            || !self.base_pseudorange_m.is_finite()
            || !self.code_m.is_finite()
            || !self.phase_cycles.is_finite()
            || !self.doppler_hz.is_finite()
        {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_SD_NUMERIC_INVALID",
                "single-difference observation contains NaN/Inf",
            ));
        }
        if timing_is_invalid(self.rover_signal_timing) || timing_is_invalid(self.base_signal_timing)
        {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_SD_TIMING_INVALID",
                "single-difference timing contains NaN/Inf",
            ));
        }
        if self.code_variance_m2 < 0.0 || self.phase_variance_cycles2 < 0.0 {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_SD_VARIANCE_INVALID",
                "single-difference variance is negative",
            ));
        }
        events
    }
}

/// Build RTK single-difference observations from a base and rover epoch.
///
/// The sign convention is `rover - base`. Only accepted observations with finite
/// code, phase, and Doppler values and both code/carrier lock set are included.
pub fn rtk_single_differences_from_obs_epochs(
    base: &ObsEpoch,
    rover: &ObsEpoch,
) -> Vec<RtkSingleDifferenceObservation> {
    let mut base_by_signal: BTreeMap<SigId, &ObsSatellite> = BTreeMap::new();
    for sat in &base.sats {
        base_by_signal.insert(sat.signal_id, sat);
    }

    let mut out = Vec::new();
    for rover_sat in &rover.sats {
        let Some(base_sat) = base_by_signal.get(&rover_sat.signal_id) else {
            continue;
        };
        if !single_difference_input_is_usable(rover_sat)
            || !single_difference_input_is_usable(base_sat)
        {
            continue;
        }

        let rover_code_variance_m2 =
            observed_or_modeled_variance_m2(rover_sat, MeasurementKind::Code);
        let base_code_variance_m2 =
            observed_or_modeled_variance_m2(base_sat, MeasurementKind::Code);
        let rover_phase_variance_cycles2 =
            observed_or_modeled_variance_m2(rover_sat, MeasurementKind::Phase);
        let base_phase_variance_cycles2 =
            observed_or_modeled_variance_m2(base_sat, MeasurementKind::Phase);

        out.push(RtkSingleDifferenceObservation {
            sig: rover_sat.signal_id,
            rover_pseudorange_m: rover_sat.pseudorange_m.0,
            rover_signal_timing: rover_sat.timing,
            base_pseudorange_m: base_sat.pseudorange_m.0,
            base_signal_timing: base_sat.timing,
            code_m: rover_sat.pseudorange_m.0 - base_sat.pseudorange_m.0,
            phase_cycles: rover_sat.carrier_phase_cycles.0 - base_sat.carrier_phase_cycles.0,
            doppler_hz: rover_sat.doppler_hz.0 - base_sat.doppler_hz.0,
            code_variance_m2: rover_code_variance_m2 + base_code_variance_m2,
            phase_variance_cycles2: rover_phase_variance_cycles2 + base_phase_variance_cycles2,
            ambiguity_rover: ambiguity_id_from_satellite(rover_sat),
            ambiguity_base: ambiguity_id_from_satellite(base_sat),
        });
    }
    out.sort_by_key(|observation| observation.sig);
    out
}

/// Choose the lowest-variance reference signal for double-difference formation.
pub fn choose_rtk_single_difference_reference_signal(
    observations: &[RtkSingleDifferenceObservation],
) -> Option<SigId> {
    observations
        .iter()
        .min_by(|left, right| {
            left.code_variance_m2
                .partial_cmp(&right.code_variance_m2)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .map(|observation| observation.sig)
}

/// Choose one reference signal per constellation using the lowest code variance.
pub fn choose_rtk_single_difference_reference_signals_by_constellation(
    observations: &[RtkSingleDifferenceObservation],
) -> BTreeMap<Constellation, SigId> {
    let mut by_constellation: BTreeMap<Constellation, Vec<RtkSingleDifferenceObservation>> =
        BTreeMap::new();
    for observation in observations {
        by_constellation
            .entry(observation.ambiguity_rover.sig.sat.constellation)
            .or_default()
            .push(observation.clone());
    }

    let mut out = BTreeMap::new();
    for (constellation, items) in by_constellation {
        if let Some(sig) = choose_rtk_single_difference_reference_signal(&items) {
            out.insert(constellation, sig);
        }
    }
    out
}

fn ambiguity_id_from_satellite(sat: &ObsSatellite) -> AmbiguityId {
    AmbiguityId { sig: sat.signal_id, signal: format!("{:?}", sat.metadata.signal.band) }
}

fn single_difference_input_is_usable(sat: &ObsSatellite) -> bool {
    sat.observation_status == ObservationStatus::Accepted
        && sat.lock_flags.code_lock
        && sat.lock_flags.carrier_lock
        && sat.pseudorange_m.0.is_finite()
        && sat.carrier_phase_cycles.0.is_finite()
        && sat.doppler_hz.0.is_finite()
}

fn observed_or_modeled_variance_m2(sat: &ObsSatellite, measurement: MeasurementKind) -> f64 {
    let observed = match measurement {
        MeasurementKind::Code => sat.pseudorange_var_m2,
        MeasurementKind::Phase => sat.carrier_phase_var_cycles2,
    };
    if observed > 0.0 {
        observed
    } else {
        modeled_variance_m2(sat.cn0_dbhz, sat.elevation_deg, measurement)
    }
}

fn modeled_variance_m2(
    cn0_dbhz: f64,
    elevation_deg: Option<f64>,
    measurement: MeasurementKind,
) -> f64 {
    let cn0_linear = 10.0_f64.powf(cn0_dbhz / 10.0).max(1.0);
    let elevation_weight = elevation_deg.unwrap_or(30.0).to_radians().sin().max(0.1);
    let nominal_sigma = match measurement {
        MeasurementKind::Code => 10.0,
        MeasurementKind::Phase => 0.02,
    };
    let sigma = nominal_sigma / (cn0_linear.sqrt() * elevation_weight);
    sigma * sigma
}

fn timing_is_invalid(timing: Option<ObsSignalTiming>) -> bool {
    let Some(timing) = timing else {
        return false;
    };
    !timing.signal_travel_time_s.0.is_finite() || !timing.transmit_gps_time.tow_s.is_finite()
}

#[derive(Debug, Clone, Copy)]
enum MeasurementKind {
    Code,
    Phase,
}
