use crate::api::{Cycles, Hertz, Meters, Seconds, SigId};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct AmbiguityId {
    pub sig: SigId,
    pub signal: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AmbiguityStatus {
    Unknown,
    Float,
    Fixed,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AmbiguityState {
    pub id: AmbiguityId,
    pub float_cycles: Cycles,
    pub variance: f64,
    pub status: AmbiguityStatus,
    pub last_update_epoch: u64,
    pub carrier_phase_arc_id: Option<String>,
    pub valid_for_carrier_phase_arc: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub(crate) struct CarrierPhaseTerms {
    pub range_m: Meters,
    pub receiver_clock_s: Seconds,
    pub sat_clock_s: Seconds,
    pub tropo_m: Meters,
    pub iono_m: Meters,
    pub wavelength_m: Meters,
    pub ambiguity_cycles: Cycles,
}

pub(crate) fn carrier_phase_cycles(terms: &CarrierPhaseTerms) -> f64 {
    let corrected_m = terms.range_m.0
        + 299_792_458.0 * (terms.receiver_clock_s.0 - terms.sat_clock_s.0)
        + terms.tropo_m.0
        - terms.iono_m.0;
    corrected_m / terms.wavelength_m.0 + terms.ambiguity_cycles.0
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SingleDifference {
    pub sig: SigId,
    pub code_m: Meters,
    pub phase_cycles: Cycles,
    pub doppler_hz: Hertz,
    pub ambiguity_rover: AmbiguityId,
    pub ambiguity_base: AmbiguityId,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DoubleDifference {
    pub ref_sig: SigId,
    pub sig: SigId,
    pub code_m: Meters,
    pub phase_cycles: Cycles,
    pub doppler_hz: Hertz,
    pub canceled: Vec<AmbiguityId>,
}

pub(crate) fn geometry_free_phase_m(
    phase1_cycles: f64,
    phase2_cycles: f64,
    lambda1_m: f64,
    lambda2_m: f64,
) -> f64 {
    phase1_cycles * lambda1_m - phase2_cycles * lambda2_m
}

pub(crate) fn melbourne_wubbena_m(
    code1_m: f64,
    code2_m: f64,
    phase1_cycles: f64,
    phase2_cycles: f64,
    lambda1_m: f64,
    lambda2_m: f64,
) -> f64 {
    let phi1 = phase1_cycles * lambda1_m;
    let phi2 = phase2_cycles * lambda2_m;
    (phi1 - phi2) - (code1_m - code2_m)
}
