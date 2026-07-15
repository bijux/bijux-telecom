#![allow(missing_docs)]

use crate::corrections::broadcast_group_delay::{
    beidou_broadcast_group_delay_code_bias_m, galileo_broadcast_group_delay_code_bias_m,
    gps_broadcast_group_delay_code_bias_m,
};
use crate::orbits::beidou::{beidou_navigation_age, sat_state_beidou_b1i};
use crate::orbits::galileo::{galileo_navigation_age, sat_state_galileo_e1};
use crate::orbits::glonass::{
    glonass_gps_minus_glonass_s, glonass_navigation_age, glonass_slot_channel_association,
    sat_state_glonass_l1,
};
use crate::orbits::gps::{gps_ephemeris_age, sat_state_gps_l1ca};
use crate::orbits::satellite_uncertainty::SatelliteStateUncertainty;
use bijux_gnss_core::api::{Constellation, MeasurementRejectReason, ObsSignalTiming, SatId, SigId};
use bijux_gnss_signal::api::{default_acquisition_signal, signal_id_wavelength_m};
use serde::{Deserialize, Serialize};

use super::solver::{PositionBroadcastNavigation, PositionObservation};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
const NAVIGATION_FUTURE_TOLERANCE_S: f64 = 1.0;

#[derive(Debug, Clone)]
pub(crate) struct PositionSolveInput {
    pub(crate) observation: PositionObservation,
    pub(crate) navigation: PositionBroadcastNavigation,
    pub(crate) receive_tow_s: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct SatelliteState {
    pub x_m: f64,
    pub y_m: f64,
    pub z_m: f64,
    pub vx_mps: f64,
    pub vy_mps: f64,
    pub vz_mps: f64,
    pub clock_bias_s: f64,
    pub clock_drift_s_per_s: f64,
    pub uncertainty: SatelliteStateUncertainty,
}

pub type PositionSatelliteState = SatelliteState;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PositionObservationCorrectionKind {
    SatelliteClock,
    Relativity,
    EarthRotation,
    BroadcastGroupDelay,
    Ionosphere,
    Troposphere,
    ReceiverAntenna,
    SatelliteAntenna,
    EarthTide,
    PhaseWindup,
    ReceiverClock,
}

pub const POSITION_OBSERVATION_CORRECTION_ORDER: [PositionObservationCorrectionKind; 11] = [
    PositionObservationCorrectionKind::SatelliteClock,
    PositionObservationCorrectionKind::Relativity,
    PositionObservationCorrectionKind::EarthRotation,
    PositionObservationCorrectionKind::BroadcastGroupDelay,
    PositionObservationCorrectionKind::Ionosphere,
    PositionObservationCorrectionKind::Troposphere,
    PositionObservationCorrectionKind::ReceiverAntenna,
    PositionObservationCorrectionKind::SatelliteAntenna,
    PositionObservationCorrectionKind::EarthTide,
    PositionObservationCorrectionKind::PhaseWindup,
    PositionObservationCorrectionKind::ReceiverClock,
];

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PositionObservationCorrectionComponent {
    pub kind: PositionObservationCorrectionKind,
    pub delta_m: f64,
    pub corrected_pseudorange_m: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PositionObservationCorrectionChain {
    pub raw_pseudorange_m: f64,
    pub components: Vec<PositionObservationCorrectionComponent>,
    pub corrected_pseudorange_m: f64,
}

impl PositionObservationCorrectionChain {
    pub fn new(raw_pseudorange_m: f64) -> Self {
        Self {
            raw_pseudorange_m,
            components: Vec::new(),
            corrected_pseudorange_m: raw_pseudorange_m,
        }
    }

    pub fn push_component(&mut self, kind: PositionObservationCorrectionKind, delta_m: f64) {
        self.corrected_pseudorange_m += delta_m;
        self.components.push(PositionObservationCorrectionComponent {
            kind,
            delta_m,
            corrected_pseudorange_m: self.corrected_pseudorange_m,
        });
    }

    pub fn reconstructed_pseudorange_m(&self) -> f64 {
        self.components.iter().fold(self.raw_pseudorange_m, |pseudorange_m, component| {
            pseudorange_m + component.delta_m
        })
    }

    pub fn reconstruction_error_m(&self) -> f64 {
        self.corrected_pseudorange_m - self.reconstructed_pseudorange_m()
    }

    pub fn component_delta_m(&self, kind: PositionObservationCorrectionKind) -> f64 {
        self.components
            .iter()
            .filter(|component| component.kind == kind)
            .map(|component| component.delta_m)
            .sum()
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct ObservationConsistencyMetrics {
    pub(crate) code_residual_m: f64,
    pub(crate) doppler_residual_hz: Option<f64>,
}

pub(crate) fn unknown_inter_system_time_offset_sats(
    observations: &[PositionObservation],
    navigation: &[PositionBroadcastNavigation],
) -> Vec<SatId> {
    let unique_constellations = observations
        .iter()
        .map(|observation| observation.sat.constellation)
        .collect::<std::collections::BTreeSet<_>>();
    if unique_constellations.len() < 2 {
        return Vec::new();
    }

    let mut unknown = Vec::new();
    for observation in observations {
        let candidates =
            navigation.iter().filter(|entry| entry.sat() == observation.sat).collect::<Vec<_>>();
        if candidates.is_empty() {
            continue;
        }
        if candidates.iter().all(|entry| !navigation_time_relationship_is_known(entry)) {
            unknown.push(observation.sat);
        }
    }
    unknown
}

pub(crate) fn navigation_time_relationship_is_known(
    navigation: &PositionBroadcastNavigation,
) -> bool {
    match navigation {
        PositionBroadcastNavigation::Gps(_) => true,
        PositionBroadcastNavigation::Galileo(_) => true,
        PositionBroadcastNavigation::Beidou(_) => true,
        PositionBroadcastNavigation::Glonass(navigation) => {
            glonass_gps_minus_glonass_s(navigation).is_some()
        }
    }
}

pub(crate) fn resolve_position_inputs(
    observations: &[PositionObservation],
    navigation: &[PositionBroadcastNavigation],
    t_rx_s: f64,
    rejected: &mut Vec<(SatId, MeasurementRejectReason)>,
) -> Vec<PositionSolveInput> {
    observations
        .iter()
        .filter_map(|obs| {
            let receive_tow_s =
                obs.gps_receive_time.map(|gps_time| gps_time.tow_s).unwrap_or(t_rx_s);
            let navigation =
                match select_navigation_with_rejection(navigation, obs.sat, receive_tow_s) {
                    Ok(navigation) => navigation,
                    Err(reason) => {
                        rejected.push((obs.sat, reason));
                        return None;
                    }
                };
            Some(PositionSolveInput {
                observation: obs.clone(),
                navigation: navigation.clone(),
                receive_tow_s,
            })
        })
        .collect()
}

pub(crate) fn select_valid_navigation(
    navigation: &[PositionBroadcastNavigation],
    sat: SatId,
    receive_tow_s: f64,
) -> Option<&PositionBroadcastNavigation> {
    select_navigation_with_rejection(navigation, sat, receive_tow_s).ok()
}

fn select_navigation_with_rejection(
    navigation: &[PositionBroadcastNavigation],
    sat: SatId,
    receive_tow_s: f64,
) -> Result<&PositionBroadcastNavigation, MeasurementRejectReason> {
    let mut saw_constellation = false;
    let mut saw_satellite = false;
    let mut best_valid: Option<(&PositionBroadcastNavigation, (f64, f64))> = None;
    let mut best_rejected: Option<((f64, f64), MeasurementRejectReason)> = None;

    for entry in navigation {
        if entry.constellation() != sat.constellation {
            continue;
        }
        saw_constellation = true;
        if entry.sat() != sat {
            continue;
        }
        saw_satellite = true;

        let score =
            navigation_age_score(entry, receive_tow_s).unwrap_or((f64::INFINITY, f64::INFINITY));
        match navigation_rejection_reason(entry, receive_tow_s) {
            None => {
                if best_valid
                    .is_none_or(|(_, best_score)| navigation_score_is_better(score, best_score))
                {
                    best_valid = Some((entry, score));
                }
            }
            Some(reason) => {
                if best_rejected
                    .is_none_or(|(best_score, _)| navigation_score_is_better(score, best_score))
                {
                    best_rejected = Some((score, reason));
                }
            }
        }
    }

    if let Some((entry, _)) = best_valid {
        return Ok(entry);
    }
    if let Some((_, reason)) = best_rejected {
        return Err(reason);
    }
    if saw_satellite {
        Err(MeasurementRejectReason::InvalidEphemeris)
    } else if saw_constellation {
        Err(MeasurementRejectReason::EphemerisMismatch)
    } else {
        Err(MeasurementRejectReason::IncompleteEphemeris)
    }
}

fn navigation_age_score(
    navigation: &PositionBroadcastNavigation,
    receive_tow_s: f64,
) -> Option<(f64, f64)> {
    match navigation {
        PositionBroadcastNavigation::Gps(ephemeris) => {
            let age = gps_ephemeris_age(ephemeris, receive_tow_s);
            Some((age.toe_age_s.max(age.toc_age_s), age.toe_age_s + age.toc_age_s))
        }
        PositionBroadcastNavigation::Galileo(navigation) => {
            let age = galileo_navigation_age(navigation, receive_tow_s);
            Some((age.toe_age_s.max(age.toc_age_s), age.toe_age_s + age.toc_age_s))
        }
        PositionBroadcastNavigation::Beidou(navigation) => {
            let age = beidou_navigation_age(navigation, receive_tow_s);
            Some((age.toe_age_s.max(age.toc_age_s), age.toe_age_s + age.toc_age_s))
        }
        PositionBroadcastNavigation::Glonass(navigation) => {
            glonass_navigation_age(navigation, receive_tow_s).map(|age| (age.age_s, age.age_s))
        }
    }
}

fn navigation_score_is_better(candidate: (f64, f64), current: (f64, f64)) -> bool {
    candidate.0.total_cmp(&current.0).then_with(|| candidate.1.total_cmp(&current.1)).is_lt()
}

fn navigation_rejection_reason(
    navigation: &PositionBroadcastNavigation,
    receive_tow_s: f64,
) -> Option<MeasurementRejectReason> {
    match navigation {
        PositionBroadcastNavigation::Gps(ephemeris) => {
            gps_ephemeris_rejection_reason(ephemeris, receive_tow_s)
        }
        PositionBroadcastNavigation::Galileo(navigation) => {
            galileo_navigation_rejection_reason(navigation, receive_tow_s)
        }
        PositionBroadcastNavigation::Beidou(navigation) => {
            beidou_navigation_rejection_reason(navigation, receive_tow_s)
        }
        PositionBroadcastNavigation::Glonass(navigation) => {
            glonass_navigation_rejection_reason(navigation, receive_tow_s)
        }
    }
}

fn galileo_navigation_rejection_reason(
    navigation: &crate::orbits::galileo::GalileoBroadcastNavigationData,
    receive_tow_s: f64,
) -> Option<MeasurementRejectReason> {
    if navigation.sat.constellation != Constellation::Galileo
        || navigation.ephemeris.sat != navigation.sat
    {
        return Some(MeasurementRejectReason::EphemerisMismatch);
    }
    if !galileo_navigation_is_complete(navigation) {
        return Some(MeasurementRejectReason::IncompleteEphemeris);
    }
    if navigation.ephemeris.iodnav != navigation.iodnav {
        return Some(MeasurementRejectReason::EphemerisMismatch);
    }
    if !navigation.signal_health.e1b_data_valid || navigation.signal_health.e1b_signal_health != 0 {
        return Some(MeasurementRejectReason::UnhealthySatellite);
    }
    if navigation_time_delta_s(receive_tow_s, navigation.ephemeris.toe_s)
        < -NAVIGATION_FUTURE_TOLERANCE_S
        || navigation_time_delta_s(receive_tow_s, navigation.clock.t0c_s)
            < -NAVIGATION_FUTURE_TOLERANCE_S
    {
        return Some(MeasurementRejectReason::EphemerisFuture);
    }
    if galileo_navigation_age(navigation, receive_tow_s).is_stale() {
        return Some(MeasurementRejectReason::EphemerisStale);
    }
    None
}

fn galileo_navigation_is_complete(
    navigation: &crate::orbits::galileo::GalileoBroadcastNavigationData,
) -> bool {
    let ephemeris = &navigation.ephemeris;
    ephemeris.sqrt_a.is_finite()
        && ephemeris.sqrt_a > 0.0
        && ephemeris.e.is_finite()
        && (0.0..1.0).contains(&ephemeris.e)
        && ephemeris.toe_s.is_finite()
        && ephemeris.i0.is_finite()
        && ephemeris.idot.is_finite()
        && ephemeris.omega0.is_finite()
        && ephemeris.omegadot.is_finite()
        && ephemeris.w.is_finite()
        && ephemeris.m0.is_finite()
        && ephemeris.delta_n.is_finite()
        && ephemeris.cuc.is_finite()
        && ephemeris.cus.is_finite()
        && ephemeris.crc.is_finite()
        && ephemeris.crs.is_finite()
        && ephemeris.cic.is_finite()
        && ephemeris.cis.is_finite()
        && navigation.clock.t0c_s.is_finite()
        && navigation.clock.af0.is_finite()
        && navigation.clock.af1.is_finite()
        && navigation.clock.af2.is_finite()
        && navigation.clock.bgd_e1_e5a_s.is_finite()
        && navigation.clock.bgd_e1_e5b_s.is_finite()
}

fn beidou_navigation_rejection_reason(
    navigation: &crate::orbits::beidou::BeidouBroadcastNavigationData,
    receive_tow_s: f64,
) -> Option<MeasurementRejectReason> {
    if navigation.sat.constellation != Constellation::Beidou
        || navigation.ephemeris.sat != navigation.sat
    {
        return Some(MeasurementRejectReason::EphemerisMismatch);
    }
    if !beidou_navigation_is_complete(navigation) {
        return Some(MeasurementRejectReason::IncompleteEphemeris);
    }
    if navigation.ephemeris.aode != navigation.clock.aodc {
        return Some(MeasurementRejectReason::EphemerisMismatch);
    }
    if !navigation.signal_health.autonomous_satellite_good {
        return Some(MeasurementRejectReason::UnhealthySatellite);
    }
    if navigation_time_delta_s(receive_tow_s, navigation.ephemeris.toe_s)
        < -NAVIGATION_FUTURE_TOLERANCE_S
        || navigation_time_delta_s(receive_tow_s, navigation.clock.toc_s)
            < -NAVIGATION_FUTURE_TOLERANCE_S
    {
        return Some(MeasurementRejectReason::EphemerisFuture);
    }
    if beidou_navigation_age(navigation, receive_tow_s).is_stale() {
        return Some(MeasurementRejectReason::EphemerisStale);
    }
    None
}

fn beidou_navigation_is_complete(
    navigation: &crate::orbits::beidou::BeidouBroadcastNavigationData,
) -> bool {
    let ephemeris = &navigation.ephemeris;
    ephemeris.sqrt_a.is_finite()
        && ephemeris.sqrt_a > 0.0
        && ephemeris.e.is_finite()
        && (0.0..1.0).contains(&ephemeris.e)
        && ephemeris.toe_s.is_finite()
        && ephemeris.i0.is_finite()
        && ephemeris.idot.is_finite()
        && ephemeris.omega0.is_finite()
        && ephemeris.omegadot.is_finite()
        && ephemeris.w.is_finite()
        && ephemeris.m0.is_finite()
        && ephemeris.delta_n.is_finite()
        && ephemeris.cuc.is_finite()
        && ephemeris.cus.is_finite()
        && ephemeris.crc.is_finite()
        && ephemeris.crs.is_finite()
        && ephemeris.cic.is_finite()
        && ephemeris.cis.is_finite()
        && navigation.clock.toc_s.is_finite()
        && navigation.clock.af0.is_finite()
        && navigation.clock.af1.is_finite()
        && navigation.clock.af2.is_finite()
        && navigation.clock.tgd1_s.is_finite()
        && navigation.clock.tgd2_s.is_finite()
}

fn glonass_navigation_rejection_reason(
    navigation: &crate::orbits::glonass::GlonassBroadcastNavigationFrame,
    receive_tow_s: f64,
) -> Option<MeasurementRejectReason> {
    if navigation.sat.constellation != Constellation::Glonass
        || navigation.immediate.sat != navigation.sat
        || !glonass_slot_channel_association(navigation).is_slot_consistent
    {
        return Some(MeasurementRejectReason::EphemerisMismatch);
    }
    if !glonass_navigation_is_complete(navigation) {
        return Some(MeasurementRejectReason::IncompleteEphemeris);
    }
    if navigation.immediate.health.line_unhealthy || navigation.immediate.health.status_code != 0 {
        return Some(MeasurementRejectReason::UnhealthySatellite);
    }
    let Some(age) = glonass_navigation_age(navigation, receive_tow_s) else {
        return Some(MeasurementRejectReason::IncompleteEphemeris);
    };
    if age.delta_time_s < -NAVIGATION_FUTURE_TOLERANCE_S {
        return Some(MeasurementRejectReason::EphemerisFuture);
    }
    if age.is_stale() {
        return Some(MeasurementRejectReason::EphemerisStale);
    }
    None
}

fn glonass_navigation_is_complete(
    navigation: &crate::orbits::glonass::GlonassBroadcastNavigationFrame,
) -> bool {
    let state = navigation.immediate.state_vector;
    let Some(system_time) = navigation.system_time else {
        return false;
    };
    navigation.immediate.ephemeris_reference_time_s < 86_400
        && state.x_m.is_finite()
        && state.y_m.is_finite()
        && state.z_m.is_finite()
        && state.vx_mps.is_finite()
        && state.vy_mps.is_finite()
        && state.vz_mps.is_finite()
        && state.ax_mps2.is_finite()
        && state.ay_mps2.is_finite()
        && state.az_mps2.is_finite()
        && navigation.immediate.relative_frequency_bias.is_finite()
        && navigation.immediate.clock_bias_s.is_finite()
        && system_time.utc_offset_s.is_finite()
        && system_time.gps_minus_glonass_s.is_finite()
}

fn gps_ephemeris_rejection_reason(
    ephemeris: &crate::orbits::gps::GpsEphemeris,
    receive_tow_s: f64,
) -> Option<MeasurementRejectReason> {
    if ephemeris.sat.constellation != Constellation::Gps {
        return Some(MeasurementRejectReason::EphemerisMismatch);
    }
    if !gps_ephemeris_is_complete(ephemeris) {
        return Some(MeasurementRejectReason::IncompleteEphemeris);
    }
    if ephemeris.sv_health != 0 {
        return Some(MeasurementRejectReason::UnhealthySatellite);
    }
    if (ephemeris.iodc & 0xff) as u8 != ephemeris.iode {
        return Some(MeasurementRejectReason::EphemerisMismatch);
    }
    if navigation_time_delta_s(receive_tow_s, ephemeris.toe_s) < -NAVIGATION_FUTURE_TOLERANCE_S
        || navigation_time_delta_s(receive_tow_s, ephemeris.toc_s) < -NAVIGATION_FUTURE_TOLERANCE_S
    {
        return Some(MeasurementRejectReason::EphemerisFuture);
    }
    if gps_ephemeris_age(ephemeris, receive_tow_s).is_stale() {
        return Some(MeasurementRejectReason::EphemerisStale);
    }
    None
}

fn gps_ephemeris_is_complete(ephemeris: &crate::orbits::gps::GpsEphemeris) -> bool {
    ephemeris.sqrt_a.is_finite()
        && ephemeris.sqrt_a > 0.0
        && ephemeris.e.is_finite()
        && (0.0..1.0).contains(&ephemeris.e)
        && ephemeris.toe_s.is_finite()
        && ephemeris.toc_s.is_finite()
        && ephemeris.i0.is_finite()
        && ephemeris.idot.is_finite()
        && ephemeris.omega0.is_finite()
        && ephemeris.omegadot.is_finite()
        && ephemeris.w.is_finite()
        && ephemeris.m0.is_finite()
        && ephemeris.delta_n.is_finite()
        && ephemeris.cuc.is_finite()
        && ephemeris.cus.is_finite()
        && ephemeris.crc.is_finite()
        && ephemeris.crs.is_finite()
        && ephemeris.cic.is_finite()
        && ephemeris.cis.is_finite()
        && ephemeris.af0.is_finite()
        && ephemeris.af1.is_finite()
        && ephemeris.af2.is_finite()
        && ephemeris.tgd.is_finite()
}

fn navigation_time_delta_s(reference_time_s: f64, navigation_time_s: f64) -> f64 {
    let mut delta_s = reference_time_s - navigation_time_s;
    while delta_s > 302_400.0 {
        delta_s -= 604_800.0;
    }
    while delta_s < -302_400.0 {
        delta_s += 604_800.0;
    }
    delta_s
}

pub(crate) fn satellite_state_from_observation(
    navigation: &PositionBroadcastNavigation,
    receive_tow_s: f64,
    pseudorange_m: f64,
    signal_timing: Option<ObsSignalTiming>,
) -> Option<SatelliteState> {
    let signal_travel_time_s = signal_timing
        .map(|timing| timing.signal_travel_time_s.0)
        .unwrap_or(pseudorange_m / SPEED_OF_LIGHT_MPS);
    let transmit_tow_s = signal_timing
        .map(|timing| timing.transmit_gps_time.tow_s)
        .unwrap_or(receive_tow_s - signal_travel_time_s);
    satellite_state_at_time(navigation, transmit_tow_s, signal_travel_time_s)
}

pub fn position_satellite_state_from_observation(
    navigation: &PositionBroadcastNavigation,
    receive_tow_s: f64,
    pseudorange_m: f64,
    signal_timing: Option<ObsSignalTiming>,
) -> Option<PositionSatelliteState> {
    satellite_state_from_observation(navigation, receive_tow_s, pseudorange_m, signal_timing)
}

pub(crate) fn satellite_state_at_time(
    navigation: &PositionBroadcastNavigation,
    transmit_tow_s: f64,
    signal_travel_time_s: f64,
) -> Option<SatelliteState> {
    match navigation {
        PositionBroadcastNavigation::Gps(ephemeris) => {
            let state = sat_state_gps_l1ca(ephemeris, transmit_tow_s, signal_travel_time_s);
            Some(SatelliteState {
                x_m: state.x_m,
                y_m: state.y_m,
                z_m: state.z_m,
                vx_mps: state.vx_mps,
                vy_mps: state.vy_mps,
                vz_mps: state.vz_mps,
                clock_bias_s: state.clock_correction.bias_s,
                clock_drift_s_per_s: state.clock_correction.drift_s_per_s,
                uncertainty: state.uncertainty,
            })
        }
        PositionBroadcastNavigation::Galileo(navigation) => {
            let state = sat_state_galileo_e1(navigation, transmit_tow_s, signal_travel_time_s);
            Some(SatelliteState {
                x_m: state.x_m,
                y_m: state.y_m,
                z_m: state.z_m,
                vx_mps: state.vx_mps,
                vy_mps: state.vy_mps,
                vz_mps: state.vz_mps,
                clock_bias_s: state.clock_correction.bias_s,
                clock_drift_s_per_s: state.clock_correction.drift_s_per_s,
                uncertainty: state.uncertainty,
            })
        }
        PositionBroadcastNavigation::Beidou(navigation) => {
            let state = sat_state_beidou_b1i(navigation, transmit_tow_s, signal_travel_time_s);
            Some(SatelliteState {
                x_m: state.x_m,
                y_m: state.y_m,
                z_m: state.z_m,
                vx_mps: state.vx_mps,
                vy_mps: state.vy_mps,
                vz_mps: state.vz_mps,
                clock_bias_s: state.clock_correction.bias_s,
                clock_drift_s_per_s: state.clock_correction.drift_s_per_s,
                uncertainty: state.uncertainty,
            })
        }
        PositionBroadcastNavigation::Glonass(navigation) => {
            let state = sat_state_glonass_l1(navigation, transmit_tow_s, signal_travel_time_s)?;
            Some(SatelliteState {
                x_m: state.x_m,
                y_m: state.y_m,
                z_m: state.z_m,
                vx_mps: state.vx_mps,
                vy_mps: state.vy_mps,
                vz_mps: state.vz_mps,
                clock_bias_s: state.clock_correction.bias_s,
                clock_drift_s_per_s: state.clock_correction.drift_s_per_s,
                uncertainty: state.uncertainty,
            })
        }
    }
}

pub(crate) fn default_position_signal_id(sat: SatId) -> Option<SigId> {
    let signal = default_acquisition_signal(sat.constellation)?;
    Some(SigId { sat, band: signal.spec.band, code: signal.spec.code })
}

pub(crate) fn position_signal_id(observation: &PositionObservation) -> Option<SigId> {
    observation.signal_id.or_else(|| default_position_signal_id(observation.sat))
}

fn broadcast_group_delay_code_bias_m(
    signal_id: Option<SigId>,
    navigation: &PositionBroadcastNavigation,
) -> f64 {
    let Some(signal_id) = signal_id else {
        return 0.0;
    };
    match navigation {
        PositionBroadcastNavigation::Gps(ephemeris) => {
            gps_broadcast_group_delay_code_bias_m(signal_id, ephemeris).unwrap_or(0.0)
        }
        PositionBroadcastNavigation::Galileo(navigation) => {
            galileo_broadcast_group_delay_code_bias_m(signal_id, navigation).unwrap_or(0.0)
        }
        PositionBroadcastNavigation::Beidou(navigation) => {
            beidou_broadcast_group_delay_code_bias_m(signal_id, navigation).unwrap_or(0.0)
        }
        PositionBroadcastNavigation::Glonass(_) => 0.0,
    }
}

pub(crate) fn corrected_pseudorange_m(
    observation: &PositionObservation,
    navigation: &PositionBroadcastNavigation,
    apply_broadcast_group_delay: bool,
) -> f64 {
    broadcast_group_delay_correction_chain(observation, navigation, apply_broadcast_group_delay)
        .corrected_pseudorange_m
}

pub(crate) fn broadcast_group_delay_correction_chain(
    observation: &PositionObservation,
    navigation: &PositionBroadcastNavigation,
    apply_broadcast_group_delay: bool,
) -> PositionObservationCorrectionChain {
    let mut chain = PositionObservationCorrectionChain::new(observation.pseudorange_m);
    if apply_broadcast_group_delay {
        chain.push_component(
            PositionObservationCorrectionKind::BroadcastGroupDelay,
            -broadcast_group_delay_code_bias_m(observation.signal_id, navigation),
        );
    }
    chain
}

pub(crate) fn observation_consistency_metrics(
    observation: &PositionObservation,
    navigation: &PositionBroadcastNavigation,
    apply_broadcast_group_delay: bool,
    receiver_position_ecef_m: [f64; 3],
    receiver_clock_bias_s: f64,
    receiver_velocity_ecef_mps: Option<[f64; 3]>,
    receiver_clock_drift_s_per_s: Option<f64>,
) -> Option<ObservationConsistencyMetrics> {
    let corrected_pseudorange_m =
        corrected_pseudorange_m(observation, navigation, apply_broadcast_group_delay);
    let receive_tow_s =
        observation.gps_receive_time.map(|gps_time| gps_time.tow_s).or_else(|| {
            observation
                .signal_timing
                .map(|timing| timing.transmit_gps_time.tow_s + timing.signal_travel_time_s.0)
        })?;
    let state = satellite_state_from_observation(
        navigation,
        receive_tow_s,
        corrected_pseudorange_m,
        observation.signal_timing,
    )?;
    let dx = receiver_position_ecef_m[0] - state.x_m;
    let dy = receiver_position_ecef_m[1] - state.y_m;
    let dz = receiver_position_ecef_m[2] - state.z_m;
    let range_m = (dx * dx + dy * dy + dz * dz).sqrt().max(1.0);
    let code_prediction_m =
        range_m + SPEED_OF_LIGHT_MPS * (receiver_clock_bias_s - state.clock_bias_s);
    let code_residual_m = corrected_pseudorange_m - code_prediction_m;

    let doppler_residual_hz = observation
        .doppler_hz
        .zip(receiver_velocity_ecef_mps)
        .zip(receiver_clock_drift_s_per_s)
        .and_then(|((observed_doppler_hz, receiver_velocity_ecef_mps), receiver_clock_drift_s)| {
            let wavelength_m =
                signal_id_wavelength_m(position_signal_id(observation)?).map(|v| v.0)?;
            let los = [dx / range_m, dy / range_m, dz / range_m];
            let relative_velocity_mps = [
                receiver_velocity_ecef_mps[0] - state.vx_mps,
                receiver_velocity_ecef_mps[1] - state.vy_mps,
                receiver_velocity_ecef_mps[2] - state.vz_mps,
            ];
            let range_rate_mps = los[0] * relative_velocity_mps[0]
                + los[1] * relative_velocity_mps[1]
                + los[2] * relative_velocity_mps[2];
            let predicted_doppler_hz = -range_rate_mps / wavelength_m
                + SPEED_OF_LIGHT_MPS * (receiver_clock_drift_s - state.clock_drift_s_per_s)
                    / wavelength_m;
            Some(observed_doppler_hz - predicted_doppler_hz)
        });

    Some(ObservationConsistencyMetrics { code_residual_m, doppler_residual_hz })
}

#[cfg(test)]
mod tests {
    use super::{
        broadcast_group_delay_correction_chain, corrected_pseudorange_m,
        default_position_signal_id, observation_consistency_metrics, position_signal_id,
        resolve_position_inputs, satellite_state_at_time, select_valid_navigation,
        PositionBroadcastNavigation, PositionObservation, PositionObservationCorrectionChain,
        PositionObservationCorrectionKind, POSITION_OBSERVATION_CORRECTION_ORDER,
        SPEED_OF_LIGHT_MPS,
    };
    use crate::corrections::broadcast_group_delay::gps_broadcast_group_delay_code_bias_m;
    use crate::estimation::position::solver::geodetic_to_ecef;
    use crate::orbits::beidou::{
        BeidouBroadcastNavigationData, BeidouClockCorrection, BeidouEphemeris,
        BeidouIonosphericCorrection, BeidouSignalHealth, BeidouSystemTime,
    };
    use crate::orbits::galileo::{
        GalileoBroadcastNavigationData, GalileoClockCorrection, GalileoEphemeris,
        GalileoIonosphericCorrection, GalileoIonosphericDisturbanceFlags, GalileoSignalHealth,
        GalileoSystemTime,
    };
    use crate::orbits::glonass::{
        GlonassAlmanacTimeData, GlonassBroadcastNavigationFrame, GlonassFrameTime,
        GlonassImmediateHealth, GlonassImmediateNavigationData, GlonassSatelliteType,
        GlonassStateVector, GlonassSystemTime,
    };
    use crate::orbits::gps::{sat_state_gps_l1ca, GpsEphemeris};
    use crate::orbits::satellite_uncertainty::{
        SatelliteHealthSource, SatelliteHealthStatus, SatelliteOrbitUncertaintySource,
    };
    use bijux_gnss_core::api::{
        Constellation, GpsTime, MeasurementRejectReason, ObsSignalTiming, SatId, Seconds, SigId,
        SignalBand, SignalCode,
    };
    use bijux_gnss_signal::api::signal_id_wavelength_m;

    fn sample_gps_ephemeris() -> GpsEphemeris {
        GpsEphemeris {
            sat: SatId { constellation: Constellation::Gps, prn: 7 },
            iodc: 1,
            iode: 1,
            week: 2209,
            sv_health: 0,
            sv_accuracy: Some(2),
            toe_s: 504_000.0,
            toc_s: 504_018.0,
            sqrt_a: 5153.7954775,
            e: 0.01,
            i0: 0.94,
            idot: 0.0,
            omega0: 0.8,
            omegadot: 0.0,
            w: 0.0,
            m0: 0.9,
            delta_n: 0.0,
            cuc: 0.0,
            cus: 0.0,
            crc: 0.0,
            crs: 0.0,
            cic: 0.0,
            cis: 0.0,
            af0: 1.0e-4,
            af1: 2.0e-11,
            af2: 0.0,
            tgd: 0.0,
        }
    }

    fn sample_galileo_navigation() -> GalileoBroadcastNavigationData {
        GalileoBroadcastNavigationData {
            sat: SatId { constellation: Constellation::Galileo, prn: 19 },
            iodnav: 0x1A5,
            gst: GalileoSystemTime { week: 2222, tow_s: 64_800 },
            sisa_e1_e5b: 77,
            signal_health: GalileoSignalHealth {
                e5b_signal_health: 0,
                e1b_signal_health: 0,
                e5b_data_valid: true,
                e1b_data_valid: true,
            },
            clock: GalileoClockCorrection {
                t0c_s: 64_800.0,
                af0: -1.7e-4,
                af1: 2.5e-12,
                af2: -3.0e-19,
                bgd_e1_e5a_s: -1.1e-9,
                bgd_e1_e5b_s: 2.4e-9,
            },
            ephemeris: GalileoEphemeris {
                sat: SatId { constellation: Constellation::Galileo, prn: 19 },
                iodnav: 0x1A5,
                toe_s: 64_800.0,
                sqrt_a: 5_440.612_319,
                e: 0.001_23,
                i0: 0.953,
                idot: -2.1e-10,
                omega0: 1.17,
                omegadot: -5.8e-9,
                w: -0.37,
                m0: 0.84,
                delta_n: 4.7e-9,
                cuc: -3.2e-6,
                cus: 4.1e-6,
                crc: 178.0,
                crs: -91.0,
                cic: 1.9e-7,
                cis: -2.4e-7,
            },
            ionosphere: GalileoIonosphericCorrection {
                ai0: 121.129_893,
                ai1: 0.351_254_133,
                ai2: 0.013_463_534_8,
                disturbance_flags: GalileoIonosphericDisturbanceFlags {
                    region_1: false,
                    region_2: false,
                    region_3: false,
                    region_4: false,
                    region_5: false,
                },
            },
        }
    }

    fn sample_beidou_navigation() -> BeidouBroadcastNavigationData {
        BeidouBroadcastNavigationData {
            sat: SatId { constellation: Constellation::Beidou, prn: 11 },
            bdt: BeidouSystemTime { week: 987, sow_s: 64_800 },
            urai: 2,
            signal_health: BeidouSignalHealth { autonomous_satellite_good: true },
            clock: BeidouClockCorrection {
                toc_s: 64_800.0,
                aodc: 3,
                af0: -1.8e-4,
                af1: 2.2e-12,
                af2: -2.6e-19,
                tgd1_s: -1.1e-9,
                tgd2_s: 2.2e-9,
            },
            ephemeris: BeidouEphemeris {
                sat: SatId { constellation: Constellation::Beidou, prn: 11 },
                aode: 3,
                toe_s: 64_800.0,
                sqrt_a: 5_282.625_128,
                e: 0.002_34,
                i0: 0.958,
                idot: -1.9e-10,
                omega0: 0.87,
                omegadot: -6.2e-9,
                w: -0.42,
                m0: 1.12,
                delta_n: 4.3e-9,
                cuc: -2.3e-6,
                cus: 3.1e-6,
                crc: 145.0,
                crs: -82.0,
                cic: 2.6e-7,
                cis: -2.2e-7,
            },
            ionosphere: BeidouIonosphericCorrection {
                alpha0: 1.0e-8,
                alpha1: -2.0e-8,
                alpha2: 3.0e-8,
                alpha3: -4.0e-8,
                beta0: 1.2e5,
                beta1: -2.4e5,
                beta2: 3.6e5,
                beta3: -4.8e5,
            },
        }
    }

    fn sample_glonass_navigation() -> GlonassBroadcastNavigationFrame {
        let sat = SatId { constellation: Constellation::Glonass, prn: 14 };
        GlonassBroadcastNavigationFrame {
            sat,
            immediate: GlonassImmediateNavigationData {
                sat,
                frame_time: GlonassFrameTime { hour: 23, minute: 18, half_minute: false },
                ephemeris_reference_time_s: 83_700,
                tb_update_interval_min: 30,
                tb_is_odd: Some(true),
                state_vector: GlonassStateVector {
                    x_m: -7_557_760.253_906_25,
                    y_m: -23_962_225.585_937_5,
                    z_m: -4_337_567.871_093_75,
                    vx_mps: 101.318_359_375,
                    vy_mps: 602.112_770_080_566_4,
                    vz_mps: -3_495.733_261_108_398_4,
                    ax_mps2: -3.725_290_298_461_914e-6,
                    ay_mps2: 0.0,
                    az_mps2: 1.862_645_149_230_957e-6,
                },
                relative_frequency_bias: 0.0,
                clock_bias_s: -2.572_406_083_345_413_2e-5,
                l2_l1_delay_s: Some(5.587_935_448e-9),
                health: GlonassImmediateHealth { line_unhealthy: false, status_code: 0 },
                immediate_data_age_days: 28,
                satellite_type: GlonassSatelliteType::GlonassM,
                reported_slot: None,
                system_time: Some(GlonassSystemTime {
                    day_number: 864,
                    four_year_interval: Some(8),
                }),
                resolved_day_index: None,
                accuracy_code: Some(2),
            },
            system_time: Some(GlonassAlmanacTimeData {
                system_time: GlonassSystemTime { day_number: 864, four_year_interval: Some(8) },
                utc_offset_s: 0.0,
                gps_minus_glonass_s: -10_782.0,
            }),
            almanac_entries: Vec::new(),
        }
    }

    #[test]
    fn observation_correction_order_names_each_solver_component_once() {
        assert_eq!(
            POSITION_OBSERVATION_CORRECTION_ORDER,
            [
                PositionObservationCorrectionKind::SatelliteClock,
                PositionObservationCorrectionKind::Relativity,
                PositionObservationCorrectionKind::EarthRotation,
                PositionObservationCorrectionKind::BroadcastGroupDelay,
                PositionObservationCorrectionKind::Ionosphere,
                PositionObservationCorrectionKind::Troposphere,
                PositionObservationCorrectionKind::ReceiverAntenna,
                PositionObservationCorrectionKind::SatelliteAntenna,
                PositionObservationCorrectionKind::EarthTide,
                PositionObservationCorrectionKind::PhaseWindup,
                PositionObservationCorrectionKind::ReceiverClock,
            ]
        );
    }

    #[test]
    fn observation_correction_chain_reconstructs_signed_components() {
        let mut chain = PositionObservationCorrectionChain::new(24_000_000.0);

        chain.push_component(PositionObservationCorrectionKind::SatelliteClock, 12_500.0);
        chain.push_component(PositionObservationCorrectionKind::BroadcastGroupDelay, -2.4);
        chain.push_component(PositionObservationCorrectionKind::Ionosphere, -4.8);
        chain.push_component(PositionObservationCorrectionKind::Troposphere, -2.1);
        chain.push_component(PositionObservationCorrectionKind::ReceiverClock, -9_000.0);

        assert_eq!(
            chain.component_delta_m(PositionObservationCorrectionKind::BroadcastGroupDelay),
            -2.4
        );
        assert_eq!(chain.reconstruction_error_m(), 0.0);
        assert_eq!(
            chain.corrected_pseudorange_m,
            24_000_000.0 + 12_500.0 - 2.4 - 4.8 - 2.1 - 9_000.0
        );
    }

    #[test]
    fn broadcast_group_delay_correction_chain_records_applied_bias() {
        let mut ephemeris = sample_gps_ephemeris();
        ephemeris.tgd = 2.25e-9;
        let navigation = PositionBroadcastNavigation::Gps(ephemeris.clone());
        let sat = ephemeris.sat;
        let signal_id = SigId { sat, band: SignalBand::L1, code: SignalCode::Ca };
        let observation = PositionObservation {
            sat,
            pseudorange_m: 23_500_000.0,
            doppler_hz: None,
            doppler_var_hz2: None,
            cn0_dbhz: 45.0,
            elevation_deg: None,
            weight: 1.0,
            gps_receive_time: None,
            signal_timing: None,
            signal_id: Some(signal_id),
        };
        let expected_bias_m = gps_broadcast_group_delay_code_bias_m(signal_id, &ephemeris)
            .expect("gps group delay bias");

        let chain = broadcast_group_delay_correction_chain(&observation, &navigation, true);
        let disabled_chain =
            broadcast_group_delay_correction_chain(&observation, &navigation, false);

        assert_eq!(chain.components.len(), 1);
        assert_eq!(
            chain.component_delta_m(PositionObservationCorrectionKind::BroadcastGroupDelay),
            -expected_bias_m
        );
        assert_eq!(chain.reconstruction_error_m(), 0.0);
        assert_eq!(
            corrected_pseudorange_m(&observation, &navigation, true),
            chain.corrected_pseudorange_m
        );
        assert_eq!(disabled_chain.components.len(), 0);
        assert_eq!(disabled_chain.corrected_pseudorange_m, observation.pseudorange_m);
    }

    #[test]
    fn default_position_signal_id_uses_registered_constellation_defaults() {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };

        assert_eq!(
            default_position_signal_id(sat),
            Some(SigId { sat, band: SignalBand::L1, code: SignalCode::Ca })
        );
    }

    #[test]
    fn satellite_state_at_time_reports_finite_rate_terms() {
        let ephemeris = sample_gps_ephemeris();
        let navigation = PositionBroadcastNavigation::Gps(ephemeris.clone());
        let transmit_tow_s = 504_018.0;
        let signal_travel_time_s = 0.078;

        let state = satellite_state_at_time(&navigation, transmit_tow_s, signal_travel_time_s)
            .expect("satellite state");
        let direct_state = sat_state_gps_l1ca(&ephemeris, transmit_tow_s, signal_travel_time_s);
        let speed_mps = (state.vx_mps * state.vx_mps
            + state.vy_mps * state.vy_mps
            + state.vz_mps * state.vz_mps)
            .sqrt();

        assert!((state.x_m - direct_state.x_m).abs() < 1.0e-9);
        assert!((state.y_m - direct_state.y_m).abs() < 1.0e-9);
        assert!((state.z_m - direct_state.z_m).abs() < 1.0e-9);
        assert!((state.vx_mps - direct_state.vx_mps).abs() < 1.0e-12);
        assert!((state.vy_mps - direct_state.vy_mps).abs() < 1.0e-12);
        assert!((state.vz_mps - direct_state.vz_mps).abs() < 1.0e-12);
        assert!((state.clock_bias_s - direct_state.clock_correction.bias_s).abs() < 1.0e-18);
        assert!(
            (state.clock_drift_s_per_s - direct_state.clock_correction.drift_s_per_s).abs()
                < 1.0e-18
        );
        assert_eq!(state.uncertainty, direct_state.uncertainty);
        assert_eq!(state.uncertainty.orbit_source, SatelliteOrbitUncertaintySource::GpsUra);
        assert_eq!(state.uncertainty.orbit_sigma_m, Some(4.85));
        assert_eq!(state.uncertainty.health_status, SatelliteHealthStatus::Healthy);
        assert_eq!(state.uncertainty.health_source, SatelliteHealthSource::GpsSvHealth);
        assert!(speed_mps > 100.0, "speed_mps={speed_mps}");
    }

    #[test]
    fn satellite_state_at_time_preserves_constellation_uncertainty_provenance() {
        let galileo = sample_galileo_navigation();
        let beidou = sample_beidou_navigation();
        let glonass = sample_glonass_navigation();

        let galileo_state = satellite_state_at_time(
            &PositionBroadcastNavigation::Galileo(galileo.clone()),
            galileo.ephemeris.toe_s,
            0.078,
        )
        .expect("galileo satellite state");
        let beidou_state = satellite_state_at_time(
            &PositionBroadcastNavigation::Beidou(beidou.clone()),
            beidou.ephemeris.toe_s,
            0.078,
        )
        .expect("beidou satellite state");
        let glonass_state = satellite_state_at_time(
            &PositionBroadcastNavigation::Glonass(glonass.clone()),
            504_918.0,
            0.078,
        )
        .expect("glonass satellite state");

        assert_eq!(
            galileo_state.uncertainty.orbit_source,
            SatelliteOrbitUncertaintySource::GalileoSisa
        );
        assert_eq!(galileo_state.uncertainty.orbit_sigma_m, Some(1.08));
        assert_eq!(
            beidou_state.uncertainty.orbit_source,
            SatelliteOrbitUncertaintySource::BeidouUrai
        );
        assert_eq!(beidou_state.uncertainty.orbit_sigma_m, Some(4.85));
        assert_eq!(
            glonass_state.uncertainty.orbit_source,
            SatelliteOrbitUncertaintySource::GlonassAccuracyCode
        );
        assert_eq!(glonass_state.uncertainty.orbit_sigma_m, Some(2.5));
    }

    #[test]
    fn gps_selection_prefers_fresh_healthy_matching_issue_data() {
        let mut stale_ephemeris = sample_gps_ephemeris();
        stale_ephemeris.toe_s -= 5_000.0;
        stale_ephemeris.toc_s -= 5_000.0;
        let fresh_ephemeris = sample_gps_ephemeris();
        let navigation = [
            PositionBroadcastNavigation::Gps(stale_ephemeris),
            PositionBroadcastNavigation::Gps(fresh_ephemeris.clone()),
        ];

        let selected =
            select_valid_navigation(&navigation, fresh_ephemeris.sat, 504_018.0).expect("selected");

        assert_eq!(selected.sat(), fresh_ephemeris.sat);
        match selected {
            PositionBroadcastNavigation::Gps(ephemeris) => {
                assert_eq!(ephemeris.toe_s, fresh_ephemeris.toe_s);
                assert_eq!(ephemeris.toc_s, fresh_ephemeris.toc_s);
            }
            _ => panic!("selected non-GPS navigation"),
        }
    }

    #[test]
    fn gps_selection_reports_stale_ephemeris() {
        let mut ephemeris = sample_gps_ephemeris();
        ephemeris.toe_s -= 8_000.0;
        ephemeris.toc_s -= 8_000.0;

        let rejected = rejected_navigation_reason(ephemeris, 504_018.0);

        assert_eq!(rejected, MeasurementRejectReason::EphemerisStale);
    }

    #[test]
    fn gps_selection_reports_future_ephemeris() {
        let mut ephemeris = sample_gps_ephemeris();
        ephemeris.toe_s += 30.0;

        let rejected = rejected_navigation_reason(ephemeris, 504_018.0);

        assert_eq!(rejected, MeasurementRejectReason::EphemerisFuture);
    }

    #[test]
    fn gps_selection_reports_unhealthy_satellite() {
        let mut ephemeris = sample_gps_ephemeris();
        ephemeris.sv_health = 1;

        let rejected = rejected_navigation_reason(ephemeris, 504_018.0);

        assert_eq!(rejected, MeasurementRejectReason::UnhealthySatellite);
    }

    #[test]
    fn gps_selection_reports_issue_mismatch() {
        let mut ephemeris = sample_gps_ephemeris();
        ephemeris.iodc = 0x102;
        ephemeris.iode = 0x03;

        let rejected = rejected_navigation_reason(ephemeris, 504_018.0);

        assert_eq!(rejected, MeasurementRejectReason::EphemerisMismatch);
    }

    #[test]
    fn gps_selection_reports_incomplete_ephemeris() {
        let mut ephemeris = sample_gps_ephemeris();
        ephemeris.sqrt_a = 0.0;

        let rejected = rejected_navigation_reason(ephemeris, 504_018.0);

        assert_eq!(rejected, MeasurementRejectReason::IncompleteEphemeris);
    }

    #[test]
    fn navigation_selection_reports_satellite_mismatch() {
        let observation_sat = SatId { constellation: Constellation::Gps, prn: 8 };
        let ephemeris = sample_gps_ephemeris();

        let rejected = rejected_navigation_reason_for_sat(ephemeris, observation_sat, 504_018.0);

        assert_eq!(rejected, MeasurementRejectReason::EphemerisMismatch);
    }

    #[test]
    fn galileo_selection_reports_stale_navigation() {
        let mut navigation = sample_galileo_navigation();
        navigation.ephemeris.toe_s -= 18_000.0;
        navigation.clock.t0c_s -= 18_000.0;

        let rejected = rejected_navigation_entry_reason(
            PositionBroadcastNavigation::Galileo(navigation),
            sample_galileo_navigation().sat,
            65_000.0,
        );

        assert_eq!(rejected, MeasurementRejectReason::EphemerisStale);
    }

    #[test]
    fn galileo_selection_reports_future_navigation() {
        let mut navigation = sample_galileo_navigation();
        navigation.ephemeris.toe_s = 65_030.0;

        let rejected = rejected_navigation_entry_reason(
            PositionBroadcastNavigation::Galileo(navigation),
            sample_galileo_navigation().sat,
            65_000.0,
        );

        assert_eq!(rejected, MeasurementRejectReason::EphemerisFuture);
    }

    #[test]
    fn galileo_selection_reports_unhealthy_e1_signal() {
        let mut navigation = sample_galileo_navigation();
        navigation.signal_health.e1b_signal_health = 2;

        let rejected = rejected_navigation_entry_reason(
            PositionBroadcastNavigation::Galileo(navigation),
            sample_galileo_navigation().sat,
            65_000.0,
        );

        assert_eq!(rejected, MeasurementRejectReason::UnhealthySatellite);
    }

    #[test]
    fn galileo_selection_reports_iodnav_mismatch() {
        let mut navigation = sample_galileo_navigation();
        navigation.ephemeris.iodnav ^= 0x1;

        let rejected = rejected_navigation_entry_reason(
            PositionBroadcastNavigation::Galileo(navigation),
            sample_galileo_navigation().sat,
            65_000.0,
        );

        assert_eq!(rejected, MeasurementRejectReason::EphemerisMismatch);
    }

    #[test]
    fn galileo_selection_reports_incomplete_navigation() {
        let mut navigation = sample_galileo_navigation();
        navigation.ephemeris.sqrt_a = f64::NAN;

        let rejected = rejected_navigation_entry_reason(
            PositionBroadcastNavigation::Galileo(navigation),
            sample_galileo_navigation().sat,
            65_000.0,
        );

        assert_eq!(rejected, MeasurementRejectReason::IncompleteEphemeris);
    }

    #[test]
    fn beidou_selection_reports_stale_navigation() {
        let mut navigation = sample_beidou_navigation();
        navigation.ephemeris.toe_s -= 18_000.0;
        navigation.clock.toc_s -= 18_000.0;

        let rejected = rejected_navigation_entry_reason(
            PositionBroadcastNavigation::Beidou(navigation),
            sample_beidou_navigation().sat,
            65_000.0,
        );

        assert_eq!(rejected, MeasurementRejectReason::EphemerisStale);
    }

    #[test]
    fn beidou_selection_reports_future_navigation() {
        let mut navigation = sample_beidou_navigation();
        navigation.clock.toc_s = 65_030.0;

        let rejected = rejected_navigation_entry_reason(
            PositionBroadcastNavigation::Beidou(navigation),
            sample_beidou_navigation().sat,
            65_000.0,
        );

        assert_eq!(rejected, MeasurementRejectReason::EphemerisFuture);
    }

    #[test]
    fn beidou_selection_reports_unhealthy_signal() {
        let mut navigation = sample_beidou_navigation();
        navigation.signal_health.autonomous_satellite_good = false;

        let rejected = rejected_navigation_entry_reason(
            PositionBroadcastNavigation::Beidou(navigation),
            sample_beidou_navigation().sat,
            65_000.0,
        );

        assert_eq!(rejected, MeasurementRejectReason::UnhealthySatellite);
    }

    #[test]
    fn beidou_selection_reports_age_of_data_mismatch() {
        let mut navigation = sample_beidou_navigation();
        navigation.ephemeris.aode ^= 0x1;

        let rejected = rejected_navigation_entry_reason(
            PositionBroadcastNavigation::Beidou(navigation),
            sample_beidou_navigation().sat,
            65_000.0,
        );

        assert_eq!(rejected, MeasurementRejectReason::EphemerisMismatch);
    }

    #[test]
    fn beidou_selection_reports_incomplete_navigation() {
        let mut navigation = sample_beidou_navigation();
        navigation.clock.tgd1_s = f64::NAN;

        let rejected = rejected_navigation_entry_reason(
            PositionBroadcastNavigation::Beidou(navigation),
            sample_beidou_navigation().sat,
            65_000.0,
        );

        assert_eq!(rejected, MeasurementRejectReason::IncompleteEphemeris);
    }

    #[test]
    fn glonass_selection_reports_stale_navigation() {
        let navigation = sample_glonass_navigation();

        let rejected = rejected_navigation_entry_reason(
            PositionBroadcastNavigation::Glonass(navigation),
            sample_glonass_navigation().sat,
            506_000.0,
        );

        assert_eq!(rejected, MeasurementRejectReason::EphemerisStale);
    }

    #[test]
    fn glonass_selection_reports_future_navigation() {
        let navigation = sample_glonass_navigation();

        let rejected = rejected_navigation_entry_reason(
            PositionBroadcastNavigation::Glonass(navigation),
            sample_glonass_navigation().sat,
            504_900.0,
        );

        assert_eq!(rejected, MeasurementRejectReason::EphemerisFuture);
    }

    #[test]
    fn glonass_selection_reports_unhealthy_satellite() {
        let mut navigation = sample_glonass_navigation();
        navigation.immediate.health.line_unhealthy = true;

        let rejected = rejected_navigation_entry_reason(
            PositionBroadcastNavigation::Glonass(navigation),
            sample_glonass_navigation().sat,
            504_918.0,
        );

        assert_eq!(rejected, MeasurementRejectReason::UnhealthySatellite);
    }

    #[test]
    fn glonass_selection_reports_missing_time_relation() {
        let mut navigation = sample_glonass_navigation();
        navigation.system_time = None;

        let rejected = rejected_navigation_entry_reason(
            PositionBroadcastNavigation::Glonass(navigation),
            sample_glonass_navigation().sat,
            504_918.0,
        );

        assert_eq!(rejected, MeasurementRejectReason::IncompleteEphemeris);
    }

    #[test]
    fn glonass_selection_reports_immediate_satellite_mismatch() {
        let mut navigation = sample_glonass_navigation();
        navigation.immediate.sat = SatId { constellation: Constellation::Glonass, prn: 13 };

        let rejected = rejected_navigation_entry_reason(
            PositionBroadcastNavigation::Glonass(navigation),
            sample_glonass_navigation().sat,
            504_918.0,
        );

        assert_eq!(rejected, MeasurementRejectReason::EphemerisMismatch);
    }

    #[test]
    fn observation_consistency_metrics_match_clean_gps_observation() {
        let ephemeris = sample_gps_ephemeris();
        let navigation = PositionBroadcastNavigation::Gps(ephemeris.clone());
        let receiver_ecef_m = geodetic_to_ecef(37.0, -122.0, 10.0);
        let receiver_clock_bias_s = 2.75e-4;
        let receiver_velocity_ecef_mps = [15.0, -4.0, 2.0];
        let receiver_clock_drift_s_per_s = 5.0e-8;
        let receive_tow_s = 504_018.07;

        let mut signal_travel_time_s = 0.07;
        let pseudorange_m = loop {
            let state = satellite_state_at_time(
                &navigation,
                receive_tow_s - signal_travel_time_s,
                signal_travel_time_s,
            )
            .expect("satellite state");
            let dx = receiver_ecef_m.0 - state.x_m;
            let dy = receiver_ecef_m.1 - state.y_m;
            let dz = receiver_ecef_m.2 - state.z_m;
            let range_m = (dx * dx + dy * dy + dz * dz).sqrt();
            let pseudorange_m =
                range_m + SPEED_OF_LIGHT_MPS * (receiver_clock_bias_s - state.clock_bias_s);
            let next_signal_travel_time_s = pseudorange_m / SPEED_OF_LIGHT_MPS;
            if (next_signal_travel_time_s - signal_travel_time_s).abs() < 1.0e-12 {
                break pseudorange_m;
            }
            signal_travel_time_s = next_signal_travel_time_s;
        };
        let state = satellite_state_at_time(
            &navigation,
            receive_tow_s - signal_travel_time_s,
            signal_travel_time_s,
        )
        .expect("satellite state");
        let dx = receiver_ecef_m.0 - state.x_m;
        let dy = receiver_ecef_m.1 - state.y_m;
        let dz = receiver_ecef_m.2 - state.z_m;
        let range_m = (dx * dx + dy * dy + dz * dz).sqrt();
        let los = [dx / range_m, dy / range_m, dz / range_m];
        let relative_velocity_mps = [
            receiver_velocity_ecef_mps[0] - state.vx_mps,
            receiver_velocity_ecef_mps[1] - state.vy_mps,
            receiver_velocity_ecef_mps[2] - state.vz_mps,
        ];
        let range_rate_mps = los[0] * relative_velocity_mps[0]
            + los[1] * relative_velocity_mps[1]
            + los[2] * relative_velocity_mps[2];
        let signal_id = position_signal_id(&PositionObservation {
            sat: ephemeris.sat,
            pseudorange_m,
            doppler_hz: None,
            doppler_var_hz2: None,
            cn0_dbhz: 45.0,
            elevation_deg: None,
            weight: 1.0,
            gps_receive_time: None,
            signal_timing: None,
            signal_id: default_position_signal_id(ephemeris.sat),
        })
        .expect("signal id");
        let wavelength_m = signal_id_wavelength_m(signal_id).expect("GPS L1 wavelength").0;
        let doppler_hz = -range_rate_mps / wavelength_m
            + SPEED_OF_LIGHT_MPS * (receiver_clock_drift_s_per_s - state.clock_drift_s_per_s)
                / wavelength_m;
        let observation = PositionObservation {
            sat: ephemeris.sat,
            pseudorange_m,
            doppler_hz: Some(doppler_hz),
            doppler_var_hz2: Some(1.0),
            cn0_dbhz: 45.0,
            elevation_deg: None,
            weight: 1.0,
            gps_receive_time: Some(GpsTime { week: ephemeris.week, tow_s: receive_tow_s }),
            signal_timing: Some(ObsSignalTiming {
                signal_travel_time_s: Seconds(signal_travel_time_s),
                transmit_gps_time: GpsTime {
                    week: ephemeris.week,
                    tow_s: receive_tow_s - signal_travel_time_s,
                },
            }),
            signal_id: Some(signal_id),
        };

        let metrics = observation_consistency_metrics(
            &observation,
            &navigation,
            false,
            [receiver_ecef_m.0, receiver_ecef_m.1, receiver_ecef_m.2],
            receiver_clock_bias_s,
            Some(receiver_velocity_ecef_mps),
            Some(receiver_clock_drift_s_per_s),
        )
        .expect("consistency metrics");

        assert!(metrics.code_residual_m.abs() < 1.0e-6, "{metrics:?}");
        assert!(
            metrics
                .doppler_residual_hz
                .is_some_and(|doppler_residual_hz| doppler_residual_hz.abs() < 1.0e-6),
            "{metrics:?}"
        );
    }

    fn rejected_navigation_reason(
        ephemeris: GpsEphemeris,
        receive_tow_s: f64,
    ) -> MeasurementRejectReason {
        rejected_navigation_reason_for_sat(ephemeris.clone(), ephemeris.sat, receive_tow_s)
    }

    fn rejected_navigation_reason_for_sat(
        ephemeris: GpsEphemeris,
        sat: SatId,
        receive_tow_s: f64,
    ) -> MeasurementRejectReason {
        rejected_navigation_entry_reason(
            PositionBroadcastNavigation::Gps(ephemeris),
            sat,
            receive_tow_s,
        )
    }

    fn rejected_navigation_entry_reason(
        navigation: PositionBroadcastNavigation,
        sat: SatId,
        receive_tow_s: f64,
    ) -> MeasurementRejectReason {
        let observation = PositionObservation {
            sat,
            pseudorange_m: 24_000_000.0,
            doppler_hz: None,
            doppler_var_hz2: None,
            cn0_dbhz: 45.0,
            elevation_deg: None,
            weight: 1.0,
            gps_receive_time: Some(GpsTime { week: 2222, tow_s: receive_tow_s }),
            signal_timing: None,
            signal_id: default_position_signal_id(sat),
        };
        let navigation = [navigation];
        let mut rejected = Vec::new();

        let inputs =
            resolve_position_inputs(&[observation], &navigation, receive_tow_s, &mut rejected);

        assert!(inputs.is_empty());
        rejected.into_iter().map(|(_, reason)| reason).next().expect("rejection reason")
    }
}
