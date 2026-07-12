#![allow(missing_docs)]

use crate::corrections::broadcast_group_delay::{
    beidou_broadcast_group_delay_code_bias_m, galileo_broadcast_group_delay_code_bias_m,
    gps_broadcast_group_delay_code_bias_m,
};
use crate::orbits::beidou::{
    beidou_navigation_age, is_beidou_navigation_valid, sat_state_beidou_b1i,
    sat_state_beidou_b1i_from_observation,
};
use crate::orbits::galileo::{
    galileo_navigation_age, is_galileo_navigation_valid, sat_state_galileo_e1,
    sat_state_galileo_e1_from_observation,
};
use crate::orbits::glonass::{
    glonass_gps_minus_glonass_s, glonass_navigation_age, is_glonass_navigation_valid,
    sat_state_glonass_l1, sat_state_glonass_l1_from_observation,
};
use crate::orbits::gps::{
    gps_ephemeris_age, is_ephemeris_valid, sat_state_gps_l1ca, sat_state_gps_l1ca_from_observation,
};
use bijux_gnss_core::api::{MeasurementRejectReason, ObsSignalTiming, SatId, SigId};

use super::solver::{PositionBroadcastNavigation, PositionObservation};

#[derive(Debug, Clone)]
pub(crate) struct PositionSolveInput {
    pub(crate) observation: PositionObservation,
    pub(crate) navigation: PositionBroadcastNavigation,
    pub(crate) receive_tow_s: f64,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct SatelliteState {
    pub(crate) x_m: f64,
    pub(crate) y_m: f64,
    pub(crate) z_m: f64,
    pub(crate) clock_bias_s: f64,
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
            let Some(navigation) = select_valid_navigation(navigation, obs.sat, receive_tow_s)
            else {
                rejected.push((obs.sat, MeasurementRejectReason::InvalidEphemeris));
                return None;
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
    navigation
        .iter()
        .filter(|entry| entry.sat() == sat)
        .filter(|entry| navigation_is_valid(entry, receive_tow_s))
        .min_by(|left, right| {
            let left_age = navigation_age_score(left, receive_tow_s);
            let right_age = navigation_age_score(right, receive_tow_s);
            left_age.0.total_cmp(&right_age.0).then_with(|| left_age.1.total_cmp(&right_age.1))
        })
}

fn navigation_is_valid(navigation: &PositionBroadcastNavigation, receive_tow_s: f64) -> bool {
    match navigation {
        PositionBroadcastNavigation::Gps(ephemeris) => is_ephemeris_valid(ephemeris, receive_tow_s),
        PositionBroadcastNavigation::Galileo(navigation) => {
            is_galileo_navigation_valid(navigation, receive_tow_s)
        }
        PositionBroadcastNavigation::Beidou(navigation) => {
            is_beidou_navigation_valid(navigation, receive_tow_s)
        }
        PositionBroadcastNavigation::Glonass(navigation) => {
            is_glonass_navigation_valid(navigation, receive_tow_s)
        }
    }
}

fn navigation_age_score(
    navigation: &PositionBroadcastNavigation,
    receive_tow_s: f64,
) -> (f64, f64) {
    match navigation {
        PositionBroadcastNavigation::Gps(ephemeris) => {
            let age = gps_ephemeris_age(ephemeris, receive_tow_s);
            (age.toe_age_s.max(age.toc_age_s), age.toe_age_s + age.toc_age_s)
        }
        PositionBroadcastNavigation::Galileo(navigation) => {
            let age = galileo_navigation_age(navigation, receive_tow_s);
            (age.toe_age_s.max(age.toc_age_s), age.toe_age_s + age.toc_age_s)
        }
        PositionBroadcastNavigation::Beidou(navigation) => {
            let age = beidou_navigation_age(navigation, receive_tow_s);
            (age.toe_age_s.max(age.toc_age_s), age.toe_age_s + age.toc_age_s)
        }
        PositionBroadcastNavigation::Glonass(navigation) => {
            let age = glonass_navigation_age(navigation, receive_tow_s)
                .expect("valid GLONASS navigation age");
            (age.age_s, age.age_s)
        }
    }
}

pub(crate) fn satellite_state_from_observation(
    navigation: &PositionBroadcastNavigation,
    receive_tow_s: f64,
    pseudorange_m: f64,
    signal_timing: Option<ObsSignalTiming>,
) -> Option<SatelliteState> {
    match navigation {
        PositionBroadcastNavigation::Gps(ephemeris) => {
            let state = sat_state_gps_l1ca_from_observation(
                ephemeris,
                receive_tow_s,
                pseudorange_m,
                signal_timing,
            );
            Some(SatelliteState {
                x_m: state.x_m,
                y_m: state.y_m,
                z_m: state.z_m,
                clock_bias_s: state.clock_correction.bias_s,
            })
        }
        PositionBroadcastNavigation::Galileo(navigation) => {
            let state = sat_state_galileo_e1_from_observation(
                navigation,
                receive_tow_s,
                pseudorange_m,
                signal_timing,
            );
            Some(SatelliteState {
                x_m: state.x_m,
                y_m: state.y_m,
                z_m: state.z_m,
                clock_bias_s: state.clock_correction.bias_s,
            })
        }
        PositionBroadcastNavigation::Beidou(navigation) => {
            let state = sat_state_beidou_b1i_from_observation(
                navigation,
                receive_tow_s,
                pseudorange_m,
                signal_timing,
            );
            Some(SatelliteState {
                x_m: state.x_m,
                y_m: state.y_m,
                z_m: state.z_m,
                clock_bias_s: state.clock_correction.bias_s,
            })
        }
        PositionBroadcastNavigation::Glonass(navigation) => {
            let state = sat_state_glonass_l1_from_observation(
                navigation,
                receive_tow_s,
                pseudorange_m,
                signal_timing,
            )?;
            Some(SatelliteState {
                x_m: state.x_m,
                y_m: state.y_m,
                z_m: state.z_m,
                clock_bias_s: state.clock_correction.bias_s,
            })
        }
    }
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
                clock_bias_s: state.clock_correction.bias_s,
            })
        }
        PositionBroadcastNavigation::Galileo(navigation) => {
            let state = sat_state_galileo_e1(navigation, transmit_tow_s, signal_travel_time_s);
            Some(SatelliteState {
                x_m: state.x_m,
                y_m: state.y_m,
                z_m: state.z_m,
                clock_bias_s: state.clock_correction.bias_s,
            })
        }
        PositionBroadcastNavigation::Beidou(navigation) => {
            let state = sat_state_beidou_b1i(navigation, transmit_tow_s, signal_travel_time_s);
            Some(SatelliteState {
                x_m: state.x_m,
                y_m: state.y_m,
                z_m: state.z_m,
                clock_bias_s: state.clock_correction.bias_s,
            })
        }
        PositionBroadcastNavigation::Glonass(navigation) => {
            let state = sat_state_glonass_l1(navigation, transmit_tow_s, signal_travel_time_s)?;
            Some(SatelliteState {
                x_m: state.x_m,
                y_m: state.y_m,
                z_m: state.z_m,
                clock_bias_s: state.clock_correction.bias_s,
            })
        }
    }
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
    if !apply_broadcast_group_delay {
        return observation.pseudorange_m;
    }
    observation.pseudorange_m - broadcast_group_delay_code_bias_m(observation.signal_id, navigation)
}
