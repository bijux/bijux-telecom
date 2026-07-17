use std::collections::BTreeMap;

use crate::orbits::beidou::BeidouBroadcastNavigationData;
use crate::orbits::galileo::GalileoBroadcastNavigationData;
use crate::orbits::glonass::GlonassBroadcastNavigationFrame;
use crate::orbits::gps::GpsEphemeris;
use bijux_gnss_core::api::{
    Constellation, GpsTime, ObsEpoch, ObsSatellite, ObsSignalTiming, ObservationStatus, SatId,
    SigId, SignalBand,
};

use super::weighting::weight_from_pseudorange_sigma;

#[derive(Debug, Clone)]
pub struct PositionObservation {
    pub sat: SatId,
    pub pseudorange_m: f64,
    pub doppler_hz: Option<f64>,
    pub doppler_var_hz2: Option<f64>,
    pub cn0_dbhz: f64,
    pub elevation_deg: Option<f64>,
    pub weight: f64,
    pub gps_receive_time: Option<GpsTime>,
    pub signal_timing: Option<ObsSignalTiming>,
    pub signal_id: Option<SigId>,
}

#[derive(Debug, Clone)]
pub enum PositionBroadcastNavigation {
    Gps(GpsEphemeris),
    Galileo(GalileoBroadcastNavigationData),
    Beidou(BeidouBroadcastNavigationData),
    Glonass(GlonassBroadcastNavigationFrame),
}

impl PositionBroadcastNavigation {
    pub fn sat(&self) -> SatId {
        match self {
            Self::Gps(ephemeris) => ephemeris.sat,
            Self::Galileo(navigation) => navigation.sat,
            Self::Beidou(navigation) => navigation.sat,
            Self::Glonass(navigation) => navigation.sat,
        }
    }

    pub fn constellation(&self) -> Constellation {
        self.sat().constellation
    }
}

pub fn position_broadcast_navigation_from_gps_ephemerides(
    ephemerides: &[GpsEphemeris],
) -> Vec<PositionBroadcastNavigation> {
    ephemerides.iter().cloned().map(PositionBroadcastNavigation::Gps).collect()
}

pub fn position_broadcast_navigation_from_galileo_navigations(
    navigations: &[GalileoBroadcastNavigationData],
) -> Vec<PositionBroadcastNavigation> {
    navigations.iter().cloned().map(PositionBroadcastNavigation::Galileo).collect()
}

pub fn position_broadcast_navigation_from_glonass_frames(
    navigation_frames: &[GlonassBroadcastNavigationFrame],
) -> Vec<PositionBroadcastNavigation> {
    navigation_frames.iter().cloned().map(PositionBroadcastNavigation::Glonass).collect()
}

pub fn position_broadcast_navigation_from_beidou_navigations(
    navigations: &[BeidouBroadcastNavigationData],
) -> Vec<PositionBroadcastNavigation> {
    navigations.iter().cloned().map(PositionBroadcastNavigation::Beidou).collect()
}

pub fn position_observations_from_epoch(epoch: &ObsEpoch) -> Vec<PositionObservation> {
    let gps_receive_time = epoch.gps_time();
    let mut preferred_by_sat: BTreeMap<SatId, &ObsSatellite> = BTreeMap::new();
    for observation in &epoch.sats {
        preferred_by_sat
            .entry(observation.signal_id.sat)
            .and_modify(|current| {
                if prefer_position_observation(observation, current) {
                    *current = observation;
                }
            })
            .or_insert(observation);
    }
    preferred_by_sat
        .into_values()
        .map(|observation| {
            let covariance_weight =
                weight_from_pseudorange_sigma(observation.covariance_pseudorange_sigma_m());
            PositionObservation {
                sat: observation.signal_id.sat,
                pseudorange_m: observation.pseudorange_m.0,
                doppler_hz: Some(observation.doppler_hz.0),
                doppler_var_hz2: Some(observation.doppler_var_hz2),
                cn0_dbhz: observation.cn0_dbhz,
                elevation_deg: observation.elevation_deg,
                weight: observation.weight.unwrap_or(1.0) * covariance_weight,
                gps_receive_time,
                signal_timing: observation.timing,
                signal_id: Some(observation.signal_id),
            }
        })
        .collect()
}

fn prefer_position_observation(candidate: &ObsSatellite, current: &ObsSatellite) -> bool {
    let candidate_rank = position_observation_preference(candidate);
    let current_rank = position_observation_preference(current);
    candidate_rank < current_rank
}

fn position_observation_preference(observation: &ObsSatellite) -> (u8, u8, u8, u8) {
    (
        observation_status_rank(observation.observation_status),
        signal_band_rank(observation.signal_id.band),
        if observation.timing.is_some() { 0 } else { 1 },
        if observation.lock_flags.carrier_lock { 0 } else { 1 },
    )
}

fn observation_status_rank(status: ObservationStatus) -> u8 {
    match status {
        ObservationStatus::Accepted => 0,
        ObservationStatus::Weak => 1,
        ObservationStatus::Missing => 2,
        ObservationStatus::Rejected => 3,
        ObservationStatus::Inconsistent => 4,
    }
}

fn signal_band_rank(band: SignalBand) -> u8 {
    match band {
        SignalBand::L1 => 0,
        SignalBand::L2 => 1,
        SignalBand::L5 => 2,
        SignalBand::E1 => 3,
        SignalBand::E5 => 4,
        SignalBand::B1 => 5,
        SignalBand::B2 => 6,
        SignalBand::Unknown => 7,
    }
}

pub(super) fn constellation_primary_band(constellation: Constellation) -> SignalBand {
    match constellation {
        Constellation::Gps => SignalBand::L1,
        Constellation::Galileo => SignalBand::E1,
        Constellation::Glonass => SignalBand::L1,
        Constellation::Beidou => SignalBand::B1,
        Constellation::Unknown => SignalBand::Unknown,
    }
}
