#![allow(missing_docs)]

use crate::corrections::broadcast_group_delay::{
    beidou_broadcast_group_delay_code_bias_m, galileo_broadcast_group_delay_code_bias_m,
    gps_broadcast_group_delay_code_bias_m,
};
use crate::orbits::beidou::{
    beidou_navigation_age, is_beidou_navigation_valid, sat_state_beidou_b1i,
};
use crate::orbits::galileo::{
    galileo_navigation_age, is_galileo_navigation_valid, sat_state_galileo_e1,
};
use crate::orbits::glonass::{
    glonass_gps_minus_glonass_s, glonass_navigation_age, is_glonass_navigation_valid,
    sat_state_glonass_l1,
};
use crate::orbits::gps::{
    gps_ephemeris_age, is_ephemeris_valid, sat_state_gps_l1ca,
};
use bijux_gnss_core::api::{
    default_acquisition_signal, signal_id_wavelength_m, MeasurementRejectReason, ObsSignalTiming,
    SatId, SigId,
};

use super::solver::{PositionBroadcastNavigation, PositionObservation};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
const SATELLITE_RATE_STEP_S: f64 = 1.0e-3;

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
    pub(crate) vx_mps: f64,
    pub(crate) vy_mps: f64,
    pub(crate) vz_mps: f64,
    pub(crate) clock_bias_s: f64,
    pub(crate) clock_drift_s_per_s: f64,
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
    let signal_travel_time_s =
        signal_timing.map(|timing| timing.signal_travel_time_s.0).unwrap_or(pseudorange_m / SPEED_OF_LIGHT_MPS);
    let transmit_tow_s = signal_timing
        .map(|timing| timing.transmit_gps_time.tow_s)
        .unwrap_or(receive_tow_s - signal_travel_time_s);
    satellite_state_at_time(navigation, transmit_tow_s, signal_travel_time_s)
}

pub(crate) fn satellite_state_at_time(
    navigation: &PositionBroadcastNavigation,
    transmit_tow_s: f64,
    signal_travel_time_s: f64,
) -> Option<SatelliteState> {
    let center = satellite_state_sample_at_time(navigation, transmit_tow_s, signal_travel_time_s)?;
    let previous = satellite_state_sample_at_time(
        navigation,
        transmit_tow_s - SATELLITE_RATE_STEP_S,
        signal_travel_time_s,
    )?;
    let next = satellite_state_sample_at_time(
        navigation,
        transmit_tow_s + SATELLITE_RATE_STEP_S,
        signal_travel_time_s,
    )?;
    let inverse_dt = 1.0 / (2.0 * SATELLITE_RATE_STEP_S);

    Some(SatelliteState {
        x_m: center.x_m,
        y_m: center.y_m,
        z_m: center.z_m,
        vx_mps: (next.x_m - previous.x_m) * inverse_dt,
        vy_mps: (next.y_m - previous.y_m) * inverse_dt,
        vz_mps: (next.z_m - previous.z_m) * inverse_dt,
        clock_bias_s: center.clock_bias_s,
        clock_drift_s_per_s: center.clock_drift_s_per_s,
    })
}

#[derive(Debug, Clone, Copy)]
struct SatelliteStateSample {
    x_m: f64,
    y_m: f64,
    z_m: f64,
    clock_bias_s: f64,
    clock_drift_s_per_s: f64,
}

fn satellite_state_sample_at_time(
    navigation: &PositionBroadcastNavigation,
    transmit_tow_s: f64,
    signal_travel_time_s: f64,
) -> Option<SatelliteStateSample> {
    match navigation {
        PositionBroadcastNavigation::Gps(ephemeris) => {
            let state = sat_state_gps_l1ca(ephemeris, transmit_tow_s, signal_travel_time_s);
            Some(SatelliteStateSample {
                x_m: state.x_m,
                y_m: state.y_m,
                z_m: state.z_m,
                clock_bias_s: state.clock_correction.bias_s,
                clock_drift_s_per_s: state.clock_correction.drift_s_per_s,
            })
        }
        PositionBroadcastNavigation::Galileo(navigation) => {
            let state = sat_state_galileo_e1(navigation, transmit_tow_s, signal_travel_time_s);
            Some(SatelliteStateSample {
                x_m: state.x_m,
                y_m: state.y_m,
                z_m: state.z_m,
                clock_bias_s: state.clock_correction.bias_s,
                clock_drift_s_per_s: state.clock_correction.drift_s_per_s,
            })
        }
        PositionBroadcastNavigation::Beidou(navigation) => {
            let state = sat_state_beidou_b1i(navigation, transmit_tow_s, signal_travel_time_s);
            Some(SatelliteStateSample {
                x_m: state.x_m,
                y_m: state.y_m,
                z_m: state.z_m,
                clock_bias_s: state.clock_correction.bias_s,
                clock_drift_s_per_s: state.clock_correction.drift_s_per_s,
            })
        }
        PositionBroadcastNavigation::Glonass(navigation) => {
            let state = sat_state_glonass_l1(navigation, transmit_tow_s, signal_travel_time_s)?;
            Some(SatelliteStateSample {
                x_m: state.x_m,
                y_m: state.y_m,
                z_m: state.z_m,
                clock_bias_s: state.clock_correction.bias_s,
                clock_drift_s_per_s: state.clock_correction.drift_s_per_s,
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
    if !apply_broadcast_group_delay {
        return observation.pseudorange_m;
    }
    observation.pseudorange_m - broadcast_group_delay_code_bias_m(observation.signal_id, navigation)
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
        default_position_signal_id, observation_consistency_metrics, position_signal_id,
        satellite_state_at_time, PositionBroadcastNavigation, PositionObservation,
        SPEED_OF_LIGHT_MPS,
    };
    use crate::estimation::position::solver::geodetic_to_ecef;
    use crate::orbits::gps::GpsEphemeris;
    use bijux_gnss_core::api::{
        signal_id_wavelength_m, Constellation, GpsTime, ObsSignalTiming, SatId, Seconds, SigId,
        SignalBand, SignalCode,
    };

    fn sample_gps_ephemeris() -> GpsEphemeris {
        GpsEphemeris {
            sat: SatId { constellation: Constellation::Gps, prn: 7 },
            iodc: 1,
            iode: 1,
            week: 2209,
            sv_health: 0,
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
        let navigation = PositionBroadcastNavigation::Gps(sample_gps_ephemeris());

        let state =
            satellite_state_at_time(&navigation, 504_018.0, 0.078).expect("satellite state");
        let speed_mps =
            (state.vx_mps * state.vx_mps + state.vy_mps * state.vy_mps + state.vz_mps * state.vz_mps)
                .sqrt();

        assert!(state.x_m.is_finite());
        assert!(state.y_m.is_finite());
        assert!(state.z_m.is_finite());
        assert!(state.clock_bias_s.is_finite());
        assert!(state.clock_drift_s_per_s.is_finite());
        assert!(speed_mps > 100.0, "speed_mps={speed_mps}");
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
}
