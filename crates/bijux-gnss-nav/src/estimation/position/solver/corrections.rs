use crate::estimation::position::navigation::{
    broadcast_group_delay_correction_chain, satellite_state_at_time,
    satellite_state_from_observation, PositionObservationCorrectionChain,
    PositionObservationCorrectionKind, PositionSolveInput, SatelliteState,
    POSITION_OBSERVATION_CORRECTION_ORDER,
};
use crate::models::atmosphere::{
    IonosphereModel, KlobucharCoefficients, KlobucharModel, SaastamoinenModel, TroposphereModel,
};
use bijux_gnss_core::api::{Constellation, Llh, Seconds};

use super::geodesy::{ecef_to_geodetic, elevation_azimuth_deg};
use super::state::{PositionEstimate, SatelliteGeometry, WorkingSetResidual};
use super::{PositionBroadcastNavigation, PositionCorrectedObservation, SPEED_OF_LIGHT_MPS};

pub(super) fn resolve_satellite_geometry(
    inputs: &[PositionSolveInput],
    estimate: &PositionEstimate,
    klobuchar: Option<&KlobucharCoefficients>,
    apply_broadcast_ionosphere: bool,
    apply_broadcast_group_delay: bool,
    apply_troposphere: bool,
) -> Option<Vec<SatelliteGeometry>> {
    let mut geometry = Vec::with_capacity(inputs.len());
    for input in inputs {
        let obs = &input.observation;
        let group_delay_chain = broadcast_group_delay_correction_chain(
            obs,
            &input.navigation,
            apply_broadcast_group_delay,
        );
        let corrected_pseudorange_m = group_delay_chain.corrected_pseudorange_m;
        let mut tau = obs
            .signal_timing
            .map(|timing| timing.signal_travel_time_s.0)
            .unwrap_or(corrected_pseudorange_m / SPEED_OF_LIGHT_MPS);
        let mut state = satellite_state_from_observation(
            &input.navigation,
            input.receive_tow_s,
            corrected_pseudorange_m,
            obs.signal_timing,
        )?;
        let mut converged = false;
        for _ in 0..5 {
            let range_m = geometric_range_m(estimate, &state);
            let receiver_clock_bias_s =
                estimate.constellation_clock_bias_s(obs.sat.constellation)?;
            let next_tau =
                predicted_signal_travel_time_s(range_m, receiver_clock_bias_s, state.clock_bias_s);
            if (next_tau - tau).abs() < 1.0e-9 {
                converged = true;
            }
            tau = next_tau;
            state = satellite_state_at_time(&input.navigation, input.receive_tow_s - tau, tau)?;
            if converged {
                break;
            }
        }
        if !converged {
            return None;
        }
        let iono_delay_m = estimate_broadcast_ionosphere_delay_m(
            estimate,
            input,
            &state,
            klobuchar,
            apply_broadcast_ionosphere,
        );
        let tropo_delay_m = estimate_saastamoinen_delay_m(estimate, &state, apply_troposphere);
        geometry.push(SatelliteGeometry {
            observation: obs.clone(),
            corrected_pseudorange_m,
            broadcast_group_delay_correction_chain: group_delay_chain,
            state,
            iono_delay_m,
            tropo_delay_m,
        });
    }
    Some(geometry)
}

pub(super) fn linearized_pseudorange_row(
    estimate: &PositionEstimate,
    geometry: &SatelliteGeometry,
) -> Option<(f64, Vec<f64>)> {
    let (range_m, mut design_row) =
        linearized_geometry_row(estimate, geometry.observation.sat.constellation, &geometry.state)?;
    let receiver_clock_bias_s =
        estimate.constellation_clock_bias_s(geometry.observation.sat.constellation)?;
    let predicted_pseudorange_m =
        predicted_pseudorange_m(range_m, receiver_clock_bias_s, geometry.state.clock_bias_s);
    let residual_m = geometry.corrected_pseudorange_m
        - geometry.iono_delay_m
        - geometry.tropo_delay_m
        - predicted_pseudorange_m;
    design_row[0] /= range_m;
    design_row[1] /= range_m;
    design_row[2] /= range_m;
    Some((residual_m, design_row))
}

fn estimate_broadcast_ionosphere_delay_m(
    estimate: &PositionEstimate,
    input: &PositionSolveInput,
    state: &SatelliteState,
    klobuchar: Option<&KlobucharCoefficients>,
    apply_broadcast_ionosphere: bool,
) -> f64 {
    if !apply_broadcast_ionosphere {
        return 0.0;
    }
    let receiver_radius_m =
        (estimate.ecef_x_m.powi(2) + estimate.ecef_y_m.powi(2) + estimate.ecef_z_m.powi(2)).sqrt();
    if !receiver_radius_m.is_finite() || receiver_radius_m < 1.0 {
        return 0.0;
    }
    let (latitude_deg, longitude_deg, altitude_m) =
        ecef_to_geodetic(estimate.ecef_x_m, estimate.ecef_y_m, estimate.ecef_z_m);
    let receiver = Llh { lat_deg: latitude_deg, lon_deg: longitude_deg, alt_m: altitude_m };
    let (azimuth_deg, elevation_deg) = elevation_azimuth_deg(
        estimate.ecef_x_m,
        estimate.ecef_y_m,
        estimate.ecef_z_m,
        state.x_m,
        state.y_m,
        state.z_m,
    );
    if !azimuth_deg.is_finite() || !elevation_deg.is_finite() || elevation_deg <= 0.0 {
        return 0.0;
    }
    match &input.navigation {
        PositionBroadcastNavigation::Gps(_) => {
            let Some(coefficients) = klobuchar else {
                return 0.0;
            };
            if input.observation.sat.constellation != Constellation::Gps {
                return 0.0;
            }
            KlobucharModel::new(*coefficients).delay_m(
                receiver,
                azimuth_deg,
                elevation_deg,
                Seconds(input.receive_tow_s),
            )
        }
        PositionBroadcastNavigation::Galileo(navigation) => {
            let Some(signal) = input.observation.signal_id else {
                return 0.0;
            };
            let Some(gps_time) = input.observation.gps_receive_time else {
                return 0.0;
            };
            let (sat_latitude_deg, sat_longitude_deg, sat_altitude_m) =
                ecef_to_geodetic(state.x_m, state.y_m, state.z_m);
            let satellite = Llh {
                lat_deg: sat_latitude_deg,
                lon_deg: sat_longitude_deg,
                alt_m: sat_altitude_m,
            };
            navigation.nequick_delay_m(signal, receiver, satellite, gps_time).unwrap_or(0.0)
        }
        PositionBroadcastNavigation::Beidou(_) | PositionBroadcastNavigation::Glonass(_) => 0.0,
    }
}

fn estimate_saastamoinen_delay_m(
    estimate: &PositionEstimate,
    state: &SatelliteState,
    apply_troposphere: bool,
) -> f64 {
    if !apply_troposphere {
        return 0.0;
    }
    let receiver_radius_m =
        (estimate.ecef_x_m.powi(2) + estimate.ecef_y_m.powi(2) + estimate.ecef_z_m.powi(2)).sqrt();
    if !receiver_radius_m.is_finite() || !(6_000_000.0..=7_000_000.0).contains(&receiver_radius_m) {
        return 0.0;
    }
    let (latitude_deg, longitude_deg, altitude_m) =
        ecef_to_geodetic(estimate.ecef_x_m, estimate.ecef_y_m, estimate.ecef_z_m);
    if !latitude_deg.is_finite()
        || !longitude_deg.is_finite()
        || !altitude_m.is_finite()
        || !(-1_000.0..=20_000.0).contains(&altitude_m)
    {
        return 0.0;
    }
    let receiver = Llh { lat_deg: latitude_deg, lon_deg: longitude_deg, alt_m: altitude_m };
    let (_azimuth_deg, elevation_deg) = elevation_azimuth_deg(
        estimate.ecef_x_m,
        estimate.ecef_y_m,
        estimate.ecef_z_m,
        state.x_m,
        state.y_m,
        state.z_m,
    );
    if !elevation_deg.is_finite() || elevation_deg <= 0.0 {
        return 0.0;
    }
    let model = SaastamoinenModel;
    model.delay_m(receiver, elevation_deg, Seconds(0.0))
}

fn geometric_range_m(estimate: &PositionEstimate, state: &SatelliteState) -> f64 {
    let dx = estimate.ecef_x_m - state.x_m;
    let dy = estimate.ecef_y_m - state.y_m;
    let dz = estimate.ecef_z_m - state.z_m;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn predicted_signal_travel_time_s(
    range_m: f64,
    receiver_clock_bias_s: f64,
    satellite_clock_bias_s: f64,
) -> f64 {
    predicted_pseudorange_m(range_m, receiver_clock_bias_s, satellite_clock_bias_s)
        / SPEED_OF_LIGHT_MPS
}

fn predicted_pseudorange_m(
    range_m: f64,
    receiver_clock_bias_s: f64,
    satellite_clock_bias_s: f64,
) -> f64 {
    range_m + receiver_clock_bias_s * SPEED_OF_LIGHT_MPS
        - satellite_clock_bias_s * SPEED_OF_LIGHT_MPS
}

pub(super) fn corrected_observation_records(
    estimate: &PositionEstimate,
    geometry: &[SatelliteGeometry],
    residuals: &[WorkingSetResidual],
) -> Option<Vec<PositionCorrectedObservation>> {
    geometry
        .iter()
        .zip(residuals)
        .map(|(geometry, residual)| {
            let geometric_range_m = geometric_range_m(estimate, &geometry.state);
            let receiver_clock_bias_s =
                estimate.constellation_clock_bias_s(geometry.observation.sat.constellation)?;
            let correction_chain = corrected_observation_chain(geometry, receiver_clock_bias_s);
            Some(PositionCorrectedObservation {
                sat: geometry.observation.sat,
                signal_id: geometry.observation.signal_id,
                correction_chain,
                geometric_range_m,
                residual_m: residual.residual_m,
            })
        })
        .collect()
}

fn corrected_observation_chain(
    geometry: &SatelliteGeometry,
    receiver_clock_bias_s: f64,
) -> PositionObservationCorrectionChain {
    let mut chain = PositionObservationCorrectionChain::new(geometry.observation.pseudorange_m);
    for kind in POSITION_OBSERVATION_CORRECTION_ORDER {
        match kind {
            PositionObservationCorrectionKind::SatelliteClock => {
                chain.push_component(kind, geometry.state.clock_bias_s * SPEED_OF_LIGHT_MPS)
            }
            PositionObservationCorrectionKind::BroadcastGroupDelay => {
                let delta_m = geometry
                    .broadcast_group_delay_correction_chain
                    .component_delta_m(PositionObservationCorrectionKind::BroadcastGroupDelay);
                let applied =
                    geometry.broadcast_group_delay_correction_chain.components.iter().any(
                        |component| {
                            component.kind == PositionObservationCorrectionKind::BroadcastGroupDelay
                                && component.applied
                        },
                    );
                chain.push_component_with_application(kind, delta_m, applied);
            }
            PositionObservationCorrectionKind::Ionosphere => chain.push_component_with_application(
                kind,
                -geometry.iono_delay_m,
                geometry.iono_delay_m != 0.0,
            ),
            PositionObservationCorrectionKind::Troposphere => chain
                .push_component_with_application(
                    kind,
                    -geometry.tropo_delay_m,
                    geometry.tropo_delay_m != 0.0,
                ),
            PositionObservationCorrectionKind::ReceiverClock => {
                chain.push_component(kind, -receiver_clock_bias_s * SPEED_OF_LIGHT_MPS);
            }
            PositionObservationCorrectionKind::Relativity
            | PositionObservationCorrectionKind::EarthRotation
            | PositionObservationCorrectionKind::ReceiverAntenna
            | PositionObservationCorrectionKind::SatelliteAntenna
            | PositionObservationCorrectionKind::EarthTide
            | PositionObservationCorrectionKind::PhaseWindup => {
                chain.push_unapplied_component(kind);
            }
        }
    }
    chain
}

pub(super) fn linearized_geometry_row(
    estimate: &PositionEstimate,
    constellation: Constellation,
    state: &SatelliteState,
) -> Option<(f64, Vec<f64>)> {
    let dx = estimate.ecef_x_m - state.x_m;
    let dy = estimate.ecef_y_m - state.y_m;
    let dz = estimate.ecef_z_m - state.z_m;
    let range_m = (dx * dx + dy * dy + dz * dz).sqrt();
    if !range_m.is_finite() || range_m <= 0.0 {
        return None;
    }
    let mut design_row = vec![0.0; estimate.clock_model.parameter_len()];
    design_row[0] = dx;
    design_row[1] = dy;
    design_row[2] = dz;
    let clock_design = estimate.clock_model.design_row(constellation)?;
    for (index, coefficient) in clock_design.into_iter().enumerate() {
        design_row[index + 3] = coefficient;
    }
    Some((range_m, design_row))
}
