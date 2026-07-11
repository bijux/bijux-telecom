use bijux_gnss_core::api::{
    ArtifactPayloadValidate, Constellation, DiagnosticEvent, DiagnosticSeverity, SigId, SignalBand,
};
use serde::{Deserialize, Serialize};

use super::double_difference::RtkDoubleDifferenceObservation;
use crate::{
    linalg::Matrix,
    orbits::gps::{sat_state_gps_l1ca_from_observation, GpsEphemeris},
};

const MIN_OBSERVATION_VARIANCE_M2: f64 = 1.0e-6;
const POSITION_CONVERGENCE_M: f64 = 1.0e-4;
const SOLVER_ITERATIONS: usize = 8;
const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

/// Float ambiguity estimate associated with one RTK double-difference observation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RtkFloatAmbiguityEstimate {
    /// Signal identity for the non-reference satellite.
    pub sig: SigId,
    /// Signal identity for the chosen reference satellite.
    pub ref_sig: SigId,
    /// Float ambiguity estimate in cycles.
    pub float_cycles: f64,
    /// Float ambiguity variance in square cycles.
    pub variance_cycles2: f64,
}

impl ArtifactPayloadValidate for RtkFloatAmbiguityEstimate {
    fn validate_payload(&self) -> Vec<DiagnosticEvent> {
        let mut events = Vec::new();
        if !self.float_cycles.is_finite() {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_FLOAT_AMBIGUITY_NUMERIC_INVALID",
                "float ambiguity contains NaN/Inf",
            ));
        }
        if !self.variance_cycles2.is_finite() || self.variance_cycles2 < 0.0 {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_FLOAT_AMBIGUITY_VARIANCE_INVALID",
                "float ambiguity variance is invalid",
            ));
        }
        events
    }
}

/// Float RTK baseline estimate and covariance expressed in the base-station ENU frame.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RtkFloatBaselineSolution {
    /// Rover-minus-base baseline in the base-station ENU frame.
    pub enu_m: [f64; 3],
    /// Rover-minus-base covariance in the base-station ENU frame.
    pub covariance_enu_m2: [[f64; 3]; 3],
    /// Float double-difference ambiguities estimated alongside the baseline.
    pub float_ambiguities: Vec<RtkFloatAmbiguityEstimate>,
}

impl ArtifactPayloadValidate for RtkFloatBaselineSolution {
    fn validate_payload(&self) -> Vec<DiagnosticEvent> {
        let mut events = Vec::new();
        if !self.enu_m.iter().all(|value| value.is_finite()) {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_FLOAT_BASELINE_NUMERIC_INVALID",
                "float baseline ENU contains NaN/Inf",
            ));
        }
        if !self.covariance_enu_m2.iter().flat_map(|row| row.iter()).all(|value| value.is_finite())
        {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_FLOAT_BASELINE_COVARIANCE_INVALID",
                "float baseline covariance contains NaN/Inf",
            ));
        }
        for ambiguity in &self.float_ambiguities {
            events.extend(ambiguity.validate_payload());
        }
        events
    }
}

/// Estimate a float RTK baseline from double-difference code and carrier observations.
pub fn rtk_float_baseline_from_double_differences(
    observations: &[RtkDoubleDifferenceObservation],
    base_ecef_m: [f64; 3],
    ephemerides: &[GpsEphemeris],
    receive_time_s: f64,
) -> Option<RtkFloatBaselineSolution> {
    rtk_float_baseline_from_double_differences_with_rover_prior(
        observations,
        base_ecef_m,
        base_ecef_m,
        ephemerides,
        receive_time_s,
    )
}

/// Estimate a float RTK baseline from double differences and an explicit rover prior.
pub fn rtk_float_baseline_from_double_differences_with_rover_prior(
    observations: &[RtkDoubleDifferenceObservation],
    base_ecef_m: [f64; 3],
    rover_prior_ecef_m: [f64; 3],
    ephemerides: &[GpsEphemeris],
    receive_time_s: f64,
) -> Option<RtkFloatBaselineSolution> {
    if observations.len() < 3 {
        return None;
    }

    let ambiguity_count = observations.len();
    let state_dimension = 3 + ambiguity_count;
    let mut rover_ecef_m = rover_prior_ecef_m;
    let mut float_ambiguities_cycles = vec![0.0; ambiguity_count];
    let mut covariance = None;

    for _ in 0..SOLVER_ITERATIONS {
        let mut design = Matrix::new(ambiguity_count * 2, state_dimension, 0.0);
        let mut residuals_m = vec![0.0; ambiguity_count * 2];
        let mut weights = vec![0.0; ambiguity_count * 2];

        for (index, observation) in observations.iter().enumerate() {
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

            let rover_signal_position = [
                rover_signal_satellite.x_m,
                rover_signal_satellite.y_m,
                rover_signal_satellite.z_m,
            ];
            let base_signal_position =
                [base_signal_satellite.x_m, base_signal_satellite.y_m, base_signal_satellite.z_m];
            let rover_reference_position = [
                rover_reference_satellite.x_m,
                rover_reference_satellite.y_m,
                rover_reference_satellite.z_m,
            ];
            let base_reference_position = [
                base_reference_satellite.x_m,
                base_reference_satellite.y_m,
                base_reference_satellite.z_m,
            ];

            let rover_signal_range_m = geometric_range_m(rover_ecef_m, rover_signal_position);
            let base_signal_range_m = geometric_range_m(base_ecef_m, base_signal_position);
            let rover_reference_range_m = geometric_range_m(rover_ecef_m, rover_reference_position);
            let base_reference_range_m = geometric_range_m(base_ecef_m, base_reference_position);
            let modeled_code_m = (rover_signal_range_m - base_signal_range_m)
                - (rover_reference_range_m - base_reference_range_m);
            let geometry_row =
                geometry_row(rover_ecef_m, rover_signal_position, rover_reference_position);
            let wavelength_m = wavelength_m(observation.sig)?;

            let code_row = index * 2;
            residuals_m[code_row] = observation.code_m - modeled_code_m;
            weights[code_row] = reciprocal_variance_weight(
                observation.code_variance_m2.max(MIN_OBSERVATION_VARIANCE_M2),
            );
            for axis in 0..3 {
                design[(code_row, axis)] = geometry_row[axis];
            }

            let phase_row = code_row + 1;
            let modeled_phase_m = modeled_code_m + wavelength_m * float_ambiguities_cycles[index];
            residuals_m[phase_row] = observation.phase_cycles * wavelength_m - modeled_phase_m;
            weights[phase_row] = reciprocal_variance_weight(
                (observation.phase_variance_cycles2 * wavelength_m * wavelength_m)
                    .max(MIN_OBSERVATION_VARIANCE_M2),
            );
            for axis in 0..3 {
                design[(phase_row, axis)] = geometry_row[axis];
            }
            design[(phase_row, 3 + index)] = wavelength_m;
        }

        let (delta_state, solved_covariance) =
            solve_weighted_normal_equation(&design, &residuals_m, &weights)?;
        rover_ecef_m[0] += delta_state[0];
        rover_ecef_m[1] += delta_state[1];
        rover_ecef_m[2] += delta_state[2];
        for (index, ambiguity) in float_ambiguities_cycles.iter_mut().enumerate() {
            *ambiguity += delta_state[3 + index];
        }
        let position_step_m = (delta_state[0] * delta_state[0]
            + delta_state[1] * delta_state[1]
            + delta_state[2] * delta_state[2])
            .sqrt();
        covariance = Some(solved_covariance);
        if position_step_m <= POSITION_CONVERGENCE_M {
            break;
        }
    }

    let covariance = covariance?;
    let covariance_ecef_m2 = covariance_3x3(&covariance);
    let covariance_enu_m2 = covariance_ecef_to_enu(base_ecef_m, covariance_ecef_m2);
    let (lat_deg, lon_deg, alt_m) = crate::estimation::position::solver::ecef_to_geodetic(
        base_ecef_m[0],
        base_ecef_m[1],
        base_ecef_m[2],
    );
    let (east_m, north_m, up_m) = crate::estimation::position::solver::ecef_to_enu(
        rover_ecef_m[0],
        rover_ecef_m[1],
        rover_ecef_m[2],
        lat_deg,
        lon_deg,
        alt_m,
    );

    let mut float_ambiguities = Vec::with_capacity(ambiguity_count);
    for (index, observation) in observations.iter().enumerate() {
        float_ambiguities.push(RtkFloatAmbiguityEstimate {
            sig: observation.sig,
            ref_sig: observation.ref_sig,
            float_cycles: float_ambiguities_cycles[index],
            variance_cycles2: covariance[(3 + index, 3 + index)].max(0.0),
        });
    }

    Some(RtkFloatBaselineSolution {
        enu_m: [east_m, north_m, up_m],
        covariance_enu_m2,
        float_ambiguities,
    })
}

fn reciprocal_variance_weight(variance_m2: f64) -> f64 {
    if variance_m2.is_finite() && variance_m2 > 0.0 {
        1.0 / variance_m2
    } else {
        0.0
    }
}

fn solve_weighted_normal_equation(
    design: &Matrix,
    residuals_m: &[f64],
    weights: &[f64],
) -> Option<(Vec<f64>, Matrix)> {
    if design.rows() != residuals_m.len() || residuals_m.len() != weights.len() {
        return None;
    }
    let mut normal = Matrix::new(design.cols(), design.cols(), 0.0);
    let mut rhs = vec![0.0; design.cols()];
    for row in 0..design.rows() {
        let weight = weights[row];
        if !weight.is_finite() || weight <= 0.0 {
            return None;
        }
        for col in 0..design.cols() {
            let design_value = design[(row, col)];
            rhs[col] += design_value * residuals_m[row] * weight;
            for inner in 0..design.cols() {
                normal[(col, inner)] += design_value * design[(row, inner)] * weight;
            }
        }
    }
    let covariance = normal.invert()?;
    let mut solution = vec![0.0; design.cols()];
    for row in 0..covariance.rows() {
        for col in 0..covariance.cols() {
            solution[row] += covariance[(row, col)] * rhs[col];
        }
    }
    Some((solution, covariance))
}

fn covariance_3x3(covariance: &Matrix) -> [[f64; 3]; 3] {
    let mut out = [[0.0; 3]; 3];
    for row in 0..3 {
        for col in 0..3 {
            out[row][col] = covariance[(row, col)];
        }
    }
    out
}

fn covariance_ecef_to_enu(
    base_ecef_m: [f64; 3],
    covariance_ecef_m2: [[f64; 3]; 3],
) -> [[f64; 3]; 3] {
    let (lat_deg, lon_deg, _alt_m) = crate::estimation::position::solver::ecef_to_geodetic(
        base_ecef_m[0],
        base_ecef_m[1],
        base_ecef_m[2],
    );
    let (sin_lat, cos_lat) = lat_deg.to_radians().sin_cos();
    let (sin_lon, cos_lon) = lon_deg.to_radians().sin_cos();
    let rotation = [
        [-sin_lon, cos_lon, 0.0],
        [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
        [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
    ];

    let mut rotated = [[0.0; 3]; 3];
    for row in 0..3 {
        for col in 0..3 {
            for inner in 0..3 {
                rotated[row][col] += rotation[row][inner] * covariance_ecef_m2[inner][col];
            }
        }
    }
    let mut covariance_enu_m2 = [[0.0; 3]; 3];
    for row in 0..3 {
        for col in 0..3 {
            for inner in 0..3 {
                covariance_enu_m2[row][col] += rotated[row][inner] * rotation[col][inner];
            }
        }
    }
    covariance_enu_m2
}

fn geometry_row(
    rover_ecef_m: [f64; 3],
    signal_satellite_ecef_m: [f64; 3],
    reference_satellite_ecef_m: [f64; 3],
) -> [f64; 3] {
    let signal_unit = range_gradient(rover_ecef_m, signal_satellite_ecef_m);
    let reference_unit = range_gradient(rover_ecef_m, reference_satellite_ecef_m);
    [
        signal_unit[0] - reference_unit[0],
        signal_unit[1] - reference_unit[1],
        signal_unit[2] - reference_unit[2],
    ]
}

fn range_gradient(receiver_ecef_m: [f64; 3], satellite_ecef_m: [f64; 3]) -> [f64; 3] {
    let range_m = geometric_range_m(receiver_ecef_m, satellite_ecef_m).max(1.0);
    [
        (receiver_ecef_m[0] - satellite_ecef_m[0]) / range_m,
        (receiver_ecef_m[1] - satellite_ecef_m[1]) / range_m,
        (receiver_ecef_m[2] - satellite_ecef_m[2]) / range_m,
    ]
}

fn geometric_range_m(receiver_ecef_m: [f64; 3], satellite_ecef_m: [f64; 3]) -> f64 {
    let dx = receiver_ecef_m[0] - satellite_ecef_m[0];
    let dy = receiver_ecef_m[1] - satellite_ecef_m[1];
    let dz = receiver_ecef_m[2] - satellite_ecef_m[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn wavelength_m(signal: SigId) -> Option<f64> {
    if signal.sat.constellation != Constellation::Gps || signal.band != SignalBand::L1 {
        return None;
    }
    Some(SPEED_OF_LIGHT_MPS / bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value())
}
