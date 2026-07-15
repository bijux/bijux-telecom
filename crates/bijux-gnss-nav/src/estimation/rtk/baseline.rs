use bijux_gnss_core::api::{
    ArtifactPayloadValidate, Constellation, DiagnosticEvent, DiagnosticSeverity, ReceiverRole,
    SigId,
};
use bijux_gnss_signal::api::{glonass_l1_carrier_hz, signal_id_wavelength_m};
use serde::{Deserialize, Serialize};

use super::double_difference::{
    rtk_double_difference_code_covariance_matrix, rtk_double_difference_phase_covariance_matrix,
    RtkDoubleDifferenceObservation, RtkGlonassInterFrequencyBiasStatus,
};
use crate::{
    linalg::Matrix,
    orbits::gps::{sat_state_gps_l1ca_from_observation, GpsEphemeris},
};

const MIN_OBSERVATION_VARIANCE_M2: f64 = 1.0e-6;
const POSITION_CONVERGENCE_M: f64 = 1.0e-4;
const SOLVER_ITERATIONS: usize = 8;
const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

/// GNSS time scale used to produce an RTK satellite state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RtkConstellationTimeScale {
    Gps,
    Galileo,
    Beidou,
    Glonass,
    Unknown,
}

impl RtkConstellationTimeScale {
    pub fn for_constellation(constellation: Constellation) -> Self {
        match constellation {
            Constellation::Gps => Self::Gps,
            Constellation::Galileo => Self::Galileo,
            Constellation::Beidou => Self::Beidou,
            Constellation::Glonass => Self::Glonass,
            Constellation::Unknown => Self::Unknown,
        }
    }
}

/// Satellite position evidence for one receiver-side RTK observable.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct RtkSatelliteStateEvidence {
    pub sig: SigId,
    pub receiver_role: ReceiverRole,
    pub time_scale: RtkConstellationTimeScale,
    pub receive_time_s: f64,
    pub transmit_time_s: Option<f64>,
    pub ecef_m: [f64; 3],
}

/// Satellite state evidence needed to model one RTK double-difference observation.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct RtkDoubleDifferenceSatelliteStates {
    pub sig: SigId,
    pub ref_sig: SigId,
    pub rover_signal: RtkSatelliteStateEvidence,
    pub base_signal: RtkSatelliteStateEvidence,
    pub rover_reference: RtkSatelliteStateEvidence,
    pub base_reference: RtkSatelliteStateEvidence,
}

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
        if self.sig.sat.constellation != self.ref_sig.sat.constellation {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_FLOAT_AMBIGUITY_CONSTELLATION_MISMATCH",
                "float ambiguity signal and reference satellites belong to different constellations",
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
    /// Cross-covariance between baseline ENU components and float ambiguities.
    pub enu_ambiguity_covariance_m_cycles: Vec<Vec<f64>>,
    /// Float double-difference ambiguities estimated alongside the baseline.
    pub float_ambiguities: Vec<RtkFloatAmbiguityEstimate>,
    /// Full float ambiguity covariance ordered like `float_ambiguities`.
    pub ambiguity_covariance_cycles2: Vec<Vec<f64>>,
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
        if self.enu_ambiguity_covariance_m_cycles.len() != 3
            || self
                .enu_ambiguity_covariance_m_cycles
                .iter()
                .any(|row| row.len() != self.float_ambiguities.len())
        {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_FLOAT_BASELINE_CROSS_COVARIANCE_SHAPE_INVALID",
                "float baseline cross-covariance shape does not match ENU and ambiguity dimensions",
            ));
        } else if !self
            .enu_ambiguity_covariance_m_cycles
            .iter()
            .flat_map(|row| row.iter())
            .all(|value| value.is_finite())
        {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_FLOAT_BASELINE_CROSS_COVARIANCE_NUMERIC_INVALID",
                "float baseline cross-covariance contains NaN/Inf",
            ));
        }
        for ambiguity in &self.float_ambiguities {
            events.extend(ambiguity.validate_payload());
        }
        if self.ambiguity_covariance_cycles2.len() != self.float_ambiguities.len()
            || self
                .ambiguity_covariance_cycles2
                .iter()
                .any(|row| row.len() != self.float_ambiguities.len())
        {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_FLOAT_AMBIGUITY_COVARIANCE_SHAPE_INVALID",
                "float ambiguity covariance shape does not match the ambiguity count",
            ));
        } else if !self
            .ambiguity_covariance_cycles2
            .iter()
            .flat_map(|row| row.iter())
            .all(|value| value.is_finite())
        {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_FLOAT_AMBIGUITY_COVARIANCE_NUMERIC_INVALID",
                "float ambiguity covariance contains NaN/Inf",
            ));
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
    let satellite_states =
        gps_satellite_states_for_double_differences(observations, ephemerides, receive_time_s)?;
    rtk_float_baseline_from_double_differences_with_satellite_states_and_rover_prior(
        observations,
        base_ecef_m,
        rover_prior_ecef_m,
        &satellite_states,
    )
}

/// Estimate a float RTK baseline from constellation-aware satellite state evidence.
pub fn rtk_float_baseline_from_double_differences_with_satellite_states(
    observations: &[RtkDoubleDifferenceObservation],
    base_ecef_m: [f64; 3],
    satellite_states: &[RtkDoubleDifferenceSatelliteStates],
) -> Option<RtkFloatBaselineSolution> {
    rtk_float_baseline_from_double_differences_with_satellite_states_and_rover_prior(
        observations,
        base_ecef_m,
        base_ecef_m,
        satellite_states,
    )
}

/// Estimate a float RTK baseline from constellation-aware satellite state evidence and a rover prior.
pub fn rtk_float_baseline_from_double_differences_with_satellite_states_and_rover_prior(
    observations: &[RtkDoubleDifferenceObservation],
    base_ecef_m: [f64; 3],
    rover_prior_ecef_m: [f64; 3],
    satellite_states: &[RtkDoubleDifferenceSatelliteStates],
) -> Option<RtkFloatBaselineSolution> {
    if observations.len() < 3 {
        return None;
    }
    if satellite_states.len() < observations.len() {
        return None;
    }

    let ambiguity_count = observations.len();
    let state_dimension = 3 + ambiguity_count;
    let measurement_covariance_m2 = measurement_covariance_m2(observations)?;
    let mut rover_ecef_m = rover_prior_ecef_m;
    let mut float_ambiguities_cycles = vec![0.0; ambiguity_count];
    let mut covariance = None;

    for _ in 0..SOLVER_ITERATIONS {
        let mut design = Matrix::new(ambiguity_count * 2, state_dimension, 0.0);
        let mut residuals_m = vec![0.0; ambiguity_count * 2];

        for (index, observation) in observations.iter().enumerate() {
            let satellite_state = satellite_states.iter().find(|state| {
                state.sig == observation.sig && state.ref_sig == observation.ref_sig
            })?;
            if !double_difference_satellite_states_are_valid(observation, satellite_state) {
                return None;
            }

            let rover_signal_position = satellite_state.rover_signal.ecef_m;
            let base_signal_position = satellite_state.base_signal.ecef_m;
            let rover_reference_position = satellite_state.rover_reference.ecef_m;
            let base_reference_position = satellite_state.base_reference.ecef_m;

            let rover_signal_range_m = geometric_range_m(rover_ecef_m, rover_signal_position);
            let base_signal_range_m = geometric_range_m(base_ecef_m, base_signal_position);
            let rover_reference_range_m = geometric_range_m(rover_ecef_m, rover_reference_position);
            let base_reference_range_m = geometric_range_m(base_ecef_m, base_reference_position);
            let modeled_code_m = (rover_signal_range_m - base_signal_range_m)
                - (rover_reference_range_m - base_reference_range_m);
            let geometry_row =
                geometry_row(rover_ecef_m, rover_signal_position, rover_reference_position);
            let wavelength_m = observation_wavelength_m(observation)?;

            let code_row = index * 2;
            residuals_m[code_row] = observation.code_m - modeled_code_m;
            for axis in 0..3 {
                design[(code_row, axis)] = geometry_row[axis];
            }

            let phase_row = code_row + 1;
            let modeled_phase_m = modeled_code_m + wavelength_m * float_ambiguities_cycles[index];
            residuals_m[phase_row] = observation.phase_cycles * wavelength_m - modeled_phase_m;
            for axis in 0..3 {
                design[(phase_row, axis)] = geometry_row[axis];
            }
            design[(phase_row, 3 + index)] = wavelength_m;
        }

        let (delta_state, solved_covariance) =
            solve_generalized_least_squares(&design, &residuals_m, &measurement_covariance_m2)?;
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
    let enu_ambiguity_covariance_m_cycles =
        covariance_enu_ambiguity_block(base_ecef_m, &covariance, 3, ambiguity_count);
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
    let ambiguity_covariance_cycles2 = covariance_block(&covariance, 3, ambiguity_count);

    Some(RtkFloatBaselineSolution {
        enu_m: [east_m, north_m, up_m],
        covariance_enu_m2,
        enu_ambiguity_covariance_m_cycles,
        float_ambiguities,
        ambiguity_covariance_cycles2,
    })
}

fn measurement_covariance_m2(observations: &[RtkDoubleDifferenceObservation]) -> Option<Matrix> {
    let code_covariance_m2 = rtk_double_difference_code_covariance_matrix(observations)?;
    let phase_covariance_cycles2 = rtk_double_difference_phase_covariance_matrix(observations)?;
    let row_count = observations.len() * 2;
    let mut covariance = Matrix::new(row_count, row_count, 0.0);
    let wavelengths_m =
        observations.iter().map(observation_wavelength_m).collect::<Option<Vec<_>>>()?;

    for row in 0..observations.len() {
        if code_covariance_m2[row].len() != observations.len()
            || phase_covariance_cycles2[row].len() != observations.len()
        {
            return None;
        }
        for col in 0..observations.len() {
            let code_value_m2 = code_covariance_m2[row][col];
            let phase_value_m2 =
                phase_covariance_cycles2[row][col] * wavelengths_m[row] * wavelengths_m[col];
            if !code_value_m2.is_finite() || !phase_value_m2.is_finite() {
                return None;
            }
            covariance[(row * 2, col * 2)] = code_value_m2;
            covariance[(row * 2 + 1, col * 2 + 1)] = phase_value_m2;
        }
        covariance[(row * 2, row * 2)] =
            covariance[(row * 2, row * 2)].max(MIN_OBSERVATION_VARIANCE_M2);
        covariance[(row * 2 + 1, row * 2 + 1)] =
            covariance[(row * 2 + 1, row * 2 + 1)].max(MIN_OBSERVATION_VARIANCE_M2);
    }
    Some(covariance)
}

fn gps_satellite_states_for_double_differences(
    observations: &[RtkDoubleDifferenceObservation],
    ephemerides: &[GpsEphemeris],
    receive_time_s: f64,
) -> Option<Vec<RtkDoubleDifferenceSatelliteStates>> {
    observations
        .iter()
        .map(|observation| {
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
            Some(RtkDoubleDifferenceSatelliteStates {
                sig: observation.sig,
                ref_sig: observation.ref_sig,
                rover_signal: gps_satellite_state_evidence(
                    observation.sig,
                    ReceiverRole::Rover,
                    receive_time_s,
                    [
                        rover_signal_satellite.x_m,
                        rover_signal_satellite.y_m,
                        rover_signal_satellite.z_m,
                    ],
                ),
                base_signal: gps_satellite_state_evidence(
                    observation.sig,
                    ReceiverRole::Base,
                    receive_time_s,
                    [
                        base_signal_satellite.x_m,
                        base_signal_satellite.y_m,
                        base_signal_satellite.z_m,
                    ],
                ),
                rover_reference: gps_satellite_state_evidence(
                    observation.ref_sig,
                    ReceiverRole::Rover,
                    receive_time_s,
                    [
                        rover_reference_satellite.x_m,
                        rover_reference_satellite.y_m,
                        rover_reference_satellite.z_m,
                    ],
                ),
                base_reference: gps_satellite_state_evidence(
                    observation.ref_sig,
                    ReceiverRole::Base,
                    receive_time_s,
                    [
                        base_reference_satellite.x_m,
                        base_reference_satellite.y_m,
                        base_reference_satellite.z_m,
                    ],
                ),
            })
        })
        .collect()
}

fn gps_satellite_state_evidence(
    sig: SigId,
    receiver_role: ReceiverRole,
    receive_time_s: f64,
    ecef_m: [f64; 3],
) -> RtkSatelliteStateEvidence {
    RtkSatelliteStateEvidence {
        sig,
        receiver_role,
        time_scale: RtkConstellationTimeScale::Gps,
        receive_time_s,
        transmit_time_s: None,
        ecef_m,
    }
}

fn double_difference_satellite_states_are_valid(
    observation: &RtkDoubleDifferenceObservation,
    states: &RtkDoubleDifferenceSatelliteStates,
) -> bool {
    observation.sig == states.sig
        && observation.ref_sig == states.ref_sig
        && observation.sig.sat.constellation == observation.ref_sig.sat.constellation
        && satellite_state_evidence_is_valid(
            &states.rover_signal,
            observation.sig,
            ReceiverRole::Rover,
        )
        && satellite_state_evidence_is_valid(
            &states.base_signal,
            observation.sig,
            ReceiverRole::Base,
        )
        && satellite_state_evidence_is_valid(
            &states.rover_reference,
            observation.ref_sig,
            ReceiverRole::Rover,
        )
        && satellite_state_evidence_is_valid(
            &states.base_reference,
            observation.ref_sig,
            ReceiverRole::Base,
        )
}

fn satellite_state_evidence_is_valid(
    state: &RtkSatelliteStateEvidence,
    expected_sig: SigId,
    expected_role: ReceiverRole,
) -> bool {
    state.sig == expected_sig
        && receiver_role_matches(state.receiver_role, expected_role)
        && state.time_scale
            == RtkConstellationTimeScale::for_constellation(expected_sig.sat.constellation)
        && state.receive_time_s.is_finite()
        && state.transmit_time_s.map(|transmit_time_s| transmit_time_s.is_finite()).unwrap_or(true)
        && state.ecef_m.iter().all(|value| value.is_finite())
}

fn receiver_role_matches(actual: ReceiverRole, expected: ReceiverRole) -> bool {
    matches!(
        (actual, expected),
        (ReceiverRole::Base, ReceiverRole::Base) | (ReceiverRole::Rover, ReceiverRole::Rover)
    )
}

fn solve_generalized_least_squares(
    design: &Matrix,
    residuals_m: &[f64],
    measurement_covariance_m2: &Matrix,
) -> Option<(Vec<f64>, Matrix)> {
    if design.rows() != residuals_m.len()
        || measurement_covariance_m2.rows() != design.rows()
        || measurement_covariance_m2.cols() != design.rows()
    {
        return None;
    }
    let information = measurement_covariance_m2.invert()?;
    let mut normal = Matrix::new(design.cols(), design.cols(), 0.0);
    let mut rhs = vec![0.0; design.cols()];
    for row in 0..design.rows() {
        for col in 0..design.cols() {
            for obs_row in 0..design.rows() {
                rhs[col] += design[(row, col)] * information[(row, obs_row)] * residuals_m[obs_row];
                for inner in 0..design.cols() {
                    normal[(col, inner)] +=
                        design[(row, col)] * information[(row, obs_row)] * design[(obs_row, inner)];
                }
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

fn covariance_block(covariance: &Matrix, start: usize, size: usize) -> Vec<Vec<f64>> {
    let mut block = vec![vec![0.0; size]; size];
    for row in 0..size {
        for col in 0..size {
            block[row][col] = covariance[(start + row, start + col)];
        }
    }
    block
}

fn covariance_enu_ambiguity_block(
    base_ecef_m: [f64; 3],
    covariance: &Matrix,
    ambiguity_start: usize,
    ambiguity_count: usize,
) -> Vec<Vec<f64>> {
    let rotation = ecef_to_enu_rotation(base_ecef_m);
    let mut ecef_block = Matrix::new(3, ambiguity_count, 0.0);
    for row in 0..3 {
        for col in 0..ambiguity_count {
            ecef_block[(row, col)] = covariance[(row, ambiguity_start + col)];
        }
    }
    let enu_block = rotation.mul(&ecef_block);
    let mut out = vec![vec![0.0; ambiguity_count]; 3];
    for row in 0..3 {
        for col in 0..ambiguity_count {
            out[row][col] = enu_block[(row, col)];
        }
    }
    out
}

fn ecef_to_enu_rotation(base_ecef_m: [f64; 3]) -> Matrix {
    let (lat_deg, lon_deg, _alt_m) = crate::estimation::position::solver::ecef_to_geodetic(
        base_ecef_m[0],
        base_ecef_m[1],
        base_ecef_m[2],
    );
    let (sin_lat, cos_lat) = lat_deg.to_radians().sin_cos();
    let (sin_lon, cos_lon) = lon_deg.to_radians().sin_cos();
    Matrix::from_parts(
        3,
        3,
        vec![
            -sin_lon,
            cos_lon,
            0.0,
            -sin_lat * cos_lon,
            -sin_lat * sin_lon,
            cos_lat,
            cos_lat * cos_lon,
            cos_lat * sin_lon,
            sin_lat,
        ],
    )
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

fn observation_wavelength_m(observation: &RtkDoubleDifferenceObservation) -> Option<f64> {
    if observation.sig.sat.constellation != Constellation::Glonass {
        return Some(signal_id_wavelength_m(observation.sig)?.0);
    }

    let evidence = observation.glonass_inter_frequency_bias;
    if evidence.status != RtkGlonassInterFrequencyBiasStatus::BiasHandled
        || evidence.signal_channel != evidence.reference_channel
    {
        return None;
    }
    let channel = evidence.signal_channel?;
    Some(SPEED_OF_LIGHT_MPS / glonass_l1_carrier_hz(channel).value())
}
