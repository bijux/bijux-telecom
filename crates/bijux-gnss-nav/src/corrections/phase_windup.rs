#![allow(missing_docs)]

use serde::{Deserialize, Serialize};

use crate::models::antenna::{
    receiver_antenna_frame_ecef, satellite_antenna_frame_ecef, ReceiverAntennaFrame,
    SatelliteAntennaFrame,
};

const FULL_TURN_RAD: f64 = 2.0 * std::f64::consts::PI;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PhaseWindupCorrection {
    pub raw_cycles: f64,
    pub continuous_cycles: f64,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct PhaseWindupState {
    pub previous_cycles: Option<f64>,
}

impl PhaseWindupState {
    pub fn apply_geometry(
        &mut self,
        receiver_pos_m: [f64; 3],
        sat_pos_m: [f64; 3],
        sun_pos_m: [f64; 3],
    ) -> Option<PhaseWindupCorrection> {
        let correction = phase_windup_correction_maybe_continuous(
            receiver_pos_m,
            sat_pos_m,
            sun_pos_m,
            self.previous_cycles,
        )?;
        self.previous_cycles = Some(correction.continuous_cycles);
        Some(correction)
    }

    pub fn reset(&mut self) {
        self.previous_cycles = None;
    }
}

pub fn phase_windup_correction_maybe_continuous(
    receiver_pos_m: [f64; 3],
    sat_pos_m: [f64; 3],
    sun_pos_m: [f64; 3],
    previous_cycles: Option<f64>,
) -> Option<PhaseWindupCorrection> {
    let receiver_frame = receiver_antenna_frame_ecef(receiver_pos_m)?;
    let satellite_frame = satellite_antenna_frame_ecef(sat_pos_m, sun_pos_m)?;
    let line_of_sight_sat_to_receiver = try_normalize3([
        receiver_pos_m[0] - sat_pos_m[0],
        receiver_pos_m[1] - sat_pos_m[1],
        receiver_pos_m[2] - sat_pos_m[2],
    ])?;
    let raw_cycles =
        signed_windup_cycles(receiver_frame, satellite_frame, line_of_sight_sat_to_receiver)?;
    Some(PhaseWindupCorrection {
        raw_cycles,
        continuous_cycles: unwrap_cycles(raw_cycles, previous_cycles),
    })
}

fn signed_windup_cycles(
    receiver_frame: ReceiverAntennaFrame,
    satellite_frame: SatelliteAntennaFrame,
    line_of_sight_sat_to_receiver: [f64; 3],
) -> Option<f64> {
    let receiver_dipole = try_normalize3(effective_dipole(
        receiver_frame.east_axis_ecef,
        receiver_frame.north_axis_ecef,
        line_of_sight_sat_to_receiver,
        1.0,
    ))?;
    let satellite_dipole = try_normalize3(effective_dipole(
        satellite_frame.x_axis_ecef,
        satellite_frame.y_axis_ecef,
        line_of_sight_sat_to_receiver,
        -1.0,
    ))?;
    let sine = dot3(line_of_sight_sat_to_receiver, cross3(satellite_dipole, receiver_dipole));
    let cosine = dot3(satellite_dipole, receiver_dipole).clamp(-1.0, 1.0);
    let cycles = sine.atan2(cosine) / FULL_TURN_RAD;
    cycles.is_finite().then_some(cycles)
}

fn effective_dipole(
    primary_axis: [f64; 3],
    secondary_axis: [f64; 3],
    line_of_sight_sat_to_receiver: [f64; 3],
    secondary_sign: f64,
) -> [f64; 3] {
    let projection =
        scale3(line_of_sight_sat_to_receiver, dot3(line_of_sight_sat_to_receiver, primary_axis));
    add3(
        sub3(primary_axis, projection),
        scale3(cross3(line_of_sight_sat_to_receiver, secondary_axis), secondary_sign),
    )
}

fn unwrap_cycles(raw_cycles: f64, previous_cycles: Option<f64>) -> f64 {
    let Some(previous_cycles) = previous_cycles else {
        return raw_cycles;
    };
    raw_cycles + (previous_cycles - raw_cycles).round()
}

fn add3(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [left[0] + right[0], left[1] + right[1], left[2] + right[2]]
}

fn sub3(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [left[0] - right[0], left[1] - right[1], left[2] - right[2]]
}

fn scale3(vector: [f64; 3], scalar: f64) -> [f64; 3] {
    [vector[0] * scalar, vector[1] * scalar, vector[2] * scalar]
}

fn cross3(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]
}

fn dot3(left: [f64; 3], right: [f64; 3]) -> f64 {
    left[0] * right[0] + left[1] * right[1] + left[2] * right[2]
}

fn norm3(vector: [f64; 3]) -> f64 {
    dot3(vector, vector).sqrt()
}

fn try_normalize3(vector: [f64; 3]) -> Option<[f64; 3]> {
    if !vector.iter().all(|value| value.is_finite()) {
        return None;
    }
    let norm = norm3(vector);
    if !norm.is_finite() || norm <= f64::EPSILON {
        return None;
    }
    Some([vector[0] / norm, vector[1] / norm, vector[2] / norm])
}

#[cfg(test)]
mod tests {
    use super::{
        phase_windup_correction_maybe_continuous, unwrap_cycles, PhaseWindupCorrection,
        PhaseWindupState,
    };

    #[test]
    fn phase_windup_is_zero_for_aligned_receiver_and_satellite_dipoles() {
        let receiver_pos_m = [6_378_137.0, 0.0, 0.0];
        let sat_pos_m = [20_200_000.0, 0.0, 0.0];
        let sun_pos_m = [0.0, 149_597_870_700.0, 0.0];

        let correction =
            phase_windup_correction_maybe_continuous(receiver_pos_m, sat_pos_m, sun_pos_m, None)
                .expect("phase wind-up correction");

        assert_eq!(correction, PhaseWindupCorrection { raw_cycles: 0.0, continuous_cycles: 0.0 });
    }

    #[test]
    fn phase_windup_reports_half_cycle_for_opposing_dipoles() {
        let receiver_pos_m = [6_378_137.0, 0.0, 0.0];
        let sat_pos_m = [20_200_000.0, 0.0, 0.0];
        let sun_pos_m = [0.0, -149_597_870_700.0, 0.0];

        let correction =
            phase_windup_correction_maybe_continuous(receiver_pos_m, sat_pos_m, sun_pos_m, None)
                .expect("phase wind-up correction");

        assert!((correction.raw_cycles.abs() - 0.5).abs() < 1.0e-12);
        assert_eq!(correction.raw_cycles, correction.continuous_cycles);
    }

    #[test]
    fn phase_windup_state_unwraps_cycles_to_nearest_previous_value() {
        assert!((unwrap_cycles(-0.49, Some(0.49)) - 0.51).abs() < 1.0e-12);
        assert!((unwrap_cycles(0.49, Some(-0.49)) + 0.51).abs() < 1.0e-12);
    }

    #[test]
    fn phase_windup_state_retains_continuity_between_epochs() {
        let receiver_pos_m = [6_378_137.0, 0.0, 0.0];
        let sat_pos_m = [20_200_000.0, 0.0, 0.0];
        let mut state = PhaseWindupState { previous_cycles: Some(0.49) };

        let correction = state
            .apply_geometry(receiver_pos_m, sat_pos_m, [0.0, -149_597_870_700.0, 0.0])
            .expect("phase wind-up correction");

        assert!((correction.continuous_cycles - 0.5).abs() < 1.0e-12);
        assert_eq!(state.previous_cycles, Some(correction.continuous_cycles));
    }
}
