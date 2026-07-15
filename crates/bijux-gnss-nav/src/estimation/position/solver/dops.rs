use crate::estimation::uncertainty::covariance_horizontal_vertical;

use super::least_squares::{covariance_from_upper_triangular, decompose_weighted_design};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PositionDops {
    pub pdop: f64,
    pub hdop: f64,
    pub vdop: f64,
    pub gdop: f64,
    pub tdop: f64,
}

pub fn position_dops_from_satellite_positions(
    receiver_ecef_m: [f64; 3],
    satellite_positions_ecef_m: &[[f64; 3]],
) -> Option<PositionDops> {
    if satellite_positions_ecef_m.len() < 4 {
        return None;
    }
    let mut h = Vec::with_capacity(satellite_positions_ecef_m.len());
    for satellite_ecef_m in satellite_positions_ecef_m {
        let dx = receiver_ecef_m[0] - satellite_ecef_m[0];
        let dy = receiver_ecef_m[1] - satellite_ecef_m[1];
        let dz = receiver_ecef_m[2] - satellite_ecef_m[2];
        let range_m = (dx * dx + dy * dy + dz * dz).sqrt();
        if !range_m.is_finite() || range_m <= 0.0 {
            return None;
        }
        h.push(vec![dx / range_m, dy / range_m, dz / range_m, 1.0]);
    }
    compute_dops(receiver_ecef_m, &h)
}

pub(super) fn compute_dops(receiver_ecef_m: [f64; 3], h: &[Vec<f64>]) -> Option<PositionDops> {
    let unit_weights = vec![1.0; h.len()];
    let decomposition = decompose_weighted_design(h, &unit_weights)?;
    if decomposition.rank < h.first()?.len() {
        return None;
    }
    let covariance = covariance_from_upper_triangular(&decomposition.r, &decomposition.pivots)?;
    let pdop = position_covariance_trace(&covariance).sqrt();
    let tdop = covariance[3][3].max(0.0).sqrt();
    let gdop = (pdop.powi(2) + tdop.powi(2)).sqrt();
    let (hdop, vdop) = local_horizontal_vertical_dops(receiver_ecef_m, &covariance)?;
    Some(PositionDops { pdop, hdop, vdop, gdop, tdop })
}

fn position_covariance_trace(inv: &[Vec<f64>]) -> f64 {
    (inv[0][0] + inv[1][1] + inv[2][2]).max(0.0)
}

pub(super) fn scaled_position_covariance_ecef_m2(
    covariance: &[Vec<f64>],
    sigma2: f64,
) -> Option<[[f64; 3]; 3]> {
    if covariance.len() < 3 || covariance.iter().take(3).any(|row| row.len() < 3) {
        return None;
    }
    let mut position_covariance = [[0.0_f64; 3]; 3];
    for row in 0..3 {
        for col in 0..3 {
            let value = covariance[row][col] * sigma2;
            if !value.is_finite() {
                return None;
            }
            position_covariance[row][col] = value;
        }
    }
    Some(position_covariance)
}

fn local_horizontal_vertical_dops(
    receiver_ecef_m: [f64; 3],
    inv: &[Vec<f64>],
) -> Option<(f64, f64)> {
    let covariance_xyz = [
        [inv[0][0], inv[0][1], inv[0][2]],
        [inv[1][0], inv[1][1], inv[1][2]],
        [inv[2][0], inv[2][1], inv[2][2]],
    ];
    covariance_horizontal_vertical(receiver_ecef_m, covariance_xyz)
}
