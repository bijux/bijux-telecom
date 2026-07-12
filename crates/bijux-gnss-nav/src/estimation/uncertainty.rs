#![allow(missing_docs)]

use crate::estimation::position::solver::ecef_to_geodetic;
use crate::linalg::Matrix;

pub(crate) fn position_covariance_ecef_m2(
    covariance: &Matrix,
    position_indices: &[usize; 3],
) -> Option<[[f64; 3]; 3]> {
    let max_index = *position_indices.iter().max()?;
    if covariance.rows() <= max_index || covariance.cols() <= max_index {
        return None;
    }

    let mut position_covariance = [[0.0_f64; 3]; 3];
    for (row_index, &state_row) in position_indices.iter().enumerate() {
        for (col_index, &state_col) in position_indices.iter().enumerate() {
            position_covariance[row_index][col_index] = covariance[(state_row, state_col)];
        }
    }
    Some(position_covariance)
}

pub(crate) fn covariance_horizontal_vertical(
    receiver_ecef_m: [f64; 3],
    covariance_ecef_m2: [[f64; 3]; 3],
) -> Option<(f64, f64)> {
    let (sigma_e_m, sigma_n_m, sigma_u_m) =
        covariance_enu_standard_deviations_m(receiver_ecef_m, covariance_ecef_m2)?;
    let horizontal = (sigma_e_m * sigma_e_m + sigma_n_m * sigma_n_m).sqrt();
    Some((horizontal, sigma_u_m))
}

pub(crate) fn covariance_enu_standard_deviations_m(
    receiver_ecef_m: [f64; 3],
    covariance_ecef_m2: [[f64; 3]; 3],
) -> Option<(f64, f64, f64)> {
    let covariance_enu = covariance_ecef_to_enu(receiver_ecef_m, covariance_ecef_m2)?;
    let sigma_e_m = covariance_enu[0][0].max(0.0).sqrt();
    let sigma_n_m = covariance_enu[1][1].max(0.0).sqrt();
    let sigma_u_m = covariance_enu[2][2].max(0.0).sqrt();
    Some((sigma_e_m, sigma_n_m, sigma_u_m))
}

pub(crate) fn covariance_ecef_to_enu(
    receiver_ecef_m: [f64; 3],
    covariance_ecef_m2: [[f64; 3]; 3],
) -> Option<[[f64; 3]; 3]> {
    let receiver_radius_m =
        receiver_ecef_m.iter().map(|component| component * component).sum::<f64>().sqrt();
    if !receiver_radius_m.is_finite() || receiver_radius_m <= 0.0 {
        return None;
    }

    let (lat_deg, lon_deg, _alt_m) =
        ecef_to_geodetic(receiver_ecef_m[0], receiver_ecef_m[1], receiver_ecef_m[2]);
    if !lat_deg.is_finite() || !lon_deg.is_finite() {
        return None;
    }

    let rotation = ecef_to_enu_rotation(lat_deg.to_radians(), lon_deg.to_radians());
    let mut covariance_enu = [[0.0_f64; 3]; 3];
    for row in 0..3 {
        for col in 0..3 {
            for inner_row in 0..3 {
                for inner_col in 0..3 {
                    covariance_enu[row][col] += rotation[row][inner_row]
                        * covariance_ecef_m2[inner_row][inner_col]
                        * rotation[col][inner_col];
                }
            }
        }
    }
    Some(covariance_enu)
}

fn ecef_to_enu_rotation(lat_rad: f64, lon_rad: f64) -> [[f64; 3]; 3] {
    let sin_lat = lat_rad.sin();
    let cos_lat = lat_rad.cos();
    let sin_lon = lon_rad.sin();
    let cos_lon = lon_rad.cos();

    [
        [-sin_lon, cos_lon, 0.0],
        [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
        [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
    ]
}

#[cfg(test)]
mod tests {
    use super::{covariance_ecef_to_enu, covariance_enu_standard_deviations_m};

    #[test]
    fn covariance_conversion_reports_enu_standard_deviations() {
        let receiver_ecef_m = [6_378_137.0, 0.0, 0.0];
        let covariance_ecef_m2 = [[9.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 4.0]];

        let covariance_enu =
            covariance_ecef_to_enu(receiver_ecef_m, covariance_ecef_m2).expect("ENU covariance");
        let (sigma_e_m, sigma_n_m, sigma_u_m) =
            covariance_enu_standard_deviations_m(receiver_ecef_m, covariance_ecef_m2)
                .expect("ENU standard deviations");

        assert_eq!(covariance_enu[0][0], 1.0);
        assert_eq!(covariance_enu[1][1], 4.0);
        assert_eq!(covariance_enu[2][2], 9.0);
        assert_eq!(sigma_e_m, 1.0);
        assert_eq!(sigma_n_m, 2.0);
        assert_eq!(sigma_u_m, 3.0);
    }
}
