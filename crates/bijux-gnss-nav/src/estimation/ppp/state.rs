#![allow(missing_docs)]

use crate::estimation::ekf::state::Ekf;
use crate::estimation::uncertainty::{
    covariance_enu_standard_deviations_m, covariance_horizontal_vertical,
    position_covariance_ecef_m2,
};

pub fn estimate_position_uncertainty(
    ekf: &Ekf,
    pos_idx: &[usize; 3],
    receiver_ecef_m: [f64; 3],
) -> (Option<[[f64; 3]; 3]>, Option<f64>, Option<f64>, Option<f64>, Option<f64>, Option<f64>) {
    let covariance = position_covariance_ecef_m2(&ekf.p, pos_idx);
    let (sigma_e_m, sigma_n_m, sigma_u_m) = covariance
        .and_then(|covariance| covariance_enu_standard_deviations_m(receiver_ecef_m, covariance))
        .map(|(sigma_e_m, sigma_n_m, sigma_u_m)| {
            (Some(sigma_e_m), Some(sigma_n_m), Some(sigma_u_m))
        })
        .unwrap_or((None, None, None));
    let (sigma_h, sigma_v) = covariance
        .and_then(|covariance| covariance_horizontal_vertical(receiver_ecef_m, covariance))
        .map(|(sigma_h, sigma_v)| (Some(sigma_h), Some(sigma_v)))
        .unwrap_or((None, None));
    (covariance, sigma_e_m, sigma_n_m, sigma_u_m, sigma_h, sigma_v)
}
