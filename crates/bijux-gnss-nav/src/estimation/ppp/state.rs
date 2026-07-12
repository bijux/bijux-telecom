#![allow(missing_docs)]

use crate::estimation::ekf::state::Ekf;
use crate::estimation::uncertainty::{covariance_horizontal_vertical, position_covariance_ecef_m2};

pub fn estimate_position_uncertainty(
    ekf: &Ekf,
    pos_idx: &[usize; 3],
    receiver_ecef_m: [f64; 3],
) -> (Option<[[f64; 3]; 3]>, Option<f64>, Option<f64>) {
    let covariance = position_covariance_ecef_m2(&ekf.p, pos_idx);
    let (sigma_h, sigma_v) = covariance
        .and_then(|covariance| covariance_horizontal_vertical(receiver_ecef_m, covariance))
        .map(|(sigma_h, sigma_v)| (Some(sigma_h), Some(sigma_v)))
        .unwrap_or((None, None));
    (covariance, sigma_h, sigma_v)
}
