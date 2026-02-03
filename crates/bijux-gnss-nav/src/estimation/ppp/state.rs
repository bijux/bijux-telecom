#![allow(missing_docs)]

use crate::estimation::ekf::state::Ekf;

pub fn estimate_sigma(ekf: &Ekf, pos_idx: &[usize; 3]) -> (Option<f64>, Option<f64>) {
    if ekf.p.rows() <= pos_idx[2] {
        return (None, None);
    }
    let sigma_x = ekf.p[(pos_idx[0], pos_idx[0])].abs().sqrt();
    let sigma_y = ekf.p[(pos_idx[1], pos_idx[1])].abs().sqrt();
    let sigma_z = ekf.p[(pos_idx[2], pos_idx[2])].abs().sqrt();
    let sigma_h = (sigma_x * sigma_x + sigma_y * sigma_y).sqrt();
    (Some(sigma_h), Some(sigma_z))
}
