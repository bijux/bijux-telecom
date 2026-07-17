#![allow(missing_docs)]

use crate::estimation::ekf::state::Ekf;
use crate::estimation::uncertainty::{
    covariance_enu_standard_deviations_m, covariance_horizontal_vertical, horizontal_error_ellipse,
    position_covariance_ecef_m2,
};

pub struct PositionUncertaintyEstimate {
    pub covariance_ecef_m2: Option<[[f64; 3]; 3]>,
    pub sigma_e_m: Option<f64>,
    pub sigma_n_m: Option<f64>,
    pub sigma_u_m: Option<f64>,
    pub horizontal_error_ellipse_major_axis_m: Option<f64>,
    pub horizontal_error_ellipse_minor_axis_m: Option<f64>,
    pub horizontal_error_ellipse_azimuth_deg: Option<f64>,
    pub sigma_h_m: Option<f64>,
    pub sigma_v_m: Option<f64>,
}

pub fn estimate_position_uncertainty(
    ekf: &Ekf,
    pos_idx: &[usize; 3],
    receiver_ecef_m: [f64; 3],
) -> PositionUncertaintyEstimate {
    let covariance = position_covariance_ecef_m2(&ekf.p, pos_idx);
    let (sigma_e_m, sigma_n_m, sigma_u_m) = covariance
        .and_then(|covariance| covariance_enu_standard_deviations_m(receiver_ecef_m, covariance))
        .map(|(sigma_e_m, sigma_n_m, sigma_u_m)| {
            (Some(sigma_e_m), Some(sigma_n_m), Some(sigma_u_m))
        })
        .unwrap_or((None, None, None));
    let (
        horizontal_error_ellipse_major_axis_m,
        horizontal_error_ellipse_minor_axis_m,
        horizontal_error_ellipse_azimuth_deg,
    ) = covariance
        .and_then(|covariance| horizontal_error_ellipse(receiver_ecef_m, covariance))
        .map(|ellipse| {
            (Some(ellipse.major_axis_m), Some(ellipse.minor_axis_m), Some(ellipse.azimuth_deg))
        })
        .unwrap_or((None, None, None));
    let (sigma_h, sigma_v) = covariance
        .and_then(|covariance| covariance_horizontal_vertical(receiver_ecef_m, covariance))
        .map(|(sigma_h, sigma_v)| (Some(sigma_h), Some(sigma_v)))
        .unwrap_or((None, None));
    PositionUncertaintyEstimate {
        covariance_ecef_m2: covariance,
        sigma_e_m,
        sigma_n_m,
        sigma_u_m,
        horizontal_error_ellipse_major_axis_m,
        horizontal_error_ellipse_minor_axis_m,
        horizontal_error_ellipse_azimuth_deg,
        sigma_h_m: sigma_h,
        sigma_v_m: sigma_v,
    }
}
