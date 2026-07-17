//! Signal-oriented fixture generation and acquisition reference helpers.

pub mod acquisition;
pub mod synthesis;

pub use acquisition::{
    expected_acquisition_code_phase_samples_f64,
    gps_l1ca_expected_acquisition_code_phase_samples,
    gps_l1ca_expected_acquisition_code_phase_samples_f64, samples_per_code,
    wrapped_code_phase_error_samples_f64,
};
pub use synthesis::{
    generate_clipped_iq16_le_bytes, generate_clipped_iq8_bytes, generate_quadrature_skew_carrier,
};
