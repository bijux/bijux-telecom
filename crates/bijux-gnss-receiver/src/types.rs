use bijux_gnss_core::{AcqError, ConfigError, InputError, NavError, SignalError, TrackError};
use thiserror::Error;

#[derive(Debug, Clone)]
pub struct ReceiverConfig {
    pub sampling_freq_hz: f64,
    pub intermediate_freq_hz: f64,
    pub code_freq_basis_hz: f64,
    pub code_length: usize,
    pub channels: usize,
    pub dll_bw_hz: f64,
    pub pll_bw_hz: f64,
    pub fll_bw_hz: f64,
    pub robust_solver: bool,
    pub huber_k: f64,
    pub raim: bool,
    pub hatch_window: u32,
    pub weighting: crate::config::NavigationWeightingProfile,
    pub iono_mode: String,
    pub tropo_enable: bool,
    pub tropo_ztd_m: f64,
}

impl Default for ReceiverConfig {
    fn default() -> Self {
        Self {
            sampling_freq_hz: 5_000_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            channels: 12,
            dll_bw_hz: 2.0,
            pll_bw_hz: 15.0,
            fll_bw_hz: 10.0,
            robust_solver: true,
            huber_k: 30.0,
            raim: true,
            hatch_window: 100,
            weighting: crate::config::NavigationWeightingProfile {
                enabled: true,
                min_elev_deg: 5.0,
                elev_exponent: 2.0,
                cn0_ref_dbhz: 50.0,
                min_weight: 0.1,
                elev_mask_deg: 5.0,
                tracking_mode_scalar_weight: 1.0,
                tracking_mode_vector_weight: 1.2,
            },
            iono_mode: "broadcast".to_string(),
            tropo_enable: true,
            tropo_ztd_m: 2.3,
        }
    }
}

#[derive(Debug, Error)]
pub enum ReceiverError {
    #[error(transparent)]
    Input(#[from] InputError),
    #[error(transparent)]
    Config(#[from] ConfigError),
    #[error(transparent)]
    Signal(#[from] SignalError),
    #[error(transparent)]
    Acquisition(#[from] AcqError),
    #[error(transparent)]
    Tracking(#[from] TrackError),
    #[error(transparent)]
    Navigation(#[from] NavError),
}
