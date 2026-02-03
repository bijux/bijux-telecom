use bijux_gnss_core::{AcqError, ConfigError, InputError, NavError, SignalError, TrackError};
use thiserror::Error;

#[derive(Debug, Clone)]
pub struct ReceiverConfig {
    pub sampling_freq_hz: f64,
    pub intermediate_freq_hz: f64,
    pub code_freq_basis_hz: f64,
    pub code_length: usize,
    pub channels: usize,
    pub early_late_spacing_chips: f64,
    pub dll_bw_hz: f64,
    pub pll_bw_hz: f64,
    pub fll_bw_hz: f64,
    pub tracking_integration_ms: u32,
    pub tracking_per_band: Vec<BandTrackingSpec>,
    pub robust_solver: bool,
    pub huber_k: f64,
    pub raim: bool,
    pub hatch_window: u32,
    pub weighting: crate::config::NavigationWeightingProfile,
    pub iono_mode: String,
    pub tropo_enable: bool,
    pub tropo_ztd_m: f64,
    pub ppp: crate::config::PppProfile,
}

impl Default for ReceiverConfig {
    fn default() -> Self {
        Self {
            sampling_freq_hz: 5_000_000.0,
            intermediate_freq_hz: 0.0,
            code_freq_basis_hz: 1_023_000.0,
            code_length: 1023,
            channels: 12,
            early_late_spacing_chips: 0.5,
            dll_bw_hz: 2.0,
            pll_bw_hz: 15.0,
            fll_bw_hz: 10.0,
            tracking_integration_ms: 1,
            tracking_per_band: Vec::new(),
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
            ppp: crate::config::PppProfile::default(),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct TrackingParams {
    pub early_late_spacing_chips: f64,
    pub dll_bw_hz: f64,
    pub pll_bw_hz: f64,
    pub fll_bw_hz: f64,
    pub integration_ms: u32,
}

#[derive(Debug, Clone)]
pub struct BandTrackingSpec {
    pub band: bijux_gnss_core::SignalBand,
    pub early_late_spacing_chips: f64,
    pub dll_bw_hz: f64,
    pub pll_bw_hz: f64,
    pub fll_bw_hz: f64,
    pub integration_ms: u32,
}

impl ReceiverConfig {
    pub fn tracking_params(&self, band: bijux_gnss_core::SignalBand) -> TrackingParams {
        if let Some(profile) = self.tracking_per_band.iter().find(|p| p.band == band) {
            return TrackingParams {
                early_late_spacing_chips: profile.early_late_spacing_chips,
                dll_bw_hz: profile.dll_bw_hz,
                pll_bw_hz: profile.pll_bw_hz,
                fll_bw_hz: profile.fll_bw_hz,
                integration_ms: profile.integration_ms,
            };
        }
        TrackingParams {
            early_late_spacing_chips: self.early_late_spacing_chips,
            dll_bw_hz: self.dll_bw_hz,
            pll_bw_hz: self.pll_bw_hz,
            fll_bw_hz: self.fll_bw_hz,
            integration_ms: self.tracking_integration_ms,
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
