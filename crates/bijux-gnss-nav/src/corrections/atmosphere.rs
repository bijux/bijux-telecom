#![allow(missing_docs)]
#![allow(dead_code)]

#[derive(Debug, Clone, Copy)]
pub enum IonoMode {
    Broadcast,
    EstimatePerSat,
    IonoFree,
}

#[derive(Debug, Clone)]
pub struct AtmosphereConfig {
    pub enable_ztd: bool,
    pub ztd_initial_m: f64,
    pub ztd_variance_m2: f64,
    pub ztd_min_m: f64,
    pub ztd_max_m: f64,
    pub iono_mode: IonoMode,
}

impl Default for AtmosphereConfig {
    fn default() -> Self {
        Self {
            enable_ztd: true,
            ztd_initial_m: 2.3,
            ztd_variance_m2: 10.0,
            ztd_min_m: 1.0,
            ztd_max_m: 10.0,
            iono_mode: IonoMode::Broadcast,
        }
    }
}

pub fn clamp_ztd(ztd_m: f64, cfg: &AtmosphereConfig) -> f64 {
    ztd_m.max(cfg.ztd_min_m).min(cfg.ztd_max_m)
}
