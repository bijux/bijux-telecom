use crate::api::{PositionFilterMotionClass, PositionWeightingModel};
use bijux_gnss_core::api::Constellation;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PositionConstellationPolicy {
    GpsOnly,
    GalileoOnly,
    GlonassOnly,
    BeidouOnly,
    #[default]
    Mixed,
}

impl PositionConstellationPolicy {
    pub fn allows(self, constellation: Constellation) -> bool {
        self.selected_constellations().contains(&constellation)
    }

    pub fn selected_constellations(self) -> &'static [Constellation] {
        match self {
            Self::GpsOnly => &[Constellation::Gps],
            Self::GalileoOnly => &[Constellation::Galileo],
            Self::GlonassOnly => &[Constellation::Glonass],
            Self::BeidouOnly => &[Constellation::Beidou],
            Self::Mixed => &[
                Constellation::Gps,
                Constellation::Galileo,
                Constellation::Glonass,
                Constellation::Beidou,
            ],
        }
    }
}

#[derive(Debug, Clone)]
pub struct PositionRuntimeThresholds {
    pub min_mean_cn0_dbhz: f64,
    pub max_pdop: f64,
    pub max_gdop: f64,
    pub max_residual_rms_m: f64,
    pub min_used_satellites: usize,
    pub min_lock_ratio: f64,
}

impl Default for PositionRuntimeThresholds {
    fn default() -> Self {
        Self {
            min_mean_cn0_dbhz: 28.0,
            max_pdop: 8.0,
            max_gdop: 12.0,
            max_residual_rms_m: 25.0,
            min_used_satellites: 4,
            min_lock_ratio: 0.7,
        }
    }
}

#[derive(Debug, Clone)]
pub struct PositionRuntimeWeightingConfig {
    pub enabled: bool,
    pub mode: PositionWeightingModel,
    pub min_elev_deg: f64,
    pub elev_exponent: f64,
    pub cn0_ref_dbhz: f64,
    pub min_weight: f64,
    pub elev_mask_deg: f64,
    pub tracking_mode_scalar_weight: f64,
    pub tracking_mode_vector_weight: f64,
}

impl Default for PositionRuntimeWeightingConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            mode: PositionWeightingModel::Elevation,
            min_elev_deg: 5.0,
            elev_exponent: 2.0,
            cn0_ref_dbhz: 50.0,
            min_weight: 0.1,
            elev_mask_deg: 5.0,
            tracking_mode_scalar_weight: 1.0,
            tracking_mode_vector_weight: 1.2,
        }
    }
}

#[derive(Debug, Clone)]
pub struct PositionRuntimeConfig {
    pub robust_solver: bool,
    pub huber_k: f64,
    pub raim: bool,
    pub position_solution_smoothing: bool,
    pub position_solution_motion_class: PositionFilterMotionClass,
    pub weighting: PositionRuntimeWeightingConfig,
    pub tropo_enable: bool,
    pub science_thresholds: PositionRuntimeThresholds,
    pub constellation_policy: PositionConstellationPolicy,
}

impl PositionRuntimeConfig {
    pub fn allows_constellation(&self, constellation: Constellation) -> bool {
        self.constellation_policy.allows(constellation)
    }

    pub fn selected_constellations(&self) -> &'static [Constellation] {
        self.constellation_policy.selected_constellations()
    }
}

impl Default for PositionRuntimeConfig {
    fn default() -> Self {
        Self {
            robust_solver: true,
            huber_k: 30.0,
            raim: true,
            position_solution_smoothing: true,
            position_solution_motion_class: PositionFilterMotionClass::Vehicle,
            weighting: PositionRuntimeWeightingConfig::default(),
            tropo_enable: true,
            science_thresholds: PositionRuntimeThresholds::default(),
            constellation_policy: PositionConstellationPolicy::Mixed,
        }
    }
}
