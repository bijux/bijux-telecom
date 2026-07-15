use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PositionWeightingModel {
    Elevation,
    Cn0,
    ElevationCn0,
}

impl Default for PositionWeightingModel {
    fn default() -> Self {
        Self::Elevation
    }
}

#[derive(Debug, Clone, Copy)]
pub struct WeightingConfig {
    pub model: PositionWeightingModel,
    pub min_elev_deg: f64,
    pub elev_exponent: f64,
    pub cn0_ref_dbhz: f64,
    pub min_weight: f64,
    pub enabled: bool,
}

impl Default for WeightingConfig {
    fn default() -> Self {
        Self {
            model: PositionWeightingModel::default(),
            min_elev_deg: 5.0,
            elev_exponent: 2.0,
            cn0_ref_dbhz: 50.0,
            min_weight: 0.1,
            enabled: true,
        }
    }
}

pub fn weight_from_elevation(elev_deg: f64, config: WeightingConfig) -> f64 {
    if !config.enabled {
        return 1.0;
    }
    let elev = elev_deg.clamp(0.0, 90.0).max(config.min_elev_deg);
    (elev / 90.0).powf(config.elev_exponent).max(config.min_weight)
}

pub fn weight_from_cn0(cn0_dbhz: f64, config: WeightingConfig) -> f64 {
    if !config.enabled || !cn0_dbhz.is_finite() {
        return 1.0;
    }
    (cn0_dbhz / config.cn0_ref_dbhz).max(config.min_weight)
}

/// Convert a pseudorange standard deviation in meters into a least-squares weight.
///
/// The returned value is the inverse measurement variance in m^-2. Invalid or
/// missing sigma values fall back to unit weighting.
pub fn weight_from_pseudorange_sigma(pseudorange_sigma_m: Option<f64>) -> f64 {
    let Some(sigma_m) = pseudorange_sigma_m else {
        return 1.0;
    };
    if !sigma_m.is_finite() || sigma_m <= 0.0 {
        return 1.0;
    }
    1.0 / sigma_m.powi(2)
}

/// Build a composite code-pseudorange weight from geometry and measurement sigma.
///
/// The geometry term is driven by the configured weighting model when available.
/// The sigma term is always driven by inverse pseudorange variance when available.
pub fn position_measurement_weight(
    cn0_dbhz: Option<f64>,
    elev_deg: Option<f64>,
    pseudorange_sigma_m: Option<f64>,
    config: WeightingConfig,
) -> f64 {
    let geometry_weight = match config.model {
        PositionWeightingModel::Elevation => {
            elev_deg.map(|elev| weight_from_elevation(elev, config)).unwrap_or(1.0)
        }
        PositionWeightingModel::Cn0 => {
            cn0_dbhz.map(|cn0| weight_from_cn0(cn0, config)).unwrap_or(1.0)
        }
        PositionWeightingModel::ElevationCn0 => {
            let elevation_weight =
                elev_deg.map(|elev| weight_from_elevation(elev, config)).unwrap_or(1.0);
            let cn0_weight = cn0_dbhz.map(|cn0| weight_from_cn0(cn0, config)).unwrap_or(1.0);
            elevation_weight * cn0_weight
        }
    };
    geometry_weight * weight_from_pseudorange_sigma(pseudorange_sigma_m)
}
