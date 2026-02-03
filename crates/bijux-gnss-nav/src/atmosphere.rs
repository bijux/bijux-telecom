#[derive(Debug, Clone)]
pub enum IonoMode {
    None,
    BroadcastKlobuchar,
    EstimatePerSat,
}

#[derive(Debug, Clone)]
pub struct TropoModel {
    pub enabled: bool,
    pub ztd_m: f64,
}

impl TropoModel {
    pub fn slant_delay_m(&self, elevation_deg: f64) -> f64 {
        if !self.enabled {
            return 0.0;
        }
        let elev = elevation_deg.to_radians().sin().max(0.1);
        self.ztd_m / elev
    }
}

pub fn iono_delay_m(mode: &IonoMode) -> f64 {
    match mode {
        IonoMode::None => 0.0,
        IonoMode::BroadcastKlobuchar => 0.0,
        IonoMode::EstimatePerSat => 0.0,
    }
}
