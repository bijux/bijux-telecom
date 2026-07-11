use bijux_gnss_core::api::SatId;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RaimFaultDetectionStatus {
    Consistent,
    FaultDetected,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RaimFaultDetection {
    pub status: RaimFaultDetectionStatus,
    pub suspect_sat: Option<SatId>,
    pub max_solution_separation_m: f64,
    pub threshold_m: f64,
}

impl RaimFaultDetection {
    pub fn consistent(max_solution_separation_m: f64, threshold_m: f64) -> Self {
        Self {
            status: RaimFaultDetectionStatus::Consistent,
            suspect_sat: None,
            max_solution_separation_m,
            threshold_m,
        }
    }

    pub fn fault_detected(
        suspect_sat: SatId,
        max_solution_separation_m: f64,
        threshold_m: f64,
    ) -> Self {
        Self {
            status: RaimFaultDetectionStatus::FaultDetected,
            suspect_sat: Some(suspect_sat),
            max_solution_separation_m,
            threshold_m,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RaimFaultExclusion {
    pub excluded_sat: SatId,
    pub pre_exclusion_rms_m: f64,
    pub post_exclusion_rms_m: f64,
    pub solution_shift_m: f64,
}

impl RaimFaultExclusion {
    pub fn improved(self) -> bool {
        self.post_exclusion_rms_m < self.pre_exclusion_rms_m
    }
}
