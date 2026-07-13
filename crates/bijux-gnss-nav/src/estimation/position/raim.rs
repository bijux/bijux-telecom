use bijux_gnss_core::api::SatId;

use crate::estimation::uncertainty::{
    covariance_enu_standard_deviations_m, horizontal_error_ellipse,
};

const FORMAL_HORIZONTAL_PROTECTION_SCALE: f64 = 6.0;
const FORMAL_VERTICAL_PROTECTION_SCALE: f64 = 6.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RaimFaultDetectionStatus {
    Consistent,
    FaultDetected,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RaimSolutionSeparationSubset {
    pub excluded_sat: SatId,
    pub separation_m: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct RaimSolutionSeparationCheck {
    pub reference_sat_count: usize,
    pub compared_subsets: Vec<RaimSolutionSeparationSubset>,
}

impl RaimSolutionSeparationCheck {
    pub fn compared_subset_count(&self) -> usize {
        self.compared_subsets.len()
    }

    pub fn max_separation(&self) -> Option<RaimSolutionSeparationSubset> {
        self.compared_subsets.iter().copied().max_by(|left, right| {
            left.separation_m.partial_cmp(&right.separation_m).unwrap_or(std::cmp::Ordering::Equal)
        })
    }
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

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PositionProtectionLevels {
    pub horizontal_m: f64,
    pub vertical_m: f64,
}

pub fn formal_protection_levels(
    receiver_ecef_m: [f64; 3],
    position_covariance_ecef_m2: [[f64; 3]; 3],
) -> Option<PositionProtectionLevels> {
    let horizontal_major_axis_m =
        horizontal_error_ellipse(receiver_ecef_m, position_covariance_ecef_m2)?.major_axis_m;
    let (_, _, sigma_u_m) =
        covariance_enu_standard_deviations_m(receiver_ecef_m, position_covariance_ecef_m2)?;
    Some(PositionProtectionLevels {
        horizontal_m: horizontal_major_axis_m * FORMAL_HORIZONTAL_PROTECTION_SCALE,
        vertical_m: sigma_u_m * FORMAL_VERTICAL_PROTECTION_SCALE,
    })
}

#[cfg(test)]
mod tests {
    use super::{formal_protection_levels, PositionProtectionLevels};

    #[test]
    fn formal_protection_levels_use_horizontal_major_axis_and_vertical_sigma() {
        let receiver_ecef_m = [6_378_137.0, 0.0, 0.0];
        let covariance_ecef_m2 = [[9.0, 0.0, 0.0], [0.0, 4.0, 0.0], [0.0, 0.0, 1.0]];

        let protection_levels = formal_protection_levels(receiver_ecef_m, covariance_ecef_m2)
            .expect("protection levels");

        assert_eq!(
            protection_levels,
            PositionProtectionLevels { horizontal_m: 12.0, vertical_m: 18.0 }
        );
    }

    #[test]
    fn formal_protection_levels_grow_with_geometry_degradation() {
        let receiver_ecef_m = [6_378_137.0, 0.0, 0.0];
        let tighter_covariance_m2 = [[4.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let looser_covariance_m2 = [[16.0, 0.0, 0.0], [0.0, 9.0, 0.0], [0.0, 0.0, 9.0]];

        let tighter = formal_protection_levels(receiver_ecef_m, tighter_covariance_m2)
            .expect("tight protection levels");
        let looser = formal_protection_levels(receiver_ecef_m, looser_covariance_m2)
            .expect("loose protection levels");

        assert!(looser.horizontal_m > tighter.horizontal_m);
        assert!(looser.vertical_m > tighter.vertical_m);
    }

    #[test]
    fn formal_protection_levels_reject_invalid_receiver_position() {
        assert!(formal_protection_levels(
            [0.0, 0.0, 0.0],
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        )
        .is_none());
    }
}
