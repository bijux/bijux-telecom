#![allow(missing_docs)]

use bijux_gnss_core::api::SatId;
use bijux_gnss_nav::api::{RaimFaultDetection, RaimFaultDetectionStatus, RaimFaultExclusion};

const MIN_PERSISTENT_SUSPECT_EPOCHS: usize = 2;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SatelliteClockAnomaly {
    pub sat: SatId,
    pub persistent_suspect_epochs: usize,
    pub max_solution_separation_m: f64,
    pub separation_threshold_m: f64,
}

pub fn advance_satellite_clock_suspect_streak(
    previous_suspect: Option<SatId>,
    previous_streak: usize,
    current_fault_detection: Option<RaimFaultDetection>,
    current_fault_exclusion: Option<RaimFaultExclusion>,
) -> (Option<SatId>, usize) {
    if current_fault_exclusion.is_some() {
        return (None, 0);
    }
    let suspect_sat = current_fault_detection.and_then(unresolved_fault_suspect_sat);
    match suspect_sat {
        Some(sat) if previous_suspect == Some(sat) => (Some(sat), previous_streak + 1),
        Some(sat) => (Some(sat), 1),
        None => (None, 0),
    }
}

pub fn detect_satellite_clock_anomaly(
    current_fault_detection: Option<RaimFaultDetection>,
    current_fault_exclusion: Option<RaimFaultExclusion>,
    persistent_suspect_epochs: usize,
) -> Option<SatelliteClockAnomaly> {
    if current_fault_exclusion.is_some()
        || persistent_suspect_epochs < MIN_PERSISTENT_SUSPECT_EPOCHS
    {
        return None;
    }
    let fault_detection = current_fault_detection?;
    let sat = unresolved_fault_suspect_sat(fault_detection)?;
    Some(SatelliteClockAnomaly {
        sat,
        persistent_suspect_epochs,
        max_solution_separation_m: fault_detection.max_solution_separation_m,
        separation_threshold_m: fault_detection.threshold_m,
    })
}

fn unresolved_fault_suspect_sat(fault_detection: RaimFaultDetection) -> Option<SatId> {
    (fault_detection.status == RaimFaultDetectionStatus::FaultDetected)
        .then_some(fault_detection.suspect_sat)
        .flatten()
}

#[cfg(test)]
mod tests {
    use bijux_gnss_core::api::{Constellation, SatId};
    use bijux_gnss_nav::api::{RaimFaultDetection, RaimFaultDetectionStatus, RaimFaultExclusion};

    use super::*;

    #[test]
    fn repeated_unresolved_suspect_classifies_clock_anomaly() {
        let sat = gps_sat(29);
        let fault_detection = RaimFaultDetection {
            status: RaimFaultDetectionStatus::FaultDetected,
            suspect_sat: Some(sat),
            max_solution_separation_m: 83.0,
            threshold_m: 50.0,
        };

        let anomaly = detect_satellite_clock_anomaly(Some(fault_detection), None, 2)
            .expect("repeated unresolved suspect should classify");

        assert_eq!(anomaly.sat, sat);
        assert_eq!(anomaly.persistent_suspect_epochs, 2);
        assert_eq!(anomaly.max_solution_separation_m, 83.0);
        assert_eq!(anomaly.separation_threshold_m, 50.0);
    }

    #[test]
    fn single_unresolved_suspect_does_not_classify_clock_anomaly() {
        let sat = gps_sat(29);
        let fault_detection = RaimFaultDetection {
            status: RaimFaultDetectionStatus::FaultDetected,
            suspect_sat: Some(sat),
            max_solution_separation_m: 83.0,
            threshold_m: 50.0,
        };

        assert!(detect_satellite_clock_anomaly(Some(fault_detection), None, 1).is_none());
    }

    #[test]
    fn excluded_raim_fault_does_not_classify_clock_anomaly() {
        let sat = gps_sat(29);
        let fault_detection = RaimFaultDetection {
            status: RaimFaultDetectionStatus::FaultDetected,
            suspect_sat: Some(sat),
            max_solution_separation_m: 83.0,
            threshold_m: 50.0,
        };
        let exclusion = RaimFaultExclusion {
            excluded_sat: sat,
            pre_exclusion_rms_m: 20.0,
            post_exclusion_rms_m: 1.5,
            solution_shift_m: 5.0,
        };

        assert!(detect_satellite_clock_anomaly(Some(fault_detection), Some(exclusion), 4).is_none());
    }

    #[test]
    fn suspect_streak_tracks_repeated_unresolved_faults() {
        let sat = gps_sat(29);
        let fault_detection = RaimFaultDetection {
            status: RaimFaultDetectionStatus::FaultDetected,
            suspect_sat: Some(sat),
            max_solution_separation_m: 83.0,
            threshold_m: 50.0,
        };

        let (suspect, streak) =
            advance_satellite_clock_suspect_streak(None, 0, Some(fault_detection), None);
        assert_eq!(suspect, Some(sat));
        assert_eq!(streak, 1);

        let (suspect, streak) =
            advance_satellite_clock_suspect_streak(suspect, streak, Some(fault_detection), None);
        assert_eq!(suspect, Some(sat));
        assert_eq!(streak, 2);
    }

    #[test]
    fn suspect_streak_resets_after_exclusion() {
        let sat = gps_sat(29);
        let fault_detection = RaimFaultDetection {
            status: RaimFaultDetectionStatus::FaultDetected,
            suspect_sat: Some(sat),
            max_solution_separation_m: 83.0,
            threshold_m: 50.0,
        };
        let exclusion = RaimFaultExclusion {
            excluded_sat: sat,
            pre_exclusion_rms_m: 20.0,
            post_exclusion_rms_m: 1.5,
            solution_shift_m: 5.0,
        };

        let (suspect, streak) = advance_satellite_clock_suspect_streak(
            Some(sat),
            3,
            Some(fault_detection),
            Some(exclusion),
        );

        assert_eq!(suspect, None);
        assert_eq!(streak, 0);
    }

    fn gps_sat(prn: u8) -> SatId {
        SatId { constellation: Constellation::Gps, prn }
    }
}
