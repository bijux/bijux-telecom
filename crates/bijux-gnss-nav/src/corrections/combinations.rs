#![allow(missing_docs)]

use std::collections::BTreeMap;

use bijux_gnss_core::{ObsEpoch, ObsSatellite, SatId, SignalBand};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct CombinationObservation {
    pub epoch_idx: u64,
    pub t_rx_s: f64,
    pub sat: SatId,
    pub band_1: SignalBand,
    pub band_2: SignalBand,
    pub f1_hz: f64,
    pub f2_hz: f64,
    pub if_code_m: Option<f64>,
    pub if_phase_m: Option<f64>,
    pub wide_lane_cycles: Option<f64>,
    pub narrow_lane_cycles: Option<f64>,
    pub melbourne_wubbena_m: Option<f64>,
    pub status: String,
    pub reason: String,
}

#[derive(Debug, Clone, Copy)]
pub enum CombinationStatus {
    Ok,
    MissingFrequency,
    LockInvalid,
    VarianceInvalid,
}

fn status_reason(status: CombinationStatus) -> (String, String) {
    match status {
        CombinationStatus::Ok => ("ok".to_string(), "ok".to_string()),
        CombinationStatus::MissingFrequency => {
            ("invalid".to_string(), "missing_frequency".to_string())
        }
        CombinationStatus::LockInvalid => ("invalid".to_string(), "lock_invalid".to_string()),
        CombinationStatus::VarianceInvalid => {
            ("invalid".to_string(), "variance_invalid".to_string())
        }
    }
}

pub fn combinations_from_obs_epochs(
    epochs: &[ObsEpoch],
    band_1: SignalBand,
    band_2: SignalBand,
) -> Vec<CombinationObservation> {
    let mut out = Vec::new();
    for epoch in epochs {
        let mut by_sat: BTreeMap<SatId, Vec<&ObsSatellite>> = BTreeMap::new();
        for sat in &epoch.sats {
            by_sat.entry(sat.signal_id.sat).or_default().push(sat);
        }
        for (sat_id, sats) in by_sat {
            let s1 = sats.iter().find(|s| s.signal_id.band == band_1);
            let s2 = sats.iter().find(|s| s.signal_id.band == band_2);
            let mut status = CombinationStatus::Ok;
            if s1.is_none() || s2.is_none() {
                status = CombinationStatus::MissingFrequency;
            }
            let f1_hz = s1
                .map(|s| s.metadata.signal.carrier_hz.value())
                .unwrap_or(0.0);
            let f2_hz = s2
                .map(|s| s.metadata.signal.carrier_hz.value())
                .unwrap_or(0.0);
            let (mut if_code_m, mut if_phase_m) = (None, None);
            let (mut wide_lane_cycles, mut narrow_lane_cycles, mut mw_m) = (None, None, None);
            if let (Some(s1), Some(s2)) = (s1, s2) {
                if !s1.lock_flags.code_lock
                    || !s1.lock_flags.carrier_lock
                    || !s2.lock_flags.code_lock
                    || !s2.lock_flags.carrier_lock
                {
                    status = CombinationStatus::LockInvalid;
                } else if !s1.pseudorange_var_m2.is_finite()
                    || !s2.pseudorange_var_m2.is_finite()
                    || !s1.carrier_phase_var_cycles2.is_finite()
                    || !s2.carrier_phase_var_cycles2.is_finite()
                {
                    status = CombinationStatus::VarianceInvalid;
                } else {
                    let f1_2 = f1_hz * f1_hz;
                    let f2_2 = f2_hz * f2_hz;
                    let denom = (f1_2 - f2_2).max(1.0);
                    let p1 = s1.pseudorange_m;
                    let p2 = s2.pseudorange_m;
                    let lambda1 = SPEED_OF_LIGHT_MPS / f1_hz.max(1.0);
                    let lambda2 = SPEED_OF_LIGHT_MPS / f2_hz.max(1.0);
                    let phi1_m = s1.carrier_phase_cycles * lambda1;
                    let phi2_m = s2.carrier_phase_cycles * lambda2;

                    if_code_m = Some((f1_2 * p1 - f2_2 * p2) / denom);
                    if_phase_m = Some((f1_2 * phi1_m - f2_2 * phi2_m) / denom);

                    let lambda_wl = SPEED_OF_LIGHT_MPS / (f1_hz - f2_hz).abs().max(1.0);
                    let lambda_nl = SPEED_OF_LIGHT_MPS / (f1_hz + f2_hz).max(1.0);
                    wide_lane_cycles = Some((phi1_m - phi2_m) / lambda_wl);
                    narrow_lane_cycles = Some((phi1_m + phi2_m) / lambda_nl);
                    mw_m = Some((phi1_m - phi2_m) - (p1 - p2));
                }
            }
            let (status_str, reason) = status_reason(status);
            out.push(CombinationObservation {
                epoch_idx: epoch.epoch_idx,
                t_rx_s: epoch.t_rx_s,
                sat: sat_id,
                band_1,
                band_2,
                f1_hz,
                f2_hz,
                if_code_m,
                if_phase_m,
                wide_lane_cycles,
                narrow_lane_cycles,
                melbourne_wubbena_m: mw_m,
                status: status_str,
                reason,
            });
        }
    }
    out
}
