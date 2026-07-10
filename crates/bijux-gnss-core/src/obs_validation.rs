#![allow(missing_docs)]
#![allow(dead_code)]

use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};

use crate::api::{ObsEpoch, SatId, Seconds, SignalBand};

/// Event describing missing band observations over time.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BandLagEvent {
    pub sat: SatId,
    pub band: SignalBand,
    pub lag_epochs: u64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InterFrequencyAlignmentReport {
    pub total_events: usize,
    pub max_lag_epochs: u64,
    pub events: Vec<BandLagEvent>,
}

pub fn check_inter_frequency_alignment(epochs: &[ObsEpoch]) -> InterFrequencyAlignmentReport {
    let mut last_seen: BTreeMap<(SatId, SignalBand), u64> = BTreeMap::new();
    let mut events = Vec::new();
    for epoch in epochs {
        let mut present: BTreeMap<SatId, Vec<SignalBand>> = BTreeMap::new();
        for sat in &epoch.sats {
            present.entry(sat.signal_id.sat).or_default().push(sat.signal_id.band);
            last_seen.entry((sat.signal_id.sat, sat.signal_id.band)).or_insert(epoch.epoch_idx);
        }
        for (sat, bands) in present {
            for ((seen_sat, seen_band), last_epoch) in &last_seen {
                if *seen_sat != sat {
                    continue;
                }
                if bands.contains(seen_band) {
                    continue;
                }
                let lag = epoch.epoch_idx.saturating_sub(*last_epoch);
                if lag > 0 {
                    events.push(BandLagEvent { sat, band: *seen_band, lag_epochs: lag });
                }
            }
        }
        for sat in &epoch.sats {
            last_seen.insert((sat.signal_id.sat, sat.signal_id.band), epoch.epoch_idx);
        }
    }
    let max_lag = events.iter().map(|event| event.lag_epochs).max().unwrap_or(0);
    InterFrequencyAlignmentReport { total_events: events.len(), max_lag_epochs: max_lag, events }
}

pub fn validate_obs_epochs(epochs: &[ObsEpoch]) -> Result<(), String> {
    let mut last_t: Option<Seconds> = None;
    for epoch in epochs {
        if let Some(prev) = last_t {
            if epoch.t_rx_s.0 < prev.0 {
                return Err("non-monotonic t_rx_s".to_string());
            }
        }
        last_t = Some(epoch.t_rx_s);
        if !epoch.t_rx_s.0.is_finite() {
            return Err("t_rx_s is not finite".to_string());
        }
        let mut seen = BTreeSet::new();
        for sat in &epoch.sats {
            if !seen.insert(sat.signal_id) {
                return Err("duplicate signal_id within epoch".to_string());
            }
            if !sat.pseudorange_m.0.is_finite()
                || !sat.carrier_phase_cycles.0.is_finite()
                || !sat.doppler_hz.0.is_finite()
            {
                return Err("non-finite observable value".to_string());
            }
            if !sat.pseudorange_var_m2.is_finite()
                || !sat.carrier_phase_var_cycles2.is_finite()
                || !sat.doppler_var_hz2.is_finite()
            {
                return Err("non-finite variance".to_string());
            }
            if sat.pseudorange_var_m2 < 0.0
                || sat.carrier_phase_var_cycles2 < 0.0
                || sat.doppler_var_hz2 < 0.0
            {
                return Err("negative variance".to_string());
            }
        }
    }
    Ok(())
}
