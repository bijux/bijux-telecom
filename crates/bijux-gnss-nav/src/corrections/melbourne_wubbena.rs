#![allow(missing_docs)]

use std::collections::BTreeMap;

use bijux_gnss_core::api::{ObsEpoch, SatId, SignalBand};
use serde::{Deserialize, Serialize};

use crate::corrections::combinations::combinations_from_obs_epochs;

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct MelbourneWubbenaThresholds {
    pub wide_lane_slip_jump_cycles: f64,
}

impl Default for MelbourneWubbenaThresholds {
    fn default() -> Self {
        Self { wide_lane_slip_jump_cycles: 0.5 }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum MelbourneWubbenaEvent {
    Unavailable,
    InsufficientHistory,
    Nominal,
    WideLaneSlipSuspect,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MelbourneWubbenaObservation {
    pub epoch_idx: u64,
    pub t_rx_s: f64,
    pub sat: SatId,
    pub band_1: SignalBand,
    pub band_2: SignalBand,
    pub melbourne_wubbena_m: Option<f64>,
    pub delta_from_previous_m: Option<f64>,
    pub delta_from_previous_wide_lane_cycles: Option<f64>,
    pub status: String,
    pub reason: String,
    pub event: MelbourneWubbenaEvent,
}

pub fn melbourne_wubbena_diagnostics_from_obs_epochs(
    epochs: &[ObsEpoch],
    band_1: SignalBand,
    band_2: SignalBand,
    thresholds: MelbourneWubbenaThresholds,
) -> Vec<MelbourneWubbenaObservation> {
    let mut diagnostics = Vec::new();
    let mut previous_by_sat: BTreeMap<SatId, f64> = BTreeMap::new();

    for combination in combinations_from_obs_epochs(epochs, band_1, band_2) {
        let mut delta_from_previous_m = None;
        let mut delta_from_previous_wide_lane_cycles = None;
        let wide_lane_wavelength_m = wide_lane_wavelength_m(combination.f1_hz, combination.f2_hz);
        let event = if combination.status != "ok" {
            previous_by_sat.remove(&combination.sat);
            MelbourneWubbenaEvent::Unavailable
        } else if let Some(melbourne_wubbena_m) = combination.melbourne_wubbena_m {
            let event = if let Some(previous) = previous_by_sat.get(&combination.sat) {
                let delta_m = melbourne_wubbena_m - previous;
                delta_from_previous_m = Some(delta_m);
                delta_from_previous_wide_lane_cycles = Some(delta_m / wide_lane_wavelength_m);
                classify_melbourne_wubbena_event(
                    delta_from_previous_wide_lane_cycles.expect("wide-lane delta"),
                    thresholds,
                )
            } else {
                MelbourneWubbenaEvent::InsufficientHistory
            };
            previous_by_sat.insert(combination.sat, melbourne_wubbena_m);
            event
        } else {
            previous_by_sat.remove(&combination.sat);
            MelbourneWubbenaEvent::Unavailable
        };

        diagnostics.push(MelbourneWubbenaObservation {
            epoch_idx: combination.epoch_idx,
            t_rx_s: combination.t_rx_s,
            sat: combination.sat,
            band_1: combination.band_1,
            band_2: combination.band_2,
            melbourne_wubbena_m: combination.melbourne_wubbena_m,
            delta_from_previous_m,
            delta_from_previous_wide_lane_cycles,
            status: combination.status,
            reason: combination.reason,
            event,
        });
    }

    diagnostics
}

fn classify_melbourne_wubbena_event(
    delta_wide_lane_cycles: f64,
    thresholds: MelbourneWubbenaThresholds,
) -> MelbourneWubbenaEvent {
    if delta_wide_lane_cycles.abs() >= thresholds.wide_lane_slip_jump_cycles {
        MelbourneWubbenaEvent::WideLaneSlipSuspect
    } else {
        MelbourneWubbenaEvent::Nominal
    }
}

fn wide_lane_wavelength_m(f1_hz: f64, f2_hz: f64) -> f64 {
    SPEED_OF_LIGHT_MPS / (f1_hz - f2_hz).abs().max(1.0)
}

#[cfg(test)]
mod tests {
    use super::{
        melbourne_wubbena_diagnostics_from_obs_epochs, MelbourneWubbenaEvent,
        MelbourneWubbenaThresholds,
    };
    use bijux_gnss_core::api::{
        signal_spec_gps_l1_ca, signal_spec_gps_l2_py, Constellation, Cycles, Hertz, LockFlags,
        Meters, ObsEpoch, ObsMetadata, ObsSatellite, ObservationEpochDecision, ObservationStatus,
        ReceiverRole, ReceiverSampleTrace, SatId, Seconds, SigId, SignalBand, SignalCode,
    };

    const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

    fn dual_frequency_epoch(
        epoch_idx: u64,
        p1_m: f64,
        p2_m: f64,
        phi1_m: f64,
        phi2_m: f64,
    ) -> ObsEpoch {
        let sat = SatId { constellation: Constellation::Gps, prn: 9 };
        let l1 = signal_spec_gps_l1_ca();
        let l2 = signal_spec_gps_l2_py();

        ObsEpoch {
            t_rx_s: Seconds(epoch_idx as f64),
            source_time: ReceiverSampleTrace::from_sample_index(epoch_idx, 1.0),
            gps_week: None,
            tow_s: None,
            epoch_idx,
            discontinuity: false,
            valid: true,
            processing_ms: None,
            role: ReceiverRole::Rover,
            sats: vec![
                ObsSatellite {
                    signal_id: SigId { sat, band: SignalBand::L1, code: SignalCode::Ca },
                    pseudorange_m: Meters(p1_m),
                    pseudorange_var_m2: 1.0,
                    carrier_phase_cycles: Cycles(
                        phi1_m / (SPEED_OF_LIGHT_MPS / l1.carrier_hz.value()),
                    ),
                    carrier_phase_var_cycles2: 0.01,
                    doppler_hz: Hertz(0.0),
                    doppler_var_hz2: 1.0,
                    cn0_dbhz: 45.0,
                    lock_flags: LockFlags {
                        code_lock: true,
                        carrier_lock: true,
                        bit_lock: false,
                        cycle_slip: false,
                    },
                    multipath_suspect: false,
                    observation_status: ObservationStatus::Accepted,
                    observation_reject_reasons: Vec::new(),
                    elevation_deg: None,
                    azimuth_deg: None,
                    weight: None,
                    timing: None,
                    error_model: None,
                    metadata: ObsMetadata {
                        tracking_mode: "test".to_string(),
                        integration_ms: 1,
                        lock_quality: 1.0,
                        smoothing_window: 0,
                        smoothing_age: 0,
                        smoothing_resets: 0,
                        signal: l1,
                        ..ObsMetadata::default()
                    },
                },
                ObsSatellite {
                    signal_id: SigId { sat, band: SignalBand::L2, code: SignalCode::Py },
                    pseudorange_m: Meters(p2_m),
                    pseudorange_var_m2: 1.0,
                    carrier_phase_cycles: Cycles(
                        phi2_m / (SPEED_OF_LIGHT_MPS / l2.carrier_hz.value()),
                    ),
                    carrier_phase_var_cycles2: 0.01,
                    doppler_hz: Hertz(0.0),
                    doppler_var_hz2: 1.0,
                    cn0_dbhz: 45.0,
                    lock_flags: LockFlags {
                        code_lock: true,
                        carrier_lock: true,
                        bit_lock: false,
                        cycle_slip: false,
                    },
                    multipath_suspect: false,
                    observation_status: ObservationStatus::Accepted,
                    observation_reject_reasons: Vec::new(),
                    elevation_deg: None,
                    azimuth_deg: None,
                    weight: None,
                    timing: None,
                    error_model: None,
                    metadata: ObsMetadata {
                        tracking_mode: "test".to_string(),
                        integration_ms: 1,
                        lock_quality: 1.0,
                        smoothing_window: 0,
                        smoothing_age: 0,
                        smoothing_resets: 0,
                        signal: l2,
                        ..ObsMetadata::default()
                    },
                },
            ],
            decision: ObservationEpochDecision::Accepted,
            decision_reason: Some("accepted_observables_present".to_string()),
            manifest: None,
        }
    }

    #[test]
    fn melbourne_wubbena_marks_nominal_wide_lane_behavior() {
        let diagnostics = melbourne_wubbena_diagnostics_from_obs_epochs(
            &[
                dual_frequency_epoch(0, 22_000_000.0, 22_000_002.0, 21_999_999.0, 22_000_000.5),
                dual_frequency_epoch(1, 22_000_000.0, 22_000_002.0, 21_999_999.01, 22_000_000.49),
            ],
            SignalBand::L1,
            SignalBand::L2,
            MelbourneWubbenaThresholds { wide_lane_slip_jump_cycles: 0.5 },
        );

        assert_eq!(diagnostics[0].event, MelbourneWubbenaEvent::InsufficientHistory);
        assert_eq!(diagnostics[1].event, MelbourneWubbenaEvent::Nominal);
        assert!(
            diagnostics[1].delta_from_previous_wide_lane_cycles.expect("wide-lane delta").abs()
                < 0.5
        );
    }

    #[test]
    fn melbourne_wubbena_marks_wide_lane_slip() {
        let diagnostics = melbourne_wubbena_diagnostics_from_obs_epochs(
            &[
                dual_frequency_epoch(0, 22_000_000.0, 22_000_002.0, 21_999_999.0, 22_000_000.5),
                dual_frequency_epoch(1, 22_000_000.0, 22_000_002.0, 22_000_005.5, 22_000_000.5),
            ],
            SignalBand::L1,
            SignalBand::L2,
            MelbourneWubbenaThresholds { wide_lane_slip_jump_cycles: 0.5 },
        );

        assert_eq!(diagnostics[1].event, MelbourneWubbenaEvent::WideLaneSlipSuspect);
        assert!(
            diagnostics[1].delta_from_previous_wide_lane_cycles.expect("wide-lane delta").abs()
                >= 0.5
        );
    }
}
