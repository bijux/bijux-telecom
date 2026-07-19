#![allow(missing_docs)]

use std::collections::BTreeMap;

use bijux_gnss_core::api::{ObsEpoch, SatId, SignalBand};
use serde::{Deserialize, Serialize};

use crate::corrections::combinations::combinations_from_obs_epochs;

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct GeometryFreeThresholds {
    pub ionosphere_delta_m: f64,
    pub cycle_slip_jump_m: f64,
}

impl Default for GeometryFreeThresholds {
    fn default() -> Self {
        Self { ionosphere_delta_m: 0.01, cycle_slip_jump_m: 0.10 }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum GeometryFreeEvent {
    Unavailable,
    InsufficientHistory,
    Nominal,
    IonosphereDrift,
    CycleSlipSuspect,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GeometryFreeObservation {
    pub epoch_idx: u64,
    pub t_rx_s: f64,
    pub sat: SatId,
    pub band_1: SignalBand,
    pub band_2: SignalBand,
    pub geometry_free_phase_m: Option<f64>,
    pub delta_from_previous_m: Option<f64>,
    pub status: String,
    pub reason: String,
    pub event: GeometryFreeEvent,
}

pub fn geometry_free_diagnostics_from_obs_epochs(
    epochs: &[ObsEpoch],
    band_1: SignalBand,
    band_2: SignalBand,
    thresholds: GeometryFreeThresholds,
) -> Vec<GeometryFreeObservation> {
    let mut diagnostics = Vec::new();
    let mut previous_by_sat: BTreeMap<SatId, f64> = BTreeMap::new();

    for combination in combinations_from_obs_epochs(epochs, band_1, band_2) {
        let mut delta_from_previous_m = None;
        let event = if combination.status != "ok" {
            previous_by_sat.remove(&combination.sat);
            GeometryFreeEvent::Unavailable
        } else if let Some(geometry_free_phase_m) = combination.geometry_free_phase_m {
            let event = if let Some(previous) = previous_by_sat.get(&combination.sat) {
                let delta_m = geometry_free_phase_m - previous;
                delta_from_previous_m = Some(delta_m);
                classify_geometry_free_event(delta_m, thresholds)
            } else {
                GeometryFreeEvent::InsufficientHistory
            };
            previous_by_sat.insert(combination.sat, geometry_free_phase_m);
            event
        } else {
            previous_by_sat.remove(&combination.sat);
            GeometryFreeEvent::Unavailable
        };

        diagnostics.push(GeometryFreeObservation {
            epoch_idx: combination.epoch_idx,
            t_rx_s: combination.t_rx_s,
            sat: combination.sat,
            band_1: combination.band_1,
            band_2: combination.band_2,
            geometry_free_phase_m: combination.geometry_free_phase_m,
            delta_from_previous_m,
            status: combination.status,
            reason: combination.reason,
            event,
        });
    }

    diagnostics
}

fn classify_geometry_free_event(
    delta_m: f64,
    thresholds: GeometryFreeThresholds,
) -> GeometryFreeEvent {
    let abs_delta_m = delta_m.abs();
    if abs_delta_m >= thresholds.cycle_slip_jump_m {
        GeometryFreeEvent::CycleSlipSuspect
    } else if abs_delta_m >= thresholds.ionosphere_delta_m {
        GeometryFreeEvent::IonosphereDrift
    } else {
        GeometryFreeEvent::Nominal
    }
}

#[cfg(test)]
mod tests {
    use super::{
        geometry_free_diagnostics_from_obs_epochs, GeometryFreeEvent, GeometryFreeThresholds,
    };
    use bijux_gnss_core::api::{
        Constellation, Cycles, Hertz, LockFlags, Meters, ObsEpoch, ObsMetadata, ObsSatellite,
        ObservationEpochDecision, ObservationStatus, ReceiverRole, ReceiverSampleTrace, SatId,
        Seconds, SigId, SignalBand, SignalCode,
    };
    use bijux_gnss_signal::api::{signal_spec_gps_l1_ca, signal_spec_gps_l2_py};

    const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

    fn dual_frequency_epoch(epoch_idx: u64, phi1_m: f64, phi2_m: f64) -> ObsEpoch {
        let sat = SatId { constellation: Constellation::Gps, prn: 7 };
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
                    pseudorange_m: Meters(22_000_000.0),
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
                    pseudorange_m: Meters(22_000_001.0),
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
    fn geometry_free_marks_ionosphere_drift() {
        let diagnostics = geometry_free_diagnostics_from_obs_epochs(
            &[
                dual_frequency_epoch(0, 22_000_000.0, 21_999_999.50),
                dual_frequency_epoch(1, 22_000_000.0, 21_999_999.46),
            ],
            SignalBand::L1,
            SignalBand::L2,
            GeometryFreeThresholds { ionosphere_delta_m: 0.01, cycle_slip_jump_m: 0.10 },
        );

        assert_eq!(diagnostics[0].event, GeometryFreeEvent::InsufficientHistory);
        assert_eq!(diagnostics[1].event, GeometryFreeEvent::IonosphereDrift);
        let delta = diagnostics[1].delta_from_previous_m.expect("delta");
        assert!((delta - 0.04).abs() < 1.0e-8, "{delta}");
    }

    #[test]
    fn geometry_free_marks_cycle_slip_jump() {
        let diagnostics = geometry_free_diagnostics_from_obs_epochs(
            &[
                dual_frequency_epoch(0, 22_000_000.0, 21_999_999.50),
                dual_frequency_epoch(1, 22_000_000.0, 21_999_999.25),
            ],
            SignalBand::L1,
            SignalBand::L2,
            GeometryFreeThresholds { ionosphere_delta_m: 0.01, cycle_slip_jump_m: 0.10 },
        );

        assert_eq!(diagnostics[1].event, GeometryFreeEvent::CycleSlipSuspect);
        let delta = diagnostics[1].delta_from_previous_m.expect("delta");
        assert!((delta - 0.25).abs() < 1.0e-8, "{delta}");
    }
}
