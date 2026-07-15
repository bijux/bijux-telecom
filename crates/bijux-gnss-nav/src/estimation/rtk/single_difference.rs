use std::collections::BTreeMap;

use bijux_gnss_core::api::{
    AmbiguityId, ArtifactPayloadValidate, Constellation, DiagnosticEvent, DiagnosticSeverity,
    GpsTime, ObsEpoch, ObsSatellite, ObsSignalTiming, ObservationStatus, SigId,
};
use serde::{Deserialize, Serialize};

use super::antenna::{modeled_pseudorange_with_antenna_corrections_m, RtkAntennaCorrectionConfig};
use crate::orbits::gps::{sat_state_gps_l1ca_from_observation, GpsEphemeris};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

/// RTK single-difference observation formed as rover minus base for one signal.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RtkSingleDifferenceObservation {
    /// Signal identity shared by the base and rover observations.
    pub sig: SigId,
    /// Lowest contributing C/N0 across rover and base for this signal.
    pub min_cn0_dbhz: f64,
    /// Whether either contributing observation was marked multipath-suspect.
    pub multipath_suspect: bool,
    /// Rover pseudorange used to form the single difference.
    pub rover_pseudorange_m: f64,
    /// Optional rover transmit-time timing carried from the observation.
    pub rover_signal_timing: Option<ObsSignalTiming>,
    /// Base pseudorange used to form the single difference.
    pub base_pseudorange_m: f64,
    /// Optional base transmit-time timing carried from the observation.
    pub base_signal_timing: Option<ObsSignalTiming>,
    /// Single-difference code observation in meters.
    pub code_m: f64,
    /// Single-difference carrier phase observation in cycles.
    pub phase_cycles: f64,
    /// Single-difference Doppler observation in hertz.
    pub doppler_hz: f64,
    /// Propagated single-difference code variance in square meters.
    pub code_variance_m2: f64,
    /// Propagated single-difference carrier variance in square cycles.
    pub phase_variance_cycles2: f64,
    /// Rover ambiguity identifier for the contributing carrier observation.
    pub ambiguity_rover: AmbiguityId,
    /// Base ambiguity identifier for the contributing carrier observation.
    pub ambiguity_base: AmbiguityId,
}

/// Residual summary for a batch of RTK single-difference code observations.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RtkSingleDifferenceResidualMetrics {
    /// Root-mean-square code residual against the provided base/rover truth.
    pub residual_rms_m: f64,
    /// Root-mean-square modeled code variance carried by the observations.
    pub predicted_rms_m: f64,
    /// Number of observations contributing to the summary.
    pub used_observations: usize,
}

impl ArtifactPayloadValidate for RtkSingleDifferenceObservation {
    fn validate_payload(&self) -> Vec<DiagnosticEvent> {
        let mut events = Vec::new();
        if !self.rover_pseudorange_m.is_finite()
            || !self.base_pseudorange_m.is_finite()
            || !self.code_m.is_finite()
            || !self.phase_cycles.is_finite()
            || !self.doppler_hz.is_finite()
            || !self.min_cn0_dbhz.is_finite()
        {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_SD_NUMERIC_INVALID",
                "single-difference observation contains NaN/Inf",
            ));
        }
        if timing_is_invalid(self.rover_signal_timing) || timing_is_invalid(self.base_signal_timing)
        {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_SD_TIMING_INVALID",
                "single-difference timing contains NaN/Inf",
            ));
        }
        if self.code_variance_m2 < 0.0 || self.phase_variance_cycles2 < 0.0 {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_SD_VARIANCE_INVALID",
                "single-difference variance is negative",
            ));
        }
        events
    }
}

/// Build RTK single-difference observations from a base and rover epoch.
///
/// The sign convention is `rover - base`. Only accepted observations with finite
/// code, phase, and Doppler values and both code/carrier lock set are included.
pub fn rtk_single_differences_from_obs_epochs(
    base: &ObsEpoch,
    rover: &ObsEpoch,
) -> Vec<RtkSingleDifferenceObservation> {
    let mut base_by_signal: BTreeMap<SigId, &ObsSatellite> = BTreeMap::new();
    for sat in &base.sats {
        base_by_signal.insert(sat.signal_id, sat);
    }

    let mut out = Vec::new();
    for rover_sat in &rover.sats {
        let Some(base_sat) = base_by_signal.get(&rover_sat.signal_id) else {
            continue;
        };
        if !single_difference_input_is_usable(rover_sat)
            || !single_difference_input_is_usable(base_sat)
        {
            continue;
        }

        let rover_code_variance_m2 =
            observed_or_modeled_variance_m2(rover_sat, MeasurementKind::Code);
        let base_code_variance_m2 =
            observed_or_modeled_variance_m2(base_sat, MeasurementKind::Code);
        let rover_phase_variance_cycles2 =
            observed_or_modeled_variance_m2(rover_sat, MeasurementKind::Phase);
        let base_phase_variance_cycles2 =
            observed_or_modeled_variance_m2(base_sat, MeasurementKind::Phase);

        out.push(RtkSingleDifferenceObservation {
            sig: rover_sat.signal_id,
            min_cn0_dbhz: rover_sat.cn0_dbhz.min(base_sat.cn0_dbhz),
            multipath_suspect: rover_sat.multipath_suspect || base_sat.multipath_suspect,
            rover_pseudorange_m: rover_sat.pseudorange_m.0,
            rover_signal_timing: rover_sat.timing,
            base_pseudorange_m: base_sat.pseudorange_m.0,
            base_signal_timing: base_sat.timing,
            code_m: rover_sat.pseudorange_m.0 - base_sat.pseudorange_m.0,
            phase_cycles: rover_sat.carrier_phase_cycles.0 - base_sat.carrier_phase_cycles.0,
            doppler_hz: rover_sat.doppler_hz.0 - base_sat.doppler_hz.0,
            code_variance_m2: rover_code_variance_m2 + base_code_variance_m2,
            phase_variance_cycles2: rover_phase_variance_cycles2 + base_phase_variance_cycles2,
            ambiguity_rover: ambiguity_id_from_satellite(rover_sat),
            ambiguity_base: ambiguity_id_from_satellite(base_sat),
        });
    }
    out.sort_by_key(|observation| observation.sig);
    out
}

/// Choose the lowest-variance reference signal for double-difference formation.
pub fn choose_rtk_single_difference_reference_signal(
    observations: &[RtkSingleDifferenceObservation],
) -> Option<SigId> {
    observations
        .iter()
        .min_by(|left, right| {
            left.multipath_suspect
                .cmp(&right.multipath_suspect)
                .then_with(|| right.min_cn0_dbhz.total_cmp(&left.min_cn0_dbhz))
                .then_with(|| left.code_variance_m2.total_cmp(&right.code_variance_m2))
        })
        .map(|observation| observation.sig)
}

/// Choose one reference signal per constellation using the lowest code variance.
pub fn choose_rtk_single_difference_reference_signals_by_constellation(
    observations: &[RtkSingleDifferenceObservation],
) -> BTreeMap<Constellation, SigId> {
    let mut by_constellation: BTreeMap<Constellation, Vec<RtkSingleDifferenceObservation>> =
        BTreeMap::new();
    for observation in observations {
        by_constellation
            .entry(observation.ambiguity_rover.sig.sat.constellation)
            .or_default()
            .push(observation.clone());
    }

    let mut out = BTreeMap::new();
    for (constellation, items) in by_constellation {
        if let Some(sig) = choose_rtk_single_difference_reference_signal(&items) {
            out.insert(constellation, sig);
        }
    }
    out
}

/// Evaluate single-difference code residuals against known base and rover positions.
///
/// This helper uses broadcast GPS ephemerides and any carried transmit-time timing on
/// the observations to reconstruct satellite states at the relevant receive times.
pub fn rtk_single_difference_residual_metrics(
    observations: &[RtkSingleDifferenceObservation],
    base_ecef_m: [f64; 3],
    rover_ecef_m: [f64; 3],
    ephemerides: &[GpsEphemeris],
    base_receive_time_s: f64,
    rover_receive_time_s: f64,
) -> Option<RtkSingleDifferenceResidualMetrics> {
    rtk_single_difference_residual_metrics_with_antenna_corrections(
        observations,
        base_ecef_m,
        rover_ecef_m,
        ephemerides,
        base_receive_time_s,
        rover_receive_time_s,
        None,
    )
}

/// Evaluate single-difference code residuals with optional satellite and receiver antenna models.
pub fn rtk_single_difference_residual_metrics_with_antenna_corrections(
    observations: &[RtkSingleDifferenceObservation],
    base_ecef_m: [f64; 3],
    rover_ecef_m: [f64; 3],
    ephemerides: &[GpsEphemeris],
    base_receive_time_s: f64,
    rover_receive_time_s: f64,
    antenna_corrections: Option<&RtkAntennaCorrectionConfig>,
) -> Option<RtkSingleDifferenceResidualMetrics> {
    if observations.is_empty() {
        return None;
    }

    let mut residuals_m = Vec::new();
    let mut predicted_variances_m2 = Vec::new();
    for observation in observations {
        let ephemeris =
            ephemerides.iter().find(|candidate| candidate.sat == observation.sig.sat)?;
        let rover_satellite = sat_state_gps_l1ca_from_observation(
            ephemeris,
            rover_receive_time_s,
            observation.rover_pseudorange_m,
            observation.rover_signal_timing,
        );
        let base_satellite = sat_state_gps_l1ca_from_observation(
            ephemeris,
            base_receive_time_s,
            observation.base_pseudorange_m,
            observation.base_signal_timing,
        );
        let gps_time = Some(GpsTime { week: ephemeris.week, tow_s: rover_receive_time_s });
        let rover_modeled_m = modeled_pseudorange_with_antenna_corrections_m(
            rover_ecef_m,
            [rover_satellite.x_m, rover_satellite.y_m, rover_satellite.z_m],
            rover_satellite.clock_correction.bias_s,
            observation.sig.sat,
            observation.sig.band,
            gps_time,
            antenna_corrections.and_then(|config| config.rover_antenna_type.as_deref()),
            antenna_corrections,
        );
        let base_modeled_m = modeled_pseudorange_with_antenna_corrections_m(
            base_ecef_m,
            [base_satellite.x_m, base_satellite.y_m, base_satellite.z_m],
            base_satellite.clock_correction.bias_s,
            observation.sig.sat,
            observation.sig.band,
            Some(GpsTime { week: ephemeris.week, tow_s: base_receive_time_s }),
            antenna_corrections.and_then(|config| config.base_antenna_type.as_deref()),
            antenna_corrections,
        );
        residuals_m.push(observation.code_m - (rover_modeled_m - base_modeled_m));
        predicted_variances_m2.push(observation.code_variance_m2.max(1.0e-6));
    }

    let residual_rms_m = (residuals_m.iter().map(|residual| residual * residual).sum::<f64>()
        / residuals_m.len() as f64)
        .sqrt();
    let predicted_rms_m =
        (predicted_variances_m2.iter().sum::<f64>() / predicted_variances_m2.len() as f64).sqrt();
    Some(RtkSingleDifferenceResidualMetrics {
        residual_rms_m,
        predicted_rms_m,
        used_observations: residuals_m.len(),
    })
}

fn ambiguity_id_from_satellite(sat: &ObsSatellite) -> AmbiguityId {
    AmbiguityId { sig: sat.signal_id, signal: format!("{:?}", sat.metadata.signal.band) }
}

fn single_difference_input_is_usable(sat: &ObsSatellite) -> bool {
    sat.observation_status == ObservationStatus::Accepted
        && sat.lock_flags.code_lock
        && sat.lock_flags.carrier_lock
        && sat.pseudorange_m.0.is_finite()
        && sat.carrier_phase_cycles.0.is_finite()
        && sat.doppler_hz.0.is_finite()
}

fn observed_or_modeled_variance_m2(sat: &ObsSatellite, measurement: MeasurementKind) -> f64 {
    let observed = match measurement {
        MeasurementKind::Code => sat.pseudorange_var_m2,
        MeasurementKind::Phase => sat.carrier_phase_var_cycles2,
    };
    if observed > 0.0 {
        observed
    } else {
        modeled_variance_m2(sat.cn0_dbhz, sat.elevation_deg, measurement)
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use bijux_gnss_core::api::{Constellation, GpsTime, ObsSignalTiming, Seconds, SignalCode};

    use super::{
        geometric_range_m, rtk_single_difference_residual_metrics,
        rtk_single_difference_residual_metrics_with_antenna_corrections,
        RtkSingleDifferenceObservation, SPEED_OF_LIGHT_MPS,
    };
    use crate::estimation::rtk::antenna::modeled_pseudorange_with_antenna_corrections_m;
    use crate::estimation::rtk::antenna::RtkAntennaCorrectionConfig;
    use crate::models::antenna::{
        ReceiverAntennaCalibration, ReceiverAntennaCalibrations, ReceiverPhaseCenterOffset,
        SatelliteAntennaCalibration, SatelliteAntennaCalibrations, SatellitePhaseCenterOffset,
    };
    use crate::orbits::gps::{sat_state_gps_l1ca_at_receive_time, GpsEphemeris};

    #[test]
    fn single_difference_antenna_corrections_reduce_matching_residual_bias() {
        let receive_gps_time = GpsTime { week: 2200, tow_s: 345_600.25 };
        let base_ecef_m = [-2_702_617.0, -4_292_747.0, 3_855_193.0];
        let rover_ecef_m = [-2_702_608.0, -4_292_752.0, 3_855_196.0];
        let ephemeris = GpsEphemeris {
            sat: bijux_gnss_core::api::SatId { constellation: Constellation::Gps, prn: 7 },
            iodc: 0,
            iode: 0,
            week: receive_gps_time.week,
            sv_health: 0,
            toe_s: receive_gps_time.tow_s - 900.0,
            toc_s: receive_gps_time.tow_s - 900.0,
            sqrt_a: 5153.7954775,
            e: 0.01,
            i0: 0.94,
            idot: 0.0,
            omega0: 0.8,
            omegadot: 0.0,
            w: 0.0,
            m0: 0.9,
            delta_n: 0.0,
            cuc: 0.0,
            cus: 0.0,
            crc: 0.0,
            crs: 0.0,
            cic: 0.0,
            cis: 0.0,
            af0: 0.0,
            af1: 0.0,
            af2: 0.0,
            tgd: 0.0,
        };
        let sat_state =
            sat_state_gps_l1ca_at_receive_time(&ephemeris, receive_gps_time.tow_s, 0.07);
        let sat_ecef_m = [sat_state.x_m, sat_state.y_m, sat_state.z_m];
        let rover_range_m = geometric_range_m(rover_ecef_m, sat_ecef_m);
        let base_range_m = geometric_range_m(base_ecef_m, sat_ecef_m);
        let rover_travel_time_s = rover_range_m / SPEED_OF_LIGHT_MPS;
        let base_travel_time_s = base_range_m / SPEED_OF_LIGHT_MPS;
        let config = RtkAntennaCorrectionConfig {
            base_antenna_type: Some("AOAD/M_T NONE".to_string()),
            rover_antenna_type: Some("TRM57971.00 NONE".to_string()),
            receiver_calibrations: Some(ReceiverAntennaCalibrations {
                entries: vec![
                    ReceiverAntennaCalibration {
                        antenna_type: "AOAD/M_T NONE".to_string(),
                        valid_from_unix_s: None,
                        valid_until_unix_s: None,
                        offsets_by_band: BTreeMap::from([(
                            bijux_gnss_core::api::SignalBand::L1,
                            ReceiverPhaseCenterOffset::new(0.03, 0.01, 0.82),
                        )]),
                        variations_by_band: BTreeMap::new(),
                    },
                    ReceiverAntennaCalibration {
                        antenna_type: "TRM57971.00 NONE".to_string(),
                        valid_from_unix_s: None,
                        valid_until_unix_s: None,
                        offsets_by_band: BTreeMap::from([(
                            bijux_gnss_core::api::SignalBand::L1,
                            ReceiverPhaseCenterOffset::new(0.15, -0.06, 1.23),
                        )]),
                        variations_by_band: BTreeMap::new(),
                    },
                ],
            }),
            satellite_calibrations: Some(SatelliteAntennaCalibrations {
                entries: vec![SatelliteAntennaCalibration {
                    sat: ephemeris.sat,
                    antenna_type: "BLOCK IIR".to_string(),
                    valid_from_unix_s: None,
                    valid_until_unix_s: None,
                    offsets_by_band: BTreeMap::from([(
                        bijux_gnss_core::api::SignalBand::L1,
                        SatellitePhaseCenterOffset::new(0.04, -0.02, 0.18),
                    )]),
                    variations_by_band: BTreeMap::new(),
                }],
            }),
        };
        let rover_modeled = modeled_pseudorange_with_antenna_corrections_m(
            rover_ecef_m,
            sat_ecef_m,
            sat_state.clock_correction.bias_s,
            ephemeris.sat,
            bijux_gnss_core::api::SignalBand::L1,
            Some(receive_gps_time),
            config.rover_antenna_type.as_deref(),
            Some(&config),
        );
        let base_modeled = modeled_pseudorange_with_antenna_corrections_m(
            base_ecef_m,
            sat_ecef_m,
            sat_state.clock_correction.bias_s,
            ephemeris.sat,
            bijux_gnss_core::api::SignalBand::L1,
            Some(receive_gps_time),
            config.base_antenna_type.as_deref(),
            Some(&config),
        );
        let observation = RtkSingleDifferenceObservation {
            sig: bijux_gnss_core::api::SigId {
                sat: ephemeris.sat,
                band: bijux_gnss_core::api::SignalBand::L1,
                code: SignalCode::Ca,
            },
            min_cn0_dbhz: 45.0,
            multipath_suspect: false,
            rover_pseudorange_m: rover_modeled,
            rover_signal_timing: Some(ObsSignalTiming {
                signal_travel_time_s: Seconds(rover_travel_time_s),
                transmit_gps_time: receive_gps_time.offset_seconds(-rover_travel_time_s),
            }),
            base_pseudorange_m: base_modeled,
            base_signal_timing: Some(ObsSignalTiming {
                signal_travel_time_s: Seconds(base_travel_time_s),
                transmit_gps_time: receive_gps_time.offset_seconds(-base_travel_time_s),
            }),
            code_m: rover_modeled - base_modeled,
            phase_cycles: 0.0,
            doppler_hz: 0.0,
            code_variance_m2: 1.0,
            phase_variance_cycles2: 1.0,
            ambiguity_rover: bijux_gnss_core::api::AmbiguityId {
                sig: bijux_gnss_core::api::SigId {
                    sat: ephemeris.sat,
                    band: bijux_gnss_core::api::SignalBand::L1,
                    code: SignalCode::Ca,
                },
                signal: "L1".to_string(),
            },
            ambiguity_base: bijux_gnss_core::api::AmbiguityId {
                sig: bijux_gnss_core::api::SigId {
                    sat: ephemeris.sat,
                    band: bijux_gnss_core::api::SignalBand::L1,
                    code: SignalCode::Ca,
                },
                signal: "L1".to_string(),
            },
        };

        let uncorrected = rtk_single_difference_residual_metrics(
            &[observation.clone()],
            base_ecef_m,
            rover_ecef_m,
            &[ephemeris.clone()],
            receive_gps_time.tow_s,
            receive_gps_time.tow_s,
        )
        .expect("uncorrected residual metrics");
        let corrected = rtk_single_difference_residual_metrics_with_antenna_corrections(
            &[observation],
            base_ecef_m,
            rover_ecef_m,
            &[ephemeris],
            receive_gps_time.tow_s,
            receive_gps_time.tow_s,
            Some(&config),
        )
        .expect("corrected residual metrics");

        assert!(corrected.residual_rms_m < 1.0e-4);
        assert!(uncorrected.residual_rms_m > corrected.residual_rms_m * 100.0);
    }
}

fn modeled_variance_m2(
    cn0_dbhz: f64,
    elevation_deg: Option<f64>,
    measurement: MeasurementKind,
) -> f64 {
    let cn0_linear = 10.0_f64.powf(cn0_dbhz / 10.0).max(1.0);
    let elevation_weight = elevation_deg.unwrap_or(30.0).to_radians().sin().max(0.1);
    let nominal_sigma = match measurement {
        MeasurementKind::Code => 10.0,
        MeasurementKind::Phase => 0.02,
    };
    let sigma = nominal_sigma / (cn0_linear.sqrt() * elevation_weight);
    sigma * sigma
}

fn timing_is_invalid(timing: Option<ObsSignalTiming>) -> bool {
    let Some(timing) = timing else {
        return false;
    };
    !timing.signal_travel_time_s.0.is_finite() || !timing.transmit_gps_time.tow_s.is_finite()
}

#[derive(Debug, Clone, Copy)]
enum MeasurementKind {
    Code,
    Phase,
}

fn modeled_pseudorange_m(
    receiver_ecef_m: [f64; 3],
    sat_ecef_m: [f64; 3],
    sat_clock_bias_s: f64,
) -> f64 {
    geometric_range_m(receiver_ecef_m, sat_ecef_m) - sat_clock_bias_s * SPEED_OF_LIGHT_MPS
}

fn geometric_range_m(receiver_ecef_m: [f64; 3], sat_ecef_m: [f64; 3]) -> f64 {
    let dx = receiver_ecef_m[0] - sat_ecef_m[0];
    let dy = receiver_ecef_m[1] - sat_ecef_m[1];
    let dz = receiver_ecef_m[2] - sat_ecef_m[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}
