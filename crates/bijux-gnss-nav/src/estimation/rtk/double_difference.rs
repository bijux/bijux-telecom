use std::collections::BTreeMap;

use bijux_gnss_core::api::{
    AmbiguityId, ArtifactPayloadValidate, Constellation, DiagnosticEvent, DiagnosticSeverity,
    GlonassFrequencyChannel, GpsTime, ObsSignalTiming, SigId,
};
use serde::{Deserialize, Serialize};

use super::antenna::{modeled_pseudorange_with_antenna_corrections_m, RtkAntennaCorrectionConfig};
use super::single_difference::{
    RtkEpochAlignmentEvidence, RtkSingleDifferenceCovarianceEvidence,
    RtkSingleDifferenceObservation,
};
use crate::orbits::gps::{sat_state_gps_l1ca_from_observation, GpsEphemeris};

/// Covariance evidence for one double difference and its reference single difference.
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize, PartialEq)]
pub struct RtkDoubleDifferenceCovarianceEvidence {
    /// Non-reference single-difference covariance evidence.
    pub signal: RtkSingleDifferenceCovarianceEvidence,
    /// Reference single-difference covariance evidence.
    pub reference: RtkSingleDifferenceCovarianceEvidence,
}

/// GLONASS FDMA receiver-bias handling state for one double difference.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RtkGlonassInterFrequencyBiasStatus {
    /// The double difference is not GLONASS and does not require FDMA bias handling.
    NotApplicable,
    /// Required GLONASS FDMA channel evidence is absent.
    ChannelEvidenceMissing,
    /// Channels differ and receiver-dependent inter-frequency calibration is required.
    CalibrationRequired,
    /// The inter-frequency bias requirement has been handled for integer ambiguity use.
    BiasHandled,
}

impl Default for RtkGlonassInterFrequencyBiasStatus {
    fn default() -> Self {
        Self::NotApplicable
    }
}

/// Evidence used to decide whether a GLONASS double-difference ambiguity is integer-compatible.
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct RtkGlonassInterFrequencyBiasEvidence {
    pub status: RtkGlonassInterFrequencyBiasStatus,
    pub signal_channel: Option<GlonassFrequencyChannel>,
    pub reference_channel: Option<GlonassFrequencyChannel>,
    pub code_bias_m: f64,
    pub phase_bias_cycles: f64,
}

/// Receiver-pair calibration for one GLONASS FDMA double-difference channel pair.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct RtkGlonassInterFrequencyBiasCalibration {
    pub signal_channel: GlonassFrequencyChannel,
    pub reference_channel: GlonassFrequencyChannel,
    pub code_bias_m: f64,
    pub phase_bias_cycles: f64,
}

/// RTK double-difference observation formed against a reference satellite.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RtkDoubleDifferenceObservation {
    /// Signal identity for the non-reference satellite.
    pub sig: SigId,
    /// Signal identity for the chosen reference satellite.
    pub ref_sig: SigId,
    /// Lowest contributing C/N0 across all four observations in the double difference.
    pub min_cn0_dbhz: f64,
    /// Whether any contributing observation was marked multipath-suspect.
    pub multipath_suspect: bool,
    /// Rover pseudorange for the non-reference satellite.
    pub rover_signal_pseudorange_m: f64,
    /// Optional rover timing for the non-reference satellite.
    pub rover_signal_timing: Option<ObsSignalTiming>,
    /// Base-station pseudorange for the non-reference satellite.
    pub base_signal_pseudorange_m: f64,
    /// Optional base-station timing for the non-reference satellite.
    pub base_signal_timing: Option<ObsSignalTiming>,
    /// Rover pseudorange for the reference satellite.
    pub rover_ref_pseudorange_m: f64,
    /// Optional rover timing for the reference satellite.
    pub rover_ref_signal_timing: Option<ObsSignalTiming>,
    /// Base-station pseudorange for the reference satellite.
    pub base_ref_pseudorange_m: f64,
    /// Optional base-station timing for the reference satellite.
    pub base_ref_signal_timing: Option<ObsSignalTiming>,
    /// Base/rover epoch alignment evidence shared by all four contributing observations.
    #[serde(default)]
    pub epoch_alignment: RtkEpochAlignmentEvidence,
    /// Covariance evidence for the signal and reference single differences.
    #[serde(default)]
    pub covariance_evidence: RtkDoubleDifferenceCovarianceEvidence,
    /// GLONASS FDMA receiver-bias evidence for integer ambiguity compatibility.
    #[serde(default)]
    pub glonass_inter_frequency_bias: RtkGlonassInterFrequencyBiasEvidence,
    /// Double-difference code observation in meters.
    pub code_m: f64,
    /// Double-difference carrier phase observation in cycles.
    pub phase_cycles: f64,
    /// Double-difference Doppler observation in hertz.
    pub doppler_hz: f64,
    /// Propagated double-difference code variance in square meters.
    pub code_variance_m2: f64,
    /// Propagated double-difference carrier variance in square cycles.
    pub phase_variance_cycles2: f64,
    /// Ambiguity identifiers canceled by the double difference.
    pub canceled: Vec<AmbiguityId>,
}

/// Residual summary for a batch of RTK double-difference code observations.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RtkDoubleDifferenceResidualMetrics {
    /// Root-mean-square code residual against the provided baseline truth.
    pub residual_rms_m: f64,
    /// Root-mean-square modeled code variance carried by the observations.
    pub predicted_rms_m: f64,
    /// Number of observations contributing to the summary.
    pub used_observations: usize,
}

impl ArtifactPayloadValidate for RtkDoubleDifferenceObservation {
    fn validate_payload(&self) -> Vec<DiagnosticEvent> {
        let mut events = Vec::new();
        if !self.rover_signal_pseudorange_m.is_finite()
            || !self.base_signal_pseudorange_m.is_finite()
            || !self.rover_ref_pseudorange_m.is_finite()
            || !self.base_ref_pseudorange_m.is_finite()
            || !self.code_m.is_finite()
            || !self.phase_cycles.is_finite()
            || !self.doppler_hz.is_finite()
            || !self.min_cn0_dbhz.is_finite()
        {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_DD_NUMERIC_INVALID",
                "double-difference observation contains NaN/Inf",
            ));
        }
        if timing_is_invalid(self.rover_signal_timing)
            || timing_is_invalid(self.base_signal_timing)
            || timing_is_invalid(self.rover_ref_signal_timing)
            || timing_is_invalid(self.base_ref_signal_timing)
        {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_DD_TIMING_INVALID",
                "double-difference timing contains NaN/Inf",
            ));
        }
        if !epoch_alignment_is_valid(&self.epoch_alignment) {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_DD_EPOCH_ALIGNMENT_INVALID",
                "double-difference base/rover epoch alignment is invalid",
            ));
        }
        if !double_difference_covariance_evidence_is_valid(&self.covariance_evidence) {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_DD_COVARIANCE_EVIDENCE_INVALID",
                "double-difference covariance evidence is invalid",
            ));
        }
        if !glonass_inter_frequency_bias_evidence_is_valid(&self.glonass_inter_frequency_bias) {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_DD_GLONASS_BIAS_EVIDENCE_INVALID",
                "GLONASS inter-frequency bias evidence is invalid",
            ));
        }
        if self.sig.sat.constellation == Constellation::Glonass
            && !rtk_glonass_inter_frequency_bias_is_integer_compatible(self)
        {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_DD_GLONASS_BIAS_UNHANDLED",
                "GLONASS double-difference ambiguity requires FDMA bias handling before integer fixing",
            ));
        }
        if self.code_variance_m2 < 0.0 || self.phase_variance_cycles2 < 0.0 {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_DD_VARIANCE_INVALID",
                "double-difference variance is negative",
            ));
        }
        events
    }
}

/// Build RTK double-difference observations from single differences and a reference signal.
pub fn rtk_double_differences_from_single_differences(
    observations: &[RtkSingleDifferenceObservation],
    ref_sig: SigId,
) -> Vec<RtkDoubleDifferenceObservation> {
    let Some(reference) = observations.iter().find(|observation| observation.sig == ref_sig) else {
        return Vec::new();
    };

    let mut out = Vec::new();
    for observation in observations {
        if observation.sig == ref_sig {
            continue;
        }
        if observation.sig.sat.constellation != reference.sig.sat.constellation {
            continue;
        }
        if !same_epoch_alignment(&observation.epoch_alignment, &reference.epoch_alignment) {
            continue;
        }
        out.push(RtkDoubleDifferenceObservation {
            sig: observation.sig,
            ref_sig: reference.sig,
            min_cn0_dbhz: observation.min_cn0_dbhz.min(reference.min_cn0_dbhz),
            multipath_suspect: observation.multipath_suspect || reference.multipath_suspect,
            rover_signal_pseudorange_m: observation.rover_pseudorange_m,
            rover_signal_timing: observation.rover_signal_timing,
            base_signal_pseudorange_m: observation.base_pseudorange_m,
            base_signal_timing: observation.base_signal_timing,
            rover_ref_pseudorange_m: reference.rover_pseudorange_m,
            rover_ref_signal_timing: reference.rover_signal_timing,
            base_ref_pseudorange_m: reference.base_pseudorange_m,
            base_ref_signal_timing: reference.base_signal_timing,
            epoch_alignment: observation.epoch_alignment,
            covariance_evidence: RtkDoubleDifferenceCovarianceEvidence {
                signal: observation.covariance_evidence,
                reference: reference.covariance_evidence,
            },
            glonass_inter_frequency_bias: glonass_inter_frequency_bias_evidence(
                observation,
                reference,
            ),
            code_m: observation.code_m - reference.code_m,
            phase_cycles: observation.phase_cycles - reference.phase_cycles,
            doppler_hz: observation.doppler_hz - reference.doppler_hz,
            code_variance_m2: double_difference_variance_m2(
                observation,
                reference,
                MeasurementKind::Code,
            ),
            phase_variance_cycles2: double_difference_variance_m2(
                observation,
                reference,
                MeasurementKind::Phase,
            ),
            canceled: vec![
                observation.ambiguity_rover.clone(),
                observation.ambiguity_base.clone(),
                reference.ambiguity_rover.clone(),
                reference.ambiguity_base.clone(),
            ],
        });
    }
    out.sort_by_key(|observation| observation.sig);
    out
}

/// Build RTK double differences independently per constellation.
pub fn rtk_double_differences_by_constellation(
    observations: &[RtkSingleDifferenceObservation],
    references: &BTreeMap<Constellation, SigId>,
) -> Vec<RtkDoubleDifferenceObservation> {
    let mut out = Vec::new();
    for (constellation, ref_sig) in references {
        let subset: Vec<RtkSingleDifferenceObservation> = observations
            .iter()
            .filter(|observation| observation.sig.sat.constellation == *constellation)
            .cloned()
            .collect();
        out.extend(rtk_double_differences_from_single_differences(&subset, *ref_sig));
    }
    out.sort_by_key(|observation| (observation.ref_sig, observation.sig));
    out
}

/// Rebuild double differences against a new reference signal without discarding the epoch state.
pub fn rtk_switch_double_difference_reference(
    observations: &[RtkDoubleDifferenceObservation],
    new_ref_sig: SigId,
) -> Option<Vec<RtkDoubleDifferenceObservation>> {
    let old_ref_sig = common_double_difference_reference(observations)?;
    if new_ref_sig == old_ref_sig {
        return Some(observations.to_vec());
    }

    let mut components = BTreeMap::new();
    let old_reference = reference_component(observations.first()?)?;
    components.insert(old_reference.sig, old_reference);
    for observation in observations {
        if observation.ref_sig != old_ref_sig {
            return None;
        }
        let component = signal_component(observation)?;
        components.insert(component.sig, component);
    }
    let reference = components.get(&new_ref_sig)?.clone();

    let mut out = Vec::new();
    for signal in components.values().cloned() {
        if signal.sig == new_ref_sig {
            continue;
        }
        if !same_epoch_alignment(&signal.epoch_alignment, &reference.epoch_alignment) {
            return None;
        }
        let mut observation = RtkDoubleDifferenceObservation {
            sig: signal.sig,
            ref_sig: reference.sig,
            min_cn0_dbhz: signal.min_cn0_dbhz.min(reference.min_cn0_dbhz),
            multipath_suspect: signal.multipath_suspect || reference.multipath_suspect,
            rover_signal_pseudorange_m: signal.rover_pseudorange_m,
            rover_signal_timing: signal.rover_signal_timing,
            base_signal_pseudorange_m: signal.base_pseudorange_m,
            base_signal_timing: signal.base_signal_timing,
            rover_ref_pseudorange_m: reference.rover_pseudorange_m,
            rover_ref_signal_timing: reference.rover_signal_timing,
            base_ref_pseudorange_m: reference.base_pseudorange_m,
            base_ref_signal_timing: reference.base_signal_timing,
            epoch_alignment: signal.epoch_alignment,
            covariance_evidence: RtkDoubleDifferenceCovarianceEvidence {
                signal: signal.covariance_evidence,
                reference: reference.covariance_evidence,
            },
            glonass_inter_frequency_bias: glonass_inter_frequency_bias_evidence_from_components(
                &signal, &reference,
            ),
            code_m: signal.code_m - reference.code_m,
            phase_cycles: signal.phase_cycles - reference.phase_cycles,
            doppler_hz: signal.doppler_hz - reference.doppler_hz,
            code_variance_m2: 0.0,
            phase_variance_cycles2: 0.0,
            canceled: vec![
                signal.ambiguity_rover,
                signal.ambiguity_base,
                reference.ambiguity_rover.clone(),
                reference.ambiguity_base.clone(),
            ],
        };
        observation.code_variance_m2 =
            double_difference_variance_from_observation(&observation, MeasurementKind::Code);
        observation.phase_variance_cycles2 =
            double_difference_variance_from_observation(&observation, MeasurementKind::Phase);
        out.push(observation);
    }
    out.sort_by_key(|observation| observation.sig);
    Some(out)
}

/// Return true when a double-difference ambiguity is safe for integer treatment.
pub fn rtk_glonass_inter_frequency_bias_is_integer_compatible(
    observation: &RtkDoubleDifferenceObservation,
) -> bool {
    observation.sig.sat.constellation != Constellation::Glonass
        || observation.glonass_inter_frequency_bias.status
            == RtkGlonassInterFrequencyBiasStatus::BiasHandled
}

/// Apply receiver-dependent GLONASS FDMA code and phase bias calibrations to double differences.
pub fn rtk_apply_glonass_inter_frequency_bias_calibrations(
    observations: &[RtkDoubleDifferenceObservation],
    calibrations: &[RtkGlonassInterFrequencyBiasCalibration],
) -> Vec<RtkDoubleDifferenceObservation> {
    observations
        .iter()
        .map(|observation| {
            let mut corrected = observation.clone();
            let Some(calibration) =
                glonass_inter_frequency_bias_calibration_for_observation(observation, calibrations)
            else {
                return corrected;
            };
            corrected.code_m -= calibration.code_bias_m;
            corrected.phase_cycles -= calibration.phase_bias_cycles;
            corrected.glonass_inter_frequency_bias = RtkGlonassInterFrequencyBiasEvidence {
                status: RtkGlonassInterFrequencyBiasStatus::BiasHandled,
                signal_channel: Some(calibration.signal_channel),
                reference_channel: Some(calibration.reference_channel),
                code_bias_m: calibration.code_bias_m,
                phase_bias_cycles: calibration.phase_bias_cycles,
            };
            corrected
        })
        .collect()
}

/// Build the code covariance matrix for an ordered set of double differences.
pub fn rtk_double_difference_code_covariance_matrix(
    observations: &[RtkDoubleDifferenceObservation],
) -> Option<Vec<Vec<f64>>> {
    double_difference_covariance_matrix(observations, MeasurementKind::Code)
}

/// Build the carrier-phase covariance matrix for an ordered set of double differences.
pub fn rtk_double_difference_phase_covariance_matrix(
    observations: &[RtkDoubleDifferenceObservation],
) -> Option<Vec<Vec<f64>>> {
    double_difference_covariance_matrix(observations, MeasurementKind::Phase)
}

/// Build the Doppler covariance matrix for an ordered set of double differences.
pub fn rtk_double_difference_doppler_covariance_matrix(
    observations: &[RtkDoubleDifferenceObservation],
) -> Option<Vec<Vec<f64>>> {
    double_difference_covariance_matrix(observations, MeasurementKind::Doppler)
}

/// Evaluate double-difference code residuals against a known rover-base baseline.
pub fn rtk_double_difference_residual_metrics(
    observations: &[RtkDoubleDifferenceObservation],
    base_ecef_m: [f64; 3],
    rover_enu_m: [f64; 3],
    ephemerides: &[GpsEphemeris],
    receive_time_s: f64,
) -> Option<RtkDoubleDifferenceResidualMetrics> {
    rtk_double_difference_residual_metrics_with_antenna_corrections(
        observations,
        base_ecef_m,
        rover_enu_m,
        ephemerides,
        receive_time_s,
        None,
    )
}

/// Evaluate double-difference code residuals with optional satellite and receiver antenna models.
pub fn rtk_double_difference_residual_metrics_with_antenna_corrections(
    observations: &[RtkDoubleDifferenceObservation],
    base_ecef_m: [f64; 3],
    rover_enu_m: [f64; 3],
    ephemerides: &[GpsEphemeris],
    receive_time_s: f64,
    antenna_corrections: Option<&RtkAntennaCorrectionConfig>,
) -> Option<RtkDoubleDifferenceResidualMetrics> {
    if observations.is_empty() {
        return None;
    }

    let rover_ecef_m = enu_to_ecef(base_ecef_m, rover_enu_m);
    let mut residuals_m = Vec::new();
    let mut predicted_variances_m2 = Vec::new();
    for observation in observations {
        let (base_receive_time_s, rover_receive_time_s) =
            aligned_receive_times(observation, receive_time_s);
        let signal_ephemeris =
            ephemerides.iter().find(|candidate| candidate.sat == observation.sig.sat)?;
        let reference_ephemeris =
            ephemerides.iter().find(|candidate| candidate.sat == observation.ref_sig.sat)?;
        let rover_signal_satellite = sat_state_gps_l1ca_from_observation(
            signal_ephemeris,
            rover_receive_time_s,
            observation.rover_signal_pseudorange_m,
            observation.rover_signal_timing,
        );
        let base_signal_satellite = sat_state_gps_l1ca_from_observation(
            signal_ephemeris,
            base_receive_time_s,
            observation.base_signal_pseudorange_m,
            observation.base_signal_timing,
        );
        let rover_reference_satellite = sat_state_gps_l1ca_from_observation(
            reference_ephemeris,
            rover_receive_time_s,
            observation.rover_ref_pseudorange_m,
            observation.rover_ref_signal_timing,
        );
        let base_reference_satellite = sat_state_gps_l1ca_from_observation(
            reference_ephemeris,
            base_receive_time_s,
            observation.base_ref_pseudorange_m,
            observation.base_ref_signal_timing,
        );
        let rover_signal_gps_time =
            Some(GpsTime { week: signal_ephemeris.week, tow_s: rover_receive_time_s });
        let base_signal_gps_time =
            Some(GpsTime { week: signal_ephemeris.week, tow_s: base_receive_time_s });
        let rover_reference_gps_time =
            Some(GpsTime { week: reference_ephemeris.week, tow_s: rover_receive_time_s });
        let base_reference_gps_time =
            Some(GpsTime { week: reference_ephemeris.week, tow_s: base_receive_time_s });
        let modeled_signal_m = modeled_pseudorange_with_antenna_corrections_m(
            rover_ecef_m,
            [rover_signal_satellite.x_m, rover_signal_satellite.y_m, rover_signal_satellite.z_m],
            rover_signal_satellite.clock_correction.bias_s,
            observation.sig.sat,
            observation.sig.band,
            rover_signal_gps_time,
            antenna_corrections.and_then(|config| config.rover_antenna_type.as_deref()),
            antenna_corrections,
        ) - modeled_pseudorange_with_antenna_corrections_m(
            base_ecef_m,
            [base_signal_satellite.x_m, base_signal_satellite.y_m, base_signal_satellite.z_m],
            base_signal_satellite.clock_correction.bias_s,
            observation.sig.sat,
            observation.sig.band,
            base_signal_gps_time,
            antenna_corrections.and_then(|config| config.base_antenna_type.as_deref()),
            antenna_corrections,
        );
        let modeled_reference_m = modeled_pseudorange_with_antenna_corrections_m(
            rover_ecef_m,
            [
                rover_reference_satellite.x_m,
                rover_reference_satellite.y_m,
                rover_reference_satellite.z_m,
            ],
            rover_reference_satellite.clock_correction.bias_s,
            observation.ref_sig.sat,
            observation.ref_sig.band,
            rover_reference_gps_time,
            antenna_corrections.and_then(|config| config.rover_antenna_type.as_deref()),
            antenna_corrections,
        ) - modeled_pseudorange_with_antenna_corrections_m(
            base_ecef_m,
            [
                base_reference_satellite.x_m,
                base_reference_satellite.y_m,
                base_reference_satellite.z_m,
            ],
            base_reference_satellite.clock_correction.bias_s,
            observation.ref_sig.sat,
            observation.ref_sig.band,
            base_reference_gps_time,
            antenna_corrections.and_then(|config| config.base_antenna_type.as_deref()),
            antenna_corrections,
        );
        let modeled_code_m = modeled_signal_m - modeled_reference_m;
        residuals_m.push(observation.code_m - modeled_code_m);
        predicted_variances_m2.push(observation.code_variance_m2.max(1.0e-6));
    }

    let residual_rms_m = (residuals_m.iter().map(|residual| residual * residual).sum::<f64>()
        / residuals_m.len() as f64)
        .sqrt();
    let predicted_rms_m =
        (predicted_variances_m2.iter().sum::<f64>() / predicted_variances_m2.len() as f64).sqrt();
    Some(RtkDoubleDifferenceResidualMetrics {
        residual_rms_m,
        predicted_rms_m,
        used_observations: residuals_m.len(),
    })
}

fn timing_is_invalid(timing: Option<ObsSignalTiming>) -> bool {
    let Some(timing) = timing else {
        return false;
    };
    !timing.signal_travel_time_s.0.is_finite() || !timing.transmit_gps_time.tow_s.is_finite()
}

fn same_epoch_alignment(
    left: &RtkEpochAlignmentEvidence,
    right: &RtkEpochAlignmentEvidence,
) -> bool {
    epoch_alignment_is_valid(left)
        && epoch_alignment_is_valid(right)
        && left.base_receive_time_s == right.base_receive_time_s
        && left.rover_receive_time_s == right.rover_receive_time_s
        && left.delta_s == right.delta_s
        && left.tolerance_s == right.tolerance_s
}

fn epoch_alignment_is_valid(evidence: &RtkEpochAlignmentEvidence) -> bool {
    evidence.base_receive_time_s.is_finite()
        && evidence.rover_receive_time_s.is_finite()
        && evidence.delta_s.is_finite()
        && evidence.tolerance_s.is_finite()
        && evidence.delta_s >= 0.0
        && evidence.tolerance_s >= 0.0
        && evidence.delta_s <= evidence.tolerance_s
        && (evidence.delta_s - (evidence.base_receive_time_s - evidence.rover_receive_time_s).abs())
            .abs()
            <= 1.0e-12
}

fn aligned_receive_times(
    observation: &RtkDoubleDifferenceObservation,
    fallback_receive_time_s: f64,
) -> (f64, f64) {
    if epoch_alignment_is_valid(&observation.epoch_alignment) {
        (
            observation.epoch_alignment.base_receive_time_s,
            observation.epoch_alignment.rover_receive_time_s,
        )
    } else {
        (fallback_receive_time_s, fallback_receive_time_s)
    }
}

#[derive(Debug, Clone)]
struct SingleDifferenceComponent {
    sig: SigId,
    min_cn0_dbhz: f64,
    multipath_suspect: bool,
    rover_pseudorange_m: f64,
    rover_signal_timing: Option<ObsSignalTiming>,
    base_pseudorange_m: f64,
    base_signal_timing: Option<ObsSignalTiming>,
    epoch_alignment: RtkEpochAlignmentEvidence,
    covariance_evidence: RtkSingleDifferenceCovarianceEvidence,
    glonass_frequency_channel: Option<GlonassFrequencyChannel>,
    code_m: f64,
    phase_cycles: f64,
    doppler_hz: f64,
    ambiguity_rover: AmbiguityId,
    ambiguity_base: AmbiguityId,
}

fn common_double_difference_reference(
    observations: &[RtkDoubleDifferenceObservation],
) -> Option<SigId> {
    let first = observations.first()?.ref_sig;
    observations.iter().all(|observation| observation.ref_sig == first).then_some(first)
}

fn signal_component(
    observation: &RtkDoubleDifferenceObservation,
) -> Option<SingleDifferenceComponent> {
    let ambiguity_rover = observation.canceled.first()?.clone();
    let ambiguity_base = observation.canceled.get(1)?.clone();
    Some(SingleDifferenceComponent {
        sig: observation.sig,
        min_cn0_dbhz: observation.min_cn0_dbhz,
        multipath_suspect: observation.multipath_suspect,
        rover_pseudorange_m: observation.rover_signal_pseudorange_m,
        rover_signal_timing: observation.rover_signal_timing,
        base_pseudorange_m: observation.base_signal_pseudorange_m,
        base_signal_timing: observation.base_signal_timing,
        epoch_alignment: observation.epoch_alignment,
        covariance_evidence: observation.covariance_evidence.signal,
        glonass_frequency_channel: observation.glonass_inter_frequency_bias.signal_channel,
        code_m: observation.code_m,
        phase_cycles: observation.phase_cycles,
        doppler_hz: observation.doppler_hz,
        ambiguity_rover,
        ambiguity_base,
    })
}

fn reference_component(
    observation: &RtkDoubleDifferenceObservation,
) -> Option<SingleDifferenceComponent> {
    let ambiguity_rover = observation.canceled.get(2)?.clone();
    let ambiguity_base = observation.canceled.get(3)?.clone();
    Some(SingleDifferenceComponent {
        sig: observation.ref_sig,
        min_cn0_dbhz: observation.min_cn0_dbhz,
        multipath_suspect: observation.multipath_suspect,
        rover_pseudorange_m: observation.rover_ref_pseudorange_m,
        rover_signal_timing: observation.rover_ref_signal_timing,
        base_pseudorange_m: observation.base_ref_pseudorange_m,
        base_signal_timing: observation.base_ref_signal_timing,
        epoch_alignment: observation.epoch_alignment,
        covariance_evidence: observation.covariance_evidence.reference,
        glonass_frequency_channel: observation.glonass_inter_frequency_bias.reference_channel,
        code_m: 0.0,
        phase_cycles: 0.0,
        doppler_hz: 0.0,
        ambiguity_rover,
        ambiguity_base,
    })
}

fn glonass_inter_frequency_bias_evidence(
    signal: &RtkSingleDifferenceObservation,
    reference: &RtkSingleDifferenceObservation,
) -> RtkGlonassInterFrequencyBiasEvidence {
    if signal.sig.sat.constellation != Constellation::Glonass {
        return RtkGlonassInterFrequencyBiasEvidence::default();
    }
    glonass_inter_frequency_bias_evidence_from_channels(
        signal.glonass_frequency_channel,
        reference.glonass_frequency_channel,
    )
}

fn glonass_inter_frequency_bias_evidence_from_components(
    signal: &SingleDifferenceComponent,
    reference: &SingleDifferenceComponent,
) -> RtkGlonassInterFrequencyBiasEvidence {
    if signal.sig.sat.constellation != Constellation::Glonass {
        return RtkGlonassInterFrequencyBiasEvidence::default();
    }
    glonass_inter_frequency_bias_evidence_from_channels(
        signal.glonass_frequency_channel,
        reference.glonass_frequency_channel,
    )
}

fn glonass_inter_frequency_bias_evidence_from_channels(
    signal_channel: Option<GlonassFrequencyChannel>,
    reference_channel: Option<GlonassFrequencyChannel>,
) -> RtkGlonassInterFrequencyBiasEvidence {
    let status = match (signal_channel, reference_channel) {
        (Some(signal), Some(reference)) if signal == reference => {
            RtkGlonassInterFrequencyBiasStatus::BiasHandled
        }
        (Some(_), Some(_)) => RtkGlonassInterFrequencyBiasStatus::CalibrationRequired,
        _ => RtkGlonassInterFrequencyBiasStatus::ChannelEvidenceMissing,
    };
    RtkGlonassInterFrequencyBiasEvidence {
        status,
        signal_channel,
        reference_channel,
        code_bias_m: 0.0,
        phase_bias_cycles: 0.0,
    }
}

fn glonass_inter_frequency_bias_calibration_for_observation<'a>(
    observation: &RtkDoubleDifferenceObservation,
    calibrations: &'a [RtkGlonassInterFrequencyBiasCalibration],
) -> Option<&'a RtkGlonassInterFrequencyBiasCalibration> {
    if observation.sig.sat.constellation != Constellation::Glonass {
        return None;
    }
    let signal_channel = observation.glonass_inter_frequency_bias.signal_channel?;
    let reference_channel = observation.glonass_inter_frequency_bias.reference_channel?;
    calibrations.iter().find(|calibration| {
        glonass_inter_frequency_bias_calibration_is_valid(calibration)
            && calibration.signal_channel == signal_channel
            && calibration.reference_channel == reference_channel
    })
}

fn glonass_inter_frequency_bias_calibration_is_valid(
    calibration: &RtkGlonassInterFrequencyBiasCalibration,
) -> bool {
    calibration.code_bias_m.is_finite() && calibration.phase_bias_cycles.is_finite()
}

fn double_difference_covariance_matrix(
    observations: &[RtkDoubleDifferenceObservation],
    measurement: MeasurementKind,
) -> Option<Vec<Vec<f64>>> {
    let mut matrix = vec![vec![0.0; observations.len()]; observations.len()];
    for (row, left) in observations.iter().enumerate() {
        if !double_difference_covariance_evidence_is_valid(&left.covariance_evidence) {
            return None;
        }
        for (col, right) in observations.iter().enumerate() {
            if !double_difference_covariance_evidence_is_valid(&right.covariance_evidence) {
                return None;
            }
            matrix[row][col] = if row == col {
                double_difference_variance_from_observation(left, measurement)
            } else {
                double_difference_cross_covariance(left, right, measurement)
            };
        }
    }
    Some(matrix)
}

fn double_difference_variance_m2(
    signal: &RtkSingleDifferenceObservation,
    reference: &RtkSingleDifferenceObservation,
    measurement: MeasurementKind,
) -> f64 {
    single_difference_variance(&signal.covariance_evidence, measurement)
        + single_difference_variance(&reference.covariance_evidence, measurement)
        - 2.0
            * single_difference_pair_covariance(
                signal.sig,
                &signal.covariance_evidence,
                reference.sig,
                &reference.covariance_evidence,
                measurement,
            )
}

fn double_difference_variance_from_observation(
    observation: &RtkDoubleDifferenceObservation,
    measurement: MeasurementKind,
) -> f64 {
    single_difference_variance(&observation.covariance_evidence.signal, measurement)
        + single_difference_variance(&observation.covariance_evidence.reference, measurement)
        - 2.0
            * single_difference_pair_covariance(
                observation.sig,
                &observation.covariance_evidence.signal,
                observation.ref_sig,
                &observation.covariance_evidence.reference,
                measurement,
            )
}

fn double_difference_cross_covariance(
    left: &RtkDoubleDifferenceObservation,
    right: &RtkDoubleDifferenceObservation,
    measurement: MeasurementKind,
) -> f64 {
    single_difference_pair_covariance(
        left.sig,
        &left.covariance_evidence.signal,
        right.sig,
        &right.covariance_evidence.signal,
        measurement,
    ) - single_difference_pair_covariance(
        left.sig,
        &left.covariance_evidence.signal,
        right.ref_sig,
        &right.covariance_evidence.reference,
        measurement,
    ) - single_difference_pair_covariance(
        left.ref_sig,
        &left.covariance_evidence.reference,
        right.sig,
        &right.covariance_evidence.signal,
        measurement,
    ) + single_difference_pair_covariance(
        left.ref_sig,
        &left.covariance_evidence.reference,
        right.ref_sig,
        &right.covariance_evidence.reference,
        measurement,
    )
}

fn single_difference_variance(
    evidence: &RtkSingleDifferenceCovarianceEvidence,
    measurement: MeasurementKind,
) -> f64 {
    match measurement {
        MeasurementKind::Code => (evidence.rover.code_m2 + evidence.base.code_m2
            - 2.0 * evidence.rover_base_code_covariance_m2)
            .max(0.0),
        MeasurementKind::Phase => (evidence.rover.phase_cycles2 + evidence.base.phase_cycles2
            - 2.0 * evidence.rover_base_phase_covariance_cycles2)
            .max(0.0),
        MeasurementKind::Doppler => (evidence.rover.doppler_hz2 + evidence.base.doppler_hz2
            - 2.0 * evidence.rover_base_doppler_covariance_hz2)
            .max(0.0),
    }
}

fn single_difference_covariance(
    left: &RtkSingleDifferenceCovarianceEvidence,
    right: &RtkSingleDifferenceCovarianceEvidence,
    measurement: MeasurementKind,
) -> f64 {
    match measurement {
        MeasurementKind::Code => {
            let rover_shared =
                left.rover.shared_clock_code_m2.min(right.rover.shared_clock_code_m2);
            let base_shared = left.base.shared_clock_code_m2.min(right.base.shared_clock_code_m2);
            rover_shared
                + base_shared
                + left.shared_code_covariance_m2.min(right.shared_code_covariance_m2)
        }
        MeasurementKind::Phase => {
            left.shared_phase_covariance_cycles2.min(right.shared_phase_covariance_cycles2)
        }
        MeasurementKind::Doppler => {
            left.shared_doppler_covariance_hz2.min(right.shared_doppler_covariance_hz2)
        }
    }
}

fn single_difference_pair_covariance(
    left_sig: SigId,
    left: &RtkSingleDifferenceCovarianceEvidence,
    right_sig: SigId,
    right: &RtkSingleDifferenceCovarianceEvidence,
    measurement: MeasurementKind,
) -> f64 {
    if left_sig == right_sig {
        single_difference_variance(left, measurement)
    } else {
        single_difference_covariance(left, right, measurement)
    }
}

fn double_difference_covariance_evidence_is_valid(
    evidence: &RtkDoubleDifferenceCovarianceEvidence,
) -> bool {
    single_difference_covariance_evidence_is_valid(&evidence.signal)
        && single_difference_covariance_evidence_is_valid(&evidence.reference)
}

fn glonass_inter_frequency_bias_evidence_is_valid(
    evidence: &RtkGlonassInterFrequencyBiasEvidence,
) -> bool {
    evidence.code_bias_m.is_finite()
        && evidence.phase_bias_cycles.is_finite()
        && match evidence.status {
            RtkGlonassInterFrequencyBiasStatus::NotApplicable => {
                evidence.signal_channel.is_none()
                    && evidence.reference_channel.is_none()
                    && evidence.code_bias_m == 0.0
                    && evidence.phase_bias_cycles == 0.0
            }
            RtkGlonassInterFrequencyBiasStatus::ChannelEvidenceMissing => {
                evidence.signal_channel.is_none() || evidence.reference_channel.is_none()
            }
            RtkGlonassInterFrequencyBiasStatus::CalibrationRequired => {
                evidence.signal_channel.is_some()
                    && evidence.reference_channel.is_some()
                    && evidence.signal_channel != evidence.reference_channel
                    && evidence.code_bias_m == 0.0
                    && evidence.phase_bias_cycles == 0.0
            }
            RtkGlonassInterFrequencyBiasStatus::BiasHandled => {
                evidence.signal_channel.is_some() && evidence.reference_channel.is_some()
            }
        }
}

fn single_difference_covariance_evidence_is_valid(
    evidence: &RtkSingleDifferenceCovarianceEvidence,
) -> bool {
    source_variance_is_valid(&evidence.rover)
        && source_variance_is_valid(&evidence.base)
        && covariance_is_bounded(
            evidence.rover_base_code_covariance_m2,
            evidence.rover.code_m2,
            evidence.base.code_m2,
        )
        && covariance_is_bounded(
            evidence.rover_base_phase_covariance_cycles2,
            evidence.rover.phase_cycles2,
            evidence.base.phase_cycles2,
        )
        && covariance_is_bounded(
            evidence.rover_base_doppler_covariance_hz2,
            evidence.rover.doppler_hz2,
            evidence.base.doppler_hz2,
        )
        && nonnegative_finite(evidence.shared_code_covariance_m2)
        && nonnegative_finite(evidence.shared_phase_covariance_cycles2)
        && nonnegative_finite(evidence.shared_doppler_covariance_hz2)
}

fn source_variance_is_valid(
    source: &super::single_difference::RtkSourceObservationVariance,
) -> bool {
    nonnegative_finite(source.code_m2)
        && nonnegative_finite(source.phase_cycles2)
        && nonnegative_finite(source.doppler_hz2)
        && nonnegative_finite(source.shared_clock_code_m2)
        && source.shared_clock_code_m2 <= source.code_m2
}

fn covariance_is_bounded(covariance: f64, left_variance: f64, right_variance: f64) -> bool {
    let bound = (left_variance * right_variance).sqrt();
    covariance.is_finite() && bound.is_finite() && covariance.abs() <= bound + 1.0e-12
}

fn nonnegative_finite(value: f64) -> bool {
    value.is_finite() && value >= 0.0
}

#[derive(Debug, Clone, Copy)]
enum MeasurementKind {
    Code,
    Phase,
    Doppler,
}

fn enu_to_ecef(base_ecef_m: [f64; 3], enu_m: [f64; 3]) -> [f64; 3] {
    let (lat_deg, lon_deg, _alt_m) = crate::estimation::position::solver::ecef_to_geodetic(
        base_ecef_m[0],
        base_ecef_m[1],
        base_ecef_m[2],
    );
    let (sin_lat, cos_lat) = lat_deg.to_radians().sin_cos();
    let (sin_lon, cos_lon) = lon_deg.to_radians().sin_cos();
    let east_m = enu_m[0];
    let north_m = enu_m[1];
    let up_m = enu_m[2];
    let dx = -sin_lon * east_m - sin_lat * cos_lon * north_m + cos_lat * cos_lon * up_m;
    let dy = cos_lon * east_m - sin_lat * sin_lon * north_m + cos_lat * sin_lon * up_m;
    let dz = cos_lat * north_m + sin_lat * up_m;
    [base_ecef_m[0] + dx, base_ecef_m[1] + dy, base_ecef_m[2] + dz]
}

fn modeled_pseudorange_m(
    receiver_ecef_m: [f64; 3],
    sat_ecef_m: [f64; 3],
    sat_clock_bias_s: f64,
) -> f64 {
    geometric_range_m(receiver_ecef_m, sat_ecef_m) - sat_clock_bias_s * 299_792_458.0
}

fn geometric_range_m(receiver_ecef_m: [f64; 3], sat_ecef_m: [f64; 3]) -> f64 {
    let dx = receiver_ecef_m[0] - sat_ecef_m[0];
    let dy = receiver_ecef_m[1] - sat_ecef_m[1];
    let dz = receiver_ecef_m[2] - sat_ecef_m[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use bijux_gnss_core::api::{
        Constellation, GpsTime, ObsSignalTiming, Seconds, SignalBand, SignalCode,
    };

    use super::{
        geometric_range_m, rtk_double_difference_residual_metrics,
        rtk_double_difference_residual_metrics_with_antenna_corrections,
        RtkDoubleDifferenceCovarianceEvidence, RtkDoubleDifferenceObservation,
        RtkGlonassInterFrequencyBiasEvidence,
    };
    use crate::estimation::position::solver::{ecef_to_enu, ecef_to_geodetic};
    use crate::estimation::rtk::antenna::{
        modeled_pseudorange_with_antenna_corrections_m, RtkAntennaCorrectionConfig,
    };
    use crate::estimation::rtk::single_difference::{
        RtkEpochAlignmentEvidence, RtkSingleDifferenceCovarianceEvidence,
        RtkSourceObservationVariance, RTK_EPOCH_ALIGNMENT_TOLERANCE_S,
    };
    use crate::models::antenna::{
        ReceiverAntennaCalibration, ReceiverAntennaCalibrations, ReceiverPhaseCenterOffset,
        SatelliteAntennaCalibration, SatelliteAntennaCalibrations, SatellitePhaseCenterOffset,
    };
    use crate::orbits::gps::{sat_state_gps_l1ca_at_receive_time, GpsEphemeris};

    #[test]
    fn double_difference_antenna_corrections_reduce_matching_residual_bias() {
        let receive_gps_time = GpsTime { week: 2200, tow_s: 345_600.25 };
        let base_ecef_m = [-2_702_617.0, -4_292_747.0, 3_855_193.0];
        let rover_ecef_m = [-2_702_608.0, -4_292_752.0, 3_855_196.0];
        let (lat_deg, lon_deg, alt_m) =
            ecef_to_geodetic(base_ecef_m[0], base_ecef_m[1], base_ecef_m[2]);
        let rover_enu_m = {
            let (east_m, north_m, up_m) = ecef_to_enu(
                rover_ecef_m[0],
                rover_ecef_m[1],
                rover_ecef_m[2],
                lat_deg,
                lon_deg,
                alt_m,
            );
            [east_m, north_m, up_m]
        };
        let signal_ephemeris = make_test_ephemeris(receive_gps_time, 7, 0.8, 0.9);
        let reference_ephemeris = make_test_ephemeris(receive_gps_time, 11, 1.6, 1.8);
        let signal_sat =
            sat_state_gps_l1ca_at_receive_time(&signal_ephemeris, receive_gps_time.tow_s, 0.07);
        let reference_sat =
            sat_state_gps_l1ca_at_receive_time(&reference_ephemeris, receive_gps_time.tow_s, 0.07);
        let signal_sat_ecef_m = [signal_sat.x_m, signal_sat.y_m, signal_sat.z_m];
        let reference_sat_ecef_m = [reference_sat.x_m, reference_sat.y_m, reference_sat.z_m];
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
                            SignalBand::L1,
                            ReceiverPhaseCenterOffset::new(0.03, 0.01, 0.82),
                        )]),
                        variations_by_band: BTreeMap::new(),
                    },
                    ReceiverAntennaCalibration {
                        antenna_type: "TRM57971.00 NONE".to_string(),
                        valid_from_unix_s: None,
                        valid_until_unix_s: None,
                        offsets_by_band: BTreeMap::from([(
                            SignalBand::L1,
                            ReceiverPhaseCenterOffset::new(0.15, -0.06, 1.23),
                        )]),
                        variations_by_band: BTreeMap::new(),
                    },
                ],
            }),
            satellite_calibrations: Some(SatelliteAntennaCalibrations {
                entries: vec![
                    SatelliteAntennaCalibration {
                        sat: signal_ephemeris.sat,
                        antenna_type: "BLOCK IIR".to_string(),
                        valid_from_unix_s: None,
                        valid_until_unix_s: None,
                        offsets_by_band: BTreeMap::from([(
                            SignalBand::L1,
                            SatellitePhaseCenterOffset::new(0.04, -0.02, 0.18),
                        )]),
                        variations_by_band: BTreeMap::new(),
                    },
                    SatelliteAntennaCalibration {
                        sat: reference_ephemeris.sat,
                        antenna_type: "BLOCK IIR".to_string(),
                        valid_from_unix_s: None,
                        valid_until_unix_s: None,
                        offsets_by_band: BTreeMap::from([(
                            SignalBand::L1,
                            SatellitePhaseCenterOffset::new(-0.03, 0.01, 0.14),
                        )]),
                        variations_by_band: BTreeMap::new(),
                    },
                ],
            }),
        };

        let observation = RtkDoubleDifferenceObservation {
            sig: make_test_sig(signal_ephemeris.sat),
            ref_sig: make_test_sig(reference_ephemeris.sat),
            min_cn0_dbhz: 45.0,
            multipath_suspect: false,
            rover_signal_pseudorange_m: modeled_with_timing(
                rover_ecef_m,
                signal_sat_ecef_m,
                signal_sat.clock_correction.bias_s,
                signal_ephemeris.sat,
                receive_gps_time,
                config.rover_antenna_type.as_deref(),
                &config,
            )
            .0,
            rover_signal_timing: Some(
                modeled_with_timing(
                    rover_ecef_m,
                    signal_sat_ecef_m,
                    signal_sat.clock_correction.bias_s,
                    signal_ephemeris.sat,
                    receive_gps_time,
                    config.rover_antenna_type.as_deref(),
                    &config,
                )
                .1,
            ),
            base_signal_pseudorange_m: modeled_with_timing(
                base_ecef_m,
                signal_sat_ecef_m,
                signal_sat.clock_correction.bias_s,
                signal_ephemeris.sat,
                receive_gps_time,
                config.base_antenna_type.as_deref(),
                &config,
            )
            .0,
            base_signal_timing: Some(
                modeled_with_timing(
                    base_ecef_m,
                    signal_sat_ecef_m,
                    signal_sat.clock_correction.bias_s,
                    signal_ephemeris.sat,
                    receive_gps_time,
                    config.base_antenna_type.as_deref(),
                    &config,
                )
                .1,
            ),
            rover_ref_pseudorange_m: modeled_with_timing(
                rover_ecef_m,
                reference_sat_ecef_m,
                reference_sat.clock_correction.bias_s,
                reference_ephemeris.sat,
                receive_gps_time,
                config.rover_antenna_type.as_deref(),
                &config,
            )
            .0,
            rover_ref_signal_timing: Some(
                modeled_with_timing(
                    rover_ecef_m,
                    reference_sat_ecef_m,
                    reference_sat.clock_correction.bias_s,
                    reference_ephemeris.sat,
                    receive_gps_time,
                    config.rover_antenna_type.as_deref(),
                    &config,
                )
                .1,
            ),
            base_ref_pseudorange_m: modeled_with_timing(
                base_ecef_m,
                reference_sat_ecef_m,
                reference_sat.clock_correction.bias_s,
                reference_ephemeris.sat,
                receive_gps_time,
                config.base_antenna_type.as_deref(),
                &config,
            )
            .0,
            base_ref_signal_timing: Some(
                modeled_with_timing(
                    base_ecef_m,
                    reference_sat_ecef_m,
                    reference_sat.clock_correction.bias_s,
                    reference_ephemeris.sat,
                    receive_gps_time,
                    config.base_antenna_type.as_deref(),
                    &config,
                )
                .1,
            ),
            epoch_alignment: RtkEpochAlignmentEvidence {
                base_receive_time_s: receive_gps_time.tow_s,
                rover_receive_time_s: receive_gps_time.tow_s,
                delta_s: 0.0,
                tolerance_s: RTK_EPOCH_ALIGNMENT_TOLERANCE_S,
            },
            covariance_evidence: RtkDoubleDifferenceCovarianceEvidence {
                signal: independent_single_difference_covariance(),
                reference: independent_single_difference_covariance(),
            },
            glonass_inter_frequency_bias: RtkGlonassInterFrequencyBiasEvidence::default(),
            code_m: 0.0,
            phase_cycles: 0.0,
            doppler_hz: 0.0,
            code_variance_m2: 1.0,
            phase_variance_cycles2: 1.0,
            canceled: Vec::new(),
        };
        let mut observation = observation;
        observation.code_m = (observation.rover_signal_pseudorange_m
            - observation.base_signal_pseudorange_m)
            - (observation.rover_ref_pseudorange_m - observation.base_ref_pseudorange_m);

        let uncorrected = rtk_double_difference_residual_metrics(
            &[observation.clone()],
            base_ecef_m,
            rover_enu_m,
            &[signal_ephemeris.clone(), reference_ephemeris.clone()],
            receive_gps_time.tow_s,
        )
        .expect("uncorrected double-difference residual metrics");
        let corrected = rtk_double_difference_residual_metrics_with_antenna_corrections(
            &[observation],
            base_ecef_m,
            rover_enu_m,
            &[signal_ephemeris, reference_ephemeris],
            receive_gps_time.tow_s,
            Some(&config),
        )
        .expect("corrected double-difference residual metrics");

        assert!(corrected.residual_rms_m < 1.0e-4);
        assert!(uncorrected.residual_rms_m > corrected.residual_rms_m * 100.0);
    }

    fn make_test_ephemeris(
        receive_gps_time: GpsTime,
        prn: u8,
        omega0: f64,
        m0: f64,
    ) -> GpsEphemeris {
        GpsEphemeris {
            sat: bijux_gnss_core::api::SatId { constellation: Constellation::Gps, prn },
            iodc: 0,
            iode: 0,
            week: receive_gps_time.week,
            sv_health: 0,
            sv_accuracy: Some(2),
            toe_s: receive_gps_time.tow_s - 900.0,
            toc_s: receive_gps_time.tow_s - 900.0,
            sqrt_a: 5153.7954775,
            e: 0.01,
            i0: 0.94,
            idot: 0.0,
            omega0,
            omegadot: 0.0,
            w: 0.0,
            m0,
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
        }
    }

    fn independent_single_difference_covariance() -> RtkSingleDifferenceCovarianceEvidence {
        RtkSingleDifferenceCovarianceEvidence {
            rover: RtkSourceObservationVariance {
                code_m2: 0.5,
                phase_cycles2: 0.5,
                doppler_hz2: 1.0,
                shared_clock_code_m2: 0.0,
            },
            base: RtkSourceObservationVariance {
                code_m2: 0.5,
                phase_cycles2: 0.5,
                doppler_hz2: 1.0,
                shared_clock_code_m2: 0.0,
            },
            rover_base_code_covariance_m2: 0.0,
            rover_base_phase_covariance_cycles2: 0.0,
            rover_base_doppler_covariance_hz2: 0.0,
            shared_code_covariance_m2: 0.0,
            shared_phase_covariance_cycles2: 0.0,
            shared_doppler_covariance_hz2: 0.0,
        }
    }

    fn make_test_sig(sat: bijux_gnss_core::api::SatId) -> bijux_gnss_core::api::SigId {
        bijux_gnss_core::api::SigId { sat, band: SignalBand::L1, code: SignalCode::Ca }
    }

    fn modeled_with_timing(
        receiver_ecef_m: [f64; 3],
        sat_ecef_m: [f64; 3],
        sat_clock_bias_s: f64,
        sat: bijux_gnss_core::api::SatId,
        receive_gps_time: GpsTime,
        antenna_type: Option<&str>,
        config: &RtkAntennaCorrectionConfig,
    ) -> (f64, ObsSignalTiming) {
        let range_m = geometric_range_m(receiver_ecef_m, sat_ecef_m);
        let travel_time_s = range_m / 299_792_458.0;
        (
            modeled_pseudorange_with_antenna_corrections_m(
                receiver_ecef_m,
                sat_ecef_m,
                sat_clock_bias_s,
                sat,
                SignalBand::L1,
                Some(receive_gps_time),
                antenna_type,
                Some(config),
            ),
            ObsSignalTiming {
                signal_travel_time_s: Seconds(travel_time_s),
                transmit_gps_time: receive_gps_time.offset_seconds(-travel_time_s),
            },
        )
    }
}
