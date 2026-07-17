use std::collections::BTreeMap;

use bijux_gnss_core::api::{
    AmbiguityId, ArtifactPayloadValidate, Constellation, DiagnosticEvent, DiagnosticSeverity,
    GlonassFrequencyChannel, GpsTime, ObsEpoch, ObsSatellite, ObsSignalTiming, ObservationStatus,
    SigId, GLONASS_L1_CARRIER_HZ, GLONASS_L1_CHANNEL_SPACING_HZ,
};
use serde::{Deserialize, Serialize};

use super::antenna::{
    modeled_pseudorange_with_antenna_corrections_m, AntennaAwarePseudorangeRequest,
    RtkAntennaCorrectionConfig,
};
use crate::orbits::gps::{sat_state_gps_l1ca_from_observation, GpsEphemeris};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
/// Default maximum base/rover epoch separation accepted for RTK differencing.
pub const RTK_EPOCH_ALIGNMENT_TOLERANCE_S: f64 = 0.0005;

/// Evidence that a base/rover epoch pair was aligned before RTK differencing.
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize, PartialEq)]
pub struct RtkEpochAlignmentEvidence {
    /// Base receiver time used for the differenced observation.
    pub base_receive_time_s: f64,
    /// Rover receiver time used for the differenced observation.
    pub rover_receive_time_s: f64,
    /// Absolute base/rover receiver-time separation.
    pub delta_s: f64,
    /// Maximum accepted receiver-time separation for this pair.
    pub tolerance_s: f64,
}

/// Per-receiver measurement variance evidence used before differencing.
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize, PartialEq)]
pub struct RtkSourceObservationVariance {
    /// Code variance in square meters.
    pub code_m2: f64,
    /// Carrier phase variance in square cycles.
    pub phase_cycles2: f64,
    /// Doppler variance in square hertz.
    pub doppler_hz2: f64,
    /// Receiver-clock code covariance shared by same-receiver satellites.
    pub shared_clock_code_m2: f64,
}

/// Configurable covariance terms applied when forming rover-minus-base differences.
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize, PartialEq)]
pub struct RtkDifferencedCovarianceConfig {
    /// Correlation coefficient between rover and base code errors for the same signal.
    pub rover_base_code_correlation: f64,
    /// Correlation coefficient between rover and base carrier-phase errors for the same signal.
    pub rover_base_phase_correlation: f64,
    /// Correlation coefficient between rover and base Doppler errors for the same signal.
    pub rover_base_doppler_correlation: f64,
    /// Code covariance shared between different single differences, such as residual atmosphere.
    pub shared_environment_code_m2: f64,
    /// Carrier-phase covariance shared between different single differences.
    pub shared_environment_phase_cycles2: f64,
    /// Doppler covariance shared between different single differences.
    pub shared_environment_doppler_hz2: f64,
}

/// Source and covariance evidence for one rover-minus-base single difference.
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize, PartialEq)]
pub struct RtkSingleDifferenceCovarianceEvidence {
    /// Rover source variance evidence.
    pub rover: RtkSourceObservationVariance,
    /// Base source variance evidence.
    pub base: RtkSourceObservationVariance,
    /// Rover/base code covariance subtracted by the single difference.
    pub rover_base_code_covariance_m2: f64,
    /// Rover/base phase covariance subtracted by the single difference.
    pub rover_base_phase_covariance_cycles2: f64,
    /// Rover/base Doppler covariance subtracted by the single difference.
    pub rover_base_doppler_covariance_hz2: f64,
    /// Cross-satellite code covariance shared by single differences in this epoch.
    pub shared_code_covariance_m2: f64,
    /// Cross-satellite phase covariance shared by single differences in this epoch.
    pub shared_phase_covariance_cycles2: f64,
    /// Cross-satellite Doppler covariance shared by single differences in this epoch.
    pub shared_doppler_covariance_hz2: f64,
}

/// RTK single-difference observation formed as rover minus base for one signal.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RtkSingleDifferenceObservation {
    /// Signal identity shared by the base and rover observations.
    pub sig: SigId,
    /// Lowest contributing C/N0 across rover and base for this signal.
    pub min_cn0_dbhz: f64,
    /// Whether either contributing observation was marked multipath-suspect.
    pub multipath_suspect: bool,
    /// GLONASS FDMA channel evidence for this signal when the signal is GLONASS L1.
    #[serde(default)]
    pub glonass_frequency_channel: Option<GlonassFrequencyChannel>,
    /// Rover pseudorange used to form the single difference.
    pub rover_pseudorange_m: f64,
    /// Optional rover transmit-time timing carried from the observation.
    pub rover_signal_timing: Option<ObsSignalTiming>,
    /// Base pseudorange used to form the single difference.
    pub base_pseudorange_m: f64,
    /// Optional base transmit-time timing carried from the observation.
    pub base_signal_timing: Option<ObsSignalTiming>,
    /// Base/rover epoch alignment evidence used before differencing.
    #[serde(default)]
    pub epoch_alignment: RtkEpochAlignmentEvidence,
    /// Source covariance evidence used to form the single-difference variances.
    #[serde(default)]
    pub covariance_evidence: RtkSingleDifferenceCovarianceEvidence,
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
        if !epoch_alignment_is_valid(&self.epoch_alignment) {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_SD_EPOCH_ALIGNMENT_INVALID",
                "single-difference base/rover epoch alignment is invalid",
            ));
        }
        if !single_difference_covariance_evidence_is_valid(&self.covariance_evidence) {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_SD_COVARIANCE_EVIDENCE_INVALID",
                "single-difference covariance evidence is invalid",
            ));
        }
        if self.sig.sat.constellation == Constellation::Glonass
            && self.glonass_frequency_channel.is_none()
        {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "RTK_SD_GLONASS_CHANNEL_MISSING",
                "GLONASS single-difference observation is missing FDMA channel evidence",
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
    rtk_single_differences_from_aligned_obs_epochs(base, rover, RTK_EPOCH_ALIGNMENT_TOLERANCE_S)
}

/// Build RTK single-difference observations after checking base/rover epoch alignment.
///
/// The function refuses the entire epoch pair when the base and rover receiver
/// times are non-finite or farther apart than the declared tolerance.
pub fn rtk_single_differences_from_aligned_obs_epochs(
    base: &ObsEpoch,
    rover: &ObsEpoch,
    tolerance_s: f64,
) -> Vec<RtkSingleDifferenceObservation> {
    rtk_single_differences_from_aligned_obs_epochs_with_covariance(
        base,
        rover,
        tolerance_s,
        RtkDifferencedCovarianceConfig::default(),
    )
}

/// Build RTK single differences with explicit covariance correlation terms.
pub fn rtk_single_differences_from_aligned_obs_epochs_with_covariance(
    base: &ObsEpoch,
    rover: &ObsEpoch,
    tolerance_s: f64,
    covariance_config: RtkDifferencedCovarianceConfig,
) -> Vec<RtkSingleDifferenceObservation> {
    let Some(epoch_alignment) = epoch_alignment_evidence(base, rover, tolerance_s) else {
        return Vec::new();
    };
    if !differenced_covariance_config_is_valid(&covariance_config) {
        return Vec::new();
    }

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

        let covariance_evidence =
            single_difference_covariance_evidence(rover_sat, base_sat, &covariance_config);
        let glonass_frequency_channel = matching_glonass_frequency_channel(rover_sat, base_sat);

        out.push(RtkSingleDifferenceObservation {
            sig: rover_sat.signal_id,
            min_cn0_dbhz: rover_sat.cn0_dbhz.min(base_sat.cn0_dbhz),
            multipath_suspect: rover_sat.multipath_suspect || base_sat.multipath_suspect,
            glonass_frequency_channel,
            rover_pseudorange_m: rover_sat.pseudorange_m.0,
            rover_signal_timing: rover_sat.timing,
            base_pseudorange_m: base_sat.pseudorange_m.0,
            base_signal_timing: base_sat.timing,
            epoch_alignment,
            covariance_evidence,
            code_m: rover_sat.pseudorange_m.0 - base_sat.pseudorange_m.0,
            phase_cycles: rover_sat.carrier_phase_cycles.0 - base_sat.carrier_phase_cycles.0,
            doppler_hz: rover_sat.doppler_hz.0 - base_sat.doppler_hz.0,
            code_variance_m2: single_difference_code_variance_m2(&covariance_evidence),
            phase_variance_cycles2: single_difference_phase_variance_cycles2(&covariance_evidence),
            ambiguity_rover: ambiguity_id_from_satellite(rover_sat),
            ambiguity_base: ambiguity_id_from_satellite(base_sat),
        });
    }
    out.sort_by_key(|observation| observation.sig);
    out
}

/// Return epoch-alignment evidence for a pair if it is safe to difference.
pub fn rtk_epoch_alignment_evidence(
    base: &ObsEpoch,
    rover: &ObsEpoch,
    tolerance_s: f64,
) -> Option<RtkEpochAlignmentEvidence> {
    epoch_alignment_evidence(base, rover, tolerance_s)
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
        let rover_modeled_m =
            modeled_pseudorange_with_antenna_corrections_m(AntennaAwarePseudorangeRequest {
                receiver_ecef_m: rover_ecef_m,
                sat_ecef_m: [rover_satellite.x_m, rover_satellite.y_m, rover_satellite.z_m],
                sat_clock_bias_s: rover_satellite.clock_correction.bias_s,
                sat: observation.sig.sat,
                band: observation.sig.band,
                gps_time,
                receiver_antenna_type: antenna_corrections
                    .and_then(|config| config.rover_antenna_type.as_deref()),
                corrections: antenna_corrections,
            });
        let base_modeled_m =
            modeled_pseudorange_with_antenna_corrections_m(AntennaAwarePseudorangeRequest {
                receiver_ecef_m: base_ecef_m,
                sat_ecef_m: [base_satellite.x_m, base_satellite.y_m, base_satellite.z_m],
                sat_clock_bias_s: base_satellite.clock_correction.bias_s,
                sat: observation.sig.sat,
                band: observation.sig.band,
                gps_time: Some(GpsTime { week: ephemeris.week, tow_s: base_receive_time_s }),
                receiver_antenna_type: antenna_corrections
                    .and_then(|config| config.base_antenna_type.as_deref()),
                corrections: antenna_corrections,
            });
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

fn matching_glonass_frequency_channel(
    rover_sat: &ObsSatellite,
    base_sat: &ObsSatellite,
) -> Option<GlonassFrequencyChannel> {
    if rover_sat.signal_id.sat.constellation != Constellation::Glonass {
        return None;
    }
    let rover_channel = glonass_frequency_channel_from_satellite(rover_sat)?;
    let base_channel = glonass_frequency_channel_from_satellite(base_sat)?;
    (rover_channel == base_channel).then_some(rover_channel)
}

fn glonass_frequency_channel_from_satellite(sat: &ObsSatellite) -> Option<GlonassFrequencyChannel> {
    if sat.signal_id.sat.constellation != Constellation::Glonass {
        return None;
    }
    let offset = (sat.metadata.signal.carrier_hz.value() - GLONASS_L1_CARRIER_HZ.value())
        / GLONASS_L1_CHANNEL_SPACING_HZ.value();
    if !offset.is_finite() {
        return None;
    }
    let channel = offset.round();
    if (offset - channel).abs() > 1.0e-6 {
        return None;
    }
    GlonassFrequencyChannel::new(channel as i8)
}

fn single_difference_input_is_usable(sat: &ObsSatellite) -> bool {
    sat.observation_status == ObservationStatus::Accepted
        && sat.lock_flags.code_lock
        && sat.lock_flags.carrier_lock
        && sat.pseudorange_m.0.is_finite()
        && sat.carrier_phase_cycles.0.is_finite()
        && sat.doppler_hz.0.is_finite()
}

fn epoch_alignment_evidence(
    base: &ObsEpoch,
    rover: &ObsEpoch,
    tolerance_s: f64,
) -> Option<RtkEpochAlignmentEvidence> {
    let base_receive_time_s = base.t_rx_s.0;
    let rover_receive_time_s = rover.t_rx_s.0;
    if !base_receive_time_s.is_finite()
        || !rover_receive_time_s.is_finite()
        || !tolerance_s.is_finite()
        || tolerance_s < 0.0
    {
        return None;
    }
    let delta_s = (base_receive_time_s - rover_receive_time_s).abs();
    if delta_s > tolerance_s {
        return None;
    }
    Some(RtkEpochAlignmentEvidence {
        base_receive_time_s,
        rover_receive_time_s,
        delta_s,
        tolerance_s,
    })
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

fn observed_or_modeled_variance_m2(sat: &ObsSatellite, measurement: MeasurementKind) -> f64 {
    let observed = match measurement {
        MeasurementKind::Code => sat.pseudorange_var_m2,
        MeasurementKind::Phase => sat.carrier_phase_var_cycles2,
        MeasurementKind::Doppler => sat.doppler_var_hz2,
    };
    if observed > 0.0 {
        observed
    } else {
        modeled_variance_m2(sat.cn0_dbhz, sat.elevation_deg, measurement)
    }
}

/// Build the code covariance matrix for an ordered set of single differences.
pub fn rtk_single_difference_code_covariance_matrix(
    observations: &[RtkSingleDifferenceObservation],
) -> Option<Vec<Vec<f64>>> {
    single_difference_covariance_matrix(observations, MeasurementKind::Code)
}

/// Build the carrier-phase covariance matrix for an ordered set of single differences.
pub fn rtk_single_difference_phase_covariance_matrix(
    observations: &[RtkSingleDifferenceObservation],
) -> Option<Vec<Vec<f64>>> {
    single_difference_covariance_matrix(observations, MeasurementKind::Phase)
}

/// Build the Doppler covariance matrix for an ordered set of single differences.
pub fn rtk_single_difference_doppler_covariance_matrix(
    observations: &[RtkSingleDifferenceObservation],
) -> Option<Vec<Vec<f64>>> {
    single_difference_covariance_matrix(observations, MeasurementKind::Doppler)
}

fn single_difference_covariance_evidence(
    rover_sat: &ObsSatellite,
    base_sat: &ObsSatellite,
    config: &RtkDifferencedCovarianceConfig,
) -> RtkSingleDifferenceCovarianceEvidence {
    let rover = source_observation_variance(rover_sat);
    let base = source_observation_variance(base_sat);
    RtkSingleDifferenceCovarianceEvidence {
        rover,
        base,
        rover_base_code_covariance_m2: correlated_covariance(
            config.rover_base_code_correlation,
            rover.code_m2,
            base.code_m2,
        ),
        rover_base_phase_covariance_cycles2: correlated_covariance(
            config.rover_base_phase_correlation,
            rover.phase_cycles2,
            base.phase_cycles2,
        ),
        rover_base_doppler_covariance_hz2: correlated_covariance(
            config.rover_base_doppler_correlation,
            rover.doppler_hz2,
            base.doppler_hz2,
        ),
        shared_code_covariance_m2: config.shared_environment_code_m2,
        shared_phase_covariance_cycles2: config.shared_environment_phase_cycles2,
        shared_doppler_covariance_hz2: config.shared_environment_doppler_hz2,
    }
}

fn source_observation_variance(sat: &ObsSatellite) -> RtkSourceObservationVariance {
    RtkSourceObservationVariance {
        code_m2: observed_or_modeled_variance_m2(sat, MeasurementKind::Code),
        phase_cycles2: observed_or_modeled_variance_m2(sat, MeasurementKind::Phase),
        doppler_hz2: observed_or_modeled_variance_m2(sat, MeasurementKind::Doppler),
        shared_clock_code_m2: sat
            .error_model
            .as_ref()
            .map(|model| model.clock_error_m.0.powi(2))
            .filter(|variance| variance.is_finite() && *variance >= 0.0)
            .unwrap_or(0.0),
    }
}

fn single_difference_code_variance_m2(evidence: &RtkSingleDifferenceCovarianceEvidence) -> f64 {
    (evidence.rover.code_m2 + evidence.base.code_m2 - 2.0 * evidence.rover_base_code_covariance_m2)
        .max(0.0)
}

fn single_difference_phase_variance_cycles2(
    evidence: &RtkSingleDifferenceCovarianceEvidence,
) -> f64 {
    (evidence.rover.phase_cycles2 + evidence.base.phase_cycles2
        - 2.0 * evidence.rover_base_phase_covariance_cycles2)
        .max(0.0)
}

fn single_difference_doppler_variance_hz2(evidence: &RtkSingleDifferenceCovarianceEvidence) -> f64 {
    (evidence.rover.doppler_hz2 + evidence.base.doppler_hz2
        - 2.0 * evidence.rover_base_doppler_covariance_hz2)
        .max(0.0)
}

fn single_difference_covariance_matrix(
    observations: &[RtkSingleDifferenceObservation],
    measurement: MeasurementKind,
) -> Option<Vec<Vec<f64>>> {
    let mut matrix = vec![vec![0.0; observations.len()]; observations.len()];
    for (row, left) in observations.iter().enumerate() {
        if !single_difference_covariance_evidence_is_valid(&left.covariance_evidence) {
            return None;
        }
        for (col, right) in observations.iter().enumerate() {
            if !single_difference_covariance_evidence_is_valid(&right.covariance_evidence) {
                return None;
            }
            matrix[row][col] = if row == col {
                single_difference_variance(&left.covariance_evidence, measurement)
            } else {
                shared_single_difference_covariance(
                    &left.covariance_evidence,
                    &right.covariance_evidence,
                    measurement,
                )
            };
        }
    }
    Some(matrix)
}

fn single_difference_variance(
    evidence: &RtkSingleDifferenceCovarianceEvidence,
    measurement: MeasurementKind,
) -> f64 {
    match measurement {
        MeasurementKind::Code => single_difference_code_variance_m2(evidence),
        MeasurementKind::Phase => single_difference_phase_variance_cycles2(evidence),
        MeasurementKind::Doppler => single_difference_doppler_variance_hz2(evidence),
    }
}

fn shared_single_difference_covariance(
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

fn differenced_covariance_config_is_valid(config: &RtkDifferencedCovarianceConfig) -> bool {
    correlation_is_valid(config.rover_base_code_correlation)
        && correlation_is_valid(config.rover_base_phase_correlation)
        && correlation_is_valid(config.rover_base_doppler_correlation)
        && nonnegative_finite(config.shared_environment_code_m2)
        && nonnegative_finite(config.shared_environment_phase_cycles2)
        && nonnegative_finite(config.shared_environment_doppler_hz2)
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

fn source_variance_is_valid(source: &RtkSourceObservationVariance) -> bool {
    nonnegative_finite(source.code_m2)
        && nonnegative_finite(source.phase_cycles2)
        && nonnegative_finite(source.doppler_hz2)
        && nonnegative_finite(source.shared_clock_code_m2)
        && source.shared_clock_code_m2 <= source.code_m2
}

fn correlated_covariance(correlation: f64, left_variance: f64, right_variance: f64) -> f64 {
    if !correlation_is_valid(correlation) {
        return 0.0;
    }
    covariance_bound(left_variance, right_variance).map(|bound| correlation * bound).unwrap_or(0.0)
}

fn covariance_is_bounded(covariance: f64, left_variance: f64, right_variance: f64) -> bool {
    let Some(bound) = covariance_bound(left_variance, right_variance) else {
        return covariance == 0.0;
    };
    covariance.is_finite() && covariance.abs() <= bound + 1.0e-12
}

fn covariance_bound(left_variance: f64, right_variance: f64) -> Option<f64> {
    (nonnegative_finite(left_variance) && nonnegative_finite(right_variance))
        .then_some((left_variance * right_variance).sqrt())
}

fn correlation_is_valid(value: f64) -> bool {
    value.is_finite() && (-1.0..=1.0).contains(&value)
}

fn nonnegative_finite(value: f64) -> bool {
    value.is_finite() && value >= 0.0
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use bijux_gnss_core::api::{Constellation, GpsTime, ObsSignalTiming, Seconds, SignalCode};

    use super::{
        geometric_range_m, rtk_single_difference_residual_metrics,
        rtk_single_difference_residual_metrics_with_antenna_corrections, RtkEpochAlignmentEvidence,
        RtkSingleDifferenceCovarianceEvidence, RtkSingleDifferenceObservation,
        RtkSourceObservationVariance, RTK_EPOCH_ALIGNMENT_TOLERANCE_S, SPEED_OF_LIGHT_MPS,
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
            sv_accuracy: Some(2),
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
        let rover_modeled =
            modeled_pseudorange_with_antenna_corrections_m(AntennaAwarePseudorangeRequest {
                receiver_ecef_m: rover_ecef_m,
                sat_ecef_m,
                sat_clock_bias_s: sat_state.clock_correction.bias_s,
                sat: ephemeris.sat,
                band: bijux_gnss_core::api::SignalBand::L1,
                gps_time: Some(receive_gps_time),
                receiver_antenna_type: config.rover_antenna_type.as_deref(),
                corrections: Some(&config),
            });
        let base_modeled =
            modeled_pseudorange_with_antenna_corrections_m(AntennaAwarePseudorangeRequest {
                receiver_ecef_m: base_ecef_m,
                sat_ecef_m,
                sat_clock_bias_s: sat_state.clock_correction.bias_s,
                sat: ephemeris.sat,
                band: bijux_gnss_core::api::SignalBand::L1,
                gps_time: Some(receive_gps_time),
                receiver_antenna_type: config.base_antenna_type.as_deref(),
                corrections: Some(&config),
            });
        let observation = RtkSingleDifferenceObservation {
            sig: bijux_gnss_core::api::SigId {
                sat: ephemeris.sat,
                band: bijux_gnss_core::api::SignalBand::L1,
                code: SignalCode::Ca,
            },
            min_cn0_dbhz: 45.0,
            multipath_suspect: false,
            glonass_frequency_channel: None,
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
            epoch_alignment: RtkEpochAlignmentEvidence {
                base_receive_time_s: receive_gps_time.tow_s,
                rover_receive_time_s: receive_gps_time.tow_s,
                delta_s: 0.0,
                tolerance_s: RTK_EPOCH_ALIGNMENT_TOLERANCE_S,
            },
            covariance_evidence: RtkSingleDifferenceCovarianceEvidence {
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
            },
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
        MeasurementKind::Doppler => 5.0,
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
    Doppler,
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
