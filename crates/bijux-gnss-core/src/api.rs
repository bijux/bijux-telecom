//! Public API for bijux-gnss-core.

pub use crate::artifact::v1::acq::AcqResultV1;
pub use crate::artifact::v1::acq_explain::AcqExplainV1;
pub use crate::artifact::v1::nav::NavSolutionEpochV1;
pub use crate::artifact::v1::obs::ObsEpochV1;
pub use crate::artifact::v1::obs_decision::ObsDecisionV1;
pub use crate::artifact::v1::support_matrix::SupportMatrixV1;
pub use crate::artifact::v1::track::TrackEpochV1;
pub use crate::artifact::v1::track_transition::TrackTransitionV1;
pub use crate::artifact::v1::{ppp, rtk};
/// Artifact headers and versioned payloads.
pub use crate::artifact::{
    convert_v1_to_v2, ArtifactHeaderV1, ArtifactKind, ArtifactPayloadValidate, ArtifactReadPolicy,
    ArtifactV1, ArtifactValidate,
};
/// Configuration composition and validation.
pub use crate::config::{BijuxGnssConfig, SchemaVersion, ValidateConfig, ValidationReport};
/// Scientific conventions and sanity checks.
pub use crate::conventions::{
    carrier_phase_increment, check_nav_solution_sanity, check_obs_epoch_sanity,
    doppler_from_phase_increment, is_solution_valid, ConventionsConfig, NavSanityConfig,
};
/// Structured diagnostics.
pub use crate::diagnostic::{
    aggregate_diagnostics, lookup_diagnostic, DiagnosticCode, DiagnosticEvent, DiagnosticSeverity,
    DiagnosticSummary, DiagnosticSummaryEntry, DIAGNOSTIC_CODES,
};
/// Canonical error types.
pub use crate::error::{
    AcqError, ConfigError, InputError, InvariantError, IoError, NavError, ParseError, SignalError,
    TrackError,
};
/// Geodetic utilities (WGS-84).
pub use crate::geo::{
    ecef_to_enu, ecef_to_enu_ref, ecef_to_geodetic, ecef_to_llh, elevation_azimuth_deg,
    geodetic_to_ecef, llh_to_ecef, Ecef, Enu, Llh,
};
/// Identity types and signal definitions.
pub use crate::ids::{
    default_acquisition_sats, default_acquisition_signal, format_sat, prns_to_sats,
    signal_registry, signal_spec_galileo_e1b, signal_spec_galileo_e1c, signal_spec_gps_l1_ca,
    signal_spec_gps_l2_py, signal_spec_gps_l5, sort_obs_sats, sort_sat_ids, sort_sig_ids,
    Constellation, FreqHz, SatId, SigId, SignalBand, SignalCode, SignalRegistryEntry, SignalSpec,
    BEIDOU_B1_CARRIER_HZ, BEIDOU_B2_CARRIER_HZ, GALILEO_E1_CARRIER_HZ, GALILEO_E5_CARRIER_HZ,
    GLONASS_L1_CARRIER_HZ, GPS_L1_CA_CARRIER_HZ, GPS_L2_PY_CARRIER_HZ, GPS_L5_CARRIER_HZ,
};
/// Observation and tracking contracts.
pub use crate::obs::{
    acq_result_stability_key, obs_epoch_stability_key, stable_acq_result_keys,
    trackable_acq_tracking_seeds, AcqAssumptions, AcqCodePhaseRefinement, AcqDopplerRefinement,
    AcqEvidence, AcqExplain, AcqExplainCandidate, AcqHypothesis, AcqRequest, AcqResult,
    AcqSearchSummary, AcqThresholdProvenance, AcqTrackingSeed, AcqUncertainty, AmbiguityId,
    AmbiguityState, AmbiguityStatus, DoubleDifference, InterSystemBias, LockFlags,
    MeasurementErrorModel, MeasurementRejectReason, NavAssumptions, NavHealthEvent,
    NavLifecycleState, NavProvenance, NavQualityFlag, NavRefusalClass, NavResidual,
    NavSolutionEpoch, NavUncertaintyClass, ObsDecisionArtifact, ObsEpoch, ObsEpochManifest,
    ObsMetadata, ObsSatellite, ObsSignalTiming, ObservationEpochDecision, ObservationStatus,
    ObservationSupportClass, ObservationUncertaintyClass, ReceiverRole, Sample, SamplesFrame,
    SatObservationDecision, SignalDelayAlignment, SignalSupportRow, SingleDifference,
    SolutionStatus, SolutionValidity, SupportMatrix, SupportStatus, TrackEpoch, TrackTransition,
    TrackingAssumptions, TrackingLifecycleState, TrackingUncertainty,
    NAV_OUTPUT_STABILITY_SIGNATURE_VERSION, NAV_SOLUTION_MODEL_VERSION,
    OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET, OBSERVATION_DOWNSTREAM_PROFILE_VERSION,
    OBSERVATION_MODEL_VERSION, TRACKING_STATE_MODEL_VERSION,
};
pub use crate::obs_validation::{
    check_dual_frequency_observations, check_inter_frequency_alignment, validate_obs_epochs,
    BandLagEvent, DualFrequencyObservationPair, DualFrequencyObservationReport,
    DualFrequencyPairIssue, DualFrequencyPairStatus, InterFrequencyAlignmentReport,
};
/// Engine boundary nav epoch alias.
pub type NavEpoch = NavSolutionEpoch;
/// Reference validation helpers.
pub use crate::reference_validation::{
    align_reference_by_time, check_solution_consistency, reference_compare, reference_ecef,
    ReferenceAlign, ReferenceCompareStats, SolutionConsistencyReport, ValidationReferenceEpoch,
};
/// Statistical summaries.
pub use crate::stats::{lla_to_ecef, stats, StatsSummary};
/// Time and epoch structures.
pub use crate::time::{
    gps_to_utc, utc_to_gps, Epoch, GpsTime, LeapSecondEntry, LeapSeconds, ReceiverSampleTrace,
    SampleClock, SampleTime, TaiTime, UtcTime,
};
/// Strong units for physical quantities.
pub use crate::units::{
    chips_to_seconds, cycles_to_meters, hz_to_rad_per_sec, Chips, Cycles, Hertz, Meters,
    MetersPerSecond, Seconds,
};
