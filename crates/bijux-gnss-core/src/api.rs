//! Public API for bijux-gnss-core.

pub use crate::artifact::v1::acq::AcqResultV1;
pub use crate::artifact::v1::nav::NavSolutionEpochV1;
pub use crate::artifact::v1::obs::ObsEpochV1;
pub use crate::artifact::v1::track::TrackEpochV1;
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
    geodetic_to_ecef, llh_to_ecef, Ecef, Enu, GpsTime, GpsWeek, Llh, Tow,
};
/// Identity types and signal definitions.
pub use crate::ids::{
    format_sat, prns_to_sats, signal_spec_gps_l1_ca, signal_spec_gps_l2_py, sort_obs_sats,
    sort_sat_ids, sort_sig_ids, Constellation, FreqHz, SatId, SigId, SignalBand, SignalCode,
    SignalSpec, GALILEO_E1_CARRIER_HZ, GALILEO_E5_CARRIER_HZ, GLONASS_L1_CARRIER_HZ,
    GPS_L1_CA_CARRIER_HZ, GPS_L2_PY_CARRIER_HZ, GPS_L5_CARRIER_HZ,
};
/// Observation and tracking contracts.
pub use crate::obs::{
    check_inter_frequency_alignment, validate_obs_epochs, AcqRequest, AcqResult, AmbiguityId,
    AmbiguityState, AmbiguityStatus, BandLagEvent, DoubleDifference, InterFrequencyAlignmentReport,
    InterSystemBias, LockFlags, MeasurementErrorModel, NavHealthEvent, NavResidual,
    NavSolutionEpoch, ObsEpoch, ObsMetadata, ObsSatellite, ReceiverRole, Sample, SamplesFrame,
    SingleDifference, SolutionStatus, TrackEpoch,
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
    Epoch, GpsTime, LeapSecondEntry, LeapSeconds, SampleClock, SampleTime, TaiTime, UtcTime,
};
/// Strong units for physical quantities.
pub use crate::units::{
    chips_to_seconds, cycles_to_meters, hz_to_rad_per_sec, Chips, Cycles, Hertz, Meters,
    MetersPerSecond, Seconds,
};
