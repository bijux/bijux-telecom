//! Public API for bijux-gnss-core.

/// Artifact headers and versioned payloads.
pub use crate::artifact::{
    check_nav_solution_finite, check_obs_epoch_finite, convert_v1_to_v2, AcqResultV1,
    ArtifactHeaderV1, ArtifactKind, ArtifactReadPolicy, ArtifactValidate, NavSolutionEpochV1,
    ObsEpochV1, TrackEpochV1,
};
/// Configuration composition and validation.
pub use crate::config::{BijuxGnssConfig, SchemaVersion, ValidateConfig, ValidationReport};
/// Scientific conventions and sanity checks.
pub use crate::conventions::{
    check_nav_solution_sanity, check_obs_epoch_sanity, is_solution_valid, ConventionsConfig,
    NavSanityConfig,
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
/// Identity types and signal definitions.
pub use crate::ids::{
    signal_spec_gps_l1_ca, signal_spec_gps_l2_py, sort_obs_sats, sort_sat_ids, sort_sig_ids,
    Constellation, FreqHz, SatId, SigId, SignalBand, SignalCode, SignalSpec, GALILEO_E1_CARRIER_HZ,
    GALILEO_E5_CARRIER_HZ, GLONASS_L1_CARRIER_HZ, GPS_L1_CA_CARRIER_HZ, GPS_L2_PY_CARRIER_HZ,
    GPS_L5_CARRIER_HZ,
};
/// Observation and tracking contracts.
pub use crate::obs::{
    check_inter_frequency_alignment, validate_obs_epochs, AcqRequest, AcqResult, AmbiguityId,
    AmbiguityState, AmbiguityStatus, DoubleDifference, InterFrequencyAlignmentReport,
    InterSystemBias, LockFlags, MeasurementErrorModel, NavHealthEvent, NavResidual,
    NavSolutionEpoch, ObsEpoch, ObsMetadata, ObsSatellite, ReceiverRole, Sample, SamplesFrame,
    SingleDifference, SolutionStatus, TrackEpoch,
};
/// Time and epoch structures.
pub use crate::time::{Epoch, GpsTime, LeapSeconds, SampleClock, SampleTime, TaiTime, UtcTime};
/// Strong units for physical quantities.
pub use crate::units::{
    chips_to_seconds, cycles_to_meters, hz_to_rad_per_sec, Chips, Cycles, Hertz, Meters,
    MetersPerSecond, Seconds,
};
