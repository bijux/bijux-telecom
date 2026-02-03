//! Public API for bijux-gnss-core.

/// Artifact headers and versioned payloads.
pub use crate::artifact::{
    check_nav_solution_finite, check_obs_epoch_finite, AcqResultV1, ArtifactCompatibility,
    ArtifactHeader, ArtifactKind, NavSolutionEpochV1, ObsEpochV1, TrackEpochV1,
};
/// Configuration composition and validation.
pub use crate::config::{BijuxGnssConfig, SchemaVersion, ValidateConfig, ValidationReport};
/// Structured diagnostics.
pub use crate::diagnostic::{DiagnosticEvent, DiagnosticSeverity};
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
    SingleDifference, TrackEpoch,
};
/// Time and epoch structures.
pub use crate::time::{Epoch, GpsTime, LeapSeconds, SampleClock, SampleTime, TaiTime, UtcTime};
/// Strong units for physical quantities.
pub use crate::units::{Chips, Cycles, Hertz, Meters, Seconds};
