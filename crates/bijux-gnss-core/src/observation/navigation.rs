use crate::api::{Constellation, SatId};
use serde::{Deserialize, Serialize};

pub const NAV_SOLUTION_MODEL_VERSION: u32 = 4;
pub const NAV_OUTPUT_STABILITY_SIGNATURE_VERSION: u32 = 2;

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, Default)]
pub enum NavUncertaintyClass {
    Low,
    Medium,
    High,
    #[default]
    Unknown,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum NavRefusalClass {
    InsufficientGeometry,
    InvalidSatelliteTime,
    InvalidEphemeris,
    InconsistentObservations,
    SolverFailure,
    UnsupportedConstellation,
    MixedConstellationInput,
    PartialDecodedNavigationState,
    ScientificPrerequisitesTooWeak,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavAssumptions {
    #[serde(default = "default_nav_time_system")]
    pub time_system: String,
    #[serde(default = "default_nav_reference_frame")]
    pub reference_frame: String,
    #[serde(default = "default_nav_clock_model")]
    pub clock_model: String,
    pub ephemeris_source: String,
    pub frame_decode_mode: String,
    #[serde(default = "default_nav_ephemeris_completeness")]
    pub ephemeris_completeness: String,
    pub ephemeris_count: usize,
}

fn default_nav_time_system() -> String {
    "gps".to_string()
}

fn default_nav_reference_frame() -> String {
    "ecef_wgs84".to_string()
}

fn default_nav_clock_model() -> String {
    "receiver_clock_bias_drift_linear".to_string()
}

fn default_nav_ephemeris_completeness() -> String {
    "unknown".to_string()
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NavProvenance {
    pub solver_family: String,
    pub weighting_mode: String,
    pub robust_solver: bool,
    pub raim_enabled: bool,
    pub satellites_used: Vec<SatId>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SolutionStatus {
    Unavailable,
    Refused,
    Degraded,
    IntegrityFailed,
    Diverged,
    CodeOnly,
    Float,
    Fixed,
}

impl SolutionStatus {
    pub fn is_valid(self) -> bool {
        !matches!(
            self,
            SolutionStatus::Unavailable
                | SolutionStatus::Refused
                | SolutionStatus::IntegrityFailed
                | SolutionStatus::Diverged
        )
    }

    pub fn lifecycle_state(self) -> NavLifecycleState {
        match self {
            SolutionStatus::Unavailable => NavLifecycleState::Unavailable,
            SolutionStatus::Refused => NavLifecycleState::Refused,
            SolutionStatus::Degraded => NavLifecycleState::Degraded,
            SolutionStatus::IntegrityFailed => NavLifecycleState::IntegrityFailed,
            SolutionStatus::Diverged => NavLifecycleState::Diverged,
            SolutionStatus::CodeOnly => NavLifecycleState::CodeOnly,
            SolutionStatus::Float => NavLifecycleState::Float,
            SolutionStatus::Fixed => NavLifecycleState::Fixed,
        }
    }

    pub fn decision_label(self) -> &'static str {
        match self {
            SolutionStatus::Unavailable => "unavailable",
            SolutionStatus::Refused => "refused",
            SolutionStatus::Degraded => "degraded",
            SolutionStatus::IntegrityFailed => "integrity_failed",
            SolutionStatus::Diverged => "diverged",
            SolutionStatus::CodeOnly | SolutionStatus::Float | SolutionStatus::Fixed => "accepted",
        }
    }

    pub fn quality_flag(self) -> NavQualityFlag {
        match self {
            SolutionStatus::Unavailable | SolutionStatus::Refused => NavQualityFlag::NoFix,
            SolutionStatus::Degraded
            | SolutionStatus::IntegrityFailed
            | SolutionStatus::Diverged => NavQualityFlag::Degraded,
            SolutionStatus::CodeOnly | SolutionStatus::Float => NavQualityFlag::Float,
            SolutionStatus::Fixed => NavQualityFlag::Fix,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum NavLifecycleState {
    #[default]
    Unavailable,
    Refused,
    Degraded,
    IntegrityFailed,
    Diverged,
    CodeOnly,
    Float,
    Fixed,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum NavQualityFlag {
    #[default]
    NoFix,
    Float,
    Fix,
    Degraded,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum SolutionValidity {
    #[default]
    Invalid,
    Coarse,
    Converging,
    Stable,
    Diverging,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MeasurementRejectReason {
    Outlier,
    Geometry,
    CycleSlip,
    InvalidEphemeris,
    EphemerisMismatch,
    EphemerisStale,
    EphemerisFuture,
    UnhealthySatellite,
    IncompleteEphemeris,
    TimeInconsistency,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum NavHealthEvent {
    CovarianceSymmetrized,
    CovarianceClamped {
        min_eigenvalue: f64,
    },
    CovarianceDiverged {
        max_variance: f64,
    },
    InnovationRejected {
        reason: String,
    },
    InnovationConsistencyAnomaly {
        normalized_innovation_squared: f64,
        lower_bound: f64,
        upper_bound: f64,
        measurement_dimension: usize,
    },
    CommonCodeDopplerAnomaly {
        common_code_step_m: f64,
        common_doppler_step_hz: f64,
        matched_satellite_count: usize,
        aligned_satellite_count: usize,
        code_step_threshold_m: f64,
        doppler_step_threshold_hz: f64,
    },
    ReplayTimingAnomaly {
        common_delay_step_m: f64,
        centered_delay_rms_m: f64,
        max_centered_delay_m: f64,
        matched_satellite_count: usize,
        positive_step_satellite_count: usize,
        common_delay_step_threshold_m: f64,
        centered_delay_rms_threshold_m: f64,
    },
    ConstellationClockInconsistency {
        constellation: Constellation,
        previous_bias_s: f64,
        current_bias_s: f64,
        bias_step_m: f64,
        bias_step_threshold_m: f64,
        supporting_satellite_count: usize,
    },
    ResidualTemporalCorrelation {
        lag1_correlation: f64,
        correlation_threshold: f64,
        matched_satellite_count: usize,
        previous_centered_rms_m: f64,
        current_centered_rms_m: f64,
        persistent_suspect_epochs: usize,
    },
    ImpossibleGeometry {
        receiver_radius_m: f64,
        altitude_m: f64,
        used_satellite_count: usize,
        min_receiver_radius_m: f64,
        max_receiver_radius_m: f64,
        min_altitude_m: f64,
        max_altitude_m: f64,
    },
    SatelliteClockAnomaly {
        sat: SatId,
        persistent_suspect_epochs: usize,
        max_solution_separation_m: f64,
        separation_threshold_m: f64,
    },
    ZtdClamped {
        before_m: f64,
        after_m: f64,
    },
}
