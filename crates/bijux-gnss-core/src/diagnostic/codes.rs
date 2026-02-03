#![allow(missing_docs)]

use crate::{DiagnosticCode, DiagnosticSeverity};

pub const DIAGNOSTIC_CODES: &[DiagnosticCode] = &[
    DiagnosticCode {
        code: "GNSS_NUMERIC_ACQ_INVALID",
        severity: DiagnosticSeverity::Error,
        meaning: "Acquisition results contain NaN/Inf",
        mitigation: "Inspect input IQ scaling and acquisition parameters.",
    },
    DiagnosticCode {
        code: "GNSS_NUMERIC_TRACK_INVALID",
        severity: DiagnosticSeverity::Error,
        meaning: "Tracking epoch contains NaN/Inf",
        mitigation: "Inspect loop configuration and correlator outputs.",
    },
    DiagnosticCode {
        code: "GNSS_NUMERIC_OBS_INVALID",
        severity: DiagnosticSeverity::Error,
        meaning: "Observation contains NaN/Inf",
        mitigation: "Check tracking outputs and observables conversion.",
    },
    DiagnosticCode {
        code: "GNSS_NUMERIC_PVT_INVALID",
        severity: DiagnosticSeverity::Error,
        meaning: "Navigation solution contains NaN/Inf",
        mitigation: "Inspect measurement inputs and solver configuration.",
    },
    DiagnosticCode {
        code: "GNSS_NUMERIC_T_RX_INVALID",
        severity: DiagnosticSeverity::Error,
        meaning: "Receiver time tag is not finite",
        mitigation: "Check sample clock and epoch timing logic.",
    },
    DiagnosticCode {
        code: "GNSS_EPOCH_ALIGN_FAIL",
        severity: DiagnosticSeverity::Warning,
        meaning: "Base/rover epoch alignment failed or had gaps",
        mitigation: "Check dataset timing and alignment tolerance.",
    },
    DiagnosticCode {
        code: "GNSS_OBS_VALIDATE_FAILED",
        severity: DiagnosticSeverity::Error,
        meaning: "Observation epoch validation failed",
        mitigation: "Inspect observation ordering, IDs, and time tags.",
    },
    DiagnosticCode {
        code: "NAV_EPHEMERIS_GAP",
        severity: DiagnosticSeverity::Warning,
        meaning: "Ephemeris coverage gap detected",
        mitigation: "Provide a dataset with complete ephemeris coverage.",
    },
    DiagnosticCode {
        code: "TRACK_LOSS_OF_LOCK",
        severity: DiagnosticSeverity::Warning,
        meaning: "Tracking reported loss of lock",
        mitigation: "Inspect signal strength and tracking loop parameters.",
    },
    DiagnosticCode {
        code: "RTK_SD_CODE_INVALID",
        severity: DiagnosticSeverity::Error,
        meaning: "RTK single-difference code contains NaN/Inf",
        mitigation: "Inspect SD generation and input observables.",
    },
    DiagnosticCode {
        code: "RTK_SD_PHASE_INVALID",
        severity: DiagnosticSeverity::Error,
        meaning: "RTK single-difference phase contains NaN/Inf",
        mitigation: "Inspect SD phase formation and lock status.",
    },
    DiagnosticCode {
        code: "RTK_SD_DOPPLER_INVALID",
        severity: DiagnosticSeverity::Error,
        meaning: "RTK single-difference Doppler contains NaN/Inf",
        mitigation: "Inspect SD Doppler formation and tracking outputs.",
    },
    DiagnosticCode {
        code: "RTK_DD_CODE_INVALID",
        severity: DiagnosticSeverity::Error,
        meaning: "RTK double-difference code contains NaN/Inf",
        mitigation: "Inspect DD generation and reference selection.",
    },
    DiagnosticCode {
        code: "RTK_DD_PHASE_INVALID",
        severity: DiagnosticSeverity::Error,
        meaning: "RTK double-difference phase contains NaN/Inf",
        mitigation: "Inspect DD phase formation and ambiguity state.",
    },
    DiagnosticCode {
        code: "RTK_DD_DOPPLER_INVALID",
        severity: DiagnosticSeverity::Error,
        meaning: "RTK double-difference Doppler contains NaN/Inf",
        mitigation: "Inspect DD Doppler formation.",
    },
    DiagnosticCode {
        code: "RTK_BASELINE_INVALID",
        severity: DiagnosticSeverity::Error,
        meaning: "RTK baseline contains NaN/Inf",
        mitigation: "Inspect baseline solver and DD residuals.",
    },
    DiagnosticCode {
        code: "RTK_QUALITY_INVALID",
        severity: DiagnosticSeverity::Error,
        meaning: "RTK baseline quality metrics contain NaN/Inf",
        mitigation: "Inspect covariance propagation and residual metrics.",
    },
    DiagnosticCode {
        code: "RTK_PRECISION_INVALID",
        severity: DiagnosticSeverity::Error,
        meaning: "RTK precision metrics contain NaN/Inf",
        mitigation: "Inspect fix ratio and precision reporting.",
    },
    DiagnosticCode {
        code: "PPP_EPOCH_INVALID",
        severity: DiagnosticSeverity::Error,
        meaning: "PPP solution epoch contains NaN/Inf",
        mitigation: "Inspect PPP filter outputs and residuals.",
    },
];
