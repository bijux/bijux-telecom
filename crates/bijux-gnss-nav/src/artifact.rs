//! Versioned artifact wrappers for navigation outputs.

use bijux_gnss_core::ArtifactHeaderV1;
use serde::{Deserialize, Serialize};

use crate::{GpsEphemeris, PppSolutionEpoch};
use bijux_gnss_core::{ArtifactValidate, DiagnosticEvent, DiagnosticSeverity};

/// Versioned ephemeris artifact (v1).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GpsEphemerisV1 {
    /// Artifact header.
    pub header: ArtifactHeaderV1,
    /// Ephemeris payload.
    pub ephemerides: Vec<GpsEphemeris>,
}

/// Versioned PPP solution artifact (v1).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PppEpochV1 {
    /// Artifact header.
    pub header: ArtifactHeaderV1,
    /// PPP solution payload.
    pub epoch: PppSolutionEpoch,
}

impl ArtifactValidate for PppEpochV1 {
    fn validate(&self) -> Vec<DiagnosticEvent> {
        let mut events = Vec::new();
        let epoch = &self.epoch;
        for value in [
            epoch.t_rx_s,
            epoch.ecef_x_m,
            epoch.ecef_y_m,
            epoch.ecef_z_m,
            epoch.clock_bias_s,
            epoch.rms_m,
            epoch.innovation_rms,
        ] {
            if !value.is_finite() {
                events.push(DiagnosticEvent::new(
                    DiagnosticSeverity::Error,
                    "PPP_EPOCH_INVALID",
                    "ppp epoch contains NaN/Inf",
                ));
                break;
            }
        }
        events
    }
}
