//! Versioned artifact wrappers for navigation outputs.

use bijux_gnss_core::ArtifactHeader;
use serde::{Deserialize, Serialize};

use crate::{GpsEphemeris, PppSolutionEpoch};

/// Versioned ephemeris artifact (v1).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GpsEphemerisV1 {
    /// Artifact header.
    pub header: ArtifactHeader,
    /// Ephemeris payload.
    pub ephemerides: Vec<GpsEphemeris>,
}

/// Versioned PPP solution artifact (v1).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PppSolutionEpochV1 {
    /// Artifact header.
    pub header: ArtifactHeader,
    /// PPP solution payload.
    pub epoch: PppSolutionEpoch,
}
