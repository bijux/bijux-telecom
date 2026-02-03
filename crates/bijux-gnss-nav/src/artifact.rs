//! Versioned artifact wrappers for navigation outputs.

use bijux_gnss_core::ArtifactHeaderV1;
use serde::{Deserialize, Serialize};

use crate::{GpsEphemeris, PppSolutionEpoch};

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
pub struct PppSolutionEpochV1 {
    /// Artifact header.
    pub header: ArtifactHeaderV1,
    /// PPP solution payload.
    pub epoch: PppSolutionEpoch,
}
