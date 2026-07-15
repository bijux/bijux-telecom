#![allow(missing_docs)]

use crate::ids::{Constellation, SignalBand, SignalCode};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum SupportStatus {
    Supported,
    Unsupported,
    Planned,
    Deprecated,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub struct SignalStageSupport {
    pub acquisition: SupportStatus,
    pub tracking: SupportStatus,
    pub data_decoding: SupportStatus,
    pub observations: SupportStatus,
    pub positioning: SupportStatus,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SignalSupportRow {
    pub constellation: Constellation,
    pub band: SignalBand,
    pub code: SignalCode,
    pub stage_support: SignalStageSupport,
    pub requirements: Vec<String>,
    pub status: SupportStatus,
    pub reason: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SupportMatrix {
    pub schema_version: u32,
    pub rows: Vec<SignalSupportRow>,
}
