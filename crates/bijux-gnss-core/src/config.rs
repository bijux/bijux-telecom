#![allow(missing_docs)]
use serde::{Deserialize, Serialize};

use crate::error::ConfigError;

/// Schema version for GNSS configuration files.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct SchemaVersion(pub u32);

/// Validation result for configuration structs.
#[derive(Debug, Clone, Default)]
pub struct ValidationReport {
    /// Hard errors that should prevent execution.
    pub errors: Vec<ConfigError>,
    /// Warnings that should be surfaced but do not block execution.
    pub warnings: Vec<String>,
}

/// Trait for validating configuration structs.
pub trait ValidateConfig {
    /// Validate the configuration and return typed errors and warnings.
    fn validate(&self) -> ValidationReport;
}

/// Root GNSS configuration composed of receiver + navigation sections.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BijuxGnssConfig<R, N> {
    /// Schema version for compatibility and migrations.
    pub schema_version: SchemaVersion,
    /// Receiver configuration section.
    pub receiver: R,
    /// Navigation configuration section.
    pub navigation: N,
}

impl SchemaVersion {
    /// Current schema version for GNSS configs.
    pub const CURRENT: SchemaVersion = SchemaVersion(1);
}

impl<R, N> BijuxGnssConfig<R, N>
where
    R: ValidateConfig,
    N: ValidateConfig,
{
    /// Validate both receiver and navigation configs.
    pub fn validate(&self) -> ValidationReport {
        let mut report = ValidationReport::default();
        let recv = self.receiver.validate();
        let nav = self.navigation.validate();
        report.errors.extend(recv.errors);
        report.errors.extend(nav.errors);
        report.warnings.extend(recv.warnings);
        report.warnings.extend(nav.warnings);
        report
    }
}
