//! Receiver entrypoint helpers.
#![allow(missing_docs)]

use crate::api::{Receiver, ReceiverPipelineConfig, ReceiverRuntime};

impl Receiver {
    /// Create a new receiver with the provided configuration.
    pub fn new(config: ReceiverPipelineConfig, runtime: ReceiverRuntime) -> Self {
        Self { config, runtime }
    }

    /// Borrow the receiver configuration.
    pub fn config(&self) -> &ReceiverPipelineConfig {
        &self.config
    }

    /// Borrow the receiver runtime options.
    pub fn runtime(&self) -> &ReceiverRuntime {
        &self.runtime
    }
}
