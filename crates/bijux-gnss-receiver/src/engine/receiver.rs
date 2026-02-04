//! Receiver entrypoint helpers.
#![allow(missing_docs)]

use crate::api::{Receiver, ReceiverRuntimeConfig};

impl Receiver {
    /// Create a new receiver with the provided configuration.
    pub fn new(config: ReceiverRuntimeConfig) -> Self {
        Self { config }
    }

    /// Borrow the receiver configuration.
    pub fn config(&self) -> &ReceiverRuntimeConfig {
        &self.config
    }
}
