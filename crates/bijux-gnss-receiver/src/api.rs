//! Public API for bijux-gnss-receiver.

/// Receiver configuration and schema.
pub use crate::runtime::receiver_config::{ReceiverConfig, ReceiverError, ReceiverProfile};
/// Receiver pipeline entrypoint.
pub use crate::Receiver;

/// I/O helpers.
pub mod data {
    pub use crate::io::data::{FileSamples, MemorySamples, SampleSource, SampleSourceError};
}

/// Runtime logging helpers.
pub mod logging {
    pub use crate::runtime::logging::*;
}

/// Stage modules for acquisition, tracking, observations, and navigation.
pub mod acquisition {
    pub use crate::stages::acquisition::*;
}

/// Tracking stage helpers.
pub mod tracking {
    pub use crate::stages::tracking::*;
}

/// Observation-building helpers.
pub mod observations {
    pub use crate::stages::observations::*;
}

/// Navigation bridge helpers.
pub mod navigation {
    pub use crate::stages::navigation::*;
}

/// RTK differencing and baseline helpers.
pub mod rtk {
    pub use crate::rtk_internal::ambiguity::*;
    pub use crate::rtk_internal::artifact::*;
    pub use crate::rtk_internal::core::*;
    pub use crate::rtk_internal::differencing::*;
    pub use crate::rtk_internal::metrics::*;
}

/// Synthetic signal generation for tests and demos.
pub mod sim {
    pub use crate::sim_internal::synthetic::*;
}
