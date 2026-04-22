//! Public façade for the bijux GNSS stack.
#![forbid(unsafe_code)]
#![cfg_attr(docsrs, feature(doc_cfg))]

/// Core domain types and schemas.
pub use bijux_gnss_core as core;
/// Receiver pipeline API and engines.
pub use bijux_gnss_receiver as receiver;
/// Signal processing primitives and generators.
pub use bijux_gnss_signal as signal;

/// Navigation API.
#[cfg(feature = "nav")]
#[cfg_attr(docsrs, doc(cfg(feature = "nav")))]
pub use bijux_gnss_nav as nav;
