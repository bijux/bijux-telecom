//! Estimation engines and shared helpers.
//!
//! This is the public facade for EKF, PPP, and PVT estimators.
#![allow(missing_docs)]
#![allow(dead_code)]

use bijux_gnss_core::SigId;

pub mod ekf;
pub mod position;
pub mod ppp;

/// Ensure a deterministic ordering for measurements keyed by signal identity.
pub fn sort_signal_ids(ids: &mut [SigId]) {
    ids.sort();
}
