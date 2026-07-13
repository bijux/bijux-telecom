//! Estimation engines and shared helpers.
//!
//! This is the public facade for EKF, PPP, PVT, and RTK estimators.
#![allow(missing_docs)]
#![allow(dead_code)]

use bijux_gnss_core::api::SigId;

pub mod ekf;
pub mod position;
pub mod ppp;
pub mod rtk;
pub mod solution_claims;
pub(crate) mod uncertainty;

/// Ensure a deterministic ordering for measurements keyed by signal identity.
pub fn sort_signal_ids(ids: &mut [SigId]) {
    ids.sort();
}
