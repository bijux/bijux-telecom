#![allow(missing_docs)]

pub mod ambiguity;
pub mod core;
pub mod differencing;
pub mod metrics;

pub use bijux_gnss_core::rtk::{
    RtkBaselineEpochV1, RtkBaselineQualityV1, RtkDdEpochV1, RtkFixAuditV1, RtkPrecisionV1,
    RtkSdEpochV1,
};

pub type RtkSdEpoch = RtkSdEpochV1<core::SdObservation>;
pub type RtkDdEpoch = RtkDdEpochV1<core::DdObservation>;
pub type RtkBaselineEpoch = RtkBaselineEpochV1<metrics::BaselineSolution>;
pub type RtkBaselineQualityEpoch = RtkBaselineQualityV1<metrics::RtkBaselineQuality>;
pub type RtkFixAuditEpoch = RtkFixAuditV1<ambiguity::FixAuditEvent>;
pub type RtkPrecisionEpoch = RtkPrecisionV1<metrics::RtkPrecision>;
