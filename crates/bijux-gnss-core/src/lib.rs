//! Core GNSS pipeline contracts and error taxonomy.
//!
//! Module map:
//! - `ids`: constellation, satellite, and signal identity types
//! - `time`: GPS/UTC/receiver time definitions and leap seconds
//! - `obs`: samples, acquisition, tracking, and observation contracts
//! - `obs` also defines error taxonomy for pipeline stages
//! - Start here: `ObsEpoch` and `SamplesFrame`
#![deny(clippy::unwrap_used)]

mod ids;
mod obs;
mod time;

pub use ids::*;
pub use obs::*;
pub use time::*;
