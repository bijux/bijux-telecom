//! Run identity helpers.

mod fingerprint;
mod schema_versions;
mod timestamp;

pub(crate) use fingerprint::{dataset_hash, run_id};
pub(crate) use schema_versions::{RUN_LAYOUT_SCHEMA_VERSION, RUN_REPORT_SCHEMA_VERSION};
pub(crate) use timestamp::now_unix_ms;
