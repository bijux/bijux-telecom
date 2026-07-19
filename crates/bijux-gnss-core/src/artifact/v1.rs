//! Versioned v1 artifact contracts.

pub mod acquisition;
pub mod navigation;
pub mod observation;
mod receiver_trace;
pub mod support_matrix;
pub mod tracking;

use receiver_trace::validate_receiver_sample_trace;
