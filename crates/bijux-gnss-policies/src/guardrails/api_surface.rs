//! API-surface and public-item checks for guardrail evaluation.

mod api_contract;
mod public_exports;

pub(crate) use api_contract::check_api_purity;
pub(crate) use public_exports::{
    check_pub_items, check_pub_items_outside_api, check_pub_use_locations_if_enabled,
    check_pub_use_spam,
};
