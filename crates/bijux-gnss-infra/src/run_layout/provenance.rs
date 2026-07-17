//! Run provenance helpers.

mod feature_inventory;
mod front_end;
mod replay_context;

use crate::run_layout::directories::RunContextArgs;
use bijux_gnss_receiver::api::ReceiverConfig;

use crate::datasets::DatasetEntry;

pub(crate) type ReplayScope = replay_context::ReplayScope;
pub(crate) type FrontEndProvenance = front_end::FrontEndProvenance;

pub(crate) fn replay_scope(args: &RunContextArgs<'_>) -> ReplayScope {
    replay_context::replay_scope(args)
}

pub(crate) fn front_end_provenance(
    args: &RunContextArgs<'_>,
    profile: &ReceiverConfig,
    dataset: Option<&DatasetEntry>,
) -> Result<FrontEndProvenance, bijux_gnss_receiver::api::core::InputError> {
    front_end::front_end_provenance(args, profile, dataset)
}

pub(crate) fn enabled_features() -> Vec<String> {
    feature_inventory::enabled_features()
}
