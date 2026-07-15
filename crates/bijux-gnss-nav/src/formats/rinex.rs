#![allow(missing_docs)]

mod navigation_format;
mod navigation_parse;
mod navigation_types;
mod observation_export;

pub use navigation_format::{
    format_rinex_navigation_dataset, write_rinex_broadcast_navigation, write_rinex_nav,
    write_rinex_navigation_dataset,
};
pub use navigation_parse::{
    parse_rinex_broadcast_navigation, parse_rinex_nav, parse_rinex_navigation_dataset,
};
pub use navigation_types::{RinexBroadcastNavigationDataset, RinexNavigationTimeSystemCorrection};
pub use observation_export::{parse_rinex_obs_header, write_rinex_obs};

#[cfg(test)]
use navigation_format::format_rinex_nav_float;
#[cfg(test)]
use navigation_parse::{
    parse_rinex_epoch_utc, parse_rinex_epoch_utc_civil, parse_rinex_float, parse_rinex_nav_header,
    parse_rinex_numeric_fields,
};

#[cfg(test)]
mod tests;
