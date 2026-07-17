#![allow(missing_docs)]

mod navigation_format;
mod navigation_parse;
mod navigation_types;
mod observation_export;

pub type RinexBroadcastNavigationDataset = navigation_types::RinexBroadcastNavigationDataset;
pub type RinexNavigationTimeSystemCorrection =
    navigation_types::RinexNavigationTimeSystemCorrection;

pub fn format_rinex_navigation_dataset(
    dataset: &RinexBroadcastNavigationDataset,
) -> Result<String, bijux_gnss_core::api::IoError> {
    navigation_format::format_rinex_navigation_dataset(dataset)
}

pub fn write_rinex_broadcast_navigation(
    path: &std::path::Path,
    navigation: &crate::orbits::gps::GpsBroadcastNavigationData,
    strict: bool,
) -> Result<(), bijux_gnss_core::api::IoError> {
    navigation_format::write_rinex_broadcast_navigation(path, navigation, strict)
}

pub fn write_rinex_nav(
    path: &std::path::Path,
    ephs: &[crate::orbits::gps::GpsEphemeris],
    strict: bool,
) -> Result<(), bijux_gnss_core::api::IoError> {
    navigation_format::write_rinex_nav(path, ephs, strict)
}

pub fn write_rinex_navigation_dataset(
    path: &std::path::Path,
    dataset: &RinexBroadcastNavigationDataset,
    strict: bool,
) -> Result<(), bijux_gnss_core::api::IoError> {
    navigation_format::write_rinex_navigation_dataset(path, dataset, strict)
}

pub fn parse_rinex_broadcast_navigation(
    data: &str,
) -> Result<crate::orbits::gps::GpsBroadcastNavigationData, bijux_gnss_core::api::ParseError> {
    navigation_parse::parse_rinex_broadcast_navigation(data)
}

pub fn parse_rinex_nav(
    data: &str,
) -> Result<Vec<crate::orbits::gps::GpsEphemeris>, bijux_gnss_core::api::ParseError> {
    navigation_parse::parse_rinex_nav(data)
}

pub fn parse_rinex_navigation_dataset(
    data: &str,
) -> Result<RinexBroadcastNavigationDataset, bijux_gnss_core::api::ParseError> {
    navigation_parse::parse_rinex_navigation_dataset(data)
}

pub fn parse_rinex_obs_header(data: &str) -> Result<(), bijux_gnss_core::api::ParseError> {
    observation_export::parse_rinex_obs_header(data)
}

pub fn write_rinex_obs(
    path: &std::path::Path,
    epochs: &[bijux_gnss_core::api::ObsEpoch],
    strict: bool,
) -> Result<(), bijux_gnss_core::api::IoError> {
    observation_export::write_rinex_obs(path, epochs, strict)
}

#[cfg(test)]
use navigation_format::format_rinex_nav_float;
#[cfg(test)]
use navigation_parse::{
    parse_rinex_epoch_utc, parse_rinex_epoch_utc_civil, parse_rinex_float, parse_rinex_nav_header,
    parse_rinex_numeric_fields,
};

#[cfg(test)]
mod tests;
