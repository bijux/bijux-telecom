use std::path::PathBuf;

use bijux_gnss_core::api::{Constellation, SatId, SignalBand, SignalCode};
use bijux_gnss_signal::api::{
    check_dual_frequency_observations, signal_spec_beidou_b1i, signal_spec_beidou_b2i,
    signal_spec_galileo_e1b, signal_spec_galileo_e5a, signal_spec_gps_l2c, signal_spec_gps_l5,
    validate_obs_epochs,
};

use super::{
    format_rinex_observation_dataset, parse_observation_cells, parse_rinex_2_observation_records,
    parse_rinex_beidou_observation_dataset, parse_rinex_galileo_observation_dataset,
    parse_rinex_gps_observation_dataset, parse_rinex_observation_dataset,
    parse_rinex_observation_header, RinexCodeBiasState, RinexObservationRecord,
    RinexObservationTimeSystem,
};

fn fixture(name: &str) -> String {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data").join(name);
    std::fs::read_to_string(&path)
        .unwrap_or_else(|_| panic!("read RINEX fixture {}", path.display()))
}

mod gps_observations;
mod header_parsing;
mod multi_constellation_observations;
mod raw_datasets;
mod record_parsing;
