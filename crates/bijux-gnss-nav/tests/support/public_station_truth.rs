#![allow(dead_code)]
#![allow(missing_docs)]

use std::path::PathBuf;
use std::sync::OnceLock;

use bijux_gnss_testkit::public_station::{
    load_public_station_truths,
    public_station_truth_by_fixture as find_public_station_truth_by_fixture,
};
#[allow(unused_imports)]
pub use bijux_gnss_testkit::public_station::{station_enu_error_m, EnuError, PublicStationTruth};

pub fn public_station_truth_by_fixture(fixture_name: &str) -> PublicStationTruth {
    find_public_station_truth_by_fixture(load_truths(), fixture_name)
        .unwrap_or_else(|_| panic!("missing public station truth for fixture {fixture_name}"))
        .clone()
}

fn load_truths() -> &'static Vec<PublicStationTruth> {
    static TRUTHS: OnceLock<Vec<PublicStationTruth>> = OnceLock::new();
    TRUTHS.get_or_init(|| {
        load_public_station_truths(&fixture_path("public_station_truth.csv"))
            .expect("parse public station truth manifest")
    })
}

fn fixture_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data").join(name)
}
