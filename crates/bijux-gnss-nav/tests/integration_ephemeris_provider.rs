#![allow(missing_docs)]
use bijux_gnss_core::api::{Constellation, SatId};
use bijux_gnss_nav::api::{CsvEphemerisProvider, EphemerisProvider};

#[test]
fn csv_ephemeris_provider_loads_entries() {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data/ephemeris.csv");
    let provider = CsvEphemerisProvider::from_csv(&path).expect("valid csv");
    let eph = provider
        .ephemeris(SatId { constellation: Constellation::Gps, prn: 7 })
        .expect("prn present");
    assert_eq!(eph.sat.prn, 7);
    assert_eq!(eph.toe_s, 2000.0);
}
