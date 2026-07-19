#![allow(dead_code, missing_docs)]

use std::path::PathBuf;

use bijux_gnss_core::api::SatId;
use bijux_gnss_nav::api::{decode_rawephem_hex, ClkProvider, GpsEphemeris, Sp3Provider};

pub struct BroadcastReferenceFixture {
    pub label: &'static str,
    pub sat: SatId,
    pub ephemeris: GpsEphemeris,
    pub sp3: Sp3Provider,
    pub clk: ClkProvider,
    pub transmit_times_s: &'static [f64],
    pub reference_times_s: &'static [f64],
}

pub fn gps_prn1_20220513_fixture() -> BroadcastReferenceFixture {
    let ephemeris = decode_rawephem_hex(
        1,
        "8b0284a1b8a52850000724918b913e21a92dc6ee0b217b0c00ffb72fac04",
        "8b0284a1b92b21fac82899520ec7b7fb31061550921d09a10d62b87b0c7c",
        "8b0284a1b9adff7d74db71f3ffaa2840ed6e0fe024cddf1effadc82106c4",
        2209,
    )
    .expect("decode PRN 1 broadcast reference ephemeris");

    BroadcastReferenceFixture {
        label: "gps_prn1_20220513",
        sat: ephemeris.sat,
        ephemeris,
        sp3: load_sp3("gps_prn1_20220513_igs_reference.sp3"),
        clk: load_clk("gps_prn1_20220513_igs_reference.clk"),
        transmit_times_s: &[503_100.0, 504_000.0, 504_900.0, 505_800.0, 506_700.0],
        reference_times_s: &[0.0, 900.0, 1_800.0, 2_700.0, 3_600.0],
    }
}

pub fn gps_prn2_20220514_fixture() -> BroadcastReferenceFixture {
    let ephemeris = decode_rawephem_hex(
        2,
        "8b0284a6e82728500051d2718b913e21a92dc6eeda427e90000001aa606d",
        "8b0284a6e8aa42f8f82d56addef337fa840a758d8d1c42a10d56437e907e",
        "8b0284a6e92f00777124a07100c7275ca76f0e76c62e1e8bffad9c42067f",
        2209,
    )
    .expect("decode PRN 2 broadcast reference ephemeris");

    BroadcastReferenceFixture {
        label: "gps_prn2_20220514",
        sat: ephemeris.sat,
        ephemeris,
        sp3: load_sp3("gps_prn2_20220514_igs_reference.sp3"),
        clk: load_clk("gps_prn2_20220514_igs_reference.clk"),
        transmit_times_s: &[518_400.0, 519_300.0, 520_200.0, 521_100.0, 522_000.0],
        reference_times_s: &[0.0, 900.0, 1_800.0, 2_700.0, 3_600.0],
    }
}

fn load_sp3(name: &str) -> Sp3Provider {
    read_fixture(name).parse().unwrap_or_else(|_| panic!("parse SP3 fixture {name}"))
}

fn load_clk(name: &str) -> ClkProvider {
    read_fixture(name).parse().unwrap_or_else(|_| panic!("parse CLK fixture {name}"))
}

fn read_fixture(name: &str) -> String {
    std::fs::read_to_string(fixture_path(name))
        .unwrap_or_else(|_| panic!("read fixture {}", fixture_path(name).display()))
}

fn fixture_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data").join(name)
}
