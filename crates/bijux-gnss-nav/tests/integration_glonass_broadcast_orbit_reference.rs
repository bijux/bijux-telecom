#![allow(missing_docs)]

use std::path::PathBuf;

use bijux_gnss_core::api::{Constellation, SatId};
use bijux_gnss_nav::api::{
    sat_state_glonass_l1, GlonassAlmanacTimeData, GlonassBroadcastNavigationFrame,
    GlonassFrameTime, GlonassImmediateHealth, GlonassImmediateNavigationData,
    GlonassSatelliteType, GlonassStateVector, GlonassSystemTime, Sp3Provider,
};

const GLONASS_BROADCAST_POSITION_TOLERANCE_M: f64 = 300.0;

#[test]
fn glonass_broadcast_position_matches_precise_reference_within_tolerance() {
    let navigation = sample_navigation();
    let precise_orbit = load_sp3("glonass_slot14_20220513_esa_reference.sp3");
    let mut max_position_error_m = 0.0_f64;

    for (transmit_tow_s, reference_t_s) in
        [(504_618.0, 1_518.0), (504_918.0, 1_818.0), (505_218.0, 2_118.0)]
    {
        let broadcast =
            sat_state_glonass_l1(&navigation, transmit_tow_s, 0.0).expect("broadcast state");
        let precise = precise_orbit
            .sat_state(navigation.sat, reference_t_s)
            .expect("SP3 state for GLONASS broadcast comparison");
        let dx_m = broadcast.x_m - precise.x_m;
        let dy_m = broadcast.y_m - precise.y_m;
        let dz_m = broadcast.z_m - precise.z_m;
        let position_error_m = (dx_m * dx_m + dy_m * dy_m + dz_m * dz_m).sqrt();
        max_position_error_m = max_position_error_m.max(position_error_m);

    }

    assert!(
        max_position_error_m <= GLONASS_BROADCAST_POSITION_TOLERANCE_M,
        "GLONASS slot 14 drifted {:.3} m against the ESA precise orbit over the sampled window",
        max_position_error_m
    );
}

fn sample_navigation() -> GlonassBroadcastNavigationFrame {
    let sat = SatId { constellation: Constellation::Glonass, prn: 14 };
    GlonassBroadcastNavigationFrame {
        sat,
        immediate: GlonassImmediateNavigationData {
            sat,
            frame_time: GlonassFrameTime { hour: 23, minute: 18, half_minute: false },
            ephemeris_reference_time_s: 83_700,
            tb_update_interval_min: 30,
            tb_is_odd: Some(true),
            state_vector: GlonassStateVector {
                x_m: -7_557_760.253_906_25,
                y_m: -23_962_225.585_937_5,
                z_m: -4_337_567.871_093_75,
                vx_mps: 101.318_359_375,
                vy_mps: 602.112_770_080_566_4,
                vz_mps: -3_495.733_261_108_398_4,
                ax_mps2: -3.725_290_298_461_914e-6,
                ay_mps2: 0.0,
                az_mps2: 1.862_645_149_230_957e-6,
            },
            relative_frequency_bias: 0.0,
            clock_bias_s: -2.572_406_083_345_413_2e-5,
            l2_l1_delay_s: Some(5.587_935_448e-9),
            health: GlonassImmediateHealth { line_unhealthy: false, status_code: 0 },
            immediate_data_age_days: 28,
            satellite_type: GlonassSatelliteType::GlonassM,
            reported_slot: None,
            system_time: Some(GlonassSystemTime { day_number: 864, four_year_interval: Some(8) }),
            accuracy_code: Some(2),
        },
        system_time: Some(GlonassAlmanacTimeData {
            system_time: GlonassSystemTime { day_number: 864, four_year_interval: Some(8) },
            utc_offset_s: 0.0,
            gps_minus_glonass_s: -10_782.0,
        }),
        almanac_entries: Vec::new(),
    }
}

fn load_sp3(name: &str) -> Sp3Provider {
    read_fixture(name).parse().unwrap_or_else(|_| panic!("parse SP3 fixture {name}"))
}

fn read_fixture(name: &str) -> String {
    std::fs::read_to_string(fixture_path(name))
        .unwrap_or_else(|_| panic!("read fixture {}", fixture_path(name).display()))
}

fn fixture_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data").join(name)
}
