use bijux_gnss_core::api::{Constellation, GpsTime, ObsSignalTiming, SatId, Seconds};
use bijux_gnss_nav::api::{
    galileo_navigation_age, galileo_satellite_clock_correction_e1, sat_state_galileo_e1,
    sat_state_galileo_e1_from_observation, select_best_galileo_navigation,
    GalileoBroadcastNavigationData, GalileoClockCorrection, GalileoEphemeris,
    GalileoIonosphericCorrection, GalileoIonosphericDisturbanceFlags, GalileoSignalHealth,
    GalileoSystemTime,
};

fn sample_navigation() -> GalileoBroadcastNavigationData {
    GalileoBroadcastNavigationData {
        sat: SatId { constellation: Constellation::Galileo, prn: 19 },
        iodnav: 0x1A5,
        gst: GalileoSystemTime { week: 2222, tow_s: 456_789 },
        sisa_e1_e5b: 77,
        signal_health: GalileoSignalHealth {
            e5b_signal_health: 0,
            e1b_signal_health: 0,
            e5b_data_valid: true,
            e1b_data_valid: true,
        },
        clock: GalileoClockCorrection {
            t0c_s: 66_000.0,
            af0: -1.7e-4,
            af1: 2.5e-12,
            af2: -3.0e-19,
            bgd_e1_e5a_s: -1.1e-9,
            bgd_e1_e5b_s: 2.4e-9,
        },
        ephemeris: GalileoEphemeris {
            sat: SatId { constellation: Constellation::Galileo, prn: 19 },
            iodnav: 0x1A5,
            toe_s: 64_800.0,
            sqrt_a: 5_440.612_319,
            e: 0.001_23,
            i0: 0.953,
            idot: -2.1e-10,
            omega0: 1.17,
            omegadot: -5.8e-9,
            w: -0.37,
            m0: 0.84,
            delta_n: 4.7e-9,
            cuc: -3.2e-6,
            cus: 4.1e-6,
            crc: 178.0,
            crs: -91.0,
            cic: 1.9e-7,
            cis: -2.4e-7,
        },
        ionosphere: GalileoIonosphericCorrection {
            ai0: 0.0,
            ai1: 0.0,
            ai2: 0.0,
            disturbance_flags: GalileoIonosphericDisturbanceFlags {
                region_1: false,
                region_2: false,
                region_3: false,
                region_4: false,
                region_5: false,
            },
        },
    }
}

#[test]
fn public_galileo_state_api_keeps_clock_and_observation_paths_consistent() {
    let navigation = sample_navigation();
    let signal_timing = ObsSignalTiming {
        signal_travel_time_s: Seconds(0.078),
        transmit_gps_time: GpsTime { week: navigation.gst.week as u32, tow_s: 65_432.122 },
    };

    let direct_state = sat_state_galileo_e1(
        &navigation,
        signal_timing.transmit_gps_time.tow_s,
        signal_timing.signal_travel_time_s.0,
    );
    let observation_state = sat_state_galileo_e1_from_observation(
        &navigation,
        65_432.200,
        24_000_000.0,
        Some(signal_timing),
    );
    let clock =
        galileo_satellite_clock_correction_e1(&navigation, signal_timing.transmit_gps_time.tow_s);

    assert!((direct_state.x_m - observation_state.x_m).abs() < 1.0e-9);
    assert!((direct_state.y_m - observation_state.y_m).abs() < 1.0e-9);
    assert!((direct_state.z_m - observation_state.z_m).abs() < 1.0e-9);
    assert!((direct_state.clock_correction.bias_s - clock.bias_s).abs() < 1.0e-18);
}

#[test]
fn public_galileo_navigation_selection_prefers_fresh_navigation() {
    let fresh = sample_navigation();
    let mut stale = sample_navigation();
    stale.clock.t0c_s -= 18_000.0;
    stale.ephemeris.toe_s -= 18_000.0;

    let age = galileo_navigation_age(&fresh, 65_000.0);
    let candidates = [stale, fresh.clone()];
    let selected =
        select_best_galileo_navigation(&candidates, fresh.sat, 65_000.0).expect("selected nav");

    assert!(age.is_valid());
    assert_eq!(selected.iodnav, fresh.iodnav);
    assert_eq!(selected.clock.t0c_s, fresh.clock.t0c_s);
}
