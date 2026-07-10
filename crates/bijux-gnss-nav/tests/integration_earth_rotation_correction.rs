#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SatId};
use bijux_gnss_nav::api::{
    geodetic_to_ecef, gps_earth_rotation_correction, sat_state_gps_l1ca,
    sat_state_gps_l1ca_at_receive_time, GpsEphemeris,
};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

fn make_eph(prn: u8, omega0: f64, m0: f64, toe_s: f64) -> GpsEphemeris {
    GpsEphemeris {
        sat: SatId { constellation: Constellation::Gps, prn },
        iodc: 0,
        iode: 0,
        week: 2200,
        sv_health: 0,
        toe_s,
        toc_s: toe_s,
        sqrt_a: 5153.7954775,
        e: 0.01,
        i0: 0.94,
        idot: 0.0,
        omega0,
        omegadot: 0.0,
        w: 0.15,
        m0,
        delta_n: 0.0,
        cuc: 0.0,
        cus: 0.0,
        crc: 0.0,
        crs: 0.0,
        cic: 0.0,
        cis: 0.0,
        af0: 0.0,
        af1: 0.0,
        af2: 0.0,
        tgd: 0.0,
    }
}

fn geometric_range_m(receiver_ecef_m: [f64; 3], sat_ecef_m: [f64; 3]) -> f64 {
    let dx = receiver_ecef_m[0] - sat_ecef_m[0];
    let dy = receiver_ecef_m[1] - sat_ecef_m[1];
    let dz = receiver_ecef_m[2] - sat_ecef_m[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

#[test]
fn earth_rotation_correction_changes_range_consistently_with_geometry() {
    let eph = make_eph(7, 1.4, 0.8, 345_600.0);
    let receiver = geodetic_to_ecef(37.3317, -122.0301, 18.0);
    let receiver_ecef_m = [receiver.0, receiver.1, receiver.2];
    let receive_tow_s = 345_600.08;

    let mut tau_s = 0.07;
    let corrected = loop {
        let state = sat_state_gps_l1ca_at_receive_time(&eph, receive_tow_s, tau_s);
        let range_m = geometric_range_m(receiver_ecef_m, [state.x_m, state.y_m, state.z_m]);
        let next_tau_s = range_m / SPEED_OF_LIGHT_MPS;
        if (next_tau_s - tau_s).abs() < 1.0e-12 {
            break state;
        }
        tau_s = next_tau_s;
    };

    let uncorrected = sat_state_gps_l1ca(&eph, receive_tow_s - tau_s, 0.0);
    let range_corrected_m =
        geometric_range_m(receiver_ecef_m, [corrected.x_m, corrected.y_m, corrected.z_m]);
    let range_uncorrected_m =
        geometric_range_m(receiver_ecef_m, [uncorrected.x_m, uncorrected.y_m, uncorrected.z_m]);
    let range_delta_m = range_corrected_m - range_uncorrected_m;
    let delta_pos_m = [
        corrected.x_m - uncorrected.x_m,
        corrected.y_m - uncorrected.y_m,
        corrected.z_m - uncorrected.z_m,
    ];
    let los_uncorrected = [
        (receiver_ecef_m[0] - uncorrected.x_m) / range_uncorrected_m,
        (receiver_ecef_m[1] - uncorrected.y_m) / range_uncorrected_m,
        (receiver_ecef_m[2] - uncorrected.z_m) / range_uncorrected_m,
    ];
    let projected_range_delta_m = -(los_uncorrected[0] * delta_pos_m[0]
        + los_uncorrected[1] * delta_pos_m[1]
        + los_uncorrected[2] * delta_pos_m[2]);
    let earth_rotation = gps_earth_rotation_correction(uncorrected.x_m, uncorrected.y_m, tau_s);

    assert!((delta_pos_m[0] - earth_rotation.delta_x_m).abs() < 1.0e-9);
    assert!((delta_pos_m[1] - earth_rotation.delta_y_m).abs() < 1.0e-9);
    assert!(delta_pos_m[2].abs() < 1.0e-9);
    assert!(range_delta_m.abs() > 1.0);
    assert!((range_delta_m - projected_range_delta_m).abs() < 1.0e-3);
    assert_eq!(range_delta_m.signum(), projected_range_delta_m.signum());
}
