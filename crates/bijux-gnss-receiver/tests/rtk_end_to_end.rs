use bijux_gnss_nav::{
    geodetic_to_ecef, sat_state_gps_l1ca, GpsEphemeris, PositionObservation, PositionSolver,
};
use bijux_gnss_receiver::rtk::baseline_from_ecef;

fn make_eph(prn: u8, omega0: f64, m0: f64) -> GpsEphemeris {
    GpsEphemeris {
        prn,
        iodc: 0,
        iode: 0,
        week: 0,
        toe_s: 0.0,
        toc_s: 0.0,
        sqrt_a: 5153.7954775,
        e: 0.01,
        i0: 0.94,
        idot: 0.0,
        omega0,
        omegadot: 0.0,
        w: 0.0,
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

#[test]
fn float_baseline_close_to_truth() {
    let base = geodetic_to_ecef(37.0, -122.0, 10.0);
    let rover = geodetic_to_ecef(37.0001, -121.9999, 10.0);
    let t_rx_s = 100_000.0;
    let ephs = vec![
        make_eph(1, 0.0, 0.0),
        make_eph(2, 0.8, 0.9),
        make_eph(3, 1.6, 1.8),
        make_eph(4, 2.4, 2.7),
        make_eph(5, 3.2, 3.6),
    ];
    let mut obs = Vec::new();
    for eph in &ephs {
        let sat = sat_state_gps_l1ca(eph, t_rx_s, 0.07);
        let dx = rover.0 - sat.x_m;
        let dy = rover.1 - sat.y_m;
        let dz = rover.2 - sat.z_m;
        let range = (dx * dx + dy * dy + dz * dz).sqrt();
        obs.push(PositionObservation {
            prn: eph.prn,
            pseudorange_m: range + 299_792_458.0 * (0.0 - sat.clock_bias_s),
            cn0_dbhz: 45.0,
            elevation_deg: None,
            weight: 1.0,
        });
    }
    let solver = PositionSolver::new();
    let solution = solver.solve_wls(&obs, &ephs, t_rx_s).expect("solution");
    let baseline = baseline_from_ecef(
        [base.0, base.1, base.2],
        [solution.ecef_x_m, solution.ecef_y_m, solution.ecef_z_m],
    );
    assert!(baseline.enu_m[0].abs() < 500.0);
}
