#![allow(missing_docs)]
use bijux_gnss_core::{Constellation, SatId};
use bijux_gnss_nav::{
    geodetic_to_ecef, sat_state_gps_l1ca, GpsEphemeris, PositionObservation, PositionSolver,
};

fn make_eph(prn: u8, omega0: f64, m0: f64) -> GpsEphemeris {
    GpsEphemeris {
        sat: SatId {
            constellation: Constellation::Gps,
            prn,
        },
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
fn fault_injection_rejects_bad_pseudorange() {
    let (rx_x, rx_y, rx_z) = geodetic_to_ecef(37.0, -122.0, 10.0);
    let t_rx_s = 100_000.0;

    let ephs = vec![
        make_eph(1, 0.0, 0.0),
        make_eph(2, 0.8, 0.9),
        make_eph(3, 1.6, 1.8),
        make_eph(4, 2.4, 2.7),
        make_eph(5, 3.2, 3.6),
        make_eph(6, 4.0, 4.5),
    ];

    let mut obs = Vec::new();
    for eph in &ephs {
        let state = sat_state_gps_l1ca(eph, t_rx_s, 0.07);
        let dx = rx_x - state.x_m;
        let dy = rx_y - state.y_m;
        let dz = rx_z - state.z_m;
        let range = (dx * dx + dy * dy + dz * dz).sqrt();
        let mut pr = range + 299_792_458.0 * (0.0 - state.clock_bias_s);
        if eph.sat.prn == 3 {
            pr += 1000.0;
        }
        obs.push(PositionObservation {
            sat: eph.sat,
            pseudorange_m: pr,
            cn0_dbhz: 45.0,
            elevation_deg: None,
            weight: 1.0,
        });
    }

    let solver = PositionSolver {
        raim: true,
        residual_gate_m: 300.0,
        ..PositionSolver::new()
    };
    let solution = solver.solve_wls(&obs, &ephs, t_rx_s).expect("solution");
    assert!(solution.rejected.contains(&SatId {
        constellation: Constellation::Gps,
        prn: 3,
    }));
}
