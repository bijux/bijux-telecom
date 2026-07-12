#![allow(missing_docs)]
mod support;

use bijux_gnss_nav::api::{geodetic_to_ecef, GpsEphemeris, PositionFilter, PositionFilterConfig};

use support::position_truth::{
    pseudorange_from_truth, sample_ephemeris, timed_position_observation,
};

struct SequentialPositionEpoch {
    t_rx_s: f64,
    truth_ecef_m: (f64, f64, f64),
    observations: Vec<bijux_gnss_nav::api::PositionObservation>,
}

fn position_error_3d_m(
    ecef_x_m: f64,
    ecef_y_m: f64,
    ecef_z_m: f64,
    truth_ecef_m: (f64, f64, f64),
) -> f64 {
    let dx = ecef_x_m - truth_ecef_m.0;
    let dy = ecef_y_m - truth_ecef_m.1;
    let dz = ecef_z_m - truth_ecef_m.2;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn sample_filter_ephemerides() -> Vec<GpsEphemeris> {
    let mut ephemerides = vec![
        sample_ephemeris(1, 0.0, 0.0),
        sample_ephemeris(2, 0.8, 0.9),
        sample_ephemeris(3, 1.6, 1.8),
        sample_ephemeris(4, 2.4, 2.7),
        sample_ephemeris(5, 3.2, 3.4),
    ];
    for ephemeris in &mut ephemerides {
        ephemeris.toe_s = 504_000.0;
        ephemeris.toc_s = 504_018.0;
    }
    ephemerides
}

fn deterministic_pseudorange_offset_m(epoch_index: usize, prn: u8) -> f64 {
    let pattern = ((epoch_index * 17 + prn as usize * 13) % 7) as f64 - 3.0;
    pattern * 0.35
}

fn static_receiver_epochs(
    ephemerides: &[GpsEphemeris],
    epoch_count: usize,
    dt_s: f64,
) -> Vec<SequentialPositionEpoch> {
    let truth_ecef_m = geodetic_to_ecef(37.0, -122.0, 10.0);
    let receiver_clock_bias_s = 2.75e-4;
    let t0_rx_s = 504_018.07 + receiver_clock_bias_s;

    (0..epoch_count)
        .map(|epoch_index| {
            let t_rx_s = t0_rx_s + epoch_index as f64 * dt_s;
            let observations = ephemerides
                .iter()
                .map(|ephemeris| {
                    let pseudorange_m =
                        pseudorange_from_truth(
                            ephemeris,
                            truth_ecef_m,
                            t_rx_s,
                            receiver_clock_bias_s,
                        ) + deterministic_pseudorange_offset_m(epoch_index, ephemeris.sat.prn);
                    timed_position_observation(ephemeris.sat, pseudorange_m, t_rx_s)
                })
                .collect();
            SequentialPositionEpoch { t_rx_s, truth_ecef_m, observations }
        })
        .collect()
}

#[test]
fn sequential_position_filter_tracks_static_receiver_across_epochs() {
    let ephemerides = sample_filter_ephemerides();
    let epochs = static_receiver_epochs(&ephemerides, 6, 1.0);
    let mut filter = PositionFilter::new(PositionFilterConfig::default());
    let mut last_velocity_norm_mps = None;

    for epoch in &epochs {
        let solution = filter
            .solve_epoch(&epoch.observations, &ephemerides, epoch.t_rx_s)
            .expect("static epoch should solve");
        let position_error_m = position_error_3d_m(
            solution.ecef_x_m,
            solution.ecef_y_m,
            solution.ecef_z_m,
            epoch.truth_ecef_m,
        );
        let velocity_norm_mps = (solution.velocity_x_mps * solution.velocity_x_mps
            + solution.velocity_y_mps * solution.velocity_y_mps
            + solution.velocity_z_mps * solution.velocity_z_mps)
            .sqrt();

        assert!(position_error_m < 12.0, "position_error_m={position_error_m}");
        assert!(solution.used_sat_count >= 4);
        last_velocity_norm_mps = Some(velocity_norm_mps);
    }

    assert!(filter.initialized);
    assert_eq!(filter.last_t_rx_s, Some(epochs.last().expect("static epochs").t_rx_s));
    assert!(last_velocity_norm_mps.expect("final velocity norm") < 1.0);
}
