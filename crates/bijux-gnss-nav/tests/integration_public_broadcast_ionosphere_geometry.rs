#![allow(missing_docs)]

mod support;

use std::collections::BTreeSet;

use support::public_broadcast_ionosphere_case::valid_broadcast_residuals;

#[test]
fn public_gps_broadcast_ionosphere_residuals_span_multiple_epochs_and_satellites() {
    let valid = valid_broadcast_residuals();
    let epoch_indices = valid.iter().map(|observation| observation.epoch_idx).collect::<BTreeSet<_>>();
    let satellites = valid.iter().map(|observation| observation.sat).collect::<BTreeSet<_>>();

    assert!(epoch_indices.len() > 5, "expected residuals across multiple AB43 epochs");
    assert!(satellites.len() > 5, "expected residuals across multiple AB43 satellites");
}

#[test]
fn public_gps_broadcast_ionosphere_residuals_preserve_valid_geometry_and_delay_envelopes() {
    let valid = valid_broadcast_residuals();

    assert!(
        valid.iter().all(|observation| {
            observation.azimuth_deg.is_some()
                && observation.elevation_deg.is_some()
                && observation.broadcast_delay_band_1_m.is_some()
                && observation.broadcast_delay_band_2_m.is_some()
        }),
        "expected valid broadcast residuals to preserve geometry and broadcast delay estimates",
    );
    assert!(
        valid.iter().all(|observation| {
            let azimuth_deg = observation.azimuth_deg.expect("azimuth");
            let elevation_deg = observation.elevation_deg.expect("elevation");
            let delay_band_1_m = observation.broadcast_delay_band_1_m.expect("L1 broadcast delay");
            let delay_band_2_m = observation.broadcast_delay_band_2_m.expect("L2 broadcast delay");

            azimuth_deg.is_finite()
                && elevation_deg.is_finite()
                && delay_band_1_m.is_finite()
                && delay_band_2_m.is_finite()
                && (0.0..=360.0).contains(&azimuth_deg)
                && elevation_deg > 0.0
                && elevation_deg <= 90.0
                && delay_band_1_m > 0.0
                && delay_band_2_m > delay_band_1_m
        }),
        "expected AB43 broadcast residuals to carry physical geometry and dispersive delays",
    );
}
