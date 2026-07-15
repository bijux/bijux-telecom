use super::*;

#[test]
fn constellation_residual_rms_tracks_pre_fit_and_post_fit_groups() {
    let gps_sat = SatId { constellation: Constellation::Gps, prn: 3 };
    let galileo_sat = SatId { constellation: Constellation::Galileo, prn: 19 };
    let pre_fit = vec![
        WorkingSetResidual {
            sat: gps_sat,
            residual_m: 3.0,
            base_weight: 1.0,
            effective_weight: 1.0,
        },
        WorkingSetResidual {
            sat: gps_sat,
            residual_m: 4.0,
            base_weight: 1.0,
            effective_weight: 1.0,
        },
        WorkingSetResidual {
            sat: galileo_sat,
            residual_m: 12.0,
            base_weight: 1.0,
            effective_weight: 1.0,
        },
    ];
    let post_fit = vec![
        (
            PositionObservation {
                sat: gps_sat,
                pseudorange_m: 24_000_000.0,
                doppler_hz: None,
                doppler_var_hz2: None,
                cn0_dbhz: 45.0,
                elevation_deg: Some(45.0),
                weight: 1.0,
                gps_receive_time: None,
                signal_timing: None,
                signal_id: None,
            },
            SatelliteState {
                x_m: 0.0,
                y_m: 0.0,
                z_m: 0.0,
                vx_mps: 0.0,
                vy_mps: 0.0,
                vz_mps: 0.0,
                clock_bias_s: 0.0,
                clock_drift_s_per_s: 0.0,
                uncertainty: SatelliteStateUncertainty::unavailable(),
            },
            1.5,
            1.0,
        ),
        (
            PositionObservation {
                sat: galileo_sat,
                pseudorange_m: 24_100_000.0,
                doppler_hz: None,
                doppler_var_hz2: None,
                cn0_dbhz: 45.0,
                elevation_deg: Some(50.0),
                weight: 1.0,
                gps_receive_time: None,
                signal_timing: None,
                signal_id: None,
            },
            SatelliteState {
                x_m: 0.0,
                y_m: 0.0,
                z_m: 0.0,
                vx_mps: 0.0,
                vy_mps: 0.0,
                vz_mps: 0.0,
                clock_bias_s: 0.0,
                clock_drift_s_per_s: 0.0,
                uncertainty: SatelliteStateUncertainty::unavailable(),
            },
            2.0,
            1.0,
        ),
    ];

    let summaries = constellation_residual_rms(&pre_fit, &post_fit);

    assert_eq!(summaries.len(), 2);
    let gps = summaries
        .iter()
        .find(|summary| summary.constellation == Constellation::Gps)
        .expect("gps summary");
    let galileo = summaries
        .iter()
        .find(|summary| summary.constellation == Constellation::Galileo)
        .expect("galileo summary");
    assert_eq!(gps.pre_fit_sat_count, 2);
    assert_eq!(gps.post_fit_sat_count, 1);
    assert!((gps.pre_fit_rms_m.expect("gps pre-fit").0 - 3.535_533_905_9).abs() < 1.0e-9);
    assert!((gps.post_fit_rms_m.expect("gps post-fit").0 - 1.5).abs() < 1.0e-12);
    assert_eq!(galileo.pre_fit_sat_count, 1);
    assert_eq!(galileo.post_fit_sat_count, 1);
    assert!((galileo.pre_fit_rms_m.expect("galileo pre-fit").0 - 12.0).abs() < 1.0e-12);
    assert!((galileo.post_fit_rms_m.expect("galileo post-fit").0 - 2.0).abs() < 1.0e-12);
}
