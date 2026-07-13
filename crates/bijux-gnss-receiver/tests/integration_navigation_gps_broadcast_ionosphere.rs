#![allow(missing_docs)]
mod support;

use bijux_gnss_core::api::{ecef_to_geodetic, Llh, ObsEpoch, Seconds};
use bijux_gnss_nav::api::{
    elevation_azimuth_deg, sat_state_gps_l1ca_from_observation, GpsBroadcastNavigationData,
    IonosphereModel, KlobucharCoefficients, KlobucharModel,
};
use bijux_gnss_receiver::api::{Navigation, NavigationFilter, Receiver, ReceiverRuntime};

use support::navigation_pipeline::{clean_synthetic_navigation_run, position_error_3d_m};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

fn sample_klobuchar_coefficients() -> KlobucharCoefficients {
    KlobucharCoefficients::new(
        [0.1212e-7, 0.1490e-7, -0.5960e-7, 0.1192e-6],
        [0.1167e6, -0.2294e6, -0.1311e6, 0.1049e7],
    )
}

fn bias_epoch_with_klobuchar(
    epoch: &ObsEpoch,
    receiver_ecef_m: (f64, f64, f64),
    navigation: &GpsBroadcastNavigationData,
) -> ObsEpoch {
    let (lat_deg, lon_deg, alt_m) =
        ecef_to_geodetic(receiver_ecef_m.0, receiver_ecef_m.1, receiver_ecef_m.2);
    let receiver = Llh { lat_deg, lon_deg, alt_m };
    let model = KlobucharModel::new(navigation.klobuchar.expect("klobuchar coefficients"));
    let mut biased = epoch.clone();

    for satellite in &mut biased.sats {
        let ephemeris = navigation
            .ephemerides
            .iter()
            .find(|ephemeris| ephemeris.sat == satellite.signal_id.sat)
            .expect("matching ephemeris");
        let state = sat_state_gps_l1ca_from_observation(
            ephemeris,
            epoch.t_rx_s.0,
            satellite.pseudorange_m.0,
            satellite.timing,
        );
        let (azimuth_deg, elevation_deg) = elevation_azimuth_deg(
            receiver_ecef_m.0,
            receiver_ecef_m.1,
            receiver_ecef_m.2,
            state.x_m,
            state.y_m,
            state.z_m,
        );
        let delay_m = model.delay_m(receiver, azimuth_deg, elevation_deg, Seconds(epoch.t_rx_s.0));
        let delay_s = delay_m / SPEED_OF_LIGHT_MPS;

        satellite.pseudorange_m.0 += delay_m;
        if let Some(timing) = &mut satellite.timing {
            timing.signal_travel_time_s.0 += delay_s;
            timing.transmit_gps_time.tow_s -= delay_s;
        }
    }

    biased
}

#[test]
fn receiver_navigation_uses_gps_broadcast_ionosphere_payload_to_recover_position() {
    let run = clean_synthetic_navigation_run();
    let observation = run.observations.first().cloned().expect("clean synthetic observation epoch");
    let navigation = GpsBroadcastNavigationData {
        ephemerides: run.profile.ephemerides.clone(),
        klobuchar: Some(sample_klobuchar_coefficients()),
    };
    let ionosphere_biased_observation =
        bias_epoch_with_klobuchar(&observation, run.profile.truth_ecef_m, &navigation);
    let mut corrected_navigation = Navigation::new(run.config.clone(), ReceiverRuntime::default());
    let mut uncorrected_navigation =
        Navigation::new(run.config.clone(), ReceiverRuntime::default());

    let corrected_solution = corrected_navigation
        .solve_epoch_with_gps_broadcast_navigation(&ionosphere_biased_observation, &navigation)
        .expect("corrected navigation solution");
    let uncorrected_solution = uncorrected_navigation
        .solve_epoch(&ionosphere_biased_observation, &navigation.ephemerides)
        .expect("uncorrected navigation solution");

    let corrected_error_m = position_error_3d_m(&corrected_solution, run.profile.truth_ecef_m);
    let uncorrected_error_m = position_error_3d_m(&uncorrected_solution, run.profile.truth_ecef_m);

    assert!(corrected_solution.valid);
    assert!(uncorrected_solution.valid);
    assert!(corrected_solution
        .explain_reasons
        .iter()
        .any(|reason| reason == "ionosphere_correction=klobuchar_broadcast"));
    assert!(!uncorrected_solution
        .explain_reasons
        .iter()
        .any(|reason| reason == "ionosphere_correction=klobuchar_broadcast"));
    assert!(corrected_error_m < 5.0, "corrected error: {corrected_error_m}");
    assert!(
        uncorrected_error_m > corrected_error_m,
        "expected payload correction to reduce position error: corrected={corrected_error_m} uncorrected={uncorrected_error_m}"
    );
}

#[test]
fn receiver_navigation_batch_runner_matches_per_epoch_gps_broadcast_navigation() {
    let run = clean_synthetic_navigation_run();
    let navigation = GpsBroadcastNavigationData {
        ephemerides: run.profile.ephemerides.clone(),
        klobuchar: Some(sample_klobuchar_coefficients()),
    };
    let ionosphere_biased_observations = run
        .observations
        .iter()
        .map(|observation| {
            bias_epoch_with_klobuchar(observation, run.profile.truth_ecef_m, &navigation)
        })
        .collect::<Vec<_>>();
    let receiver = Receiver::new(run.config.clone(), ReceiverRuntime::default());
    let batch_solutions = receiver.solve_observation_epochs_with_gps_broadcast_navigation(
        &ionosphere_biased_observations,
        &navigation,
    );
    let mut per_epoch_navigation = Navigation::new(run.config.clone(), ReceiverRuntime::default());
    let per_epoch_solutions = ionosphere_biased_observations
        .iter()
        .filter_map(|observation| {
            per_epoch_navigation.solve_epoch_with_gps_broadcast_navigation(observation, &navigation)
        })
        .collect::<Vec<_>>();

    assert_eq!(batch_solutions.len(), per_epoch_solutions.len());
    for (batch, per_epoch) in batch_solutions.iter().zip(per_epoch_solutions.iter()) {
        assert_eq!(batch.epoch.index, per_epoch.epoch.index);
        assert_eq!(batch.status, per_epoch.status);
        assert_eq!(batch.explain_decision, per_epoch.explain_decision);
        assert_eq!(batch.explain_reasons, per_epoch.explain_reasons);
        assert_eq!(batch.used_sat_count, per_epoch.used_sat_count);
        assert!((batch.ecef_x_m.0 - per_epoch.ecef_x_m.0).abs() < 1.0e-9);
        assert!((batch.ecef_y_m.0 - per_epoch.ecef_y_m.0).abs() < 1.0e-9);
        assert!((batch.ecef_z_m.0 - per_epoch.ecef_z_m.0).abs() < 1.0e-9);
    }
}

#[test]
fn receiver_navigation_batch_runner_matches_per_epoch_ephemeris_navigation() {
    let run = clean_synthetic_navigation_run();
    let receiver = Receiver::new(run.config.clone(), ReceiverRuntime::default());
    let batch_solutions =
        receiver.solve_observation_epochs(&run.observations, &run.profile.ephemerides);
    let mut per_epoch_navigation = Navigation::new(run.config.clone(), ReceiverRuntime::default());
    let per_epoch_solutions = run
        .observations
        .iter()
        .filter_map(|observation| {
            per_epoch_navigation.solve_epoch(observation, &run.profile.ephemerides)
        })
        .collect::<Vec<_>>();

    assert_eq!(batch_solutions.len(), per_epoch_solutions.len());
    for (batch, per_epoch) in batch_solutions.iter().zip(per_epoch_solutions.iter()) {
        assert_eq!(batch.epoch.index, per_epoch.epoch.index);
        assert_eq!(batch.status, per_epoch.status);
        assert_eq!(batch.explain_decision, per_epoch.explain_decision);
        assert_eq!(batch.explain_reasons, per_epoch.explain_reasons);
        assert_eq!(batch.used_sat_count, per_epoch.used_sat_count);
        assert!((batch.ecef_x_m.0 - per_epoch.ecef_x_m.0).abs() < 1.0e-9);
        assert!((batch.ecef_y_m.0 - per_epoch.ecef_y_m.0).abs() < 1.0e-9);
        assert!((batch.ecef_z_m.0 - per_epoch.ecef_z_m.0).abs() < 1.0e-9);
    }
}

#[test]
fn receiver_navigation_filter_batch_runner_matches_per_epoch_filter_execution() {
    let run = clean_synthetic_navigation_run();
    let navigation = GpsBroadcastNavigationData {
        ephemerides: run.profile.ephemerides.clone(),
        klobuchar: Some(sample_klobuchar_coefficients()),
    };
    let ionosphere_biased_observations = run
        .observations
        .iter()
        .map(|observation| {
            bias_epoch_with_klobuchar(observation, run.profile.truth_ecef_m, &navigation)
        })
        .collect::<Vec<_>>();
    let receiver = Receiver::new(run.config.clone(), ReceiverRuntime::default());
    let batch_solutions = receiver.solve_observation_epochs_with_gps_broadcast_navigation_filter(
        &ionosphere_biased_observations,
        &navigation,
    );
    let mut per_epoch_filter = NavigationFilter::from_pipeline_config(&run.config);
    let per_epoch_solutions = ionosphere_biased_observations
        .iter()
        .filter_map(|observation| {
            per_epoch_filter.solve_epoch(
                observation,
                &navigation.ephemerides,
                navigation.klobuchar.as_ref(),
            )
        })
        .collect::<Vec<_>>();

    assert_eq!(batch_solutions.len(), per_epoch_solutions.len());
    for (batch, per_epoch) in batch_solutions.iter().zip(per_epoch_solutions.iter()) {
        assert_eq!(batch.epoch.index, per_epoch.epoch.index);
        assert_eq!(batch.status, per_epoch.status);
        assert_eq!(batch.explain_decision, per_epoch.explain_decision);
        assert_eq!(batch.explain_reasons, per_epoch.explain_reasons);
        assert_eq!(batch.used_sat_count, per_epoch.used_sat_count);
        assert!((batch.ecef_x_m.0 - per_epoch.ecef_x_m.0).abs() < 1.0e-9);
        assert!((batch.ecef_y_m.0 - per_epoch.ecef_y_m.0).abs() < 1.0e-9);
        assert!((batch.ecef_z_m.0 - per_epoch.ecef_z_m.0).abs() < 1.0e-9);
    }
}
