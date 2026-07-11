#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{ReceiverSampleTrace, SignalDelayAlignment};
use bijux_gnss_receiver::api::{
    observations_from_tracking_results, sim::generate_l1_ca_multi, ReceiverPipelineConfig,
    ReceiverRuntime, TrackingEngine,
};

use support::navigation_truth::{
    four_satellite_pvt_scenario, synthetic_pseudorange_m, truth_seeded_acquisition_results,
};

const PSEUDORANGE_TOLERANCE_M: f64 = 150.0;

#[test]
fn observations_use_resolved_signal_delay_alignment_for_pseudorange() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 5,
        tracking_budget_ms: 100.0,
        tracking_over_budget_action: "continue".to_string(),
        ..ReceiverPipelineConfig::default()
    };
    let profile = four_satellite_pvt_scenario(&config);
    let frame = generate_l1_ca_multi(&config, &profile.scenario);
    let source_time = ReceiverSampleTrace::from_sample_time(frame.t0);
    let mut acquisitions =
        truth_seeded_acquisition_results(&config, source_time, &profile.scenario);
    for acquisition in &mut acquisitions {
        acquisition.signal_delay_alignment = Some(SignalDelayAlignment {
            whole_code_periods: profile.pseudorange_epoch_base,
            source: "synthetic_truth".to_string(),
        });
    }

    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(&frame, &acquisitions);
    let observation_report = observations_from_tracking_results(&config, &tracks, 10);
    let observation_epoch = observation_report
        .output
        .iter()
        .find(|epoch| {
            epoch.epoch_idx == profile.target_epoch_idx && epoch.valid && epoch.sats.len() >= 4
        })
        .expect("aligned observation epoch");

    for sat in &observation_epoch.sats {
        let ephemeris = profile
            .ephemerides
            .iter()
            .find(|ephemeris| ephemeris.sat == sat.signal_id.sat)
            .expect("ephemeris for observed satellite");
        let expected_pseudorange_m =
            synthetic_pseudorange_m(ephemeris, 100_000.0, profile.truth_ecef_m);
        let error_m = (sat.pseudorange_m.0 - expected_pseudorange_m).abs();

        assert_eq!(sat.metadata.pseudorange_model, "tracked_code_phase_alignment");
        assert_eq!(sat.metadata.signal_delay_alignment_source, "synthetic_truth");
        assert!(
            error_m <= PSEUDORANGE_TOLERANCE_M,
            "pseudorange error exceeded tolerance for {:?}: observed={} expected={} error={}",
            sat.signal_id.sat,
            sat.pseudorange_m.0,
            expected_pseudorange_m,
            error_m,
        );
    }
}
