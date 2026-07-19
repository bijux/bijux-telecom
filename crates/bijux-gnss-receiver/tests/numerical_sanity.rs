#![allow(missing_docs)]
mod support;

use std::fs;

use bijux_gnss_receiver::api::{
    observations_from_tracking_results,
    sim::{generate_l1_ca_multi, SyntheticScenario},
    AcquisitionEngine, ReceiverPipelineConfig, ReceiverRuntime, TrackingEngine,
};

use support::navigation_truth::{four_satellite_pvt_scenario, truth_seeded_acquisition_results};

#[test]
fn numerical_sanity_pipeline() {
    let scenario = load_scenario();
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: scenario.sample_rate_hz,
        intermediate_freq_hz: scenario.intermediate_freq_hz,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 12,
        tracking_budget_ms: 100.0,
        tracking_over_budget_action: "continue".to_string(),
        ..ReceiverPipelineConfig::default()
    };

    let frame = generate_l1_ca_multi(&config, &scenario);
    let sats: Vec<bijux_gnss_core::api::SatId> =
        scenario.satellites.iter().map(|s| s.sat).collect();
    let runtime = bijux_gnss_receiver::api::ReceiverRuntime::default();
    let acq = AcquisitionEngine::new(config.clone(), runtime.clone()).with_doppler(10_000, 500);
    let acq_results = acq.run_fft(&frame, &sats);

    for res in &acq_results {
        assert!(res.peak.is_finite());
        assert!(res.mean.is_finite());
        assert!(res.peak_mean_ratio.is_finite());
        assert!(res.peak_second_ratio.is_finite());
    }

    let tracking = TrackingEngine::new(config.clone(), runtime);
    let tracks = tracking.track_from_acquisition(&frame, &acq_results);

    for track in &tracks {
        for epoch in &track.epochs {
            assert!(epoch.prompt_i.is_finite());
            assert!(epoch.prompt_q.is_finite());
            assert!(epoch.carrier_hz.0.is_finite());
            assert!(epoch.code_rate_hz.0.is_finite());
            assert!(epoch.cn0_dbhz.is_finite());
            assert!(epoch.dll_err.is_finite());
            assert!(epoch.pll_err.is_finite());
            assert!(epoch.fll_err.is_finite());
        }
    }

    let obs_epochs = observations_from_tracking_results(&config, &tracks, 10).output;
    for epoch in &obs_epochs {
        assert!(epoch.t_rx_s.0.is_finite());
        for sat in &epoch.sats {
            assert!(sat.pseudorange_m.0.is_finite());
            assert!(sat.carrier_phase_cycles.0.is_finite());
            assert!(sat.doppler_hz.0.is_finite());
            assert!(sat.cn0_dbhz.is_finite());
        }
    }
}

#[test]
fn multisatellite_tracking_numerical_sanity_pipeline() {
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
    let source_time = bijux_gnss_core::api::ReceiverSampleTrace::from_sample_time(frame.t0);
    let acquisitions = truth_seeded_acquisition_results(&config, source_time, &profile.scenario);
    let tracks = TrackingEngine::new(config.clone(), ReceiverRuntime::default())
        .track_from_acquisition(&frame, &acquisitions);
    let report = observations_from_tracking_results(&config, &tracks, 10);

    assert!(
        tracks.iter().filter(|track| !track.epochs.is_empty()).count() >= 4,
        "expected at least four tracked channels",
    );
    let grouped_epoch = report
        .output
        .iter()
        .find(|epoch| epoch.valid && epoch.sats.len() >= 4)
        .expect("valid multisatellite observation epoch");
    assert_eq!(grouped_epoch.decision, bijux_gnss_core::api::ObservationEpochDecision::Accepted);
    assert!(grouped_epoch.t_rx_s.0.is_finite());
    for sat in &grouped_epoch.sats {
        assert!(sat.pseudorange_m.0.is_finite());
        assert!(sat.carrier_phase_cycles.0.is_finite());
        assert!(sat.doppler_hz.0.is_finite());
        assert!(sat.cn0_dbhz.is_finite());
    }
}

fn load_scenario() -> SyntheticScenario {
    let path =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../configs/scenarios/basic.toml");
    let contents = fs::read_to_string(path).expect("scenario file");
    toml::from_str(&contents).expect("valid scenario")
}
