#![allow(missing_docs)]
use std::fs;

use bijux_gnss_receiver::api::{
    observations_from_tracking,
    sim::{generate_l1_ca_multi, SyntheticScenario},
    AcquisitionEngine, ReceiverPipelineConfig, TrackingEngine,
};

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
    let tracks =
        tracking.track_from_acquisition(&frame, &acq_results, bijux_gnss_core::api::SignalBand::L1);

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

    let obs_epochs = if let Some(first) = tracks.first() {
        observations_from_tracking(&config, &first.epochs).0
    } else {
        Vec::new()
    };
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

fn load_scenario() -> SyntheticScenario {
    let path =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../configs/scenarios/basic.toml");
    let contents = fs::read_to_string(path).expect("scenario file");
    toml::from_str(&contents).expect("valid scenario")
}
