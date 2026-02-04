#![allow(missing_docs)]
use std::fs;

use bijux_gnss_receiver::api::{
    AcquisitionEngine,
    sim::{generate_l1_ca_multi, SyntheticScenario},
    TrackingEngine,
    ReceiverConfig,
};

#[test]
fn golden_tracking_from_scenario() {
    let scenario = load_scenario();
    let config = ReceiverConfig {
        sampling_freq_hz: scenario.sample_rate_hz,
        intermediate_freq_hz: scenario.intermediate_freq_hz,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 12,
        tracking_budget_ms: 100.0,
        tracking_over_budget_action: "continue".to_string(),
        ..ReceiverConfig::default()
    };

    let frame = generate_l1_ca_multi(&config, &scenario);
    let sats: Vec<bijux_gnss_core::api::SatId> =
        scenario.satellites.iter().map(|s| s.sat).collect();
    let acq = Acquisition::new(config.clone()).with_doppler(10_000, 500);
    let acq_results = acq.run_fft(&frame, &sats);

    let tracking = Tracking::new(config);
    let tracks =
        tracking.track_from_acquisition(&frame, &acq_results, bijux_gnss_core::api::SignalBand::L1);

    for track in &tracks {
        assert!(!track.epochs.is_empty());
        let locked_epochs = track.epochs.iter().filter(|e| e.lock).count();
        let lock_ratio = locked_epochs as f64 / track.epochs.len() as f64;
        assert!(
            locked_epochs > 0,
            "no lock for {:?}-{}",
            track.sat.constellation,
            track.sat.prn
        );
        assert!(
            lock_ratio > 0.1,
            "lock ratio too low for {:?}-{}: {lock_ratio}",
            track.sat.constellation,
            track.sat.prn
        );
        let mean_cn0 =
            track.epochs.iter().map(|e| e.cn0_dbhz).sum::<f64>() / track.epochs.len() as f64;
        let var_cn0 = track
            .epochs
            .iter()
            .map(|e| (e.cn0_dbhz - mean_cn0).powi(2))
            .sum::<f64>()
            / track.epochs.len() as f64;
        let std_cn0 = var_cn0.sqrt();
        assert!(
            std_cn0 < 10.0,
            "CN0 instability too high for {:?}-{}: {std_cn0}",
            track.sat.constellation,
            track.sat.prn
        );
    }
}

fn load_scenario() -> SyntheticScenario {
    let path =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../configs/scenarios/basic.toml");
    let contents = fs::read_to_string(path).expect("scenario file");
    toml::from_str(&contents).expect("valid scenario")
}
