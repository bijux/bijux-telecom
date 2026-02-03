use std::fs;

use bijux_gnss_receiver::{
    acquisition::Acquisition,
    sim::{generate_l1_ca_multi, SyntheticScenario},
    tracking::Tracking,
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
        ..ReceiverConfig::default()
    };

    let frame = generate_l1_ca_multi(&config, &scenario);
    let sats: Vec<bijux_gnss_core::SatId> = scenario.satellites.iter().map(|s| s.sat).collect();
    let acq = Acquisition::new(config.clone()).with_doppler(10_000, 500);
    let acq_results = acq.run_fft(&frame, &sats);

    let tracking = Tracking::new(config);
    let tracks =
        tracking.track_from_acquisition(&frame, &acq_results, bijux_gnss_core::SignalBand::L1);

    for track in &tracks {
        assert!(!track.epochs.is_empty());
        let locked_epochs = track.epochs.iter().filter(|e| e.lock).count();
        assert!(
            locked_epochs > 0,
            "no lock for {:?}-{}",
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
