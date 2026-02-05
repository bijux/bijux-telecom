#![allow(missing_docs)]
use std::fs;

use bijux_gnss_receiver::api::{
    AcquisitionEngine,
    sim::{generate_l1_ca_multi, SyntheticScenario},
    ReceiverPipelineConfig,
};

#[test]
fn golden_acquisition_from_scenario() {
    let scenario = load_scenario();
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: scenario.sample_rate_hz,
        intermediate_freq_hz: scenario.intermediate_freq_hz,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 12,
        ..ReceiverPipelineConfig::default()
    };

    let frame = generate_l1_ca_multi(&config, &scenario);
    let sats: Vec<bijux_gnss_core::api::SatId> =
        scenario.satellites.iter().map(|s| s.sat).collect();
    let doppler_step_hz = 500.0;
    let runtime = bijux_gnss_receiver::api::ReceiverRuntimeConfig::default();
    let acq = Acquisition::new(config, runtime).with_doppler(10_000, doppler_step_hz as i32);
    let results = acq.run_fft(&frame, &sats);

    for (sat, res) in scenario.satellites.iter().zip(results.iter()) {
        assert_eq!(sat.sat, res.sat);
        let doppler_err =
            (sat.doppler_hz - (res.carrier_hz.0 - scenario.intermediate_freq_hz)).abs();
        assert!(
            doppler_err <= doppler_step_hz * 5.0,
            "doppler error too large: {doppler_err}"
        );
        assert!(
            res.peak_mean_ratio >= 2.0,
            "peak/mean ratio too low: {}",
            res.peak_mean_ratio
        );
        assert!(
            res.peak_second_ratio >= 1.1,
            "peak/second ratio too low: {}",
            res.peak_second_ratio
        );
    }
}

fn load_scenario() -> SyntheticScenario {
    let path =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../configs/scenarios/basic.toml");
    let contents = fs::read_to_string(path).expect("scenario file");
    toml::from_str(&contents).expect("valid scenario")
}
