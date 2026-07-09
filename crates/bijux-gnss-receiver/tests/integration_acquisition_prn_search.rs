#![allow(missing_docs)]
use std::fs;
use std::path::Path;

use bijux_gnss_core::api::{AcqHypothesis, AcqSearchSummary};
use bijux_gnss_receiver::api::{
    sim::{generate_l1_ca_multi, SyntheticScenario},
    AcquisitionEngine, ReceiverPipelineConfig, ReceiverRuntime,
};
use bijux_gnss_signal::api::samples_per_code;

#[test]
fn acquisition_searches_every_requested_prn_and_reports_signal_metrics() {
    let scenario = load_scenario();
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: scenario.sample_rate_hz,
        intermediate_freq_hz: scenario.intermediate_freq_hz,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 12,
        ..ReceiverPipelineConfig::default()
    };
    let samples_per_code =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let frame = generate_l1_ca_multi(&config, &scenario);
    let sats: Vec<_> = scenario.satellites.iter().map(|sat| sat.sat).collect();
    let doppler_step_hz = 500.0;
    let acquisition =
        AcquisitionEngine::new(config, ReceiverRuntime::default()).with_doppler(10_000, 500);

    let results = acquisition.run_fft(&frame, &sats);
    let summary = AcqSearchSummary::from_results(&results);

    assert_eq!(results.len(), sats.len());
    assert_eq!(summary.searched_satellites, sats.len());
    assert_eq!(
        summary.accepted + summary.ambiguous + summary.rejected + summary.deferred,
        sats.len()
    );

    for satellite in &scenario.satellites {
        let result = results
            .iter()
            .find(|result| result.sat == satellite.sat)
            .expect("result for requested satellite");
        let doppler_err =
            (satellite.doppler_hz - (result.carrier_hz.0 - scenario.intermediate_freq_hz)).abs();

        assert!(
            matches!(result.hypothesis, AcqHypothesis::Accepted | AcqHypothesis::Ambiguous),
            "expected a trackable acquisition outcome for {:?}, got {}",
            satellite.sat,
            result.hypothesis
        );
        assert!(doppler_err <= doppler_step_hz * 5.0, "doppler error too large: {doppler_err}");
        assert!(
            result.code_phase_samples < samples_per_code,
            "code phase out of range for {:?}: {}",
            satellite.sat,
            result.code_phase_samples
        );
        assert!(result.peak.is_finite());
        assert!(result.peak_mean_ratio.is_finite());
        assert!(result.peak_second_ratio.is_finite());
        assert!(result.explain_selection_reason.is_some());

        let assumptions = result.assumptions.as_ref().expect("acquisition assumptions");
        assert_eq!(assumptions.code_phase_search_mode, "full_code");
        assert_eq!(assumptions.code_phase_search_bins, samples_per_code);
    }
}

fn load_scenario() -> SyntheticScenario {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../configs/scenarios/basic.toml");
    let contents = fs::read_to_string(path).expect("scenario file");
    toml::from_str(&contents).expect("valid scenario")
}
