#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SatId};
use bijux_gnss_receiver::api::{
    sim::{generate_l1_ca, SyntheticSignalParams},
    AcquisitionEngine, ReceiverPipelineConfig, ReceiverRuntime,
};

#[test]
fn acquisition_uncertainty_harness_preserves_accepted_candidate() {
    let sat = gps_l1_ca_satellite();
    let config = uncertainty_profile();
    let frame = accepted_signal_frame(&config, sat, 0x2407_4601);
    let run = AcquisitionEngine::new(config.clone(), ReceiverRuntime::default())
        .with_doppler(config.acquisition_doppler_search_hz, config.acquisition_doppler_step_hz)
        .run_fft_topn_with_explain(&frame, &[sat], 4, 1, 1);

    assert_eq!(run.results.len(), 1);
    assert_eq!(run.explains.len(), 1);
    assert!(!run.results[0].is_empty());
    assert_eq!(run.results[0][0].sat, sat);
    assert_eq!(run.results[0][0].hypothesis.to_string(), "accepted", "{run:?}");
    assert!(run.results[0][0].doppler_refinement.is_some(), "{run:?}");
    assert!(run.results[0][0].code_phase_refinement.is_some(), "{run:?}");
}

#[test]
fn acquisition_reports_uncertainty_for_accepted_candidate() {
    let sat = gps_l1_ca_satellite();
    let config = uncertainty_profile();
    let frame = accepted_signal_frame(&config, sat, 0x2407_4601);
    let run = AcquisitionEngine::new(config.clone(), ReceiverRuntime::default())
        .with_doppler(config.acquisition_doppler_search_hz, config.acquisition_doppler_step_hz)
        .run_fft_topn_with_explain(&frame, &[sat], 4, 1, 1);
    let best = &run.results[0][0];
    let uncertainty = best
        .uncertainty
        .as_ref()
        .unwrap_or_else(|| panic!("accepted acquisition uncertainty missing: {run:?}"));

    assert_eq!(best.hypothesis.to_string(), "accepted", "{run:?}");
    assert!(uncertainty.doppler_hz.is_finite());
    assert!(uncertainty.code_phase_samples.is_finite());
    assert!(uncertainty.doppler_hz > 0.0, "{run:?}");
    assert!(uncertainty.code_phase_samples > 0.0, "{run:?}");
}

#[test]
fn acquisition_uncertainty_stays_within_search_resolution_bounds() {
    let sat = gps_l1_ca_satellite();
    let config = uncertainty_profile();
    let frame = accepted_signal_frame(&config, sat, 0x2407_4601);
    let results = AcquisitionEngine::new(config.clone(), ReceiverRuntime::default())
        .with_doppler(config.acquisition_doppler_search_hz, config.acquisition_doppler_step_hz)
        .run_fft(&frame, &[sat]);
    let best = &results[0];
    let uncertainty = best
        .uncertainty
        .as_ref()
        .unwrap_or_else(|| panic!("accepted acquisition uncertainty missing: {results:?}"));

    assert_eq!(best.hypothesis.to_string(), "accepted", "{results:?}");
    assert!(
        uncertainty.doppler_hz <= config.acquisition_doppler_step_hz as f64 / 2.0 + f64::EPSILON,
        "{results:?}"
    );
    assert!(uncertainty.code_phase_samples <= 0.5 + f64::EPSILON, "{results:?}");
}

#[test]
fn acquisition_primary_result_preserves_acceptance_after_candidate_truncation() {
    let sat = gps_l1_ca_satellite();
    let config = uncertainty_profile();
    let frame = accepted_signal_frame(&config, sat, 0x2407_4601);
    let primary = AcquisitionEngine::new(config.clone(), ReceiverRuntime::default())
        .with_doppler(config.acquisition_doppler_search_hz, config.acquisition_doppler_step_hz)
        .run_fft(&frame, &[sat]);
    let ranked = AcquisitionEngine::new(config.clone(), ReceiverRuntime::default())
        .with_doppler(config.acquisition_doppler_search_hz, config.acquisition_doppler_step_hz)
        .run_fft_topn_with_explain(&frame, &[sat], 4, 1, 1);

    assert_eq!(primary.len(), 1);
    assert_eq!(primary[0].hypothesis.to_string(), "accepted", "{primary:?}");
    assert_eq!(ranked.results[0][0].hypothesis.to_string(), "accepted", "{ranked:?}");
}

fn uncertainty_profile() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 4_000_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        acquisition_doppler_search_hz: 1_500,
        acquisition_doppler_step_hz: 250,
        acquisition_peak_second_threshold: 1.01,
        ..ReceiverPipelineConfig::default()
    }
}

fn accepted_signal_frame(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    seed: u64,
) -> bijux_gnss_core::api::SamplesFrame {
    generate_l1_ca(
        config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            doppler_hz: 375.0,
            code_phase_chips: 200.375,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 58.0,
            data_bit_flip: false,
        },
        seed,
        0.001,
    )
}

fn gps_l1_ca_satellite() -> SatId {
    SatId { constellation: Constellation::Gps, prn: 3 }
}
