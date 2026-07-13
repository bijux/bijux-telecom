#![allow(missing_docs)]

use bijux_gnss_core::api::{AcqSearchSummary, Constellation, SatId};
use bijux_gnss_receiver::api::{
    sim::{generate_l1_ca, SyntheticSignalParams},
    AcquisitionEngine, ReceiverPipelineConfig, ReceiverRuntime,
};

#[test]
fn weak_peak_harness_preserves_selected_candidate() {
    let sat = gps_l1_ca_satellite();
    let config = weak_peak_profile(2.5);
    let frame = weak_signal_frame(&config, sat, 20.0, 0x2407_2001);
    let run = AcquisitionEngine::new(config.clone(), ReceiverRuntime::default())
        .with_doppler(config.acquisition_doppler_search_hz, config.acquisition_doppler_step_hz)
        .run_fft_topn_with_explain(&frame, &[sat], 4, 1, 1);

    assert_eq!(run.results.len(), 1);
    assert_eq!(run.explains.len(), 1);
    assert!(!run.results[0].is_empty());
    assert!(run.results[0].len() <= 4);
    assert!(!run.explains[0].selected_reason.is_empty());
    assert!(run.explains[0].candidate_count >= 1);
}

#[test]
fn weak_signal_below_peak_mean_threshold_is_rejected() {
    let sat = gps_l1_ca_satellite();
    let config = weak_peak_profile(10.0);
    let frame = weak_signal_frame(&config, sat, 20.0, 0x2407_2002);
    let run = AcquisitionEngine::new(config.clone(), ReceiverRuntime::default())
        .with_doppler(config.acquisition_doppler_search_hz, config.acquisition_doppler_step_hz)
        .run_fft_topn_with_explain(&frame, &[sat], 4, 1, 1);
    let best = &run.results[0][0];

    assert_eq!(best.hypothesis.to_string(), "rejected", "{run:?}");
    assert!(best.peak_mean_ratio < config.acquisition_peak_mean_threshold, "{run:?}");
    assert_eq!(run.explains[0].selected_reason, "low_peak_metric", "{run:?}");
    assert!(run.explains[0].candidates[0].reason.contains("below threshold 10.000000"), "{run:?}");
}

#[test]
fn weak_signal_rejection_counts_as_rejected() {
    let sat = gps_l1_ca_satellite();
    let config = weak_peak_profile(10.0);
    let frame = weak_signal_frame(&config, sat, 20.0, 0x2407_2003);
    let results = AcquisitionEngine::new(config.clone(), ReceiverRuntime::default())
        .with_doppler(config.acquisition_doppler_search_hz, config.acquisition_doppler_step_hz)
        .run_fft(&frame, &[sat]);
    let summary = AcqSearchSummary::from_results(&results);

    assert_eq!(results.len(), 1);
    assert_eq!(results[0].hypothesis.to_string(), "rejected", "{results:?}");
    assert_eq!(summary.searched_satellites, 1);
    assert_eq!(summary.rejected, 1);
    assert_eq!(summary.accepted, 0);
    assert_eq!(summary.ambiguous, 0);
    assert_eq!(summary.deferred, 0);
}

fn weak_peak_profile(acquisition_peak_mean_threshold: f32) -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        acquisition_doppler_search_hz: 1_500,
        acquisition_doppler_step_hz: 250,
        acquisition_peak_mean_threshold,
        ..ReceiverPipelineConfig::default()
    }
}

fn weak_signal_frame(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    cn0_db_hz: f32,
    seed: u64,
) -> bijux_gnss_core::api::SamplesFrame {
    generate_l1_ca(
        config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 250.0,
            code_phase_chips: 300.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz,
            data_bit_flip: false,
        },
        seed,
        0.001,
    )
}

fn gps_l1_ca_satellite() -> SatId {
    SatId { constellation: Constellation::Gps, prn: 7 }
}
