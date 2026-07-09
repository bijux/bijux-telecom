#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SatId};
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
