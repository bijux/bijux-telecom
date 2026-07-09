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
