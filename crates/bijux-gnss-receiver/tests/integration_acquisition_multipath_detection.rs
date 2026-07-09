#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SatId};
use bijux_gnss_receiver::api::{
    sim::{generate_l1_ca_multi, SyntheticScenario, SyntheticSignalParams},
    AcquisitionEngine, ReceiverPipelineConfig, ReceiverRuntime,
};

#[test]
fn acquisition_multipath_harness_preserves_ranked_candidates() {
    let sat = gps_l1_ca_satellite();
    let config = multipath_profile();
    let frame = delayed_secondary_path_frame(
        &config,
        0.001,
        [
            path_signal(sat, 0.0, 300.0, 46.0),
            path_signal(sat, 0.0, 360.0, 42.0),
        ],
        0x2407_4501,
    );
    let run = AcquisitionEngine::new(config.clone(), ReceiverRuntime::default())
        .with_doppler(config.acquisition_doppler_search_hz, config.acquisition_doppler_step_hz)
        .run_fft_topn_with_explain(&frame, &[sat], 4, 1, 1);

    assert_eq!(run.results.len(), 1);
    assert_eq!(run.explains.len(), 1);
    assert!(!run.results[0].is_empty());
    assert!(run.results[0].len() <= 4);
    assert!(run.explains[0].candidate_count >= 1);
    assert!(!run.explains[0].selected_reason.is_empty());
}

fn multipath_profile() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        acquisition_doppler_search_hz: 500,
        acquisition_doppler_step_hz: 250,
        ..ReceiverPipelineConfig::default()
    }
}

fn delayed_secondary_path_frame(
    config: &ReceiverPipelineConfig,
    duration_s: f64,
    signals: [SyntheticSignalParams; 2],
    seed: u64,
) -> bijux_gnss_core::api::SamplesFrame {
    generate_l1_ca_multi(
        config,
        &SyntheticScenario {
            sample_rate_hz: config.sampling_freq_hz,
            intermediate_freq_hz: config.intermediate_freq_hz,
            duration_s,
            seed,
            satellites: signals.into_iter().collect(),
            ephemerides: Vec::new(),
            id: "acquisition_multipath_detection".to_string(),
        },
    )
}

fn path_signal(
    sat: SatId,
    doppler_hz: f64,
    code_phase_chips: f64,
    cn0_db_hz: f32,
) -> SyntheticSignalParams {
    SyntheticSignalParams {
        sat,
        doppler_hz,
        code_phase_chips,
        carrier_phase_rad: 0.0,
        cn0_db_hz,
        data_bit_flip: false,
    }
}

fn gps_l1_ca_satellite() -> SatId {
    SatId { constellation: Constellation::Gps, prn: 7 }
}
