#![allow(missing_docs)]

use bijux_gnss_core::api::{Constellation, SatId};
use bijux_gnss_receiver::api::{
    sim::{generate_l1_ca, SyntheticSignalParams},
    AcquisitionEngine, ReceiverPipelineConfig, ReceiverRuntime,
};

#[test]
fn wrong_prn_harness_preserves_requested_prn_results() {
    let config = wrong_prn_profile();
    let signal_sat = gps_l1_ca_satellite(7);
    let requested_sats = [gps_l1_ca_satellite(7), gps_l1_ca_satellite(8), gps_l1_ca_satellite(9)];
    let frame = single_prn_frame(&config, signal_sat, 0x2407_3001);
    let run = AcquisitionEngine::new(config.clone(), ReceiverRuntime::default())
        .with_doppler(config.acquisition_doppler_search_hz, config.acquisition_doppler_step_hz)
        .run_fft_topn_with_explain(&frame, &requested_sats, 4, 1, 1);

    assert_eq!(run.results.len(), requested_sats.len());
    assert_eq!(run.explains.len(), requested_sats.len());
    assert!(run.results.iter().all(|candidates| !candidates.is_empty()));
    assert!(run.explains.iter().all(|explain| explain.candidate_count >= 1));
}

fn wrong_prn_profile() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        acquisition_doppler_search_hz: 1_500,
        acquisition_doppler_step_hz: 250,
        ..ReceiverPipelineConfig::default()
    }
}

fn single_prn_frame(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    seed: u64,
) -> bijux_gnss_core::api::SamplesFrame {
    generate_l1_ca(
        config,
        SyntheticSignalParams {
            sat,
            doppler_hz: 250.0,
            code_phase_chips: 300.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 48.0,
            data_bit_flip: false,
        },
        seed,
        0.001,
    )
}

fn gps_l1_ca_satellite(prn: u8) -> SatId {
    SatId { constellation: Constellation::Gps, prn }
}
