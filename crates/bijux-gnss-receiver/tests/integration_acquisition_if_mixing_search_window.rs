#![allow(missing_docs)]

use bijux_gnss_core::api::{AcqHypothesis, Constellation, SatId};
use bijux_gnss_receiver::api::{
    carrier_hz_from_doppler_hz,
    sim::{generate_l1_ca, SyntheticSignalParams},
    AcquisitionEngine, ReceiverPipelineConfig, ReceiverRuntime,
};
use bijux_gnss_signal::api::samples_per_code;

const SEARCH_HZ: i32 = 1_500;
const DOPPLER_BIN_HZ: i32 = 250;
const COHERENT_MS: u32 = 10;

#[test]
fn acquisition_reports_upper_edge_when_high_if_signal_sits_outside_search_window() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 1_250_000.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 12,
        ..ReceiverPipelineConfig::default()
    };
    let samples_per_code =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let sat = SatId { constellation: Constellation::Gps, prn: 11 };
    let acquisition = AcquisitionEngine::new(config.clone(), ReceiverRuntime::default())
        .with_doppler(SEARCH_HZ, DOPPLER_BIN_HZ);

    let mut signal_config = config.clone();
    signal_config.intermediate_freq_hz += 2_000.0;
    let frame = generate_l1_ca(
        &signal_config,
        SyntheticSignalParams {
            sat,
            doppler_hz: 0.0,
            code_phase_chips: 210.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 48.0,
            data_bit_flip: false,
        },
        4_277_009_102,
        COHERENT_MS as f64 * samples_per_code as f64 / config.sampling_freq_hz,
    );
    let run = acquisition.run_fft_topn_with_explain(&frame, &[sat], 4, COHERENT_MS, 1);
    let best = run.results.first().and_then(|candidates| candidates.first()).expect("candidate");
    let explain = run.explains.first().expect("explain");
    let selected = explain.candidates.first().expect("selected candidate");
    let expected_upper_edge_hz =
        carrier_hz_from_doppler_hz(config.intermediate_freq_hz, SEARCH_HZ as f64);

    assert_eq!(best.hypothesis.to_string(), AcqHypothesis::Rejected.to_string());
    assert_eq!(explain.selected_reason, "signal_outside_search_range");
    assert!(
        selected.reason.contains("upper search edge"),
        "expected upper-edge search diagnostic, got: {}",
        selected.reason
    );
    assert!(
        selected.reason.contains(&format!("{expected_upper_edge_hz:.3}")),
        "expected search diagnostic to mention upper edge carrier {expected_upper_edge_hz:.3}, got: {}",
        selected.reason
    );
}
