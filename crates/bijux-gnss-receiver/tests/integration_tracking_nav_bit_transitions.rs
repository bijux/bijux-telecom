#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{
    AcqHypothesis, AcqResult, Constellation, Hertz, ReceiverSampleTrace, SatId, SignalBand,
};
use bijux_gnss_receiver::api::{
    sim::{generate_l1_ca, SyntheticSignalParams},
    ReceiverPipelineConfig, ReceiverRuntime, TrackingEngine,
};

use support::tracking_truth::{
    nav_bit_transition_epoch_indices, post_lock_epochs, wrapped_code_phase_error_samples,
};

const CLEAN_NAV_BIT_CN0_DB_HZ: f32 = 52.0;
const CLEAN_NAV_BIT_DURATION_S: f64 = 0.065;

fn accepted_acquisition(sat: SatId, doppler_hz: f64, code_phase_samples: usize) -> AcqResult {
    AcqResult {
        sat,
        signal_band: SignalBand::L1,
        source_time: ReceiverSampleTrace::default(),
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(doppler_hz),
        carrier_hz: Hertz(doppler_hz),
        code_phase_samples,
        peak: 1.0,
        second_peak: 0.1,
        mean: 0.01,
        peak_mean_ratio: 20.0,
        peak_second_ratio: 10.0,
        cn0_proxy: CLEAN_NAV_BIT_CN0_DB_HZ,
        score: 1.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: Some("seeded_nav_bit_tracking_start".to_string()),
        doppler_refinement: None,
        code_phase_refinement: None,
        uncertainty: None,
    }
}

fn nav_bit_tracking_config() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 15.0,
        fll_bw_hz: 10.0,
        ..ReceiverPipelineConfig::default()
    }
}

fn track_clean_nav_bit_case(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    doppler_hz: f64,
    code_phase_chips: f64,
    carrier_phase_rad: f64,
) -> Vec<bijux_gnss_core::api::TrackEpoch> {
    let expected_code_phase_samples =
        code_phase_chips * config.sampling_freq_hz / config.code_freq_basis_hz;
    let frame = generate_l1_ca(
        config,
        SyntheticSignalParams {
            sat,
            doppler_hz,
            code_phase_chips,
            carrier_phase_rad,
            cn0_db_hz: CLEAN_NAV_BIT_CN0_DB_HZ,
            data_bit_flip: true,
        },
        0x57A1_1C7D,
        CLEAN_NAV_BIT_DURATION_S,
    );
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(
        &frame,
        &[accepted_acquisition(
            sat,
            doppler_hz,
            expected_code_phase_samples.round() as usize,
        )],
    );

    tracks.first().expect("track").epochs.clone()
}

#[test]
fn tracking_reports_nav_bit_lock_across_clean_bit_transition_windows() {
    let config = nav_bit_tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 12 };
    let epochs = track_clean_nav_bit_case(&config, sat, 120.0, 144.375, 0.3);
    let post_lock = post_lock_epochs(&epochs);
    let transition_epochs = nav_bit_transition_epoch_indices(&epochs, config.sampling_freq_hz);
    let expected_code_phase_samples = 144.375 * config.sampling_freq_hz / config.code_freq_basis_hz;

    assert!(epochs.len() >= 60, "epochs={epochs:?}");
    assert!(!post_lock.is_empty(), "tracking never reached lock: epochs={epochs:?}");
    assert_eq!(transition_epochs, vec![20, 40, 60], "epochs={epochs:?}");
    assert!(
        epochs.iter().any(|epoch| epoch.nav_bit_lock),
        "tracking never reported nav-bit lock: epochs={epochs:?}"
    );
    assert!(
        post_lock
            .iter()
            .any(|epoch| wrapped_code_phase_error_samples(
                &config,
                epoch.code_phase_samples.0,
                expected_code_phase_samples
            ) <= 1.0),
        "tracking never settled near the seeded code phase in nav-bit scenario: epochs={epochs:?}"
    );
}
