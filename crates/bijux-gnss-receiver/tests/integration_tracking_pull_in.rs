#![allow(missing_docs)]

use bijux_gnss_core::api::{
    AcqHypothesis, AcqResult, Constellation, Hertz, ReceiverSampleTrace, SatId, SignalBand,
};
use bijux_gnss_receiver::api::{
    sim::{generate_l1_ca, SyntheticSignalParams},
    ReceiverPipelineConfig, ReceiverRuntime, TrackingEngine,
};

fn accepted_acquisition(sat: SatId, doppler_hz: f64, code_phase_samples: usize) -> AcqResult {
    AcqResult {
        sat,
        signal_band: SignalBand::L1,
        glonass_frequency_channel: None,
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
        cn0_proxy: 60.0,
        score: 1.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: Some("seeded_carrier_pull_in_start".to_string()),
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    }
}

#[test]
fn tracking_converges_carrier_pull_in_before_declaring_pll_lock() {
    let config = ReceiverPipelineConfig {
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
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 17 };
    let true_doppler_hz = 220.0;
    let seeded_doppler_hz = 20.0;
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            doppler_hz: true_doppler_hz,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.25,
            cn0_db_hz: 60.0,
            data_bit_flip: false,
        },
        0xA531_1D0F,
        0.030,
    );
    let tracking = TrackingEngine::new(config, ReceiverRuntime::default());

    let tracks =
        tracking.track_from_acquisition(&frame, &[accepted_acquisition(sat, seeded_doppler_hz, 0)]);
    let epochs = &tracks.first().expect("track").epochs;
    assert!(epochs.len() >= 10, "epochs={epochs:?}");

    let first_pll_lock_index = epochs
        .iter()
        .position(|epoch| epoch.pll_lock)
        .unwrap_or_else(|| panic!("pll lock must eventually be declared: epochs={epochs:?}"));
    assert!(first_pll_lock_index > 0, "pll lock declared too early: epochs={epochs:?}");
    assert!(
        epochs[..first_pll_lock_index]
            .iter()
            .all(|epoch| !epoch.pll_lock && epoch.lock_state == "pull_in"),
        "epochs before pll lock must remain in pull_in: epochs={epochs:?}"
    );
    assert!(
        epochs[..first_pll_lock_index]
            .iter()
            .any(|epoch| epoch.fll_lock
                && epoch.lock_state_reason.as_deref() == Some("carrier_pull_in")),
        "pull_in must expose FLL-assisted convergence before PLL lock: epochs={epochs:?}"
    );

    let initial_freq_error = (epochs[0].carrier_hz.0 - true_doppler_hz).abs();
    let first_pll_lock_freq_error =
        (epochs[first_pll_lock_index].carrier_hz.0 - true_doppler_hz).abs();
    assert!(
        first_pll_lock_freq_error < initial_freq_error,
        "carrier pull-in did not improve seeded frequency error before pll lock: initial={initial_freq_error}, first_pll_lock={first_pll_lock_freq_error}, epochs={epochs:?}"
    );
    assert!(
        epochs[first_pll_lock_index].lock_state_reason.as_deref() == Some("carrier_converged"),
        "pll lock must be declared by explicit carrier convergence: epochs={epochs:?}"
    );
}
