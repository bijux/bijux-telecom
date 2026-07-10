#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{
    AcqHypothesis, AcqResult, Constellation, Hertz, ReceiverSampleTrace, SatId, SignalBand,
};
use bijux_gnss_receiver::api::{
    sim::{generate_l1_ca_with_phase_windows, SyntheticPhaseWindow, SyntheticSignalParams},
    ReceiverPipelineConfig, ReceiverRuntime, TrackingEngine,
};

use support::tracking_truth::epoch_indices_with_lock_state_reason;

const PRELOCK_CN0_DBHZ: f32 = 60.0;

fn accepted_acquisition(sat: SatId, code_phase_samples: usize) -> AcqResult {
    AcqResult {
        sat,
        signal_band: SignalBand::L1,
        source_time: ReceiverSampleTrace::default(),
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(0.0),
        carrier_hz: Hertz(0.0),
        code_phase_samples,
        peak: 1.0,
        second_peak: 0.1,
        mean: 0.01,
        peak_mean_ratio: 20.0,
        peak_second_ratio: 10.0,
        cn0_proxy: PRELOCK_CN0_DBHZ,
        score: 1.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: Some("phase_jump_tracking_start".to_string()),
        doppler_refinement: None,
        code_phase_refinement: None,
        uncertainty: None,
    }
}

fn tracking_config() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
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

#[test]
fn tracking_reports_phase_jump_on_carrier_discontinuity() {
    let config = tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 21 };
    let jump_start_s = 0.030;
    let jump_start_sample = (jump_start_s * config.sampling_freq_hz).round() as u64;
    let frame = generate_l1_ca_with_phase_windows(
        &config,
        SyntheticSignalParams {
            sat,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: PRELOCK_CN0_DBHZ,
            data_bit_flip: false,
        },
        &[SyntheticPhaseWindow {
            start_s: jump_start_s,
            end_s: 0.090,
            phase_offset_rad: std::f64::consts::TAU * 0.38,
        }],
        0xA11C_E5E1,
        0.090,
    );
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(&frame, &[accepted_acquisition(sat, 0)]);
    let epochs = &tracks.first().expect("track").epochs;

    assert!(
        epochs.iter().any(|epoch| {
            epoch.sample_index < jump_start_sample
                && epoch.lock_state == "tracking"
                && epoch.pll_lock
                && epoch.fll_lock
        }),
        "tracking must establish lock before the phase jump: epochs={epochs:?}"
    );

    let phase_jump_indices = epoch_indices_with_lock_state_reason(epochs, "phase_jump");
    assert!(
        !phase_jump_indices.is_empty(),
        "phase discontinuities must emit explicit phase_jump reasons: epochs={epochs:?}"
    );
    assert!(
        phase_jump_indices.iter().any(|index| {
            let epoch = &epochs[*index];
            epoch.lock_state == "lost"
                && epoch.sample_index >= jump_start_sample
                && epoch.cycle_slip
                && epoch.cycle_slip_reason.as_deref() == Some("phase_jump")
        }),
        "phase jumps must become explicit loss-of-lock events with cycle-slip evidence: epochs={epochs:?}"
    );
    assert!(
        epochs.iter().all(|epoch| epoch.lock_state_reason.as_deref() != Some("prompt_power_drop")),
        "carrier discontinuities must not be mislabeled as prompt-power loss: epochs={epochs:?}"
    );
}
