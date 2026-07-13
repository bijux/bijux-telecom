#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{
    AcqHypothesis, AcqResult, Constellation, Hertz, ReceiverSampleTrace, SatId, SignalBand,
};
use bijux_gnss_receiver::api::{
    core::Sample,
    sim::{generate_l1_ca_with_fades, SyntheticFadeWindow, SyntheticSignalParams},
    ReceiverPipelineConfig, ReceiverRuntime, TrackingEngine,
};

use support::tracking_truth::epoch_indices_with_lock_state_reason;

const PRELOCK_CN0_DBHZ: f32 = 60.0;

fn accepted_acquisition(sat: SatId, code_phase_samples: usize) -> AcqResult {
    AcqResult {
        sat,
        signal_band: SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        glonass_frequency_channel: None,
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
        explain_selection_reason: Some("prompt_power_drop_tracking_start".to_string()),
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
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
fn tracking_reports_prompt_power_drop_after_sustained_signal_loss() {
    let config = tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 21 };
    let fade_start_s = 0.030;
    let fade_end_s = 0.200;
    let fade_start_sample = (fade_start_s * config.sampling_freq_hz).round() as u64;
    let fade_end_sample = (fade_end_s * config.sampling_freq_hz).round() as u64;
    let mut frame = generate_l1_ca_with_fades(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: PRELOCK_CN0_DBHZ,
            navigation_data: false.into(),
        },
        &[SyntheticFadeWindow { start_s: fade_start_s, end_s: fade_end_s, signal_scale: 0.0 }],
        0x600D_600D,
        0.260,
    );
    for sample in &mut frame.iq[fade_start_sample as usize..fade_end_sample as usize] {
        *sample = Sample::new(0.0, 0.0);
    }
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(&frame, &[accepted_acquisition(sat, 0)]);
    let epochs = &tracks.first().expect("track").epochs;

    assert!(
        epochs.iter().any(|epoch| {
            epoch.sample_index < fade_start_sample
                && epoch.lock_state == "tracking"
                && epoch.pll_lock
                && epoch.fll_lock
        }),
        "tracking must establish lock before the power drop: epochs={epochs:?}"
    );

    let prompt_power_drop_indices =
        epoch_indices_with_lock_state_reason(epochs, "prompt_power_drop");
    assert!(
        !prompt_power_drop_indices.is_empty(),
        "sustained fades must emit explicit prompt_power_drop loss reasons: epochs={epochs:?}"
    );
    assert!(
        prompt_power_drop_indices.iter().any(|index| {
            let epoch = &epochs[*index];
            epoch.lock_state == "lost"
                && epoch.sample_index >= fade_start_sample
                && epoch.sample_index < fade_end_sample
        }),
        "prompt_power_drop must appear while the signal remains absent: epochs={epochs:?}"
    );
    assert!(
        epochs.iter().all(|epoch| epoch.lock_state_reason.as_deref() != Some("lock_lost")),
        "generic lock_lost must not mask a prompt-power collapse: epochs={epochs:?}"
    );
    assert!(
        prompt_power_drop_indices.iter().all(|index| epochs[*index].cycle_slip_reason.is_none()),
        "prompt-power loss must not be mislabeled as a phase event: epochs={epochs:?}"
    );
}
