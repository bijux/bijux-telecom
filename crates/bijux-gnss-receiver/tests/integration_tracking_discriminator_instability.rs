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
        explain_selection_reason: Some("discriminator_instability_tracking_start".to_string()),
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

fn oscillating_phase_windows(start_s: f64, cycle_count: usize) -> Vec<SyntheticPhaseWindow> {
    (0..cycle_count)
        .map(|index| {
            let offset_start_s = start_s + (index as f64 * 0.002);
            SyntheticPhaseWindow {
                start_s: offset_start_s,
                end_s: offset_start_s + 0.001,
                phase_offset_rad: std::f64::consts::TAU * 0.30,
            }
        })
        .collect()
}

#[test]
fn tracking_reports_discriminator_instability_before_phase_slip() {
    let config = tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 21 };
    let instability_start_s = 0.030;
    let instability_end_s = 0.040;
    let instability_start_sample = (instability_start_s * config.sampling_freq_hz).round() as u64;
    let instability_end_sample = (instability_end_s * config.sampling_freq_hz).round() as u64;
    let phase_windows = oscillating_phase_windows(instability_start_s, 5);
    let frame = generate_l1_ca_with_phase_windows(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: PRELOCK_CN0_DBHZ,
            data_bit_flip: false,
        },
        &phase_windows,
        0xD15C_A110,
        0.090,
    );
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(&frame, &[accepted_acquisition(sat, 0)]);
    let epochs = &tracks.first().expect("track").epochs;

    assert!(
        epochs.iter().any(|epoch| {
            epoch.sample_index < instability_start_sample
                && epoch.lock_state == "tracking"
                && epoch.pll_lock
                && epoch.fll_lock
        }),
        "tracking must establish lock before the discriminator stress window: epochs={epochs:?}"
    );

    let instability_indices =
        epoch_indices_with_lock_state_reason(epochs, "discriminator_instability");
    assert!(
        !instability_indices.is_empty(),
        "repeated sub-slip phase errors must emit discriminator_instability: epochs={epochs:?}"
    );
    assert!(
        instability_indices.iter().any(|index| {
            let epoch = &epochs[*index];
            epoch.lock_state == "lost"
                && epoch.sample_index >= instability_start_sample
                && epoch.sample_index < instability_end_sample
                && !epoch.cycle_slip
                && epoch.cycle_slip_reason.is_none()
        }),
        "discriminator instability must surface before any phase slip classification: epochs={epochs:?}"
    );
    assert!(
        epochs.iter().all(|epoch| epoch.lock_state_reason.as_deref() != Some("prompt_power_drop")),
        "strong-prompt instability must not be mislabeled as prompt-power loss: epochs={epochs:?}"
    );
}
