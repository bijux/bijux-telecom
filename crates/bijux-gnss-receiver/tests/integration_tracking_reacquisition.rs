#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{
    AcqHypothesis, AcqResult, Constellation, Hertz, ReceiverSampleTrace, SatId, SignalBand,
};
use bijux_gnss_receiver::api::{
    sim::{generate_l1_ca_with_fades, SyntheticFadeWindow, SyntheticSignalParams},
    ReceiverPipelineConfig, ReceiverRuntime, TrackingEngine,
};

use support::tracking_truth::{
    epoch_indices_with_lock_state, epoch_indices_with_lock_state_reason,
};

const REACQUISITION_PRELOCK_CN0_DBHZ: f32 = 60.0;
const REACQUISITION_STABLE_TRACKING_EPOCHS: usize = 5;

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
        cn0_proxy: REACQUISITION_PRELOCK_CN0_DBHZ,
        score: 1.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: Some("tracking_reacquisition_start".to_string()),
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
fn tracking_reacquires_after_bounded_signal_interruption() {
    let config = tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 21 };
    let interruption_start_s = 0.030;
    let interruption_end_s = 0.190;
    let frame = generate_l1_ca_with_fades(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: REACQUISITION_PRELOCK_CN0_DBHZ,
            navigation_data: false.into(),
        },
        &[SyntheticFadeWindow {
            start_s: interruption_start_s,
            end_s: interruption_end_s,
            signal_scale: 0.0,
        }],
        0x51AC_0001,
        0.350,
    );
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(&frame, &[accepted_acquisition(sat, 0)]);
    let epochs = &tracks.first().expect("track").epochs;

    let interruption_start_sample = (interruption_start_s * config.sampling_freq_hz).round() as u64;
    let interruption_end_sample = (interruption_end_s * config.sampling_freq_hz).round() as u64;
    let lost_indices = epoch_indices_with_lock_state(epochs, "lost");
    let reacquired_indices = epoch_indices_with_lock_state_reason(epochs, "reacquired");

    assert!(
        lost_indices.iter().any(|index| {
            let sample_index = epochs[*index].sample_index;
            sample_index >= interruption_start_sample && sample_index < interruption_end_sample
        }),
        "bounded interruptions must drive the channel into lost state before reacquisition: epochs={epochs:?}"
    );
    assert!(
        !reacquired_indices.is_empty(),
        "signal return after lock loss must emit an explicit reacquired reason: epochs={epochs:?}"
    );

    let reacquired_index = *reacquired_indices
        .iter()
        .find(|index| epochs[**index].sample_index >= interruption_end_sample)
        .unwrap_or_else(|| {
            panic!("reacquisition must occur after the interruption ends: epochs={epochs:?}")
        });

    let stable_tracking_epochs = epochs[reacquired_index..]
        .iter()
        .filter(|epoch| {
            epoch.sample_index >= interruption_end_sample
                && epoch.lock_state == "tracking"
                && epoch.lock_state_reason.as_deref() != Some("reacquisition_failed")
                && epoch.pll_lock
                && epoch.fll_lock
                && !epoch.cycle_slip
        })
        .count();
    assert!(
        stable_tracking_epochs >= REACQUISITION_STABLE_TRACKING_EPOCHS,
        "reacquisition must return to a stable tracking window after interruption: stable_tracking_epochs={stable_tracking_epochs}, epochs={epochs:?}"
    );
}

#[test]
fn tracking_reports_failed_reacquisition_when_signal_does_not_return() {
    let config = tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 21 };
    let interruption_start_s = 0.030;
    let frame = generate_l1_ca_with_fades(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: REACQUISITION_PRELOCK_CN0_DBHZ,
            navigation_data: false.into(),
        },
        &[SyntheticFadeWindow { start_s: interruption_start_s, end_s: 0.300, signal_scale: 0.0 }],
        0x51AC_0002,
        0.300,
    );
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(&frame, &[accepted_acquisition(sat, 0)]);
    let epochs = &tracks.first().expect("track").epochs;

    let interruption_start_sample = (interruption_start_s * config.sampling_freq_hz).round() as u64;
    let failed_reacquisition_indices =
        epoch_indices_with_lock_state_reason(epochs, "reacquisition_failed");
    let reacquired_indices = epoch_indices_with_lock_state_reason(epochs, "reacquired");

    assert!(
        !failed_reacquisition_indices.is_empty(),
        "missing signal return must emit explicit reacquisition_failed epochs: epochs={epochs:?}"
    );
    assert!(
        failed_reacquisition_indices
            .iter()
            .all(|index| epochs[*index].sample_index >= interruption_start_sample),
        "reacquisition failure must only appear after lock has actually been lost: epochs={epochs:?}"
    );
    assert!(
        reacquired_indices.is_empty(),
        "a channel without signal return must not claim successful reacquisition: epochs={epochs:?}"
    );
}
