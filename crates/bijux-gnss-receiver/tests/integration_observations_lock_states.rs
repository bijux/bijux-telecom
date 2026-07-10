#![allow(missing_docs)]

use bijux_gnss_core::api::{
    AcqHypothesis, AcqResult, Constellation, Hertz, ReceiverSampleTrace, SatId, SignalBand,
};
use bijux_gnss_receiver::api::{
    observations_from_tracking_results,
    sim::{
        generate_l1_ca_with_fades, generate_l1_ca_with_phase_windows, SyntheticFadeWindow,
        SyntheticPhaseWindow, SyntheticSignalParams,
    },
    ReceiverPipelineConfig, ReceiverRuntime, TrackingEngine,
};

const PRELOCK_CN0_DBHZ: f32 = 60.0;

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
        cn0_proxy: PRELOCK_CN0_DBHZ,
        score: 1.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: Some("observation_lock_state_tracking_start".to_string()),
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
fn observation_epochs_preserve_degraded_lock_state_during_short_fade() {
    let config = tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 21 };
    let fade_start_s = 0.030;
    let fade_end_s = 0.040;
    let duration_s = 0.090;
    let frame = generate_l1_ca_with_fades(
        &config,
        SyntheticSignalParams {
            sat,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: PRELOCK_CN0_DBHZ,
            data_bit_flip: false,
        },
        &[SyntheticFadeWindow { start_s: fade_start_s, end_s: fade_end_s, signal_scale: 0.0 }],
        0x51AC_2001,
        duration_s,
    );
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(&frame, &[accepted_acquisition(sat, 0.0, 0)]);
    let report = observations_from_tracking_results(&config, &tracks, 10);

    let fade_start_epoch_idx = (fade_start_s * 1000.0).round() as u64;
    let fade_end_epoch_idx = (fade_end_s * 1000.0).round() as u64;
    let degraded_rows = report
        .output
        .iter()
        .flat_map(|epoch| epoch.sats.iter().map(move |sat| (epoch.epoch_idx, sat)))
        .filter(|(epoch_idx, sat)| {
            *epoch_idx >= fade_start_epoch_idx
                && *epoch_idx < fade_end_epoch_idx
                && sat.metadata.observation_lock_state == "degraded"
        })
        .collect::<Vec<_>>();

    assert!(
        !degraded_rows.is_empty(),
        "short-fade observations must retain degraded lock rows: report={:?}",
        report.output
    );
    assert!(degraded_rows
        .iter()
        .all(|(_, sat)| sat.metadata.tracking_lock_state == sat.metadata.observation_lock_state));
}

#[test]
fn observation_epochs_preserve_cycle_slip_lock_state() {
    let config = tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 21 };
    let doppler_hz = 120.0;
    let jump_start_s: f64 = 0.030;
    let jump_start_epoch_idx = (jump_start_s * 1000.0).round() as u64;
    let frame = generate_l1_ca_with_phase_windows(
        &config,
        SyntheticSignalParams {
            sat,
            doppler_hz,
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
        0x51AC_2003,
        0.090,
    );
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks =
        tracking.track_from_acquisition(&frame, &[accepted_acquisition(sat, doppler_hz, 0)]);
    let report = observations_from_tracking_results(&config, &tracks, 10);

    let cycle_slip_rows = report
        .output
        .iter()
        .flat_map(|epoch| epoch.sats.iter().map(move |sat| (epoch.epoch_idx, sat)))
        .filter(|(epoch_idx, sat)| {
            *epoch_idx >= jump_start_epoch_idx && sat.metadata.observation_lock_state == "cycle_slip"
        })
        .collect::<Vec<_>>();

    assert!(
        !cycle_slip_rows.is_empty(),
        "phase discontinuities must retain cycle-slip observation rows: report={:?}",
        report.output
    );
    assert!(cycle_slip_rows.iter().all(|(_, sat)| {
        sat.lock_flags.cycle_slip
            && sat.metadata.tracking_lock_state == sat.metadata.observation_lock_state
    }));
}
