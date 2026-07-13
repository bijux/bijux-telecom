#![allow(missing_docs)]

use bijux_gnss_core::api::{
    AcqHypothesis, AcqResult, Constellation, Hertz, ReceiverSampleTrace, SatId, SignalBand,
};
use bijux_gnss_receiver::api::{
    observations_from_tracking_results,
    sim::{
        generate_l1_ca, generate_l1_ca_with_phase_windows, SyntheticPhaseWindow,
        SyntheticSignalParams,
    },
    ReceiverPipelineConfig, ReceiverRuntime, TrackingEngine,
};

const PRELOCK_CN0_DBHZ: f32 = 60.0;

fn accepted_acquisition(sat: SatId, doppler_hz: f64, code_phase_samples: usize) -> AcqResult {
    AcqResult {
        sat,
        signal_band: SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
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
        cn0_proxy: PRELOCK_CN0_DBHZ,
        score: 1.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: Some("observation_carrier_phase_seed".to_string()),
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
fn observations_keep_clean_carrier_phase_arc_continuous() {
    let config = tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 21 };
    let frame = generate_l1_ca(
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
        0x0B51_0A71,
        0.020,
    );
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(&frame, &[accepted_acquisition(sat, 0.0, 0)]);
    let report = observations_from_tracking_results(&config, &tracks, 10);

    let locked_sats = report
        .output
        .iter()
        .flat_map(|epoch| epoch.sats.iter())
        .filter(|sat| sat.lock_flags.code_lock && sat.lock_flags.carrier_lock)
        .collect::<Vec<_>>();

    assert!(locked_sats.len() >= 2, "missing stable locked observations: {:?}", report.output);
    assert_eq!(locked_sats[0].metadata.carrier_phase_model, "tracked_carrier_cycles");
    assert_eq!(locked_sats[0].metadata.carrier_phase_continuity, "arc_start");
    assert_eq!(locked_sats[1].metadata.carrier_phase_continuity, "continuous");
    assert_eq!(
        locked_sats[1].metadata.carrier_phase_arc_start_sample_index,
        locked_sats[0].metadata.time_tag_sample_index
    );
}

#[test]
fn observations_mark_phase_jump_carrier_phase_as_unusable() {
    let config = tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 21 };
    let doppler_hz = 120.0;
    let jump_start_s = 0.030;
    let jump_start_sample = (jump_start_s * config.sampling_freq_hz).round() as u64;
    let frame = generate_l1_ca_with_phase_windows(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: PRELOCK_CN0_DBHZ,
            navigation_data: false.into(),
        },
        &[SyntheticPhaseWindow {
            start_s: jump_start_s,
            end_s: 0.090,
            phase_offset_rad: std::f64::consts::TAU * 0.38,
        }],
        0xA11C_E5F1,
        0.090,
    );
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks =
        tracking.track_from_acquisition(&frame, &[accepted_acquisition(sat, doppler_hz, 0)]);
    let phase_jump_epoch_idx = tracks
        .first()
        .expect("track")
        .epochs
        .iter()
        .find(|epoch| {
            epoch.sample_index >= jump_start_sample
                && epoch.cycle_slip
                && epoch.cycle_slip_reason.as_deref() == Some("phase_jump")
        })
        .map(|epoch| epoch.epoch.index)
        .unwrap_or_else(|| panic!("tracking did not expose phase-jump slips: {tracks:?}"));

    let report = observations_from_tracking_results(&config, &tracks, 10);

    let (_, slipped_sat) = report
        .output
        .iter()
        .filter(|epoch| epoch.epoch_idx >= phase_jump_epoch_idx)
        .flat_map(|epoch| epoch.sats.iter().map(move |sat| (epoch.epoch_idx, sat)))
        .find(|(_, sat)| {
            sat.metadata.carrier_phase_continuity == "unusable" && sat.lock_flags.cycle_slip
        })
        .unwrap_or_else(|| {
            panic!("missing phase-jump carrier-phase rejection: {:?}", report.output)
        });

    assert_eq!(slipped_sat.metadata.carrier_phase_model, "tracked_carrier_cycles");
    assert_eq!(slipped_sat.metadata.tracking_state, "lost");
    assert!(!slipped_sat.lock_flags.carrier_lock);
    assert!(slipped_sat
        .observation_reject_reasons
        .iter()
        .any(|reason| reason == "cycle_slip" || reason == "tracking_unlock"));
}
