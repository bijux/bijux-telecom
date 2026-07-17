#![allow(missing_docs)]

use bijux_gnss_core::api::{
    AcqHypothesis, AcqResult, Constellation, Hertz, ReceiverSampleTrace, SatId, SignalBand,
};
use bijux_gnss_receiver::api::{
    observations_from_tracking_results,
    sim::{
        generate_l1_ca, generate_l1_ca_with_fades, generate_l1_ca_with_phase_windows,
        SyntheticFadeWindow, SyntheticPhaseWindow, SyntheticSignalParams,
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
        doppler_rate_hz_per_s: 0.0,
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

#[test]
fn observations_keep_carrier_phase_arc_after_recoverable_fade() {
    let config = tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 21 };
    let doppler_hz = 120.0;
    let fade_start_s = 0.030;
    let fade_end_s = 0.035;
    let fade_start_sample = (fade_start_s * config.sampling_freq_hz).round() as u64;
    let fade_end_sample = (fade_end_s * config.sampling_freq_hz).round() as u64;
    let frame = generate_l1_ca_with_fades(
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
        &[SyntheticFadeWindow { start_s: fade_start_s, end_s: fade_end_s, signal_scale: 0.1 }],
        0xC0A5_7A11,
        0.090,
    );
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks =
        tracking.track_from_acquisition(&frame, &[accepted_acquisition(sat, doppler_hz, 0)]);
    let report = observations_from_tracking_results(&config, &tracks, 10);
    let sats = report.output.iter().flat_map(|epoch| epoch.sats.iter()).collect::<Vec<_>>();

    let pre_fade = sats
        .iter()
        .rev()
        .find(|sat| {
            sat.metadata.time_tag_sample_index < fade_start_sample
                && sat.metadata.carrier_phase_continuity == "continuous"
                && !sat.lock_flags.cycle_slip
        })
        .unwrap_or_else(|| panic!("missing pre-fade continuous carrier arc: {:?}", report.output));
    let during_fade = sats
        .iter()
        .find(|sat| {
            sat.metadata.time_tag_sample_index >= fade_start_sample
                && sat.metadata.time_tag_sample_index < fade_end_sample
                && sat.metadata.carrier_phase_continuity == "continuous"
                && !sat.lock_flags.cycle_slip
        })
        .unwrap_or_else(|| {
            panic!("missing fade-window continuous carrier arc: {:?}", report.output)
        });
    let post_fade = sats
        .iter()
        .find(|sat| {
            sat.metadata.time_tag_sample_index >= fade_end_sample
                && sat.lock_flags.carrier_lock
                && sat.metadata.carrier_phase_continuity == "continuous"
                && !sat.lock_flags.cycle_slip
        })
        .unwrap_or_else(|| panic!("missing post-fade continuous carrier arc: {:?}", report.output));

    assert_eq!(
        during_fade.metadata.carrier_phase_arc_start_sample_index,
        pre_fade.metadata.carrier_phase_arc_start_sample_index
    );
    assert_eq!(
        post_fade.metadata.carrier_phase_arc_start_sample_index,
        pre_fade.metadata.carrier_phase_arc_start_sample_index
    );
}

#[test]
fn observations_start_new_carrier_phase_arc_after_reacquisition() {
    let config = tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 21 };
    let interruption_start_s = 0.030;
    let interruption_end_s = 0.090;
    let interruption_start_sample = (interruption_start_s * config.sampling_freq_hz).round() as u64;
    let interruption_end_sample = (interruption_end_s * config.sampling_freq_hz).round() as u64;
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
            cn0_db_hz: PRELOCK_CN0_DBHZ,
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
    let tracks = tracking.track_from_acquisition(&frame, &[accepted_acquisition(sat, 0.0, 0)]);
    let report = observations_from_tracking_results(&config, &tracks, 10);
    let sats = report.output.iter().flat_map(|epoch| epoch.sats.iter()).collect::<Vec<_>>();
    let arc_events = sats
        .iter()
        .filter(|sat| {
            sat.metadata.time_tag_sample_index >= interruption_start_sample
                && (sat.metadata.carrier_phase_continuity != "continuous"
                    || sat.metadata.observation_lock_state == "reacquired")
        })
        .map(|sat| {
            (
                sat.metadata.time_tag_sample_index,
                sat.metadata.observation_lock_state.as_str(),
                sat.metadata.observation_lock_reason.as_deref(),
                sat.metadata.carrier_phase_continuity.as_str(),
                sat.lock_flags.cycle_slip,
            )
        })
        .collect::<Vec<_>>();

    assert!(
        sats.iter().any(|sat| {
            sat.metadata.time_tag_sample_index >= interruption_start_sample
                && sat.metadata.time_tag_sample_index < interruption_end_sample
                && matches!(sat.metadata.carrier_phase_continuity.as_str(), "coasted" | "unusable")
                && !sat.lock_flags.cycle_slip
        }),
        "interruption must degrade carrier-phase continuity before arc reset: {:?}",
        report.output
    );

    let arc_reset = sats
        .iter()
        .find(|sat| {
            sat.metadata.time_tag_sample_index >= interruption_end_sample
                && sat.metadata.carrier_phase_continuity == "reset_after_unlock"
                && sat.lock_flags.cycle_slip
        })
        .unwrap_or_else(|| panic!("missing post-loss carrier-arc reset: {arc_events:?}"));
    let settled = sats
        .iter()
        .find(|sat| {
            sat.metadata.time_tag_sample_index > arc_reset.metadata.time_tag_sample_index
                && sat.lock_flags.carrier_lock
                && sat.metadata.carrier_phase_continuity == "continuous"
                && !sat.lock_flags.cycle_slip
        })
        .unwrap_or_else(|| panic!("missing settled post-reacquisition arc: {:?}", report.output));

    assert_eq!(
        arc_reset
            .metadata
            .carrier_phase_arc
            .as_ref()
            .expect("reset carrier-phase arc")
            .start_reason,
        "loss_of_lock"
    );
    assert_eq!(
        settled.metadata.carrier_phase_arc_start_sample_index,
        arc_reset.metadata.carrier_phase_arc_start_sample_index
    );
}

#[test]
fn observations_keep_carrier_phase_arc_across_data_bit_transitions() {
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
            navigation_data: true.into(),
        },
        0xDA7A_B171,
        0.055,
    );
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(&frame, &[accepted_acquisition(sat, 0.0, 0)]);
    let report = observations_from_tracking_results(&config, &tracks, 10);
    let sats = report.output.iter().flat_map(|epoch| epoch.sats.iter()).collect::<Vec<_>>();

    for transition_epoch in [20_u64, 40] {
        let transition_sample = transition_epoch * config.code_length as u64;
        let sat = sats
            .iter()
            .find(|sat| sat.metadata.time_tag_sample_index == transition_sample)
            .unwrap_or_else(|| {
                panic!(
                    "missing observation at data-bit transition sample {transition_sample}: {:?}",
                    report.output
                )
            });

        assert_eq!(sat.metadata.carrier_phase_continuity, "continuous");
        assert!(!sat.lock_flags.cycle_slip);
        assert!(
            sat.metadata.observation_reject_reasons.iter().all(|reason| reason != "cycle_slip"),
            "{sat:?}"
        );
    }
}
