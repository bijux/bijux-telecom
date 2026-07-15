use super::*;
use std::collections::BTreeSet;

use bijux_gnss_core::api::{
    Chips, Epoch, Hertz, Meters, SatId, SignalDelayAlignment, TrackingTransmitTime,
    TrackingUncertainty,
};
use bijux_gnss_signal::api::{
    registered_signal_registry_entries, signal_cycles_to_meters, signal_meters_to_cycles,
};

#[path = "tests/divergence_detection.rs"]
mod divergence_detection;
#[path = "tests/measurement_variance.rs"]
mod measurement_variance;
#[path = "tests/pseudorange_resolution.rs"]
mod pseudorange_resolution;
#[path = "tests/rejection_status.rs"]
mod rejection_status;
#[path = "tests/residual_reporting.rs"]
mod residual_reporting;
#[path = "tests/signal_metadata.rs"]
mod signal_metadata;
#[path = "tests/signal_support.rs"]
mod signal_support;
#[path = "tests/timing_metadata.rs"]
mod timing_metadata;

fn make_tracking_epoch(
    prn: u8,
    config: &ReceiverPipelineConfig,
    epoch_idx: u64,
    carrier_hz: f64,
) -> TrackEpoch {
    make_tracking_epoch_with_phase(prn, config, epoch_idx, carrier_hz, 0.0)
}

fn make_tracking_epoch_with_phase(
    prn: u8,
    config: &ReceiverPipelineConfig,
    epoch_idx: u64,
    carrier_hz: f64,
    carrier_phase_cycles: f64,
) -> TrackEpoch {
    let sample_index = epoch_idx
        * samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length)
            as u64;
    TrackEpoch {
        epoch: Epoch { index: epoch_idx },
        sample_index,
        source_time: ReceiverSampleTrace::from_sample_index(sample_index, config.sampling_freq_hz),
        sat: SatId { constellation: Constellation::Gps, prn },
        prompt_i: 1.0,
        prompt_q: 0.0,
        carrier_hz: Hertz(carrier_hz),
        carrier_phase_cycles: Cycles(carrier_phase_cycles),
        code_rate_hz: Hertz(config.code_freq_basis_hz),
        code_phase_samples: Chips(0.0),
        lock: true,
        cn0_dbhz: 45.0,
        pll_lock: true,
        dll_lock: true,
        fll_lock: true,
        cycle_slip: false,
        nav_bit_lock: false,
        dll_err: 0.0,
        pll_err: 0.0,
        fll_err: 0.0,
        anti_false_lock: false,
        cycle_slip_reason: None,
        lock_state: "tracking".to_string(),
        lock_state_reason: Some("stable_tracking".to_string()),
        tracking_uncertainty: Some(test_tracking_uncertainty()),
        processing_ms: None,
        ..TrackEpoch::default()
    }
}

fn test_tracking_uncertainty() -> TrackingUncertainty {
    TrackingUncertainty {
        code_phase_samples: 0.05,
        carrier_phase_cycles: 0.02,
        doppler_hz: 1.0,
        cn0_dbhz: 0.5,
    }
}

fn set_code_phase_uncertainty(epoch: &mut TrackEpoch, code_phase_samples: f64) {
    let mut uncertainty =
        epoch.tracking_uncertainty.clone().unwrap_or_else(test_tracking_uncertainty);
    uncertainty.code_phase_samples = code_phase_samples;
    epoch.tracking_uncertainty = Some(uncertainty);
}

fn make_tracking_epoch_with_alignment(
    prn: u8,
    config: &ReceiverPipelineConfig,
    epoch_idx: u64,
    carrier_hz: f64,
    carrier_phase_cycles: f64,
    whole_code_periods: u64,
    code_phase_samples: f64,
) -> TrackEpoch {
    TrackEpoch {
        code_phase_samples: Chips(test_tracking_code_phase_samples(config, code_phase_samples)),
        signal_delay_alignment: Some(SignalDelayAlignment {
            whole_code_periods,
            sample_delay_samples: 0,
            source: "synthetic_truth".to_string(),
        }),
        ..make_tracking_epoch_with_phase(prn, config, epoch_idx, carrier_hz, carrier_phase_cycles)
    }
}

fn test_tracking_code_phase_samples(
    config: &ReceiverPipelineConfig,
    aligned_code_phase_samples: f64,
) -> f64 {
    if !aligned_code_phase_samples.is_finite() || aligned_code_phase_samples < 0.0 {
        return aligned_code_phase_samples;
    }
    let period_samples =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length)
            as f64;
    (period_samples - aligned_code_phase_samples).rem_euclid(period_samples)
}

fn make_observation_ready_epoch(
    prn: u8,
    config: &ReceiverPipelineConfig,
    epoch_idx: u64,
) -> TrackEpoch {
    make_tracking_epoch_with_alignment(prn, config, epoch_idx, 0.0, 0.0, 68, 128.0)
}

fn aligned_pseudorange_m(
    config: &ReceiverPipelineConfig,
    whole_code_periods: u64,
    code_phase_samples: f64,
) -> f64 {
    let code_phase_chips = code_phase_samples
        / (samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length)
            as f64
            / config.code_length as f64);
    ((whole_code_periods as f64 * config.code_length as f64) + code_phase_chips)
        / config.code_freq_basis_hz
        * SPEED_OF_LIGHT_MPS
}

fn test_tracking_code_phase_samples_for_signal(
    config: &ReceiverPipelineConfig,
    signal: SignalSpec,
    code_length: usize,
    aligned_code_phase_chips: f64,
) -> f64 {
    if !aligned_code_phase_chips.is_finite() || aligned_code_phase_chips < 0.0 {
        return aligned_code_phase_chips;
    }
    let samples_per_chip = config.sampling_freq_hz / signal.code_rate_hz;
    let period_samples = samples_per_chip * code_length as f64;
    let aligned_code_phase_samples = aligned_code_phase_chips * samples_per_chip;
    (period_samples - aligned_code_phase_samples).rem_euclid(period_samples)
}

fn aligned_pseudorange_m_for_signal(
    signal: SignalSpec,
    code_length: usize,
    whole_code_periods: u64,
    code_phase_chips: f64,
) -> f64 {
    ((whole_code_periods as f64 * code_length as f64) + code_phase_chips) / signal.code_rate_hz
        * SPEED_OF_LIGHT_MPS
}

fn gps_l1ca_decoded_time_epoch(
    prn: u8,
    config: &ReceiverPipelineConfig,
    epoch_idx: u64,
    capture_start_gps_time: GpsTime,
    receive_gps_time: GpsTime,
    decoded_code_periods: f64,
    aligned_code_phase_chips: f64,
) -> TrackEpoch {
    let signal = signal_spec_gps_l1_ca();
    let code_period_s = 1023.0 / signal.code_rate_hz;
    let code_delay_s = aligned_code_phase_chips / signal.code_rate_hz;
    let expected_signal_travel_time_s = decoded_code_periods * code_period_s + code_delay_s;
    let decoded_transmit_time = receive_gps_time.offset_seconds(-expected_signal_travel_time_s);
    let epoch_sample_index = ((receive_gps_time.tow_s - capture_start_gps_time.tow_s)
        * config.sampling_freq_hz)
        .round() as u64;

    TrackEpoch {
        sat: SatId { constellation: Constellation::Gps, prn },
        signal_band: SignalBand::L1,
        carrier_hz: Hertz(tracked_signal_center_hz(config.intermediate_freq_hz, signal)),
        code_rate_hz: Hertz(signal.code_rate_hz),
        code_phase_samples: Chips(test_tracking_code_phase_samples_for_signal(
            config,
            signal,
            1023,
            aligned_code_phase_chips,
        )),
        transmit_time: Some(TrackingTransmitTime {
            transmit_gps_time: decoded_transmit_time,
            source: "decoded_lnav_how".to_string(),
        }),
        sample_index: epoch_sample_index,
        source_time: ReceiverSampleTrace::from_sample_index(
            epoch_sample_index,
            config.sampling_freq_hz,
        ),
        signal_delay_alignment: None,
        ..make_tracking_epoch_with_phase(
            prn,
            config,
            epoch_idx,
            tracked_signal_center_hz(config.intermediate_freq_hz, signal),
            0.0,
        )
    }
}

fn epoch_sample_index(config: &ReceiverPipelineConfig, epoch_idx: u64) -> u64 {
    epoch_idx
        * samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length)
            as u64
}

fn observation_sample_index(
    config: &ReceiverPipelineConfig,
    initial_sample_index: u64,
    observation_offset: u64,
) -> u64 {
    initial_sample_index + observation_offset * observation_interval_samples(config)
}

fn make_track(prn: u8, config: &ReceiverPipelineConfig) -> TrackingResult {
    let sat = SatId { constellation: Constellation::Gps, prn };
    let epoch = make_tracking_epoch(prn, config, 70, 0.0);
    TrackingResult {
        sat,
        carrier_hz: 0.0,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "deferred".to_string(),
        acquisition_score: 0.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: 0.0,
        acq_to_track_state: "deferred".to_string(),
        epochs: vec![epoch],
        transitions: Vec::new(),
    }
}

fn track_from_epoch(epoch: TrackEpoch) -> TrackingResult {
    let sat = epoch.sat;
    TrackingResult {
        sat,
        carrier_hz: epoch.carrier_hz.0,
        code_phase_samples: epoch.code_phase_samples.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: epoch.carrier_hz.0,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![epoch],
        transitions: Vec::new(),
    }
}

#[test]
fn observation_satellite_order_is_canonical() {
    let config = ReceiverPipelineConfig::default();
    let tracks_a = vec![make_track(2, &config), make_track(1, &config)];
    let tracks_b = vec![make_track(1, &config), make_track(2, &config)];

    let report_a = observations_from_tracking_results(&config, &tracks_a, 10);
    let report_b = observations_from_tracking_results(&config, &tracks_b, 10);

    let json_a = serde_json::to_string(&report_a.output).unwrap();
    let json_b = serde_json::to_string(&report_b.output).unwrap();
    assert_eq!(json_a, json_b);
}

#[test]
fn observations_from_tracking_derives_if_relative_doppler_from_carrier() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 2_000.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let doppler_hz = -250.0;
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(
        config.intermediate_freq_hz,
        doppler_hz,
    );
    let epochs = vec![
        make_tracking_epoch(7, &config, 70, carrier_hz),
        make_tracking_epoch(7, &config, 71, carrier_hz),
    ];

    let (observations, diagnostics) = observations_from_tracking(&config, &epochs);

    assert!(diagnostics.is_empty(), "unexpected diagnostics: {diagnostics:?}");
    assert_eq!(observations.len(), 2);
    assert!(
        observations
            .iter()
            .all(|epoch| (epoch.sats[0].doppler_hz.0 - doppler_hz).abs() <= f64::EPSILON),
        "{observations:?}"
    );
}

#[test]
fn observations_from_tracking_results_preserve_distinct_satellite_dopplers() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 2_000.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let track_a = TrackingResult {
        sat: SatId { constellation: Constellation::Gps, prn: 3 },
        carrier_hz: 0.0,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "deferred".to_string(),
        acquisition_score: 0.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: 0.0,
        acq_to_track_state: "deferred".to_string(),
        epochs: vec![make_tracking_epoch(
            3,
            &config,
            70,
            crate::pipeline::doppler::carrier_hz_from_doppler_hz(
                config.intermediate_freq_hz,
                125.0,
            ),
        )],
        transitions: Vec::new(),
    };
    let track_b = TrackingResult {
        sat: SatId { constellation: Constellation::Gps, prn: 8 },
        carrier_hz: 0.0,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "deferred".to_string(),
        acquisition_score: 0.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: 0.0,
        acq_to_track_state: "deferred".to_string(),
        epochs: vec![make_tracking_epoch(
            8,
            &config,
            70,
            crate::pipeline::doppler::carrier_hz_from_doppler_hz(
                config.intermediate_freq_hz,
                -175.0,
            ),
        )],
        transitions: Vec::new(),
    };

    let report = observations_from_tracking_results(&config, &[track_a, track_b], 10);
    let epoch = report.output.first().expect("observation epoch");
    let dopplers =
        epoch.sats.iter().map(|sat| (sat.signal_id.sat.prn, sat.doppler_hz.0)).collect::<Vec<_>>();

    assert!(report.events.is_empty(), "unexpected diagnostics: {:?}", report.events);
    assert_eq!(dopplers, vec![(3, 125.0), (8, -175.0)]);
}

#[test]
fn observations_mark_carrier_phase_arc_start_and_continuity() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 125.0);
    let epochs = vec![
        make_tracking_epoch_with_phase(6, &config, 70, carrier_hz, 10.00),
        make_tracking_epoch_with_phase(6, &config, 71, carrier_hz, 10.125),
        make_tracking_epoch_with_phase(6, &config, 72, carrier_hz, 10.250),
    ];

    let (observations, diagnostics) = observations_from_tracking(&config, &epochs);

    assert!(diagnostics.is_empty(), "unexpected diagnostics: {diagnostics:?}");
    assert_eq!(observations.len(), 3);
    assert_eq!(observations[0].sats[0].metadata.carrier_phase_continuity, "arc_start");
    assert_eq!(observations[0].sats[0].metadata.carrier_phase_arc_start_epoch_idx, 70);
    assert_eq!(
        observations[0].sats[0].metadata.carrier_phase_arc_start_sample_index,
        observations[0].sats[0].metadata.time_tag_sample_index
    );
    assert_eq!(
        observations[1].sats[0]
            .metadata
            .cycle_slip_evidence
            .as_ref()
            .expect("cycle-slip evidence")
            .triggered_detectors(),
        Vec::<CycleSlipDetector>::new()
    );
    assert_eq!(observations[1].sats[0].metadata.carrier_phase_continuity, "continuous");
    assert_eq!(observations[2].sats[0].metadata.carrier_phase_continuity, "continuous");
    assert!((observations[1].sats[0].carrier_phase_cycles.0 - 10.125).abs() <= f64::EPSILON);
    assert_eq!(
        observations[2].sats[0].metadata.carrier_phase_arc_start_sample_index,
        observations[0].sats[0].metadata.carrier_phase_arc_start_sample_index
    );
    let first_arc = observations[0].sats[0]
        .metadata
        .carrier_phase_arc
        .as_ref()
        .expect("first carrier-phase arc");
    assert_eq!(first_arc.signal_id, observations[0].sats[0].signal_id);
    assert_eq!(first_arc.start_reason, "arc_start");
    assert!(first_arc.valid_for_smoothing);
    assert!(first_arc.valid_for_ambiguity);
    assert_eq!(
        observations[1].sats[0]
            .metadata
            .carrier_phase_arc
            .as_ref()
            .expect("continuous carrier-phase arc")
            .id,
        first_arc.id
    );
    assert_eq!(
        observations[2].sats[0]
            .metadata
            .carrier_phase_arc
            .as_ref()
            .expect("second continuous carrier-phase arc")
            .id,
        first_arc.id
    );
}

#[test]
fn observations_reset_carrier_phase_arc_after_unlock() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 100.0);
    let unlocked_epoch = TrackEpoch {
        epoch: Epoch { index: 71 },
        sample_index: epoch_sample_index(&config, 71),
        source_time: ReceiverSampleTrace::from_sample_index(
            epoch_sample_index(&config, 71),
            config.sampling_freq_hz,
        ),
        sat: SatId { constellation: Constellation::Gps, prn: 10 },
        carrier_hz: Hertz(carrier_hz),
        lock: false,
        pll_lock: false,
        dll_lock: false,
        fll_lock: false,
        lock_state: "lost".to_string(),
        ..TrackEpoch::default()
    };
    let track = TrackingResult {
        sat: SatId { constellation: Constellation::Gps, prn: 10 },
        carrier_hz: carrier_hz,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: carrier_hz,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![
            make_tracking_epoch_with_phase(10, &config, 70, carrier_hz, 8.0),
            unlocked_epoch,
            make_tracking_epoch_with_phase(10, &config, 72, carrier_hz, 0.5),
        ],
        transitions: Vec::new(),
    };

    let report = observations_from_tracking_results(&config, &[track], 10);
    let unlocked = &report.output[1].sats[0];
    let relocked = &report.output[2].sats[0];

    assert_eq!(unlocked.metadata.carrier_phase_continuity, "unusable");
    assert_eq!(unlocked.metadata.carrier_phase_arc_start_epoch_idx, 0);
    let unusable_arc =
        unlocked.metadata.carrier_phase_arc.as_ref().expect("unusable carrier-phase boundary");
    assert_eq!(unusable_arc.start_reason, "loss_of_lock");
    assert!(!unusable_arc.valid_for_smoothing);
    assert!(!unusable_arc.valid_for_ambiguity);
    assert_eq!(relocked.metadata.carrier_phase_continuity, "reset_after_unlock");
    assert_eq!(relocked.metadata.carrier_phase_arc_start_epoch_idx, 72);
    assert_eq!(
        relocked.metadata.carrier_phase_arc_start_sample_index,
        epoch_sample_index(&config, 72)
    );
    let relocked_arc =
        relocked.metadata.carrier_phase_arc.as_ref().expect("relocked carrier-phase arc");
    assert_eq!(relocked_arc.start_reason, "loss_of_lock");
    assert!(relocked_arc.valid_for_smoothing);
    assert_ne!(unusable_arc.id, relocked_arc.id);
    assert!(relocked.lock_flags.cycle_slip);
    let evidence = relocked.metadata.cycle_slip_evidence.as_ref().expect("cycle-slip evidence");
    assert!(evidence.detected);
    assert_eq!(evidence.primary_reason.as_deref(), Some("loss_of_lock"));
    assert!(evidence.triggered_detectors().contains(&CycleSlipDetector::TrackingLock));
    assert!((relocked.carrier_phase_cycles.0 - 0.5).abs() <= f64::EPSILON);
}

#[test]
fn observations_record_phase_innovation_cycle_slip_evidence() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 125.0);
    let epochs = vec![
        make_tracking_epoch_with_phase(11, &config, 70, carrier_hz, 10.0),
        make_tracking_epoch_with_phase(11, &config, 71, carrier_hz, 10.125),
        make_tracking_epoch_with_phase(11, &config, 72, carrier_hz, 10.750),
    ];

    let (observations, diagnostics) = observations_from_tracking(&config, &epochs);

    assert!(diagnostics.is_empty(), "unexpected diagnostics: {diagnostics:?}");
    let slipped = &observations[2].sats[0];
    let evidence = slipped.metadata.cycle_slip_evidence.as_ref().expect("cycle-slip evidence");
    assert!(slipped.lock_flags.cycle_slip);
    assert!(evidence.detected);
    assert_eq!(evidence.primary_reason.as_deref(), Some("carrier_phase_discontinuity"));
    assert!(
        evidence.triggered_detectors().contains(&CycleSlipDetector::DopplerPredictedPhase),
        "{evidence:?}"
    );
    assert!(
        evidence.triggered_detectors().contains(&CycleSlipDetector::PhaseInnovation),
        "{evidence:?}"
    );
    assert_eq!(evidence.detection_probability_budget, CYCLE_SLIP_DETECTION_PROBABILITY_BUDGET);
    assert_eq!(evidence.false_alarm_probability_budget, CYCLE_SLIP_FALSE_ALARM_PROBABILITY_BUDGET);
}

#[test]
fn observations_coast_carrier_phase_arc_through_recoverable_fade() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 125.0);
    let faded_epoch = TrackEpoch {
        lock: false,
        pll_lock: false,
        dll_lock: false,
        fll_lock: false,
        lock_state: "degraded".to_string(),
        lock_state_reason: Some("signal_fade".to_string()),
        ..make_tracking_epoch_with_phase(13, &config, 71, carrier_hz, 10.125)
    };
    let recovered_epoch = TrackEpoch {
        lock_state_reason: Some("fade_recovered".to_string()),
        ..make_tracking_epoch_with_phase(13, &config, 72, carrier_hz, 10.250)
    };
    let track = TrackingResult {
        sat: SatId { constellation: Constellation::Gps, prn: 13 },
        carrier_hz,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: carrier_hz,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![
            make_tracking_epoch_with_phase(13, &config, 70, carrier_hz, 10.0),
            faded_epoch,
            recovered_epoch,
        ],
        transitions: Vec::new(),
    };

    let report = observations_from_tracking_results(&config, &[track], 10);
    let first = &report.output[0].sats[0];
    let faded = &report.output[1].sats[0];
    let recovered = &report.output[2].sats[0];

    assert_eq!(first.metadata.carrier_phase_continuity, "arc_start");
    assert_eq!(faded.metadata.carrier_phase_continuity, "coasted");
    assert!(!faded.lock_flags.cycle_slip);
    let first_arc = first.metadata.carrier_phase_arc.as_ref().expect("first arc");
    assert_eq!(
        faded.metadata.carrier_phase_arc_start_sample_index,
        first.metadata.carrier_phase_arc_start_sample_index
    );
    assert_eq!(
        faded.metadata.carrier_phase_arc.as_ref().expect("coasted carrier-phase arc").id,
        first_arc.id
    );
    assert_eq!(recovered.metadata.carrier_phase_continuity, "continuous");
    assert!(!recovered.lock_flags.cycle_slip);
    assert_eq!(
        recovered.metadata.carrier_phase_arc_start_sample_index,
        first.metadata.carrier_phase_arc_start_sample_index
    );
    assert_eq!(
        recovered.metadata.carrier_phase_arc.as_ref().expect("recovered carrier-phase arc").id,
        first_arc.id
    );
}

#[test]
fn observations_start_new_carrier_phase_arc_after_reacquisition() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 100.0);
    let lost_epoch = TrackEpoch {
        epoch: Epoch { index: 71 },
        sample_index: epoch_sample_index(&config, 71),
        source_time: ReceiverSampleTrace::from_sample_index(
            epoch_sample_index(&config, 71),
            config.sampling_freq_hz,
        ),
        sat: SatId { constellation: Constellation::Gps, prn: 14 },
        carrier_hz: Hertz(carrier_hz),
        lock: false,
        pll_lock: false,
        dll_lock: false,
        fll_lock: false,
        lock_state: "lost".to_string(),
        lock_state_reason: Some("prompt_power_drop".to_string()),
        ..TrackEpoch::default()
    };
    let reacquired_epoch = TrackEpoch {
        lock_state_reason: Some("reacquired".to_string()),
        ..make_tracking_epoch_with_phase(14, &config, 72, carrier_hz, 0.5)
    };
    let settled_epoch = make_tracking_epoch_with_phase(14, &config, 73, carrier_hz, 0.6);
    let track = TrackingResult {
        sat: SatId { constellation: Constellation::Gps, prn: 14 },
        carrier_hz,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: carrier_hz,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![
            make_tracking_epoch_with_phase(14, &config, 70, carrier_hz, 8.0),
            lost_epoch,
            reacquired_epoch,
            settled_epoch,
        ],
        transitions: Vec::new(),
    };

    let report = observations_from_tracking_results(&config, &[track], 10);
    let lost = &report.output[1].sats[0];
    let reacquired = &report.output[2].sats[0];
    let settled = &report.output[3].sats[0];

    assert_eq!(lost.metadata.carrier_phase_continuity, "unusable");
    assert_eq!(reacquired.metadata.carrier_phase_continuity, "reset_after_reacquisition");
    assert_eq!(reacquired.metadata.carrier_phase_arc_start_epoch_idx, 72);
    let lost_arc = lost.metadata.carrier_phase_arc.as_ref().expect("lost carrier-phase boundary");
    let reacquired_arc =
        reacquired.metadata.carrier_phase_arc.as_ref().expect("reacquired carrier-phase arc");
    assert_eq!(lost_arc.start_reason, "loss_of_lock");
    assert!(!lost_arc.valid_for_smoothing);
    assert_eq!(reacquired_arc.start_reason, "reacquired");
    assert!(reacquired_arc.valid_for_smoothing);
    assert_ne!(lost_arc.id, reacquired_arc.id);
    assert!(reacquired.lock_flags.cycle_slip);
    assert_eq!(settled.metadata.carrier_phase_continuity, "continuous");
    assert_eq!(settled.metadata.carrier_phase_arc_start_epoch_idx, 72);
    assert_eq!(
        settled.metadata.carrier_phase_arc.as_ref().expect("settled carrier-phase arc").id,
        reacquired_arc.id
    );
}

#[test]
fn observations_reset_carrier_phase_arc_after_cycle_slip() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 125.0);
    let slip_epoch = TrackEpoch {
        cycle_slip: true,
        cycle_slip_reason: Some("phase_jump".to_string()),
        ..make_tracking_epoch_with_phase(12, &config, 71, carrier_hz, 21.0)
    };
    let track = TrackingResult {
        sat: SatId { constellation: Constellation::Gps, prn: 12 },
        carrier_hz,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: carrier_hz,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![
            make_tracking_epoch_with_phase(12, &config, 70, carrier_hz, 10.0),
            slip_epoch,
            make_tracking_epoch_with_phase(12, &config, 72, carrier_hz, 21.125),
        ],
        transitions: Vec::new(),
    };

    let report = observations_from_tracking_results(&config, &[track], 10);
    let slipped = &report.output[1].sats[0];
    let post_slip = &report.output[2].sats[0];

    assert_eq!(slipped.metadata.carrier_phase_continuity, "reset_after_cycle_slip");
    assert_eq!(slipped.metadata.carrier_phase_arc_start_epoch_idx, 71);
    let slipped_arc =
        slipped.metadata.carrier_phase_arc.as_ref().expect("slipped carrier-phase arc");
    assert_eq!(slipped_arc.start_reason, "phase_jump");
    assert!(slipped_arc.valid_for_smoothing);
    assert!(slipped.lock_flags.cycle_slip);
    assert_eq!(post_slip.metadata.carrier_phase_continuity, "continuous");
    assert_eq!(post_slip.metadata.carrier_phase_arc_start_epoch_idx, 71);
    assert_eq!(
        post_slip.metadata.carrier_phase_arc_start_sample_index,
        epoch_sample_index(&config, 71)
    );
    assert_eq!(
        post_slip.metadata.carrier_phase_arc.as_ref().expect("post-slip carrier-phase arc").id,
        slipped_arc.id
    );
}

#[test]
fn observations_advance_hatch_metadata_during_continuous_lock() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 125.0);
    let track = TrackingResult {
        sat: SatId { constellation: Constellation::Gps, prn: 18 },
        carrier_hz,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: carrier_hz,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![
            make_tracking_epoch_with_alignment(18, &config, 70, carrier_hz, 10.0, 68, 0.0),
            make_tracking_epoch_with_alignment(18, &config, 71, carrier_hz, 10.125, 68, 0.0),
            make_tracking_epoch_with_alignment(18, &config, 72, carrier_hz, 10.250, 68, 0.0),
        ],
        transitions: Vec::new(),
    };

    let report = observations_from_tracking_results(&config, &[track], 3);
    let sats = report
        .output
        .iter()
        .map(|epoch| epoch.sats.first().expect("observation satellite"))
        .collect::<Vec<_>>();

    assert_eq!(sats.len(), 3);
    assert!(sats.iter().all(|sat| sat.metadata.smoothing_window == 3));
    assert_eq!(
        sats.iter().map(|sat| sat.metadata.smoothing_age).collect::<Vec<_>>(),
        vec![1, 2, 3]
    );
    assert!(sats.iter().all(|sat| sat.metadata.smoothing_resets == 0));
}

#[test]
fn observations_leave_hatch_smoothing_uninitialized_during_unlock() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 125.0);
    let unlocked_epoch = TrackEpoch {
        lock: false,
        pll_lock: false,
        dll_lock: false,
        fll_lock: false,
        lock_state: "lost".to_string(),
        lock_state_reason: Some("prompt_power_drop".to_string()),
        ..make_tracking_epoch_with_alignment(19, &config, 71, carrier_hz, 10.125, 68, 0.0)
    };
    let track = TrackingResult {
        sat: SatId { constellation: Constellation::Gps, prn: 19 },
        carrier_hz,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: carrier_hz,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![
            make_tracking_epoch_with_alignment(19, &config, 70, carrier_hz, 10.0, 68, 0.0),
            unlocked_epoch,
            make_tracking_epoch_with_alignment(19, &config, 72, carrier_hz, 10.250, 68, 0.0),
        ],
        transitions: Vec::new(),
    };

    let report = observations_from_tracking_results(&config, &[track], 10);
    let locked = &report.output[0].sats[0];
    let unlocked = &report.output[1].sats[0];
    let relocked = &report.output[2].sats[0];
    let expected_raw = aligned_pseudorange_m(&config, 68, 0.0);

    assert_eq!(locked.metadata.smoothing_age, 1);
    assert_eq!(locked.metadata.smoothing_resets, 0);
    assert_eq!(unlocked.metadata.smoothing_age, 0);
    assert_eq!(unlocked.metadata.smoothing_resets, 1);
    assert!((unlocked.pseudorange_m.0 - expected_raw).abs() <= f64::EPSILON);
    assert_eq!(relocked.metadata.smoothing_age, 1);
    assert_eq!(relocked.metadata.smoothing_resets, 1);
    assert!((relocked.pseudorange_m.0 - expected_raw).abs() <= f64::EPSILON);
}

#[test]
fn observations_restart_hatch_smoothing_on_detected_cycle_slip() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 125.0);
    let track = TrackingResult {
        sat: SatId { constellation: Constellation::Gps, prn: 20 },
        carrier_hz,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: carrier_hz,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![
            make_tracking_epoch_with_alignment(20, &config, 70, carrier_hz, 10.0, 68, 0.0),
            make_tracking_epoch_with_alignment(20, &config, 71, carrier_hz, 10.125, 68, 0.0),
            make_tracking_epoch_with_alignment(20, &config, 72, carrier_hz, 10.250, 68, 0.02),
            make_tracking_epoch_with_alignment(20, &config, 73, carrier_hz, 10.375, 68, 0.02),
        ],
        transitions: Vec::new(),
    };

    let report = observations_from_tracking_results(&config, &[track], 10);
    let sats = report.output.iter().map(|epoch| &epoch.sats[0]).collect::<Vec<_>>();
    let slipped = sats[2];
    let post_slip = sats[3];
    let expected_raw = aligned_pseudorange_m(&config, 68, 0.02);

    assert_eq!(
        sats.iter().map(|sat| sat.metadata.smoothing_age).collect::<Vec<_>>(),
        vec![1, 2, 1, 2]
    );
    assert_eq!(
        sats.iter().map(|sat| sat.metadata.smoothing_resets).collect::<Vec<_>>(),
        vec![0, 0, 1, 1]
    );
    assert!(slipped.lock_flags.cycle_slip);
    let evidence = slipped.metadata.cycle_slip_evidence.as_ref().expect("cycle-slip evidence");
    let detectors = evidence.triggered_detectors();
    let divergence = evidence
        .contributors
        .iter()
        .find(|contributor| contributor.detector == CycleSlipDetector::CodeCarrierDivergence)
        .expect("code-carrier divergence contributor");
    assert!(detectors.contains(&CycleSlipDetector::CodeCarrierDivergence));
    assert!(divergence.triggered);
    assert!(
        divergence.value.expect("divergence value")
            > divergence.threshold.expect("divergence threshold")
    );
    assert_eq!(divergence.units, "m");
    assert_eq!(evidence.primary_reason.as_deref(), Some("code_carrier_divergence"));
    assert_eq!(evidence.detection_probability_budget, CYCLE_SLIP_DETECTION_PROBABILITY_BUDGET);
    assert_eq!(evidence.false_alarm_probability_budget, CYCLE_SLIP_FALSE_ALARM_PROBABILITY_BUDGET);
    assert!((slipped.pseudorange_m.0 - expected_raw).abs() <= f64::EPSILON);
    assert_eq!(post_slip.metadata.smoothing_age, 2);
}

#[test]
fn observations_refuse_hatch_smoothing_across_invalid_carrier_phase_arc() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 125.0);
    let carrier_invalid = TrackEpoch {
        pll_lock: false,
        lock_state: "degraded".to_string(),
        lock_state_reason: Some("carrier_loop_unstable".to_string()),
        ..make_tracking_epoch_with_alignment(21, &config, 71, carrier_hz, 10.125, 68, 0.01)
    };
    let track = TrackingResult {
        sat: SatId { constellation: Constellation::Gps, prn: 21 },
        carrier_hz,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: carrier_hz,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![
            make_tracking_epoch_with_alignment(21, &config, 70, carrier_hz, 10.0, 68, 0.0),
            carrier_invalid,
            make_tracking_epoch_with_alignment(21, &config, 72, carrier_hz, 0.5, 68, 0.02),
        ],
        transitions: Vec::new(),
    };

    let report = observations_from_tracking_results(&config, &[track], 10);
    let sats = report.output.iter().map(|epoch| &epoch.sats[0]).collect::<Vec<_>>();
    let first_arc = sats[0].metadata.carrier_phase_arc.as_ref().expect("first carrier-phase arc");
    let invalid_arc =
        sats[1].metadata.carrier_phase_arc.as_ref().expect("invalid carrier-phase boundary");
    let reset_arc = sats[2].metadata.carrier_phase_arc.as_ref().expect("reset carrier-phase arc");

    assert_eq!(
        sats.iter().map(|sat| sat.metadata.smoothing_age).collect::<Vec<_>>(),
        vec![1, 0, 1]
    );
    assert_eq!(
        sats.iter().map(|sat| sat.metadata.smoothing_resets).collect::<Vec<_>>(),
        vec![0, 1, 1]
    );
    assert_eq!(sats[1].metadata.carrier_phase_continuity, "unusable");
    assert_eq!(invalid_arc.start_reason, "loss_of_lock");
    assert!(!invalid_arc.valid_for_smoothing);
    assert_ne!(first_arc.id, invalid_arc.id);
    assert_eq!(sats[2].metadata.carrier_phase_continuity, "reset_after_unlock");
    assert!(reset_arc.valid_for_smoothing);
    assert_ne!(invalid_arc.id, reset_arc.id);
}
