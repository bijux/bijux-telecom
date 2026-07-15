use super::*;
use std::collections::BTreeSet;

use bijux_gnss_core::api::{
    Chips, Epoch, Hertz, Meters, SatId, SignalDelayAlignment, TrackingTransmitTime,
    TrackingUncertainty,
};
use bijux_gnss_signal::api::{
    registered_signal_registry_entries, signal_cycles_to_meters, signal_meters_to_cycles,
};

#[path = "tests/measurement_variance.rs"]
mod measurement_variance;
#[path = "tests/pseudorange_resolution.rs"]
mod pseudorange_resolution;
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

fn residual_epoch(
    report: &StepReport<Vec<ObservationResidualEpochReport>>,
    epoch_idx: u64,
) -> &ObservationResidualEpochReport {
    report.output.iter().find(|epoch| epoch.epoch_idx == epoch_idx).expect("residual epoch")
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

#[test]
fn observations_preserve_tracking_cn0_on_accepted_rows() {
    let config = ReceiverPipelineConfig::default();
    let sat = SatId { constellation: Constellation::Gps, prn: 10 };
    let expected_cn0_dbhz = 47.5;
    let track = TrackingResult {
        sat,
        carrier_hz: 0.0,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: 0.0,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![TrackEpoch {
            epoch: Epoch { index: 70 },
            sample_index: epoch_sample_index(&config, 70),
            source_time: ReceiverSampleTrace::from_sample_index(
                epoch_sample_index(&config, 70),
                config.sampling_freq_hz,
            ),
            sat,
            lock: true,
            cn0_dbhz: expected_cn0_dbhz,
            pll_lock: true,
            dll_lock: true,
            fll_lock: true,
            lock_state: "tracking".to_string(),
            lock_state_reason: Some("stable_tracking".to_string()),
            tracking_uncertainty: Some(test_tracking_uncertainty()),
            ..TrackEpoch::default()
        }],
        transitions: Vec::new(),
    };

    let report = observations_from_tracking_results(&config, &[track], 10);
    let epoch = report.output.first().expect("observation epoch");
    let sat = epoch.sats.first().expect("observation satellite");

    assert_eq!(sat.observation_status, ObservationStatus::Accepted);
    assert!((sat.cn0_dbhz - expected_cn0_dbhz).abs() <= f64::EPSILON, "{sat:?}");
}

#[test]
fn observations_record_locked_observation_lock_state() {
    let config = ReceiverPipelineConfig::default();
    let epoch = make_observation_ready_epoch(14, &config, 70);
    let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
    let sat = report.output[0].sats.first().expect("observation satellite");

    assert_eq!(sat.metadata.observation_lock_state, "locked");
    assert_eq!(sat.metadata.observation_lock_reason.as_deref(), Some("stable_tracking"));
    assert_eq!(sat.metadata.tracking_lock_state, sat.metadata.observation_lock_state);
}

#[test]
fn observations_record_degraded_observation_lock_state() {
    let config = ReceiverPipelineConfig::default();
    let mut epoch = make_observation_ready_epoch(15, &config, 70);
    epoch.lock_state = "degraded".to_string();
    epoch.lock_state_reason = Some("signal_fade".to_string());
    let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
    let sat = report.output[0].sats.first().expect("observation satellite");

    assert_eq!(sat.metadata.observation_lock_state, "degraded");
    assert_eq!(sat.metadata.observation_lock_reason.as_deref(), Some("signal_fade"));
    assert_eq!(sat.metadata.tracking_lock_state, sat.metadata.observation_lock_state);
}

#[test]
fn observations_record_lost_observation_lock_state() {
    let config = ReceiverPipelineConfig::default();
    let mut epoch = make_observation_ready_epoch(16, &config, 70);
    epoch.lock = false;
    epoch.pll_lock = false;
    epoch.dll_lock = false;
    epoch.fll_lock = false;
    epoch.lock_state = "lost".to_string();
    epoch.lock_state_reason = Some("prompt_power_drop".to_string());
    let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
    let sat = report.output[0].sats.first().expect("observation satellite");

    assert_eq!(sat.metadata.observation_lock_state, "lost");
    assert_eq!(sat.metadata.observation_lock_reason.as_deref(), Some("prompt_power_drop"));
    assert_eq!(sat.metadata.tracking_lock_state, sat.metadata.observation_lock_state);
}

#[test]
fn observations_record_reacquired_observation_lock_state() {
    let config = ReceiverPipelineConfig::default();
    let mut epoch = make_observation_ready_epoch(17, &config, 70);
    epoch.lock_state_reason = Some("reacquired".to_string());
    let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
    let sat = report.output[0].sats.first().expect("observation satellite");

    assert_eq!(sat.metadata.observation_lock_state, "reacquired");
    assert_eq!(sat.metadata.observation_lock_reason.as_deref(), Some("reacquired"));
    assert_eq!(sat.metadata.tracking_lock_state, sat.metadata.observation_lock_state);
}

#[test]
fn observations_record_cycle_slip_observation_lock_state() {
    let config = ReceiverPipelineConfig::default();
    let mut epoch = make_observation_ready_epoch(18, &config, 70);
    epoch.lock = false;
    epoch.pll_lock = false;
    epoch.dll_lock = false;
    epoch.fll_lock = false;
    epoch.cycle_slip = true;
    epoch.cycle_slip_reason = Some("phase_jump".to_string());
    epoch.lock_state = "lost".to_string();
    epoch.lock_state_reason = Some("phase_jump".to_string());
    let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
    let sat = report.output[0].sats.first().expect("observation satellite");

    assert_eq!(sat.metadata.observation_lock_state, "cycle_slip");
    assert_eq!(sat.metadata.observation_lock_reason.as_deref(), Some("phase_jump"));
    assert_eq!(sat.metadata.tracking_lock_state, sat.metadata.observation_lock_state);
}

#[test]
fn observations_declare_if_relative_doppler_contract() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 2_000.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Gps, prn: 10 };
    let expected_doppler_hz = -250.0;
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(
        config.intermediate_freq_hz,
        expected_doppler_hz,
    );
    let track = TrackingResult {
        sat,
        carrier_hz,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: carrier_hz,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![make_tracking_epoch(10, &config, 70, carrier_hz)],
        transitions: Vec::new(),
    };

    let report = observations_from_tracking_results(&config, &[track], 10);
    let epoch = report.output.first().expect("observation epoch");
    let sat = epoch.sats.first().expect("observation satellite");

    assert_eq!(
        sat.metadata.doppler_model,
        bijux_gnss_core::api::OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET
    );
    assert!((sat.doppler_hz.0 - expected_doppler_hz).abs() <= f64::EPSILON, "{sat:?}");
}

#[test]
fn observations_emit_glonass_l1_signal_identity_and_fdma_relative_doppler() {
    let channel = GlonassFrequencyChannel::new(-4).expect("channel -4 must be valid");
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 2_044_000.0,
        intermediate_freq_hz: GPS_L1_CA_CARRIER_HZ.value() - glonass_l1_carrier_hz(channel).value(),
        code_freq_basis_hz: 511_000.0,
        code_length: 511,
        ..ReceiverPipelineConfig::default()
    };
    let sat = SatId { constellation: Constellation::Glonass, prn: 8 };
    let expected_doppler_hz = 125.0;
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, expected_doppler_hz);
    let mut epoch = make_tracking_epoch(8, &config, 70, carrier_hz);
    epoch.sat = sat;
    epoch.signal_band = SignalBand::L1;
    epoch.glonass_frequency_channel = Some(channel);
    epoch.code_rate_hz = Hertz(511_000.0);
    let track = TrackingResult {
        sat,
        carrier_hz,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: carrier_hz,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![epoch],
        transitions: Vec::new(),
    };

    let report = observations_from_tracking_results(&config, &[track], 10);
    let epoch = report.output.first().expect("observation epoch");
    let obs_sat = epoch.sats.first().expect("observation satellite");

    assert_eq!(obs_sat.signal_id.sat, sat);
    assert_eq!(obs_sat.signal_id.band, SignalBand::L1);
    assert_eq!(obs_sat.signal_id.code, SignalCode::Unknown);
    assert_eq!(obs_sat.metadata.signal.code, SignalCode::Unknown);
    assert_eq!(obs_sat.metadata.signal.carrier_hz.value(), glonass_l1_carrier_hz(channel).value());
    assert!((obs_sat.doppler_hz.0 - expected_doppler_hz).abs() <= f64::EPSILON, "{obs_sat:?}");
}

#[test]
fn observations_emit_gps_l2c_signal_identity_and_civil_code_pseudorange() {
    let signal = signal_spec_gps_l2c();
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 5_115_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: signal.code_rate_hz,
        code_length: 10230,
        ..ReceiverPipelineConfig::default()
    };
    let whole_code_periods = 4;
    let aligned_code_phase_chips = 768.0;
    let expected_pseudorange_m = aligned_pseudorange_m_for_signal(
        signal,
        10230,
        whole_code_periods,
        aligned_code_phase_chips,
    );
    let epoch = TrackEpoch {
        sat: SatId { constellation: Constellation::Gps, prn: 11 },
        signal_band: SignalBand::L2,
        carrier_hz: Hertz(tracked_signal_center_hz(config.intermediate_freq_hz, signal)),
        code_rate_hz: Hertz(signal.code_rate_hz),
        code_phase_samples: Chips(test_tracking_code_phase_samples_for_signal(
            &config,
            signal,
            10230,
            aligned_code_phase_chips,
        )),
        signal_delay_alignment: Some(SignalDelayAlignment {
            whole_code_periods,
            sample_delay_samples: 0,
            source: "synthetic_truth".to_string(),
        }),
        ..make_tracking_epoch_with_phase(
            11,
            &config,
            70,
            tracked_signal_center_hz(config.intermediate_freq_hz, signal),
            0.0,
        )
    };
    let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
    let obs_sat = report.output[0].sats.first().expect("observation satellite");

    assert_eq!(obs_sat.signal_id.band, SignalBand::L2);
    assert_eq!(obs_sat.signal_id.code, SignalCode::L2C);
    assert_eq!(obs_sat.metadata.signal, signal);
    assert!((obs_sat.metadata.signal.code_rate_hz - 511_500.0).abs() <= f64::EPSILON);
    assert_eq!(obs_sat.metadata.pseudorange_model, "tracked_code_phase_alignment");
    assert!((obs_sat.pseudorange_m.0 - expected_pseudorange_m).abs() <= 1.0e-6, "{obs_sat:?}");
}

#[test]
fn observations_subtract_known_sample_delay_from_aligned_pseudorange() {
    let signal = signal_spec_gps_l1_ca();
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: signal.code_rate_hz,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let whole_code_periods = 12;
    let aligned_code_phase_chips = 512.0;
    let sample_delay_samples = 20;
    let samples_per_chip = config.sampling_freq_hz / signal.code_rate_hz;
    let delayed_code_phase_chips =
        aligned_code_phase_chips + sample_delay_samples as f64 / samples_per_chip;
    let expected_pseudorange_m = aligned_pseudorange_m_for_signal(
        signal,
        1023,
        whole_code_periods,
        aligned_code_phase_chips,
    );
    let epoch = TrackEpoch {
        sat: SatId { constellation: Constellation::Gps, prn: 7 },
        signal_band: SignalBand::L1,
        carrier_hz: Hertz(tracked_signal_center_hz(config.intermediate_freq_hz, signal)),
        code_rate_hz: Hertz(signal.code_rate_hz),
        code_phase_samples: Chips(test_tracking_code_phase_samples_for_signal(
            &config,
            signal,
            1023,
            delayed_code_phase_chips,
        )),
        signal_delay_alignment: Some(SignalDelayAlignment {
            whole_code_periods,
            sample_delay_samples,
            source: "synthetic_truth".to_string(),
        }),
        ..make_tracking_epoch_with_phase(
            7,
            &config,
            70,
            tracked_signal_center_hz(config.intermediate_freq_hz, signal),
            0.0,
        )
    };

    let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
    let obs_sat = report.output[0].sats.first().expect("observation satellite");

    assert_eq!(obs_sat.metadata.pseudorange_model, "tracked_code_phase_alignment");
    assert!((obs_sat.pseudorange_m.0 - expected_pseudorange_m).abs() <= 1.0e-6, "{obs_sat:?}");
}

#[test]
fn observations_emit_gps_l5_signal_identity_and_aligned_pseudorange() {
    let signal = signal_spec_gps_l5();
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: signal.code_rate_hz,
        code_length: 10230,
        ..ReceiverPipelineConfig::default()
    };
    let whole_code_periods = 8;
    let aligned_code_phase_chips = 2_048.0;
    let expected_pseudorange_m = aligned_pseudorange_m_for_signal(
        signal,
        10230,
        whole_code_periods,
        aligned_code_phase_chips,
    );
    let epoch = TrackEpoch {
        sat: SatId { constellation: Constellation::Gps, prn: 12 },
        signal_band: SignalBand::L5,
        carrier_hz: Hertz(tracked_signal_center_hz(config.intermediate_freq_hz, signal)),
        code_rate_hz: Hertz(signal.code_rate_hz),
        code_phase_samples: Chips(test_tracking_code_phase_samples_for_signal(
            &config,
            signal,
            10230,
            aligned_code_phase_chips,
        )),
        signal_delay_alignment: Some(SignalDelayAlignment {
            whole_code_periods,
            sample_delay_samples: 0,
            source: "synthetic_truth".to_string(),
        }),
        ..make_tracking_epoch_with_phase(
            12,
            &config,
            70,
            tracked_signal_center_hz(config.intermediate_freq_hz, signal),
            0.0,
        )
    };
    let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
    let obs_sat = report.output[0].sats.first().expect("observation satellite");

    assert_eq!(obs_sat.signal_id.band, SignalBand::L5);
    assert_eq!(obs_sat.signal_id.code, SignalCode::L5I);
    assert_eq!(obs_sat.metadata.signal, signal);
    assert!((obs_sat.metadata.signal.code_rate_hz - 10_230_000.0).abs() <= f64::EPSILON);
    assert_eq!(obs_sat.metadata.pseudorange_model, "tracked_code_phase_alignment");
    assert!((obs_sat.pseudorange_m.0 - expected_pseudorange_m).abs() <= 1.0e-6, "{obs_sat:?}");
}

#[test]
fn observations_preserve_gps_l5_q_signal_identity_and_aligned_pseudorange() {
    let signal = signal_spec_gps_l5_q();
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: signal.code_rate_hz,
        code_length: 10230,
        ..ReceiverPipelineConfig::default()
    };
    let whole_code_periods = 8;
    let aligned_code_phase_chips = 2_048.0;
    let expected_pseudorange_m = aligned_pseudorange_m_for_signal(
        signal,
        10230,
        whole_code_periods,
        aligned_code_phase_chips,
    );
    let epoch = TrackEpoch {
        sat: SatId { constellation: Constellation::Gps, prn: 12 },
        signal_band: SignalBand::L5,
        signal_code: SignalCode::L5Q,
        carrier_hz: Hertz(tracked_signal_center_hz(config.intermediate_freq_hz, signal)),
        code_rate_hz: Hertz(signal.code_rate_hz),
        code_phase_samples: Chips(test_tracking_code_phase_samples_for_signal(
            &config,
            signal,
            10230,
            aligned_code_phase_chips,
        )),
        signal_delay_alignment: Some(SignalDelayAlignment {
            whole_code_periods,
            sample_delay_samples: 0,
            source: "synthetic_truth".to_string(),
        }),
        ..make_tracking_epoch_with_phase(
            12,
            &config,
            70,
            tracked_signal_center_hz(config.intermediate_freq_hz, signal),
            0.0,
        )
    };
    let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
    let obs_sat = report.output[0].sats.first().expect("observation satellite");

    assert_eq!(obs_sat.signal_id.band, SignalBand::L5);
    assert_eq!(obs_sat.signal_id.code, SignalCode::L5Q);
    assert_eq!(obs_sat.metadata.signal, signal);
    assert!((obs_sat.metadata.signal.code_rate_hz - 10_230_000.0).abs() <= f64::EPSILON);
    assert_eq!(obs_sat.metadata.pseudorange_model, "tracked_code_phase_alignment");
    assert!((obs_sat.pseudorange_m.0 - expected_pseudorange_m).abs() <= 1.0e-6, "{obs_sat:?}");
}

#[test]
fn observations_emit_galileo_e5_signal_identity_and_aligned_pseudorange() {
    let signal = signal_spec_galileo_e5a();
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 10_230_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: signal.code_rate_hz,
        code_length: 10230,
        ..ReceiverPipelineConfig::default()
    };
    let whole_code_periods = 6;
    let aligned_code_phase_chips = 1_536.0;
    let expected_pseudorange_m = aligned_pseudorange_m_for_signal(
        signal,
        10230,
        whole_code_periods,
        aligned_code_phase_chips,
    );
    let expected_phase_cycles = signal_meters_to_cycles(Meters(expected_pseudorange_m), signal).0;
    let epoch = TrackEpoch {
        sat: SatId { constellation: Constellation::Galileo, prn: 11 },
        signal_band: SignalBand::E5,
        carrier_hz: Hertz(tracked_signal_center_hz(config.intermediate_freq_hz, signal)),
        code_rate_hz: Hertz(signal.code_rate_hz),
        code_phase_samples: Chips(test_tracking_code_phase_samples_for_signal(
            &config,
            signal,
            10230,
            aligned_code_phase_chips,
        )),
        signal_delay_alignment: Some(SignalDelayAlignment {
            whole_code_periods,
            sample_delay_samples: 0,
            source: "synthetic_truth".to_string(),
        }),
        ..make_tracking_epoch_with_phase(
            11,
            &config,
            70,
            tracked_signal_center_hz(config.intermediate_freq_hz, signal),
            expected_phase_cycles,
        )
    };
    let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
    let obs_sat = report.output[0].sats.first().expect("observation satellite");

    assert_eq!(obs_sat.signal_id.band, SignalBand::E5);
    assert_eq!(obs_sat.signal_id.code, SignalCode::E5a);
    assert_eq!(obs_sat.metadata.signal, signal);
    assert!((obs_sat.metadata.signal.code_rate_hz - 10_230_000.0).abs() <= f64::EPSILON);
    assert_eq!(obs_sat.metadata.pseudorange_model, "tracked_code_phase_alignment");
    assert!((obs_sat.pseudorange_m.0 - expected_pseudorange_m).abs() <= 1.0e-6, "{obs_sat:?}");
    assert!(
        (signal_cycles_to_meters(obs_sat.carrier_phase_cycles, obs_sat.metadata.signal).0
            - expected_pseudorange_m)
            .abs()
            <= 1.0e-6,
        "{obs_sat:?}"
    );
}

#[test]
fn observations_emit_beidou_b2_signal_identity_and_aligned_pseudorange() {
    let signal = signal_spec_beidou_b2i();
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 2_046_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: signal.code_rate_hz,
        code_length: 2046,
        ..ReceiverPipelineConfig::default()
    };
    let whole_code_periods = 8;
    let aligned_code_phase_chips = 768.0;
    let expected_pseudorange_m = aligned_pseudorange_m_for_signal(
        signal,
        2046,
        whole_code_periods,
        aligned_code_phase_chips,
    );
    let expected_phase_cycles = signal_meters_to_cycles(Meters(expected_pseudorange_m), signal).0;
    let epoch = TrackEpoch {
        sat: SatId { constellation: Constellation::Beidou, prn: 11 },
        signal_band: SignalBand::B2,
        carrier_hz: Hertz(tracked_signal_center_hz(config.intermediate_freq_hz, signal)),
        code_rate_hz: Hertz(signal.code_rate_hz),
        code_phase_samples: Chips(test_tracking_code_phase_samples_for_signal(
            &config,
            signal,
            2046,
            aligned_code_phase_chips,
        )),
        signal_delay_alignment: Some(SignalDelayAlignment {
            whole_code_periods,
            sample_delay_samples: 0,
            source: "synthetic_truth".to_string(),
        }),
        ..make_tracking_epoch_with_phase(
            11,
            &config,
            70,
            tracked_signal_center_hz(config.intermediate_freq_hz, signal),
            expected_phase_cycles,
        )
    };
    let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
    let obs_sat = report.output[0].sats.first().expect("observation satellite");

    assert_eq!(obs_sat.signal_id.band, SignalBand::B2);
    assert_eq!(obs_sat.signal_id.code, SignalCode::B2I);
    assert_eq!(obs_sat.metadata.signal, signal);
    assert!((obs_sat.metadata.signal.code_rate_hz - 2_046_000.0).abs() <= f64::EPSILON);
    assert_eq!(obs_sat.metadata.pseudorange_model, "tracked_code_phase_alignment");
    assert!((obs_sat.pseudorange_m.0 - expected_pseudorange_m).abs() <= 1.0e-6, "{obs_sat:?}");
    assert!(
        (signal_cycles_to_meters(obs_sat.carrier_phase_cycles, obs_sat.metadata.signal).0
            - expected_pseudorange_m)
            .abs()
            <= 1.0e-6,
        "{obs_sat:?}"
    );
}

#[test]
fn observation_metadata_sets_support_and_uncertainty_classes() {
    let config = ReceiverPipelineConfig::default();
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let mut weak = make_observation_ready_epoch(7, &config, 70);
    weak.cn0_dbhz = 20.0;
    let mut missing = make_observation_ready_epoch(8, &config, 70);
    missing.lock = false;
    missing.cn0_dbhz = 45.0;
    missing.lock_state = "lost".to_string();
    let tracks = vec![
        TrackingResult {
            sat,
            carrier_hz: 0.0,
            code_phase_samples: 0.0,
            acquisition_hypothesis: "deferred".to_string(),
            acquisition_score: 0.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: 0.0,
            acq_to_track_state: "deferred".to_string(),
            epochs: vec![weak],
            transitions: Vec::new(),
        },
        TrackingResult {
            sat: SatId { constellation: Constellation::Gps, prn: 8 },
            carrier_hz: 0.0,
            code_phase_samples: 0.0,
            acquisition_hypothesis: "deferred".to_string(),
            acquisition_score: 0.0,
            acquisition_code_phase_samples: 0,
            acquisition_carrier_hz: 0.0,
            acq_to_track_state: "deferred".to_string(),
            epochs: vec![missing],
            transitions: Vec::new(),
        },
    ];
    let report = observations_from_tracking_results(&config, &tracks, 10);
    let epoch = report.output.iter().find(|row| row.epoch_idx == 70).expect("epoch");
    let labels = epoch
        .sats
        .iter()
        .map(|sat| {
            (
                sat.metadata.observation_support_class.clone(),
                sat.metadata.observation_uncertainty_class.clone(),
            )
        })
        .collect::<Vec<_>>();
    assert!(labels
        .iter()
        .any(|(support, uncertainty)| support == "degraded" && uncertainty == "high"));
    assert!(labels
        .iter()
        .any(|(support, uncertainty)| support == "degraded" && uncertainty == "low"));
}

#[test]
fn apply_epoch_decision_refuses_malformed_duplicate_signal_set() {
    let mut epoch = nav_observation_epoch_fixture(0);
    let duplicate = epoch.sats[0].clone();
    epoch.sats.push(duplicate);
    apply_epoch_decision(&mut epoch);
    assert_eq!(epoch.decision, ObservationEpochDecision::Rejected);
    let reason = epoch.decision_reason.expect("decision reason");
    assert!(reason.contains("malformed_observation_set"));
    assert!(reason.contains("duplicate signal_id"));
}

#[test]
fn observations_reject_sample_rate_mismatch_tracking_reason() {
    let config = ReceiverPipelineConfig::default();
    let sat = SatId { constellation: Constellation::Gps, prn: 11 };
    let mismatch_epoch = TrackEpoch {
        epoch: Epoch { index: 0 },
        sample_index: 0,
        sat,
        lock: true,
        cn0_dbhz: 45.0,
        dll_lock: false,
        pll_lock: false,
        lock_state: "tracking".to_string(),
        lock_state_reason: Some("sample_rate_mismatch".to_string()),
        ..TrackEpoch::default()
    };
    let track = TrackingResult {
        sat,
        carrier_hz: 0.0,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: 0.0,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![mismatch_epoch],
        transitions: Vec::new(),
    };

    let report = observations_from_tracking_results(&config, &[track], 10);
    let epoch = report.output.first().expect("observation epoch");
    let sat = epoch.sats.first().expect("observation satellite");

    assert_eq!(sat.observation_status, ObservationStatus::Inconsistent);
    assert!(sat.observation_reject_reasons.iter().any(|reason| reason == "sample_rate_mismatch"));
    assert_eq!(epoch.decision, ObservationEpochDecision::Rejected);
    assert_eq!(epoch.decision_reason.as_deref(), Some("inconsistent_observable"));
}

#[test]
fn observations_preserve_tracking_cn0_on_unlock_rows() {
    let config = ReceiverPipelineConfig::default();
    let sat = SatId { constellation: Constellation::Gps, prn: 11 };
    let expected_cn0_dbhz = 31.25;
    let unlocked_epoch = TrackEpoch {
        epoch: Epoch { index: 70 },
        sample_index: epoch_sample_index(&config, 70),
        source_time: ReceiverSampleTrace::from_sample_index(
            epoch_sample_index(&config, 70),
            config.sampling_freq_hz,
        ),
        sat,
        lock: false,
        pll_lock: false,
        dll_lock: false,
        fll_lock: false,
        cn0_dbhz: expected_cn0_dbhz,
        lock_state: "lost".to_string(),
        lock_state_reason: Some("prompt_power_drop".to_string()),
        ..TrackEpoch::default()
    };
    let report =
        observations_from_tracking_results(&config, &[track_from_epoch(unlocked_epoch)], 10);
    let epoch = report.output.first().expect("observation epoch");
    let sat = epoch.sats.first().expect("observation satellite");

    assert_eq!(sat.observation_status, ObservationStatus::Missing);
    assert!((sat.cn0_dbhz - expected_cn0_dbhz).abs() <= f64::EPSILON, "{sat:?}");
}

#[test]
fn observations_preserve_tracking_cn0_on_inconsistent_rows() {
    let config = ReceiverPipelineConfig::default();
    let sat = SatId { constellation: Constellation::Gps, prn: 12 };
    let expected_cn0_dbhz = 36.5;
    let mismatch_epoch = TrackEpoch {
        epoch: Epoch { index: 70 },
        sample_index: epoch_sample_index(&config, 70),
        source_time: ReceiverSampleTrace::from_sample_index(
            epoch_sample_index(&config, 70),
            config.sampling_freq_hz,
        ),
        sat,
        lock: true,
        pll_lock: false,
        dll_lock: false,
        cn0_dbhz: expected_cn0_dbhz,
        lock_state: "tracking".to_string(),
        lock_state_reason: Some("sample_rate_mismatch".to_string()),
        ..TrackEpoch::default()
    };
    let report =
        observations_from_tracking_results(&config, &[track_from_epoch(mismatch_epoch)], 10);
    let epoch = report.output.first().expect("observation epoch");
    let sat = epoch.sats.first().expect("observation satellite");

    assert_eq!(sat.observation_status, ObservationStatus::Inconsistent);
    assert!((sat.cn0_dbhz - expected_cn0_dbhz).abs() <= f64::EPSILON, "{sat:?}");
}

#[test]
fn observations_keep_doppler_on_tracking_unlock_epoch() {
    let config = ReceiverPipelineConfig::default();
    let sat = SatId { constellation: Constellation::Gps, prn: 12 };
    let expected_doppler_hz = 125.0;
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(
        config.intermediate_freq_hz,
        expected_doppler_hz,
    );
    let unlocked_epoch = TrackEpoch {
        epoch: Epoch { index: 70 },
        sample_index: epoch_sample_index(&config, 70),
        source_time: ReceiverSampleTrace::from_sample_index(
            epoch_sample_index(&config, 70),
            config.sampling_freq_hz,
        ),
        sat,
        carrier_hz: Hertz(carrier_hz),
        code_rate_hz: Hertz(config.code_freq_basis_hz),
        lock: false,
        pll_lock: false,
        dll_lock: false,
        fll_lock: false,
        cn0_dbhz: 45.0,
        lock_state: "lost".to_string(),
        lock_state_reason: Some("prompt_power_drop".to_string()),
        ..TrackEpoch::default()
    };
    let report =
        observations_from_tracking_results(&config, &[track_from_epoch(unlocked_epoch)], 10);
    let epoch = report.output.first().expect("observation epoch");
    let sat = epoch.sats.first().expect("observation satellite");

    assert_eq!(sat.observation_status, ObservationStatus::Missing);
    assert_eq!(
        sat.metadata.doppler_model,
        bijux_gnss_core::api::OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET
    );
    assert!((sat.doppler_hz.0 - expected_doppler_hz).abs() <= f64::EPSILON, "{sat:?}");
    assert!(sat.doppler_var_hz2.is_finite());
}

#[test]
fn observations_keep_doppler_on_sample_rate_mismatch_epoch() {
    let config = ReceiverPipelineConfig::default();
    let sat = SatId { constellation: Constellation::Gps, prn: 13 };
    let expected_doppler_hz = -80.0;
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(
        config.intermediate_freq_hz,
        expected_doppler_hz,
    );
    let mismatch_epoch = TrackEpoch {
        epoch: Epoch { index: 70 },
        sample_index: epoch_sample_index(&config, 70),
        source_time: ReceiverSampleTrace::from_sample_index(
            epoch_sample_index(&config, 70),
            config.sampling_freq_hz,
        ),
        sat,
        carrier_hz: Hertz(carrier_hz),
        code_rate_hz: Hertz(config.code_freq_basis_hz),
        lock: true,
        pll_lock: false,
        dll_lock: false,
        cn0_dbhz: 45.0,
        lock_state: "tracking".to_string(),
        lock_state_reason: Some("sample_rate_mismatch".to_string()),
        ..TrackEpoch::default()
    };
    let report =
        observations_from_tracking_results(&config, &[track_from_epoch(mismatch_epoch)], 10);
    let epoch = report.output.first().expect("observation epoch");
    let sat = epoch.sats.first().expect("observation satellite");

    assert_eq!(sat.observation_status, ObservationStatus::Inconsistent);
    assert_eq!(
        sat.metadata.doppler_model,
        bijux_gnss_core::api::OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET
    );
    assert!((sat.doppler_hz.0 - expected_doppler_hz).abs() <= f64::EPSILON, "{sat:?}");
    assert!(sat.doppler_var_hz2.is_finite());
}

#[test]
fn observations_reject_non_positive_pseudorange() {
    let config = ReceiverPipelineConfig::default();
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 0.0);
    let epoch = make_tracking_epoch_with_alignment(12, &config, 70, carrier_hz, 0.0, 0, -8.0);
    let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);

    let epoch = report.output.first().expect("observation epoch");
    let sat = epoch.sats.first().expect("observation satellite");

    assert_eq!(sat.observation_status, ObservationStatus::Inconsistent);
    assert!(sat
        .observation_reject_reasons
        .iter()
        .any(|reason| reason == "non_positive_pseudorange"));
    assert_eq!(epoch.decision, ObservationEpochDecision::Rejected);
    assert_eq!(epoch.decision_reason.as_deref(), Some("inconsistent_observable"));
    assert!(!epoch.valid);
}

#[test]
fn observations_reject_out_of_bounds_pseudorange() {
    let config = ReceiverPipelineConfig::default();
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 0.0);
    let epoch = make_tracking_epoch_with_alignment(13, &config, 70, carrier_hz, 0.0, 200_000, 0.0);
    let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);

    let epoch = report.output.first().expect("observation epoch");
    let sat = epoch.sats.first().expect("observation satellite");

    assert_eq!(sat.observation_status, ObservationStatus::Inconsistent);
    assert!(sat
        .observation_reject_reasons
        .iter()
        .any(|reason| reason == "pseudorange_out_of_bounds"));
    assert_eq!(epoch.decision, ObservationEpochDecision::Rejected);
    assert_eq!(epoch.decision_reason.as_deref(), Some("inconsistent_observable"));
    assert!(!epoch.valid);
}

#[test]
fn observations_reject_non_finite_pseudorange() {
    let config = ReceiverPipelineConfig::default();
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 0.0);
    let epoch = make_tracking_epoch_with_alignment(14, &config, 70, carrier_hz, 0.0, 68, f64::NAN);
    let (epochs, _) = observations_from_tracking(&config, &[epoch]);
    let epoch = epochs.first().expect("observation epoch");
    let sat = epoch.sats.first().expect("observation satellite");

    assert_eq!(sat.observation_status, ObservationStatus::Inconsistent);
    assert!(sat.observation_reject_reasons.iter().any(|reason| reason == "non_finite_pseudorange"));
    assert_eq!(epoch.decision, ObservationEpochDecision::Rejected);
    assert_eq!(epoch.decision_reason.as_deref(), Some("inconsistent_observable"));
    assert!(!epoch.valid);
}

#[test]
fn observation_residuals_report_raw_corrected_and_expected_pseudorange() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 125.0);
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
            make_tracking_epoch_with_alignment(21, &config, 71, carrier_hz, 10.125, 68, 0.0),
        ],
        transitions: Vec::new(),
    };

    let report = observation_residuals_from_tracking_results_with_gps_anchor(
        &config,
        Some(GpsTime { week: 2200, tow_s: 345_600.0 }),
        &[track],
        10,
    );
    let epoch = residual_epoch(&report, 71);
    let sat = epoch.sats.first().expect("residual satellite");
    let lambda_m = SPEED_OF_LIGHT_MPS / GPS_L1_CA_CARRIER_HZ.value();
    let expected_raw = aligned_pseudorange_m(&config, 68, 0.0);
    let expected_corrected = expected_raw + 0.5 * 0.125 * lambda_m;

    assert!((sat.pseudorange_m.raw - expected_raw).abs() <= 1.0e-9, "{sat:?}");
    assert!((sat.pseudorange_m.corrected - expected_corrected).abs() <= 1.0e-9, "{sat:?}");
    assert_eq!(sat.pseudorange_m.expected, Some(expected_raw));
    assert_eq!(
        sat.pseudorange_m.reference_model.as_deref(),
        Some("signal_travel_time_from_gps_anchor")
    );
    assert!(
        (sat.pseudorange_m.residual.expect("pseudorange residual")
            - (expected_corrected - expected_raw))
            .abs()
            <= 1.0e-9,
        "{sat:?}"
    );
    assert!(sat.pseudorange_m.sigma.expect("pseudorange sigma") > 0.0);
}

#[test]
fn observation_residuals_predict_carrier_phase_for_continuous_arcs() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        ..ReceiverPipelineConfig::default()
    };
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 125.0);
    let track = TrackingResult {
        sat: SatId { constellation: Constellation::Gps, prn: 22 },
        carrier_hz,
        code_phase_samples: 0.0,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: carrier_hz,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![
            make_tracking_epoch_with_alignment(22, &config, 70, carrier_hz, 10.0, 68, 0.0),
            make_tracking_epoch_with_alignment(22, &config, 71, carrier_hz, 10.125, 68, 0.0),
            make_tracking_epoch_with_alignment(22, &config, 72, carrier_hz, 10.250, 68, 0.0),
        ],
        transitions: Vec::new(),
    };

    let report = observation_residuals_from_tracking_results(&config, &[track], 10);
    let epoch = residual_epoch(&report, 72);
    let sat = epoch.sats.first().expect("residual satellite");

    assert_eq!(
        sat.carrier_phase_cycles.reference_model.as_deref(),
        Some("previous_continuous_phase_prediction")
    );
    assert_eq!(sat.carrier_phase_cycles.expected, Some(10.250));
    assert!(sat.carrier_phase_cycles.residual.expect("carrier residual").abs() <= 1.0e-12);
    assert_eq!(sat.carrier_phase_cycles.raw, 10.250);
    assert_eq!(sat.carrier_phase_cycles.corrected, 10.250);
}

#[test]
fn observation_residuals_surface_epoch_and_satellite_rejections() {
    let config = ReceiverPipelineConfig::default();
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 0.0);
    let epoch = make_tracking_epoch_with_alignment(23, &config, 70, carrier_hz, 0.0, 0, -8.0);
    let report =
        observation_residuals_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
    let epoch = residual_epoch(&report, 70);
    let sat = epoch.sats.first().expect("residual satellite");

    assert!(!epoch.accepted);
    assert_eq!(epoch.decision, ObservationEpochDecision::Rejected);
    assert_eq!(epoch.decision_reason.as_deref(), Some("inconsistent_observable"));
    assert!(!sat.accepted);
    assert_eq!(sat.observation_status, ObservationStatus::Inconsistent);
    assert!(sat
        .observation_reject_reasons
        .iter()
        .any(|reason| reason == "non_positive_pseudorange"));
}

#[test]
fn observation_decisions_surface_impossible_pseudorange_reason() {
    let config = ReceiverPipelineConfig::default();
    let carrier_hz = crate::pipeline::doppler::carrier_hz_from_doppler_hz(0.0, 0.0);
    let epoch = make_tracking_epoch_with_alignment(15, &config, 70, carrier_hz, 0.0, 0, -8.0);
    let report = observations_from_tracking_results(&config, &[track_from_epoch(epoch)], 10);
    let decisions = observation_decisions_from_epochs(&report.output);
    let decision = decisions.first().expect("observation decision");

    assert_eq!(decision.decision, ObservationEpochDecision::Rejected);
    assert!(decision.reasons.iter().any(|reason| reason == "non_positive_pseudorange"));
}

#[test]
fn code_carrier_divergence_decomposition_exempts_expected_ionosphere_from_multipath() {
    let mut state = CodeCarrierDivergenceState::default();
    let l1_delay_before_m = 3.0;
    let l1_delay_after_m = 9.0;
    let l1_jump_m = 2.0 * (l1_delay_after_m - l1_delay_before_m);
    let l2_delay_before_m = scaled_ionosphere_delay_m(
        l1_delay_before_m,
        signal_spec_gps_l1_ca(),
        signal_spec_gps_l2c(),
    );
    let l2_delay_after_m =
        scaled_ionosphere_delay_m(l1_delay_after_m, signal_spec_gps_l1_ca(), signal_spec_gps_l2c());
    let l2_jump_m = 2.0 * (l2_delay_after_m - l2_delay_before_m);

    let mut before = divergence_epoch(l1_delay_before_m, 106.0, 0.0, l2_delay_before_m, 0.0);
    assert_ne!(before.sats[0].signal_id.band, before.sats[1].signal_id.band);
    assert_eq!(before.sats[0].signal_id.sat, before.sats[1].signal_id.sat);
    assert!(before.sats[0].lock_flags.code_lock);
    assert!(before.sats[1].lock_flags.code_lock);
    assert!(before.sats[0].metadata.signal.carrier_hz.value() > 0.0);
    assert!(before.sats[1].metadata.signal.carrier_hz.value() > 0.0);
    assert_ne!(
        before.sats[0].metadata.signal.carrier_hz.value(),
        before.sats[1].metadata.signal.carrier_hz.value()
    );
    assert!(before.sats[0].pseudorange_m.0.is_finite());
    assert!(before.sats[1].pseudorange_m.0.is_finite());
    assert!(dual_frequency_code_delay_evidence(&before.sats[0], &before.sats[1]).is_some());
    assert_eq!(ionosphere_delay_evidence_from_epoch(&before).len(), 2);
    apply_code_carrier_divergence_decomposition(&mut before, &mut state);
    let mut after = divergence_epoch(
        l1_delay_after_m,
        106.0 + l1_jump_m,
        l1_jump_m,
        l2_delay_after_m,
        l2_jump_m,
    );
    apply_code_carrier_divergence_decomposition(&mut after, &mut state);

    let sat =
        after.sats.iter().find(|sat| sat.signal_id.band == SignalBand::L1).expect("L1 observation");
    let divergence =
        sat.metadata.code_carrier_divergence.expect("code-carrier divergence decomposition");

    assert!(!sat.multipath_suspect, "{sat:?}");
    assert!((divergence.expected_ionosphere_m - l1_jump_m).abs() < 1.0e-6);
    assert!(divergence.multipath_m.abs() < 1.0e-9);
    assert!(divergence.unexplained_m.abs() < 1.0e-6);
    assert_eq!(sat.error_model.as_ref().expect("error model").multipath_proxy_m, Meters(0.0));
}

#[test]
fn code_carrier_divergence_decomposition_preserves_ionosphere_sign() {
    let mut state = CodeCarrierDivergenceState::default();
    let l1_delay_before_m = 9.0;
    let l1_delay_after_m = 3.0;
    let l1_jump_m = 2.0 * (l1_delay_after_m - l1_delay_before_m);
    let l2_delay_before_m = scaled_ionosphere_delay_m(
        l1_delay_before_m,
        signal_spec_gps_l1_ca(),
        signal_spec_gps_l2c(),
    );
    let l2_delay_after_m =
        scaled_ionosphere_delay_m(l1_delay_after_m, signal_spec_gps_l1_ca(), signal_spec_gps_l2c());
    let l2_jump_m = 2.0 * (l2_delay_after_m - l2_delay_before_m);

    let mut before = divergence_epoch(l1_delay_before_m, 118.0, 0.0, l2_delay_before_m, 0.0);
    apply_code_carrier_divergence_decomposition(&mut before, &mut state);
    let mut after = divergence_epoch(
        l1_delay_after_m,
        118.0 + l1_jump_m,
        l1_jump_m,
        l2_delay_after_m,
        l2_jump_m,
    );
    apply_code_carrier_divergence_decomposition(&mut after, &mut state);

    let sat =
        after.sats.iter().find(|sat| sat.signal_id.band == SignalBand::L1).expect("L1 observation");
    let divergence =
        sat.metadata.code_carrier_divergence.expect("code-carrier divergence decomposition");

    assert!(!sat.multipath_suspect, "{sat:?}");
    assert!((divergence.jump_m - l1_jump_m).abs() < 1.0e-6);
    assert!((divergence.expected_ionosphere_m - l1_jump_m).abs() < 1.0e-6);
    assert!(divergence.multipath_m.abs() < 1.0e-9);
    assert!(divergence.unexplained_m.abs() < 1.0e-6);
}

#[test]
fn code_carrier_divergence_decomposition_keeps_residual_multipath_separate() {
    let mut state = CodeCarrierDivergenceState::default();
    let l1_delay_before_m = 3.0;
    let l1_delay_after_m = 9.0;
    let residual_multipath_m = 8.0;
    let l1_ionosphere_jump_m = 2.0 * (l1_delay_after_m - l1_delay_before_m);
    let l1_jump_m = l1_ionosphere_jump_m + residual_multipath_m;
    let l2_delay_before_m = scaled_ionosphere_delay_m(
        l1_delay_before_m,
        signal_spec_gps_l1_ca(),
        signal_spec_gps_l2c(),
    );
    let l2_delay_after_m =
        scaled_ionosphere_delay_m(l1_delay_after_m, signal_spec_gps_l1_ca(), signal_spec_gps_l2c());
    let l2_jump_m = 2.0 * (l2_delay_after_m - l2_delay_before_m);

    let mut before = divergence_epoch(l1_delay_before_m, 106.0, 0.0, l2_delay_before_m, 0.0);
    assert_eq!(ionosphere_delay_evidence_from_epoch(&before).len(), 2);
    apply_code_carrier_divergence_decomposition(&mut before, &mut state);
    let mut after = divergence_epoch(
        l1_delay_after_m,
        106.0 + l1_jump_m,
        l1_jump_m,
        l2_delay_after_m,
        l2_jump_m,
    );
    apply_code_carrier_divergence_decomposition(&mut after, &mut state);

    let sat =
        after.sats.iter().find(|sat| sat.signal_id.band == SignalBand::L1).expect("L1 observation");
    let divergence =
        sat.metadata.code_carrier_divergence.expect("code-carrier divergence decomposition");

    assert!(sat.multipath_suspect, "{sat:?}");
    assert!((divergence.expected_ionosphere_m - l1_ionosphere_jump_m).abs() < 1.0e-6);
    assert!((divergence.multipath_m - residual_multipath_m).abs() < 1.0e-6);
    assert!(divergence.unexplained_m.abs() < 1.0e-6);
    assert!(
        (sat.error_model.as_ref().expect("error model").multipath_proxy_m.0 - residual_multipath_m)
            .abs()
            < 1.0e-6
    );
}

#[test]
fn dual_frequency_cycle_slip_fusion_records_geometry_free_detector() {
    let l1_delay_m = 4.0;
    let l2_delay_m =
        scaled_ionosphere_delay_m(l1_delay_m, signal_spec_gps_l1_ca(), signal_spec_gps_l2c());
    let previous = divergence_epoch(l1_delay_m, 100.0, 0.0, l2_delay_m, 0.0);
    let mut current = divergence_epoch(l1_delay_m, 100.0, 0.0, l2_delay_m, 0.0);
    current.epoch_idx = 71;
    let signal = current.sats[0].metadata.signal;
    current.sats[0].carrier_phase_cycles = Cycles(
        current.sats[0].carrier_phase_cycles.0 + signal_meters_to_cycles(Meters(0.25), signal).0,
    );
    current.sats[0].pseudorange_m = Meters(current.sats[0].pseudorange_m.0 + 10.0);
    let raw_pseudorange_m = current.sats[0].pseudorange_m.0 - 10.0;
    let mut raw_snapshots = std::collections::HashMap::new();
    raw_snapshots.insert(
        observation_snapshot_key(current.epoch_idx, current.sats[0].signal_id),
        raw_observation_snapshot(&current.sats[0], raw_pseudorange_m),
    );
    let mut hatch = std::collections::HashMap::new();

    apply_dual_frequency_cycle_slip_fusion(
        &mut current,
        Some(&previous),
        &raw_snapshots,
        &mut hatch,
    );

    let sat = current
        .sats
        .iter()
        .find(|sat| sat.signal_id.band == SignalBand::L1)
        .expect("L1 observation");
    let evidence = sat.metadata.cycle_slip_evidence.as_ref().expect("cycle-slip evidence");
    assert!(sat.lock_flags.cycle_slip, "{sat:?}");
    assert!(evidence.triggered_detectors().contains(&CycleSlipDetector::GeometryFreePhase));
    assert_eq!(sat.metadata.observation_lock_reason.as_deref(), Some("geometry_free_phase"));
    assert!((sat.pseudorange_m.0 - raw_pseudorange_m).abs() < 1.0e-9);
    assert_eq!(sat.metadata.smoothing_age, 1);
}

#[test]
fn dual_frequency_cycle_slip_fusion_records_melbourne_wubbena_detector() {
    let l1_delay_m = 4.0;
    let l2_delay_m =
        scaled_ionosphere_delay_m(l1_delay_m, signal_spec_gps_l1_ca(), signal_spec_gps_l2c());
    let previous = divergence_epoch(l1_delay_m, 100.0, 0.0, l2_delay_m, 0.0);
    let mut current = divergence_epoch(l1_delay_m, 100.0, 0.0, l2_delay_m, 0.0);
    current.epoch_idx = 72;
    current.sats[0].pseudorange_m = Meters(current.sats[0].pseudorange_m.0 - 2.0);
    let mut raw_snapshots = std::collections::HashMap::new();
    raw_snapshots.insert(
        observation_snapshot_key(current.epoch_idx, current.sats[0].signal_id),
        raw_observation_snapshot(&current.sats[0], current.sats[0].pseudorange_m.0),
    );
    let mut hatch = std::collections::HashMap::new();

    apply_dual_frequency_cycle_slip_fusion(
        &mut current,
        Some(&previous),
        &raw_snapshots,
        &mut hatch,
    );

    let sat = current
        .sats
        .iter()
        .find(|sat| sat.signal_id.band == SignalBand::L1)
        .expect("L1 observation");
    let evidence = sat.metadata.cycle_slip_evidence.as_ref().expect("cycle-slip evidence");
    assert!(sat.lock_flags.cycle_slip, "{sat:?}");
    assert!(evidence.triggered_detectors().contains(&CycleSlipDetector::MelbourneWubbena));
    assert_eq!(sat.metadata.observation_lock_reason.as_deref(), Some("melbourne_wubbena"));
}

fn scaled_ionosphere_delay_m(
    reference_delay_m: f64,
    reference_signal: SignalSpec,
    target_signal: SignalSpec,
) -> f64 {
    reference_delay_m * reference_signal.carrier_hz.value().powi(2)
        / target_signal.carrier_hz.value().powi(2)
}

fn divergence_epoch(
    l1_ionosphere_delay_m: f64,
    l1_raw_divergence_m: f64,
    l1_divergence_jump_m: f64,
    l2_ionosphere_delay_m: f64,
    l2_divergence_jump_m: f64,
) -> ObsEpoch {
    let base_range_m = 20_200_000.0;
    ObsEpoch {
        t_rx_s: Seconds(70.0),
        source_time: ReceiverSampleTrace::from_sample_index(286_440, 4_092_000.0),
        gps_week: None,
        tow_s: None,
        epoch_idx: 70,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        role: ReceiverRole::Rover,
        sats: vec![
            divergence_satellite(
                signal_spec_gps_l1_ca(),
                l1_ionosphere_delay_m,
                l1_raw_divergence_m,
                l1_divergence_jump_m,
                base_range_m,
            ),
            divergence_satellite(
                signal_spec_gps_l2c(),
                l2_ionosphere_delay_m,
                100.0 + l2_divergence_jump_m,
                l2_divergence_jump_m,
                base_range_m,
            ),
        ],
        decision: ObservationEpochDecision::Accepted,
        decision_reason: None,
        manifest: None,
    }
}

fn divergence_satellite(
    signal: SignalSpec,
    ionosphere_delay_m: f64,
    raw_divergence_m: f64,
    divergence_jump_m: f64,
    base_range_m: f64,
) -> ObsSatellite {
    let pseudorange_m = base_range_m + ionosphere_delay_m;
    let carrier_phase_m = pseudorange_m - raw_divergence_m;
    ObsSatellite {
        signal_id: SigId {
            sat: SatId { constellation: Constellation::Gps, prn: 9 },
            band: signal.band,
            code: signal.code,
        },
        pseudorange_m: Meters(pseudorange_m),
        pseudorange_var_m2: 1.0,
        carrier_phase_cycles: signal_meters_to_cycles(Meters(carrier_phase_m), signal),
        carrier_phase_var_cycles2: 1.0,
        doppler_hz: Hertz(0.0),
        doppler_var_hz2: 1.0,
        cn0_dbhz: 45.0,
        lock_flags: LockFlags {
            code_lock: true,
            carrier_lock: true,
            bit_lock: true,
            cycle_slip: false,
        },
        multipath_suspect: false,
        observation_status: ObservationStatus::Accepted,
        observation_reject_reasons: Vec::new(),
        elevation_deg: Some(45.0),
        azimuth_deg: Some(0.0),
        weight: None,
        timing: None,
        error_model: Some(bijux_gnss_core::api::MeasurementErrorModel {
            thermal_noise_m: Meters(0.0),
            tracking_jitter_m: Meters(1.0),
            multipath_proxy_m: Meters(0.0),
            clock_error_m: Meters(0.0),
        }),
        metadata: ObsMetadata {
            signal,
            integration_ms: 20,
            code_carrier_divergence: Some(CodeCarrierDivergence::from_terms(
                raw_divergence_m,
                divergence_jump_m,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )),
            ..ObsMetadata::default()
        },
    }
}
