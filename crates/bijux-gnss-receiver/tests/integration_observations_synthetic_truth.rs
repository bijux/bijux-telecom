#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{
    Chips, Cycles, Epoch, Hertz, ReceiverSampleTrace, SignalDelayAlignment, TrackEpoch,
};
use bijux_gnss_receiver::api::{
    sim::{validate_truth_guided_observations, SyntheticObservationTruthReference},
    ReceiverPipelineConfig, TrackingResult,
};

use support::navigation_truth::four_satellite_pvt_scenario;

const SYNTHETIC_REFERENCE_RECEIVE_TIME_S: f64 = 100_000.0;
const MAX_ABS_ERROR_TOLERANCE: f64 = 1.0e-6;

#[test]
fn observation_truth_validation_reports_all_observable_errors_per_satellite() {
    let config = ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 5,
        ..ReceiverPipelineConfig::default()
    };
    let profile = four_satellite_pvt_scenario(&config);
    let tracks = profile
        .scenario
        .satellites
        .iter()
        .map(|signal| synthetic_truth_track(&config, signal, profile.pseudorange_epoch_base))
        .collect::<Vec<_>>();
    let report = validate_truth_guided_observations(
        &config,
        &tracks,
        &profile.scenario,
        &SyntheticObservationTruthReference {
            receive_time_s: SYNTHETIC_REFERENCE_RECEIVE_TIME_S,
            receiver_ecef_m: [
                profile.truth_ecef_m.0,
                profile.truth_ecef_m.1,
                profile.truth_ecef_m.2,
            ],
        },
        10,
    );

    assert_eq!(report.scenario_id, profile.scenario.id);
    assert_eq!(report.satellites.len(), profile.scenario.satellites.len());

    for satellite in &report.satellites {
        assert!(satellite.notes.is_empty(), "{satellite:?}");

        let pseudorange = satellite
            .pseudorange_error_m
            .as_ref()
            .unwrap_or_else(|| panic!("missing pseudorange stats for {:?}", satellite.sat));
        assert_eq!(pseudorange.count, 1, "{satellite:?}");
        assert!(pseudorange.max_abs_error <= MAX_ABS_ERROR_TOLERANCE, "{satellite:?}");

        let carrier_phase = satellite
            .carrier_phase_error_cycles
            .as_ref()
            .unwrap_or_else(|| panic!("missing carrier-phase stats for {:?}", satellite.sat));
        assert_eq!(carrier_phase.count, 1, "{satellite:?}");
        assert_eq!(satellite.carrier_phase_arcs_evaluated, 1, "{satellite:?}");
        assert!(carrier_phase.max_abs_error <= MAX_ABS_ERROR_TOLERANCE, "{satellite:?}");

        let doppler = satellite
            .doppler_error_hz
            .as_ref()
            .unwrap_or_else(|| panic!("missing Doppler stats for {:?}", satellite.sat));
        assert_eq!(doppler.count, 1, "{satellite:?}");
        assert!(doppler.max_abs_error <= MAX_ABS_ERROR_TOLERANCE, "{satellite:?}");

        let cn0 = satellite
            .cn0_error_db_hz
            .as_ref()
            .unwrap_or_else(|| panic!("missing C/N0 stats for {:?}", satellite.sat));
        assert_eq!(cn0.count, 1, "{satellite:?}");
        assert!(cn0.max_abs_error <= MAX_ABS_ERROR_TOLERANCE, "{satellite:?}");
    }
}

fn synthetic_truth_track(
    config: &ReceiverPipelineConfig,
    signal: &bijux_gnss_receiver::api::sim::SyntheticSignalParams,
    whole_code_periods: u64,
) -> TrackingResult {
    let samples_per_chip = config.sampling_freq_hz / config.code_freq_basis_hz;
    let code_phase_samples = signal.code_phase_chips * samples_per_chip;
    let carrier_phase_cycles = signal.carrier_phase_rad / std::f64::consts::TAU;
    let epoch = TrackEpoch {
        epoch: Epoch { index: 0 },
        sample_index: 0,
        source_time: ReceiverSampleTrace::from_sample_index(0, config.sampling_freq_hz),
        sat: signal.sat,
        prompt_i: 1.0,
        prompt_q: 0.0,
        early_i: 0.0,
        early_q: 0.0,
        late_i: 0.0,
        late_q: 0.0,
        carrier_hz: Hertz(signal.doppler_hz),
        carrier_phase_cycles: Cycles(carrier_phase_cycles),
        code_rate_hz: Hertz(config.code_freq_basis_hz),
        code_phase_samples: Chips(code_phase_samples),
        lock: true,
        cn0_dbhz: signal.cn0_db_hz as f64,
        pll_lock: true,
        dll_lock: true,
        fll_lock: true,
        cycle_slip: false,
        nav_bit_lock: false,
        navigation_bit_sign: None,
        dll_err: 0.0,
        pll_err: 0.0,
        fll_err: 0.0,
        anti_false_lock: false,
        cycle_slip_reason: None,
        lock_state: "tracking".to_string(),
        lock_state_reason: Some("stable_tracking".to_string()),
        channel_id: Some(signal.sat.prn),
        channel_uid: format!("Gps-{:02}-truth", signal.sat.prn),
        tracking_provenance: "integration_observations_synthetic_truth".to_string(),
        tracking_assumptions: None,
        signal_delay_alignment: Some(SignalDelayAlignment {
            whole_code_periods,
            source: "synthetic_truth".to_string(),
        }),
        tracking_uncertainty: None,
        processing_ms: None,
    };

    TrackingResult {
        sat: signal.sat,
        carrier_hz: signal.doppler_hz,
        code_phase_samples,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: code_phase_samples.round() as usize,
        acquisition_carrier_hz: signal.doppler_hz,
        acq_to_track_state: "accepted".to_string(),
        epochs: vec![epoch],
        transitions: Vec::new(),
    }
}
