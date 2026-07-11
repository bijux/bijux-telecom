#![allow(missing_docs)]

use std::f64::consts::TAU;

use bijux_gnss_core::api::{
    Chips, Cycles, Epoch, Hertz, ReceiverSampleTrace, SignalDelayAlignment, TrackEpoch,
    TrackingUncertainty,
};
use bijux_gnss_receiver::api::{
    carrier_hz_from_doppler_hz, sim::SyntheticObservationTruthReference, ReceiverPipelineConfig,
    TrackingResult,
};

use crate::support::navigation_truth::{four_satellite_pvt_scenario, SyntheticPvtScenario};

pub const SYNTHETIC_REFERENCE_RECEIVE_TIME_S: f64 = 100_000.0;
pub const TRACKING_CARRIER_PHASE_SIGMA_CYCLES: f64 = 0.05;
pub const TRACKING_DOPPLER_SIGMA_HZ: f64 = 0.25;
pub const TRACKING_CN0_SIGMA_DBHZ: f64 = 0.75;

#[derive(Debug, Clone)]
pub struct ObservationTruthFixture {
    pub config: ReceiverPipelineConfig,
    pub profile: SyntheticPvtScenario,
    pub reference: SyntheticObservationTruthReference,
    pub tracks: Vec<TrackingResult>,
}

pub fn build_observation_truth_fixture(
    config: ReceiverPipelineConfig,
    epoch_count: usize,
) -> ObservationTruthFixture {
    build_observation_truth_fixture_with_cycle_slips(config, epoch_count, &[])
}

pub fn build_observation_truth_fixture_with_cycle_slips(
    config: ReceiverPipelineConfig,
    epoch_count: usize,
    cycle_slip_epoch_indices: &[usize],
) -> ObservationTruthFixture {
    let mut profile = four_satellite_pvt_scenario(&config);
    for (signal, doppler_hz) in
        profile.scenario.satellites.iter_mut().zip([-250.0, -125.0, 0.0, 125.0, 250.0])
    {
        signal.doppler_hz = doppler_hz;
    }
    let tracks = profile
        .scenario
        .satellites
        .iter()
        .map(|signal| {
            synthetic_truth_track(
                &config,
                signal,
                profile.pseudorange_epoch_base,
                epoch_count,
                cycle_slip_epoch_indices,
                0.0,
            )
        })
        .collect::<Vec<_>>();

    ObservationTruthFixture {
        reference: SyntheticObservationTruthReference {
            receive_time_s: SYNTHETIC_REFERENCE_RECEIVE_TIME_S,
            receiver_ecef_m: [
                profile.truth_ecef_m.0,
                profile.truth_ecef_m.1,
                profile.truth_ecef_m.2,
            ],
        },
        config,
        profile,
        tracks,
    }
}

fn synthetic_truth_track(
    config: &ReceiverPipelineConfig,
    signal: &bijux_gnss_receiver::api::sim::SyntheticSignalParams,
    whole_code_periods: u64,
    epoch_count: usize,
    cycle_slip_epoch_indices: &[usize],
    carrier_bias_cycles: f64,
) -> TrackingResult {
    let code_phase_samples = tracking_code_phase_samples(config, signal.code_phase_chips);
    let sample_step = (config.sampling_freq_hz * 0.001).round() as u64;
    let base_carrier_phase_cycles = signal.carrier_phase_rad / TAU;

    let mut slip_bias_cycles = 0.0;
    let epochs = (0..epoch_count)
        .map(|epoch_index| {
            if cycle_slip_epoch_indices.contains(&epoch_index) {
                slip_bias_cycles += 12.0;
            }
            let sample_index = epoch_index as u64 * sample_step;
            let elapsed_s = sample_index as f64 / config.sampling_freq_hz;
            let carrier_phase_cycles = base_carrier_phase_cycles
                + carrier_bias_cycles
                + slip_bias_cycles
                + signal.doppler_hz * elapsed_s;
            TrackEpoch {
                epoch: Epoch { index: epoch_index as u64 },
                sample_index,
                source_time: ReceiverSampleTrace::from_sample_index(
                    sample_index,
                    config.sampling_freq_hz,
                ),
                sat: signal.sat,
                prompt_i: 1.0,
                prompt_q: 0.0,
                early_i: 0.0,
                early_q: 0.0,
                late_i: 0.0,
                late_q: 0.0,
                carrier_hz: Hertz(carrier_hz_from_doppler_hz(
                    config.intermediate_freq_hz,
                    signal.doppler_hz,
                )),
                carrier_phase_cycles: Cycles(carrier_phase_cycles),
                code_rate_hz: Hertz(config.code_freq_basis_hz),
                code_phase_samples: Chips(code_phase_samples),
                lock: true,
                cn0_dbhz: signal.cn0_db_hz as f64,
                pll_lock: true,
                dll_lock: true,
                fll_lock: true,
                cycle_slip: cycle_slip_epoch_indices.contains(&epoch_index),
                nav_bit_lock: false,
                navigation_bit_sign: None,
                dll_err: 0.0,
                pll_err: 0.0,
                fll_err: 0.0,
                anti_false_lock: false,
                cycle_slip_reason: cycle_slip_epoch_indices
                    .contains(&epoch_index)
                    .then_some("synthetic_phase_jump".to_string()),
                lock_state: "tracking".to_string(),
                lock_state_reason: Some("stable_tracking".to_string()),
                channel_id: Some(signal.sat.prn),
                channel_uid: format!("Gps-{:02}-observation-truth", signal.sat.prn),
                tracking_provenance: "integration_observations_truth_table".to_string(),
                tracking_assumptions: None,
                signal_delay_alignment: Some(SignalDelayAlignment {
                    whole_code_periods,
                    source: "synthetic_truth".to_string(),
                }),
                tracking_uncertainty: Some(TrackingUncertainty {
                    code_phase_samples: 0.25,
                    carrier_phase_cycles: TRACKING_CARRIER_PHASE_SIGMA_CYCLES,
                    doppler_hz: TRACKING_DOPPLER_SIGMA_HZ,
                    cn0_dbhz: TRACKING_CN0_SIGMA_DBHZ,
                }),
                processing_ms: None,
            }
        })
        .collect::<Vec<_>>();

    TrackingResult {
        sat: signal.sat,
        carrier_hz: signal.doppler_hz,
        code_phase_samples,
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: code_phase_samples.round() as usize,
        acquisition_carrier_hz: signal.doppler_hz,
        acq_to_track_state: "accepted".to_string(),
        epochs,
        transitions: Vec::new(),
    }
}

fn tracking_code_phase_samples(config: &ReceiverPipelineConfig, aligned_code_phase_chips: f64) -> f64 {
    let samples_per_chip = config.sampling_freq_hz / config.code_freq_basis_hz;
    let period_samples = samples_per_chip * config.code_length as f64;
    let aligned_code_phase_samples = aligned_code_phase_chips * samples_per_chip;
    if !aligned_code_phase_samples.is_finite() || aligned_code_phase_samples < 0.0 {
        return aligned_code_phase_samples;
    }
    (period_samples - aligned_code_phase_samples).rem_euclid(period_samples)
}
