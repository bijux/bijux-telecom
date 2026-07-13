#![allow(missing_docs)]

use std::f64::consts::TAU;

use bijux_gnss_core::api::{
    Chips, Cycles, Epoch, Hertz, ReceiverSampleTrace, SignalBand, SignalDelayAlignment, SignalSpec,
    TrackEpoch, TrackingUncertainty,
};
use bijux_gnss_nav::api::GpsEphemeris;
use bijux_gnss_receiver::api::{
    carrier_hz_from_doppler_hz,
    sim::{
        SyntheticIonosphereDelayModel, SyntheticObservationTruthReference, SyntheticSignalParams,
    },
    ReceiverPipelineConfig, TrackingResult,
};
use bijux_gnss_signal::api::{signal_spec_gps_l1_ca, signal_spec_gps_l2c, signal_spec_gps_l5};

use crate::support::navigation_truth::{
    four_satellite_pvt_scenario, synthetic_pseudorange_m, SyntheticPvtScenario,
};

pub const SYNTHETIC_REFERENCE_RECEIVE_TIME_S: f64 = 100_000.0;
pub const TRACKING_CARRIER_PHASE_SIGMA_CYCLES: f64 = 0.05;
pub const TRACKING_DOPPLER_SIGMA_HZ: f64 = 0.25;
pub const TRACKING_CN0_SIGMA_DBHZ: f64 = 0.75;
const GPS_L1_REFERENCE_IONOSPHERE_DELAY_M: f64 = 5.0;

#[derive(Debug, Clone)]
pub struct ObservationTruthFixture {
    pub config: ReceiverPipelineConfig,
    pub profile: SyntheticPvtScenario,
    pub reference: SyntheticObservationTruthReference,
    pub tracks: Vec<TrackingResult>,
}

#[derive(Debug, Clone)]
struct ObservationTruthTrackSeed {
    signal: SyntheticSignalParams,
    signal_spec: SignalSpec,
    whole_code_periods: u64,
}

#[allow(dead_code)]
pub fn build_observation_truth_fixture(
    config: ReceiverPipelineConfig,
    epoch_count: usize,
) -> ObservationTruthFixture {
    build_observation_truth_fixture_with_cycle_slips(config, epoch_count, &[])
}

#[allow(dead_code)]
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

    let track_seeds = profile
        .scenario
        .satellites
        .iter()
        .map(|signal| ObservationTruthTrackSeed {
            signal: signal.clone(),
            signal_spec: signal_spec_gps_l1_ca(),
            whole_code_periods: profile.pseudorange_epoch_base,
        })
        .collect::<Vec<_>>();
    let tracks = track_seeds
        .iter()
        .map(|seed| {
            synthetic_truth_track(
                &config,
                &seed.signal,
                seed.signal_spec,
                seed.whole_code_periods,
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
            ionosphere_delay_model: None,
        },
        config,
        profile,
        tracks,
    }
}

#[allow(dead_code)]
pub fn build_mixed_band_observation_truth_fixture(
    config: ReceiverPipelineConfig,
    epoch_count: usize,
) -> ObservationTruthFixture {
    build_observation_truth_fixture_with_parallel_signal(config, epoch_count, signal_spec_gps_l2c())
}

#[allow(dead_code)]
pub fn build_l5_observation_truth_fixture(
    config: ReceiverPipelineConfig,
    epoch_count: usize,
) -> ObservationTruthFixture {
    build_observation_truth_fixture_with_parallel_signal(config, epoch_count, signal_spec_gps_l5())
}

fn build_observation_truth_fixture_with_parallel_signal(
    config: ReceiverPipelineConfig,
    epoch_count: usize,
    parallel_signal: SignalSpec,
) -> ObservationTruthFixture {
    let mut profile = four_satellite_pvt_scenario(&config);
    for (signal, doppler_hz) in
        profile.scenario.satellites.iter_mut().zip([-250.0, -125.0, 0.0, 125.0, 250.0])
    {
        signal.doppler_hz = doppler_hz;
    }
    let ionosphere_delay_model = SyntheticIonosphereDelayModel {
        reference_delay_m: GPS_L1_REFERENCE_IONOSPHERE_DELAY_M,
        reference_signal: signal_spec_gps_l1_ca(),
    };

    let mut track_seeds = profile
        .scenario
        .satellites
        .iter()
        .map(|signal| {
            let ephemeris = profile
                .ephemerides
                .iter()
                .find(|candidate| candidate.sat == signal.sat)
                .expect("fixture ephemeris for L1 observation signal");
            dispersive_track_seed(
                signal.clone(),
                signal_spec_gps_l1_ca(),
                ephemeris,
                profile.truth_ecef_m,
                ionosphere_delay_model,
            )
        })
        .collect::<Vec<_>>();

    let l1_reference = track_seeds[0].clone();
    let ephemeris = profile
        .ephemerides
        .iter()
        .find(|candidate| candidate.sat == l1_reference.signal.sat)
        .expect("fixture ephemeris for duplicated observation signal");
    track_seeds.insert(
        1,
        dispersive_track_seed(
            l1_reference.signal.clone(),
            parallel_signal,
            ephemeris,
            profile.truth_ecef_m,
            ionosphere_delay_model,
        ),
    );
    profile.scenario.satellites = track_seeds.iter().map(|seed| seed.signal.clone()).collect();

    let tracks = track_seeds
        .iter()
        .map(|seed| {
            synthetic_truth_track(
                &config,
                &seed.signal,
                seed.signal_spec,
                seed.whole_code_periods,
                epoch_count,
                &[],
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
            ionosphere_delay_model: Some(ionosphere_delay_model),
        },
        config,
        profile,
        tracks,
    }
}

fn dispersive_track_seed(
    mut signal: SyntheticSignalParams,
    signal_spec: SignalSpec,
    ephemeris: &GpsEphemeris,
    truth_ecef_m: (f64, f64, f64),
    ionosphere_delay_model: SyntheticIonosphereDelayModel,
) -> ObservationTruthTrackSeed {
    let geometric_pseudorange_m =
        synthetic_pseudorange_m(ephemeris, SYNTHETIC_REFERENCE_RECEIVE_TIME_S, truth_ecef_m);
    let pseudorange_m = ionosphere_delay_model
        .pseudorange_m(geometric_pseudorange_m, signal_spec)
        .expect("finite synthetic ionosphere pseudorange");
    let (whole_code_periods, code_phase_chips) =
        signal_code_alignment_from_pseudorange_m(pseudorange_m, signal_spec);
    signal.signal_band = signal_spec.band;
    signal.code_phase_chips = code_phase_chips;

    ObservationTruthTrackSeed { signal, signal_spec, whole_code_periods }
}

fn synthetic_truth_track(
    config: &ReceiverPipelineConfig,
    signal: &SyntheticSignalParams,
    signal_spec: SignalSpec,
    whole_code_periods: u64,
    epoch_count: usize,
    cycle_slip_epoch_indices: &[usize],
    carrier_bias_cycles: f64,
) -> TrackingResult {
    let code_length = signal_code_length(signal_spec);
    let code_phase_samples =
        tracking_code_phase_samples(config, signal_spec, code_length, signal.code_phase_chips);
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
                signal_band: signal_spec.band,
                signal_code: signal_spec.code,
                glonass_frequency_channel: None,
                prompt_i: 1.0,
                prompt_q: 0.0,
                early_i: 0.0,
                early_q: 0.0,
                late_i: 0.0,
                late_q: 0.0,
                carrier_hz: Hertz(tracked_carrier_hz(config, signal_spec, signal.doppler_hz)),
                carrier_phase_cycles: Cycles(carrier_phase_cycles),
                code_rate_hz: Hertz(signal_spec.code_rate_hz),
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
                channel_uid: format!(
                    "Gps-{:02}-{}-observation-truth",
                    signal.sat.prn,
                    signal_label(signal_spec)
                ),
                tracking_provenance: "integration_observations_truth_table".to_string(),
                tracking_assumptions: None,
                signal_delay_alignment: Some(SignalDelayAlignment {
                    whole_code_periods,
                    sample_delay_samples: 0,
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

fn tracking_code_phase_samples(
    config: &ReceiverPipelineConfig,
    signal_spec: SignalSpec,
    code_length: usize,
    aligned_code_phase_chips: f64,
) -> f64 {
    if !aligned_code_phase_chips.is_finite() || aligned_code_phase_chips < 0.0 {
        return aligned_code_phase_chips;
    }
    let samples_per_chip = config.sampling_freq_hz / signal_spec.code_rate_hz;
    let period_samples = samples_per_chip * code_length as f64;
    let aligned_code_phase_samples = aligned_code_phase_chips * samples_per_chip;
    (period_samples - aligned_code_phase_samples).rem_euclid(period_samples)
}

#[allow(dead_code)]
fn signal_code_alignment_from_pseudorange_m(
    pseudorange_m: f64,
    signal_spec: SignalSpec,
) -> (u64, f64) {
    let code_length = signal_code_length(signal_spec) as f64;
    let pseudorange_chips = pseudorange_m * signal_spec.code_rate_hz / 299_792_458.0;
    let whole_code_periods = (pseudorange_chips / code_length).floor() as u64;
    let code_phase_chips = pseudorange_chips - whole_code_periods as f64 * code_length;
    (whole_code_periods, code_phase_chips.rem_euclid(code_length))
}

fn signal_code_length(signal_spec: SignalSpec) -> usize {
    match signal_spec.band {
        SignalBand::L1 => 1023,
        SignalBand::L2 => 10230,
        SignalBand::L5 => 10230,
        _ => panic!("unsupported observation truth signal {:?}", signal_spec),
    }
}

fn signal_label(signal_spec: SignalSpec) -> &'static str {
    match signal_spec.band {
        SignalBand::L1 => "l1",
        SignalBand::L2 => "l2c",
        SignalBand::L5 => "l5",
        _ => "signal",
    }
}

fn tracked_carrier_hz(
    config: &ReceiverPipelineConfig,
    signal_spec: SignalSpec,
    doppler_hz: f64,
) -> f64 {
    carrier_hz_from_doppler_hz(
        config.intermediate_freq_hz
            + (signal_spec.carrier_hz.value() - bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ.value()),
        doppler_hz,
    )
}
