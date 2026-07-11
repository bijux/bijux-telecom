#![allow(missing_docs)]

use bijux_gnss_core::api::{
    Chips, Constellation, Cycles, Epoch, Hertz, ObsEpoch, ReceiverSampleTrace, SatId, TrackEpoch,
    OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET,
};
use bijux_gnss_receiver::api::{
    carrier_hz_from_doppler_hz, observations_from_tracking_results, ReceiverPipelineConfig,
    TrackingResult,
};

const CLEAN_TRACKED_DOPPLER_CN0_DB_HZ: f32 = 75.0;
const CLEAN_TRACKED_DOPPLER_MAX_ERROR_HZ: f64 = 10.0;
const OBSERVATION_START_EPOCH_INDEX: u64 = 70;
const OBSERVATION_WINDOW_EPOCHS: u64 = 12;

fn observation_tracking_config(intermediate_freq_hz: f64) -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 18.0,
        fll_bw_hz: 12.0,
        ..ReceiverPipelineConfig::default()
    }
}

fn observation_epoch_samples(config: &ReceiverPipelineConfig) -> u64 {
    ((config.sampling_freq_hz * config.code_length as f64) / config.code_freq_basis_hz).round()
        as u64
}

fn locked_tracking_epoch(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    epoch_idx: u64,
    sample_index: u64,
    doppler_hz: f64,
    carrier_phase_cycles: f64,
) -> TrackEpoch {
    TrackEpoch {
        epoch: Epoch { index: epoch_idx },
        sample_index,
        source_time: ReceiverSampleTrace::from_sample_index(sample_index, config.sampling_freq_hz),
        sat,
        prompt_i: 1.0,
        prompt_q: 0.0,
        early_i: 0.0,
        early_q: 0.0,
        late_i: 0.0,
        late_q: 0.0,
        carrier_hz: Hertz(carrier_hz_from_doppler_hz(config.intermediate_freq_hz, doppler_hz)),
        carrier_phase_cycles: Cycles(carrier_phase_cycles),
        code_rate_hz: Hertz(config.code_freq_basis_hz),
        code_phase_samples: Chips(0.0),
        lock: true,
        cn0_dbhz: CLEAN_TRACKED_DOPPLER_CN0_DB_HZ as f64,
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
        lock_state_reason: None,
        channel_id: Some(0),
        channel_uid: format!("Gps-{:02}-ch00", sat.prn),
        tracking_provenance: "observation_tracking_fixture".to_string(),
        tracking_assumptions: None,
        signal_delay_alignment: None,
        tracking_uncertainty: None,
        processing_ms: None,
    }
}

fn observation_track_with_doppler_model(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    doppler_at_epoch: impl Fn(u64, u64) -> f64,
) -> TrackingResult {
    let samples_per_epoch = observation_epoch_samples(config);
    let mut epochs = Vec::with_capacity(OBSERVATION_WINDOW_EPOCHS as usize);
    let mut carrier_phase_cycles = 0.25;
    for offset in 0..OBSERVATION_WINDOW_EPOCHS {
        let epoch_idx = OBSERVATION_START_EPOCH_INDEX + offset;
        let sample_index = epoch_idx * samples_per_epoch;
        let doppler_hz = doppler_at_epoch(epoch_idx, sample_index);
        if offset > 0 {
            carrier_phase_cycles += doppler_hz * 0.001;
        }
        epochs.push(locked_tracking_epoch(
            config,
            sat,
            epoch_idx,
            sample_index,
            doppler_hz,
            carrier_phase_cycles,
        ));
    }

    TrackingResult {
        sat,
        carrier_hz: epochs.last().map(|epoch| epoch.carrier_hz.0).unwrap_or_default(),
        code_phase_samples: epochs
            .last()
            .map(|epoch| epoch.code_phase_samples.0)
            .unwrap_or_default(),
        acquisition_hypothesis: "accepted".to_string(),
        acquisition_score: 1.0,
        acquisition_code_phase_samples: 0,
        acquisition_carrier_hz: epochs.first().map(|epoch| epoch.carrier_hz.0).unwrap_or_default(),
        acq_to_track_state: "accepted".to_string(),
        epochs,
        transitions: Vec::new(),
    }
}

fn observation_track_with_constant_doppler(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    doppler_hz: f64,
) -> TrackingResult {
    observation_track_with_doppler_model(config, sat, move |_, _| doppler_hz)
}

fn observation_track_with_linear_doppler(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    initial_doppler_hz: f64,
    doppler_rate_hz_per_s: f64,
) -> TrackingResult {
    observation_track_with_doppler_model(config, sat, move |_, sample_index| {
        expected_linear_doppler_hz(
            sample_index,
            config.sampling_freq_hz,
            initial_doppler_hz,
            doppler_rate_hz_per_s,
        )
    })
}

fn observations_from_tracks(
    config: &ReceiverPipelineConfig,
    tracks: Vec<TrackingResult>,
) -> Vec<ObsEpoch> {
    let report = observations_from_tracking_results(config, &tracks, 10);
    assert!(report.events.is_empty(), "unexpected observation diagnostics: {:?}", report.events);
    report.output
}

fn stable_tracking_dopplers_hz(observations: &[ObsEpoch]) -> Vec<f64> {
    observations
        .iter()
        .flat_map(|epoch| epoch.sats.iter())
        .filter(|sat| {
            sat.metadata.tracking_state == "tracking"
                && sat.lock_flags.code_lock
                && sat.lock_flags.carrier_lock
        })
        .map(|sat| sat.doppler_hz.0)
        .collect()
}

fn stable_tracking_sats(observations: &[ObsEpoch]) -> Vec<&bijux_gnss_core::api::ObsSatellite> {
    observations
        .iter()
        .flat_map(|epoch| epoch.sats.iter())
        .filter(|sat| {
            sat.metadata.tracking_state == "tracking"
                && sat.lock_flags.code_lock
                && sat.lock_flags.carrier_lock
        })
        .collect()
}

fn stable_tracking_observation_rows(observations: &[ObsEpoch]) -> Vec<(u64, f64)> {
    observations
        .iter()
        .flat_map(|epoch| epoch.sats.iter())
        .filter(|sat| {
            sat.metadata.tracking_state == "tracking"
                && sat.lock_flags.code_lock
                && sat.lock_flags.carrier_lock
        })
        .map(|sat| (sat.metadata.time_tag_sample_index, sat.doppler_hz.0))
        .collect()
}

fn expected_linear_doppler_hz(
    sample_index: u64,
    sample_rate_hz: f64,
    initial_doppler_hz: f64,
    doppler_rate_hz_per_s: f64,
) -> f64 {
    initial_doppler_hz + ((sample_index as f64) / sample_rate_hz) * doppler_rate_hz_per_s
}

#[test]
fn observations_declare_tracked_if_relative_doppler_model() {
    let config = observation_tracking_config(2_000.0);
    let sat = SatId { constellation: Constellation::Gps, prn: 13 };
    let observations = observations_from_tracks(
        &config,
        vec![observation_track_with_constant_doppler(&config, sat, -180.0)],
    );
    let stable_sats = stable_tracking_sats(&observations);

    assert!(!stable_sats.is_empty(), "observations={observations:?}");
    assert!(stable_sats.iter().all(|sat| {
        sat.metadata.doppler_model == OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET
    }));
}

#[test]
fn observations_match_clean_constant_tracking_doppler() {
    let config = observation_tracking_config(0.0);
    let sat = SatId { constellation: Constellation::Gps, prn: 11 };
    let true_doppler_hz = 120.0;
    let observations = observations_from_tracks(
        &config,
        vec![observation_track_with_constant_doppler(&config, sat, true_doppler_hz)],
    );
    let stable_dopplers_hz = stable_tracking_dopplers_hz(&observations);

    assert!(!stable_dopplers_hz.is_empty(), "observations={observations:?}");
    assert!(
        stable_dopplers_hz
            .iter()
            .all(|doppler_hz| (*doppler_hz - true_doppler_hz).abs() <= CLEAN_TRACKED_DOPPLER_MAX_ERROR_HZ),
        "stable observation doppler exceeded clean-tracking tolerance {CLEAN_TRACKED_DOPPLER_MAX_ERROR_HZ} Hz: stable_dopplers_hz={stable_dopplers_hz:?}, observations={observations:?}"
    );
}

#[test]
fn observations_preserve_negative_tracked_doppler_with_intermediate_frequency() {
    let config = observation_tracking_config(2_000.0);
    let sat = SatId { constellation: Constellation::Gps, prn: 14 };
    let true_doppler_hz = -250.0;
    let observations = observations_from_tracks(
        &config,
        vec![observation_track_with_constant_doppler(&config, sat, true_doppler_hz)],
    );
    let stable_dopplers_hz = stable_tracking_dopplers_hz(&observations);

    assert!(!stable_dopplers_hz.is_empty(), "observations={observations:?}");
    assert!(
        stable_dopplers_hz
            .iter()
            .all(|doppler_hz| (*doppler_hz - true_doppler_hz).abs() <= CLEAN_TRACKED_DOPPLER_MAX_ERROR_HZ),
        "if-aware observation doppler exceeded tolerance {CLEAN_TRACKED_DOPPLER_MAX_ERROR_HZ} Hz: stable_dopplers_hz={stable_dopplers_hz:?}, observations={observations:?}"
    );
}

#[test]
fn observations_follow_positive_tracked_doppler_ramp() {
    let config = observation_tracking_config(0.0);
    let sat = SatId { constellation: Constellation::Gps, prn: 16 };
    let initial_doppler_hz = 180.0;
    let doppler_rate_hz_per_s = 40.0;
    let observations = observations_from_tracks(
        &config,
        vec![observation_track_with_linear_doppler(
            &config,
            sat,
            initial_doppler_hz,
            doppler_rate_hz_per_s,
        )],
    );
    let stable_rows = stable_tracking_observation_rows(&observations);

    assert!(!stable_rows.is_empty(), "observations={observations:?}");
    assert!(
        stable_rows.iter().all(|(sample_index, doppler_hz)| {
            let expected = expected_linear_doppler_hz(
                *sample_index,
                config.sampling_freq_hz,
                initial_doppler_hz,
                doppler_rate_hz_per_s,
            );
            (*doppler_hz - expected).abs() <= CLEAN_TRACKED_DOPPLER_MAX_ERROR_HZ
        }),
        "ramped observation doppler exceeded tolerance {CLEAN_TRACKED_DOPPLER_MAX_ERROR_HZ} Hz: stable_rows={stable_rows:?}, observations={observations:?}"
    );
}

#[test]
fn observations_follow_negative_tracked_doppler_ramp() {
    let config = observation_tracking_config(0.0);
    let sat = SatId { constellation: Constellation::Gps, prn: 22 };
    let initial_doppler_hz = -220.0;
    let doppler_rate_hz_per_s = -35.0;
    let observations = observations_from_tracks(
        &config,
        vec![observation_track_with_linear_doppler(
            &config,
            sat,
            initial_doppler_hz,
            doppler_rate_hz_per_s,
        )],
    );
    let stable_rows = stable_tracking_observation_rows(&observations);

    assert!(!stable_rows.is_empty(), "observations={observations:?}");
    assert!(
        stable_rows.iter().all(|(sample_index, doppler_hz)| {
            let expected = expected_linear_doppler_hz(
                *sample_index,
                config.sampling_freq_hz,
                initial_doppler_hz,
                doppler_rate_hz_per_s,
            );
            (*doppler_hz - expected).abs() <= CLEAN_TRACKED_DOPPLER_MAX_ERROR_HZ
        }),
        "negative-ramp observation doppler exceeded tolerance {CLEAN_TRACKED_DOPPLER_MAX_ERROR_HZ} Hz: stable_rows={stable_rows:?}, observations={observations:?}"
    );
}
