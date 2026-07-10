#![allow(missing_docs)]

use bijux_gnss_core::api::{
    AcqHypothesis, AcqResult, Constellation, Hertz, ObsEpoch, ReceiverSampleTrace, SatId,
    SignalBand,
};
use bijux_gnss_receiver::api::{
    carrier_hz_from_doppler_hz, observations_from_tracking_results,
    sim::{generate_l1_ca, SyntheticSignalParams},
    ReceiverPipelineConfig, ReceiverRuntime, TrackingEngine,
};

const CLEAN_TRACKED_DOPPLER_CN0_DB_HZ: f32 = 75.0;
const CLEAN_TRACKED_DOPPLER_DURATION_S: f64 = 0.120;
const CLEAN_TRACKED_DOPPLER_MAX_ERROR_HZ: f64 = 10.0;
const VALID_OBSERVATION_START_EPOCH_INDEX: u64 = 70;

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

fn accepted_acquisition(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    doppler_hz: f64,
    code_phase_samples: usize,
) -> AcqResult {
    AcqResult {
        sat,
        signal_band: SignalBand::L1,
        source_time: ReceiverSampleTrace::default(),
        candidate_rank: 1,
        is_primary_candidate: true,
        doppler_hz: Hertz(doppler_hz),
        carrier_hz: Hertz(carrier_hz_from_doppler_hz(config.intermediate_freq_hz, doppler_hz)),
        code_phase_samples,
        peak: 1.0,
        second_peak: 0.1,
        mean: 0.01,
        peak_mean_ratio: 20.0,
        peak_second_ratio: 10.0,
        cn0_proxy: CLEAN_TRACKED_DOPPLER_CN0_DB_HZ,
        score: 1.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: Some("seeded_observation_tracking_start".to_string()),
        doppler_refinement: None,
        code_phase_refinement: None,
        uncertainty: None,
    }
}

fn track_observation_case(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    true_doppler_hz: f64,
    seeded_doppler_hz: f64,
    code_phase_chips: f64,
    carrier_phase_rad: f64,
) -> Vec<ObsEpoch> {
    let expected_code_phase_samples =
        code_phase_chips * config.sampling_freq_hz / config.code_freq_basis_hz;
    let frame = generate_l1_ca(
        config,
        SyntheticSignalParams {
            sat,
            doppler_hz: true_doppler_hz,
            code_phase_chips,
            carrier_phase_rad,
            cn0_db_hz: CLEAN_TRACKED_DOPPLER_CN0_DB_HZ,
            data_bit_flip: false,
        },
        0x0B5E_0001,
        CLEAN_TRACKED_DOPPLER_DURATION_S,
    );
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let mut tracks = tracking.track_from_acquisition(
        &frame,
        &[accepted_acquisition(
            config,
            sat,
            seeded_doppler_hz,
            expected_code_phase_samples.round() as usize,
        )],
    );
    for track in &mut tracks {
        track.epochs.retain(|epoch| epoch.epoch.index >= VALID_OBSERVATION_START_EPOCH_INDEX);
    }
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

#[test]
fn observations_match_clean_constant_tracking_doppler() {
    let config = observation_tracking_config(0.0);
    let sat = SatId { constellation: Constellation::Gps, prn: 11 };
    let true_doppler_hz = 120.0;
    let observations = track_observation_case(&config, sat, true_doppler_hz, 80.0, 211.25, 0.40);
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
    let observations = track_observation_case(&config, sat, true_doppler_hz, -150.0, 322.75, 0.15);
    let stable_dopplers_hz = stable_tracking_dopplers_hz(&observations);

    assert!(!stable_dopplers_hz.is_empty(), "observations={observations:?}");
    assert!(
        stable_dopplers_hz
            .iter()
            .all(|doppler_hz| (*doppler_hz - true_doppler_hz).abs() <= CLEAN_TRACKED_DOPPLER_MAX_ERROR_HZ),
        "if-aware observation doppler exceeded tolerance {CLEAN_TRACKED_DOPPLER_MAX_ERROR_HZ} Hz: stable_dopplers_hz={stable_dopplers_hz:?}, observations={observations:?}"
    );
}
