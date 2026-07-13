#![allow(missing_docs)]

use bijux_gnss_core::api::{
    AcqHypothesis, AcqResult, Constellation, Hertz, ObsEpoch, ReceiverSampleTrace, SatId,
    SignalBand, OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET,
};
use bijux_gnss_receiver::api::{
    observations_from_tracking_results,
    sim::{generate_l1_ca, SyntheticSignalParams},
    ReceiverPipelineConfig, ReceiverRuntime, TrackingEngine,
};

const CLEAN_TRACKING_CN0_DBHZ: f32 = 60.0;
const CLEAN_TRACKING_DOPPLER_MAX_ERROR_HZ: f64 = 10.0;

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
        cn0_proxy: CLEAN_TRACKING_CN0_DBHZ,
        score: 1.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: Some("observation_doppler_tracking_seed".to_string()),
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    }
}

fn tracking_config(intermediate_freq_hz: f64) -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 1_023_000.0,
        intermediate_freq_hz,
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

fn track_clean_signal(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    true_doppler_hz: f64,
) -> Vec<ObsEpoch> {
    let frame = generate_l1_ca(
        config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: true_doppler_hz,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.30,
            cn0_db_hz: CLEAN_TRACKING_CN0_DBHZ,
            data_bit_flip: false,
        },
        0x6D6F_4450,
        0.020,
    );
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks =
        tracking.track_from_acquisition(&frame, &[accepted_acquisition(sat, true_doppler_hz, 0)]);
    observations_from_tracking_results(config, &tracks, 10).output
}

fn locked_observation_rows(observations: &[ObsEpoch]) -> Vec<(String, f64)> {
    observations
        .iter()
        .flat_map(|epoch| epoch.sats.iter())
        .filter(|sat| sat.lock_flags.code_lock && sat.lock_flags.carrier_lock)
        .map(|sat| (sat.metadata.doppler_model.clone(), sat.doppler_hz.0))
        .collect()
}

#[test]
fn observations_emit_positive_doppler_from_tracking_output() {
    let config = tracking_config(0.0);
    let sat = SatId { constellation: Constellation::Gps, prn: 19 };
    let true_doppler_hz = 120.0;
    let observations = track_clean_signal(&config, sat, true_doppler_hz);
    let locked_rows = locked_observation_rows(&observations);

    assert!(!locked_rows.is_empty(), "observations={observations:?}");
    assert!(locked_rows.iter().all(|(doppler_model, doppler_hz)| {
        doppler_model == OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET
            && (*doppler_hz - true_doppler_hz).abs() <= CLEAN_TRACKING_DOPPLER_MAX_ERROR_HZ
    }));
}

#[test]
fn observations_emit_negative_if_relative_doppler_from_tracking_output() {
    let config = tracking_config(2_000.0);
    let sat = SatId { constellation: Constellation::Gps, prn: 24 };
    let true_doppler_hz = -180.0;
    let observations = track_clean_signal(&config, sat, true_doppler_hz);
    let locked_rows = locked_observation_rows(&observations);

    assert!(!locked_rows.is_empty(), "observations={observations:?}");
    assert!(locked_rows.iter().all(|(doppler_model, doppler_hz)| {
        doppler_model == OBSERVATION_DOPPLER_MODEL_TRACKED_CARRIER_IF_OFFSET
            && (*doppler_hz - true_doppler_hz).abs() <= CLEAN_TRACKING_DOPPLER_MAX_ERROR_HZ
    }));
}
