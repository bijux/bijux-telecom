#![allow(missing_docs)]

use bijux_gnss_core::api::{
    AcqHypothesis, AcqResult, AcqUncertainty, Constellation, Hertz, ReceiverSampleTrace, SatId,
    SignalBand,
};
use bijux_gnss_receiver::api::{
    sim::{generate_l1_ca, SyntheticSignalParams},
    ReceiverPipelineConfig, ReceiverRuntime, TrackingEngine,
};

fn accepted_acquisition(
    sat: SatId,
    doppler_hz: f64,
    code_phase_samples: usize,
    uncertainty: Option<AcqUncertainty>,
) -> AcqResult {
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
        cn0_proxy: 48.0,
        score: 1.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: Some("seeded_tracking_start".to_string()),
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty,
    }
}

fn tight_tracking_uncertainty() -> Option<AcqUncertainty> {
    Some(AcqUncertainty { doppler_hz: 250.0, code_phase_samples: 0.25 })
}

fn synthetic_frame(
    config: &ReceiverPipelineConfig,
    code_phase_chips: f64,
) -> bijux_gnss_core::api::SamplesFrame {
    generate_l1_ca(
        config,
        SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 11 },
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: 0.0,
            code_phase_chips,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 48.0,
            data_bit_flip: false,
        },
        0x5A17_2026,
        0.012,
    )
}

#[test]
fn tracking_preserves_stable_code_phase_with_matching_sample_rate() {
    let config = ReceiverPipelineConfig::default();
    let frame = synthetic_frame(&config, 0.0);
    let sat = SatId { constellation: Constellation::Gps, prn: 11 };
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let acquisitions = vec![accepted_acquisition(
        sat,
        config.intermediate_freq_hz,
        0,
        tight_tracking_uncertainty(),
    )];

    let tracks = tracking.track_from_acquisition(&frame, &acquisitions);
    let epochs = &tracks.first().expect("track").epochs;

    assert!(
        epochs
            .iter()
            .all(|epoch| epoch.lock_state_reason.as_deref() != Some("sample_rate_mismatch")),
        "epochs={epochs:?}"
    );
}

#[test]
fn tracking_marks_persistent_code_phase_drift_as_sample_rate_mismatch() {
    let true_config = ReceiverPipelineConfig::default();
    let frame = synthetic_frame(&true_config, 0.0);
    let sat = SatId { constellation: Constellation::Gps, prn: 11 };

    let mut declared_config = true_config.clone();
    declared_config.sampling_freq_hz = 5_050_000.0;
    let tracking = TrackingEngine::new(declared_config.clone(), ReceiverRuntime::default());
    let acquisitions = vec![accepted_acquisition(
        sat,
        declared_config.intermediate_freq_hz,
        0,
        tight_tracking_uncertainty(),
    )];

    let tracks = tracking.track_from_acquisition(&frame, &acquisitions);
    let epochs = &tracks.first().expect("track").epochs;

    let first_mismatch = epochs
        .iter()
        .find(|epoch| epoch.lock_state_reason.as_deref() == Some("sample_rate_mismatch"))
        .unwrap_or_else(|| panic!("sample rate mismatch must be reported: {epochs:?}"));
    assert!(first_mismatch.cycle_slip || !first_mismatch.dll_lock || !first_mismatch.pll_lock);
}

#[test]
fn tracking_does_not_claim_sample_rate_mismatch_when_acquisition_uncertainty_is_wide() {
    let true_config = ReceiverPipelineConfig::default();
    let frame = synthetic_frame(&true_config, 0.0);
    let sat = SatId { constellation: Constellation::Gps, prn: 11 };

    let mut declared_config = true_config.clone();
    declared_config.sampling_freq_hz = 5_050_000.0;
    let tracking = TrackingEngine::new(declared_config.clone(), ReceiverRuntime::default());
    let acquisitions = vec![accepted_acquisition(
        sat,
        declared_config.intermediate_freq_hz,
        0,
        Some(AcqUncertainty { doppler_hz: 1_000.0, code_phase_samples: 2.0 }),
    )];

    let tracks = tracking.track_from_acquisition(&frame, &acquisitions);
    let epochs = &tracks.first().expect("track").epochs;

    assert!(
        epochs
            .iter()
            .all(|epoch| epoch.lock_state_reason.as_deref() != Some("sample_rate_mismatch")),
        "wide acquisition uncertainty must suppress a hidden sample-rate claim: {epochs:?}"
    );
}
