#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{
    AcqHypothesis, AcqResult, Constellation, Hertz, ReceiverSampleTrace, SatId, SignalBand,
};
use bijux_gnss_receiver::api::{
    sim::{generate_l1_ca_multi, SyntheticScenario, SyntheticSignalParams},
    ReceiverPipelineConfig, ReceiverRuntime, TrackingEngine,
};
const TRACKING_UNCERTAINTY_DURATION_S: f64 = 0.012;

#[test]
fn tracking_epochs_emit_finite_uncertainty() {
    let config = tracking_uncertainty_config();
    let epochs = track_uncertainty_case(&config, 58.0);
    let tracked_epochs = fully_locked_tracking_epochs(&epochs);

    assert!(!tracked_epochs.is_empty(), "missing fully locked tracking epochs: {epochs:?}");
    for epoch in tracked_epochs {
        let uncertainty = epoch
            .tracking_uncertainty
            .as_ref()
            .unwrap_or_else(|| panic!("tracking uncertainty missing for stable epoch: {epoch:?}"));
        assert!(uncertainty.code_phase_samples.is_finite() && uncertainty.code_phase_samples > 0.0);
        assert!(
            uncertainty.carrier_phase_cycles.is_finite() && uncertainty.carrier_phase_cycles > 0.0
        );
        assert!(uncertainty.doppler_hz.is_finite() && uncertainty.doppler_hz > 0.0);
        assert!(uncertainty.cn0_dbhz.is_finite() && uncertainty.cn0_dbhz > 0.0);
    }
}

#[test]
fn degraded_tracking_inflates_uncertainty_over_fully_locked_epochs() {
    let config = tracking_uncertainty_config();
    let epochs = track_uncertainty_case(&config, 58.0);
    let locked_means =
        mean_fully_locked_tracking_uncertainty(&epochs).expect("fully locked uncertainty");
    let degraded_means = mean_degraded_tracking_uncertainty(&epochs).expect("degraded uncertainty");

    assert!(
        degraded_means.0 > locked_means.0,
        "degraded tracking should inflate code-phase uncertainty: locked={locked_means:?} degraded={degraded_means:?}"
    );
    assert!(
        degraded_means.1 > locked_means.1,
        "degraded tracking should inflate carrier-phase uncertainty: locked={locked_means:?} degraded={degraded_means:?}"
    );
    assert!(
        degraded_means.2 > locked_means.2,
        "degraded tracking should inflate Doppler uncertainty: locked={locked_means:?} degraded={degraded_means:?}"
    );
    assert!(
        degraded_means.3 > locked_means.3,
        "degraded tracking should inflate C/N0 uncertainty: locked={locked_means:?} degraded={degraded_means:?}"
    );
}

#[test]
fn weaker_cn0_inflates_emitted_code_uncertainty() {
    let config = tracking_uncertainty_config();
    let strong_epochs = track_uncertainty_case_with_duration(&config, 58.0, 0.08);
    let weak_epochs = track_uncertainty_case_with_duration(&config, 35.0, 0.08);
    let strong_means =
        mean_all_tracking_uncertainty(&strong_epochs).expect("strong-signal uncertainty");
    let weak_means = mean_all_tracking_uncertainty(&weak_epochs).expect("weak-signal uncertainty");

    assert!(
        weak_means.0 > strong_means.0,
        "weaker C/N0 should inflate emitted code uncertainty: strong={strong_means:?} weak={weak_means:?}"
    );
}

fn tracking_uncertainty_config() -> ReceiverPipelineConfig {
    tracking_uncertainty_config_with_integration_ms(1)
}

fn tracking_uncertainty_config_with_integration_ms(
    tracking_integration_ms: u32,
) -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
        code_freq_basis_hz: 1_023_000.0,
        code_length: 1023,
        channels: 4,
        early_late_spacing_chips: 0.5,
        dll_bw_hz: 2.0,
        pll_bw_hz: 18.0,
        fll_bw_hz: 12.0,
        tracking_integration_ms,
        ..ReceiverPipelineConfig::default()
    }
}

fn track_uncertainty_case(
    config: &ReceiverPipelineConfig,
    cn0_db_hz: f32,
) -> Vec<bijux_gnss_core::api::TrackEpoch> {
    track_uncertainty_case_with_duration(config, cn0_db_hz, TRACKING_UNCERTAINTY_DURATION_S)
}

fn track_uncertainty_case_with_duration(
    config: &ReceiverPipelineConfig,
    cn0_db_hz: f32,
    duration_s: f64,
) -> Vec<bijux_gnss_core::api::TrackEpoch> {
    let scenario = SyntheticScenario {
        sample_rate_hz: config.sampling_freq_hz,
        intermediate_freq_hz: config.intermediate_freq_hz,
        receiver_clock_frequency_bias_hz: 0.0,
        duration_s,
        seed: 0x7100_7000 + cn0_db_hz.round() as u64,
        satellites: vec![SyntheticSignalParams {
            sat: SatId { constellation: Constellation::Gps, prn: 7 },
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: -750.0,
            code_phase_chips: 211.25,
            carrier_phase_rad: 0.2,
            cn0_db_hz,
            navigation_data: false.into(),
        }],
        ephemerides: Vec::new(),
        id: format!("tracking_uncertainty_{cn0_db_hz:.0}"),
    };
    let frame = generate_l1_ca_multi(config, &scenario);
    let period_samples = ((config.sampling_freq_hz * config.code_length as f64)
        / config.code_freq_basis_hz)
        .round() as usize;
    let injected_sample = (scenario.satellites[0].code_phase_chips * config.sampling_freq_hz
        / config.code_freq_basis_hz)
        .round() as usize
        % period_samples.max(1);
    let seeded_code_phase_samples =
        (period_samples.max(1) - injected_sample) % period_samples.max(1);
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(
        &frame,
        &[accepted_acquisition(
            scenario.satellites[0].sat,
            scenario.satellites[0].doppler_hz,
            seeded_code_phase_samples,
            cn0_db_hz,
        )],
    );

    tracks.first().expect("track").epochs.clone()
}

fn accepted_acquisition(
    sat: SatId,
    doppler_hz: f64,
    code_phase_samples: usize,
    cn0_db_hz: f32,
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
        doppler_rate_hz_per_s: 0.0,
        carrier_hz: Hertz(doppler_hz),
        code_phase_samples,
        peak: 1.0,
        second_peak: 0.1,
        mean: 0.01,
        peak_mean_ratio: 20.0,
        peak_second_ratio: 10.0,
        cn0_proxy: cn0_db_hz,
        score: 1.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: Some("tracking_uncertainty_seed".to_string()),
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    }
}

fn fully_locked_tracking_epochs(
    epochs: &[bijux_gnss_core::api::TrackEpoch],
) -> Vec<&bijux_gnss_core::api::TrackEpoch> {
    epochs
        .iter()
        .filter(|epoch| epoch.dll_lock && epoch.pll_lock && epoch.fll_lock && !epoch.cycle_slip)
        .collect()
}

fn mean_fully_locked_tracking_uncertainty(
    epochs: &[bijux_gnss_core::api::TrackEpoch],
) -> Option<(f64, f64, f64, f64)> {
    let locked_epochs = fully_locked_tracking_epochs(epochs);
    mean_tracking_uncertainty_rows(&locked_epochs)
}

fn mean_all_tracking_uncertainty(
    epochs: &[bijux_gnss_core::api::TrackEpoch],
) -> Option<(f64, f64, f64, f64)> {
    let rows = epochs.iter().collect::<Vec<_>>();
    mean_tracking_uncertainty_rows(&rows)
}

fn mean_degraded_tracking_uncertainty(
    epochs: &[bijux_gnss_core::api::TrackEpoch],
) -> Option<(f64, f64, f64, f64)> {
    let degraded_epochs = epochs
        .iter()
        .filter(|epoch| {
            epoch.lock_state == "degraded"
                || !epoch.lock
                || epoch.lock_state_reason.as_deref() == Some("signal_fade")
        })
        .collect::<Vec<_>>();
    mean_tracking_uncertainty_rows(&degraded_epochs)
}

fn mean_tracking_uncertainty_rows(
    epochs: &[&bijux_gnss_core::api::TrackEpoch],
) -> Option<(f64, f64, f64, f64)> {
    if epochs.is_empty() {
        return None;
    }
    let mut count = 0usize;
    let mut code = 0.0;
    let mut carrier = 0.0;
    let mut doppler = 0.0;
    let mut cn0 = 0.0;
    for epoch in epochs {
        let uncertainty = epoch.tracking_uncertainty.as_ref()?;
        code += uncertainty.code_phase_samples;
        carrier += uncertainty.carrier_phase_cycles;
        doppler += uncertainty.doppler_hz;
        cn0 += uncertainty.cn0_dbhz;
        count += 1;
    }
    Some((code / count as f64, carrier / count as f64, doppler / count as f64, cn0 / count as f64))
}
