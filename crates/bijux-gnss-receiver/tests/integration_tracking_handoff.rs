#![allow(missing_docs)]

use bijux_gnss_core::api::{
    AcqHypothesis, AcqResult, AcqUncertainty, Constellation, Hertz, ReceiverSampleTrace, SatId,
    SignalBand,
};
use bijux_gnss_receiver::api::{
    signal::samples_per_code,
    sim::{generate_l1_ca, SyntheticSignalParams},
    ReceiverConfig, ReceiverPipelineConfig, ReceiverRuntime, TrackingEngine,
};

fn tight_uncertainty() -> Option<AcqUncertainty> {
    Some(AcqUncertainty {
        doppler_hz: 250.0,
        code_phase_samples: 0.25,
        doppler_rate_hz_per_s: None,
        covariance: None,
    })
}

fn accepted_acquisition(
    sat: SatId,
    signal_band: SignalBand,
    source_time: ReceiverSampleTrace,
    doppler_hz: f64,
    code_phase_samples: usize,
    uncertainty: Option<AcqUncertainty>,
) -> AcqResult {
    AcqResult {
        sat,
        signal_band,
        signal_code: bijux_gnss_core::api::SignalCode::Unknown,
        glonass_frequency_channel: None,
        source_time,
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
        cn0_proxy: 48.0,
        score: 1.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: Some("tracking_handoff".to_string()),
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty,
    }
}

fn synthetic_frame(
    config: &ReceiverPipelineConfig,
    sat: SatId,
) -> bijux_gnss_core::api::SamplesFrame {
    generate_l1_ca(
        config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Ca,
            doppler_hz: 0.0,
            code_phase_chips: 0.0,
            carrier_phase_rad: 0.0,
            cn0_db_hz: 48.0,
            navigation_data: false.into(),
        },
        0x71A0_2026,
        0.012,
    )
}

fn l2_tracking_override_config() -> ReceiverPipelineConfig {
    let config: ReceiverConfig = toml::from_str(
        r#"
schema_version = 1
sample_rate_hz = 5000000.0
intermediate_freq_hz = 0.0
quantization_bits = 16
code_freq_basis_hz = 1023000.0
code_length = 1023
seed = 1

[acquisition]
doppler_search_hz = 10000
doppler_step_hz = 500
integration_ms = 1
noncoherent_integration = 1
peak_mean_threshold = 2.5
peak_second_threshold = 1.5

[tracking]
early_late_spacing_chips = 0.5
dll_bw_hz = 2.0
pll_bw_hz = 15.0
fll_bw_hz = 10.0
max_channels = 8
per_epoch_budget_ms = 0.7
integration_ms = 1

[[tracking.per_band]]
band = "l2"
early_late_spacing_chips = 0.25
dll_bw_hz = 4.5
pll_bw_hz = 18.0
fll_bw_hz = 6.5
integration_ms = 7

[navigation]
robust_solver = true
huber_k = 30.0
raim = true
hatch_window = 100
iono_mode = "broadcast"
tropo_enable = true
tropo_ztd_m = 2.3

[navigation.weighting]
enabled = true
mode = "elevation"
min_elev_deg = 5.0
elev_exponent = 2.0
cn0_ref_dbhz = 50.0
min_weight = 0.1
elev_mask_deg = 5.0
tracking_mode_scalar_weight = 1.0
tracking_mode_vector_weight = 1.2
"#,
    )
    .expect("valid receiver config");
    config.to_pipeline_config()
}

fn e1_tracking_override_config() -> ReceiverPipelineConfig {
    let config: ReceiverConfig = toml::from_str(
        r#"
schema_version = 1
sample_rate_hz = 4092000.0
intermediate_freq_hz = 0.0
quantization_bits = 16
code_freq_basis_hz = 1023000.0
code_length = 4092
seed = 1

[acquisition]
doppler_search_hz = 10000
doppler_step_hz = 500
integration_ms = 20
noncoherent_integration = 1
peak_mean_threshold = 2.5
peak_second_threshold = 1.5

[tracking]
early_late_spacing_chips = 0.5
dll_bw_hz = 2.0
pll_bw_hz = 15.0
fll_bw_hz = 10.0
max_channels = 8
per_epoch_budget_ms = 0.7
integration_ms = 1

[[tracking.per_band]]
band = "e1"
early_late_spacing_chips = 0.25
dll_bw_hz = 4.5
pll_bw_hz = 18.0
fll_bw_hz = 6.5
integration_ms = 2

[navigation]
robust_solver = true
huber_k = 30.0
raim = true
hatch_window = 100
iono_mode = "broadcast"
tropo_enable = true
tropo_ztd_m = 2.3

[navigation.weighting]
enabled = true
mode = "elevation"
min_elev_deg = 5.0
elev_exponent = 2.0
cn0_ref_dbhz = 50.0
min_weight = 0.1
elev_mask_deg = 5.0
tracking_mode_scalar_weight = 1.0
tracking_mode_vector_weight = 1.2
"#,
    )
    .expect("valid receiver config");
    config.to_pipeline_config()
}

#[test]
fn tracking_starts_from_explicit_acquisition_sample_index() {
    let config = ReceiverPipelineConfig::default();
    let sat = SatId { constellation: Constellation::Gps, prn: 11 };
    let mut frame = synthetic_frame(&config, sat);
    frame.t0.sample_index = 10_000;
    let samples_per_code =
        samples_per_code(config.sampling_freq_hz, config.code_freq_basis_hz, config.code_length);
    let explicit_start_sample_index = frame.t0.sample_index + (samples_per_code as u64 * 2);
    let acquisition = accepted_acquisition(
        sat,
        SignalBand::L1,
        ReceiverSampleTrace::from_sample_index(
            explicit_start_sample_index,
            config.sampling_freq_hz,
        ),
        0.0,
        0,
        tight_uncertainty(),
    );

    let tracking = TrackingEngine::new(config, ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(&frame, &[acquisition]);
    let epochs = &tracks.first().expect("track").epochs;
    let first_epoch = epochs.first().expect("tracked epoch");

    assert_eq!(first_epoch.sample_index, explicit_start_sample_index);
    assert_eq!(first_epoch.source_time.sample_index, explicit_start_sample_index);
    assert!(
        epochs.iter().all(|epoch| epoch.sample_index >= explicit_start_sample_index),
        "epochs must not start before the explicit acquisition handoff: {epochs:?}"
    );
    assert!(
        epochs.len() < frame.len() / samples_per_code,
        "tracking should consume fewer epochs after starting mid-frame"
    );
}

#[test]
fn tracking_reports_signal_default_l1_spacing() {
    let config = ReceiverPipelineConfig::default();
    let sat = SatId { constellation: Constellation::Gps, prn: 15 };
    let frame = synthetic_frame(&config, sat);
    let acquisition = accepted_acquisition(
        sat,
        SignalBand::L1,
        ReceiverSampleTrace::from_sample_time(frame.t0),
        0.0,
        0,
        tight_uncertainty(),
    );

    let tracking = TrackingEngine::new(config, ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(&frame, &[acquisition]);
    let first_epoch = tracks.first().and_then(|track| track.epochs.first()).expect("tracked epoch");
    let assumptions = first_epoch.tracking_assumptions.as_ref().expect("tracking assumptions");

    assert_eq!(assumptions.early_late_spacing_chips, 0.25);
    assert_eq!(assumptions.discriminator_family, "early_prompt_late");
    assert!(
        first_epoch
            .tracking_provenance
            .contains("code_discriminator=double_delta_early_prompt_late"),
        "tracking provenance must report the active code discriminator: {first_epoch:?}"
    );
}

#[test]
fn tracking_uses_explicit_signal_band_parameters() {
    let config = l2_tracking_override_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 15 };
    let frame = synthetic_frame(&config, sat);
    let acquisition = accepted_acquisition(
        sat,
        SignalBand::L2,
        ReceiverSampleTrace::from_sample_time(frame.t0),
        0.0,
        0,
        tight_uncertainty(),
    );

    let tracking = TrackingEngine::new(config, ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(&frame, &[acquisition]);
    let first_epoch = tracks.first().and_then(|track| track.epochs.first()).expect("tracked epoch");
    let epochs = &tracks.first().expect("track").epochs;
    let assumptions = first_epoch.tracking_assumptions.as_ref().expect("tracking assumptions");

    assert_eq!(assumptions.integration_ms, 7);
    assert_eq!(assumptions.early_late_spacing_chips, 0.25);
    assert_eq!(assumptions.dll_bw_hz, 4.5);
    assert_eq!(assumptions.pll_bw_hz, 18.0);
    assert_eq!(assumptions.fll_bw_hz, 6.5);
    assert_eq!(epochs.len(), 1, "epochs={epochs:?}");
    assert_eq!(epochs[0].sample_index, 0);
    assert!(
        first_epoch.tracking_provenance.contains("acq_signal_band=L2"),
        "tracking provenance must preserve the explicit acquisition band: {first_epoch:?}"
    );
}

#[test]
fn tracking_uses_explicit_galileo_e1_parameters() {
    let config = e1_tracking_override_config();
    let sat = SatId { constellation: Constellation::Galileo, prn: 15 };
    let frame = synthetic_frame(&config, sat);
    let acquisition = accepted_acquisition(
        sat,
        SignalBand::E1,
        ReceiverSampleTrace::from_sample_time(frame.t0),
        0.0,
        0,
        tight_uncertainty(),
    );

    let tracking = TrackingEngine::new(config, ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(&frame, &[acquisition]);
    let first_epoch = tracks.first().and_then(|track| track.epochs.first()).expect("tracked epoch");
    let epochs = &tracks.first().expect("track").epochs;
    let assumptions = first_epoch.tracking_assumptions.as_ref().expect("tracking assumptions");

    assert_eq!(assumptions.integration_ms, 2);
    assert_eq!(assumptions.early_late_spacing_chips, 0.25);
    assert_eq!(assumptions.dll_bw_hz, 4.5);
    assert_eq!(assumptions.pll_bw_hz, 18.0);
    assert_eq!(assumptions.fll_bw_hz, 6.5);
    assert_eq!(epochs.len(), 2, "epochs={epochs:?}");
    assert_eq!(epochs[0].sample_index, 0);
    assert_eq!(epochs[1].sample_index, 32_736);
    assert!(
        first_epoch.tracking_provenance.contains("acq_signal_band=E1"),
        "tracking provenance must preserve the explicit Galileo E1 band: {first_epoch:?}"
    );
    assert!(
        first_epoch.tracking_provenance.contains("code_discriminator=early_prompt_late"),
        "tracking provenance must report the active Galileo E1 code discriminator: {first_epoch:?}"
    );
}
