#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{
    AcqHypothesis, AcqResult, Constellation, Hertz, ReceiverSampleTrace, SatId, SignalBand,
};
use bijux_gnss_receiver::api::{
    sim::{
        expected_acquisition_code_phase_samples, generate_l1_ca_with_carrier_dynamics,
        SyntheticCarrierDynamicsParams, SyntheticSignalParams,
    },
    ReceiverPipelineConfig, ReceiverRuntime, TrackingEngine,
};

use support::tracking_truth::{
    first_tracking_lock_epoch_index, post_lock_carrier_frequency_errors_under_quadratic_doppler_hz,
    post_lock_code_phase_errors_samples, post_lock_epochs, stable_tracking_window,
};

const CARRIER_DYNAMICS_CN0_DB_HZ: f32 = 75.0;
const CARRIER_DYNAMICS_DURATION_S: f64 = 0.080;
const BOUNDED_DOPPLER_RATE_HZ_PER_S: f64 = 40.0;
const BOUNDED_DOPPLER_JERK_HZ_PER_S2: f64 = 1_000.0;
const BOUNDED_CARRIER_ERROR_MAX_HZ: f64 = 18.0;
const BOUNDED_CODE_ERROR_MAX_SAMPLES: f64 = 1.25;
const BOUNDED_MIN_STABLE_TRACKING_EPOCHS: usize = 5;

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
        doppler_rate_hz_per_s: BOUNDED_DOPPLER_RATE_HZ_PER_S,
        carrier_hz: Hertz(doppler_hz),
        code_phase_samples,
        peak: 1.0,
        second_peak: 0.1,
        mean: 0.01,
        peak_mean_ratio: 20.0,
        peak_second_ratio: 10.0,
        cn0_proxy: CARRIER_DYNAMICS_CN0_DB_HZ,
        score: 1.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: Some("carrier_dynamics_tracking_start".to_string()),
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    }
}

fn carrier_dynamics_tracking_config() -> ReceiverPipelineConfig {
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
        adaptive_tracking_enabled: true,
        tracking_integration_ms: 1,
        ..ReceiverPipelineConfig::default()
    }
}

fn carrier_dynamics_signal(
    sat: SatId,
    initial_doppler_hz: f64,
    code_phase_chips: f64,
    carrier_phase_rad: f64,
    doppler_rate_hz_per_s: f64,
    doppler_jerk_hz_per_s2: f64,
) -> SyntheticCarrierDynamicsParams {
    SyntheticCarrierDynamicsParams {
        signal: SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Unknown,
            doppler_hz: initial_doppler_hz,
            code_phase_chips,
            carrier_phase_rad,
            cn0_db_hz: CARRIER_DYNAMICS_CN0_DB_HZ,
            navigation_data: false.into(),
        },
        doppler_rate_hz_per_s,
        doppler_jerk_hz_per_s2,
    }
}

#[test]
fn tracking_preserves_lock_inside_declared_acceleration_and_jerk_envelope() {
    let config = carrier_dynamics_tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 24 };
    let initial_doppler_hz = 180.0;
    let code_phase_chips = 211.25;
    let carrier_phase_rad = 0.0;
    let frame = generate_l1_ca_with_carrier_dynamics(
        &config,
        carrier_dynamics_signal(
            sat,
            initial_doppler_hz,
            code_phase_chips,
            carrier_phase_rad,
            BOUNDED_DOPPLER_RATE_HZ_PER_S,
            BOUNDED_DOPPLER_JERK_HZ_PER_S2,
        ),
        0xC4D1_6001,
        CARRIER_DYNAMICS_DURATION_S,
    );
    let seeded_code_phase_samples =
        expected_acquisition_code_phase_samples(&config, &frame, code_phase_chips);
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(
        &frame,
        &[accepted_acquisition(sat, initial_doppler_hz, seeded_code_phase_samples)],
    );
    let epochs = &tracks.first().expect("track").epochs;
    let first_lock_epoch_index = first_tracking_lock_epoch_index(epochs).unwrap_or_else(|| {
        panic!("tracking never reached stable lock under carrier dynamics: epochs={epochs:?}")
    });
    let post_lock = post_lock_epochs(epochs);
    let carrier_errors_hz = post_lock_carrier_frequency_errors_under_quadratic_doppler_hz(
        epochs,
        config.sampling_freq_hz,
        initial_doppler_hz,
        BOUNDED_DOPPLER_RATE_HZ_PER_S,
        BOUNDED_DOPPLER_JERK_HZ_PER_S2,
    );
    let code_errors_samples =
        post_lock_code_phase_errors_samples(&config, epochs, seeded_code_phase_samples as f64);

    assert!(epochs.len() >= 70, "epochs={epochs:?}");
    assert!(
        post_lock.iter().all(|epoch| {
            epoch.lock
                && matches!(epoch.lock_state.as_str(), "tracking" | "degraded")
                && !epoch.cycle_slip
        }),
        "tracking did not preserve lock after carrier dynamics lock at epoch {first_lock_epoch_index}: epochs={epochs:?}"
    );
    assert!(
        stable_tracking_window(epochs, BOUNDED_MIN_STABLE_TRACKING_EPOCHS).len()
            >= BOUNDED_MIN_STABLE_TRACKING_EPOCHS,
        "tracking did not sustain a stable tracking window: epochs={epochs:?}"
    );
    assert!(
        carrier_errors_hz.iter().all(|error_hz| *error_hz <= BOUNDED_CARRIER_ERROR_MAX_HZ),
        "carrier error exceeded bounded dynamics envelope: carrier_errors_hz={carrier_errors_hz:?}, epochs={epochs:?}"
    );
    assert!(
        code_errors_samples.iter().all(|error_samples| *error_samples <= BOUNDED_CODE_ERROR_MAX_SAMPLES),
        "code error exceeded bounded dynamics envelope: code_errors_samples={code_errors_samples:?}, epochs={epochs:?}"
    );
}
