#![allow(missing_docs)]

mod support;

use bijux_gnss_core::api::{
    AcqHypothesis, AcqResult, Constellation, Hertz, ReceiverSampleTrace, SatId, SignalBand,
};
use bijux_gnss_receiver::api::{
    sim::{
        expected_acquisition_code_phase_samples, expected_acquisition_code_phase_samples_f64,
        generate_l1_ca, SyntheticSignalParams,
    },
    ReceiverPipelineConfig, ReceiverRuntime, TrackingEngine,
};

use support::tracking_truth::{
    carrier_frequency_error_hz, carrier_phase_steps_cycles, code_phase_error_samples,
    nav_bit_transition_epoch_indices, post_lock_epochs, wrapped_code_phase_error_samples,
};

const CLEAN_NAV_BIT_CN0_DB_HZ: f32 = 52.0;
const CLEAN_NAV_BIT_DURATION_S: f64 = 0.065;
const CLEAN_NAV_BIT_LOCKED_CARRIER_ERROR_MAX_HZ: f64 = 15.0;
const CLEAN_NAV_BIT_LOCKED_CODE_ERROR_MAX_SAMPLES: f64 = 1.0;
const CLEAN_NAV_BIT_MIN_PHASE_STEP_CYCLES: f64 = 0.01;
const CLEAN_NAV_BIT_MAX_PHASE_STEP_CYCLES: f64 = 0.35;
const GPS_L1CA_NAV_BIT_PERIOD_MS: usize = 20;

fn accepted_acquisition(sat: SatId, doppler_hz: f64, code_phase_samples: usize) -> AcqResult {
    AcqResult {
        sat,
        signal_band: SignalBand::L1,
        signal_code: bijux_gnss_core::api::SignalCode::Ca,
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
        cn0_proxy: CLEAN_NAV_BIT_CN0_DB_HZ,
        score: 1.0,
        hypothesis: AcqHypothesis::Accepted,
        assumptions: None,
        evidence: Vec::new(),
        threshold_provenance: None,
        explain_selection_reason: Some("seeded_nav_bit_tracking_start".to_string()),
        doppler_refinement: None,
        code_phase_refinement: None,
        signal_delay_alignment: None,
        uncertainty: None,
    }
}

fn nav_bit_tracking_config() -> ReceiverPipelineConfig {
    ReceiverPipelineConfig {
        sampling_freq_hz: 4_092_000.0,
        intermediate_freq_hz: 0.0,
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

fn track_clean_nav_bit_case(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    doppler_hz: f64,
    code_phase_chips: f64,
    carrier_phase_rad: f64,
) -> Vec<bijux_gnss_core::api::TrackEpoch> {
    track_clean_nav_bit_case_with_cn0(
        config,
        sat,
        doppler_hz,
        code_phase_chips,
        carrier_phase_rad,
        CLEAN_NAV_BIT_CN0_DB_HZ,
    )
}

fn track_clean_nav_bit_case_with_cn0(
    config: &ReceiverPipelineConfig,
    sat: SatId,
    doppler_hz: f64,
    code_phase_chips: f64,
    carrier_phase_rad: f64,
    cn0_db_hz: f32,
) -> Vec<bijux_gnss_core::api::TrackEpoch> {
    let frame = generate_l1_ca(
        config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Ca,
            doppler_hz,
            code_phase_chips,
            carrier_phase_rad,
            cn0_db_hz,
            data_bit_flip: true,
        },
        0x57A1_1C7D,
        CLEAN_NAV_BIT_DURATION_S,
    );
    let seeded_code_phase_samples =
        expected_acquisition_code_phase_samples(config, &frame, code_phase_chips);
    let tracking = TrackingEngine::new(config.clone(), ReceiverRuntime::default());
    let tracks = tracking.track_from_acquisition(
        &frame,
        &[accepted_acquisition(sat, doppler_hz, seeded_code_phase_samples)],
    );

    tracks.first().expect("track").epochs.clone()
}

fn first_post_lock_nav_bit_transition_epoch_index(
    epochs: &[bijux_gnss_core::api::TrackEpoch],
    sample_rate_hz: f64,
) -> usize {
    let first_lock_epoch_index = epochs
        .iter()
        .position(|epoch| epoch.lock_state == "tracking" && epoch.pll_lock && epoch.fll_lock)
        .unwrap_or_else(|| panic!("tracking never reached stable lock: epochs={epochs:?}"));
    nav_bit_transition_epoch_indices(epochs, sample_rate_hz)
        .into_iter()
        .find(|epoch_index| *epoch_index >= first_lock_epoch_index)
        .unwrap_or_else(|| {
            panic!(
                "tracking never observed a nav-bit transition after lock: first_lock_epoch_index={first_lock_epoch_index}, epochs={epochs:?}"
            )
        })
}

fn post_transition_epochs(
    epochs: &[bijux_gnss_core::api::TrackEpoch],
    sample_rate_hz: f64,
) -> &[bijux_gnss_core::api::TrackEpoch] {
    let first_transition_epoch_index =
        first_post_lock_nav_bit_transition_epoch_index(epochs, sample_rate_hz);
    &epochs[first_transition_epoch_index..]
}

#[test]
fn tracking_reports_nav_bit_lock_across_clean_bit_transition_windows() {
    let config = nav_bit_tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 12 };
    let epochs = track_clean_nav_bit_case(&config, sat, 120.0, 144.375, 0.3);
    let post_lock = post_lock_epochs(&epochs);
    let transition_epochs = nav_bit_transition_epoch_indices(&epochs, config.sampling_freq_hz);
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Ca,
            doppler_hz: 120.0,
            code_phase_chips: 144.375,
            carrier_phase_rad: 0.3,
            cn0_db_hz: CLEAN_NAV_BIT_CN0_DB_HZ,
            data_bit_flip: true,
        },
        0x57A1_1C7D,
        CLEAN_NAV_BIT_DURATION_S,
    );
    let expected_code_phase_samples =
        expected_acquisition_code_phase_samples_f64(&config, &frame, 144.375);

    assert!(epochs.len() >= 60, "epochs={epochs:?}");
    assert!(!post_lock.is_empty(), "tracking never reached lock: epochs={epochs:?}");
    assert_eq!(transition_epochs, vec![20, 40, 60], "epochs={epochs:?}");
    assert!(
        epochs.iter().any(|epoch| epoch.nav_bit_lock),
        "tracking never reported nav-bit lock: epochs={epochs:?}"
    );
    assert!(
        post_lock.iter().any(|epoch| wrapped_code_phase_error_samples(
            &config,
            epoch.code_phase_samples.0,
            expected_code_phase_samples
        ) <= 1.0),
        "tracking never settled near the seeded code phase in nav-bit scenario: epochs={epochs:?}"
    );
}

#[test]
fn tracking_preserves_channel_lock_after_the_first_post_lock_nav_bit_transition() {
    let config = nav_bit_tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 12 };
    let epochs = track_clean_nav_bit_case(&config, sat, 120.0, 144.375, 0.3);
    let first_transition_epoch_index =
        first_post_lock_nav_bit_transition_epoch_index(&epochs, config.sampling_freq_hz);
    let post_transition_epochs = post_transition_epochs(&epochs, config.sampling_freq_hz);

    assert!(
        post_transition_epochs
            .iter()
            .all(|epoch| epoch.lock),
        "prompt lock dropped after nav-bit transition at epoch {first_transition_epoch_index}: epochs={epochs:?}"
    );
    assert!(
        post_transition_epochs
            .iter()
            .all(|epoch| epoch.lock_state != "lost" && !epoch.cycle_slip),
        "tracking reported loss or cycle slip after nav-bit transition at epoch {first_transition_epoch_index}: epochs={epochs:?}"
    );
    assert!(
        post_transition_epochs
            .iter()
            .all(|epoch| epoch.lock_state_reason.as_deref() != Some("lock_lost")),
        "tracking reported explicit lock loss after nav-bit transition at epoch {first_transition_epoch_index}: epochs={epochs:?}"
    );
    assert!(
        post_transition_epochs
            .iter()
            .any(|epoch| epoch.nav_bit_lock),
        "tracking never advertised nav-bit lock after the first nav-bit transition: epochs={epochs:?}"
    );
}

#[test]
fn tracking_reaches_lock_before_high_doppler_nav_bit_transition() {
    let config = nav_bit_tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 3 };
    let epochs = track_clean_nav_bit_case_with_cn0(&config, sat, 750.0, 200.125, 0.0, 58.0);
    let transition_epochs = nav_bit_transition_epoch_indices(&epochs, config.sampling_freq_hz);
    let first_transition_epoch_index =
        *transition_epochs.first().unwrap_or_else(|| panic!("epochs={epochs:?}"));
    let pre_transition_epochs = &epochs[..first_transition_epoch_index];
    let post_transition_epochs = &epochs[first_transition_epoch_index..];

    assert!(
        pre_transition_epochs.iter().any(|epoch| {
            epoch.lock_state == "tracking" && epoch.pll_lock && epoch.fll_lock && !epoch.cycle_slip
        }),
        "tracking never reached a clean locked state before the first nav-bit transition: epochs={epochs:?}"
    );
    assert!(
        post_transition_epochs
            .iter()
            .all(|epoch| epoch.lock && epoch.lock_state != "lost" && !epoch.cycle_slip),
        "tracking did not preserve lock through the first high-doppler nav-bit transition: epochs={epochs:?}"
    );
}

#[test]
fn tracking_keeps_bounded_code_and_carrier_errors_after_nav_bit_lock() {
    let config = nav_bit_tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 12 };
    let true_doppler_hz = 120.0;
    let code_phase_chips = 144.375;
    let epochs = track_clean_nav_bit_case(&config, sat, true_doppler_hz, code_phase_chips, 0.3);
    let frame = generate_l1_ca(
        &config,
        SyntheticSignalParams {
            sat,
            glonass_frequency_channel: None,
            signal_band: bijux_gnss_core::api::SignalBand::L1,
            signal_code: bijux_gnss_core::api::SignalCode::Ca,
            doppler_hz: true_doppler_hz,
            code_phase_chips,
            carrier_phase_rad: 0.3,
            cn0_db_hz: CLEAN_NAV_BIT_CN0_DB_HZ,
            data_bit_flip: true,
        },
        0x57A1_1C7D,
        CLEAN_NAV_BIT_DURATION_S,
    );
    let expected_code_phase_samples =
        expected_acquisition_code_phase_samples_f64(&config, &frame, code_phase_chips);
    let post_transition_epochs = post_transition_epochs(&epochs, config.sampling_freq_hz);
    let carrier_errors_hz = post_transition_epochs
        .iter()
        .map(|epoch| carrier_frequency_error_hz(epoch, true_doppler_hz))
        .collect::<Vec<_>>();
    let code_errors_samples = post_transition_epochs
        .iter()
        .map(|epoch| code_phase_error_samples(&config, epoch, expected_code_phase_samples))
        .collect::<Vec<_>>();

    assert!(
        post_transition_epochs.iter().all(|epoch| epoch.nav_bit_lock),
        "nav-bit lock did not remain asserted after the first nav-bit transition: epochs={epochs:?}"
    );
    assert!(
        carrier_errors_hz
            .iter()
            .all(|error_hz| *error_hz <= CLEAN_NAV_BIT_LOCKED_CARRIER_ERROR_MAX_HZ),
        "carrier error exceeded nav-bit lock threshold {CLEAN_NAV_BIT_LOCKED_CARRIER_ERROR_MAX_HZ} Hz: carrier_errors_hz={carrier_errors_hz:?}, epochs={epochs:?}"
    );
    assert!(
        code_errors_samples
            .iter()
            .all(|error_samples| *error_samples <= CLEAN_NAV_BIT_LOCKED_CODE_ERROR_MAX_SAMPLES),
        "code error exceeded nav-bit lock threshold {CLEAN_NAV_BIT_LOCKED_CODE_ERROR_MAX_SAMPLES} samples: code_errors_samples={code_errors_samples:?}, epochs={epochs:?}"
    );
}

#[test]
fn tracking_keeps_carrier_phase_continuous_through_nav_bit_transition() {
    let config = nav_bit_tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 12 };
    let epochs = track_clean_nav_bit_case(&config, sat, 120.0, 144.375, 0.3);
    let first_transition_epoch_index =
        first_post_lock_nav_bit_transition_epoch_index(&epochs, config.sampling_freq_hz);
    let transition_window = &epochs[first_transition_epoch_index.saturating_sub(1)..];
    let phase_steps_cycles = carrier_phase_steps_cycles(transition_window);

    assert!(
        transition_window.len() >= 3,
        "tracking did not leave enough epochs around the first nav-bit transition: epochs={epochs:?}"
    );
    assert!(
        phase_steps_cycles
            .iter()
            .all(|step_cycles| step_cycles.is_finite()),
        "carrier phase steps must remain finite through nav-bit transitions: phase_steps_cycles={phase_steps_cycles:?}, epochs={epochs:?}"
    );
    assert!(
        phase_steps_cycles
            .iter()
            .all(|step_cycles| *step_cycles >= CLEAN_NAV_BIT_MIN_PHASE_STEP_CYCLES),
        "carrier phase must keep advancing through nav-bit transitions instead of wrapping backward: phase_steps_cycles={phase_steps_cycles:?}, epochs={epochs:?}"
    );
    assert!(
        phase_steps_cycles
            .iter()
            .all(|step_cycles| *step_cycles <= CLEAN_NAV_BIT_MAX_PHASE_STEP_CYCLES),
        "carrier phase must not take slip-sized jumps at nav-bit transitions: phase_steps_cycles={phase_steps_cycles:?}, epochs={epochs:?}"
    );
}

#[test]
fn tracking_recovers_navigation_bit_signs_from_prompt_history() {
    let config = nav_bit_tracking_config();
    let sat = SatId { constellation: Constellation::Gps, prn: 12 };
    let epochs = track_clean_nav_bit_case(&config, sat, 120.0, 144.375, 0.3);
    let recovered_epochs = epochs
        .iter()
        .filter_map(|epoch| epoch.navigation_bit_sign.map(|sign| (epoch.sample_index, sign)))
        .collect::<Vec<_>>();
    let expected_recovered_epoch_count =
        (epochs.len() / GPS_L1CA_NAV_BIT_PERIOD_MS) * GPS_L1CA_NAV_BIT_PERIOD_MS;
    let recovered_blocks =
        recovered_epochs.chunks_exact(GPS_L1CA_NAV_BIT_PERIOD_MS).collect::<Vec<_>>();

    assert!(
        !recovered_epochs.is_empty(),
        "tracking did not recover nav-bit signs: epochs={epochs:?}"
    );
    assert_eq!(
        recovered_epochs.len(),
        expected_recovered_epoch_count,
        "tracking should recover one sign for every complete 20 ms navigation bit window: epochs={epochs:?}"
    );
    assert_eq!(
        recovered_blocks.len(),
        3,
        "synthetic 65 ms run should yield three complete recovered navigation bits: recovered_epochs={recovered_epochs:?}, epochs={epochs:?}"
    );
    assert!(
        recovered_blocks.iter().all(|block| {
            let block_sign = block[0].1;
            block.iter().all(|(_, sign)| *sign == block_sign)
        }),
        "recovered navigation-bit signs must remain constant within each 20 ms block: recovered_epochs={recovered_epochs:?}, epochs={epochs:?}"
    );
    assert!(
        recovered_blocks.windows(2).all(|pair| pair[0][0].1 == -pair[1][0].1),
        "adjacent recovered navigation-bit blocks must flip sign in the alternating synthetic scenario: recovered_epochs={recovered_epochs:?}, epochs={epochs:?}"
    );
}
