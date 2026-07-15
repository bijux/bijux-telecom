use super::*;

fn vector_measurement(
    channel_id: u8,
    sample_index: u64,
    cn0_dbhz: f64,
    fll_error_hz: f64,
    code_rate_error_hz: f64,
    carrier_rate_hz_per_s: f64,
) -> super::VectorTrackingMeasurement {
    super::VectorTrackingMeasurement {
        sat: SatId { constellation: Constellation::Gps, prn: channel_id + 1 },
        channel_id,
        epoch_idx: sample_index / 5_000,
        sample_index,
        cn0_dbhz,
        dll_error_samples: 0.05,
        pll_error_rad: 0.02,
        fll_error_hz,
        code_rate_error_hz,
        carrier_rate_hz_per_s,
        prompt_locked: true,
        dll_locked: true,
        pll_locked: true,
        fll_locked: true,
        channel_state: ChannelState::Tracking,
    }
}

#[test]
fn vector_tracking_rejects_weak_or_incomplete_evidence() {
    let weak = vector_measurement(0, 5_000, 34.9, 1.0, 0.1, 0.0);
    let mut unlocked = vector_measurement(1, 5_000, 42.0, 1.0, 0.1, 0.0);
    unlocked.pll_locked = false;
    let strong = vector_measurement(2, 5_000, 42.0, 1.0, 0.1, 0.0);

    assert!(!super::vector_tracking_measurement_is_usable(weak));
    assert!(!super::vector_tracking_measurement_is_usable(unlocked));
    assert!(super::vector_tracking_measurement_is_usable(strong));
    assert!(super::vector_tracking_prediction(&[strong]).is_none());
}

#[test]
fn vector_tracking_prediction_uses_latest_measurement_per_channel() {
    let mut state = super::VectorTrackingState::default();
    state.record(vector_measurement(0, 100, 42.0, 1.0, 0.10, 4.0), 1_000.0);
    state.record(vector_measurement(0, 110, 42.0, 5.0, 0.50, 8.0), 1_000.0);
    state.record(vector_measurement(1, 108, 42.0, 7.0, 0.70, 12.0), 1_000.0);

    let prediction = state.prediction_for(120, 1_000.0).expect("vector prediction");

    assert_eq!(prediction.contributor_count, 2);
    assert_eq!(prediction.sample_index, 110);
    assert!((prediction.receiver_position_code_phase_error_samples - 0.05).abs() < 1.0e-9);
    assert!((prediction.receiver_clock_frequency_error_hz - 6.0).abs() < 1.0e-9);
    assert!((prediction.receiver_clock_frequency_residual_spread_hz - 2.0).abs() < 1.0e-9);
    assert!((prediction.receiver_clock_frequency_max_residual_hz - 1.0).abs() < 1.0e-9);
    assert!((prediction.receiver_code_rate_error_hz - 0.60).abs() < 1.0e-9);
    assert!((prediction.receiver_motion_frequency_rate_hz_per_s - 10.0).abs() < 1.0e-9);
}

#[test]
fn common_tracking_frequency_estimate_uses_weighted_stable_channel_residuals() {
    let low_weight = vector_measurement(0, 100, 35.0, 4.0, 0.10, 1_000.0);
    let high_weight = vector_measurement(1, 110, 45.0, 8.0, 0.20, -1_000.0);

    let estimate = super::common_tracking_frequency_estimate(&[low_weight, high_weight])
        .expect("common tracking frequency estimate");

    assert_eq!(estimate.support_count, 2);
    assert_eq!(estimate.sample_index, 110);
    assert!((estimate.estimated_frequency_error_hz - 7.636_363_636_363_637).abs() < 1.0e-9);
    assert!((estimate.mean_cn0_dbhz - 44.090_909_090_909_09).abs() < 1.0e-9);
    assert_eq!(estimate.supporting_channels.len(), 2);
    assert!(estimate.supporting_channels.iter().all(|channel| channel.residual_hz.is_finite()));
    assert!(
        estimate.max_supporting_residual_hz > 0.0
            && estimate.residual_spread_hz > estimate.max_supporting_residual_hz,
        "estimate should expose residual spread and maximum residual: {estimate:?}"
    );
}

#[test]
fn common_tracking_frequency_estimate_rejects_insufficient_stable_support() {
    let mut unlocked = vector_measurement(0, 100, 45.0, 4.0, 0.10, 0.0);
    unlocked.pll_locked = false;
    let stable = vector_measurement(1, 110, 45.0, 8.0, 0.20, 0.0);

    assert!(super::common_tracking_frequency_estimate(&[unlocked, stable]).is_none());
}

#[test]
fn common_tracking_frequency_estimate_rejects_channel_frequency_outliers() {
    let common_left = vector_measurement(0, 100, 45.0, -2.0, 0.10, 0.0);
    let common_right = vector_measurement(1, 100, 45.0, 1.0, 0.10, 0.0);
    let outlier = vector_measurement(2, 100, 45.0, -148.0, 0.10, 0.0);

    let estimate = super::common_tracking_frequency_estimate(&[common_left, common_right, outlier])
        .expect("common tracking frequency estimate");

    assert_eq!(estimate.support_count, 2);
    assert!(
        estimate.supporting_channels.iter().all(|channel| channel.frequency_error_hz > -10.0),
        "{estimate:?}"
    );
    assert!(estimate.max_supporting_residual_hz <= 1.5, "{estimate:?}");
}

#[test]
fn common_tracking_frequency_estimate_does_not_absorb_satellite_specific_motion() {
    let fast_approaching = vector_measurement(0, 100, 45.0, 6.0, 0.10, 4_000.0);
    let fast_receding = vector_measurement(1, 110, 45.0, 6.0, 0.20, -4_000.0);

    let estimate = super::common_tracking_frequency_estimate(&[fast_approaching, fast_receding])
        .expect("common tracking frequency estimate");

    assert!((estimate.estimated_frequency_error_hz - 6.0).abs() < 1.0e-9);
    assert!(estimate.max_supporting_residual_hz <= f64::EPSILON);
}

#[test]
fn vector_tracking_state_reports_latest_common_frequency_estimate() {
    let mut state = super::VectorTrackingState::default();
    state.record(vector_measurement(0, 100, 45.0, 2.0, 0.10, 0.0), 1_000.0);
    state.record(vector_measurement(0, 120, 45.0, 4.0, 0.10, 0.0), 1_000.0);
    state.record(vector_measurement(1, 118, 45.0, 6.0, 0.10, 0.0), 1_000.0);

    let estimate = state.common_frequency_estimate(1_000.0).expect("common frequency estimate");

    assert_eq!(estimate.support_count, 2);
    assert_eq!(estimate.sample_index, 120);
    assert!((estimate.estimated_frequency_error_hz - 5.0).abs() < 1.0e-9);
    assert!(estimate.supporting_channels.iter().all(|channel| channel.frequency_error_hz >= 4.0));
}

#[test]
fn vector_tracking_application_bounds_weak_channel_aid() {
    let config = ReceiverPipelineConfig::default();
    let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
    let sat = SatId { constellation: Constellation::Gps, prn: 1 };
    let signal_model = super::TrackingSignalModel::for_sat(&config, sat);
    let mut loop_state = tracking.initial_loop_state(
        &signal_model,
        1500.0,
        0.0,
        31.0,
        None,
        false,
        config.tracking_params(SignalBand::L1),
        false,
    );
    loop_state.state = ChannelState::PullIn;
    let prediction = super::VectorTrackingPrediction {
        sample_index: 10_000,
        contributor_count: 3,
        mean_cn0_dbhz: 44.0,
        receiver_position_code_phase_error_samples: 10.0,
        receiver_clock_frequency_error_hz: 1_000.0,
        receiver_clock_frequency_residual_spread_hz: 0.0,
        receiver_clock_frequency_max_residual_hz: 0.0,
        receiver_code_rate_error_hz: 50.0,
        receiver_motion_frequency_rate_hz_per_s: 5_000.0,
    };

    let application =
        super::vector_tracking_application(prediction, &loop_state).expect("vector aid");

    assert!(
        (application.carrier_frequency_correction_hz
            + super::VECTOR_TRACKING_MAX_CARRIER_AID_HZ * 0.35)
            .abs()
            < 1.0e-9
    );
    assert!(
        (application.code_rate_correction_hz - super::VECTOR_TRACKING_MAX_CODE_RATE_AID_HZ * 0.35)
            .abs()
            < 1.0e-9
    );
    assert!(
        (application.code_phase_correction_samples
            - super::VECTOR_TRACKING_MAX_CODE_PHASE_AID_SAMPLES * 0.35)
            .abs()
            < 1.0e-9
    );
    assert!(
        (application.carrier_rate_correction_hz_per_s
            - super::VECTOR_TRACKING_MAX_CARRIER_RATE_AID_HZ_PER_S * 0.35)
            .abs()
            < 1.0e-9
    );
    loop_state.state = ChannelState::Lost;
    assert!(super::vector_tracking_application(prediction, &loop_state).is_none());
}

#[test]
fn vector_tracking_application_preserves_stable_frequency_support_channels() {
    let config = ReceiverPipelineConfig::default();
    let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
    let sat = SatId { constellation: Constellation::Gps, prn: 1 };
    let signal_model = super::TrackingSignalModel::for_sat(&config, sat);
    let mut loop_state = tracking.initial_loop_state(
        &signal_model,
        1500.0,
        0.0,
        45.0,
        None,
        false,
        config.tracking_params(SignalBand::L1),
        false,
    );
    loop_state.state = ChannelState::Tracking;
    let prediction = super::VectorTrackingPrediction {
        sample_index: 10_000,
        contributor_count: 3,
        mean_cn0_dbhz: 45.0,
        receiver_position_code_phase_error_samples: 0.20,
        receiver_clock_frequency_error_hz: 6.0,
        receiver_clock_frequency_residual_spread_hz: 0.50,
        receiver_clock_frequency_max_residual_hz: 0.25,
        receiver_code_rate_error_hz: 1.0,
        receiver_motion_frequency_rate_hz_per_s: 4.0,
    };

    let application =
        super::vector_tracking_application(prediction, &loop_state).expect("vector aid");

    assert_eq!(application.carrier_frequency_correction_hz, 0.0);
    assert!(
        application.code_rate_correction_hz > 0.0,
        "stable support channels should still consume non-carrier vector aids: {application:?}"
    );
}

#[test]
fn vector_tracking_application_reduces_common_frequency_prediction_error() {
    let config = ReceiverPipelineConfig::default();
    let tracking = Tracking::new(config.clone(), ReceiverRuntime::default());
    let sat = SatId { constellation: Constellation::Gps, prn: 1 };
    let signal_model = super::TrackingSignalModel::for_sat(&config, sat);
    let mut loop_state = tracking.initial_loop_state(
        &signal_model,
        43.0,
        0.0,
        31.0,
        None,
        false,
        config.tracking_params(SignalBand::L1),
        false,
    );
    loop_state.state = ChannelState::PullIn;
    let prediction = super::VectorTrackingPrediction {
        sample_index: 10_000,
        contributor_count: 3,
        mean_cn0_dbhz: 45.0,
        receiver_position_code_phase_error_samples: 0.0,
        receiver_clock_frequency_error_hz: -4.0,
        receiver_clock_frequency_residual_spread_hz: 0.50,
        receiver_clock_frequency_max_residual_hz: 0.25,
        receiver_code_rate_error_hz: 0.0,
        receiver_motion_frequency_rate_hz_per_s: 0.0,
    };

    let application =
        super::vector_tracking_application(prediction, &loop_state).expect("vector aid");
    let unaided_error_hz = (loop_state.carrier_hz - 45.0).abs();
    let aided_error_hz =
        (loop_state.carrier_hz + application.carrier_frequency_correction_hz - 45.0).abs();

    assert!(
            aided_error_hz < unaided_error_hz,
            "common frequency correction should improve the carrier prediction: application={application:?} unaided_error_hz={unaided_error_hz} aided_error_hz={aided_error_hz}"
        );
}
