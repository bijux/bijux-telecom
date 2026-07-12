#![allow(missing_docs)]

use bijux_gnss_core::api::{
    signal_spec_beidou_b1i, signal_spec_beidou_b2i, signal_spec_galileo_e1b,
    signal_spec_galileo_e5a, signal_spec_gps_l1_ca, signal_spec_gps_l2_py, signal_spec_gps_l5,
    Constellation, Cycles, Hertz, LockFlags, Meters, ObsEpoch, ObsMetadata, ObsSatellite,
    ObservationEpochDecision, ObservationStatus, ReceiverRole, ReceiverSampleTrace, SatId,
    Seconds, SigId, SignalBand, SignalCode, SignalSpec,
};
use bijux_gnss_nav::api::narrow_lane_from_obs_epochs;

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

fn dual_frequency_epoch(
    sat: SatId,
    first: (SignalBand, SignalCode, SignalSpec, f64, f64),
    second: (SignalBand, SignalCode, SignalSpec, f64, f64),
) -> ObsEpoch {
    ObsEpoch {
        t_rx_s: Seconds(0.0),
        source_time: ReceiverSampleTrace::from_sample_index(0, 1_000.0),
        gps_week: None,
        tow_s: None,
        epoch_idx: 0,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        role: ReceiverRole::Rover,
        sats: vec![
            satellite(sat, first.0, first.1, first.2, first.3, first.4),
            satellite(sat, second.0, second.1, second.2, second.3, second.4),
        ],
        decision: ObservationEpochDecision::Accepted,
        decision_reason: Some("accepted_observables_present".to_string()),
        manifest: None,
    }
}

fn satellite(
    sat: SatId,
    band: SignalBand,
    code: SignalCode,
    signal: SignalSpec,
    ambiguity_cycles: f64,
    variance_cycles2: f64,
) -> ObsSatellite {
    let wavelength_m = SPEED_OF_LIGHT_MPS / signal.carrier_hz.value();
    let base_range_m = 24_000_000.0;
    ObsSatellite {
        signal_id: SigId { sat, band, code },
        pseudorange_m: Meters(base_range_m),
        pseudorange_var_m2: 1.0,
        carrier_phase_cycles: Cycles(base_range_m / wavelength_m + ambiguity_cycles),
        carrier_phase_var_cycles2: variance_cycles2,
        doppler_hz: Hertz(0.0),
        doppler_var_hz2: 1.0,
        cn0_dbhz: 45.0,
        lock_flags: LockFlags {
            code_lock: true,
            carrier_lock: true,
            bit_lock: false,
            cycle_slip: false,
        },
        multipath_suspect: false,
        observation_status: ObservationStatus::Accepted,
        observation_reject_reasons: Vec::new(),
        elevation_deg: None,
        azimuth_deg: None,
        weight: None,
        timing: None,
        error_model: None,
        metadata: ObsMetadata {
            tracking_mode: "test".to_string(),
            integration_ms: 1,
            lock_quality: 45.0,
            smoothing_window: 0,
            smoothing_age: 0,
            smoothing_resets: 0,
            signal,
            ..ObsMetadata::default()
        },
    }
}

fn assert_narrow_lane_formula(
    sat: SatId,
    first: (SignalBand, SignalCode, SignalSpec, f64, f64),
    second: (SignalBand, SignalCode, SignalSpec, f64, f64),
) {
    let epoch = dual_frequency_epoch(sat, first, second);
    let observations = narrow_lane_from_obs_epochs(&[epoch], first.0, second.0);
    assert_eq!(observations.len(), 1);
    assert_eq!(observations[0].status, "ok");

    let base_range_m = 24_000_000.0;
    let lambda_1 = SPEED_OF_LIGHT_MPS / first.2.carrier_hz.value();
    let lambda_2 = SPEED_OF_LIGHT_MPS / second.2.carrier_hz.value();
    let expected_narrow_lane_wavelength_m =
        SPEED_OF_LIGHT_MPS / (first.2.carrier_hz.value() + second.2.carrier_hz.value());
    let expected_phase_m = 2.0 * base_range_m + lambda_1 * first.3 + lambda_2 * second.3;
    let expected_phase_cycles = expected_phase_m / expected_narrow_lane_wavelength_m;
    let expected_variance_m2 =
        lambda_1.powi(2) * first.4 + lambda_2.powi(2) * second.4;
    let expected_variance_cycles2 =
        expected_variance_m2 / expected_narrow_lane_wavelength_m.powi(2);

    assert!(
        (observations[0]
            .narrow_lane_wavelength_m
            .expect("narrow-lane wavelength")
            - expected_narrow_lane_wavelength_m)
            .abs()
            < 1.0e-12
    );
    assert!((observations[0].phase_m.expect("phase meters") - expected_phase_m).abs() < 1.0e-6);
    assert!(
        (observations[0].phase_cycles.expect("phase cycles") - expected_phase_cycles).abs()
            < 1.0e-6
    );
    assert!(
        (observations[0].variance_m2.expect("variance meters") - expected_variance_m2).abs()
            < 1.0e-12
    );
    assert!(
        (observations[0]
            .variance_cycles2
            .expect("variance cycles")
            - expected_variance_cycles2)
            .abs()
            < 1.0e-12
    );
}

#[test]
fn gps_l1_l2_narrow_lane_matches_wavelength_ambiguity_and_variance_formula() {
    assert_narrow_lane_formula(
        SatId { constellation: Constellation::Gps, prn: 11 },
        (SignalBand::L1, SignalCode::Ca, signal_spec_gps_l1_ca(), 17.0, 0.01),
        (SignalBand::L2, SignalCode::Py, signal_spec_gps_l2_py(), 11.0, 0.04),
    );
}

#[test]
fn gps_l1_l5_narrow_lane_matches_wavelength_ambiguity_and_variance_formula() {
    assert_narrow_lane_formula(
        SatId { constellation: Constellation::Gps, prn: 12 },
        (SignalBand::L1, SignalCode::Ca, signal_spec_gps_l1_ca(), 19.0, 0.01),
        (SignalBand::L5, SignalCode::Unknown, signal_spec_gps_l5(), 7.0, 0.09),
    );
}

#[test]
fn galileo_e1_e5_narrow_lane_matches_wavelength_ambiguity_and_variance_formula() {
    assert_narrow_lane_formula(
        SatId { constellation: Constellation::Galileo, prn: 19 },
        (SignalBand::E1, SignalCode::E1B, signal_spec_galileo_e1b(), 23.0, 0.01),
        (SignalBand::E5, SignalCode::E5a, signal_spec_galileo_e5a(), 9.0, 0.04),
    );
}

#[test]
fn beidou_b1_b2_narrow_lane_matches_wavelength_ambiguity_and_variance_formula() {
    assert_narrow_lane_formula(
        SatId { constellation: Constellation::Beidou, prn: 7 },
        (SignalBand::B1, SignalCode::B1I, signal_spec_beidou_b1i(), 21.0, 0.01),
        (SignalBand::B2, SignalCode::B2I, signal_spec_beidou_b2i(), 8.0, 0.16),
    );
}
