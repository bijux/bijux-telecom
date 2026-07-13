#![allow(missing_docs)]

use bijux_gnss_core::api::{
    Constellation, Cycles, Hertz, LockFlags, Meters, ObsEpoch, ObsMetadata, ObsSatellite,
    ObservationEpochDecision, ObservationStatus, ReceiverRole, ReceiverSampleTrace, SatId, Seconds,
    SigId, SignalBand, SignalCode, SignalSpec,
};
use bijux_gnss_nav::api::combinations_from_obs_epochs;
use bijux_gnss_signal::api::{
    signal_spec_beidou_b1i, signal_spec_beidou_b2i, signal_spec_galileo_e1b,
    signal_spec_galileo_e5a, signal_spec_gps_l1_ca, signal_spec_gps_l2_py, signal_spec_gps_l5,
};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

fn dual_frequency_epoch(
    sat: SatId,
    first: (SignalBand, SignalCode, SignalSpec, f64),
    second: (SignalBand, SignalCode, SignalSpec, f64),
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
            satellite(sat, first.0, first.1, first.2, first.3),
            satellite(sat, second.0, second.1, second.2, second.3),
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
) -> ObsSatellite {
    let wavelength_m = SPEED_OF_LIGHT_MPS / signal.carrier_hz.value();
    let base_range_m = 24_000_000.0;
    ObsSatellite {
        signal_id: SigId { sat, band, code },
        pseudorange_m: Meters(base_range_m),
        pseudorange_var_m2: 1.0,
        carrier_phase_cycles: Cycles(base_range_m / wavelength_m + ambiguity_cycles),
        carrier_phase_var_cycles2: 0.01,
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

fn assert_wide_lane_formula(
    sat: SatId,
    first: (SignalBand, SignalCode, SignalSpec, f64),
    second: (SignalBand, SignalCode, SignalSpec, f64),
) {
    let epoch = dual_frequency_epoch(sat, first, second);
    let combinations = combinations_from_obs_epochs(&[epoch], first.0, second.0);
    assert_eq!(combinations.len(), 1);
    assert_eq!(combinations[0].status, "ok");

    let lambda_1 = SPEED_OF_LIGHT_MPS / first.2.carrier_hz.value();
    let lambda_2 = SPEED_OF_LIGHT_MPS / second.2.carrier_hz.value();
    let expected_wide_lane_wavelength_m =
        SPEED_OF_LIGHT_MPS / (first.2.carrier_hz.value() - second.2.carrier_hz.value()).abs();
    let expected_wide_lane_cycles =
        (lambda_1 * first.3 - lambda_2 * second.3) / expected_wide_lane_wavelength_m;

    assert!(
        (combinations[0].wide_lane_wavelength_m.expect("wide-lane wavelength")
            - expected_wide_lane_wavelength_m)
            .abs()
            < 1.0e-12
    );
    assert!(
        (combinations[0].wide_lane_cycles.expect("wide-lane cycles") - expected_wide_lane_cycles)
            .abs()
            < 1.0e-6
    );
}

#[test]
fn gps_l1_l2_wide_lane_matches_wavelength_and_ambiguity_formula() {
    assert_wide_lane_formula(
        SatId { constellation: Constellation::Gps, prn: 11 },
        (SignalBand::L1, SignalCode::Ca, signal_spec_gps_l1_ca(), 17.0),
        (SignalBand::L2, SignalCode::Py, signal_spec_gps_l2_py(), 11.0),
    );
}

#[test]
fn gps_l1_l5_wide_lane_matches_wavelength_and_ambiguity_formula() {
    assert_wide_lane_formula(
        SatId { constellation: Constellation::Gps, prn: 12 },
        (SignalBand::L1, SignalCode::Ca, signal_spec_gps_l1_ca(), 19.0),
        (SignalBand::L5, SignalCode::Unknown, signal_spec_gps_l5(), 7.0),
    );
}

#[test]
fn galileo_e1_e5_wide_lane_matches_wavelength_and_ambiguity_formula() {
    assert_wide_lane_formula(
        SatId { constellation: Constellation::Galileo, prn: 19 },
        (SignalBand::E1, SignalCode::E1B, signal_spec_galileo_e1b(), 23.0),
        (SignalBand::E5, SignalCode::E5a, signal_spec_galileo_e5a(), 9.0),
    );
}

#[test]
fn beidou_b1_b2_wide_lane_matches_wavelength_and_ambiguity_formula() {
    assert_wide_lane_formula(
        SatId { constellation: Constellation::Beidou, prn: 7 },
        (SignalBand::B1, SignalCode::B1I, signal_spec_beidou_b1i(), 21.0),
        (SignalBand::B2, SignalCode::B2I, signal_spec_beidou_b2i(), 8.0),
    );
}
