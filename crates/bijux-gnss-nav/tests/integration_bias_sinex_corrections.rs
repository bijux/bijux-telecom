#![allow(missing_docs)]

use bijux_gnss_core::api::{
    first_order_ionosphere_code_delay_m, signal_spec_galileo_e1b, signal_spec_galileo_e5a,
    signal_spec_gps_l1_ca, signal_spec_gps_l2_py, Constellation, Cycles, Hertz, LockFlags, Meters,
    ObsEpoch, ObsMetadata, ObsSatellite, ObservationEpochDecision, ObservationStatus, ReceiverRole,
    ReceiverSampleTrace, SatId, Seconds, SigId, SignalBand, SignalCode, SignalSpec,
};
use bijux_gnss_nav::api::{iono_free_code_from_obs_epochs_with_biases, BiasSinexProvider};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

#[test]
fn absolute_bias_sinex_corrects_gps_l1_l2_iono_free_code() {
    let provider = include_str!("data/gps_l1_l2_absolute_biases.bia")
        .parse::<BiasSinexProvider>()
        .expect("bias sinex parse");
    let sat = SatId { constellation: Constellation::Gps, prn: 23 };
    let l1 = signal_spec_gps_l1_ca();
    let l2 = signal_spec_gps_l2_py();
    let base_range_m = 20_200_000.0;
    let iono_l1_m = 4.0;
    let l1_bias_m = 3.0e-9 * SPEED_OF_LIGHT_MPS;
    let l2_bias_m = 9.0e-9 * SPEED_OF_LIGHT_MPS;
    let epoch = dual_frequency_epoch(
        sat,
        SignalBand::L1,
        SignalCode::Ca,
        l1,
        base_range_m + iono_l1_m + l1_bias_m,
        SignalBand::L2,
        SignalCode::Py,
        l2,
        base_range_m + dispersive_delay_at_band(iono_l1_m, l1, l2) + l2_bias_m,
    );

    let observations = iono_free_code_from_obs_epochs_with_biases(
        &[epoch],
        SignalBand::L1,
        SignalBand::L2,
        Some(&provider),
    );

    assert_eq!(observations.len(), 1);
    assert_eq!(observations[0].status, "ok");
    assert!(observations[0].code_bias_m.expect("iono-free code bias").abs() > 0.1);
    assert!(
        (observations[0].corrected_code_m.expect("corrected iono-free code") - base_range_m).abs()
            < 1.0e-6
    );
}

#[test]
fn relative_bias_sinex_derives_galileo_e1_e5_corrections() {
    let provider = include_str!("data/galileo_e1_e5_relative_biases.bia")
        .parse::<BiasSinexProvider>()
        .expect("bias sinex parse");
    let sat = SatId { constellation: Constellation::Galileo, prn: 8 };
    let e1 = signal_spec_galileo_e1b();
    let e5 = signal_spec_galileo_e5a();
    let base_range_m = 23_400_000.0;
    let iono_e1_m = 6.0;
    let isb_m = 5.0e-9 * SPEED_OF_LIGHT_MPS;
    let dsb_m = 7.9161e-9 * SPEED_OF_LIGHT_MPS;
    let (weight_1, weight_2) = iono_free_weights(e1.carrier_hz.value(), e5.carrier_hz.value());
    let e1_bias_m = isb_m + weight_2 * dsb_m;
    let e5_bias_m = isb_m - weight_1 * dsb_m;
    let epoch = dual_frequency_epoch(
        sat,
        SignalBand::E1,
        SignalCode::E1B,
        e1,
        base_range_m + iono_e1_m + e1_bias_m,
        SignalBand::E5,
        SignalCode::E5a,
        e5,
        base_range_m + dispersive_delay_at_band(iono_e1_m, e1, e5) + e5_bias_m,
    );

    let observations = iono_free_code_from_obs_epochs_with_biases(
        &[epoch],
        SignalBand::E1,
        SignalBand::E5,
        Some(&provider),
    );

    assert_eq!(observations.len(), 1);
    assert_eq!(observations[0].status, "ok");
    assert!((observations[0].code_bias_m.expect("iono-free code bias") - isb_m).abs() < 1.0e-6);
    assert!(
        (observations[0].corrected_code_m.expect("corrected iono-free code") - base_range_m).abs()
            < 1.0e-6
    );
}

fn dual_frequency_epoch(
    sat: SatId,
    first_band: SignalBand,
    first_code: SignalCode,
    first_signal: SignalSpec,
    first_pseudorange_m: f64,
    second_band: SignalBand,
    second_code: SignalCode,
    second_signal: SignalSpec,
    second_pseudorange_m: f64,
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
            satellite(sat, first_band, first_code, first_signal, first_pseudorange_m),
            satellite(sat, second_band, second_code, second_signal, second_pseudorange_m),
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
    pseudorange_m: f64,
) -> ObsSatellite {
    ObsSatellite {
        signal_id: SigId { sat, band, code },
        pseudorange_m: Meters(pseudorange_m),
        pseudorange_var_m2: 1.0,
        carrier_phase_cycles: Cycles(0.0),
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

fn dispersive_delay_at_band(
    iono_l1_m: f64,
    l1_signal: SignalSpec,
    target_signal: SignalSpec,
) -> f64 {
    first_order_ionosphere_code_delay_m(Meters(iono_l1_m), l1_signal, target_signal)
        .expect("finite first-order ionosphere delay")
        .0
}

fn iono_free_weights(f1_hz: f64, f2_hz: f64) -> (f64, f64) {
    let f1_2 = f1_hz * f1_hz;
    let f2_2 = f2_hz * f2_hz;
    let denom = f1_2 - f2_2;
    (f1_2 / denom, -f2_2 / denom)
}
