#![allow(missing_docs)]

use bijux_gnss_core::api::{
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
        l1,
        l2,
        base_range_m + iono_l1_m + l1_bias_m,
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

fn dual_frequency_epoch(
    sat: SatId,
    first_signal: SignalSpec,
    second_signal: SignalSpec,
    first_pseudorange_m: f64,
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
            satellite(sat, SignalBand::L1, SignalCode::Ca, first_signal, first_pseudorange_m),
            satellite(sat, SignalBand::L2, SignalCode::Py, second_signal, second_pseudorange_m),
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
    iono_l1_m * (l1_signal.carrier_hz.value() * l1_signal.carrier_hz.value())
        / (target_signal.carrier_hz.value() * target_signal.carrier_hz.value())
}
