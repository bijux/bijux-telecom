use bijux_gnss_core::api::{
    ArtifactPayloadValidate, Chips, Constellation, Cycles, Epoch, Hertz, ReceiverSampleTrace,
    SatId, TrackEpoch, TrackingUncertainty,
};

fn sample_track_epoch() -> TrackEpoch {
    TrackEpoch {
        epoch: Epoch { index: 7 },
        sample_index: 4_092,
        source_time: ReceiverSampleTrace::from_sample_index(4_092, 4_092_000.0),
        sat: SatId { constellation: Constellation::Gps, prn: 7 },
        prompt_i: 1.0,
        prompt_q: 0.0,
        early_i: 0.5,
        early_q: 0.0,
        late_i: 0.5,
        late_q: 0.0,
        carrier_hz: Hertz(250.0),
        carrier_phase_cycles: Cycles(0.0),
        code_rate_hz: Hertz(1_023_000.0),
        code_phase_samples: Chips(12.5),
        lock: true,
        cn0_dbhz: 45.0,
        pll_lock: true,
        dll_lock: true,
        fll_lock: true,
        cycle_slip: false,
        nav_bit_lock: false,
        navigation_bit_sign: None,
        dll_err: 0.0,
        pll_err: 0.0,
        fll_err: 0.0,
        anti_false_lock: false,
        cycle_slip_reason: None,
        lock_state: "tracking".to_string(),
        lock_state_reason: Some("stable_tracking".to_string()),
        channel_id: Some(0),
        channel_uid: "Gps-07-ch00".to_string(),
        tracking_provenance: "tracking-artifact-validation".to_string(),
        tracking_assumptions: None,
        signal_delay_alignment: None,
        tracking_uncertainty: Some(TrackingUncertainty {
            code_phase_samples: 0.25,
            carrier_phase_cycles: 0.05,
            doppler_hz: 1.5,
            cn0_dbhz: 0.75,
        }),
        processing_ms: None,
    }
}

#[test]
fn tracking_artifact_validation_accepts_finite_uncertainty() {
    let diagnostics = sample_track_epoch().validate_payload();
    assert!(
        diagnostics.iter().all(|event| event.code != "GNSS_NUMERIC_TRACK_UNCERTAINTY_INVALID"),
        "{diagnostics:?}"
    );
}

#[test]
fn tracking_artifact_validation_rejects_invalid_uncertainty() {
    let mut epoch = sample_track_epoch();
    epoch.tracking_uncertainty = Some(TrackingUncertainty {
        code_phase_samples: -0.25,
        carrier_phase_cycles: 0.05,
        doppler_hz: 1.5,
        cn0_dbhz: f64::NAN,
    });

    let diagnostics = epoch.validate_payload();

    assert!(
        diagnostics.iter().any(|event| event.code == "GNSS_NUMERIC_TRACK_UNCERTAINTY_INVALID"),
        "{diagnostics:?}"
    );
}

#[test]
fn tracking_artifact_validation_rejects_invalid_navigation_bit_sign() {
    let mut epoch = sample_track_epoch();
    epoch.navigation_bit_sign = Some(0);

    let diagnostics = epoch.validate_payload();

    assert!(
        diagnostics
            .iter()
            .any(|event| event.code == "GNSS_TRACK_NAVIGATION_BIT_SIGN_INVALID"),
        "{diagnostics:?}"
    );
}
