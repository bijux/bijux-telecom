#![allow(missing_docs)]
use bijux_gnss_core::api::{
    signal_registry, validate_obs_epochs, Constellation, Cycles, Hertz, Meters, ObsEpoch,
    ObsMetadata, ObsSatellite, ReceiverRole, SatId, Seconds, SigId, SignalBand, SignalCode,
};
use proptest::prelude::*;

fn make_epoch(t_rx_s: f64) -> ObsEpoch {
    let sig = SigId {
        sat: SatId { constellation: Constellation::Gps, prn: 1 },
        band: SignalBand::L1,
        code: SignalCode::Ca,
    };
    ObsEpoch {
        t_rx_s: Seconds(t_rx_s),
        gps_week: None,
        tow_s: None,
        epoch_idx: (t_rx_s * 1000.0).round() as u64,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        role: ReceiverRole::Rover,
        sats: vec![ObsSatellite {
            signal_id: sig,
            pseudorange_m: Meters(20_000_000.0),
            pseudorange_var_m2: 1.0,
            carrier_phase_cycles: Cycles(1.0),
            carrier_phase_var_cycles2: 1.0,
            doppler_hz: Hertz(500.0),
            doppler_var_hz2: 1.0,
            cn0_dbhz: 40.0,
            lock_flags: bijux_gnss_core::api::LockFlags {
                code_lock: true,
                carrier_lock: true,
                bit_lock: true,
                cycle_slip: false,
            },
            multipath_suspect: false,
            observation_status: bijux_gnss_core::api::ObservationStatus::Accepted,
            observation_reject_reasons: Vec::new(),
            elevation_deg: Some(45.0),
            azimuth_deg: Some(90.0),
            weight: Some(1.0),
            error_model: None,
            metadata: ObsMetadata {
                tracking_mode: "scalar".to_string(),
                integration_ms: 1,
                lock_quality: 1.0,
                smoothing_window: 1,
                smoothing_age: 0,
                smoothing_resets: 0,
                signal: signal_registry(sig.sat.constellation, sig.band, sig.code)
                    .expect("signal registry must include GPS L1 C/A")
                    .spec,
                ..ObsMetadata::default()
            },
        }],
        decision: bijux_gnss_core::api::ObservationEpochDecision::Accepted,
        decision_reason: Some("accepted_observables_present".to_string()),
        manifest: None,
    }
}

proptest! {
    #[test]
    fn validate_obs_epochs_accepts_monotonic(times in prop::collection::vec(0f64..10f64, 2..20)) {
        let mut times = times;
        times.sort_by(|a,b| a.total_cmp(b));
        let epochs: Vec<_> = times.into_iter().enumerate().map(|(idx, t)| {
            let mut epoch = make_epoch(t + idx as f64 * 1e-6);
            epoch.epoch_idx = idx as u64;
            epoch
        }).collect();
        prop_assert!(validate_obs_epochs(&epochs).is_ok());
    }

    #[test]
    fn validate_obs_epochs_rejects_non_monotonic(t1 in 0f64..10f64, t2 in 0f64..10f64) {
        prop_assume!(t2 < t1);
        let epochs = vec![make_epoch(t1), make_epoch(t2)];
        prop_assert!(validate_obs_epochs(&epochs).is_err());
    }
}
