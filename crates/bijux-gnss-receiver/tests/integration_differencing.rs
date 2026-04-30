#![allow(missing_docs)]
use bijux_gnss_core::api::{
    Constellation, LockFlags, ObsEpoch, ObsMetadata, ObsSatellite, ReceiverRole, SatId, SigId,
    SignalBand, SignalSpec,
};
use bijux_gnss_receiver::api::{build_dd, build_sd, choose_ref_sat_per_constellation};
use bijux_gnss_receiver::api::{double_difference, single_difference};

fn make_epoch(prn: u8, pseudo: f64, phase: f64, doppler: f64) -> ObsEpoch {
    ObsEpoch {
        t_rx_s: bijux_gnss_core::api::Seconds(0.0),
        gps_week: None,
        tow_s: None,
        epoch_idx: 0,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        role: ReceiverRole::Rover,
        sats: vec![ObsSatellite {
            signal_id: SigId {
                sat: SatId { constellation: Constellation::Gps, prn },
                band: SignalBand::L1,
                code: bijux_gnss_core::api::SignalCode::Ca,
            },
            pseudorange_m: bijux_gnss_core::api::Meters(pseudo),
            pseudorange_var_m2: 1.0,
            carrier_phase_cycles: bijux_gnss_core::api::Cycles(phase),
            carrier_phase_var_cycles2: 0.01,
            doppler_hz: bijux_gnss_core::api::Hertz(doppler),
            doppler_var_hz2: 4.0,
            cn0_dbhz: 45.0,
            lock_flags: LockFlags {
                code_lock: true,
                carrier_lock: true,
                bit_lock: false,
                cycle_slip: false,
            },
            multipath_suspect: false,
            elevation_deg: None,
            azimuth_deg: None,
            weight: None,
            error_model: None,
            metadata: ObsMetadata {
                tracking_mode: "test".to_string(),
                integration_ms: 1,
                lock_quality: 45.0,
                smoothing_window: 0,
                smoothing_age: 0,
                smoothing_resets: 0,
                signal: SignalSpec {
                    constellation: Constellation::Gps,
                    band: SignalBand::L1,
                    code: bijux_gnss_core::api::SignalCode::Ca,
                    code_rate_hz: 1_023_000.0,
                    carrier_hz: bijux_gnss_core::api::GPS_L1_CA_CARRIER_HZ,
                },
                ..ObsMetadata::default()
            },
        }],
    }
}

#[test]
fn single_and_double_difference_basic() {
    let rover = make_epoch(1, 20_000_100.0, 1_000.5, -500.0);
    let base = make_epoch(1, 20_000_000.0, 1_000.0, -450.0);
    let sds = single_difference(&rover, &base);
    assert_eq!(sds.len(), 1);
    assert!((sds[0].code_m.0 - 100.0).abs() < 1e-6);

    let rover2 = make_epoch(2, 21_000_000.0, 1_500.0, -400.0);
    let base2 = make_epoch(2, 20_999_900.0, 1_499.8, -380.0);
    let mut rover_all = rover.clone();
    rover_all.sats.push(rover2.sats[0].clone());
    let mut base_all = base.clone();
    base_all.sats.push(base2.sats[0].clone());
    let sds = single_difference(&rover_all, &base_all);
    let ref_sig = sds[0].sig;
    let dds = double_difference(&sds, ref_sig);
    assert_eq!(dds.len(), 1);
    assert_eq!(dds[0].ref_sig.sat.prn, 1);
}

#[test]
fn rtk_sd_dd_builders_work() {
    let rover = make_epoch(1, 20_000_100.0, 1_000.5, -500.0);
    let base = make_epoch(1, 20_000_000.0, 1_000.0, -450.0);
    let sd = build_sd(&base, &rover);
    assert_eq!(sd.len(), 1);
    let refs = choose_ref_sat_per_constellation(&sd);
    let ref_sig = *refs.values().next().expect("ref");
    let dd = build_dd(&sd, ref_sig);
    assert!(dd.is_empty());
}
