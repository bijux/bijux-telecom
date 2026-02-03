use bijux_gnss_core::{
    Constellation, LockFlags, ObsEpoch, ObsMetadata, ObsSatellite, SignalBand, SignalSpec,
};
use bijux_gnss_receiver::differencing::{double_difference, single_difference};
use bijux_gnss_receiver::rtk::{build_dd, build_sd, choose_ref_sat_per_constellation};

fn make_epoch(prn: u8, pseudo: f64, phase: f64, doppler: f64) -> ObsEpoch {
    ObsEpoch {
        t_rx_s: 0.0,
        gps_week: None,
        tow_s: None,
        epoch_idx: 0,
        discontinuity: false,
        sats: vec![ObsSatellite {
            prn,
            pseudorange_m: pseudo,
            carrier_phase_cycles: phase,
            doppler_hz: doppler,
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
                    code_rate_hz: 1_023_000.0,
                    carrier_hz: 1_575_420_000.0,
                },
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
    assert!((sds[0].code_m - 100.0).abs() < 1e-6);

    let rover2 = make_epoch(2, 21_000_000.0, 1_500.0, -400.0);
    let base2 = make_epoch(2, 20_999_900.0, 1_499.8, -380.0);
    let mut rover_all = rover.clone();
    rover_all.sats.push(rover2.sats[0].clone());
    let mut base_all = base.clone();
    base_all.sats.push(base2.sats[0].clone());
    let sds = single_difference(&rover_all, &base_all);
    let dds = double_difference(&sds, 1);
    assert_eq!(dds.len(), 1);
    assert_eq!(dds[0].ref_prn, 1);
}

#[test]
fn rtk_sd_dd_builders_work() {
    let rover = make_epoch(1, 20_000_100.0, 1_000.5, -500.0);
    let base = make_epoch(1, 20_000_000.0, 1_000.0, -450.0);
    let sd = build_sd(&base, &rover);
    assert_eq!(sd.len(), 1);
    let refs = choose_ref_sat_per_constellation(&sd);
    let ref_prn = *refs.values().next().expect("ref");
    let dd = build_dd(&sd, ref_prn);
    assert!(dd.is_empty());
}
