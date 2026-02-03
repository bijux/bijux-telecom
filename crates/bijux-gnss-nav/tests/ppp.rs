use bijux_gnss_core::{
    signal_spec_gps_l1_ca, Constellation, LockFlags, ObsEpoch, ObsMetadata, ObsSatellite,
    ReceiverRole, SatId, SigId, SignalBand, SignalCode,
};
use bijux_gnss_nav::{BroadcastProductsProvider, GpsEphemeris, PppConfig, PppFilter};

fn make_eph(prn: u8) -> GpsEphemeris {
    GpsEphemeris {
        sat: SatId {
            constellation: Constellation::Gps,
            prn,
        },
        iodc: 0,
        iode: 0,
        week: 0,
        toe_s: 0.0,
        toc_s: 0.0,
        sqrt_a: 5153.7954775,
        e: 0.01,
        i0: 0.94,
        idot: 0.0,
        omega0: 0.0,
        omegadot: 0.0,
        w: 0.0,
        m0: 0.0,
        delta_n: 0.0,
        cuc: 0.0,
        cus: 0.0,
        crc: 0.0,
        crs: 0.0,
        cic: 0.0,
        cis: 0.0,
        af0: 0.0,
        af1: 0.0,
        af2: 0.0,
        tgd: 0.0,
    }
}

fn make_obs(epoch_idx: u64, t_rx_s: f64, prn: u8) -> ObsEpoch {
    let sat = SatId {
        constellation: Constellation::Gps,
        prn,
    };
    let spec = signal_spec_gps_l1_ca();
    ObsEpoch {
        t_rx_s,
        gps_week: None,
        tow_s: None,
        epoch_idx,
        discontinuity: false,
        role: ReceiverRole::Rover,
        sats: vec![ObsSatellite {
            signal_id: SigId {
                sat,
                band: SignalBand::L1,
                code: SignalCode::Ca,
            },
            pseudorange_m: 20_000_000.0 + prn as f64,
            pseudorange_var_m2: 4.0,
            carrier_phase_cycles: 1_000.0 + prn as f64,
            carrier_phase_var_cycles2: 0.01,
            doppler_hz: -500.0,
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
                signal: spec,
            },
        }],
    }
}

fn make_obs_with_slips(epoch_idx: u64, t_rx_s: f64, prns: &[u8]) -> ObsEpoch {
    let sats = prns
        .iter()
        .map(|&prn| {
            let sat = SatId {
                constellation: Constellation::Gps,
                prn,
            };
            let spec = signal_spec_gps_l1_ca();
            ObsSatellite {
                signal_id: SigId {
                    sat,
                    band: SignalBand::L1,
                    code: SignalCode::Ca,
                },
                pseudorange_m: 20_000_000.0 + prn as f64,
                pseudorange_var_m2: 4.0,
                carrier_phase_cycles: 1_000.0 + prn as f64,
                carrier_phase_var_cycles2: 0.01,
                doppler_hz: -500.0,
                doppler_var_hz2: 4.0,
                cn0_dbhz: 45.0,
                lock_flags: LockFlags {
                    code_lock: true,
                    carrier_lock: true,
                    bit_lock: false,
                    cycle_slip: true,
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
                    signal: spec,
                },
            }
        })
        .collect();
    ObsEpoch {
        t_rx_s,
        gps_week: None,
        tow_s: None,
        epoch_idx,
        discontinuity: false,
        role: ReceiverRole::Rover,
        sats,
    }
}

#[test]
fn ppp_resets_on_gap() {
    let cfg = PppConfig {
        reset_gap_s: 0.01,
        ..PppConfig::default()
    };
    let mut ppp = PppFilter::new(cfg);
    let ephs = vec![make_eph(1), make_eph(2), make_eph(3), make_eph(4)];
    let products = BroadcastProductsProvider::new(ephs.clone());
    let epoch1 = make_obs(1, 0.0, 1);
    let _ = ppp.solve_epoch(&epoch1, &ephs, &products);
    let epoch2 = make_obs(2, 1.0, 1);
    let _ = ppp.solve_epoch(&epoch2, &ephs, &products);
    assert_eq!(ppp.health.last_reset_reason.as_deref(), Some("epoch_gap"));
}

#[test]
fn ppp_handles_missing_products() {
    struct EmptyProducts;
    impl bijux_gnss_nav::ProductsProvider for EmptyProducts {
        fn sat_state(
            &self,
            _sat: SatId,
            _t_s: f64,
            _diag: &mut bijux_gnss_nav::ProductDiagnostics,
        ) -> Option<bijux_gnss_nav::GpsSatState> {
            None
        }

        fn clock_bias_s(
            &self,
            _sat: SatId,
            _t_s: f64,
            _diag: &mut bijux_gnss_nav::ProductDiagnostics,
        ) -> Option<f64> {
            None
        }

        fn coverage_s(&self, _sat: SatId) -> Option<(f64, f64)> {
            None
        }
    }
    let mut ppp = PppFilter::new(PppConfig::default());
    let ephs = vec![make_eph(1), make_eph(2), make_eph(3), make_eph(4)];
    let epoch = make_obs(1, 0.0, 1);
    let sol = ppp.solve_epoch(&epoch, &ephs, &EmptyProducts);
    assert!(sol.is_none());
}

#[test]
fn ppp_resets_on_mass_slip() {
    let mut ppp = PppFilter::new(PppConfig::default());
    let ephs = vec![make_eph(1), make_eph(2), make_eph(3), make_eph(4)];
    let products = BroadcastProductsProvider::new(ephs.clone());
    let epoch = make_obs_with_slips(1, 0.0, &[1, 2, 3, 4]);
    let _ = ppp.solve_epoch(&epoch, &ephs, &products);
    assert_eq!(ppp.health.last_reset_reason.as_deref(), Some("mass_slip"));
}

#[test]
fn ppp_iono_storm_triggers_residual_gate() {
    let cfg = PppConfig {
        residual_gate_m: 1.0,
        ..PppConfig::default()
    };
    let mut ppp = PppFilter::new(cfg);
    let ephs = vec![make_eph(1), make_eph(2), make_eph(3), make_eph(4)];
    let products = BroadcastProductsProvider::new(ephs.clone());
    let mut epoch = make_obs(1, 0.0, 1);
    epoch.sats.push(make_obs(1, 0.0, 2).sats[0].clone());
    epoch.sats.push(make_obs(1, 0.0, 3).sats[0].clone());
    epoch.sats.push(make_obs(1, 0.0, 4).sats[0].clone());
    for sat in &mut epoch.sats {
        sat.pseudorange_m += 50_000.0;
    }
    let sol = ppp.solve_epoch(&epoch, &ephs, &products);
    assert!(sol.is_none());
}
