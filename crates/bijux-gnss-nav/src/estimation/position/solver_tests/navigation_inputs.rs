use super::*;

fn sample_ephemeris(sat: SatId, toe_s: f64, toc_s: f64) -> GpsEphemeris {
    GpsEphemeris {
        sat,
        iodc: 0,
        iode: 0,
        week: 1573,
        sv_health: 0,
        sv_accuracy: Some(2),
        toe_s,
        toc_s,
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

fn sample_galileo_navigation(sat: SatId, toe_s: f64, t0c_s: f64) -> GalileoBroadcastNavigationData {
    GalileoBroadcastNavigationData {
        sat,
        iodnav: 0x01,
        gst: GalileoSystemTime { week: 2222, tow_s: toe_s as u32 },
        sisa_e1_e5b: 0,
        signal_health: GalileoSignalHealth {
            e5b_signal_health: 0,
            e1b_signal_health: 0,
            e5b_data_valid: true,
            e1b_data_valid: true,
        },
        clock: GalileoClockCorrection {
            t0c_s,
            af0: 0.0,
            af1: 0.0,
            af2: 0.0,
            bgd_e1_e5a_s: 0.0,
            bgd_e1_e5b_s: 0.0,
        },
        ephemeris: GalileoEphemeris {
            sat,
            iodnav: 0x01,
            toe_s,
            sqrt_a: 5_440.612_319,
            e: 0.001_23,
            i0: 0.953,
            idot: -2.1e-10,
            omega0: 1.17,
            omegadot: -5.8e-9,
            w: -0.37,
            m0: 0.84,
            delta_n: 4.7e-9,
            cuc: -3.2e-6,
            cus: 4.1e-6,
            crc: 178.0,
            crs: -91.0,
            cic: 1.9e-7,
            cis: -2.4e-7,
        },
        ionosphere: GalileoIonosphericCorrection {
            ai0: 0.0,
            ai1: 0.0,
            ai2: 0.0,
            disturbance_flags: GalileoIonosphericDisturbanceFlags {
                region_1: false,
                region_2: false,
                region_3: false,
                region_4: false,
                region_5: false,
            },
        },
    }
}

fn sample_glonass_navigation(
    sat: SatId,
    ephemeris_reference_time_s: u32,
    gps_minus_glonass_s: f64,
) -> GlonassBroadcastNavigationFrame {
    GlonassBroadcastNavigationFrame {
        sat,
        immediate: GlonassImmediateNavigationData {
            sat,
            frame_time: GlonassFrameTime { hour: 23, minute: 18, half_minute: false },
            ephemeris_reference_time_s,
            tb_update_interval_min: 30,
            tb_is_odd: Some(true),
            state_vector: GlonassStateVector {
                x_m: -7_557_760.253_906_25,
                y_m: -23_962_225.585_937_5,
                z_m: -4_337_567.871_093_75,
                vx_mps: 101.318_359_375,
                vy_mps: 602.112_770_080_566_4,
                vz_mps: -3_495.733_261_108_398_4,
                ax_mps2: -3.725_290_298_461_914e-6,
                ay_mps2: 0.0,
                az_mps2: 1.862_645_149_230_957e-6,
            },
            relative_frequency_bias: 0.0,
            clock_bias_s: -2.572_406_083_345_413_2e-5,
            l2_l1_delay_s: Some(5.587_935_448e-9),
            health: GlonassImmediateHealth { line_unhealthy: false, status_code: 0 },
            immediate_data_age_days: 28,
            satellite_type: GlonassSatelliteType::GlonassM,
            reported_slot: None,
            system_time: Some(GlonassSystemTime { day_number: 864, four_year_interval: Some(8) }),
            resolved_day_index: None,
            accuracy_code: Some(2),
        },
        system_time: Some(GlonassAlmanacTimeData {
            system_time: GlonassSystemTime { day_number: 864, four_year_interval: Some(8) },
            utc_offset_s: 0.0,
            gps_minus_glonass_s,
        }),
        almanac_entries: Vec::new(),
    }
}

fn sample_beidou_navigation(sat: SatId, toe_s: f64, toc_s: f64) -> BeidouBroadcastNavigationData {
    BeidouBroadcastNavigationData {
        sat,
        bdt: BeidouSystemTime { week: 888, sow_s: toe_s as u32 },
        urai: 0,
        signal_health: BeidouSignalHealth { autonomous_satellite_good: true },
        clock: BeidouClockCorrection {
            toc_s,
            aodc: 0,
            af0: 0.0,
            af1: 0.0,
            af2: 0.0,
            tgd1_s: 0.0,
            tgd2_s: 0.0,
        },
        ephemeris: BeidouEphemeris {
            sat,
            aode: 0,
            toe_s,
            sqrt_a: 5_282.61,
            e: 0.0021,
            i0: 0.962,
            idot: -1.4e-10,
            omega0: 0.77,
            omegadot: -7.9e-9,
            w: -0.41,
            m0: 0.53,
            delta_n: 3.4e-9,
            cuc: -1.2e-6,
            cus: 3.1e-6,
            crc: 146.0,
            crs: -84.0,
            cic: 1.1e-7,
            cis: -1.7e-7,
        },
        ionosphere: BeidouIonosphericCorrection {
            alpha0: 0.0,
            alpha1: 0.0,
            alpha2: 0.0,
            alpha3: 0.0,
            beta0: 0.0,
            beta1: 0.0,
            beta2: 0.0,
            beta3: 0.0,
        },
    }
}

#[test]
fn resolve_position_inputs_uses_nearest_valid_ephemeris() {
    let sat = SatId { constellation: Constellation::Gps, prn: 13 };
    let observations = vec![PositionObservation {
        sat,
        pseudorange_m: 24_000_000.0,
        doppler_hz: None,
        doppler_var_hz2: None,
        cn0_dbhz: 45.0,
        elevation_deg: None,
        weight: 1.0,
        gps_receive_time: None,
        signal_timing: None,
        signal_id: None,
    }];
    let ephemerides = vec![
        sample_ephemeris(sat, 100_000.0, 100_000.0),
        sample_ephemeris(sat, 200_000.0, 200_000.0),
    ];
    let navigation = position_broadcast_navigation_from_gps_ephemerides(&ephemerides);
    let mut rejected = Vec::new();

    let inputs = resolve_position_inputs(&observations, &navigation, 200_030.0, &mut rejected);

    assert!(rejected.is_empty());
    assert_eq!(inputs.len(), 1);
    match &inputs[0].navigation {
        PositionBroadcastNavigation::Gps(ephemeris) => {
            assert_eq!(ephemeris.toe_s, 200_000.0);
            assert_eq!(ephemeris.toc_s, 200_000.0);
        }
        PositionBroadcastNavigation::Galileo(_)
        | PositionBroadcastNavigation::Beidou(_)
        | PositionBroadcastNavigation::Glonass(_) => panic!("expected gps navigation"),
    }
}

#[test]
fn position_broadcast_navigation_preserves_satellite_identity() {
    let gps_sat = SatId { constellation: Constellation::Gps, prn: 13 };
    let galileo_sat = SatId { constellation: Constellation::Galileo, prn: 19 };
    let beidou_sat = SatId { constellation: Constellation::Beidou, prn: 11 };
    let glonass_sat = SatId { constellation: Constellation::Glonass, prn: 14 };
    let gps_navigation =
        PositionBroadcastNavigation::Gps(sample_ephemeris(gps_sat, 200_000.0, 200_000.0));
    let galileo_navigation = PositionBroadcastNavigation::Galileo(sample_galileo_navigation(
        galileo_sat,
        64_800.0,
        66_000.0,
    ));
    let beidou_navigation = PositionBroadcastNavigation::Beidou(sample_beidou_navigation(
        beidou_sat, 345_600.0, 345_600.0,
    ));
    let glonass_navigation = PositionBroadcastNavigation::Glonass(sample_glonass_navigation(
        glonass_sat,
        83_700,
        -10_782.0,
    ));

    assert_eq!(gps_navigation.sat(), gps_sat);
    assert_eq!(gps_navigation.constellation(), Constellation::Gps);
    assert_eq!(galileo_navigation.sat(), galileo_sat);
    assert_eq!(galileo_navigation.constellation(), Constellation::Galileo);
    assert_eq!(beidou_navigation.sat(), beidou_sat);
    assert_eq!(beidou_navigation.constellation(), Constellation::Beidou);
    assert_eq!(glonass_navigation.sat(), glonass_sat);
    assert_eq!(glonass_navigation.constellation(), Constellation::Glonass);
}

#[test]
fn gps_ephemerides_convert_into_position_navigation_entries() {
    let ephemerides = vec![
        sample_ephemeris(
            SatId { constellation: Constellation::Gps, prn: 13 },
            100_000.0,
            100_000.0,
        ),
        sample_ephemeris(
            SatId { constellation: Constellation::Gps, prn: 14 },
            200_000.0,
            200_000.0,
        ),
    ];

    let navigation = position_broadcast_navigation_from_gps_ephemerides(&ephemerides);

    assert_eq!(navigation.len(), 2);
    assert_eq!(navigation[0].sat(), ephemerides[0].sat);
    assert_eq!(navigation[1].sat(), ephemerides[1].sat);
}

#[test]
fn galileo_navigation_convert_into_position_navigation_entries() {
    let navigations = vec![
        sample_galileo_navigation(
            SatId { constellation: Constellation::Galileo, prn: 19 },
            504_000.0,
            504_018.0,
        ),
        sample_galileo_navigation(
            SatId { constellation: Constellation::Galileo, prn: 24 },
            504_000.0,
            504_018.0,
        ),
    ];

    let navigation = position_broadcast_navigation_from_galileo_navigations(&navigations);

    assert_eq!(navigation.len(), 2);
    assert_eq!(navigation[0].sat(), navigations[0].sat);
    assert_eq!(navigation[1].sat(), navigations[1].sat);
}

#[test]
fn glonass_frames_convert_into_position_navigation_entries() {
    let navigation_frames = vec![
        sample_glonass_navigation(
            SatId { constellation: Constellation::Glonass, prn: 14 },
            83_700,
            -10_782.0,
        ),
        sample_glonass_navigation(
            SatId { constellation: Constellation::Glonass, prn: 21 },
            84_600,
            -10_782.0,
        ),
    ];

    let navigation = position_broadcast_navigation_from_glonass_frames(&navigation_frames);

    assert_eq!(navigation.len(), 2);
    assert_eq!(navigation[0].sat(), navigation_frames[0].sat);
    assert_eq!(navigation[1].sat(), navigation_frames[1].sat);
}

#[test]
fn beidou_navigation_convert_into_position_navigation_entries() {
    let navigations = vec![
        sample_beidou_navigation(
            SatId { constellation: Constellation::Beidou, prn: 11 },
            345_600.0,
            345_600.0,
        ),
        sample_beidou_navigation(
            SatId { constellation: Constellation::Beidou, prn: 12 },
            346_200.0,
            346_200.0,
        ),
    ];

    let navigation = position_broadcast_navigation_from_beidou_navigations(&navigations);

    assert_eq!(navigation.len(), 2);
    assert_eq!(navigation[0].sat(), navigations[0].sat);
    assert_eq!(navigation[1].sat(), navigations[1].sat);
}

#[test]
fn resolve_position_inputs_accepts_valid_glonass_navigation() {
    let sat = SatId { constellation: Constellation::Glonass, prn: 14 };
    let observations = vec![PositionObservation {
        sat,
        pseudorange_m: 24_000_000.0,
        doppler_hz: None,
        doppler_var_hz2: None,
        cn0_dbhz: 45.0,
        elevation_deg: None,
        weight: 1.0,
        gps_receive_time: None,
        signal_timing: None,
        signal_id: None,
    }];
    let navigation_frames = vec![sample_glonass_navigation(sat, 83_700, -10_782.0)];
    let navigation = position_broadcast_navigation_from_glonass_frames(&navigation_frames);
    let mut rejected = Vec::new();

    let inputs = resolve_position_inputs(&observations, &navigation, 504_918.0, &mut rejected);

    assert!(rejected.is_empty());
    assert_eq!(inputs.len(), 1);
    match &inputs[0].navigation {
        PositionBroadcastNavigation::Glonass(navigation) => {
            assert_eq!(navigation.sat, sat);
            assert_eq!(navigation.immediate.ephemeris_reference_time_s, 83_700);
        }
        PositionBroadcastNavigation::Gps(_)
        | PositionBroadcastNavigation::Galileo(_)
        | PositionBroadcastNavigation::Beidou(_) => panic!("expected glonass navigation"),
    }
}

#[test]
fn resolve_position_inputs_accepts_valid_beidou_navigation() {
    let sat = SatId { constellation: Constellation::Beidou, prn: 11 };
    let observations = vec![PositionObservation {
        sat,
        pseudorange_m: 24_000_000.0,
        doppler_hz: None,
        doppler_var_hz2: None,
        cn0_dbhz: 45.0,
        elevation_deg: None,
        weight: 1.0,
        gps_receive_time: None,
        signal_timing: None,
        signal_id: None,
    }];
    let navigations = vec![sample_beidou_navigation(sat, 345_600.0, 345_600.0)];
    let navigation = position_broadcast_navigation_from_beidou_navigations(&navigations);
    let mut rejected = Vec::new();

    let inputs = resolve_position_inputs(&observations, &navigation, 345_630.0, &mut rejected);

    assert!(rejected.is_empty());
    assert_eq!(inputs.len(), 1);
    match &inputs[0].navigation {
        PositionBroadcastNavigation::Beidou(navigation) => {
            assert_eq!(navigation.sat, sat);
            assert_eq!(navigation.ephemeris.toe_s, 345_600.0);
            assert_eq!(navigation.clock.toc_s, 345_600.0);
        }
        PositionBroadcastNavigation::Gps(_)
        | PositionBroadcastNavigation::Galileo(_)
        | PositionBroadcastNavigation::Glonass(_) => panic!("expected beidou navigation"),
    }
}

#[test]
fn glonass_navigation_without_gps_time_offset_is_not_time_resolved() {
    let sat = SatId { constellation: Constellation::Glonass, prn: 14 };
    let navigation = sample_glonass_navigation(sat, 83_700, -10_782.0);

    assert!(navigation_time_relationship_is_known(&PositionBroadcastNavigation::Glonass(
        navigation.clone(),
    )));

    let mut navigation_without_system_time = navigation;
    navigation_without_system_time.system_time = None;

    assert!(!navigation_time_relationship_is_known(&PositionBroadcastNavigation::Glonass(
        navigation_without_system_time
    )));
}

#[test]
fn mixed_observations_detect_unknown_glonass_time_offset() {
    let gps_sat = SatId { constellation: Constellation::Gps, prn: 13 };
    let glonass_sat = SatId { constellation: Constellation::Glonass, prn: 14 };
    let observations = vec![
        PositionObservation {
            sat: gps_sat,
            pseudorange_m: 24_000_000.0,
            doppler_hz: None,
            doppler_var_hz2: None,
            cn0_dbhz: 45.0,
            elevation_deg: None,
            weight: 1.0,
            gps_receive_time: None,
            signal_timing: None,
            signal_id: None,
        },
        PositionObservation {
            sat: glonass_sat,
            pseudorange_m: 24_100_000.0,
            doppler_hz: None,
            doppler_var_hz2: None,
            cn0_dbhz: 45.0,
            elevation_deg: None,
            weight: 1.0,
            gps_receive_time: None,
            signal_timing: None,
            signal_id: None,
        },
    ];
    let mut glonass_navigation = sample_glonass_navigation(glonass_sat, 83_700, -10_782.0);
    glonass_navigation.system_time = None;
    let navigation = vec![
        PositionBroadcastNavigation::Gps(sample_ephemeris(gps_sat, 200_000.0, 200_000.0)),
        PositionBroadcastNavigation::Glonass(glonass_navigation),
    ];

    let unknown = unknown_inter_system_time_offset_sats(&observations, &navigation);

    assert_eq!(unknown, vec![glonass_sat]);
}
