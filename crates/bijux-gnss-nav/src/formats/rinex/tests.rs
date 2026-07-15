use super::{
    format_rinex_nav_float, format_rinex_navigation_dataset, parse_rinex_broadcast_navigation,
    parse_rinex_epoch_utc, parse_rinex_epoch_utc_civil, parse_rinex_float, parse_rinex_nav,
    parse_rinex_nav_header, parse_rinex_navigation_dataset, parse_rinex_numeric_fields,
    write_rinex_broadcast_navigation, write_rinex_nav, write_rinex_obs,
};
use crate::formats::rinex_obs::{parse_rinex_observation_dataset, RinexObservationRecord};
use crate::models::atmosphere::KlobucharCoefficients;
use crate::orbits::beidou::BeidouSystemTime;
use crate::orbits::galileo::GalileoSystemTime;
use crate::orbits::glonass::GlonassSystemTime;
use crate::orbits::gps::{GpsBroadcastNavigationData, GpsEphemeris};
use bijux_gnss_core::api::{
    Constellation, Cycles, Hertz, LockFlags, Meters, ObsEpoch, ObsMetadata, ObsSatellite,
    ObservationStatus, ReceiverRole, SatId, Seconds, SigId, SignalBand, SignalCode,
};

fn sample_ephemeris() -> GpsEphemeris {
    GpsEphemeris {
        sat: SatId { constellation: Constellation::Gps, prn: 8 },
        iodc: 97,
        iode: 11,
        week: 2209,
        sv_health: 0,
        sv_accuracy: Some(2),
        toe_s: 345_600.0,
        toc_s: 504_018.0,
        sqrt_a: 5_153.795_477_5,
        e: 1.234_567_890_123e-2,
        i0: 9.4e-1,
        idot: 7.8e-10,
        omega0: 1.5,
        omegadot: -8.9e-9,
        w: 2.1e-1,
        m0: 6.0e-1,
        delta_n: 4.5e-9,
        cuc: 1.2e-6,
        cus: 2.3e-6,
        crc: 321.0,
        crs: 25.0,
        cic: 4.5e-8,
        cis: 5.6e-8,
        af0: -1.234_567_890_123e-4,
        af1: 2.345_678_901_234e-12,
        af2: 0.0,
        tgd: -1.9e-8,
    }
}

fn rinex_obs_output_path(name: &str) -> std::path::PathBuf {
    let output_dir =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../artifacts/rust-test/rinex");
    std::fs::create_dir_all(&output_dir).expect("create RINEX test artifact directory");
    output_dir.join(format!("{name}-{}.rnx", std::process::id()))
}

fn observation(
    constellation: Constellation,
    prn: u8,
    band: SignalBand,
    code: SignalCode,
    pseudorange_m: f64,
    carrier_phase_cycles: f64,
    doppler_hz: f64,
    cn0_dbhz: f64,
    cycle_slip: bool,
) -> ObsSatellite {
    ObsSatellite {
        signal_id: SigId { sat: SatId { constellation, prn }, band, code },
        pseudorange_m: Meters(pseudorange_m),
        pseudorange_var_m2: 1.0,
        carrier_phase_cycles: Cycles(carrier_phase_cycles),
        carrier_phase_var_cycles2: 0.01,
        doppler_hz: Hertz(doppler_hz),
        doppler_var_hz2: 1.0,
        cn0_dbhz,
        lock_flags: LockFlags { code_lock: true, carrier_lock: true, bit_lock: false, cycle_slip },
        multipath_suspect: false,
        observation_status: ObservationStatus::Accepted,
        observation_reject_reasons: Vec::new(),
        elevation_deg: None,
        azimuth_deg: None,
        weight: None,
        timing: None,
        error_model: None,
        metadata: ObsMetadata::default(),
    }
}

#[test]
fn write_rinex_obs_emits_supported_measurements() {
    let epoch = ObsEpoch {
        t_rx_s: Seconds(0.0),
        source_time: Default::default(),
        gps_week: Some(2209),
        tow_s: Some(Seconds(504_000.0)),
        epoch_idx: 0,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        role: ReceiverRole::Rover,
        sats: vec![
            observation(
                Constellation::Gps,
                1,
                SignalBand::L1,
                SignalCode::Ca,
                20_000_000.125,
                100_000.250,
                -500.125,
                45.5,
                true,
            ),
            observation(
                Constellation::Gps,
                1,
                SignalBand::L2,
                SignalCode::L2C,
                21_000_000.875,
                110_000.500,
                -400.500,
                44.25,
                false,
            ),
            observation(
                Constellation::Galileo,
                11,
                SignalBand::E1,
                SignalCode::E1B,
                24_000_000.000,
                200_000.000,
                -250.000,
                47.0,
                false,
            ),
        ],
        decision: Default::default(),
        decision_reason: None,
        manifest: None,
    };
    let output_path = rinex_obs_output_path("write-supported-observations");

    write_rinex_obs(&output_path, &[epoch], true).expect("write RINEX observation file");
    let encoded = std::fs::read_to_string(&output_path).expect("read RINEX observation file");
    let dataset =
        parse_rinex_observation_dataset(&encoded).expect("parse written RINEX observations");

    assert_eq!(
        dataset.observation_types_by_system[&'G'],
        ["C1C", "L1C", "D1C", "S1C", "C2L", "L2L", "D2L", "S2L"]
    );
    assert_eq!(dataset.observation_types_by_system[&'E'], ["C1C", "L1C", "D1C", "S1C"]);
    let RinexObservationRecord::Epoch(parsed_epoch) = &dataset.records[0] else {
        panic!("expected observation epoch");
    };
    let gps = parsed_epoch
        .satellites
        .iter()
        .find(|satellite| satellite.system == 'G' && satellite.satellite.prn == 1)
        .expect("GPS satellite record");
    assert_eq!(gps.observations[0].value, Some(20_000_000.125));
    assert_eq!(gps.observations[1].value, Some(100_000.250));
    assert_eq!(gps.observations[1].loss_of_lock_indicator, Some(1));
    assert_eq!(gps.observations[4].value, Some(21_000_000.875));
    assert_eq!(gps.observations[7].value, Some(44.250));
}

#[test]
fn rinex_nav_header_reports_version_and_mixed_flag() {
    let data = "\
     3.04           NAVIGATION DATA     M (MIXED)           RINEX VERSION / TYPE
bijux-gnss                              PGM / RUN BY / DATE
                                                            END OF HEADER
";

    let (header, line_count) = parse_rinex_nav_header(data).expect("valid nav header");

    assert_eq!(header.version, 3.04);
    assert!(header.is_mixed);
    assert_eq!(header.klobuchar, None);
    assert_eq!(line_count, 3);
}

#[test]
fn parse_rinex_nav_header_reads_rinex_2_klobuchar_coefficients() {
    let data = "\
     2.11           NAVIGATION DATA     G (GPS)             RINEX VERSION / TYPE
bijux-gnss                              PGM / RUN BY / DATE
 1.2120D-08 1.4900D-08-5.9600D-08 1.1920D-07          ION ALPHA
 1.1670D+05-2.2940D+05-1.3110D+05 1.0490D+06          ION BETA
                                                            END OF HEADER
";

    let (header, _) = parse_rinex_nav_header(data).expect("RINEX 2 header");

    assert_eq!(
        header.klobuchar,
        Some(KlobucharCoefficients::new(
            [0.1212e-7, 0.1490e-7, -0.5960e-7, 0.1192e-6],
            [0.1167e6, -0.2294e6, -0.1311e6, 0.1049e7],
        ))
    );
}

#[test]
fn parse_rinex_nav_header_reads_rinex_3_klobuchar_coefficients() {
    let data = "\
     3.04           NAVIGATION DATA     G (GPS)             RINEX VERSION / TYPE
bijux-gnss                              PGM / RUN BY / DATE
GPSA  1.2120D-08 1.4900D-08-5.9600D-08 1.1920D-07       IONOSPHERIC CORR
GPSB  1.1670D+05-2.2940D+05-1.3110D+05 1.0490D+06       IONOSPHERIC CORR
                                                            END OF HEADER
";

    let (header, _) = parse_rinex_nav_header(data).expect("RINEX 3 header");

    assert_eq!(
        header.klobuchar,
        Some(KlobucharCoefficients::new(
            [0.1212e-7, 0.1490e-7, -0.5960e-7, 0.1192e-6],
            [0.1167e6, -0.2294e6, -0.1311e6, 0.1049e7],
        ))
    );
}

#[test]
fn parse_rinex_nav_header_reads_time_system_corrections() {
    let data = "\
     3.05           NAVIGATION DATA     M (MIXED)           RINEX VERSION / TYPE
bijux-gnss                              PGM / RUN BY / DATE
GAUT -1.8626451492D-09-8.881784197D-16 432000 2209 GAL UTC TIME SYSTEM CORR
BDUT  4.6566128731D-10 0.000000000D+00 345600 0850 BDS UTC TIME SYSTEM CORR
                                                            END OF HEADER
";

    let (header, _) = parse_rinex_nav_header(data).expect("RINEX NAV header");

    assert_eq!(header.version, 3.05);
    assert!(header.is_mixed);
    assert_eq!(header.time_system_corrections.len(), 2);
    assert_eq!(header.time_system_corrections[0].code, "GAUT");
    assert!((header.time_system_corrections[0].a0_s + 1.862_645_149_2e-9).abs() < 1.0e-20);
    assert_eq!(header.time_system_corrections[0].reference_time_s, 432_000);
    assert_eq!(header.time_system_corrections[0].reference_week, 2209);
    assert_eq!(header.time_system_corrections[0].provider.as_deref(), Some("GAL"));
    assert_eq!(header.time_system_corrections[0].utc_id.as_deref(), Some("UTC"));
    assert_eq!(header.time_system_corrections[1].code, "BDUT");
}

#[test]
fn rinex_nav_float_parses_fortran_exponents() {
    let value = parse_rinex_float("-1.981128007174D-04").expect("D exponent float");

    assert!((value + 1.981_128_007_174e-4).abs() < 1.0e-16);
}

#[test]
fn rinex_nav_float_formats_fortran_exponents() {
    let field = format_rinex_nav_float(-1.981_128_007_174e-4);

    assert_eq!(field.len(), 19);
    assert!(field.contains('D'));
    assert!(!field.contains('E'));
}

#[test]
fn rinex_nav_numeric_fields_extract_adjacent_values() {
    let fields = parse_rinex_numeric_fields(
        "    1.700000000000D+01-1.234567890123D-04 2.345678901234D-12 0.000000000000D+00",
    )
    .expect("numeric fields");

    assert_eq!(fields.len(), 4);
    assert_eq!(fields[0], 17.0);
    assert!((fields[1] + 1.234_567_890_123e-4).abs() < 1.0e-16);
    assert!((fields[2] - 2.345_678_901_234e-12).abs() < 1.0e-24);
    assert_eq!(fields[3], 0.0);
}

#[test]
fn rinex_nav_epoch_conversion_preserves_utc_in_gps_time() {
    let utc = parse_rinex_epoch_utc(2022, 5, 13, 20, 0, 0.0).expect("utc epoch");
    let gps = crate::time::gps_time_from_utc(utc);

    assert_eq!(gps.week, 2209);
    assert!((gps.tow_s - 504_018.0).abs() < 1.0e-9);
}

#[test]
fn rinex_nav_epoch_conversion_preserves_declared_leap_second_in_gps_time() {
    let civil = parse_rinex_epoch_utc_civil(2016, 12, 31, 23, 59, 60.5).expect("leap epoch");
    let gps = crate::time::utc_civil_to_gps_with_offset(
        civil,
        &bijux_gnss_core::api::LeapSeconds::default_table(),
    )
    .expect("gps conversion")
    .time;

    assert_eq!(civil.format_utc(), "2016-12-31T23:59:60.5Z");
    assert_eq!(gps.week, 1930);
    assert!((gps.tow_s - 17.5).abs() < 1.0e-9);
    assert!(parse_rinex_epoch_utc(2016, 12, 31, 23, 59, 60.0).is_err());
}

#[test]
fn parse_rinex_nav_reads_gps_rinex_3_ephemeris() {
    let data = "\
     3.04           NAVIGATION DATA     G (GPS)             RINEX VERSION / TYPE
bijux-gnss                              PGM / RUN BY / DATE
                                                            END OF HEADER
G01 2022 05 13 20 00 00-1.234567890123D-04 2.345678901234D-12 0.000000000000D+00
    1.100000000000D+01 2.500000000000D+01 4.500000000000D-09 6.000000000000D-01
    1.200000000000D-06 1.234567890123D-02 2.300000000000D-06 5.153795477500D+03
    3.456000000000D+05 4.500000000000D-08 1.500000000000D+00 5.600000000000D-08
    9.400000000000D-01 3.210000000000D+02 2.100000000000D-01-8.900000000000D-09
    7.800000000000D-10 0.000000000000D+00 2.209000000000D+03 0.000000000000D+00
    2.400000000000D+00 0.000000000000D+00-1.900000000000D-08 9.700000000000D+01
    0.000000000000D+00 4.000000000000D+00 0.000000000000D+00 0.000000000000D+00
";

    let ephs = parse_rinex_nav(data).expect("RINEX NAV parse");

    assert_eq!(ephs.len(), 1);
    let eph = &ephs[0];
    assert_eq!(eph.sat, SatId { constellation: Constellation::Gps, prn: 1 });
    assert_eq!(eph.week, 2209);
    assert_eq!(eph.toe_s, 345_600.0);
    assert!((eph.toc_s - 504_018.0).abs() < 1.0e-9);
    assert_eq!(eph.iode, 11);
    assert_eq!(eph.iodc, 97);
    assert!((eph.af0 + 1.234_567_890_123e-4).abs() < 1.0e-16);
    assert!((eph.af1 - 2.345_678_901_234e-12).abs() < 1.0e-24);
    assert!((eph.sqrt_a - 5_153.795_477_5).abs() < 1.0e-12);
    assert!((eph.e - 1.234_567_890_123e-2).abs() < 1.0e-15);
    assert!((eph.delta_n - 4.5e-9).abs() < 1.0e-18);
    assert!((eph.tgd + 1.9e-8).abs() < 1.0e-18);
}

#[test]
fn parse_rinex_navigation_dataset_preserves_gps_and_header_corrections() {
    let data = "\
     3.05           NAVIGATION DATA     M (MIXED)           RINEX VERSION / TYPE
bijux-gnss                              PGM / RUN BY / DATE
GPGA  1.0000000000D-09 2.000000000D-15 345600 2209 GPS GAL TIME SYSTEM CORR
                                                            END OF HEADER
G01 2022 05 13 20 00 00-1.234567890123D-04 2.345678901234D-12 0.000000000000D+00
    1.100000000000D+01 2.500000000000D+01 4.500000000000D-09 6.000000000000D-01
    1.200000000000D-06 1.234567890123D-02 2.300000000000D-06 5.153795477500D+03
    3.456000000000D+05 4.500000000000D-08 1.500000000000D+00 5.600000000000D-08
    9.400000000000D-01 3.210000000000D+02 2.100000000000D-01-8.900000000000D-09
    7.800000000000D-10 0.000000000000D+00 2.209000000000D+03 0.000000000000D+00
    2.400000000000D+00 0.000000000000D+00-1.900000000000D-08 9.700000000000D+01
    0.000000000000D+00 4.000000000000D+00 0.000000000000D+00 0.000000000000D+00
";

    let dataset = parse_rinex_navigation_dataset(data).expect("parse mixed navigation dataset");

    assert_eq!(dataset.version, 3.05);
    assert_eq!(dataset.gps.len(), 1);
    assert!(dataset.galileo.is_empty());
    assert!(dataset.beidou.is_empty());
    assert!(dataset.glonass.is_empty());
    assert_eq!(dataset.time_system_corrections.len(), 1);
    assert_eq!(dataset.time_system_corrections[0].code, "GPGA");
    assert_eq!(dataset.time_system_corrections[0].provider.as_deref(), Some("GPS"));
    assert_eq!(dataset.time_system_corrections[0].utc_id.as_deref(), Some("GAL"));
}

#[test]
fn parse_rinex_navigation_dataset_reads_galileo_records() {
    let data = "\
     3.05           NAVIGATION DATA     M (MIXED)           RINEX VERSION / TYPE
bijux-gnss                              PGM / RUN BY / DATE
                                                            END OF HEADER
E19 1980 01 06 18 20 00-1.700000000000D-04 2.500000000000D-12-3.000000000000D-19
    4.210000000000D+02-9.100000000000D+01 4.700000000000D-09 8.400000000000D-01
   -3.200000000000D-06 1.230000000000D-03 4.100000000000D-06 5.440612319000D+03
    6.480000000000D+04 1.900000000000D-07 1.170000000000D+00-2.400000000000D-07
    9.530000000000D-01 1.780000000000D+02-3.700000000000D-01-5.800000000000D-09
   -2.100000000000D-10 5.000000000000D+00 0.000000000000D+00 7.700000000000D+01
    3.000000000000D+00-1.100000000000D-09 2.400000000000D-09 6.500000000000D+04
    0.000000000000D+00 0.000000000000D+00 0.000000000000D+00 0.000000000000D+00
";

    let dataset = parse_rinex_navigation_dataset(data).expect("parse Galileo navigation");

    assert!(dataset.gps.is_empty());
    assert_eq!(dataset.galileo.len(), 1);
    let galileo = &dataset.galileo[0];
    assert_eq!(galileo.sat, SatId { constellation: Constellation::Galileo, prn: 19 });
    assert_eq!(galileo.iodnav, 421);
    assert_eq!(galileo.gst, GalileoSystemTime { week: 0, tow_s: 65_000 });
    assert_eq!(galileo.sisa_e1_e5b, 77);
    assert_eq!(galileo.signal_health.e5b_signal_health, 3);
    assert!((galileo.clock.t0c_s - 66_000.0).abs() < 1.0e-9);
    assert!((galileo.clock.af0 + 1.7e-4).abs() < 1.0e-16);
    assert!((galileo.clock.bgd_e1_e5a_s + 1.1e-9).abs() < 1.0e-20);
    assert!((galileo.ephemeris.sqrt_a - 5_440.612_319).abs() < 1.0e-12);
    assert!((galileo.ephemeris.toe_s - 64_800.0).abs() < 1.0e-9);
}

#[test]
fn parse_rinex_navigation_dataset_reads_beidou_records() {
    let data = "\
     3.05           NAVIGATION DATA     M (MIXED)           RINEX VERSION / TYPE
bijux-gnss                              PGM / RUN BY / DATE
                                                            END OF HEADER
C11 2006 01 01 18 00 00-1.800000000000D-04 2.200000000000D-12-2.600000000000D-19
    3.000000000000D+00-8.200000000000D+01 4.300000000000D-09 1.120000000000D+00
   -2.300000000000D-06 2.340000000000D-03 3.100000000000D-06 5.282625128000D+03
    6.480000000000D+04 2.600000000000D-07 8.700000000000D-01-2.200000000000D-07
    9.580000000000D-01 1.450000000000D+02-4.200000000000D-01-6.200000000000D-09
   -1.900000000000D-10 0.000000000000D+00 0.000000000000D+00 0.000000000000D+00
    2.000000000000D+00 0.000000000000D+00-1.100000000000D-09 2.200000000000D-09
    6.470000000000D+04 3.000000000000D+00 0.000000000000D+00 0.000000000000D+00
";

    let dataset = parse_rinex_navigation_dataset(data).expect("parse BeiDou navigation");

    assert!(dataset.gps.is_empty());
    assert!(dataset.galileo.is_empty());
    assert_eq!(dataset.beidou.len(), 1);
    let beidou = &dataset.beidou[0];
    assert_eq!(beidou.sat, SatId { constellation: Constellation::Beidou, prn: 11 });
    assert_eq!(beidou.bdt, BeidouSystemTime { week: 0, sow_s: 64_700 });
    assert_eq!(beidou.urai, 2);
    assert!(beidou.signal_health.autonomous_satellite_good);
    assert_eq!(beidou.clock.aodc, 3);
    assert!((beidou.clock.toc_s - 64_800.0).abs() < 1.0e-9);
    assert!((beidou.clock.tgd1_s + 1.1e-9).abs() < 1.0e-20);
    assert_eq!(beidou.ephemeris.aode, 3);
    assert!((beidou.ephemeris.sqrt_a - 5_282.625_128).abs() < 1.0e-12);
    assert!((beidou.ephemeris.toe_s - 64_800.0).abs() < 1.0e-9);
}

#[test]
fn parse_rinex_navigation_dataset_reads_glonass_records() {
    let data = "\
     3.05           NAVIGATION DATA     M (MIXED)           RINEX VERSION / TYPE
bijux-gnss                              PGM / RUN BY / DATE
                                                            END OF HEADER
R07 2022 05 13 23 15 30-2.572406083345D-05 8.000000000000D-13 8.370000000000D+04
   -7.557760253906D+03 1.013183593750D-01-3.725290298462D-09 0.000000000000D+00
   -2.396222558594D+04 6.021127700806D-01 0.000000000000D+00-4.000000000000D+00
   -4.337567871094D+03-3.495733261108D+00 1.862645149231D-09 2.800000000000D+01
";

    let dataset = parse_rinex_navigation_dataset(data).expect("parse GLONASS navigation");

    assert!(dataset.gps.is_empty());
    assert!(dataset.galileo.is_empty());
    assert!(dataset.beidou.is_empty());
    assert_eq!(dataset.glonass.len(), 1);
    let frame = &dataset.glonass[0];
    assert_eq!(frame.sat, SatId { constellation: Constellation::Glonass, prn: 7 });
    assert_eq!(frame.immediate.reported_slot.expect("reported slot").value(), 7);
    assert_eq!(frame.immediate.frame_time.hour, 23);
    assert_eq!(frame.immediate.frame_time.minute, 15);
    assert!(frame.immediate.frame_time.half_minute);
    assert_eq!(frame.immediate.ephemeris_reference_time_s, 83_700);
    assert_eq!(
        frame.immediate.system_time,
        Some(GlonassSystemTime { day_number: 133, four_year_interval: None })
    );
    assert!((frame.immediate.clock_bias_s - 2.572_406_083_345e-5).abs() < 1.0e-17);
    assert!((frame.immediate.relative_frequency_bias - 8.0e-13).abs() < 1.0e-24);
    assert!((frame.immediate.state_vector.x_m + 7_557_760.253_906).abs() < 1.0e-6);
    assert!((frame.immediate.state_vector.vy_mps - 602.112_770_080_6).abs() < 1.0e-9);
    assert!((frame.immediate.state_vector.az_mps2 - 1.862_645_149_231e-6).abs() < 1.0e-18);
    assert!(!frame.immediate.health.line_unhealthy);
    assert_eq!(frame.immediate.health.status_code, 0);
    assert_eq!(frame.immediate.immediate_data_age_days, 28);
}

#[test]
fn format_rinex_navigation_dataset_round_trips_mixed_records() {
    let data = "\
     3.05           NAVIGATION DATA     M (MIXED)           RINEX VERSION / TYPE
bijux-gnss                              PGM / RUN BY / DATE
GPGA  1.0000000000D-09 2.000000000D-15 345600 2209 GPS GAL TIME SYSTEM CORR
                                                            END OF HEADER
G01 2022 05 13 20 00 00-1.234567890123D-04 2.345678901234D-12 0.000000000000D+00
    1.100000000000D+01 2.500000000000D+01 4.500000000000D-09 6.000000000000D-01
    1.200000000000D-06 1.234567890123D-02 2.300000000000D-06 5.153795477500D+03
    3.456000000000D+05 4.500000000000D-08 1.500000000000D+00 5.600000000000D-08
    9.400000000000D-01 3.210000000000D+02 2.100000000000D-01-8.900000000000D-09
    7.800000000000D-10 0.000000000000D+00 2.209000000000D+03 0.000000000000D+00
    2.400000000000D+00 0.000000000000D+00-1.900000000000D-08 9.700000000000D+01
    0.000000000000D+00 4.000000000000D+00 0.000000000000D+00 0.000000000000D+00
E19 1980 01 06 18 20 00-1.700000000000D-04 2.500000000000D-12-3.000000000000D-19
    4.210000000000D+02-9.100000000000D+01 4.700000000000D-09 8.400000000000D-01
   -3.200000000000D-06 1.230000000000D-03 4.100000000000D-06 5.440612319000D+03
    6.480000000000D+04 1.900000000000D-07 1.170000000000D+00-2.400000000000D-07
    9.530000000000D-01 1.780000000000D+02-3.700000000000D-01-5.800000000000D-09
   -2.100000000000D-10 5.000000000000D+00 0.000000000000D+00 7.700000000000D+01
    3.000000000000D+00-1.100000000000D-09 2.400000000000D-09 6.500000000000D+04
    0.000000000000D+00 0.000000000000D+00 0.000000000000D+00 0.000000000000D+00
C11 2006 01 01 18 00 00-1.800000000000D-04 2.200000000000D-12-2.600000000000D-19
    3.000000000000D+00-8.200000000000D+01 4.300000000000D-09 1.120000000000D+00
   -2.300000000000D-06 2.340000000000D-03 3.100000000000D-06 5.282625128000D+03
    6.480000000000D+04 2.600000000000D-07 8.700000000000D-01-2.200000000000D-07
    9.580000000000D-01 1.450000000000D+02-4.200000000000D-01-6.200000000000D-09
   -1.900000000000D-10 0.000000000000D+00 0.000000000000D+00 0.000000000000D+00
    2.000000000000D+00 0.000000000000D+00-1.100000000000D-09 2.200000000000D-09
    6.470000000000D+04 3.000000000000D+00 0.000000000000D+00 0.000000000000D+00
R07 2022 05 13 23 15 30-2.572406083345D-05 8.000000000000D-13 8.370000000000D+04
   -7.557760253906D+03 1.013183593750D-01-3.725290298462D-09 0.000000000000D+00
   -2.396222558594D+04 6.021127700806D-01 0.000000000000D+00-4.000000000000D+00
   -4.337567871094D+03-3.495733261108D+00 1.862645149231D-09 2.800000000000D+01
";

    let dataset = parse_rinex_navigation_dataset(data).expect("parse mixed navigation");
    let encoded = format_rinex_navigation_dataset(&dataset).expect("format mixed navigation");
    let parsed =
        parse_rinex_navigation_dataset(&encoded).expect("parse formatted mixed navigation");

    assert_eq!(parsed.version, 3.05);
    assert_eq!(parsed.time_system_corrections, dataset.time_system_corrections);
    assert_eq!(parsed.gps.len(), 1);
    assert_eq!(parsed.galileo.len(), 1);
    assert_eq!(parsed.beidou.len(), 1);
    assert_eq!(parsed.glonass.len(), 1);
    assert_eq!(parsed.gps[0].sat, dataset.gps[0].sat);
    assert_eq!(parsed.gps[0].week, dataset.gps[0].week);
    assert!((parsed.gps[0].toc_s - dataset.gps[0].toc_s).abs() < 1.0e-6);
    assert_eq!(parsed.galileo[0].sat, dataset.galileo[0].sat);
    assert_eq!(parsed.galileo[0].iodnav, dataset.galileo[0].iodnav);
    assert!(
        (parsed.galileo[0].ephemeris.sqrt_a - dataset.galileo[0].ephemeris.sqrt_a).abs() < 1.0e-9
    );
    assert_eq!(parsed.beidou[0].sat, dataset.beidou[0].sat);
    assert_eq!(parsed.beidou[0].bdt, dataset.beidou[0].bdt);
    assert!((parsed.beidou[0].clock.tgd1_s - dataset.beidou[0].clock.tgd1_s).abs() < 1.0e-20);
    assert_eq!(parsed.glonass[0].sat, dataset.glonass[0].sat);
    assert_eq!(
        parsed.glonass[0].immediate.ephemeris_reference_time_s,
        dataset.glonass[0].immediate.ephemeris_reference_time_s
    );
    assert!(
        (parsed.glonass[0].immediate.clock_bias_s - dataset.glonass[0].immediate.clock_bias_s)
            .abs()
            < 1.0e-17
    );
    assert!(
        (parsed.glonass[0].immediate.state_vector.x_m
            - dataset.glonass[0].immediate.state_vector.x_m)
            .abs()
            < 1.0e-6
    );
}

#[test]
fn parse_rinex_nav_reads_gps_epoch_on_inserted_leap_second() {
    let data = "\
     3.04           NAVIGATION DATA     G (GPS)             RINEX VERSION / TYPE
bijux-gnss                              PGM / RUN BY / DATE
                                                            END OF HEADER
G01 2016 12 31 23 59 60-1.234567890123D-04 2.345678901234D-12 0.000000000000D+00
    1.100000000000D+01 2.500000000000D+01 4.500000000000D-09 6.000000000000D-01
    1.200000000000D-06 1.234567890123D-02 2.300000000000D-06 5.153795477500D+03
    3.456000000000D+05 4.500000000000D-08 1.500000000000D+00 5.600000000000D-08
    9.400000000000D-01 3.210000000000D+02 2.100000000000D-01-8.900000000000D-09
    7.800000000000D-10 0.000000000000D+00 1.930000000000D+03 0.000000000000D+00
    2.400000000000D+00 0.000000000000D+00-1.900000000000D-08 9.700000000000D+01
    0.000000000000D+00 4.000000000000D+00 0.000000000000D+00 0.000000000000D+00
";

    let ephs = parse_rinex_nav(data).expect("RINEX NAV leap second parse");

    assert_eq!(ephs.len(), 1);
    assert_eq!(ephs[0].week, 1930);
    assert!((ephs[0].toc_s - 17.0).abs() < 1.0e-9);
}

#[test]
fn parse_rinex_nav_reads_gps_rinex_2_ephemeris() {
    let data = "\
     2.11           NAVIGATION DATA     G (GPS)             RINEX VERSION / TYPE
bijux-gnss                              PGM / RUN BY / DATE
                                                            END OF HEADER
 1 22  5 13 20  0  0-1.234567890123D-04 2.345678901234D-12 0.000000000000D+00
    1.100000000000D+01 2.500000000000D+01 4.500000000000D-09 6.000000000000D-01
    1.200000000000D-06 1.234567890123D-02 2.300000000000D-06 5.153795477500D+03
    3.456000000000D+05 4.500000000000D-08 1.500000000000D+00 5.600000000000D-08
    9.400000000000D-01 3.210000000000D+02 2.100000000000D-01-8.900000000000D-09
    7.800000000000D-10 0.000000000000D+00 2.209000000000D+03 0.000000000000D+00
    2.400000000000D+00 0.000000000000D+00-1.900000000000D-08 9.700000000000D+01
    0.000000000000D+00 4.000000000000D+00
";

    let ephs = parse_rinex_nav(data).expect("RINEX 2 NAV parse");

    assert_eq!(ephs.len(), 1);
    assert_eq!(ephs[0].week, 2209);
    assert_eq!(ephs[0].sat.prn, 1);
}

#[test]
fn parse_rinex_broadcast_navigation_returns_ephemerides_and_klobuchar() {
    let data = "\
     3.04           NAVIGATION DATA     G (GPS)             RINEX VERSION / TYPE
bijux-gnss                              PGM / RUN BY / DATE
GPSA  1.2120D-08 1.4900D-08-5.9600D-08 1.1920D-07       IONOSPHERIC CORR
GPSB  1.1670D+05-2.2940D+05-1.3110D+05 1.0490D+06       IONOSPHERIC CORR
                                                            END OF HEADER
G01 2022 05 13 20 00 00-1.234567890123D-04 2.345678901234D-12 0.000000000000D+00
    1.100000000000D+01 2.500000000000D+01 4.500000000000D-09 6.000000000000D-01
    1.200000000000D-06 1.234567890123D-02 2.300000000000D-06 5.153795477500D+03
    3.456000000000D+05 4.500000000000D-08 1.500000000000D+00 5.600000000000D-08
    9.400000000000D-01 3.210000000000D+02 2.100000000000D-01-8.900000000000D-09
    7.800000000000D-10 0.000000000000D+00 2.209000000000D+03 0.000000000000D+00
    2.400000000000D+00 0.000000000000D+00-1.900000000000D-08 9.700000000000D+01
    0.000000000000D+00 4.000000000000D+00 0.000000000000D+00 0.000000000000D+00
";

    let navigation = parse_rinex_broadcast_navigation(data).expect("broadcast navigation");

    assert_eq!(navigation.ephemerides.len(), 1);
    assert_eq!(
        navigation.klobuchar,
        Some(KlobucharCoefficients::new(
            [0.1212e-7, 0.1490e-7, -0.5960e-7, 0.1192e-6],
            [0.1167e6, -0.2294e6, -0.1311e6, 0.1049e7],
        ))
    );
}

#[test]
fn write_rinex_nav_round_trips_through_parser() {
    let eph = sample_ephemeris();
    let path = std::env::temp_dir().join(format!(
        "bijux-rinex-nav-roundtrip-{}-{}.rnx",
        std::process::id(),
        eph.sat.prn
    ));

    write_rinex_nav(&path, std::slice::from_ref(&eph), true).expect("write nav");
    let data = std::fs::read_to_string(&path).expect("read nav");
    let parsed = parse_rinex_nav(&data).expect("parse written nav");
    std::fs::remove_file(&path).expect("remove nav fixture");

    assert_eq!(parsed.len(), 1);
    let parsed = &parsed[0];
    assert_eq!(parsed.sat, eph.sat);
    assert_eq!(parsed.week, eph.week);
    assert_eq!(parsed.iode, eph.iode);
    assert_eq!(parsed.iodc, eph.iodc);
    assert!((parsed.toe_s - eph.toe_s).abs() < 1.0e-9);
    assert!((parsed.toc_s - eph.toc_s).abs() < 1.0e-6);
    assert!((parsed.af0 - eph.af0).abs() < 1.0e-16);
    assert!((parsed.af1 - eph.af1).abs() < 1.0e-24);
    assert!((parsed.tgd - eph.tgd).abs() < 1.0e-18);
}

#[test]
fn write_rinex_broadcast_navigation_round_trips_klobuchar() {
    let klobuchar = KlobucharCoefficients::new(
        [0.1212e-7, 0.1490e-7, -0.5960e-7, 0.1192e-6],
        [0.1167e6, -0.2294e6, -0.1311e6, 0.1049e7],
    );
    let navigation = GpsBroadcastNavigationData {
        ephemerides: vec![sample_ephemeris()],
        klobuchar: Some(klobuchar),
    };
    let path = std::env::temp_dir().join(format!(
        "bijux-rinex-broadcast-navigation-roundtrip-{}-{}.rnx",
        std::process::id(),
        navigation.ephemerides[0].sat.prn
    ));

    write_rinex_broadcast_navigation(&path, &navigation, true).expect("write broadcast navigation");
    let data = std::fs::read_to_string(&path).expect("read broadcast navigation");
    let parsed = parse_rinex_broadcast_navigation(&data).expect("parse written navigation");
    std::fs::remove_file(&path).expect("remove nav fixture");

    assert_eq!(parsed.ephemerides.len(), 1);
    assert_eq!(parsed.ephemerides[0].sat, navigation.ephemerides[0].sat);
    assert_eq!(parsed.klobuchar, Some(klobuchar));
}
