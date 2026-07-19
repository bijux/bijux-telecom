use super::*;

fn antenna_range_geometry(
    gps_time: Option<bijux_gnss_core::api::GpsTime>,
    receiver_pos_m: [f64; 3],
    sat_pos_m: [f64; 3],
) -> AntennaRangeGeometry {
    AntennaRangeGeometry { gps_time, receiver_pos_m, sat_pos_m }
}

fn antenna_phase_geometry(
    gps_time: Option<bijux_gnss_core::api::GpsTime>,
    receiver_pos_m: [f64; 3],
    sat_pos_m: [f64; 3],
    elevation_deg: f64,
    azimuth_deg: Option<f64>,
) -> AntennaPhaseGeometry {
    AntennaPhaseGeometry {
        range: antenna_range_geometry(gps_time, receiver_pos_m, sat_pos_m),
        elevation_deg,
        azimuth_deg,
    }
}

fn dual_frequency_signal(
    primary_band: SignalBand,
    primary_frequency_hz: f64,
    secondary_band: SignalBand,
    secondary_frequency_hz: f64,
) -> DualFrequencySignal {
    DualFrequencySignal {
        primary_band,
        primary_frequency_hz,
        secondary_band,
        secondary_frequency_hz,
    }
}

#[test]
fn ppp_filter_uses_satellite_antenna_calibration_for_single_frequency_range() {
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let gps_time = Some(bijux_gnss_core::api::GpsTime { week: 2200, tow_s: 86_400.0 });
    let sat_pos_m = [20_200_000.0, 14_000_000.0, 21_700_000.0];
    let receiver_pos_m = [1_111_111.0, -4_222_222.0, 4_333_333.0];
    let calibrations = SatelliteAntennaCalibrations {
        entries: vec![SatelliteAntennaCalibration {
            sat,
            antenna_type: "GPS TEST".to_string(),
            valid_from_unix_s: None,
            valid_until_unix_s: None,
            offsets_by_band: BTreeMap::from([(
                SignalBand::L1,
                SatellitePhaseCenterOffset::new(0.12, -0.04, 0.95),
            )]),
            variations_by_band: BTreeMap::new(),
        }],
    };

    let expected = calibrations
        .range_correction_m(
            sat,
            SignalBand::L1,
            antenna_range_geometry(gps_time, receiver_pos_m, sat_pos_m),
        )
        .expect("single-frequency antenna correction");
    let actual =
        single_frequency_antenna_range_correction_m(SingleFrequencyAntennaCorrectionRequest {
            satellite_calibrations: Some(&calibrations),
            receiver_antenna_type: None,
            receiver_calibrations: None,
            sat,
            band: SignalBand::L1,
            geometry: antenna_phase_geometry(gps_time, receiver_pos_m, sat_pos_m, 85.0, None),
        });

    assert!(actual.abs() > 1.0e-6);
    assert!((actual - expected).abs() < 1.0e-12);
}

#[test]
fn ppp_filter_includes_phase_variation_for_single_frequency_antenna_range() {
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let gps_time = Some(bijux_gnss_core::api::GpsTime { week: 2200, tow_s: 86_400.0 });
    let sat_pos_m = [20_200_000.0, 14_000_000.0, 21_700_000.0];
    let receiver_pos_m = [1_111_111.0, -4_222_222.0, 4_333_333.0];
    let receiver_antenna_type = "AOAD/M_T NONE";
    let satellite_calibrations = SatelliteAntennaCalibrations {
        entries: vec![SatelliteAntennaCalibration {
            sat,
            antenna_type: "GPS TEST".to_string(),
            valid_from_unix_s: None,
            valid_until_unix_s: None,
            offsets_by_band: BTreeMap::from([(
                SignalBand::L1,
                SatellitePhaseCenterOffset::new(0.0, 0.0, 0.0),
            )]),
            variations_by_band: BTreeMap::from([(
                SignalBand::L1,
                AntennaPhaseCenterVariation::no_azimuth(0.0, 10.0, vec![0.0, 0.10]),
            )]),
        }],
    };
    let receiver_calibrations = ReceiverAntennaCalibrations {
        entries: vec![ReceiverAntennaCalibration {
            antenna_type: receiver_antenna_type.to_string(),
            valid_from_unix_s: None,
            valid_until_unix_s: None,
            offsets_by_band: BTreeMap::from([(
                SignalBand::L1,
                ReceiverPhaseCenterOffset::new(0.0, 0.0, 0.0),
            )]),
            variations_by_band: BTreeMap::from([(
                SignalBand::L1,
                AntennaPhaseCenterVariation::no_azimuth(0.0, 10.0, vec![0.0, 0.06]),
            )]),
        }],
    };

    let actual =
        single_frequency_antenna_range_correction_m(SingleFrequencyAntennaCorrectionRequest {
            satellite_calibrations: Some(&satellite_calibrations),
            receiver_antenna_type: Some(receiver_antenna_type),
            receiver_calibrations: Some(&receiver_calibrations),
            sat,
            band: SignalBand::L1,
            geometry: antenna_phase_geometry(gps_time, receiver_pos_m, sat_pos_m, 85.0, None),
        });

    assert!((actual - 0.08).abs() < 1.0e-12);
}

#[test]
fn ppp_filter_uses_satellite_antenna_calibration_for_iono_free_range() {
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let gps_time = Some(bijux_gnss_core::api::GpsTime { week: 2200, tow_s: 86_400.0 });
    let sat_pos_m = [20_200_000.0, 14_000_000.0, 21_700_000.0];
    let receiver_pos_m = [1_111_111.0, -4_222_222.0, 4_333_333.0];
    let l1 = signal_spec_gps_l1_ca();
    let l2 = signal_spec_gps_l2_py();
    let calibrations = SatelliteAntennaCalibrations {
        entries: vec![SatelliteAntennaCalibration {
            sat,
            antenna_type: "GPS TEST".to_string(),
            valid_from_unix_s: None,
            valid_until_unix_s: None,
            offsets_by_band: BTreeMap::from([
                (SignalBand::L1, SatellitePhaseCenterOffset::new(0.08, 0.01, 0.91)),
                (SignalBand::L2, SatellitePhaseCenterOffset::new(0.14, -0.03, 1.12)),
            ]),
            variations_by_band: BTreeMap::new(),
        }],
    };

    let expected = calibrations
        .iono_free_range_correction_m(
            sat,
            dual_frequency_signal(
                SignalBand::L1,
                l1.carrier_hz.value(),
                SignalBand::L2,
                l2.carrier_hz.value(),
            ),
            antenna_range_geometry(gps_time, receiver_pos_m, sat_pos_m),
        )
        .expect("iono-free antenna correction");
    let actual = iono_free_antenna_range_correction_m(IonoFreeAntennaCorrectionRequest {
        satellite_calibrations: Some(&calibrations),
        receiver_antenna_type: None,
        receiver_calibrations: None,
        sat,
        signal: dual_frequency_signal(
            SignalBand::L1,
            l1.carrier_hz.value(),
            SignalBand::L2,
            l2.carrier_hz.value(),
        ),
        geometry: antenna_phase_geometry(gps_time, receiver_pos_m, sat_pos_m, 85.0, None),
    });

    assert!(actual.abs() > 1.0e-6);
    assert!((actual - expected).abs() < 1.0e-12);
}

#[test]
fn ppp_filter_includes_phase_variation_for_iono_free_antenna_range() {
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let gps_time = Some(bijux_gnss_core::api::GpsTime { week: 2200, tow_s: 86_400.0 });
    let sat_pos_m = [20_200_000.0, 14_000_000.0, 21_700_000.0];
    let receiver_pos_m = [1_111_111.0, -4_222_222.0, 4_333_333.0];
    let receiver_antenna_type = "AOAD/M_T NONE";
    let l1 = signal_spec_gps_l1_ca();
    let l2 = signal_spec_gps_l2_py();
    let satellite_calibrations = SatelliteAntennaCalibrations {
        entries: vec![SatelliteAntennaCalibration {
            sat,
            antenna_type: "GPS TEST".to_string(),
            valid_from_unix_s: None,
            valid_until_unix_s: None,
            offsets_by_band: BTreeMap::from([
                (SignalBand::L1, SatellitePhaseCenterOffset::new(0.0, 0.0, 0.0)),
                (SignalBand::L2, SatellitePhaseCenterOffset::new(0.0, 0.0, 0.0)),
            ]),
            variations_by_band: BTreeMap::from([
                (
                    SignalBand::L1,
                    AntennaPhaseCenterVariation::no_azimuth(0.0, 10.0, vec![0.0, 0.10]),
                ),
                (
                    SignalBand::L2,
                    AntennaPhaseCenterVariation::no_azimuth(0.0, 10.0, vec![0.0, 0.02]),
                ),
            ]),
        }],
    };
    let receiver_calibrations = ReceiverAntennaCalibrations {
        entries: vec![ReceiverAntennaCalibration {
            antenna_type: receiver_antenna_type.to_string(),
            valid_from_unix_s: None,
            valid_until_unix_s: None,
            offsets_by_band: BTreeMap::from([
                (SignalBand::L1, ReceiverPhaseCenterOffset::new(0.0, 0.0, 0.0)),
                (SignalBand::L2, ReceiverPhaseCenterOffset::new(0.0, 0.0, 0.0)),
            ]),
            variations_by_band: BTreeMap::from([
                (
                    SignalBand::L1,
                    AntennaPhaseCenterVariation::no_azimuth(0.0, 10.0, vec![0.0, 0.06]),
                ),
                (
                    SignalBand::L2,
                    AntennaPhaseCenterVariation::no_azimuth(0.0, 10.0, vec![0.0, 0.01]),
                ),
            ]),
        }],
    };

    let expected = satellite_calibrations
        .iono_free_range_correction_with_phase_variation_m(
            sat,
            dual_frequency_signal(
                SignalBand::L1,
                l1.carrier_hz.value(),
                SignalBand::L2,
                l2.carrier_hz.value(),
            ),
            antenna_phase_geometry(gps_time, receiver_pos_m, sat_pos_m, 85.0, None),
        )
        .expect("satellite antenna correction")
        + receiver_calibrations
            .iono_free_range_correction_with_phase_variation_m(
                receiver_antenna_type,
                dual_frequency_signal(
                    SignalBand::L1,
                    l1.carrier_hz.value(),
                    SignalBand::L2,
                    l2.carrier_hz.value(),
                ),
                antenna_phase_geometry(gps_time, receiver_pos_m, sat_pos_m, 85.0, None),
            )
            .expect("receiver antenna correction");
    let actual = iono_free_antenna_range_correction_m(IonoFreeAntennaCorrectionRequest {
        satellite_calibrations: Some(&satellite_calibrations),
        receiver_antenna_type: Some(receiver_antenna_type),
        receiver_calibrations: Some(&receiver_calibrations),
        sat,
        signal: dual_frequency_signal(
            SignalBand::L1,
            l1.carrier_hz.value(),
            SignalBand::L2,
            l2.carrier_hz.value(),
        ),
        geometry: antenna_phase_geometry(gps_time, receiver_pos_m, sat_pos_m, 85.0, None),
    });

    assert!(expected.abs() > 1.0e-6);
    assert!((actual - expected).abs() < 1.0e-12);
}

#[test]
fn ppp_filter_uses_receiver_antenna_calibration_for_single_frequency_range() {
    let gps_time = Some(bijux_gnss_core::api::GpsTime { week: 2200, tow_s: 86_400.0 });
    let sat_pos_m = [20_200_000.0, 14_000_000.0, 21_700_000.0];
    let receiver_pos_m = [1_111_111.0, -4_222_222.0, 4_333_333.0];
    let receiver_antenna_type = "AOAD/M_T NONE";
    let calibrations = ReceiverAntennaCalibrations {
        entries: vec![ReceiverAntennaCalibration {
            antenna_type: receiver_antenna_type.to_string(),
            valid_from_unix_s: None,
            valid_until_unix_s: None,
            offsets_by_band: BTreeMap::from([(
                SignalBand::L1,
                ReceiverPhaseCenterOffset::new(0.12, -0.04, 0.95),
            )]),
            variations_by_band: BTreeMap::new(),
        }],
    };

    let expected = calibrations
        .range_correction_m(
            receiver_antenna_type,
            SignalBand::L1,
            antenna_range_geometry(gps_time, receiver_pos_m, sat_pos_m),
        )
        .expect("single-frequency receiver antenna correction");
    let actual =
        single_frequency_antenna_range_correction_m(SingleFrequencyAntennaCorrectionRequest {
            satellite_calibrations: None,
            receiver_antenna_type: Some(receiver_antenna_type),
            receiver_calibrations: Some(&calibrations),
            sat: SatId { constellation: Constellation::Gps, prn: 7 },
            band: SignalBand::L1,
            geometry: antenna_phase_geometry(gps_time, receiver_pos_m, sat_pos_m, 85.0, None),
        });

    assert!(actual.abs() > 1.0e-6);
    assert!((actual - expected).abs() < 1.0e-12);
}

#[test]
fn ppp_filter_uses_receiver_antenna_calibration_for_iono_free_range() {
    let gps_time = Some(bijux_gnss_core::api::GpsTime { week: 2200, tow_s: 86_400.0 });
    let sat_pos_m = [20_200_000.0, 14_000_000.0, 21_700_000.0];
    let receiver_pos_m = [1_111_111.0, -4_222_222.0, 4_333_333.0];
    let receiver_antenna_type = "AOAD/M_T NONE";
    let l1 = signal_spec_gps_l1_ca();
    let l2 = signal_spec_gps_l2_py();
    let calibrations = ReceiverAntennaCalibrations {
        entries: vec![ReceiverAntennaCalibration {
            antenna_type: receiver_antenna_type.to_string(),
            valid_from_unix_s: None,
            valid_until_unix_s: None,
            offsets_by_band: BTreeMap::from([
                (SignalBand::L1, ReceiverPhaseCenterOffset::new(0.08, 0.01, 0.91)),
                (SignalBand::L2, ReceiverPhaseCenterOffset::new(0.14, -0.03, 1.12)),
            ]),
            variations_by_band: BTreeMap::new(),
        }],
    };

    let expected = calibrations
        .iono_free_range_correction_m(
            receiver_antenna_type,
            dual_frequency_signal(
                SignalBand::L1,
                l1.carrier_hz.value(),
                SignalBand::L2,
                l2.carrier_hz.value(),
            ),
            antenna_range_geometry(gps_time, receiver_pos_m, sat_pos_m),
        )
        .expect("iono-free receiver antenna correction");
    let actual = iono_free_antenna_range_correction_m(IonoFreeAntennaCorrectionRequest {
        satellite_calibrations: None,
        receiver_antenna_type: Some(receiver_antenna_type),
        receiver_calibrations: Some(&calibrations),
        sat: SatId { constellation: Constellation::Gps, prn: 7 },
        signal: dual_frequency_signal(
            SignalBand::L1,
            l1.carrier_hz.value(),
            SignalBand::L2,
            l2.carrier_hz.value(),
        ),
        geometry: antenna_phase_geometry(gps_time, receiver_pos_m, sat_pos_m, 85.0, None),
    });

    assert!(actual.abs() > 1.0e-6);
    assert!((actual - expected).abs() < 1.0e-12);
}

#[test]
fn ppp_filter_receiver_antenna_type_selects_matching_calibration() {
    let gps_time = Some(bijux_gnss_core::api::GpsTime { week: 2200, tow_s: 86_400.0 });
    let sat_pos_m = [20_200_000.0, 14_000_000.0, 21_700_000.0];
    let receiver_pos_m = [1_111_111.0, -4_222_222.0, 4_333_333.0];
    let calibrations = ReceiverAntennaCalibrations {
        entries: vec![
            ReceiverAntennaCalibration {
                antenna_type: "AOAD/M_T NONE".to_string(),
                valid_from_unix_s: None,
                valid_until_unix_s: None,
                offsets_by_band: BTreeMap::from([(
                    SignalBand::L1,
                    ReceiverPhaseCenterOffset::new(0.05, 0.01, 0.70),
                )]),
                variations_by_band: BTreeMap::new(),
            },
            ReceiverAntennaCalibration {
                antenna_type: "TRM57971.00 NONE".to_string(),
                valid_from_unix_s: None,
                valid_until_unix_s: None,
                offsets_by_band: BTreeMap::from([(
                    SignalBand::L1,
                    ReceiverPhaseCenterOffset::new(0.25, 0.08, 1.30),
                )]),
                variations_by_band: BTreeMap::new(),
            },
        ],
    };

    let aoad =
        single_frequency_antenna_range_correction_m(SingleFrequencyAntennaCorrectionRequest {
            satellite_calibrations: None,
            receiver_antenna_type: Some("AOAD/M_T NONE"),
            receiver_calibrations: Some(&calibrations),
            sat: SatId { constellation: Constellation::Gps, prn: 7 },
            band: SignalBand::L1,
            geometry: antenna_phase_geometry(gps_time, receiver_pos_m, sat_pos_m, 85.0, None),
        });
    let trimble =
        single_frequency_antenna_range_correction_m(SingleFrequencyAntennaCorrectionRequest {
            satellite_calibrations: None,
            receiver_antenna_type: Some("TRM57971.00 NONE"),
            receiver_calibrations: Some(&calibrations),
            sat: SatId { constellation: Constellation::Gps, prn: 7 },
            band: SignalBand::L1,
            geometry: antenna_phase_geometry(gps_time, receiver_pos_m, sat_pos_m, 85.0, None),
        });

    assert!((trimble - aoad).abs() > 1.0e-6);
}
