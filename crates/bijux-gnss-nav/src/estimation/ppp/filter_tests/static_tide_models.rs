use super::*;

#[derive(Debug, Clone)]
struct MappedProductsProvider {
    states: BTreeMap<SatId, GpsSatState>,
}

impl ProductsProvider for MappedProductsProvider {
    fn sat_state(
        &self,
        sat: SatId,
        _t_s: f64,
        _diag: &mut ProductDiagnostics,
    ) -> Option<GpsSatState> {
        self.states.get(&sat).cloned()
    }

    fn clock_correction(
        &self,
        sat: SatId,
        _t_s: f64,
        _diag: &mut ProductDiagnostics,
    ) -> Option<GpsSatelliteClockCorrection> {
        self.states.get(&sat).map(|state| state.clock_correction.clone())
    }

    fn coverage_s(&self, _sat: SatId) -> Option<(f64, f64)> {
        None
    }
}

fn enu_offset_to_ecef(base_ecef_m: [f64; 3], enu_m: [f64; 3]) -> [f64; 3] {
    let (lat_deg, lon_deg, _alt_m) =
        ecef_to_geodetic(base_ecef_m[0], base_ecef_m[1], base_ecef_m[2]);
    let (sin_lat, cos_lat) = lat_deg.to_radians().sin_cos();
    let (sin_lon, cos_lon) = lon_deg.to_radians().sin_cos();
    let east_m = enu_m[0];
    let north_m = enu_m[1];
    let up_m = enu_m[2];
    let dx = -sin_lon * east_m - sin_lat * cos_lon * north_m + cos_lat * cos_lon * up_m;
    let dy = cos_lon * east_m - sin_lat * sin_lon * north_m + cos_lat * sin_lon * up_m;
    let dz = cos_lat * north_m + sin_lat * up_m;
    [base_ecef_m[0] + dx, base_ecef_m[1] + dy, base_ecef_m[2] + dz]
}

fn euclidean_distance_m(lhs: [f64; 3], rhs: [f64; 3]) -> f64 {
    let dx = lhs[0] - rhs[0];
    let dy = lhs[1] - rhs[1];
    let dz = lhs[2] - rhs[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn make_static_ppp_epoch(
    epoch_idx: u64,
    gps_time: GpsTime,
    receiver_ecef_m: [f64; 3],
    satellite_positions_m: &[(SatId, [f64; 3])],
    ztd_m: f64,
) -> ObsEpoch {
    let signal = signal_spec_gps_l1_ca();
    let wavelength_m = signal_wavelength_m(signal).0;
    let sats = satellite_positions_m
        .iter()
        .map(|(sat, sat_pos_m)| {
            let range_m = euclidean_distance_m(receiver_ecef_m, *sat_pos_m);
            let (_az_deg, elevation_deg) = elevation_azimuth_deg(
                receiver_ecef_m[0],
                receiver_ecef_m[1],
                receiver_ecef_m[2],
                sat_pos_m[0],
                sat_pos_m[1],
                sat_pos_m[2],
            );
            let troposphere_mapping = SaastamoinenModel::mapping_factor(elevation_deg);
            let pseudorange_m = range_m + ztd_m * troposphere_mapping;
            ObsSatellite {
                signal_id: SigId { sat: *sat, band: SignalBand::L1, code: SignalCode::Ca },
                pseudorange_m: Meters(pseudorange_m),
                pseudorange_var_m2: 0.01,
                carrier_phase_cycles: Cycles(pseudorange_m / wavelength_m),
                carrier_phase_var_cycles2: 1.0e-4,
                doppler_hz: Hertz(0.0),
                doppler_var_hz2: 1.0,
                cn0_dbhz: 48.0,
                lock_flags: LockFlags {
                    code_lock: true,
                    carrier_lock: true,
                    bit_lock: true,
                    cycle_slip: false,
                },
                multipath_suspect: false,
                observation_status: ObservationStatus::Accepted,
                observation_reject_reasons: Vec::new(),
                elevation_deg: Some(elevation_deg),
                azimuth_deg: None,
                weight: Some(1.0),
                timing: None,
                error_model: None,
                metadata: ObsMetadata {
                    tracking_mode: "test".to_string(),
                    integration_ms: 1,
                    lock_quality: 48.0,
                    smoothing_window: 0,
                    smoothing_age: 0,
                    smoothing_resets: 0,
                    signal,
                    ..ObsMetadata::default()
                },
            }
        })
        .collect();
    ObsEpoch {
        t_rx_s: Seconds(gps_time.to_seconds()),
        source_time: ReceiverSampleTrace::from_sample_index(epoch_idx, 1_000.0),
        gps_week: Some(gps_time.week),
        tow_s: Some(Seconds(gps_time.tow_s)),
        epoch_idx,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        role: ReceiverRole::Rover,
        sats,
        decision: ObservationEpochDecision::Accepted,
        decision_reason: Some("accepted_observables_present".to_string()),
        manifest: None,
    }
}

#[test]
fn ppp_filter_reduces_static_residuals_when_ocean_tide_loading_is_configured() {
    let base_receiver_ecef_m = {
        let (x, y, z) = geodetic_to_ecef(37.0, -122.0, 15.0);
        [x, y, z]
    };
    let gps_time = GpsTime { week: 2200, tow_s: 86_400.0 };
    let ocean_tide_loading_model = OceanTideLoadingModel {
        reference_time: gps_time,
        constituents: vec![OceanTideLoadingConstituent::new(
            OceanTideConstituent::M2,
            0.04,
            0.0,
            0.03,
            180.0,
            0.12,
            0.0,
        )],
    };
    let displacement_ecef_m = ocean_tide_loading_model
        .displacement_ecef_m(base_receiver_ecef_m, gps_time)
        .expect("ocean loading displacement");
    let displaced_receiver_ecef_m = [
        base_receiver_ecef_m[0] + displacement_ecef_m[0],
        base_receiver_ecef_m[1] + displacement_ecef_m[1],
        base_receiver_ecef_m[2] + displacement_ecef_m[2],
    ];
    let (lat_deg, lon_deg, alt_m) =
        ecef_to_geodetic(base_receiver_ecef_m[0], base_receiver_ecef_m[1], base_receiver_ecef_m[2]);
    let ztd_m = SaastamoinenModel::zenith_delay_m(Llh { lat_deg, lon_deg, alt_m });

    let satellite_positions_m = vec![
        (
            SatId { constellation: Constellation::Gps, prn: 1 },
            enu_offset_to_ecef(base_receiver_ecef_m, [16_000_000.0, 0.0, 20_500_000.0]),
        ),
        (
            SatId { constellation: Constellation::Gps, prn: 2 },
            enu_offset_to_ecef(base_receiver_ecef_m, [-14_000_000.0, 8_000_000.0, 21_000_000.0]),
        ),
        (
            SatId { constellation: Constellation::Gps, prn: 3 },
            enu_offset_to_ecef(base_receiver_ecef_m, [6_000_000.0, 14_000_000.0, 20_000_000.0]),
        ),
        (
            SatId { constellation: Constellation::Gps, prn: 4 },
            enu_offset_to_ecef(base_receiver_ecef_m, [-8_000_000.0, -15_000_000.0, 20_800_000.0]),
        ),
        (
            SatId { constellation: Constellation::Gps, prn: 5 },
            enu_offset_to_ecef(base_receiver_ecef_m, [12_000_000.0, -9_000_000.0, 19_800_000.0]),
        ),
    ];
    let provider = MappedProductsProvider {
        states: satellite_positions_m
            .iter()
            .map(|(sat, sat_pos_m)| {
                (
                    *sat,
                    GpsSatState {
                        x_m: sat_pos_m[0],
                        y_m: sat_pos_m[1],
                        z_m: sat_pos_m[2],
                        vx_mps: 0.0,
                        vy_mps: 0.0,
                        vz_mps: 0.0,
                        clock_correction: GpsSatelliteClockCorrection::from_bias_s(0.0),
                        uncertainty: SatelliteStateUncertainty::unavailable(),
                    },
                )
            })
            .collect(),
    };
    let ephs: Vec<_> = satellite_positions_m.iter().map(|(sat, _)| make_eph(sat.prn)).collect();
    let mut corrected_filter = PppFilter::new(PppConfig {
        use_iono_free: false,
        use_doppler: false,
        enable_iono_state: false,
        ocean_tide_loading_model: Some(ocean_tide_loading_model.clone()),
        ..PppConfig::default()
    });
    corrected_filter.seed_receiver_state(base_receiver_ecef_m, 0.0);

    let mut uncorrected_filter = PppFilter::new(PppConfig {
        use_iono_free: false,
        use_doppler: false,
        enable_iono_state: false,
        ..PppConfig::default()
    });
    uncorrected_filter.seed_receiver_state(base_receiver_ecef_m, 0.0);
    let mut corrected = None;
    let mut uncorrected = None;
    for epoch_idx in 0..6 {
        let obs = make_static_ppp_epoch(
            epoch_idx,
            gps_time,
            displaced_receiver_ecef_m,
            &satellite_positions_m,
            ztd_m,
        );
        corrected = corrected_filter.solve_epoch(&obs, &ephs, &provider);
        uncorrected = uncorrected_filter.solve_epoch(&obs, &ephs, &provider);
    }
    let corrected = corrected.expect("PPP solution with ocean tide loading");
    let uncorrected = uncorrected.expect("PPP solution without ocean tide loading");

    let corrected_pos_m = [corrected.ecef_x_m, corrected.ecef_y_m, corrected.ecef_z_m];
    let uncorrected_pos_m = [uncorrected.ecef_x_m, uncorrected.ecef_y_m, uncorrected.ecef_z_m];
    let corrected_monument_error_m = euclidean_distance_m(corrected_pos_m, base_receiver_ecef_m);
    let uncorrected_monument_error_m =
        euclidean_distance_m(uncorrected_pos_m, base_receiver_ecef_m);

    assert!(
        corrected_monument_error_m < uncorrected_monument_error_m,
        "corrected monument error {:.6} vs uncorrected monument error {:.6}",
        corrected_monument_error_m,
        uncorrected_monument_error_m
    );
    assert!(
        corrected.innovation_rms < uncorrected.innovation_rms,
        "corrected innovation rms {:.6} vs uncorrected innovation rms {:.6}",
        corrected.innovation_rms,
        uncorrected.innovation_rms
    );
    assert!(
        corrected.rms_m < uncorrected.rms_m,
        "corrected rms {:.6} vs uncorrected rms {:.6}",
        corrected.rms_m,
        uncorrected.rms_m
    );
    assert!(
        uncorrected_monument_error_m - corrected_monument_error_m > 0.005,
        "corrected monument error {:.6} vs uncorrected monument error {:.6}",
        corrected_monument_error_m,
        uncorrected_monument_error_m
    );
}

#[test]
fn ppp_filter_reduces_static_residuals_when_solid_earth_tide_is_configured() {
    let base_receiver_ecef_m = {
        let (x, y, z) = geodetic_to_ecef(47.0, 8.0, 450.0);
        [x, y, z]
    };
    let start_time = GpsTime { week: 2200, tow_s: 43_200.0 };
    let solid_earth_tide_model = SolidEarthTideModel;
    let displacement_ecef_m = solid_earth_tide_model
        .displacement_ecef_m(base_receiver_ecef_m, start_time)
        .expect("solid Earth tide displacement");
    let displaced_receiver_ecef_m = [
        base_receiver_ecef_m[0] + displacement_ecef_m[0],
        base_receiver_ecef_m[1] + displacement_ecef_m[1],
        base_receiver_ecef_m[2] + displacement_ecef_m[2],
    ];
    let (lat_deg, lon_deg, alt_m) =
        ecef_to_geodetic(base_receiver_ecef_m[0], base_receiver_ecef_m[1], base_receiver_ecef_m[2]);
    let ztd_m = SaastamoinenModel::zenith_delay_m(Llh { lat_deg, lon_deg, alt_m });

    let satellite_positions_m = vec![
        (
            SatId { constellation: Constellation::Gps, prn: 11 },
            enu_offset_to_ecef(base_receiver_ecef_m, [15_000_000.0, 1_000_000.0, 21_500_000.0]),
        ),
        (
            SatId { constellation: Constellation::Gps, prn: 14 },
            enu_offset_to_ecef(base_receiver_ecef_m, [-11_000_000.0, 9_000_000.0, 20_800_000.0]),
        ),
        (
            SatId { constellation: Constellation::Gps, prn: 18 },
            enu_offset_to_ecef(base_receiver_ecef_m, [8_000_000.0, 15_000_000.0, 19_700_000.0]),
        ),
        (
            SatId { constellation: Constellation::Gps, prn: 22 },
            enu_offset_to_ecef(base_receiver_ecef_m, [-9_000_000.0, -14_000_000.0, 20_400_000.0]),
        ),
        (
            SatId { constellation: Constellation::Gps, prn: 25 },
            enu_offset_to_ecef(base_receiver_ecef_m, [13_000_000.0, -7_000_000.0, 20_100_000.0]),
        ),
        (
            SatId { constellation: Constellation::Gps, prn: 31 },
            enu_offset_to_ecef(base_receiver_ecef_m, [-4_000_000.0, 16_000_000.0, 21_000_000.0]),
        ),
    ];
    let provider = MappedProductsProvider {
        states: satellite_positions_m
            .iter()
            .map(|(sat, sat_pos_m)| {
                (
                    *sat,
                    GpsSatState {
                        x_m: sat_pos_m[0],
                        y_m: sat_pos_m[1],
                        z_m: sat_pos_m[2],
                        vx_mps: 0.0,
                        vy_mps: 0.0,
                        vz_mps: 0.0,
                        clock_correction: GpsSatelliteClockCorrection::from_bias_s(0.0),
                        uncertainty: SatelliteStateUncertainty::unavailable(),
                    },
                )
            })
            .collect(),
    };
    let ephs: Vec<_> = satellite_positions_m.iter().map(|(sat, _)| make_eph(sat.prn)).collect();
    let mut corrected_filter = PppFilter::new(PppConfig {
        use_iono_free: false,
        use_doppler: false,
        enable_iono_state: false,
        solid_earth_tide_model: Some(solid_earth_tide_model),
        ..PppConfig::default()
    });
    corrected_filter.seed_receiver_state(base_receiver_ecef_m, 0.0);

    let mut uncorrected_filter = PppFilter::new(PppConfig {
        use_iono_free: false,
        use_doppler: false,
        enable_iono_state: false,
        ..PppConfig::default()
    });
    uncorrected_filter.seed_receiver_state(base_receiver_ecef_m, 0.0);

    let mut corrected = None;
    let mut uncorrected = None;
    for epoch_idx in 0..8 {
        let obs = make_static_ppp_epoch(
            epoch_idx,
            start_time,
            displaced_receiver_ecef_m,
            &satellite_positions_m,
            ztd_m,
        );
        corrected = corrected_filter.solve_epoch(&obs, &ephs, &provider);
        uncorrected = uncorrected_filter.solve_epoch(&obs, &ephs, &provider);
    }

    let corrected = corrected.expect("PPP solution with solid Earth tide");
    let uncorrected = uncorrected.expect("PPP solution without solid Earth tide");
    let corrected_pos_m = [corrected.ecef_x_m, corrected.ecef_y_m, corrected.ecef_z_m];
    let uncorrected_pos_m = [uncorrected.ecef_x_m, uncorrected.ecef_y_m, uncorrected.ecef_z_m];
    let corrected_monument_error_m = euclidean_distance_m(corrected_pos_m, base_receiver_ecef_m);
    let uncorrected_monument_error_m =
        euclidean_distance_m(uncorrected_pos_m, base_receiver_ecef_m);

    assert!(
        corrected_monument_error_m < uncorrected_monument_error_m,
        "corrected monument error {:.6} vs uncorrected monument error {:.6}",
        corrected_monument_error_m,
        uncorrected_monument_error_m
    );
    assert!(
        corrected.innovation_rms < uncorrected.innovation_rms,
        "corrected innovation rms {:.6} vs uncorrected innovation rms {:.6}",
        corrected.innovation_rms,
        uncorrected.innovation_rms
    );
    assert!(
        corrected.rms_m < uncorrected.rms_m,
        "corrected rms {:.6} vs uncorrected rms {:.6}",
        corrected.rms_m,
        uncorrected.rms_m
    );
    assert!(
        uncorrected_monument_error_m - corrected_monument_error_m > 0.005,
        "corrected monument error {:.6} vs uncorrected monument error {:.6}",
        corrected_monument_error_m,
        uncorrected_monument_error_m
    );
}
