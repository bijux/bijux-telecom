use std::collections::{BTreeMap, BTreeSet};

use super::{
    iono_free_antenna_range_correction_m, iono_free_satellite_representatives,
    phase_windup_cycles_for_satellite, ppp_code_sigma_m, ppp_common_range_sigma_m,
    ppp_indices_from_state_identities, ppp_measurement_observations, ppp_phase_sigma_cycles,
    ppp_state_label, ppp_stochastic_evidence_from_config, resolved_code_bias_m,
    resolved_iono_free_code_bias_m, single_frequency_antenna_range_correction_m, PppFilter,
    PppPhaseBreaks, SPEED_OF_LIGHT_MPS,
};
use crate::api::{
    ecef_to_geodetic, elevation_azimuth_deg, geodetic_to_ecef, BroadcastProductsProvider,
    GpsEphemeris, GpsSatState, GpsSatelliteClockCorrection, OceanTideConstituent,
    OceanTideLoadingConstituent, OceanTideLoadingModel, PppConfig, ProductDiagnostics,
    ProductsProvider, SatelliteClockUncertaintySource, SatelliteHealthSource,
    SatelliteHealthStatus, SatelliteOrbitUncertaintySource, SatelliteStateUncertainty,
    SolidEarthTideModel,
};
use crate::corrections::biases::{
    CodeBias, CodeBiasProvider, PhaseBias, SignalCodeBiases, SignalPhaseBiases, ZeroBiases,
};
use crate::corrections::phase_windup::PhaseWindupState;
use crate::estimation::ppp::config::{
    PppArMode, PppIntegerAmbiguityKind, PppLifecycleEventKind, PppMeasurementNoise,
    PppPreciseProductAction, PppProductSupport, PppStateIdentity, WlAmbiguity,
};
use crate::estimation::ppp::measurements::iono_free_code_observation_from_obs;
use crate::formats::precise_products::{
    PreciseProductDiscontinuity, PreciseProductDiscontinuityKind, PreciseProductSurface,
};
use crate::models::antenna::{
    AntennaPhaseCenterVariation, ReceiverAntennaCalibration, ReceiverAntennaCalibrations,
    ReceiverPhaseCenterOffset, SatelliteAntennaCalibration, SatelliteAntennaCalibrations,
    SatellitePhaseCenterOffset,
};
use crate::models::atmosphere::SaastamoinenModel;
use bijux_gnss_core::api::{
    Constellation, Cycles, GpsTime, Hertz, Llh, LockFlags, Meters, ObsEpoch, ObsMetadata,
    ObsSatellite, ObservationEpochDecision, ObservationStatus, ReceiverRole, ReceiverSampleTrace,
    SatId, Seconds, SigId, SignalBand, SignalCode,
};
use bijux_gnss_signal::api::{signal_spec_gps_l1_ca, signal_spec_gps_l2_py, signal_wavelength_m};

#[path = "tests/ambiguity_resolution.rs"]
mod ambiguity_resolution;
#[path = "tests/state_lifecycle.rs"]
mod state_lifecycle;

#[derive(Debug, Clone)]
struct StubProductsProvider {
    state: GpsSatState,
    precise_clock_bias_s: Option<f64>,
}

impl ProductsProvider for StubProductsProvider {
    fn sat_state(
        &self,
        _sat: SatId,
        _t_s: f64,
        _diag: &mut ProductDiagnostics,
    ) -> Option<GpsSatState> {
        Some(self.state.clone())
    }

    fn clock_correction(
        &self,
        _sat: SatId,
        _t_s: f64,
        _diag: &mut ProductDiagnostics,
    ) -> Option<GpsSatelliteClockCorrection> {
        self.precise_clock_bias_s.map(GpsSatelliteClockCorrection::from_bias_s)
    }

    fn coverage_s(&self, _sat: SatId) -> Option<(f64, f64)> {
        None
    }
}

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

fn make_eph(prn: u8) -> GpsEphemeris {
    GpsEphemeris {
        sat: SatId { constellation: Constellation::Gps, prn },
        iodc: 0,
        iode: 0,
        week: 0,
        sv_health: 0,
        sv_accuracy: Some(2),
        toe_s: 0.0,
        toc_s: 0.0,
        sqrt_a: 5153.7954775,
        e: 0.02,
        i0: 0.94,
        idot: 0.0,
        omega0: 0.1,
        omegadot: 0.0,
        w: 0.2,
        m0: 0.3,
        delta_n: 0.0,
        cuc: 0.0,
        cus: 0.0,
        crc: 0.0,
        crs: 0.0,
        cic: 0.0,
        cis: 0.0,
        af0: 1.0e-4,
        af1: -2.0e-12,
        af2: 3.0e-20,
        tgd: 8.0e-9,
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

fn ppp_test_satellite(sat: SatId) -> ObsSatellite {
    ObsSatellite {
        signal_id: SigId { sat, band: SignalBand::L1, code: SignalCode::Ca },
        pseudorange_m: Meters(20_000_000.0),
        pseudorange_var_m2: 1.0,
        carrier_phase_cycles: Cycles(100.0),
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
            signal: signal_spec_gps_l1_ca(),
            ..ObsMetadata::default()
        },
    }
}

fn ppp_test_signal_satellite(sat: SatId, band: SignalBand, code: SignalCode) -> ObsSatellite {
    let mut observation = ppp_test_satellite(sat);
    observation.signal_id = SigId { sat, band, code };
    observation.metadata.signal = match (band, code) {
        (SignalBand::L1, SignalCode::Ca) => signal_spec_gps_l1_ca(),
        (SignalBand::L2, SignalCode::Py) => signal_spec_gps_l2_py(),
        _ => observation.metadata.signal,
    };
    observation
}

fn ppp_test_epoch(sats: Vec<ObsSatellite>) -> ObsEpoch {
    ObsEpoch {
        t_rx_s: Seconds(0.0),
        source_time: ReceiverSampleTrace::from_sample_index(0, 1_000.0),
        gps_week: Some(2200),
        tow_s: Some(Seconds(0.0)),
        epoch_idx: 0,
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

fn precise_product_discontinuity(
    sat: SatId,
    surface: PreciseProductSurface,
    kind: PreciseProductDiscontinuityKind,
) -> PreciseProductDiscontinuity {
    PreciseProductDiscontinuity { sat, t_s: 10.0, surface, kind }
}

#[test]
fn ppp_precise_product_policy_bridges_missing_support_without_state_reset() {
    let observation = ppp_test_satellite(SatId { constellation: Constellation::Gps, prn: 22 });
    let sig = observation.signal_id;
    let mut filter = PppFilter::new(PppConfig { enable_iono_state: true, ..PppConfig::default() });
    filter.ensure_states(&[&observation], &PppPhaseBreaks::default());
    let ambiguity_index = filter.indices.ambiguity[&sig];
    let ionosphere_index = filter.indices.iono[&sig.sat];

    let usable = filter.apply_precise_product_policy(
        5,
        sig.sat,
        sig,
        &[precise_product_discontinuity(
            sig.sat,
            PreciseProductSurface::Orbit,
            PreciseProductDiscontinuityKind::MissingSatellite,
        )],
    );

    assert!(usable);
    assert_eq!(filter.indices.ambiguity.get(&sig), Some(&ambiguity_index));
    assert_eq!(filter.indices.iono.get(&sig.sat), Some(&ionosphere_index));
    assert!(filter.health.lifecycle_events.iter().any(|event| {
        event.kind == PppLifecycleEventKind::PreciseProductDiscontinuity
            && event.reason
                == "precise_product_orbit_missing_satellite_action_bridge_with_broadcast"
            && event.removed_states.is_empty()
    }));
}

#[test]
fn ppp_precise_product_policy_inflates_satellite_state_covariance() {
    let observation = ppp_test_satellite(SatId { constellation: Constellation::Gps, prn: 23 });
    let sig = observation.signal_id;
    let mut config = PppConfig { enable_iono_state: true, ..PppConfig::default() };
    config.precise_product_policy.orbit_gap_action = PppPreciseProductAction::InflateSatelliteState;
    config.precise_product_policy.satellite_state_inflation = 12.0;
    let mut filter = PppFilter::new(config);
    filter.ensure_states(&[&observation], &PppPhaseBreaks::default());
    let ambiguity_index = filter.indices.ambiguity[&sig];
    let ionosphere_index = filter.indices.iono[&sig.sat];
    let ambiguity_variance = filter.ekf.p[(ambiguity_index, ambiguity_index)];
    let ionosphere_variance = filter.ekf.p[(ionosphere_index, ionosphere_index)];

    let usable = filter.apply_precise_product_policy(
        6,
        sig.sat,
        sig,
        &[precise_product_discontinuity(
            sig.sat,
            PreciseProductSurface::Orbit,
            PreciseProductDiscontinuityKind::OrbitGap,
        )],
    );

    assert!(usable);
    assert_eq!(filter.indices.ambiguity.get(&sig), Some(&ambiguity_index));
    assert_eq!(filter.indices.iono.get(&sig.sat), Some(&ionosphere_index));
    assert!(
        (filter.ekf.p[(ambiguity_index, ambiguity_index)] - ambiguity_variance * 12.0).abs()
            < 1.0e-9
    );
    assert!(
        (filter.ekf.p[(ionosphere_index, ionosphere_index)] - ionosphere_variance * 12.0).abs()
            < 1.0e-9
    );
}

#[test]
fn ppp_precise_product_policy_resets_satellite_states_on_clock_jump() {
    let observation = ppp_test_satellite(SatId { constellation: Constellation::Gps, prn: 24 });
    let sig = observation.signal_id;
    let mut filter = PppFilter::new(PppConfig { enable_iono_state: true, ..PppConfig::default() });
    filter.ensure_states(&[&observation], &PppPhaseBreaks::default());
    filter.last_seen_iono.insert(sig.sat, 4);
    filter.last_seen_amb.insert(sig, 4);
    filter.residual_history.insert(sig, vec![3.0]);

    let usable = filter.apply_precise_product_policy(
        7,
        sig.sat,
        sig,
        &[precise_product_discontinuity(
            sig.sat,
            PreciseProductSurface::Clock,
            PreciseProductDiscontinuityKind::ClockJump,
        )],
    );

    assert!(usable);
    assert!(!filter.indices.ambiguity.contains_key(&sig));
    assert!(!filter.indices.iono.contains_key(&sig.sat));
    assert!(!filter.last_seen_amb.contains_key(&sig));
    assert!(!filter.last_seen_iono.contains_key(&sig.sat));
    assert!(!filter.residual_history.contains_key(&sig));
    assert!(filter.health.lifecycle_events.iter().any(|event| {
        event.kind == PppLifecycleEventKind::PreciseProductDiscontinuity
            && event.reason == "precise_product_clock_clock_jump_action_reset_satellite_state"
            && event.removed_states.contains(&super::PppStateIdentity::SlantIonosphere(sig.sat))
            && event.removed_states.contains(&super::PppStateIdentity::CarrierAmbiguity(sig))
    }));
}

#[test]
fn ppp_precise_product_policy_refuses_flagged_orbit_records() {
    let observation = ppp_test_satellite(SatId { constellation: Constellation::Gps, prn: 25 });
    let sig = observation.signal_id;
    let mut filter = PppFilter::new(PppConfig::default());

    let usable = filter.apply_precise_product_policy(
        8,
        sig.sat,
        sig,
        &[precise_product_discontinuity(
            sig.sat,
            PreciseProductSurface::Orbit,
            PreciseProductDiscontinuityKind::OrbitFlag,
        )],
    );

    assert!(!usable);
    assert!(filter.health.lifecycle_events.iter().any(|event| {
        event.kind == PppLifecycleEventKind::PreciseProductDiscontinuity
            && event.reason == "precise_product_orbit_orbit_flag_action_refuse_satellite"
            && event.sat == Some(sig.sat)
            && event.signal == Some(sig)
    }));
}

#[test]
fn ppp_filter_uses_precise_clock_correction_when_available() {
    let eph = make_eph(1);
    let t_s = 1_350.0;
    let state = BroadcastProductsProvider::new(vec![eph.clone()])
        .sat_state(eph.sat, t_s, &mut ProductDiagnostics::default())
        .expect("broadcast state");
    let provider = StubProductsProvider { state, precise_clock_bias_s: Some(2.5e-9) };
    let filter = PppFilter::new(PppConfig::default());

    let (_state, clock_bias_s, fallback, _support, _discontinuities) =
        filter.sat_state(&provider, &eph, eph.sat, t_s).expect("PPP precise clock state");

    assert!((clock_bias_s - 2.5e-9).abs() < 1e-18);
    assert!(!fallback);
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

#[test]
fn ppp_filter_falls_back_to_current_broadcast_clock_when_precise_clock_is_missing() {
    let eph = make_eph(1);
    let t_s = 1_350.0;
    let state = BroadcastProductsProvider::new(vec![eph.clone()])
        .sat_state(eph.sat, t_s, &mut ProductDiagnostics::default())
        .expect("broadcast state");
    let provider = StubProductsProvider { state, precise_clock_bias_s: None };
    let filter = PppFilter::new(PppConfig::default());

    let (_state, clock_bias_s, fallback, _support, _discontinuities) = filter
        .sat_state(&provider, &eph, eph.sat, t_s)
        .expect("PPP broadcast fallback clock state");
    let expected = crate::api::gps_satellite_clock_correction(&eph, t_s);

    assert!((clock_bias_s - expected.bias_s).abs() < 1e-18);
    assert!(fallback);
}

#[test]
fn ppp_solution_epoch_exposes_ecef_position_covariance() {
    let mut filter = PppFilter::new(PppConfig::default());
    filter.seed_receiver_state([6_378_137.0, 10.0, 10.0], 4.0e-6);
    filter.ekf.p[(filter.indices.pos[0], filter.indices.pos[0])] = 4.0;
    filter.ekf.p[(filter.indices.pos[1], filter.indices.pos[1])] = 9.0;
    filter.ekf.p[(filter.indices.pos[2], filter.indices.pos[2])] = 16.0;
    filter.ekf.p[(filter.indices.pos[0], filter.indices.pos[1])] = 0.5;
    filter.ekf.p[(filter.indices.pos[1], filter.indices.pos[0])] = 0.5;

    let solution = filter.solution_epoch(
        7,
        7.0,
        Vec::new(),
        0,
        ppp_stochastic_evidence_from_config(&filter.config),
    );
    let covariance =
        solution.position_covariance_ecef_m2.expect("PPP solution should emit position covariance");

    assert_eq!(covariance[0][0], 4.0);
    assert_eq!(covariance[1][1], 9.0);
    assert_eq!(covariance[2][2], 16.0);
    assert_eq!(covariance[0][1], 0.5);
    assert_eq!(covariance[1][0], 0.5);
    assert!(solution.sigma_e_m.expect("east sigma").is_finite());
    assert!(solution.sigma_n_m.expect("north sigma").is_finite());
    assert!(solution.sigma_u_m.expect("up sigma").is_finite());
    assert!(solution
        .horizontal_error_ellipse_major_axis_m
        .expect("ellipse major axis")
        .is_finite());
    assert!(solution
        .horizontal_error_ellipse_minor_axis_m
        .expect("ellipse minor axis")
        .is_finite());
    assert!(solution.horizontal_error_ellipse_azimuth_deg.expect("ellipse azimuth").is_finite());
}

#[test]
fn iono_free_mode_keeps_one_representative_per_satellite() {
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let l1 = ObsSatellite {
        signal_id: SigId { sat, band: SignalBand::L1, code: SignalCode::Ca },
        pseudorange_m: Meters(20_000_000.0),
        pseudorange_var_m2: 1.0,
        carrier_phase_cycles: Cycles(100.0),
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
            signal: signal_spec_gps_l1_ca(),
            ..ObsMetadata::default()
        },
    };
    let l2 = ObsSatellite {
        signal_id: SigId { sat, band: SignalBand::L2, code: SignalCode::Py },
        metadata: ObsMetadata { signal: signal_spec_gps_l2_py(), ..l1.metadata.clone() },
        ..l1.clone()
    };
    let sats = vec![&l1, &l2];

    let representatives = iono_free_satellite_representatives(&sats);

    assert_eq!(representatives.len(), 1);
    assert_eq!(representatives[0].signal_id.band, SignalBand::L1);
}

#[test]
fn ppp_filter_resolves_iono_free_code_bias_from_both_signals() {
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let l1 = ObsSatellite {
        signal_id: SigId { sat, band: SignalBand::L1, code: SignalCode::Ca },
        pseudorange_m: Meters(20_200_010.0),
        pseudorange_var_m2: 1.0,
        carrier_phase_cycles: Cycles(100.0),
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
            signal: signal_spec_gps_l1_ca(),
            ..ObsMetadata::default()
        },
    };
    let l2 = ObsSatellite {
        signal_id: SigId { sat, band: SignalBand::L2, code: SignalCode::Py },
        pseudorange_m: Meters(20_200_005.0),
        metadata: ObsMetadata { signal: signal_spec_gps_l2_py(), ..l1.metadata.clone() },
        ..l1.clone()
    };
    let obs = ObsEpoch {
        t_rx_s: Seconds(0.0),
        source_time: ReceiverSampleTrace::from_sample_index(0, 1_000.0),
        gps_week: None,
        tow_s: None,
        epoch_idx: 0,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        role: ReceiverRole::Rover,
        sats: vec![l1, l2],
        decision: ObservationEpochDecision::Accepted,
        decision_reason: Some("accepted_observables_present".to_string()),
        manifest: None,
    };
    let measurement = iono_free_code_observation_from_obs(&obs, sat).expect("iono-free code");
    let bias_table = SignalCodeBiases::from_biases([
        CodeBias { sig: measurement.signal_1, bias_m: 2.0 },
        CodeBias { sig: measurement.signal_2, bias_m: -0.5 },
    ]);

    let resolved_bias_m = resolved_iono_free_code_bias_m(&bias_table, measurement);

    assert!((resolved_bias_m - 2.0).abs() > 1.0e-3);
    assert!(resolved_bias_m.is_finite());
}

#[test]
fn ppp_filter_resolves_single_frequency_code_bias_at_epoch_time() {
    struct TimeAwareBiasProvider;

    impl CodeBiasProvider for TimeAwareBiasProvider {
        fn code_bias_m(&self, _sig: SigId) -> Option<f64> {
            None
        }

        fn code_bias_m_at(
            &self,
            _sig: SigId,
            time: Option<bijux_gnss_core::api::GpsTime>,
        ) -> Option<f64> {
            Some(time?.tow_s)
        }
    }

    let signal = SigId {
        sat: SatId { constellation: Constellation::Gps, prn: 7 },
        band: SignalBand::L1,
        code: SignalCode::Ca,
    };

    let resolved_bias_m = resolved_code_bias_m(
        &TimeAwareBiasProvider,
        signal,
        Some(bijux_gnss_core::api::GpsTime { week: 0, tow_s: 123.5 }),
    );

    assert!((resolved_bias_m - 123.5).abs() < f64::EPSILON);
}

#[test]
fn ppp_filter_tracks_phase_windup_by_satellite() {
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let (receiver_x_m, receiver_y_m, receiver_z_m) = geodetic_to_ecef(45.0, 12.0, 80.0);
    let receiver_pos_m = [receiver_x_m, receiver_y_m, receiver_z_m];
    let sat_pos_m = [15_600_000.0, -10_200_000.0, 21_700_000.0];
    let mut states = BTreeMap::new();

    let first = phase_windup_cycles_for_satellite(
        &mut states,
        sat,
        receiver_pos_m,
        sat_pos_m,
        Some(GpsTime { week: 2200, tow_s: 86_400.0 }),
    );
    let second = phase_windup_cycles_for_satellite(
        &mut states,
        sat,
        receiver_pos_m,
        sat_pos_m,
        Some(GpsTime { week: 2200, tow_s: 86_430.0 }),
    );

    assert!(first.is_finite());
    assert!(second.is_finite());
    assert!(states.contains_key(&sat));
    assert!((second - first).abs() < 0.5);
}

#[test]
fn ppp_filter_skips_phase_windup_without_epoch_time() {
    let sat = SatId { constellation: Constellation::Gps, prn: 7 };
    let mut states = BTreeMap::new();

    let windup_cycles = phase_windup_cycles_for_satellite(
        &mut states,
        sat,
        [1.0, 0.0, 0.0],
        [20_200_000.0, 14_000_000.0, 21_700_000.0],
        None,
    );

    assert_eq!(windup_cycles, 0.0);
    assert!(states.is_empty());
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
        .range_correction_m(sat, SignalBand::L1, gps_time, sat_pos_m, receiver_pos_m)
        .expect("single-frequency antenna correction");
    let actual = single_frequency_antenna_range_correction_m(
        Some(&calibrations),
        None,
        None,
        sat,
        SignalBand::L1,
        gps_time,
        sat_pos_m,
        receiver_pos_m,
        85.0,
        None,
    );

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

    let actual = single_frequency_antenna_range_correction_m(
        Some(&satellite_calibrations),
        Some(receiver_antenna_type),
        Some(&receiver_calibrations),
        sat,
        SignalBand::L1,
        gps_time,
        sat_pos_m,
        receiver_pos_m,
        85.0,
        None,
    );

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
            SignalBand::L1,
            l1.carrier_hz.value(),
            SignalBand::L2,
            l2.carrier_hz.value(),
            gps_time,
            sat_pos_m,
            receiver_pos_m,
        )
        .expect("iono-free antenna correction");
    let actual = iono_free_antenna_range_correction_m(
        Some(&calibrations),
        None,
        None,
        sat,
        SignalBand::L1,
        l1.carrier_hz.value(),
        SignalBand::L2,
        l2.carrier_hz.value(),
        gps_time,
        sat_pos_m,
        receiver_pos_m,
        85.0,
        None,
    );

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
            SignalBand::L1,
            l1.carrier_hz.value(),
            SignalBand::L2,
            l2.carrier_hz.value(),
            gps_time,
            sat_pos_m,
            receiver_pos_m,
            85.0,
            None,
        )
        .expect("satellite antenna correction")
        + receiver_calibrations
            .iono_free_range_correction_with_phase_variation_m(
                receiver_antenna_type,
                SignalBand::L1,
                l1.carrier_hz.value(),
                SignalBand::L2,
                l2.carrier_hz.value(),
                gps_time,
                receiver_pos_m,
                sat_pos_m,
                85.0,
                None,
            )
            .expect("receiver antenna correction");
    let actual = iono_free_antenna_range_correction_m(
        Some(&satellite_calibrations),
        Some(receiver_antenna_type),
        Some(&receiver_calibrations),
        sat,
        SignalBand::L1,
        l1.carrier_hz.value(),
        SignalBand::L2,
        l2.carrier_hz.value(),
        gps_time,
        sat_pos_m,
        receiver_pos_m,
        85.0,
        None,
    );

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
            gps_time,
            receiver_pos_m,
            sat_pos_m,
        )
        .expect("single-frequency receiver antenna correction");
    let actual = single_frequency_antenna_range_correction_m(
        None,
        Some(receiver_antenna_type),
        Some(&calibrations),
        SatId { constellation: Constellation::Gps, prn: 7 },
        SignalBand::L1,
        gps_time,
        sat_pos_m,
        receiver_pos_m,
        85.0,
        None,
    );

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
            SignalBand::L1,
            l1.carrier_hz.value(),
            SignalBand::L2,
            l2.carrier_hz.value(),
            gps_time,
            receiver_pos_m,
            sat_pos_m,
        )
        .expect("iono-free receiver antenna correction");
    let actual = iono_free_antenna_range_correction_m(
        None,
        Some(receiver_antenna_type),
        Some(&calibrations),
        SatId { constellation: Constellation::Gps, prn: 7 },
        SignalBand::L1,
        l1.carrier_hz.value(),
        SignalBand::L2,
        l2.carrier_hz.value(),
        gps_time,
        sat_pos_m,
        receiver_pos_m,
        85.0,
        None,
    );

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

    let aoad = single_frequency_antenna_range_correction_m(
        None,
        Some("AOAD/M_T NONE"),
        Some(&calibrations),
        SatId { constellation: Constellation::Gps, prn: 7 },
        SignalBand::L1,
        gps_time,
        sat_pos_m,
        receiver_pos_m,
        85.0,
        None,
    );
    let trimble = single_frequency_antenna_range_correction_m(
        None,
        Some("TRM57971.00 NONE"),
        Some(&calibrations),
        SatId { constellation: Constellation::Gps, prn: 7 },
        SignalBand::L1,
        gps_time,
        sat_pos_m,
        receiver_pos_m,
        85.0,
        None,
    );

    assert!((trimble - aoad).abs() > 1.0e-6);
}
