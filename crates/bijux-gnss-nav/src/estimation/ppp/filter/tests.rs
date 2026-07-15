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
#[path = "tests/antenna_calibration.rs"]
mod antenna_calibration;
#[path = "tests/correction_models.rs"]
mod correction_models;
#[path = "tests/precise_clock_selection.rs"]
mod precise_clock_selection;
#[path = "tests/precise_product_policy.rs"]
mod precise_product_policy;
#[path = "tests/state_lifecycle.rs"]
mod state_lifecycle;
#[path = "tests/static_tide_models.rs"]
mod static_tide_models;

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
