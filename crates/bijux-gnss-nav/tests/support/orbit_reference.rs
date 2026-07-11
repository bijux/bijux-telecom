#![allow(dead_code, missing_docs)]

use bijux_gnss_core::api::SatId;
use bijux_gnss_nav::api::{
    sat_state_gps_l1ca, BroadcastProductsProvider, GpsSatState, ProductDiagnostics,
    ProductsProvider,
};

use super::broadcast_reference::{
    gps_prn1_20220513_fixture, gps_prn2_20220514_fixture, BroadcastReferenceFixture,
};

#[derive(Debug, Clone, Copy)]
pub struct OrbitReferenceSample {
    pub label: &'static str,
    pub sat: SatId,
    pub transmit_tow_s: f64,
    pub reference_t_s: f64,
    pub precise_ecef_m: (f64, f64, f64),
}

#[derive(Debug, Clone, Copy)]
pub struct OrbitReferenceError {
    pub label: &'static str,
    pub sat: SatId,
    pub transmit_tow_s: f64,
    pub reference_t_s: f64,
    pub dx_m: f64,
    pub dy_m: f64,
    pub dz_m: f64,
    pub position_error_m: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct OrbitReferenceSummary {
    pub sample_count: usize,
    pub max_position_error_m: f64,
    pub rms_position_error_m: f64,
}

#[derive(Debug, Clone)]
pub struct ProviderOrbitReferenceComparison {
    pub errors: Vec<OrbitReferenceError>,
    pub fallbacks: Vec<String>,
}

pub fn broadcast_reference_fixtures() -> [BroadcastReferenceFixture; 2] {
    [gps_prn1_20220513_fixture(), gps_prn2_20220514_fixture()]
}

pub fn orbit_reference_samples(fixture: &BroadcastReferenceFixture) -> Vec<OrbitReferenceSample> {
    fixture
        .transmit_times_s
        .iter()
        .copied()
        .zip(fixture.reference_times_s.iter().copied())
        .map(|(transmit_tow_s, reference_t_s)| {
            let precise = fixture
                .sp3
                .sat_state(fixture.sat, reference_t_s)
                .expect("SP3 state for orbit reference sample");
            OrbitReferenceSample {
                label: fixture.label,
                sat: fixture.sat,
                transmit_tow_s,
                reference_t_s,
                precise_ecef_m: (precise.x_m, precise.y_m, precise.z_m),
            }
        })
        .collect()
}

pub fn direct_broadcast_orbit_errors(
    fixture: &BroadcastReferenceFixture,
) -> Vec<OrbitReferenceError> {
    orbit_reference_samples(fixture)
        .into_iter()
        .map(|sample| {
            sample.compare_state(&sat_state_gps_l1ca(
                &fixture.ephemeris,
                sample.transmit_tow_s,
                0.0,
            ))
        })
        .collect()
}

pub fn provider_broadcast_orbit_errors(
    fixture: &BroadcastReferenceFixture,
) -> Vec<OrbitReferenceError> {
    provider_broadcast_orbit_comparison(fixture).errors
}

pub fn provider_broadcast_orbit_comparison(
    fixture: &BroadcastReferenceFixture,
) -> ProviderOrbitReferenceComparison {
    let provider = BroadcastProductsProvider::new(vec![fixture.ephemeris.clone()]);
    let mut diagnostics = ProductDiagnostics::new();

    let errors = orbit_reference_samples(fixture)
        .into_iter()
        .map(|sample| {
            let state = provider
                .sat_state(sample.sat, sample.transmit_tow_s, &mut diagnostics)
                .expect("broadcast provider state for orbit reference sample");
            sample.compare_state(&state)
        })
        .collect();

    ProviderOrbitReferenceComparison { errors, fallbacks: diagnostics.fallbacks }
}

pub fn summarize_orbit_errors(errors: &[OrbitReferenceError]) -> OrbitReferenceSummary {
    assert!(!errors.is_empty(), "orbit reference summary requires at least one sample");
    let sample_count = errors.len();
    let max_position_error_m =
        errors.iter().map(|error| error.position_error_m).fold(f64::NEG_INFINITY, f64::max);
    let mean_square_error_m2 =
        errors.iter().map(|error| error.position_error_m.powi(2)).sum::<f64>()
            / sample_count as f64;

    OrbitReferenceSummary {
        sample_count,
        max_position_error_m,
        rms_position_error_m: mean_square_error_m2.sqrt(),
    }
}

impl OrbitReferenceSample {
    pub fn compare_state(&self, state: &GpsSatState) -> OrbitReferenceError {
        let dx_m = state.x_m - self.precise_ecef_m.0;
        let dy_m = state.y_m - self.precise_ecef_m.1;
        let dz_m = state.z_m - self.precise_ecef_m.2;
        OrbitReferenceError {
            label: self.label,
            sat: self.sat,
            transmit_tow_s: self.transmit_tow_s,
            reference_t_s: self.reference_t_s,
            dx_m,
            dy_m,
            dz_m,
            position_error_m: (dx_m * dx_m + dy_m * dy_m + dz_m * dz_m).sqrt(),
        }
    }
}
