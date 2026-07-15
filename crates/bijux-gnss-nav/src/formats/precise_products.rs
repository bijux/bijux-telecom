#![allow(missing_docs)]

use bijux_gnss_core::api::SatId;

use crate::corrections::biases::CodeBiasProvider;
use crate::corrections::broadcast_group_delay::gps_broadcast_group_delay_code_bias_m;
use crate::formats::bias_sinex::BiasSinexProvider;
use crate::orbits::gps::{
    gps_ephemeris_age, gps_satellite_clock_correction, sat_state_gps_l1ca, select_best_ephemeris,
    GpsEphemeris, GpsSatState, GpsSatelliteClockCorrection,
};

use crate::formats::clk::{ClkInterpolationSummary, ClkProvider};
use crate::formats::sp3::{Sp3InterpolationSummary, Sp3Provider};
use crate::orbits::satellite_uncertainty::{
    clk_sigma_uncertainty, SatelliteClockUncertaintySource,
};

#[derive(Debug, Clone)]
pub struct ProductDiagnostics {
    pub fallbacks: Vec<String>,
    pub sp3_interpolation_summary: Option<Sp3InterpolationSummary>,
    pub clk_interpolation_summary: Option<ClkInterpolationSummary>,
}

impl ProductDiagnostics {
    pub fn new() -> Self {
        Self {
            fallbacks: Vec::new(),
            sp3_interpolation_summary: None,
            clk_interpolation_summary: None,
        }
    }

    pub fn fallback(&mut self, msg: impl Into<String>) {
        self.fallbacks.push(msg.into());
    }

    pub fn precise_orbit_interpolation(&mut self, summary: Sp3InterpolationSummary) {
        self.sp3_interpolation_summary = Some(summary);
    }

    pub fn precise_clock_interpolation(&mut self, summary: ClkInterpolationSummary) {
        self.clk_interpolation_summary = Some(summary);
    }
}

impl Default for ProductDiagnostics {
    fn default() -> Self {
        Self::new()
    }
}

pub trait ProductsProvider {
    fn sat_state(&self, sat: SatId, t_s: f64, diag: &mut ProductDiagnostics)
        -> Option<GpsSatState>;
    fn clock_correction(
        &self,
        sat: SatId,
        t_s: f64,
        diag: &mut ProductDiagnostics,
    ) -> Option<GpsSatelliteClockCorrection>;
    fn coverage_s(&self, sat: SatId) -> Option<(f64, f64)>;
}

#[derive(Debug, Clone)]
pub struct BroadcastProductsProvider {
    ephs: Vec<GpsEphemeris>,
}

impl BroadcastProductsProvider {
    pub fn new(ephs: Vec<GpsEphemeris>) -> Self {
        Self { ephs }
    }

    fn active_ephemeris(
        &self,
        sat: SatId,
        t_s: f64,
        diag: &mut ProductDiagnostics,
    ) -> Option<&GpsEphemeris> {
        let eph = select_best_ephemeris(&self.ephs, sat, t_s)?;
        let age = gps_ephemeris_age(eph, t_s);
        if age.is_stale() {
            diag.fallback(format!(
                "broadcast ephemeris stale for {:?} at {:.3}s (toe_age_s={:.3}, toc_age_s={:.3}, limit_s={:.3})",
                sat, t_s, age.toe_age_s, age.toc_age_s, age.max_age_s
            ));
            return None;
        }
        Some(eph)
    }
}

impl ProductsProvider for BroadcastProductsProvider {
    fn sat_state(
        &self,
        sat: SatId,
        t_s: f64,
        diag: &mut ProductDiagnostics,
    ) -> Option<GpsSatState> {
        let eph = self.active_ephemeris(sat, t_s, diag)?;
        Some(sat_state_gps_l1ca(eph, t_s, 0.0))
    }

    fn clock_correction(
        &self,
        sat: SatId,
        t_s: f64,
        diag: &mut ProductDiagnostics,
    ) -> Option<GpsSatelliteClockCorrection> {
        let eph = self.active_ephemeris(sat, t_s, diag)?;
        Some(gps_satellite_clock_correction(eph, t_s))
    }

    fn coverage_s(&self, _sat: SatId) -> Option<(f64, f64)> {
        None
    }
}

#[derive(Debug, Clone)]
pub struct Products {
    pub broadcast: BroadcastProductsProvider,
    pub sp3: Option<Sp3Provider>,
    pub clk: Option<ClkProvider>,
    pub dcb: Option<BiasSinexProvider>,
}

impl Products {
    pub fn new(broadcast: BroadcastProductsProvider) -> Self {
        Self { broadcast, sp3: None, clk: None, dcb: None }
    }

    pub fn with_sp3(mut self, sp3: Sp3Provider) -> Self {
        self.sp3 = Some(sp3);
        self
    }

    pub fn with_clk(mut self, clk: ClkProvider) -> Self {
        self.clk = Some(clk);
        self
    }

    pub fn with_dcb(mut self, dcb: BiasSinexProvider) -> Self {
        self.dcb = Some(dcb);
        self
    }

    fn attach_clock_uncertainty(
        &self,
        mut state: GpsSatState,
        sat: SatId,
        t_s: f64,
    ) -> GpsSatState {
        if let Some(clk) = &self.clk {
            state.uncertainty = state.uncertainty.with_clock_sigma_s(
                clk_sigma_uncertainty(clk, sat, t_s),
                SatelliteClockUncertaintySource::ClkSigma,
            );
        }
        state
    }
}

impl ProductsProvider for Products {
    fn sat_state(
        &self,
        sat: SatId,
        t_s: f64,
        diag: &mut ProductDiagnostics,
    ) -> Option<GpsSatState> {
        if let Some(sp3) = &self.sp3 {
            if let Some((start, end)) = sp3.coverage_s(sat) {
                if t_s >= start && t_s <= end {
                    if let Some(state) = sp3.sat_state(sat, t_s) {
                        if let Some(summary) = sp3.interpolation_summary(sat) {
                            diag.precise_orbit_interpolation(summary);
                        }
                        return Some(self.attach_clock_uncertainty(state, sat, t_s));
                    }
                    diag.fallback(format!(
                        "SP3 unusable for {:?} at {:.3}s because interpolation crossed a gap, prediction, maneuver, or event",
                        sat, t_s
                    ));
                } else {
                    diag.fallback(format!("SP3 out of coverage for {:?}", sat));
                }
            }
        }
        diag.fallback(format!("SP3 missing for {:?}, using broadcast", sat));
        self.broadcast
            .sat_state(sat, t_s, diag)
            .map(|state| self.attach_clock_uncertainty(state, sat, t_s))
    }

    fn clock_correction(
        &self,
        sat: SatId,
        t_s: f64,
        diag: &mut ProductDiagnostics,
    ) -> Option<GpsSatelliteClockCorrection> {
        if let Some(clk) = &self.clk {
            if let Some((start, end)) = clk.coverage_s(sat) {
                if t_s >= start && t_s <= end {
                    if let Some(correction) = clk.clock_correction(sat, t_s) {
                        if let Some(summary) = clk.interpolation_summary(sat) {
                            diag.precise_clock_interpolation(summary);
                        }
                        return Some(correction);
                    }
                    diag.fallback(format!(
                        "CLK unusable for {:?} at {:.3}s because interpolation crossed a gap or clock jump",
                        sat, t_s
                    ));
                } else {
                    diag.fallback(format!("CLK out of coverage for {:?}", sat));
                }
            }
        }
        diag.fallback(format!("CLK missing for {:?}, using broadcast", sat));
        self.broadcast.clock_correction(sat, t_s, diag)
    }

    fn coverage_s(&self, sat: SatId) -> Option<(f64, f64)> {
        if let Some(sp3) = &self.sp3 {
            if let Some(range) = sp3.coverage_s(sat) {
                return Some(range);
            }
        }
        None
    }
}

impl CodeBiasProvider for Products {
    fn code_bias_m(&self, sig: bijux_gnss_core::api::SigId) -> Option<f64> {
        combine_code_bias_terms(
            self.broadcast.code_bias_m(sig),
            self.dcb.as_ref().and_then(|provider| provider.code_bias_m(sig)),
        )
    }

    fn code_bias_m_at(
        &self,
        sig: bijux_gnss_core::api::SigId,
        time: Option<bijux_gnss_core::api::GpsTime>,
    ) -> Option<f64> {
        combine_code_bias_terms(
            self.broadcast.code_bias_m_at(sig, time),
            self.dcb.as_ref().and_then(|provider| provider.code_bias_m_at(sig, time)),
        )
    }
}

impl CodeBiasProvider for BroadcastProductsProvider {
    fn code_bias_m(&self, sig: bijux_gnss_core::api::SigId) -> Option<f64> {
        let ephemeris = self.ephs.iter().filter(|ephemeris| ephemeris.sat == sig.sat).max_by(
            |left, right| {
                left.toc_s.total_cmp(&right.toc_s).then_with(|| left.toe_s.total_cmp(&right.toe_s))
            },
        )?;
        gps_broadcast_group_delay_code_bias_m(sig, ephemeris)
    }

    fn code_bias_m_at(
        &self,
        sig: bijux_gnss_core::api::SigId,
        time: Option<bijux_gnss_core::api::GpsTime>,
    ) -> Option<f64> {
        let time = time?;
        let ephemeris = select_best_ephemeris(&self.ephs, sig.sat, time.tow_s)?;
        gps_ephemeris_age(ephemeris, time.tow_s)
            .is_valid()
            .then(|| gps_broadcast_group_delay_code_bias_m(sig, ephemeris))
            .flatten()
    }
}

fn combine_code_bias_terms(left: Option<f64>, right: Option<f64>) -> Option<f64> {
    match (left, right) {
        (Some(left), Some(right)) => Some(left + right),
        (Some(left), None) => Some(left),
        (None, Some(right)) => Some(right),
        (None, None) => None,
    }
}
