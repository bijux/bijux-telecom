#![allow(missing_docs)]

use bijux_gnss_core::api::SatId;

use crate::orbits::gps::{
    gps_ephemeris_age, gps_satellite_clock_correction, sat_state_gps_l1ca, select_best_ephemeris,
    GpsEphemeris, GpsSatState, GpsSatelliteClockCorrection,
};

use crate::formats::clk::{ClkInterpolationSummary, ClkProvider};
use crate::formats::sp3::{Sp3InterpolationSummary, Sp3Provider};

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
}

impl Products {
    pub fn new(broadcast: BroadcastProductsProvider) -> Self {
        Self { broadcast, sp3: None, clk: None }
    }

    pub fn with_sp3(mut self, sp3: Sp3Provider) -> Self {
        self.sp3 = Some(sp3);
        self
    }

    pub fn with_clk(mut self, clk: ClkProvider) -> Self {
        self.clk = Some(clk);
        self
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
                        return Some(state);
                    }
                } else {
                    diag.fallback(format!("SP3 out of coverage for {:?}", sat));
                }
            }
        }
        diag.fallback(format!("SP3 missing for {:?}, using broadcast", sat));
        self.broadcast.sat_state(sat, t_s, diag)
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
                    if let Some(bias) = clk.bias_s(sat, t_s) {
                        if let Some(summary) = clk.interpolation_summary(sat) {
                            diag.precise_clock_interpolation(summary);
                        }
                        return Some(GpsSatelliteClockCorrection::from_bias_s(bias));
                    }
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
