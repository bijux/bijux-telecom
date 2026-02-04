#![allow(missing_docs)]

use bijux_gnss_core::api::SatId;

use crate::orbits::gps::{sat_state_gps_l1ca, GpsEphemeris, GpsSatState};

use crate::formats::clk::ClkProvider;
use crate::formats::sp3::Sp3Provider;

#[derive(Debug, Clone)]
pub struct ProductDiagnostics {
    pub fallbacks: Vec<String>,
}

impl ProductDiagnostics {
    pub fn new() -> Self {
        Self {
            fallbacks: Vec::new(),
        }
    }

    pub fn fallback(&mut self, msg: impl Into<String>) {
        self.fallbacks.push(msg.into());
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
    fn clock_bias_s(&self, sat: SatId, t_s: f64, diag: &mut ProductDiagnostics) -> Option<f64>;
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
}

impl ProductsProvider for BroadcastProductsProvider {
    fn sat_state(
        &self,
        sat: SatId,
        t_s: f64,
        _diag: &mut ProductDiagnostics,
    ) -> Option<GpsSatState> {
        let eph = self.ephs.iter().find(|e| e.sat == sat)?;
        Some(sat_state_gps_l1ca(eph, t_s, 0.0))
    }

    fn clock_bias_s(&self, sat: SatId, t_s: f64, _diag: &mut ProductDiagnostics) -> Option<f64> {
        let eph = self.ephs.iter().find(|e| e.sat == sat)?;
        Some(sat_state_gps_l1ca(eph, t_s, 0.0).clock_bias_s)
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
        Self {
            broadcast,
            sp3: None,
            clk: None,
        }
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

    fn clock_bias_s(&self, sat: SatId, t_s: f64, diag: &mut ProductDiagnostics) -> Option<f64> {
        if let Some(clk) = &self.clk {
            if let Some((start, end)) = clk.coverage_s(sat) {
                if t_s >= start && t_s <= end {
                    if let Some(bias) = clk.bias_s(sat, t_s) {
                        return Some(bias);
                    }
                } else {
                    diag.fallback(format!("CLK out of coverage for {:?}", sat));
                }
            }
        }
        diag.fallback(format!("CLK missing for {:?}, using broadcast", sat));
        self.broadcast.clock_bias_s(sat, t_s, diag)
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
