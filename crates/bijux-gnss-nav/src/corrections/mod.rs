mod atmosphere;
mod biases;
mod combinations;

pub use atmosphere::*;
pub use biases::*;
pub use combinations::*;

#[derive(Debug, Clone)]
pub struct CorrectionContext;

#[derive(Debug, Clone)]
pub struct Corrections {
    pub receiver_apc_m: [f64; 3],
    pub satellite_apc_m: [f64; 3],
    pub phase_windup_cycles: f64,
    pub relativity_s: f64,
    pub earth_tide_m: [f64; 3],
}

impl Default for Corrections {
    fn default() -> Self {
        Self {
            receiver_apc_m: [0.0, 0.0, 0.0],
            satellite_apc_m: [0.0, 0.0, 0.0],
            phase_windup_cycles: 0.0,
            relativity_s: 0.0,
            earth_tide_m: [0.0, 0.0, 0.0],
        }
    }
}

pub fn compute_corrections(_ctx: &CorrectionContext) -> Corrections {
    Corrections::default()
}
