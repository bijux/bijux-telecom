#![allow(missing_docs)]

pub mod atmosphere;
pub mod biases;
pub mod broadcast_group_delay;
pub mod broadcast_ionosphere_residuals;
pub mod combinations;
pub(crate) mod dual_frequency;
pub mod geometry_free;
pub mod iono_free_code;
pub mod iono_free_phase;
pub mod measured_ionosphere;
pub mod melbourne_wubbena;
pub mod narrow_lane;
pub mod phase_windup;

/// Resolved per-epoch correction inputs for measurement modeling.
#[derive(Debug, Clone, Default)]
pub struct CorrectionContext {
    /// Receiver displacement in ECEF meters from solid Earth and loading effects.
    pub earth_tide_m: [f64; 3],
}

impl CorrectionContext {
    /// Creates a context with a resolved receiver displacement.
    pub fn with_earth_tide_m(earth_tide_m: [f64; 3]) -> Self {
        Self { earth_tide_m }
    }

    /// Accumulates an additional receiver displacement into the context.
    pub fn add_earth_tide_m(&mut self, delta_m: [f64; 3]) {
        self.earth_tide_m[0] += delta_m[0];
        self.earth_tide_m[1] += delta_m[1];
        self.earth_tide_m[2] += delta_m[2];
    }
}

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

pub fn compute_corrections(ctx: &CorrectionContext) -> Corrections {
    Corrections { earth_tide_m: ctx.earth_tide_m, ..Corrections::default() }
}

#[cfg(test)]
mod tests {
    use super::{compute_corrections, CorrectionContext};

    #[test]
    fn correction_context_accumulates_earth_tide_displacement() {
        let mut context = CorrectionContext::with_earth_tide_m([0.01, -0.02, 0.03]);

        context.add_earth_tide_m([0.04, 0.01, -0.05]);

        assert!((context.earth_tide_m[0] - 0.05).abs() < 1.0e-12);
        assert!((context.earth_tide_m[1] + 0.01).abs() < 1.0e-12);
        assert!((context.earth_tide_m[2] + 0.02).abs() < 1.0e-12);
    }

    #[test]
    fn compute_corrections_preserves_earth_tide_displacement() {
        let corrections =
            compute_corrections(&CorrectionContext::with_earth_tide_m([0.012, -0.004, 0.031]));

        assert_eq!(corrections.earth_tide_m, [0.012, -0.004, 0.031]);
    }
}
