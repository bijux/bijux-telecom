#![allow(missing_docs)]

use std::f64::consts::TAU;

/// Numerically controlled oscillator for mixing at a given frequency.
#[derive(Debug, Clone)]
pub struct Nco {
    phase_rad: f64,
    phase_step_rad: f64,
}

impl Nco {
    pub fn new(freq_hz: f64, sample_rate_hz: f64) -> Self {
        let phase_step_rad = TAU * freq_hz / sample_rate_hz;
        Self {
            phase_rad: 0.0,
            phase_step_rad,
        }
    }

    pub fn set_freq(&mut self, freq_hz: f64, sample_rate_hz: f64) {
        self.phase_step_rad = TAU * freq_hz / sample_rate_hz;
    }

    pub fn next_sin_cos(&mut self) -> (f64, f64) {
        let sin = self.phase_rad.sin();
        let cos = self.phase_rad.cos();
        self.phase_rad = (self.phase_rad + self.phase_step_rad) % TAU;
        (sin, cos)
    }
}
