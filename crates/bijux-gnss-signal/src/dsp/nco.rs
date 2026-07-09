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
        Self::with_phase(freq_hz, sample_rate_hz, 0.0)
    }

    pub fn with_phase(freq_hz: f64, sample_rate_hz: f64, phase_rad: f64) -> Self {
        let phase_step_rad = TAU * freq_hz / sample_rate_hz;
        Self { phase_rad: wrap_phase_rad(phase_rad), phase_step_rad }
    }

    pub fn from_sample_index(
        freq_hz: f64,
        sample_rate_hz: f64,
        start_sample_index: u64,
        phase_offset_rad: f64,
    ) -> Self {
        let start_phase_rad = wrap_phase_rad(
            phase_offset_rad + phase_step_rad(freq_hz, sample_rate_hz) * start_sample_index as f64,
        );
        Self::with_phase(freq_hz, sample_rate_hz, start_phase_rad)
    }

    pub fn set_freq(&mut self, freq_hz: f64, sample_rate_hz: f64) {
        self.phase_step_rad = phase_step_rad(freq_hz, sample_rate_hz);
    }

    pub fn set_phase(&mut self, phase_rad: f64) {
        self.phase_rad = wrap_phase_rad(phase_rad);
    }

    pub fn phase_rad(&self) -> f64 {
        self.phase_rad
    }

    pub fn next_sin_cos(&mut self) -> (f64, f64) {
        let sin = self.phase_rad.sin();
        let cos = self.phase_rad.cos();
        self.phase_rad = wrap_phase_rad(self.phase_rad + self.phase_step_rad);
        (sin, cos)
    }
}

fn phase_step_rad(freq_hz: f64, sample_rate_hz: f64) -> f64 {
    TAU * freq_hz / sample_rate_hz
}

fn wrap_phase_rad(phase_rad: f64) -> f64 {
    phase_rad.rem_euclid(TAU)
}
