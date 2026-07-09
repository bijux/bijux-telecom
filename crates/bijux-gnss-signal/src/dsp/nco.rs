#![allow(missing_docs)]

use std::f64::consts::TAU;

/// Numerically controlled oscillator for mixing at a given frequency.
#[derive(Debug, Clone)]
pub struct Nco {
    phase_offset_rad: f64,
    freq_hz: f64,
    sample_rate_hz: f64,
    sample_index: u64,
}

impl Nco {
    pub fn new(freq_hz: f64, sample_rate_hz: f64) -> Self {
        Self::with_phase(freq_hz, sample_rate_hz, 0.0)
    }

    pub fn with_phase(freq_hz: f64, sample_rate_hz: f64, phase_rad: f64) -> Self {
        Self {
            phase_offset_rad: wrap_phase_rad(phase_rad),
            freq_hz,
            sample_rate_hz,
            sample_index: 0,
        }
    }

    pub fn from_sample_index(
        freq_hz: f64,
        sample_rate_hz: f64,
        start_sample_index: u64,
        phase_offset_rad: f64,
    ) -> Self {
        Self {
            phase_offset_rad: wrap_phase_rad(phase_offset_rad),
            freq_hz,
            sample_rate_hz,
            sample_index: start_sample_index,
        }
    }

    pub fn set_freq(&mut self, freq_hz: f64, sample_rate_hz: f64) {
        let current_phase_rad = self.phase_rad();
        self.freq_hz = freq_hz;
        self.sample_rate_hz = sample_rate_hz;
        self.phase_offset_rad = wrap_phase_rad(
            current_phase_rad
                - phase_advance_rad(self.freq_hz, self.sample_rate_hz, self.sample_index),
        );
    }

    pub fn set_phase(&mut self, phase_rad: f64) {
        self.phase_offset_rad = wrap_phase_rad(
            phase_rad - phase_advance_rad(self.freq_hz, self.sample_rate_hz, self.sample_index),
        );
    }

    pub fn phase_rad(&self) -> f64 {
        wrap_phase_rad(
            self.phase_offset_rad
                + phase_advance_rad(self.freq_hz, self.sample_rate_hz, self.sample_index),
        )
    }

    pub fn next_sin_cos(&mut self) -> (f64, f64) {
        let phase_rad = self.phase_rad();
        let sin = phase_rad.sin();
        let cos = phase_rad.cos();
        self.sample_index = self.sample_index.saturating_add(1);
        (sin, cos)
    }
}

fn phase_advance_rad(freq_hz: f64, sample_rate_hz: f64, sample_index: u64) -> f64 {
    let elapsed_seconds = sample_index as f64 / sample_rate_hz;
    TAU * freq_hz * elapsed_seconds
}

fn wrap_phase_rad(phase_rad: f64) -> f64 {
    phase_rad.rem_euclid(TAU)
}
