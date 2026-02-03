#![allow(missing_docs)]

use num_complex::Complex;

use bijux_gnss_core::{Constellation, SampleClock, SampleTime, SamplesFrame, SatId, TrackEpoch};

use crate::logging;
use crate::ReceiverConfig;
use bijux_gnss_core::Sample;
use bijux_gnss_signal::samples_per_code;
use bijux_gnss_signal::Nco;
use bijux_gnss_signal::{generate_ca_code, Prn};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChannelState {
    Idle,
    Acquired,
    PullIn,
    Tracking,
    Lost,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChannelEvent {
    Acquire,
    PullIn,
    Track,
    Lose,
    Reset,
}

#[derive(Debug, Clone)]
pub struct Channel {
    pub id: u8,
    pub state: ChannelState,
}

impl Channel {
    pub fn new(id: u8) -> Self {
        Self {
            id,
            state: ChannelState::Idle,
        }
    }

    pub fn apply(&mut self, event: ChannelEvent) {
        let next = match (self.state, event) {
            (ChannelState::Idle, ChannelEvent::Acquire) => ChannelState::Acquired,
            (ChannelState::Acquired, ChannelEvent::PullIn) => ChannelState::PullIn,
            (ChannelState::PullIn, ChannelEvent::Track) => ChannelState::Tracking,
            (_, ChannelEvent::Lose) => ChannelState::Lost,
            (_, ChannelEvent::Reset) => ChannelState::Idle,
            (state, _) => state,
        };
        self.state = Tracking::transition_state(self.id, self.state, next);
    }
}

#[derive(Debug, Clone)]
pub struct CorrelatorOutput {
    pub early: Complex<f32>,
    pub prompt: Complex<f32>,
    pub late: Complex<f32>,
}

#[derive(Debug, Clone)]
pub struct TrackingResult {
    pub sat: SatId,
    pub carrier_hz: f64,
    pub code_phase_samples: f64,
    pub epochs: Vec<TrackEpoch>,
}

#[derive(Debug, Clone)]
struct LoopState {
    carrier_hz: f64,
    code_rate_hz: f64,
    code_phase_samples: f64,
    prev_prompt: Option<Complex<f32>>,
    state: ChannelState,
}

/// Tracking engine with basic E/P/L correlation per epoch.
pub struct Tracking {
    config: ReceiverConfig,
}

impl Tracking {
    pub fn new(config: ReceiverConfig) -> Self {
        Self { config }
    }

    pub fn transition_state(channel: u8, from: ChannelState, to: ChannelState) -> ChannelState {
        if from != to {
            logging::channel_state_change(channel, from, to);
        }
        to
    }

    pub fn correlate_epoch(
        &self,
        frame: &SamplesFrame,
        sat: SatId,
        carrier_freq_hz: f64,
        code_phase_samples: f64,
        early_late_spacing_chips: f64,
    ) -> CorrelatorOutput {
        let samples_per_code = samples_per_code(
            self.config.sampling_freq_hz,
            self.config.code_freq_basis_hz,
            self.config.code_length,
        );
        let n = samples_per_code.min(frame.len());
        let code = generate_ca_code(Prn(sat.prn));
        let samples_per_chip = n as f64 / code.len() as f64;
        let early_offset = code_phase_samples - early_late_spacing_chips * samples_per_chip;
        let late_offset = code_phase_samples + early_late_spacing_chips * samples_per_chip;

        let mut nco = Nco::new(-carrier_freq_hz, self.config.sampling_freq_hz);
        let mut early = Complex::new(0.0f32, 0.0f32);
        let mut prompt = Complex::new(0.0f32, 0.0f32);
        let mut late = Complex::new(0.0f32, 0.0f32);

        for i in 0..n {
            let (sin, cos) = nco.next_sin_cos();
            let rot = Complex::new(cos as f32, -sin as f32);
            let mixed = frame.iq[i] * rot;

            let early_code = code_at(&code, samples_per_chip, i as f64 + early_offset);
            let prompt_code = code_at(&code, samples_per_chip, i as f64 + code_phase_samples);
            let late_code = code_at(&code, samples_per_chip, i as f64 + late_offset);

            early += mixed * early_code;
            prompt += mixed * prompt_code;
            late += mixed * late_code;
        }

        CorrelatorOutput {
            early,
            prompt,
            late,
        }
    }

    pub fn track_epoch(
        &self,
        frame: &SamplesFrame,
        sat: SatId,
        carrier_freq_hz: f64,
        code_phase_samples: f64,
        early_late_spacing_chips: f64,
    ) -> (TrackEpoch, CorrelatorOutput) {
        let clock = SampleClock::new(self.config.sampling_freq_hz);
        let epoch = clock.epoch_from_samples(frame.t0.sample_index);
        let correlator = self.correlate_epoch(
            frame,
            sat,
            carrier_freq_hz,
            code_phase_samples,
            early_late_spacing_chips,
        );
        let cn0_dbhz = estimate_cn0_dbhz(correlator.prompt, correlator.early + correlator.late);
        logging::lock_status(0, correlator.prompt.norm() > 0.0);
        let track_epoch = TrackEpoch {
            epoch,
            sample_index: frame.t0.sample_index,
            sat,
            prompt_i: correlator.prompt.re,
            prompt_q: correlator.prompt.im,
            carrier_hz: carrier_freq_hz,
            code_rate_hz: self.config.code_freq_basis_hz,
            code_phase_samples,
            lock: correlator.prompt.norm() > 0.0,
            cn0_dbhz,
            pll_lock: false,
            dll_lock: false,
            fll_lock: false,
            cycle_slip: false,
            nav_bit_lock: false,
            dll_err: 0.0,
            pll_err: 0.0,
            fll_err: 0.0,
        };
        (track_epoch, correlator)
    }

    pub fn run(&self, samples: &[Sample]) -> Vec<TrackingResult> {
        if samples.is_empty() {
            return Vec::new();
        }
        let frame = SamplesFrame::new(
            SampleTime {
                sample_index: 0,
                sample_rate_hz: self.config.sampling_freq_hz,
            },
            1.0 / self.config.sampling_freq_hz,
            samples.to_vec(),
        );
        let epochs = self.track_epochs(
            &frame,
            SatId {
                constellation: Constellation::Gps,
                prn: 1,
            },
            0.0,
            0.0,
            0.5,
            5,
        );
        vec![TrackingResult {
            sat: SatId {
                constellation: Constellation::Gps,
                prn: 1,
            },
            carrier_hz: 0.0,
            code_phase_samples: 0.0,
            epochs,
        }]
    }

    pub fn track_from_acquisition(
        &self,
        frame: &SamplesFrame,
        acquisitions: &[bijux_gnss_core::AcqResult],
        band: bijux_gnss_core::SignalBand,
    ) -> Vec<TrackingResult> {
        let params = self.config.tracking_params(band);
        let max = self.config.channels;
        acquisitions
            .iter()
            .take(max.max(1))
            .map(|acq| {
                let mut epochs = self.track_epochs(
                    frame,
                    acq.sat,
                    acq.carrier_hz,
                    acq.code_phase_samples as f64,
                    params.early_late_spacing_chips,
                    self.epochs_in_frame(frame),
                );
                let mut lock_loss = 0usize;
                for epoch in &epochs {
                    if epoch.lock {
                        lock_loss = 0;
                    } else {
                        lock_loss += 1;
                    }
                    if lock_loss >= 3 {
                        if let Some((carrier_hz, code_phase_samples)) = self.quick_reacquire(
                            frame,
                            acq.sat,
                            epoch.carrier_hz,
                            epoch.code_phase_samples,
                        ) {
                            epochs = self.track_epochs(
                                frame,
                                acq.sat,
                                carrier_hz,
                                code_phase_samples,
                                params.early_late_spacing_chips,
                                self.epochs_in_frame(frame),
                            );
                        }
                        break;
                    }
                }
                TrackingResult {
                    sat: acq.sat,
                    carrier_hz: acq.carrier_hz,
                    code_phase_samples: acq.code_phase_samples as f64,
                    epochs,
                }
            })
            .collect()
    }

    fn epochs_in_frame(&self, frame: &SamplesFrame) -> usize {
        let samples_per_code = samples_per_code(
            self.config.sampling_freq_hz,
            self.config.code_freq_basis_hz,
            self.config.code_length,
        );
        (frame.len() / samples_per_code).max(1)
    }

    fn track_epochs(
        &self,
        frame: &SamplesFrame,
        sat: SatId,
        carrier_hz: f64,
        code_phase_samples: f64,
        early_late_spacing_chips: f64,
        epochs: usize,
    ) -> Vec<TrackEpoch> {
        let samples_per_code = samples_per_code(
            self.config.sampling_freq_hz,
            self.config.code_freq_basis_hz,
            self.config.code_length,
        );
        let mut state = LoopState {
            carrier_hz,
            code_rate_hz: self.config.code_freq_basis_hz,
            code_phase_samples,
            prev_prompt: None,
            state: ChannelState::Acquired,
        };

        let mut out = Vec::new();
        for epoch_idx in 0..epochs {
            let start = epoch_idx * samples_per_code;
            let end = (start + samples_per_code).min(frame.len());
            if start >= end {
                break;
            }
            let epoch_frame = SamplesFrame::new(
                SampleTime {
                    sample_index: frame.t0.sample_index + start as u64,
                    sample_rate_hz: frame.t0.sample_rate_hz,
                },
                frame.dt_s,
                frame.iq[start..end].to_vec(),
            );

            let (track_epoch, corr) = self.track_epoch(
                &epoch_frame,
                sat,
                state.carrier_hz,
                state.code_phase_samples,
                early_late_spacing_chips,
            );

            let (dll_err, pll_err, fll_err, lock) = discriminators(&corr, state.prev_prompt);
            state.prev_prompt = Some(corr.prompt);

            if lock {
                state.state = ChannelState::Tracking;
            } else if state.state == ChannelState::Tracking {
                state.state = ChannelState::Lost;
            } else {
                state.state = ChannelState::PullIn;
            }

            let dll_lock = dll_err.abs() < 0.2;
            let pll_lock = pll_err.abs() < 0.2;
            let fll_lock = fll_err.abs() < 0.2;
            let cycle_slip = false;
            let anti_false_lock =
                (corr.early.norm() - corr.late.norm()).abs() < 0.2 * corr.prompt.norm();

            let cn0_dbhz = track_epoch.cn0_dbhz;
            let params = self.config.tracking_params(bijux_gnss_core::SignalBand::L1);
            let (dll_bw, pll_bw, fll_bw) = adaptive_bandwidth(
                params.dll_bw_hz,
                params.pll_bw_hz,
                params.fll_bw_hz,
                cn0_dbhz,
            );
            state.code_rate_hz += dll_bw * dll_err as f64;
            if state.state == ChannelState::PullIn {
                state.carrier_hz += fll_bw * fll_err as f64;
            }
            state.carrier_hz += pll_bw * pll_err as f64;

            state.code_phase_samples =
                (state.code_phase_samples + samples_per_code as f64) % samples_per_code as f64;

            out.push(TrackEpoch {
                lock,
                carrier_hz: state.carrier_hz,
                code_rate_hz: state.code_rate_hz,
                code_phase_samples: state.code_phase_samples,
                pll_lock,
                dll_lock,
                fll_lock,
                cycle_slip,
                nav_bit_lock: anti_false_lock,
                dll_err,
                pll_err,
                fll_err,
                ..track_epoch
            });
        }
        out
    }
}

impl Tracking {
    fn quick_reacquire(
        &self,
        frame: &SamplesFrame,
        sat: SatId,
        carrier_hz: f64,
        code_phase_samples: f64,
    ) -> Option<(f64, f64)> {
        let doppler_bins = [-500.0, -250.0, 0.0, 250.0, 500.0];
        let code_bins = [-2.0, -1.0, 0.0, 1.0, 2.0];
        let mut best = None;
        let mut best_metric = 0.0_f32;
        for d in doppler_bins {
            for c in code_bins {
                let corr =
                    self.correlate_epoch(frame, sat, carrier_hz + d, code_phase_samples + c, 0.5);
                let metric = corr.prompt.norm();
                if metric > best_metric {
                    best_metric = metric;
                    best = Some((carrier_hz + d, code_phase_samples + c));
                }
            }
        }
        best
    }
}

fn adaptive_bandwidth(dll_bw: f64, pll_bw: f64, fll_bw: f64, cn0_dbhz: f64) -> (f64, f64, f64) {
    if cn0_dbhz < 25.0 {
        (dll_bw * 0.5, pll_bw * 0.5, fll_bw * 0.5)
    } else if cn0_dbhz > 40.0 {
        (dll_bw * 1.5, pll_bw * 1.5, fll_bw * 1.5)
    } else {
        (dll_bw, pll_bw, fll_bw)
    }
}

fn discriminators(
    corr: &CorrelatorOutput,
    prev_prompt: Option<Complex<f32>>,
) -> (f32, f32, f32, bool) {
    let e = corr.early.norm();
    let l = corr.late.norm();
    let p = corr.prompt.norm();
    let dll = if e + l > 0.0 { (e - l) / (e + l) } else { 0.0 };
    let pll = corr.prompt.im.atan2(corr.prompt.re);
    let fll = if let Some(prev) = prev_prompt {
        let dot = corr.prompt.re * prev.re + corr.prompt.im * prev.im;
        let det = corr.prompt.im * prev.re - corr.prompt.re * prev.im;
        det.atan2(dot)
    } else {
        0.0
    };
    let lock = p > 0.1 && dll.abs() < 0.5;
    (dll, pll, fll, lock)
}

fn estimate_cn0_dbhz(prompt: Complex<f32>, noise: Complex<f32>) -> f64 {
    let signal_power = (prompt.norm_sqr() as f64).max(1e-12);
    let noise_power = (noise.norm_sqr() as f64).max(1e-12);
    let snr = (signal_power / noise_power).max(1e-12);
    10.0 * snr.log10() + 30.0
}

fn code_at(code: &[i8], samples_per_chip: f64, sample_index: f64) -> Complex<f32> {
    let code_len_samples = samples_per_chip * code.len() as f64;
    let wrapped = sample_index.rem_euclid(code_len_samples);
    let chip_index = (wrapped / samples_per_chip).floor() as usize;
    let chip = code[chip_index] as f32;
    Complex::new(chip, 0.0)
}
