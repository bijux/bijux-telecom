#![allow(missing_docs)]

use num_complex::Complex;

use bijux_gnss_core::api::{
    Chips, Constellation, Hertz, SampleClock, SampleTime, SamplesFrame, SatId, Seconds, TrackEpoch,
};

use crate::runtime::logging;
use crate::runtime::receiver_config::ReceiverConfig;
use bijux_gnss_core::api::Sample;
use bijux_gnss_signal::api::samples_per_code;
use bijux_gnss_signal::api::Nco;
use bijux_gnss_signal::api::{adaptive_bandwidth, code_at, discriminators, estimate_cn0_dbhz};
use bijux_gnss_signal::api::{generate_ca_code, Prn};

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
        let code = ca_code_or_default(sat.prn);
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
        channel_id: u8,
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
        if !correlator.prompt.re.is_finite() || !correlator.prompt.im.is_finite() {
            crate::runtime::logging::diagnostic(&bijux_gnss_core::api::DiagnosticEvent::new(
                bijux_gnss_core::api::DiagnosticSeverity::Error,
                "TRACK_NUMERIC_INVALID",
                "tracking correlator produced NaN/Inf",
            ));
            crate::runtime::diagnostics::dump_on_error(
                "tracking correlator produced NaN/Inf",
                None,
                None,
            );
        }
        let track_epoch = TrackEpoch {
            epoch,
            sample_index: frame.t0.sample_index,
            sat,
            prompt_i: correlator.prompt.re,
            prompt_q: correlator.prompt.im,
            carrier_hz: Hertz(carrier_freq_hz),
            code_rate_hz: Hertz(self.config.code_freq_basis_hz),
            code_phase_samples: Chips(code_phase_samples),
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
            processing_ms: None,
        };
        #[cfg(feature = "tracing")]
        {
            tracing::debug!(
                run_id = %std::env::var("BIJUX_RUN_ID").unwrap_or_else(|_| "unknown".to_string()),
                channel_id,
                epoch_idx = track_epoch.epoch.index,
                t_rx_s = track_epoch.sample_index as f64 / self.config.sampling_freq_hz,
                prn = track_epoch.sat.prn,
                "tracking epoch"
            );
        }
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
            Seconds(1.0 / self.config.sampling_freq_hz),
            samples.to_vec(),
        );
        let epochs = self.track_epochs(
            &frame,
            0,
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
        acquisitions: &[bijux_gnss_core::api::AcqResult],
        band: bijux_gnss_core::api::SignalBand,
    ) -> Vec<TrackingResult> {
        let params = self.config.tracking_params(band);
        let max = self.config.channels;
        acquisitions
            .iter()
            .take(max.max(1))
            .enumerate()
            .map(|(channel_idx, acq)| {
                let channel_id = channel_idx as u8;
                let mut epochs = self.track_epochs(
                    frame,
                    channel_id,
                    acq.sat,
                    acq.carrier_hz.0,
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
                            epoch.carrier_hz.0,
                            epoch.code_phase_samples.0,
                        ) {
                            epochs = self.track_epochs(
                                frame,
                                channel_id,
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
                    carrier_hz: acq.carrier_hz.0,
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
        channel_id: u8,
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
            let alloc_before = crate::runtime::alloc::allocation_count();
            let start_time = std::time::Instant::now();
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

            let (mut track_epoch, corr) = self.track_epoch(
                &epoch_frame,
                channel_id,
                sat,
                state.carrier_hz,
                state.code_phase_samples,
                early_late_spacing_chips,
            );
            track_epoch.processing_ms = Some(start_time.elapsed().as_secs_f64() * 1000.0);
            if let Some(ms) = track_epoch.processing_ms {
                if ms > self.config.tracking_budget_ms {
                    logging::diagnostic(&bijux_gnss_core::api::DiagnosticEvent::new(
                        bijux_gnss_core::api::DiagnosticSeverity::Warning,
                        "TRACK_BUDGET_EXCEEDED",
                        format!("tracking epoch exceeded budget: {ms:.3} ms"),
                    ));
                    if self.config.tracking_over_budget_action == "drop_epochs" {
                        break;
                    }
                }
            }
            let alloc_after = crate::runtime::alloc::allocation_count();
            if alloc_after > alloc_before {
                logging::diagnostic(&bijux_gnss_core::api::DiagnosticEvent::new(
                    bijux_gnss_core::api::DiagnosticSeverity::Warning,
                    "TRACK_ALLOCATIONS",
                    format!(
                        "tracking epoch allocated {} times",
                        alloc_after - alloc_before
                    ),
                ));
            }

            let (dll_err, pll_err, fll_err, lock) =
                discriminators(corr.early, corr.prompt, corr.late, state.prev_prompt);
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
            let params = self
                .config
                .tracking_params(bijux_gnss_core::api::SignalBand::L1);
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
                carrier_hz: Hertz(state.carrier_hz),
                code_rate_hz: Hertz(state.code_rate_hz),
                code_phase_samples: Chips(state.code_phase_samples),
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

fn ca_code_or_default(prn: u8) -> Vec<i8> {
    match generate_ca_code(Prn(prn)) {
        Ok(code) => code,
        Err(_) => vec![1; 1023],
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

#[cfg(test)]
mod tests {
    use super::{ChannelState, Tracking};

    #[test]
    fn tracking_recovery_from_loss_of_lock() {
        let lost = Tracking::transition_state(1, ChannelState::Tracking, ChannelState::Lost);
        assert_eq!(lost, ChannelState::Lost);
        let pull_in = Tracking::transition_state(1, lost, ChannelState::PullIn);
        assert_eq!(pull_in, ChannelState::PullIn);
        let tracking = Tracking::transition_state(1, pull_in, ChannelState::Tracking);
        assert_eq!(tracking, ChannelState::Tracking);
    }

    #[test]
    fn tracking_reset_after_gap() {
        let lost = Tracking::transition_state(2, ChannelState::Tracking, ChannelState::Lost);
        let reset = Tracking::transition_state(2, lost, ChannelState::Idle);
        assert_eq!(reset, ChannelState::Idle);
    }
}
