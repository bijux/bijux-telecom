use bijux_gnss_core::{
    Constellation, LockFlags, ObsEpoch, ObsMetadata, ObsSatellite, SignalBand, SignalSpec,
    TrackEpoch,
};

use crate::signal::samples_per_code;
use crate::tracking::TrackingResult;
use crate::types::ReceiverConfig;

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;
const TWO_PI: f64 = std::f64::consts::PI * 2.0;

#[derive(Debug, Clone)]
struct PhaseState {
    phase_cycles: f64,
    last_phase_cycles: f64,
    initialized: bool,
}

#[derive(Debug, Clone)]
struct HatchState {
    smoothed_m: f64,
    count: u32,
    last_carrier_cycles: f64,
    last_divergence_m: f64,
    age: u32,
    resets: u32,
    initialized: bool,
}

pub fn observations_from_tracking(config: &ReceiverConfig, epochs: &[TrackEpoch]) -> Vec<ObsEpoch> {
    if epochs.is_empty() {
        return Vec::new();
    }

    let samples_per_code = samples_per_code(
        config.sampling_freq_hz,
        config.code_freq_basis_hz,
        config.code_length,
    );
    let samples_per_chip = samples_per_code as f64 / config.code_length as f64;
    let code_rate_hz = config.code_freq_basis_hz;

    let mut phase_state = PhaseState {
        phase_cycles: 0.0,
        last_phase_cycles: 0.0,
        initialized: false,
    };

    let mut out = Vec::with_capacity(epochs.len());
    let mut last_sample_index = None;
    let clock_bias_s = 0.0_f64;

    for epoch in epochs {
        let t_rx_s = epoch.sample_index as f64 / config.sampling_freq_hz + clock_bias_s;

        let expected_step = (config.sampling_freq_hz * 0.001).round() as u64;
        let discontinuity = match last_sample_index {
            Some(prev) => epoch.sample_index.saturating_sub(prev) != expected_step,
            None => false,
        };
        if let Some(prev) = last_sample_index {
            if epoch.sample_index < prev {
                eprintln!(
                    "time audit: sample index went backwards ({} -> {})",
                    prev, epoch.sample_index
                );
            }
        }
        if discontinuity {
            eprintln!(
                "time audit: discontinuity at sample index {}",
                epoch.sample_index
            );
        }
        last_sample_index = Some(epoch.sample_index);

        let code_phase_chips = epoch.code_phase_samples / samples_per_chip;
        let code_epoch = epoch.epoch.index as f64;
        let code_time_s =
            (code_epoch * config.code_length as f64 + code_phase_chips) / code_rate_hz;
        let pseudorange_m = code_time_s * SPEED_OF_LIGHT_MPS + clock_bias_s * SPEED_OF_LIGHT_MPS;

        let prompt_phase_cycles = (epoch.prompt_q as f64).atan2(epoch.prompt_i as f64) / TWO_PI;
        let doppler_hz = epoch.carrier_hz;
        let dt_s = 0.001;
        let mut cycle_slip = false;

        if !phase_state.initialized {
            phase_state.phase_cycles = prompt_phase_cycles;
            phase_state.last_phase_cycles = prompt_phase_cycles;
            phase_state.initialized = true;
        } else {
            let predicted = phase_state.phase_cycles + doppler_hz * dt_s;
            let mut measured = prompt_phase_cycles;
            let delta = measured - phase_state.last_phase_cycles;
            if delta > 0.5 {
                measured -= 1.0;
            } else if delta < -0.5 {
                measured += 1.0;
            }
            let residual = (measured - predicted).abs();
            if residual > 0.25 {
                cycle_slip = true;
            }
            phase_state.phase_cycles = predicted + (measured - predicted);
            phase_state.last_phase_cycles = measured;
        }

        let prompt_power =
            (epoch.prompt_i * epoch.prompt_i + epoch.prompt_q * epoch.prompt_q) as f64;
        let cn0_dbhz = 10.0 * (prompt_power.max(1e-9)).log10();

        let sat = ObsSatellite {
            prn: epoch.prn,
            pseudorange_m,
            carrier_phase_cycles: phase_state.phase_cycles,
            doppler_hz,
            cn0_dbhz,
            lock_flags: LockFlags {
                code_lock: epoch.lock,
                carrier_lock: epoch.lock,
                bit_lock: false,
                cycle_slip,
            },
            multipath_suspect: false,
            elevation_deg: None,
            azimuth_deg: None,
            weight: None,
            error_model: None,
            metadata: ObsMetadata {
                tracking_mode: "scalar".to_string(),
                integration_ms: 1,
                lock_quality: cn0_dbhz,
                smoothing_window: 0,
                smoothing_age: 0,
                smoothing_resets: 0,
                signal: SignalSpec {
                    constellation: Constellation::Gps,
                    band: SignalBand::L1,
                    code_rate_hz: config.code_freq_basis_hz,
                    carrier_hz: 1_575_420_000.0,
                },
            },
        };

        out.push(ObsEpoch {
            t_rx_s,
            gps_week: None,
            tow_s: None,
            epoch_idx: epoch.epoch.index,
            discontinuity,
            sats: vec![sat],
        });
    }

    out
}

pub fn observations_from_tracking_results(
    config: &ReceiverConfig,
    tracks: &[TrackingResult],
    hatch_window: u32,
) -> Vec<ObsEpoch> {
    use std::collections::BTreeMap;

    let mut by_epoch: BTreeMap<u64, ObsEpoch> = BTreeMap::new();
    let mut hatch: std::collections::HashMap<u8, HatchState> = std::collections::HashMap::new();
    for track in tracks {
        let obs = observations_from_tracking(config, &track.epochs);
        for epoch in obs {
            let entry = by_epoch.entry(epoch.epoch_idx).or_insert_with(|| ObsEpoch {
                t_rx_s: epoch.t_rx_s,
                gps_week: epoch.gps_week,
                tow_s: epoch.tow_s,
                epoch_idx: epoch.epoch_idx,
                discontinuity: epoch.discontinuity,
                sats: Vec::new(),
            });
            entry.discontinuity |= epoch.discontinuity;
            for mut sat in epoch.sats {
                let lambda_m = SPEED_OF_LIGHT_MPS / 1_575_420_000.0;
                let state = hatch.entry(sat.prn).or_insert(HatchState {
                    smoothed_m: sat.pseudorange_m,
                    count: 0,
                    last_carrier_cycles: sat.carrier_phase_cycles,
                    last_divergence_m: 0.0,
                    age: 0,
                    resets: 0,
                    initialized: false,
                });
                let window = hatch_window.max(1) as f64;
                if sat.lock_flags.cycle_slip || !sat.lock_flags.code_lock {
                    state.initialized = false;
                    state.resets = state.resets.saturating_add(1);
                    state.age = 0;
                }
                if !state.initialized {
                    state.smoothed_m = sat.pseudorange_m;
                    state.count = 1;
                    state.last_carrier_cycles = sat.carrier_phase_cycles;
                    state.last_divergence_m =
                        sat.pseudorange_m - sat.carrier_phase_cycles * lambda_m;
                    state.initialized = true;
                    state.age = 1;
                } else {
                    let delta_carrier =
                        (sat.carrier_phase_cycles - state.last_carrier_cycles) * lambda_m;
                    let pred = state.smoothed_m + delta_carrier;
                    let count = (state.count as f64).min(window);
                    state.smoothed_m = pred + (sat.pseudorange_m - pred) / count;
                    state.count = state.count.saturating_add(1);
                    state.last_carrier_cycles = sat.carrier_phase_cycles;
                    state.last_divergence_m =
                        sat.pseudorange_m - sat.carrier_phase_cycles * lambda_m;
                    state.age = state.age.saturating_add(1);
                }
                sat.pseudorange_m = state.smoothed_m;
                sat.metadata.smoothing_window = hatch_window;
                sat.metadata.smoothing_age = state.age;
                sat.metadata.smoothing_resets = state.resets;
                let divergence_m = sat.pseudorange_m - sat.carrier_phase_cycles * lambda_m;
                let threshold_m = slip_threshold_m(sat.cn0_dbhz, sat.elevation_deg);
                let divergence_jump = (divergence_m - state.last_divergence_m).abs();
                if divergence_jump > threshold_m {
                    sat.lock_flags.cycle_slip = true;
                }
                sat.multipath_suspect = divergence_jump > threshold_m * 0.8;
                sat.error_model = Some(bijux_gnss_core::MeasurementErrorModel {
                    thermal_noise_m: 1.0,
                    tracking_jitter_m: (1.0 / sat.cn0_dbhz.max(1.0)) * 10.0,
                    multipath_proxy_m: divergence_jump,
                    clock_error_m: 0.0,
                });
                entry.sats.push(sat);
            }
        }
    }
    by_epoch.into_values().collect()
}

fn slip_threshold_m(cn0_dbhz: f64, elevation_deg: Option<f64>) -> f64 {
    let cn0 = cn0_dbhz.clamp(20.0, 60.0);
    let cn0_factor = 45.0 / cn0;
    let elev = elevation_deg.unwrap_or(30.0).clamp(0.0, 90.0);
    let elev_factor = 1.0 + (30.0 - elev).max(0.0) / 30.0;
    5.0 * cn0_factor * elev_factor
}
