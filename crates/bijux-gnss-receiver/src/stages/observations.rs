#![allow(missing_docs)]

use bijux_gnss_core::api::{
    Cycles, LockFlags, Meters, ObsEpoch, ObsMetadata, ObsSatellite, ReceiverRole, Seconds, SigId,
    SignalBand, TrackEpoch,
};

use crate::runtime::receiver_config::ReceiverConfig;
use crate::stages::tracking::TrackingResult;
use bijux_gnss_signal::api::samples_per_code;

#[cfg(test)]
use bijux_gnss_core::api::{Constellation, Hertz, SatId, SignalCode};

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
        let t_rx_s = Seconds(epoch.sample_index as f64 / config.sampling_freq_hz + clock_bias_s);

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

        let code_phase_chips = epoch.code_phase_samples.0 / samples_per_chip;
        let code_epoch = epoch.epoch.index as f64;
        let code_time_s =
            (code_epoch * config.code_length as f64 + code_phase_chips) / code_rate_hz;
        let pseudorange_m =
            Meters(code_time_s * SPEED_OF_LIGHT_MPS + clock_bias_s * SPEED_OF_LIGHT_MPS);

        let prompt_phase_cycles = (epoch.prompt_q as f64).atan2(epoch.prompt_i as f64) / TWO_PI;
        let doppler_hz = epoch.carrier_hz;
        let dt_s = 0.001;
        let mut cycle_slip = false;

        if !phase_state.initialized {
            phase_state.phase_cycles = prompt_phase_cycles;
            phase_state.last_phase_cycles = prompt_phase_cycles;
            phase_state.initialized = true;
        } else {
            let predicted = phase_state.phase_cycles + doppler_hz.0 * dt_s;
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

        let variance_m2 = (1.0 / cn0_dbhz.max(1.0)).powi(2);
        let mut signal = bijux_gnss_core::api::signal_spec_gps_l1_ca();
        signal.code_rate_hz = config.code_freq_basis_hz;
        let sat = ObsSatellite {
            signal_id: SigId {
                sat: epoch.sat,
                band: SignalBand::L1,
                code: bijux_gnss_core::api::SignalCode::Ca,
            },
            pseudorange_m,
            pseudorange_var_m2: variance_m2,
            carrier_phase_cycles: Cycles(phase_state.phase_cycles),
            carrier_phase_var_cycles2: variance_m2 * 0.01,
            doppler_hz,
            doppler_var_hz2: 4.0,
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
                integration_ms: config.tracking_integration_ms,
                lock_quality: cn0_dbhz,
                smoothing_window: 0,
                smoothing_age: 0,
                smoothing_resets: 0,
                signal,
            },
        };

        let mut epoch = ObsEpoch {
            t_rx_s,
            gps_week: None,
            tow_s: None,
            epoch_idx: epoch.epoch.index,
            discontinuity,
            valid: true,
            processing_ms: None,
            role: ReceiverRole::Rover,
            sats: vec![sat],
        };
        let events = bijux_gnss_core::api::check_obs_epoch_sanity(&epoch);
        if events
            .iter()
            .any(|e| matches!(e.severity, bijux_gnss_core::api::DiagnosticSeverity::Error))
        {
            epoch.valid = false;
            for event in events {
                crate::runtime::logging::diagnostic(&event);
            }
        }
        out.push(epoch);
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
    let mut hatch: std::collections::HashMap<bijux_gnss_core::api::SigId, HatchState> =
        std::collections::HashMap::new();
    for track in tracks {
        let obs = observations_from_tracking(config, &track.epochs);
        for epoch in obs {
            let start_time = std::time::Instant::now();
            let entry = by_epoch.entry(epoch.epoch_idx).or_insert_with(|| ObsEpoch {
                t_rx_s: epoch.t_rx_s,
                gps_week: epoch.gps_week,
                tow_s: epoch.tow_s,
                epoch_idx: epoch.epoch_idx,
                discontinuity: epoch.discontinuity,
                valid: true,
                processing_ms: Some(0.0),
                role: epoch.role,
                sats: Vec::new(),
            });
            entry.discontinuity |= epoch.discontinuity;
            for mut sat in epoch.sats {
                let lambda_m = SPEED_OF_LIGHT_MPS / sat.metadata.signal.carrier_hz.value();
                let state = hatch.entry(sat.signal_id).or_insert(HatchState {
                    smoothed_m: sat.pseudorange_m.0,
                    count: 0,
                    last_carrier_cycles: sat.carrier_phase_cycles.0,
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
                    state.smoothed_m = sat.pseudorange_m.0;
                    state.count = 1;
                    state.last_carrier_cycles = sat.carrier_phase_cycles.0;
                    state.last_divergence_m =
                        sat.pseudorange_m.0 - sat.carrier_phase_cycles.0 * lambda_m;
                    state.initialized = true;
                    state.age = 1;
                } else {
                    let delta_carrier =
                        (sat.carrier_phase_cycles.0 - state.last_carrier_cycles) * lambda_m;
                    let pred = state.smoothed_m + delta_carrier;
                    let count = (state.count as f64).min(window);
                    state.smoothed_m = pred + (sat.pseudorange_m.0 - pred) / count;
                    state.count = state.count.saturating_add(1);
                    state.last_carrier_cycles = sat.carrier_phase_cycles.0;
                    state.last_divergence_m =
                        sat.pseudorange_m.0 - sat.carrier_phase_cycles.0 * lambda_m;
                    state.age = state.age.saturating_add(1);
                }
                sat.pseudorange_m = Meters(state.smoothed_m);
                sat.metadata.smoothing_window = hatch_window;
                sat.metadata.smoothing_age = state.age;
                sat.metadata.smoothing_resets = state.resets;
                let divergence_m = sat.pseudorange_m.0 - sat.carrier_phase_cycles.0 * lambda_m;
                let threshold_m = slip_threshold_m(sat.cn0_dbhz, sat.elevation_deg);
                let divergence_jump = (divergence_m - state.last_divergence_m).abs();
                if divergence_jump > threshold_m {
                    sat.lock_flags.cycle_slip = true;
                }
                sat.multipath_suspect = divergence_jump > threshold_m * 0.8;
                sat.error_model = Some(bijux_gnss_core::api::MeasurementErrorModel {
                    thermal_noise_m: Meters(1.0),
                    tracking_jitter_m: Meters((1.0 / sat.cn0_dbhz.max(1.0)) * 10.0),
                    multipath_proxy_m: Meters(divergence_jump),
                    clock_error_m: Meters(0.0),
                });
                entry.sats.push(sat);
            }
            if let Some(total) = entry.processing_ms.as_mut() {
                *total += start_time.elapsed().as_secs_f64() * 1000.0;
            }
        }
    }
    let mut out = Vec::new();
    for mut epoch in by_epoch.into_values() {
        let events = bijux_gnss_core::api::check_obs_epoch_sanity(&epoch);
        if events
            .iter()
            .any(|e| matches!(e.severity, bijux_gnss_core::api::DiagnosticSeverity::Error))
        {
            epoch.valid = false;
            for event in events {
                crate::runtime::logging::diagnostic(&event);
            }
        }
        out.push(epoch);
    }
    out
}

fn slip_threshold_m(cn0_dbhz: f64, elevation_deg: Option<f64>) -> f64 {
    let cn0 = cn0_dbhz.clamp(20.0, 60.0);
    let cn0_factor = 45.0 / cn0;
    let elev = elevation_deg.unwrap_or(30.0).clamp(0.0, 90.0);
    let elev_factor = 1.0 + (30.0 - elev).max(0.0) / 30.0;
    5.0 * cn0_factor * elev_factor
}

#[cfg(test)]
pub(crate) fn fake_obs_epoch_for_nav_tests(epoch_idx: u64) -> ObsEpoch {
    let sats = (1..=4)
        .map(|prn| ObsSatellite {
            signal_id: SigId {
                sat: SatId {
                    constellation: Constellation::Gps,
                    prn,
                },
                band: SignalBand::L1,
                code: SignalCode::Ca,
            },
            pseudorange_m: Meters(20_200_000.0 + prn as f64),
            pseudorange_var_m2: 100.0,
            carrier_phase_cycles: Cycles(0.0),
            carrier_phase_var_cycles2: 1.0,
            doppler_hz: Hertz(0.0),
            doppler_var_hz2: 1.0,
            cn0_dbhz: 45.0,
            lock_flags: LockFlags {
                code_lock: true,
                carrier_lock: true,
                bit_lock: true,
                cycle_slip: false,
            },
            multipath_suspect: false,
            elevation_deg: Some(45.0),
            azimuth_deg: Some(0.0),
            weight: Some(1.0),
            error_model: None,
            metadata: ObsMetadata {
                tracking_mode: "scalar".to_string(),
                integration_ms: 1,
                lock_quality: 1.0,
                smoothing_window: 0,
                smoothing_age: 0,
                smoothing_resets: 0,
                signal: bijux_gnss_core::api::signal_spec_gps_l1_ca(),
            },
        })
        .collect();
    ObsEpoch {
        t_rx_s: Seconds(epoch_idx as f64 * 0.001),
        gps_week: None,
        tow_s: None,
        epoch_idx,
        discontinuity: false,
        valid: true,
        processing_ms: None,
        role: ReceiverRole::Rover,
        sats,
    }
}
