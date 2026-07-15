#[derive(Debug, Default)]
struct TrackingNoiseAccumulator {
    scenario_ids: Vec<String>,
    satellites: Vec<SatId>,
    truth_cn0_db_hz: Vec<f64>,
    truth_abs_doppler_hz: Vec<f64>,
    dll_jitter_samples: Vec<f64>,
    pll_phase_error_cycles: Vec<f64>,
    doppler_error_hz: Vec<f64>,
    cn0_bias_db_hz: Vec<f64>,
    cycle_slip_count: usize,
}

/// Return the receiver signal identities that have executable tracking support.
pub fn supported_tracking_signal_identities() -> Vec<SyntheticTrackingSignalIdentity> {
    let mut signals = bijux_gnss_signal::api::registered_signal_registry_entries()
        .into_iter()
        .filter_map(|entry| {
            let signal = entry.spec;
            let glonass_frequency_channel =
                (signal.constellation == Constellation::Glonass && signal.band == SignalBand::L1)
                    .then(|| {
                        GlonassFrequencyChannel::new(0)
                            .expect("GLONASS channel 0 must be valid")
                    });
            let supported = crate::pipeline::tracking::supports_tracking_signal_with_channel(
                representative_tracking_sat(signal.constellation),
                signal.band,
                signal.code,
                glonass_frequency_channel,
            );
            supported.then_some(SyntheticTrackingSignalIdentity {
                constellation: signal.constellation,
                signal_band: signal.band,
                signal_code: signal.code,
                glonass_frequency_channel,
            })
        })
        .collect::<Vec<_>>();
    signals.sort();
    signals.dedup();
    signals
}

/// Characterize empirical tracking noise for every receiver-supported signal identity.
pub fn characterize_supported_tracking_noise(
    reports: &[SyntheticTrackingTruthTableReport],
    required_stable_epoch_count: usize,
) -> SyntheticTrackingNoiseReport {
    let required_stable_epoch_count = required_stable_epoch_count.max(1);
    let supported_signals = supported_tracking_signal_identities();
    let mut scenario_ids = reports
        .iter()
        .map(|report| report.scenario_id.clone())
        .collect::<Vec<_>>();
    scenario_ids.sort();
    scenario_ids.dedup();

    let mut accumulators = BTreeMap::<SyntheticTrackingSignalIdentity, TrackingNoiseAccumulator>::new();
    for report in reports {
        for satellite in &report.satellites {
            let signal = SyntheticTrackingSignalIdentity {
                constellation: satellite.sat.constellation,
                signal_band: satellite.signal_band,
                signal_code: satellite.signal_code,
                glonass_frequency_channel: satellite.glonass_frequency_channel,
            };
            let accumulator = accumulators.entry(signal).or_default();
            push_unique(&mut accumulator.scenario_ids, report.scenario_id.clone());
            push_unique(&mut accumulator.satellites, satellite.sat);

            for epoch in satellite.epochs.iter().filter(|epoch| epoch.stable_tracking_epoch) {
                accumulator.truth_cn0_db_hz.push(epoch.expected_cn0_db_hz);
                accumulator.truth_abs_doppler_hz.push(epoch.expected_doppler_hz.abs());
                accumulator.dll_jitter_samples.push(epoch.code_phase_error_samples);
                accumulator.pll_phase_error_cycles.push(epoch.pll_phase_error_cycles);
                accumulator.doppler_error_hz.push(epoch.doppler_error_hz);
                accumulator.cn0_bias_db_hz.push(epoch.measured_cn0_dbhz - epoch.expected_cn0_db_hz);
                if epoch.cycle_slip {
                    accumulator.cycle_slip_count += 1;
                }
            }
        }
    }

    let mut profiles = accumulators
        .into_iter()
        .filter_map(|(signal, accumulator)| {
            build_tracking_noise_profile(signal, accumulator, required_stable_epoch_count)
        })
        .collect::<Vec<_>>();
    profiles.sort_by_key(|profile| profile.signal);
    let covered_signals = profiles.iter().map(|profile| profile.signal).collect::<Vec<_>>();
    let missing_signals = supported_signals
        .iter()
        .copied()
        .filter(|signal| !covered_signals.contains(signal))
        .collect::<Vec<_>>();
    let under_sampled_signals = profiles
        .iter()
        .filter(|profile| !profile.pass)
        .map(|profile| profile.signal)
        .collect::<Vec<_>>();
    let pass = !supported_signals.is_empty()
        && missing_signals.is_empty()
        && under_sampled_signals.is_empty()
        && profiles.len() == supported_signals.len();

    SyntheticTrackingNoiseReport {
        scenario_ids,
        required_stable_epoch_count,
        supported_signal_count: supported_signals.len(),
        characterized_signal_count: profiles.len(),
        missing_signals,
        under_sampled_signals,
        profiles,
        pass,
    }
}

fn build_tracking_noise_profile(
    signal: SyntheticTrackingSignalIdentity,
    accumulator: TrackingNoiseAccumulator,
    required_stable_epoch_count: usize,
) -> Option<SyntheticTrackingNoiseProfile> {
    let stable_epoch_count = accumulator.dll_jitter_samples.iter().filter(|value| value.is_finite()).count();
    if stable_epoch_count == 0 {
        return None;
    }
    let pass = stable_epoch_count >= required_stable_epoch_count;
    let cycle_slip_probability = accumulator.cycle_slip_count as f64 / stable_epoch_count as f64;

    Some(SyntheticTrackingNoiseProfile {
        signal,
        satellite_count: accumulator.satellites.len(),
        stable_epoch_count,
        min_cn0_db_hz: finite_min(&accumulator.truth_cn0_db_hz),
        max_cn0_db_hz: finite_max(&accumulator.truth_cn0_db_hz),
        min_abs_doppler_hz: finite_min(&accumulator.truth_abs_doppler_hz),
        max_abs_doppler_hz: finite_max(&accumulator.truth_abs_doppler_hz),
        dll_jitter_samples: summarize_tracking_noise_distribution(
            &accumulator.dll_jitter_samples,
        ),
        pll_phase_error_cycles: summarize_tracking_noise_distribution(
            &accumulator.pll_phase_error_cycles,
        ),
        doppler_error_hz: summarize_tracking_noise_distribution(&accumulator.doppler_error_hz),
        cn0_bias_db_hz: summarize_tracking_noise_distribution(&accumulator.cn0_bias_db_hz),
        cycle_slip_count: accumulator.cycle_slip_count,
        cycle_slip_probability,
        pass,
    })
}

fn summarize_tracking_noise_distribution(values: &[f64]) -> SyntheticTrackingNoiseDistribution {
    let finite_values = values.iter().copied().filter(|value| value.is_finite()).collect::<Vec<_>>();
    if finite_values.is_empty() {
        return SyntheticTrackingNoiseDistribution {
            sample_count: 0,
            mean: 0.0,
            rms: 0.0,
            standard_deviation: 0.0,
            p50_abs: 0.0,
            p95_abs: 0.0,
            max_abs: 0.0,
        };
    }

    let sample_count = finite_values.len();
    let mean = finite_values.iter().sum::<f64>() / sample_count as f64;
    let rms = (finite_values.iter().map(|value| value * value).sum::<f64>() / sample_count as f64)
        .sqrt();
    let standard_deviation = (finite_values
        .iter()
        .map(|value| {
            let residual = value - mean;
            residual * residual
        })
        .sum::<f64>()
        / sample_count as f64)
        .sqrt();
    let mut absolute_values = finite_values.iter().map(|value| value.abs()).collect::<Vec<_>>();
    absolute_values.sort_by(f64::total_cmp);
    let p50_abs = percentile_nearest_rank(&absolute_values, 0.50);
    let p95_abs = percentile_nearest_rank(&absolute_values, 0.95);
    let max_abs = absolute_values.last().copied().unwrap_or_default();

    SyntheticTrackingNoiseDistribution {
        sample_count,
        mean,
        rms,
        standard_deviation,
        p50_abs,
        p95_abs,
        max_abs,
    }
}

fn percentile_nearest_rank(sorted_values: &[f64], percentile: f64) -> f64 {
    if sorted_values.is_empty() {
        return 0.0;
    }
    let rank = ((sorted_values.len() as f64 * percentile).ceil() as usize).saturating_sub(1);
    sorted_values[rank.min(sorted_values.len() - 1)]
}

fn finite_min(values: &[f64]) -> f64 {
    values
        .iter()
        .copied()
        .filter(|value| value.is_finite())
        .min_by(f64::total_cmp)
        .unwrap_or_default()
}

fn finite_max(values: &[f64]) -> f64 {
    values
        .iter()
        .copied()
        .filter(|value| value.is_finite())
        .max_by(f64::total_cmp)
        .unwrap_or_default()
}

fn representative_tracking_sat(constellation: Constellation) -> SatId {
    SatId {
        constellation,
        prn: match constellation {
            Constellation::Glonass => 8,
            Constellation::Unknown => 0,
            _ => 1,
        },
    }
}

fn push_unique<T: Ord>(values: &mut Vec<T>, value: T) {
    if !values.contains(&value) {
        values.push(value);
        values.sort();
    }
}
