fn detect_sample_rate_mismatch(
    epochs: &[TrackEpoch],
    samples_per_code: usize,
) -> Option<CodePhaseStabilityDiagnostic> {
    if epochs.len() <= SAMPLE_RATE_MISMATCH_WINDOW_EPOCHS {
        return None;
    }

    let phase_step_limit_samples = SAMPLE_RATE_MISMATCH_MIN_PHASE_DRIFT_SAMPLES
        .max(samples_per_code as f64 * SAMPLE_RATE_MISMATCH_MIN_PHASE_DRIFT_FRACTION);
    let catastrophic_phase_step_samples =
        phase_step_limit_samples * SAMPLE_RATE_MISMATCH_CATASTROPHIC_PHASE_STEP_MULTIPLIER;
    let instability_markers = epochs
        .windows(2)
        .enumerate()
        .map(|(index, pair)| {
            let previous = &pair[0];
            let current = &pair[1];
            let phase_step_samples = wrapped_code_phase_delta_samples(
                current.code_phase_samples.0,
                previous.code_phase_samples.0,
                samples_per_code,
            );
            let abs_phase_step_samples = phase_step_samples.abs();
            let unstable = abs_phase_step_samples > phase_step_limit_samples || current.cycle_slip;
            let supported = sample_rate_mismatch_supported_epoch(current);
            (index + 1, abs_phase_step_samples, unstable, supported)
        })
        .collect::<Vec<_>>();

    if let Some((first_unstable_epoch_index, max_abs_phase_step_samples, _, _)) =
        instability_markers.iter().copied().find(|(_, abs_phase_step_samples, _, supported)| {
            *supported && *abs_phase_step_samples >= catastrophic_phase_step_samples
        })
    {
        return Some(CodePhaseStabilityDiagnostic {
            first_unstable_epoch_index,
            max_abs_phase_step_samples,
            phase_step_limit_samples,
        });
    }

    for window in instability_markers.windows(SAMPLE_RATE_MISMATCH_WINDOW_EPOCHS) {
        let unstable_count =
            window.iter().filter(|(_, _, unstable, supported)| *unstable && *supported).count();
        if unstable_count >= SAMPLE_RATE_MISMATCH_MIN_UNSTABLE_EPOCHS_IN_WINDOW {
            let first_unstable_epoch_index = window
                .iter()
                .find(|(_, _, unstable, supported)| *unstable && *supported)
                .map(|(epoch_index, _, _, _)| *epoch_index)
                .expect("window contains unstable epoch");
            let max_abs_phase_step_samples = window
                .iter()
                .filter(|(_, _, unstable, supported)| *unstable && *supported)
                .map(|(_, abs_phase_step_samples, _, _)| *abs_phase_step_samples)
                .fold(0.0_f64, f64::max);
            return Some(CodePhaseStabilityDiagnostic {
                first_unstable_epoch_index,
                max_abs_phase_step_samples,
                phase_step_limit_samples,
            });
        }
    }

    None
}

fn sample_rate_mismatch_supported_epoch(epoch: &TrackEpoch) -> bool {
    if !epoch.cn0_dbhz.is_finite() || epoch.cn0_dbhz < SAMPLE_RATE_MISMATCH_MIN_CN0_DBHZ {
        return false;
    }
    if !matches!(epoch.lock_state.as_str(), "pull_in" | "tracking" | "degraded") {
        return false;
    }
    if epoch.lock_state_reason.as_deref().is_some_and(|reason| {
        matches!(
            reason,
            "phase_jump"
                | "prompt_power_drop"
                | "discriminator_instability"
                | "lock_lost"
                | "reacquisition_failed"
        )
    }) {
        return false;
    }
    epoch.lock || epoch.fll_lock || epoch.pll_lock
}

#[cfg(test)]
fn tracking_epoch_samples(
    sample_rate_hz: f64,
    code_freq_basis_hz: f64,
    code_length: usize,
    tracking_params: TrackingParams,
) -> usize {
    let code_period_samples = samples_per_code(sample_rate_hz, code_freq_basis_hz, code_length);
    code_period_samples.saturating_mul(tracking_params.integration_ms.max(1) as usize)
}

#[cfg(test)]
fn tracking_epoch_count(frame_len_samples: usize, epoch_len_samples: usize) -> usize {
    let epoch_len_samples = epoch_len_samples.max(1);
    if frame_len_samples == 0 {
        return 0;
    }
    frame_len_samples.div_ceil(epoch_len_samples)
}
