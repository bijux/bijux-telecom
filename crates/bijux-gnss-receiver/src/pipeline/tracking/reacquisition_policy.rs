#[derive(Debug, Clone, Copy, PartialEq)]
struct ReacquisitionSeed {
    carrier_hz: f64,
    code_phase_samples: f64,
    cn0_dbhz: f64,
    carrier_sign: ReacquisitionCarrierSign,
    secondary_code_phase_periods: Option<usize>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ReacquisitionCarrierSign {
    Aligned,
    Inverted,
}

impl ReacquisitionCarrierSign {
    fn phase_offset_cycles(self) -> f64 {
        match self {
            Self::Aligned => 0.0,
            Self::Inverted => 0.5,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct ReacquisitionHypothesis {
    doppler_bin: i8,
    code_bin: i8,
    carrier_sign: ReacquisitionCarrierSign,
    secondary_code_phase_periods: Option<usize>,
}

#[derive(Debug, Clone, Copy)]
struct ReacquisitionCandidate {
    hypothesis: ReacquisitionHypothesis,
    carrier_hz: f64,
    code_phase_samples: f64,
    cn0_dbhz: f64,
    prompt_power: f32,
}

impl ReacquisitionCandidate {
    fn seed(self) -> ReacquisitionSeed {
        ReacquisitionSeed {
            carrier_hz: self.carrier_hz,
            code_phase_samples: self.code_phase_samples,
            cn0_dbhz: self.cn0_dbhz,
            carrier_sign: self.hypothesis.carrier_sign,
            secondary_code_phase_periods: self.hypothesis.secondary_code_phase_periods,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum ReacquisitionSelection {
    Accepted(ReacquisitionSeed),
    Refused,
}

fn select_reacquisition_candidate(
    candidates: &[ReacquisitionCandidate],
    min_cn0_dbhz: f64,
    min_prompt_power: f32,
) -> ReacquisitionSelection {
    let mut eligible = candidates
        .iter()
        .copied()
        .filter(|candidate| {
            candidate.cn0_dbhz.is_finite()
                && candidate.prompt_power.is_finite()
                && (candidate.cn0_dbhz >= min_cn0_dbhz
                    || (candidate.cn0_dbhz
                        >= min_cn0_dbhz - REACQUISITION_PROMPT_EVIDENCE_CN0_MARGIN_DB
                        && candidate.prompt_power >= min_prompt_power))
        })
        .collect::<Vec<_>>();
    eligible.sort_by(|left, right| {
        right
            .cn0_dbhz
            .partial_cmp(&left.cn0_dbhz)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| {
                right
                    .prompt_power
                    .partial_cmp(&left.prompt_power)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .then_with(|| {
                left.hypothesis.doppler_bin.abs().cmp(&right.hypothesis.doppler_bin.abs())
            })
            .then_with(|| left.hypothesis.code_bin.abs().cmp(&right.hypothesis.code_bin.abs()))
    });

    let Some(best) = eligible.first().copied() else {
        return ReacquisitionSelection::Refused;
    };
    let ambiguous_alternate = eligible.iter().skip(1).any(|candidate| {
        candidate.cn0_dbhz >= best.cn0_dbhz - REACQUISITION_AMBIGUITY_CN0_MARGIN_DB
            && candidate.prompt_power
                >= best.prompt_power * REACQUISITION_AMBIGUITY_PROMPT_POWER_RATIO
            && !same_reacquisition_signal_location(candidate.hypothesis, best.hypothesis)
    });
    if ambiguous_alternate {
        ReacquisitionSelection::Refused
    } else {
        let selected = eligible
            .iter()
            .copied()
            .find(|candidate| {
                candidate.hypothesis.carrier_sign == ReacquisitionCarrierSign::Aligned
                    && same_reacquisition_signal_location(candidate.hypothesis, best.hypothesis)
            })
            .unwrap_or(best);
        ReacquisitionSelection::Accepted(selected.seed())
    }
}

fn same_reacquisition_signal_location(
    left: ReacquisitionHypothesis,
    right: ReacquisitionHypothesis,
) -> bool {
    left.doppler_bin == right.doppler_bin
        && (left.code_bin - right.code_bin).abs() <= 1
        && left.secondary_code_phase_periods == right.secondary_code_phase_periods
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct SustainedLockLossSeed {
    carrier_hz: f64,
    code_phase_samples: f64,
    code_rate_hz: f64,
    sample_index: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ReacquisitionOutcome {
    NotNeeded,
    SeedUnavailable,
    Failed,
    Started,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum LossOfLockCause {
    PromptPowerDrop,
    DiscriminatorInstability,
    PhaseJump,
}

impl LossOfLockCause {
    fn reason(self) -> &'static str {
        match self {
            Self::PromptPowerDrop => "prompt_power_drop",
            Self::DiscriminatorInstability => "discriminator_instability",
            Self::PhaseJump => "phase_jump",
        }
    }
}

fn reacquisition_min_cn0_dbhz(lock_reference_cn0_dbhz: f64) -> f64 {
    if !lock_reference_cn0_dbhz.is_finite() || lock_reference_cn0_dbhz <= 0.0 {
        return TRACKING_LOCK_MIN_CN0_DBHZ;
    }
    (lock_reference_cn0_dbhz - REACQUISITION_REFERENCE_CN0_MARGIN_DBHZ)
        .max(TRACKING_LOCK_MIN_CN0_DBHZ)
}

fn reacquisition_carrier_signs(
    signal_model: &TrackingSignalModel,
) -> &'static [ReacquisitionCarrierSign] {
    if signal_model.carrier_phase_transition_source().allows_half_cycle_transition() {
        &[ReacquisitionCarrierSign::Aligned, ReacquisitionCarrierSign::Inverted]
    } else {
        &[ReacquisitionCarrierSign::Aligned]
    }
}

fn reacquisition_secondary_code_phase_periods(
    signal_model: &TrackingSignalModel,
) -> Vec<Option<usize>> {
    let Some(component) = secondary_code_sync_component(signal_model) else {
        return vec![None];
    };
    let Some(period) = component.secondary_code_period() else {
        return vec![None];
    };
    (0..period.max(1)).map(Some).collect()
}
