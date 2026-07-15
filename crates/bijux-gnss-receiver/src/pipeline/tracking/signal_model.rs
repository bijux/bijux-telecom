struct TrackingStartContext {
    seed: AcqTrackingSeed,
    acquisition_hypothesis: String,
    acquisition_hypothesis_rank: u8,
    acquisition_score: f32,
    acquisition_code_phase_samples: usize,
    acquisition_carrier_hz: f64,
    acquisition_cn0_proxy_dbhz: f64,
    subcarrier_code_phase_refined: bool,
    acq_to_track_state: String,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct CarrierAidingReference {
    tracked_carrier_hz: f64,
    tracked_doppler_hz: f64,
    code_rate_hz: f64,
}

#[derive(Debug, Clone)]
struct TrackingSignalModel {
    signal_band: SignalBand,
    signal_code: SignalCode,
    glonass_frequency_channel: Option<bijux_gnss_core::api::GlonassFrequencyChannel>,
    signal_spec: SignalSpec,
    component_role: SignalComponentRole,
    secondary_code: Option<SignalSecondaryCodeSpec>,
    code_rate_hz: f64,
    code_length: usize,
    local_code_model: LocalCodeModel,
    discriminator_family: TrackingDiscriminatorFamily,
    phase_transition_source: TrackingPhaseTransitionSource,
    aiding_mode: TrackingAidingMode,
    pilot_component: Option<TrackingComponentModel>,
    data_symbol_component: Option<TrackingComponentModel>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TrackingAidingMode {
    None,
    PilotCarrier,
}

impl TrackingAidingMode {
    fn label(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::PilotCarrier => "pilot_carrier",
        }
    }
}

#[derive(Debug, Clone)]
struct TrackingComponentModel {
    role: SignalComponentRole,
    code_length: usize,
    phase_transition_source: TrackingPhaseTransitionSource,
    local_code_model: TrackingComponentLocalCodeModel,
}

impl TrackingComponentModel {
    fn sample_value_from_primary_phase(
        &self,
        primary_chip_phase: f64,
        primary_code_period_index: usize,
        primary_code_length: usize,
    ) -> f32 {
        let total_chip_phase =
            primary_code_period_index as f64 * primary_code_length as f64 + primary_chip_phase;
        let component_code_length = self.code_length.max(1) as f64;
        let component_chip_phase = total_chip_phase.rem_euclid(component_code_length);
        let component_primary_code_period_index = if total_chip_phase <= 0.0 {
            0
        } else {
            (total_chip_phase / component_code_length).floor() as usize
        };
        self.local_code_model
            .sample_tracking_value(component_chip_phase, component_primary_code_period_index)
            .unwrap_or(0.0)
    }

    fn secondary_code_period(&self) -> Option<usize> {
        self.local_code_model.secondary_code_period()
    }

    fn secondary_code_symbol(&self, primary_code_period_index: usize) -> Option<i8> {
        self.local_code_model.secondary_code_symbol(primary_code_period_index)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct SecondaryCodePromptSample {
    primary_code_period_index: usize,
    prompt: Complex<f32>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct SecondaryCodePhaseScore {
    phase_periods: usize,
    likelihood: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct SecondaryCodeSyncResult {
    phase_periods: usize,
    confidence: f64,
    best_likelihood: f64,
    next_best_likelihood: f64,
    observed_periods: usize,
    accepted: bool,
}

#[derive(Debug, Clone, PartialEq)]
enum TrackingComponentLocalCodeModel {
    Local(LocalCodeModel),
    GpsL2cCl { code: Vec<i8> },
    GalileoE5aQ { code: Vec<i8>, secondary_code: [i8; 100] },
    GalileoE5bQ { code: Vec<i8>, secondary_code: [i8; 100] },
}

impl TrackingComponentLocalCodeModel {
    fn sample_tracking_value(
        &self,
        chip_phase: f64,
        primary_code_period_index: usize,
    ) -> Result<f32, bijux_gnss_signal::api::SignalError> {
        match self {
            Self::Local(local_code_model) => {
                local_code_model.sample_tracking_value(chip_phase, primary_code_period_index)
            }
            Self::GpsL2cCl { code } => code_value_at_phase(code, chip_phase),
            Self::GalileoE5aQ { code, secondary_code } => {
                Ok(code_value_at_phase(code, chip_phase)?
                    * galileo_e5a_q_epoch_symbol(secondary_code, primary_code_period_index) as f32
                    * std::f32::consts::FRAC_1_SQRT_2)
            }
            Self::GalileoE5bQ { code, secondary_code } => {
                Ok(code_value_at_phase(code, chip_phase)?
                    * galileo_e5b_q_epoch_symbol(secondary_code, primary_code_period_index) as f32
                    * std::f32::consts::FRAC_1_SQRT_2)
            }
        }
    }

    fn secondary_code_period(&self) -> Option<usize> {
        match self {
            Self::Local(LocalCodeModel::GpsL5Q { .. }) => Some(GPS_L5_Q_PRIMARY_EPOCHS_PER_SYMBOL),
            Self::GalileoE5aQ { .. } => Some(GALILEO_E5A_Q_SECONDARY_CODE_CHIPS),
            Self::GalileoE5bQ { .. } => Some(GALILEO_E5B_Q_SECONDARY_CODE_CHIPS),
            Self::Local(_) | Self::GpsL2cCl { .. } => None,
        }
    }

    fn secondary_code_symbol(&self, primary_code_period_index: usize) -> Option<i8> {
        match self {
            Self::Local(LocalCodeModel::GpsL5Q { .. }) => {
                Some(gps_l5_q_epoch_symbol(primary_code_period_index))
            }
            Self::GalileoE5aQ { secondary_code, .. } => {
                Some(galileo_e5a_q_epoch_symbol(secondary_code, primary_code_period_index))
            }
            Self::GalileoE5bQ { secondary_code, .. } => {
                Some(galileo_e5b_q_epoch_symbol(secondary_code, primary_code_period_index))
            }
            Self::Local(_) | Self::GpsL2cCl { .. } => None,
        }
    }
}

fn secondary_code_sync_from_prompt_history(
    component: &TrackingComponentModel,
    prompt_history: &[SecondaryCodePromptSample],
) -> Option<SecondaryCodeSyncResult> {
    let period = component.secondary_code_period()?;
    if period == 0 || prompt_history.len() < SECONDARY_CODE_SYNC_MIN_OBSERVED_CHIPS {
        return None;
    }
    let mut scores = (0..period)
        .map(|phase| secondary_code_phase_score(component, prompt_history, phase))
        .collect::<Vec<_>>();
    scores.sort_by(|left, right| {
        right
            .likelihood
            .partial_cmp(&left.likelihood)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| left.phase_periods.cmp(&right.phase_periods))
    });
    let best = scores.first().copied()?;
    let next_best = scores
        .get(1)
        .copied()
        .unwrap_or(SecondaryCodePhaseScore { phase_periods: best.phase_periods, likelihood: 0.0 });
    let confidence = secondary_code_sync_confidence(best.likelihood, next_best.likelihood);
    Some(SecondaryCodeSyncResult {
        phase_periods: best.phase_periods,
        confidence,
        best_likelihood: best.likelihood,
        next_best_likelihood: next_best.likelihood,
        observed_periods: prompt_history.len(),
        accepted: confidence >= SECONDARY_CODE_SYNC_MIN_CONFIDENCE,
    })
}

fn secondary_code_phase_score(
    component: &TrackingComponentModel,
    prompt_history: &[SecondaryCodePromptSample],
    phase_periods: usize,
) -> SecondaryCodePhaseScore {
    let mut accumulated_alignment = 0.0_f64;
    let mut transition_count = 0usize;
    let mut previous_prompt: Option<Complex<f64>> = None;
    let reference_period =
        prompt_history.first().map(|sample| sample.primary_code_period_index).unwrap_or_default();
    for sample in prompt_history {
        let relative_period = sample.primary_code_period_index.saturating_sub(reference_period);
        let Some(observed_symbol) =
            component.secondary_code_symbol(sample.primary_code_period_index)
        else {
            continue;
        };
        let Some(candidate_symbol) =
            component.secondary_code_symbol(phase_periods + relative_period)
        else {
            continue;
        };
        let raw_prompt = sample.prompt * observed_symbol as f32;
        let candidate_prompt = Complex::new(
            (raw_prompt.re * candidate_symbol as f32) as f64,
            (raw_prompt.im * candidate_symbol as f32) as f64,
        );
        if let Some(previous_prompt) = previous_prompt {
            let norm_product = previous_prompt.norm() * candidate_prompt.norm();
            if norm_product > f64::EPSILON {
                let alignment = (previous_prompt.conj() * candidate_prompt).re / norm_product;
                accumulated_alignment += ((alignment + 1.0) * 0.5).clamp(0.0, 1.0);
                transition_count += 1;
            }
        }
        previous_prompt = Some(candidate_prompt);
    }
    let likelihood =
        if transition_count > 0 { accumulated_alignment / transition_count as f64 } else { 0.0 };
    SecondaryCodePhaseScore { phase_periods, likelihood }
}

fn secondary_code_sync_confidence(best_likelihood: f64, next_best_likelihood: f64) -> f64 {
    if best_likelihood <= f64::EPSILON {
        return 0.0;
    }
    ((best_likelihood - next_best_likelihood.max(0.0)) / best_likelihood).clamp(0.0, 1.0)
}

fn secondary_code_sync_component(
    signal_model: &TrackingSignalModel,
) -> Option<TrackingComponentModel> {
    let component = signal_model.carrier_component();
    (component.phase_transition_source == TrackingPhaseTransitionSource::SecondaryCode
        && component.secondary_code_period().is_some())
    .then_some(component)
}

fn update_secondary_code_synchronization(
    signal_model: &TrackingSignalModel,
    state: &mut LoopState,
    primary_code_period_index: usize,
    carrier_prompt: Complex<f32>,
) -> Option<SecondaryCodeSyncResult> {
    let component = secondary_code_sync_component(signal_model)?;
    let period = component.secondary_code_period()?;
    if !carrier_prompt.re.is_finite() || !carrier_prompt.im.is_finite() {
        return state.secondary_code_sync;
    }
    state
        .secondary_code_prompt_history
        .push_back(SecondaryCodePromptSample { primary_code_period_index, prompt: carrier_prompt });
    while state.secondary_code_prompt_history.len() > period {
        state.secondary_code_prompt_history.pop_front();
    }
    let prompt_history = state.secondary_code_prompt_history.iter().copied().collect::<Vec<_>>();
    state.secondary_code_sync = select_secondary_code_synchronization(
        state.secondary_code_sync,
        secondary_code_sync_from_prompt_history(&component, &prompt_history),
    );
    state.secondary_code_sync
}

fn select_secondary_code_synchronization(
    current: Option<SecondaryCodeSyncResult>,
    candidate: Option<SecondaryCodeSyncResult>,
) -> Option<SecondaryCodeSyncResult> {
    match (current, candidate) {
        (None, candidate) => candidate,
        (current @ Some(_), None) => current,
        (Some(current), Some(candidate)) if candidate.accepted && !current.accepted => {
            Some(candidate)
        }
        (Some(current), Some(candidate))
            if candidate.accepted == current.accepted
                && candidate.confidence > current.confidence =>
        {
            Some(candidate)
        }
        (Some(current), Some(_)) => Some(current),
    }
}

fn carrier_phase_transition_source_for_prompt(
    signal_model: &TrackingSignalModel,
    secondary_code_sync: Option<SecondaryCodeSyncResult>,
) -> TrackingPhaseTransitionSource {
    let transition_source = signal_model.carrier_phase_transition_source();
    if transition_source != TrackingPhaseTransitionSource::SecondaryCode {
        return transition_source;
    }
    if secondary_code_sync.is_some_and(|sync| sync.accepted) {
        TrackingPhaseTransitionSource::SecondaryCode
    } else {
        TrackingPhaseTransitionSource::None
    }
}

fn secondary_code_sync_provenance(
    signal_model: &TrackingSignalModel,
    sync: Option<SecondaryCodeSyncResult>,
) -> Option<String> {
    secondary_code_sync_component(signal_model)?;
    Some(match sync {
        Some(sync) => format!(
            " secondary_code_sync={} secondary_code_phase_periods={} secondary_code_sync_confidence={:.6} secondary_code_best_likelihood={:.6} secondary_code_next_likelihood={:.6} secondary_code_observed_periods={}",
            if sync.accepted { "accepted" } else { "rejected" },
            sync.phase_periods,
            sync.confidence,
            sync.best_likelihood,
            sync.next_best_likelihood,
            sync.observed_periods
        ),
        None => " secondary_code_sync=insufficient".to_string(),
    })
}

fn prompt_center_primary_code_period_index(
    epoch_primary_code_period_index: usize,
    base_chip_phase: f64,
    tracked_chips_per_sample: f64,
    coherent_samples: usize,
    primary_code_length: usize,
) -> usize {
    let code_length = primary_code_length.max(1) as f64;
    let prompt_center_chip_phase =
        base_chip_phase + tracked_chips_per_sample * coherent_samples as f64 * 0.5;
    let period_offset = (prompt_center_chip_phase / code_length).floor().max(0.0) as usize;
    epoch_primary_code_period_index.saturating_add(period_offset)
}

#[derive(Debug, Clone, Copy)]
struct TrackingEpochCorrelation {
    primary: CorrelatorOutput,
    double_delta_outer: Option<CorrelatorOutput>,
    carrier_prompt: Complex<f32>,
    carrier_prompt_source: CarrierPromptSource,
    data_prompt: Option<Complex<f32>>,
    secondary_code_prompt_period_index: usize,
    subcarrier_ambiguity_guard: Option<SubcarrierAmbiguityGuard>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CarrierPromptSource {
    Primary,
    Pilot,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct SubcarrierAmbiguityGuard {
    prompt_power: f32,
    strongest_alternate_power: f32,
    strongest_alternate_offset_chips: f64,
    prompt_relative_power: f32,
}

impl SubcarrierAmbiguityGuard {
    fn detected(self) -> bool {
        !self.prompt_relative_power.is_finite()
            || self.prompt_relative_power < SUBCARRIER_AMBIGUITY_MIN_PROMPT_RELATIVE_POWER
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TrackingDiscriminatorFamily {
    EarlyPromptLate,
    BocEarlyPromptLate,
    CbocEarlyPromptLate,
}

impl TrackingDiscriminatorFamily {
    fn label(self) -> &'static str {
        match self {
            Self::EarlyPromptLate => "early_prompt_late",
            Self::BocEarlyPromptLate => "unambiguous_boc_early_prompt_late",
            Self::CbocEarlyPromptLate => "unambiguous_cboc_early_prompt_late",
        }
    }

    fn requires_unambiguous_code_lock(self) -> bool {
        matches!(self, Self::BocEarlyPromptLate | Self::CbocEarlyPromptLate)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CodeDiscriminatorMode {
    EarlyPromptLate,
    DoubleDeltaEarlyPromptLate,
}

impl CodeDiscriminatorMode {
    fn label(self) -> &'static str {
        match self {
            Self::EarlyPromptLate => "early_prompt_late",
            Self::DoubleDeltaEarlyPromptLate => "double_delta_early_prompt_late",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TrackingPhaseTransitionSource {
    None,
    DataSymbol,
    SecondaryCode,
}

impl TrackingPhaseTransitionSource {
    fn allows_half_cycle_transition(self) -> bool {
        !matches!(self, Self::None)
    }

    fn reports_navigation_bit_lock(self) -> bool {
        matches!(self, Self::DataSymbol)
    }

    fn label(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::DataSymbol => "data_symbol",
            Self::SecondaryCode => "secondary_code",
        }
    }
}

impl TrackingSignalModel {
    fn for_sat(config: &ReceiverPipelineConfig, sat: SatId) -> Self {
        match sat.constellation {
            Constellation::Gps => {
                Self::for_sat_signal_band(config, sat, SignalBand::L1, SignalCode::Ca, None)
            }
            Constellation::Galileo => {
                Self::for_sat_signal_band(config, sat, SignalBand::E1, SignalCode::E1B, None)
            }
            Constellation::Beidou => {
                Self::for_sat_signal_band(config, sat, SignalBand::B1, SignalCode::B1I, None)
            }
            Constellation::Glonass => {
                Self::for_sat_signal_band(config, sat, SignalBand::L1, SignalCode::Unknown, None)
            }
            _ => Self::fallback(config, sat, SignalBand::L1),
        }
    }

    fn for_sat_signal_band(
        config: &ReceiverPipelineConfig,
        sat: SatId,
        signal_band: SignalBand,
        signal_code: SignalCode,
        glonass_frequency_channel: Option<bijux_gnss_core::api::GlonassFrequencyChannel>,
    ) -> Self {
        let signal_code = if signal_code == SignalCode::Unknown {
            default_signal_code_for_band(sat.constellation, signal_band)
        } else {
            signal_code
        };
        let registry_entry = resolved_signal_registry_entry(
            sat,
            signal_band,
            signal_code,
            glonass_frequency_channel,
        )
        .ok()
        .flatten();
        match (
            default_local_code_model_for_signal(sat, signal_band, signal_code).ok().flatten(),
            registry_entry,
        ) {
            (Some(local_code_model), Some(registry_entry)) => {
                let component = registry_entry
                    .default_component()
                    .expect("tracking registry entry must expose a default component");
                let (aiding_mode, pilot_component, data_symbol_component) =
                    joint_tracking_components(
                        sat,
                        signal_band,
                        signal_code,
                        glonass_frequency_channel,
                        &registry_entry,
                        component,
                    );
                Self {
                    signal_band,
                    signal_code,
                    glonass_frequency_channel: ((sat.constellation == Constellation::Glonass)
                        && (signal_band == SignalBand::L1))
                        .then_some(glonass_frequency_channel)
                        .flatten(),
                    signal_spec: registry_entry.spec,
                    component_role: component.role,
                    secondary_code: component.secondary_code,
                    code_rate_hz: component.primary_code_rate_hz,
                    code_length: component.primary_code_chips as usize,
                    local_code_model,
                    discriminator_family: tracking_discriminator_family(component.subcarrier),
                    phase_transition_source: tracking_phase_transition_source(component),
                    aiding_mode,
                    pilot_component,
                    data_symbol_component,
                }
            }
            _ => Self::fallback(config, sat, signal_band),
        }
    }

    fn fallback(config: &ReceiverPipelineConfig, sat: SatId, signal_band: SignalBand) -> Self {
        let local_code_model = if sat.constellation == Constellation::Gps {
            LocalCodeModel::gps_l1_ca_or_ones(sat.prn)
        } else {
            LocalCodeModel::ones(config.code_length.max(1), config.code_freq_basis_hz)
        };
        Self {
            signal_band,
            signal_code: default_signal_code_for_band(sat.constellation, signal_band),
            glonass_frequency_channel: None,
            signal_spec: SignalSpec {
                constellation: sat.constellation,
                band: signal_band,
                code: default_signal_code_for_band(sat.constellation, signal_band),
                code_rate_hz: config.code_freq_basis_hz,
                carrier_hz: FreqHz::new(0.0),
            },
            component_role: SignalComponentRole::Data,
            secondary_code: None,
            code_rate_hz: config.code_freq_basis_hz,
            code_length: config.code_length.max(1),
            local_code_model,
            discriminator_family: TrackingDiscriminatorFamily::EarlyPromptLate,
            phase_transition_source: TrackingPhaseTransitionSource::None,
            aiding_mode: TrackingAidingMode::None,
            pilot_component: None,
            data_symbol_component: None,
        }
    }

    fn supports_tracking(
        sat: SatId,
        signal_band: SignalBand,
        signal_code: SignalCode,
        glonass_frequency_channel: Option<bijux_gnss_core::api::GlonassFrequencyChannel>,
    ) -> bool {
        default_local_code_model_for_signal(sat, signal_band, signal_code).ok().flatten().is_some()
            && resolved_signal_registry_entry(
                sat,
                signal_band,
                signal_code,
                glonass_frequency_channel,
            )
            .ok()
            .flatten()
            .and_then(|entry| entry.default_component().copied())
            .is_some()
    }

    fn samples_per_code(&self, sample_rate_hz: f64) -> usize {
        samples_per_code(sample_rate_hz, self.code_rate_hz, self.code_length)
    }

    fn tracking_epoch_samples(
        &self,
        sample_rate_hz: f64,
        tracking_params: TrackingParams,
    ) -> usize {
        self.samples_per_code(sample_rate_hz)
            .saturating_mul(tracking_params.integration_ms.max(1) as usize)
    }

    fn nominal_chips_per_sample(&self, sample_rate_hz: f64) -> f64 {
        self.code_rate_hz / sample_rate_hz
    }

    fn value_at_phase(&self, chip_phase: f64, epoch_primary_code_period_index: usize) -> f32 {
        let code_length = self.code_length.max(1) as f64;
        let period_offset = (chip_phase / code_length).floor() as i64;
        let primary_code_period_index =
            (epoch_primary_code_period_index as i64 + period_offset).max(0) as usize;
        let wrapped_chip_phase = chip_phase.rem_euclid(code_length);
        self.local_code_model
            .sample_tracking_value(wrapped_chip_phase, primary_code_period_index)
            .unwrap_or(0.0)
    }

    fn nominal_carrier_hz(&self) -> f64 {
        self.signal_spec.carrier_hz.0
    }

    fn carrier_phase_transition_source(&self) -> TrackingPhaseTransitionSource {
        self.carrier_component().phase_transition_source
    }

    fn carrier_component(&self) -> TrackingComponentModel {
        self.pilot_component.clone().unwrap_or_else(|| TrackingComponentModel {
            role: self.component_role,
            code_length: self.code_length,
            phase_transition_source: self.phase_transition_source,
            local_code_model: TrackingComponentLocalCodeModel::Local(self.local_code_model.clone()),
        })
    }

    fn data_symbol_component(&self) -> Option<&TrackingComponentModel> {
        self.data_symbol_component.as_ref()
    }

    fn supports_epoch_data_symbol_sign_recovery(&self) -> bool {
        self.data_symbol_component.is_some() && !self.supports_navigation_bit_sign_recovery()
    }

    fn supports_navigation_bit_sign_recovery(&self) -> bool {
        self.signal_spec.constellation == Constellation::Gps
            && self.signal_band == SignalBand::L1
            && self.signal_code == SignalCode::Ca
            && self.phase_transition_source.reports_navigation_bit_lock()
    }
}

fn signal_default_early_late_spacing_chips(signal_model: &TrackingSignalModel) -> f64 {
    match signal_model.discriminator_family {
        TrackingDiscriminatorFamily::BocEarlyPromptLate
        | TrackingDiscriminatorFamily::CbocEarlyPromptLate => {
            NARROW_SUBCARRIER_EARLY_LATE_SPACING_CHIPS
        }
        TrackingDiscriminatorFamily::EarlyPromptLate
            if signal_model.phase_transition_source
                == TrackingPhaseTransitionSource::SecondaryCode =>
        {
            SECONDARY_CODE_EARLY_LATE_SPACING_CHIPS
        }
        TrackingDiscriminatorFamily::EarlyPromptLate
            if signal_model.code_rate_hz >= 10_000_000.0 - f64::EPSILON =>
        {
            NARROW_HIGH_RATE_EARLY_LATE_SPACING_CHIPS
        }
        TrackingDiscriminatorFamily::EarlyPromptLate => NARROW_BPSK_EARLY_LATE_SPACING_CHIPS,
    }
}

fn resolve_signal_tracking_params(
    config: &ReceiverPipelineConfig,
    signal_model: &TrackingSignalModel,
) -> TrackingParams {
    let mut params = config.tracking_params(signal_model.signal_band);
    if config.tracking_per_band.iter().any(|profile| profile.band == signal_model.signal_band) {
        return params;
    }
    params.early_late_spacing_chips =
        params.early_late_spacing_chips.min(signal_default_early_late_spacing_chips(signal_model));
    params
}

fn code_discriminator_mode(signal_model: &TrackingSignalModel) -> CodeDiscriminatorMode {
    match signal_model.discriminator_family {
        TrackingDiscriminatorFamily::EarlyPromptLate
            if signal_model.phase_transition_source
                != TrackingPhaseTransitionSource::SecondaryCode =>
        {
            CodeDiscriminatorMode::DoubleDeltaEarlyPromptLate
        }
        TrackingDiscriminatorFamily::EarlyPromptLate => CodeDiscriminatorMode::EarlyPromptLate,
        TrackingDiscriminatorFamily::BocEarlyPromptLate
        | TrackingDiscriminatorFamily::CbocEarlyPromptLate => {
            CodeDiscriminatorMode::EarlyPromptLate
        }
    }
}

fn tracking_dll_discriminator(correlation: &TrackingEpochCorrelation) -> f32 {
    let (early_late_dll_err, _, _, _) = discriminators(
        correlation.primary.early,
        correlation.primary.prompt,
        correlation.primary.late,
        None,
    );
    if let Some(outer) = correlation.double_delta_outer {
        let double_delta_dll_err = double_delta_dll_discriminator(
            correlation.primary.early,
            correlation.primary.late,
            outer.early,
            outer.late,
        );
        if double_delta_dll_err.abs() < early_late_dll_err.abs() {
            return double_delta_dll_err;
        }
        return early_late_dll_err;
    }
    early_late_dll_err
}

fn joint_tracking_components(
    sat: SatId,
    signal_band: SignalBand,
    signal_code: SignalCode,
    glonass_frequency_channel: Option<bijux_gnss_core::api::GlonassFrequencyChannel>,
    registry_entry: &bijux_gnss_core::api::SignalRegistryEntry,
    primary_component: &bijux_gnss_core::api::SignalComponentSpec,
) -> (TrackingAidingMode, Option<TrackingComponentModel>, Option<TrackingComponentModel>) {
    let data_component = registry_entry.component(SignalComponentRole::Data);
    let pilot_component = registry_entry.component(SignalComponentRole::Pilot);

    if data_component.is_some() && pilot_component.is_some() {
        let built_pilot = (primary_component.role != SignalComponentRole::Pilot)
            .then(|| {
                tracking_component_model_for_signal_role(
                    sat,
                    signal_band,
                    signal_code,
                    SignalComponentRole::Pilot,
                    glonass_frequency_channel,
                )
            })
            .flatten();
        let built_data = (primary_component.role == SignalComponentRole::Pilot)
            .then(|| {
                tracking_component_model_for_signal_role(
                    sat,
                    signal_band,
                    signal_code,
                    SignalComponentRole::Data,
                    glonass_frequency_channel,
                )
            })
            .flatten()
            .or_else(|| {
                (primary_component.role == SignalComponentRole::Data).then(|| {
                    TrackingComponentModel {
                        role: SignalComponentRole::Data,
                        code_length: primary_component.primary_code_chips as usize,
                        phase_transition_source: tracking_phase_transition_source(
                            primary_component,
                        ),
                        local_code_model: TrackingComponentLocalCodeModel::Local(
                            default_local_code_model_for_signal(sat, signal_band, signal_code)
                                .ok()
                                .flatten()
                                .expect("primary data component local code model"),
                        ),
                    }
                })
            });
        return (TrackingAidingMode::PilotCarrier, built_pilot, built_data);
    }

    let companion_signal_code = complementary_joint_tracking_signal_code(signal_band, signal_code);
    let Some(companion_signal_code) = companion_signal_code else {
        return (TrackingAidingMode::None, None, None);
    };
    let companion_role = if primary_component.role == SignalComponentRole::Data {
        SignalComponentRole::Pilot
    } else {
        SignalComponentRole::Data
    };
    let companion_component = tracking_component_model_for_signal_role(
        sat,
        signal_band,
        companion_signal_code,
        companion_role,
        glonass_frequency_channel,
    );
    match (primary_component.role, companion_component) {
        (SignalComponentRole::Data, Some(pilot_component)) => (
            TrackingAidingMode::PilotCarrier,
            Some(pilot_component),
            default_local_code_model_for_signal(sat, signal_band, signal_code).ok().flatten().map(
                |local_code_model| TrackingComponentModel {
                    role: SignalComponentRole::Data,
                    code_length: primary_component.primary_code_chips as usize,
                    phase_transition_source: tracking_phase_transition_source(primary_component),
                    local_code_model: TrackingComponentLocalCodeModel::Local(local_code_model),
                },
            ),
        ),
        (SignalComponentRole::Pilot, Some(data_component)) => {
            (TrackingAidingMode::PilotCarrier, None, Some(data_component))
        }
        _ => (TrackingAidingMode::None, None, None),
    }
}

fn complementary_joint_tracking_signal_code(
    signal_band: SignalBand,
    signal_code: SignalCode,
) -> Option<SignalCode> {
    match (signal_band, signal_code) {
        (SignalBand::L5, SignalCode::L5I) => Some(SignalCode::L5Q),
        (SignalBand::L5, SignalCode::L5Q) => Some(SignalCode::L5I),
        _ => None,
    }
}

fn tracking_component_model_for_signal_role(
    sat: SatId,
    signal_band: SignalBand,
    signal_code: SignalCode,
    role: SignalComponentRole,
    glonass_frequency_channel: Option<bijux_gnss_core::api::GlonassFrequencyChannel>,
) -> Option<TrackingComponentModel> {
    let registry_entry =
        resolved_signal_registry_entry(sat, signal_band, signal_code, glonass_frequency_channel)
            .ok()
            .flatten()?;
    let component = registry_entry.component(role)?;
    let local_code_model =
        tracking_component_local_code_model_for_signal_role(sat, signal_band, signal_code, role)?;
    Some(TrackingComponentModel {
        role,
        code_length: component.primary_code_chips as usize,
        phase_transition_source: tracking_phase_transition_source(component),
        local_code_model,
    })
}

fn tracking_component_local_code_model_for_signal_role(
    sat: SatId,
    signal_band: SignalBand,
    signal_code: SignalCode,
    role: SignalComponentRole,
) -> Option<TrackingComponentLocalCodeModel> {
    match (sat.constellation, signal_band, signal_code, role) {
        (Constellation::Gps, SignalBand::L2, SignalCode::L2C, SignalComponentRole::Pilot) => {
            Some(TrackingComponentLocalCodeModel::GpsL2cCl {
                code: generate_gps_l2c_cl_code(sat.prn).ok()?,
            })
        }
        (Constellation::Galileo, SignalBand::E5, SignalCode::E5a, SignalComponentRole::Pilot) => {
            Some(TrackingComponentLocalCodeModel::GalileoE5aQ {
                code: generate_galileo_e5a_q_code(sat.prn).ok()?,
                secondary_code: galileo_e5a_q_secondary_code(sat.prn).ok()?,
            })
        }
        (Constellation::Galileo, SignalBand::E5, SignalCode::E5b, SignalComponentRole::Pilot) => {
            Some(TrackingComponentLocalCodeModel::GalileoE5bQ {
                code: generate_galileo_e5b_q_code(sat.prn).ok()?,
                secondary_code: galileo_e5b_q_secondary_code(sat.prn).ok()?,
            })
        }
        _ => default_local_code_model_for_signal(sat, signal_band, signal_code)
            .ok()
            .flatten()
            .map(TrackingComponentLocalCodeModel::Local),
    }
}

fn tracking_discriminator_family(subcarrier: SignalSubcarrierSpec) -> TrackingDiscriminatorFamily {
    match subcarrier {
        SignalSubcarrierSpec::None => TrackingDiscriminatorFamily::EarlyPromptLate,
        SignalSubcarrierSpec::Boc { .. } => TrackingDiscriminatorFamily::BocEarlyPromptLate,
        SignalSubcarrierSpec::Cboc { .. } => TrackingDiscriminatorFamily::CbocEarlyPromptLate,
    }
}

fn subcarrier_ambiguity_guard(
    signal_model: &TrackingSignalModel,
    samples: &[Complex<f32>],
    sample_rate_hz: f64,
    carrier_freq_hz: f64,
    carrier_phase_cycles: f64,
    base_chip_phase: f64,
    tracked_chips_per_sample: f64,
    epoch_primary_code_period_index: usize,
    prompt: Complex<f32>,
) -> Option<SubcarrierAmbiguityGuard> {
    if !signal_model.discriminator_family.requires_unambiguous_code_lock() {
        return None;
    }

    let prompt_power = prompt.norm_sqr();
    if !prompt_power.is_finite() {
        return Some(SubcarrierAmbiguityGuard {
            prompt_power,
            strongest_alternate_power: f32::INFINITY,
            strongest_alternate_offset_chips: 0.0,
            prompt_relative_power: 0.0,
        });
    }

    let mut strongest_alternate_power = 0.0_f32;
    let mut strongest_alternate_offset_chips = 0.0_f64;
    for offset_chips in SUBCARRIER_AMBIGUITY_GUARD_OFFSETS_CHIPS {
        let alternate = correlate_early_prompt_late(
            samples,
            sample_rate_hz,
            carrier_freq_hz,
            carrier_phase_offset_radians(carrier_phase_cycles),
            base_chip_phase + offset_chips,
            tracked_chips_per_sample,
            0.0,
            |chip_phase| signal_model.value_at_phase(chip_phase, epoch_primary_code_period_index),
        )
        .prompt;
        let alternate_power = alternate.norm_sqr();
        if alternate_power.is_finite() && alternate_power > strongest_alternate_power {
            strongest_alternate_power = alternate_power;
            strongest_alternate_offset_chips = offset_chips;
        }
    }

    let prompt_relative_power = if strongest_alternate_power > f32::EPSILON {
        prompt_power / strongest_alternate_power
    } else if prompt_power.is_finite() {
        f32::INFINITY
    } else {
        0.0
    };

    Some(SubcarrierAmbiguityGuard {
        prompt_power,
        strongest_alternate_power,
        strongest_alternate_offset_chips,
        prompt_relative_power,
    })
}

fn subcarrier_ambiguity_detected(guard: Option<SubcarrierAmbiguityGuard>) -> bool {
    guard.is_some_and(SubcarrierAmbiguityGuard::detected)
}

fn subcarrier_ambiguity_provenance(guard: Option<SubcarrierAmbiguityGuard>) -> Option<String> {
    let guard = guard?;
    Some(format!(
        " subcarrier_ambiguity_guard=side_peak_guard prompt_relative_power={:.6} strongest_alternate_offset_chips={:.6} strongest_alternate_power={:.6}",
        guard.prompt_relative_power,
        guard.strongest_alternate_offset_chips,
        guard.strongest_alternate_power,
    ))
}

fn tracking_phase_transition_source(
    component: &bijux_gnss_core::api::SignalComponentSpec,
) -> TrackingPhaseTransitionSource {
    if component.secondary_code.is_some() {
        TrackingPhaseTransitionSource::SecondaryCode
    } else if component.symbol_period_s.is_some() && component.role == SignalComponentRole::Data {
        TrackingPhaseTransitionSource::DataSymbol
    } else {
        TrackingPhaseTransitionSource::None
    }
}

fn select_carrier_prompt(
    primary_prompt: Complex<f32>,
    pilot_prompt: Option<Complex<f32>>,
    aiding_mode: TrackingAidingMode,
    require_pilot: bool,
) -> (Complex<f32>, CarrierPromptSource) {
    if aiding_mode != TrackingAidingMode::PilotCarrier {
        return (primary_prompt, CarrierPromptSource::Primary);
    }
    let Some(pilot_prompt) = pilot_prompt else {
        return (primary_prompt, CarrierPromptSource::Primary);
    };
    let primary_norm = primary_prompt.norm();
    let pilot_norm = pilot_prompt.norm();
    if require_pilot && pilot_norm > f32::EPSILON {
        return (pilot_prompt, CarrierPromptSource::Pilot);
    }
    if primary_norm <= f32::EPSILON {
        return (pilot_prompt, CarrierPromptSource::Pilot);
    }
    if pilot_norm >= primary_norm * JOINT_COMPONENT_MIN_PROMPT_RATIO {
        (pilot_prompt, CarrierPromptSource::Pilot)
    } else {
        (primary_prompt, CarrierPromptSource::Primary)
    }
}

pub(crate) fn supports_tracking_signal(
    sat: SatId,
    signal_band: SignalBand,
    signal_code: SignalCode,
) -> bool {
    supports_tracking_signal_with_channel(sat, signal_band, signal_code, None)
}

pub(crate) fn supports_tracking_signal_with_channel(
    sat: SatId,
    signal_band: SignalBand,
    signal_code: SignalCode,
    glonass_frequency_channel: Option<bijux_gnss_core::api::GlonassFrequencyChannel>,
) -> bool {
    TrackingSignalModel::supports_tracking(sat, signal_band, signal_code, glonass_frequency_channel)
}

