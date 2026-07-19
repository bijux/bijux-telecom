//! Tracking-loop adaptation controller.

pub(crate) const WEAK_SIGNAL_ENTRY_CN0_DBHZ: f64 = 31.0;
pub(crate) const WEAK_SIGNAL_EXIT_CN0_DBHZ: f64 = 34.0;
pub(crate) const DYNAMIC_STRESS_ENTRY_FLL_ERROR_HZ: f64 = 12.0;
pub(crate) const DYNAMIC_STRESS_EXIT_FLL_ERROR_HZ: f64 = 6.0;
pub(crate) const DYNAMIC_STRESS_ENTRY_CARRIER_RATE_HZ_PER_S: f64 = 200.0;
pub(crate) const DYNAMIC_STRESS_EXIT_CARRIER_RATE_HZ_PER_S: f64 = 50.0;
pub(crate) const WEAK_SIGNAL_INTEGRATION_MS: u32 = 5;
pub(crate) const DYNAMIC_STRESS_INTEGRATION_MS: u32 = 1;
pub(crate) const MAX_TRACKING_ADAPTATION_PENDING_EPOCHS: u8 = 4;

/// Tracking loop profile selected by the adaptation controller.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrackingLoopProfileKind {
    /// Use the configured base loop parameters unchanged.
    Nominal,
    /// Narrow the loops and lengthen coherent integration for weak but stable signals.
    WeakSignal,
    /// Widen the loops and shorten coherent integration under strong dynamics.
    DynamicStress,
}

/// Loop parameters chosen for the next tracking epoch.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrackingLoopProfile {
    /// Effective DLL bandwidth in Hz.
    pub dll_bw_hz: f64,
    /// Effective PLL bandwidth in Hz.
    pub pll_bw_hz: f64,
    /// Effective FLL bandwidth in Hz.
    pub fll_bw_hz: f64,
    /// Effective coherent integration length in milliseconds.
    pub integration_ms: u32,
}

/// Inputs evaluated by the tracking adaptation controller.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrackingAdaptationInput {
    /// Current CN0 estimate in dB-Hz.
    pub cn0_dbhz: f64,
    /// Current FLL discriminator error in Hz.
    pub fll_error_hz: f64,
    /// Current carrier frequency-rate estimate in Hz/s.
    pub carrier_rate_hz_per_s: f64,
    /// Whether the carrier loop is stable enough to support longer coherent integration.
    pub carrier_lock_ready: bool,
    /// Whether the channel is already in a steady tracking state.
    pub steady_state_lock: bool,
    /// Whether recent discriminator behavior is stable.
    pub discriminator_stable: bool,
}

/// Stateful hysteresis memory for tracking adaptation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TrackingAdaptationState {
    /// Currently active loop profile.
    pub active_profile: TrackingLoopProfileKind,
    pub(crate) pending_profile: Option<TrackingLoopProfileKind>,
    pub(crate) pending_epochs: u8,
}

impl Default for TrackingAdaptationState {
    fn default() -> Self {
        Self {
            active_profile: TrackingLoopProfileKind::Nominal,
            pending_profile: None,
            pending_epochs: 0,
        }
    }
}

/// Result of one tracking adaptation step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrackingAdaptationDecision {
    /// Updated adaptation state to persist into the next epoch.
    pub state: TrackingAdaptationState,
    /// Active loop profile kind chosen for the next epoch.
    pub profile_kind: TrackingLoopProfileKind,
    /// Effective loop parameters chosen for the next epoch.
    pub profile: TrackingLoopProfile,
}

/// Advance the tracking adaptation controller by one epoch.
pub fn advance_tracking_adaptation(
    base_profile: TrackingLoopProfile,
    state: TrackingAdaptationState,
    input: TrackingAdaptationInput,
) -> TrackingAdaptationDecision {
    let requested_profile = requested_tracking_loop_profile(state.active_profile, input);
    let required_confirmation_epochs = required_confirmation_epochs(requested_profile);
    let mut next_state = state;

    if requested_profile == state.active_profile {
        next_state.pending_profile = None;
        next_state.pending_epochs = 0;
    } else if next_state.pending_profile == Some(requested_profile) {
        next_state.pending_epochs =
            next_state.pending_epochs.saturating_add(1).min(MAX_TRACKING_ADAPTATION_PENDING_EPOCHS);
    } else {
        next_state.pending_profile = Some(requested_profile);
        next_state.pending_epochs = 1;
    }

    if requested_profile != state.active_profile
        && next_state.pending_epochs >= required_confirmation_epochs
    {
        next_state.active_profile = requested_profile;
        next_state.pending_profile = None;
        next_state.pending_epochs = 0;
    }

    let profile = tracking_loop_profile(base_profile, next_state.active_profile);
    TrackingAdaptationDecision {
        state: next_state,
        profile_kind: next_state.active_profile,
        profile,
    }
}

fn tracking_loop_profile(
    base_profile: TrackingLoopProfile,
    profile_kind: TrackingLoopProfileKind,
) -> TrackingLoopProfile {
    match profile_kind {
        TrackingLoopProfileKind::Nominal => TrackingLoopProfile {
            integration_ms: base_profile.integration_ms.max(1),
            ..base_profile
        },
        TrackingLoopProfileKind::WeakSignal => TrackingLoopProfile {
            dll_bw_hz: base_profile.dll_bw_hz * 0.6,
            pll_bw_hz: base_profile.pll_bw_hz * 0.55,
            fll_bw_hz: base_profile.fll_bw_hz * 0.5,
            integration_ms: weak_signal_integration_ms(base_profile.integration_ms),
        },
        TrackingLoopProfileKind::DynamicStress => TrackingLoopProfile {
            dll_bw_hz: base_profile.dll_bw_hz * 2.0,
            pll_bw_hz: base_profile.pll_bw_hz * 3.5,
            fll_bw_hz: base_profile.fll_bw_hz * 4.0,
            integration_ms: DYNAMIC_STRESS_INTEGRATION_MS,
        },
    }
}

fn weak_signal_integration_ms(base_integration_ms: u32) -> u32 {
    base_integration_ms.clamp(WEAK_SIGNAL_INTEGRATION_MS, 10)
}

fn requested_tracking_loop_profile(
    active_profile: TrackingLoopProfileKind,
    input: TrackingAdaptationInput,
) -> TrackingLoopProfileKind {
    if weak_signal_requested(active_profile, input) {
        return TrackingLoopProfileKind::WeakSignal;
    }
    if dynamic_stress_requested(active_profile, input) {
        return TrackingLoopProfileKind::DynamicStress;
    }
    TrackingLoopProfileKind::Nominal
}

fn dynamic_stress_requested(
    active_profile: TrackingLoopProfileKind,
    input: TrackingAdaptationInput,
) -> bool {
    let fll_error_hz = input.fll_error_hz.abs();
    let carrier_rate_hz_per_s = input.carrier_rate_hz_per_s.abs();
    let reliable_dynamic_evidence = input.cn0_dbhz.is_finite()
        && (input.cn0_dbhz >= WEAK_SIGNAL_ENTRY_CN0_DBHZ || !input.steady_state_lock);
    let (fll_threshold_hz, carrier_rate_threshold_hz_per_s) =
        if active_profile == TrackingLoopProfileKind::DynamicStress {
            (DYNAMIC_STRESS_EXIT_FLL_ERROR_HZ, DYNAMIC_STRESS_EXIT_CARRIER_RATE_HZ_PER_S)
        } else {
            (DYNAMIC_STRESS_ENTRY_FLL_ERROR_HZ, DYNAMIC_STRESS_ENTRY_CARRIER_RATE_HZ_PER_S)
        };

    (reliable_dynamic_evidence
        && (fll_error_hz >= fll_threshold_hz
            || carrier_rate_hz_per_s >= carrier_rate_threshold_hz_per_s))
        || (!input.discriminator_stable
            && !input.steady_state_lock
            && input.cn0_dbhz.is_finite()
            && input.cn0_dbhz >= WEAK_SIGNAL_EXIT_CN0_DBHZ)
}

fn weak_signal_requested(
    active_profile: TrackingLoopProfileKind,
    input: TrackingAdaptationInput,
) -> bool {
    if !input.carrier_lock_ready || !input.steady_state_lock || !input.discriminator_stable {
        return false;
    }
    let cn0_threshold_dbhz = if active_profile == TrackingLoopProfileKind::WeakSignal {
        WEAK_SIGNAL_EXIT_CN0_DBHZ
    } else {
        WEAK_SIGNAL_ENTRY_CN0_DBHZ
    };
    input.cn0_dbhz.is_finite() && input.cn0_dbhz <= cn0_threshold_dbhz
}

fn required_confirmation_epochs(profile_kind: TrackingLoopProfileKind) -> u8 {
    match profile_kind {
        TrackingLoopProfileKind::Nominal => 4,
        TrackingLoopProfileKind::WeakSignal => 2,
        TrackingLoopProfileKind::DynamicStress => 1,
    }
}
