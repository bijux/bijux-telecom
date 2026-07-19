const SAMPLE_RATE_MISMATCH_MIN_CN0_DBHZ: f64 = 18.0;
const TRACKING_LOCK_MIN_CN0_DBHZ: f64 = 28.0;
const TRACKING_LOCK_REFUSAL_EPOCHS: u8 = 3;
const SHORT_FADE_MAX_DURATION_S: f64 = 0.100;
const SAMPLE_RATE_MISMATCH_MIN_PHASE_DRIFT_FRACTION: f64 = 0.0025;
const SAMPLE_RATE_MISMATCH_MIN_PHASE_DRIFT_SAMPLES: f64 = 12.0;
const SAMPLE_RATE_MISMATCH_WINDOW_EPOCHS: usize = 4;
const SAMPLE_RATE_MISMATCH_MIN_UNSTABLE_EPOCHS_IN_WINDOW: usize = 3;
const CYCLE_SLIP_PHASE_DELTA_CYCLES: f64 = 0.35;
const NAV_BIT_PHASE_STEP_CYCLES: f64 = 0.5;
// Pull-in can still carry a few extra tenths of cycle of residual phase error
// when the first 20 ms nav-bit edge arrives on a strong high-Doppler signal.
// Keep the detector wide enough to classify that transition correctly instead
// of escalating it into a false cycle slip.
const NAV_BIT_PHASE_STEP_TOLERANCE_CYCLES: f64 = 0.2;
// Require a meaningful continuity improvement before rewriting a phase jump as
// a nav-bit edge. This keeps smaller non-nav slips from being mistaken for
// half-cycle data-bit transitions.
const NAV_BIT_PHASE_MIN_IMPROVEMENT_CYCLES: f64 = 0.25;
const DLL_FALSE_UNLOCK_PROBABILITY: f64 = 1.0e-6;
const PLL_FALSE_UNLOCK_PROBABILITY: f64 = 1.0e-6;
const FLL_FALSE_UNLOCK_PROBABILITY: f64 = 1.0e-6;
const PROMPT_POWER_DROP_RATIO_THRESHOLD: f32 = 0.2;
const DISCRIMINATOR_INSTABILITY_MIN_PROMPT_POWER_RATIO: f32 = 0.5;
const DISCRIMINATOR_INSTABILITY_REQUIRED_EPOCHS: u8 = 2;
const DEGRADED_FADE_INSTABILITY_GRACE_EPOCHS: u16 = 2;
const SHORT_FADE_RECOVERY_GRACE_EPOCHS: u16 = 5;
const NARROW_BPSK_EARLY_LATE_SPACING_CHIPS: f64 = 0.25;
const NARROW_HIGH_RATE_EARLY_LATE_SPACING_CHIPS: f64 = 0.10;
const NARROW_SUBCARRIER_EARLY_LATE_SPACING_CHIPS: f64 = 0.25;
const SECONDARY_CODE_EARLY_LATE_SPACING_CHIPS: f64 = 0.5;
// If prompt energy returns near the short-fade boundary, the loops may need a
// few extra epochs to re-enter tracking without opening a long interruption gap.
const SHORT_FADE_RELOCK_EVIDENCE_GRACE_EPOCHS: u16 = 3;
// Enter steady tracking only after the carrier/code loops stay jointly locked
// across a short sustained window instead of a single optimistic epoch.
const PULL_IN_REQUIRED_STABLE_EPOCHS: u8 = 3;
// FLL-assisted pull-in can still carry residual phase bias well above the
// steady-state PLL gate while the carrier frequency is otherwise converged.
// Admit tracking once the phase error stays within a bounded quadrant and the
// carrier estimators agree, then let the narrower PLL gate take over.
const CARRIER_CONVERGENCE_MAX_PHASE_ERROR_RAD: f32 = 1.6;
const REACQUISITION_REQUIRED_LOST_EPOCHS: usize = 3;
const REACQUISITION_CONFIRMATION_EPOCHS: u8 = 2;
const REACQUISITION_PULL_IN_EPOCH_BUDGET: u8 = 20;
const REACQUISITION_REFERENCE_CN0_MARGIN_DBHZ: f64 = 12.0;
const REACQUISITION_STABLE_TRACKING_EPOCHS: u8 = 5;
// Reacquisition must settle to a bounded residual Doppler uncertainty before
// the channel can claim steady recovered lock.
const REACQUISITION_STABLE_TRACKING_MAX_DOPPLER_UNCERTAINTY_HZ: f64 = 20.0;
const REACQUISITION_AMBIGUITY_CN0_MARGIN_DB: f64 = 1.5;
const REACQUISITION_AMBIGUITY_PROMPT_POWER_RATIO: f32 = 0.97;
const REACQUISITION_REFERENCE_PROMPT_POWER_RATIO: f32 = 0.25;
const REACQUISITION_PROMPT_EVIDENCE_CN0_MARGIN_DB: f64 = 3.0;
const TRACKING_CN0_WINDOW_EPOCHS: usize = 8;
const TRACKING_CN0_MIN_WINDOW_EPOCHS: usize = 4;
const TRACKING_UNCERTAINTY_WINDOW_EPOCHS: usize = 8;
const VECTOR_TRACKING_MIN_CONTRIBUTORS: usize = 2;
const COMMON_TRACKING_FREQUENCY_MIN_CONTRIBUTORS: usize = 2;
const COMMON_TRACKING_FREQUENCY_MAX_MEDIAN_RESIDUAL_HZ: f64 = 12.0;
const VECTOR_TRACKING_MIN_CN0_DBHZ: f64 = 35.0;
const VECTOR_TRACKING_HISTORY_SECONDS: f64 = 0.050;
const VECTOR_TRACKING_MAX_CARRIER_AID_HZ: f64 = 25.0;
const VECTOR_TRACKING_MAX_CODE_RATE_AID_HZ: f64 = 2.0;
const VECTOR_TRACKING_MAX_CODE_PHASE_AID_SAMPLES: f64 = 2.0;
const VECTOR_TRACKING_MAX_CARRIER_RATE_AID_HZ_PER_S: f64 = 1_000.0;
const SAMPLE_RATE_MISMATCH_CATASTROPHIC_PHASE_STEP_MULTIPLIER: f64 = 8.0;
const LOW_RESOLUTION_DLL_MIN_SAMPLE_SEPARATION: f64 = 1.0;
const JOINT_COMPONENT_MIN_PROMPT_RATIO: f32 = 0.35;
const SECONDARY_CODE_SYNC_MIN_CONFIDENCE: f64 = 0.02;
const SECONDARY_CODE_SYNC_MIN_OBSERVED_CHIPS: usize = 4;
const CARRIER_AID_MIN_DOPPLER_WINDOW_HZ: f64 = 25_000.0;
const CARRIER_AID_DOPPLER_WINDOW_MARGIN_HZ: f64 = 500.0;
const SUBCARRIER_AMBIGUITY_MIN_PROMPT_RELATIVE_POWER: f32 = 0.05;
const SUBCARRIER_AMBIGUITY_GUARD_OFFSETS_CHIPS: [f64; 2] = [-0.5, 0.5];
const DOPPLER_ESTIMATOR_SPREAD_LOCK_MULTIPLIER: f64 = 3.0;
const DOPPLER_ESTIMATOR_MIN_SPREAD_LIMIT_HZ: f64 = 75.0;
const DOPPLER_ESTIMATOR_PROVENANCE_TOKEN_COUNT: usize = 6;
