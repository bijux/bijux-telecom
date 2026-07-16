//! Public API for bijux-gnss-policies.

/// Run guardrail checks for a crate root.
pub use crate::guardrails::check;
/// Guardrail configuration structure.
pub use crate::guardrails::GuardrailConfig;
/// Guardrail error type.
pub use crate::guardrails::GuardrailError;
/// Guardrail result type.
pub use crate::guardrails::Result;
