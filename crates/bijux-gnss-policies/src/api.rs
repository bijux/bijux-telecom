//! Public API for bijux-gnss-policies.

/// Run guardrail checks for a crate root.
pub use crate::guardrails::check;
/// Guardrail error type.
pub use crate::guardrails::error::GuardrailError;
/// Guardrail result type.
pub use crate::guardrails::error::Result;
/// Guardrail configuration structure.
pub use crate::guardrails::GuardrailConfig;
