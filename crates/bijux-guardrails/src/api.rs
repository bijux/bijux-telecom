//! Public API for bijux-guardrails.

/// Run guardrail checks for a crate root.
pub use crate::internal::check;
/// Guardrail configuration structure.
pub use crate::internal::GuardrailConfig;
/// Guardrail error type.
pub use crate::internal::GuardrailError;
/// Guardrail result type.
pub use crate::internal::Result;
