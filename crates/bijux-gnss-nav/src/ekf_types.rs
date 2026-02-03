#[derive(Debug, Clone)]
pub struct EkfConfig {
    pub gating_chi2_code: Option<f64>,
    pub gating_chi2_phase: Option<f64>,
    pub gating_chi2_doppler: Option<f64>,
    pub huber_k: Option<f64>,
    pub square_root: bool,
    pub covariance_epsilon: f64,
    pub divergence_max_variance: f64,
}

#[derive(Debug, Clone)]
pub struct EkfHealth {
    pub innovation_rms: f64,
    pub rejected: usize,
    pub last_rejection: Option<String>,
    pub rejection_reasons: Vec<String>,
    pub last_rejection_code: Option<RejectionReason>,
    pub condition_number: Option<f64>,
    pub whiteness_ratio: Option<f64>,
    pub predicted_variance: Option<f64>,
    pub observed_variance: Option<f64>,
    pub events: Vec<NavHealthEvent>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RejectionReason {
    SingularS,
    Chi2Gate,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MeasurementKind {
    Code,
    Doppler,
    Phase,
}

