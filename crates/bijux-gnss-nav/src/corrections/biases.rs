use bijux_gnss_core::SigId;

#[derive(Debug, Clone)]
pub struct CodeBias {
    pub sig: SigId,
    pub bias_m: f64,
}

#[derive(Debug, Clone)]
pub struct PhaseBias {
    pub sig: SigId,
    pub bias_cycles: f64,
}

pub trait CodeBiasProvider {
    fn code_bias_m(&self, sig: SigId) -> Option<f64>;
}

pub trait PhaseBiasProvider {
    fn phase_bias_cycles(&self, sig: SigId) -> Option<f64>;
}

#[derive(Debug, Clone)]
pub struct ZeroBiases;

impl CodeBiasProvider for ZeroBiases {
    fn code_bias_m(&self, _sig: SigId) -> Option<f64> {
        Some(0.0)
    }
}

impl PhaseBiasProvider for ZeroBiases {
    fn phase_bias_cycles(&self, _sig: SigId) -> Option<f64> {
        Some(0.0)
    }
}
