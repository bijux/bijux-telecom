#[derive(Debug, Clone)]
pub(super) struct ClockModel {
    bias_s: f64,
    drift_s: f64,
}

impl ClockModel {
    pub(super) fn new() -> Self {
        Self { bias_s: 0.0, drift_s: 0.0 }
    }

    pub(super) fn update(&mut self, measurement_bias_s: f64, dt_s: f64) -> (f64, f64) {
        let alpha = 0.1;
        let beta = 0.01;
        let mut bias = self.bias_s + self.drift_s * dt_s;
        let residual = measurement_bias_s - bias;
        bias += alpha * residual;
        let drift = self.drift_s + (beta * residual / dt_s.max(1e-6));
        self.bias_s = bias;
        self.drift_s = drift;
        (self.bias_s, self.drift_s)
    }
}
