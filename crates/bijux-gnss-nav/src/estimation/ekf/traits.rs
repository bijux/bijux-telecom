use super::MeasurementKind;
use crate::Matrix;

pub trait StateModel {
    fn state_dim(&self) -> usize;
    fn propagate(&self, x: &mut [f64], p: &mut Matrix, dt_s: f64);
}

pub trait MeasurementModel {
    fn name(&self) -> &'static str;
    fn kind(&self) -> MeasurementKind;
    fn measurement_dim(&self) -> usize;
    fn observation(&self) -> &[f64];
    fn h(&self, x: &[f64], out: &mut [f64]);
    fn jacobian(&self, x: &[f64], h: &mut Matrix);
    fn covariance(&self, x: &[f64], r: &mut Matrix);
}
