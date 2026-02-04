//! Atmospheric delay model scaffolding.
#![allow(missing_docs)]

use bijux_gnss_core::api::{Llh, Seconds};

pub trait IonosphereModel {
    fn delay_m(&self, _receiver: Llh, _az_deg: f64, _el_deg: f64, _t: Seconds) -> f64;
}

pub trait TroposphereModel {
    fn delay_m(&self, _receiver: Llh, _el_deg: f64, _t: Seconds) -> f64;
}

#[derive(Debug, Clone)]
pub struct KlobucharModel;

impl IonosphereModel for KlobucharModel {
    fn delay_m(&self, _receiver: Llh, _az_deg: f64, _el_deg: f64, _t: Seconds) -> f64 {
        0.0
    }
}

#[derive(Debug, Clone)]
pub struct SaastamoinenModel;

impl TroposphereModel for SaastamoinenModel {
    fn delay_m(&self, _receiver: Llh, _el_deg: f64, _t: Seconds) -> f64 {
        0.0
    }
}
