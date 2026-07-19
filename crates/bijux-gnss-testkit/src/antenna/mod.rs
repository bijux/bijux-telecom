//! Deterministic antenna-effect fixtures for PPP and RTK validation.

mod effects;
mod synthesis;

pub use effects::{
    gps_l1_ppp_antenna_effect_case, gps_l1_rtk_antenna_effect_case, GpsL1PppAntennaEffectCase,
    GpsL1RtkAntennaEffectCase,
};
