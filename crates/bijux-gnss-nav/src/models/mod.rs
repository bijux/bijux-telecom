//! Shared physical and environmental models.

pub mod antenna;
pub mod atmosphere;
pub mod celestial;
pub mod nequick;
pub mod ocean_tide_loading;
pub mod solid_earth_tide;

#[allow(dead_code)]
const MODELS_NAMESPACE: &str = "models";
