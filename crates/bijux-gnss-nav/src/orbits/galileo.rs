#![allow(missing_docs)]

use serde::{Deserialize, Serialize};

use bijux_gnss_core::api::{Constellation, SatId};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct GalileoSystemTime {
    pub week: u16,
    pub tow_s: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct GalileoSignalHealth {
    pub e5b_signal_health: u8,
    pub e1b_signal_health: u8,
    pub e5b_data_valid: bool,
    pub e1b_data_valid: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct GalileoIonosphericDisturbanceFlags {
    pub region_1: bool,
    pub region_2: bool,
    pub region_3: bool,
    pub region_4: bool,
    pub region_5: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GalileoIonosphericCorrection {
    pub ai0: f64,
    pub ai1: f64,
    pub ai2: f64,
    pub disturbance_flags: GalileoIonosphericDisturbanceFlags,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GalileoClockCorrection {
    pub t0c_s: f64,
    pub af0: f64,
    pub af1: f64,
    pub af2: f64,
    pub bgd_e1_e5a_s: f64,
    pub bgd_e1_e5b_s: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GalileoSatelliteClockCorrection {
    pub bias_s: f64,
    pub drift_s_per_s: f64,
    pub drift_rate_s_per_s2: f64,
    pub base_bias_s: f64,
    pub relativistic_s: f64,
    pub bgd_e1_e5a_s: f64,
    pub bgd_e1_e5b_s: f64,
}

impl GalileoSatelliteClockCorrection {
    pub fn from_bias_s(bias_s: f64) -> Self {
        Self {
            bias_s,
            drift_s_per_s: 0.0,
            drift_rate_s_per_s2: 0.0,
            base_bias_s: bias_s,
            relativistic_s: 0.0,
            bgd_e1_e5a_s: 0.0,
            bgd_e1_e5b_s: 0.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GalileoSatState {
    pub x_m: f64,
    pub y_m: f64,
    pub z_m: f64,
    pub clock_correction: GalileoSatelliteClockCorrection,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GalileoEarthRotationCorrection {
    pub signal_travel_time_s: f64,
    pub rotation_rad: f64,
    pub delta_x_m: f64,
    pub delta_y_m: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GalileoNavigationAge {
    pub reference_time_s: f64,
    pub toe_age_s: f64,
    pub toc_age_s: f64,
    pub max_age_s: f64,
}

impl GalileoNavigationAge {
    pub fn is_valid(&self) -> bool {
        self.toe_age_s <= self.max_age_s && self.toc_age_s <= self.max_age_s
    }

    pub fn is_stale(&self) -> bool {
        !self.is_valid()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct GalileoEphemeris {
    pub sat: SatId,
    pub iodnav: u16,
    pub toe_s: f64,
    pub sqrt_a: f64,
    pub e: f64,
    pub i0: f64,
    pub idot: f64,
    pub omega0: f64,
    pub omegadot: f64,
    pub w: f64,
    pub m0: f64,
    pub delta_n: f64,
    pub cuc: f64,
    pub cus: f64,
    pub crc: f64,
    pub crs: f64,
    pub cic: f64,
    pub cis: f64,
}

impl GalileoEphemeris {
    pub fn with_prn(prn: u8, iodnav: u16) -> Self {
        Self {
            sat: SatId { constellation: Constellation::Galileo, prn },
            iodnav,
            toe_s: 0.0,
            sqrt_a: 0.0,
            e: 0.0,
            i0: 0.0,
            idot: 0.0,
            omega0: 0.0,
            omegadot: 0.0,
            w: 0.0,
            m0: 0.0,
            delta_n: 0.0,
            cuc: 0.0,
            cus: 0.0,
            crc: 0.0,
            crs: 0.0,
            cic: 0.0,
            cis: 0.0,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GalileoBroadcastNavigationData {
    pub sat: SatId,
    pub iodnav: u16,
    pub gst: GalileoSystemTime,
    pub sisa_e1_e5b: u8,
    pub signal_health: GalileoSignalHealth,
    pub clock: GalileoClockCorrection,
    pub ephemeris: GalileoEphemeris,
    pub ionosphere: GalileoIonosphericCorrection,
}
