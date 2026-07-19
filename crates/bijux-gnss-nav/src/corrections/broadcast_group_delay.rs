#![allow(missing_docs)]

use bijux_gnss_core::api::{
    Constellation, FreqHz, GpsTime, SigId, SignalBand, SignalCode, GPS_L1_CA_CARRIER_HZ,
    GPS_L2_PY_CARRIER_HZ, GPS_L5_CARRIER_HZ,
};

use crate::corrections::biases::CodeBiasProvider;
use crate::orbits::beidou::{
    is_beidou_navigation_valid, select_best_beidou_navigation, BeidouBroadcastNavigationData,
};
use crate::orbits::galileo::{
    is_galileo_navigation_valid, select_best_galileo_navigation, GalileoBroadcastNavigationData,
};
use crate::orbits::gps::{is_ephemeris_valid, select_best_ephemeris, GpsEphemeris};

const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

#[derive(Debug, Clone, Default)]
pub struct BroadcastGroupDelayBiases {
    gps_ephemerides: Vec<GpsEphemeris>,
    galileo_navigations: Vec<GalileoBroadcastNavigationData>,
    beidou_navigations: Vec<BeidouBroadcastNavigationData>,
}

impl BroadcastGroupDelayBiases {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn from_gps_ephemerides(ephemerides: Vec<GpsEphemeris>) -> Self {
        Self { gps_ephemerides: ephemerides, ..Self::default() }
    }

    pub fn from_galileo_navigations(navigations: Vec<GalileoBroadcastNavigationData>) -> Self {
        Self { galileo_navigations: navigations, ..Self::default() }
    }

    pub fn from_beidou_navigations(navigations: Vec<BeidouBroadcastNavigationData>) -> Self {
        Self { beidou_navigations: navigations, ..Self::default() }
    }

    pub fn with_gps_ephemerides<I>(mut self, ephemerides: I) -> Self
    where
        I: IntoIterator<Item = GpsEphemeris>,
    {
        self.gps_ephemerides.extend(ephemerides);
        self
    }

    pub fn with_galileo_navigations<I>(mut self, navigations: I) -> Self
    where
        I: IntoIterator<Item = GalileoBroadcastNavigationData>,
    {
        self.galileo_navigations.extend(navigations);
        self
    }

    pub fn with_beidou_navigations<I>(mut self, navigations: I) -> Self
    where
        I: IntoIterator<Item = BeidouBroadcastNavigationData>,
    {
        self.beidou_navigations.extend(navigations);
        self
    }

    fn gps_code_bias_m_at(&self, sig: SigId, time: Option<GpsTime>) -> Option<f64> {
        let ephemeris = match time {
            Some(time) => {
                let ephemeris = select_best_ephemeris(&self.gps_ephemerides, sig.sat, time.tow_s)?;
                is_ephemeris_valid(ephemeris, time.tow_s).then_some(ephemeris)?
            }
            None => latest_gps_ephemeris(&self.gps_ephemerides, sig.sat)?,
        };
        gps_broadcast_group_delay_code_bias_m(sig, ephemeris)
    }

    fn galileo_code_bias_m_at(&self, sig: SigId, time: Option<GpsTime>) -> Option<f64> {
        let navigation = match time {
            Some(time) => {
                let navigation =
                    select_best_galileo_navigation(&self.galileo_navigations, sig.sat, time.tow_s)?;
                is_galileo_navigation_valid(navigation, time.tow_s).then_some(navigation)?
            }
            None => latest_galileo_navigation(&self.galileo_navigations, sig.sat)?,
        };
        galileo_broadcast_group_delay_code_bias_m(sig, navigation)
    }

    fn beidou_code_bias_m_at(&self, sig: SigId, time: Option<GpsTime>) -> Option<f64> {
        let navigation = match time {
            Some(time) => {
                let navigation =
                    select_best_beidou_navigation(&self.beidou_navigations, sig.sat, time.tow_s)?;
                is_beidou_navigation_valid(navigation, time.tow_s).then_some(navigation)?
            }
            None => latest_beidou_navigation(&self.beidou_navigations, sig.sat)?,
        };
        beidou_broadcast_group_delay_code_bias_m(sig, navigation)
    }
}

impl CodeBiasProvider for BroadcastGroupDelayBiases {
    fn code_bias_m(&self, sig: SigId) -> Option<f64> {
        self.code_bias_m_at(sig, None)
    }

    fn code_bias_m_at(&self, sig: SigId, time: Option<GpsTime>) -> Option<f64> {
        match sig.sat.constellation {
            Constellation::Gps => self.gps_code_bias_m_at(sig, time),
            Constellation::Galileo => self.galileo_code_bias_m_at(sig, time),
            Constellation::Beidou => self.beidou_code_bias_m_at(sig, time),
            Constellation::Glonass | Constellation::Unknown => None,
        }
    }
}

pub fn gps_broadcast_group_delay_code_bias_m(sig: SigId, ephemeris: &GpsEphemeris) -> Option<f64> {
    if sig.sat != ephemeris.sat {
        return None;
    }
    let carrier_hz = gps_group_delay_carrier_hz(sig.band, sig.code)?.value();
    let l1_hz = GPS_L1_CA_CARRIER_HZ.value();
    let scale = (l1_hz * l1_hz) / (carrier_hz * carrier_hz) - 1.0;
    Some(SPEED_OF_LIGHT_MPS * ephemeris.tgd * scale)
}

fn gps_group_delay_carrier_hz(band: SignalBand, code: SignalCode) -> Option<FreqHz> {
    match (band, code) {
        (SignalBand::L1, SignalCode::Ca | SignalCode::Unknown) => Some(GPS_L1_CA_CARRIER_HZ),
        (SignalBand::L2, SignalCode::Py | SignalCode::L2C) => Some(GPS_L2_PY_CARRIER_HZ),
        (SignalBand::L5, SignalCode::L5I | SignalCode::L5Q | SignalCode::Unknown) => {
            Some(GPS_L5_CARRIER_HZ)
        }
        _ => None,
    }
}

pub fn galileo_broadcast_group_delay_code_bias_m(
    sig: SigId,
    navigation: &GalileoBroadcastNavigationData,
) -> Option<f64> {
    if sig.sat != navigation.sat {
        return None;
    }
    let bias_s = match (sig.band, sig.code) {
        (SignalBand::E1, SignalCode::E1B | SignalCode::E1C) => 0.0,
        (SignalBand::E5, SignalCode::E5a) => -navigation.clock.bgd_e1_e5a_s,
        (SignalBand::E5, SignalCode::E5b) => -navigation.clock.bgd_e1_e5b_s,
        _ => return None,
    };
    Some(SPEED_OF_LIGHT_MPS * bias_s)
}

pub fn beidou_broadcast_group_delay_code_bias_m(
    sig: SigId,
    navigation: &BeidouBroadcastNavigationData,
) -> Option<f64> {
    if sig.sat != navigation.sat {
        return None;
    }
    let bias_s = match (sig.band, sig.code) {
        (SignalBand::B1, SignalCode::B1I) => 0.0,
        (SignalBand::B2, SignalCode::B2I) => navigation.clock.tgd2_s - navigation.clock.tgd1_s,
        _ => return None,
    };
    Some(SPEED_OF_LIGHT_MPS * bias_s)
}

fn latest_gps_ephemeris(
    ephemerides: &[GpsEphemeris],
    sat: bijux_gnss_core::api::SatId,
) -> Option<&GpsEphemeris> {
    ephemerides.iter().filter(|ephemeris| ephemeris.sat == sat).max_by(|left, right| {
        left.toc_s.total_cmp(&right.toc_s).then_with(|| left.toe_s.total_cmp(&right.toe_s))
    })
}

fn latest_galileo_navigation(
    navigations: &[GalileoBroadcastNavigationData],
    sat: bijux_gnss_core::api::SatId,
) -> Option<&GalileoBroadcastNavigationData> {
    navigations.iter().filter(|navigation| navigation.sat == sat).max_by(|left, right| {
        left.clock
            .t0c_s
            .total_cmp(&right.clock.t0c_s)
            .then_with(|| left.ephemeris.toe_s.total_cmp(&right.ephemeris.toe_s))
    })
}

fn latest_beidou_navigation(
    navigations: &[BeidouBroadcastNavigationData],
    sat: bijux_gnss_core::api::SatId,
) -> Option<&BeidouBroadcastNavigationData> {
    navigations.iter().filter(|navigation| navigation.sat == sat).max_by(|left, right| {
        left.clock
            .toc_s
            .total_cmp(&right.clock.toc_s)
            .then_with(|| left.ephemeris.toe_s.total_cmp(&right.ephemeris.toe_s))
    })
}

#[cfg(test)]
mod tests {
    use bijux_gnss_core::api::{
        Constellation, GpsTime, SatId, SigId, SignalBand, SignalCode, GPS_L1_CA_CARRIER_HZ,
        GPS_L2_PY_CARRIER_HZ, GPS_L5_CARRIER_HZ,
    };

    use super::{
        beidou_broadcast_group_delay_code_bias_m, galileo_broadcast_group_delay_code_bias_m,
        gps_broadcast_group_delay_code_bias_m, BroadcastGroupDelayBiases, SPEED_OF_LIGHT_MPS,
    };
    use crate::corrections::biases::CodeBiasProvider;
    use crate::orbits::beidou::{
        BeidouBroadcastNavigationData, BeidouClockCorrection, BeidouEphemeris,
        BeidouIonosphericCorrection, BeidouSignalHealth, BeidouSystemTime,
    };
    use crate::orbits::galileo::{
        GalileoBroadcastNavigationData, GalileoClockCorrection, GalileoEphemeris,
        GalileoIonosphericCorrection, GalileoIonosphericDisturbanceFlags, GalileoSignalHealth,
        GalileoSystemTime,
    };
    use crate::orbits::gps::GpsEphemeris;

    fn gps_signal(prn: u8, band: SignalBand, code: SignalCode) -> SigId {
        SigId { sat: SatId { constellation: Constellation::Gps, prn }, band, code }
    }

    fn galileo_signal(prn: u8, band: SignalBand, code: SignalCode) -> SigId {
        SigId { sat: SatId { constellation: Constellation::Galileo, prn }, band, code }
    }

    fn beidou_signal(prn: u8, band: SignalBand, code: SignalCode) -> SigId {
        SigId { sat: SatId { constellation: Constellation::Beidou, prn }, band, code }
    }

    fn sample_gps_ephemeris(toc_s: f64, tgd: f64) -> GpsEphemeris {
        GpsEphemeris {
            sat: SatId { constellation: Constellation::Gps, prn: 7 },
            iodc: 0,
            iode: 0,
            week: 0,
            sv_health: 0,
            sv_accuracy: Some(2),
            toe_s: toc_s,
            toc_s,
            sqrt_a: 5153.7954775,
            e: 0.02,
            i0: 0.94,
            idot: 0.0,
            omega0: 0.1,
            omegadot: 0.0,
            w: 0.2,
            m0: 0.3,
            delta_n: 0.0,
            cuc: 0.0,
            cus: 0.0,
            crc: 0.0,
            crs: 0.0,
            cic: 0.0,
            cis: 0.0,
            af0: 0.0,
            af1: 0.0,
            af2: 0.0,
            tgd,
        }
    }

    fn sample_galileo_navigation() -> GalileoBroadcastNavigationData {
        GalileoBroadcastNavigationData {
            sat: SatId { constellation: Constellation::Galileo, prn: 11 },
            iodnav: 3,
            gst: GalileoSystemTime { week: 2222, tow_s: 456_000 },
            sisa_e1_e5b: 0,
            signal_health: GalileoSignalHealth {
                e5b_signal_health: 0,
                e1b_signal_health: 0,
                e5b_data_valid: true,
                e1b_data_valid: true,
            },
            clock: GalileoClockCorrection {
                t0c_s: 456_000.0,
                af0: 0.0,
                af1: 0.0,
                af2: 0.0,
                bgd_e1_e5a_s: -3.5e-9,
                bgd_e1_e5b_s: 6.25e-9,
            },
            ephemeris: GalileoEphemeris {
                sat: SatId { constellation: Constellation::Galileo, prn: 11 },
                iodnav: 3,
                toe_s: 456_000.0,
                sqrt_a: 5_440.612_319,
                e: 0.001,
                i0: 0.95,
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
            },
            ionosphere: GalileoIonosphericCorrection {
                ai0: 0.0,
                ai1: 0.0,
                ai2: 0.0,
                disturbance_flags: GalileoIonosphericDisturbanceFlags {
                    region_1: false,
                    region_2: false,
                    region_3: false,
                    region_4: false,
                    region_5: false,
                },
            },
        }
    }

    fn sample_beidou_navigation() -> BeidouBroadcastNavigationData {
        BeidouBroadcastNavigationData {
            sat: SatId { constellation: Constellation::Beidou, prn: 14 },
            bdt: BeidouSystemTime { week: 888, sow_s: 345_600 },
            urai: 0,
            signal_health: BeidouSignalHealth { autonomous_satellite_good: true },
            clock: BeidouClockCorrection {
                toc_s: 345_600.0,
                aodc: 0,
                af0: 0.0,
                af1: 0.0,
                af2: 0.0,
                tgd1_s: -2.0e-9,
                tgd2_s: 5.5e-9,
            },
            ephemeris: BeidouEphemeris {
                sat: SatId { constellation: Constellation::Beidou, prn: 14 },
                aode: 0,
                toe_s: 345_600.0,
                sqrt_a: 5_282.61,
                e: 0.002,
                i0: 0.96,
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
            },
            ionosphere: BeidouIonosphericCorrection {
                alpha0: 0.0,
                alpha1: 0.0,
                alpha2: 0.0,
                alpha3: 0.0,
                beta0: 0.0,
                beta1: 0.0,
                beta2: 0.0,
                beta3: 0.0,
            },
        }
    }

    #[test]
    fn gps_bias_scales_with_signal_frequency() {
        let ephemeris = sample_gps_ephemeris(12_000.0, 8.0e-9);
        let l1 = gps_signal(7, SignalBand::L1, SignalCode::Ca);
        let l2 = gps_signal(7, SignalBand::L2, SignalCode::Py);
        let l5 = gps_signal(7, SignalBand::L5, SignalCode::L5Q);

        let l1_bias_m = gps_broadcast_group_delay_code_bias_m(l1, &ephemeris).expect("L1 bias");
        let l2_bias_m = gps_broadcast_group_delay_code_bias_m(l2, &ephemeris).expect("L2 bias");
        let l5_bias_m = gps_broadcast_group_delay_code_bias_m(l5, &ephemeris).expect("L5 bias");

        let expected_l2 = SPEED_OF_LIGHT_MPS
            * ephemeris.tgd
            * ((GPS_L1_CA_CARRIER_HZ.value() / GPS_L2_PY_CARRIER_HZ.value()).powi(2) - 1.0);
        let expected_l5 = SPEED_OF_LIGHT_MPS
            * ephemeris.tgd
            * ((GPS_L1_CA_CARRIER_HZ.value() / GPS_L5_CARRIER_HZ.value()).powi(2) - 1.0);

        assert!(l1_bias_m.abs() < 1.0e-12);
        assert!((l2_bias_m - expected_l2).abs() < 1.0e-9);
        assert!((l5_bias_m - expected_l5).abs() < 1.0e-9);
    }

    #[test]
    fn gps_bias_uses_band_frequency_when_code_is_unknown() {
        let ephemeris = sample_gps_ephemeris(12_000.0, 8.0e-9);
        let explicit_l5 = gps_signal(7, SignalBand::L5, SignalCode::L5Q);
        let normalized_l5 = gps_signal(7, SignalBand::L5, SignalCode::Unknown);

        assert_eq!(
            gps_broadcast_group_delay_code_bias_m(normalized_l5, &ephemeris),
            gps_broadcast_group_delay_code_bias_m(explicit_l5, &ephemeris)
        );
    }

    #[test]
    fn gps_bias_rejects_ambiguous_l2_unknown_code() {
        let ephemeris = sample_gps_ephemeris(12_000.0, 8.0e-9);
        let ambiguous_l2 = gps_signal(7, SignalBand::L2, SignalCode::Unknown);

        assert_eq!(gps_broadcast_group_delay_code_bias_m(ambiguous_l2, &ephemeris), None);
    }

    #[test]
    fn galileo_bias_uses_bgd_relative_to_e1() {
        let navigation = sample_galileo_navigation();
        let e1 = galileo_signal(11, SignalBand::E1, SignalCode::E1B);
        let e5a = galileo_signal(11, SignalBand::E5, SignalCode::E5a);
        let e5b = galileo_signal(11, SignalBand::E5, SignalCode::E5b);

        assert_eq!(galileo_broadcast_group_delay_code_bias_m(e1, &navigation), Some(0.0));
        assert_eq!(
            galileo_broadcast_group_delay_code_bias_m(e5a, &navigation),
            Some(-SPEED_OF_LIGHT_MPS * navigation.clock.bgd_e1_e5a_s)
        );
        assert_eq!(
            galileo_broadcast_group_delay_code_bias_m(e5b, &navigation),
            Some(-SPEED_OF_LIGHT_MPS * navigation.clock.bgd_e1_e5b_s)
        );
    }

    #[test]
    fn beidou_bias_uses_b2_relative_offset() {
        let navigation = sample_beidou_navigation();
        let b1 = beidou_signal(14, SignalBand::B1, SignalCode::B1I);
        let b2 = beidou_signal(14, SignalBand::B2, SignalCode::B2I);

        assert_eq!(beidou_broadcast_group_delay_code_bias_m(b1, &navigation), Some(0.0));
        assert_eq!(
            beidou_broadcast_group_delay_code_bias_m(b2, &navigation),
            Some(SPEED_OF_LIGHT_MPS * (navigation.clock.tgd2_s - navigation.clock.tgd1_s))
        );
    }

    #[test]
    fn provider_selects_time_matching_navigation_record() {
        let early = sample_gps_ephemeris(10_000.0, -4.0e-9);
        let late = sample_gps_ephemeris(12_000.0, 7.0e-9);
        let provider =
            BroadcastGroupDelayBiases::from_gps_ephemerides(vec![early.clone(), late.clone()]);
        let signal = gps_signal(7, SignalBand::L2, SignalCode::Py);

        let early_bias_m = provider
            .code_bias_m_at(signal, Some(GpsTime { week: 0, tow_s: 10_005.0 }))
            .expect("early bias");
        let late_bias_m = provider
            .code_bias_m_at(signal, Some(GpsTime { week: 0, tow_s: 12_005.0 }))
            .expect("late bias");

        assert_ne!(early_bias_m, late_bias_m);
        assert_eq!(
            early_bias_m,
            gps_broadcast_group_delay_code_bias_m(signal, &early).expect("expected early bias")
        );
        assert_eq!(
            late_bias_m,
            gps_broadcast_group_delay_code_bias_m(signal, &late).expect("expected late bias")
        );
    }
}
