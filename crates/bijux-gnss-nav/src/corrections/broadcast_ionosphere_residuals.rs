#![allow(missing_docs)]

use std::collections::BTreeMap;

use bijux_gnss_core::api::{
    first_order_ionosphere_code_delay_m, signal_spec_gps_l1_ca, Llh, Meters, ObsEpoch,
    ObsSatellite, SatId, SigId, SignalBand,
};

use crate::corrections::measured_ionosphere::{
    measured_ionosphere_from_obs_epochs, MeasuredIonosphereObservation,
};
use crate::estimation::position::navigation::{
    satellite_state_from_observation, select_valid_navigation,
};
use crate::estimation::position::solver::{
    ecef_to_geodetic, elevation_azimuth_deg, position_broadcast_navigation_from_gps_ephemerides,
    PositionBroadcastNavigation,
};
use crate::orbits::gps::GpsBroadcastNavigationData;

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct BroadcastIonosphereResidualObservation {
    pub epoch_idx: u64,
    pub t_rx_s: f64,
    pub sat: SatId,
    pub signal_1: Option<SigId>,
    pub signal_2: Option<SigId>,
    pub band_1: SignalBand,
    pub band_2: SignalBand,
    pub broadcast_model: String,
    pub azimuth_deg: Option<f64>,
    pub elevation_deg: Option<f64>,
    pub measured_code_delay_band_1_m: Option<f64>,
    pub measured_code_delay_band_2_m: Option<f64>,
    pub measured_phase_delay_band_1_m: Option<f64>,
    pub measured_phase_delay_band_2_m: Option<f64>,
    pub broadcast_delay_band_1_m: Option<f64>,
    pub broadcast_delay_band_2_m: Option<f64>,
    pub code_residual_band_1_m: Option<f64>,
    pub code_residual_band_2_m: Option<f64>,
    pub phase_residual_band_1_m: Option<f64>,
    pub phase_residual_band_2_m: Option<f64>,
    pub measured_code_status: String,
    pub measured_code_reason: String,
    pub measured_phase_status: String,
    pub measured_phase_reason: String,
    pub broadcast_status: String,
    pub broadcast_reason: String,
}

#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct BroadcastIonosphereResidualStats {
    pub sample_count: usize,
    pub mean_residual_m: f64,
    pub mean_abs_residual_m: f64,
    pub rms_residual_m: f64,
    pub max_abs_residual_m: f64,
}

#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct BroadcastIonosphereResidualSummary {
    pub observation_count: usize,
    pub broadcast_valid_count: usize,
    pub code_band_1: Option<BroadcastIonosphereResidualStats>,
    pub code_band_2: Option<BroadcastIonosphereResidualStats>,
    pub phase_band_1: Option<BroadcastIonosphereResidualStats>,
    pub phase_band_2: Option<BroadcastIonosphereResidualStats>,
}

pub fn summarize_broadcast_ionosphere_residuals(
    observations: &[BroadcastIonosphereResidualObservation],
) -> BroadcastIonosphereResidualSummary {
    BroadcastIonosphereResidualSummary {
        observation_count: observations.len(),
        broadcast_valid_count: observations
            .iter()
            .filter(|observation| observation.broadcast_status == "ok")
            .count(),
        code_band_1: summarize_residual_component(
            observations.iter().filter_map(|observation| observation.code_residual_band_1_m),
        ),
        code_band_2: summarize_residual_component(
            observations.iter().filter_map(|observation| observation.code_residual_band_2_m),
        ),
        phase_band_1: summarize_residual_component(
            observations.iter().filter_map(|observation| observation.phase_residual_band_1_m),
        ),
        phase_band_2: summarize_residual_component(
            observations.iter().filter_map(|observation| observation.phase_residual_band_2_m),
        ),
    }
}

pub fn gps_broadcast_ionosphere_residuals_from_obs_epochs(
    epochs: &[ObsEpoch],
    receiver_ecef_m: (f64, f64, f64),
    navigation: &GpsBroadcastNavigationData,
    band_1: SignalBand,
    band_2: SignalBand,
) -> Vec<BroadcastIonosphereResidualObservation> {
    let measured = measured_ionosphere_from_obs_epochs(epochs, band_1, band_2);
    let epochs_by_idx = epochs.iter().map(|epoch| (epoch.epoch_idx, epoch)).collect::<BTreeMap<_, _>>();
    let receiver = receiver_llh_from_ecef(receiver_ecef_m);
    let navigation_entries =
        position_broadcast_navigation_from_gps_ephemerides(&navigation.ephemerides);

    measured
        .into_iter()
        .map(|observation| {
            let Some(epoch) = epochs_by_idx.get(&observation.epoch_idx).copied() else {
                return with_broadcast_failure(
                    observation,
                    "gps_klobuchar",
                    "invalid",
                    "missing_epoch",
                );
            };
            compare_gps_broadcast_against_measured(
                observation,
                epoch,
                receiver_ecef_m,
                receiver,
                navigation,
                &navigation_entries,
            )
        })
        .collect()
}

fn summarize_residual_component(
    values: impl Iterator<Item = f64>,
) -> Option<BroadcastIonosphereResidualStats> {
    let values = values.filter(|value| value.is_finite()).collect::<Vec<_>>();
    if values.is_empty() {
        return None;
    }

    let sample_count = values.len();
    let sum = values.iter().sum::<f64>();
    let mean_residual_m = sum / sample_count as f64;
    let mean_abs_residual_m = values.iter().map(|value| value.abs()).sum::<f64>() / sample_count as f64;
    let rms_residual_m =
        (values.iter().map(|value| value * value).sum::<f64>() / sample_count as f64).sqrt();
    let max_abs_residual_m =
        values.iter().map(|value| value.abs()).fold(0.0_f64, f64::max);

    Some(BroadcastIonosphereResidualStats {
        sample_count,
        mean_residual_m,
        mean_abs_residual_m,
        rms_residual_m,
        max_abs_residual_m,
    })
}

fn compare_gps_broadcast_against_measured(
    observation: MeasuredIonosphereObservation,
    epoch: &ObsEpoch,
    receiver_ecef_m: (f64, f64, f64),
    receiver: Option<Llh>,
    navigation: &GpsBroadcastNavigationData,
    navigation_entries: &[PositionBroadcastNavigation],
) -> BroadcastIonosphereResidualObservation {
    let mut residual = residual_observation_from_measured(observation, "gps_klobuchar");
    let Some(receiver) = receiver else {
        residual.broadcast_status = "invalid".to_string();
        residual.broadcast_reason = "invalid_receiver_position".to_string();
        return residual;
    };

    let (first, second) =
        match dual_frequency_observation_pair(epoch, residual.sat, residual.band_1, residual.band_2)
        {
            Some(pair) => pair,
            None => {
                residual.broadcast_status = "invalid".to_string();
                residual.broadcast_reason = "missing_frequency".to_string();
                return residual;
            }
        };
    residual.signal_1 = Some(first.signal_id);
    residual.signal_2 = Some(second.signal_id);

    let Some(gps_time) = epoch.gps_time() else {
        residual.broadcast_status = "invalid".to_string();
        residual.broadcast_reason = "invalid_receive_time".to_string();
        return residual;
    };
    let Some(selected_navigation) =
        select_valid_navigation(navigation_entries, residual.sat, gps_time.tow_s)
    else {
        residual.broadcast_status = "invalid".to_string();
        residual.broadcast_reason = "invalid_ephemeris".to_string();
        return residual;
    };
    let Some(state) = satellite_state_from_observation(
        selected_navigation,
        gps_time.tow_s,
        first.pseudorange_m.0,
        first.timing,
    ) else {
        residual.broadcast_status = "invalid".to_string();
        residual.broadcast_reason = "invalid_geometry".to_string();
        return residual;
    };

    let (azimuth_deg, elevation_deg) = elevation_azimuth_deg(
        receiver_ecef_m.0,
        receiver_ecef_m.1,
        receiver_ecef_m.2,
        state.x_m,
        state.y_m,
        state.z_m,
    );
    if !azimuth_deg.is_finite() || !elevation_deg.is_finite() || elevation_deg <= 0.0 {
        residual.broadcast_status = "invalid".to_string();
        residual.broadcast_reason = "invalid_geometry".to_string();
        return residual;
    }
    residual.azimuth_deg = Some(azimuth_deg);
    residual.elevation_deg = Some(elevation_deg);

    let Some(l1_delay_m) =
        navigation.klobuchar_delay_l1_m(receiver, azimuth_deg, elevation_deg, gps_time.tow_s)
    else {
        residual.broadcast_status = "invalid".to_string();
        residual.broadcast_reason = "missing_klobuchar_coefficients".to_string();
        return residual;
    };
    let l1_signal = signal_spec_gps_l1_ca();
    let Some(broadcast_delay_band_1_m) =
        first_order_ionosphere_code_delay_m(Meters(l1_delay_m), l1_signal, first.metadata.signal)
            .map(|delay| delay.0)
    else {
        residual.broadcast_status = "invalid".to_string();
        residual.broadcast_reason = "broadcast_scaling_invalid".to_string();
        return residual;
    };
    let Some(broadcast_delay_band_2_m) =
        first_order_ionosphere_code_delay_m(Meters(l1_delay_m), l1_signal, second.metadata.signal)
            .map(|delay| delay.0)
    else {
        residual.broadcast_status = "invalid".to_string();
        residual.broadcast_reason = "broadcast_scaling_invalid".to_string();
        return residual;
    };

    residual.broadcast_delay_band_1_m = Some(broadcast_delay_band_1_m);
    residual.broadcast_delay_band_2_m = Some(broadcast_delay_band_2_m);
    residual.code_residual_band_1_m = residual
        .measured_code_delay_band_1_m
        .map(|delay_m| delay_m - broadcast_delay_band_1_m);
    residual.code_residual_band_2_m = residual
        .measured_code_delay_band_2_m
        .map(|delay_m| delay_m - broadcast_delay_band_2_m);
    residual.phase_residual_band_1_m = residual
        .measured_phase_delay_band_1_m
        .map(|delay_m| delay_m - broadcast_delay_band_1_m);
    residual.phase_residual_band_2_m = residual
        .measured_phase_delay_band_2_m
        .map(|delay_m| delay_m - broadcast_delay_band_2_m);
    residual.broadcast_status = "ok".to_string();
    residual.broadcast_reason = "ok".to_string();
    residual
}

fn residual_observation_from_measured(
    observation: MeasuredIonosphereObservation,
    broadcast_model: &str,
) -> BroadcastIonosphereResidualObservation {
    BroadcastIonosphereResidualObservation {
        epoch_idx: observation.epoch_idx,
        t_rx_s: observation.t_rx_s,
        sat: observation.sat,
        signal_1: observation.signal_1,
        signal_2: observation.signal_2,
        band_1: observation.band_1,
        band_2: observation.band_2,
        broadcast_model: broadcast_model.to_string(),
        azimuth_deg: None,
        elevation_deg: None,
        measured_code_delay_band_1_m: observation.code_delay_band_1_m,
        measured_code_delay_band_2_m: observation.code_delay_band_2_m,
        measured_phase_delay_band_1_m: observation.phase_delay_band_1_m,
        measured_phase_delay_band_2_m: observation.phase_delay_band_2_m,
        broadcast_delay_band_1_m: None,
        broadcast_delay_band_2_m: None,
        code_residual_band_1_m: None,
        code_residual_band_2_m: None,
        phase_residual_band_1_m: None,
        phase_residual_band_2_m: None,
        measured_code_status: observation.code_status,
        measured_code_reason: observation.code_reason,
        measured_phase_status: observation.phase_status,
        measured_phase_reason: observation.phase_reason,
        broadcast_status: "invalid".to_string(),
        broadcast_reason: "unresolved".to_string(),
    }
}

fn with_broadcast_failure(
    observation: MeasuredIonosphereObservation,
    broadcast_model: &str,
    broadcast_status: &str,
    broadcast_reason: &str,
) -> BroadcastIonosphereResidualObservation {
    let mut residual = residual_observation_from_measured(observation, broadcast_model);
    residual.broadcast_status = broadcast_status.to_string();
    residual.broadcast_reason = broadcast_reason.to_string();
    residual
}

fn receiver_llh_from_ecef(receiver_ecef_m: (f64, f64, f64)) -> Option<Llh> {
    if !receiver_ecef_m.0.is_finite()
        || !receiver_ecef_m.1.is_finite()
        || !receiver_ecef_m.2.is_finite()
    {
        return None;
    }
    let (lat_deg, lon_deg, alt_m) =
        ecef_to_geodetic(receiver_ecef_m.0, receiver_ecef_m.1, receiver_ecef_m.2);
    if !lat_deg.is_finite() || !lon_deg.is_finite() || !alt_m.is_finite() {
        return None;
    }
    Some(Llh { lat_deg, lon_deg, alt_m })
}

fn dual_frequency_observation_pair<'a>(
    epoch: &'a ObsEpoch,
    sat: SatId,
    band_1: SignalBand,
    band_2: SignalBand,
) -> Option<(&'a ObsSatellite, &'a ObsSatellite)> {
    let first = epoch.sats.iter().find(|observation| {
        observation.signal_id.sat == sat && observation.signal_id.band == band_1
    })?;
    let second = epoch.sats.iter().find(|observation| {
        observation.signal_id.sat == sat && observation.signal_id.band == band_2
    })?;
    Some((first, second))
}

#[cfg(test)]
mod tests {
    use super::{
        summarize_broadcast_ionosphere_residuals, BroadcastIonosphereResidualObservation,
        BroadcastIonosphereResidualStats,
    };
    use bijux_gnss_core::api::{Constellation, SatId, SignalBand};

    fn sample_observation(
        code_band_1: Option<f64>,
        code_band_2: Option<f64>,
        phase_band_1: Option<f64>,
        phase_band_2: Option<f64>,
        broadcast_status: &str,
    ) -> BroadcastIonosphereResidualObservation {
        BroadcastIonosphereResidualObservation {
            epoch_idx: 0,
            t_rx_s: 0.0,
            sat: SatId { constellation: Constellation::Gps, prn: 11 },
            signal_1: None,
            signal_2: None,
            band_1: SignalBand::L1,
            band_2: SignalBand::L2,
            broadcast_model: "klobuchar".to_string(),
            azimuth_deg: Some(120.0),
            elevation_deg: Some(35.0),
            measured_code_delay_band_1_m: None,
            measured_code_delay_band_2_m: None,
            measured_phase_delay_band_1_m: None,
            measured_phase_delay_band_2_m: None,
            broadcast_delay_band_1_m: None,
            broadcast_delay_band_2_m: None,
            code_residual_band_1_m: code_band_1,
            code_residual_band_2_m: code_band_2,
            phase_residual_band_1_m: phase_band_1,
            phase_residual_band_2_m: phase_band_2,
            measured_code_status: "ok".to_string(),
            measured_code_reason: "ok".to_string(),
            measured_phase_status: "ok".to_string(),
            measured_phase_reason: "ok".to_string(),
            broadcast_status: broadcast_status.to_string(),
            broadcast_reason: broadcast_status.to_string(),
        }
    }

    fn assert_stats(
        stats: BroadcastIonosphereResidualStats,
        sample_count: usize,
        mean_residual_m: f64,
        mean_abs_residual_m: f64,
        rms_residual_m: f64,
        max_abs_residual_m: f64,
    ) {
        assert_eq!(stats.sample_count, sample_count);
        assert!((stats.mean_residual_m - mean_residual_m).abs() < 1.0e-12);
        assert!((stats.mean_abs_residual_m - mean_abs_residual_m).abs() < 1.0e-12);
        assert!((stats.rms_residual_m - rms_residual_m).abs() < 1.0e-12);
        assert!((stats.max_abs_residual_m - max_abs_residual_m).abs() < 1.0e-12);
    }

    #[test]
    fn broadcast_ionosphere_residual_summary_aggregates_each_component() {
        let summary = summarize_broadcast_ionosphere_residuals(&[
            sample_observation(Some(1.0), Some(-2.0), Some(0.5), None, "ok"),
            sample_observation(Some(-3.0), Some(4.0), None, Some(-1.5), "ok"),
            sample_observation(None, None, Some(1.5), Some(2.5), "invalid"),
        ]);

        assert_eq!(summary.observation_count, 3);
        assert_eq!(summary.broadcast_valid_count, 2);
        assert_stats(
            summary.code_band_1.expect("code L1 stats"),
            2,
            -1.0,
            2.0,
            (5.0_f64).sqrt(),
            3.0,
        );
        assert_stats(
            summary.code_band_2.expect("code L2 stats"),
            2,
            1.0,
            3.0,
            (10.0_f64).sqrt(),
            4.0,
        );
        assert_stats(
            summary.phase_band_1.expect("phase L1 stats"),
            2,
            1.0,
            1.0,
            (1.25_f64).sqrt(),
            1.5,
        );
        assert_stats(
            summary.phase_band_2.expect("phase L2 stats"),
            2,
            0.5,
            2.0,
            (4.25_f64).sqrt(),
            2.5,
        );
    }

    #[test]
    fn broadcast_ionosphere_residual_summary_ignores_missing_components() {
        let summary = summarize_broadcast_ionosphere_residuals(&[sample_observation(
            None,
            None,
            None,
            None,
            "invalid",
        )]);

        assert_eq!(summary.observation_count, 1);
        assert_eq!(summary.broadcast_valid_count, 0);
        assert!(summary.code_band_1.is_none());
        assert!(summary.code_band_2.is_none());
        assert!(summary.phase_band_1.is_none());
        assert!(summary.phase_band_2.is_none());
    }
}
