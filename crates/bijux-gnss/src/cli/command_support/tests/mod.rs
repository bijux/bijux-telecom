#[cfg(test)]
mod tests {
    use super::{
        apply_raw_iq_metadata, enforce_locked_capture_value, load_frame_window,
        read_broadcast_navigation_data, read_ephemeris, read_tracking_dump, DopplerSearchSettings,
        RawIqSignalQualityReport, TrackingReport, TrackingRow,
    };
    use crate::RawIqMetadata;
    use crate::{CommonArgs, ReceiverConfig, ReceiverPipelineConfig, ReportFormat};
    use bijux_gnss_infra::api::core::{
        ArtifactHeaderV1, ArtifactReadPolicy, Chips, Constellation, Cycles, Epoch, Hertz,
        LockFlags, Meters, NavLifecycleState, NavSolutionEpoch, NavUncertaintyClass, ObsEpoch,
        ObsMetadata, ObsSatellite, ObservationEpochDecision, ObservationStatus, ReceiverRole,
        ReceiverSampleTrace, SatId, Seconds, SigId, SignalBand, SignalCode, SignalDelayAlignment,
        SolutionStatus, SolutionValidity, TrackEpoch, TrackEpochV1, TrackingUncertainty,
        GPS_L1_CA_CARRIER_HZ, NAV_OUTPUT_STABILITY_SIGNATURE_VERSION, NAV_SOLUTION_MODEL_VERSION,
    };
    use bijux_gnss_infra::api::nav::{
        write_rinex_broadcast_navigation, write_rinex_nav, BiasSinexProvider,
        GpsBroadcastNavigationData, GpsEphemeris, KlobucharCoefficients,
    };
    use bijux_gnss_signal::api::{signal_registry, signal_spec_gps_l1_ca, signal_spec_gps_l2_py};
    use std::fs;
    use std::path::{Path, PathBuf};
    use std::time::{SystemTime, UNIX_EPOCH};

    const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

    include!("input_loading.rs");

    fn unique_iq_fixture_path(name: &str) -> PathBuf {
        let nanos = SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos();
        std::env::temp_dir().join(format!("bijux_{}_{}_{}.iq", name, std::process::id(), nanos))
    }

    fn unique_artifact_output_dir(name: &str) -> PathBuf {
        let nanos = SystemTime::now().duration_since(UNIX_EPOCH).expect("unix epoch").as_nanos();
        std::env::temp_dir().join(format!("bijux_{}_{}_{}", name, std::process::id(), nanos))
    }

    fn bias_sinex_fixture(name: &str) -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../bijux-gnss-nav/tests/data").join(name)
    }

    fn sample_common_args(out_dir: PathBuf) -> CommonArgs {
        CommonArgs {
            config: None,
            dataset: None,
            unregistered_dataset: true,
            out: Some(out_dir),
            report: ReportFormat::Json,
            seed: None,
            deterministic: true,
            dump: None,
            sidecar: None,
            resume: None,
        }
    }

    fn dual_frequency_epoch(
        epoch_idx: u64,
        p1_m: f64,
        p2_m: f64,
        phi1_m: f64,
        phi2_m: f64,
    ) -> ObsEpoch {
        dual_frequency_epoch_for_pair(
            epoch_idx,
            SatId { constellation: Constellation::Gps, prn: 12 },
            (SignalBand::L1, SignalCode::Ca, p1_m, phi1_m),
            (SignalBand::L2, SignalCode::Py, p2_m, phi2_m),
        )
    }

    fn dual_frequency_epoch_for_pair(
        epoch_idx: u64,
        sat: SatId,
        first: (SignalBand, SignalCode, f64, f64),
        second: (SignalBand, SignalCode, f64, f64),
    ) -> ObsEpoch {
        ObsEpoch {
            t_rx_s: Seconds(epoch_idx as f64),
            source_time: ReceiverSampleTrace::from_sample_index(epoch_idx, 1.0),
            gps_week: None,
            tow_s: None,
            epoch_idx,
            discontinuity: false,
            valid: true,
            processing_ms: None,
            role: ReceiverRole::Rover,
            sats: vec![
                dual_frequency_satellite(sat, first.0, first.1, first.2, first.3),
                dual_frequency_satellite(sat, second.0, second.1, second.2, second.3),
            ],
            decision: ObservationEpochDecision::Accepted,
            decision_reason: Some("accepted_observables_present".to_string()),
            manifest: None,
        }
    }

    fn dual_frequency_satellite(
        sat: SatId,
        band: SignalBand,
        code: SignalCode,
        pseudorange_m: f64,
        phase_m: f64,
    ) -> ObsSatellite {
        let signal = signal_registry(sat.constellation, band, code)
            .expect("dual-frequency signal must exist")
            .spec;
        let wavelength_m = SPEED_OF_LIGHT_MPS / signal.carrier_hz.value();

        ObsSatellite {
            signal_id: SigId { sat, band, code },
            pseudorange_m: Meters(pseudorange_m),
            pseudorange_var_m2: 1.0,
            carrier_phase_cycles: Cycles(phase_m / wavelength_m),
            carrier_phase_var_cycles2: 0.01,
            doppler_hz: Hertz(0.0),
            doppler_var_hz2: 1.0,
            cn0_dbhz: 45.0,
            lock_flags: LockFlags {
                code_lock: true,
                carrier_lock: true,
                bit_lock: false,
                cycle_slip: false,
            },
            multipath_suspect: false,
            observation_status: ObservationStatus::Accepted,
            observation_reject_reasons: Vec::new(),
            elevation_deg: None,
            azimuth_deg: None,
            weight: None,
            timing: None,
            error_model: None,
            metadata: ObsMetadata {
                tracking_mode: "test".to_string(),
                integration_ms: 1,
                lock_quality: 1.0,
                smoothing_window: 0,
                smoothing_age: 0,
                smoothing_resets: 0,
                signal,
                ..ObsMetadata::default()
            },
        }
    }

    fn sample_ephemeris() -> GpsEphemeris {
        GpsEphemeris {
            sat: SatId { constellation: Constellation::Gps, prn: 8 },
            iodc: 97,
            iode: 11,
            week: 2209,
            sv_health: 0,
            sv_accuracy: Some(2),
            toe_s: 345_600.0,
            toc_s: 504_018.0,
            sqrt_a: 5_153.795_477_5,
            e: 1.234_567_890_123e-2,
            i0: 9.4e-1,
            idot: 7.8e-10,
            omega0: 1.5,
            omegadot: -8.9e-9,
            w: 2.1e-1,
            m0: 6.0e-1,
            delta_n: 4.5e-9,
            cuc: 1.2e-6,
            cus: 2.3e-6,
            crc: 321.0,
            crs: 25.0,
            cic: 4.5e-8,
            cis: 5.6e-8,
            af0: -1.234_567_890_123e-4,
            af1: 2.345_678_901_234e-12,
            af2: 0.0,
            tgd: -1.9e-8,
        }
    }

    fn sample_klobuchar_coefficients() -> KlobucharCoefficients {
        KlobucharCoefficients::new(
            [0.1212e-7, 0.1490e-7, -0.5960e-7, 0.1192e-6],
            [0.1167e6, -0.2294e6, -0.1311e6, 0.1049e7],
        )
    }

    fn sample_front_end_metrics() -> bijux_gnss_infra::api::signal::IqFrontEndMetrics {
        bijux_gnss_infra::api::signal::IqFrontEndMetrics {
            sample_count: 4_092,
            i_mean: 0.0,
            q_mean: 0.0,
            i_power: 1.0,
            q_power: 1.0,
            iq_power_ratio: 1.0,
            power_imbalance_warning: false,
            quadrature_error_deg: Some(0.0),
            quadrature_error_warning: false,
            clipping_pct: Some(0.0),
            clipping_warning: false,
            centered_rms: 1.0,
            zero_signal_detected: false,
            zero_signal_reason: None,
            precision_claims_allowed: true,
            precision_claims_refused_reason: None,
            rms: 1.0,
            dc_imbalance: 0.0,
        }
    }

    include!("capture_loading.rs");

    include!("observation_output.rs");

    include!("dual_frequency_outputs.rs");
}
