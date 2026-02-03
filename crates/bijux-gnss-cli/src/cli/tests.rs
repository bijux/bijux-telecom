#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rinex_obs_has_header() {
        let epoch = ObsEpoch {
            t_rx_s: 0.0,
            gps_week: Some(0),
            tow_s: Some(0.0),
            epoch_idx: 0,
            discontinuity: false,
            role: bijux_gnss_core::ReceiverRole::Rover,
            sats: vec![bijux_gnss_core::ObsSatellite {
                signal_id: bijux_gnss_core::SigId {
                    sat: bijux_gnss_core::SatId {
                        constellation: bijux_gnss_core::Constellation::Gps,
                        prn: 1,
                    },
                    band: bijux_gnss_core::SignalBand::L1,
                    code: bijux_gnss_core::SignalCode::Ca,
                },
                pseudorange_m: 20_000_000.0,
                pseudorange_var_m2: 1.0,
                carrier_phase_cycles: 1000.0,
                carrier_phase_var_cycles2: 0.01,
                doppler_hz: -500.0,
                doppler_var_hz2: 4.0,
                cn0_dbhz: 40.0,
                lock_flags: bijux_gnss_core::LockFlags {
                    code_lock: true,
                    carrier_lock: true,
                    bit_lock: false,
                    cycle_slip: false,
                },
                multipath_suspect: false,
                elevation_deg: None,
                azimuth_deg: None,
                weight: None,
                error_model: None,
                metadata: bijux_gnss_core::ObsMetadata {
                    tracking_mode: "test".to_string(),
                    integration_ms: 1,
                    lock_quality: 1.0,
                    smoothing_window: 0,
                    smoothing_age: 0,
                    smoothing_resets: 0,
                    signal: bijux_gnss_core::SignalSpec {
                        constellation: bijux_gnss_core::Constellation::Gps,
                        band: bijux_gnss_core::SignalBand::L1,
                        code: bijux_gnss_core::SignalCode::Ca,
                        code_rate_hz: 1_023_000.0,
                        carrier_hz: bijux_gnss_core::GPS_L1_CA_CARRIER_HZ,
                    },
                },
            }],
        };
        let path = std::path::Path::new("/tmp/rinex_obs_test.rnx");
        write_rinex_obs(path, &[epoch], true).expect("write obs");
        let data = std::fs::read_to_string(path).expect("read obs");
        assert!(data.contains("RINEX VERSION / TYPE"));
        assert!(data.contains("END OF HEADER"));
        for line in data.lines() {
            assert!(line.len() <= 80, "line too long: {}", line.len());
        }
    }

    #[test]
    fn rinex_nav_has_header() {
        let eph = GpsEphemeris {
            sat: SatId {
                constellation: Constellation::Gps,
                prn: 1,
            },
            iodc: 1,
            iode: 1,
            week: 0,
            toe_s: 0.0,
            toc_s: 0.0,
            sqrt_a: 5153.7954775,
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
            af0: 0.0,
            af1: 0.0,
            af2: 0.0,
            tgd: 0.0,
        };
        let path = std::path::Path::new("/tmp/rinex_nav_test.rnx");
        write_rinex_nav(path, &[eph], true).expect("write nav");
        let data = std::fs::read_to_string(path).expect("read nav");
        assert!(data.contains("NAVIGATION DATA"));
        assert!(data.contains("END OF HEADER"));
        for line in data.lines() {
            assert!(line.len() <= 80, "line too long: {}", line.len());
        }
    }
}
