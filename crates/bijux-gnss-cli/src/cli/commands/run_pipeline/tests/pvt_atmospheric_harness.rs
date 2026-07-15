fn add_klobuchar_delay_to_pvt_case(
        pvt_case: &SyntheticPvtCase,
        ephs: &[GpsEphemeris],
        klobuchar: KlobucharCoefficients,
    ) -> SyntheticPvtCase {
        let (lat_deg, lon_deg, alt_m) = ecef_to_geodetic(
            pvt_case.truth_ecef_m.0,
            pvt_case.truth_ecef_m.1,
            pvt_case.truth_ecef_m.2,
        );
        let receiver = bijux_gnss_infra::api::core::Llh { lat_deg, lon_deg, alt_m };
        let model = KlobucharModel::new(klobuchar);
        let mut biased_epoch = pvt_case.obs_epoch.clone();
        for sat in &mut biased_epoch.sats {
            let ephemeris = ephs
                .iter()
                .find(|ephemeris| ephemeris.sat == sat.signal_id.sat)
                .expect("matching ephemeris");
            let timing = sat.timing.expect("timing");
            let state = sat_state_gps_l1ca(
                ephemeris,
                timing.transmit_gps_time.tow_s,
                timing.signal_travel_time_s.0,
            );
            let (azimuth_deg, elevation_deg) = elevation_azimuth_deg(
                pvt_case.truth_ecef_m.0,
                pvt_case.truth_ecef_m.1,
                pvt_case.truth_ecef_m.2,
                state.x_m,
                state.y_m,
                state.z_m,
            );
            let delay_m = model.delay_m(
                receiver,
                azimuth_deg,
                elevation_deg,
                Seconds(pvt_case.obs_epoch.t_rx_s.0),
            );
            let delay_s = delay_m / SPEED_OF_LIGHT_MPS;
            sat.pseudorange_m.0 += delay_m;
            sat.timing = Some(ObsSignalTiming {
                signal_travel_time_s: Seconds(timing.signal_travel_time_s.0 + delay_s),
                transmit_gps_time: timing.transmit_gps_time.offset_seconds(-delay_s),
            });
        }
        SyntheticPvtCase {
            obs_epoch: biased_epoch,
            truth_ecef_m: pvt_case.truth_ecef_m,
            receiver_clock_bias_s: pvt_case.receiver_clock_bias_s,
        }
    }

