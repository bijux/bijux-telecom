/// Build a truth-guided PVT table from synthetic navigation solutions.
pub fn validate_truth_guided_pvt_table(
    scenario_id: &str,
    solutions: &[NavSolutionEpoch],
    reference: &[SyntheticPvtTruthReferenceEpoch],
) -> SyntheticPvtTruthTableReport {
    let reference_by_epoch =
        reference.iter().map(|epoch| (epoch.position.epoch_idx, epoch)).collect::<BTreeMap<_, _>>();
    let mut epochs = Vec::new();
    let mut unmatched_solution_epochs = Vec::new();
    let mut matched_epoch_indices = std::collections::BTreeSet::new();

    for solution in solutions {
        let Some(reference_epoch) = reference_by_epoch.get(&solution.epoch.index) else {
            unmatched_solution_epochs.push(solution.epoch.index);
            continue;
        };

        matched_epoch_indices.insert(solution.epoch.index);
        let truth_ecef = reference_ecef(&reference_epoch.position);
        let truth_geodetic = ecef_to_geodetic(truth_ecef.0, truth_ecef.1, truth_ecef.2);
        let (east_m, north_m, up_m) = ecef_to_enu(
            solution.ecef_x_m.0,
            solution.ecef_y_m.0,
            solution.ecef_z_m.0,
            reference_epoch.position.latitude_deg,
            reference_epoch.position.longitude_deg,
            reference_epoch.position.altitude_m,
        );
        let horiz_m = (east_m * east_m + north_m * north_m).sqrt();
        let vert_m = up_m.abs();
        let error_3d_m = (horiz_m * horiz_m + up_m * up_m).sqrt();
        let clock_bias_truth_m = reference_epoch.clock_bias_s * SPEED_OF_LIGHT_MPS;

        epochs.push(SyntheticPvtTruthTableEpoch {
            artifact_id: solution.artifact_id.clone(),
            source_observation_epoch_id: solution.source_observation_epoch_id.clone(),
            epoch_index: solution.epoch.index,
            receive_time_s: solution.t_rx_s.0,
            truth_ecef_m: SyntheticPvtTruthTableEcef {
                x_m: truth_ecef.0,
                y_m: truth_ecef.1,
                z_m: truth_ecef.2,
            },
            measured_ecef_m: SyntheticPvtTruthTableEcef {
                x_m: solution.ecef_x_m.0,
                y_m: solution.ecef_y_m.0,
                z_m: solution.ecef_z_m.0,
            },
            position_covariance_ecef_m2: solution.position_covariance_ecef_m2,
            ecef_error_m: SyntheticPvtTruthTableEcef {
                x_m: solution.ecef_x_m.0 - truth_ecef.0,
                y_m: solution.ecef_y_m.0 - truth_ecef.1,
                z_m: solution.ecef_z_m.0 - truth_ecef.2,
            },
            truth_geodetic: SyntheticPvtTruthTableGeodetic {
                latitude_deg: truth_geodetic.0,
                longitude_deg: truth_geodetic.1,
                altitude_m: truth_geodetic.2,
            },
            measured_geodetic: SyntheticPvtTruthTableGeodetic {
                latitude_deg: solution.latitude_deg,
                longitude_deg: solution.longitude_deg,
                altitude_m: solution.altitude_m.0,
            },
            enu_error_m: SyntheticPvtTruthTableEnuError {
                east_m,
                north_m,
                up_m,
                horiz_m,
                vert_m,
                error_3d_m,
            },
            clock_bias: SyntheticPvtTruthTableClockBias {
                truth_s: reference_epoch.clock_bias_s,
                measured_s: solution.clock_bias_s.0,
                error_s: solution.clock_bias_s.0 - reference_epoch.clock_bias_s,
                truth_m: clock_bias_truth_m,
                measured_m: solution.clock_bias_m.0,
                error_m: solution.clock_bias_m.0 - clock_bias_truth_m,
            },
            residual_rms_m: solution.rms_m.0,
            pre_fit_residual_rms_m: solution.pre_fit_residual_rms_m.map(|value| value.0),
            post_fit_residual_rms_m: solution.post_fit_residual_rms_m.map(|value| value.0),
            dop: SyntheticPvtTruthTableDop {
                pdop: solution.pdop,
                hdop: solution.hdop,
                vdop: solution.vdop,
                gdop: solution.gdop,
                tdop: solution.tdop,
            },
            solution_status: solution.status,
            solution_quality: solution.quality,
            solution_validity: solution.validity,
            valid: solution.valid,
            sat_count: solution.sat_count,
            used_sat_count: solution.used_sat_count,
            rejected_sat_count: solution.rejected_sat_count,
        });
    }

    let unused_reference_epochs = reference
        .iter()
        .map(|epoch| epoch.position.epoch_idx)
        .filter(|epoch_idx| !matched_epoch_indices.contains(epoch_idx))
        .collect::<Vec<_>>();

    SyntheticPvtTruthTableReport {
        scenario_id: scenario_id.to_string(),
        solution_count: solutions.len(),
        matched_epoch_count: epochs.len(),
        unmatched_solution_epochs,
        unused_reference_epochs,
        epochs,
    }
}

fn carrier_phase_arc_start_sample_index(
    observation: &bijux_gnss_core::api::ObsSatellite,
) -> Option<u64> {
    (observation.lock_flags.carrier_lock
        && observation.metadata.carrier_phase_continuity != "unusable")
        .then_some(observation.metadata.carrier_phase_arc_start_sample_index)
}

fn carrier_phase_arc_bias_cycles_by_start_sample(
    config: &ReceiverPipelineConfig,
    sat_state: &SatState,
    observed_rows: &[ObservedSatelliteRow<'_>],
) -> BTreeMap<u64, f64> {
    let mut differences_by_arc = BTreeMap::<u64, Vec<f64>>::new();
    for row in observed_rows {
        let Some(arc_start_sample_index) = carrier_phase_arc_start_sample_index(row.observation)
        else {
            continue;
        };
        let truth_cycles = sat_state
            .carrier_phase_rad_at(row.sample_index as f64 / config.sampling_freq_hz)
            / std::f64::consts::TAU;
        differences_by_arc
            .entry(arc_start_sample_index)
            .or_default()
            .push(row.observation.carrier_phase_cycles.0 - truth_cycles);
    }

    differences_by_arc
        .into_iter()
        .map(|(arc_start_sample_index, differences)| {
            (arc_start_sample_index, stats(&differences).median)
        })
        .collect()
}

fn comparable_signal_band_observations(
    observations: &[ObsEpoch],
    sat: SatId,
    signal_band: Option<bijux_gnss_core::api::SignalBand>,
) -> Vec<ObservedSatelliteRow<'_>> {
    observations
        .iter()
        .filter(|epoch| epoch.valid)
        .flat_map(|epoch| {
            let artifact_id = epoch
                .manifest
                .as_ref()
                .map(|manifest| manifest.artifact_id.clone())
                .unwrap_or_else(|| format!("obs-epoch-{:010}", epoch.epoch_idx));
            let epoch_id = epoch
                .manifest
                .as_ref()
                .map(|manifest| manifest.epoch_id.clone())
                .unwrap_or_else(|| bijux_gnss_core::api::obs_epoch_stability_key(epoch));
            epoch.sats.iter().filter_map(move |observation| {
                (observation.signal_id.sat == sat
                    && signal_band.is_none_or(|band| observation.signal_id.band == band)
                    && matches!(
                        observation.observation_status,
                        ObservationStatus::Accepted | ObservationStatus::Weak
                    ))
                .then_some(ObservedSatelliteRow {
                    artifact_id: artifact_id.clone(),
                    epoch_id: epoch_id.clone(),
                    epoch_index: epoch.epoch_idx,
                    sample_index: epoch.source_time.sample_index,
                    observation,
                })
            })
        })
        .collect()
}

fn synthetic_pseudorange_m(
    ephemeris: &GpsEphemeris,
    receive_time_s: f64,
    receiver_ecef_m: [f64; 3],
) -> f64 {
    let mut tau_s = 0.07;
    let mut pseudorange_m = 0.0;
    for _ in 0..10 {
        let sat = sat_state_gps_l1ca(ephemeris, receive_time_s - tau_s, tau_s);
        let dx = receiver_ecef_m[0] - sat.x_m;
        let dy = receiver_ecef_m[1] - sat.y_m;
        let dz = receiver_ecef_m[2] - sat.z_m;
        let geometric_range_m = (dx * dx + dy * dy + dz * dz).sqrt();
        pseudorange_m = geometric_range_m - sat.clock_correction.bias_s * SPEED_OF_LIGHT_MPS;
        let next_tau_s = pseudorange_m / SPEED_OF_LIGHT_MPS;
        if (next_tau_s - tau_s).abs() < 1.0e-12 {
            break;
        }
        tau_s = next_tau_s;
    }
    pseudorange_m
}
