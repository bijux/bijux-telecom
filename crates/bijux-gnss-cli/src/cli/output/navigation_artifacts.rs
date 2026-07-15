#[derive(Debug, Serialize)]
struct NavSolutionOutput {
    schema_version: u32,
    epoch_idx: u64,
    t_rx_s: f64,
    source_time: ReceiverSampleTraceOutput,
    source_observation_epoch_id: String,
    ecef_m: [f64; 3],
    llh_deg: [f64; 3],
    velocity_mps: Option<[f64; 3]>,
    clock_bias_s: f64,
    clock_bias_m: f64,
    clock_drift_s_per_s: f64,
    dops: NavSolutionDops,
    covariance: NavSolutionCovariance,
    solver_diagnostics: NavSolutionSolverDiagnostics,
    fix_quality: bijux_gnss_infra::api::core::NavQualityFlag,
    validity: bijux_gnss_infra::api::core::SolutionValidity,
    rejected_measurements: usize,
}

#[derive(Debug, Serialize)]
struct NavSolutionDops {
    pdop: f64,
    hdop: Option<f64>,
    vdop: Option<f64>,
    gdop: Option<f64>,
    tdop: Option<f64>,
}

#[derive(Debug, Serialize)]
struct NavSolutionCovariance {
    sigma_h_m: Option<f64>,
    sigma_v_m: Option<f64>,
    covariance_xyz_m2: Option<[f64; 3]>,
}

#[derive(Debug, Serialize)]
struct NavSolutionSolverDiagnostics {
    wls_solver_rank: Option<usize>,
    wls_condition_number: Option<f64>,
    ekf_condition_number: Option<f64>,
}

#[derive(Debug, Serialize)]
struct ReceiverSampleTraceOutput {
    sample_index: u64,
    sample_rate_hz: f64,
    receiver_time_s: f64,
}

fn write_nav_solution_outputs(
    out_dir: &Path,
    solutions: &[bijux_gnss_infra::api::core::NavSolutionEpoch],
) -> Result<()> {
    let mut lines = Vec::new();
    for sol in solutions {
        let output = NavSolutionOutput {
            schema_version: 1,
            epoch_idx: sol.epoch.index,
            t_rx_s: sol.t_rx_s.0,
            source_time: ReceiverSampleTraceOutput {
                sample_index: sol.source_time.sample_index,
                sample_rate_hz: sol.source_time.sample_rate_hz,
                receiver_time_s: sol.source_time.receiver_time_s.0,
            },
            source_observation_epoch_id: sol.source_observation_epoch_id.clone(),
            ecef_m: [sol.ecef_x_m.0, sol.ecef_y_m.0, sol.ecef_z_m.0],
            llh_deg: [sol.latitude_deg, sol.longitude_deg, sol.altitude_m.0],
            velocity_mps: None,
            clock_bias_s: sol.clock_bias_s.0,
            clock_bias_m: sol.clock_bias_m.0,
            clock_drift_s_per_s: sol.clock_drift_s_per_s,
            dops: NavSolutionDops {
                pdop: sol.pdop,
                hdop: sol.hdop,
                vdop: sol.vdop,
                gdop: sol.gdop,
                tdop: sol.tdop,
            },
            covariance: NavSolutionCovariance {
                sigma_h_m: sol.sigma_h_m.map(|m| m.0),
                sigma_v_m: sol.sigma_v_m.map(|m| m.0),
                covariance_xyz_m2: None,
            },
            solver_diagnostics: NavSolutionSolverDiagnostics {
                wls_solver_rank: sol.wls_solver_rank,
                wls_condition_number: sol.wls_condition_number,
                ekf_condition_number: sol.ekf_condition_number,
            },
            fix_quality: sol.quality,
            validity: sol.validity,
            rejected_measurements: sol.residuals.iter().filter(|r| r.rejected).count(),
        };
        lines.push(serde_json::to_string(&output)?);
    }
    let path = out_dir.join("nav_solution.jsonl");
    fs::write(&path, lines.join("\n"))?;
    let schema = schema_path("nav_solution.schema.json");
    if schema.exists() {
        validate_jsonl_schema(&schema, &path, false)?;
    }
    Ok(())
}
