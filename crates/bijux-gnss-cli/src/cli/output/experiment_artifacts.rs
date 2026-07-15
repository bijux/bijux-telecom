fn write_experiment_run(
    out_dir: &Path,
    idx: usize,
    result: &ExperimentRunResult,
    obs: &[ObsEpoch],
    solutions: &[bijux_gnss_infra::api::core::NavSolutionEpoch],
) -> Result<()> {
    let run_dir = out_dir.join(format!("run_{idx:03}"));
    fs::create_dir_all(&run_dir)?;
    fs::write(run_dir.join("result.json"), serde_json::to_string_pretty(result)?)?;
    let schema_path = schema_path("experiment_run.schema.json");
    if schema_path.exists() {
        validate_json_schema(&schema_path, &run_dir.join("result.json"), false)?;
    }

    let mut cn0_lines = Vec::new();
    cn0_lines.push("epoch_idx,constellation,prn,cn0_dbhz".to_string());
    for epoch in obs {
        for sat in &epoch.sats {
            cn0_lines.push(format!(
                "{},{:?},{},{}",
                epoch.epoch_idx,
                sat.signal_id.sat.constellation,
                sat.signal_id.sat.prn,
                sat.cn0_dbhz
            ));
        }
    }
    fs::write(run_dir.join("cn0.csv"), cn0_lines.join("\n"))?;
    validate_csv_schema(
        &run_dir.join("cn0.csv"),
        "epoch_idx,constellation,prn,cn0_dbhz",
        &[CsvType::U64, CsvType::Str, CsvType::U8, CsvType::F64],
    )?;

    let mut residual_lines = Vec::new();
    residual_lines.push("epoch_idx,constellation,prn,residual_m,rejected".to_string());
    for sol in solutions {
        for res in &sol.residuals {
            residual_lines.push(format!(
                "{},{:?},{},{},{}",
                sol.epoch.index, res.sat.constellation, res.sat.prn, res.residual_m.0, res.rejected
            ));
        }
    }
    fs::write(run_dir.join("residuals.csv"), residual_lines.join("\n"))?;
    validate_csv_schema(
        &run_dir.join("residuals.csv"),
        "epoch_idx,constellation,prn,residual_m,rejected",
        &[CsvType::U64, CsvType::Str, CsvType::U8, CsvType::F64, CsvType::Bool],
    )?;

    let mut ambiguity_lines = Vec::new();
    ambiguity_lines.push("epoch_idx,constellation,prn,carrier_phase_cycles".to_string());
    for epoch in obs {
        for sat in &epoch.sats {
            ambiguity_lines.push(format!(
                "{},{:?},{},{}",
                epoch.epoch_idx,
                sat.signal_id.sat.constellation,
                sat.signal_id.sat.prn,
                sat.carrier_phase_cycles.0
            ));
        }
    }
    fs::write(run_dir.join("ambiguities.csv"), ambiguity_lines.join("\n"))?;
    validate_csv_schema(
        &run_dir.join("ambiguities.csv"),
        "epoch_idx,constellation,prn,carrier_phase_cycles",
        &[CsvType::U64, CsvType::Str, CsvType::U8, CsvType::F64],
    )?;
    Ok(())
}
