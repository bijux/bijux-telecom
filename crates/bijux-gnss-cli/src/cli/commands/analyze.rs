fn handle_analyze(command: GnssCommand) -> Result<()> {
    let GnssCommand::Analyze {
        run_dir,
        reference,
    } = command else {
        bail!("invalid command for handler");
    };

    let run_dir = run_dir.canonicalize()?;
    let artifacts_dir = run_dir.join("artifacts");
    let out_dir = artifacts_dir.join("analyze");
    fs::create_dir_all(&out_dir)?;

    let metrics_path = run_dir.join("metrics_summary.json");
    if metrics_path.exists() {
        let data = fs::read_to_string(&metrics_path)?;
        fs::write(out_dir.join("metrics_summary.json"), data)?;
    }

    let mut summary = serde_json::json!({
        "run_dir": run_dir.display().to_string(),
        "nav_epochs": 0,
        "obs_epochs": 0
    });

    let obs_path = artifacts_dir.join("obs").join("obs.jsonl");
    if obs_path.exists() {
        let obs_epochs = read_obs_epochs(&obs_path)?;
        summary["obs_epochs"] = serde_json::json!(obs_epochs.len());
        write_cn0_csv(&out_dir, &obs_epochs)?;
    }

    let nav_path = artifacts_dir.join("pvt").join("pvt.jsonl");
    if nav_path.exists() {
        let solutions = read_nav_solutions(&nav_path)?;
        summary["nav_epochs"] = serde_json::json!(solutions.len());
        write_residual_rms_csv(&out_dir, &solutions)?;
        if let Some(reference_path) = reference {
            let reference_epochs = read_reference_epochs(&reference_path)?;
            write_position_error_csv(&out_dir, &solutions, &reference_epochs)?;
        }
    }

    fs::write(
        out_dir.join("summary.json"),
        serde_json::to_string_pretty(&summary)?,
    )?;

    #[cfg(feature = "plots")]
    {
        let _ = plot_csv(&out_dir.join("cn0_vs_time.csv"), &out_dir.join("cn0_vs_time.png"));
        let _ = plot_csv(
            &out_dir.join("residual_rms_vs_time.csv"),
            &out_dir.join("residual_rms_vs_time.png"),
        );
        let _ = plot_csv(
            &out_dir.join("position_error_vs_time.csv"),
            &out_dir.join("position_error_vs_time.png"),
        );
    }

    println!("wrote analyze artifacts to {}", out_dir.display());
    Ok(())
}

fn handle_diff(command: GnssCommand) -> Result<()> {
    let GnssCommand::Diff { run_a, run_b } = command else {
        bail!("invalid command for handler");
    };
    let run_a = run_a.canonicalize()?;
    let run_b = run_b.canonicalize()?;

    let report = diff_runs(&run_a, &run_b)?;
    println!("{}", serde_json::to_string_pretty(&report)?);
    Ok(())
}

fn diff_runs(run_a: &Path, run_b: &Path) -> Result<serde_json::Value> {
    let nav_a = run_a.join("artifacts/pvt/pvt.jsonl");
    let nav_b = run_b.join("artifacts/pvt/pvt.jsonl");
    let obs_a = run_a.join("artifacts/obs/obs.jsonl");
    let obs_b = run_b.join("artifacts/obs/obs.jsonl");

    let (mean_rms_a, conv_epoch_a) = nav_summary(&nav_a)?;
    let (mean_rms_b, conv_epoch_b) = nav_summary(&nav_b)?;
    let cn0_a = cn0_mean(&obs_a)?;
    let cn0_b = cn0_mean(&obs_b)?;

    Ok(serde_json::json!({
        "run_a": run_a.display().to_string(),
        "run_b": run_b.display().to_string(),
        "mean_rms_m": { "run_a": mean_rms_a, "run_b": mean_rms_b, "delta": mean_rms_b - mean_rms_a },
        "convergence_epoch_idx": { "run_a": conv_epoch_a, "run_b": conv_epoch_b },
        "mean_cn0_dbhz": { "run_a": cn0_a, "run_b": cn0_b, "delta": cn0_b - cn0_a }
    }))
}

fn nav_summary(path: &Path) -> Result<(f64, Option<u64>)> {
    if !path.exists() {
        return Ok((0.0, None));
    }
    let sols = read_nav_solutions(path)?;
    if sols.is_empty() {
        return Ok((0.0, None));
    }
    let mean_rms = sols.iter().map(|s| s.rms_m.0).sum::<f64>() / sols.len() as f64;
    let conv_epoch = sols
        .iter()
        .find(|s| matches!(s.validity, bijux_gnss_infra::api::core::SolutionValidity::Stable))
        .map(|s| s.epoch.index);
    Ok((mean_rms, conv_epoch))
}

fn cn0_mean(path: &Path) -> Result<f64> {
    if !path.exists() {
        return Ok(0.0);
    }
    let obs = read_obs_epochs(path)?;
    let mut sum = 0.0;
    let mut count = 0.0;
    for epoch in obs {
        for sat in epoch.sats {
            sum += sat.cn0_dbhz;
            count += 1.0;
        }
    }
    Ok(if count == 0.0 { 0.0 } else { sum / count })
}

fn write_cn0_csv(out_dir: &Path, obs: &[ObsEpoch]) -> Result<()> {
    let mut lines = Vec::new();
    lines.push("epoch_idx,t_rx_s,constellation,prn,cn0_dbhz".to_string());
    for epoch in obs {
        for sat in &epoch.sats {
            lines.push(format!(
                "{},{},{:?},{},{}",
                epoch.epoch_idx,
                epoch.t_rx_s.0,
                sat.signal_id.sat.constellation,
                sat.signal_id.sat.prn,
                sat.cn0_dbhz
            ));
        }
    }
    fs::write(out_dir.join("cn0_vs_time.csv"), lines.join("\n"))?;
    Ok(())
}

fn write_residual_rms_csv(
    out_dir: &Path,
    solutions: &[bijux_gnss_infra::api::core::NavSolutionEpoch],
) -> Result<()> {
    let mut lines = Vec::new();
    lines.push("epoch_idx,t_rx_s,residual_rms_m".to_string());
    for sol in solutions {
        let mut sum = 0.0;
        let mut count = 0.0;
        for res in &sol.residuals {
            if res.rejected {
                continue;
            }
            sum += res.residual_m.0 * res.residual_m.0;
            count += 1.0;
        }
        let rms = if count == 0.0 { 0.0 } else { (sum / count).sqrt() };
        lines.push(format!("{},{},{}", sol.epoch.index, sol.t_rx_s.0, rms));
    }
    fs::write(out_dir.join("residual_rms_vs_time.csv"), lines.join("\n"))?;
    Ok(())
}

fn write_position_error_csv(
    out_dir: &Path,
    solutions: &[bijux_gnss_infra::api::core::NavSolutionEpoch],
    reference: &[bijux_gnss_infra::api::core::ValidationReferenceEpoch],
) -> Result<()> {
    let mut lines = Vec::new();
    lines.push("epoch_idx,position_error_m".to_string());
    let mut ref_by_epoch = std::collections::BTreeMap::new();
    for ref_epoch in reference {
        ref_by_epoch.insert(ref_epoch.epoch_idx, ref_epoch);
    }
    for sol in solutions {
        if let Some(r) = ref_by_epoch.get(&sol.epoch.index) {
            if let (Some(x), Some(y), Some(z)) = (r.ecef_x_m, r.ecef_y_m, r.ecef_z_m) {
                let dx = sol.ecef_x_m.0 - x;
                let dy = sol.ecef_y_m.0 - y;
                let dz = sol.ecef_z_m.0 - z;
                let err = (dx * dx + dy * dy + dz * dz).sqrt();
                lines.push(format!("{},{}", sol.epoch.index, err));
            }
        }
    }
    fs::write(out_dir.join("position_error_vs_time.csv"), lines.join("\n"))?;
    Ok(())
}

#[cfg(feature = "plots")]
fn plot_csv(input: &Path, output: &Path) -> Result<()> {
    use plotters::prelude::*;
    if !input.exists() {
        return Ok(());
    }
    let data = fs::read_to_string(input)?;
    let mut points = Vec::new();
    for (idx, line) in data.lines().enumerate() {
        if idx == 0 || line.trim().is_empty() {
            continue;
        }
        let parts: Vec<&str> = line.split(',').collect();
        if parts.len() < 2 {
            continue;
        }
        let x: f64 = parts[0].parse().unwrap_or(0.0);
        let y: f64 = parts[parts.len() - 1].parse().unwrap_or(0.0);
        points.push((x, y));
    }
    if points.is_empty() {
        return Ok(());
    }
    let x_max = points.iter().map(|p| p.0).fold(0.0, f64::max);
    let y_max = points.iter().map(|p| p.1).fold(0.0, f64::max);
    let root = BitMapBackend::new(output, (800, 400)).into_drawing_area();
    root.fill(&WHITE)?;
    let mut chart = ChartBuilder::on(&root)
        .margin(10)
        .caption("Metric", ("sans-serif", 20))
        .x_label_area_size(30)
        .y_label_area_size(40)
        .build_cartesian_2d(0.0..x_max.max(1.0), 0.0..y_max.max(1.0))?;
    chart.configure_mesh().draw()?;
    chart.draw_series(LineSeries::new(points, &BLUE))?;
    Ok(())
}
