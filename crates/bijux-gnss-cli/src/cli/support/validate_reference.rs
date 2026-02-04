fn reference_ecef(r: &ValidationReferenceEpoch) -> (f64, f64, f64) {
    if let (Some(x), Some(y), Some(z)) = (r.ecef_x_m, r.ecef_y_m, r.ecef_z_m) {
        (x, y, z)
    } else {
        bijux_gnss_nav::geodetic_to_ecef(r.latitude_deg, r.longitude_deg, r.altitude_m)
    }
}

fn align_reference_by_time(
    solutions: &[bijux_gnss_core::NavSolutionEpoch],
    reference: &[ValidationReferenceEpoch],
    policy: ReferenceAlign,
) -> Vec<ValidationReferenceEpoch> {
    let mut out = Vec::new();
    let mut ref_sorted: Vec<_> = reference
        .iter()
        .filter_map(|r| r.t_rx_s.map(|t| (t, r)))
        .collect();
    ref_sorted.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    for sol in solutions {
        let t = sol.t_rx_s.0;
        let mut best = None;
        for window in ref_sorted.windows(2) {
            if t >= window[0].0 && t <= window[1].0 {
                let (t0, r0) = window[0];
                let (t1, r1) = window[1];
                let alpha = if (t1 - t0).abs() < 1e-9 {
                    0.0
                } else {
                    (t - t0) / (t1 - t0)
                };
                let chosen = match policy {
                    ReferenceAlign::Nearest => {
                        if (t - t0).abs() <= (t1 - t).abs() {
                            r0.clone()
                        } else {
                            r1.clone()
                        }
                    }
                    ReferenceAlign::Linear => {
                        let (x0, y0, z0) = reference_ecef(r0);
                        let (x1, y1, z1) = reference_ecef(r1);
                        let x = x0 + alpha * (x1 - x0);
                        let y = y0 + alpha * (y1 - y0);
                        let z = z0 + alpha * (z1 - z0);
                        ValidationReferenceEpoch {
                            epoch_idx: sol.epoch.index,
                            t_rx_s: Some(t),
                            latitude_deg: r0.latitude_deg,
                            longitude_deg: r0.longitude_deg,
                            altitude_m: r0.altitude_m,
                            ecef_x_m: Some(x),
                            ecef_y_m: Some(y),
                            ecef_z_m: Some(z),
                            vel_x_mps: r0.vel_x_mps,
                            vel_y_mps: r0.vel_y_mps,
                            vel_z_mps: r0.vel_z_mps,
                        }
                    }
                };
                best = Some(chosen);
                break;
            }
        }
        if let Some(mut ref_epoch) = best {
            ref_epoch.epoch_idx = sol.epoch.index;
            out.push(ref_epoch);
        }
    }
    out
}

#[derive(Debug, Serialize)]
#[allow(dead_code)]
struct ReferenceCompareStats {
    count: usize,
    horiz_rms_m: f64,
    vert_rms_m: f64,
}

#[allow(dead_code)]
fn reference_compare(
    solutions: &[bijux_gnss_core::NavSolutionEpoch],
    reference: &[ValidationReferenceEpoch],
) -> (Vec<String>, ReferenceCompareStats) {
    let mut ref_map = std::collections::BTreeMap::new();
    for r in reference {
        ref_map.insert(r.epoch_idx, r);
    }
    let mut rows = Vec::new();
    rows.push("epoch_idx,dx_m,dy_m,dz_m,horiz_m,vert_m".to_string());
    let mut horiz = Vec::new();
    let mut vert = Vec::new();
    for sol in solutions {
        if let Some(r) = ref_map.get(&sol.epoch.index) {
            let (x, y, z) = lla_to_ecef(r.latitude_deg, r.longitude_deg, r.altitude_m);
            let dx = sol.ecef_x_m.0 - x;
            let dy = sol.ecef_y_m.0 - y;
            let dz = sol.ecef_z_m.0 - z;
            let h = (dx * dx + dy * dy).sqrt();
            let v = dz.abs();
            horiz.push(h);
            vert.push(v);
            rows.push(format!(
                "{},{:.4},{:.4},{:.4},{:.4},{:.4}",
                sol.epoch.index, dx, dy, dz, h, v
            ));
        }
    }
    let horiz_rms = if horiz.is_empty() {
        0.0
    } else {
        (horiz.iter().map(|v| v * v).sum::<f64>() / horiz.len() as f64).sqrt()
    };
    let vert_rms = if vert.is_empty() {
        0.0
    } else {
        (vert.iter().map(|v| v * v).sum::<f64>() / vert.len() as f64).sqrt()
    };
    (
        rows,
        ReferenceCompareStats {
            count: horiz.len(),
            horiz_rms_m: horiz_rms,
            vert_rms_m: vert_rms,
        },
    )
}

fn check_solution_consistency(
    solutions: &[bijux_gnss_core::NavSolutionEpoch],
) -> SolutionConsistencyReport {
    let mut position_jump_count = 0;
    let mut clock_jump_count = 0;
    let mut pdop_spike_count = 0;
    let mut warnings = Vec::new();
    let mut prev: Option<&bijux_gnss_core::NavSolutionEpoch> = None;
    let mut prev_pdop: Option<f64> = None;
    for sol in solutions {
        if let Some(prev_sol) = prev {
            let dx = sol.ecef_x_m.0 - prev_sol.ecef_x_m.0;
            let dy = sol.ecef_y_m.0 - prev_sol.ecef_y_m.0;
            let dz = sol.ecef_z_m.0 - prev_sol.ecef_z_m.0;
            let dist = (dx * dx + dy * dy + dz * dz).sqrt();
            if dist > 50.0 {
                position_jump_count += 1;
            }
            let clock_jump = (sol.clock_bias_s.0 - prev_sol.clock_bias_s.0).abs();
            if clock_jump > 1e-3 {
                clock_jump_count += 1;
            }
        }
        if let Some(prev_pdop) = prev_pdop {
            if sol.pdop > prev_pdop * 2.5 && sol.pdop > 6.0 {
                pdop_spike_count += 1;
            }
        }
        prev_pdop = Some(sol.pdop);
        prev = Some(sol);
    }
    if position_jump_count > 0 {
        warnings.push("position jumps detected".to_string());
    }
    if clock_jump_count > 0 {
        warnings.push("clock jumps detected".to_string());
    }
    if pdop_spike_count > 0 {
        warnings.push("pdop spikes detected".to_string());
    }
    SolutionConsistencyReport {
        position_jump_count,
        clock_jump_count,
        pdop_spike_count,
        warnings,
    }
}

fn check_time_consistency(
    tracks: &[bijux_gnss_receiver::tracking::TrackingResult],
    sample_rate_hz: f64,
) -> TimeConsistencyReport {
    let mut epoch_backward = 0;
    let mut epoch_gaps = 0;
    let mut sample_backward = 0;
    let mut warnings = Vec::new();
    let mut epochs_checked = 0;
    let mut sample_step_mismatch = 0;
    let configured_step = if sample_rate_hz > 0.0 {
        Some((sample_rate_hz * 0.001).round().max(1.0) as u64)
    } else {
        None
    };
    let mut expected_step: Option<u64> = configured_step;
    let mut sample_step_total = 0u64;
    let mut sample_step_count = 0u64;

    for track in tracks {
        let mut prev_epoch: Option<u64> = None;
        let mut prev_sample: Option<u64> = None;
        for epoch in &track.epochs {
            epochs_checked += 1;
            if let Some(prev) = prev_epoch {
                if epoch.epoch.index < prev {
                    epoch_backward += 1;
                } else if epoch.epoch.index > prev + 1 {
                    epoch_gaps += 1;
                }
            }
            if let Some(prev) = prev_sample {
                if epoch.sample_index < prev {
                    sample_backward += 1;
                } else {
                    let step = epoch.sample_index - prev;
                    if let Some(expected) = expected_step {
                        if step != expected {
                            sample_step_mismatch += 1;
                        }
                    } else {
                        expected_step = Some(step);
                    }
                    sample_step_total = sample_step_total.saturating_add(step);
                    sample_step_count = sample_step_count.saturating_add(1);
                }
            }
            prev_epoch = Some(epoch.epoch.index);
            prev_sample = Some(epoch.sample_index);
        }
    }

    if epoch_backward > 0 {
        warnings.push("epoch index went backwards".to_string());
    }
    if epoch_gaps > 0 {
        warnings.push("epoch index gaps detected".to_string());
    }
    if sample_backward > 0 {
        warnings.push("sample index went backwards".to_string());
    }
    if sample_step_mismatch > 0 {
        warnings.push("sample cadence mismatch detected".to_string());
    }

    TimeConsistencyReport {
        channels: tracks.len(),
        epochs_checked,
        epoch_backward,
        epoch_gaps,
        sample_backward,
        sample_step_mismatch,
        expected_step,
        observed_step_mean: if sample_step_count > 0 {
            Some(sample_step_total as f64 / sample_step_count as f64)
        } else {
            None
        },
        warnings,
    }
}
