use super::*;

pub(crate) fn validation_evidence_bundle(
    obs: &[ObsEpoch],
    solutions: &[NavSolutionEpoch],
    report: &ValidationReport,
) -> serde_json::Value {
    let mut constellation_counts = std::collections::BTreeMap::new();
    let mut cn0_values = Vec::new();
    let mut lock_total = 0usize;
    let mut lock_good = 0usize;

    for epoch in obs {
        for sat in &epoch.sats {
            let key = format!("{:?}", sat.signal_id.sat.constellation);
            *constellation_counts.entry(key).or_insert(0usize) += 1;
            cn0_values.push(sat.cn0_dbhz);
            lock_total += 1;
            if sat.lock_flags.code_lock && sat.lock_flags.carrier_lock && !sat.lock_flags.cycle_slip
            {
                lock_good += 1;
            }
        }
    }

    let cn0_mean =
        (!cn0_values.is_empty()).then_some(cn0_values.iter().sum::<f64>() / cn0_values.len() as f64);
    let cn0_min = cn0_values.iter().cloned().reduce(f64::min);
    let cn0_max = cn0_values.iter().cloned().reduce(f64::max);

    let mut refusal_counts = std::collections::BTreeMap::new();
    let mut pdop_values = Vec::new();
    let mut gdop_values = Vec::new();
    let mut rms_values = Vec::new();
    for solution in solutions {
        pdop_values.push(solution.pdop);
        if let Some(gdop) = solution.gdop {
            gdop_values.push(gdop);
        }
        rms_values.push(solution.rms_m.0);
        if let Some(refusal) = solution.refusal_class {
            let key = format!("{refusal:?}");
            *refusal_counts.entry(key).or_insert(0usize) += 1;
        }
    }
    let pdop_mean =
        (!pdop_values.is_empty()).then_some(pdop_values.iter().sum::<f64>() / pdop_values.len() as f64);
    let pdop_max = pdop_values.iter().cloned().reduce(f64::max);
    let gdop_mean =
        (!gdop_values.is_empty()).then_some(gdop_values.iter().sum::<f64>() / gdop_values.len() as f64);
    let gdop_max = gdop_values.iter().cloned().reduce(f64::max);
    let residual_rms_mean =
        (!rms_values.is_empty()).then_some(rms_values.iter().sum::<f64>() / rms_values.len() as f64);
    let used_sat_mean = (!solutions.is_empty()).then_some(
        solutions.iter().map(|solution| solution.used_sat_count as f64).sum::<f64>()
            / solutions.len() as f64,
    );
    let stable_solution_count = solutions
        .iter()
        .filter(|solution| {
            matches!(
                solution.validity,
                bijux_gnss_infra::api::core::SolutionValidity::Stable
            )
        })
        .count();
    let weak_integrity_stable_count = report
        .integrity
        .iter()
        .filter(|entry| {
            !matches!(entry.class, bijux_gnss_infra::api::receiver::NavIntegrityClass::Nominal)
        })
        .count();
    let integrity_evidence_missing_stable_count = report
        .integrity
        .iter()
        .filter(|entry| {
            matches!(
                entry.class,
                bijux_gnss_infra::api::receiver::NavIntegrityClass::IntegrityEvidenceMissing
            )
        })
        .count();
    let reference_position_budget_pass =
        report.budgets.reference_position_error_3d_m_max.and_then(|budget_m| {
            (!report.reference_position_errors.is_empty()).then_some(
                report.reference_position_errors.iter().all(|error| error.error_3d_m <= budget_m),
            )
        });

    let mut claim_evidence_violations = Vec::new();
    if let Some(value) = cn0_mean {
        if value < report.science_policy.min_mean_cn0_dbhz {
            claim_evidence_violations.push(format!(
                "mean_cn0_below_policy:{value:.3}<{}",
                report.science_policy.min_mean_cn0_dbhz
            ));
        }
    }
    if let Some(value) = pdop_mean {
        if value > report.science_policy.max_pdop {
            claim_evidence_violations.push(format!(
                "mean_pdop_above_policy:{value:.3}>{}",
                report.science_policy.max_pdop
            ));
        }
    }
    if let Some(value) = gdop_mean {
        if value > report.science_policy.max_gdop {
            claim_evidence_violations.push(format!(
                "mean_gdop_above_policy:{value:.3}>{}",
                report.science_policy.max_gdop
            ));
        }
    }
    if let Some(value) = residual_rms_mean {
        if value > report.science_policy.max_residual_rms_m {
            claim_evidence_violations.push(format!(
                "mean_residual_rms_above_policy:{value:.3}>{}",
                report.science_policy.max_residual_rms_m
            ));
        }
    }
    if let Some(value) = used_sat_mean {
        if value < report.science_policy.min_used_satellites as f64 {
            claim_evidence_violations.push(format!(
                "mean_used_satellites_below_policy:{value:.3}<{}",
                report.science_policy.min_used_satellites
            ));
        }
    }
    let lock_quality_ratio =
        (lock_total > 0).then_some(lock_good as f64 / lock_total as f64);
    if let Some(value) = lock_quality_ratio {
        if value < report.science_policy.min_lock_ratio {
            claim_evidence_violations.push(format!(
                "lock_quality_ratio_below_policy:{value:.3}<{}",
                report.science_policy.min_lock_ratio
            ));
        }
    }
    if stable_solution_count > 0 && weak_integrity_stable_count > 0 {
        claim_evidence_violations.push(format!(
            "stable_solutions_with_non_nominal_integrity:{weak_integrity_stable_count}/{stable_solution_count}"
        ));
    }
    if stable_solution_count > 0 && integrity_evidence_missing_stable_count > 0 {
        claim_evidence_violations.push(format!(
            "stable_solutions_missing_integrity_evidence:{integrity_evidence_missing_stable_count}/{stable_solution_count}"
        ));
    }

    serde_json::json!({
        "schema_version": 1,
        "physical": {
            "observation_epochs": obs.len(),
            "constellation_counts": constellation_counts,
            "cn0_dbhz_mean": cn0_mean,
            "cn0_dbhz_min": cn0_min,
            "cn0_dbhz_max": cn0_max,
            "lock_quality_ratio": lock_quality_ratio
        },
        "numerical": {
            "solution_epochs": solutions.len(),
            "stable_solution_epochs": stable_solution_count,
            "east_error_rms_m": report.east_error_m.rms,
            "north_error_rms_m": report.north_error_m.rms,
            "up_error_rms_m": report.up_error_m.rms,
            "horiz_error_rms_m": report.horiz_error_m.rms,
            "vert_error_rms_m": report.vert_error_m.rms,
            "error_3d_rms_m": report.error_3d_m.rms,
            "reference_match_count": report.reference_position_errors.len(),
            "reference_error_3d_max_m": report.error_3d_m.max,
            "reference_error_3d_budget_m_max": report.budgets.reference_position_error_3d_m_max,
            "reference_error_3d_budget_pass": reference_position_budget_pass,
            "mean_used_satellites": used_sat_mean,
            "pdop_mean": pdop_mean,
            "pdop_max": pdop_max,
            "gdop_mean": gdop_mean,
            "gdop_max": gdop_max,
            "residual_rms_mean_m": residual_rms_mean,
            "refusal_counts": refusal_counts,
            "stable_integrity_evidence_missing_epochs": integrity_evidence_missing_stable_count
        },
        "diagnostics": {
            "advisory": report.diagnostic_partition.advisory_diagnostics,
            "enforced_refusals": report.diagnostic_partition.enforced_refusals
        },
        "claim_evidence_guard": {
            "policy": report.science_policy,
            "supported": claim_evidence_violations.is_empty(),
            "violations": claim_evidence_violations
        }
    })
}
