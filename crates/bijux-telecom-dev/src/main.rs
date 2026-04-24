use std::collections::BTreeSet;
use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::Command;

use anyhow::{bail, Context, Result};
use clap::{Parser, Subcommand};
use regex::Regex;

#[derive(Parser, Debug)]
#[command(name = "bijux-telecom-dev")]
#[command(about = "Maintainer tooling for the bijux-telecom workspace")]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand, Debug)]
enum Commands {
    /// Validate audit allowlist quality contract.
    AuditAllowlist {
        /// Workspace root override.
        #[arg(long)]
        workspace_root: Option<PathBuf>,
    },
    /// Validate deny policy deviations governance contract.
    DenyPolicyDeviations {
        /// Workspace root override.
        #[arg(long)]
        workspace_root: Option<PathBuf>,
    },
    /// Print cargo audit ignore arguments derived from audit allowlist.
    AuditIgnoreArgs {
        /// Workspace root override.
        #[arg(long)]
        workspace_root: Option<PathBuf>,
    },
    /// Run benchmark suite and compare against baseline.
    BenchCompare {
        /// Fail if a regression exceeds threshold.
        #[arg(long, default_value_t = false)]
        strict: bool,

        /// Regression threshold (1.10 means 10% slower is regression).
        #[arg(long, default_value_t = 1.10)]
        threshold: f64,

        /// Workspace root override.
        #[arg(long)]
        workspace_root: Option<PathBuf>,
    },
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    match cli.command {
        Commands::AuditAllowlist { workspace_root } => run_audit_allowlist_check(workspace_root),
        Commands::DenyPolicyDeviations { workspace_root } => {
            run_deny_policy_deviations_check(workspace_root)
        }
        Commands::AuditIgnoreArgs { workspace_root } => run_audit_ignore_args(workspace_root),
        Commands::BenchCompare { strict, threshold, workspace_root } => {
            run_bench_compare(strict, threshold, workspace_root)
        }
    }
}

fn run_audit_ignore_args(workspace_root: Option<PathBuf>) -> Result<()> {
    let root =
        workspace_root.unwrap_or(std::env::current_dir().context("resolve current directory")?);
    let path = root.join("audit-allowlist.toml");
    if !path.is_file() {
        return Ok(());
    }

    let payload = fs::read_to_string(&path).with_context(|| format!("read {}", path.display()))?;
    let value: toml::Value =
        toml::from_str(&payload).with_context(|| format!("parse {}", path.display()))?;
    let mut advisory_ids = BTreeSet::new();

    if let Some(rows) = value.get("advisory").and_then(toml::Value::as_array) {
        for row in rows {
            let advisory_id = row.get("id").and_then(toml::Value::as_str).unwrap_or("").trim();
            if is_rustsec_id(advisory_id) {
                advisory_ids.insert(advisory_id.to_string());
            }
        }
    }

    if let Some(ignore_rows) = value
        .get("advisories")
        .and_then(toml::Value::as_table)
        .and_then(|table| table.get("ignore"))
        .and_then(toml::Value::as_array)
    {
        for advisory_value in ignore_rows {
            let advisory_id = advisory_value.as_str().unwrap_or("").trim();
            if is_rustsec_id(advisory_id) {
                advisory_ids.insert(advisory_id.to_string());
            }
        }
    }

    let args =
        advisory_ids.into_iter().map(|id| format!("--ignore {id}")).collect::<Vec<_>>().join(" ");
    println!("{args}");
    Ok(())
}

fn run_audit_allowlist_check(workspace_root: Option<PathBuf>) -> Result<()> {
    let root =
        workspace_root.unwrap_or(std::env::current_dir().context("resolve current directory")?);
    let path = root.join("audit-allowlist.toml");
    if !path.is_file() {
        bail!("missing {}", path.display());
    }
    let payload = fs::read_to_string(&path).with_context(|| format!("read {}", path.display()))?;
    let value: toml::Value =
        toml::from_str(&payload).with_context(|| format!("parse {}", path.display()))?;
    let advisories =
        value.get("advisory").and_then(toml::Value::as_array).cloned().unwrap_or_default();
    if advisories.is_empty() {
        println!("check-audit-allowlist: passed");
        return Ok(());
    }

    let today = current_iso_day()?;
    let mut errors = Vec::new();
    for (index, row) in advisories.iter().enumerate() {
        let label = format!("advisory[{index}]");
        let id = row.get("id").and_then(toml::Value::as_str).unwrap_or("").trim();
        let why = row.get("why").and_then(toml::Value::as_str).unwrap_or("").trim();
        let owner = row.get("owner").and_then(toml::Value::as_str).unwrap_or("").trim();
        let link = row.get("link").and_then(toml::Value::as_str).unwrap_or("").trim();
        let expiry = row.get("expiry").and_then(toml::Value::as_str).unwrap_or("").trim();

        if !is_rustsec_id(id) {
            errors.push(format!("{label}: id must match RUSTSEC-YYYY-NNNN"));
        }
        if why.is_empty() {
            errors.push(format!("{label}: missing why"));
        }
        if owner.is_empty() {
            errors.push(format!("{label}: missing owner"));
        }
        if !(link.starts_with("http://") || link.starts_with("https://")) {
            errors.push(format!("{label}: link must be http(s)"));
        }
        if !is_iso_day(expiry) {
            errors.push(format!("{label}: expiry must be YYYY-MM-DD"));
        } else if expiry < today.as_str() {
            errors.push(format!("{label}: expiry has passed ({expiry})"));
        }
    }

    if errors.is_empty() {
        println!("check-audit-allowlist: passed");
        return Ok(());
    }

    bail!("audit allowlist quality gate failed:\n{}", errors.join("\n"));
}

fn run_deny_policy_deviations_check(workspace_root: Option<PathBuf>) -> Result<()> {
    let root =
        workspace_root.unwrap_or(std::env::current_dir().context("resolve current directory")?);
    let path = root.join("configs/rust/deny.deviations.toml");
    if !path.is_file() {
        bail!("missing {}", path.display());
    }
    let payload = fs::read_to_string(&path).with_context(|| format!("read {}", path.display()))?;
    let value: toml::Value =
        toml::from_str(&payload).with_context(|| format!("parse {}", path.display()))?;
    let rows = value.get("deviation").and_then(toml::Value::as_array).cloned().unwrap_or_default();
    if rows.is_empty() {
        println!("check-deny-policy-deviations: passed");
        return Ok(());
    }
    let today = current_iso_day()?;
    let mut errors = Vec::new();
    for (index, row) in rows.iter().enumerate() {
        let label = format!("deviation[{index}]");
        let id = row.get("id").and_then(toml::Value::as_str).unwrap_or("").trim();
        let owner = row.get("owner").and_then(toml::Value::as_str).unwrap_or("").trim();
        let reason = row.get("reason").and_then(toml::Value::as_str).unwrap_or("").trim();
        let expiry = row.get("expiry").and_then(toml::Value::as_str).unwrap_or("").trim();
        let review = row.get("review").and_then(toml::Value::as_str).unwrap_or("").trim();
        if id.is_empty() {
            errors.push(format!("{label}: missing id"));
        }
        if owner.is_empty() {
            errors.push(format!("{label}: missing owner"));
        }
        if reason.is_empty() {
            errors.push(format!("{label}: missing reason"));
        }
        if !is_iso_day(expiry) {
            errors.push(format!("{label}: expiry must be YYYY-MM-DD"));
        } else if expiry < today.as_str() {
            errors.push(format!("{label}: expiry has passed ({expiry})"));
        }
        if !(review.starts_with("http://") || review.starts_with("https://")) {
            errors.push(format!("{label}: review must be an http(s) link"));
        } else if !review.contains("bijux-std") {
            errors.push(format!("{label}: review must reference bijux-std"));
        }
    }
    if errors.is_empty() {
        println!("check-deny-policy-deviations: passed");
        return Ok(());
    }
    bail!("deny policy deviations governance gate failed:\n{}", errors.join("\n"));
}

fn current_iso_day() -> Result<String> {
    let output =
        Command::new("date").args(["+%Y-%m-%d"]).output().context("resolve current date")?;
    if !output.status.success() {
        bail!("resolve current date failed");
    }
    Ok(String::from_utf8(output.stdout).context("date output is not UTF-8")?.trim().to_string())
}

fn is_iso_day(value: &str) -> bool {
    value.len() == 10
        && value.chars().nth(4) == Some('-')
        && value.chars().nth(7) == Some('-')
        && value
            .chars()
            .enumerate()
            .all(|(index, ch)| (index == 4 || index == 7) || ch.is_ascii_digit())
}

fn is_rustsec_id(value: &str) -> bool {
    value.len() == 17
        && value.starts_with("RUSTSEC-")
        && value.as_bytes().get(12) == Some(&b'-')
        && value[8..12].chars().all(|ch| ch.is_ascii_digit())
        && value[13..17].chars().all(|ch| ch.is_ascii_digit())
}

fn run_bench_compare(strict: bool, threshold: f64, workspace_root: Option<PathBuf>) -> Result<()> {
    let root =
        workspace_root.unwrap_or(std::env::current_dir().context("resolve current directory")?);
    let artifacts_dir = root.join("artifacts");
    let benchmarks_dir = root.join("benchmarks");
    let baseline = benchmarks_dir.join("bencher_baseline.txt");
    let current = benchmarks_dir.join("bencher_current.txt");
    let benchmark_log = artifacts_dir.join("benchmarks.txt");

    fs::create_dir_all(&artifacts_dir).context("create artifacts directory")?;
    fs::create_dir_all(&benchmarks_dir).context("create benchmarks directory")?;

    println!("Running benchmarks (bencher output)...");
    let mut combined = String::new();
    combined.push_str(&run_bench(
        &root,
        "bijux-gnss-receiver",
        &["bench_correlator", "bench_acquisition_fft", "bench_tracking_update"],
    )?);
    combined.push_str(&run_bench(&root, "bijux-gnss-nav", &["bench_ekf_update"])?);

    fs::write(&benchmark_log, combined.as_bytes()).context("write benchmark log")?;
    write_current_snapshot(&combined, &current)?;

    if !baseline.exists() {
        println!(
            "No benchmark baseline found at {}. Skipping regression check.",
            baseline.display()
        );
        return Ok(());
    }

    println!("Comparing benchmarks to baseline...");
    let regressions = compare_baseline(&baseline, &current, threshold)?;
    for line in &regressions {
        eprintln!("{line}");
    }

    if strict && !regressions.is_empty() {
        bail!("benchmark regression threshold exceeded with strict mode enabled");
    }

    println!("Benchmark comparison complete.");
    Ok(())
}

fn run_bench(workspace_root: &Path, package: &str, benches: &[&str]) -> Result<String> {
    let mut args = vec!["bench", "-p", package];
    for bench in benches {
        args.push("--bench");
        args.push(bench);
    }
    args.push("--");
    args.push("--output-format");
    args.push("bencher");

    let output = Command::new("cargo")
        .current_dir(workspace_root)
        .args(&args)
        .output()
        .with_context(|| format!("run cargo bench for package {package}"))?;

    let stdout = String::from_utf8(output.stdout).context("cargo bench stdout is not UTF-8")?;
    let stderr = String::from_utf8(output.stderr).context("cargo bench stderr is not UTF-8")?;

    print!("{stdout}");
    eprint!("{stderr}");

    if !output.status.success() {
        bail!("cargo bench failed for package {package}");
    }

    Ok(stdout)
}

fn write_current_snapshot(bench_output: &str, current_path: &Path) -> Result<()> {
    let line_re = Regex::new(r"^test ([^ ]+) .* bench:\s*([0-9,]+) ns/iter$")
        .context("compile benchmark line regex")?;

    let mut rows = Vec::<(String, u64)>::new();
    for line in bench_output.lines() {
        if let Some(caps) = line_re.captures(line) {
            let name = caps.get(1).map(|m| m.as_str()).unwrap_or_default().to_string();
            let value = caps
                .get(2)
                .map(|m| m.as_str())
                .unwrap_or_default()
                .replace(',', "")
                .parse::<u64>()
                .with_context(|| format!("parse benchmark value for {name}"))?;
            rows.push((name, value));
        }
    }

    rows.sort_by(|a, b| a.0.cmp(&b.0));

    let mut out = fs::File::create(current_path)
        .with_context(|| format!("create {}", current_path.display()))?;
    for (name, value) in rows {
        writeln!(out, "{name} {value}")
            .with_context(|| format!("write {}", current_path.display()))?;
    }

    Ok(())
}

fn compare_baseline(baseline: &Path, current: &Path, threshold: f64) -> Result<Vec<String>> {
    let baseline_text =
        fs::read_to_string(baseline).with_context(|| format!("read {}", baseline.display()))?;
    let current_text =
        fs::read_to_string(current).with_context(|| format!("read {}", current.display()))?;

    let mut baseline_map = std::collections::BTreeMap::<String, u64>::new();
    for line in baseline_text.lines() {
        let mut parts = line.split_whitespace();
        let Some(name) = parts.next() else { continue };
        let Some(value) = parts.next() else { continue };
        let parsed =
            value.parse::<u64>().with_context(|| format!("parse baseline value for {name}"))?;
        baseline_map.insert(name.to_string(), parsed);
    }

    let mut regressions = Vec::new();
    for line in current_text.lines() {
        let mut parts = line.split_whitespace();
        let Some(name) = parts.next() else { continue };
        let Some(value) = parts.next() else { continue };
        let current_ns =
            value.parse::<u64>().with_context(|| format!("parse current value for {name}"))?;
        let Some(baseline_ns) = baseline_map.get(name) else {
            continue;
        };
        let ratio = current_ns as f64 / *baseline_ns as f64;
        if ratio > threshold {
            regressions.push(format!(
                "Regression: {name} current={current_ns} ns baseline={baseline_ns} ns ({ratio:.2}x)"
            ));
        }
    }

    Ok(regressions)
}
