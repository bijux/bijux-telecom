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
        Commands::BenchCompare {
            strict,
            threshold,
            workspace_root,
        } => run_bench_compare(strict, threshold, workspace_root),
    }
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
        &[
            "bench_correlator",
            "bench_acquisition_fft",
            "bench_tracking_update",
        ],
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
            let name = caps
                .get(1)
                .map(|m| m.as_str())
                .unwrap_or_default()
                .to_string();
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
        let parsed = value
            .parse::<u64>()
            .with_context(|| format!("parse baseline value for {name}"))?;
        baseline_map.insert(name.to_string(), parsed);
    }

    let mut regressions = Vec::new();
    for line in current_text.lines() {
        let mut parts = line.split_whitespace();
        let Some(name) = parts.next() else { continue };
        let Some(value) = parts.next() else { continue };
        let current_ns = value
            .parse::<u64>()
            .with_context(|| format!("parse current value for {name}"))?;
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
