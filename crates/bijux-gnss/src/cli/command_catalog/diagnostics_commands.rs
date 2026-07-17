use super::*;

#[derive(Subcommand)]
pub(crate) enum DiagnosticsCommand {
    /// Print concise operator map for run, diagnose, replay, compare, and export workflows
    OperatorMap {
        #[command(flatten)]
        common: CommonArgs,
    },

    /// Print operator workflow map for run, inspect, validate, diagnose, replay, and compare tasks
    Workflow {
        #[command(flatten)]
        common: CommonArgs,
    },

    /// Summarize diagnostics from a run directory
    Summarize {
        #[command(flatten)]
        common: CommonArgs,

        /// Run directory (expects artifacts/ inside)
        #[arg(long, value_name = "DIR")]
        run_dir: PathBuf,

        /// Max number of codes to print
        #[arg(long, default_value_t = 10)]
        top: usize,
    },

    /// Explain run replay scope, artifact identity coverage, and cache behavior
    Explain {
        #[command(flatten)]
        common: CommonArgs,

        /// Run directory (expects manifest.json and artifacts/)
        #[arg(long, value_name = "DIR")]
        run_dir: PathBuf,
    },

    /// Verify reproducibility/audit bundle integrity for a run directory
    VerifyRepro {
        #[command(flatten)]
        common: CommonArgs,

        /// Run directory (expects manifest.json and artifacts/)
        #[arg(long, value_name = "DIR")]
        run_dir: PathBuf,
    },

    /// Compare two run directories using reproducibility and quality evidence
    Compare {
        #[command(flatten)]
        common: CommonArgs,

        /// Baseline run directory
        #[arg(long, value_name = "DIR")]
        baseline_run_dir: PathBuf,

        /// Candidate run directory
        #[arg(long, value_name = "DIR")]
        candidate_run_dir: PathBuf,
    },

    /// Audit replay determinism and classify drift between two run directories
    ReplayAudit {
        #[command(flatten)]
        common: CommonArgs,

        /// Baseline run directory
        #[arg(long, value_name = "DIR")]
        baseline_run_dir: PathBuf,

        /// Candidate run directory
        #[arg(long, value_name = "DIR")]
        candidate_run_dir: PathBuf,
    },

    /// Gate advanced RTK/PPP workflow claims using support maturity and recorded evidence
    AdvancedGate {
        #[command(flatten)]
        common: CommonArgs,

        /// Run directory to evaluate
        #[arg(long, value_name = "DIR")]
        run_dir: PathBuf,

        /// Advanced workflow mode to evaluate
        #[arg(long, value_enum)]
        mode: AdvancedGateMode,

        /// Fail command if gate does not pass
        #[arg(long)]
        strict: bool,
    },

    /// Summarize run artifacts grouped by stage to make outputs easier to locate
    ArtifactInventory {
        #[command(flatten)]
        common: CommonArgs,

        /// Run directory to inspect
        #[arg(long, value_name = "DIR")]
        run_dir: PathBuf,
    },

    /// Emit focused local debugging workflow for acquisition, tracking, observations, and PVT
    DebugPlan {
        #[command(flatten)]
        common: CommonArgs,

        /// Run directory to inspect
        #[arg(long, value_name = "DIR")]
        run_dir: PathBuf,
    },

    /// Emit digestible benchmark summary without dropping rigorous metrics
    BenchmarkSummary {
        #[command(flatten)]
        common: CommonArgs,

        /// Run directory to inspect
        #[arg(long, value_name = "DIR")]
        run_dir: PathBuf,
    },

    /// Run medium integration gate across replay integrity and evidence stability
    MediumGate {
        #[command(flatten)]
        common: CommonArgs,

        /// Run directory to inspect
        #[arg(long, value_name = "DIR")]
        run_dir: PathBuf,

        /// Fail command when medium gate does not pass
        #[arg(long)]
        strict: bool,
    },

    /// Operator-focused run state, quality, and evidence summary
    OperatorStatus {
        #[command(flatten)]
        common: CommonArgs,

        /// Run directory to inspect
        #[arg(long, value_name = "DIR")]
        run_dir: PathBuf,
    },

    /// Structured multi-channel summary for large GNSS runs
    ChannelSummary {
        #[command(flatten)]
        common: CommonArgs,

        /// Run directory to inspect
        #[arg(long, value_name = "DIR")]
        run_dir: PathBuf,
    },

    /// Export reproducible review bundle with key artifacts and checksums
    ExportBundle {
        #[command(flatten)]
        common: CommonArgs,

        /// Run directory to export from
        #[arg(long, value_name = "DIR")]
        run_dir: PathBuf,

        /// Optional bundle output directory
        #[arg(long, value_name = "DIR")]
        out_dir: Option<PathBuf>,
    },

    /// List stable machine-readable diagnostic report contracts for downstream systems
    MachineCatalog {
        #[command(flatten)]
        common: CommonArgs,
    },

    /// Report CLI/API parity across core operator workflows
    ApiParity {
        #[command(flatten)]
        common: CommonArgs,
    },

    /// Print compact expert workflow guidance with high-signal command routes
    ExpertGuide {
        #[command(flatten)]
        common: CommonArgs,
    },

    /// Browse historical run summaries from a root directory
    HistoryBrowse {
        #[command(flatten)]
        common: CommonArgs,

        /// Root directory that contains run directories
        #[arg(long, value_name = "DIR", default_value = "runs")]
        root_dir: PathBuf,

        /// Maximum number of runs to return
        #[arg(long, default_value_t = 20)]
        limit: usize,
    },

    /// Explain optimal command route for a debugging objective
    RouteExplain {
        #[command(flatten)]
        common: CommonArgs,

        /// Debugging objective topic
        #[arg(long, value_enum)]
        topic: RouteTopic,
    },

    /// Operator-focused workflow packs tuned for run, triage, and comparison loops
    OperatorWorkflow {
        #[command(flatten)]
        common: CommonArgs,

        /// Workflow profile to emit
        #[arg(long, value_enum)]
        profile: WorkflowProfile,
    },

    /// Evaluate operator ergonomics while preserving evidence and gate rigor
    OperatorErgonomics {
        #[command(flatten)]
        common: CommonArgs,

        /// Run directory to evaluate
        #[arg(long, value_name = "DIR")]
        run_dir: PathBuf,
    },

    /// Emit audit trail for overrides, replay controls, and high-risk policy exceptions
    AuditTrail {
        #[command(flatten)]
        common: CommonArgs,

        /// Run directory to evaluate
        #[arg(long, value_name = "DIR")]
        run_dir: PathBuf,
    },

    /// Trace external tools, correction inputs, and environment dependencies for a run
    DependencyTrace {
        #[command(flatten)]
        common: CommonArgs,

        /// Run directory to evaluate
        #[arg(long, value_name = "DIR")]
        run_dir: PathBuf,
    },

    /// Classify run trust level for downstream use
    TrustClass {
        #[command(flatten)]
        common: CommonArgs,

        /// Run directory to evaluate
        #[arg(long, value_name = "DIR")]
        run_dir: PathBuf,
    },

    /// Emphasize concrete engineering/scientific trust checks over ceremonial signals
    IntegrityFocus {
        #[command(flatten)]
        common: CommonArgs,

        /// Run directory to evaluate
        #[arg(long, value_name = "DIR")]
        run_dir: PathBuf,
    },
}

#[derive(ValueEnum, Clone, Copy, Debug)]
pub(crate) enum AdvancedGateMode {
    Rtk,
    Ppp,
}

#[derive(ValueEnum, Clone, Copy, Debug)]
pub(crate) enum RouteTopic {
    Integrity,
    Replay,
    Compare,
    Export,
}

#[derive(ValueEnum, Clone, Copy, Debug)]
pub(crate) enum WorkflowProfile {
    Run,
    Triage,
    Compare,
}
