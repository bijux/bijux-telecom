#![allow(missing_docs)]
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

mod codes;

/// Severity of a diagnostic event.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum DiagnosticSeverity {
    /// Informational event.
    Info,
    /// Warning-level event.
    Warning,
    /// Error-level event.
    Error,
}

/// Structured diagnostic event emitted by pipeline stages.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DiagnosticEvent {
    /// Severity of the event.
    pub severity: DiagnosticSeverity,
    /// Stable machine-readable code.
    pub code: String,
    /// Human-readable message.
    pub message: String,
    /// Optional structured context.
    pub context: Vec<(String, String)>,
}

impl DiagnosticEvent {
    /// Create a new diagnostic event.
    pub fn new(
        severity: DiagnosticSeverity,
        code: impl Into<String>,
        message: impl Into<String>,
    ) -> Self {
        Self {
            severity,
            code: code.into(),
            message: message.into(),
            context: Vec::new(),
        }
    }

    /// Attach a context key/value pair.
    pub fn with_context(mut self, key: impl Into<String>, value: impl Into<String>) -> Self {
        self.context.push((key.into(), value.into()));
        self
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DiagnosticSummaryEntry {
    pub code: String,
    pub severity: DiagnosticSeverity,
    pub count: usize,
    pub first_epoch: Option<u64>,
    pub last_epoch: Option<u64>,
    pub stages: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DiagnosticSummary {
    pub total: usize,
    pub entries: Vec<DiagnosticSummaryEntry>,
}

#[derive(Debug, Clone, Copy)]
pub struct DiagnosticCode {
    pub code: &'static str,
    pub severity: DiagnosticSeverity,
    pub meaning: &'static str,
    pub mitigation: &'static str,
}

pub const DIAGNOSTIC_CODES: &[DiagnosticCode] = codes::DIAGNOSTIC_CODES;

pub fn lookup_diagnostic(code: &str) -> Option<&'static DiagnosticCode> {
    DIAGNOSTIC_CODES.iter().find(|entry| entry.code == code)
}

#[derive(Debug, Clone)]
struct AggregateEntry {
    severity: DiagnosticSeverity,
    count: usize,
    first_epoch: Option<u64>,
    last_epoch: Option<u64>,
    stages: BTreeSet<String>,
}

pub fn aggregate_diagnostics(events: &[DiagnosticEvent]) -> DiagnosticSummary {
    use std::collections::BTreeMap;

    let mut map: BTreeMap<String, AggregateEntry> = BTreeMap::new();
    for event in events {
        let entry = map.entry(event.code.clone()).or_insert(AggregateEntry {
            severity: event.severity,
            count: 0,
            first_epoch: None,
            last_epoch: None,
            stages: BTreeSet::new(),
        });
        entry.severity = event.severity;
        entry.count += 1;
        for (key, value) in &event.context {
            if key == "epoch" {
                if let Ok(epoch) = value.parse::<u64>() {
                    entry.first_epoch =
                        Some(entry.first_epoch.map(|v| v.min(epoch)).unwrap_or(epoch));
                    entry.last_epoch =
                        Some(entry.last_epoch.map(|v| v.max(epoch)).unwrap_or(epoch));
                }
            }
            if key == "stage" {
                entry.stages.insert(value.clone());
            }
        }
    }
    let entries = map
        .into_iter()
        .map(|(code, entry)| DiagnosticSummaryEntry {
            code,
            severity: entry.severity,
            count: entry.count,
            first_epoch: entry.first_epoch,
            last_epoch: entry.last_epoch,
            stages: entry.stages.into_iter().collect(),
        })
        .collect();
    DiagnosticSummary {
        total: events.len(),
        entries,
    }
}
