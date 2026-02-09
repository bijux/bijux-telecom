//! Effect boundary types and runtime configuration.
#![allow(missing_docs)]

use std::path::PathBuf;
use std::sync::Arc;

use bijux_gnss_core::api::DiagnosticEvent;

#[derive(Debug, Clone, Default)]
pub struct ReceiverRuntimeConfig {
    pub run_id: Option<String>,
    pub trace_dir: Option<PathBuf>,
    pub run_dir: Option<PathBuf>,
    pub diagnostics_dump: bool,
}

pub trait Logger: Send + Sync {
    fn event(&self, e: &DiagnosticEvent);
}

pub trait TraceSink: Send + Sync {
    fn record(&self, t: TraceRecord);
}

pub trait MetricsSink: Send + Sync {
    fn metric(&self, m: Metric);
}

#[derive(Clone)]
pub struct ReceiverRuntime {
    pub config: ReceiverRuntimeConfig,
    pub logger: Arc<dyn Logger>,
    pub trace: Arc<dyn TraceSink>,
    pub metrics: Arc<dyn MetricsSink>,
}

impl ReceiverRuntime {
    pub fn new(config: ReceiverRuntimeConfig) -> Self {
        Self {
            config,
            logger: Arc::new(NullLogger),
            trace: Arc::new(NullTrace),
            metrics: Arc::new(NullMetrics),
        }
    }

    pub fn with_sinks(
        config: ReceiverRuntimeConfig,
        logger: Arc<dyn Logger>,
        trace: Arc<dyn TraceSink>,
        metrics: Arc<dyn MetricsSink>,
    ) -> Self {
        Self {
            config,
            logger,
            trace,
            metrics,
        }
    }
}

impl Default for ReceiverRuntime {
    fn default() -> Self {
        Self::new(ReceiverRuntimeConfig::default())
    }
}

#[derive(Debug, Clone)]
pub struct TraceRecord {
    pub name: &'static str,
    pub fields: Vec<(&'static str, String)>,
}

#[derive(Debug, Clone)]
pub struct Metric {
    pub name: &'static str,
    pub value: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct NullLogger;

impl Logger for NullLogger {
    fn event(&self, _e: &DiagnosticEvent) {}
}

#[derive(Debug, Clone, Copy)]
pub struct NullTrace;

impl TraceSink for NullTrace {
    fn record(&self, _t: TraceRecord) {}
}

#[derive(Debug, Clone, Copy)]
pub struct NullMetrics;

impl MetricsSink for NullMetrics {
    fn metric(&self, _m: Metric) {}
}
