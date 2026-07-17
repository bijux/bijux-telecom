use crate::api::{DiagnosticEvent, DiagnosticSeverity, ReceiverSampleTrace, Seconds};

pub(super) fn validate_receiver_sample_trace(
    trace: ReceiverSampleTrace,
    context: &str,
    expected_receiver_time: Option<Seconds>,
    expected_sample_index: Option<u64>,
) -> Vec<DiagnosticEvent> {
    let mut events = Vec::new();
    if let Err(reason) = trace.validate() {
        events.push(DiagnosticEvent::new(
            DiagnosticSeverity::Error,
            "GNSS_RECEIVER_SAMPLE_TRACE_INVALID",
            format!("{context} receiver sample trace invalid: {reason}"),
        ));
    }
    if let Some(sample_index) = expected_sample_index {
        if trace.sample_index != sample_index {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "GNSS_RECEIVER_SAMPLE_TRACE_SAMPLE_MISMATCH",
                format!("{context} receiver sample trace sample index does not match"),
            ));
        }
    }
    if let Some(receiver_time) = expected_receiver_time {
        let tolerance_s = if trace.sample_rate_hz > 0.0 {
            (0.5 / trace.sample_rate_hz).max(1.0e-12)
        } else {
            1.0e-12
        };
        if (trace.receiver_time_s.0 - receiver_time.0).abs() > tolerance_s {
            events.push(DiagnosticEvent::new(
                DiagnosticSeverity::Error,
                "GNSS_RECEIVER_SAMPLE_TRACE_TIME_MISMATCH",
                format!("{context} receiver sample trace time does not match"),
            ));
        }
    }
    events
}
