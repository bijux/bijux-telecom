pub(crate) fn runtime_config_from_env(
    common: &CommonArgs,
    run_dir: Option<PathBuf>,
) -> bijux_gnss_infra::api::receiver::ReceiverRuntime {
    runtime_config_from_capture_start(common, run_dir, None)
}

pub(crate) fn runtime_config_from_capture_start(
    common: &CommonArgs,
    run_dir: Option<PathBuf>,
    capture_start_utc: Option<&str>,
) -> bijux_gnss_infra::api::receiver::ReceiverRuntime {
    if common.deterministic {
        std::env::set_var("RAYON_NUM_THREADS", "1");
        std::env::set_var("BIJUX_DETERMINISTIC", "1");
    }
    let config = bijux_gnss_infra::api::receiver::ReceiverRuntimeConfig {
        run_id: std::env::var("BIJUX_RUN_ID").ok(),
        trace_dir: common.dump.clone(),
        run_dir: run_dir.or_else(|| std::env::var("BIJUX_RUN_DIR").ok().map(PathBuf::from)),
        diagnostics_dump: std::env::var("BIJUX_DIAGNOSTICS_DUMP").ok().as_deref() == Some("1"),
        capture_start_gps_time: capture_start_utc.and_then(capture_start_gps_time),
    };
    bijux_gnss_infra::api::receiver::ReceiverRuntime::new(config)
}

pub(crate) fn capture_start_gps_time(
    capture_start_utc: &str,
) -> Option<bijux_gnss_infra::api::core::GpsTime> {
    let utc = time::OffsetDateTime::parse(
        capture_start_utc,
        &time::format_description::well_known::Rfc3339,
    )
    .ok()?;
    Some(bijux_gnss_infra::api::nav::gps_time_from_utc(
        bijux_gnss_infra::api::core::UtcTime {
            unix_s: utc.unix_timestamp_nanos() as f64 / 1_000_000_000.0,
        },
    ))
}

#[cfg(test)]
mod tests {
    use super::{capture_start_gps_time, runtime_config_from_capture_start};
    use crate::{CommonArgs, ReportFormat};

    fn common_args() -> CommonArgs {
        CommonArgs {
            config: None,
            dataset: None,
            unregistered_dataset: true,
            out: None,
            report: ReportFormat::Table,
            seed: None,
            deterministic: false,
            dump: None,
            sidecar: None,
            resume: None,
        }
    }

    #[test]
    fn capture_start_gps_time_parses_rfc3339_timestamp() {
        let gps_time = capture_start_gps_time("2022-03-27T11:32:04.2147593125Z").expect("gps time");
        assert_eq!(gps_time.week, 2203);
        assert!((gps_time.tow_s - 41_542.2147593125).abs() <= 1.0e-6);
    }

    #[test]
    fn runtime_config_from_capture_start_sets_runtime_anchor() {
        let runtime =
            runtime_config_from_capture_start(&common_args(), None, Some("2026-07-09T00:00:00Z"));

        let gps_time = runtime.config.capture_start_gps_time.expect("runtime gps anchor");
        assert_eq!(gps_time.week, 2426);
        assert!((gps_time.tow_s - 345_618.0).abs() <= 1.0e-9);
    }
}
