#![allow(missing_docs)]

use crate::pipeline::signal_capabilities::signal_support_row;
use bijux_gnss_core::api::SupportMatrix;
use bijux_gnss_signal::api::registered_signal_registry_entries;

const SIGNAL_SUPPORT_MATRIX_SCHEMA_VERSION: u32 = 2;

pub fn build_support_matrix() -> SupportMatrix {
    let mut rows =
        registered_signal_registry_entries().iter().map(signal_support_row).collect::<Vec<_>>();

    rows.sort_by_key(|row| {
        (format!("{:?}", row.constellation), format!("{:?}", row.band), format!("{:?}", row.code))
    });
    SupportMatrix { schema_version: SIGNAL_SUPPORT_MATRIX_SCHEMA_VERSION, rows }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pipeline::signal_capabilities::{signal_execution_support, signal_support_row};
    use bijux_gnss_core::api::{
        Constellation, SignalBand, SignalCode, SignalStageSupport, SupportStatus,
    };

    #[test]
    fn gps_l1_support_row_reports_full_execution_path() {
        let matrix = build_support_matrix();
        let row = matrix
            .rows
            .iter()
            .find(|row| {
                row.constellation == Constellation::Gps
                    && row.band == SignalBand::L1
                    && row.code == SignalCode::Ca
            })
            .expect("GPS L1 C/A support row");

        assert_eq!(row.status, SupportStatus::Supported);
        assert_eq!(
            row.stage_support,
            SignalStageSupport {
                acquisition: SupportStatus::Supported,
                tracking: SupportStatus::Supported,
                data_decoding: SupportStatus::Supported,
                observations: SupportStatus::Supported,
                positioning: SupportStatus::Supported,
            }
        );
        assert!(row.requirements.is_empty());
    }

    #[test]
    fn galileo_e5_support_row_reports_executable_tracking_capability() {
        let matrix = build_support_matrix();
        let row = matrix
            .rows
            .iter()
            .find(|row| {
                row.constellation == Constellation::Galileo
                    && row.band == SignalBand::E5
                    && row.code == SignalCode::E5a
            })
            .expect("Galileo E5a support row");

        assert_eq!(row.status, SupportStatus::Planned);
        assert_eq!(row.stage_support.acquisition, SupportStatus::Supported);
        assert_eq!(row.stage_support.tracking, SupportStatus::Supported);
        assert_eq!(row.stage_support.data_decoding, SupportStatus::Planned);
        assert_eq!(row.stage_support.observations, SupportStatus::Supported);
        assert_eq!(row.stage_support.positioning, SupportStatus::Planned);
        assert!(row.requirements.is_empty());
    }

    #[test]
    fn glonass_l1_support_row_reports_channel_gated_tracking_capability() {
        let matrix = build_support_matrix();
        let row = matrix
            .rows
            .iter()
            .find(|row| {
                row.constellation == Constellation::Glonass
                    && row.band == SignalBand::L1
                    && row.code == SignalCode::Unknown
            })
            .expect("GLONASS L1 support row");

        assert_eq!(row.status, SupportStatus::Planned);
        assert_eq!(row.stage_support.acquisition, SupportStatus::Supported);
        assert_eq!(row.stage_support.tracking, SupportStatus::Supported);
        assert_eq!(row.stage_support.data_decoding, SupportStatus::Planned);
        assert_eq!(row.stage_support.observations, SupportStatus::Supported);
        assert_eq!(row.stage_support.positioning, SupportStatus::Supported);
        assert_eq!(row.requirements, vec!["glonass_frequency_channel_available".to_string()]);
    }

    #[test]
    fn support_matrix_rows_follow_signal_execution_support() {
        let matrix = build_support_matrix();

        for row in &matrix.rows {
            let execution = signal_execution_support(row.constellation, row.band, row.code);
            assert_eq!(
                row.stage_support.acquisition,
                if execution.acquisition {
                    SupportStatus::Supported
                } else {
                    SupportStatus::Planned
                },
                "{row:?}"
            );
            assert_eq!(
                row.stage_support.tracking,
                if execution.tracking { SupportStatus::Supported } else { SupportStatus::Planned },
                "{row:?}"
            );
            assert_eq!(
                row.stage_support.data_decoding,
                if execution.data_decoding {
                    SupportStatus::Supported
                } else {
                    SupportStatus::Planned
                },
                "{row:?}"
            );
            assert_eq!(
                row.stage_support.observations,
                if execution.observations {
                    SupportStatus::Supported
                } else {
                    SupportStatus::Planned
                },
                "{row:?}"
            );
            assert_eq!(
                row.stage_support.positioning,
                if execution.positioning {
                    SupportStatus::Supported
                } else {
                    SupportStatus::Planned
                },
                "{row:?}"
            );
        }
    }

    #[test]
    fn support_matrix_contains_every_registered_signal_row_once() {
        let matrix = build_support_matrix();
        let registered = registered_signal_registry_entries();

        assert_eq!(matrix.rows.len(), registered.len());
        for entry in registered {
            let matches = matrix
                .rows
                .iter()
                .filter(|row| {
                    row.constellation == entry.spec.constellation
                        && row.band == entry.spec.band
                        && row.code == entry.spec.code
                })
                .count();
            assert_eq!(matches, 1, "{entry:?}");
        }
    }

    #[test]
    fn support_matrix_rows_match_capability_row_derivation() {
        let matrix = build_support_matrix();

        for entry in registered_signal_registry_entries() {
            let expected = signal_support_row(&entry);
            let row = matrix
                .rows
                .iter()
                .find(|row| {
                    row.constellation == entry.spec.constellation
                        && row.band == entry.spec.band
                        && row.code == entry.spec.code
                })
                .expect("support matrix row");
            assert_eq!(row.stage_support, expected.stage_support, "{row:?}");
            assert_eq!(row.requirements, expected.requirements, "{row:?}");
            assert_eq!(row.status, expected.status, "{row:?}");
            assert_eq!(row.reason, expected.reason, "{row:?}");
        }
    }

    #[test]
    fn registered_nonexecuted_signals_remain_planned_without_supported_stages() {
        let matrix = build_support_matrix();

        for (constellation, band, code) in [
            (Constellation::Gps, SignalBand::L2, SignalCode::Py),
            (Constellation::Galileo, SignalBand::E1, SignalCode::E1C),
        ] {
            let row = matrix
                .rows
                .iter()
                .find(|row| {
                    row.constellation == constellation && row.band == band && row.code == code
                })
                .expect("registered support row");
            assert_eq!(row.status, SupportStatus::Planned, "{row:?}");
            assert_eq!(
                row.stage_support,
                SignalStageSupport {
                    acquisition: SupportStatus::Planned,
                    tracking: SupportStatus::Planned,
                    data_decoding: SupportStatus::Planned,
                    observations: SupportStatus::Planned,
                    positioning: SupportStatus::Planned,
                },
                "{row:?}"
            );
        }
    }
}
