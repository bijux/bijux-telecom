#![allow(missing_docs)]

use bijux_gnss_core::api::{
    signal_registry, Constellation, SignalBand, SignalCode, SignalStageSupport, SignalSupportRow,
    SupportMatrix, SupportStatus,
};

const SIGNAL_SUPPORT_MATRIX_SCHEMA_VERSION: u32 = 2;

pub fn build_support_matrix() -> SupportMatrix {
    let constellations =
        [Constellation::Gps, Constellation::Galileo, Constellation::Glonass, Constellation::Beidou];
    let bands = [
        SignalBand::L1,
        SignalBand::L2,
        SignalBand::L5,
        SignalBand::E1,
        SignalBand::E5,
        SignalBand::B1,
        SignalBand::B2,
    ];
    let codes = [
        SignalCode::Ca,
        SignalCode::L2C,
        SignalCode::Py,
        SignalCode::B1I,
        SignalCode::B2I,
        SignalCode::E1B,
        SignalCode::E1C,
        SignalCode::E5a,
        SignalCode::E5b,
        SignalCode::Unknown,
    ];

    let mut rows = Vec::new();
    for constellation in constellations {
        for band in bands {
            for code in codes {
                let registered = signal_registry(constellation, band, code).is_some();
                let stage_support =
                    derive_signal_stage_support(constellation, band, code, registered);
                let requirements = derive_signal_requirements(constellation, band, code);
                let status =
                    aggregate_signal_status(stage_support, registered, !requirements.is_empty());
                if matches!(status, SupportStatus::Unsupported) && !registered {
                    continue;
                }
                rows.push(SignalSupportRow {
                    constellation,
                    band,
                    code,
                    stage_support,
                    requirements: requirements.clone(),
                    status,
                    reason: summarize_signal_support(stage_support, &requirements),
                });
            }
        }
    }

    rows.sort_by_key(|row| {
        (format!("{:?}", row.constellation), format!("{:?}", row.band), format!("{:?}", row.code))
    });
    SupportMatrix { schema_version: SIGNAL_SUPPORT_MATRIX_SCHEMA_VERSION, rows }
}

fn derive_signal_stage_support(
    constellation: Constellation,
    band: SignalBand,
    code: SignalCode,
    registered: bool,
) -> SignalStageSupport {
    SignalStageSupport {
        acquisition: stage_status(
            acquisition_supports_signal(constellation, band, code),
            registered,
        ),
        tracking: stage_status(tracking_supports_signal(constellation, band, code), registered),
        data_decoding: stage_status(
            data_decoding_supports_signal(constellation, band, code),
            registered,
        ),
        observations: stage_status(
            observation_supports_signal(constellation, band, code),
            registered,
        ),
        positioning: stage_status(
            positioning_supports_signal(constellation, band, code),
            registered,
        ),
    }
}

fn stage_status(executable: bool, registered: bool) -> SupportStatus {
    if executable {
        SupportStatus::Supported
    } else if registered {
        SupportStatus::Planned
    } else {
        SupportStatus::Unsupported
    }
}

fn aggregate_signal_status(
    stage_support: SignalStageSupport,
    registered: bool,
    has_requirements: bool,
) -> SupportStatus {
    let stages = [
        stage_support.acquisition,
        stage_support.tracking,
        stage_support.data_decoding,
        stage_support.observations,
        stage_support.positioning,
    ];
    let supported_count =
        stages.iter().filter(|status| **status == SupportStatus::Supported).count();
    let planned_count = stages.iter().filter(|status| **status == SupportStatus::Planned).count();

    if supported_count == stages.len() && !has_requirements {
        SupportStatus::Supported
    } else if supported_count > 0 || planned_count > 0 || registered {
        SupportStatus::Planned
    } else {
        SupportStatus::Unsupported
    }
}

fn summarize_signal_support(stage_support: SignalStageSupport, requirements: &[String]) -> String {
    let supported = stage_names_with_status(stage_support, SupportStatus::Supported);
    let planned = stage_names_with_status(stage_support, SupportStatus::Planned);
    let deprecated = stage_names_with_status(stage_support, SupportStatus::Deprecated);

    let mut parts = Vec::new();
    if !supported.is_empty() {
        parts.push(format!("supported stages={}", supported.join(",")));
    }
    if !planned.is_empty() {
        parts.push(format!("planned stages={}", planned.join(",")));
    }
    if !deprecated.is_empty() {
        parts.push(format!("deprecated stages={}", deprecated.join(",")));
    }
    if !requirements.is_empty() {
        parts.push(format!("requirements={}", requirements.join(",")));
    }
    if parts.is_empty() {
        "no executable stages are registered".to_string()
    } else {
        parts.join("; ")
    }
}

fn stage_names_with_status(
    stage_support: SignalStageSupport,
    target: SupportStatus,
) -> Vec<&'static str> {
    let mut names = Vec::new();
    if stage_support.acquisition == target {
        names.push("acquisition");
    }
    if stage_support.tracking == target {
        names.push("tracking");
    }
    if stage_support.data_decoding == target {
        names.push("data_decoding");
    }
    if stage_support.observations == target {
        names.push("observations");
    }
    if stage_support.positioning == target {
        names.push("positioning");
    }
    names
}

fn acquisition_supports_signal(
    constellation: Constellation,
    band: SignalBand,
    code: SignalCode,
) -> bool {
    matches!(
        (constellation, band, code),
        (Constellation::Gps, SignalBand::L1, SignalCode::Ca)
            | (Constellation::Galileo, SignalBand::E1, SignalCode::E1B)
            | (Constellation::Beidou, SignalBand::B1, SignalCode::B1I)
            | (Constellation::Glonass, SignalBand::L1, SignalCode::Unknown)
    )
}

fn tracking_supports_signal(
    constellation: Constellation,
    band: SignalBand,
    code: SignalCode,
) -> bool {
    acquisition_supports_signal(constellation, band, code)
}

fn data_decoding_supports_signal(
    constellation: Constellation,
    band: SignalBand,
    code: SignalCode,
) -> bool {
    matches!((constellation, band, code), (Constellation::Gps, SignalBand::L1, SignalCode::Ca))
}

fn observation_supports_signal(
    constellation: Constellation,
    band: SignalBand,
    code: SignalCode,
) -> bool {
    matches!(
        (constellation, band, code),
        (Constellation::Gps, SignalBand::L1, SignalCode::Ca)
            | (Constellation::Gps, SignalBand::L2, SignalCode::L2C)
            | (Constellation::Gps, SignalBand::L5, SignalCode::Unknown)
            | (Constellation::Galileo, SignalBand::E1, SignalCode::E1B)
            | (Constellation::Galileo, SignalBand::E5, SignalCode::E5a)
            | (Constellation::Beidou, SignalBand::B1, SignalCode::B1I)
            | (Constellation::Beidou, SignalBand::B2, SignalCode::B2I)
            | (Constellation::Glonass, SignalBand::L1, SignalCode::Unknown)
    )
}

fn positioning_supports_signal(
    constellation: Constellation,
    band: SignalBand,
    code: SignalCode,
) -> bool {
    matches!(
        (constellation, band, code),
        (Constellation::Gps, SignalBand::L1, SignalCode::Ca)
            | (Constellation::Galileo, SignalBand::E1, SignalCode::E1B)
            | (Constellation::Beidou, SignalBand::B1, SignalCode::B1I)
            | (Constellation::Glonass, SignalBand::L1, SignalCode::Unknown)
    )
}

fn derive_signal_requirements(
    constellation: Constellation,
    band: SignalBand,
    code: SignalCode,
) -> Vec<String> {
    let mut requirements = Vec::new();
    if matches!(
        (constellation, band, code),
        (Constellation::Glonass, SignalBand::L1, SignalCode::Unknown)
    ) {
        requirements.push("glonass_frequency_channel_available".to_string());
    }
    if matches!(
        (constellation, band, code),
        (Constellation::Gps, SignalBand::L2, SignalCode::L2C)
            | (Constellation::Gps, SignalBand::L5, SignalCode::Unknown)
            | (Constellation::Galileo, SignalBand::E5, SignalCode::E5a)
            | (Constellation::Beidou, SignalBand::B2, SignalCode::B2I)
    ) {
        requirements.push("tracked_epoch_input".to_string());
    }
    requirements
}

#[cfg(test)]
mod tests {
    use super::*;

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
    fn galileo_e5_support_row_reports_observation_only_capability() {
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
        assert_eq!(row.stage_support.acquisition, SupportStatus::Planned);
        assert_eq!(row.stage_support.tracking, SupportStatus::Planned);
        assert_eq!(row.stage_support.data_decoding, SupportStatus::Planned);
        assert_eq!(row.stage_support.observations, SupportStatus::Supported);
        assert_eq!(row.stage_support.positioning, SupportStatus::Planned);
        assert!(row.requirements.iter().any(|value| value == "tracked_epoch_input"));
    }
}
