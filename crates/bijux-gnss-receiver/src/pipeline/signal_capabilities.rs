#![allow(missing_docs)]

use bijux_gnss_core::api::{
    Constellation, GlonassFrequencyChannel, SatId, SignalBand, SignalCode, SignalRegistryEntry,
    SignalStageSupport, SignalSupportRow, SupportStatus,
};
#[cfg(test)]
use bijux_gnss_signal::api::signal_registry;
use bijux_gnss_signal::api::AcquisitionSignalModel;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct SignalExecutionSupport {
    pub acquisition: bool,
    pub tracking: bool,
    pub data_decoding: bool,
    pub observations: bool,
    pub positioning: bool,
}

pub(crate) fn signal_execution_support(
    constellation: Constellation,
    band: SignalBand,
    code: SignalCode,
) -> SignalExecutionSupport {
    SignalExecutionSupport {
        acquisition: supports_acquisition_signal(constellation, band, code),
        tracking: supports_tracking_signal(constellation, band, code),
        data_decoding: supports_data_decoding_signal(constellation, band, code),
        observations: supports_observation_signal(constellation, band, code),
        positioning: supports_positioning_signal(constellation, band, code),
    }
}

pub(crate) fn signal_support_row(entry: &SignalRegistryEntry) -> SignalSupportRow {
    let signal = entry.spec;
    let execution = signal_execution_support(signal.constellation, signal.band, signal.code);
    let stage_support = derive_signal_stage_support(execution);
    let requirements = signal_capability_requirements(signal.constellation, stage_support);
    let status = aggregate_signal_status(stage_support, !requirements.is_empty());

    SignalSupportRow {
        constellation: signal.constellation,
        band: signal.band,
        code: signal.code,
        stage_support,
        requirements: requirements.clone(),
        status,
        reason: summarize_signal_support(stage_support, &requirements),
    }
}

pub(crate) fn supports_acquisition_signal(
    constellation: Constellation,
    band: SignalBand,
    code: SignalCode,
) -> bool {
    AcquisitionSignalModel::for_sat_signal(
        sample_sat(constellation),
        Some(band),
        code,
        sample_glonass_frequency_channel(constellation),
    )
    .ok()
    .flatten()
    .is_some()
}

pub(crate) fn supports_tracking_signal(
    constellation: Constellation,
    band: SignalBand,
    code: SignalCode,
) -> bool {
    crate::pipeline::tracking::supports_tracking_signal(sample_sat(constellation), band, code)
}

pub(crate) fn supports_data_decoding_signal(
    constellation: Constellation,
    band: SignalBand,
    code: SignalCode,
) -> bool {
    matches!((constellation, band, code), (Constellation::Gps, SignalBand::L1, SignalCode::Ca))
}

pub(crate) fn supports_observation_signal(
    constellation: Constellation,
    band: SignalBand,
    code: SignalCode,
) -> bool {
    crate::pipeline::observations::supports_observation_signal(constellation, band, code)
}

#[cfg(feature = "nav")]
pub(crate) fn supports_positioning_signal(
    constellation: Constellation,
    band: SignalBand,
    code: SignalCode,
) -> bool {
    bijux_gnss_nav::api::supports_positioning_signal(constellation, band, code)
}

#[cfg(not(feature = "nav"))]
pub(crate) fn supports_positioning_signal(
    _constellation: Constellation,
    _band: SignalBand,
    _code: SignalCode,
) -> bool {
    false
}

fn sample_sat(constellation: Constellation) -> SatId {
    SatId { constellation, prn: representative_prn(constellation) }
}

fn sample_glonass_frequency_channel(
    constellation: Constellation,
) -> Option<GlonassFrequencyChannel> {
    (constellation == Constellation::Glonass)
        .then(|| GlonassFrequencyChannel::new(0).expect("GLONASS channel 0 must be valid"))
}

fn derive_signal_stage_support(execution: SignalExecutionSupport) -> SignalStageSupport {
    SignalStageSupport {
        acquisition: stage_status(execution.acquisition),
        tracking: stage_status(execution.tracking),
        data_decoding: stage_status(execution.data_decoding),
        observations: stage_status(execution.observations),
        positioning: stage_status(execution.positioning),
    }
}

fn stage_status(executable: bool) -> SupportStatus {
    if executable {
        SupportStatus::Supported
    } else {
        SupportStatus::Planned
    }
}

fn signal_capability_requirements(
    constellation: Constellation,
    stage_support: SignalStageSupport,
) -> Vec<String> {
    let mut requirements = Vec::new();
    if stage_support.observations == SupportStatus::Supported
        && stage_support.tracking != SupportStatus::Supported
    {
        requirements.push("tracked_epoch_input".to_string());
    }
    if constellation == Constellation::Glonass
        && [
            stage_support.acquisition,
            stage_support.tracking,
            stage_support.observations,
            stage_support.positioning,
        ]
        .into_iter()
        .any(|status| status == SupportStatus::Supported)
    {
        requirements.push("glonass_frequency_channel_available".to_string());
    }
    requirements
}

fn aggregate_signal_status(
    stage_support: SignalStageSupport,
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

    if supported_count == stages.len() && !has_requirements {
        SupportStatus::Supported
    } else {
        SupportStatus::Planned
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

fn representative_prn(constellation: Constellation) -> u8 {
    match constellation {
        Constellation::Gps
        | Constellation::Galileo
        | Constellation::Glonass
        | Constellation::Beidou => 1,
        Constellation::Unknown => 0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gps_l1_reports_full_execution_support() {
        let support = signal_execution_support(Constellation::Gps, SignalBand::L1, SignalCode::Ca);

        assert_eq!(
            support,
            SignalExecutionSupport {
                acquisition: true,
                tracking: true,
                data_decoding: true,
                observations: true,
                positioning: cfg!(feature = "nav"),
            }
        );
    }

    #[test]
    fn executable_registered_signals_reflect_receiver_stage_capabilities() {
        let gps_l2c = signal_execution_support(Constellation::Gps, SignalBand::L2, SignalCode::L2C);
        assert_eq!(
            gps_l2c,
            SignalExecutionSupport {
                acquisition: false,
                tracking: true,
                data_decoding: false,
                observations: true,
                positioning: false,
            }
        );

        let galileo_e5 =
            signal_execution_support(Constellation::Galileo, SignalBand::E5, SignalCode::E5a);
        assert_eq!(
            galileo_e5,
            SignalExecutionSupport {
                acquisition: true,
                tracking: true,
                data_decoding: false,
                observations: true,
                positioning: false,
            }
        );

        let galileo_e5b =
            signal_execution_support(Constellation::Galileo, SignalBand::E5, SignalCode::E5b);
        assert_eq!(
            galileo_e5b,
            SignalExecutionSupport {
                acquisition: true,
                tracking: true,
                data_decoding: false,
                observations: true,
                positioning: false,
            }
        );
    }

    #[test]
    fn tracking_requires_supported_band_and_supported_signal_identity() {
        assert!(supports_tracking_signal(Constellation::Galileo, SignalBand::E1, SignalCode::E1B));
        assert!(supports_tracking_signal(Constellation::Gps, SignalBand::L2, SignalCode::L2C));
        assert!(!supports_tracking_signal(Constellation::Galileo, SignalBand::E1, SignalCode::E1C));
        assert!(supports_tracking_signal(Constellation::Gps, SignalBand::L5, SignalCode::L5I));
        assert!(!supports_tracking_signal(Constellation::Gps, SignalBand::L5, SignalCode::Unknown));
    }

    #[test]
    fn signal_support_row_marks_glonass_channel_dependency_as_requirement() {
        let row = signal_support_row(
            &signal_registry(Constellation::Glonass, SignalBand::L1, SignalCode::Unknown)
                .expect("GLONASS registry entry"),
        );

        assert!(row
            .requirements
            .iter()
            .any(|value| value == "glonass_frequency_channel_available"));
    }

    #[test]
    fn registered_signal_capabilities_match_receiver_stage_boundaries() {
        let cases = [
            (
                Constellation::Gps,
                SignalBand::L1,
                SignalCode::Ca,
                SignalExecutionSupport {
                    acquisition: true,
                    tracking: true,
                    data_decoding: true,
                    observations: true,
                    positioning: cfg!(feature = "nav"),
                },
            ),
            (
                Constellation::Gps,
                SignalBand::L2,
                SignalCode::L2C,
                SignalExecutionSupport {
                    acquisition: false,
                    tracking: true,
                    data_decoding: false,
                    observations: true,
                    positioning: false,
                },
            ),
            (
                Constellation::Gps,
                SignalBand::L2,
                SignalCode::Py,
                SignalExecutionSupport {
                    acquisition: false,
                    tracking: false,
                    data_decoding: false,
                    observations: false,
                    positioning: false,
                },
            ),
            (
                Constellation::Gps,
                SignalBand::L5,
                SignalCode::L5I,
                SignalExecutionSupport {
                    acquisition: true,
                    tracking: true,
                    data_decoding: false,
                    observations: true,
                    positioning: false,
                },
            ),
            (
                Constellation::Galileo,
                SignalBand::E1,
                SignalCode::E1B,
                SignalExecutionSupport {
                    acquisition: true,
                    tracking: true,
                    data_decoding: false,
                    observations: true,
                    positioning: cfg!(feature = "nav"),
                },
            ),
            (
                Constellation::Galileo,
                SignalBand::E1,
                SignalCode::E1C,
                SignalExecutionSupport {
                    acquisition: false,
                    tracking: false,
                    data_decoding: false,
                    observations: false,
                    positioning: false,
                },
            ),
            (
                Constellation::Galileo,
                SignalBand::E5,
                SignalCode::E5a,
                SignalExecutionSupport {
                    acquisition: true,
                    tracking: true,
                    data_decoding: false,
                    observations: true,
                    positioning: false,
                },
            ),
            (
                Constellation::Galileo,
                SignalBand::E5,
                SignalCode::E5b,
                SignalExecutionSupport {
                    acquisition: true,
                    tracking: true,
                    data_decoding: false,
                    observations: true,
                    positioning: false,
                },
            ),
            (
                Constellation::Beidou,
                SignalBand::B1,
                SignalCode::B1I,
                SignalExecutionSupport {
                    acquisition: true,
                    tracking: true,
                    data_decoding: false,
                    observations: true,
                    positioning: cfg!(feature = "nav"),
                },
            ),
            (
                Constellation::Beidou,
                SignalBand::B2,
                SignalCode::B2I,
                SignalExecutionSupport {
                    acquisition: false,
                    tracking: false,
                    data_decoding: false,
                    observations: true,
                    positioning: false,
                },
            ),
            (
                Constellation::Glonass,
                SignalBand::L1,
                SignalCode::Unknown,
                SignalExecutionSupport {
                    acquisition: true,
                    tracking: true,
                    data_decoding: false,
                    observations: true,
                    positioning: cfg!(feature = "nav"),
                },
            ),
        ];

        for (constellation, band, code, expected) in cases {
            assert_eq!(
                signal_execution_support(constellation, band, code),
                expected,
                "{constellation:?} {band:?} {code:?}"
            );
        }
    }
}
