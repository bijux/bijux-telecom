use bijux_gnss_core::api::{
    AcqComponentCombinationMode, GlonassFrequencyChannel, SatId, SignalBand, SignalCode,
    SignalComponentRole,
};
use bijux_gnss_signal::api::{
    resolved_signal_registry_entry, sample_galileo_e5a_q_primary_code,
    sample_galileo_e5b_q_primary_code, AcquisitionSignalModel, SignalError,
};

#[derive(Debug, Clone)]
pub(crate) struct AcquisitionStrategyPlan {
    pub search_model: AcquisitionSignalModel,
    pub combination_mode: AcqComponentCombinationMode,
    pub components: Vec<AcquisitionComponentPlan>,
}

#[derive(Debug, Clone)]
pub(crate) struct AcquisitionComponentPlan {
    pub role: SignalComponentRole,
    local_code: AcquisitionComponentLocalCode,
}

#[derive(Debug, Clone)]
enum AcquisitionComponentLocalCode {
    SearchModel(AcquisitionSignalModel),
    GalileoE5aPilot { sat: SatId },
    GalileoE5bPilot { sat: SatId },
}

impl AcquisitionComponentPlan {
    pub fn sample_local_code_period(
        &self,
        sample_rate_hz: f64,
        sample_count: usize,
    ) -> Result<Vec<f32>, SignalError> {
        match &self.local_code {
            AcquisitionComponentLocalCode::SearchModel(model) => {
                model.sampled_local_code_period(sample_rate_hz, sample_count)
            }
            AcquisitionComponentLocalCode::GalileoE5aPilot { sat } => {
                sample_galileo_e5a_q_primary_code(sat.prn, sample_rate_hz, 0.0, sample_count)
            }
            AcquisitionComponentLocalCode::GalileoE5bPilot { sat } => {
                sample_galileo_e5b_q_primary_code(sat.prn, sample_rate_hz, 0.0, sample_count)
            }
        }
    }

    pub fn matches_component(&self, other: &Self) -> bool {
        self.role == other.role && self.local_code.matches_identity(&other.local_code)
    }
}

impl AcquisitionComponentLocalCode {
    fn matches_identity(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::SearchModel(_), Self::SearchModel(_)) => true,
            (Self::GalileoE5aPilot { sat: left }, Self::GalileoE5aPilot { sat: right }) => {
                left == right
            }
            (Self::GalileoE5bPilot { sat: left }, Self::GalileoE5bPilot { sat: right }) => {
                left == right
            }
            _ => false,
        }
    }
}

pub(crate) fn acquisition_strategies_for_signal(
    sat: SatId,
    signal_band: SignalBand,
    signal_code: SignalCode,
    glonass_frequency_channel: Option<GlonassFrequencyChannel>,
    coherent_ms: u32,
) -> Result<Vec<AcquisitionStrategyPlan>, SignalError> {
    let registry_entry =
        resolved_signal_registry_entry(sat, signal_band, signal_code, glonass_frequency_channel)?
            .ok_or(SignalError::UnsupportedSignalDefinition {
            constellation: sat.constellation,
            signal_band,
            signal_code,
        })?;
    let search_model = AcquisitionSignalModel::for_sat_signal(
        sat,
        Some(signal_band),
        signal_code,
        glonass_frequency_channel,
    )?
    .ok_or(SignalError::UnsupportedSignalDefinition {
        constellation: sat.constellation,
        signal_band,
        signal_code,
    })?;
    let default_role = registry_entry.default_component_role;
    let default_component = AcquisitionComponentPlan {
        role: default_role,
        local_code: AcquisitionComponentLocalCode::SearchModel(search_model.clone()),
    };

    let mut strategies = vec![AcquisitionStrategyPlan {
        search_model: search_model.clone(),
        combination_mode: AcqComponentCombinationMode::SingleComponent,
        components: vec![default_component.clone()],
    }];

    match (sat.constellation, signal_band, signal_code) {
        (bijux_gnss_core::api::Constellation::Galileo, SignalBand::E5, SignalCode::E5a) => {
            let pilot_component = AcquisitionComponentPlan {
                role: SignalComponentRole::Pilot,
                local_code: AcquisitionComponentLocalCode::GalileoE5aPilot { sat },
            };
            strategies.push(AcquisitionStrategyPlan {
                search_model: search_model.clone(),
                combination_mode: AcqComponentCombinationMode::SingleComponent,
                components: vec![pilot_component.clone()],
            });
            strategies.push(AcquisitionStrategyPlan {
                search_model: search_model.clone(),
                combination_mode: AcqComponentCombinationMode::NoncoherentComponentSum,
                components: vec![default_component.clone(), pilot_component.clone()],
            });
            if coherent_ms == search_model.code_period_ms {
                strategies.push(AcquisitionStrategyPlan {
                    search_model,
                    combination_mode: AcqComponentCombinationMode::CoherentComponentSum,
                    components: vec![default_component, pilot_component],
                });
            }
        }
        (bijux_gnss_core::api::Constellation::Galileo, SignalBand::E5, SignalCode::E5b) => {
            let pilot_component = AcquisitionComponentPlan {
                role: SignalComponentRole::Pilot,
                local_code: AcquisitionComponentLocalCode::GalileoE5bPilot { sat },
            };
            strategies.push(AcquisitionStrategyPlan {
                search_model: search_model.clone(),
                combination_mode: AcqComponentCombinationMode::SingleComponent,
                components: vec![pilot_component.clone()],
            });
            strategies.push(AcquisitionStrategyPlan {
                search_model: search_model.clone(),
                combination_mode: AcqComponentCombinationMode::NoncoherentComponentSum,
                components: vec![default_component.clone(), pilot_component.clone()],
            });
            if coherent_ms == search_model.code_period_ms {
                strategies.push(AcquisitionStrategyPlan {
                    search_model,
                    combination_mode: AcqComponentCombinationMode::CoherentComponentSum,
                    components: vec![default_component, pilot_component],
                });
            }
        }
        _ => {}
    }

    Ok(strategies)
}

#[cfg(test)]
mod tests {
    use super::acquisition_strategies_for_signal;
    use bijux_gnss_core::api::{
        AcqComponentCombinationMode, Constellation, SatId, SignalBand, SignalCode,
        SignalComponentRole,
    };

    #[test]
    fn galileo_e5a_strategies_cover_component_modes_at_one_millisecond() {
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };

        let strategies =
            acquisition_strategies_for_signal(sat, SignalBand::E5, SignalCode::E5a, None, 1)
                .expect("Galileo E5a acquisition strategies");

        assert_eq!(strategies.len(), 4);
        assert_eq!(
            strategies.iter().map(|strategy| strategy.combination_mode).collect::<Vec<_>>(),
            vec![
                AcqComponentCombinationMode::SingleComponent,
                AcqComponentCombinationMode::SingleComponent,
                AcqComponentCombinationMode::NoncoherentComponentSum,
                AcqComponentCombinationMode::CoherentComponentSum,
            ]
        );
        assert_eq!(
            strategies[0].components.iter().map(|component| component.role).collect::<Vec<_>>(),
            vec![SignalComponentRole::Data]
        );
        assert_eq!(
            strategies[1].components.iter().map(|component| component.role).collect::<Vec<_>>(),
            vec![SignalComponentRole::Pilot]
        );
        assert_eq!(
            strategies[2].components.iter().map(|component| component.role).collect::<Vec<_>>(),
            vec![SignalComponentRole::Data, SignalComponentRole::Pilot]
        );
        assert_eq!(
            strategies[3].components.iter().map(|component| component.role).collect::<Vec<_>>(),
            vec![SignalComponentRole::Data, SignalComponentRole::Pilot]
        );
    }

    #[test]
    fn galileo_e5a_strategies_omit_coherent_component_sum_beyond_one_millisecond() {
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };

        let strategies =
            acquisition_strategies_for_signal(sat, SignalBand::E5, SignalCode::E5a, None, 2)
                .expect("Galileo E5a acquisition strategies");

        assert_eq!(strategies.len(), 3);
        assert!(strategies.iter().all(|strategy| strategy.combination_mode
            != AcqComponentCombinationMode::CoherentComponentSum));
    }

    #[test]
    fn gps_l5q_strategy_preserves_single_pilot_component() {
        let sat = SatId { constellation: Constellation::Gps, prn: 9 };

        let strategies =
            acquisition_strategies_for_signal(sat, SignalBand::L5, SignalCode::L5Q, None, 1)
                .expect("GPS L5Q acquisition strategies");

        assert_eq!(strategies.len(), 1);
        assert_eq!(strategies[0].combination_mode, AcqComponentCombinationMode::SingleComponent);
        assert_eq!(strategies[0].components.len(), 1);
        assert_eq!(strategies[0].components[0].role, SignalComponentRole::Pilot);
    }
}
