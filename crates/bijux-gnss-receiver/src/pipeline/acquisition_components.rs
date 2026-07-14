use bijux_gnss_core::api::{
    AcqComponentCombinationMode, GlonassFrequencyChannel, SatId, SignalBand, SignalCode,
    SignalComponentRole, SignalComponentSpec,
};
use bijux_gnss_signal::api::{
    galileo_e5a_q_secondary_code, galileo_e5b_q_secondary_code, gps_l5_q_neumann_hoffman_code,
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
    data_symbol_code_periods: Option<usize>,
    secondary_code_period_signs: Option<Vec<i8>>,
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

    pub fn data_symbol_code_periods(&self) -> Option<usize> {
        self.data_symbol_code_periods
    }

    pub fn secondary_code_period_signs(&self) -> Option<&[i8]> {
        self.secondary_code_period_signs.as_deref()
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
    let default_component_spec =
        registry_entry.component(default_role).ok_or(SignalError::UnsupportedSignalDefinition {
            constellation: sat.constellation,
            signal_band,
            signal_code,
        })?;
    let default_component = AcquisitionComponentPlan {
        role: default_role,
        data_symbol_code_periods: component_data_symbol_code_periods(default_component_spec),
        secondary_code_period_signs: None,
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
                data_symbol_code_periods: None,
                secondary_code_period_signs: Some(
                    galileo_e5a_q_secondary_code(sat.prn)?.into_iter().collect(),
                ),
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
                data_symbol_code_periods: None,
                secondary_code_period_signs: Some(
                    galileo_e5b_q_secondary_code(sat.prn)?.into_iter().collect(),
                ),
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

    if sat.constellation == bijux_gnss_core::api::Constellation::Gps
        && signal_band == SignalBand::L5
        && signal_code == SignalCode::L5Q
    {
        strategies[0].components[0].secondary_code_period_signs =
            Some(gps_l5_q_neumann_hoffman_code().as_slice().to_vec());
    }

    Ok(strategies)
}

fn component_data_symbol_code_periods(component: &SignalComponentSpec) -> Option<usize> {
    if component.role != SignalComponentRole::Data {
        return None;
    }
    let symbol_period_s = component.symbol_period_s?;
    if !symbol_period_s.is_finite() || symbol_period_s <= 0.0 {
        return None;
    }
    let code_period_s = component.primary_code_period_s;
    if !code_period_s.is_finite() || code_period_s <= 0.0 {
        return None;
    }
    let ratio = symbol_period_s / code_period_s;
    let rounded = ratio.round();
    if !rounded.is_finite() || rounded < 1.0 || (ratio - rounded).abs() > 1.0e-9 {
        return None;
    }
    Some(rounded as usize)
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
        assert_eq!(strategies[0].components[0].data_symbol_code_periods(), Some(20));
        assert_eq!(
            strategies[1].components.iter().map(|component| component.role).collect::<Vec<_>>(),
            vec![SignalComponentRole::Pilot]
        );
        assert_eq!(strategies[1].components[0].data_symbol_code_periods(), None);
        assert_eq!(
            strategies[1].components[0]
                .secondary_code_period_signs()
                .expect("Galileo E5a pilot secondary code")
                .len(),
            100
        );
        assert_eq!(
            strategies[2].components.iter().map(|component| component.role).collect::<Vec<_>>(),
            vec![SignalComponentRole::Data, SignalComponentRole::Pilot]
        );
        assert_eq!(strategies[2].components[0].data_symbol_code_periods(), Some(20));
        assert_eq!(strategies[2].components[1].data_symbol_code_periods(), None);
        assert_eq!(
            strategies[2].components[1]
                .secondary_code_period_signs()
                .expect("Galileo E5a pilot secondary code")
                .len(),
            100
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
        assert_eq!(strategies[0].components[0].data_symbol_code_periods(), None);
        assert_eq!(
            strategies[0].components[0]
                .secondary_code_period_signs()
                .expect("GPS L5Q pilot secondary code")
                .len(),
            20
        );
    }

    #[test]
    fn gps_l1_data_component_records_twenty_primary_periods_per_symbol() {
        let sat = SatId { constellation: Constellation::Gps, prn: 3 };

        let strategies =
            acquisition_strategies_for_signal(sat, SignalBand::L1, SignalCode::Ca, None, 20)
                .expect("GPS L1 acquisition strategies");

        assert_eq!(strategies.len(), 1);
        assert_eq!(strategies[0].components[0].data_symbol_code_periods(), Some(20));
    }

    #[test]
    fn galileo_e5b_data_component_records_four_primary_periods_per_symbol() {
        let sat = SatId { constellation: Constellation::Galileo, prn: 11 };

        let strategies =
            acquisition_strategies_for_signal(sat, SignalBand::E5, SignalCode::E5b, None, 20)
                .expect("Galileo E5b acquisition strategies");

        assert_eq!(strategies[0].components[0].data_symbol_code_periods(), Some(4));
    }
}
