use bijux_gnss_core::api::{
    AcqComponentCombinationMode, AcqComponentProvenance, AcqComponentStatistic, AcqResult,
};

use crate::pipeline::acquisition_components::{AcquisitionComponentPlan, AcquisitionStrategyPlan};
use crate::pipeline::acquisition_symbol_hypotheses::coherent_data_sign_hypotheses;

use super::correlation_accumulation::ComponentCorrelationAccumulation;
use super::correlation_metrics;

pub(super) fn unique_strategy_components(
    strategies: &[AcquisitionStrategyPlan],
) -> Vec<AcquisitionComponentPlan> {
    let mut unique = Vec::new();
    for strategy in strategies {
        for component in &strategy.components {
            if unique
                .iter()
                .all(|existing: &AcquisitionComponentPlan| !existing.matches_component(component))
            {
                unique.push(component.clone());
            }
        }
    }
    unique
}

pub(super) fn strategy_component_indexes(
    strategy: &AcquisitionStrategyPlan,
    components: &[AcquisitionComponentPlan],
) -> Vec<usize> {
    strategy
        .components
        .iter()
        .map(|component| {
            components
                .iter()
                .position(|existing| existing.matches_component(component))
                .expect("strategy components must exist in the deduplicated strategy inventory")
        })
        .collect()
}

pub(super) fn strategy_component_provenance(
    strategy: &AcquisitionStrategyPlan,
    component_indexes: &[usize],
    component_accumulations: &[ComponentCorrelationAccumulation],
) -> AcqComponentProvenance {
    AcqComponentProvenance {
        combination_mode: strategy.combination_mode,
        components: strategy
            .components
            .iter()
            .zip(component_indexes.iter().copied())
            .map(|(component, component_index)| {
                let metrics = correlation_metrics(
                    &component_accumulations[component_index].noncoherent_accumulator,
                );
                AcqComponentStatistic {
                    role: component.role,
                    peak: metrics.peak,
                    second_peak: metrics.second,
                    mean: metrics.mean,
                    peak_mean_ratio: metrics.peak / (metrics.mean + 1e-6),
                    peak_second_ratio: metrics.peak / (metrics.second + 1e-6),
                    secondary_code_phase_periods: component_accumulations[component_index]
                        .secondary_code_phase_periods,
                }
            })
            .collect(),
    }
}

pub(super) fn strategy_supports_search_model_refinement(
    strategy: &AcquisitionStrategyPlan,
) -> bool {
    strategy.combination_mode == AcqComponentCombinationMode::SingleComponent
        && strategy.components.len() == 1
}

pub(super) fn strategy_uses_data_sign_hypotheses(
    strategy: &AcquisitionStrategyPlan,
    coherent_periods: u32,
) -> bool {
    strategy
        .components
        .iter()
        .any(|component| component_data_sign_hypotheses(component, coherent_periods).is_some())
}

fn strategy_matches_component_provenance(
    strategy: &AcquisitionStrategyPlan,
    provenance: &AcqComponentProvenance,
) -> bool {
    strategy.combination_mode == provenance.combination_mode
        && strategy.components.len() == provenance.components.len()
        && strategy
            .components
            .iter()
            .map(|component| component.role)
            .eq(provenance.components.iter().map(|component| component.role))
}

pub(super) fn candidate_uses_data_sign_hypotheses(
    candidate: &AcqResult,
    strategies: &[AcquisitionStrategyPlan],
    coherent_periods: u32,
) -> bool {
    let Some(provenance) = candidate.component_provenance() else {
        return false;
    };
    strategies.iter().any(|strategy| {
        strategy_matches_component_provenance(strategy, provenance)
            && strategy_uses_data_sign_hypotheses(strategy, coherent_periods)
    })
}

pub(super) fn component_data_sign_hypotheses(
    component: &AcquisitionComponentPlan,
    coherent_periods: u32,
) -> Option<Vec<Vec<i8>>> {
    if coherent_periods <= 1 {
        return None;
    }
    Some(coherent_data_sign_hypotheses(
        coherent_periods as usize,
        component.data_symbol_code_periods()?,
    ))
}
