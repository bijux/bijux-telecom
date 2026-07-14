use std::collections::BTreeSet;

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct CoherentSecondaryCodePhaseHypothesis {
    pub secondary_code_phase_periods: u32,
    pub period_signs: Vec<i8>,
}

pub(crate) fn coherent_data_sign_hypotheses(
    coherent_periods: usize,
    symbol_code_periods: usize,
) -> Vec<Vec<i8>> {
    if coherent_periods == 0 {
        return Vec::new();
    }
    let symbol_code_periods = symbol_code_periods.max(1);
    let mut unique = BTreeSet::new();

    for start_offset in 0..symbol_code_periods {
        let symbol_indexes = (0..coherent_periods)
            .map(|period_index| (start_offset + period_index) / symbol_code_periods)
            .collect::<Vec<_>>();
        let symbol_count = symbol_indexes.last().copied().unwrap_or(0) + 1;
        let sign_variants = 1usize << symbol_count.saturating_sub(1);

        for variant in 0..sign_variants {
            let mut symbol_signs = vec![1_i8; symbol_count];
            for symbol_index in 1..symbol_count {
                if ((variant >> (symbol_index - 1)) & 1) == 1 {
                    symbol_signs[symbol_index] = -1;
                }
            }
            unique.insert(
                symbol_indexes
                    .iter()
                    .map(|&symbol_index| symbol_signs[symbol_index])
                    .collect::<Vec<_>>(),
            );
        }
    }

    if unique.is_empty() {
        unique.insert(vec![1_i8; coherent_periods]);
    }

    unique.into_iter().collect()
}

pub(crate) fn coherent_secondary_code_phase_hypotheses(
    coherent_periods: usize,
    secondary_code_period_signs: &[i8],
) -> Vec<CoherentSecondaryCodePhaseHypothesis> {
    if coherent_periods == 0 || secondary_code_period_signs.is_empty() {
        return Vec::new();
    }

    secondary_code_period_signs
        .iter()
        .enumerate()
        .map(|(phase, _)| CoherentSecondaryCodePhaseHypothesis {
            secondary_code_phase_periods: phase as u32,
            period_signs: (0..coherent_periods)
                .map(|period_index| {
                    secondary_code_period_signs
                        [(phase + period_index) % secondary_code_period_signs.len()]
                })
                .collect(),
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::{coherent_data_sign_hypotheses, coherent_secondary_code_phase_hypotheses};

    #[test]
    fn single_period_hypotheses_reduce_to_positive_sign() {
        let hypotheses = coherent_data_sign_hypotheses(1, 20);

        assert_eq!(hypotheses, vec![vec![1]]);
    }

    #[test]
    fn gps_l1_twenty_millisecond_window_covers_unknown_boundary_offsets() {
        let hypotheses = coherent_data_sign_hypotheses(20, 20);

        assert!(hypotheses.contains(&vec![1; 20]));
        let mut late_flip = vec![1; 20];
        late_flip[19] = -1;
        assert!(hypotheses.contains(&late_flip));
        let mut early_flip = vec![1; 20];
        early_flip[1..].fill(-1);
        assert!(hypotheses.contains(&early_flip));
    }

    #[test]
    fn galileo_e5b_four_millisecond_symbols_cover_multiple_symbol_sequences() {
        let hypotheses = coherent_data_sign_hypotheses(20, 4);

        assert!(
            hypotheses.contains(&vec![1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,])
        );
        assert!(hypotheses
            .contains(&vec![1, 1, 1, 1, -1, -1, -1, -1, 1, 1, 1, 1, -1, -1, -1, -1, 1, 1, 1, 1,]));
        assert!(hypotheses
            .contains(&vec![1, 1, 1, -1, -1, -1, -1, 1, 1, 1, 1, -1, -1, -1, -1, 1, 1, 1, 1, -1,]));
    }

    #[test]
    fn secondary_code_phase_hypotheses_cover_each_alignment_once() {
        let hypotheses = coherent_secondary_code_phase_hypotheses(4, &[1, -1, -1]);

        assert_eq!(hypotheses.len(), 3);
        assert_eq!(hypotheses[0].period_signs, vec![1, -1, -1, 1]);
        assert_eq!(hypotheses[1].secondary_code_phase_periods, 1);
        assert_eq!(hypotheses[1].period_signs, vec![-1, -1, 1, -1]);
        assert_eq!(hypotheses[2].secondary_code_phase_periods, 2);
        assert_eq!(hypotheses[2].period_signs, vec![-1, 1, -1, -1]);
    }
}
