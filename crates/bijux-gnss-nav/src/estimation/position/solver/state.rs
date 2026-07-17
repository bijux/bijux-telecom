use std::collections::BTreeSet;

use super::geodesy::ecef_to_geodetic;
use super::observation_inputs::constellation_primary_band;
use super::{
    Constellation, InterSystemBias, PositionObservation, PositionObservationCorrectionChain,
    SatId, SatelliteState, Seconds,
};

#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) struct ClockStateModel {
    pub(super) reference_constellation: Constellation,
    pub(super) offset_constellations: Vec<Constellation>,
}

impl ClockStateModel {
    pub(super) fn from_constellations<I>(constellations: I) -> Option<Self>
    where
        I: IntoIterator<Item = Constellation>,
    {
        let unique = constellations.into_iter().collect::<BTreeSet<_>>();
        if unique.is_empty() {
            return None;
        }
        let reference_constellation = if unique.contains(&Constellation::Gps) {
            Constellation::Gps
        } else {
            *unique.iter().next().expect("non-empty constellation set")
        };
        let offset_constellations = unique
            .into_iter()
            .filter(|constellation| *constellation != reference_constellation)
            .collect();
        Some(Self { reference_constellation, offset_constellations })
    }

    pub(super) fn from_inputs(
        inputs: &[super::super::navigation::PositionSolveInput],
    ) -> Option<Self> {
        Self::from_constellations(inputs.iter().map(|input| input.observation.sat.constellation))
    }

    pub(super) fn state_len(&self) -> usize {
        1 + self.offset_constellations.len()
    }

    pub(super) fn parameter_len(&self) -> usize {
        3 + self.state_len()
    }

    pub(super) fn offset_index(&self, constellation: Constellation) -> Option<usize> {
        self.offset_constellations
            .iter()
            .position(|candidate| *candidate == constellation)
            .map(|index| index + 1)
    }

    pub(super) fn contains(&self, constellation: Constellation) -> bool {
        constellation == self.reference_constellation || self.offset_index(constellation).is_some()
    }

    pub(super) fn reference_clock_bias_s(&self, state: &[f64]) -> f64 {
        state.first().copied().unwrap_or(0.0)
    }

    pub(super) fn constellation_clock_bias_s(
        &self,
        state: &[f64],
        constellation: Constellation,
    ) -> Option<f64> {
        if constellation == self.reference_constellation {
            return Some(self.reference_clock_bias_s(state));
        }
        let offset_index = self.offset_index(constellation)?;
        Some(self.reference_clock_bias_s(state) + state.get(offset_index).copied().unwrap_or(0.0))
    }

    pub(super) fn design_row(&self, constellation: Constellation) -> Option<Vec<f64>> {
        if !self.contains(constellation) {
            return None;
        }
        let mut row = vec![0.0; self.state_len()];
        row[0] = 1.0;
        if let Some(offset_index) = self.offset_index(constellation) {
            row[offset_index] = 1.0;
        }
        Some(row)
    }

    pub(super) fn reproject_state(
        &self,
        previous: &ClockStateModel,
        previous_state: &[f64],
    ) -> Vec<f64> {
        let mut projected_state = vec![0.0; self.state_len()];
        projected_state[0] = previous
            .constellation_clock_bias_s(previous_state, self.reference_constellation)
            .unwrap_or(previous.reference_clock_bias_s(previous_state));
        for (index, constellation) in self.offset_constellations.iter().copied().enumerate() {
            let constellation_bias_s = previous
                .constellation_clock_bias_s(previous_state, constellation)
                .unwrap_or(projected_state[0]);
            projected_state[index + 1] = constellation_bias_s - projected_state[0];
        }
        projected_state
    }

    pub(super) fn inter_system_biases(&self, state: &[f64]) -> Vec<InterSystemBias> {
        self.offset_constellations
            .iter()
            .enumerate()
            .map(|(index, constellation)| InterSystemBias {
                constellation: *constellation,
                band: Some(constellation_primary_band(*constellation)),
                bias_s: Seconds(state.get(index + 1).copied().unwrap_or(0.0)),
            })
            .collect()
    }
}

#[derive(Debug, Clone)]
pub(super) struct PositionEstimate {
    pub(super) ecef_x_m: f64,
    pub(super) ecef_y_m: f64,
    pub(super) ecef_z_m: f64,
    pub(super) clock_model: ClockStateModel,
    pub(super) clock_state_s: Vec<f64>,
}

impl PositionEstimate {
    pub(super) fn origin(clock_model: ClockStateModel) -> Self {
        Self {
            ecef_x_m: 0.0,
            ecef_y_m: 0.0,
            ecef_z_m: 0.0,
            clock_state_s: vec![0.0; clock_model.state_len()],
            clock_model,
        }
    }

    pub(super) fn reproject(&self, clock_model: ClockStateModel) -> Self {
        if self.clock_model == clock_model {
            return self.clone();
        }
        Self {
            ecef_x_m: self.ecef_x_m,
            ecef_y_m: self.ecef_y_m,
            ecef_z_m: self.ecef_z_m,
            clock_state_s: clock_model.reproject_state(&self.clock_model, &self.clock_state_s),
            clock_model,
        }
    }

    pub(super) fn reference_clock_bias_s(&self) -> f64 {
        self.clock_model.reference_clock_bias_s(&self.clock_state_s)
    }

    pub(super) fn constellation_clock_bias_s(
        &self,
        constellation: Constellation,
    ) -> Option<f64> {
        self.clock_model
            .constellation_clock_bias_s(&self.clock_state_s, constellation)
    }

    pub(super) fn llh(&self) -> super::Llh {
        let (latitude_deg, longitude_deg, altitude_m) =
            ecef_to_geodetic(self.ecef_x_m, self.ecef_y_m, self.ecef_z_m);
        super::Llh {
            lat_deg: latitude_deg,
            lon_deg: longitude_deg,
            alt_m: altitude_m,
        }
    }
}

#[derive(Debug, Clone)]
pub(super) struct WorkingSetResidual {
    pub(super) sat: SatId,
    pub(super) residual_m: f64,
    pub(super) base_weight: f64,
    pub(super) effective_weight: f64,
}

#[derive(Debug, Default)]
pub(super) struct ConstellationResidualAccumulator {
    pub(super) pre_fit_sum_sq_m2: f64,
    pub(super) pre_fit_sat_count: usize,
    pub(super) post_fit_sum_sq_m2: f64,
    pub(super) post_fit_sat_count: usize,
}

#[derive(Debug, Clone)]
pub(super) struct SatelliteGeometry {
    pub(super) observation: PositionObservation,
    pub(super) corrected_pseudorange_m: f64,
    pub(super) broadcast_group_delay_correction_chain: PositionObservationCorrectionChain,
    pub(super) state: SatelliteState,
    pub(super) iono_delay_m: f64,
    pub(super) tropo_delay_m: f64,
}

#[derive(Debug, Clone)]
pub(super) struct WorkingSetSolution {
    pub(super) estimate: PositionEstimate,
    pub(super) geometry: Vec<SatelliteGeometry>,
    pub(super) residuals: Vec<WorkingSetResidual>,
    pub(super) covariance: Option<Vec<Vec<f64>>>,
    pub(super) covariance_symmetrized: bool,
    pub(super) covariance_clamped: bool,
    pub(super) covariance_max_variance: Option<f64>,
    pub(super) solver_rank: usize,
    pub(super) solver_condition_number: Option<f64>,
}

#[derive(Debug, Clone)]
pub(super) struct RaimExclusionCandidate {
    pub(super) excluded_index: usize,
    pub(super) excluded_sat: SatId,
    pub(super) candidate_estimate: PositionEstimate,
    pub(super) pre_exclusion_rms_m: f64,
    pub(super) post_exclusion_rms_m: f64,
    pub(super) solution_shift_m: f64,
}
