use crate::api::RunArtifacts;
use crate::pipeline::observations::ObservationPipelineArtifacts;

impl RunArtifacts {
    /// Reconstruct the observation-stage artifacts emitted during this run.
    pub fn observation_artifacts(&self) -> ObservationPipelineArtifacts {
        ObservationPipelineArtifacts {
            epochs: self.observations.clone(),
            residuals: self.observation_residuals.clone(),
            measurement_quality: self.observation_measurement_quality.clone(),
        }
    }
}
