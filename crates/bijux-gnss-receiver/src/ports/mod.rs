//! Receiver port traits (I/O boundaries).
#![allow(missing_docs)]

use bijux_gnss_core::api::{NavSolutionEpochV1, ObsEpochV1, SamplesFrame, Seconds};

pub trait SampleSource {
    type Error;
    fn next_samples(&mut self, frame_len: usize) -> Result<Option<SamplesFrame>, Self::Error>;
}

impl<T> SampleSource for T
where
    T: bijux_gnss_signal::api::SignalSource,
{
    type Error = T::Error;

    fn next_samples(&mut self, frame_len: usize) -> Result<Option<SamplesFrame>, Self::Error> {
        self.next_frame(frame_len)
    }
}

pub trait ArtifactSink {
    type Error;
    fn write_obs(&mut self, obs: &ObsEpochV1) -> Result<(), Self::Error>;
    fn write_nav(&mut self, nav: &NavSolutionEpochV1) -> Result<(), Self::Error>;
}

pub trait Clock {
    fn now_s(&self) -> Seconds;
}

pub struct SystemClock;

impl Clock for SystemClock {
    fn now_s(&self) -> Seconds {
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs_f64();
        Seconds(now)
    }
}
