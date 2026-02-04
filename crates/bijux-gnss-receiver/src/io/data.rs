#![allow(missing_docs)]

use std::fs::File;
use std::io::{Read, Seek, SeekFrom};

use thiserror::Error;

use bijux_gnss_core::api::{SampleClock, SampleTime, SamplesFrame, Seconds};

use bijux_gnss_signal::api::{iq_i16_to_samples, SampleSource, SignalSource};

#[derive(Debug, Error)]
pub enum SampleSourceError {
    #[error("i/o error: {0}")]
    Io(#[from] std::io::Error),

    #[error("IQ sample stream length is not even")]
    InvalidIqLength,
}

/// Simple in-memory sample source for tests and examples.
pub struct MemorySamples {
    data: Vec<i16>,
    cursor_samples: usize,
    clock: SampleClock,
}

impl MemorySamples {
    pub fn new(data: Vec<i16>, sample_rate_hz: f64) -> Result<Self, SampleSourceError> {
        if !data.len().is_multiple_of(2) {
            return Err(SampleSourceError::InvalidIqLength);
        }
        Ok(Self {
            data,
            cursor_samples: 0,
            clock: SampleClock::new(sample_rate_hz),
        })
    }
}

impl SignalSource for MemorySamples {
    type Error = SampleSourceError;

    fn sample_rate_hz(&self) -> f64 {
        self.clock.sample_rate_hz
    }

    fn next_frame(&mut self, frame_len: usize) -> Result<Option<SamplesFrame>, SampleSourceError> {
        let total_samples = self.data.len() / 2;
        let remaining = total_samples.saturating_sub(self.cursor_samples);
        if remaining == 0 {
            return Ok(None);
        }
        let count = remaining.min(frame_len);
        let start = self.cursor_samples * 2;
        let end = start + count * 2;
        let iq = iq_i16_to_samples(&self.data[start..end]);
        let t0 = SampleTime {
            sample_index: self.cursor_samples as u64,
            sample_rate_hz: self.clock.sample_rate_hz,
        };
        let frame = SamplesFrame::new(t0, Seconds(self.clock.dt_s()), iq);
        self.cursor_samples += count;
        Ok(Some(frame))
    }

    fn is_done(&self) -> bool {
        self.cursor_samples >= self.data.len() / 2
    }
}

/// File-backed sample source for raw i16 samples.
pub struct FileSamples {
    file: File,
    done: bool,
    clock: SampleClock,
    sample_index: u64,
}

impl FileSamples {
    pub fn open(
        path: &str,
        offset_bytes: u64,
        sample_rate_hz: f64,
    ) -> Result<Self, SampleSourceError> {
        let mut file = File::open(path)?;
        if offset_bytes > 0 {
            file.seek(SeekFrom::Start(offset_bytes))?;
        }
        Ok(Self {
            file,
            done: false,
            clock: SampleClock::new(sample_rate_hz),
            sample_index: 0,
        })
    }
}

impl SignalSource for FileSamples {
    type Error = SampleSourceError;

    fn sample_rate_hz(&self) -> f64 {
        self.clock.sample_rate_hz
    }

    fn next_frame(&mut self, frame_len: usize) -> Result<Option<SamplesFrame>, SampleSourceError> {
        if self.done {
            return Ok(None);
        }
        let byte_len = frame_len * 4;
        let mut buf = vec![0u8; byte_len];
        let read = self.file.read(&mut buf)?;
        if read == 0 {
            self.done = true;
            return Ok(None);
        }
        let i16_count = read / 2;
        if !i16_count.is_multiple_of(2) {
            self.done = true;
            return Err(SampleSourceError::InvalidIqLength);
        }
        let samples_read = i16_count / 2;
        let mut i16_buf = vec![0i16; i16_count];
        for i in 0..i16_count {
            let lo = buf[2 * i] as u16;
            let hi = buf[2 * i + 1] as u16;
            i16_buf[i] = i16::from_le_bytes([lo as u8, hi as u8]);
        }
        if samples_read < frame_len {
            self.done = true;
        }
        let iq = iq_i16_to_samples(&i16_buf[..samples_read * 2]);
        let t0 = SampleTime {
            sample_index: self.sample_index,
            sample_rate_hz: self.clock.sample_rate_hz,
        };
        let frame = SamplesFrame::new(t0, Seconds(self.clock.dt_s()), iq);
        self.sample_index += samples_read as u64;
        Ok(Some(frame))
    }

    fn is_done(&self) -> bool {
        self.done
    }
}

impl SampleSource for FileSamples {
    type Error = SampleSourceError;

    fn next_samples(&mut self, frame_len: usize) -> Result<Option<SamplesFrame>, Self::Error> {
        self.next_frame(frame_len)
    }
}
