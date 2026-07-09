#![allow(missing_docs)]

use std::fs::File;
use std::io::{Read, Seek, SeekFrom};
use std::path::Path;

use thiserror::Error;

use bijux_gnss_core::api::{SampleClock, SampleTime, SamplesFrame, Seconds};

use bijux_gnss_signal::api::{
    iq_f32_to_samples, iq_i16_to_samples, iq_i8_to_samples, IqSampleFormat, RawIqMetadata,
    SampleSource, SignalSource,
};

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
        if data.len() % 2 != 0 {
            return Err(SampleSourceError::InvalidIqLength);
        }
        Ok(Self { data, cursor_samples: 0, clock: SampleClock::new(sample_rate_hz) })
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

/// File-backed sample source for raw IQ captures.
pub struct FileSamples {
    file: File,
    done: bool,
    clock: SampleClock,
    sample_index: u64,
    metadata: RawIqMetadata,
}

impl FileSamples {
    /// Open a raw IQ capture using explicit metadata.
    pub fn open_raw_iq(path: &Path, metadata: RawIqMetadata) -> Result<Self, SampleSourceError> {
        let mut file = File::open(path)?;
        if metadata.offset_bytes > 0 {
            file.seek(SeekFrom::Start(metadata.offset_bytes))?;
        }
        Ok(Self {
            file,
            done: false,
            clock: SampleClock::new(metadata.sample_rate_hz),
            sample_index: 0,
            metadata,
        })
    }

    /// Metadata declared for this raw IQ source.
    pub fn metadata(&self) -> &RawIqMetadata {
        &self.metadata
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
        let byte_len = frame_len * self.metadata.format.bytes_per_complex_sample();
        let mut buf = vec![0u8; byte_len];
        let read = self.file.read(&mut buf)?;
        if read == 0 {
            self.done = true;
            return Ok(None);
        }
        let (iq, samples_read) = decode_samples(&self.metadata, &buf[..read])?;
        if samples_read < frame_len {
            self.done = true;
        }
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

fn decode_samples(
    metadata: &RawIqMetadata,
    raw_bytes: &[u8],
) -> Result<(Vec<bijux_gnss_core::api::Sample>, usize), SampleSourceError> {
    match metadata.format {
        IqSampleFormat::Iq8 => decode_i8_samples(raw_bytes),
        IqSampleFormat::Iq16Le => decode_i16_le_samples(raw_bytes),
        IqSampleFormat::Cf32Le => decode_cf32_le_samples(raw_bytes),
    }
}

fn decode_i8_samples(
    raw_bytes: &[u8],
) -> Result<(Vec<bijux_gnss_core::api::Sample>, usize), SampleSourceError> {
    if raw_bytes.len() % 2 != 0 {
        return Err(SampleSourceError::InvalidIqLength);
    }
    let mut i8_buf = vec![0i8; raw_bytes.len()];
    for (idx, byte) in raw_bytes.iter().enumerate() {
        i8_buf[idx] = *byte as i8;
    }
    Ok((iq_i8_to_samples(&i8_buf), raw_bytes.len() / 2))
}

fn decode_i16_le_samples(
    raw_bytes: &[u8],
) -> Result<(Vec<bijux_gnss_core::api::Sample>, usize), SampleSourceError> {
    let i16_count = raw_bytes.len() / 2;
    if i16_count * 2 != raw_bytes.len() || i16_count % 2 != 0 {
        return Err(SampleSourceError::InvalidIqLength);
    }
    let mut i16_buf = vec![0i16; i16_count];
    for i in 0..i16_count {
        i16_buf[i] = i16::from_le_bytes([raw_bytes[2 * i], raw_bytes[2 * i + 1]]);
    }
    Ok((iq_i16_to_samples(&i16_buf), i16_count / 2))
}

fn decode_cf32_le_samples(
    raw_bytes: &[u8],
) -> Result<(Vec<bijux_gnss_core::api::Sample>, usize), SampleSourceError> {
    let f32_count = raw_bytes.len() / 4;
    if f32_count * 4 != raw_bytes.len() || f32_count % 2 != 0 {
        return Err(SampleSourceError::InvalidIqLength);
    }
    let mut f32_buf = vec![0f32; f32_count];
    for i in 0..f32_count {
        f32_buf[i] = f32::from_le_bytes([
            raw_bytes[4 * i],
            raw_bytes[4 * i + 1],
            raw_bytes[4 * i + 2],
            raw_bytes[4 * i + 3],
        ]);
    }
    Ok((iq_f32_to_samples(&f32_buf), f32_count / 2))
}
