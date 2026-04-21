#![allow(missing_docs)]

#[cfg(feature = "alloc-trace")]
use std::sync::atomic::{AtomicU64, Ordering};

#[cfg(feature = "alloc-trace")]
static ALLOCATIONS: AtomicU64 = AtomicU64::new(0);

#[cfg(feature = "alloc-trace")]
pub fn allocation_count() -> u64 {
    ALLOCATIONS.load(Ordering::Relaxed)
}

#[cfg(not(feature = "alloc-trace"))]
pub fn allocation_count() -> u64 {
    0
}
