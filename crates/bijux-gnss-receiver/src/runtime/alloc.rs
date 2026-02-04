#![allow(missing_docs)]

#[cfg(feature = "alloc-trace")]
use std::alloc::{GlobalAlloc, Layout, System};
#[cfg(feature = "alloc-trace")]
use std::sync::atomic::{AtomicU64, Ordering};

#[cfg(feature = "alloc-trace")]
static ALLOCATIONS: AtomicU64 = AtomicU64::new(0);

#[cfg(feature = "alloc-trace")]
pub struct CountingAlloc;

#[cfg(feature = "alloc-trace")]
unsafe impl GlobalAlloc for CountingAlloc {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let ptr = System.alloc(layout);
        if !ptr.is_null() {
            ALLOCATIONS.fetch_add(1, Ordering::Relaxed);
        }
        ptr
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        System.dealloc(ptr, layout);
    }
}

#[cfg(feature = "alloc-trace")]
#[global_allocator]
static GLOBAL: CountingAlloc = CountingAlloc;

#[cfg(feature = "alloc-trace")]
pub fn allocation_count() -> u64 {
    ALLOCATIONS.load(Ordering::Relaxed)
}

#[cfg(not(feature = "alloc-trace"))]
pub fn allocation_count() -> u64 {
    0
}
