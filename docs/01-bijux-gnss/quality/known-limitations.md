---
title: Known Limitations
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Known Limitations

This handbook does not pretend the command crate is large in code for the same
reason the lower scientific crates are large.

## Main Limitations

- the command boundary is intentionally thin, so many behavioral questions must
  still hand off to lower-owner handbooks
- the test surface is integration-heavy, which requires discipline when
  choosing the right validation slice
- facade convenience can create pressure to grow the top package beyond its
  proper role
