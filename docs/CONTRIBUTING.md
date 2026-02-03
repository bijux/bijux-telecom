# Contributing

## Code Standards
- Follow `docs/CODEBASE_RULES.md` and `docs/NAMING.md`.
- Avoid thin modules; keep module ownership crisp.
- Keep crate depth ≤ 4.

## Tests
See `docs/TESTING.md` for test taxonomy and commands.

## CI
All changes must pass:
```bash
cargo make ci-fast
```
