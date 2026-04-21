# Bijux Docs Shell

This directory is the canonical shell source for Bijux site chrome.

Ownership:
- `partials/`: shell structure (header/nav partials).
- `styles/`: shell visual system and responsive behavior.
- `scripts/`: shell runtime state and navigation behavior.
- `assets/`: shared logo and site icon sources.

Consumption model:
1. Edit files in `shared/bijux-docs/*` only.
2. Run `make bijux-docs-sync`.
3. Generated mirrors are synchronized into:
   - `docs/overrides/partials/*`
   - `docs/assets/styles/*`
   - `docs/assets/javascripts/mermaid-init.js`
   - `docs/assets/javascripts/shell/*`
   - `docs/assets/javascripts/navigation-sync.js`
   - `docs/assets/bijux_icon.png`
   - `docs/assets/bijux_logo_hq.png`
   - `docs/assets/site-icons/*`
