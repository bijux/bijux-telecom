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
   - `mkdocs.shared.yml` (`extra.bijux.hub_links`)
   - `docs/overrides/partials/*`
   - `docs/assets/styles/*`
   - `docs/assets/javascripts/mermaid-init.js`
   - `docs/assets/javascripts/shell/*`
   - `docs/assets/javascripts/navigation-sync.js`
   - `docs/assets/bijux_icon.png`
   - `docs/assets/bijux_logo_hq.png`
   - `docs/assets/site-icons/*`

`config/hub-links.json` is the canonical cross-repository hub registry.
Project `mkdocs.yml` files own only repository identity and project-specific
MkDocs values; they do not carry a second hub list. Synchronization removes
an existing root-level hub block after writing the inherited canonical block.
