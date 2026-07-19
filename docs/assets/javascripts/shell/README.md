# Bijux Shell JavaScript Ownership

- `theme-persistence.js`: cross-project theme durability via configured `extra.bijux.theme_key` and palette synchronization.
- `viewport-profile.js`: viewport profile state (`phone|normal|desktop|wide`) and resize/orientation updates.
- `nav-state.js`: site-tab and canonical-path active state decisions.
- `detail-tabs.js`: detail-strip visibility, active detail-tab state, and mode-scoped (`desktop|phone`) sync controllers.
- `nav-reveal.js`: scroll-reveal behavior separated by mode (`desktop|phone`) plus mobile drawer context hooks.
- `bootstrap.js`: viewport-routed shell navigation wiring on `document$.subscribe(...)`.
- `mermaid-init.js`: canonical Mermaid renderer bootstrap, theme-mode mapping, and theme-change rerender behavior.
- `nav-sync.js`: compatibility entrypoint consumed by MkDocs `extra_javascript`.
