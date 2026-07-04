# Shell Contract

Required `mkdocs` contract for Bijux projects:

```yaml
extra:
  bijux:
    repository: <string>
    hub_links:
      - key: <string>
        label: <string>
        url: <absolute-url>
    nav_mode: default
    theme_key: bijux:theme
```

Rules:
- `repository`: active repository key used for hub highlighting.
- `hub_links`: top-level Bijux network links. Shared shell rendering keeps the canonical order `Bijux`, `Core`, `Proteomics`, `Pollenomics`, `Phylogenetics`, `Canon`, `Atlas`, `Masterclass`, then appends any extra project-specific links after that canonical strip.
- `nav_mode`: shell navigation mode; `default` is canonical.
- `theme_key`: shared localStorage key for cross-project theme persistence.
- `markdown_extensions`: shared config must keep a `pymdownx.superfences` custom fence named `mermaid`.
- `extra_javascript`: shared config must load `assets/javascripts/vendor/mermaid-11.6.0.min.js` and `assets/javascripts/mermaid-init.js` so Mermaid diagrams render consistently across repositories.

Projects may change repository/link labels and URLs, but must not fork shell partial structure or shell runtime logic.

Shared docs assets are source-of-truth in `assets/`:
- `assets/bijux_icon.png`
- `assets/bijux_logo_hq.png`
- `assets/site-icons/favicon.ico`
- `assets/site-icons/apple-touch-icon.png`
- `assets/site-icons/apple-touch-icon-precomposed.png`
