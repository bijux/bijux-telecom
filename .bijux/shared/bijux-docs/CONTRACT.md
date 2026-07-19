# Shell Contract

Required `mkdocs` contract for Bijux projects:

```yaml
extra:
  bijux:
    repository: <string> # project-owned in mkdocs.yml
    nav_mode: default    # inherited from mkdocs.shared.yml
    theme_key: bijux:theme
    hub_links:
      - key: <string>
        label: <string>
        url: <absolute-url>
```

Rules:
- `repository`: active repository key used for hub highlighting.
- `hub_links`: inherited top-level Bijux network links synchronized from `config/hub-links.json` into `mkdocs.shared.yml`. Registry array order is presentation order in every desktop and mobile navigation surface.
- `nav_mode`: shell navigation mode; `default` is canonical.
- `theme_key`: shared localStorage key for cross-project theme persistence.
- `markdown_extensions`: shared config must keep a `pymdownx.superfences` custom fence named `mermaid`.
- `extra_javascript`: shared config must load `assets/javascripts/vendor/mermaid-11.6.0.min.js` and `assets/javascripts/mermaid-init.js` so Mermaid diagrams render consistently across repositories.

Projects set `extra.bijux.repository` in `mkdocs.yml`. They must not duplicate
or override `hub_links` there. Hub membership, labels, URLs, and order change
only in the shared registry. `make bijux-docs-sync` writes the canonical block
to `mkdocs.shared.yml` and removes the former root-level duplicate.

Shared docs assets are source-of-truth in `assets/`:
- `assets/bijux_icon.png`
- `assets/bijux_logo_hq.png`
- `assets/site-icons/favicon.ico`
- `assets/site-icons/apple-touch-icon.png`
- `assets/site-icons/apple-touch-icon-precomposed.png`
