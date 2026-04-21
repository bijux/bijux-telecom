# Bijux Shell CSS Ownership

- `00-tokens.css`: global tokens only.
- `01-theme.css`: scheme variables and scheme-bound visual overrides.
- `02-layout.css`: shell/page layout and baseline typography structure.
- `03-header.css`: header bar and hub-strip layout/visuals.
- `04-nav.css`: site tabs, detail tabs, section selector, and sidebar/drawer nav visuals.
- `05-content.css`: markdown prose, headings, tables, code, and blockquote styling.
- `06-components.css`: reusable UI components (hero, panels, cards, callouts, media).
- `07-utilities.css`: focus and helper utility rules.
- `08-responsive.css`: all media-query rules, including phone shell-mode behavior.

`extra.css` is the shell stylesheet manifest and should only import these files.
