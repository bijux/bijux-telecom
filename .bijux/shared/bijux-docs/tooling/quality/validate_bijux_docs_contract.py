#!/usr/bin/env python3
"""Validate Bijux docs contract in MkDocs configuration."""

from __future__ import annotations

import yaml
from pathlib import Path
import sys

CANONICAL_HUB_KEYS = (
    "bijux",
    "bijux-core",
    "bijux-proteomics",
    "bijux-pollenomics",
    "bijux-phylogenetics",
    "bijux-canon",
    "bijux-atlas",
    "bijux-masterclass",
)
MERMAID_SCRIPTS = (
    "assets/javascripts/vendor/mermaid-11.6.0.min.js",
    "assets/javascripts/mermaid-init.js",
)


class MkDocsLoader(yaml.SafeLoader):
    """SafeLoader that tolerates MkDocs python/name tags."""


def _construct_unknown(loader: MkDocsLoader, tag_suffix: str, node: yaml.Node):
    if isinstance(node, yaml.ScalarNode):
        return loader.construct_scalar(node)
    if isinstance(node, yaml.SequenceNode):
        return loader.construct_sequence(node)
    if isinstance(node, yaml.MappingNode):
        return loader.construct_mapping(node)
    return None


MkDocsLoader.add_multi_constructor("", _construct_unknown)


def load_yaml(path: Path) -> dict:
    try:
        return yaml.load(path.read_text(encoding="utf-8"), Loader=MkDocsLoader) or {}
    except Exception as exc:  # pragma: no cover - surfaced in command output
        raise RuntimeError(f"Failed to load {path}: {exc}") from exc


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def validate_hub_links(hub_links: list[dict], config_name: str) -> None:
    keys: list[str] = []
    for idx, link in enumerate(hub_links, start=1):
        require(isinstance(link, dict), f"{config_name}: hub_links[{idx}] must be a mapping")
        key = link.get("key")
        require(bool(key), f"{config_name}: hub_links[{idx}].key is required")
        require(key not in keys, f"{config_name}: hub_links[{idx}].key '{key}' is duplicated")
        keys.append(key)
        require(bool(link.get("label")), f"{config_name}: hub_links[{idx}].label is required")
        url = link.get("url")
        require(isinstance(url, str) and url.startswith("http"), f"{config_name}: hub_links[{idx}].url must be absolute")

    canonical_positions = [CANONICAL_HUB_KEYS.index(key) for key in keys if key in CANONICAL_HUB_KEYS]
    require(
        canonical_positions == sorted(canonical_positions),
        f"{config_name}: hub_links canonical repositories must appear in Bijux/Core/Proteomics/Pollenomics/Phylogenetics/Canon/Atlas/Masterclass order",
    )


def validate_mermaid_contract(config: dict, config_name: str) -> None:
    markdown_extensions = config.get("markdown_extensions") or []
    require(
        isinstance(markdown_extensions, list),
        f"{config_name}: markdown_extensions must be a list",
    )
    mermaid_fence_present = False
    for extension in markdown_extensions:
        if not isinstance(extension, dict):
            continue
        superfences = extension.get("pymdownx.superfences")
        if not isinstance(superfences, dict):
            continue
        custom_fences = superfences.get("custom_fences") or []
        for fence in custom_fences:
            if isinstance(fence, dict) and fence.get("name") == "mermaid":
                mermaid_fence_present = True
                break
        if mermaid_fence_present:
            break
    require(
        mermaid_fence_present,
        f"{config_name}: markdown_extensions must include a pymdownx.superfences mermaid custom fence",
    )

    extra_javascript = config.get("extra_javascript") or []
    require(
        isinstance(extra_javascript, list),
        f"{config_name}: extra_javascript must be a list",
    )
    for script in MERMAID_SCRIPTS:
        require(
            script in extra_javascript,
            f"{config_name}: extra_javascript must include {script}",
        )


def validate_contract(config: dict, config_name: str) -> None:
    extra = config.get("extra") or {}
    bijux = extra.get("bijux") or {}

    require(isinstance(bijux, dict), f"{config_name}: extra.bijux must be a mapping")
    require(bool(bijux.get("repository")), f"{config_name}: extra.bijux.repository is required")
    require(bijux.get("nav_mode") == "default", f"{config_name}: extra.bijux.nav_mode must be 'default'")
    require(bijux.get("theme_key") == "bijux:theme", f"{config_name}: extra.bijux.theme_key must be 'bijux:theme'")

    hub_links = bijux.get("hub_links")
    require(isinstance(hub_links, list) and hub_links, f"{config_name}: extra.bijux.hub_links must be a non-empty list")
    validate_hub_links(hub_links, config_name)


if __name__ == "__main__":
    repo_root = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else Path.cwd()
    shared_cfg = load_yaml(repo_root / "mkdocs.shared.yml")
    root_cfg = load_yaml(repo_root / "mkdocs.yml")

    # Shared config must define baseline bijux contract keys.
    shared_bijux = (shared_cfg.get("extra") or {}).get("bijux") or {}
    require(bool(shared_bijux.get("repository")), "mkdocs.shared.yml: extra.bijux.repository is required")
    require(shared_bijux.get("nav_mode") == "default", "mkdocs.shared.yml: extra.bijux.nav_mode must be 'default'")
    require(shared_bijux.get("theme_key") == "bijux:theme", "mkdocs.shared.yml: extra.bijux.theme_key must be 'bijux:theme'")
    validate_mermaid_contract(shared_cfg, "mkdocs.shared.yml")

    # Root config must provide project-level values, including hub links.
    validate_contract(root_cfg, "mkdocs.yml")

    print("Bijux docs contract validation passed")
