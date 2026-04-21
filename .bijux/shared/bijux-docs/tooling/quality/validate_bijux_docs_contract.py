#!/usr/bin/env python3
"""Validate Bijux docs contract in MkDocs configuration."""

from __future__ import annotations

import yaml
from pathlib import Path
import sys


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


def validate_contract(config: dict, config_name: str) -> None:
    extra = config.get("extra") or {}
    bijux = extra.get("bijux") or {}

    require(isinstance(bijux, dict), f"{config_name}: extra.bijux must be a mapping")
    require(bool(bijux.get("repository")), f"{config_name}: extra.bijux.repository is required")
    require(bijux.get("nav_mode") == "default", f"{config_name}: extra.bijux.nav_mode must be 'default'")
    require(bijux.get("theme_key") == "bijux:theme", f"{config_name}: extra.bijux.theme_key must be 'bijux:theme'")

    hub_links = bijux.get("hub_links")
    require(isinstance(hub_links, list) and hub_links, f"{config_name}: extra.bijux.hub_links must be a non-empty list")

    for idx, link in enumerate(hub_links, start=1):
        require(isinstance(link, dict), f"{config_name}: hub_links[{idx}] must be a mapping")
        require(bool(link.get("key")), f"{config_name}: hub_links[{idx}].key is required")
        require(bool(link.get("label")), f"{config_name}: hub_links[{idx}].label is required")
        url = link.get("url")
        require(isinstance(url, str) and url.startswith("http"), f"{config_name}: hub_links[{idx}].url must be absolute")


if __name__ == "__main__":
    repo_root = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else Path.cwd()
    shared_cfg = load_yaml(repo_root / "mkdocs.shared.yml")
    root_cfg = load_yaml(repo_root / "mkdocs.yml")

    # Shared config must define baseline bijux contract keys.
    shared_bijux = (shared_cfg.get("extra") or {}).get("bijux") or {}
    require(bool(shared_bijux.get("repository")), "mkdocs.shared.yml: extra.bijux.repository is required")
    require(shared_bijux.get("nav_mode") == "default", "mkdocs.shared.yml: extra.bijux.nav_mode must be 'default'")
    require(shared_bijux.get("theme_key") == "bijux:theme", "mkdocs.shared.yml: extra.bijux.theme_key must be 'bijux:theme'")

    # Root config must provide project-level values, including hub links.
    validate_contract(root_cfg, "mkdocs.yml")

    print("Bijux docs contract validation passed")
