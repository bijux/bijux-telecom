//! Shared guardrail policy enforcement across crates.
//! See docs/GLOSSARY.md for acronym definitions.

#![deny(missing_docs)]

use std::path::Path;

mod api_surface;
pub mod config;
mod content_policy;
pub mod error;
mod source_tree;

use self::api_surface::{
    check_api_purity, check_pub_items, check_pub_items_outside_api,
    check_pub_use_locations_if_enabled, check_pub_use_spam,
};
use self::config::GuardrailConfig;
use self::content_policy::{check_panic_expect, check_purity_zones, check_stage_id_strings};
use self::error::Result;
use self::source_tree::{
    check_depth, check_empty_modules, check_forbidden_filenames, check_mod_only_dirs,
    check_mod_reexports_only, collect_rs_files,
};

/// Run guardrail checks on a crate root.
pub fn check(crate_root: &Path, config: &GuardrailConfig) -> Result<()> {
    let src_dir = crate_root.join("src");
    let files = collect_rs_files(&src_dir)?;
    check_depth(&src_dir, &files, config)?;
    check_mod_only_dirs(&src_dir)?;
    check_empty_modules(&files)?;
    check_mod_reexports_only(&files)?;
    check_pub_items(&files, config)?;
    check_api_purity(&files)?;
    if config.enforce_pub_use_api_only {
        check_pub_items_outside_api(&files)?;
    }
    check_forbidden_filenames(&files)?;
    check_pub_use_locations_if_enabled(&files, config)?;
    if config.forbid_pub_use_spam {
        check_pub_use_spam(&files, config)?;
    }
    if config.forbid_panic_expect {
        check_panic_expect(&files, config)?;
    }
    if config.forbid_stage_id_strings {
        check_stage_id_strings(&files, config)?;
    }
    check_purity_zones(&files, config)?;
    Ok(())
}
