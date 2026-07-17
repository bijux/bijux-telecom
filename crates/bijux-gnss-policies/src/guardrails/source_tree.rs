//! Source-tree discovery and topology checks for guardrail evaluation.

mod discovery;
mod naming;
mod topology;

pub(crate) use discovery::collect_rs_files;
pub(crate) use naming::check_forbidden_filenames;
pub(crate) use topology::{
    check_depth, check_empty_modules, check_mod_only_dirs, check_mod_reexports_only,
};
