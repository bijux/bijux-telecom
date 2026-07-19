require_tool = command -v "$(1)" >/dev/null 2>&1 || { echo "required tool is unavailable: $(1)" >&2; exit 1; }
require_file = test -f "$(1)" || { echo "required file is unavailable: $(1)" >&2; exit 1; }
require_executable = test -x "$(1)" || { echo "required executable is unavailable: $(1)" >&2; exit 1; }
require_var = test -n "$${$(1):-}" || { echo "required variable is unset: $(1)" >&2; exit 1; }
print_section = printf '\n== %s ==\n' "$(1)"
safe_remove = case "$(abspath $(1))" in "$(BIJUX_ARTIFACT_BOUNDARY)"|"$(BIJUX_ARTIFACT_BOUNDARY)"/*) rm -rf "$(abspath $(1))" ;; *) echo "refusing to remove outside $(BIJUX_ARTIFACT_BOUNDARY): $(abspath $(1))" >&2; exit 1 ;; esac
