ARTIFACT_ROOT ?= artifacts

require_tool = command -v $(1) >/dev/null 2>&1 || { echo "$(1) is required" >&2; exit 1; }
require_file = test -f "$(1)" || { echo "required file missing: $(1)" >&2; exit 1; }
require_var = test -n "$${$(1):-}" || { echo "required variable missing: $(1)" >&2; exit 1; }
print_section = printf '\n== %s ==\n' "$(1)"
safe_rm = case "$(abspath $(1))" in "$(abspath $(ARTIFACT_ROOT))"/*) rm -rf "$(1)" ;; *) echo "refusing to delete outside $(ARTIFACT_ROOT): $(1)" >&2; exit 1 ;; esac
