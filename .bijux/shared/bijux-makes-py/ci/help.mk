HELP_DEFINE_TARGET ?= 1
HELP_TARGET ?= help
HELP_FILES ?= $(MAKEFILE_LIST)
HELP_WIDTH ?= 20
HELP_SECTION_PRIORITY ?= Repository Orchestration Docs Core Test Lint Quality Security Build API SBOM Publish General

ifeq ($(HELP_DEFINE_TARGET),1)
$(HELP_TARGET):
	@awk -v help_priority='$(HELP_SECTION_PRIORITY)' 'BEGIN{FS=":.*##"; OFS=""; section=""; priority_count=split(help_priority, priority_sections, /[[:space:]]+/)} \
	  function add_section(name) { \
	    if (!(name in section_seen)) { \
	      section_seen[name] = 1; \
	      section_order[++section_count] = name; \
	    } \
	  } \
	  function print_section(name) { \
	    if (section_lines[name] == "" || printed_section[name]) { \
	      return; \
	    } \
	    if (printed_any) { \
	      print ""; \
	    } \
	    print "\033[1m" name "\033[0m"; \
	    printf "%s", section_lines[name]; \
	    printed_any = 1; \
	    printed_section[name] = 1; \
	  } \
	  /^##@/ { \
	    gsub(/^##@ */,""); \
	    section = $$0; \
	    add_section(section); \
	    next; \
	  } \
	  /^[a-zA-Z0-9_.-]+:.*##/ { \
	    if (section == "") { \
	      section = "General"; \
	      add_section(section); \
	    } \
	    if (!( $$1 in target_seen)) { \
	      target_seen[$$1] = 1; \
	      section_lines[section] = section_lines[section] sprintf("  \033[36m%-$(HELP_WIDTH)s\033[0m %s\n", $$1, $$2); \
	    } \
	  } \
	  END { \
	    for (i = 1; i <= priority_count; i++) { \
	      print_section(priority_sections[i]); \
	    } \
	    for (i = 1; i <= section_count; i++) { \
	      print_section(section_order[i]); \
	    } \
	  }' \
	  $(HELP_FILES)
.PHONY: $(HELP_TARGET)
endif
