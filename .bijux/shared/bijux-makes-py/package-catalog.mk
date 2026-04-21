ROOT_PACKAGE_PROFILE_DIR ?= $(ROOT_MAKEFILE_DIR)/packages
PACKAGE_MAKE_DIR ?= $(ROOT_PACKAGE_PROFILE_DIR)
PACKAGE ?=
PACKAGE_FIELD_SEPARATOR ?= |
PACKAGE_GROUP_SEPARATOR ?= ,
PACKAGE_RECORDS ?=
PACKAGE_ALIASES ?=

ROOT_PACKAGE_TARGETS ?= test lint quality security api build sbom clean
ROOT_TARGET_GROUPS_test ?= test
ROOT_TARGET_GROUPS_lint ?= check
ROOT_TARGET_GROUPS_quality ?= check
ROOT_TARGET_GROUPS_security ?= check
ROOT_TARGET_GROUPS_api ?= api
ROOT_TARGET_GROUPS_build ?= buildable
ROOT_TARGET_GROUPS_sbom ?= sbom
ROOT_TARGET_GROUPS_clean ?= all
ROOT_TARGET_FALLBACK_GROUPS_test ?= check
ROOT_TARGET_SHARED_ENV_test ?= 1
ROOT_TARGET_SHARED_ENV_lint ?= 1
ROOT_TARGET_SHARED_ENV_quality ?= 1
ROOT_TARGET_SHARED_ENV_security ?= 1
ROOT_TARGET_SHARED_ENV_api ?= 0
ROOT_TARGET_SHARED_ENV_build ?= 0
ROOT_TARGET_SHARED_ENV_sbom ?= 0
ROOT_TARGET_SHARED_ENV_clean ?= 0
ROOT_TARGET_POST_clean ?= @$(MAKE) clean-root-artifacts

define register_package_record
PACKAGE_GROUPS_$(word 1,$(subst $(PACKAGE_FIELD_SEPARATOR), ,$(1))) := $(subst $(PACKAGE_GROUP_SEPARATOR), ,$(word 2,$(subst $(PACKAGE_FIELD_SEPARATOR), ,$(1))))
PACKAGE_PROFILE_$(word 1,$(subst $(PACKAGE_FIELD_SEPARATOR), ,$(1))) := $(ROOT_PACKAGE_PROFILE_DIR)/$(word 3,$(subst $(PACKAGE_FIELD_SEPARATOR), ,$(1)))
endef

$(foreach record,$(PACKAGE_RECORDS),$(eval $(call register_package_record,$(record))))

ALL_PACKAGES := $(foreach record,$(PACKAGE_RECORDS),$(word 1,$(subst $(PACKAGE_FIELD_SEPARATOR), ,$(record))))

define packages_in_group
$(strip $(foreach package,$(ALL_PACKAGES),$(if $(filter $(1),$(PACKAGE_GROUPS_$(package))),$(package))))
endef

define packages_in_groups
$(strip \
$(if $(filter all,$(1)),$(ALL_PACKAGES), \
$(foreach package,$(ALL_PACKAGES),$(if $(filter $(1),$(PACKAGE_GROUPS_$(package))),$(package)))))
endef

define packages_for_target
$(strip \
$(or $(call packages_in_groups,$(ROOT_TARGET_GROUPS_$(1))), \
$(call packages_in_groups,$(ROOT_TARGET_FALLBACK_GROUPS_$(1)))))
endef

PRIMARY_PACKAGES ?= $(call packages_in_group,primary)
COMPAT_PACKAGES ?= $(call packages_in_group,compat)
CHECK_PACKAGES ?= $(call packages_in_group,check)
API_PACKAGES ?= $(call packages_in_group,api)
BUILD_PACKAGES ?= $(call packages_in_group,buildable)
SBOM_PACKAGES ?= $(call packages_in_group,sbom)

VALID_PACKAGE_VALUES := $(ALL_PACKAGES) $(foreach mapping,$(PACKAGE_ALIASES),$(word 1,$(subst =, ,$(mapping))))
ROOT_PACKAGE_DIRS := $(addprefix $(CURDIR)/packages/,$(ALL_PACKAGES))
ROOT_DISCOVERED_PACKAGE_DIRS := $(sort $(wildcard $(CURDIR)/packages/*))
ROOT_DECLARED_PACKAGE_PROFILE_FILES := $(foreach package,$(ALL_PACKAGES),$(PACKAGE_PROFILE_$(package)))
ROOT_MISSING_PACKAGE_DIRS := $(filter-out $(ROOT_DISCOVERED_PACKAGE_DIRS),$(ROOT_PACKAGE_DIRS))
ROOT_MISSING_PACKAGE_PROFILE_FILES := $(foreach file,$(ROOT_DECLARED_PACKAGE_PROFILE_FILES),$(if $(wildcard $(file)),,$(file)))
ROOT_UNDECLARED_PACKAGE_DIRS := $(filter-out $(ROOT_PACKAGE_DIRS),$(ROOT_DISCOVERED_PACKAGE_DIRS))

ifneq ($(strip $(ROOT_MISSING_PACKAGE_DIRS)),)
$(error Package inventory references missing directories: $(ROOT_MISSING_PACKAGE_DIRS))
endif

ifneq ($(strip $(ROOT_MISSING_PACKAGE_PROFILE_FILES)),)
$(error Package inventory references missing profiles: $(ROOT_MISSING_PACKAGE_PROFILE_FILES))
endif

ifneq ($(strip $(ROOT_UNDECLARED_PACKAGE_DIRS)),)
$(error Package directories are missing from makes/packages.mk: $(notdir $(ROOT_UNDECLARED_PACKAGE_DIRS)))
endif

define resolve_package
$(strip \
$(if $(filter $(1),$(ALL_PACKAGES)),$(1), \
$(foreach mapping,$(PACKAGE_ALIASES), \
$(if $(filter $(1),$(word 1,$(subst =, ,$(mapping)))),$(word 2,$(subst =, ,$(mapping)))))))
endef

define resolve_package_profile
$(strip $(or $(PACKAGE_PROFILE_$(1)),$(PACKAGE_MAKE_DIR)/$(1).mk))
endef

define assert_package
	@if [ -n "$(PACKAGE)" ] && [ -z "$(call resolve_package,$(PACKAGE))" ]; then \
	  echo "Unknown package '$(PACKAGE)'."; \
	  echo "Valid package values:"; \
	  printf "  %s\n" $(VALID_PACKAGE_VALUES); \
	  exit 2; \
	fi
endef

$(foreach target,$(ROOT_PACKAGE_TARGETS),$(eval ROOT_TARGET_PACKAGES_$(target) ?= $(call packages_for_target,$(target))))
PACKAGE_PROFILE_MAPPINGS := $(foreach package,$(ALL_PACKAGES),$(package)=$(call resolve_package_profile,$(package)))
