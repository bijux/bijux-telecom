API_MODE ?= none
API_ARTIFACTS_DIR ?= $(PROJECT_ARTIFACTS_DIR)/api
API_LINT_DIR ?= $(API_ARTIFACTS_DIR)/lint
API_TEST_DIR ?= $(API_ARTIFACTS_DIR)/test
API_LOG ?= $(API_ARTIFACTS_DIR)/server.log
API_HOST ?= 127.0.0.1
API_PORT ?= 8000
API_NONE_PREREQS ?= install
API_NONE_MESSAGE ?= No API contracts are defined for $(PROJECT_SLUG).
API_NONE_STATUS_FILE ?= $(API_ARTIFACTS_DIR)/status.txt
API_FREEZE_MAKEFILE ?= $(ROOT_MAKE_DIR)/api-freeze.mk
API_ARTIFACTS_DIR_ABS := $(abspath $(API_ARTIFACTS_DIR))
API_LINT_DIR_ABS := $(abspath $(API_LINT_DIR))
API_TEST_DIR_ABS := $(abspath $(API_TEST_DIR))
API_SELF_MAKE ?= $(SELF_MAKE)

ifeq ($(API_MODE),none)
.PHONY: api api-install api-lint api-test api-clean openapi-drift

api: $(API_NONE_PREREQS)
	@mkdir -p "$(API_ARTIFACTS_DIR)"
	@printf "%s\n" "$(API_NONE_MESSAGE)" | tee "$(API_NONE_STATUS_FILE)"

api-install:
	@printf "%s\n" "$(API_NONE_MESSAGE)"

api-lint:
	@printf "%s\n" "$(API_NONE_MESSAGE)"

api-test:
	@printf "%s\n" "$(API_NONE_MESSAGE)"

openapi-drift:
	@printf "%s\n" "$(API_NONE_MESSAGE)"

api-clean:
	@rm -rf "$(API_ARTIFACTS_DIR)"
else
ifeq ($(API_MODE),contract)
include $(ROOT_MAKE_DIR)/bijux-py/api-contract.mk
else ifeq ($(API_MODE),live-contract)
include $(ROOT_MAKE_DIR)/bijux-py/api-live-contract.mk
else ifeq ($(API_MODE),freeze)
include $(API_FREEZE_MAKEFILE)
else
$(error Unsupported API_MODE '$(API_MODE)'; expected none, contract, live-contract, or freeze)
endif
endif
