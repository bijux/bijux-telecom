ROOT_MK_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))

include $(ROOT_MK_DIR)/_macro.mk
include $(ROOT_MK_DIR)/bijux-docs.mk
include $(ROOT_MK_DIR)/docs.mk
include $(ROOT_MK_DIR)/rust.mk
include $(ROOT_MK_DIR)/release.mk
include $(ROOT_MK_DIR)/ci.mk
include $(ROOT_MK_DIR)/bijux-std.mk
include $(ROOT_MK_DIR)/contracts.mk

BIJUX_MAKE_COMPONENTS := rust
include $(CURDIR)/.bijux/shared/bijux-makes/bijux.mk
