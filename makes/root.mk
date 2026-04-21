ROOT_MK_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))

include $(ROOT_MK_DIR)/_macro.mk
include $(ROOT_MK_DIR)/_internal.mk
include $(ROOT_MK_DIR)/rust.mk
include $(ROOT_MK_DIR)/ci.mk
include $(ROOT_MK_DIR)/bijux-std.mk
