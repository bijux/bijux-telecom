BIJUX_MAKES_RS_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))

include $(BIJUX_MAKES_RS_DIR)/cargo.mk
