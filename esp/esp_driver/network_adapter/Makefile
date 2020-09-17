#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_VERSION := 0.2
ifndef MAKE_RESTARTS
$(info *********************************************************************************)
$(info *              Building ESP-Hosted firmware version :: $(PROJECT_VERSION)            )
$(info *********************************************************************************)
endif

PROJECT_NAME := network_adapter_$(PROJECT_VERSION)
CPPFLAGS += -DPROJECT_VERSION=$(PROJECT_VERSION)

EXTRA_COMPONENT_DIRS := $(realpath ../../../common)
include $(IDF_PATH)/make/project.mk
