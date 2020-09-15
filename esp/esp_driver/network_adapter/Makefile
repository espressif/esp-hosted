#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_VER := 0.2
$(info *********************************************************************************)
$(info *              Building ESP-Hosted firmware version :: $(PROJECT_VER)                )
$(info *********************************************************************************)

PROJECT_NAME := network_adapter_$(PROJECT_VER)
CPPFLAGS += -DPROJECT_VER=$(PROJECT_VER)

EXTRA_COMPONENT_DIRS := $(realpath ../../../common)
include $(IDF_PATH)/make/project.mk
