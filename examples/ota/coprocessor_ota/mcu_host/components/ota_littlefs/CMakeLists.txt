# LittleFS OTA Component CMakeLists.txt
# This component handles LittleFS-based OTA updates

# Only process firmware files if LittleFS OTA is actually selected
message(STATUS "🔍 LittleFS OTA: CONFIG_OTA_METHOD_LITTLEFS = ${CONFIG_OTA_METHOD_LITTLEFS}")
if(CONFIG_OTA_METHOD_LITTLEFS)
	set(SRC_LIST "ota_littlefs.c")

	# Check if we have any firmware files (just for early validation)
	file(GLOB SLAVE_FW_BIN_FILES "${CMAKE_CURRENT_SOURCE_DIR}/slave_fw_bin/*.bin")
	if(NOT SLAVE_FW_BIN_FILES)
		message(WARNING "🔴 LittleFS OTA: No .bin files found in slave_fw_bin directory!")
	endif()
else()
	# LittleFS OTA not selected, skip firmware processing
	message(STATUS "LittleFS OTA: Not selected, skipping firmware processing")
endif()

# Register the component
idf_component_register(SRCS ${SRC_LIST}
	INCLUDE_DIRS "."
	PRIV_REQUIRES "littlefs" "esp_hosted" "bootloader_support" "esp_app_format")

# Create LittleFS partition image after component registration
if(CONFIG_OTA_METHOD_LITTLEFS)
	set(TEMP_LITTLEFS_DIR "${CMAKE_CURRENT_BINARY_DIR}/temp_littlefs")

	# Create a custom target that runs the common script to find newest firmware dynamically
	add_custom_target(prepare_littlefs_temp
		COMMAND ${CMAKE_COMMAND}
			-DSOURCE_COMPONENT_DIR=${CMAKE_CURRENT_SOURCE_DIR}
			-DBINARY_COMPONENT_DIR=${CMAKE_CURRENT_BINARY_DIR}
			-DCOMPONENT_NAME=LittleFS OTA
			-DOTA_ACTION=prepare_littlefs
			-P ${CMAKE_CURRENT_SOURCE_DIR}/../common_ota_scripts/find_newest_firmware.cmake
		COMMENT "LittleFS OTA: Dynamically finding and preparing newest firmware"
		VERBATIM
	)

	# Make the target depend on all firmware files so it rebuilds when they change
	file(GLOB CURRENT_FIRMWARE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/slave_fw_bin/*.bin")
	if(CURRENT_FIRMWARE_FILES)
		add_custom_target(firmware_files_dependency DEPENDS ${CURRENT_FIRMWARE_FILES})
		add_dependencies(prepare_littlefs_temp firmware_files_dependency)
	endif()

	# Create the LittleFS partition image with proper dependency
	littlefs_create_partition_image(storage ${TEMP_LITTLEFS_DIR} FLASH_IN_PROJECT DEPENDS prepare_littlefs_temp)
	# No POST_BUILD cleanup of TEMP_LITTLEFS_DIR: attaching it to ${COMPONENT_LIB}
	# races the storage-image target (which reads that dir) under parallel ninja and
	# intermittently deletes it first. The dir lives in the build tree (cleaned on a
	# clean build), so leaving it is harmless.
endif()
