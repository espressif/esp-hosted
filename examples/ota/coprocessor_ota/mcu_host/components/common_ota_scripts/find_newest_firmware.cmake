# Common script to find the newest firmware file dynamically during build
# This runs every time during build, not just during configuration
#
# Required input variables:
#   - SOURCE_COMPONENT_DIR: Path to the component source directory
#   - BINARY_COMPONENT_DIR: Path to the component binary directory
#   - OTA_ACTION: "prepare_littlefs" or "select_for_partition"
#
# Optional input variables:
#   - COMPONENT_NAME: Name for logging (default: "OTA")

if(NOT DEFINED COMPONENT_NAME)
    set(COMPONENT_NAME "OTA")
endif()

# Use the passed component directories
set(SOURCE_DIR "${SOURCE_COMPONENT_DIR}/slave_fw_bin")
set(TEMP_DIR "${BINARY_COMPONENT_DIR}/temp_littlefs")

message(STATUS "${COMPONENT_NAME}: Dynamically searching for newest firmware...")
message(STATUS "${COMPONENT_NAME}: Searching in ${SOURCE_DIR}")

# Get all .bin files
file(GLOB FIRMWARE_FILES "${SOURCE_DIR}/*.bin")

if(NOT FIRMWARE_FILES)
    message(FATAL_ERROR "${COMPONENT_NAME}: No .bin files found in ${SOURCE_DIR}")
endif()

# Find the newest file by timestamp
set(NEWEST_FILE "")
set(NEWEST_TIMESTAMP 0)

foreach(FIRMWARE_FILE ${FIRMWARE_FILES})
    file(TIMESTAMP "${FIRMWARE_FILE}" FILE_TIMESTAMP "%s")
    message(STATUS "${COMPONENT_NAME}: File ${FIRMWARE_FILE} timestamp: ${FILE_TIMESTAMP}")
    if(FILE_TIMESTAMP GREATER NEWEST_TIMESTAMP)
        set(NEWEST_FILE "${FIRMWARE_FILE}")
        set(NEWEST_TIMESTAMP "${FILE_TIMESTAMP}")
    endif()
endforeach()

get_filename_component(NEWEST_FILENAME "${NEWEST_FILE}" NAME)
message(STATUS "${COMPONENT_NAME}: Selected newest firmware: ${NEWEST_FILENAME}")

# Perform the requested action
if(OTA_ACTION STREQUAL "prepare_littlefs")
    # Clean and recreate temp directory for LittleFS
    file(REMOVE_RECURSE "${TEMP_DIR}")
    file(MAKE_DIRECTORY "${TEMP_DIR}")
    file(COPY "${NEWEST_FILE}" DESTINATION "${TEMP_DIR}")
    message(STATUS "${COMPONENT_NAME}: Prepared ${NEWEST_FILENAME} in temp directory for LittleFS")

elseif(OTA_ACTION STREQUAL "select_for_partition")
    # Write selection to files for partition OTA
    file(WRITE "${BINARY_COMPONENT_DIR}/selected_firmware.txt" "${NEWEST_FILE}")
    file(WRITE "${BINARY_COMPONENT_DIR}/selected_firmware_name.txt" "${NEWEST_FILENAME}")
    message(STATUS "${COMPONENT_NAME}: Saved firmware selection for partition flashing")

else()
    message(FATAL_ERROR "${COMPONENT_NAME}: Unknown OTA_ACTION: ${OTA_ACTION}")
endif()

message(STATUS "${COMPONENT_NAME}: Firmware selection completed")