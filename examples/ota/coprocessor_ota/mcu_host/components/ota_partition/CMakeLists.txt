# Partition OTA Component CMakeLists.txt
# This component handles Partition-based OTA updates

idf_component_register(
    SRCS "ota_partition.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES "esp_partition" "esp_hosted" "bootloader_support" "esp_app_format"
)

if(CONFIG_OTA_METHOD_PARTITION)
# --- Step 0: Get partition offset from partition table ---
set(SLAVE_FW_PARTITION_NAME "slave_fw")
file(READ "${CMAKE_SOURCE_DIR}/partitions.csv" PARTITION_TABLE_CONTENT)
# Require comma immediately after the partition name so descriptive
# comments containing "slave_fw" (e.g. the table-of-contents header) don't
# accidentally match and capture "Offset" from the column-header row.
string(REGEX MATCH "${SLAVE_FW_PARTITION_NAME},[^,]*,[^,]*,[ ]*([^,]*)" PARTITION_MATCH "${PARTITION_TABLE_CONTENT}")
if(PARTITION_MATCH)
    string(REGEX REPLACE "${SLAVE_FW_PARTITION_NAME},[^,]*,[^,]*,[ ]*([^,]*)" "\\1" SLAVE_FW_OFFSET "${PARTITION_MATCH}")
    string(STRIP "${SLAVE_FW_OFFSET}" SLAVE_FW_OFFSET)
    message(STATUS "✅ Partition OTA: Found ${SLAVE_FW_PARTITION_NAME} partition at offset: ${SLAVE_FW_OFFSET}")
else()
    set(SLAVE_FW_OFFSET "0x5F0000")  # Fallback to hardcoded value
    message(WARNING "⚠️ Partition OTA: Could not find ${SLAVE_FW_PARTITION_NAME} in partitions.csv, using fallback offset: ${SLAVE_FW_OFFSET}")
endif()

# --- Step 1: Check if firmware files exist (early validation) ---
message(STATUS "🔍 Partition OTA: Checking for firmware files...")

file(GLOB SLAVE_FW_BIN_FILES "${CMAKE_CURRENT_SOURCE_DIR}/slave_fw_bin/*.bin")

if(SLAVE_FW_BIN_FILES)
    message(STATUS "✅ Partition OTA: Found firmware files for dynamic selection")

    # Get Python and esptool path from IDF
    idf_build_get_property(PYTHON PYTHON)
    idf_build_get_property(ESPTOOL_PATH ESPTOOL_PATH)

    # Create a target that dynamically selects the newest firmware
    add_custom_target(select_partition_firmware
        COMMAND ${CMAKE_COMMAND}
            -DSOURCE_COMPONENT_DIR=${CMAKE_CURRENT_SOURCE_DIR}
            -DBINARY_COMPONENT_DIR=${CMAKE_CURRENT_BINARY_DIR}
            -DCOMPONENT_NAME=Partition OTA
            -DOTA_ACTION=select_for_partition
            -P ${CMAKE_CURRENT_SOURCE_DIR}/../common_ota_scripts/find_newest_firmware.cmake
        COMMENT "Partition OTA: Dynamically selecting newest firmware"
        VERBATIM
    )

    # Create a target that flashes the selected firmware
    add_custom_target(flash_slave_fw
        DEPENDS select_partition_firmware
        COMMAND ${CMAKE_COMMAND}
            -DBINARY_COMPONENT_DIR=${CMAKE_CURRENT_BINARY_DIR}
            -DSLAVE_FW_OFFSET=${SLAVE_FW_OFFSET}
            -DPYTHON=${PYTHON}
            -DCOMPONENT_NAME=Partition OTA
            -P ${CMAKE_CURRENT_SOURCE_DIR}/../common_ota_scripts/flash_selected_firmware.cmake
        COMMENT "Partition OTA: Flashing selected firmware to partition"
        VERBATIM
    )

    # Make flash_slave_fw depend on the main project binary being built successfully
    idf_build_get_property(project_name PROJECT_NAME)
    add_dependencies(flash_slave_fw ${project_name}.elf)

    # Add flash_slave_fw to the flash target only after successful build
    add_dependencies(flash flash_slave_fw)

else()
    message(WARNING "⚠️ Partition OTA: No .bin files found in slave_fw_bin directory. Nothing to flash.")

    # Add warning target for flash command when no slave firmware present
    add_custom_target(warn_no_slave_fw
        COMMAND ${CMAKE_COMMAND} -E echo ""
        COMMAND ${CMAKE_COMMAND} -E echo "⚠️  WARNING: Partition OTA is enabled but no slave firmware found!"
        COMMAND ${CMAKE_COMMAND} -E echo "    Expected: .bin files in components/ota_partition/slave_fw_bin/"
        COMMAND ${CMAKE_COMMAND} -E echo "    Current flash will only flash the host application"
        COMMAND ${CMAKE_COMMAND} -E echo "    The partition at ${SLAVE_FW_OFFSET} will not be flashed this time"
        COMMAND ${CMAKE_COMMAND} -E echo ""
        COMMENT "Warning: No slave firmware binaries found"
        VERBATIM
    )

    # Make flash target show this warning
    add_dependencies(flash warn_no_slave_fw)
endif()
endif()