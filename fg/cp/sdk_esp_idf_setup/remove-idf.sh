#!/usr/bin/env bash

### Script to remove ESP-IDF
#
# Delete the ESP-IDF directory, if it exists
#
###

ESP_IDF_DIR="esp-idf"

show_help() {
    echo "Usage: .\remove-idf.ps1 [-help/-h]"
    echo "Remove $ESP_IDF_DIR directory."
    echo ""
    echo "Options:"
    echo " -h, --help             Show this help message"
    exit 0
}

# Parse CLI arguments
for arg in "$@"; do
    case "$arg" in
        -h|--help)
            show_help
            break;
            ;;
        *)
            echo "Unknown option: $arg"
            show_help
            break;
            ;;
    esac
done

# Check if esp-idf is present
is_esp_idf_dir_present() {
    if [ -d "${ESP_IDF_DIR}" ]
    then
        echo 1
    else
        echo 0
    fi
}

# check for presence of ESP-IDF directory
ESP_IDF_PRESENT=$(is_esp_idf_dir_present)

if [ "${ESP_IDF_PRESENT}" -eq "1" ]
then
    echo "WARNING: This will delete ${ESP_IDF_DIR} directory."
    read -p "Continue? [y/N]: " confirm
    if [[ "$confirm" != "y" && "$confirm" != "Y" ]]
    then
        echo "Aborted."
        exit 1
    fi

    rm -fr ${ESP_IDF_DIR}
    result=$?
    if [ "${result}" -ne "0" ]
    then
        echo "FAILED to delete ${ESP_IDF_DIR} directory. Aborting."
        exit 1
    fi
else
    echo "${ESP_IDF_DIR} directory does not exists: cannot delete"
    echo "Aborted."
    exit 1
fi
