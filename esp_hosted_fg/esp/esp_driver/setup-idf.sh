#!/usr/bin/env bash

### Script to set up ESP-IDF
#
# If -force is given, it will delete the ESP-IDF directory
#
# If ESP-IDF directory does not exist, it will check out ESP-IDF with
# the latest commit from that branch. Else, it will just exit.
#
# If a commit ID is provided by the user, it will check out the branch
# with that commit ID, if the branch is clean
#
###

ESP_IDF_DIR="esp-idf"

# IDF branch we are tracking
ESP_IDF_BRANCH="release/v5.4"

# note: commit id should exist on the tracked IDF branch
COMMIT_ID=0

FORCE=0

show_help() {
    echo "Setup ESP-IDF (${ESP_IDF_BRANCH} branch)."
    echo "Default is with latest commit in the branch."
    echo ""
    echo "Usage: ./setup-idf.sh [option]"
    echo ""
    echo "Options:"
    echo "--force                 Force reset ESP-IDF (will delete all local changes)"
    echo "--use-commit <commitID> Check out ESP-IDF ${ESP_IDF_BRANCH} branch with commitID"
    echo " -h, --help             Show this help message"
    exit 0
}

# Check if esp-idf is present
is_esp_idf_dir_present() {
    if [ -d "${ESP_IDF_DIR}" ]
    then
        echo 1
    else
        echo 0
    fi
}

delete_esp_idf() {
    if [ "${ESP_IDF_PRESENT}" -eq "0" ]
    then
        echo "${ESP_IDF_DIR} directory does not exists: cannot delete"
        echo "Aborted."
        exit 1
    fi
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
}

clone_esp_idf() {
    git clone --branch "${ESP_IDF_BRANCH}" --depth 100 https://github.com/espressif/esp-idf.git "${ESP_IDF_DIR}"
    result=$?
    if [ "${result}" -ne "0" ]
    then
        echo "git clone FAILED. Aborting."
        exit 1
    fi
}

check_out_commit_id() {
    if [ "${COMMIT_ID}" != "0" ]
    then
        cd ${ESP_IDF_DIR}
        output=$(git status --porcelain --untracked-files=no)
        if [ "x${output}" != "x" ]
        then
            echo "$ESP_IDF_DIR has changes (modified files)."
            echo "Backup the changes and clean-up the repository first."
            exit 1
        fi
        echo "Checking out commit ID ${COMMIT_ID}"
        git checkout ${COMMIT_ID}
        result=$?
        cd ..
        if [ "${result}" -ne "0" ]
        then
            echo "git checkout -f ${COMMIT_ID} FAILED. Aborting."
            exit 1
        fi
    fi
}

init_idf_submodules() {
    cd ${ESP_IDF_DIR}
    git submodule update --init --depth 1 --recursive
    result=$?
    cd ..
    if [ "${result}" != "0" ]
    then
        echo "git submodule update FAILED. Aborting."
        exit 1
    fi
}

# Parse CLI arguments
for arg in "$@"; do
    case "$arg" in
        --force)
            FORCE=1
            break;
            ;;
        --use-commit)
            COMMIT_ID="$2"
            break;
            ;;
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

# check for presence of ESP-IDF directory
ESP_IDF_PRESENT=$(is_esp_idf_dir_present)

# delete ESP-IDF directory if requested
if [ "${FORCE}" -ne "0" ]
then
    delete_esp_idf
fi

# recheck for presence of ESP-IDF directory (may have been deleted)
ESP_IDF_PRESENT=$(is_esp_idf_dir_present)

newdir="0"

if [ "${ESP_IDF_PRESENT}" -eq "0" ]
then
    echo "Cloning ESP-IDF \"${ESP_IDF_BRANCH}\" into ${ESP_IDF_DIR}"
    clone_esp_idf
    newdir="1"
else
    echo "${ESP_IDF_DIR} already exists: reusing directory"
fi

# check out commit ID, if provided
check_out_commit_id

if [ "${newdir}" -eq "1" -o "${COMMIT_ID}" != "0" ]
then
    echo "initializing submodules"
    init_idf_submodules

    echo "Installing prerequisites for esp-idf"
    cd ${ESP_IDF_DIR}
    ./install.sh
    result=$?
    cd ..
    if [ "${result}" -ne "0" ]
    then
        echo "esp-idf install.sh FAILED. Aborting."
        exit 1
    fi
fi

echo "To complete setup, run:"
echo "    . ${ESP_IDF_DIR}/export.sh"

exit 0
