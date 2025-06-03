#!/usr/bin/env bash
set -e

ESP_IDF_DIR="esp-idf"
FORCE=0
UPDATE_IDF=0

show_help() {
    echo "Usage: ./setup.sh [options]"
    echo ""
    echo "Options:"
    echo "  -f               Force reset and re-clone esp-idf (will delete all local changes)"
    echo "  -u, --update-idf Only update esp-idf to specific commit (resets esp-idf dir only)"
    echo "  -h, --help       Show this help message"
    exit 0
}

# Parse CLI arguments
for arg in "$@"; do
    case "$arg" in
        -f)
            FORCE=1
            ;;
        -u|--update-idf)
            UPDATE_IDF=1
            ;;
        -h|--help)
            show_help
            ;;
        *)
            echo "Unknown option: $arg"
            show_help
            ;;
    esac
done

# Load .env
if [ ! -f .env ]; then
    echo "ERROR: .env file not found!"
    exit 1
fi

source .env

# Check required env variables
if [ -z "$IDF_TAG" ] || [ -z "$IDF_COMMIT" ]; then
    echo "ERROR: IDF_TAG or IDF_COMMIT not defined in .env"
    exit 1
fi

# Check if esp-idf exists and its commit
ESP_IDF_PRESENT=0
ESP_IDF_CORRECT=0

if [ -d "$ESP_IDF_DIR" ]; then
    ESP_IDF_PRESENT=1
    cd "$ESP_IDF_DIR"
    CURRENT_COMMIT=$(git rev-parse HEAD 2>/dev/null || echo "unknown")
    cd ..
    if [ "$CURRENT_COMMIT" == "$IDF_COMMIT" ]; then
        ESP_IDF_CORRECT=1
    fi
fi

# Block if commit mismatch and not -f or -u
if [ $ESP_IDF_PRESENT -eq 1 ] && [ $ESP_IDF_CORRECT -eq 0 ] && [ $FORCE -eq 0 ] && [ $UPDATE_IDF -eq 0 ]; then
    echo "ERROR: esp-idf is at $CURRENT_COMMIT but expected $IDF_COMMIT."
    echo "Use -f to reset or -u to update."
    exit 1
fi

# Handle -f
if [ $FORCE -eq 1 ]; then
    echo "WARNING: This will reset the repo and delete all local changes."
    read -p "Continue? [y/N]: " confirm
    if [[ "$confirm" != "y" && "$confirm" != "Y" ]]; then
        echo "Aborted."
        exit 1
    fi

    echo "Resetting main repo"
    git reset --hard
    rm -rf "$ESP_IDF_DIR"
fi

# Clone esp-idf if not present (for all cases)
if [ ! -d "$ESP_IDF_DIR" ]; then
    echo "ESP hosted: cloning esp-idf at commit $IDF_COMMIT (tag: $IDF_TAG)"
    git clone --branch "$IDF_TAG" --depth 100 https://github.com/espressif/esp-idf.git "$ESP_IDF_DIR"
    cd "$ESP_IDF_DIR"
    git checkout -f "$IDF_COMMIT"
    echo "ESP hosted: applying rom patch"
    git apply ../lib/rom.patch
    echo "ESP hosted: initializing submodules"
    git submodule update --init --depth 1 --recursive
    echo "ESP hosted: installing prerequisites for esp-idf"
    ./install.sh
    cd ..
    ESP_IDF_PRESENT=1
    ESP_IDF_CORRECT=1
fi

# Handle -u
if [ $UPDATE_IDF -eq 1 ]; then
    echo "WARNING: This will reset changes inside esp-idf only."
    read -p "Continue? [y/N]: " confirm
    if [[ "$confirm" != "y" && "$confirm" != "Y" ]]; then
        echo "Aborted."
        exit 1
    fi

    if [ ! -d "$ESP_IDF_DIR" ]; then
        echo "esp-idf not found. Cloning it now..."
        git clone --branch "$IDF_TAG" --depth 100 https://github.com/espressif/esp-idf.git "$ESP_IDF_DIR"
    fi

    echo "Updating esp-idf to commit $IDF_COMMIT"
    cd "$ESP_IDF_DIR"
    git fetch --depth 100 origin "$IDF_TAG"
    git reset --hard "$IDF_COMMIT"
    echo "ESP hosted: applying rom patch"
    git apply ../lib/rom.patch
    git clean -fdx
    echo "ESP hosted: updating submodules"
    git submodule update --init --depth 1 --recursive
    echo "ESP hosted: installing prerequisites for esp-idf"
    ./install.sh
    cd ..

    ESP_IDF_CORRECT=1
    echo "###### Setup Done ######"
    exit 0
fi

echo "ESP hosted: replacing wireless libraries"
mkdir -p "$ESP_IDF_DIR/components/esp_wifi/lib"
rm -rf "$ESP_IDF_DIR/components/esp_wifi/lib/"*
cp -r lib/* "$ESP_IDF_DIR/components/esp_wifi/lib/"

echo "###### Setup Done ######"
