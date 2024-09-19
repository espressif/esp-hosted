#!/usr/bin/env bash

SHELL_RC="$HOME/.bashrc"

# Step 1: Check if curl and git are installed
echo "============== Step 1: Checking if dependencies are installed =============="
if ! command -v curl &> /dev/null; then
    echo "curl is not installed. Please install curl."
    exit 1
fi

if ! command -v git &> /dev/null; then
    echo "git is not installed. Please install git."
    exit 1
fi

echo "All required dependencies are installed."

# Step 2: Fetch the branches from the GitHub API and find the latest stable release branch
echo "============== Step 2: Fetching branch list from ESP-IDF GitHub API =============="
LATEST_BRANCH=$(curl -s https://api.github.com/repos/espressif/esp-idf/branches | grep -o '"name": "release/[^"]*' | awk -F'"' '{print $4}' | sort -V | tail -n 1)
echo "Latest stable branch found: $LATEST_BRANCH"

# Step 3: Clone or update the ESP-IDF repository
if [ ! -d "$HOME/esp-idf" ]; then
	echo "========= Step 3: Cloning the ESP-IDF repository (takes 3-4 mins) ============="
    git clone -b "$LATEST_BRANCH" --recursive --depth 1 https://github.com/espressif/esp-idf.git "$HOME/esp-idf"
else
    echo "ESP-IDF repository already exists. Updating..."
    cd "$HOME/esp-idf" || exit
    git checkout "$LATEST_BRANCH"
    git pull --recurse-submodules
fi

# Log the current commit hash
cd "$HOME/esp-idf" || exit
IDF_COMMIT=$(git rev-parse HEAD)
echo "<< ESP-IDF is set to commit: $IDF_COMMIT >>"

# Step 4: Set up the ESP-IDF environment
echo "============== Step 4: Setting up the ESP-IDF environment ================"
"$HOME/esp-idf/install.sh"
source "$HOME/esp-idf/export.sh"

# Step 5: Optionally add an alias to shell configuration for easy setup
echo "============== Step 5: Adding alias to shell configuration ==============="
if ! grep -q "alias get-idf" "$SHELL_RC"; then
    read -p "Do you want to add the alias 'get-idf' to $SHELL_RC? [yes/no] " -r
    if [[ $REPLY =~ ^[Yy][Ee][Ss]$ ]]; then
        echo "alias get-idf='source $HOME/esp-idf/export.sh'" >> "$SHELL_RC"
        echo "ESP-IDF setup alias added to $SHELL_RC. Run 'get-idf' to configure your environment."
    else
        echo "Alias not added. You can manually add 'alias get-idf=\"source $HOME/esp-idf/export.sh\"' to $SHELL_RC."
    fi
else
    echo "ESP-IDF setup alias already exists in $SHELL_RC."
fi

# Step 6: Inform the user to reload the shell
echo "============== Step 6: Informing user to reload shell ==============="
echo "\nPlease run 'source $SHELL_RC' to reload the shell with the new alias."
echo "\nIn a new shell, run 'get-idf' to enable the ESP-IDF environment."
