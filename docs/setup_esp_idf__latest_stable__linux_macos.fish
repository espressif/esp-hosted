#!/usr/bin/env fish

echo "Setting up ESP-IDF using fish"
set SHELL_RC "$HOME/.config/fish/config.fish"

# Step 1: Check if curl and git are installed
echo "============== Step 1: Checking if dependencies =========================="
if not type -q curl
    echo "curl is not installed. Please install curl."
    exit 1
end

if not type -q git
    echo "git is not installed. Please install git."
    exit 1
end

echo "Dependencies for current script are installed."

# Step 2: Fetch the branches from the GitHub API and find the latest stable release branch
echo "============== Step 2: Fetching branch list from ESP-IDF GitHub API =============="
set LATEST_BRANCH (curl -s https://api.github.com/repos/espressif/esp-idf/branches | grep -o '"name": "release/[^"]*' | awk -F'"' '{print $4}' | sort -V | tail -n 1)

# Log the latest branch found
echo "Latest stable branch found: $LATEST_BRANCH"

# Step 3: Clone or update the ESP-IDF repository
if not test -d "$HOME/esp-idf"
    echo "============== Step 3: Cloning the ESP-IDF repository ================"
    git clone -b "$LATEST_BRANCH" --recursive --depth 1 https://github.com/espressif/esp-idf.git "$HOME/esp-idf"
else
    echo "ESP-IDF repository already exists."
    cd "$HOME/esp-idf" || exit
    git checkout "$LATEST_BRANCH"
    git pull --recurse-submodules
end

cd "$HOME/esp-idf" || exit
set IDF_COMMIT (git rev-parse HEAD)
echo "ESP-IDF is set to commit: $IDF_COMMIT"

# Step 4: Set up the ESP-IDF environment
echo "============== Step 4: Setting up the ESP-IDF environment ================"
"$HOME/esp-idf/install.fish"
source "$HOME/esp-idf/export.fish" | source

# Step 5: Add alias to Fish config
echo "============== Step 5: Adding alias to Fish configuration ==============="
if not grep -q "alias get-idf" "$SHELL_RC"
    read -P "Do you want to add the alias 'get-idf' to $SHELL_RC? [yes/no] " response
    if string match -i -r '^yes$' $response
        echo "alias get-idf='source $HOME/esp-idf/export.fish | source'" >> "$SHELL_RC"
        echo "ESP-IDF setup alias added to $SHELL_RC. Run 'get-idf' to configure your environment."
    else
        echo "Alias not added. You can manually add 'alias get-idf=\". $HOME/esp-idf/export.fish | source\"' to $SHELL_RC."
    end
else
    echo "ESP-IDF setup alias already exists in $SHELL_RC."
end

# Step 6: Inform user to reload shell
echo "============== Step 6: Informing user to reload shell ==============="
echo "Please run 'source $SHELL_RC' to reload the shell with the new alias."
echo "In new shell, run 'get-idf' to enable ESP-IDF environment."

