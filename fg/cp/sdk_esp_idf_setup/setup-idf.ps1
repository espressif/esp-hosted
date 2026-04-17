param(
    [switch]$force,
    [string]$usecommitid,
    [Alias("h")]
    [switch]$help
)

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

$ESP_IDF_DIR="esp-idf"

# IDF branch we are tracking
$ESP_IDF_BRANCH="release/v5.4"

function Show-Help {
    Write-Host "Setup ESP-IDF ($ESP_IDF_BRANCH branch)."
    Write-Host "Default is with latest commit in the branch."
    Write-Host ""
    Write-Host "Usage: .\setup-idf.ps1 [option]"
    Write-Host ""
    Write-Host "Options:"
    Write-Host "  -force                   Force reset ESP-IDF (will delete $ESP_IDF_DIR directory)"
    Write-Host "  -usecommitid <commitID>  Check out ESP-IDF $ESP_IDF_BRANCH with commitID"
    Write-Host "  -help/-h                 Show this help message"
    exit 0
}

if ($help) { Show-Help }

# force reset ESP-IDF if requested
if ($force) {
    if (Test-Path -Path $ESP_IDF_DIR) {
        Write-Host "WARNING: This will delete $ESP_IDF_DIR directory."
        $response = Read-Host "Continue? [y/N]"
        if ($response -ne 'y') { Write-Host "Aborted."; exit 1 }
        Write-Host "Deleting $ESP_IDF_DIR."
        Remove-Item -Path $ESP_IDF_DIR -Recurse -Force
    } else {
        Write-Error "$ESP_IDF_DIR directory does not exists: cannot delete"
        exit 1
    }
}

$newdir = 0

# if $ESP_IDF_DIR does not exist, do git clone
if (-not (Test-Path -Path $ESP_IDF_DIR)) {
    Write-Host "Cloning ESP-IDF $ESP_IDF_BRANCH in $ESP_IDF_DIR"
    git clone --branch "$ESP_IDF_BRANCH" --depth 100 https://github.com/espressif/esp-idf.git "$ESP_IDF_DIR"
    $result = $LASTEXITCODE
    if ($result -ne 0) {
        Write-Error "Failed to clone into $ESP_IDF_DIR"
        exit 1
    }
    # we just created ESP-IDF directory
    $newdir = 1
} else {
    Write-Host "$ESP_IDF_DIR already exists: reusing directory"
}

# checkout with the commitID, but only if the repository is clean
if ($usecommitid) {
    Push-Location $ESP_IDF_DIR
    $output = git status --porcelain --untracked-files=no
    if ($output) {
        Pop-Location
        Write-Host "$ESP_IDF_DIR has changes (modified files)."
        Write-Host "Backup the changes and clean-up the repository first."
        exit 1
    }
    Write-Host "checking out commit ID $usecommitid"
    git checkout $usecommitid
    $result = $LASTEXITCODE
    if ($result -ne 0) {
        Pop-Location
        Write-Error "Failed to check out commit $usecommitid"
        exit 1
    }
    Pop-Location
}

if (($newdir -eq "1") -or ($usecommitid)) {
    Write-Host "Initialising submodules"
    Push-Location $ESP_IDF_DIR
    git submodule update --init --depth 1 --recursive
    $result = $LASTEXITCODE
    if ($result -ne 0) {
        Pop-Location
        Write-Error "Failed to initialise submodules"
        exit 1
    }
    Write-Host "installing prerequisites for esp-idf"
    .\install.ps1
    Pop-Location
}

Write-Host "To complete setup, run:"
Write-Host "    $ESP_IDF_DIR\export.ps1"
