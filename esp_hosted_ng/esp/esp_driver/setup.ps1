param(
    [switch]$f,
    [switch]$u,
    [switch]$h
)

function Show-Help {
    Write-Host "Usage: ./setup.ps1 [-f] [-u] [-h]"
    Write-Host ""
    Write-Host "Options:"
    Write-Host "  -f             Force reset and re-clone esp-idf (will delete all local changes)"
    Write-Host "  -u             Only update esp-idf to specific commit (resets esp-idf dir only)"
    Write-Host "  -h             Show this help message"
    exit 0
}

if ($h) { Show-Help }

$envFile = ".env"
if (-not (Test-Path $envFile)) {
    Write-Error ".env file not found!"
    exit 1
}

# Parse .env
Get-Content $envFile | ForEach-Object {
    if ($_ -match '^\s*([^#][^=]*)=(.*)') {
        $name = $matches[1].Trim()
        $value = $matches[2].Trim().Trim('"')
        Set-Item -Path "env:$name" -Value $value
    }
}

if (-not $env:IDF_TAG -or -not $env:IDF_COMMIT) {
    Write-Error "IDF_TAG or IDF_COMMIT not defined in .env"
    exit 1
}

$IDF_TAG = $env:IDF_TAG
$IDF_COMMIT = $env:IDF_COMMIT
$ESP_IDF_DIR = "esp-idf"

# Check current esp-idf status
$ESP_IDF_PRESENT = Test-Path $ESP_IDF_DIR
$ESP_IDF_CORRECT = $false
$CURRENT_COMMIT = ""

if ($ESP_IDF_PRESENT) {
    Push-Location $ESP_IDF_DIR
    try {
        $CURRENT_COMMIT = git rev-parse HEAD 2>$null
        if ($CURRENT_COMMIT -eq $IDF_COMMIT) {
            $ESP_IDF_CORRECT = $true
        }
    } catch {}
    Pop-Location
}

# Commit mismatch but no -f or -u
if ($ESP_IDF_PRESENT -and -not $ESP_IDF_CORRECT -and -not $f -and -not $u) {
    Write-Error "esp-idf is at $CURRENT_COMMIT but expected $IDF_COMMIT. Use -f to reset or -u to update."
    exit 1
}

# Handle -f (full reset)
if ($f) {
    $response = Read-Host "WARNING: This will reset the repo and delete all local changes. Continue? [y/N]"
    if ($response -ne 'y') { Write-Host "Aborted."; exit 1 }

    git reset --hard
    Remove-Item -Recurse -Force $ESP_IDF_DIR -ErrorAction SilentlyContinue
}

# Clone if not present
if (-not (Test-Path $ESP_IDF_DIR)) {
    Write-Host "ESP hosted: cloning esp-idf at commit $IDF_COMMIT (tag: $IDF_TAG)"
    git clone --branch $IDF_TAG --depth 100 https://github.com/espressif/esp-idf.git $ESP_IDF_DIR
    Push-Location $ESP_IDF_DIR
    git checkout -f $IDF_COMMIT
    Write-Host "ESP hosted: initializing submodules"
    git submodule update --init --depth 1 --recursive
    Write-Host "ESP hosted: installing prerequisites for esp-idf"
    .\install.ps1
    Pop-Location
    $ESP_IDF_PRESENT = $true
    $ESP_IDF_CORRECT = $true
}

# Handle -u (update-only)
if ($u) {
    $response = Read-Host "WARNING: This will reset changes inside esp-idf only. Continue? [y/N]"
    if ($response -ne 'y') { Write-Host "Aborted."; exit 1 }

    if (-not (Test-Path $ESP_IDF_DIR)) {
        Write-Host "esp-idf not found. Cloning it now..."
        git clone --branch $IDF_TAG --depth 100 https://github.com/espressif/esp-idf.git $ESP_IDF_DIR
    }

    Push-Location $ESP_IDF_DIR
    git fetch --depth 100 origin $IDF_TAG
    git reset --hard $IDF_COMMIT
    git clean -fdx
    Write-Host "ESP hosted: updating submodules"
    git submodule update --init --depth 1 --recursive
    Write-Host "ESP hosted: installing prerequisites for esp-idf"
    .\install.ps1
    Pop-Location
}

# Copy wireless libs (always after valid clone or update)
Write-Host "ESP hosted: replacing wireless libraries"

$targets = @("esp32", "esp32c2", "esp32c3", "esp32c5", "esp32c6", "esp32s2", "esp32s3")

foreach ($t in $targets) {
    $dest = ".\esp-idf\components\esp_wifi\lib\$t\"
    if (-not (Test-Path $dest)) {
        New-Item -ItemType Directory -Path $dest -Force | Out-Null
    }
    Copy-Item ".\lib\$t\*" $dest -Recurse -Force -ErrorAction SilentlyContinue
}

Write-Host "###### Setup Done ######"
