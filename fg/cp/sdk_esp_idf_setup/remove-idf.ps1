param(
    [Alias("h")]
    [switch]$help
)

### Script to remove ESP-IDF
#
# Delete the ESP-IDF directory, if it exists
#
###


$ESP_IDF_DIR="esp-idf"

function Show-Help {
    Write-Host "Usage: .\remove-idf.ps1 [-help/-h]"
    Write-Host "Remove $ESP_IDF_DIR directory."
    Write-Host ""
    Write-Host "Options:"
    Write-Host "  -help/-h                 Show this help message"
    exit 0
}

if ($help) { Show-Help }

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
