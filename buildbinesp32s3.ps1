<#
.SYNOPSIS
    Builds the motionsimbot ESP32-S3 firmware and outputs binaries to rdytoflashbin/

.DESCRIPTION
    Initializes the ESP-IDF v5.5+ environment, sets the target to esp32s3,
    compiles the project with idf.py, and copies all output binaries
    as well as generating a merged factory image in rdytoflashbin/.

    The device number controls the mDNS hostname: motionsimbot<N>.local
    (e.g. device 2 => motionsimbot2.local). Defaults to 1 if not specified.

.PARAMETER DeviceNumber
    The mDNS device number suffix (e.g. 1 => motionsimbot1.local). Defaults to 1.

.EXAMPLE
    .\buildbinesp32s3.ps1           # builds motionsimbot1.local
    .\buildbinesp32s3.ps1 -DeviceNumber 2   # builds motionsimbot2.local
#>

[CmdletBinding()]
param(
    [Parameter(Mandatory=$false)]
    [int]$DeviceNumber = -1
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectDir = Join-Path $ScriptDir "motionsimbot"
$OutputDir  = Join-Path $ScriptDir "rdytoflashbin"

Write-Host "========================================================" -ForegroundColor Cyan
Write-Host "   MotionSimBot ESP32-S3 Firmware Build Pipeline" -ForegroundColor Cyan
Write-Host "========================================================" -ForegroundColor Cyan

# Prompt for device number if not supplied as a parameter
if ($DeviceNumber -lt 1) {
    $input = Read-Host "mDNS device number suffix (default: 1) => motionsimbot<N>.local"
    if ([string]::IsNullOrWhiteSpace($input)) {
        $DeviceNumber = 1
    } else {
        $DeviceNumber = [int]$input
    }
}
Write-Host "--> Building for: motionsimbot$DeviceNumber.local" -ForegroundColor Magenta

# 1. Locate ESP-IDF PowerShell activation profile
$IdfProfile = $null
$CandidateProfiles = @(
    "C:\Espressif\tools\Microsoft.v5.5.4.PowerShell_profile.ps1",
    "C:\Espressif\tools\Microsoft.v6.0.PowerShell_profile.ps1"
)

foreach ($p in $CandidateProfiles) {
    if (Test-Path $p) {
        $IdfProfile = $p
        break
    }
}

if (-not $IdfProfile) {
    Write-Error "Could not find ESP-IDF PowerShell activation script. Checked: $($CandidateProfiles -join ', ')"
    exit 1
}

Write-Host "-> Activating ESP-IDF environment: $IdfProfile" -ForegroundColor Green
$env:PYTHONIOENCODING = "utf-8"
$env:PYTHONUTF8 = "1"
. $IdfProfile

# 2. Enter motionsimbot project directory
Push-Location $ProjectDir
try {
    # If sdkconfig does not exist, set target to esp32s3
    if (-not (Test-Path "sdkconfig")) {
        Write-Host "-> Initializing target esp32s3..." -ForegroundColor Yellow
        idf.py set-target esp32s3
    }

    Write-Host "-> Compiling motionsimbot firmware (DEVICE_NUMBER=$DeviceNumber)..." -ForegroundColor Green
    $DeviceArg = "-DDEVICE_NUMBER=$DeviceNumber"
    idf.py build $DeviceArg
    if ($LASTEXITCODE -ne 0) {
        throw "Firmware build failed with exit code $LASTEXITCODE"
    }

    # 3. Create ready-to-flash output directory
    if (-not (Test-Path $OutputDir)) {
        New-Item -ItemType Directory -Path $OutputDir -Force | Out-Null
    }

    Write-Host "-> Copying binaries to $OutputDir..." -ForegroundColor Green
    Copy-Item "build/bootloader/bootloader.bin"          -Destination "$OutputDir/bootloader.bin" -Force
    Copy-Item "build/partition_table/partition-table.bin" -Destination "$OutputDir/partition-table.bin" -Force
    Copy-Item "build/ota_data_initial.bin"              -Destination "$OutputDir/ota_data_initial.bin" -Force
    Copy-Item "build/motionsimbot.bin"                  -Destination "$OutputDir/motionsimbot.bin" -Force

    # 4. Generate merged 4MB factory image for single-file flashing at 0x0
    Write-Host "-> Generating merged 4MB factory binary (offset 0x0)..." -ForegroundColor Green
    esptool.py --chip esp32s3 merge_bin `
        -o "$OutputDir/motionsimbot_factory_4mb.bin" `
        --flash_mode dio --flash_freq 80m --flash_size 4MB `
        0x0 "$OutputDir/bootloader.bin" `
        0x8000 "$OutputDir/partition-table.bin" `
        0xe000 "$OutputDir/ota_data_initial.bin" `
        0x20000 "$OutputDir/motionsimbot.bin"

    Write-Host "`n[SUCCESS] Build complete! motionsimbot$DeviceNumber.local binaries ready in: $OutputDir" -ForegroundColor Green
    Get-ChildItem -Path $OutputDir | Select-Object Name, Length, LastWriteTime | Format-Table -AutoSize

    Write-Host "To flash to ESP32-S3 run:" -ForegroundColor Yellow
    Write-Host "  .\flashbinesp32s3.ps1 COM9" -ForegroundColor White
    Write-Host "  => Device will appear at: http://motionsimbot$DeviceNumber.local/" -ForegroundColor Cyan
}
finally {
    Pop-Location
}
