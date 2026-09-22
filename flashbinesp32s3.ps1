<#
.SYNOPSIS
    Flashes the MotionSimBot ESP32-S3 firmware binaries from rdytoflashbin/

.DESCRIPTION
    Flashes the pre-built binaries or merged factory image in rdytoflashbin/
    to the target ESP32-S3 over the specified COM port.

.PARAMETER Port
    The COM port of the connected ESP32-S3 (e.g. COM9, COM5).

.EXAMPLE
    .\flashbinesp32s3.ps1 COM9
    .\flashbinesp32s3.ps1 COM5
#>

[CmdletBinding()]
param(
    [Parameter(Position = 0, Mandatory = $false)]
    [string]$Port
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$BinDir    = Join-Path $ScriptDir "rdytoflashbin"

Write-Host "========================================================" -ForegroundColor Cyan
Write-Host "   MotionSimBot ESP32-S3 Firmware Flashing Tool" -ForegroundColor Cyan
Write-Host "========================================================" -ForegroundColor Cyan

# 1. Detect or validate COM port
if (-not $Port) {
    try {
        $AvailablePorts = [System.IO.Ports.SerialPort]::GetPortNames()
    } catch {
        $AvailablePorts = @()
    }

    if ($AvailablePorts.Count -eq 1) {
        $Port = $AvailablePorts[0]
        Write-Host "-> Auto-detected COM Port: $Port" -ForegroundColor Green
    } elseif ($AvailablePorts.Count -gt 1) {
        Write-Host "Multiple COM ports detected: $($AvailablePorts -join ', ')" -ForegroundColor Yellow
        $Port = Read-Host "Enter COM port to flash (e.g. $($AvailablePorts[0]))"
    } else {
        $Port = Read-Host "Enter COM port to flash (e.g. COM9)"
    }
}

if (-not $Port) {
    Write-Error "No COM port specified. Usage: .\flashbinesp32s3.ps1 COM5"
    exit 1
}

$Port = $Port.ToUpper().Trim()
Write-Host "-> Target Port: $Port" -ForegroundColor Green

# 2. Check for binary files in rdytoflashbin/
if (-not (Test-Path $BinDir)) {
    Write-Error "Directory '$BinDir' does not exist! Please run .\buildbinesp32s3.ps1 first."
    exit 1
}

$MergedBin = Join-Path $BinDir "motionsimbot_factory_4mb.bin"
$AppBin    = Join-Path $BinDir "motionsimbot.bin"
$BootBin   = Join-Path $BinDir "bootloader.bin"
$PartBin   = Join-Path $BinDir "partition-table.bin"
$OtaBin    = Join-Path $BinDir "ota_data_initial.bin"

# 3. Locate ESP-IDF PowerShell profile or Python venv
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

if ($IdfProfile) {
    Write-Host "-> Activating ESP-IDF environment: $IdfProfile" -ForegroundColor DarkGray
    $env:PYTHONIOENCODING = "utf-8"
    $env:PYTHONUTF8 = "1"
    . $IdfProfile
}

# 4. Flash binary
if (Test-Path $MergedBin) {
    Write-Host "-> Flashing merged factory image ($MergedBin) at 0x0..." -ForegroundColor Cyan
    esptool.py --chip esp32s3 -p $Port -b 460800 --before=default_reset --after=hard_reset write_flash --flash_mode dio --flash_freq 80m --flash_size 4MB 0x0 $MergedBin
} elseif (Test-Path $AppBin) {
    Write-Host "-> Flashing multi-partition binaries..." -ForegroundColor Cyan
    esptool.py --chip esp32s3 -p $Port -b 460800 --before=default_reset --after=hard_reset write_flash --flash_mode dio --flash_freq 80m --flash_size 4MB `
        0x0 $BootBin `
        0x8000 $PartBin `
        0xe000 $OtaBin `
        0x20000 $AppBin
} else {
    Write-Error "No valid binaries found in $BinDir! Please run .\buildbinesp32s3.ps1 first."
    exit 1
}

if ($LASTEXITCODE -eq 0) {
    Write-Host "`n[SUCCESS] Flashed MotionSimBot successfully to $Port!" -ForegroundColor Green
} else {
    Write-Host "`n[ERROR] Flashing failed with exit code $LASTEXITCODE" -ForegroundColor Red
}
