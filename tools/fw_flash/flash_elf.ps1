# flash_elf.ps1 -- burn a revo-built firmware.elf on a flash-node machine
#
# Usage:
#   powershell -NoProfile -ExecutionPolicy Bypass -File flash_elf.ps1 C:\Users\me\fw.elf [projectDir]
#
# WHY THIS FILE EXISTS (2026-11-10, cost three failed flash attempts):
#   openocd evaluates the -c string with TCL. Backslashes inside it are
#   escape sequences, so a Windows path like C:\Users\12631\fw.elf becomes
#   C:UsersV31<FF>w.elf  (\U -> U, \126 -> octal 'V', \f -> form feed) and
#   openocd reports: Error: couldn't open C:UsersV31...elf  + Programming Failed.
#   => ALWAYS pass the path with forward slashes.
#
# KEEP THIS FILE PURE ASCII. Windows PowerShell 5.1 decodes a BOM-less UTF-8
# script as ANSI/GBK; non-ASCII comments turn into garbage and can break
# parsing of the following lines (observed: $elfTcl silently became empty,
# producing -c 'program "" verify reset exit').

$ErrorActionPreference = 'Continue'

if ($args.Count -lt 1) {
    Write-Output 'usage: flash_elf.ps1 <elfPath> [projectDir]'
    exit 1
}

$elfWin = $args[0]
$projDir = if ($args.Count -ge 2) { $args[1] } else { '' }

$ocd = Join-Path $env:USERPROFILE '.platformio\packages\tool-openocd\bin\openocd.exe'

Write-Output "ELF=$elfWin"
if (-not (Test-Path $elfWin)) { Write-Output 'ELF_MISSING'; exit 2 }
Write-Output ('ELF_BYTES=' + (Get-Item $elfWin).Length)

if (-not (Test-Path $ocd)) { Write-Output 'OCD_MISSING'; exit 3 }

if ($projDir -and (Test-Path $projDir)) { Set-Location $projDir }

# TCL sees the -c string: forward slashes only.
$elfTcl = $elfWin -replace '\\', '/'
Write-Output "ELF_TCL=$elfTcl"

$cmd = 'program "' + $elfTcl + '" verify reset exit'
Write-Output "OCD_CMD=$cmd"
Write-Output '----- openocd begin -----'
& $ocd -f interface/cmsis-dap.cfg -f target/stm32h7x.cfg -c $cmd 2>&1 | ForEach-Object { $_.ToString() }
Write-Output '----- openocd end -----'
Write-Output ('OCD_EXIT=' + $LASTEXITCODE)

# Success marker: ** Verified OK ** and OCD_EXIT=0.
# Verified OK only proves the bytes reached flash -- still capture one boot log.
