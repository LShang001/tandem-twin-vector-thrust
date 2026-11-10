# capture_boot.ps1 -- reset the board via openocd and capture the boot log
#
# Usage:
#   powershell -NoProfile -ExecutionPolicy Bypass -File capture_boot.ps1 COM12 [seconds] [readerDir]
#
# Ports: use the USB-C debug port (Serial8/UART8). The DBG console lives on
# Serial6 (telemetry) -- a different COM port entirely.
#
# WHY: `Verified OK` only proves bytes reached flash, not that the firmware
# boots. Always capture one boot log and look for the IMU init + main loop lines.
#
# KEEP THIS FILE PURE ASCII (PowerShell 5.1 reads BOM-less UTF-8 as ANSI/GBK).

$ErrorActionPreference = 'Continue'

$port = if ($args.Count -ge 1) { $args[0] } else { 'COM12' }
$secs = if ($args.Count -ge 2) { $args[1] } else { '14' }
$readerDir = if ($args.Count -ge 3) { $args[2] } else { $env:USERPROFILE }

$ocd = Join-Path $env:USERPROFILE '.platformio\packages\tool-openocd\bin\openocd.exe'
$reader = Join-Path $readerDir 'read_serial.py'
$out = Join-Path $env:USERPROFILE 'boot.log'
$err = Join-Path $env:USERPROFILE 'boot.err'
Remove-Item $out, $err -ErrorAction SilentlyContinue

$pyExe = (Get-Command python -ErrorAction SilentlyContinue).Source
if (-not $pyExe) { Write-Output 'PYTHON_MISSING'; exit 4 }
if (-not (Test-Path $reader)) { Write-Output ('READER_MISSING ' + $reader); exit 5 }

Write-Output "PY=$pyExe"
Write-Output "PORT=$port"

# Start the reader first, then reset, so the whole boot sequence is captured.
$proc = Start-Process -FilePath $pyExe -ArgumentList @('-X', 'utf8', $reader, $port, $secs) `
    -RedirectStandardOutput $out -RedirectStandardError $err -PassThru -NoNewWindow
Write-Output ('READER_PID=' + $proc.Id)

Start-Sleep -Milliseconds 1800

Write-Output '----- reset -----'
& $ocd -f interface/cmsis-dap.cfg -f target/stm32h7x.cfg -c "init" -c "reset run" -c "shutdown" 2>&1 |
    Where-Object { $_ -match 'reset|Error' } | ForEach-Object { $_.ToString() }
Write-Output ('OCD_RESET_EXIT=' + $LASTEXITCODE)

$proc.WaitForExit(30000) | Out-Null
Start-Sleep -Milliseconds 300

Write-Output '----- boot.log -----'
if (Test-Path $out) { Get-Content $out -Raw } else { Write-Output '(no boot.log)' }
Write-Output '----- boot.err -----'
if (Test-Path $err) {
    $e = Get-Content $err -Raw
    if ($e) { $e } else { Write-Output '(empty)' }
} else { Write-Output '(no boot.err)' }
