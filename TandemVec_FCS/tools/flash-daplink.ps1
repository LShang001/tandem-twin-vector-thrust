# flash-daplink.ps1 — reliable DAPLink flash for TandemVec_FCS
# Usage: powershell -ExecutionPolicy Bypass -File tools\flash-daplink.ps1 [-Elf <path>]
# Why: CMSIS-DAP at high SWD speed intermittently reports checksum mismatch on this
# board (isolated bit flips). 20-100 kHz + verify/retry is reliable. OpenOCD
# `program` leaves the target halted, so always finish with `reset run`.
param(
  [string]$Elf = "",
  [int[]]$Speeds = @(20, 50, 100)
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
if ([string]::IsNullOrWhiteSpace($Elf)) {
  $Elf = Join-Path $root ".pio\build\TandemVec_FCS\firmware.elf"
}
if (-not (Test-Path $Elf)) {
  throw "ELF not found: $Elf (run 'pio run -e TandemVec_FCS' first)"
}

$ocd = Join-Path $env:USERPROFILE ".platformio\packages\tool-openocd\bin\openocd.exe"
if (-not (Test-Path $ocd)) {
  throw "OpenOCD not found: $ocd"
}

Write-Host "[flash] ELF  = $Elf"
Write-Host "[flash] OCD  = $ocd"

foreach ($speed in $Speeds) {
  Write-Host "[flash] trying adapter speed ${speed} kHz ..."
  $ocdArgs = @(
    "-f", "interface/cmsis-dap.cfg",
    "-f", "target/stm32h7x.cfg",
    "-c", "init; adapter speed $speed; reset halt; program `"$Elf`" verify; reset run; shutdown"
  )
  $out = & $ocd @ocdArgs 2>&1 | Out-String
  $out | Write-Host
  if ($out -match "Verified OK" -and $out -notmatch "Verify Failed") {
    Write-Host "[flash] SUCCESS at ${speed} kHz"
    exit 0
  }
}

Write-Host "[flash] all speeds failed."
Write-Host "[flash] If OpenOCD reports CMSIS-DAP CMD_INFO/command mismatch:"
Write-Host "        physically unplug/replug the DAPLink USB (software re-enumeration is not enough)."
exit 2
