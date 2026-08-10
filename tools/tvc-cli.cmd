@echo off
chcp 65001 >nul
rem ============================================================
rem  TandemVec FCS CLI launcher
rem  Encoding policy (2026-08-11): cmd parses batch files with the
rem  process-start codepage; runtime "chcp" does NOT affect parsing
rem  of the file itself -> comments/commands must stay ASCII.
rem  chcp 65001 is kept for echo/Python stdout (UTF-8); PYTHONUTF8=1
rem  forces Python UTF-8 output in both console and pipe contexts.
rem  Usage: tvc-cli <cli args...>  e.g. --port COM10 param get att_yaw.kp
rem  Kills stale cli.py processes first (Windows serial handle leak on
rem  abnormal exit -> COM10 access denied).
rem ============================================================
setlocal
set CLI=%~dp0..\TandemVec_FCS\GCS\server\cli.py
set PYTHONUTF8=1

rem Kill stale cli.py processes holding the serial port
powershell -NoProfile -Command "Get-CimInstance Win32_Process -Filter \"Name='python.exe'\" | Where-Object { $_.CommandLine -like '*cli.py*' } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }" 2>nul
if errorlevel 1 goto run
rem wait for port release (ping delay avoids MSYS /t path conversion)
ping -n 2 127.0.0.1 >nul

:run
py -3.12 %CLI% %*
