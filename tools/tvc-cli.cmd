@echo off
rem TandemVec FCS CLI launcher (ASCII ONLY - GBK batch parser, no CJK comments!)
rem Usage: tvc-cli <cli args...>  e.g. tvc-cli --port COM10 param get att_yaw.kp
rem 2026-08-11: kill stale cli.py processes before start (Windows serial handle
rem leak on abnormal exit -> COM10 access denied). Ping delay avoids MSYS /t
rem path conversion that breaks "timeout /t".
setlocal
set CLI=%~dp0..\TandemVec_FCS\GCS\server\cli.py

rem Kill stale cli.py processes holding the serial port
powershell -NoProfile -Command "Get-CimInstance Win32_Process -Filter \"Name='python.exe'\" | Where-Object { $_.CommandLine -like '*cli.py*' } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }" 2>nul
if errorlevel 1 goto run
rem wait for port release
ping -n 2 127.0.0.1 >nul

:run
py -3.12 %CLI% %*
