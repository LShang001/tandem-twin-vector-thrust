@echo off
rem TandemVec FCS CLI launcher (ASCII only - GBK batch parser)
rem Usage: tvc-cli <cli args...>  e.g. tvc-cli --port COM10 param get att_yaw.kp
rem ★ 2026-08-11 串口问题根治：启动前清理残留 cli.py 进程（异常退出/被强杀时
rem   Windows 串口句柄残留 → COM10 拒绝访问；kill 旧进程后端口释放）
setlocal
set CLI=%~dp0..\TandemVec_FCS\GCS\server\cli.py

rem Kill stale cli.py processes holding the serial port
powershell -NoProfile -Command "Get-CimInstance Win32_Process -Filter \"Name='python.exe'\" | Where-Object { $_.CommandLine -like '*cli.py*' } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }" 2>nul
if errorlevel 1 goto run
rem wait for port release (ping delay avoids MSYS /t path conversion of timeout cmd)
ping -n 2 127.0.0.1 >nul

:run
py -3.12 %CLI% %*
