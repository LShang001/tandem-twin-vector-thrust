@echo off
rem ============================================================
rem  py-web-lab launcher
rem  ? starts Python server in its own window (title: py-web-lab)
rem  ? opens browser at http://127.0.0.1:PORT/ when ready
rem  ? if already running, just opens the browser
rem  ? set PWL_NO_OPEN=1 to skip auto-open
rem  ? set PWL_PORT=xxxx to override port (default 8090)
rem ============================================================
cd /d %~dp0
if not defined PWL_PORT set PWL_PORT=8090

netstat -an | findstr /r ":%PWL_PORT% .*LISTENING" >nul 2>&1
if %errorlevel%==0 goto open

start "py-web-lab" python -m uvicorn main:app --host 127.0.0.1 --port %PWL_PORT%
echo Starting server on port %PWL_PORT%...
set /a tries=0
:wait
timeout /t 1 /nobreak >nul
set /a tries+=1
netstat -an | findstr /r ":%PWL_PORT% .*LISTENING" >nul 2>&1
if %errorlevel%==0 goto open
if %tries% lss 30 goto wait

echo.
echo   [ERROR] port %PWL_PORT% not listening after 30s.
echo   Maybe another program occupies it (check: netstat -ano ^| findstr :%PWL_PORT%).
echo   Retry with a different port:  set PWL_PORT=8091  then run start.bat
pause
exit /b 1

:open
if "%PWL_NO_OPEN%"=="1" goto done
start "" http://127.0.0.1:%PWL_PORT%/

:done
echo.
echo   py-web-lab ready: http://127.0.0.1:%PWL_PORT%/
echo   Server runs in its own window (title: py-web-lab).
echo   Stop it with Ctrl+C there, or use the "close server" button on the page.
