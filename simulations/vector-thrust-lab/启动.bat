@echo off
chcp 65001 >nul
cd /d "%~dp0"
rem Tandem Vec VTOL simulation lab launcher
rem (ASCII only: cmd parses batch with process-start codepage, chcp at
rem runtime does not fix UTF-8 content in the file itself)
echo TandemVec simulation lab - starting http server...
start http://localhost:8080
python -m http.server 8080
pause
