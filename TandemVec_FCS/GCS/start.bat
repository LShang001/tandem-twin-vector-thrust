@echo off
rem ============================================================
rem  TandemVec GCS 启动脚本
rem  浏览器自动打开 http://127.0.0.1:8091/
rem ============================================================
cd /d %~dp0
py -3.12 -m pip install -q -r requirements.txt 2>nul
start "" http://127.0.0.1:8091/
py -3.12 -m uvicorn server.main:app --host 127.0.0.1 --port 8091
pause
