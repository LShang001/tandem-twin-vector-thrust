@echo off
rem py-web-lab 启动脚本（Windows）
cd /d %~dp0
python -m uvicorn main:app --host 127.0.0.1 --port 8090
