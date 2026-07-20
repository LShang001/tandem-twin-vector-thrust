@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo 🚀 纵列双发矢量推力飞行器 · 启动中...
start http://localhost:8080
python -m http.server 8080
pause
