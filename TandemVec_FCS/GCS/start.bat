@echo off
chcp 65001 >nul
rem ============================================================
rem  TandemVec GCS 启动脚本（桌面窗口模式）
rem  双击 → 后端自动启动 + 原生窗口自动打开，关窗即停服务
rem  pywebview 缺失时自动回退浏览器模式
rem ============================================================
cd /d %~dp0
py -3.12 -m pip install -q -r requirements.txt 2>nul
py -3.12 app.py %*
pause
