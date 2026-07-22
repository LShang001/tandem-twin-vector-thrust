@echo off
REM ===========================================================================
REM  纵列双发矢量推力飞行器 LaTeX 文档编译脚本
REM  依赖：MiKTeX (xelatex) + 中文支持 (ctex)
REM  用法：双击运行，或在命令行：build.bat
REM ===========================================================================

cd /d "%~dp0"

echo [1/3] xelatex (first pass)...
xelatex -interaction=nonstopmode -synctex=1 main.tex > build.log 2>&1
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: First pass failed. See build.log
    exit /b 1
)

echo [2/3] xelatex (second pass - cross-references)...
xelatex -interaction=nonstopmode main.tex >> build.log 2>&1

echo [3/3] xelatex (third pass - finalize)...
xelatex -interaction=nonstopmode main.tex >> build.log 2>&1

REM 清理辅助文件（保留 PDF 和 .tex 源文件）
del /q main.aux main.log main.out main.toc main.synctex.gz 2>nul

echo.
echo ============================================
echo  编译完成 → main.pdf
echo  日志见 build.log
echo ============================================

REM 可选：打开 PDF
REM start main.pdf
