@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0"

if /I "%~1"=="clean" goto clean

echo [1/5] XeLaTeX initial pass...
xelatex -interaction=nonstopmode -halt-on-error -synctex=1 main.tex > build.log 2>&1
if errorlevel 1 goto failed

echo [2/5] BibTeX8 bibliography pass...
bibtex8 main >> build.log 2>&1
if errorlevel 1 goto failed

echo [3/5] XeLaTeX bibliography pass...
xelatex -interaction=nonstopmode -halt-on-error main.tex >> build.log 2>&1
if errorlevel 1 goto failed

echo [4/5] XeLaTeX cross-reference pass...
xelatex -interaction=nonstopmode -halt-on-error main.tex >> build.log 2>&1
if errorlevel 1 goto failed

echo [5/5] XeLaTeX final pass...
xelatex -interaction=nonstopmode -halt-on-error main.tex >> build.log 2>&1
if errorlevel 1 goto failed

findstr /C:"There were undefined citations" /C:"There were undefined references" main.log >nul
if not errorlevel 1 (
  echo ERROR: Undefined citations or references remain. See build.log
  exit /b 1
)

findstr /C:"Missing character" main.log >nul
if not errorlevel 1 (
  echo ERROR: Missing glyphs remain. See build.log
  exit /b 1
)

findstr /C:"Overfull \\hbox" main.log >nul
if not errorlevel 1 (
  echo ERROR: Overfull hbox warnings remain. See build.log
  exit /b 1
)

findstr /C:"Warning--" main.blg >nul
if not errorlevel 1 (
  echo ERROR: BibTeX warnings remain. See build.log
  exit /b 1
)

echo Build complete: main.pdf
echo Log: build.log
exit /b 0

:clean
del /q main.aux main.log main.out main.toc main.synctex.gz main.bbl main.blg 2>nul
echo Auxiliary files removed.
exit /b 0

:failed
echo ERROR: Build failed. See build.log
exit /b 1
