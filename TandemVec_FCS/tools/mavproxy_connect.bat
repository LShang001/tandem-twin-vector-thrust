@echo off
REM ============================================================
REM  MAVProxy 一键连接（需要在真实控制台窗口里运行）
REM  原因：MAVProxy 的 prompt_toolkit 强制要求 stdout 是控制台，
REM        SSH / 重定向 / 管道都会报 NoConsoleScreenBufferError。
REM  用法：双击本文件，或在 cmd 窗口里执行。
REM ============================================================
setlocal
set PY=%USERPROFILE%\.platformio\penv\Scripts\python.exe
set MP=%USERPROFILE%\.platformio\penv\Scripts\mavproxy.py

echo [1/2] 切换飞控到 MAVLink 模式...
"%PY%" -c "import serial,time; s=serial.Serial('COM6',921600,timeout=0.1); time.sleep(2); s.reset_input_buffer(); s.write(b'DBG\n'); time.sleep(1.5); s.read(1024); s.write(b'proto mavlink\n'); time.sleep(1.2); s.read(512); s.write(b'exit\n'); time.sleep(1.5); s.close(); print('  OK')"

echo [2/2] 启动 MAVProxy（连 COM6 @ 921600）...
echo.
echo   常用命令：
echo     status                 查看状态
echo     param show att_roll    看参数
echo     param set att_roll.kp 5.0   改参数
echo     arm throttle           尝试解锁（会被拒绝并说明原因）
echo     mode guided            尝试切模式（会被拒绝）
echo     exit                   退出
echo.
"%PY%" "%MP%" --master=COM6 --baudrate=921600

endlocal
pause
