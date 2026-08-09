@echo off
rem TandemVec FCS CLI launcher (ASCII only - GBK batch parser)
rem Usage: tvc-cli <cli args...>  e.g. tvc-cli --port COM10 param get att_yaw.kp
setlocal
set CLI=%~dp0..\TandemVec_FCS\GCS\server\cli.py
py -3.12 %CLI% %*
