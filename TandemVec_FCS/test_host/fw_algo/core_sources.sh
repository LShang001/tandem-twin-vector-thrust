# shellcheck shell=bash
# 固件核心源清单（**唯一**）：SIL 可执行与宿主直调共享库都从这里取源。
#
# 新增固件模块（src/*.cpp 或替换 lib 实现）时只改本文件，然后：
#   1) bash test_host/fw_algo/build_fw_algo.sh      # 共享库
#   2) bash test_host/fw_sil/build_incremental.sh   # SIL 可执行
# 两侧必须同时重建；只改一侧会让"仿真用的算法"与"固件烧的算法"分叉。
#
# 说明：本清单只含两目标**共用**的固件核心；各自特有文件（SIL 主循环、
# fw_algo 的 ABI 封装、ano_params 注册表）由各构建脚本自行追加。
FW_CORE_SOURCES="
src/flight_control.cpp
src/navigation_task.cpp
src/state_data.cpp
lib/ublox-main/src/ubx.cpp
test_host/fw_sil/fw_sil_stubs.cpp
lib/DPS310-Pressure-Sensor/src/Dps310.cpp
lib/DPS310-Pressure-Sensor/src/DpsClass.cpp
lib/units/src/convang.cpp
lib/units/src/convangacc.cpp
lib/units/src/convangvel.cpp
lib/MadgwickAHRS-master/src/MadgwickAHRS.cpp
"
FW_CORE_C_SOURCES="lib/ublox-main/src/minmea.c"
