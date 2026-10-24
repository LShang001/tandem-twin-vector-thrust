#!/usr/bin/env bash
# 宿主机平台无关算法回归测试运行器
# 用途：在不依赖 STM32 硬件的前提下，用 g++ 编译运行 include/ 下三个平台无关算法的回归测试。
# 对应 AGENTS.md「修改这三个文件后，可以也应该在宿主机上用 g++ 单独验证」。
#
# 用法（在仓库根目录）：
#   bash test_host/run_all.sh
# 退出码：0 表示全部通过，非 0 表示有失败项。
set -u

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# 输出目录可用环境变量覆盖：MSYS/bash 下 g++ 无法写中文路径（Invalid argument），
# PowerShell 环境请设 BIN_DIR 到英文路径（如 $env:TEMP\tv_fcs_bin）。
BIN_DIR="${BIN_DIR:-$ROOT/test_host/bin}"
mkdir -p "$BIN_DIR"

INCLUDES="-I$ROOT/include -I$ROOT/lib/eigen/src"
STUB_INCLUDE="-I$ROOT/test_host/stub"
CXX=${CXX:-g++}
STD=${STD:-c++17}
EXE_SUFFIX=""
case "$(uname -s)" in
  MINGW*|MSYS*|CYGWIN*) EXE_SUFFIX=".exe" ;;
esac

fail=0

run_one() {
  local src="$1" name="$2" extra_includes="$3"
  local out="$BIN_DIR/$name$EXE_SUFFIX"
  echo "---- 编译 $name ----"
  if ! $CXX -std=$STD $INCLUDES $extra_includes "$src" -o "$out" 2>&1; then
    echo "[BUILD FAIL] $name"
    fail=1
    return
  fi
  echo "---- 运行 $name ----"
  if ! "$out"; then
    echo "[RUN FAIL] $name"
    fail=1
  fi
  echo
}

# 平台无关算法（include/ins_*.h，不依赖 Arduino）：
run_one "$ROOT/test_host/test_gnss_dynamic_weight.cpp" gdw ""
run_one "$ROOT/test_host/test_static_aid_profile.cpp"  sap ""
# 执行器故障监测（ActuatorFaultMonitor.h，纯平台无关——舵机卡死/单发推力损失
# 的“误差+饱和+持续时间”检测与动作建议；F3/F4 故障矩阵的固件侧最小模块）：
run_one "$ROOT/test_host/test_actuator_fault_monitor.cpp" afm ""
run_one "$ROOT/test_host/test_static_detector.cpp"     sd  ""
run_one "$ROOT/test_host/test_imu_health_monitor.cpp"  imuhealth ""
run_one "$ROOT/test_host/test_gnss_consistency.cpp"    gnssc ""
run_one "$ROOT/test_host/test_vehicle_profile.cpp"     vp  ""
run_one "$ROOT/test_host/test_generic_allocation.cpp"  ga  ""
run_one "$ROOT/test_host/test_altitude_reference.cpp"  ar  ""
run_one "$ROOT/test_host/test_gnss_epoch_timing.cpp"    get ""
run_one "$ROOT/test_host/test_gnss_pps_latency.cpp"      pps ""
run_one "$ROOT/test_host/test_gnss_delay_batch.cpp"      bdelay ""
run_one "$ROOT/test_host/test_gnss_delay_auto.cpp"       dauto ""
run_one "$ROOT/test_host/test_ins_vibration_adaptive.cpp" vib ""
run_one "$ROOT/test_host/test_bias_temp_observer.cpp" btemp ""
run_one "$ROOT/test_host/test_bias_temp_table.cpp" btable ""
run_one "$ROOT/test_host/test_bias_temp_pipeline.cpp" btpipe ""

# 通用变量注册表 AnoVars（include/AnoVars.h，纯平台无关——注册表遍历/
# 各类型→float 序列化/名称截断/watch 集合上限/rate 钳制/越界容错）：
run_one "$ROOT/test_host/test_ano_vars.cpp"            av  ""

# FPV 摇杆曲线 RcCurve（include/RcCurve.h，纯平台无关——expo 中心压低/super 边缘
# 双曲放大/rc_rate 线性缩放/死区/限幅/极点保护，Betaflight 三参数模型）：
run_one "$ROOT/test_host/test_rc_curve.cpp"            rc  ""

# 控制器角度域与物理 SI 域边界（deg/s² ↔ rad/s²），含旧/新 PID 动态等价性：
run_one "$ROOT/test_host/test_control_units.cpp"        cu  "$STUB_INCLUDE"

# AnoComProtocol 打包回归（★2026-08-10 COMM-001：buildFrame 长度保护/校验
# 单循环边拷边算/组模式合并 write 字节流等价。多文件编译 + stub Stream）：
echo "---- 编译 ap ----"
if ! $CXX -std=$STD $INCLUDES $STUB_INCLUDE -I$ROOT/lib/AnoComProtocol \
     "$ROOT/test_host/test_anocom_packing.cpp" \
     "$ROOT/lib/AnoComProtocol/AnoComProtocol.cpp" \
     -o "$BIN_DIR/ap$EXE_SUFFIX" 2>&1; then
  echo "[BUILD FAIL] ap"
  fail=1
else
  echo "---- 运行 ap ----"
  if ! "$BIN_DIR/ap$EXE_SUFFIX"; then
    echo "[RUN FAIL] ap"
    fail=1
  fi
  echo
fi

# 磁力计解析/轴对齐数学层（lib/MagSensor/MagMath.h 纯平台无关——
# IST8310/QMC5883L 灵敏度与解码、芯片系→FRD 旋转矩阵、TiltCompass 北向判据）：
run_one "$ROOT/test_host/test_mag_parse.cpp"           mp  "-I$ROOT/lib/MagSensor -I$ROOT/lib/navigation-main/src"

# 磁力计椭球标定数学层（lib/MagCal/MagCalMath.h 纯平台无关——
# CRC32/Jacobi/线性代数工具/合成椭球拟合恢复精度/质量门拒绝路径）：
run_one "$ROOT/test_host/test_mag_cal.cpp"             mc  "-I$ROOT/lib/MagCal"

# 大疆式两圈引导校准状态机（lib/MagCal/MagCalGuide.h 纯平台无关——
# 姿态/旋转/扇区覆盖判定、两阶段自动切换、门限拒绝路径）：
run_one "$ROOT/test_host/test_mag_guide.cpp"          mg  "-I$ROOT/lib/MagCal"
# 磁参考场/测量门控（2026-10 生产磁链路）：纯头文件 + Eigen
run_one "$ROOT/test_host/test_mag_reference.cpp"      mref "-I$ROOT/lib/navigation-main/src"
run_one "$ROOT/test_host/test_mag_gate.cpp"           mgate ""

# 平台相关算法（依赖 Arduino.h，通过 test_host/stub/Arduino.h 桩在宿主机编译）：
run_one "$ROOT/test_host/test_vertical_kf.cpp"         vkf "$STUB_INCLUDE"

# WS2812 RGB 灯效状态机（FcsLedState 模式/颜色/节奏/BOOT 序列，
# 通过 stub 的 WS2812Driver mock 记录像素断言）：
run_one "$ROOT/test_host/test_led_status.cpp"          led "$STUB_INCLUDE"

# 纵列双发矢量推力飞行器控制分配（纯平台无关头文件，无 Arduino 依赖）：
run_one "$ROOT/test_host/test_tandemvec_allocation.cpp" ta ""

# ★2026-08-10 AirframeModel 通用框架四旋翼 demo（换机型=填数据，验证跨机型）：
run_one "$ROOT/test_host/test_airframe_generic.cpp"       ag ""

# 惯量逆解交叉耦合前馈（★2026-08-10 通用层，ω×(I·ω)+ω×h，仿真同构；
# 含 ω×h 符号回归与理想刚体闭环对照）：
run_one "$ROOT/test_host/test_inertia_decoupling.cpp"    id ""

# flight_control.cpp 姿态环闭环仿真（轴序 FRD 修复验证）：
# 按源码公式独立实现，驱动刚体角动力学 + 推进力矩闭环
run_one "$ROOT/test_host/test_flight_control_axis.cpp"  fca ""

# PositionPID v3 回归测试（无 Arduino 依赖，纯平台无关）：
run_one "$ROOT/test_host/test_position_pid.cpp" pp ""

# 在线辨识（RLS + 自适应增益调度），纯平台无关：
run_one "$ROOT/test_host/test_online_id.cpp"            oi ""

# 全阶闭环仿真与灵敏度分析。依赖 PositionPID / ComplementaryFilter
# （后者 include Arduino.h），故需 stub：
run_one "$ROOT/test_host/test_advanced_theory.cpp"      at  "$STUB_INCLUDE"
run_one "$ROOT/test_host/test_robustness.cpp"           rb  "$STUB_INCLUDE"
run_one "$ROOT/test_host/test_qual_analysis.cpp"        qa  "$STUB_INCLUDE"
run_one "$ROOT/test_host/test_comprehensive_sim.cpp"    cs  "$STUB_INCLUDE"

# 15 状态 EKF 宿主回归（ekf_15_state.h 为 header-only，但依赖 bfs::convang 系列
# 与 src/ 下的快速协方差块；需 -DEKF_HOST_REGRESSION 打开测试接口、-O2 控制
# 模板展开段数）：
echo "---- 编译 ekf15 ----"
if ! $CXX -std=$STD -O2 -DEKF_HOST_REGRESSION -D_USE_MATH_DEFINES \
     $INCLUDES -I$ROOT/src -I$ROOT/lib/navigation-main/src -I$ROOT/lib/units/src \
     "$ROOT/test_host/test_ekf_15state.cpp" \
     "$ROOT/lib/units/src/convang.cpp" \
     "$ROOT/lib/units/src/convangacc.cpp" \
     "$ROOT/lib/units/src/convangvel.cpp" \
     -o "$BIN_DIR/ekf15$EXE_SUFFIX" 2>&1; then
  echo "[BUILD FAIL] ekf15"
  fail=1
else
  echo "---- 运行 ekf15 ----"
  if ! "$BIN_DIR/ekf15$EXE_SUFFIX"; then
    echo "[RUN FAIL] ekf15"
    fail=1
  fi
  echo
fi

# EKF 测速源辅助接口回归（aiding 速度更新豁免零速守卫 + 失锁重锚，
# 2026-08-21 室内光流速度融合配套；依赖同 ekf15）：
echo "---- 编译 aidvel ----"
if ! $CXX -std=$STD -O2 -DEKF_HOST_REGRESSION -D_USE_MATH_DEFINES \
     $INCLUDES -I$ROOT/src -I$ROOT/lib/navigation-main/src -I$ROOT/lib/units/src \
     "$ROOT/test_host/test_ekf_aiding_velocity.cpp" \
     "$ROOT/lib/units/src/convang.cpp" \
     "$ROOT/lib/units/src/convangacc.cpp" \
     "$ROOT/lib/units/src/convangvel.cpp" \
     -o "$BIN_DIR/aidvel$EXE_SUFFIX" 2>&1; then
  echo "[BUILD FAIL] aidvel"
  fail=1
else
  echo "---- 运行 aidvel ----"
  if ! "$BIN_DIR/aidvel$EXE_SUFFIX"; then
    echo "[RUN FAIL] aidvel"
    fail=1
  fi
  echo
fi


# 高速 GNSS/INS 延时补偿回归（30 m/s、50 ms、5 Hz；年龄补偿应显著优于无补偿）：
echo "---- 编译 delayhs ----"
if ! $CXX -std=$STD -O2 -DEKF_HOST_REGRESSION -D_USE_MATH_DEFINES      $INCLUDES -I$ROOT/src -I$ROOT/lib/navigation-main/src -I$ROOT/lib/units/src      "$ROOT/test_host/test_ekf_delay_highspeed.cpp"      "$ROOT/lib/units/src/convang.cpp"      "$ROOT/lib/units/src/convangacc.cpp"      "$ROOT/lib/units/src/convangvel.cpp"      -o "$BIN_DIR/delayhs$EXE_SUFFIX" 2>&1; then
  echo "[BUILD FAIL] delayhs"
  fail=1
else
  echo "---- 运行 delayhs ----"
  if ! "$BIN_DIR/delayhs$EXE_SUFFIX"; then
    echo "[RUN FAIL] delayhs"
    fail=1
  fi
  echo
fi

# 传感器标称精度（ICM-42688/UBX PVT）+ 高速延时补偿：默认参数应不劣于无补偿：
echo "---- 编译 hsreal ----"
if ! $CXX -std=$STD -O2 -DEKF_HOST_REGRESSION -D_USE_MATH_DEFINES      $INCLUDES -I$ROOT/src -I$ROOT/lib/navigation-main/src -I$ROOT/lib/units/src      "$ROOT/test_host/test_ekf_highspeed_realistic.cpp"      "$ROOT/lib/units/src/convang.cpp"      "$ROOT/lib/units/src/convangacc.cpp"      "$ROOT/lib/units/src/convangvel.cpp"      -o "$BIN_DIR/hsreal$EXE_SUFFIX" 2>&1; then
  echo "[BUILD FAIL] hsreal"
  fail=1
else
  echo "---- 运行 hsreal ----"
  if ! "$BIN_DIR/hsreal$EXE_SUFFIX"; then
    echo "[RUN FAIL] hsreal"
    fail=1
  fi
  echo
fi

if [ "$fail" -eq 0 ]; then
  echo "=== 全部宿主机回归测试通过 ==="
  exit 0
else
  echo "=== 存在失败项 ==="
  exit 1
fi
