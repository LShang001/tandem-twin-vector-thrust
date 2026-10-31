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
CC=${CC:-gcc}
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
run_one "$ROOT/test_host/test_ins_prelaunch_align.cpp" pla ""
run_one "$ROOT/test_host/test_ins_ahrs_gravity_gate.cpp" ahg ""
# IMU 滤波参数（ICM-42688-P AAF 表/notch 公式，纯头文件无 Arduino 依赖）：
run_one "$ROOT/test_host/test_imu_filter_math.cpp" imufilter ""
# ICM-45686 配置默认值/WOM 阈值换算（需 fw_sil 的 Arduino SPI/Wire 桩）：
run_one "$ROOT/test_host/test_icm45686_config.cpp" icm45686cfg "-I$ROOT/test_host/fw_sil/stub -I$ROOT/lib/ICM45686/src -I$ROOT/lib/ICM45686/src/imu"
# 弹载减震座传递率（SIL 研究设施，纯标量公式）：
run_one "$ROOT/test_host/test_sil_vibration_isolator.cpp" iso "-I$ROOT/test_host/fw_sil"
run_one "$ROOT/test_host/test_imu_health_monitor.cpp"  imuhealth ""
run_one "$ROOT/test_host/test_ins_imu_saturation.cpp"   satq ""
run_one "$ROOT/test_host/test_ins_baro_health.cpp"      baroh ""
run_one "$ROOT/test_host/test_gnss_consistency.cpp"    gnssc ""
run_one "$ROOT/test_host/test_vehicle_profile.cpp"     vp  ""
run_one "$ROOT/test_host/test_generic_allocation.cpp"  ga  ""
run_one "$ROOT/test_host/test_tandemvec_allocator.cpp" tva ""
# 垂直推力机型映射（ThrustAllocation.h，纯平台无关——比力→总推力/油门、
# 倾角补偿、上下饱和、非有限输入；锁旧 mapFloat 逐值口径）：
run_one "$ROOT/test_host/test_thrust_allocation.cpp" talloc ""
# 可选振荡鲁棒层（AdaptiveGainDamper.h，纯平台无关，默认不接入飞控）：
run_one "$ROOT/test_host/test_adaptive_gain_damper.cpp" agd ""
# 现代控制候选：一阶 ADRC/ESO（纯模块，默认不接飞控）：
run_one "$ROOT/test_host/test_adrc_controller.cpp" adrc ""
# 速率预测/超前补偿（纯模块，默认不接飞控）：
run_one "$ROOT/test_host/test_rate_lead_predictor.cpp" rlp ""
# 实测角加速度反馈 AAF（纯模块，默认不接飞控；lead SIL 失败后的修正路线）：
run_one "$ROOT/test_host/test_rate_accel_feedback.cpp" aaf ""
# 速率相位超前（纯模块，默认不接飞控；不放大差分噪声）：
run_one "$ROOT/test_host/test_rate_phase_lead.cpp" rpl ""
# 偏航振荡检测（纯模块，默认不接导航；位置保持降级候选）：
run_one "$ROOT/test_host/test_yaw_oscillation_detector.cpp" yod ""
# 绝对航向辅助/位置保持降级门控（HeadingAidingGate.h，纯平台无关——
# Mag/YawGsf 新鲜度、失效立即降级、恢复 3 s 锁存、YawGsf 速度门限）：
run_one "$ROOT/test_host/test_heading_aiding_gate.cpp" hag ""
# 液体火箭变质量推进/喷管几何（LiquidRocketModel.h，纯平台无关——
# 点火/解锁互锁、阀门/推力速率、Tmin、比冲流量、耗尽关机、两轴 asin 分解）：
run_one "$ROOT/test_host/test_liquid_rocket_model.cpp" lrm ""
# 共轴双桨 VTVL DirectTvcRate 控制律（CoaxialVtvControl.h，纯平台无关——
# 通道极性/限幅/手动旁路/偏航差速 + 简化 6DOF 刚体闭环）：
run_one "$ROOT/test_host/test_coaxial_vtv_control.cpp" cvc ""
# 共轴参数可标定化（FlightCtrlParams.h::CoaxialCtrlParams，纯平台无关——
# 默认值逐位 = 旧硬编码、注册表 coax.* 顺序、默认参数逐拍输出与改造前一致）：
run_one "$ROOT/test_host/test_coax_params.cpp" cparams ""
# 共轴分配器控制契约（ControlContracts.h / CoaxialAllocator.h，纯平台无关——
# 逆解往返、轴序/符号对齐 SIL plant、低推力边界、非有限输入、残差语义、名义点手感）：
run_one "$ROOT/test_host/test_allocator_contracts.cpp" aco ""
# 制导/位置环加速度 → 推力方向纯函数（PowerAllocation.h，纯平台无关——
# 圆形加速度限幅、比力归一化、倾角限幅；与拆分前 flight_control.cpp 内联代码等价）：
run_one "$ROOT/test_host/test_power_allocation.cpp" pwa ""
# 共轴分配器 3DOF α 跟踪（真实惯量积分；名义/惯量失配/饱和/偏航权限）：
run_one "$ROOT/test_host/test_alpha_tracking.cpp" atrk ""
# 液体火箭分配器试点（α+油门→摆座/RCS/主阀；名义往返、饱和、零推力、非有限）：
run_one "$ROOT/test_host/test_liquid_rocket_allocator.cpp" lra ""
# 共轴双桨 VTVL rigorous 动力学模型（kT·ω² 推力、反扭含 Jp·ω̇、双转子陀螺、
# 摆座几何、6DOF 半隐式积分、自由落体能量守恒、地面接触/摩擦；纯平台无关）：
run_one "$ROOT/test_host/test_coaxial_dynamics_model.cpp" cdyn "-I$ROOT/test_host/fw_sil"
# 导航 IMU 增量源选择 + 健康键故障后持（NavImuIncrementSelect.h，纯平台无关——
# 关闭惰性、Failed 当拍回退、后持递减/刷新、hold 归一、reset、确认延迟不变量）：
run_one "$ROOT/test_host/test_nav_imu_increment_select.cpp" navisel ""
echo "---- 编译 lrccaegt ----"
if ! $CXX -std=$STD -O2 $INCLUDES -I"$ROOT/test_host/reference/ccaegt" \
     "$ROOT/test_host/test_liquid_ccaegt_reference.cpp" \
     "$ROOT/test_host/reference/ccaegt/CCAEGT.cpp" \
     -o "$BIN_DIR/lrccaegt$EXE_SUFFIX" 2>&1; then
  echo "[BUILD FAIL] lrccaegt"
  fail=1
else
  echo "---- 运行 lrccaegt ----"
  if ! "$BIN_DIR/lrccaegt$EXE_SUFFIX"; then
    echo "[RUN FAIL] lrccaegt"
    fail=1
  fi
  echo
fi
run_one "$ROOT/test_host/test_altitude_reference.cpp"  ar  ""
run_one "$ROOT/test_host/test_gnss_epoch_timing.cpp"    get ""
run_one "$ROOT/test_host/test_gnss_pps_latency.cpp"      pps ""
run_one "$ROOT/test_host/test_gnss_delay_batch.cpp"      bdelay ""
run_one "$ROOT/test_host/test_gnss_delay_auto.cpp"       dauto ""
run_one "$ROOT/test_host/test_ins_vibration_adaptive.cpp" vib ""
run_one "$ROOT/test_host/test_ins_vibration_hf.cpp"        vibhf ""
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
# GNSS+光流组合失效检测（连续丢失/滞回恢复/非法 dt）
run_one "$ROOT/test_host/test_nav_aid_loss.cpp"       navaid ""

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

# CRSF 断链通道合成（FailsafeLinkLossPlan.h，纯平台无关——在空中判据三维网格
# 穷举/通道合成不变量/旧判据差异点；固件 linkDownCallback 与本测试共用同一份实现）：
run_one "$ROOT/test_host/test_failsafe_plan.cpp"       fsplan ""

# DETA100 帧校验防回退（源码守卫：CRC16 比较不得再被注释、失败必须计入拒帧计数；
# 该模块只能在 main.cpp 中包含，运行时行为由上板联调覆盖）：
run_one "$ROOT/test_host/test_deta100_crc_guard.cpp"  deta100g ""

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

# 固件算法宿主直调接口（fw_algo）：经 C ABI 的句柄必须与直接调用 Ekf15State
# 逐位一致。EKF_NAV_TELEMETRY 只增加只读协方差访问器，不改滤波数学。
echo "---- 编译 fwalgo ----"
if ! $CXX -std=$STD -O2 -D_USE_MATH_DEFINES -DEKF_NAV_TELEMETRY \
     -DEIGEN_DONT_VECTORIZE -DEIGEN_DONT_PARALLELIZE \
     $INCLUDES -I$ROOT/test_host -I$ROOT/src -I$ROOT/lib/navigation-main/src \
     -I$ROOT/lib/units/src \
     "$ROOT/test_host/fw_algo/fw_algo_api.cpp" \
     "$ROOT/test_host/test_fw_algo_api.cpp" \
     "$ROOT/lib/units/src/convang.cpp" \
     "$ROOT/lib/units/src/convangacc.cpp" \
     "$ROOT/lib/units/src/convangvel.cpp" \
     -o "$BIN_DIR/fwalgo$EXE_SUFFIX" 2>&1; then
  echo "[BUILD FAIL] fwalgo"
  fail=1
else
  echo "---- 运行 fwalgo ----"
  if ! "$BIN_DIR/fwalgo$EXE_SUFFIX"; then
    echo "[RUN FAIL] fwalgo"
    fail=1
  fi
  echo
fi

# 固件控制分配 ABI（fw_ctrl）：默认参数/悬停状态/分配器/推力分配逐位一致。
echo "---- 编译 fwctrl ----"
if ! $CXX -std=$STD -O2 -D_USE_MATH_DEFINES -DEKF_NAV_TELEMETRY \
     -DEIGEN_DONT_VECTORIZE -DEIGEN_DONT_PARALLELIZE \
     $INCLUDES -I$ROOT/test_host -I$ROOT/src -I$ROOT/lib/navigation-main/src \
     -I$ROOT/lib/units/src \
     "$ROOT/test_host/fw_algo/fw_algo_api.cpp" \
     "$ROOT/test_host/fw_algo/fw_ctrl_api.cpp" \
     "$ROOT/test_host/test_fw_ctrl_api.cpp" \
     "$ROOT/lib/units/src/convang.cpp" \
     "$ROOT/lib/units/src/convangacc.cpp" \
     "$ROOT/lib/units/src/convangvel.cpp" \
     -o "$BIN_DIR/fwctrl$EXE_SUFFIX" 2>&1; then
  echo "[BUILD FAIL] fwctrl"
  fail=1
else
  echo "---- 运行 fwctrl ----"
  if ! "$BIN_DIR/fwctrl$EXE_SUFFIX"; then
    echo "[RUN FAIL] fwctrl"
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

# 大过载/大机动 + GNSS 杆臂补偿回归（2026-10-26）：
# 8 g 协调转弯 + 2 m 机体系杆臂；EKF 杆臂补偿后 pos<1 m，关闭补偿应失败。
echo "---- 编译 highdyn ----"
if ! $CXX -std=$STD -O2 -DEKF_HOST_REGRESSION -D_USE_MATH_DEFINES -DEIGEN_DONT_VECTORIZE -DEIGEN_DONT_PARALLELIZE      $INCLUDES -I$ROOT/src -I$ROOT/lib/navigation-main/src -I$ROOT/lib/units/src      "$ROOT/test_host/test_ekf_high_dynamic.cpp"      "$ROOT/lib/units/src/convang.cpp"      "$ROOT/lib/units/src/convangacc.cpp"      "$ROOT/lib/units/src/convangvel.cpp"      -o "$BIN_DIR/highdyn$EXE_SUFFIX" 2>&1; then
  echo "[BUILD FAIL] highdyn"
  fail=1
else
  echo "---- 运行 highdyn 杆臂回归 ----"
  if ! "$BIN_DIR/highdyn$EXE_SUFFIX" --profile turn --load 8 --speed 80       --duration 20 --latency-ms 20 --lever-m 2 --lever-comp 2 --seed 1       --assert-rmse 1.0 --json >/dev/null; then
    echo "[RUN FAIL] highdyn lever-comp"
    fail=1
  fi
  if "$BIN_DIR/highdyn$EXE_SUFFIX" --profile turn --load 8 --speed 80       --duration 20 --latency-ms 20 --lever-m 2 --lever-comp 0 --seed 1       --assert-rmse 1.0 --json >/dev/null 2>&1; then
    echo "[RUN FAIL] highdyn lever-off 应超过 1 m"
    fail=1
  fi
  echo
fi

# ICM-45686 FIFO/WOM/APEX（mock transport：C 源 + C++ 测试混编）
IMU45686_INC="-I$ROOT/lib/ICM45686/src -I$ROOT/lib/ICM45686/src/imu"
IMU45686_SRCS="inv_imu_driver inv_imu_transport inv_imu_driver_advanced inv_imu_driver_aux1 inv_imu_edmp inv_imu_edmp_compass inv_imu_edmp_wearable inv_imu_i2cm inv_imu_selftest"
echo "---- 编译 icm45686_fifo_mock ----"
build_ok=1
for f in $IMU45686_SRCS; do
  if ! $CC -std=c11 -O2 $IMU45686_INC -c "$ROOT/lib/ICM45686/src/imu/$f.c" -o "$BIN_DIR/$f.o" 2>&1; then
    build_ok=0
    break
  fi
done
if [ "$build_ok" -eq 1 ] && ! $CXX -std=$STD -O2 $IMU45686_INC "$ROOT/test_host/test_icm45686_fifo_mock.cpp" "$BIN_DIR"/inv_imu_*.o -o "$BIN_DIR/fifomock$EXE_SUFFIX" 2>&1; then
  build_ok=0
fi
if [ "$build_ok" -eq 1 ]; then
  echo "---- 运行 icm45686_fifo_mock ----"
  if ! "$BIN_DIR/fifomock$EXE_SUFFIX"; then
    echo "[RUN FAIL] icm45686_fifo_mock"
    fail=1
  fi
else
  echo "[BUILD FAIL] icm45686_fifo_mock"
  fail=1
fi
echo

if [ "$fail" -eq 0 ]; then
  echo "=== 全部宿主机回归测试通过 ==="
  exit 0
else
  echo "=== 存在失败项 ==="
  exit 1
fi
