// ============================================================
//  FlightCtrlParams.h — 实机控制参数唯一事实源（2026-08-08 C 路径重构）
//
//  ★ 实机调参唯一入口：kFlightCtrlParams（数值 = 运行时生效值，固件侧可变，
//    支持上位机 AnoCom 0xE1 在线写入；出厂默认值 = kFlightCtrlParamsDefaults）
//  方案：docs/C路径-参数集中与遥测结构化方案.md
//
//  本头文件为纯平台无关（无 Arduino/Eigen 依赖），固件（state_data.cpp）
//  与宿主机测试（test_host/test_flight_control_axis.cpp）共用同一实例，
//  保证测试参数永不与实机漂移。
//
//  注意：数值域为 PositionPID 实际语义（deg 域）。
// ============================================================
#pragma once

// ============================================================
//  限幅常量（结构体初值的别名，供 state_data.cpp 保留符号兼容）
//  —— 调参以本文件为准，state_data.cpp 中的同名常量只是别名
// ============================================================
static constexpr float kMaxTargetRate       = 80.0f;   // 姿态外环最大输出（deg/s）
static constexpr float kPosCtrlMaxSpeedCmd  = 1.5f;    // 位置环最大速度指令（m/s）
static constexpr float kMaxAccelCmd         = 2.6f;    // 速度环最大加速度指令（m/s²）

// ============================================================
//  4.0 控制参数集中定义
// ============================================================
struct PidTuneParams
{
    float kp, ki, kd;        // 增益（PositionPID 构造参数 1-3）
    float out_min, out_max;  // 输出限幅（构造参数 4-5）
    float int_limit;         // 积分状态钳位（构造参数 7）
    float threshold;         // 积分分离阈值（构造参数 8）
    float filter_alpha;      // 微分滤波系数（构造参数 9）
    bool  enabled;           // 该环是否参与控制（false = 僵尸参数，仅保留对象兼容）
};

struct FlightCtrlParams
{
    // 姿态串级（deg 域）—— 现状值 2026-08-07 实机收敛
    PidTuneParams att_roll,  att_pitch,  att_yaw;   // 外环 kp=2.5/2.5/0.8（att_yaw 未启用）
    PidTuneParams rate_roll, rate_pitch, rate_yaw;  // 内环 kp=0.25/0.25/0.20
    // 垂直串级
    PidTuneParams alt_pos, alt_vel;                 // 1.0 | 5.0/0.00625
    // 水平位置/速度串级
    PidTuneParams pos_n, pos_e, vel_n, vel_e;       // 0.25 | 1.75/0.00125/Kd=10
    // 控制滤波器 alpha（ComplementaryFilter 构造参数）
    float speed_filter_alpha[3];      // 角速率滤波 roll/pitch/yaw（0.3）
    float angle_out_filter_alpha[3];  // 外环输出滤波（0.85）
    float output_filter_alpha[3];     // 内环输出滤波（0.25/0.25/0.12，yaw 实机调出）
};

// ============================================================
//  默认实例（唯一事实源）
//
//  ★ 出厂默认值：kFlightCtrlParamsDefaults（static constexpr 只读，
//    每个 TU 一份副本，固件/宿主机测试各自持有相同数值，无链接问题）
//
//  ★ 运行时生效值（固件侧可变）：
//    - 固件编译（-DTANDEMVEC_FIRMWARE，platformio.ini build_flags）：
//      kFlightCtrlParams 为 extern 声明，由 state_data.cpp 定义唯一可变
//      实例（以默认值初始化），上位机可经 AnoCom 0xE1 在线写入；
//    - 宿主机测试（test_host，单文件编译无链接）：
//      kFlightCtrlParams 保持 static constexpr 只读副本，测试参数永不与
//      出厂值漂移。
// ============================================================
static constexpr FlightCtrlParams kFlightCtrlParamsDefaults = {
    // ---- 姿态外环（deg 域）----
    // ★ 2026-08-09：内环定稿 0.30 后外环小幅回调 2.5→2.3（提串级阻尼 ζ，
    // 降角度超调；若仍偏冲下一步 2.2）
    /* att_roll  */ { 2.3f,    0.0f,     0.0f,   -kMaxTargetRate, kMaxTargetRate, kMaxTargetRate * 0.5f, 0.0f, 0.0f, true  },
    /* att_pitch */ { 2.3f,    0.0f,     0.0f,   -kMaxTargetRate, kMaxTargetRate, kMaxTargetRate * 0.5f, 0.0f, 0.0f, true  },
    /* att_yaw   */ { 0.8f,    0.0f,     0.0f,   -kMaxTargetRate, kMaxTargetRate, kMaxTargetRate * 0.3f, 0.0f, 0.0f, false }, // 未启用（航向=纯速率指令）
    // ---- 角速率内环 ----
    // ★ 2026-08-09 定稿：0.30 在线降 0.28 实测"几乎不超调"（RAM 写验证后固化）。
    // 轨迹：0.25→0.30→0.33→0.45(震荡)→0.36(震荡)→0.33→0.30→0.28(RAM确认)
    /* rate_roll */ { 0.28f,   0.0003f,  0.0f,   -100.0f, 100.0f, 10.0f, 30.0f, 0.2f, true },
    /* rate_pitch*/ { 0.28f,   0.0003f,  0.0f,   -100.0f, 100.0f, 10.0f, 30.0f, 0.2f, true },
    /* rate_yaw  */ { 0.22f,   0.001f,   0.0f,   -100.0f, 100.0f, 20.0f, 60.0f, 0.2f, true }, // 差速内环：2026-08-09 限幅10→20（d_yaw=7.36 已占74%）、分离阈值30→60（yaw 环慢，误差久滞>30° 致机动期间积分挂起）；0.22 稳定点
    // ---- 垂直串级 ----
    /* alt_pos   */ { 1.0f,    0.0f,     0.0f,   -1.0f,   1.0f,   250.0f, 0.0f, 0.0f, true },
    /* alt_vel   */ { 5.0f,    0.00625f, 0.0f,   -18.75f, 12.5f,  10.0f,  0.0f, 0.2f, true },
    // ---- 水平位置/速度串级（Kd=10 = 连续域 0.05/0.005）----
    /* pos_n     */ { 0.25f,   0.0f,     0.0f,   -kPosCtrlMaxSpeedCmd, kPosCtrlMaxSpeedCmd, kPosCtrlMaxSpeedCmd * 0.5f, 0.0f, 0.5f, true },
    /* pos_e     */ { 0.25f,   0.0f,     0.0f,   -kPosCtrlMaxSpeedCmd, kPosCtrlMaxSpeedCmd, kPosCtrlMaxSpeedCmd * 0.5f, 0.0f, 0.5f, true },
    /* vel_n     */ { 1.75f,   0.00125f, 10.0f,  -kMaxAccelCmd, kMaxAccelCmd, kMaxAccelCmd * 0.5f, 0.0f, 0.5f, true },
    /* vel_e     */ { 1.75f,   0.00125f, 10.0f,  -kMaxAccelCmd, kMaxAccelCmd, kMaxAccelCmd * 0.5f, 0.0f, 0.5f, true },
    // ---- 控制滤波器 alpha ----
    /* speed_filter_alpha     */ { 0.3f, 0.3f, 0.3f },
    /* angle_out_filter_alpha */ { 0.85f, 0.85f, 0.85f },
    /* output_filter_alpha    */ { 0.30f, 0.30f, 0.12f },  // roll/pitch 2026-08-09 内环提升：0.25→0.30；yaw=0.12 实机调出（差速抑震荡）
};

#ifdef TANDEMVEC_FIRMWARE
// 固件：运行时可变唯一实例（state_data.cpp 定义，默认值 = kFlightCtrlParamsDefaults）
extern FlightCtrlParams kFlightCtrlParams;
#else
// 宿主机测试：只读副本（单文件编译，无链接）
static constexpr FlightCtrlParams kFlightCtrlParams = kFlightCtrlParamsDefaults;
#endif
