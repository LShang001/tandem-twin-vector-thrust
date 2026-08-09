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
#pragma once
#include <cstdint>  // uint8_t（inertia_comp_mask，2026-08-10）
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
    // 姿态串级（deg 域）—— ★2026-08-09 dt 重构：Ki 为连续域（旧离散值×200）；终值 0.30/2.6/60/20 已实机确认
    PidTuneParams att_roll,  att_pitch,  att_yaw;   // 外环 kp=2.5/2.5/0.8（att_yaw 未启用）
    PidTuneParams rate_roll, rate_pitch, rate_yaw;  // 内环 kp=0.25/0.25/0.20
    // 垂直串级
    PidTuneParams alt_pos, alt_vel;                 // 1.0 | 5.0/0.00625
    // 水平位置/速度串级
    PidTuneParams pos_n, pos_e, vel_n, vel_e;       // 0.25 | 1.75/0.00125/Kd=10
    // 控制滤波器 alpha（ComplementaryFilter 构造参数）
    float speed_filter_alpha[3];      // 角速率滤波 roll/pitch/yaw（0.4）
    float speed_filter_alpha2[3];     // 二级滤波（0.99≈直通——★2026-08-09 实测：物理减震底座已隔离振动，数字滤波只加滞后；留参数便于将来复测）
    float angle_out_filter_alpha[3];  // 外环输出滤波（0.85）
    float output_filter_alpha[3];     // 内环输出滤波（0.9——2026-08-09 极端测试确认：执行机构本身滤高频，滞后纯负收益；yaw 旧 0.12 抑震荡为过时产物）
    // ★ 2026-08-10 惯量逆解交叉耦合前馈使能掩码（InertiaDecoupling.h）：
    //   bit0=ω×(I·ω) 陀螺耦合、bit1=ω×h 转子陀螺项。默认全开——悬停 ω≈0 交叉项≈0
    //   无副作用；实机 A/B 或异常时可在线置 0 关闭（0xE1 写入）。
    uint8_t inertia_comp_mask;         // 前馈使能掩码（默认 0x03）
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
    // ★ 2026-08-09：2.5→2.3（提阻尼）→2.2（再降一档，先稳）
    /* att_roll  */ { 2.6f,    0.0f,     0.0f,   -kMaxTargetRate, kMaxTargetRate, kMaxTargetRate * 0.5f, 0.0f, 0.0f, true  },
    /* att_pitch */ { 2.6f,    0.0f,     0.0f,   -kMaxTargetRate, kMaxTargetRate, kMaxTargetRate * 0.5f, 0.0f, 0.0f, true  },
    /* att_yaw   */ { 1.0f,    0.0f,     0.0f,   -kMaxTargetRate, kMaxTargetRate, kMaxTargetRate * 0.3f, 0.0f, 0.0f, true }, // ★2026-08-09 航向锁启用（ratchet hold：摇杆=角速度、回中=锁航向；0.8→1.0 更利落）
    // ---- 角速率内环 ----
    // ★ 2026-08-09 定稿：0.30 在线降 0.28 实测"几乎不超调"（RAM 写验证后固化）；
    // 随后 0.28→0.26 再降一档（先稳后冲，滤波升级换来的余量）
    /* rate_roll */ { 0.28f,   0.1f,     0.0f,   -100.0f, 100.0f, 20.0f, 60.0f, 0.2f, true },
    /* rate_pitch*/ { 0.28f,   0.1f,     0.0f,   -100.0f, 100.0f, 20.0f, 60.0f, 0.2f, true },
    /* rate_yaw  */ { 0.22f,   0.1f,     0.0f,   -100.0f, 100.0f, 20.0f, 60.0f, 0.2f, true }, // 差速内环：ki 0.1（=旧 0.0005；ESC 校准后不对称消除，弃 0.2）；限幅 20/阈值 60（2026-08-09）
    // ---- 垂直串级 ----
    /* alt_pos   */ { 1.0f,    0.0f,     0.0f,   -1.0f,   1.0f,   250.0f, 0.0f, 0.0f, true },
    /* alt_vel   */ { 5.0f,    1.25f,    0.0f,   -18.75f, 12.5f,  10.0f,  0.0f, 0.2f, true },
    // ---- 水平位置/速度串级（Kd=10 = 连续域 0.05/0.005）----
    /* pos_n     */ { 0.25f,   0.0f,     0.0f,   -kPosCtrlMaxSpeedCmd, kPosCtrlMaxSpeedCmd, kPosCtrlMaxSpeedCmd * 0.5f, 0.0f, 0.5f, true },
    /* pos_e     */ { 0.25f,   0.0f,     0.0f,   -kPosCtrlMaxSpeedCmd, kPosCtrlMaxSpeedCmd, kPosCtrlMaxSpeedCmd * 0.5f, 0.0f, 0.5f, true },
    /* vel_n     */ { 1.75f,   0.25f,    10.0f,  -kMaxAccelCmd, kMaxAccelCmd, kMaxAccelCmd * 0.5f, 0.0f, 0.5f, true },
    /* vel_e     */ { 1.75f,   0.25f,    10.0f,  -kMaxAccelCmd, kMaxAccelCmd, kMaxAccelCmd * 0.5f, 0.0f, 0.5f, true },
    // ---- 控制滤波器 alpha ----
    // ★ 2026-08-09 滤波大审计结论：飞控下有物理减震底座 + 执行机构天然低通，
    //   数字滤波只加相位滞后（实测 4.2°@1Hz 即触发内环振铃，6.9° 必抖）——
    //   全链滤波放到最弱：α1=0.4（留少量噪声抑制）/ 二级≈直通 / 输出≈直通
    /* speed_filter_alpha     */ { 0.4f, 0.4f, 0.4f },
    /* speed_filter_alpha2    */ { 0.99f, 0.99f, 0.99f },
    /* angle_out_filter_alpha */ { 0.85f, 0.85f, 0.85f },
    /* output_filter_alpha    */ { 0.9f, 0.9f, 0.9f },
    /* inertia_comp_mask      */ 0x03,   // ★2026-08-10 前馈全开（陀螺耦合+转子陀螺）
};

#ifdef TANDEMVEC_FIRMWARE
// 固件：运行时可变唯一实例（state_data.cpp 定义，默认值 = kFlightCtrlParamsDefaults）
extern FlightCtrlParams kFlightCtrlParams;
#else
// 宿主机测试：只读副本（单文件编译，无链接）
static constexpr FlightCtrlParams kFlightCtrlParams = kFlightCtrlParamsDefaults;
#endif
