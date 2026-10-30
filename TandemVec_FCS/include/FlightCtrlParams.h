/**
 * @file    FlightCtrlParams.h
 * @module  固件/控制（控制参数唯一事实源，纯平台无关）
 * @brief   12 个控制环的增益/限幅/滤波 + 惯量前馈掩码 + FPV 摇杆曲线参数
 * @details 输入：无（编译期默认值 + 运行时可变实例）；输出：kFlightCtrlParams
 *          （固件侧可由 AnoCom 0xE1 在线写入）与只读默认值 kFlightCtrlParamsDefaults。
 *
 * ★ 实机调参唯一入口：kFlightCtrlParams（数值 = 运行时生效值，固件侧可变，
 *   支持上位机 AnoCom 0xE1 在线写入；出厂默认值 = kFlightCtrlParamsDefaults）
 * 方案：docs/C路径-参数集中与遥测结构化方案.md
 *
 * 本头文件为纯平台无关（无 Arduino/Eigen 依赖），固件（state_data.cpp）
 * 与宿主机测试（test_host/test_flight_control_axis.cpp）共用同一实例，
 * 保证测试参数永不与实机漂移。
 *
 * 单位约定（★ 2026-08-11 量纲统一后，按环分别读）：
 *   姿态外环 att_*：输入 deg（误差）、输出 deg/s；kp 无量纲、ki 1/s
 *   角速率内环 rate_*：输入 deg/s、输出 deg/s²；kp 1/s、ki 1/s²、限幅 deg/s²
 *   垂直/水平环：m、m/s、m/s²，kp/ki/kd 按串级量纲（见各字段行尾注释）
 *   进入惯量逆解前由 ControlUnits.h 唯一转换为 rad/s²
 *
 * @theory  串级 PID 与积分分离/输出限幅参数化
 * @ref     CTL-001 §8.6（固件整条控制链的增益口径表）；
 *          @ref AGENTS.md 踩坑记录（速率环量纲统一、FPV 曲线、SAS 限幅口径）
 * @frame   N/A（标量表；角度域与坐标系无关）
 * @units   见上"单位约定"；时间 s，角速率 deg/s，角加速度 deg/s²
 * @redline 速率环量纲、SAS 限幅、差速增益相关参数（改动需按红线流程）
 * @note    ① 旧混合域速率增益 0.28/0.20/0.10 已废止：新值 = 旧值×180/π；
 *          ② yaw 的 ki 在 2026-08-12 下调 25%（5.729578→4.297184）；
 *          ③ 本文件里出现"双份 #pragma once"是历史残留，已清理。
 * @see     ControlUnits.h、TandemVec_Config.h、flight_control.cpp
 */
#pragma once
#include <cstdint>  // uint8_t（inertia_comp_mask，2026-08-10）
#include "CoaxialVtvControl.h"  // 共轴 DirectTvcRate 控制律参数（纯平台无关，2026-10-30）

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
/**
 * @brief 单个控制环的完整参数（增益 + 限幅 + 积分/滤波）
 * @note  字段顺序即 PositionPID 构造参数顺序；单位随环而定：
 *        角度环 deg/deg/s，位置环 m/m/s，速度环 m/s/m/s²。
 *        int_limit 是"积分项输出上限"（|ki·∫e dt|），PositionPID 内部反推
 *        积分状态界 = int_limit/ki；threshold 为积分分离阈值（误差同量纲）。
 */
struct PidTuneParams
{
    float kp, ki, kd;        // 增益（PositionPID 构造参数 1-3；单位随环）
    float out_min, out_max;  // 输出限幅（构造参数 4-5；量纲 = 该环输出）
    float int_limit;         // 积分项输出上限 |ki·∫e dt|（内部反推状态界，构造参数 7）
    float threshold;         // 积分分离阈值（构造参数 8；量纲 = 该环输入误差）
    float filter_alpha;      // 微分滤波系数（构造参数 9；0=1 直通，越小越强）
    bool  enabled;           // 该环是否参与控制（false = 僵尸参数，仅保留对象兼容）
};

/**
 * @brief 共轴双桨 VTVL 机型参数（★ 2026-10-30 可标定化，默认值 = 原函数内硬编码）
 * @note  仅 BFS_COAXIAL_VTVL 编译分支消费，但结构体放本文件以复用同一参数事实源：
 *        `ctrl` = DirectTvcRate 控制律 18 项（字段/默认值见 CoaxialVtvControl.h），
 *        其余 = 原 flight_control.cpp 共轴分支内的物理/输出常量。
 *        ★ 默认值逐位不变：test_host/test_coax_params.cpp 逐字段比对旧常量位型；
 *        PWM 范围/舵机半行程/修正角限幅/增益默认值均属安全关键项，不得改值。
 */
struct CoaxialCtrlParams
{
    bfs::CoaxialVtvParams ctrl;      // 控制律 18 项（增益/限幅/滤波，见 CoaxialVtvControl.h）
    float mass_kg = 0.84f;           // 等效质量 [kg]（推力分配用，MODEL-DEFAULT 未标定）
    float max_total_thrust_n = 14.0f;// 最大总推力 [N]（推力分配用，MODEL-DEFAULT 未标定）
    float servo_half_angle_deg = 15.0f; // 舵机半行程角 [deg]（输出标定）
    float motor_min_us = 1000.0f;    // 电机最小 PWM [µs]
    float motor_max_us = 2000.0f;    // 电机最大 PWM [µs]
    float attitude_limit_deg = 45.0f;// 姿态安全限幅 [deg]（超限锁输出）
    float dt_s = 0.005f;             // 共轴控制周期 [s]（200Hz）
};

/**
 * @brief 共轴参数注册表顺序（单一事实源：ano_params.cpp 与 host 测试共用）
 * @note  X(线上名, CoaxialCtrlParams 成员路径)；名称 ≤20B，ID = 表尾追加顺序。
 *        GCS/server/params.py::expected_names() 必须保持同序同拼写。
 */
#define BFS_COAX_PARAM_LIST(X)                    \
    X(coax.att_kp_roll, ctrl.att_kp_roll)         \
    X(coax.att_kp_pitch, ctrl.att_kp_pitch)       \
    X(coax.rate_kp_roll, ctrl.rate_kp_roll)       \
    X(coax.rate_ki_roll, ctrl.rate_ki_roll)       \
    X(coax.rate_kp_pitch, ctrl.rate_kp_pitch)     \
    X(coax.rate_ki_pitch, ctrl.rate_ki_pitch)     \
    X(coax.rate_kp_yaw, ctrl.rate_kp_yaw)         \
    X(coax.rate_ki_yaw, ctrl.rate_ki_yaw)         \
    X(coax.max_rate_dps, ctrl.max_target_rate_dps)\
    X(coax.max_corr_deg, ctrl.max_correction_deg) \
    X(coax.max_yaw_us, ctrl.max_yaw_diff_us)      \
    X(coax.att_int_lim, ctrl.att_integral_limit)  \
    X(coax.rate_int_lim, ctrl.rate_integral_limit)\
    X(coax.yaw_int_lim, ctrl.yaw_integral_limit)  \
    X(coax.yaw_int_thr, ctrl.yaw_integral_threshold)\
    X(coax.att_flt_alpha, ctrl.att_rate_filter_alpha)\
    X(coax.corr_flt_alpha, ctrl.correction_filter_alpha)\
    X(coax.pid_deriv_alpha, ctrl.pid_derivative_filter_alpha)\
    X(coax.mass_kg, mass_kg)                      \
    X(coax.max_thrust_n, max_total_thrust_n)      \
    X(coax.servo_half_deg, servo_half_angle_deg)  \
    X(coax.motor_min_us, motor_min_us)            \
    X(coax.motor_max_us, motor_max_us)            \
    X(coax.att_limit_deg, attitude_limit_deg)     \
    X(coax.dt_s, dt_s)

#define BFS_COAX_PARAM_COUNT 25

// 安全关键默认值锁定（逐位；任何改动都必须走红线评审）
static_assert(CoaxialCtrlParams{}.mass_kg == 0.84f, "共轴质量默认值必须保持 0.84kg");
static_assert(CoaxialCtrlParams{}.max_total_thrust_n == 14.0f, "共轴最大推力默认值必须保持 14N");
static_assert(CoaxialCtrlParams{}.servo_half_angle_deg == 15.0f, "共轴舵机半行程默认值必须保持 15deg");
static_assert(CoaxialCtrlParams{}.motor_min_us == 1000.0f, "共轴电机最小 PWM 默认值必须保持 1000us");
static_assert(CoaxialCtrlParams{}.motor_max_us == 2000.0f, "共轴电机最大 PWM 默认值必须保持 2000us");
static_assert(CoaxialCtrlParams{}.attitude_limit_deg == 45.0f, "共轴姿态安全限幅默认值必须保持 45deg");
static_assert(CoaxialCtrlParams{}.dt_s == 0.005f, "共轴控制周期默认值必须保持 5ms");
static_assert(CoaxialCtrlParams{}.ctrl.max_correction_deg == 10.0f, "共轴修正角限幅默认值必须保持 10deg");

/**
 * @brief 全部控制环参数集合（固件唯一可调参数块）
 * @note  11 个 PidTuneParams + 4 组滤波 alpha + 前馈掩码 + FPV 曲线三参数；
 *        被 state_data.cpp 的 PID 构造、flight_control.cpp 的调度与
 *        test_host 的宿主机回归共同读取（同一数值，杜绝两侧漂移）。
 */
struct FlightCtrlParams
{
    // 姿态串级（角度域）—— ★2026-08-09 dt 重构：Ki 为连续域（旧离散值×200）
    // 外环：输入 deg、输出 deg/s；att_yaw 仅作参数容器（航向走 ratchet hold）
    PidTuneParams att_roll,  att_pitch,  att_yaw;
    // 内环：输入 deg/s，输出 deg/s²；kp 单位 1/s，ki 单位 1/s²。
    // kd 当前作用于每拍 Δomega（未除 dt），数值单位仍为 1/s；现役值均为 0。
    PidTuneParams rate_roll, rate_pitch, rate_yaw;
    // 垂直串级（m → m/s → m/s²）
    PidTuneParams alt_pos, alt_vel;
    // 水平位置/速度串级（m → m/s → m/s²，N/E 对称）
    PidTuneParams pos_n, pos_e, vel_n, vel_e;
    // 控制滤波器 alpha（ComplementaryFilter 构造参数）
    float speed_filter_alpha[3];      // 角速率滤波 roll/pitch/yaw（0.4）
    float speed_filter_alpha2[3];     // 二级滤波（0.99≈直通——★2026-08-09 实测：物理减震底座已隔离振动，数字滤波只加滞后；留参数便于将来复测）
    float angle_out_filter_alpha[3];  // 外环输出滤波（0.85）
    float output_filter_alpha[3];     // 内环输出滤波（0.9——2026-08-09 极端测试确认：执行机构本身滤高频，滞后纯负收益；yaw 旧 0.12 抑震荡为过时产物）
    // ★ 2026-08-10 惯量逆解交叉耦合前馈使能掩码（InertiaDecoupling.h）：
    //   bit0=ω×(I·ω) 陀螺耦合、bit1=ω×h 转子陀螺项。默认全开——悬停 ω≈0 交叉项≈0
    //   无副作用；实机 A/B 或异常时可在线置 0 关闭（0xE1 写入）。
    uint8_t inertia_comp_mask;         // 前馈使能掩码（默认 0x03）

    // ---- FPV 摇杆曲线（★2026-08-11 接入 RATE_MODE，Betaflight 三参数模型）----
    //   rc_rate  全局灵敏度（0-2.5）——线性缩放整条曲线，满杆基准 200°/s
    //   rc_expo  输入侧中心曲线（0-1）——压中心斜率（默认 0.2）
    //   rc_super 输出侧边缘曲线（0-1，双曲增益）——默认 roll/pitch 0.7、yaw 0.55
    //   RATE_MODE：omega_ref = rcRateCurve(rc, rc_rate, expo, super, 最大角速率)
    float rc_rate;                    // 全局灵敏度（默认 1.0）
    float rc_expo[3];                 // 每轴 expo（roll/pitch/yaw，默认 0.2）
    float rc_super[3];                // 每轴 super（默认 0.7/0.7/0.55）
    // ---- 航向源（2026-10-25 无磁/磁仅初始对准）----
    float heading_aid_mode;           // 0=磁连续 1=磁仅初始 2=无磁手动
    float heading_align_deg;          // 手动对准航向 [deg]，真北顺时针
    // ---- GNSS 天线杆臂（相对 IMU，机体系，[m]）----
    // 高角速率/大机动下天线位置/速度与 IMU 不同；默认 0（保持旧行为）。
    float gnss_lever_x_m;
    float gnss_lever_y_m;
    float gnss_lever_z_m;
    // ---- 共轴双桨 VTVL 机型参数（2026-10-30；仅 BFS_COAXIAL_VTVL 分支消费）----
    CoaxialCtrlParams coax;           // 控制律 + 物理/输出常量（默认 = 原硬编码）
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
// 航向源编译期默认（0=磁连续/1=磁仅初始/2=无磁手动；GCS 0xE1 可在线覆盖）
#ifndef BFS_HEADING_AID_MODE
#define BFS_HEADING_AID_MODE 0
#endif
#ifndef BFS_HEADING_ALIGN_DEG
#define BFS_HEADING_ALIGN_DEG 0.0f
#endif
static constexpr FlightCtrlParams kFlightCtrlParamsDefaults = {
    // ---- 姿态外环（deg 域：输入 deg、输出 deg/s）----
    // ★ 2026-08-09：2.5→2.3（提阻尼）→2.2（再降一档，先稳）
    // ★ 2026-08-10 COMM-001 固化：2.6→2.8（实机在线调参稳定，随双环重构烧录）
    // att_yaw = 5.0 是航向锁（ratchet hold）比例增益：2026-08-09 启用，1.2 实测发散
    //   回退 1.0；τm 观测器 + 增益调度修复后 5.0 实机验证稳定，随 COMM-001 固化。
    //   int_limit = 0.3 × 80 = 24 deg/s（回中后最大航向纠偏角速率）。
    /* att_roll  */ {2.8f, 0.0f, 0.0f, -kMaxTargetRate, kMaxTargetRate, kMaxTargetRate * 0.5f, 0.0f, 0.0f, true},
    /* att_pitch */ {2.8f, 0.0f, 0.0f, -kMaxTargetRate, kMaxTargetRate, kMaxTargetRate * 0.5f, 0.0f, 0.0f, true},
    /* att_yaw   */ {5.0f, 0.0f, 0.0f, -kMaxTargetRate, kMaxTargetRate, kMaxTargetRate * 0.3f, 0.0f, 0.0f, true},
    // ---- 角速率内环（2026-08-11 量纲迁移：输入 deg/s → 输出 deg/s²）----
    // 数值由旧混合域参数整体乘 180/π，进入物理层再乘 π/180，控制行为严格等价：
    //   roll/pitch: 0.28 → 16.042818；yaw: 0.20 → 11.459156；ki: 0.10 → 5.729578。
    // ★2026-08-12 偏航 Ki 首轮保守下调 25%: 5.729578 → 4.297184 s^-2，
    //   Ti=Kp/Ki 由 2.0s 增至 2.67s，减弱差速电机通道的低频积分作用。
    // 输出/积分限幅同乘 180/π；积分分离阈值仍是 deg/s，不变。
    /* rate_roll */ {16.042818f, 5.729578f, 0.0f, -5729.578f, 5729.578f, 1145.9156f, 60.0f, 0.2f, true},
    /* rate_pitch*/ {16.042818f, 5.729578f, 0.0f, -5729.578f, 5729.578f, 1145.9156f, 60.0f, 0.2f, true},
    /* rate_yaw  */ {11.459156f, 4.297184f, 0.0f, -5729.578f, 5729.578f, 1145.9156f, 60.0f, 0.2f, true},
    // ---- 垂直串级（m → m/s → m/s²）----
    /* alt_pos   */ {1.0f, 0.0f, 0.0f, -1.0f, 1.0f, 250.0f, 0.0f, 0.0f, true},
    /* alt_vel   */ {5.0f, 1.25f, 0.0f, -18.75f, 12.5f, 10.0f, 0.0f, 0.2f, true},
    // ---- 水平位置/速度串级（Kd=10 = 连续域 0.05/0.005）----
    /* pos_n     */ {0.25f, 0.0f, 0.0f, -kPosCtrlMaxSpeedCmd, kPosCtrlMaxSpeedCmd,
                     kPosCtrlMaxSpeedCmd * 0.5f, 0.0f, 0.5f, true},
    /* pos_e     */ {0.25f, 0.0f, 0.0f, -kPosCtrlMaxSpeedCmd, kPosCtrlMaxSpeedCmd,
                     kPosCtrlMaxSpeedCmd * 0.5f, 0.0f, 0.5f, true},
    /* vel_n     */ {1.75f, 0.25f, 10.0f, -kMaxAccelCmd, kMaxAccelCmd, kMaxAccelCmd * 0.5f, 0.0f, 0.5f, true},
    /* vel_e     */ {1.75f, 0.25f, 10.0f, -kMaxAccelCmd, kMaxAccelCmd, kMaxAccelCmd * 0.5f, 0.0f, 0.5f, true},
    // ---- 控制滤波器 alpha ----
    // ★ 2026-08-09 滤波大审计结论：飞控下有物理减震底座 + 执行机构天然低通，
    //   数字滤波只加相位滞后（实测 4.2°@1Hz 即触发内环振铃，6.9° 必抖）——
    //   全链滤波放到最弱：α1=0.4（留少量噪声抑制）/ 二级≈直通 / 输出≈直通
    /* speed_filter_alpha     */ { 0.4f, 0.4f, 0.4f },
    /* speed_filter_alpha2    */ { 0.99f, 0.99f, 0.99f },
    /* angle_out_filter_alpha */ { 0.85f, 0.85f, 0.85f },
    /* output_filter_alpha    */ { 0.9f, 0.9f, 0.9f },
    /* inertia_comp_mask      */ 0x03,   // ★2026-08-10 前馈全开（陀螺耦合+转子陀螺）
    // ---- FPV 摇杆曲线（★2026-08-11 RATE_MODE 接入）----
    //   默认：rc_rate=0.5（★2026-08-11 用户指定，实机试手感后下调——满杆基准
    //   100°/s；×super 后 roll/pitch 理论 333°/s、yaw 222°/s，不触 600 限幅）；
    //   expo 全轴 0.25（用户指定，中心更细腻）；super roll/pitch 0.7、yaw 0.55。
    //   实机手感调整走 0xE1 在线写
    /* rc_rate               */ 0.5f,
    /* rc_expo[3]            */ { 0.25f, 0.25f, 0.25f },
    /* rc_super[3]           */ { 0.7f, 0.7f, 0.55f },
    /* heading_aid_mode      */ static_cast<float>(BFS_HEADING_AID_MODE),
    /* heading_align_deg     */ BFS_HEADING_ALIGN_DEG,
    /* gnss_lever_x_m        */ 0.0f,
    /* gnss_lever_y_m        */ 0.0f,
    /* gnss_lever_z_m        */ 0.0f,
    // ---- 共轴双桨 VTVL（2026-10-30）：默认值 = 原 flight_control.cpp 硬编码 ----
    /* coax                  */ CoaxialCtrlParams{},
};

#ifdef TANDEMVEC_FIRMWARE
// 固件：运行时可变唯一实例（state_data.cpp 定义，默认值 = kFlightCtrlParamsDefaults）
extern FlightCtrlParams kFlightCtrlParams;
#else
// 宿主机测试：只读副本（单文件编译，无链接）
static constexpr FlightCtrlParams kFlightCtrlParams = kFlightCtrlParamsDefaults;
#endif
