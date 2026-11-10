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
    float attitude_limit_deg = 90.0f;// 姿态安全限幅 [deg]（仅非手动模式超限锁输出）
    float dt_s = 0.005f;             // 共轴控制周期 [s]（200Hz）

    // ---- ★ task-10 α+油门分配器参数（默认保持 legacy 角度路径，旧字段顺序/默认值不变）----
    float alloc_mode = 0.0f;              // 已废弃：固件忽略（α-native 为唯一路径），ID 保留兼容
    float lever_m = 0.21f;                 // TVC 力臂 [m]（α→力矩换算）★2026-11-10 用户确认实测 21 cm（原 0.15 估值偏低 40%）
    float yaw_moment_nm_per_us = 1.0e-6f; // 差速 PWM→偏航力矩系数 [N·m/µs]（MODEL-DEFAULT，SIL 归一化 yaw 模型换算，非台架值）
    // ★ 2026-11-10 惯量重估：原 0.05/0.05/0.08 是按"0.85 m 均质杆"量级给的，对 0.84 kg
    //   级共轴双桨大了 10~20 倍，导致分配器 δ=asin(I·α/(l·T)) 算出的舵角远大于实际需要
    //   （等效把控制增益放大了十几倍），这是此前"原始等效增益也抖到无法起飞"的根因之一。
    //   按质量分布估算（0.84 kg、桨盘约 0.15 m、电池/机架半径 0.05–0.08 m）：
    //     横滚/俯仰：0.6×0.065² + 0.01×0.075² ≈ 0.003 kg·m²
    //     偏航：质量集中在轴线附近，0.6×0.04² ≈ 0.001 kg·m²
    //   ★ 这是量级估计，不是台架值。上电后可用 DBG `id` 读 b=I名义/I实际 校正：
    //     实际 I = 名义 I / b。改本组后速率环增益必须按同比例重算（见 BFS_RATE_*）。
    // ★ 2026-11-10 修正：0.003/0.001 是本机按"小型多旋翼"拍的量级，错了约一个数量级。
    //   依据：若真实惯量只有 0.003，原版固件 0.25 舵角增益在实飞中会超调 800%（闭环仿真），
    //   而原版是实飞验证稳定的；按 0.25 增益给出约 100 ms 合理响应反推，真实惯量在
    //   0.04~0.05 量级——与原版固件写的 0.05 一致。
    //   α 链的闭环增益 = kp·I_nom/(l·T)，I_nom 错 14 倍 ⇒ 同样 α 指令物理角加速度只有 7%，
    //   这就是"修正幅度小、响应不足"的根因，也是此前把 kp 加到 100/144 反而震的原因。
    float inertia_roll_kgm2 = 0.015f;       // 滚转轴转动惯量 [kg·m²]（反推自原版实飞响应）
    float inertia_pitch_kgm2 = 0.015f;      // 俯仰轴转动惯量 [kg·m²]（反推自原版实飞响应）
    float inertia_yaw_kgm2 = 0.02f;         // 偏航轴转动惯量 [kg·m²]（绕推力轴，取原版 0.08 的 1/4）
};

/**
 * @brief 共轴参数注册表顺序（单一事实源：ano_params.cpp 与 host 测试共用）
 * @note  X(线上名, CoaxialCtrlParams 成员路径)；名称 ≤20B，ID = 表尾追加顺序。
 *        GCS/server/params.py::expected_names() 必须保持同序同拼写。
 */
#define BFS_COAX_PARAM_LIST(X) \
    X(coax.max_corr_deg, ctrl.max_correction_deg) \
    X(coax.max_yaw_us, ctrl.max_yaw_diff_us) \
    X(coax.mass_kg, mass_kg) \
    X(coax.max_thrust_n, max_total_thrust_n) \
    X(coax.servo_half_deg, servo_half_angle_deg) \
    X(coax.motor_min_us, motor_min_us) \
    X(coax.motor_max_us, motor_max_us) \
    X(coax.att_limit_deg, attitude_limit_deg) \
    X(coax.dt_s, dt_s) \
    X(coax.alloc_mode, alloc_mode) \
    X(coax.lever_m, lever_m) \
    X(coax.yaw_nm_per_us, yaw_moment_nm_per_us) \
    X(coax.inertia_roll, inertia_roll_kgm2) \
    X(coax.inertia_pitch, inertia_pitch_kgm2) \
    X(coax.inertia_yaw, inertia_yaw_kgm2)

#define BFS_COAX_PARAM_COUNT 15

/**
 * @brief 单涵道 + X 型四燃气舵机型硬件/输出参数（★ 2026-11-05 新增）
 * @note  仅 BFS_SINGLE_EDF_4VANE 编译分支消费；控制环增益仍走通用 att_ 与 rate_ 两组
 *        （速率环输出 deg/s²，与纵列/共轴同一量纲），本结构只放"构型硬件与
 *        输出标定"这类机型专属常量：
 *          - 质量/最大推力：推力分配（ThrustAllocation）用；
 *          - 满舵角/舵效/惯量：SingleEdf4VaneAllocator 的 α→力矩→舵偏换算；
 *          - 偏航配平：抵消涵道风扇反扭矩（绕推力轴 = 存档系 z_b = 偏航轴）；
 *          - PWM 中位/半行程/硬限位：四舵与推进器的输出标定；
 *          - 大倾角阈值：推进切断联锁（与源机型一致，默认 120°（2026-11-05 由源机型 45° 放宽，可在线调 10–180°））。
 *        ★ 默认值来源：源机型 Universal-EDF 的 `kParamsSingleEdf4Vane`（质量 1.15 kg /
 *        最大推力 35 N / 满舵角 45°）与默认参数表；其中满舵角在本固件取 **20°**
 *        （源机型文档与代码存在 20°/45° 口径冲突，本固件按"机械行程保守值"取 20°，
 *        标定后按实测替换）。所有数值均为 MODEL-DEFAULT，未标定。
 * @units  kg、N、deg、N·m/deg、kg·m²、%、µs、s
 * @redline 满舵角与舵效系数直接决定控制权限，改动需按标定流程记录依据
 */
struct SingleEdf4VaneHwParams
{
    float mass_kg = 1.15f;                  // 整机质量 [kg]（MODEL-DEFAULT，未实测）
    float max_total_thrust_n = 35.0f;       // 单涵道最大推力 [N]（MODEL-DEFAULT）
    float vane_max_deg = 20.0f;             // 单舵机械满偏角 [deg]（待实测替换）
    // ★ 控制舵角上限 [deg]：调参界面上直观的“舵面最多偏多少度”。
    //   速率环的角加速度限幅由它折算（见 state_data.cpp applyFlightCtrlParams）：
    //     alpha_full = ctrl_vane_deg * (4k / I) * (180/pi)
    //   分配器的联合限幅取 min(vane_max_deg, ctrl_vane_deg)：机械限位是硬上限，
    //   本值是软件上限（可调小做保守飞行）。
    float ctrl_vane_deg = 20.0f;
    float vane_moment_nm_per_deg = 0.02f;   // 额定推力点舵效 [N·m/deg]（MODEL-DEFAULT）
    float inertia_roll_kgm2 = 0.02f;        // 滚转轴惯量 [kg·m²]（MODEL-DEFAULT）
    float inertia_pitch_kgm2 = 0.02f;       // 俯仰轴惯量 [kg·m²]（MODEL-DEFAULT）
    float inertia_yaw_kgm2 = 0.03f;         // 偏航轴惯量 [kg·m²]（MODEL-DEFAULT）
    float yaw_trim_deg = 0.0f;              // 涵道反扭矩配平 [deg]（未标定，0=不配平）
    // 逐舵物理方向符号（+1/-1）：标定时只改这里，不动分配矩阵。
    // ★ 默认值 = 源机型 Universal-EDF 的头文件 VANE1..4_DIR（+1,−1,+1,−1），
    //   已由用户在 2026-10-12 台架逐舵核对确认（源 docs/knowledge/vane-motion-paths.md）。
    //   轴置换不改变"哪个舵机正转为哪个物理方向"，故该组合可直接沿用；若换舵机/换
    //   安装方向，必须按 CTL-008 §7 重新标定。
    float vane_dir[4] = {+1.0f, -1.0f, +1.0f, -1.0f};
    float vane_center_pct = 50.0f;          // 舵机中位占空比 [%]（50% = 1500µs）
    float vane_span_pct = 16.67f;           // 满舵角对应占空比增量 [%]
    float vane_min_pct = 33.33f;            // 舵机占空比硬下限 [%]（防撞结构）
    float vane_max_pct = 66.66f;            // 舵机占空比硬上限 [%]
    float thruster_min_us = 1000.0f;        // 推进器最低油门脉宽 [µs]（0% 出力）
    float thruster_max_us = 2000.0f;        // 推进器满油门脉宽 [µs]
    // 大倾角切推进阈值 [deg]：
    // ★ 2026-11-05 用户要求：45° 太保守，大机动/翻跟头时一超就切推力反而危险。
    //   放宽到 120°——只有明显倒扣才切；要完全关闭就在上位机设 180
    //   （门控写作 `tilt <= limit`，180° 恒成立）。
    float attitude_limit_deg = 120.0f;
    float dt_s = 0.005f;                    // 控制周期 [s]（200Hz）
};

// 单涵道机型默认实例（固件与宿主测试共用同一数值；21 项已注册 AnoCom 0xE1
// 在线参数表，见下方 BFS_SINGLE_EDF_PARAM_LIST；标定可直接在线进行）。
static constexpr SingleEdf4VaneHwParams kSingleEdf4VaneHwParamsDefaults{};

// 安全关键默认值锁定（逐位）：满舵角/舵效/惯量/中位/行程/限幅改动必须走红线评审
static_assert(kSingleEdf4VaneHwParamsDefaults.vane_max_deg == 20.0f,
              "单涵道满舵角默认值必须保持 20deg（源机型 45° 待标定，本固件取保守值）");
static_assert(kSingleEdf4VaneHwParamsDefaults.vane_center_pct == 50.0f,
              "舵机中位默认值必须保持 50%（1500µs）");
static_assert(kSingleEdf4VaneHwParamsDefaults.vane_min_pct == 33.33f &&
                  kSingleEdf4VaneHwParamsDefaults.vane_max_pct == 66.66f,
              "舵机占空比硬限幅默认值必须保持 33.33–66.66%");
static_assert(kSingleEdf4VaneHwParamsDefaults.thruster_min_us == 1000.0f &&
                  kSingleEdf4VaneHwParamsDefaults.thruster_max_us == 2000.0f,
              "推进器脉宽默认值必须保持 1000–2000µs");
static_assert(kSingleEdf4VaneHwParamsDefaults.attitude_limit_deg == 120.0f,
              "大倾角切推进阈值默认值必须保持 120deg（用户 2026-11-05 要求放宽，可在线调至 180 关闭）");

/**
 * @brief 单涵道机型参数注册表顺序（单一事实源：ano_params.cpp 与 host 测试共用）
 * @note  X(线上名, SingleEdf4VaneHwParams 成员路径)；名称 ≤20B，ID = 表尾追加顺序。
 *        GCS/server/params.py::expected_names() 必须保持同序同拼写。
 *        ★ 这 21 项全部可经 AnoCom 0xE1 在线读写；写入后下一次 200Hz 控制拍即生效
 *        （混控器每拍读 kFlightCtrlParams.single_edf，无缓存、无重建）。
 *        ★ 安全关键：vane_max_deg / vane_dirx / vane_moment 直接改变控制权限，
 *        GCS 侧已给保守范围；改动仍须按 CTL-008 §7 标定流程逐项记录依据。
 */
#define BFS_SINGLE_EDF_PARAM_LIST(X) \
    X(sedf.mass_kg, mass_kg) \
    X(sedf.max_thrust_n, max_total_thrust_n) \
    X(sedf.vane_max_deg, vane_max_deg) \
    X(sedf.ctrl_vane_deg, ctrl_vane_deg) \
    X(sedf.vane_moment, vane_moment_nm_per_deg) \
    X(sedf.inertia_roll, inertia_roll_kgm2) \
    X(sedf.inertia_pitch, inertia_pitch_kgm2) \
    X(sedf.inertia_yaw, inertia_yaw_kgm2) \
    X(sedf.yaw_trim_deg, yaw_trim_deg) \
    X(sedf.vane_dir1, vane_dir[0]) \
    X(sedf.vane_dir2, vane_dir[1]) \
    X(sedf.vane_dir3, vane_dir[2]) \
    X(sedf.vane_dir4, vane_dir[3]) \
    X(sedf.vane_center, vane_center_pct) \
    X(sedf.vane_span, vane_span_pct) \
    X(sedf.vane_min_pct, vane_min_pct) \
    X(sedf.vane_max_pct, vane_max_pct) \
    X(sedf.thr_min_us, thruster_min_us) \
    X(sedf.thr_max_us, thruster_max_us) \
    X(sedf.att_limit_deg, attitude_limit_deg) \
    X(sedf.dt_s, dt_s)

#define BFS_SINGLE_EDF_PARAM_COUNT 21

// 安全关键默认值锁定（逐位；任何改动都必须走红线评审）
static_assert(CoaxialCtrlParams{}.mass_kg == 0.84f, "共轴质量默认值必须保持 0.84kg");
static_assert(CoaxialCtrlParams{}.max_total_thrust_n == 14.0f, "共轴最大推力默认值必须保持 14N");
static_assert(CoaxialCtrlParams{}.servo_half_angle_deg == 15.0f, "共轴舵机半行程默认值必须保持 15deg");
static_assert(CoaxialCtrlParams{}.motor_min_us == 1000.0f, "共轴电机最小 PWM 默认值必须保持 1000us");
static_assert(CoaxialCtrlParams{}.motor_max_us == 2000.0f, "共轴电机最大 PWM 默认值必须保持 2000us");
static_assert(CoaxialCtrlParams{}.attitude_limit_deg == 90.0f, "共轴姿态安全限幅默认值必须保持 90deg（手动模式旁路）");
static_assert(CoaxialCtrlParams{}.dt_s == 0.005f, "共轴控制周期默认值必须保持 5ms");
static_assert(CoaxialCtrlParams{}.ctrl.max_correction_deg == 10.0f, "共轴修正角限幅默认值必须保持 10deg");
// 分配器物理参数默认值锁定（MODEL-DEFAULT）；alloc_mode 已废弃，固件不再读取
static_assert(CoaxialCtrlParams{}.alloc_mode == 0.0f, "alloc_mode 已废弃，保留默认 0 兼容旧 GCS");
static_assert(CoaxialCtrlParams{}.lever_m == 0.21f, "TVC 力臂默认值必须与实测一致（当前 0.21m，2026-11-10 实测）");
static_assert(CoaxialCtrlParams{}.yaw_moment_nm_per_us == 1.0e-6f, "差速力矩系数默认值必须保持 1e-6N·m/µs（SIL 归一化 yaw 模型）");
static_assert(CoaxialCtrlParams{}.inertia_roll_kgm2 == 0.015f, "滚转惯量默认值必须与实飞反推值一致（当前 0.015kg·m²）");
static_assert(CoaxialCtrlParams{}.inertia_pitch_kgm2 == 0.015f, "俯仰惯量默认值必须与实飞反推值一致（当前 0.015kg·m²）");
static_assert(CoaxialCtrlParams{}.inertia_yaw_kgm2 == 0.02f, "偏航惯量默认值必须与实飞反推值一致（当前 0.02kg·m²）");

/**
 * @brief 全部控制环参数集合（固件唯一可调参数块）
 * @note  12 个 PidTuneParams（12×9 = 108 项）+ 4 组滤波 alpha + 前馈掩码 + FPV 曲线三参数；
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
    // ---- 单涵道 + X 型四燃气舵机型参数（2026-11-05；仅 BFS_SINGLE_EDF_4VANE 分支消费）----
    // ★ 可经 AnoCom 0xE1 在线读写（21 项，见 BFS_SINGLE_EDF_PARAM_LIST）；
    //   混控器每拍从 kFlightCtrlParams.single_edf 读取，写入即生效。
    SingleEdf4VaneHwParams single_edf; // 质量/推力/舵角/舵效/惯量/PWM/联锁阈值
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
// ============================================================
//  通用 PID 默认值（按机型编译期选择；变量仍是同一套 att_*/rate_*）
//  共轴：控制环输出统一角加速度，硬件换算在 CoaxialAllocator；
//  数值按共轴旧修正角口径在悬停点折算（不是独立 PID 变量）。
// ============================================================
// ★ 2026-11-06 审计补强：机型宏必须三选一。同时定义会让参数表、控制分支、
//   health 车型标签互相错位（例如 COAXIAL + SINGLE_EDF 同时开时，标签打
//   SingleEdf4Vane 而控制走共轴）——这类错误编译期就该拦住，不留到上板。
#if defined(BFS_COAXIAL_VTVL) && defined(BFS_SINGLE_EDF_4VANE)
#error "BFS_COAXIAL_VTVL 与 BFS_SINGLE_EDF_4VANE 互斥，不能同时定义（见 platformio.ini 各 env）"
#endif
#ifdef BFS_COAXIAL_VTVL
// ============================================================
//  共轴双桨 VTVL：★ 2026-11-10 统一到通用控制链
// ------------------------------------------------------------
//  控制结构已与纵列/单涵道一致：execute_attitude_controller（姿态外环 + 速率内环）
//  → execute_yaw_controller（偏航速率环）→ CoaxialAllocator（α → 摆角/差速）
//  → mapCoaxialVtvOutputs。生产路径不再有独立的 CoaxialVtvController 实例。
//
//  本组速率环参数是**通用 PID 的 deg/s² 域**（输入 deg/s、输出 deg/s²）。
//  ★ 2026-11-10 实测结论：照搬单涵道 57.3/18.3 会抖到无法起飞（一推油就大幅摆动）。
//    原因：共轴 10° 舵角只对应 246.7 deg/s²，57.3/24.67 ≈ 2.32 deg 舵角 per(deg/s)，
//    是原始共轴 0.25 的 9.3 倍；偏航 11.4592 更是原始 3.8 µs 的约 4210 倍。
//    现回到**原始共轴等效值**：rate 6.168/1.974（= 0.25 / 0.08 折算到 α 域，
//    rad↔deg 两次换算相消，数值与旧 rad 域参数相同），yaw 0.00272/0.02149
//    （= 3.8 µs/(deg/s)、ki 30 折算）。要更软就按比例往下调这一组。
//  ★ out_max 仍由 applyFlightCtrlParams() 按 coax 物理量重算，不照抄涵道的 4583：
//    out_max_roll = g_deg · max_corr_deg ≈ 246.7 deg/s²（10° 舵角，共轴机械上限）
//    out_max_yaw  = (k_yaw/I_yaw) · max_yaw_us · (180/π) ≈ 0.1289 deg/s²（180 µs）
//  ★ 姿态外环（att_*）本来就是 deg 域，4.25 与涵道相同，不动。
// ============================================================
#define BFS_ATT_ROLL_DEFAULT  {4.25f, 0.0f, 0.0f, -kMaxTargetRate, kMaxTargetRate, kMaxTargetRate * 0.5f, 0.0f, 0.0f, true}
#define BFS_ATT_PITCH_DEFAULT {4.25f, 0.0f, 0.0f, -kMaxTargetRate, kMaxTargetRate, kMaxTargetRate * 0.5f, 0.0f, 0.0f, true}
// ★ 共轴航向外环单独取 0.5（纵列/单涵道仍 5.0）：实飞调参结论，5.0 时回中航向锁太猛，
//   0.5 后航向基本只靠内环角速率阻尼，回中不再急拉。
#define BFS_ATT_YAW_DEFAULT   {0.0f, 0.0f, 0.0f, -kMaxTargetRate, kMaxTargetRate, kMaxTargetRate * 0.3f, 0.0f, 0.0f, true}
// 速率环 roll/pitch：★ 2026-11-10 随力臂 0.21 m / 惯量 0.003 重算。
//   舵角域等效 δ/err = kp·I/(l·T)，取原始共轴 0.25 deg/(deg/s)：
//     kp = 0.25·(l·T)/I = 0.25·(0.21×8.224)/0.003 ≈ 144；ki 同理 ≈ 46。
//   限幅由 applyFlightCtrlParams 按物理量派生（10° 舵角 ≈ 5756 deg/s²，5° 积分限≈2878）。
//   若仍抖或不跟手，按比例整体缩放这一组即可（改一个数另一个同比）。
#define BFS_RATE_ROLL_DEFAULT {28.8f, 9.2f, 0.0f, -1151.0f, 1151.0f, 575.6f, 60.0f, 0.2f, true}
#define BFS_RATE_PITCH_DEFAULT {28.8f, 9.2f, 0.0f, -1151.0f, 1151.0f, 575.6f, 60.0f, 0.2f, true}
// 偏航：diff/α = I_yaw/k_yaw = 0.001/1e-6 = 1000 µs per rad/s²（=17.45 µs per deg/s²）。
//   原始共轴 3.8 µs/(deg/s) 对应 kp=0.218；实飞手感偏猛，取约一半 → kp=0.1（≈1.75 µs）。
//   ki=0.8 同比例偏温和。
#define BFS_RATE_YAW_DEFAULT  {0.0109f, 0.086f, 0.0f, -0.516f, 0.516f, 0.143f, 40.0f, 0.2f, true}
#elif defined(BFS_SINGLE_EDF_4VANE)
// ============================================================
//  单涵道 + X 型四燃气舵：按源机型 Universal-EDF 的等效增益折算
// ------------------------------------------------------------
//  源机型控制律输出的是**舵角**（deg）：rate kp=0.25（deg per deg/s，
//  即角速率误差 1°/s → 0.25° 舵偏；att kp=4.25）。
//  本固件控制层输出**角加速度** α，经 allocator 换算：
//      δ[rad] = I·α[rad/s²] / (4k)，I=0.02 kg·m²、k=0.02 N·m/deg
//             ⇒ δ[deg] ≈ 0.00436 · α[deg/s²]
//  要让闭环响应与源一致（δ = 0.25·rate_err），需
//      α = 0.25/0.00436 · rate_err = 57.3 · rate_err  ⇒ kp_rate = 57.3 s⁻¹
//      ki_rate = 0.08/0.00436 = 18.3 s⁻²（源连续域 ki=0.08）
//  ★ 这组数值随 I/k 的标定值变化：I 或 k 实测更新后必须同步重算本组
//    （公式写在这里，不把数字当物理常量）。
//  ★ 输出限幅仍取 ±5729.578 deg/s²（与纵列同量纲上限），由 allocator 的
//    联合限幅兜底，不会因增益变大而打满舵机。
#define BFS_ATT_ROLL_DEFAULT  {4.25f, 0.0f, 0.0f, -kMaxTargetRate, kMaxTargetRate, kMaxTargetRate * 0.5f, 0.0f, 0.0f, true}
#define BFS_ATT_PITCH_DEFAULT {4.25f, 0.0f, 0.0f, -kMaxTargetRate, kMaxTargetRate, kMaxTargetRate * 0.5f, 0.0f, 0.0f, true}
#define BFS_ATT_YAW_DEFAULT   {5.0f, 0.0f, 0.0f, -kMaxTargetRate, kMaxTargetRate, kMaxTargetRate * 0.3f, 0.0f, 0.0f, true}
// ★ 单涵道使用“舵角域”内环：速率环输出直接是三轴舵角需求 [deg]，
//   限幅就是舵面最大偏角（默认 ±20°），与源机型 Universal-EDF 一致。
//   ★ 内环输出 = 目标角加速度 α [deg/s²]，与纵列/共轴同一契约；机型差异全部由
//   SingleEdf4VaneAllocator 按（惯量 I、舵效 k、满舵角、矩阵、逐舵方向）换算。
//   ★ 限幅口径：用户在参数表设的是**机械摆角** sedf.ctrl_vane_deg [deg]，
//   applyFlightCtrlParams() 内部换算成 PID 输出限幅：
//       out_max = ctrl_vane_deg · (4k/I) · 180/π
//   默认 20° → roll/pitch 4583.66 deg/s²、yaw 3055.78 deg/s²（yaw 惯量 0.03 更大）。
//   ★ kp 来源：源机型 Universal-EDF 舵角域实测 0.25 deg/(deg/s)，换算到 α 域为
//       0.25 · (4·0.02/0.02) · (180/π) = 57.3 s⁻¹   （数学上与舵角域等价）
//   注：I 与 k 仍为 MODEL-DEFAULT 估算值；标定后换算结果自动跟随，无需改增益。
#define BFS_RATE_ROLL_DEFAULT {57.3f, 18.3f, 0.0f, -5729.578f, 5729.578f, 1145.9156f, 60.0f, 0.2f, true}
#define BFS_RATE_PITCH_DEFAULT {57.3f, 18.3f, 0.0f, -5729.578f, 5729.578f, 1145.9156f, 60.0f, 0.2f, true}
#define BFS_RATE_YAW_DEFAULT  {11.459156f, 4.297184f, 0.0f, -5729.578f, 5729.578f, 1145.9156f, 60.0f, 0.2f, true}
#else
#define BFS_ATT_ROLL_DEFAULT  {2.8f, 0.0f, 0.0f, -kMaxTargetRate, kMaxTargetRate, kMaxTargetRate * 0.5f, 0.0f, 0.0f, true}
#define BFS_ATT_PITCH_DEFAULT {2.8f, 0.0f, 0.0f, -kMaxTargetRate, kMaxTargetRate, kMaxTargetRate * 0.5f, 0.0f, 0.0f, true}
#define BFS_ATT_YAW_DEFAULT   {5.0f, 0.0f, 0.0f, -kMaxTargetRate, kMaxTargetRate, kMaxTargetRate * 0.3f, 0.0f, 0.0f, true}
#define BFS_RATE_ROLL_DEFAULT {16.042818f, 5.729578f, 0.0f, -5729.578f, 5729.578f, 1145.9156f, 60.0f, 0.2f, true}
#define BFS_RATE_PITCH_DEFAULT {16.042818f, 5.729578f, 0.0f, -5729.578f, 5729.578f, 1145.9156f, 60.0f, 0.2f, true}
#define BFS_RATE_YAW_DEFAULT  {11.459156f, 4.297184f, 0.0f, -5729.578f, 5729.578f, 1145.9156f, 60.0f, 0.2f, true}
#endif

static constexpr FlightCtrlParams kFlightCtrlParamsDefaults = {
    // ---- 姿态外环（deg 域：输入 deg、输出 deg/s）----
    // ★ 2026-08-09：2.5→2.3（提阻尼）→2.2（再降一档，先稳）
    // ★ 2026-08-10 COMM-001 固化：2.6→2.8（实机在线调参稳定，随双环重构烧录）
    // att_yaw = 5.0 是航向锁（ratchet hold）比例增益：2026-08-09 启用，1.2 实测发散
    //   回退 1.0；τm 观测器 + 增益调度修复后 5.0 实机验证稳定，随 COMM-001 固化。
    //   int_limit = 0.3 × 80 = 24 deg/s（回中后最大航向纠偏角速率）。
    /* att_roll  */ BFS_ATT_ROLL_DEFAULT,
    /* att_pitch */ BFS_ATT_PITCH_DEFAULT,
    /* att_yaw   */ BFS_ATT_YAW_DEFAULT,
    // ---- 角速率内环（2026-08-11 量纲迁移：输入 deg/s → 输出 deg/s²）----
    // 数值由旧混合域参数整体乘 180/π，进入物理层再乘 π/180，控制行为严格等价：
    //   roll/pitch: 0.28 → 16.042818；yaw: 0.20 → 11.459156；ki: 0.10 → 5.729578。
    // ★2026-08-12 偏航 Ki 首轮保守下调 25%: 5.729578 → 4.297184 s^-2，
    //   Ti=Kp/Ki 由 2.0s 增至 2.67s，减弱差速电机通道的低频积分作用。
    // 输出/积分限幅同乘 180/π；积分分离阈值仍是 deg/s，不变。
    /* rate_roll */ BFS_RATE_ROLL_DEFAULT,
    /* rate_pitch*/ BFS_RATE_PITCH_DEFAULT,
    /* rate_yaw  */ BFS_RATE_YAW_DEFAULT,
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
    // ★ 2026-11-10：共轴不再单独用 0.3/0.25，与涵道/纵列统一为 0.4/0.9。
    //   原共轴 out_alpha=0.25 配 Madgwick 姿态够用，但统一后控制用 EKF 姿态、
    //   延迟本就更大，再叠一级强低通（25% 新值、等效 4 拍≈20 ms）相位滞后过量，
    //   实飞表现为推油超调/振荡，故对齐涵道的 0.9。
#ifdef BFS_COAXIAL_VTVL
    // 共轴：反馈 0.8（τ 1.25ms）、输出 0.5（τ 5ms）——实飞把延迟从 26ms 压到 13ms 的折中档
    /* speed_filter_alpha     */ { 0.8f, 0.8f, 0.8f },
    /* speed_filter_alpha2    */ { 0.99f, 0.99f, 0.99f },
    /* angle_out_filter_alpha */ { 0.85f, 0.85f, 0.85f },
    /* output_filter_alpha    */ { 0.5f, 0.5f, 0.5f },
#else
    /* speed_filter_alpha     */ { 0.4f, 0.4f, 0.4f },
    /* speed_filter_alpha2    */ { 0.99f, 0.99f, 0.99f },
    /* angle_out_filter_alpha */ { 0.85f, 0.85f, 0.85f },
    /* output_filter_alpha    */ { 0.9f, 0.9f, 0.9f },
#endif
    /* inertia_comp_mask      */ 0x03,   // ★2026-08-10 前馈全开（陀螺耦合+转子陀螺）
    // ---- FPV 摇杆曲线（★2026-08-11 RATE_MODE 接入）----
    //   默认：rc_rate=0.5（★2026-08-11 用户指定，实机试手感后下调——满杆基准
    //   100°/s；×super 后 roll/pitch 理论 333°/s、yaw 222°/s，不触 600 限幅）；
    //   expo 全轴 0.25（用户指定，中心更细腻）；super roll/pitch 0.7、yaw 0.55。
    //   实机手感调整走 0xE1 在线写
#ifdef BFS_COAXIAL_VTVL
    // ★ 2026-11-10 共轴单独取 0.12：原版共轴固件 RATE 模式是线性 ±80 dps，
    //   而 TandemVec 摇杆曲线满杆 = rc_rate×200/(1-super) = 0.5×200/0.3 = 333 dps，
    //   同样打满杆目标速率是原版的 4.16 倍，速率环看到 4 倍误差 → 舵角 4 倍 → 抖。
    //   0.12 对应满杆 80 dps，与原版量程一致；纵列/单涵道保持 0.5 不动。
    /* rc_rate               */ 0.12f,
#else
    /* rc_rate               */ 0.5f,
#endif
    /* rc_expo[3]            */ { 0.25f, 0.25f, 0.25f },
    /* rc_super[3]           */ { 0.7f, 0.7f, 0.55f },
    /* heading_aid_mode      */ static_cast<float>(BFS_HEADING_AID_MODE),
    /* heading_align_deg     */ BFS_HEADING_ALIGN_DEG,
    /* gnss_lever_x_m        */ 0.0f,
    /* gnss_lever_y_m        */ 0.0f,
    /* gnss_lever_z_m        */ 0.0f,
    // ---- 共轴双桨 VTVL（2026-10-30）：默认值 = 原 flight_control.cpp 硬编码 ----
    /* coax                  */ CoaxialCtrlParams{},
    // ---- 单涵道 + X 型四燃气舵（2026-11-05）：默认值见 SingleEdf4VaneHwParams ----
    /* single_edf            */ SingleEdf4VaneHwParams{},
};

#ifdef TANDEMVEC_FIRMWARE
// 固件：运行时可变唯一实例（state_data.cpp 定义，默认值 = kFlightCtrlParamsDefaults）
extern FlightCtrlParams kFlightCtrlParams;
#else
// 宿主机测试：只读副本（单文件编译，无链接）
static constexpr FlightCtrlParams kFlightCtrlParams = kFlightCtrlParamsDefaults;
#endif
