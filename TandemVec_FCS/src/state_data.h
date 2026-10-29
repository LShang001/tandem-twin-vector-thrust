/**
 * @file    state_data.h
 * @module  固件/基础设施（全局状态中枢：类型定义与跨模块变量声明）
 * @brief   集中声明全部跨模块共享的状态、配置常量、外设对象与遥测结构
 *
 * 本头文件集中管理飞控系统中所有跨模块共享的：
 *   - 枚举类型 (ControlMode, LoiterState 等)
 *   - 结构体类型 (ControlInputs_t, ControlOutputs_t, OpticalFlowData, Task, GncTelemetry)
 *   - 全局变量的 extern 声明
 *
 * @details 输入：无（纯声明）；处理：按“硬件资源 → 物理常数/限幅 → 控制参数与 PID →
 *          滤波器 → 传感器/外设对象 → 系统标志 → 遥测/缓冲区”分区组织；
 *          输出：被各 .cpp 包含后直接访问的全局符号。
 *          不变量：每个变量只在 state_data.cpp 定义一次；本文件不含函数实现；
 *          声明顺序与 state_data.cpp 的分区顺序一一对应。
 * @theory  N/A（数据聚合层）；真值语义见各字段行尾注释与 docs/C路径方案 §4
 * @ref     docs/C路径-参数集中与遥测结构化方案.md §4（FlightCtrlParams/GncTelemetry）；
 *          COMM-001 §3（SERIAL6_BAUDRATE=921600 与 200Hz 任务节奏）；
 *          AGENTS.md 踩坑记录（光流双新鲜度通道、上电无信号 raw_rc 全 0）
 * @frame   以注释逐字段标注：IMU/角速率=FRD 机体系，位置/速度=NED，
 *          姿态内部 rad（欧拉角约定见 MOD-002），
 *          执行器为摆座机械角 deg / 差速归一化 Δω
 * @units   SI：m、m/s、m/s²、rad、rad/s、rad/s²；角度遥测域另有 deg/s、deg/s²（字段名带 _deg/_dps）；
 *          时间 ms（millis）或 µs（micros）；PWM µs（988-2012）
 * @note    ★ 两条易错约束：
 *          ① `SERIAL6_BAUDRATE` 是地面站波特率唯一事实源，改波特率只改这一处；
 *          ② 光流速度与测距是两条独立新鲜度通道（`last_update_ms` vs
 *             `last_range_update_ms`），测距帧不得刷新速度通道时间戳。
 * @see     src/state_data.cpp、src/communication.cpp、include/FlightCtrlParams.h
 *
 * 设计原则：
 *   - 所有变量只在 state_data.cpp 中定义一次
 *   - 其他模块通过 #include "state_data.h" 访问共享状态
 *   - 不包含任何函数实现（函数在各模块 .cpp 中）
 */
#pragma once

#include <Arduino.h>
#include <Wire.h>

// Vector3 类型定义（使用 guard 防止与 QuaternionMath.h 中的定义冲突）
#ifndef VECTOR3_TYPE_GUARD
#define VECTOR3_TYPE_GUARD
struct Vector3
{
  double x, y, z;
  Vector3(double x_val = 0.0, double y_val = 0.0, double z_val = 0.0)
      : x(x_val), y(y_val), z(z_val) {}
};
#endif

// ===== 第三方库头文件（全局变量类型依赖） =====
#include "PositionPID.h"
#include "VerticalKF.h"
#include "VerticalKF_2State.h"
#include "HorizontalKF.h"
#include "ComplementaryFilter.h"
#include "MedianFilter.h"
#include "IMU_AutoCalibrator.h"
#include "MadgwickAHRS.h"
#include "StatusLED.h"
#include "WS2812Driver.h"
#include "Ws2812AppStatus.h"
#include "W25N01GV.h"
#include "W25N01GV_log.h"
#include "AnoComProtocol.h"
#include "CrsfSerial.h"
#include "crsf.h"
#include "common_rc.h"
#include "MagSensor.h"
#include "Dps310.h"
#include "ICM42688.h"
#include "LQS48_Flow.h"
#include "MTF02P.h"
#include "navigation.h"
#include "ubx.h"
#include "deta100_types.h" // DETA100 类型定义（枚举、结构体），可安全多文件包含
#include "MedianFilter.h"  // DETA100 滤波器依赖
// 注意: DETA100_module.h 在头文件中定义了解析实现和内部静态状态，
// 不能被多个 .cpp 包含。由 main.cpp 自行 include。

/*
 * ==========================================================================================
 * [1] 数据结构与枚举定义 (Data Structures & Enums)
 * ==========================================================================================
 * 必须最先定义，供后续变量和函数使用
 */

// 飞行控制模式枚举
enum ControlMode
{
  MANUAL,        // 手动模式：手动油门 + 姿态自稳
  AUTO_POSITION, // 位置保持模式：手动油门 + 水平位置闭环
  AUTO_ALTITUDE, // 高度保持模式：自动高度 + 水平位置闭环
  GUIDED         // 制导模式：由机载计算机指令控制
};

// 姿态子模式枚举
enum AttitudeSubMode
{
  ATTITUDE_MODE, // 角度模式 (Angle)
  RATE_MODE      // 角速度模式 (Rate)
};

// Loiter模式状态机 (悬停 vs 移动)
enum LoiterState
{
  LOITER_HOVERING, // 悬停状态：位置环生效
  LOITER_MOVING    // 移动状态：速度环生效
};

// 垂直Loiter状态机
enum VerticalLoiterState
{
  ALTITUDE_HOVERING, // 垂直悬停
  ALTITUDE_MOVING    // 垂直变高
};

// 光流传感器数据结构
/**
 * @brief 光流传感器共享状态（速度与测距两条独立新鲜度通道）
 *
 * 写入点：sensor_peripheral.cpp 各协议处理函数；消费方：navigation_task（EKF 融合）。
 * @note ★ `last_update_ms` 只由**光流（速度）帧**刷新，`last_range_update_ms` 只由
 *       **测距帧**刷新——若测距帧刷新速度时间戳，光流停流后冻结速度会被 EKF 当新测量
 *       反复融合；导航侧按时间戳判重，时间戳在走就每次都融。
 * @ref AGENTS.md 踩坑记录（光流速度/测距两条独立新鲜度通道，2026-08-21）
 * @ref PROTOCOL-001 §二.3.2（0x31 原始光流）/§二.3.4（0x34 通用测距）字段语义
 * @units 高度/速度 m、m/s（机体系）；时间 millis
 */
struct OpticalFlowData
{
  float distance_m;      // 倾斜补偿后的高度 (m)
  uint8_t range_quality; // 测距信号质量
  bool is_range_valid;   // 测距数据有效标志
  float velocity_x_mps;  // 旋转补偿后的X轴速度 (m/s, Body Frame)
  float velocity_y_mps;  // 旋转补偿后的Y轴速度 (m/s, Body Frame)
  uint8_t flow_quality;  // 光流信号质量
  bool is_flow_valid;    // 光流数据有效标志
  uint32_t timestamp_ms; // 传感器时间戳
  uint32_t last_update_ms; // 最近一次收到光流（速度）帧的系统时刻 (millis, 用于超时失效)
  // 测距通道独立新鲜度：MSP/MAVLink 的测距与光流是独立帧，单通道停流时
  // 不得让另一通道的冻结数据冒充新鲜（2026-08-21 双通道拆分校正）。
  uint32_t last_range_update_ms; // 最近一次写入 distance_m 的系统时刻
};

// 光流数据最大允许陈旧时间：超过该值仍未收到新帧，导航融合应视为光流失效。
constexpr uint32_t kFlowDataStaleTimeoutMs = 300U;

// 光流传感器类型（UART7 自动识别结果；加新传感器时扩展此枚举与检测状态机）
enum FlowSensorType : uint8_t
{
  FLOW_SENSOR_NONE = 0,          // 未识别到传感器
  FLOW_SENSOR_LQS48 = 1,         // LQ-S48 自定义协议 @921600（也兼容 115200）
  FLOW_SENSOR_MTF02P_MICOLINK = 2, // MTF-02P Micolink 协议 @115200
  FLOW_SENSOR_MTF02P_MAVLINK_APM = 3, // MTF-02P AUTO/AUTO 版，识别 ArduPilot 后走 MAVLink APM 输出
  FLOW_SENSOR_MTF02P_MSP = 4       // MTF-02P AUTO 版在 INAV/MSP 模式下输出 MSP2 OPTICALFLOW/RANGEFINDER
};

// 控制输入结构体 (遥控器与开关量)
typedef struct
{
  float roll_raw;                // 滚转通道 (988-2012)
  float pitch_raw;               // 俯仰通道
  float throttle_raw;            // 油门通道
  float yaw_raw;                 // 偏航通道
  float mode_channel_val;        // 模式通道
  bool is_unlocked;              // 解锁状态
  bool is_ignition_enabled;      // 点火使能
  bool is_manual_tvc;            // 手动TVC开关
  AttitudeSubMode attitude_mode; // 姿态/角速度模式
} ControlInputs_t;

// 控制输出结构体 — **机型无关控制层的边界**
// 位置/速度/姿态/角速率环只产生"期望天向比力 + 三轴 SI 角加速度"；
// 质量、最大推力、推力曲线、摆座几何、电机数量都不属于本层。
// 从本结构往后才进入机型逆解：
//   垂直通道 → ThrustAllocation（比力→总推力/油门/阀门）
//   转动通道 → TandemVecAllocator / 共轴 DirectTvcRate / 火箭摆座/RCS
// 不要再把电机数量、摆角、BTRUE、差速开方耦合进姿态/速率控制。
// 速率 PID 在角度域输出 deg/s²；写入本结构前统一转换为 rad/s²。
// 轴向（★ 2026-08-07 轴置换后，旧注释"上摆=偏航/差速=滚转"已废止；
// 锚点 = 手动模式摇杆直通实机验证）：alpha_roll→体轴x→上摆δ_f，
// alpha_pitch→体轴y→下摆δ_t，alpha_yaw→体轴z→差速Δω
typedef struct
{
  // 最终执行器油门：手动模式为 RC 直通值，自动模式由 mix 层机型推力映射回填。
  // 它属于机型输出口径，不是跨机型物理量；跨机型统一量是 specific_force_up。
  float throttle_percent;
  // 垂直通道机型无关输出：期望总比力（含抵消重力的 1g）沿 NED 天向，m/s²。
  float specific_force_up_mps2;
  // 推力轴在 NED 天向的投影 |R33|，已按安全下限钳到 0.5。
  float thrust_cos_tilt;
  // true = throttle_percent 是遥控直通（手动）；false = 由 specific_force 映射。
  bool throttle_direct;
  float alpha_roll_radps2;  // rad/s²  体轴x角加速度（滚转/上摆δ_f）
  float alpha_pitch_radps2; // rad/s²  体轴y角加速度（俯仰/下摆δ_t）
  float alpha_yaw_radps2;   // rad/s²  体轴z角加速度（偏航/差速Δω）
} ControlOutputs_t;

// 任务调度结构体
/**
 * @brief 调度器任务表项（tasks[] 元素，见 src/task_scheduler.cpp）
 * @note `interval` 的单位是**定时器 tick（0.5ms）**而非 ms；注册接口收 ms 后换算。
 *       `BFS_TASK_FRACTIONAL_INTERVALS` 打开时另存 Q8 目标间隔与小数误差，
 *       用于 7.735ms 这类非整数 tick 周期（见 task_scheduler_interval.h）。
 *       `flag` 由 TIM8 ISR 置位、主循环清除，故必须 volatile。
 * @ref COMM-001 §3（200Hz 任务节奏）
 */
struct Task
{
  void (*function)();   // 任务函数指针
  const char *name;     // 任务名，供性能诊断输出识别
  uint32_t interval;    // 执行间隔 (ticks)
#ifdef BFS_TASK_FRACTIONAL_INTERVALS
  uint32_t interval_q8;      // 固定点目标间隔，单位为 1/256 tick
  uint16_t interval_error_q8; // 小数 tick 累积误差
#endif
  uint32_t lastRunTime; // 上次执行时间
  bool enabled;         // 使能标志
  volatile bool flag;   // 执行请求标志
};

/*
 * ==========================================================================================
 * [2] 硬件资源配置 (Hardware Resources)
 * ==========================================================================================
 * 包含：串口对象、引脚定义、PWM配置、总线配置
 */

// --- 2.1 串口资源分配 ---
extern Uart Serial1; // USART1: ELRS接收机 (CRSF输入)
extern Uart Serial2; // USART2: 发动机控制器 / 数据转发
extern Uart Serial3; // USART3: 黑匣子数据记录 (高速)
extern Uart Serial4; // UART4:  DETA100 IMU/GNSS模块
extern Uart Serial5; // UART5:  上位机轨迹规划接口
extern Uart Serial6; // USART6: AnoCom地面站通信
extern Uart Serial7; // UART7:  光流传感器接口
extern Uart Serial8; // UART8:  USB Type-C 调试输出

// 串口引用别名
extern Uart &receiverSerial;    // 遥控接收
extern Uart &transmitterSerial; // 外部设备/发动机
extern Uart &opticalFlowSerial; // 光流

// --- 2.2 执行机构引脚 (Actuators) ---
#define TVC_ROLL_SERVO_PIN PA0  // [TIM2_CH1] TVC 滚转舵机 (SERVO1)
#define TVC_PITCH_SERVO_PIN PA1 // [TIM2_CH2] TVC 俯仰舵机 (SERVO2)
#define MOTOR1_PIN PA2          // [TIM2_CH3] 主电机1 / CW (SERVO3)
#define MOTOR2_PIN PA3          // [TIM2_CH4] 主电机2 / CCW (SERVO4)
#define FUEL_VALVE_PIN PC8      // [TIM3_CH3] 燃料阀门控制舵机 (SERVO7)

// 兼容旧代码定义的宏别名
#define SERVO1_PIN TVC_ROLL_SERVO_PIN
#define SERVO2_PIN TVC_PITCH_SERVO_PIN
#define SERVO3_PIN MOTOR1_PIN
#define SERVO4_PIN MOTOR2_PIN
#define SERVO7_PIN FUEL_VALVE_PIN

// --- 2.3 传感器与ADC引脚 ---
#define TVC_FEEDBACK_PIN_1 PB0 // [ADC] TVC角度传感器1
#define TVC_FEEDBACK_PIN_2 PB1 // [ADC] TVC角度传感器2
#define FUEL_LEVEL_PIN PC9     // [GPIO/ADC] 液位传感器/备用
#define ADC_BATT_PIN PC5       // [ADC] 电池电压采样 (U40 电池检测连接器 VBAT/ADC)
#define ADC_CURR_PIN PC3       // [ADC] 电流采样 (未使用)
#define ADC_EXT_PIN  PC0       // [ADC] 扩展模拟输入 (未使用)

// 电池电压采样分压比（ADC_BATT → 电池电压换算系数）
// 视觉分析确认（2026-08-08）：VBAT → 10kΩ → ADC_BATT(PC5) → 1kΩ → GND
//   分压比 = (10k+1k)/1k = 11.0（双模型交叉验证一致）
//   量程验证：3S 12.6V→1.145V / 4S 16.8V→1.527V / 6S 25.2V→2.29V，均在 ADC 0-3.3V 内
#ifndef BATTERY_DIVIDER_RATIO
#define BATTERY_DIVIDER_RATIO 11.0f
#endif

// 兼容旧代码定义的宏别名
#define SERVO5_PIN TVC_FEEDBACK_PIN_1
#define SERVO6_PIN TVC_FEEDBACK_PIN_2
#define SERVO8_PIN FUEL_LEVEL_PIN
#define FUEL_PIN FUEL_LEVEL_PIN

// --- 2.4 系统指示与控制引脚 ---
extern const int LED_yellow; // 黄色LED: 传感器异常/警告
extern const int LED_green;  // 绿色LED: 系统运行/任务心跳
extern const int ignition;   // 点火控制: MOS管高电平触发

// --- 2.5 总线引脚与对象 (I2C/SPI) ---
#define MAG_SDA_PIN PB7
#define MAG_SCL_PIN PB6
extern TwoWire Wire1; // 自定义I2C总线1 (磁力计)

#define BARO_SDA_PIN PB11
#define BARO_SCL_PIN PB10
#define DPS310_ADDR_SEL PE15
extern TwoWire IIC2; // 自定义I2C总线2 (气压计)

// SPI引脚定义 (板载IMU)
extern const int SPI_MOSI;
extern const int SPI_MISO;
extern const int SPI_SCLK;
extern const int SPI_CS;

// --- 2.6 硬件扩展预留引脚 (Hardware Expansion Reserved) ---
// 以下引脚在PCB上已焊接并连接到MCU，固件尚未驱动，供未来扩展使用。
// 详见 docs/电路拓扑参考.md §12 未使用硬件资源。

// CAN总线 (MCP2515 + TJA1050, SPI2)
#define CAN1_CS_PIN  PB12  // SPI2 片选 (MCP2515 CS)
#define CAN1_INT_PIN PD10  // MCP2515 中断
#define CAN_SPI_SCK  PB13  // SPI2 时钟
#define CAN_SPI_MISO PB14  // SPI2 主入从出
#define CAN_SPI_MOSI PB15  // SPI2 主出从入

// 外部Flash (W25N01GV, 1Gbit NAND, SPI3)
#define FLASH_CS_PIN  PA15  // SPI3 片选
#define FLASH_SPI_SCK PC10  // SPI3 时钟
#define FLASH_SPI_MISO PC11 // SPI3 主入从出
#define FLASH_SPI_MOSI PB2  // SPI3 主出从入

// WS2812 可编程RGB灯带
#define WS2812_PIN PD15  // 单总线RGB LED (备用状态指示)

// 空闲SPI4总线 (PCB已引出，无连接器件，需飞线使用)
#define SPI4_NSS_PIN  PE11  // SPI4 片选
#define SPI4_SCK_PIN  PE12  // SPI4 时钟
#define SPI4_MISO_PIN PE13  // SPI4 主入从出
#define SPI4_MOSI_PIN PE14  // SPI4 主出从入

// UART硬件流控 (当前8路UART均未启用流控，对应引脚空闲)
#define USART2_CTS_PIN PD4   // Serial2 CTS
#define UART4_CTS_PIN  PD11  // Serial4 CTS
#define UART4_RTS_PIN  PD12  // Serial4 RTS

// 未连接GPIO — 以下MCU引脚在原理图中无wire连接，可飞线扩展
// PE2(pin1):  SAI1_CK1, TIM3_CH1
// PE3(pin2):  SAI1_D1,  TIM3_CH2
// PA8(pin67): MCO(时钟输出), TIM1_CH1
// PD13(pin60): TIM5_CH3
// PD14(pin61): TIM5_CH4
// PB8(pin95):  I2C1_SCL复用, TIM4_CH3
// PB9(pin96):  I2C1_SDA复用, TIM4_CH4

// --- 2.6b 串口波特率配置（★ 唯一权威源，文档引用此处而非具体数值）---
// Serial6 波特率随数传模块型号调整（2026-08-10 COMM-001：2M→921600——2M 在
// DAP-Link VCP 丢帧 ~15%，921600 位时间翻倍显著降丢帧；带宽余量仍 87%；
// 换数传模块时只改这里，main.cpp 用此宏，文档无需同步）
// @ref COMM-001 §2 决策 D1（波特率 2M→921600）、§3（≈11.5KB/s，921600 利用率 ~12.5%）
#define SERIAL6_BAUDRATE 921600UL   // USART6: AnoCom/MAVLink 地面站通信

// --- 2.7 PWM 与 定时器配置 ---
#define CUSTOM_PWM_FREQUENCY 333 // PWM频率 (Hz)
#define CUSTOM_PWM_RESOLUTION 16 // PWM分辨率 (位)

// 硬件定时器 (用于驱动任务调度器)
extern HardwareTimer *TaskTimer;

/*
 * ==========================================================================================
 * [3] 物理模型与安全限制 (Physics Model & Safety Limits)
 * ==========================================================================================
 */

// --- 3.1 物理常数 ---
// 唯一事实源：TandemVec_Config.h kDefaultTandemVecParams.g（= models/aircraft-model.json §g）
extern const float G_TO_MS2;      // 1g → m/s² 换算常数（IMU 加表量程换算）
extern const float G_ACCEL_CONST; // 重力加速度 (m/s²)（推力合成/垂直估计/位置刹车共用）
extern float ekf_gravity_mps2;    // EKF 当地重力 (Somigliana, 随经纬高)；控制律消费，EKF 未初始化时 = G_ACCEL_CONST
extern float initial_mass;        // 飞行器初始质量 (kg)

// --- 3.2 控制器输出限幅 ---
extern const float MAX_THRUST;                  // 最大推力估算值 (N)
extern const float MAX_TARGET_RATE;             // 姿态环最大目标角速率 (deg/s)
extern const float MAX_CORRECTION;              // TVC最大摆角修正量 (deg)
extern const float MAX_ANGLE_COMMAND;           // 手动/定点模式最大倾角 (deg)
extern const float POS_CTRL_MAX_TILT_ANGLE_RAD; // 位置环最大倾斜角 (rad)
extern const float POS_CTRL_MAX_THRUST_COMP;    // 对应最大水平推力分量

// 手动模式下的最大角速率限制
extern const float MAX_MANUAL_rollRATE;
extern const float MAX_MANUAL_pitchRATE;
extern const float MAX_MANUAL_yawRATE;

// --- 3.3 位置与速度控制限制 ---
extern const float POS_CTRL_MAX_SPEED_CMD; // 位置环输出的最大目标水平速度 (m/s)
extern const float MAX_ACCEL_CMD;          // 速度环输出的最大水平加速度 (m/s^2)
extern const float MAX_POSITION_ERROR;     // 最大位置误差限制 (m)
extern const float MAX_LOITER_SPEED_CMD;   // Loiter模式摇杆满舵时的最大速度 (m/s)

// --- 3.4 摇杆死区设置 ---
extern const float POSITION_DEADZONE;  // 位置控制死区 (m)
extern const float VELOCITY_DEADZONE;  // 速度控制死区 (m/s)
extern const float RC_LOITER_DEADZONE; // Loiter模式摇杆死区 (Raw Value)

// ============================================================
//  4.0 控制参数集中定义（2026-08-08 C路径重构）
//  ★ 实机调参唯一入口：kFlightCtrlParams（数值 = 运行时生效值）
//  结构体与实例已抽到 include/FlightCtrlParams.h（纯平台无关，
//  固件与宿主机测试共用，保证参数不漂移）；本处仅保留遥测结构。
//  方案：docs/C路径-参数集中与遥测结构化方案.md
// ============================================================
#include "FlightCtrlParams.h"

// ============================================================
//  4.0b 控制链遥测集中（GncTelemetry — 每层中间量，供 CAN/AnoCom/Serial8）
//  写入点：flight_control.cpp（execute_attitude_controller /
//  execute_yaw_controller / mix_and_output_commands 尾部）
// ============================================================
struct GncTelemetry
{
    float error_deg[3];      // 姿态角误差（yaw 恒 0：航向为纯速率指令，无姿态回中）
    float omega_ref_dps[3];  // 目标角速率（外环输出滤波后，deg/s）
    float alpha_ref_dps2[3];   // 角度域角加速度指令（内环输出滤波后，deg/s²）
    float alpha_ref_radps2[3]; // 物理域角加速度指令（惯量逆解输入，rad/s²）
    float M_cmd[3];          // 分配前力矩（差速增益调度后，N·m）
    float M_ff[3];           // ★2026-08-10 交叉耦合前馈力矩（模型系分量，N·m；惯性解耦诊断）
    float w0_eff;            // 分配工作点（含 w0_floor，rad/s）
    float yaw_gain_sched;    // 差速回路增益调度系数
    float delta_f_deg, delta_t_deg, dw;  // 执行器指令（摆角 deg / 差速归一化）
    // 垂直/推力链路诊断（只读遥测，不参与控制）：用于固件-仿真对拍时
    // 区分"高度环输出差异"与"推力分配差异"。
    float specific_force_up_mps2; // 期望天向比力（含 1g）
    float thrust_cos_tilt;        // 推力方向倾角余弦
    float throttle_cmd_pct;       // 分配后的油门指令 [%]
    float alt_error_m;            // 高度环误差（target − current）[m]
    float target_vz_mps;          // 高度环输出的目标垂直速度 [m/s]
    bool  alloc_sat[3];      // 分配饱和标记（δ_f/δ_t/Δω）
    // 在线振动估计（只读诊断；自适应 EKF 需显式开关）
    float vib_accel_rms_g;      // 高通加速度 RMS (g)
    float vib_gyro_rms_dps;     // 高通陀螺 RMS (dps)
    float vib_accel_std_suggest;// 建议 accel_std (m/s²)
    bool  vib_high;             // 高振动标志
    // 零偏跳变监控（只告警，不直接回滚）
    float bias_jump_accel_delta; // 加计零偏相对慢基线偏差 (m/s²)
    float bias_jump_gyro_delta;  // 陀螺零偏相对慢基线偏差 (rad/s)
    uint16_t bias_jump_count;    // 累计确认跳变次数
    bool  bias_jump_hold;        // 跳变后冻结零偏修正（安全保持）
    bool  bias_slow_drift;       // 慢漂移告警（温漂等）
    float bias_drift_accel_rate; // 加计慢漂移率 (m/s²/s)
    float bias_drift_gyro_rate;  // 陀螺慢漂移率 (rad/s²)
    // 温度-零偏诊断（只估计，不直接补偿）
    float bias_temp_slope_dps_c; // 陀螺零偏温度斜率 (dps/°C)
    float bias_temp_span_c;
    float bias_temp_confidence;
};

extern GncTelemetry gnc_tel;

/*
 * ==========================================================================================
 * [4] 控制算法参数与PID实例 (Control Parameters & PID Instances)
 * ==========================================================================================
 */

// ============================================================
//  4.0a 控制参数运行时实例（★ 实机调参唯一入口）
//  固件侧（-DTANDEMVEC_FIRMWARE）为可变唯一实例，定义于 state_data.cpp，
//  以 kFlightCtrlParamsDefaults（出厂默认值）初始化；
//  上位机可经 AnoCom 0xE1 在线写入，写入后由 applyFlightCtrlParams()
//  同步到全部 PID/滤波器实例（无扰，保留控制状态）。
// ============================================================
extern FlightCtrlParams kFlightCtrlParams;
void applyFlightCtrlParams();   // 同步 kFlightCtrlParams → 12 PID + 9 控制滤波器

// --- 4.1 姿态控制 (Roll/Pitch/Yaw) — FRD 体轴映射（x_b=前/y_b=右/z_b=下）---
// ★ 2026-08-07 轴置换后执行器分配（旧注释"VTOL x_b朝上/Roll=体轴z"已废止；
// 锚点 = 手动模式摇杆直通实机验证）：Roll→体轴x→上摆δ_f，Pitch→体轴y→下摆δ_t，
// Yaw→体轴z→差速Δω
extern PositionPID rollAnglePID;  // Roll外环: error_deg[0] → 目标滚转速率
extern PositionPID rollRatePID;   // Roll内环: 体轴x速率误差 → 上摆角δ_f (deg)
extern PositionPID pitchAnglePID; // Pitch外环: error_deg[1] → 目标俯仰速率
extern PositionPID pitchRatePID;  // Pitch内环: 体轴y速率误差 → 下摆角δ_t (deg)
extern PositionPID yawAnglePID;   // Yaw外环: q_err.x → 目标航向速率
extern PositionPID yawRatePID;    // Yaw内环: 体轴x速率误差 → 差速Δω [-0.7,+0.7]（航向）

// --- 4.2 垂直控制 (Altitude/Z-Axis) ---
extern PositionPID altitudePositionPController;   // 外环: 高度误差 -> 目标垂直速度
extern PositionPID altitudeVelocityPIDController; // 内环: 垂直速度误差 -> 目标垂直加速度

// --- 4.3 水平位置控制 (Position/XY-Axis) ---
extern PositionPID northPosPID; // 北向位置环
extern PositionPID eastPosPID;  // 东向位置环
extern PositionPID northVelPID; // 北向速度环
extern PositionPID eastVelPID;  // 东向速度环

/*
 * ==========================================================================================
 * [5] 状态估计与信号滤波 (State Estimation & Signal Filtering)
 * ==========================================================================================
 */

// --- 5.1 卡尔曼滤波器 (Kalman Filters) ---
extern VerticalKF vertical_estimator;               // 基础KF (备用)
extern VerticalKF_2State vertical_estimator_2state; // 2状态KF (主要使用)

// KF噪声参数
extern const float KF_V_Q_ACCEL;
extern const float KF_V_Q_BIAS;
extern const float KF_V_Q_POS;
extern const float KF_V_Q_VEL;
extern const float LASER_NOISE_STD;
extern const float BARO_NOISE_STD;

// 水平状态估计
extern HorizontalKF_1Axis_VelMeas kf_north; // 北向估计器
extern HorizontalKF_1Axis_VelMeas kf_east;  // 东向估计器

// 水平卡尔曼滤波参数
extern float kf_h_q_accel;
extern float kf_h_q_pos;
extern float kf_h_r_vel_base;
extern float kf_h_outlier_gate;

// --- 5.2 组合导航系统 ---
extern Uart &gpsSerialPort;
extern bfs::Ubx ubx;
extern bfs::Ekf15State nav_ekf;

// --- 5.3 信号滤波器 (Signal Filters) ---
// 传感器原始数据滤波
extern ComplementaryFilter accelXFilter, accelYFilter, accelZFilter;
extern ComplementaryFilter gyroXFilter, gyroYFilter, gyroZFilter;
extern ComplementaryFilter baro_altitude_filter;
extern ComplementaryFilter temperature_filter;
extern ComplementaryFilter pressure_filter;

// 控制相关滤波
extern ComplementaryFilter altitude_rate_target_filter;
extern ComplementaryFilter rollSpeedFilter, pitchSpeedFilter, yawSpeedFilter;
extern ComplementaryFilter rollSpeedFilter2, pitchSpeedFilter2, yawSpeedFilter2;   // ★ 2026-08-09 二级滤波（级联二阶）
extern ComplementaryFilter rollAngleOutputFilter, pitchAngleOutputFilter, yawAngleOutputFilter;
extern ComplementaryFilter rollOutputFilter, pitchOutputFilter, yawOutputFilter;
extern ComplementaryFilter thrustCompN_filter, thrustCompE_filter;

// 反馈传感器滤波
extern ComplementaryFilter angleSensor1Filter;
extern ComplementaryFilter angleSensor2Filter;

// 开关量去抖动 (中值滤波)
extern MedianFilter unlockMedian;
extern MedianFilter ignitionMedian;
extern MedianFilter fuelMedianFilter;
extern ComplementaryFilter fuelLowpassFilter;

// 光流数据滤波
extern MedianFilter flowDistanceMedianFilter;
extern ComplementaryFilter flowDistanceFilter;
extern ComplementaryFilter flowVelXFilter;
extern ComplementaryFilter flowVelYFilter;
extern ComplementaryFilter estimatedVerticalVelocityFilter;

// IMU自动校准工具
extern IMU_AutoCalibrator imuCalibrator;

/*
 * ==========================================================================================
 * [6] 传感器与驱动对象 (Sensor Objects & Drivers)
 * ==========================================================================================
 */
extern MagSensor magSensor;               // 磁力计统一管理 (IST8310/QMC5883L 自动探测, Wire1)
extern Dps310 Dps310Sensor;               // 气压计
extern ICM42688 IMU;                      // 板载IMU (SPI, 10MHz)
extern MTF02P mtf02pSensor;          // 光流传感器 (MTF02P)
extern LQS48_Flow lqs48Sensor; // 光流传感器 (LQS48)

// 姿态解算算法
extern Madgwick madgwick; // Madgwick AHRS 互补滤波算法

// 辅助设备
extern StatusLED statusLed;   // 状态指示灯驱动
extern WS2812Driver ws2812Led; // WS2812 RGB状态灯 (PD15, 通用库驱动)
extern Ws2812AppStatus ws2812AppStatus; // WS2812 应用状态机 (呼吸/闪烁)
extern SPIClass FLASH_SPI;    // Flash 专用 SPI3 实例
extern W25N01GV flash;        // W25N01GV NAND Flash (SPI3)
extern W25N01GVLog flashLog;  // Flash 日志层 (环形缓冲+后台写)
extern AnoComProtocol AnoCom; // 匿名地面站协议
extern CrsfSerial crsf;       // CRSF遥控协议

/*
 * ==========================================================================================
 * [7] 系统状态与通信缓存 (System State & Communication Buffers)
 * ==========================================================================================
 */

// --- 7.1 全局标志位 (Flags) ---
extern volatile bool dps310_read_success; // 气压计读取标志
extern volatile bool imu_read_success;    // IMU读取标志
extern volatile bool isDatalogging;       // 黑匣子记录状态
extern volatile bool fuelOK;              // 燃料充足标志
extern bool trajectoryPlanningStarted;    // 轨迹规划启动标志
extern bool isLinkUp;                     // 遥控链路状态
extern bool failsafe_in_flight;               // 失控保护：飞行中失控保持解锁
extern bool newEngineDataReceived;        // 发动机数据更新标志
extern bool positionHoldEnabled;          // 定点模式激活标志

// 状态机静态变量
extern LoiterState loiter_state;                  // Loiter模式状态
extern VerticalLoiterState vertical_loiter_state; // 垂直Loiter状态

// --- 7.2 传感器数据缓存 ---
// IMU辅助变量
extern float icm_Qw, icm_Qx, icm_Qy, icm_Qz;
extern float icm_Roll, icm_Pitch, icm_Yaw;
extern float backup_ahrs_Qw, backup_ahrs_Qx, backup_ahrs_Qy, backup_ahrs_Qz;
extern float backup_ahrs_roll, backup_ahrs_pitch, backup_ahrs_yaw;
extern float ahrs_yaw_correction_rad;
extern bool use_ekf_attitude_output;
extern float icm_gyro_x, icm_gyro_y, icm_gyro_z;
extern float icm_accel_x, icm_accel_y, icm_accel_z;

// IMU 降采样累加器
extern volatile float acc_sum_x, acc_sum_y, acc_sum_z;
extern volatile float gyro_sum_x, gyro_sum_y, gyro_sum_z;
extern volatile int imu_sample_count;

// EKF双子样增量缓存
extern const int NAV_IMU_DELTA_BUFFER_SIZE;
extern volatile float imu_delta_theta_x[];
extern volatile float imu_delta_theta_y[];
extern volatile float imu_delta_theta_z[];
extern volatile float imu_delta_v_x[];
extern volatile float imu_delta_v_y[];
extern volatile float imu_delta_v_z[];
extern volatile float imu_delta_dt_s[];
extern volatile float imu_delta_time_sum_s;
extern volatile int imu_delta_sample_count;
extern volatile bool imu_delta_overflow;

// 滤波后的角速度（Vector3 类型定义在 QuaternionMath.h 中，仅 main.cpp 包含）
// 此处使用匿名结构体前向声明兼容类型
struct Vector3;
extern Vector3 current_omega_dps_body_filtered;

// 气压计数据：高度统一为起飞点向上为正的相对高度 (m)
extern float imu_die_temp_c;               // ICM42688 芯片温度 (°C)，1 Hz 更新
extern float temperature_raw, pressure_raw;
extern float temperature, pressure;
extern float baro_altitude_raw, baro_altitude;
extern float baro_altitude_offset;

// 气压高度 EKF 融合遥测 (仅在 BFS_EKF_BARO_ALTITUDE_UPDATE 启用时更新)
extern float baro_ekf_innovation_m;   // 最新创新项 (NED-D 方向, m)
extern float baro_ekf_test_ratio;      // NIS/门限比值 (>1 表示被拒绝)
extern bool baro_ekf_fused;           // 本帧是否融合成功
extern float ekf_accel_bias_z_mps2;  // EKF 估计 Z 轴加表零偏 (机体系 FRD, 下为正, m/s^2)

// 气压健康门控（opt-in BFS_BARO_HEALTH_GATE）：检测 Drift/Spike/Stuck 后
// 暂停气压融合与“气压替换 GNSS 垂直”仲裁；SIL 用 --baro-health-gate 做 A/B。
extern bool g_baro_health_gate_enable;
// GNSS 速度野值软恢复（opt-in，默认关；SIL --gnss-vel-subset 覆盖）
extern bool g_gnss_vel_subset_recovery_enable;
// 速度子集允许触发的最大位置残差 [m]（默认 25，§4.24t；SIL --gnss-vel-subset-posres）
extern float g_gnss_vel_subset_max_pos_resid_m;
// 每个"完整更新历元"允许的子集触发次数（默认 1，§4.24w；255=不限，SIL --gnss-vel-subset-maxfires）
extern int g_gnss_vel_subset_max_fires_per_epoch;
extern bool g_baro_health_suspect;       // 锁存后的可疑标志（含 5 s 恢复保持）
extern uint16_t g_baro_health_flags;     // 1=Drift 2=Spike 4=Stuck
extern uint16_t g_baro_health_flags_cum; // 本次运行内出现过的 flag 位或
extern uint32_t g_baro_health_sus_count; // Suspect 进入次数
extern float g_baro_health_drift_mps;    // 残差斜率低通 (m/s)
extern uint32_t g_baro_health_skips;     // 因可疑被跳过的气压 EKF 更新次数

// 光流全局数据实例
extern volatile OpticalFlowData flow_data;

// UART7 光流传感器自动识别状态
extern volatile FlowSensorType active_flow_sensor; // 当前锁定的光流传感器类型
extern volatile uint32_t flow_sensor_baud;         // 当前 UART7 实际波特率

// 室内无 GNSS 时光流速度融合 EKF 的统计（导航任务更新，DBG `flow` 查看）
extern volatile uint32_t g_flow_fusion_attempts;
extern volatile uint32_t g_flow_fusion_fused;
extern volatile uint32_t g_flow_fusion_rejected;
extern volatile uint32_t g_flow_fusion_reanchors; // 连续拒绝触发的速度重锚次数
extern volatile uint32_t g_baro_vel_aid_attempts;
extern volatile uint32_t g_baro_vel_aid_fused;
extern volatile uint32_t g_baro_vel_aid_rejected;
extern bool g_baro_vel_aid_enable;
extern bool g_flare_enable;


/*
 * ==========================================================================================
 * 通信协议与数据缓存区
 * ==========================================================================================
 */

// 发动机控制器通信协议
#define ENGINE_FRAME_LEN 15
#define ENGINE_PAYLOAD_LEN 12

extern uint8_t engineDataReceiveBuffer[];
extern uint8_t engineDataReceiveIndex;
extern float receivedP1;
extern float receivedP2;
extern float receivedValveControl;

// 电池电压/电流监测（ADC_BATT PC5 采样，供 ELRS/AnoCom 遥测共用）
extern volatile float bat_voltage_mv;   // 电池电压 (mV)
extern volatile float bat_current_ca;   // 电池电流 (0.1A，未接电流采样时为 0)

// τm 一阶观测器转速估计（flight_control.cpp mix 层更新；2026-08-10 暴露为全局
// 供 AnoVars 通用变量上报——观测值 = 实际转速实时估计）
extern float g_wf_est;   // 前电机观测转速 (rad/s)
extern float g_wt_est;   // 尾电机观测转速 (rad/s)

// 遥控器 CRSF 协议数据
#define CRSF_BUFFER_SIZE 26
#define RC_INPUT_MAX_CHANNELS 16
#define CRSF_FRAME_TYPE_RC_CHANNELS_PACKED 0x16
#define CRSF_PAYLOAD_SIZE_RC_CHANNELS 22

extern uint16_t raw_rc_values[];
extern uint16_t raw_rc_count;

// 机载计算机引导指令通信
#define GUIDANCE_CMD_FRAME_LEN 14
#define GUIDANCE_CMD_HEADER 0xAA
#define GUIDANCE_CMD_TAIL 0x55
#define GUIDANCE_CMD_PAYLOAD_LEN 12
#define GUIDANCE_CMD_TIMEOUT_MS 500

extern volatile bool new_guidance_command_received;
extern volatile unsigned long last_guidance_command_millis;
extern float guidance_accel_E_cmd;
extern float guidance_accel_N_cmd;
extern float guidance_accel_U_cmd;

// --- 7.3 导航与状态估计结果 ---
extern float estimated_height;
extern float estimated_velocity;
extern float fused_north_pos, fused_north_vel;
extern float fused_east_pos, fused_east_vel;

// 2状态垂直KF独立输出（与EKF并行运行，仅供遥测对比，不参与控制）
extern float vfk_height;   // 垂直KF估计高度 (m, 向上为正)
extern float vfk_velocity; // 垂直KF估计速度 (m/s, 向上为正)

// 相对位置与起飞点原点：原点为 WGS84 LLA，位置为 NED (Down 为正)
extern bool is_origin_position_set;
extern bool is_origin_lla_set;
extern float origin_north, origin_east, origin_down;
extern double origin_lat_deg, origin_lon_deg, origin_lat_rad, origin_lon_rad, origin_alt_m;
// GNSS 垂直一致性重锚门控（opt-in）
extern bool g_gnss_v_gate_enable;
extern bool g_gnss_vert_suspect;
extern uint32_t g_gnss_vert_gate_frames;
extern float relative_north, relative_east, relative_down;
extern float ubx_relative_north, ubx_relative_east, ubx_relative_down;

// 导航系统状态标志
extern bool nav_system_initialized;
extern unsigned long last_ekf_update_us;
extern bool nav_initialized_with_fake_origin;
extern bool nav_has_real_gnss_anchor;

// --- 导航数据源选择 ---
// DETA100 为可选外部模块，内置 ICM42688+EKF+UBX 为默认主路径。
// 上电自动检测 Serial4 上 DETA100 协议帧，决定数据源。
enum class NavDataSource : uint8_t
{
  INTERNAL = 0, // 内置 ICM42688 + EKF + UBX GNSS（默认）
  DETA100  = 1  // 外接 DETA100 组合导航模块
};
extern NavDataSource nav_data_source;
extern bool deta100_online;             // DETA100 模块在线标志（上电检测后锁定）
extern bool deta100_detect_done;        // 上电检测是否已完成
extern uint32_t deta100_last_frame_ms;  // DETA100 最后收到有效帧的时间戳
extern const uint32_t DETA100_DETECT_TIMEOUT_MS; // 上电检测窗口时长
extern const uint32_t DETA100_ONLINE_TIMEOUT_MS;  // 运行中判定离线超时
extern uint32_t last_gnss_data_ms;
extern const uint32_t GNSS_NAV_DATA_TIMEOUT_MS;
extern uint32_t last_gnss_nav_valid_ms;
extern const uint32_t GNSS_NAV_DROPOUT_HOLD_MS;

// GNSS 融合质量门限与运行诊断
extern const float GNSS_MAX_HORZ_ACC_M;
extern const float GNSS_MAX_VERT_ACC_M;
extern const float GNSS_MAX_SPD_ACC_MPS;
extern const float GNSS_MAX_PDOP;
extern const int8_t GNSS_MIN_SV;
extern const float GNSS_DOWNGRADE_MAX_SPD_ACC_MPS;
extern const uint32_t GNSS_PUMP_MAX_BYTES;

struct GnssRuntimeStatus
{
  bool parser_ready = true;                  // UBX 解析器是否可用
  bool has_measurement = false;              // 是否收到过至少一帧 GNSS epoch
  bool has_last_tow = false;                 // 是否记录过上一帧融合 iTOW
  uint32_t last_tow_ms = 0;                  // 上一帧融合 iTOW，用于去重
  bool has_last_epoch_tow = false;           // 是否记录过最近原始 epoch iTOW
  uint32_t last_epoch_tow_ms = 0;            // 最近原始 epoch iTOW
  uint32_t epoch_count = 0;                  // 累计成功入队 epoch 数
  uint32_t fix3d_count = 0;                  // 质量通过的 3D fix 次数
  uint32_t duplicate_itow_count = 0;         // 重复 iTOW 计数
  uint32_t tow_mismatch_count = 0;           // PVT/EOE iTOW 不一致计数
  uint32_t quality_fail_count = 0;           // 质量门限拒绝计数
  uint32_t downgrade_count = 0;              // 降级为速度-only 融合次数
  uint32_t pending_epoch_count = 0;          // 当前待消费 epoch 数
  uint32_t parser_queue_overflow_count = 0;  // UBX 队列溢出计数
  uint32_t parser_eoe_without_pvt_count = 0; // EOE 无 PVT 计数
  uint32_t parser_nav_pvt_count = 0;         // NAV-PVT 合法消息计数
  uint32_t parser_nav_eoe_count = 0;         // NAV-EOE 合法消息计数
  bool has_obs = false;                      // 是否有原始观测快照
  float last_obs_fix_type = 0.0f;
  float last_obs_num_sv = 0.0f;
  float last_obs_h_acc_m = -1.0f;
  float last_obs_v_acc_m = -1.0f;
  float last_obs_s_acc_mps = -1.0f;
  float last_obs_p_dop = -1.0f;
  float last_obs_vel_n_mps = 0.0f;
  float last_obs_vel_e_mps = 0.0f;
  float last_obs_vel_d_mps = 0.0f;
  Eigen::Vector3d last_obs_lla_rad_m = Eigen::Vector3d::Zero();
  float last_test_ratio = 0.0f;              // 最近一次 EKF GNSS 门控比
  float last_measurement_age_s = 0.0f;       // 最近 epoch 映射到 MCU 时间轴后的真实回放年龄
  uint32_t time_mapping_invalid_count = 0;   // epoch 缺少本地接收时间，退回固定延迟的次数
  // 无 PPS 批处理延时估计（默认关闭，仅诊断；不改变 age）
  bool auto_delay_valid = false;
  uint32_t auto_delay_last_ms = 0;
  float auto_delay_tau_s = 0.0f;
  float auto_delay_residual_mps = 0.0f;
  float auto_delay_sharpness = 0.0f;
  // 拒绝原因/样本计数：只给一个 valid 位不足以区分"没激励""数据被粗差门
  // 吃掉""代价平底"三种病因，而三者修法完全不同（2026-10-27 导弹段定位）。
  uint8_t auto_delay_reject = 0;   ///< InsGnssDelayAutoReject
  uint16_t auto_delay_n_gnss = 0;  ///< 本次参与拟合的 GNSS 样本数（剔除后）
  uint16_t auto_delay_n_erased = 0;///< 本次被粗差门剔除的 GNSS 样本数
  float auto_delay_speed_max_mps = 0.0f; ///< 窗口最大参考速度（门限自适应依据）
  // ★ 应用侧可观测：`auto_delay_valid` 是**锁存**标志（只在估计成功那一次刷新），
  //   它=1 不代表本历元真的用了估计值。导弹 SIL 实测四臂 τ̂ 中位 56~124 ms、
  //   valid=20/20，但位置误差与完全不补偿相同 ⇒ 必须能直接看到"有没有应用、
  //   被哪一条挡住"，否则这条通路是不可诊断的黑盒（2026-10-27）。
  uint32_t auto_delay_applied_count = 0;  ///< 估计值实际替换固定先验的次数
  uint8_t auto_delay_apply_block = 0;     ///< 最近一次未应用的原因位图
  //   bit0=高振动否决 bit1=估计过期(>5 s) bit2=锐度不足(<0.003)
  //   bit3=与先验偏差超门限 bit4=时间映射无效(不进本分支)
  // GNSS 慢漂移/欺骗一致性监测（2026-10，ins_gnss_consistency.h）
  bool consistency_suspect = false;
  uint32_t consistency_reject_count = 0;
  uint32_t consistency_suspect_count = 0;
  float consistency_accel_res = 0.0f;
  float consistency_pos_res = 0.0f;
  float consistency_flow_res = 0.0f;
  float consistency_baro_span = 0.0f;
  uint8_t consistency_reason = 0;
};
extern GnssRuntimeStatus gnss_status;

/**
 * @brief 磁测量运行诊断（只读，供遥测/DBG）
 * @note `last_gate_reason` 对应 bfs::MagGateReason 的整数值；NAV 层写，
 *       通信层只读。`init_source`: 0=pseudo，1=real mag。
 */
struct MagRuntimeStatus
{
  bool present = false;
  bool fresh = false;
  bool cal_valid = false;
  bool ref_valid = false;
  uint32_t seq = 0;
  uint32_t age_ms = 0;
  float norm_ut = 0.0f;
  float norm_dev_ut = 0.0f;
  float dip_deg = 0.0f;
  float dip_dev_deg = 0.0f;
  float heading_error_deg = 0.0f;
  float track_dev_deg = 0.0f;
  float last_kick_deg = 0.0f;    // 最近一次磁更新对 EKF 航向的瞬时修正量
  float last_kick_rp_deg = 0.0f; // 最近一次磁更新对 roll/pitch 的最大瞬时修正量
  uint8_t last_gate_reason = 0;
  uint8_t init_source = 0;
  uint32_t fused = 0;
  uint32_t gate_rejected = 0;
  uint32_t ekf_rejected = 0;
};
extern MagRuntimeStatus mag_status;

// 绝对航向辅助与位置保持门控（2026-10，HeadingAidingGate.h）：
// 磁矢量/YawGsf 最近成功融合时间不足时，AUTO_POSITION/GUIDED 视为航向不可观，
// 上层应降级为不含水平位置闭环的 AUTO_ALTITUDE。`g_pos_hold_degraded` 是门控
// 建议，`g_mode_degraded_heading` 是本拍实际发生的模式降级。
extern bool g_heading_aiding_valid;
extern float g_heading_aiding_age_s; // <0 表示从未融合
extern bool g_pos_hold_degraded;
// 是否允许 ratchet heading hold（无磁悬停=false，仅位置保持可用）。
extern bool g_heading_hold_allowed;
// 航向源模式（2026-10，无磁/磁仅初始对准）：
// 0=MAG_ONLY（默认，磁连续融合）；1=MAG_INIT_ONLY（磁仅初始化，之后靠运动
// EKF/陀螺）；2=NO_MAG_MANUAL（无磁，起飞前手动对准 + 运动 EKF）。
extern uint8_t g_heading_aid_mode;
extern float g_heading_align_deg; // 手动对准航向（度，真北顺时针）
// 无磁离地时陀螺零偏 z 限幅 [rad/s]：防止 GNSS 更新把真实偏航率吸收成零偏。
// 0.01 rad/s ≈ 0.57°/s，20 s 累积 ≤11.5°，落在短时保持窗口内。
extern float g_nomag_bias_z_max_radps;
// 最近一次有效 GNSS 距今时间 [s]；从未拿到过 GNSS 时为 1e5（按“永远失联”
// 触发上层失效保护），不是 0。
extern float g_gnss_outage_age_s;
// 无磁 20 s 保持窗口剩余时间 [s]；-1 表示当前不是无磁模式（遥测/告警用）。
extern float g_nomag_hold_left_s;
// 地面静止校准（无磁定点前置条件）。默认要求连续静止 30 s；
// SIL 可把 required 设为 0 只跑飞行段回归。
extern bool g_imu_static_calibrated;
extern float g_static_calib_s;
extern float g_static_calib_required_s;
// 研究用：EKF 加速度过程噪声缩放（默认 1.0，导航初始化前生效）。
extern float g_ekf_q_accel_scale;
// GNSS 失联后重捕获的一次性目标重锚请求：位置保持把当前点作为新目标，
// 避免失联期间累积的位置误差在重捕获后变成大姿态指令。
extern bool g_pos_hold_reanchor;
extern bool g_mode_degraded_heading;
// 本拍长 GNSS 失联水平保护策略触发（opt-in 宏；若航向门控已先降级，
// 则只作诊断标志，不代表本宏改变了模式）。
extern bool g_mode_degraded_gnss;
// SIL 测试用：进入 AUTO_POSITION 时叠加的位置目标偏移 [m]（固件默认 0）。
extern float g_sil_pos_leg_n_m;
extern float g_sil_pos_leg_e_m;

// IMU 健康监测（2026-10，ins_imu_health.h）；0=Ok, 1=Warning, 2=Failed。
extern uint8_t g_imu_health_state;
extern uint16_t g_imu_health_flags;
extern float g_imu_health_frozen_s;
extern float g_imu_health_no_sample_s;
extern float g_imu_health_sample_rate_hz;

// 导航 IMU 增量源诊断（NavImuIncrementSelect.h，仅
// BFS_NAVIGATION_RAW_IMU_INCREMENT 打开时累加；宏关闭恒为 0）。
// 回退占空比 = fb/(raw+fb)，用于实机确认健康键误触发率。
extern uint32_t g_nav_increment_raw_ticks;
extern uint32_t g_nav_increment_fallback_ticks;

// GNSS 一致性拒绝开关：默认只告警；SIL/后续实机验证通过后再启用拒绝。
// （2026-10 仿真发现 1 s 窗口在爬升/位置保持动态下会被姿态误差触发，
//  因此基线默认 warn-only，避免把正常 GNSS 拒掉。）
extern bool g_gnss_consistency_reject_enabled;

extern bfs::AidSourceStatusTracker aid_tracker;
extern uint32_t static_start_time;
extern bool is_static_confirmed;

// --- 7.4 控制目标与输出 ---
// ★ 2026-08-08 C路径重构：rollRateTarget/pitchRateTarget/yawRateTarget、
//   error_*_deg、roll/pitch/yaw_output 已收拢进 gnc_tel（见 §4.0b）
extern float rollTarget, pitchTarget;
extern float targetNorth, targetEast;
extern float targetVelNorth, targetVelEast;
extern float target_altitude;
extern float target_vertical_velocity;
extern float altitudeRateTarget;
extern bool autoAltitudeMode;
extern float target_accel_z_up_global;
extern float tvcTargetAngle1, tvcTargetAngle2;
extern float thrust_comp_N, thrust_comp_E;
extern float throttlePercent;

// ---- 在线参数辨识结果（★ 纯观测，不参与控制回路）----
// 由 runGNCExecutive 每拍写入，供地面站/黑匣子记录。
// b = I_nominal / I_actual：>1 表示真实惯量小于名义值，<1 表示大于。
// 首飞用途：飞几个架次后看 b_est 是否稳定收敛，再人工决定是否调整增益。
// 详见 include/TandemVec_OnlineID.h 与 AGENTS.md §在线辨识。
extern float id_b_est[3];      // 惯量比估计 [roll, pitch, yaw]
extern float id_d_est[3];      // 总扰动估计 (rad/s²)
extern float id_cg_mm;         // 重心偏移估计 (mm，仅悬停时更新)
extern bool  id_excited[3];    // 各轴激励是否充分（估计是否在更新）
extern float id_kp_suggest[3]; // 建议的 Kp_r（= Kp_nominal/√b，仅供参考，未生效）
extern float ch1_output, ch2_output;
extern float ch3_output, ch4_output;

// 执行器指令（物理量纲，全模式有效：mix 输出级统一写入）——
// 供 AnoCom 0x40 执行器帧（上位机 TVC 摆角/推力实时显示）。
// 摆角 deg：+ = 有向摆角（★2026-08-07 轴置换：上摆=滚转执行器 / 下摆=俯仰执行器），限幅 ±MAX_CORRECTION
extern float g_tvc_upper_deg;   // 上摆座角指令 deg（滚转）
extern float g_tvc_lower_deg;    // 下摆座角指令 deg（俯仰）

// 当前飞行模式 (由 runGNCExecutive 写入, 供 handleAnoCom 遥测发送)
extern ControlMode g_current_flight_mode;
// 当前解锁状态 (由 runGNCExecutive 写入, 供 handleAnoCom 遥测发送)
extern bool g_is_unlocked;

// 执行器故障监测（2026-10，ActuatorFaultMonitor.h）
// 只读诊断标志：不直接改变控制输出；后续故障重构由控制层消费。
extern bool g_actuator_servo_jam;
extern bool g_actuator_thrust_loss;
extern bool g_actuator_controlled_descent_request;

/*
 * ==========================================================================================
 * [8] 任务调度系统 (Task Scheduler)
 * ==========================================================================================
 */
#define MAX_TASKS 30
extern Task tasks[];
extern uint8_t taskCount;

/*
 * ==========================================================================================
 * DETA100 模块全局变量（数据包实例和标志位）
 * ==========================================================================================
 */
extern MedianFilter HeadingFilter, PitchFilter, RollFilter;
extern IMUPacket IMU_Packet;
extern AHRSPacket AHRS_Packet;
extern INS_GNSSPacket INS_GNSS_Packet;
extern GeodeticPosPacket Geodetic_Pos_Packet;
extern StatusPacket Status_Packet;
extern bool Data_of_IMU, Data_of_AHRS, Data_of_INS_GNSS, Data_of_Geodetic_Pos, Data_of_Status;
extern uint8_t IMU_Data[], AHRS_Data[], INS_GNSS_Data[], Geodetic_Pos_Data[], Status_Data[];
extern uint8_t Fd_data[];
