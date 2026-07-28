/**
 * @file state_data.h
 * @brief 全局状态数据中枢 — 所有跨模块共享的类型定义与变量声明
 *
 * 本头文件集中管理飞控系统中所有跨模块共享的：
 *   - 枚举类型 (ControlMode, LoiterState 等)
 *   - 结构体类型 (ControlInputs_t, ControlOutputs_t, OpticalFlowData, Task)
 *   - 全局变量的 extern 声明
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
#include "AnoComProtocol.h"
#include "CrsfSerial.h"
#include "crsf.h"
#include "common_rc.h"
#include "IST8310.h"
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
struct OpticalFlowData
{
  float distance_m;      // 倾斜补偿后的高度 (m)
  uint8_t range_quality; // 测距信号质量
  bool is_range_valid;   // 测距数据有效标志
  float velocity_x_mps;  // 旋转补偿后的X轴速度 (m/s, Body Frame)
  float velocity_y_mps;  // 旋转补偿后的Y轴速度 (m/s, Body Frame)
  uint8_t flow_quality;  // 光流信号质量
  bool is_flow_valid;    // 光流数据有效标志
  uint32_t timestamp_ms; // 时间戳
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

// 控制输出结构体
// 内环 PID 输出角加速度(rad/s²)，底层控制分配根据物理模型逆解执行器指令
// 轴向：alpha_roll=体轴z（侧倾），alpha_pitch=体轴y（俯仰），alpha_yaw=体轴x（VTOL航向）
typedef struct
{
  float throttle_percent; // 油门百分比 (0-100)
  float alpha_roll;       // rad/s²  体轴z角加速度（VTOL侧倾/前摆δ_f）
  float alpha_pitch;      // rad/s²  体轴y角加速度（俯仰/尾摆δ_t）
  float alpha_yaw;        // rad/s²  体轴x角加速度（VTOL航向/差速Δω）
} ControlOutputs_t;

// 任务调度结构体
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
extern HardwareSerial Serial1; // USART1: ELRS接收机 (CRSF输入)
extern HardwareSerial Serial2; // USART2: 发动机控制器 / 数据转发
extern HardwareSerial Serial3; // USART3: 黑匣子数据记录 (高速)
extern HardwareSerial Serial4; // UART4:  DETA100 IMU/GNSS模块
extern HardwareSerial Serial5; // UART5:  上位机轨迹规划接口
extern HardwareSerial Serial6; // USART6: AnoCom地面站通信
extern HardwareSerial Serial7; // UART7:  光流传感器接口
extern HardwareSerial Serial8; // UART8:  USB Type-C 调试输出

// 串口引用别名
extern HardwareSerial &receiverSerial;    // 遥控接收
extern HardwareSerial &transmitterSerial; // 外部设备/发动机
extern HardwareSerial &opticalFlowSerial; // 光流

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
extern const float G_TO_MS2;      // 长沙地区标准重力常数
extern const float G_ACCEL_CONST; // 当地重力加速度 (m/s^2, 长沙)
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

/*
 * ==========================================================================================
 * [4] 控制算法参数与PID实例 (Control Parameters & PID Instances)
 * ==========================================================================================
 */

// --- 4.1 姿态控制 (Roll/Pitch/Yaw) — VTOL 体轴映射 ---
// VTOL（x_b朝上）：Roll=体轴z（侧倾），Pitch=体轴y（俯仰），Yaw=体轴x（航向）
extern PositionPID rollAnglePID;  // Roll外环: q_err.z → 目标侧倾速率 (deg/s)
extern PositionPID rollRatePID;   // Roll内环: 体轴z速率误差 → 前摆角δ_f (deg)
extern PositionPID pitchAnglePID; // Pitch外环: q_err.y → 目标俯仰速率
extern PositionPID pitchRatePID;  // Pitch内环: 体轴y速率误差 → 尾摆角δ_t (deg)
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
extern HardwareSerial &gpsSerialPort;
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
extern IST8310 compass;                   // 磁力计
extern Dps310 Dps310Sensor;               // 气压计
extern ICM42688 IMU;                      // 板载IMU (SPI, 10MHz)
extern MTF02P opticalFlowSensor;          // 光流传感器 (MTF02P)
extern LQS48_Flow opticalFlowSensorLQS48; // 光流传感器 (LQS48)

// 姿态解算算法
extern Madgwick madgwick; // Madgwick AHRS 互补滤波算法

// 辅助设备
extern StatusLED statusLed;   // 状态指示灯驱动
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
extern float temperature_raw, pressure_raw;
extern float temperature, pressure;
extern float baro_altitude_raw, baro_altitude;
extern float baro_altitude_offset;

// 气压高度 EKF 融合遥测 (仅在 BFS_EKF_BARO_ALTITUDE_UPDATE 启用时更新)
extern float baro_ekf_innovation_m;   // 最新创新项 (NED-D 方向, m)
extern float baro_ekf_test_ratio;      // NIS/门限比值 (>1 表示被拒绝)
extern bool baro_ekf_fused;           // 本帧是否融合成功
extern float ekf_accel_bias_z_mps2;  // EKF 估计 Z 轴加表零偏 (机体系 FRD, 下为正, m/s^2)

// 光流全局数据实例
extern volatile OpticalFlowData flow_data;

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

// 相对位置与起飞点原点：原点为 WGS84 LLA，位置为 NED (Down 为正)
extern bool is_origin_position_set;
extern bool is_origin_lla_set;
extern float origin_north, origin_east, origin_down;
extern double origin_lat_deg, origin_lon_deg, origin_lat_rad, origin_lon_rad, origin_alt_m;
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
};
extern GnssRuntimeStatus gnss_status;
extern bfs::AidSourceStatusTracker aid_tracker;
extern uint32_t static_start_time;
extern bool is_static_confirmed;

// --- 7.4 控制目标与输出 ---
extern float rollTarget, pitchTarget;
extern float yawRateTarget;
extern float rollRateTarget, pitchRateTarget;
extern float error_roll_deg, error_pitch_deg, error_yaw_deg;
extern float targetNorth, targetEast;
extern float targetVelNorth, targetVelEast;
extern float target_altitude;
extern float target_vertical_velocity;
extern float altitudeRateTarget;
extern bool autoAltitudeMode;
extern float target_accel_z_up_global;
extern float tvcTargetAngle1, tvcTargetAngle2;
extern float thrust_comp_N, thrust_comp_E;
extern float roll_output;
extern float pitch_output;
extern float yaw_output;  // 航向角加速度 alpha_yaw (rad/s²)，遥测用
extern float throttlePercent;
extern float ch1_output, ch2_output;
extern float ch3_output, ch4_output;

// 当前飞行模式 (由 runGNCExecutive 写入, 供 handleAnoCom 遥测发送)
extern ControlMode g_current_flight_mode;
// 当前解锁状态 (由 runGNCExecutive 写入, 供 handleAnoCom 遥测发送)
extern bool g_is_unlocked;

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
