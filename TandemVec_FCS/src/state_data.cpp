/**
 * @file state_data.cpp
 * @brief 全局状态数据定义 — 所有跨模块共享变量的唯一定义点
 *
 * 本文件定义 state_data.h 中声明的所有全局变量。
 * 每个变量只在此文件中定义一次，避免多重定义错误。
 * 所有变量按功能分区组织，与 state_data.h 的声明顺序一一对应。
 *
 * 注意: Vector3 类型由 QuaternionMath.h 提供，但该头文件包含非 inline 函数定义，
 * 不能在多个 .cpp 中包含。current_omega_dps_body_filtered 的定义移至 main.cpp。
 */
#include "state_data.h"
#include "TandemVec_Config.h"   // 引入唯一参数源，使 initial_mass 与 P.m 保持同步

/*
 * ==========================================================================================
 * [2] 硬件资源配置
 * ==========================================================================================
 */

// --- 2.1 串口资源分配 ---
HardwareSerial Serial1(PA10, PA9); // USART1: ELRS接收机 (CRSF输入) [420000 baud]
HardwareSerial Serial2(PD6, PD5);  // USART2: 发动机控制器 / 数据转发 [921600 baud]
HardwareSerial Serial3(PD9, PD8);  // USART3: 黑匣子数据记录 (高速) [1500000 baud]
HardwareSerial Serial4(PD0, PD1);  // UART4:  DETA100 IMU/GNSS模块 (输入) [921600 baud]
HardwareSerial Serial5(PD2, PC12); // UART5:  上位机轨迹规划接口 [921600 baud]
HardwareSerial Serial6(PC7, PC6);  // USART6: AnoCom地面站通信 (数传) [921600 baud]
HardwareSerial Serial7(PE7, PE8);  // UART7:  光流传感器接口 [921600 baud]
HardwareSerial Serial8(PE0, PE1);  // UART8:  USB Type-C 调试输出 [921600 baud]

// 串口引用别名
HardwareSerial &receiverSerial = Serial1;    // 遥控接收
HardwareSerial &transmitterSerial = Serial2; // 外部设备/发动机
HardwareSerial &opticalFlowSerial = Serial7; // 光流

// --- 2.4 系统指示与控制引脚 ---
const int LED_yellow = PE4; // 黄色LED: 传感器异常/警告
const int LED_green = PE5;  // 绿色LED: 系统运行/任务心跳
const int ignition = PE6;   // 点火控制: MOS管高电平触发

// --- 2.5 总线引脚与对象 ---
TwoWire Wire1(MAG_SDA_PIN, MAG_SCL_PIN);  // 自定义I2C总线1 (磁力计)
TwoWire IIC2(BARO_SDA_PIN, BARO_SCL_PIN); // 自定义I2C总线2 (气压计)

// SPI引脚定义 (板载IMU)
const int SPI_MOSI = PA7;
const int SPI_MISO = PA6;
const int SPI_SCLK = PA5;
const int SPI_CS = PA4;

// 硬件定时器
HardwareTimer *TaskTimer = new HardwareTimer(TIM8);

/*
 * ==========================================================================================
 * [3] 物理模型与安全限制
 * ==========================================================================================
 */

// --- 3.1 物理常数（全部从可测量物理量推导，来源见注释）---
// 重力加速度唯一事实源：TandemVec_Config.h kDefaultTandemVecParams.g（源自 models/aircraft-model.json）。
// G_TO_MS2（IMU 1g→m/s² 换算）与 G_ACCEL_CONST（推力合成/垂直估计/位置刹车）均引用之，避免多源漂移。
const float G_TO_MS2 = kDefaultTandemVecParams.g;
const float G_ACCEL_CONST = kDefaultTandemVecParams.g;

// EKF 按当前经纬高计算的 WGS-84 当地重力（Somigliana），控制律消费；
// 初始/未初始化时回退到 G_ACCEL_CONST（模型默认 9.81），由 navigation_task 每次 EKF 输出刷新。
float ekf_gravity_mps2 = G_ACCEL_CONST;
float initial_mass = kDefaultTandemVecParams.m;  // 唯一来源：TandemVec_Config.h §质量
                                                  // 修改质量请改 kDefaultTandemVecParams.m

// -------- 悬停基准点物理量（40%油门, w0=460rad/s）--------
// T_hover = kT × w0² = 1.04e-5 × 460² = 2.20 N
// My_max_hover = b × T_hover × sin(25°) = 0.315×2.20×0.4226 = 0.293 N·m
//
// ★ I为组件级别估算(I≈0.022), 未实测, 不确定度较大
//   α_max = My_max/I — 既依赖I又依赖kT, 两者都不精确
//   → 不从此推导PID限幅的具体数值
//   → 从保守起点开始, 实飞逐次增大α限幅, 直到响应满意或振荡
//   → 最终的α限幅值就是系统的标定结果, 隐含了真实惯性和推力的校准

// --- 3.2 控制器输出限幅（全部从上述物理量推导）---

const float MAX_THRUST = 27.5f;
//  来源: 2×kT×wMax² = 2×1.04e-5×1150² = 27.5N (2212+9047+3S)

const float MAX_TARGET_RATE = 50.0f;
//  来源: α_max_hover × τ_rise_1s ≈ 49 deg/s² × 1s ≈ 50 deg/s
//  外环输出的目标角速率上限。悬停时 α_max≈49deg/s²，选择1秒内可达的速率。

const float MAX_CORRECTION = 15.0f;  // 2026-08-07 实机：TVC 摆角限幅收紧到 15°
//  来源: dMax = 0.436rad = 25° (TandemVec_Config.h)。舵机物理行程。

const float MAX_ANGLE_COMMAND = 30.0f;
//  来源: 手动最大倾角(deg)。需小于90°-dMax=65°保持执行器余量。
const float POS_CTRL_MAX_TILT_ANGLE_RAD = 15.0f * DEG_TO_RAD;
//  来源: 位置环最大倾角 15°。sin(15)=0.26, 水平加速 a_max=0.26×g≈2.5m/s²。
const float POS_CTRL_MAX_THRUST_COMP = sinf(POS_CTRL_MAX_TILT_ANGLE_RAD);

const float MAX_MANUAL_rollRATE  = 50.0f;  // = MAX_TARGET_RATE
const float MAX_MANUAL_pitchRATE = 50.0f;
const float MAX_MANUAL_yawRATE   = 35.0f;  // VTOL 悬停：yaw 摇杆→差速（绕 x_b）保守速率；差速电机时延+低效能

// --- 3.3 位置与速度控制限制 ---
const float POS_CTRL_MAX_SPEED_CMD = 1.5f;
const float MAX_ACCEL_CMD = 2.6f;
const float MAX_POSITION_ERROR = 5.0f;
const float MAX_LOITER_SPEED_CMD = 1.5f;

// --- 3.4 摇杆死区设置 ---
const float POSITION_DEADZONE = 0.1f;
const float VELOCITY_DEADZONE = 0.02f;
const float RC_LOITER_DEADZONE = 25.0f;

/*
 * ==========================================================================================
 * [4] 控制算法参数与PID实例
 * ==========================================================================================
 */

// --- 4.1 姿态控制 (Roll/Pitch/Yaw) — 解析最优增益 ---
//
//  级联P-P闭环=二阶系统: θ/θ_ref=ωn²/(s²+2ζωn·s+ωn²)
//    ωn² = (Kp_r×57.3) × Kp_a,    2ζωn = Kp_r×57.3
//
//  Roll/Pitch: Kp_a=5.0, Kp_r=0.30 → Kp_r_eff=17.2 → ζ=0.93, ωn=9.3(1.5Hz)
//    (当前=保守偏阻尼值，避免悬停振荡。实飞可增至 Kp_a=5.5 Kp_r=0.30→ζ=0.78)
//  Yaw: Kp_a=4.0, Kp_r=0.15 → ζ=0.83, ωn=5.9(0.94Hz) (差速通道保守)
//
//  积分: Ki=0.0003(内环) → 不干扰ζ, 配平τ≈7s → 消CG偏移/推力不对称/风偏静差
//    PositionPID no-dt约定: Ki×200=等效连续增益。0.0003×200=0.06/s
// 2026-08-07 实机再整定（差速抑超调 / TVC 加阻尼，按上方二阶公式反解）：
//  差速轴(roll/Δω, 绕x_b): 被控对象含电机一阶滞后 τm=0.28s，
//    带宽必须 ≪1/τm → 取 ωn≈1.8, ζ≈1.1（过阻尼无超调）:
//    Kp_r=0.07 → 2ζωn=4.0; Kp_a=0.8 → ωn=√(4.0×0.8)=1.79, ζ=1.12
//  TVC轴(pitch尾摆/yaw前摆): 舵机快(333Hz)、无电机滞后，可提高带宽补阻尼，
//    取 ωn≈4.6, ζ≈0.93: Kp_r=0.15 → 2ζωn=8.6; Kp_a=2.5 → ωn=4.64
//    （原 0.05/1.0 组合 ζ=0.85 但 ωn 仅 1.7，响应软、等效阻尼不足）
PositionPID rollAnglePID(0.8f, 0.0f, 0.0f);   // 差速外环：过阻尼设计 ζ≈1.1
PositionPID rollRatePID(0.07f, 0.0003f, 0.0f); // 差速内环：受 τm=0.28s 电机滞后限带
PositionPID pitchAnglePID(2.5f, 0.0f, 0.0f);  // 尾摆外环：提高带宽
PositionPID pitchRatePID(0.15f, 0.0003f, 0.0f); // 尾摆内环：+阻尼 (0.05→0.15)
PositionPID yawAnglePID(2.0f, 0.0f, 0.0f);   // 前摆外环：略保守于俯仰
PositionPID yawRatePID(0.13f, 0.0003f, 0.0f); // 前摆内环：+阻尼 (0.03→0.13), ζ≈0.97

// --- 4.2 垂直控制 (高度/速度串级PID) ---
// 外环: 高度误差 -> 目标垂直速度 (纯比例, Kp=1.0)
PositionPID altitudePositionPController(1.0f, 0.0f, 0.0f);
// 内环: 垂直速度误差 -> 目标垂直加速度 (Kp=5.0, Ki=0.00625, Kd=0)
PositionPID altitudeVelocityPIDController(5.0f, 1.25f * 0.005f, 0.0f / 0.005f);

// --- 4.3 水平位置控制 (位置/速度串级PID) ---
// 北向位置环 (纯比例, Kp=0.25)
PositionPID northPosPID(0.25f, 0.0f * 0.005f, 0.0f);
// 东向位置环 (纯比例, Kp=0.25)
PositionPID eastPosPID(0.25f, 0.0f * 0.005f, 0.0f);
// 北向速度环 (Kp=1.75, Ki=0.00125, Kd=0.05连续域→离散=0.05/0.005=10)
PositionPID northVelPID(1.75f, 0.25f * 0.005f, 0.05f / 0.005f);
// 东向速度环 (Kp=1.75, Ki=0.00125, Kd=0.05连续域→离散=0.05/0.005=10)
PositionPID eastVelPID(1.75f, 0.25f * 0.005f, 0.05f / 0.005f);

/*
 * ==========================================================================================
 * [5] 状态估计与信号滤波
 * ==========================================================================================
 */

// --- 5.1 卡尔曼滤波器 ---
VerticalKF vertical_estimator;
VerticalKF_2State vertical_estimator_2state;

const float KF_V_Q_ACCEL = 0.4f;
const float KF_V_Q_BIAS = 0.02f;
const float KF_V_Q_POS = 0.0025f;
const float KF_V_Q_VEL = 0.005f;
const float LASER_NOISE_STD = 0.05f;
const float BARO_NOISE_STD = 0.35f;

HorizontalKF_1Axis_VelMeas kf_north;
HorizontalKF_1Axis_VelMeas kf_east;

float kf_h_q_accel = 0.35f;
float kf_h_q_pos = 0.01f;
float kf_h_r_vel_base = 0.15f;
float kf_h_outlier_gate = 4.0f;

// --- 5.2 组合导航系统 ---
HardwareSerial &gpsSerialPort = Serial4;
bfs::Ubx ubx(&gpsSerialPort);
bfs::Ekf15State nav_ekf;

// --- 5.3 信号滤波器 ---
// 传感器原始数据滤波 (alpha值越小滤波越强，响应越慢)
ComplementaryFilter accelXFilter(0.02f), accelYFilter(0.02f), accelZFilter(0.05f); // 加速度计低通滤波 (alpha=0.02/0.05)
ComplementaryFilter gyroXFilter(0.15f), gyroYFilter(0.15f), gyroZFilter(0.15f);    // 陀螺仪低通滤波 (alpha=0.15)
ComplementaryFilter baro_altitude_filter(0.4f);                                    // 气压高度低通滤波 (alpha=0.4, 较快响应)
ComplementaryFilter temperature_filter(0.1f);                                      // 温度低通滤波 (alpha=0.1)
ComplementaryFilter pressure_filter(0.2f);                                         // 气压低通滤波 (alpha=0.2)

// 控制相关滤波
ComplementaryFilter altitude_rate_target_filter(0.3f);                                   // 垂直速度目标值滤波
ComplementaryFilter rollSpeedFilter(0.3f), pitchSpeedFilter(0.3f), yawSpeedFilter(0.3f); // 角速率滤波
ComplementaryFilter rollAngleOutputFilter(0.85f), pitchAngleOutputFilter(0.85f), yawAngleOutputFilter(0.85f); // 姿态外环输出滤波
ComplementaryFilter rollOutputFilter(0.25f), pitchOutputFilter(0.25f);                   // TVC摆角输出滤波
ComplementaryFilter yawOutputFilter(0.25f);                                                // 前摆角（偏航）输出滤波（新增）
ComplementaryFilter thrustCompN_filter(0.6f), thrustCompE_filter(0.6f);                  // 水平推力补偿滤波

// 反馈传感器滤波
ComplementaryFilter angleSensor1Filter(0.8f); // TVC角度传感器1滤波 (alpha=0.8, 弱滤波保持响应)
ComplementaryFilter angleSensor2Filter(0.8f); // TVC角度传感器2滤波

// 开关量去抖动 (中值滤波窗口大小为奇数)
MedianFilter unlockMedian(7);                 // 解锁开关去抖动 (7点中值)
MedianFilter ignitionMedian(5);               // 点火开关去抖动 (5点中值)
MedianFilter fuelMedianFilter(9);             // 燃料液位去抖动 (9点中值)
ComplementaryFilter fuelLowpassFilter(0.02f); // 燃料液位低通滤波 (alpha=0.02, 强平滑)

// 光流数据滤波
MedianFilter flowDistanceMedianFilter(5);                  // 光流测距中值滤波 (5点, 去除脉冲噪声)
ComplementaryFilter flowDistanceFilter(0.8f);              // 光流测距低通滤波 (alpha=0.8)
ComplementaryFilter flowVelXFilter(0.3f);                  // 光流X轴速度低通滤波
ComplementaryFilter flowVelYFilter(0.3f);                  // 光流Y轴速度低通滤波
ComplementaryFilter estimatedVerticalVelocityFilter(0.4f); // 垂直速度估算低通滤波

IMU_AutoCalibrator imuCalibrator;

/*
 * ==========================================================================================
 * [6] 传感器与驱动对象
 * ==========================================================================================
 */
// --- 硬件传感器驱动对象 ---
IST8310 compass;                     // IST8310 磁力计驱动 (I2C, Wire1)
Dps310 Dps310Sensor;                 // DPS310 气压计驱动 (I2C, IIC2, 0x77)
ICM42688 IMU(SPI, SPI_CS, 10000000); // ICM42688 六轴IMU驱动 (SPI, CS=PA4, 10MHz)
MTF02P opticalFlowSensor;            // MTF02P 光流传感器驱动 (Serial7)
LQS48_Flow opticalFlowSensorLQS48;   // LQS48 光流传感器驱动 (Serial7)

// --- 算法对象 ---
Madgwick madgwick; // Madgwick AHRS 姿态解算算法

// --- 辅助设备驱动对象 ---
StatusLED statusLed(LED_green);                 // 状态LED驱动 (绿灯, PE5)
AnoComProtocol AnoCom(&Serial6);                // 匿名地面站协议驱动 (Serial6)
CrsfSerial crsf(receiverSerial, CRSF_BAUDRATE); // CRSF遥控协议驱动 (Serial1, 420kbaud)

/*
 * ==========================================================================================
 * [7] 系统状态与通信缓存
 * ==========================================================================================
 */

// --- 7.1 全局标志位 ---
volatile bool dps310_read_success = false;
volatile bool imu_read_success = false;
volatile bool isDatalogging = false;
volatile bool fuelOK = false;
bool trajectoryPlanningStarted = false;
bool isLinkUp = false;
bool failsafe_in_flight = false;  // 失控保护：飞行中失控保持解锁
bool newEngineDataReceived = false;
bool positionHoldEnabled = false;

LoiterState loiter_state = LOITER_HOVERING;
VerticalLoiterState vertical_loiter_state = ALTITUDE_HOVERING;

// --- 7.2 传感器数据缓存 ---
float icm_Qw, icm_Qx, icm_Qy, icm_Qz;
float icm_Roll, icm_Pitch, icm_Yaw;
float backup_ahrs_Qw = 1.0f, backup_ahrs_Qx = 0.0f, backup_ahrs_Qy = 0.0f, backup_ahrs_Qz = 0.0f;
float backup_ahrs_roll = 0.0f, backup_ahrs_pitch = 0.0f, backup_ahrs_yaw = 0.0f;
float ahrs_yaw_correction_rad = 0.0f;
bool use_ekf_attitude_output = false;
float icm_gyro_x, icm_gyro_y, icm_gyro_z;
float icm_accel_x, icm_accel_y, icm_accel_z;

volatile float acc_sum_x = 0, acc_sum_y = 0, acc_sum_z = 0;
volatile float gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;
volatile int imu_sample_count = 0;

const int NAV_IMU_DELTA_BUFFER_SIZE = 64;
volatile float imu_delta_theta_x[NAV_IMU_DELTA_BUFFER_SIZE] = {0};
volatile float imu_delta_theta_y[NAV_IMU_DELTA_BUFFER_SIZE] = {0};
volatile float imu_delta_theta_z[NAV_IMU_DELTA_BUFFER_SIZE] = {0};
volatile float imu_delta_v_x[NAV_IMU_DELTA_BUFFER_SIZE] = {0};
volatile float imu_delta_v_y[NAV_IMU_DELTA_BUFFER_SIZE] = {0};
volatile float imu_delta_v_z[NAV_IMU_DELTA_BUFFER_SIZE] = {0};
volatile float imu_delta_dt_s[NAV_IMU_DELTA_BUFFER_SIZE] = {0};
volatile float imu_delta_time_sum_s = 0.0f;
volatile int imu_delta_sample_count = 0;
volatile bool imu_delta_overflow = false;

float temperature_raw = 0.0f, pressure_raw = 0.0f;
float temperature = 0.0f, pressure = 0.0f;
float baro_altitude_raw = 0.0f, baro_altitude = 0.0f;
float baro_altitude_offset = 0.0f;

// 气压高度 EKF 融合遥测
float baro_ekf_innovation_m = 0.0f;
float ekf_accel_bias_z_mps2 = 0.0f;  // EKF Z轴加速度零偏遥测 (m/s^2)
float baro_ekf_test_ratio = 0.0f;
bool baro_ekf_fused = false;

volatile OpticalFlowData flow_data = {0};

// 通信协议数据
// 发动机控制器通信
uint8_t engineDataReceiveBuffer[ENGINE_FRAME_LEN]; // 发动机数据接收缓冲区 (15字节帧)
uint8_t engineDataReceiveIndex = 0;                // 当前接收字节索引
float receivedP1 = 0.0f;                           // 接收的氧压P1数据 (MPa)
float receivedP2 = 0.0f;                           // 接收的氧压P2数据 (MPa)
float receivedValveControl = 0.0f;                 // 接收的阀门控制量

uint16_t raw_rc_values[RC_INPUT_MAX_CHANNELS] = {0};
uint16_t raw_rc_count = 0;

volatile bool new_guidance_command_received = false;
volatile unsigned long last_guidance_command_millis = 0;
float guidance_accel_E_cmd = 0.0f;
float guidance_accel_N_cmd = 0.0f;
float guidance_accel_U_cmd = 0.0f;

// --- 7.3 导航与状态估计结果 ---
float estimated_height = 0.0f;
float estimated_velocity = 0.0f;
float fused_north_pos = 0.0f, fused_north_vel = 0.0f;
float fused_east_pos = 0.0f, fused_east_vel = 0.0f;

// 2状态垂直KF独立输出（与EKF并行运行，仅供遥测对比，不参与控制）
float vfk_height = 0.0f;
float vfk_velocity = 0.0f;

bool is_origin_position_set = false;
bool is_origin_lla_set = false;
float origin_north = 0.0f, origin_east = 0.0f, origin_down = 0.0f;
double origin_lat_deg = 0.0, origin_lon_deg = 0.0, origin_lat_rad = 0.0, origin_lon_rad = 0.0, origin_alt_m = 0.0;
float relative_north = 0.0f, relative_east = 0.0f, relative_down = 0.0f;
float ubx_relative_north = 0.0f;
float ubx_relative_east = 0.0f;
float ubx_relative_down = 0.0f;

bool nav_system_initialized = false;
unsigned long last_ekf_update_us = 0;
bool nav_initialized_with_fake_origin = false;
bool nav_has_real_gnss_anchor = false;

// --- 导航数据源选择 ---
NavDataSource nav_data_source = NavDataSource::INTERNAL; // 默认内置数据源
bool deta100_online = false;             // DETA100 模块在线标志
bool deta100_detect_done = false;        // 上电检测是否已完成
uint32_t deta100_last_frame_ms = 0;      // DETA100 最后收到有效帧的时间戳
const uint32_t DETA100_DETECT_TIMEOUT_MS = 2000;  // 上电检测窗口 2s
const uint32_t DETA100_ONLINE_TIMEOUT_MS = 500;   // 运行中判定离线超时 500ms
uint32_t last_gnss_data_ms = 0;
const uint32_t GNSS_NAV_DATA_TIMEOUT_MS = 300;
uint32_t last_gnss_nav_valid_ms = 0;
const uint32_t GNSS_NAV_DROPOUT_HOLD_MS = 1000;

const float GNSS_MAX_HORZ_ACC_M = 6.0f;
const float GNSS_MAX_VERT_ACC_M = 12.0f;
const float GNSS_MAX_SPD_ACC_MPS = 1.0f;
const float GNSS_MAX_PDOP = 3.0f;
const int8_t GNSS_MIN_SV = 9;
const float GNSS_DOWNGRADE_MAX_SPD_ACC_MPS = 0.5f;
const uint32_t GNSS_PUMP_MAX_BYTES = 512U;
GnssRuntimeStatus gnss_status;
bfs::AidSourceStatusTracker aid_tracker;
uint32_t static_start_time = 0;
bool is_static_confirmed = false;

// --- 7.4 控制目标与输出 ---
// 姿态目标 (度)
float rollTarget = 0.0f, pitchTarget = 0.0f;         // Roll/Pitch 目标角度 (deg)
float yawRateTarget = 0.0f;                          // Yaw 目标角速率 (deg/s)
float rollRateTarget = 0.0f, pitchRateTarget = 0.0f; // Roll/Pitch 目标角速率 (deg/s)
float error_roll_deg = 0.0f, error_pitch_deg = 0.0f, error_yaw_deg = 0.0f; // 姿态角误差 (deg)

// 位置目标 (NED系, 米)
float targetNorth = 0.0f, targetEast = 0.0f;       // 水平位置目标 (m, NED系)
float targetVelNorth = 0.0f, targetVelEast = 0.0f; // 水平速度目标 (m/s, NED系)
float target_altitude = 0.0f;                      // 目标高度 (m, 向上为正)
float target_vertical_velocity = 0.0f;             // 目标垂直速度 (m/s, 向上为正)
float altitudeRateTarget = 0.0f;                   // 遥控器映射的目标垂直速度 (m/s)
bool autoAltitudeMode = false;                     // 自动高度模式激活标志
float target_accel_z_up_global = 0.0f;             // 天向运动目标加速度 (m/s^2, 向上为正)

// TVC 目标角度 (度)
float tvcTargetAngle1 = 0.0f, tvcTargetAngle2 = 0.0f; // TVC 通道1/2 目标摆角 (deg)

// 推力补偿分量 (单位推力矢量的水平分量)
float thrust_comp_N = 0.0f, thrust_comp_E = 0.0f; // 北向/东向推力补偿 (sin(tilt))

// 控制输出
float roll_output = 0.0f;     // 滚转角加速度 alpha_roll (rad/s²)（mix层再×Ix→Mx→差速Δω→滚转，FRD 轴序）
float pitch_output = 0.0f;    // Pitch TVC 修正量 (deg)
float yaw_output = 0.0f;   // 偏航角加速度 alpha_yaw (rad/s²)（mix层再×Iz→Mz→前摆δ_f→偏航，FRD 轴序）
float throttlePercent = 0.0f; // 油门百分比 (0-100%)

// ---- 在线参数辨识结果（★ 纯观测，不参与控制回路）----
// 初值 b=1.0 表示"与名义惯量一致"，激励不足时保持该值。
float id_b_est[3]      = {1.0f, 1.0f, 1.0f};
float id_d_est[3]      = {0.0f, 0.0f, 0.0f};
float id_cg_mm         = 0.0f;
bool  id_excited[3]    = {false, false, false};
float id_kp_suggest[3] = {0.0f, 0.0f, 0.0f};

// 执行机构输出百分比 (用于遥测显示)
float ch1_output = 0.0f, ch2_output = 0.0f; // PA0:前摆舵机(偏航/δ_f), PA1:尾摆舵机(俯仰/δ_t)
float ch3_output = 0.0f, ch4_output = 0.0f; // 主电机1/2 输出百分比

// 当前飞行模式 (由 runGNCExecutive 写入, 供 handleAnoCom 遥测发送)
ControlMode g_current_flight_mode = MANUAL;
// 当前解锁状态 (由 runGNCExecutive 写入, 供 handleAnoCom 遥测发送)
bool g_is_unlocked = false;

/*
 * ==========================================================================================
 * [8] 任务调度系统
 * ==========================================================================================
 */
Task tasks[MAX_TASKS];
uint8_t taskCount = 0;

/*
 * ==========================================================================================
 * DETA100 模块全局变量
 * ==========================================================================================
 */
// DETA100 输出数据的中值滤波器 (用于去除 AHRS 欧拉角的脉冲噪声)
MedianFilter HeadingFilter(5); // 航向角中值滤波 (5点窗口)
MedianFilter PitchFilter(3);   // 俯仰角中值滤波 (3点窗口)
MedianFilter RollFilter(3);    // 滚转角中值滤波 (3点窗口)

// DETA100 解析后的结构化数据包实例
IMUPacket IMU_Packet;                  // IMU 原始数据包 (陀螺仪/加速度计/磁力计)
AHRSPacket AHRS_Packet;                // AHRS 姿态数据包 (欧拉角/四元数)
INS_GNSSPacket INS_GNSS_Packet;        // INS/GNSS 组合导航数据包 (位置/速度/加速度)
GeodeticPosPacket Geodetic_Pos_Packet; // 大地坐标数据包 (经纬高/精度)
StatusPacket Status_Packet;            // 系统状态数据包 (故障/滤波器状态)

// DETA100 数据就绪标志 (由 Read_DETA100Data 设置, 由 DataUnpacking 清除)
bool Data_of_IMU = false;          // IMU 数据已接收标志
bool Data_of_AHRS = false;         // AHRS 数据已接收标志
bool Data_of_INS_GNSS = false;     // INS/GNSS 数据已接收标志
bool Data_of_Geodetic_Pos = false; // 大地坐标数据已接收标志
bool Data_of_Status = false;       // 系统状态数据已接收标志

// DETA100 原始字节缓冲区 (由 Read_DETA100Data 填充, 由 DataUnpacking 解析)
uint8_t IMU_Data[IMU_TYPE_LEN];                   // IMU 原始帧缓冲区 (64字节)
uint8_t AHRS_Data[AHRS_TYPE_LEN];                 // AHRS 原始帧缓冲区 (56字节)
uint8_t INS_GNSS_Data[INS_GNSS_TYPE_LEN];         // INS/GNSS 原始帧缓冲区 (80字节)
uint8_t Geodetic_Pos_Data[GEODETIC_POS_TYPE_LEN]; // 大地坐标原始帧缓冲区 (40字节)
uint8_t Status_Data[STATUS_TYPE_LEN];             // 系统状态原始帧缓冲区 (12字节)
uint8_t Fd_data[256];                             // 通用帧接收缓冲区
