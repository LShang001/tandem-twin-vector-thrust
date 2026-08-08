/*
 * ==========================================================================================
 * Coaxial_TVC_Drone_FCS - 共轴双桨推力矢量飞行控制系统 (模块化重构版)
 * ==========================================================================================
 *
 * 版本：Rev 5.0 - Modular Architecture
 * 架构师/作者：LShang
 *
 * 模块化架构：
 *   state_data.h/cpp    - 全局变量声明/定义、枚举/结构体
 *   math_utils.h        - 纯数学工具函数 (inline header-only)
 *   task_scheduler.h/cpp - 任务调度器
 *   sensor_imu.h/cpp     - IMU 传感器采集与预处理
 *   sensor_peripheral.h/cpp - 外设传感器 (气压计、光流、角度传感器)
 *   navigation_task.h/cpp - 导航与状态估计 (EKF、垂直/水平KF)
 *   flight_control.h/cpp - 飞行控制律 (PID、TVC、混控)
 *   communication.h/cpp  - 通信与遥测 (CRSF、MAVLink、ANO)
 *   main.cpp             - 系统初始化与主循环 (本文件)
 */

// ===== 模块化头文件 =====
#include "state_data.h"        // 全局变量声明、枚举/结构体定义
#include "math_utils.h"        // 纯数学工具函数 (inline header-only)
#include "task_scheduler.h"    // 任务调度器
#include "sensor_imu.h"        // IMU 传感器
#include "sensor_peripheral.h" // 外设传感器
#include "navigation_task.h"   // 导航与状态估计
#include "flight_control.h"    // 飞行控制
#include "communication.h"     // 通信与遥测
#include "can_bus.h"           // CAN 总线通信

// ===== 仅 main.cpp 使用的实现型头文件 =====
// DETA100_module.h 含解析实现和内部静态状态，必须只在 main.cpp 中引入。
#include "DETA100_module.h"            // DETA100 IMU/GNSS 模块驱动
#include "QuaternionMath.h"            // 四元数数学运算
#include "TVC_Control_3rdOrder_Poly.h" // TVC 三阶多项式控制
#include "TVC_Control_Geometric.h"     // TVC 几何控制
#include "GeoDisplacement.h"           // 经纬度位移计算
#include "MAVLink.h"                   // MAVLink 地面站协议

// ===== 硬件看门狗 (IWDG) =====
// 主循环卡死（SPI 挂起/死循环/长时间阻塞）时由 IWDG 硬件复位恢复，避免飞行器失控持续。
// 超时 3.0s（LSI 32kHz ÷ 预分频 256 = 125Hz；reload 374 = 2.99s）。
// 调试时可定义 BFS_DISABLE_IWDG 关闭（IWDG 一旦启用只能断电复位才能关闭）。
#ifndef BFS_DISABLE_IWDG
#include "stm32h7xx_hal_iwdg.h"
static IWDG_HandleTypeDef s_hiwdg;
static void initIwdg(void)
{
  s_hiwdg.Instance = IWDG1;             // STM32H743 单实例编号 IWDG1
  s_hiwdg.Init.Prescaler = IWDG_PRESCALER_256; // 32kHz / 256 = 125Hz
  s_hiwdg.Init.Reload = 374;                   // 125Hz × 2.99s
  s_hiwdg.Init.Window = IWDG_WINDOW_DISABLE;
  if (HAL_IWDG_Init(&s_hiwdg) != HAL_OK)
  {
    Serial8.println("[IWDG] 初始化失败（无看门狗保护运行）");
  }
  else
  {
    Serial8.println("[IWDG] 硬件看门狗已启用 (3.0s)");
  }
}
static inline void kickIwdg(void) { HAL_IWDG_Refresh(&s_hiwdg); }
#endif // BFS_DISABLE_IWDG

#ifndef BFS_DPS310_TASK_INTERVAL_MS
#define BFS_DPS310_TASK_INTERVAL_MS 7.735f
#endif

#ifdef BFS_TASK_PROFILE
#define BFS_ADD_TASK(function, interval, name) addTaskNamed(function, interval, name)
#else
#define BFS_ADD_TASK(function, interval, name) addTask(function, interval)
#endif

// Vector3 类型的全局变量定义（Vector3 定义在 QuaternionMath.h 中，不能被多个 .cpp 包含）
Vector3 current_omega_dps_body_filtered = Vector3(0.0f, 0.0f, 0.0f);

// ========================================================================
// DETA100 数据接收任务（依赖 DETA100_module.h 中的函数，仅 main.cpp 包含）
// 仅当 nav_data_source == NavDataSource::DETA100 时才被调度执行。
// ========================================================================
/**
 * @brief DETA100 惯性导航模块数据接收与解析任务
 *
 * 从 Serial4 (921600 baud) 读取 DETA100 模块的串口数据帧。
 * 仅当 nav_data_source == NavDataSource::DETA100 时才应被调度执行。
 *
 * 处理流程：
 * 1. Read_DETA100Data: 从串口读取字节，解析协议帧，设置数据就绪标志
 * 2. 检查是否有新数据包到达 (IMU/AHRS/INS_GNSS/Geodetic_Pos/Status)
 * 3. DataUnpacking: 将原始字节缓冲区解析为结构化数据包
 * 4. 更新 DETA100 心跳时间戳，维护在线状态
 * 5. 黄灯闪烁指示 AHRS 数据更新
 */
void handleDeta100()
{
  Read_DETA100Data(Serial4); // 从 Serial4 读取并解析 DETA100 数据帧

  // 检查是否有任何类型的新数据包到达
  if (Data_of_IMU || Data_of_AHRS || Data_of_INS_GNSS || Data_of_Geodetic_Pos || Data_of_Status)
  {
    // 更新 DETA100 心跳时间戳
    deta100_last_frame_ms = millis();
    deta100_online = true;

    // AHRS 数据到达时翻转黄灯，作为数据接收的心跳指示
    if (Data_of_AHRS)
    {
      digitalToggle(LED_yellow);
    }
    DataUnpacking(); // 将原始字节解析为结构化数据包
  }

  // 运行中超时检测
  if (deta100_online && (millis() - deta100_last_frame_ms > DETA100_ONLINE_TIMEOUT_MS))
  {
    deta100_online = false;
    Serial8.println("[DETA100] Offline! Timeout detected.");
  }
}

/*
 * ==========================================================================================
 * 系统初始化函数 (setup)
 * ==========================================================================================
 *
 * Arduino 框架入口函数，在 main() 中调用一次。
 * 按照严格的顺序初始化所有硬件外设、传感器驱动、控制算法参数和任务调度器。
 * 初始化失败时通过死循环 + LED 闪烁指示故障类型。
 */
void setup()
{
  // ====================================================================
  // 1. 串口初始化 (按波特率从高到低排列)
  // ====================================================================
  Serial3.begin(1500000);          // USART3: 黑匣子数据记录 (15Mbaud, 最高速)
  Serial8.begin(921600);           // UART8:  USB Type-C 调试输出 (921600 baud)
  Serial4.begin(921600);           // UART4:  DETA100 IMU/GNSS 模块数据输入 (921600 baud)
  Serial6.begin(SERIAL6_BAUDRATE);  // USART6: AnoCom/MAVLink 地面站通信 (波特率见 state_data.h SERIAL6_BAUDRATE)
  Serial5.begin(921600);           // UART5:  上位机轨迹规划接口 (921600 baud)
  setup_transmitter_uart();        // USART2: 发动机控制器/数据转发 (921600 baud)
  opticalFlowSerial.begin(921600); // UART7:  光流传感器 (921600 baud)

  ubx.Config(&gpsSerialPort); // 将 Serial4 绑定到 UBX 协议解析器 (默认路径，DETA100 检测通过后切换)

  // ====================================================================
  // 1.5 DETA100 上电自动检测
  // ====================================================================
  // 在 Serial4 上检测 DETA100 协议帧。如果检测窗口内收到有效帧，
  // 则切换为 DETA100 数据源，否则保持默认的内置 UBX GNSS 路径。
  //
  // 注意: DETA100 与 UBX 共用 Serial4，两者协议帧头不同 (DETA100=0xFC, UBX=0xB5)。
  // Read_DETA100Data 内部是 while(available()) 读空循环，会把 UBX 字节当成非法
  // DETA100 帧头丢弃; ubx.Pump 同样会读空缓冲并把 DETA100 字节当非 0xB5 丢弃。
  // 若两者在同一次循环里都无差别调用，会互相窃取字节。因此这里按帧轮换:
  // 奇数轮只喂 DETA100 解析器、偶数轮只喂 UBX 解析器，保证两类协议的字节都能进入
  // 各自的状态机，避免一方饿死另一方 (检测窗口内 1ms delay 下轮换频率约 500Hz)。
  {
    Serial8.println("[DETA100] Detecting on Serial4...");
    uint32_t detect_start = millis();
    uint8_t detect_toggle = 0; // 帧轮换计数器: 奇数轮 DETA100, 偶数轮 UBX
    while (millis() - detect_start < DETA100_DETECT_TIMEOUT_MS)
    {
      detect_toggle++;
      if (detect_toggle & 0x01U)
      {
        // 奇数轮: 尝试解析 DETA100 协议帧
        Read_DETA100Data(Serial4);
        if (Data_of_IMU || Data_of_AHRS || Data_of_INS_GNSS || Data_of_Geodetic_Pos || Data_of_Status)
        {
          // 收到 DETA100 有效帧，确认为 DETA100 数据源
          nav_data_source = NavDataSource::DETA100;
          deta100_online = true;
          deta100_detect_done = true;
          deta100_last_frame_ms = millis();
          Serial8.println("[DETA100] Detected! Switching to DETA100 data source.");
          break;
        }
      }
      else
      {
        // 偶数轮: 泵入 UBX 协议，避免纯 UBX GNSS 接收机的数据被丢弃
        if (ubx.Pump(GNSS_PUMP_MAX_BYTES, micros()))
        {
          last_gnss_data_ms = millis();
        }
      }
      delay(1); // 让出 CPU，避免忙等
    }
    if (!deta100_detect_done)
    {
      // 检测窗口内未收到 DETA100 帧，确认为内置数据源
      nav_data_source = NavDataSource::INTERNAL;
      deta100_detect_done = true;
      Serial8.println("[DETA100] Not detected. Using internal IMU+EKF+UBX data source.");
    }
  }

  // ====================================================================
  // 2. CRSF 遥控协议初始化
  // ====================================================================
  crsf.begin();                            // 初始化 CRSF 协议驱动 (Serial1, 420kbaud)
  crsf.onPacketChannels = &packetChannels; // 注册通道数据回调 (接收到通道值时调用)
  crsf.onLinkUp = &linkUpCallback;         // 注册链路建立回调
  crsf.onLinkDown = &linkDownCallback;     // 注册链路断开回调 (触发失控保护)

  // ====================================================================
  // 3. 硬件外设初始化
  // ====================================================================
  setup_led();                                  // 配置 LED 和点火引脚 (黄灯/绿灯/点火MOS管)
  setupTimer();                                 // 初始化 TIM8 硬件定时器 (2kHz 任务调度中断)
  statusLed.begin();                            // 初始化状态 LED 驱器 (PWM 呼吸灯/闪烁)
  ws2812Led.beginDma(TIM4, TIM_CHANNEL_4);      // WS2812: 启用 TIM4 DMA 方案 (H7, 失败回落 bitbang)
  ws2812Led.begin();                            // 初始化 WS2812 RGB状态灯 (PD15)
  flash.begin();                                // 初始化 W25N01GV NAND Flash (SPI3, 读ID+清写保护)
  flashLog.begin();                             // 初始化 Flash 日志层 (恢复游标)
  pinMode(FUEL_PIN, INPUT_PULLUP);              // 燃料液位传感器引脚 (上拉输入)
  analogWriteFrequency(CUSTOM_PWM_FREQUENCY);   // 设置全局 PWM 频率 (333Hz)
  analogWriteResolution(CUSTOM_PWM_RESOLUTION); // 设置全局 PWM 分辨率 (16位)
  analogReadResolution(16);                     // 设置 ADC 分辨率 (16位, 0-65535)

  // ====================================================================
  // ====================================================================
  // 4. PID 参数配置 — 全部从物理量推导（见 state_data.cpp §3.1 注释）
  //
  // 级联闭环: θ/θ_ref = ωn²/(s²+2ζωn·s+ωn²)
  //   内环带宽=Kp_r×57.3(rad/s),  外环ωn²=内环带宽×Kp_a,  ζ=内环带宽/(2ωn)
  //
  // Pitch/Roll: Kp_r=0.30 → 内环带宽=17.2rad/s(2.7Hz)
  //             Kp_a=5.0  → ωn=9.3rad/s(1.5Hz) ζ=0.93
  // Yaw(差速):  Kp_r=0.15 → 内环带宽=8.6rad/s(1.4Hz) 保守因电机τ=0.28s
  //             Kp_a=4.0  → ωn=5.9rad/s(0.94Hz) ζ=0.83
  //
  // PID输出限幅从悬停可达到的最大角加速度导出(α_max ≈ 0.8~1.0 rad/s²)
  // 满油门时α_max ≈ 5~9 rad/s², 限制宽松—响应随油门自然增强
  // ====================================================================

  // --- Roll/Pitch 外环: 角度误差(deg)→目标角速率(deg/s) ---
  // ★ 2026-08-08 C路径重构：限幅/积分/滤波统一读自 kFlightCtrlParams（§4.0，
  //   构造已带同值，此处保留调用以显式表达"setup 后生效值"）
  rollAnglePID.setOutputLimits(kFlightCtrlParams.att_roll.out_min, kFlightCtrlParams.att_roll.out_max);
  pitchAnglePID.setOutputLimits(kFlightCtrlParams.att_pitch.out_min, kFlightCtrlParams.att_pitch.out_max);
  rollAnglePID.setIntegralLimit(kFlightCtrlParams.att_roll.int_limit);
  pitchAnglePID.setIntegralLimit(kFlightCtrlParams.att_pitch.int_limit);
  rollAnglePID.setIntegralThreshold(kFlightCtrlParams.att_roll.threshold);
  pitchAnglePID.setIntegralThreshold(kFlightCtrlParams.att_pitch.threshold);

  // --- Roll内环: α限幅设极大→PID输出永不截断→调参所见即所得 ---
  // 真正的物理限制(电机力矩/转速)在底层自动生效, 不需要软件复刻
  rollRatePID.setOutputLimits(kFlightCtrlParams.rate_roll.out_min, kFlightCtrlParams.rate_roll.out_max);
  rollRatePID.setIntegralLimit(kFlightCtrlParams.rate_roll.int_limit);
  rollRatePID.setIntegralThreshold(kFlightCtrlParams.rate_roll.threshold);
  rollRatePID.setFilterCoefficient(kFlightCtrlParams.rate_roll.filter_alpha);

  // --- Pitch内环: 同Roll ---
  pitchRatePID.setOutputLimits(kFlightCtrlParams.rate_pitch.out_min, kFlightCtrlParams.rate_pitch.out_max);
  pitchRatePID.setIntegralLimit(kFlightCtrlParams.rate_pitch.int_limit);
  pitchRatePID.setIntegralThreshold(kFlightCtrlParams.rate_pitch.threshold);
  pitchRatePID.setFilterCoefficient(kFlightCtrlParams.rate_pitch.filter_alpha);

  // --- Yaw外环（未启用 enabled=false，参数随结构体）---
  yawAnglePID.setOutputLimits(kFlightCtrlParams.att_yaw.out_min, kFlightCtrlParams.att_yaw.out_max);
  yawAnglePID.setIntegralLimit(kFlightCtrlParams.att_yaw.int_limit);
  yawAnglePID.setIntegralThreshold(kFlightCtrlParams.att_yaw.threshold);

  // --- Yaw内环: 同Roll,α不截断 ---
  yawRatePID.setOutputLimits(kFlightCtrlParams.rate_yaw.out_min, kFlightCtrlParams.rate_yaw.out_max);
  yawRatePID.setIntegralLimit(kFlightCtrlParams.rate_yaw.int_limit);
  yawRatePID.setIntegralThreshold(kFlightCtrlParams.rate_yaw.threshold);
  yawRatePID.setFilterCoefficient(kFlightCtrlParams.rate_yaw.filter_alpha);

  // --- 高度串级 PID ---
  // 外环 (高度 -> 目标垂直速度): 纯比例控制, 输出限幅 ±1.0 m/s
  altitudePositionPController.setOutputLimits(kFlightCtrlParams.alt_pos.out_min, kFlightCtrlParams.alt_pos.out_max);
  // 内环 (垂直速度 -> 目标垂直加速度): PID控制
  // 输出限幅: -18.75 ~ +12.5 m/s^2 (不对称: 向下减速能力 > 向上加速能力)
  altitudeVelocityPIDController.setOutputLimits(kFlightCtrlParams.alt_vel.out_min, kFlightCtrlParams.alt_vel.out_max);
  altitudeVelocityPIDController.setIntegralLimit(kFlightCtrlParams.alt_vel.int_limit);
  altitudeVelocityPIDController.setFilterCoefficient(kFlightCtrlParams.alt_vel.filter_alpha);

  // --- 传感器初始化 ---
  if (!initMagnetometer())
  {
    Serial8.println("WARNING: Magnetometer Init Failed. AUTO modes degraded.");
    // 磁力计失效时AUTO_POS/GUIDED模式航向不可观测，仅手动飞行可用
    // LED_yellow已在下面处理，此处仅记录错误
  }

  if (initICM42688() != 0)
  {
    Serial8.println("Onboard ICM42688 initialization FAILED!");
    while (1)
    {
      digitalToggle(LED_yellow);
      delay(200);
    }
  }
  Serial8.println("Onboard ICM42688 initialized successfully.");

  madgwick.begin(2000);

  if (initDPS310() != 0)
  {
    Serial8.println("Onboard DPS310 initialization FAILED! (continuing without baro)");
    // 降级运行: DPS310 失败不阻塞系统, 气压/高度数据缺失需飞控自行处理
  }
  else
  {
    Serial8.println("Onboard DPS310 initialized successfully.");
  }

  opticalFlowSensorLQS48.begin(opticalFlowSerial);
  Serial8.println("LQS48 Optical Flow Sensor Initialized.");

  // ====================================================================
  // 5.5 CAN 总线初始化 (MCP2515 on SPI2)
  // ====================================================================
  // 独立 SPI2 实例, 与 SPI1 (ICM42688) 完全隔离。
  // 初始化失败不阻塞系统, CAN 通信功能不可用但飞控正常运行。
  initCAN();

  // --- 滤波器初始化 ---
  flowDistanceFilter.initialize(0.0f);
  flowVelXFilter.initialize(0.0f);
  flowVelYFilter.initialize(0.0f);
  estimatedVerticalVelocityFilter.initialize(0.0f);

  // --- 卡尔曼滤波器初始化 ---
  float initial_altitude = 0.0f;
  vertical_estimator.begin(initial_altitude, KF_V_Q_ACCEL, KF_V_Q_BIAS);
  vertical_estimator_2state.begin(initial_altitude, KF_V_Q_POS, KF_V_Q_VEL);
  Serial8.println("Vertical Kalman Filter Initialized.");

  kf_north.begin(kf_h_q_accel, kf_h_q_pos);
  kf_east.begin(kf_h_q_accel, kf_h_q_pos);
  Serial8.println("Horizontal Kalman Filters Initialized.");

  // ====================================================================
  // 任务注册 (addTask 函数名, 执行间隔ms)
  // ====================================================================
  // 调度器无抢占机制，按注册顺序 FIFO 轮询。串口发送类任务（ElrsBattery/
  // PosVelTx）在 TX 缓冲满时会阻塞等待，若排在 GNC 之前会延迟控制环执行。
  // 因此控制执行层提前到这些串口发送任务之前，确保 GNC 优先执行。
  // 例外: handleAnoCom 紧邻 EKF 注册（排在 GNC 之前），目的是同一帧内消费
  // 最新 EKF 输出以减少遥测延迟；其下行发送已加 availableForWrite 非阻塞保护，
  // 上行接收的 TX 回传同样有保护，不会阻塞后续 GNC。
  // 间隔参数单位为毫秒，会被转换为 2kHz 定时器滴答数。
  // 注释掉的任务为可选功能，按需启用。

  // --- 传感器采集层 (高频) ---
  BFS_ADD_TASK(handleICM42688, 0.5f, "ICM42688");                         // IMU 数据采集 (2kHz, 05ms) - 最高频率，姿态解算基础
  BFS_ADD_TASK(handleDPS310, BFS_DPS310_TASK_INTERVAL_MS, "DPS310");      // 气压计读取 - 默认约129Hz, 贴近128Hz压力输出且保留FIFO余量
  BFS_ADD_TASK(handleLQS48Flow, 2.0f, "LQS48Flow");                       // LQS48 光流传感器 (500Hz, 2ms) - 水平速度/测距

  // --- DETA100 数据源 (仅当检测到 DETA100 时注册) ---
  // 必须在状态估计层之前注册，确保同周期内 handleNavigationSystem 读到本周期最新的 DETA100 数据。
  if (nav_data_source == NavDataSource::DETA100)
  {
    BFS_ADD_TASK(handleDeta100, 5.0f, "Deta100"); // DETA100 数据接收 (200Hz, 5ms)
  }

  // --- 状态估计层 (中频) ---
  // EKF 组合导航排在垂向/水平 KF 之前, 同帧内 IMU delta 积累完毕后即刻预测,
  // 减少 EKF→遥测输出的延迟。AnoCom 紧邻 EKF 注册, 同一帧内 EKF 输出立即可用。
  BFS_ADD_TASK(handleNavigationSystem, 5.0f, "Navigation"); // EKF 组合导航 (200Hz, 5ms) - 姿态/位置/速度主滤波器
  BFS_ADD_TASK(handleAnoCom, 5.0f, "AnoCom");               // AnoCom 地面站 (200Hz, 5ms) - 紧邻 EKF, 同帧消费最新输出
  BFS_ADD_TASK(handleDebugTask, 5.0f, "Debug");              // 调试模式任务 (200Hz) - Serial6 "DBG\n" 入口，与遥测解耦

  // VerticalKF 与 EKF 并行运行，输出到独立的 vfk_height/vfk_velocity（不覆盖 EKF 输出），
  // 仅供地面站遥测对比一致性，不参与控制律。
  BFS_ADD_TASK(handleVerticalEstimation, 5.0f, "VerticalKF"); // 垂直卡尔曼滤波器 (200Hz, 5ms) - 气压/激光高度融合
  // handleHorizontalEstimation 的输出已不再覆盖 relative/INS 速度 (由 EKF 接管),
  // kf_north/kf_east 纯加速度积分无闭环校正, 注册已注释以节约 200Hz CPU.
  // addTask(handleHorizontalEstimation, 5.0f); // 水平卡尔曼滤波器 - 保留为光流后备

  // --- 控制执行层 (中频) ---
  // GNC 排在 handleAnoCom 之后（AnoCom 是有意例外，见上方注释），但在
  // ElrsBattery/PosVelTx 等其余串口发送任务之前，避免阻塞延迟控制环。
  BFS_ADD_TASK(handleGuidanceCommands, 2.0f, "GuidanceRx"); // 制导指令接收 (500Hz, 2ms) - 上位机加速度指令
  BFS_ADD_TASK(runGNCExecutive, 5.0f, "GNC");               // GNC 飞行控制 (200Hz, 5ms) - 制导+控制+混控输出

  // --- 通信遥测层 (低频) ---
  // 串口发送任务排在 GNC 之后，即使偶发阻塞也不影响控制环实时性。
  BFS_ADD_TASK(handleCANBus, 5.0f, "CANBus");                       // CAN 总线发送 (200Hz, 5ms) - 飞控状态广播
  BFS_ADD_TASK(sendElrsBatteryData, 40.0f, "ElrsBattery");      // ELRS 电量回传 (25Hz, 40ms) - 遥控器显示氧压
  BFS_ADD_TASK(sendElrsAttitudeData, 40.0f, "ElrsAttitude");    // ELRS 姿态回传 (25Hz) - 遥控器姿态球
  BFS_ADD_TASK(sendElrsBaroAltitudeData, 40.0f, "ElrsBaro");    // ELRS 高度+垂直速度回传 (25Hz) - 遥控器高度显示
  BFS_ADD_TASK(sendElrsFlightModeData, 100.0f, "ElrsMode");     // ELRS 飞行模式回传 (10Hz) - 遥控器模式名
  BFS_ADD_TASK(sendElrsGpsData, 100.0f, "ElrsGps");              // ELRS GNSS 位置回传 (10Hz) - 遥控器地图
  BFS_ADD_TASK(sendElrsVarioData, 40.0f, "ElrsVario");           // ELRS 垂直速度回传 (25Hz) - 遥控器变率计
  BFS_ADD_TASK(sendElrsTempData, 100.0f, "ElrsTemp");            // ELRS 温度回传 (10Hz) - 遥控器温度
  BFS_ADD_TASK(updateBatteryMonitor, 100.0f, "BattMon");          // 电池电压采样 (10Hz) - ADC_BATT PC5
  BFS_ADD_TASK(sendPositionVelocityData, 20.0f, "PosVelTx");    // 上位机数据发送 (50Hz, 20ms) - 轨迹规划用
  // addTask(handleMavlink, 5.0f);          // MAVLink 遥测 (200Hz, 5ms) - Mission Planner/QGC (与 AnoCom 二选一)
  BFS_ADD_TASK(handleStatusLedTask, 10.0f, "StatusLed");        // LED 状态指示 (100Hz, 10ms) - 呼吸灯/闪烁
  BFS_ADD_TASK(handleFlashService, 10.0f, "FlashLog");         // Flash 黑匣子后台写 (100Hz, 每tick≤2帧=190B) - 低优先级
  // 注：每页 1 帧（95B 传输 ~40µs + 编程 0.7ms），100Hz × 2 帧 = 200 帧/s 与产帧持平。
  //    CAN 互斥（s_flashWriting 50ms 窗口）防 SPI 冲突；IWDG 无压力（v2 实测）。
  // --- 可选任务 (按需启用) ---
  // addTask(handleElrs, 4.0f);              // ELRS 原始遥控接收 (250Hz) - 测试原始遥控链路时启用
  // addTask(handleCrsf, 4.0f);              // CRSF 通道转发 (250Hz) - 测试通道转发时启用
  // addTask(handleTelemetry, 5.0f);         // Serial8 调试遥测 (200Hz) - 需要额外调试输出时启用
  // addTask(handleOpticalFlow, 1.0f);       // MTF02P 光流处理 (1000Hz) - 与 LQS48 对比测试时启用
  // addTask(handleAnoCom, 5.0f);          // AnoCom - 已移至 EKF 之后紧邻注册 (见状态估计层)
  // addTask(handleAngleSensors, 4.0f);      // TVC 角度传感器 (250Hz) - 需要角度闭环/标定时启用
  BFS_ADD_TASK(handleDataLogging, 5.0f, "DataLog");   // 黑匣子数据记录 (200Hz) - Serial3 CSV + Flash 双写（解锁触发）
  // addTask(handleFlowTelemetry, 20.0f);    // 光流调试打印 (50Hz) - 调光流补偿时启用

  Serial8.println("VTVL_ElectricDualRotor_FC Initialized.");
  Serial8.println("System Setup Complete. Starting main loop...");

#ifndef BFS_DISABLE_IWDG
  // 所有初始化完成后再启用看门狗（避免长初始化被 IWDG 误复位）
  initIwdg();
#endif
}

/*
 * ==========================================================================================
 * 主循环函数 (loop)
 * ==========================================================================================
 *
 * Arduino 框架主循环，在 setup() 完成后被 main() 无限调用。
 * 循环频率取决于最慢的任务执行时间，通常在 1kHz 以上。
 *
 * 主循环执行三个关键操作：
 * 1. CRSF 协议轮询: 处理遥控器串口数据
 * 2. UBX 协议轮询: 处理 GNSS 接收机数据
 * 3. 任务调度执行: 遍历任务列表执行到期任务
 *
 * 注意: 所有高频操作 (IMU 2kHz, 控制 200Hz) 都在 taskExecutor() 中
 * 通过定时器中断设置的标志位触发，不在主循环中直接轮询。
 */
void loop()
{
  // ====================================================================
  // 1. 看门狗喂狗（主循环每轮刷新；任一任务阻塞 >3s 触发硬件复位）
  // ====================================================================
#ifndef BFS_DISABLE_IWDG
  kickIwdg();
#endif

  // ====================================================================
  // 2. CRSF 遥控协议轮询
  // ====================================================================
  // 驱动 CrsfSerial 库的内部状态机，从 Serial1 读取字节并解析 CRSF 帧。
  // 解析成功后会调用 packetChannels() 回调更新 raw_rc_values。
  // 链路状态变化时会调用 linkUpCallback/linkDownCallback。
  crsf.loop();

  // ====================================================================
  // 2. UBX GNSS 协议轮询
  // ====================================================================
  // 高频泵入 UBX 协议栈，从 Serial4 搬运字节并把完整 NAV-PVT/NAV-EOE epoch 入队。
  // 导航任务再按 200Hz 节奏 PopEpoch() 消费，避免新观测覆盖尚未融合的旧观测。
  // DETA100 模式下 Serial4 由 handleDeta100 独占解析 DETA100 协议帧 (帧头 0xFC)。
  // 若此处仍调用 ubx.Pump，会把 DETA100 字节当作非 UBX 字节 (非 0xB5) 丢弃，
  // 导致 DETA100 任务收不到数据、deta100_online 超时变 false、系统静默回退到内置 EKF。
  // 因此仅 INTERNAL 模式才泵入 UBX。
  if (nav_data_source != NavDataSource::DETA100)
  {
    if (ubx.Pump(GNSS_PUMP_MAX_BYTES, micros()))
    {
      last_gnss_data_ms = millis();  // 记录最后 GNSS epoch 接收时间 (用于新鲜度判断)
    }
  }

  // ====================================================================
  // 3. 任务调度执行
  // ====================================================================
  // 遍历所有已注册任务，检查定时器中断设置的标志位。
  // 到期的任务会被依次执行，执行顺序与注册顺序一致。
  // 所有任务函数在主循环上下文中执行 (非中断)，可安全使用串口/I2C/SPI。
  taskExecutor();
}
