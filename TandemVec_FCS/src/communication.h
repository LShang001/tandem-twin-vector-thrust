/**
 * @file communication.h
 * @brief 通信与遥测模块：CRSF遥控、MAVLink遥测、ANO地面站、上位机通信
 *
 * 本模块负责飞控系统的所有对外通信：
 *
 *   遥控链路:
 *   - CRSF/ELRS 协议: 420kbaud, 16通道遥控输入, 链路状态回调
 *   - 失控保护: 链路断开时通道归中/归零
 *
 *   遥测链路:
 *   - MAVLink: 兼容 Mission Planner/QGC, HEARTBEAT/ATTITUDE/GPS/IMU/BATTERY
 *   - AnoCom: 匿名地面站协议, 4组分时发送 (IMU+姿态/高度+控制/速度+PWM/位置+GPS)
 *
 *   上位机通信:
 *   - Serial5: 轨迹规划接口, 34字节帧 (8个float位置+速度数据)
 *   - Serial5: 制导指令接收, 14字节帧 (3个float三轴加速度指令)
 *
 *   发动机通信:
 *   - Serial2: 发动机控制器数据收发, 15字节帧 (3个float氧压+阀门数据)
 *
 *   数据记录:
 *   - Serial3: 黑匣子高速数据记录, CSV格式, 1.5Mbaud
 *
 *   注意: MAVLink 和 AnoCom 共用 Serial6，互斥使用。
 */
#pragma once
#include "state_data.h"

/**
 * @brief CRSF 通道数据回调
 *
 * 当 CRSF 接收到新的通道数据包时被调用。
 * 将 16 个通道的 PWM 值 (988-2012) 存入 raw_rc_values 数组。
 */
void packetChannels();

/**
 * @brief CRSF 遥控链路上线回调
 *
 * 遥控器与接收机建立连接时触发，设置 isLinkUp = true。
 */
void linkUpCallback();

/**
 * @brief CRSF 遥控链路断开回调 (失控保护)
 *
 * 遥控器与接收机断开连接时触发：
 * - 通道1/2/4 (Roll/Pitch/Yaw) 归中到 1500
 * - 其他通道归零到 988 (油门最低, 解锁关闭)
 */
void linkDownCallback();

/**
 * @brief CRSF 链路质量统计回调 (ELRS LQ/RSSI)
 *
 * ELRS 接收机 0x14 帧（LINK_STATISTICS）解析后触发，固件取 uplink LQ
 * 供 100Hz 灯效任务做弱信号渐进预警（早于 300ms 断链判定）。
 */
void elrsLinkStatsCallback(crsfLinkStatistics_t *ls);

/**
 * @brief 查询当前是否处于 DBG 调试模式
 *
 * 供任务调度器（BFS_TASK_PROFILE）门控画像输出——仅 DBG 模式下
 * 才向 Serial6 打印任务统计表，避免文本污染正常 AnoCom 遥测流。
 */
bool isDebugModeActive();

/**
 * @brief 黑匣子段通道名提供者（main.cpp 注册给 flashLog）
 *
 * 返回逗号分隔的通道名字符串——writeSegmentHeader 写段起始页时
 * 用它把 S 帧拼进段头同页（2026-08-09 修复：S 帧此前随解锁游标写入，
 * 远离段头导致导出读不到通道名）。
 */
const char *bbSegmentNames();

/**
 * @brief 解析发动机控制器数据帧
 *
 * 校验帧头 (0xA5)、长度、校验和，通过 memcpy 小端序解析三个 float：
 * receivedP1 (氧压1), receivedP2 (氧压2), receivedValveControl (阀门控制)
 */
void processReceivedEngineData();

/**
 * @brief 接收发动机控制器串口数据
 *
 * 从 Serial2 逐字节读取数据，实现帧同步状态机。
 * 帧头 (0xA5) 匹配后开始缓存，收满 15 字节后调用解析函数。
 */
void receiveEngineData();

/**
 * @brief 构建并发送 CRSF 通道数据帧
 *
 * 将 16 个 PWM 通道值打包为 CRSF RC Channels Packed 帧：
 * - PWM (988-2012) -> CRSF原始值 (172-1811), 11位编码
 * - 使用 64 位累加器进行位打包
 * - CRC8-DVB-S2 校验
 *
 * @param channels 16个通道的PWM值数组
 * @return true 发送成功, false 发送失败
 */
bool send_crsf_frame(uint16_t channels[RC_INPUT_MAX_CHANNELS]);

/**
 * @brief 初始化 CRSF 遥控串口
 *
 * 配置 Serial1 (ELRS接收机): 420000 波特率, 8N1
 */
void setup_crsf_uart();

/**
 * @brief 初始化发动机/数据转发串口
 *
 * 配置 Serial2 (发动机控制器): 921600 波特率
 */
void setup_transmitter_uart();

/**
 * @brief 初始化系统 LED 和点火引脚
 *
 * 配置黄灯、绿灯、点火控制引脚为输出模式，设置初始电平。
 */
void setup_led();

/**
 * @brief 匿名地面站通信任务 (200Hz, 双向)
 *
 * 上行接收 (第一层):
 * - 每次调用开头执行 receiveData() 消费 Serial6 上行字节 (单轮上限 256 字节)
 * - 校验通过的帧触发 onAnoRxFrame 回调
 * - 对参数类帧 (0xE0/0xE1) 回传 0x00 校验帧 (安全协议, 占位实现)
 * - 对 0xE0 读取设备信息命令返回 0xE3 设备信息帧
 * - 回传 TX 发送带 availableForWrite 非阻塞保护, 不执行任何控制逻辑
 *
 * 下行发送: 4 组分时发送机制，每 4 个调用周期完成一轮:
 * - 组0: IMU数据 + 姿态欧拉角 + 目标姿态
 * - 组1: 高度数据 + 飞控模式 + 姿态控制量
 * - 组2: 目标速度 + 飞行速度 + PWM输出
 * - 组3: 位置偏移 + 电压电流(氧压) + GPS信息
 *
 * 下行发送前检查 Serial6 TX 缓冲空间，不足时跳过本帧。
 * 数据在新周期开始时一次性采集，避免发送过程中数据不一致。
 */
void handleAnoCom();

/**
 * @brief ELRS 原始遥控数据接收任务 (250Hz)
 *
 * 从 Serial1 读取 CRSF 数据并调用 crsf_parse 解析。
 * 最多尝试 3 次解析，失败后清空串口缓冲区。
 */
void handleElrs();

/**
 * @brief CRSF 通道转发任务 (250Hz)
 *
 * 接收发动机数据，构建16通道CRSF帧并发送：
 * - 通道1-2: Roll/Pitch 直接转发
 * - 通道3: 油门百分比映射回PWM
 * - 通道5: 燃料安全联锁 (fuelOK为false时强制锁定)
 * - 通道6-9: 点火/模式/TVC/姿态模式
 * - 通道10-16: 直接转发
 */
void handleCrsf();

/**
 * @brief ELRS 电池数据回传任务 (25Hz)
 *
 * 将氧压和GNSS状态映射到 CRSF 电池传感器协议字段：
 * - 电压字段 <- receivedP1 * 10
 * - 电流字段 <- receivedP2 * 10
 * - 容量字段 <- GNSS定位状态 * 10
 * - 剩余电量 <- receivedP1 * 10
 */
void sendElrsBatteryData();

/**
 * @brief ELRS 姿态回传任务 (25Hz)
 *
 * 将飞控姿态欧拉角映射到 CRSF ATTITUDE 协议字段，遥控器姿态球显示。
 */
void sendElrsAttitudeData();

/**
 * @brief ELRS 气压高度+垂直速度回传任务 (25Hz)
 *
 * 将 DPS310 气压高度和 EKF 垂直速度映射到 CRSF BARO_ALTITUDE 字段。
 */
void sendElrsBaroAltitudeData();

/**
 * @brief ELRS 飞行模式回传任务 (10Hz)
 *
 * 将当前控制模式名映射到 CRSF FLIGHT_MODE 字段（16 字符）。
 */
void sendElrsFlightModeData();

/**
 * @brief ELRS GNSS 位置回传任务 (10Hz)
 *
 * 将 UBX GNSS 定位数据映射到 CRSF GPS 协议字段，遥控器地图/位置显示。
 */
void sendElrsGpsData();

/**
 * @brief ELRS 垂直速度回传任务 (25Hz)
 *
 * 将 EKF 垂直速度映射到 CRSF VARIO 协议字段，遥控器变率计显示。
 */
void sendElrsVarioData();

/**
 * @brief ELRS 温度回传任务 (10Hz)
 *
 * 将 DPS310 气压计温度映射到 CRSF TEMP 协议字段，遥控器温度显示。
 */
void sendElrsTempData();

/**
 * @brief 电池电压采样任务 (10Hz)
 *
 * 从 ADC_BATT (PC5) 读取电池分压电压并换算为真实电压。
 * 输出全局量 bat_voltage_mv，供 ELRS / AnoCom 遥测共用。
 */
void updateBatteryMonitor();

/**
 * @brief Serial8 调试遥测任务 (200Hz)
 *
 * 通过 USB Type-C 串口打印完整的飞行数据 CSV 行：
 * 姿态欧拉角, 四元数, 角速度, 加速度, 姿态误差, 时间戳
 */
void handleTelemetry();

/**
 * @brief 黑匣子数据记录任务 (200Hz)
 *
 * 通过 Serial3 (1.5Mbaud) 记录飞行数据到黑匣子：
 * - 段开始/结束标记 (#LOG_START / #LOG_END)
 * - CSV 格式: 时间, 姿态, IMU, 速度, 位置, TVC角度, 发动机数据
 * - 检测解锁上升沿/下降沿自动启停记录
 */
void handleDataLogging();

/**
 * @brief 上位机数据发送任务 (50Hz)
 *
 * 通过 Serial5 向轨迹规划上位机发送 34 字节帧：
 * - 帧头 0xAA + 8个float (标志, 时间, 位置XYZ, 速度XYZ) + 帧尾 0x55
 */
void sendPositionVelocityData();

/**
 * @brief 制导指令接收任务 (250Hz)
 *
 * 从 Serial5 接收上位机下发的 14 字节制导指令帧：
 * - 帧头 0xAA + 3个float (加速度U/E/N, m/s^2) + 帧尾 0x55
 * - 高性能状态机解析，支持快速重同步
 * - 安全检查: NaN/Inf检测 + 加速度范围限制 (XY: 10m/s^2, Z: 10m/s^2)
 */
void handleGuidanceCommands();

/**
 * @brief 状态 LED 控制任务 (100Hz)
 *
 * 根据 IMU 校准状态和解锁状态控制 LED 闪烁模式：
 * - 校准中: 快闪 (10Hz)
 * - 已解锁: 慢闪 (5Hz)
 * - 待机: 呼吸灯效果
 */
void handleStatusLedTask();

/**
 * @brief 调试模式独立任务（200Hz）
 *
 * Serial6 收到 "DBG\n" 进入调试模式（handleDebugConsole），"exit" 退出。
 * 2026-08-08 从 handleAnoCom 移出：调试通道与 AnoCom 遥测解耦，
 * 避免遥测洪水淹没 DBG 检测。
 */
void handleDebugTask();

/**
 * @brief Flash 黑匣子后台写任务（低优先级，100Hz）
 *
 * 调 flashLog.logService()，每 tick 最多写 W25N01GV_LOG_MAX_WRITES 帧到 NAND。
 */
void handleFlashService();

/**
 * @brief 调试模式控制台（Serial6 共用）
 *
 * 进入方式：Serial6 收到 "DBG\n"（地面站数传协议下该序列不会自然出现）
 * 退出方式：调试模式下发 "exit"
 * 命令集：help / ws <r> <g> <b> / wsoff / wsseq [ms] / wsstat / ver / exit
 */
void handleDebugConsole(HardwareSerial &serial, char *line, uint8_t *lineLen,
                        uint8_t maxLen, bool &dbgMode);

/**
 * @brief GPIO 诊断：打印 PD15 (WS2812) 实时寄存器状态
 * 调试命令 "gpio" 调用
 */
void debugGpioDump(HardwareSerial &serial);

/**
 * @brief TIM4+DMA 诊断：验证 DMA 链路是否真的工作
 * 调试命令 "tim4" 调用
 */
void debugTim4Dump(HardwareSerial &serial);

/**
 * @brief Flash 调试命令（id/erase/stat/dump/test）
 * 调试命令 "flash <sub>" 调用
 */
void debugFlashCommand(HardwareSerial &serial, char *args);

/**
 * @brief MAVLink 遥测发送任务 (200Hz)
 *
 * 通过 Serial6 发送 MAVLink 协议数据包，兼容 Mission Planner / QGC：
 * - 1Hz: HEARTBEAT, SYS_STATUS, BATTERY_STATUS
 * - 20Hz: GLOBAL_POSITION_INT, VFR_HUD, RC_CHANNELS_RAW
 * - 40Hz: ATTITUDE, ATTITUDE_QUATERNION, RAW_IMU
 */
void handleMavlink();
