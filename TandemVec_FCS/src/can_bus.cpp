/**
 * @file can_bus.cpp
 * @brief CAN 总线通信实现
 *
 * 基于 MCP2515 (SPI2) + TJA1050 实现 CAN 通信：
 * - 200Hz 周期轮发飞控状态数据 (IMU/导航/控制/系统)
 * - 主循环轮询接收 CAN 帧 (不注册 MCP2515 中断回调, 避免 ISR 中操作 SPI2)
 *
 * SPI2 引脚: SCK=PB13, MISO=PB14, MOSI=PB15
 * MCP2515: CS=PB12, INT=PD10
 *
 * @note arduino-CAN 库已修改: MCP2515.cpp 新增 setSPI() 支持自定义 SPI 实例,
 *       所有 SPI.begin/transfer/transaction 改为 _spi->xxx, 默认指向全局 SPI。
 *       SPI2 通过 setSPI(CAN_SPI) 注入, 与 SPI1 (ICM42688 IMU) 完全隔离。
 *
 * 详见 docs/电路拓扑参考.md §12.1
 */

#include "can_bus.h"
#include "state_data.h"
#include <SPI.h>
#include <CAN.h>  // arduino-CAN: MCP2515Class CAN 全局实例

// --- 独立 SPI2 实例 (与 SPI1/ICM42688 完全隔离) ---
// STM32duino SPIClass 构造: SPIClass(mosi, miso, sclk, ssel)
// PB15=SPI2_MOSI, PB14=SPI2_MISO, PB13=SPI2_SCK
static SPIClass CAN_SPI(CAN_SPI_MOSI, CAN_SPI_MISO, CAN_SPI_SCK);

// --- CAN 总线状态 ---
static bool can_initialized = false;
static uint32_t can_send_error_count = 0;
static uint32_t can_send_success_count = 0;

// 轮发帧计数器
static uint8_t can_frame_index = 0;

// --- 帧轮发控制 ---
// CAN 标准帧数据区最大 8 字节 = 2 个 float32
// 每周期发 1 帧, 8 帧一个完整循环
//   帧0: IMU陀螺 X/Y (rad/s)
//   帧1: IMU陀螺 Z + IMU加速度 X (rad/s + m/s²)
//   帧2: IMU加速度 Y/Z (m/s²)
//   帧3: EKF位置 N/E (m)
//   帧4: EKF速度 VN/VE (m/s)
//   帧5: 控制输出 throttle%/yaw_output
//   帧6: 控制输出 roll_output/pitch_output
//   帧7: 系统状态 roll/pitch (deg)
// 系统状态帧 pitch/yaw/height 每 5 个周期追加 (40Hz)
#define CAN_FRAME_CYCLE       8  // 8 帧一个完整循环
#define CAN_STATUS_DIVIDER    5  // 系统状态追加帧分频
#define CAN_RX_MAX_PER_TICK   2  // 每个200Hz周期最多处理2帧, 防止外部高流量拖住调度

// --- 辅助: 发送 2 个 float (8 字节, 恰好填满 CAN 数据区) ---
static bool sendFloatPair(int id, float a, float b) {
  if (!CAN.beginPacket(id)) {
    can_send_error_count++;
    return false;
  }
  CAN.write((const uint8_t*)&a, sizeof(float));
  CAN.write((const uint8_t*)&b, sizeof(float));
  if (!CAN.endPacket()) {
    can_send_error_count++;
    return false;
  }
  can_send_success_count++;
  return true;
}

// --- 初始化 ---
int initCAN(void) {
  Serial8.println("[CAN] 正在初始化 MCP2515 (SPI2, PB12/PB13/PB14/PB15, INT=PD10)...");

  // 注入独立 SPI2 实例到 MCP2515 (与 SPI1/ICM42688 完全隔离)
  CAN.setSPI(CAN_SPI);

  // 配置 MCP2515 参数 (必须在 begin() 前调用)
  CAN.setPins(CAN1_CS_PIN, CAN1_INT_PIN);
  CAN.setClockFrequency(CAN_CLOCK_FREQ);
  CAN.setSPIFrequency(CAN_SPI_FREQ);

  if (!CAN.begin(CAN_BAUD_RATE)) {
    Serial8.println("[CAN] MCP2515 初始化失败 (检查 SPI2 接线和 8MHz 晶振)");
    return -1;
  }

  // 不注册 CAN.onReceive(): arduino-CAN 的中断处理会在 ISR 中执行 parsePacket()
  // 并操作 SPI2；STM32duino 的 SPI.usingInterrupt() 为空实现，无法保证事务互斥。
  // 因此接收统一在 handleCANBus() 主循环上下文中轮询处理。
  can_initialized = true;
  Serial8.println("[CAN] MCP2515 初始化成功 (500 Kbps, 标准帧 11-bit)");
  return 0;
}

// --- 主循环中轮询处理接收帧 ---
static void pollCanReceive(void) {
  // 在主循环上下文中操作 SPI2, 与 handleCANBus 发送路径串行, 无并发冲突。
  // 每周期设置处理预算, 防止外部高流量/故障节点持续填充 RX 缓冲导致调度阻塞。
  for (uint8_t processed = 0; processed < CAN_RX_MAX_PER_TICK; processed++) {
    int packetSize = CAN.parsePacket();
    if (packetSize <= 0) break;

    // TODO: 根据 CAN.packetId() 解析控制指令, 将数据拷贝到全局状态。
    // 当前仅消费帧, 防止 MCP2515 RX 缓冲溢出。
    while (CAN.available()) {
      (void)CAN.read();
    }
  }
}

// --- 200Hz 周期任务 ---
void handleCANBus(void) {
  if (!can_initialized) return;

  // 先处理接收 (在发送之前, 避免发送阻塞延迟接收)
  pollCanReceive();

  // 轮发: 每周期发 1 帧, 8 帧循环 (每帧 2 个 float = 8 字节, 不超过 CAN 限制)
  switch (can_frame_index % CAN_FRAME_CYCLE) {
    case 0: {
      // IMU 陀螺 X/Y (rad/s, FRD机体系)
      sendFloatPair(CAN_ID_IMU_GYRO, icm_gyro_x, icm_gyro_y);
      break;
    }
    case 1: {
      // IMU 陀螺 Z + 加速度 X (rad/s + m/s²)
      sendFloatPair(CAN_ID_IMU_ACCEL, icm_gyro_z, icm_accel_x);
      break;
    }
    case 2: {
      // IMU 加速度 Y/Z (m/s², FRD机体系)
      sendFloatPair(CAN_ID_IMU_ACCEL_YZ, icm_accel_y, icm_accel_z);
      break;
    }
    case 3: {
      // EKF 位置 N/E (m, NED系)
      sendFloatPair(CAN_ID_NAV_POS, relative_north, relative_east);
      break;
    }
    case 4: {
      // EKF 速度 VN/VE (m/s, NED系)
      sendFloatPair(CAN_ID_NAV_VEL, fused_north_vel, fused_east_vel);
      break;
    }
    case 5: {
      // 控制输出: 油门% + yaw输出
      sendFloatPair(CAN_ID_CTRL_OUTPUT, throttlePercent, yaw_output);
      break;
    }
    case 6: {
      // 控制输出: roll输出 + pitch输出
      sendFloatPair(CAN_ID_CTRL_OUTPUT_RP, roll_output, pitch_output);
      break;
    }
    case 7: {
      // 系统状态: roll/pitch (deg, AHRS姿态)
      sendFloatPair(CAN_ID_SYS_STATUS, backup_ahrs_roll, backup_ahrs_pitch);
      break;
    }
  }
  can_frame_index++;

  // 系统状态追加帧: yaw + height (每 5 个周期, 40Hz)
  if (can_frame_index % CAN_STATUS_DIVIDER == 0) {
    sendFloatPair(CAN_ID_SYS_HEIGHT, backup_ahrs_yaw, estimated_height);
  }
}

// --- 诊断查询 ---
uint32_t getCanSendErrorCount(void) {
  return can_send_error_count;
}

uint32_t getCanSendSuccessCount(void) {
  return can_send_success_count;
}

bool isCanInitialized(void) {
  return can_initialized;
}