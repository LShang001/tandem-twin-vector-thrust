#include "communication.h"
#include "math_utils.h"
#include "navigation_task.h"
#include "sensor_peripheral.h"
#include "MAVLink.h"

// ========================================================================
// 通信模块局部定义
// ========================================================================

// 调试用黑匣子强制记录超时（datalog 命令置位，handleFlashService 超时复位）
static uint32_t s_dbgLogUntil = 0;
// 诊断计数：logService 调用次数（flash stat 显示，验证后台任务是否在跑）
static uint32_t s_flashServiceCount = 0;
// 诊断计数：logPush 调用次数（验证黑匣子产帧是否进入 ring）
static uint32_t s_flashPushCount = 0;
// 调试模式标志（文件级，handleDebugTask 置位，handleAnoCom 读取跳过遥测）
static bool s_dbg_mode = false;
// Flash 写页互斥标志（logService 写页时置位，CAN 任务跳过避免 SPI 冲突）
// ★ 2026-08-08：Flash 2048B 写页关中断 655µs 会让 CAN 任务延迟 → MCP2515
//   超时触发 reset → SPI2 transfer 卡死（OVR）→ 主循环冻结。互斥后消除。
volatile bool s_flashWriting = false;
volatile uint32_t s_flashWritingUntil = 0;

// 串口5发送缓冲区，用于发送给上位机进行轨迹规划的数据
// 缓冲区大小：1(帧头) + 8个float * 4字节/float + 1(帧尾) = 34字节
uint8_t serial5Buffer[34];

// 高性能制导指令解析器状态枚举
enum class PacketState : uint8_t
{
  WAIT_HEADER, // 等待帧头 (0xAA)
  RX_PAYLOAD,  // 接收有效载荷 (12 Bytes)
  WAIT_TAIL    // 等待帧尾 (0x55)
};

// 单次最大处理字节数，防止串口缓冲区积压时阻塞主循环过久
static const uint8_t MAX_BYTES_PER_LOOP = 64;

// 加速度安全阈值 (单位: m/s^2)，防止解析出错误的大数导致炸机
static const float MAX_SAFE_ACCEL_XY = 10.0f;
static const float MAX_SAFE_ACCEL_Z = 10.0f;

// 串口发送非阻塞保护：TX 缓冲剩余空间不足时跳过本帧发送，避免阻塞控制环。
// 统计各通道跳过次数，供调试参考（可通过 Serial8 查看）。
static uint32_t ano_tx_skipped = 0;
static uint32_t elrs_tx_skipped = 0;
static uint32_t posvel_tx_skipped = 0;

void packetChannels()
{
  // 获取遥控通道值并存储到 raw_rc_values 数组
  for (int i = 0; i < RC_INPUT_MAX_CHANNELS; ++i)
  {
    raw_rc_values[i] = crsf.getChannel(i + 1); // 获取通道的值
  }
}

void linkUpCallback()
{
  Serial8.println("Link is up!");
  isLinkUp = true; // 设置链接状态为正常
  failsafe_in_flight = false;  // 链路恢复，清除失控保护标志
}

void linkDownCallback()
{
  Serial8.println("Link is down! Failsafe activated.");
  isLinkUp = false;

  /*
   * 失控保护策略：
   * - 地面状态（油门<1100或未解锁）：保持解锁开关关闭（988），电机锁定。
   * - 飞行状态（油门>=1100且已解锁）：保持解锁开关打开（1500），摇杆归中，
   *   模式切到 AUTO_ALTITUDE（中档 1500），让自动高度+姿态自稳接管。
   *   操作者恢复操控后可重新接管。
   */
  bool was_in_flight = (raw_rc_values[4] > 1500) && (raw_rc_values[2] > 1100);
  failsafe_in_flight = was_in_flight;  // 记录失控保护模式

  for (int i = 0; i < RC_INPUT_MAX_CHANNELS; ++i)
  {
    if (i == 0 || i == 1 || i == 3)
    {
      // 摇杆通道归中
      raw_rc_values[i] = 1500;
    }
    else if (i == 4)
    {
      // 通道5（解锁开关）：飞行中保持解锁，地面锁定
      raw_rc_values[i] = was_in_flight ? 1500 : 988;
    }
    else if (i == 6)
    {
      // 通道7（模式开关）：飞行中切到 AUTO_ALTITUDE（中档 1500）
      raw_rc_values[i] = was_in_flight ? 1500 : 988;
    }
    else
    {
      raw_rc_values[i] = 988;
    }
  }
}

void processReceivedEngineData()
{
  // 检查帧头 (0xA5) 和有效载荷长度字段
  if (engineDataReceiveBuffer[0] == 0xA5 &&
      engineDataReceiveBuffer[1] == ENGINE_PAYLOAD_LEN)
  {
    // 计算校验和：从长度字节开始，累加到最后一个有效载荷字节
    uint8_t calculated_checksum = 0;
    for (int i = 1; i < (1 + 1 + ENGINE_PAYLOAD_LEN); i++) // Index 1 (len) to 1+ENGINE_PAYLOAD_LEN (last payload byte)
    {
      calculated_checksum += engineDataReceiveBuffer[i];
    }

    // 获取帧中的校验和字节 (位于有效载荷之后)
    uint8_t received_checksum = engineDataReceiveBuffer[1 + 1 + ENGINE_PAYLOAD_LEN];

    // 校验和匹配
    if (calculated_checksum == received_checksum)
    {
      // 按小端格式直接内存拷贝三个float数据
      memcpy(&receivedP1, &engineDataReceiveBuffer[2], sizeof(receivedP1));                                         // 第一个float从索引2开始
      memcpy(&receivedP2, &engineDataReceiveBuffer[2 + sizeof(float)], sizeof(receivedP2));                         // 第二个float
      memcpy(&receivedValveControl, &engineDataReceiveBuffer[2 + 2 * sizeof(float)], sizeof(receivedValveControl)); // 第三个float

      newEngineDataReceived = true; // 设置新数据接收标志
    }
    // else { Serial8.println("Engine data checksum error!"); } // 可选的调试信息
  }
  // else { Serial8.println("Engine data frame header/len error!"); } // 可选的调试信息
}

void receiveEngineData()
{
  while (transmitterSerial.available()) // 检查串口是否有数据
  {
    uint8_t byte_received = transmitterSerial.read(); // 读取一个字节

    // 如果是帧的第一个字节，但不是帧头 (0xA5)，则丢弃并继续等待帧头
    if (engineDataReceiveIndex == 0 && byte_received != 0xA5)
    {
      continue;
    }

    // 将接收到的字节存入缓冲区
    engineDataReceiveBuffer[engineDataReceiveIndex++] = byte_received;

    // 如果接收到的字节数达到完整一帧的长度
    if (engineDataReceiveIndex == ENGINE_FRAME_LEN)
    {
      processReceivedEngineData(); // 处理接收到的完整数据帧
      engineDataReceiveIndex = 0;  // 重置缓冲区索引，准备接收下一帧
    }
    // 防止缓冲区溢出 (虽然理论上ENGINE_FRAME_LEN会先满足)
    if (engineDataReceiveIndex >= ENGINE_FRAME_LEN)
    {
      engineDataReceiveIndex = 0;
    }
  }
}

bool send_crsf_frame(uint16_t channels[RC_INPUT_MAX_CHANNELS])
{
  // 定义CRSF帧结构相关常量
  const int payload_size = CRSF_PAYLOAD_SIZE_RC_CHANNELS;     // 22字节
  const int header_len = 3;                                   // 同步字节 + 长度字节 + 类型字节
  const int crc_len = 1;                                      // CRC字节
  const int frame_size = header_len + payload_size + crc_len; // 总帧长 26字节
  uint8_t send_buffer[frame_size] = {0};                      // 初始化发送缓冲区

  int offset = 0; // 用于在缓冲区中定位

  // 1. 构建帧头
  send_buffer[offset++] = CRSF_SYNC_BYTE;                     // 同步字节 (0xC8)
  send_buffer[offset++] = payload_size + 1 + 1;               // 长度字段 (类型字节 + 负载字节 + CRC字节) = 1 + 22 + 1 = 24
  send_buffer[offset++] = CRSF_FRAME_TYPE_RC_CHANNELS_PACKED; // 类型字段 (0x16 for RC channels)

  // 2. 构建负载 (22字节, 16个通道，每个通道11位)
  uint8_t payload[payload_size] = {0}; // 初始化负载缓冲区
  int byte_idx_payload = 0;            // 负载缓冲区的当前字节索引
  uint64_t bit_accumulator = 0;        // 64位累加器，用于临时存储通道的位数据
  int bits_in_accumulator = 0;         // 累加器中当前的位数

  // 遍历每个通道，进行映射和打包
  for (int i = 0; i < RC_INPUT_MAX_CHANNELS; ++i)
  {
    // 获取PWM值 (通常988-2012) 并确保在此范围内
    uint16_t pwm_value = channels[i];
    pwm_value = constrain(pwm_value, 988, 2012);

    // 将PWM值映射到CRSF原始通道值 (172-1811)
    // 映射公式：CRSF_raw = ((PWM - 988) * (1811-172) / (2012-988)) + 172
    // ELRS/Betaflight 使用的简化（近似）公式：
    // CRSF_raw = round(((PWM - 988.0) * 1638.0 / 1024.0) + 172.0)
    // 确保使用浮点运算以保持精度，然后转换为uint16_t
    uint16_t crsf_raw_value = static_cast<uint16_t>(roundf(((static_cast<float>(pwm_value) - 988.0f) * 1638.0f / 1024.0f) + 172.0f));
    crsf_raw_value &= 0x07FF; // 确保值在11位范围内 (0-2047)

    // 将11位CRSF值添加到64位累加器的低位
    bit_accumulator |= (static_cast<uint64_t>(crsf_raw_value) << bits_in_accumulator);
    bits_in_accumulator += 11; // 已添加11位

    // 从累加器中提取完整的字节 (8位) 并存入负载缓冲区
    while (bits_in_accumulator >= 8)
    {
      if (byte_idx_payload < payload_size)
      {                                                       // 确保不超出负载缓冲区
        payload[byte_idx_payload++] = bit_accumulator & 0xFF; // 提取最低8位
      }
      bit_accumulator >>= 8;    // 右移8位，移除已处理的位
      bits_in_accumulator -= 8; // 减少8位
    }
  }

  // 处理累加器中剩余的不足一个字节的位 (理论上最后应该填满或剩少量位)
  if (bits_in_accumulator > 0 && byte_idx_payload < payload_size)
  {
    payload[byte_idx_payload++] = bit_accumulator & 0xFF;
  }

  // 确保负载长度为22字节，不足则用0填充 (理论上应该刚好填满)
  while (byte_idx_payload < payload_size)
  {
    payload[byte_idx_payload++] = 0;
  }

  // 将构建好的负载复制到发送缓冲区
  memcpy(&send_buffer[offset], payload, payload_size);
  offset += payload_size;

  // 3. 计算CRC校验和
  // CRC计算范围是从类型字段(Type)开始，到负载(Payload)的最后一个字节结束。
  // 即 send_buffer[2] 到 send_buffer[2 + (payload_size + 1) - 1]
  // 长度为 (payload_size + 1) 字节
  uint8_t crc_val = crc8_dvb_s2_buf(&send_buffer[2], payload_size + 1); // Type (1 byte) + Payload (22 bytes)
  send_buffer[offset++] = crc_val;

  // 4. 通过串口发送CRSF帧
  size_t bytesSent = transmitterSerial.write(send_buffer, offset); // offset 此时应等于 frame_size

  // 返回发送是否成功 (发送的字节数是否等于期望的帧大小)
  return bytesSent == static_cast<size_t>(offset);
}

void setup_crsf_uart()
{
  receiverSerial.begin(420000, SERIAL_8N1); // 420000波特率，8数据位，无校验，1停止位
  while (!receiverSerial)
  {
    ; // 等待串口初始化完成
  }
  Serial8.println("Receiver Serial1 (ELRS) Initialized at 420000 baud.");
}

void setup_transmitter_uart()
{
  transmitterSerial.begin(921600); // 根据通信对象调整波特率
  while (!transmitterSerial)
  {
    ; // 等待串口初始化完成
  }
  Serial8.println("Transmitter Serial2 Initialized at 921600 baud.");
}

void setup_led()
{
  pinMode(LED_yellow, OUTPUT);
  pinMode(LED_green, OUTPUT);
  pinMode(ignition, OUTPUT);      // 点火控制引脚
  digitalWrite(LED_yellow, HIGH); // 初始黄灯亮
  digitalWrite(LED_green, LOW);   // 初始绿灯灭
  digitalWrite(ignition, LOW);    // 初始点火关闭
}

// ========================================================================
// AnoCom 上行帧处理 (第一层: 接收 + 校验帧回传 + 设备信息返回)
// ========================================================================

// 设备信息常量 (供 0xE3 设备信息返回帧使用)
static const char ANO_DEVICE_NAME[] = "VTVL_DualRotor_FCS";
static const uint8_t ANO_DEVICE_ID = ANO_LOCAL_ADDR; // 0x05
static const int16_t ANO_HW_VERSION = 1;             // 硬件版本 V1.0
static const int16_t ANO_SW_VERSION = 5;             // 软件版本 Rev 5.0
static const int16_t ANO_BL_VERSION = 0;             // 无 Bootloader
static const int16_t ANO_PT_VERSION = 1;             // 通信协议版本 V1

// 统计: 收到的上行帧数 (按功能码分类, 供调试参考)
static uint32_t ano_rx_total = 0;
static uint32_t ano_rx_check_sent = 0;
static uint32_t ano_rx_device_info_sent = 0;

/**
 * @brief AnoCom 上行帧回调 (由 AnoComProtocol::parseData 在校验通过后调用)
 *
 * 第一层只处理两个安全协议相关的功能码:
 *   - 收到需要回传校验的帧 (0xE0 参数命令 / 0xE1 参数写入) → 回传 0x00 校验帧
 *   - 收到 0xE0 读取设备信息命令 → 返回 0xE3 设备信息帧
 *
 * 本回调不执行任何控制逻辑, 不修改控制参数, 不影响飞控行为。
 * 回调中的 TX 发送带 availableForWrite 非阻塞保护, 避免阻塞控制环。
 */
static void onAnoRxFrame(uint8_t funcCode, uint8_t *data, uint16_t len)
{
  ano_rx_total++;

  // 需要回传校验帧的功能码 (安全协议要求: 参数写入/命令控制类帧必须返回 0x00 校验帧)
  // 第一层暂不对所有命令帧回传, 只对参数类帧回传 (0xE0/0xE1)
  if (funcCode == ANO_FUNC_PARAM_CMD || funcCode == ANO_FUNC_PARAM_WRITE_READ)
  {
    // 校验值填 0: parseData 回调签名未传递原始校验值, 地面站会判定校验失败并重试。
    // 第一层为占位实现, 参数读写功能需后续层补全完整校验值传递后再可用。
    // TX 非阻塞保护: 缓冲不足时跳过回传, 避免阻塞控制环
    if (Serial6.availableForWrite() >= 12) // 0x00 校验帧 = 8 帧开销 + 3 DATA + 1 余量
    {
      AnoCom.sendDataCheck(funcCode, 0, 0);
      ano_rx_check_sent++;
    }

    // 0xE0 参数命令: 检查是否为"读取设备信息"命令
    // 协议定义 DATA[0] 为 CMD 码: 0x00=读取设备信息, 0x01=读取参数个数, 0x02=读取参数值, 0x03=读取参数信息
    if (funcCode == ANO_FUNC_PARAM_CMD && len >= 1 && data[0] == 0x00)
    {
      // 0xE3 设备信息帧 = 8 帧开销 + 29 DATA (1+8+20)
      if (Serial6.availableForWrite() >= 40)
      {
        AnoCom.sendDeviceInfo(ANO_DEVICE_ID, ANO_HW_VERSION, ANO_SW_VERSION,
                              ANO_BL_VERSION, ANO_PT_VERSION, ANO_DEVICE_NAME);
        ano_rx_device_info_sent++;
      }
    }
  }
}

/**
 * @brief 调试模式任务（200Hz）
 *
 * ★ 2026-08-08 重构：DBG 检测放回 handleAnoCom 最前（receiveData 之前），
 *   否则 AnoCom 的 receiveData 会把 DBG 字节当协议帧头吃掉。
 *   本任务只在"调试模式已进入"时处理命令；进入检测在 handleAnoCom。
 */
void handleDebugTask()
{
  static char dbg_line[64];
  static uint8_t dbg_line_len = 0;

  if (s_dbg_mode) {
    handleDebugConsole(Serial6, dbg_line, &dbg_line_len, sizeof(dbg_line), s_dbg_mode);
  }
}

void handleAnoCom()
{
  // ---- 调试模式入口检测（必须在 receiveData 之前！）----
  // 否则 AnoCom 的 receiveData 会把 "DBG\n" 字节当协议帧头（非 0xAB）丢弃。
  // 进入调试模式后 s_dbg_mode=true，下方跳过所有遥测逻辑。
  static char dbg_line[64];
  static uint8_t dbg_line_len = 0;
  if (!s_dbg_mode) {
    while (Serial6.available()) {
      char c = (char)Serial6.read();
      if (c == '\n') {
        if (dbg_line_len == 3 && dbg_line[0] == 'D' && dbg_line[1] == 'B' && dbg_line[2] == 'G') {
          s_dbg_mode = true;
          dbg_line_len = 0;
          Serial6.println(F("[DBG] Debug mode ON. Type 'help' for commands, 'exit' to quit."));
          return;
        }
        dbg_line_len = 0;  // 非匹配行，丢弃
      } else if (dbg_line_len < (int)(sizeof(dbg_line) - 1)) {
        dbg_line[dbg_line_len++] = c;
      }
    }
  }

  // 调试模式下跳过所有遥测逻辑（RX/TX 都由 handleDebugTask 接管）
  if (s_dbg_mode) {
    return;
  }

  // ---- 上行接收: 消费 Serial6 缓冲区中的上行帧 ----
  // receiveData() 内部 while(available) 读空, 帧校验通过后调用 onAnoRxFrame 回调。
  // 首次调用时注册回调 (静态初始化保证只注册一次)。
  static bool rx_callback_registered = false;
  if (!rx_callback_registered)
  {
    AnoCom.setRxCallback(onAnoRxFrame);
    rx_callback_registered = true;
  }
  AnoCom.receiveData();

  // ---- 下行发送 (原有逻辑) ----
  // 静态变量，用于跟踪当前发送的数据组索引和是否开始新一轮数据采集
  static uint8_t group_index = 0;               // 当前发送的数据包组索引 (0-3)
  static bool new_cycle_data_collection = true; // 是否需要采集新一轮数据的标志

  // 遥测控制量安全转换：isfinite 防 NaN/Inf（constrain 对 NaN 无效），
  // 限幅到协议 ±5000 量程，异常值归零仅损坏显示不影响飞行。
  auto anoCtrlSafe = [](float v) -> float {
    return (isfinite(v)) ? constrain(v, -5000.0f, 5000.0f) : 0.0f;
  };

  // 静态数据缓存，仅在new_cycle_data_collection为true时更新
  // 惯性传感器数据
  static float acc_x_ano, acc_y_ano, acc_z_ano;
  static float gyr_x_ano, gyr_y_ano, gyr_z_ano;
  // 姿态信息 (欧拉角)
  static float roll_ano, pitch_ano, yaw_ano;
  static uint8_t fusionSta = 0; // 初始化为 0 (未初始化)
  // 高度数据
  static float alt_bar, alt_add, alt_fu;
  // 姿态控制量
  static float roll_ctrl_ano, pitch_ctrl_ano, yaw_ctrl_ano;
  // 油门控制量
  static float throttle_ctrl_ano;
  // 目标速度数据
  static float target_speed_x_ano, target_speed_y_ano, target_speed_z_ano;
  // 目标姿态数据
  static float target_roll_ano, target_pitch_ano, target_yaw_ano;
  // 飞行速度数据 (NED)
  static float vel_north_ano, vel_east_ano, vel_down_ano;
  // PWM控制量
  static float pwm_ch1_ano, pwm_ch2_ano, pwm_ch3_ano, pwm_ch4_ano;
  static float pwm_ch5_ano, pwm_ch6_ano, pwm_ch7_ano, pwm_ch8_ano;
  // 位置偏移数据 (cm)
  static float pos_north_cm_ano, pos_east_cm_ano, pos_down_cm_ano;
  // GNSS传感器信息
  static int8_t gnss_fix_type_ano;
  static uint8_t gps_num_sat_ano; // 卫星数
  static float gps_longitude_deg_ano, gps_latitude_deg_ano, gps_height_msl_ano;
  static float gps_vel_north_ano, gps_vel_east_ano, gps_vel_down_ano;
  static float gps_speed_acc_ano, gps_v_acc_ano, gps_pdop_ano;
  // 发动机氧压传感器数据
  static float oxygen_pressureP1_ano, oxygen_pressureP2_ano;
  static float bat_voltage, bat_current, fc_voltage, fc_current;
  static uint16_t power_state = 0; // 初始化为 0 (无故障)
  // 飞控模式遥测 (0x06 帧字段)
  static uint8_t flight_mode_ano = 0;  // MODE: 0=MANUAL,1=AUTO_POSITION,2=AUTO_ALTITUDE,3=GUIDED
  static uint8_t flight_sflag_ano = 0; // SFLAG: 0=锁定, 1=解锁

  // 仅在新循环开始时（即发送完一组4个包后），才重新获取所有需要发送的数据
  if (new_cycle_data_collection)
  {
    const bool gnss_data_fresh_for_telemetry = isGnssDataFreshForNav();
    const bool gnss_instant_valid_for_telemetry =
        (ubx.fix() >= bfs::Ubx::FIX_3D) && gnss_data_fresh_for_telemetry;

    // === 从全局变量或传感器数据包中采集当前时刻的数据 ===
    // IMU (来自DETA100的IMU_Packet)
    acc_x_ano = IMU_Packet.accelerometer_x;          // m/s^2
    acc_y_ano = IMU_Packet.accelerometer_y;          // m/s^2
    acc_z_ano = IMU_Packet.accelerometer_z;          // m/s^2
    gyr_x_ano = IMU_Packet.gyroscope_x * RAD_TO_DEG; // deg/s
    gyr_y_ano = IMU_Packet.gyroscope_y * RAD_TO_DEG; // deg/s
    gyr_z_ano = IMU_Packet.gyroscope_z * RAD_TO_DEG; // deg/s

    // AHRS (来自DETA100的AHRS_Packet)
    roll_ano = AHRS_Packet.Roll * RAD_TO_DEG;
    pitch_ano = AHRS_Packet.Pitch * RAD_TO_DEG;
    // 将航向角从0-2π范围转换为-π~π范围，以便显示
    yaw_ano = (AHRS_Packet.Heading > M_PI) ? (AHRS_Packet.Heading - 2 * M_PI) * RAD_TO_DEG : AHRS_Packet.Heading * RAD_TO_DEG; // deg

    fusionSta = nav_system_initialized; // EKF初始化融合状态
    // 编码数据源信息到 fusionSta 高位: bit7=DETA100在线, bit6=数据源类型(0=内置,1=DETA100)
    if (deta100_online)
      fusionSta |= 0x80;
    if (nav_data_source == NavDataSource::DETA100)
      fusionSta |= 0x40;

    // 高度
    alt_bar = baro_altitude;        // 气压高度 (米)
    alt_add = vfk_height;           // 2状态垂直KF高度 (与EKF对比用，激光通道空闲)
    alt_fu = estimated_height;      // EKF融合高度 (控制律消费)

    // 控制量 (来自 GNC 输出 gnc_tel)，经 anoCtrlSafe 限幅 ±5000 + NaN 防护
    // ★ 2026-08-08 C路径重构：读取来源收拢为 gnc_tel（缩放系数不变，输出数值不变）
    roll_ctrl_ano = anoCtrlSafe(gnc_tel.alpha_ref[0] * 10.0f);  // 滚转控制输出，放大10倍用于显示
    pitch_ctrl_ano = anoCtrlSafe(gnc_tel.alpha_ref[1] * 10.0f); // 俯仰控制输出，放大10倍
    yaw_ctrl_ano = anoCtrlSafe(gnc_tel.alpha_ref[2] * 10.0f);   // 偏航控制输出（alpha_yaw rad/s²），放大10倍
    throttle_ctrl_ano = anoCtrlSafe(throttlePercent * 10.0f);    // 油门百分比，放大10倍

    // 目标值 (来自控制逻辑)
    target_speed_x_ano = targetVelNorth * 100;           // 水平目标速度X (如果适用)
    target_speed_y_ano = targetVelEast * 100;            // 水平目标速度Y (如果适用)
    target_speed_z_ano = target_vertical_velocity * 100; // 目标垂直速度 (m/s)
    target_roll_ano = rollTarget * 100;                  // 目标滚转角 (deg) - 主要用于手动模式记录
    target_pitch_ano = pitchTarget * 100;                // 目标俯仰角 (deg) - 主要用于手动模式记录
    target_yaw_ano = gnc_tel.omega_ref_dps[2] * 100;     // 目标偏航角速率 (deg/s)

    // 速度 (来自 EKF 的 INS_GNSS_Packet.velocity_*, NED系)
    // EKF 输出桥已写入 nav_ekf.ned_vel_mps()，无 GNSS 时由 ZUPT/Gravity 静止辅助闭环约束。
    // 不再使用 fused_north_vel/fused_east_vel（kf_north/kf_east 纯加速度积分，无零速更新会漂移）。
    vel_north_ano = INS_GNSS_Packet.velocity_north; // m/s
    vel_east_ano = INS_GNSS_Packet.velocity_east;   // m/s
    vel_down_ano = -INS_GNSS_Packet.velocity_down;  // m/s (取反为向上为正)

    if (gnss_instant_valid_for_telemetry)
    {
      gps_vel_north_ano = ubx.north_vel_mps(); // m/s
      gps_vel_east_ano = ubx.east_vel_mps();   // m/s
      gps_vel_down_ano = -ubx.down_vel_mps();  // m/s (取反为向上为正)
    }
    else
    {
      gps_vel_north_ano = 0;
      gps_vel_east_ano = 0;
      gps_vel_down_ano = 0;
    }

    // PWM输出 (来自遥控器原始值，或CRSF发送前的最终值)
    pwm_ch1_ano = raw_rc_values[0];
    pwm_ch2_ano = raw_rc_values[1];
    pwm_ch3_ano = raw_rc_values[2]; // 油门通道
    pwm_ch4_ano = raw_rc_values[3];
    pwm_ch5_ano = raw_rc_values[4]; // 解锁通道
    pwm_ch6_ano = raw_rc_values[5]; // 点火通道
    pwm_ch7_ano = raw_rc_values[6]; // 模式通道
    pwm_ch8_ano = raw_rc_values[7]; // TVC手动通道

    // 位置 (来自全局计算的相对位置)，转换为厘米
    pos_north_cm_ano = relative_north * 100.0f;
    pos_east_cm_ano = relative_east * 100.0f;
    pos_down_cm_ano = -relative_down * 100.0f; // 取反为高度

    // 使用 UBX 原始计算值
    // pos_north_cm_ano = ubx_relative_north * 100.0f;
    // pos_east_cm_ano = ubx_relative_east * 100.0f;
    // AnoCom 高度通常向上为正，NED Down 向下为正，所以取反
    // pos_down_cm_ano = -ubx_relative_down * 100.0f;

    // GNSS信息 (来自DETA100的Status_Packet和Geodetic_Pos_Packet)
    gnss_fix_type_ano = Status_Packet.filter_status.gnss_fix_status;
    // gnss_fix_type_ano = ubx.fix();
    if (gnss_data_fresh_for_telemetry)
    {
      gps_num_sat_ano = ubx.num_sv();                  // 使用UBX解析出来的卫星数
      gps_longitude_deg_ano = ubx.lon_deg();           // 度
      gps_latitude_deg_ano = ubx.lat_deg();            // 度
      gps_height_msl_ano = Geodetic_Pos_Packet.height; // 大地高 (米)
      gps_pdop_ano = ubx.pdop();                       // 位置精度因子
      gps_speed_acc_ano = ubx.spd_acc_mps();           // 速度精度 (米/秒)
      gps_v_acc_ano = Geodetic_Pos_Packet.vAcc;        // 垂直精度 (米)
    }
    else
    {
      // GNSS串口断流后，UBX对象仍保留最后一帧缓存；地面站遥测必须清零，避免误判仍有卫导。
      gps_num_sat_ano = 0;
      gps_longitude_deg_ano = 0.0f;
      gps_latitude_deg_ano = 0.0f;
      gps_height_msl_ano = 0.0f;
      gps_pdop_ano = 0.0f;
      gps_speed_acc_ano = 0.0f;
      gps_v_acc_ano = 0.0f;
    }

    // 发动机氧压数据 (来自发动机控制器回传)
    oxygen_pressureP1_ano = receivedP1;
    oxygen_pressureP2_ano = receivedP2;

    bat_voltage = 12.6;
    bat_current = 16.8;
    fc_voltage = oxygen_pressureP1_ano;
    fc_current = oxygen_pressureP2_ano;

    // 飞控模式与解锁状态 (供 0x06 飞控运行模式帧)
    // 使用 GNC 的判定结果 (滤波 + 链路状态), 而非原始 RC 值, 确保与飞控实际状态一致
    flight_mode_ano = static_cast<uint8_t>(g_current_flight_mode);
    flight_sflag_ano = g_is_unlocked ? 1 : 0;

    new_cycle_data_collection = false; // 数据采集完成，清除标志
  }

  // 根据当前组索引，发送对应的数据包
  // 非阻塞保护：发送前检查 Serial6 TX 缓冲剩余空间，不足时跳过本帧发送。
  // 每组最多 3 个包，最大单包 31 字节（GPSInfo1），保守上限 100 字节。
  if (Serial6.availableForWrite() < 100)
  {
    ano_tx_skipped++;
    return;
  }

  // 发送组1的数据包 (IMU, Euler Angles, Target Attitude)
  if (group_index == 0)
  {
    AnoCom.sendIMUData(acc_x_ano, acc_y_ano, acc_z_ano, gyr_x_ano, gyr_y_ano, gyr_z_ano, 0);
    AnoCom.sendAttitudeEuler(roll_ano, pitch_ano, yaw_ano, fusionSta); // `system_id` = 1
    AnoCom.sendTargetAttitude(target_roll_ano, target_pitch_ano, target_yaw_ano);
  }
  // 发送组2的数据包 (Altitude, Flight Mode, Attitude Control Output)
  else if (group_index == 1)
  {
    AnoCom.sendAltitudeData(alt_bar, alt_add, alt_fu, 1); // `system_id` = 1
    AnoCom.sendFlightMode(flight_mode_ano, flight_sflag_ano, 0, 0, 0); // 飞控模式 + 解锁状态
    // 参数顺序必须与 AnoComProtocol::sendAttitudeControl(ctrlRol, ctrlPit, ctrlThr, ctrlYaw) 一致：
    // 第3个参数是油门、第4个是偏航（曾错位传成 yaw, throttle，导致地面站油门/偏航字段互换）
    AnoCom.sendAttitudeControl(roll_ctrl_ano, pitch_ctrl_ano, throttle_ctrl_ano, yaw_ctrl_ano);
  }
  // 发送组3的数据包 (Target Speed, Flight Speed, PWM Output)
  else if (group_index == 2)
  {
    AnoCom.sendTargetSpeed(target_speed_x_ano, target_speed_y_ano, target_speed_z_ano);
    AnoCom.sendFlightSpeed(vel_north_ano * 100, vel_east_ano * 100, vel_down_ano * 100); // NED速度
    AnoCom.sendPWMOutput(pwm_ch1_ano, pwm_ch2_ano, pwm_ch3_ano, pwm_ch4_ano,
                         pwm_ch5_ano, pwm_ch6_ano, pwm_ch7_ano, pwm_ch8_ano);
  }
  // 发送组4的数据包 (Position Offset, Voltage/Current (used for pressure), GPS Info)
  else if (group_index == 3)
  {
    AnoCom.sendPosOffset(pos_north_cm_ano, pos_east_cm_ano, pos_down_cm_ano);           // 位置(cm)
    AnoCom.sendVoltCurr(bat_voltage, bat_current, fc_voltage, fc_current, power_state); // 用电压电流通道传压力
    AnoCom.sendGPSInfo1(gnss_fix_type_ano, gps_num_sat_ano,
                        gps_longitude_deg_ano, gps_latitude_deg_ano, gps_height_msl_ano,
                        gps_vel_north_ano, gps_vel_east_ano, gps_vel_down_ano, // 使用GPS的NED速度
                        gps_pdop_ano, gps_speed_acc_ano, gps_v_acc_ano);
  }

  // 增加组计数器，准备发送下一组数据
  group_index++;

  // 如果已发送完所有4组数据，则重置组计数器，并设置标志以便在下一轮开始时重新采集数据
  if (group_index >= 4)
  {
    group_index = 0;
    new_cycle_data_collection = true;
  }
}

void handleElrs()
{
  static uint8_t parse_buffer[CRSF_BUFFER_SIZE]; // 静态缓冲区，用于存储串口读取的数据
  const int MAX_PARSE_ATTEMPTS = 3;              // 最大解析尝试次数，防止卡死
  int parse_attempts_count = 0;                  // 当前解析尝试计数

  // 检查接收机串口是否有数据可读
  if (receiverSerial.available())
  {
    // 循环读取串口中所有可用的字节，直到成功解析或达到最大尝试次数
    while (receiverSerial.available() > 0 && parse_attempts_count < MAX_PARSE_ATTEMPTS)
    {
      // 从串口读取数据到解析缓冲区，最多读取CRSF_BUFFER_SIZE字节
      size_t numBytesRead = receiverSerial.readBytes(parse_buffer, CRSF_BUFFER_SIZE);

      if (numBytesRead > 0) // 确保实际读取到了数据
      {
        // 调用CRSF解析函数
        bool crsf_parse_result = crsf_parse(&parse_buffer[0], numBytesRead, // 使用实际读取的字节数
                                            raw_rc_values, &raw_rc_count,
                                            RC_INPUT_MAX_CHANNELS);
        if (crsf_parse_result) // 如果解析成功
        {
          // 清空串口缓冲区中剩余的字节，防止旧数据干扰下次解析
          while (receiverSerial.available())
          {
            receiverSerial.read();
          }
          break; // 解析成功，跳出内部while循环
        }
        else // 如果解析失败
        {
          parse_attempts_count++; // 增加尝试次数
                                  // (可选) 丢弃当前缓冲区内容或部分内容，尝试重新同步
                                  // 此处简化为清空串口，等待下一帧
          while (receiverSerial.available())
          {
            receiverSerial.read();
          }
        }
      }
      // else { delayMicroseconds(50); } // 如果没读到数据，可以稍作等待，避免空轮询太快
    }
  }

  // 如果尝试次数达到最大值仍未成功解析
  if (parse_attempts_count >= MAX_PARSE_ATTEMPTS)
  {
    Serial8.println("Max parse attempts reached without successful CRSF data parsing.");
    // (可选) 此处可以添加错误处理逻辑，例如重置某些状态或发出警告
  }
}

void handleCrsf()
{
  // (如果启用) 接收来自发动机控制器的数据
  receiveEngineData();

  // 如果成功接收到新的发动机数据，可以在此处理 (当前代码中未显式使用 receivedP1/P2)
  if (newEngineDataReceived)
  {
    // 在这里可以使用 receivedP1 和 receivedP2 的值进行进一步的处理
    // 例如：将接收到的P1和P2数据添加到CRSF数据包中发送出去
    // ... 处理 receivedP1 和 receivedP2 的代码 ...

    // 清除数据接收标志位，准备接收下一组数据
    newEngineDataReceived = false;
  }

  // 准备要发送的16个CRSF通道值
  uint16_t send_values[RC_INPUT_MAX_CHANNELS];

  // 通道1, 2: 通常是Roll, Pitch遥控输入，直接转发
  send_values[0] = raw_rc_values[0];
  send_values[1] = raw_rc_values[1];

  // 通道3: 油门，将计算得到的油门百分比 `throttlePercent` (0-100) 映射回CRSF的PWM范围 (988-2012)
  send_values[2] = static_cast<uint16_t>(mapFloat(throttlePercent, 0.0f, 100.0f, 988.0f, 2012.0f));

  // 通道4: Yaw遥控输入，直接转发
  send_values[3] = raw_rc_values[3];

  // 通道5: 解锁开关。根据燃料是否充足 (`fuelOK`) 进行安全联锁。
  // 如果燃料不足 (`fuelOK`为false)，则强制发送解锁通道为低值 (1000)，即使遥控器处于解锁位置。
  send_values[4] = fuelOK ? raw_rc_values[4] : 1000; // 如果fuelOK为true, 使用通道5原始值; 否则设为1000 (锁定)

  // 通道6: 点火开关，直接转发
  send_values[5] = raw_rc_values[5];

  // 通道7: 实验模式通道，当前固定为1000 (低位)
  send_values[6] = 1000;

  // 通道8: TVC手动/发动机PID/ADRC模式切换，直接转发
  send_values[7] = raw_rc_values[7];

  // 通道9-16: 直接转发遥控器原始值
  for (int i = 8; i < RC_INPUT_MAX_CHANNELS; ++i)
  {
    send_values[i] = raw_rc_values[i];
  }

  // 将构建好的16通道数据通过CRSF协议发送出去
  send_crsf_frame(send_values);
}

/**
 * @brief ELRS 电池数据回传任务 (25Hz)
 *
 * 将电池电压/电流映射到 CRSF BATTERY_SENSOR 协议字段，遥控器电量显示。
 * - voltage   = bat_voltage_mv / 100 (协议实际单位 0.1V，EdgeTX precision=1)
 * - current   = bat_current_ca (0.1A) — 未接入时保持 0
 * - remaining = 电池剩余百分比 — 需容量监测，未接入时保持 0
 *
 * 数据源：ADC_BATT (PC5) 电压采样，见 updateBatteryMonitor()。
 * 注：2026-08-08 移除原氧压映射（液体火箭项目遗留，不适用于当前电动 VTOL）。
 * 注：CRSF 协议头注释 "mv*100" 为过时错误注释，EdgeTX/ELRS 实际按 0.1V 解释
 *     （EdgeTX crossfire.cpp: CS(...,UNIT_VOLTS,1)，precision=1 → 显示值 = 原始值/10）。
 */
void sendElrsBatteryData()
{
  // 非阻塞保护：Serial1 (ELRS) TX 缓冲不足时跳过本帧，避免阻塞控制环。
  // CRSF BATTERY_SENSOR 帧净荷 8 字节 + 帧头/CRC 共 12 字节，保守上限 20 字节。
  if (Serial1.availableForWrite() < 20)
  {
    elrs_tx_skipped++;
    return;
  }

  // 初始化电池数据结构体
  crsf_sensor_battery_t elrsBatteryData;

  // 电压：mV → 0.1V 单位（大端）
  elrsBatteryData.voltage = htobe16((uint16_t)(bat_voltage_mv / 100.0f));

  // 电流：0.1A 单位（大端）
  elrsBatteryData.current = htobe16((uint16_t)bat_current_ca);

  // 剩余容量/百分比：无容量监测时置 0（避免错误显示）
  elrsBatteryData.capacity = 0;
  elrsBatteryData.remaining = 0;

  // 打包并发送电池数据包
  crsf.queuePacket(CRSF_FRAMETYPE_BATTERY_SENSOR, &elrsBatteryData, sizeof(elrsBatteryData));
}

/**
 * @brief ELRS 姿态回传任务 (25Hz)
 *
 * 将飞控姿态欧拉角映射到 CRSF ATTITUDE 协议字段，遥控器姿态球显示。
 * - pitch = AHRS_Packet.Pitch (rad) → 度 × 100（0.01° 粒度，int16 不溢出）
 * - roll  = AHRS_Packet.Roll  (rad) → 度 × 100
 * - yaw   = AHRS_Packet.Heading (rad) → 度 × 100
 *
 * 2026-08-08 修改：原发送"弧度×10000"在部分遥控器显示层不做 rad→deg
 * 转换，屏幕显示弧度数值却标 °。改为直接发送度值（×100）。
 * 注：EdgeTX 标准解析为 value/10 后按 precision=3 显示（即 ÷10000），
 *     若遥控器显示数值差 100 倍，需把该传感器精度(precision)改为 2 位。
 */
void sendElrsAttitudeData()
{
  // 非阻塞保护：ATTITUDE 帧净荷 6 字节 + 帧头/CRC 共 10 字节
  if (Serial1.availableForWrite() < 16)
  {
    elrs_tx_skipped++;
    return;
  }

  crsf_sensor_attitude_t attitudeData;
  attitudeData.pitch = htobe16((int16_t)(AHRS_Packet.Pitch * RAD_TO_DEG * 100.0f));
  attitudeData.roll  = htobe16((int16_t)(AHRS_Packet.Roll * RAD_TO_DEG * 100.0f));
  attitudeData.yaw   = htobe16((int16_t)(AHRS_Packet.Heading * RAD_TO_DEG * 100.0f));

  crsf.queuePacket(CRSF_FRAMETYPE_ATTITUDE, &attitudeData, sizeof(attitudeData));
}

/**
 * @brief ELRS 气压高度+垂直速度回传任务 (25Hz)
 *
 * 将 DPS310 气压高度和 EKF 垂直速度映射到 CRSF BARO_ALTITUDE 字段：
 * - altitude    = baro_altitude (m) × 10 + 10000dm 基准（高比特位=0 表示分米）
 * - verticalspd = estimated_velocity (m/s) × 100 (cm/s)
 */
void sendElrsBaroAltitudeData()
{
  // 非阻塞保护：BARO_ALTITUDE 帧净荷 4 字节 + 帧头/CRC 共 8 字节
  if (Serial1.availableForWrite() < 12)
  {
    elrs_tx_skipped++;
    return;
  }

  crsf_sensor_baro_vario_t baroData;
  // 高度：米 → 分米（+10000 偏移，满足协议格式）
  int16_t alt_dm = (int16_t)(baro_altitude * 10) + 10000;
  baroData.altitude = htobe16((uint16_t)alt_dm);
  // 垂直速度：m/s → cm/s
  baroData.verticalspd = htobe16((int16_t)(estimated_velocity * 100));

  crsf.queuePacket(CRSF_FRAMETYPE_BARO_ALTITUDE, &baroData, sizeof(baroData));
}

/**
 * @brief ELRS 飞行模式回传任务 (10Hz)
 *
 * 将当前控制模式名映射到 CRSF FLIGHT_MODE 字段（16 字符）。
 */
void sendElrsFlightModeData()
{
  // 非阻塞保护：FLIGHT_MODE 帧净荷 16 字节 + 帧头/CRC 共 20 字节
  if (Serial1.availableForWrite() < 24)
  {
    elrs_tx_skipped++;
    return;
  }

  // 从遥控器通道6获取当前控制模式
  ControlMode mode = getControlMode((float)raw_rc_values[6]);

  crsf_flight_mode_t modeData;
  memset(&modeData, 0, sizeof(modeData));

  // 模式名映射（对应 getControlMode 的枚举）
  const char *modeName;
  switch (mode)
  {
    case MANUAL:         modeName = "MANUAL"; break;
    case AUTO_POSITION:  modeName = "POSHOLD"; break;
    case AUTO_ALTITUDE:  modeName = "ALTHOLD"; break;
    case GUIDED:         modeName = "GUIDED"; break;
    default:             modeName = "AUTO"; break;
  }
  strncpy(modeData.flight_mode, modeName, sizeof(modeData.flight_mode) - 1);

  crsf.queuePacket(CRSF_FRAMETYPE_FLIGHT_MODE, &modeData, sizeof(modeData));
}

/**
 * @brief ELRS GNSS 位置回传任务 (10Hz)
 *
 * 将 UBX GNSS 定位数据映射到 CRSF GPS 协议字段，遥控器地图/位置显示：
 * - latitude/longitude   = 经纬度 × 1e7（1e-7 度分辨率）
 * - groundspeed          = 北/东速度合成地速 (km/h × 10)
 * - heading              = 运动方向 (度 × 100)
 * - altitude             = WGS84 高度 + 1000m 偏移
 * - satellites           = 参与解算卫星数
 *
 * 数据新鲜度：仅当 GNSS 数据新鲜且 3D 定位时发送真实数据，否则发零值。
 */
void sendElrsGpsData()
{
  // 非阻塞保护：GPS 帧净荷 15 字节 + 帧头/CRC 共 19 字节
  if (Serial1.availableForWrite() < 24)
  {
    elrs_tx_skipped++;
    return;
  }

  crsf_sensor_gps_t gpsData = {0};

  // 检查 GNSS 数据新鲜度（复用遥测层同一判定逻辑）
  const bool gnss_data_fresh = isGnssDataFreshForNav();
  if (gnss_data_fresh && ubx.fix() >= bfs::Ubx::FIX_3D)
  {
    // 经纬度 (度 → 1e-7 度)
    gpsData.latitude = htobe32((int32_t)(ubx.lat_deg() * 1e7));
    gpsData.longitude = htobe32((int32_t)(ubx.lon_deg() * 1e7));

    // 地速：北/东速度合成 (m/s → km/h × 10)
    const float vn = ubx.north_vel_mps(), ve = ubx.east_vel_mps();
    const float ground_speed_mps = sqrtf(vn * vn + ve * ve);
    gpsData.groundspeed = htobe16((uint16_t)(ground_speed_mps * 3.6f * 10.0f));

    // 航向：运动方向 (rad → 度 × 100)，NED 系 atan2(east, north)
    float heading_deg = atan2f(ve, vn) * RAD_TO_DEG;
    if (heading_deg < 0) heading_deg += 360.0f;
    gpsData.heading = htobe16((uint16_t)(heading_deg * 100.0f));

    // 高度：WGS84 椭球高 + 1000m 偏移（协议要求）
    gpsData.altitude = htobe16((uint16_t)(Geodetic_Pos_Packet.height + 1000.0f));

    // 卫星数
    gpsData.satellites = (uint8_t)ubx.num_sv();
  }
  // 数据不新鲜或无 3D 定位时保持全零，遥控器显示无效

  crsf.queuePacket(CRSF_FRAMETYPE_GPS, &gpsData, sizeof(gpsData));
}

/**
 * @brief ELRS 垂直速度回传任务 (25Hz)
 *
 * 将 EKF 垂直速度映射到 CRSF VARIO 协议字段，遥控器变率计显示。
 * - verticalspd = estimated_velocity (m/s, 向上为正) × 100 (cm/s)
 */
void sendElrsVarioData()
{
  // 非阻塞保护：VARIO 帧净荷 2 字节 + 帧头/CRC 共 6 字节
  if (Serial1.availableForWrite() < 12)
  {
    elrs_tx_skipped++;
    return;
  }

  crsf_sensor_vario_t varioData;
  varioData.verticalspd = htobe16((int16_t)(estimated_velocity * 100));

  crsf.queuePacket(CRSF_FRAMETYPE_VARIO, &varioData, sizeof(varioData));
}

/**
 * @brief ELRS 温度回传任务 (10Hz)
 *
 * 将 DPS310 气压计温度映射到 CRSF TEMP 协议字段，遥控器温度显示。
 * - source_id    = 0 (FC 本体)
 * - temperature[0] = temperature × 10 (0.1°C)
 */
void sendElrsTempData()
{
  // 非阻塞保护：TEMP 帧净荷 41 字节 (1+20×2) + 帧头/CRC 共 45 字节
  if (Serial1.availableForWrite() < 48)
  {
    elrs_tx_skipped++;
    return;
  }

  crsf_sensor_temp_t tempData;
  memset(&tempData, 0, sizeof(tempData));
  tempData.source_id = 0;  // 0 = FC 包括所有 ESC
  tempData.temperature[0] = htobe16((int16_t)(temperature * 10.0f));

  crsf.queuePacket(CRSF_FRAMETYPE_TEMP, &tempData, sizeof(tempData));
}

/**
 * @brief 电池电压采样任务 (10Hz)
 *
 * 从 ADC_BATT (PC5) 读取电池分压电压，换算为电池真实电压 (mV)。
 * - ADC 16位 (0-65535)，参考 3.3V
 * - 分压比 BATTERY_DIVIDER_RATIO 需按 PCB 实测标定（默认 1.0）
 * - 未标定前输出 = ADC 原始电压，量程 0-3.3V
 *
 * 输出全局量 bat_voltage_mv，供 ELRS / AnoCom 遥测共用。
 */
void updateBatteryMonitor()
{
  // ADC 16位采样 → 分压点电压 (V) → 分压比换算 → 电池电压 (mV)
  int adcValue = analogRead(ADC_BATT_PIN);
  float dividerVoltage = static_cast<float>(adcValue) * (3.3f / 65535.0f);
  bat_voltage_mv = dividerVoltage * BATTERY_DIVIDER_RATIO * 1000.0f;

  // 电流采样未接入（ADC_CURR 无传感器），保持 0
  bat_current_ca = 0.0f;
}

void handleTelemetry()
{
  float filteredSensor1Angle, filteredSensor2Angle;
  getFilteredTVCAngles(filteredSensor1Angle, filteredSensor2Angle);

  // 打印姿态欧拉角 (Roll, Pitch, Heading in degrees)
  Serial8.print(AHRS_Packet.Roll * RAD_TO_DEG, 2);
  Serial8.print(", ");
  Serial8.print(AHRS_Packet.Pitch * RAD_TO_DEG, 2);
  Serial8.print(", ");
  Serial8.print(AHRS_Packet.Heading * RAD_TO_DEG, 2);
  Serial8.print(", ");
  // 打印姿态四元数 (Qw, Qx, Qy, Qz)
  Serial8.print(AHRS_Packet.Qw, 4);
  Serial8.print(", ");
  Serial8.print(AHRS_Packet.Qx, 4);
  Serial8.print(", ");
  Serial8.print(AHRS_Packet.Qy, 4);
  Serial8.print(", ");
  Serial8.print(AHRS_Packet.Qz, 4);
  Serial8.print(", ");
  // 打印角速度
  Serial8.print(IMU_Packet.gyroscope_x * RAD_TO_DEG, 2);
  Serial8.print(", ");
  Serial8.print(IMU_Packet.gyroscope_y * RAD_TO_DEG, 2);
  Serial8.print(", ");
  Serial8.print(IMU_Packet.gyroscope_z * RAD_TO_DEG, 2);
  Serial8.print(", ");
  // 打印加速度
  Serial8.print(IMU_Packet.accelerometer_x, 2);
  Serial8.print(", ");
  Serial8.print(IMU_Packet.accelerometer_y, 2);
  Serial8.print(", ");
  Serial8.print(IMU_Packet.accelerometer_z, 2);
  Serial8.print(", ");
  // 打印姿态欧拉角误差（来源 gnc_tel，2026-08-08 C路径重构）
  Serial8.print(gnc_tel.error_deg[0], 2);
  Serial8.print(", ");
  Serial8.print(gnc_tel.error_deg[1], 2);
  Serial8.print(", ");
  // 打印当前时间戳 (毫秒，保留3位小数)
  Serial8.print(micros() / 1000.0f, 3);
  Serial8.println(); // 换行
}

void handleDataLogging()
{
  // 记录使能 = 真实解锁 OR 调试强制（datalog 命令，不受 flight_control 每帧覆盖）
  // 注意：不能直接改 isDatalogging（flight_control.cpp:89 每帧用 inputs.is_unlocked 覆盖），
  // 用独立调试标志 + 超时
  bool dbgLogging = (s_dbgLogUntil != 0 && (int32_t)(millis() - s_dbgLogUntil) < 0);
  bool loggingActive = isDatalogging || dbgLogging;

  static bool wasLogging = false; // 记录上一次调用时的状态
  static uint32_t segmentId = 0;  // 分段编号（从 1 开始）

  const unsigned long currentTime = millis();

  // —— 检测开始：false → true ——
  if (loggingActive && !wasLogging)
  {
    segmentId++;
    // 段开始标记（以 # 开头，避免与 CSV 冲突）
    Serial3.print(F("#LOG_START,segment="));
    Serial3.print(segmentId);
    Serial3.print(F(",t_ms="));
    Serial3.println(currentTime);

    // 可选：打印列名头（仅本段第一次，方便后处理）
    Serial3.println(F(
        "t_ms,"
        "roll_deg,pitch_deg,heading_deg,"
        "accel_x_ms2,accel_y_ms2,accel_z_ms2,"
        "gyro_x_dps,gyro_y_dps,gyro_z_dps,"
        "vel_n_ms,vel_e_ms,vel_d_ms,"
        "rel_n_m,rel_e_m,rel_d_m,"
        "tvc1_deg,tvc2_deg,"
        "valve_ctrl,p1,p2"));
  }

  // —— 正常数据输出 ——
  if (loggingActive)
  {
    float filteredSensor1Angle, filteredSensor2Angle;
    getFilteredTVCAngles(filteredSensor1Angle, filteredSensor2Angle); // 获取TVC反馈角度

    // 与 CSV 列一一对应的数据快照（也用于 Flash 双写，21 float = 84B payload）
    float d[21];
    d[0]  = (float)currentTime;
    d[1]  = AHRS_Packet.Roll * RAD_TO_DEG;
    d[2]  = AHRS_Packet.Pitch * RAD_TO_DEG;
    d[3]  = AHRS_Packet.Heading * RAD_TO_DEG;
    d[4]  = IMU_Packet.accelerometer_x;
    d[5]  = IMU_Packet.accelerometer_y;
    d[6]  = IMU_Packet.accelerometer_z;
    d[7]  = IMU_Packet.gyroscope_x * RAD_TO_DEG;
    d[8]  = IMU_Packet.gyroscope_y * RAD_TO_DEG;
    d[9]  = IMU_Packet.gyroscope_z * RAD_TO_DEG;
    d[10] = INS_GNSS_Packet.velocity_north;
    d[11] = INS_GNSS_Packet.velocity_east;
    d[12] = INS_GNSS_Packet.velocity_down;
    d[13] = relative_north;
    d[14] = relative_east;
    d[15] = relative_down;
    d[16] = filteredSensor1Angle;
    d[17] = filteredSensor2Angle;
    d[18] = receivedValveControl;
    d[19] = receivedP1;
    d[20] = receivedP2;

    // 依次打印各项数据，以逗号分隔（保持原有顺序与精度）
    // ★ 调试强制模式（datalog）跳过 Serial3 输出——避免洪水淹没调试串口，
    //   只写 Flash（验证 Flash 写入链路用）
    if (!dbgLogging)
    {
    Serial3.print(currentTime);
    Serial3.print(",");
    // 姿态欧拉角 (度)
    Serial3.print(d[1], 2);
    Serial3.print(",");
    Serial3.print(d[2], 2);
    Serial3.print(",");
    Serial3.print(d[3], 2);
    Serial3.print(",");
    // IMU加速度计数据 (m/s^2)
    Serial3.print(d[4], 4);
    Serial3.print(",");
    Serial3.print(d[5], 4);
    Serial3.print(",");
    Serial3.print(d[6], 4);
    Serial3.print(",");
    // IMU陀螺仪数据 (度/秒)
    Serial3.print(d[7], 2);
    Serial3.print(",");
    Serial3.print(d[8], 2);
    Serial3.print(",");
    Serial3.print(d[9], 2);
    Serial3.print(",");
    // INS/GNSS组合导航速度 (NED系, m/s)
    Serial3.print(d[10], 3);
    Serial3.print(",");
    Serial3.print(d[11], 3);
    Serial3.print(",");
    Serial3.print(d[12], 3);
    Serial3.print(",");
    // INS/GNSS组合导航相对位置 (NED系, m)
    Serial3.print(d[13], 3);
    Serial3.print(",");
    Serial3.print(d[14], 3);
    Serial3.print(",");
    Serial3.print(d[15], 3);
    Serial3.print(",");
    // TVC角度传感器反馈 (度)
    Serial3.print(d[16], 2);
    Serial3.print(",");
    Serial3.print(d[17], 2);
    Serial3.print(",");
    // 发动机控制器回传数据
    Serial3.print(d[18], 2); // 阀门控制量
    Serial3.print(",");
    Serial3.print(d[19], 2); // 压力1
    Serial3.print(",");
    Serial3.print(d[20], 2); // 压力2
    Serial3.println();       // 换行，结束当前行数据
    }

    // Flash 双写：payload 为 84 字节二进制快照（非阻塞，µs 级入环形缓冲）
    // 注意：仅当 Flash 初始化成功（present）才写，避免无用开销
    if (flash.isPresent())
    {
      s_flashPushCount++;
      flashLog.logPush((const uint8_t *)d);
    }
  }

  // —— 检测结束：true → false ——
  if (!loggingActive && wasLogging)
  {
    Serial3.print(F("#LOG_END,segment="));
    Serial3.print(segmentId);
    Serial3.print(F(",t_ms="));
    Serial3.println(currentTime);
    // 打印十个空行，以便于后续的数据解析
    for (int i = 0; i < 10; i++)
    {
      Serial3.println();
    }
  }

  wasLogging = loggingActive; // 更新边沿检测状态
}

// [已迁移至 main.cpp] handleDeta100 (依赖 DETA100_module.h 函数)

void sendPositionVelocityData()
{
  // 1. 准备8个float类型的数据
  float data_to_send[8] = {
      static_cast<float>(trajectoryPlanningStarted), // [0] 点火/轨迹规划开始标志 (0或1)
      millis() / 1000.0f,                            // [1] 时间 t (秒)
      // 位置数据使用相对位置
      relative_north,    // [2] 相对北向位置 rx
      relative_east,     // [3] 相对东向位置 ry
      -estimated_height, // [4] 相对地向位置 rz
      // 速度 (NED系, 米/秒) - 来自DETA100 INS_GNSS_Packet
      INS_GNSS_Packet.velocity_north, // [5] 北向速度 vx
      INS_GNSS_Packet.velocity_east,  // [6] 东向速度 vy
      -estimated_velocity             // [7] 地向速度 vz，使用气压计融合速度
                                      // 加速度 (机体坐标系, m/s^2) - 来自DETA100 IMU_Packet
                                      // IMU_Packet.accelerometer_x, // [8] 机体X轴加速度 ax
                                      // IMU_Packet.accelerometer_y, // [9] 机体Y轴加速度 ay
                                      // IMU_Packet.accelerometer_z, // [10] 机体Z轴加速度 az
                                      // // 姿态欧拉角 (弧度) - 来自DETA100 AHRS_Packet
                                      // AHRS_Packet.Roll,    // [11] 滚转角 roll
                                      // AHRS_Packet.Pitch,   // [12] 俯仰角 pitch
                                      // AHRS_Packet.Heading, // [13] 偏航角 yaw
                                      // // 角速度 (机体坐标系, 弧度/秒) - 来自DETA100 AHRS_Packet
                                      // AHRS_Packet.RollSpeed,   // [14] 滚转角速度
                                      // AHRS_Packet.PitchSpeed,  // [15] 俯仰角速度
                                      // AHRS_Packet.HeadingSpeed // [16] 偏航角速度
  };

  // 2. 构建帧头和帧尾
  serial5Buffer[0] = 0xAA;         // 帧头
  serial5Buffer[1 + 8 * 4] = 0x55; // 帧尾，位于所有数据之后 (索引33)

  // 3. 将每个float数据转换为大端字节序并填充到缓冲区
  // for (int i = 0; i < 8; i++)
  // {
  //   // 数据从缓冲区的第1个字节开始存放 (索引1)，每个float占4字节
  //   floatToBigEndianBytes(data_to_send[i], &serial5Buffer[1 + i * 4]);
  // }

  // 3. 将每个float数据直接填充到缓冲区
  for (int i = 0; i < 8; i++)
  {
    // 数据从缓冲区的第1个字节开始存放 (索引1)，每个float占4字节
    memcpy(&serial5Buffer[1 + i * 4], &data_to_send[i], sizeof(float));
  }

  // 4. 通过Serial5发送完整的数据帧 (总共34字节)
  // 非阻塞保护：TX 缓冲不足时跳过本帧，避免阻塞控制环。
  if (Serial5.availableForWrite() < 34)
  {
    posvel_tx_skipped++;
    return;
  }
  Serial5.write(serial5Buffer, 34);
}

void handleGuidanceCommands()
{
  // --- 静态状态保持 (Static State Persistence) ---
  static PacketState current_state = PacketState::WAIT_HEADER;
  static uint8_t payload_buffer[GUIDANCE_CMD_PAYLOAD_LEN];
  static uint8_t payload_index = 0;

  // --- 临时变量 ---
  uint8_t byte_in;
  uint8_t processed_count = 0;

  // 循环读取串口，直到缓冲区空或达到单次处理上限
  while (Serial5.available() && processed_count < MAX_BYTES_PER_LOOP)
  {
    byte_in = Serial5.read(); // 读取一个字节
    processed_count++;        // 计数器累加

    switch (current_state)
    {
    // -----------------------------------------------------------
    // 状态 1: 寻找帧头
    // -----------------------------------------------------------
    case PacketState::WAIT_HEADER:
      if (byte_in == GUIDANCE_CMD_HEADER) // 0xAA
      {
        current_state = PacketState::RX_PAYLOAD;
        payload_index = 0; // 重置载荷索引
      }
      break;

    // -----------------------------------------------------------
    // 状态 2: 接收数据载荷 (12 Bytes)
    // -----------------------------------------------------------
    case PacketState::RX_PAYLOAD:
      payload_buffer[payload_index++] = byte_in;

      // 如果收满了12个字节，切换状态去检查帧尾
      if (payload_index >= GUIDANCE_CMD_PAYLOAD_LEN)
      {
        current_state = PacketState::WAIT_TAIL;
      }
      break;

    // -----------------------------------------------------------
    // 状态 3: 校验帧尾并解码
    // -----------------------------------------------------------
    case PacketState::WAIT_TAIL:
      if (byte_in == GUIDANCE_CMD_TAIL) // 0x55 -> 校验成功
      {
        // --- 1. 零拷贝解析 (Zero-Copy Parsing) ---
        // 变量名已修改：从 raw_throttle/tilt 更改为 raw_acc_*
        // 对应关系：
        // Packet Byte 0-3  -> Accel Up (原 throttle 位置)
        // Packet Byte 4-7  -> Accel East (原 tilt_E 位置)
        // Packet Byte 8-11 -> Accel North (原 tilt_N 位置)
        float raw_acc_U, raw_acc_E, raw_acc_N;

        // 使用 memcpy 防止内存对齐问题
        memcpy(&raw_acc_U, &payload_buffer[0], sizeof(float));
        memcpy(&raw_acc_E, &payload_buffer[4], sizeof(float));
        memcpy(&raw_acc_N, &payload_buffer[8], sizeof(float));

        // --- 2. 安全性检查 (Sanity Check) ---
        // 更新了校验逻辑：
        // 1. 检查 NaN/Inf
        // 2. 检查数值范围是否符合 m/s^2 的物理直觉
        //    旧逻辑的 -5.0~5.0 可能对垂直加速度不够用 (急升可能需要更大)
        //    使用 fabsf 取绝对值进行判断更简洁
        bool is_valid = isfinite(raw_acc_U) && isfinite(raw_acc_E) && isfinite(raw_acc_N) &&
                        (fabsf(raw_acc_E) < MAX_SAFE_ACCEL_XY) &&
                        (fabsf(raw_acc_N) < MAX_SAFE_ACCEL_XY) &&
                        (fabsf(raw_acc_U) < MAX_SAFE_ACCEL_Z);

        if (is_valid)
        {
          // --- 3. 更新全局原子变量 ---
          guidance_accel_U_cmd = raw_acc_U;
          guidance_accel_E_cmd = raw_acc_E;
          guidance_accel_N_cmd = raw_acc_N;

          new_guidance_command_received = true;
          last_guidance_command_millis = millis();

          // (可选) 调试输出
          // Serial8.print("Cmd Acc: ");
          // Serial8.print(guidance_accel_U_cmd); Serial8.print(" ");
          // Serial8.print(guidance_accel_E_cmd); Serial8.print(" ");
          // Serial8.println(guidance_accel_N_cmd);
        }
        else
        {
          // 可以在这里添加错误计数或警告，说明接收到了非法的加速度值
        }

        // 成功接收一帧后，回到寻找下一帧头的状态
        current_state = PacketState::WAIT_HEADER;
      }
      else
      {
        // --- 关键优化：快速重同步 (Fast Resync) ---
        if (byte_in == GUIDANCE_CMD_HEADER)
        {
          current_state = PacketState::RX_PAYLOAD;
          payload_index = 0;
        }
        else
        {
          current_state = PacketState::WAIT_HEADER;
        }
      }
      break;

    default:
      current_state = PacketState::WAIT_HEADER;
      break;
    }
  }
}

void handleStatusLedTask()
{
  // 获取解锁状态
  bool is_armed = (raw_rc_values[4] > 1500);

  // 获取校准状态
  bool is_calibrating = imuCalibrator.isCalibrating();

  // 更新LED
  statusLed.update(is_calibrating, is_armed);
  ws2812AppStatus.update(is_calibrating, is_armed);   // WS2812 RGB状态灯 (与绿灯并行)
}

/**
 * @brief Flash 黑匣子后台写任务（低优先级）
 *
 * 每 tick 调用 flashLog.logService()（内部预算 W25N01GV_LOG_MAX_WRITES 帧），
 * 从 RAM 环形缓冲批量写页到 NAND。页编程 ~0.7ms/页，2 页 = 1.4ms 阻塞，
 * 对 100Hz 任务（10ms 周期）占 14% —— 可接受（CAN_RX_MAX_PER_TICK 同款预算模式）。
 */
void handleFlashService()
{
  // 调试黑匣子强制记录超时自动复位（datalog 命令；不动 isDatalogging，
  // 它由 flight_control 每帧按真实解锁状态维护）
  if (s_dbgLogUntil != 0 && (int32_t)(millis() - s_dbgLogUntil) >= 0)
  {
    s_dbgLogUntil = 0;
  }

  s_flashServiceCount++;
  // ★ Flash 写页互斥：写页 + 之后 50ms 跳过 CAN（MCP2515 需足够时间恢复，
  //   否则超时 reset → SPI2 OVR 卡死）。每 200ms 写 1 页，跳 50ms CAN 可接受。
  s_flashWriting = true;
  flashLog.logService();
  s_flashWritingUntil = millis() + 50;
  s_flashWriting = false;
}

void handleMavlink()
{
  static uint32_t call_count = 0; // uint32 防止 200Hz 下快速溢出
  call_count++;

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // ====================================================================
  // HEARTBEAT (1Hz)
  // ====================================================================
  if (call_count % 200 == 0)
  {
    ControlMode mode = getControlMode((float)raw_rc_values[6]);
    bool armed = (raw_rc_values[4] > 1500);

    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    if (armed)
      base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    switch (mode)
    {
    case MANUAL:
      base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
      break;
    case AUTO_POSITION:
    case AUTO_ALTITUDE:
      base_mode |= MAV_MODE_FLAG_AUTO_ENABLED;
      break;
    case GUIDED:
      base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
      break;
    }

    uint32_t custom_mode = (uint32_t)mode;
    uint8_t system_status = armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY;

    mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg,
                               MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC,
                               base_mode, custom_mode, system_status);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial6.write(buf, len);
  }

  // ====================================================================
  // SYS_STATUS (1Hz)
  // ====================================================================
  if (call_count % 200 == 1)
  {
    uint32_t sensors = MAV_SYS_STATUS_SENSOR_3D_GYRO |
                       MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                       MAV_SYS_STATUS_SENSOR_3D_MAG |
                       MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE |
                       MAV_SYS_STATUS_SENSOR_GPS |
                       MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;

    uint16_t vbat_mv = 12600; // 12.6V = 3S LiPo (mV)
    int16_t ibat_ca = -1;     // 电流未知 (cA)

    mavlink_msg_sys_status_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg,
                                sensors, sensors, sensors,
                                0, vbat_mv, ibat_ca, -1, // load, voltage, current, remaining
                                0, 0, 0, 0, 0, 0,        // drop, errors
                                0, 0, 0);                // extended
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial6.write(buf, len);
  }

  // ====================================================================
  // ATTITUDE (40Hz) — 姿态欧拉角
  // ====================================================================
  if (call_count % 5 == 0)
  {
    mavlink_msg_attitude_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg,
                              millis(),
                              AHRS_Packet.Roll,
                              AHRS_Packet.Pitch,
                              AHRS_Packet.Heading,
                              IMU_Packet.gyroscope_x,
                              IMU_Packet.gyroscope_y,
                              IMU_Packet.gyroscope_z);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial6.write(buf, len);
  }

  // ====================================================================
  // ATTITUDE_QUATERNION (40Hz) — 四元数姿态
  // ====================================================================
  if (call_count % 5 == 1)
  {
    static const float repr_offset_q[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    mavlink_msg_attitude_quaternion_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg,
                                         millis(),
                                         AHRS_Packet.Qw, AHRS_Packet.Qx,
                                         AHRS_Packet.Qy, AHRS_Packet.Qz,
                                         IMU_Packet.gyroscope_x,
                                         IMU_Packet.gyroscope_y,
                                         IMU_Packet.gyroscope_z,
                                         repr_offset_q);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial6.write(buf, len);
  }

  // ====================================================================
  // GLOBAL_POSITION_INT (20Hz) — GPS + 速度 + 航向
  // ====================================================================
  if (call_count % 10 == 2)
  {
    int32_t lat = (int32_t)(Geodetic_Pos_Packet.latitude * 180.0 / M_PI * 1e7);
    int32_t lon = (int32_t)(Geodetic_Pos_Packet.longitude * 180.0 / M_PI * 1e7);
    int32_t alt_ellipsoid = (int32_t)(Geodetic_Pos_Packet.height * 1000.0f);
    int32_t relative_alt = (int32_t)(estimated_height * 1000.0f);

    int16_t vx = (int16_t)constrain(INS_GNSS_Packet.velocity_north * 100.0f, -32767.0f, 32767.0f);
    int16_t vy = (int16_t)constrain(INS_GNSS_Packet.velocity_east * 100.0f, -32767.0f, 32767.0f);
    int16_t vz = (int16_t)constrain(INS_GNSS_Packet.velocity_down * 100.0f, -32767.0f, 32767.0f);
    uint16_t hdg = (uint16_t)(AHRS_Packet.Heading * 180.0 / M_PI * 100.0);

    mavlink_msg_global_position_int_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg,
                                         millis(),
                                         lat, lon, alt_ellipsoid, relative_alt,
                                         vx, vy, vz, hdg);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial6.write(buf, len);
  }

  // ====================================================================
  // VFR_HUD (20Hz) — Mission Planner HUD 主界面
  // ====================================================================
  if (call_count % 10 == 3)
  {
    float groundspeed = sqrtf(INS_GNSS_Packet.velocity_north * INS_GNSS_Packet.velocity_north +
                              INS_GNSS_Packet.velocity_east * INS_GNSS_Packet.velocity_east);
    float heading_deg = AHRS_Packet.Heading * RAD_TO_DEG;
    float climb_rate = -INS_GNSS_Packet.velocity_down;

    mavlink_msg_vfr_hud_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg,
                             0.0f, groundspeed,
                             (int16_t)heading_deg,
                             (uint16_t)throttlePercent,
                             estimated_height, climb_rate);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial6.write(buf, len);
  }

  // ====================================================================
  // RC_CHANNELS_RAW (20Hz)
  // ====================================================================
  if (call_count % 10 == 4)
  {
    mavlink_msg_rc_channels_raw_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg,
                                     millis(), 0,
                                     (uint16_t)raw_rc_values[0],
                                     (uint16_t)raw_rc_values[1],
                                     (uint16_t)raw_rc_values[2],
                                     (uint16_t)raw_rc_values[3],
                                     (uint16_t)raw_rc_values[4],
                                     (uint16_t)raw_rc_values[5],
                                     (uint16_t)raw_rc_values[6],
                                     (uint16_t)raw_rc_values[7],
                                     255);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial6.write(buf, len);
  }

  // ====================================================================
  // RAW_IMU (40Hz) — 加速度计 + 陀螺仪原始值
  // ====================================================================
  if (call_count % 5 == 3)
  {
    int16_t ax = (int16_t)constrain(IMU_Packet.accelerometer_x * 1000.0f / G_ACCEL_CONST, -32767.0f, 32767.0f);
    int16_t ay = (int16_t)constrain(IMU_Packet.accelerometer_y * 1000.0f / G_ACCEL_CONST, -32767.0f, 32767.0f);
    int16_t az = (int16_t)constrain(IMU_Packet.accelerometer_z * 1000.0f / G_ACCEL_CONST, -32767.0f, 32767.0f);
    int16_t gx = (int16_t)constrain(IMU_Packet.gyroscope_x * 1000.0f, -32767.0f, 32767.0f);
    int16_t gy = (int16_t)constrain(IMU_Packet.gyroscope_y * 1000.0f, -32767.0f, 32767.0f);
    int16_t gz = (int16_t)constrain(IMU_Packet.gyroscope_z * 1000.0f, -32767.0f, 32767.0f);

    mavlink_msg_raw_imu_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg,
                             (uint64_t)millis() * 1000ULL,
                             ax, ay, az, gx, gy, gz,
                             0, 0, 0, 0, 0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial6.write(buf, len);
  }

  // ====================================================================
  // BATTERY_STATUS (1Hz)
  // ====================================================================
  if (call_count % 200 == 5)
  {
    uint16_t voltages[10] = {0};
    voltages[0] = 12600;
    uint16_t voltages_ext[4] = {0};

    mavlink_msg_battery_status_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg,
                                    0, MAV_BATTERY_FUNCTION_ALL,
                                    MAV_BATTERY_TYPE_LIPO, INT16_MAX,
                                    voltages, -1, -1, -1, -1, 0,
                                    MAV_BATTERY_CHARGE_STATE_UNDEFINED,
                                    voltages_ext, 0, 0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial6.write(buf, len);
  }
}

// ========================================================================
// 调试模式控制台（Serial6 共用，收到 "DBG\n" 进入）
// ========================================================================

/**
 * @brief 调试命令解释器（调试模式下替代正常数传逻辑）
 *
 * 命令集（\n 结尾）：
 *   help          列出所有命令
 *   ws <r> <g> <b> 发送一帧指定颜色 (0-255)
 *   wsoff          WS2812 熄灭
 *   wsseq [ms]     自动测试序列 红→绿→蓝→白→灭 (默认 500ms/色)
 *   ver           版本/启动信息
 *   exit          退出调试模式，恢复地面站数传
 *
 * @param serial  使用的串口（Serial6）
 * @param line    行缓冲
 * @param lineLen 行缓冲长度指针
 * @param maxLen  缓冲最大长度
 * @param dbgMode 调试模式标志（引用，exit 时置 false）
 */
void handleDebugConsole(HardwareSerial &serial, char *line, uint8_t *lineLen,
                        uint8_t maxLen, bool &dbgMode)
{
  // ---- 收集一行（\n 结尾）----
  while (serial.available()) {
    char c = (char)serial.read();
    if (c == '\n') {
      line[*lineLen] = '\0';
      *lineLen = 0;

      // ---- 解析命令 ----
      if (strncmp(line, "help", 4) == 0) {
        serial.println(F("[DBG] Commands:"));
        serial.println(F("[DBG]   ws <r> <g> <b>   - send color frame (0-255)"));
        serial.println(F("[DBG]   wsoff             - turn WS2812 off"));
        serial.println(F("[DBG]   wsseq [ms]        - auto test sequence (default 500)"));
        serial.println(F("[DBG]   wsstat            - show WS2812 status"));
        serial.println(F("[DBG]   wsmode <0|1>      - switch driver (0=bitbang 1=tim4_dma)"));
        serial.println(F("[DBG]   ver               - version info"));
        serial.println(F("[DBG]   exit              - back to normal telemetry"));
      }
      else if (strncmp(line, "ws ", 3) == 0) {
        int r, g, b;
        if (sscanf(line + 3, "%d %d %d", &r, &g, &b) == 3) {
          ws2812Led.setPixelColor(0,
                                  (uint8_t)constrain(r, 0, 255),
                                  (uint8_t)constrain(g, 0, 255),
                                  (uint8_t)constrain(b, 0, 255));
          ws2812Led.show();
          serial.print(F("[DBG] WS2812 <- R"));
          serial.print(r); serial.print(F(" G")); serial.print(g);
          serial.print(F(" B")); serial.println(b);
        } else {
          serial.println(F("[DBG] usage: ws <r> <g> <b>"));
        }
      }
      else if (strncmp(line, "wsoff", 5) == 0) {
        ws2812Led.clear();
        ws2812Led.show();
        serial.println(F("[DBG] WS2812 off"));
      }
      else if (strncmp(line, "wsseq", 5) == 0) {
        uint32_t ms = 500;
        int ms_arg;
        if (sscanf(line + 5, "%d", &ms_arg) == 1) ms = (uint32_t)constrain(ms_arg, 50, 5000);
        serial.print(F("[DBG] sequence, "));
        serial.print(ms); serial.println(F("ms/color..."));
        // 自动测试序列：红 → 绿 → 蓝 → 白 → 灭
        ws2812Led.fill(255, 0, 0); ws2812Led.show(); delay(ms);
        ws2812Led.fill(0, 255, 0); ws2812Led.show(); delay(ms);
        ws2812Led.fill(0, 0, 255); ws2812Led.show(); delay(ms);
        ws2812Led.fill(255, 255, 255); ws2812Led.show(); delay(ms);
        ws2812Led.clear(); ws2812Led.show();
        serial.println(F("[DBG] sequence done"));
      }
      else if (strcmp(line, "wsstat") == 0) {  // 精确匹配，避免抢 wsstatic 前缀
        serial.print(F("[DBG] pin="));
        serial.println(WS2812_PIN);
        serial.print(F("[DBG] pixels="));
        serial.println(ws2812Led.numPixels());
        serial.print(F("[DBG] brightness="));
        serial.println(ws2812Led.getBrightness());
        serial.print(F("[DBG] driver="));
        serial.println((int)ws2812Led.getMode());   // 0=bitbang 1=TIM4_DMA
        serial.print(F("[DBG] dmaActive="));
        serial.println(ws2812Led.isDmaActive());
      }
      else if (strncmp(line, "wsmode", 6) == 0) {
        int m;
        if (sscanf(line + 6, "%d", &m) == 1) {
          if (m == 0 || m == 1) {
            ws2812Led.setMode((WS2812Driver::Mode)m);
            serial.print(F("[DBG] driver switched to "));
            serial.println(m == 0 ? F("BITBANG") : F("TIM4_DMA"));
          } else {
            serial.println(F("[DBG] usage: wsmode <0|1> (0=bitbang 1=tim4_dma)"));
          }
        } else {
          serial.print(F("[DBG] current driver="));
          serial.println((int)ws2812Led.getMode());
        }
      }
      else if (strncmp(line, "wsfault", 7) == 0) {
        int f;
        if (sscanf(line + 7, "%d", &f) == 1) {
          ws2812AppStatus.setFault(f != 0);
          serial.print(F("[DBG] fault="));
          serial.println(f != 0 ? F("ON (red blink)") : F("OFF"));
        } else {
          serial.println(F("[DBG] usage: wsfault <0|1>"));
        }
      }
      else if (strncmp(line, "gpio", 4) == 0) {
        debugGpioDump(serial);
      }
      else if (strcmp(line, "tim4") == 0) {
        debugTim4Dump(serial);
      }
      else if (strncmp(line, "flash ", 6) == 0) {
        debugFlashCommand(serial, line + 6);
      }
      else if (strncmp(line, "datalog", 7) == 0) {
        // 强制黑匣子记录 N 秒（异步：置位调试标志后返回，超时自动复位）。
        // 不直接改 isDatalogging（flight_control 每帧覆盖）；handleDataLogging
        // 用独立调试标志判断。验证 Flash 双写链路用。
        int secs;
        if (sscanf(line + 7, "%d", &secs) == 1) {
          secs = constrain(secs, 1, 60);
          s_dbgLogUntil = millis() + (unsigned long)secs * 1000;
          serial.print(F("[DBG] datalog forced ON for "));
          serial.print(secs);
          serial.println(F("s (auto-reset)"));
        } else {
          serial.println(F("[DBG] usage: datalog <secs>"));
        }
      }
      else if (strncmp(line, "wsstatic", 8) == 0) {
        int lvl;
        if (sscanf(line + 8, "%d", &lvl) == 1) {
          serial.print(F("[DBG] wsstatic not supported by WS2812Driver lib"));
          serial.println(F(" (use ws <r> <g> <b> instead)"));
        } else {
          serial.println(F("[DBG] usage: wsstatic <0|1>"));
        }
      }
      else if (strncmp(line, "ver", 3) == 0) {
        serial.println(F("[DBG] TandemVec FCS debug console"));
        serial.print(F("[DBG] F_CPU="));
        serial.println(F_CPU);
      }
      else if (strncmp(line, "exit", 4) == 0) {
        serial.println(F("[DBG] Debug mode OFF. Back to telemetry."));
        dbgMode = false;
      }
      else if (*line == '\0') {
        // 空行忽略
      }
      else {
        serial.print(F("[DBG] unknown: '"));
        serial.print(line);
        serial.println(F("' (type 'help')"));
      }
    }
    else if (*lineLen < maxLen - 1) {
      line[(*lineLen)++] = c;
    }
  }
}

// ========================================================================
// GPIO 诊断命令（调试模式: gpio）— 读 PD15 实时寄存器状态
// ========================================================================
// 注：此函数通过调试控制台追加注册，见 handleDebugConsole 的 "gpio" 分支
void debugGpioDump(HardwareSerial &serial)
{
  // GPIOD 寄存器（H743: AHB4 域）
  GPIO_TypeDef *gpiob = GPIOB; // 使用 GPIO 宏（框架已映射）
  GPIO_TypeDef *gpiod = GPIOD;
  serial.println(F("[DBG] === GPIO PD15 state ==="));
  serial.print(F("[DBG] GPIOD MODER=0x"));
  serial.println(gpiod->MODER, HEX);
  serial.print(F("[DBG] GPIOD ODR=0x"));
  serial.println(gpiod->ODR, HEX);
  serial.print(F("[DBG] GPIOD IDR=0x"));
  serial.println(gpiod->IDR, HEX);
  serial.print(F("[DBG] GPIOD AFR2=0x"));
  serial.println(gpiod->AFR[1], HEX);
  // PD15 位
  serial.print(F("[DBG] PD15 MODER bits="));
  serial.println((gpiod->MODER >> 30) & 0x3, DEC);
  serial.print(F("[DBG] PD15 ODR bit="));
  serial.println((gpiod->ODR >> 15) & 0x1, DEC);
  serial.print(F("[DBG] PD15 IDR bit="));
  serial.println((gpiod->IDR >> 15) & 0x1, DEC);
  serial.print(F("[DBG] PD15 AFR="));
  serial.println((gpiod->AFR[1] >> 28) & 0xF, DEC);   // 0-15, TIM4 需要 2
}

// ========================================================================
// TIM4 + DMA 寄存器诊断（调试模式: tim4）— 验证 DMA 链路是否真的工作
// ========================================================================
void debugTim4Dump(HardwareSerial &serial)
{
  serial.println(F("[DBG] === TIM4 + DMA state ==="));
  serial.print(F("[DBG] PCLK1="));
  serial.println(HAL_RCC_GetPCLK1Freq());
  serial.print(F("[DBG] D2CFGR=0x"));
  serial.println(RCC->D2CFGR, HEX);          // D2PPRE1 字段在 bit4-6
  serial.print(F("[DBG] D2PPRE1 field=0x"));
  serial.println((RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) >> 4, HEX);
  serial.print(F("[DBG] SystemCoreClock="));
  serial.println(SystemCoreClock);
  serial.print(F("[DBG] TIM4 CR1=0x"));
  serial.println(TIM4->CR1, HEX);
  serial.print(F("[DBG] TIM4 DIER=0x"));
  serial.println(TIM4->DIER, HEX);           // bit8 UDE 应=1
  serial.print(F("[DBG] TIM4 ARR=0x"));
  serial.println(TIM4->ARR, HEX);
  serial.print(F("[DBG] TIM4 CNT=0x"));
  serial.println(TIM4->CNT, HEX);            // 应在计数
  serial.print(F("[DBG] TIM4 CCR4=0x"));
  serial.println(TIM4->CCR4, HEX);           // DMA 写入目标
  serial.print(F("[DBG] TIM4 CCMR2=0x"));
  serial.println(TIM4->CCMR2, HEX);
  serial.print(F("[DBG] TIM4 CCER=0x"));
  serial.println(TIM4->CCER, HEX);           // bit12 CC4E 应=1
  serial.print(F("[DBG] DMA1_S6 NDTR=0x"));
  serial.println((DMA1_Stream6->NDTR), HEX); // 剩余传输数 (传完=0)
  serial.print(F("[DBG] DMA1_S6 CR=0x"));
  serial.println((DMA1_Stream6->CR), HEX);
  serial.print(F("[DBG] DMAMUX1_Ch6 CCR=0x"));
  serial.println((DMAMUX1_Channel6->CCR), HEX);  // 应为 32 (TIM4_UP)
  // 动态观察：读两次 CNT 和 CCR4
  uint32_t cnt1 = TIM4->CNT;
  uint32_t ccr1 = TIM4->CCR4;
  serial.print(F("[DBG] snapshot1 CNT=0x"));
  serial.println(cnt1, HEX);
  serial.print(F("[DBG] snapshot1 CCR4=0x"));
  serial.println(ccr1, HEX);
  delay(10);
  serial.print(F("[DBG] snapshot2 CNT=0x"));
  serial.println(TIM4->CNT, HEX);
  serial.print(F("[DBG] snapshot2 CCR4=0x"));
  serial.println(TIM4->CCR4, HEX);

  // ---- DMA buffer 内容（库内部诊断）----
  serial.print(F("[DBG] dmaBufAddr=0x"));
  serial.println(ws2812Led.dmaBufAddr(), HEX);
  serial.print(F("[DBG] dmaBufLen="));
  serial.println(ws2812Led.dmaBufLen());
  serial.print(F("[DBG] dmaDstAddr=0x"));
  serial.println(ws2812Led.dmaDstAddr(), HEX);
  // 前 8 个字 + 复位后第一个数据字
  for (uint32_t i = 0; i < 8; i++) {
    serial.print(F("[DBG] buf["));
    serial.print(i);
    serial.print(F("]=0x"));
    serial.println(ws2812Led.dmaBufAt(i), HEX);
  }
  serial.print(F("[DBG] buf[40]=0x"));
  serial.println(ws2812Led.dmaBufAt(40), HEX);
  serial.print(F("[DBG] buf[41]=0x"));
  serial.println(ws2812Led.dmaBufAt(41), HEX);
}

// ========================================================================
// Flash 调试命令（调试模式: flash <sub>）
//   flash id          读 JEDEC ID 验证芯片
//   flash erase       整片擦除（~2s 阻塞）
//   flash stat        游标/缓冲占用/坏块/掉帧统计
//   flash dump <page> 读回一页 hexdump（前 32 字节）
//   flash test        写读回一致性测试（写 5 帧到当前游标，读回验证）
// ========================================================================
void debugFlashCommand(HardwareSerial &serial, char *args)
{
  if (strncmp(args, "id", 2) == 0) {
    serial.print(F("[DBG] Flash JEDEC ID: 0x"));
    serial.print(flash.manufacturerId(), HEX);
    serial.print(F(" 0x"));
    serial.print(flash.deviceId(), HEX);
    serial.print(F(" 0x"));
    serial.println(flash.deviceId2(), HEX);
    serial.print(F("[DBG] present="));
    serial.println(flash.isPresent() ? F("YES") : F("NO"));
  }
  else if (strncmp(args, "erase", 5) == 0) {
    serial.println(F("[DBG] Erasing whole device... (~2s)"));
    bool ok = flashLog.format();
    serial.print(F("[DBG] erase done ok="));
    serial.println(ok ? F("YES") : F("NO"));
  }
  else if (strncmp(args, "stat", 4) == 0) {
    serial.print(F("[DBG] present="));
    serial.println(flash.isPresent() ? F("YES") : F("NO"));
    serial.print(F("[DBG] cursorPage="));
    serial.println(flashLog.cursorPage());
    serial.print(F("[DBG] segment="));
    serial.println(flashLog.segment());
    serial.print(F("[DBG] buffered="));
    serial.println(flashLog.buffered());
    serial.print(F("[DBG] written="));
    serial.println(flashLog.written());
    serial.print(F("[DBG] pagesWritten="));
    serial.println(flashLog.pagesWritten());
    serial.print(F("[DBG] dropped="));
    serial.println(flashLog.dropped());
    serial.print(F("[DBG] badBlocks="));
    serial.println(flashLog.badBlocks());
    serial.print(F("[DBG] busy="));
    serial.println(flashLog.busy() ? F("YES") : F("NO"));
    serial.print(F("[DBG] serviceCount="));
    serial.println(s_flashServiceCount);
    serial.print(F("[DBG] pushCount="));
    serial.println(s_flashPushCount);
    serial.print(F("[DBG] totalPages="));
    serial.println(W25N01GV_TOTAL_PAGES);
  }
  else if (strncmp(args, "dump", 4) == 0) {
    uint32_t page;
    if (sscanf(args + 4, "%lu", &page) == 1 && page < W25N01GV_TOTAL_PAGES) {
      uint8_t buf[64];
      bool ok = flashLog.debugReadPage(page, buf, sizeof(buf));
      serial.print(F("[DBG] page "));
      serial.print(page);
      serial.print(F(" read ok="));
      serial.println(ok ? F("YES") : F("NO"));
      serial.print(F("[DBG] ecc="));
      serial.println(flash.lastEccStatus());
      // hexdump first 32 bytes
      for (uint8_t i = 0; i < 32; i++) {
        if (buf[i] < 16) serial.print(F("0"));
        serial.print(buf[i], HEX);
        serial.print(F(" "));
        if (i % 16 == 15) serial.println();
      }
    } else {
      serial.println(F("[DBG] usage: flash dump <page>"));
    }
  }
  else if (strncmp(args, "test", 4) == 0) {
    serial.println(F("[DBG] write-readback test..."));
    serial.flush();
    // 写 5 帧（用递增 payload）到当前游标
    uint8_t payload[W25N01GV_LOG_PAYLOAD];
    for (int i = 0; i < 5; i++) {
      for (uint16_t j = 0; j < sizeof(payload); j++) payload[j] = (uint8_t)(i * 31 + j);
      serial.print(F("[DBG] push#"));
      serial.println(i);
      serial.flush();
      flashLog.logPush(payload);
      serial.print(F("[DBG] pushed ok, buffered="));
      serial.println(flashLog.buffered());
      serial.flush();
    }
    serial.print(F("[DBG] pushed 5 frames, buffered="));
    serial.println(flashLog.buffered());
    serial.flush();
    // 手动 service 一次写光（测试用，正常由 handleFlashService 后台写）
    for (int s = 0; s < 5; s++) {
      serial.print(F("[DBG] service#"));
      serial.print(s);
      serial.flush();
      flashLog.logService();
      serial.print(F(" pages="));
      serial.print(flashLog.pagesWritten());
      serial.print(F(" buffered="));
      serial.print(flashLog.buffered());
      serial.print(F(" progErr="));
      serial.println(flash.lastProgError());
      serial.flush();
    }
    serial.print(F("[DBG] after service: written="));
    serial.println(flashLog.written());
    serial.print(F("[DBG] buffered="));
    serial.println(flashLog.buffered());
    // 读回最后写入页验证 magic（v2: 页内多帧，检查页首帧）
    if (flashLog.written() > 0) {
      uint32_t page = flashLog.cursorPage() - 1;
      static uint8_t buf[W25N01GV_PAGE_SIZE];   // static: 避免栈溢出
      bool ok = flashLog.debugReadPage(page, buf, sizeof(buf));
      serial.print(F("[DBG] readback page="));
      serial.print(page);
      serial.print(F(" ok="));
      serial.println(ok ? F("YES") : F("NO"));
      if (ok) {
        serial.print(F("[DBG] magic=0x"));
        serial.print(buf[0], HEX);
        serial.print(buf[1], HEX);
        serial.print(F(" type="));
        serial.println(buf[2], HEX);   // 0x49=I 0x50=P
      }
    }
  }
  else if (strncmp(args, "export", 6) == 0) {
    // 批量导出页数据：flash export <startPage> <count>
    // 每页输出完整 2048B 原始数据（v2 页内打包多帧 I/P 混合），
    // python 工具按 magic 同步切帧解析。输出为原始字节。
    uint32_t startPage, count;
    if (sscanf(args + 6, "%lu %lu", &startPage, &count) == 2 &&
        startPage < W25N01GV_TOTAL_PAGES && count > 0) {
      if (count > 1000) count = 1000;
      serial.print(F("[DBG] export start="));
      serial.print(startPage);
      serial.print(F(" count="));
      serial.println(count);
      serial.flush();   // 让提示先发出，再输出二进制

      static uint8_t buf[W25N01GV_PAGE_SIZE];   // static: 避免栈溢出
      for (uint32_t i = 0; i < count; i++) {
        uint32_t page = startPage + i;
        if (flashLog.debugReadPage(page, buf, W25N01GV_PAGE_SIZE)) {
          serial.write(buf, W25N01GV_PAGE_SIZE);   // 整页原始字节
        }
      }
      serial.println();
    } else {
      serial.println(F("[DBG] usage: flash export <startPage> <count>"));
    }
  }
  else {
    serial.println(F("[DBG] flash subcommands: id | erase | stat | dump <page> | test | export <page> <count>"));
  }
}
