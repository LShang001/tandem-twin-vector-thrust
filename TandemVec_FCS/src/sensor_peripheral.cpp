#include "sensor_peripheral.h"
#include "math_utils.h"
#include "Dps310FifoPolicy.h"
#include "MAVLink.h" // MAVLink APM 光流传感器自动识别（UART7 发心跳 + 解 OPTICAL_FLOW/DISTANCE_SENSOR）

#include <cmath>

#ifndef BFS_DPS310_TEMP_MEAS_RATE
#define BFS_DPS310_TEMP_MEAS_RATE DPS__MEASUREMENT_RATE_1
#endif

#ifndef BFS_DPS310_PRESS_MEAS_RATE
#define BFS_DPS310_PRESS_MEAS_RATE DPS__MEASUREMENT_RATE_128
#endif

#ifndef BFS_DPS310_TEMP_OSR
#define BFS_DPS310_TEMP_OSR DPS__OVERSAMPLING_RATE_4
#endif

#ifndef BFS_DPS310_PRESS_OSR
#define BFS_DPS310_PRESS_OSR DPS__OVERSAMPLING_RATE_2
#endif

#ifdef BFS_PROFILE_TO_ANOCOM_SERIAL
#define BFS_PROFILE_SERIAL Serial6
#else
#define BFS_PROFILE_SERIAL Serial8
#endif

namespace
{
  constexpr int DPS310_INIT_MAX_ATTEMPTS = 150;         // 10ms一次, 约1.5s内等待首次数据
  constexpr int DPS310_INIT_DATA_TIMEOUT_ERROR = -1002; // 自定义错误码：初始化阶段等待气压数据超时
  int16_t dps310_last_read_status = DPS__FAIL_UNFINISHED;

  int resetDps310OnBus()
  {
    // 固件上传只复位 MCU, 不一定给 DPS310 断电。初始化前软复位传感器,
    // 清除可能残留的命令测量状态, 再让驱动重新读取系数并配置连续模式。
    IIC2.beginTransmission(0x77);
    IIC2.write(0x0C);
    IIC2.write(0x09);
    const int ret = IIC2.endTransmission();
    delay(50);
    return ret;
  }

#ifdef BFS_DPS310_DIAG
  struct Dps310Diag
  {
    uint32_t ok = 0;
    uint32_t unfinished = 0;
    uint32_t overflow = 0;
    uint32_t other_fail = 0;
    uint32_t flush_overflow = 0;
    uint32_t flush_unknown = 0;
    uint32_t sum_us = 0;
    uint32_t max_us = 0;
    uint32_t last_success_ms = 0;
    uint32_t sample_count = 0;
    float pressure_mean_pa = 0.0f;
    float pressure_m2_pa = 0.0f;
    float raw_alt_mean_m = 0.0f;
    float raw_alt_m2_m = 0.0f;
    int16_t last_status = 0;
  };

  Dps310Diag dps310_diag;

  void printDps310InitDiag(int16_t start_status, int init_attempts, int16_t last_read_status)
  {
    BFS_PROFILE_SERIAL.print("[DPS310_INIT] start=");
    BFS_PROFILE_SERIAL.print(start_status);
    BFS_PROFILE_SERIAL.print(" attempts=");
    BFS_PROFILE_SERIAL.print(init_attempts);
    BFS_PROFILE_SERIAL.print(" last=");
    BFS_PROFILE_SERIAL.print(last_read_status);
    BFS_PROFILE_SERIAL.print(" temp_mr=");
    BFS_PROFILE_SERIAL.print(BFS_DPS310_TEMP_MEAS_RATE);
    BFS_PROFILE_SERIAL.print(" temp_osr=");
    BFS_PROFILE_SERIAL.print(BFS_DPS310_TEMP_OSR);
    BFS_PROFILE_SERIAL.print(" prs_mr=");
    BFS_PROFILE_SERIAL.print(BFS_DPS310_PRESS_MEAS_RATE);
    BFS_PROFILE_SERIAL.print(" prs_osr=");
    BFS_PROFILE_SERIAL.print(BFS_DPS310_PRESS_OSR);
    BFS_PROFILE_SERIAL.println();
  }

  void printDps310RegisterSnapshot(const char *stage)
  {
    BFS_PROFILE_SERIAL.print("[DPS310_REG] stage=");
    BFS_PROFILE_SERIAL.print(stage);
    BFS_PROFILE_SERIAL.print(" prs_cfg=");
    BFS_PROFILE_SERIAL.print(Dps310Sensor.readDebugRegister(0x06));
    BFS_PROFILE_SERIAL.print(" tmp_cfg=");
    BFS_PROFILE_SERIAL.print(Dps310Sensor.readDebugRegister(0x07));
    BFS_PROFILE_SERIAL.print(" meas_cfg=");
    BFS_PROFILE_SERIAL.print(Dps310Sensor.readDebugRegister(0x08));
    BFS_PROFILE_SERIAL.print(" cfg_reg=");
    BFS_PROFILE_SERIAL.print(Dps310Sensor.readDebugRegister(0x09));
    BFS_PROFILE_SERIAL.print(" int_sts=");
    BFS_PROFILE_SERIAL.print(Dps310Sensor.readDebugRegister(0x0A));
    BFS_PROFILE_SERIAL.print(" fifo_sts=");
    BFS_PROFILE_SERIAL.print(Dps310Sensor.readDebugRegister(0x0B));
    BFS_PROFILE_SERIAL.print(" fifo_fl=");
    BFS_PROFILE_SERIAL.print(Dps310Sensor.readDebugRegister(0x0C));
    BFS_PROFILE_SERIAL.println();
  }

  void recordDps310Read(int16_t status, uint32_t elapsed_us)
  {
    dps310_diag.last_status = status;
    dps310_diag.sum_us += elapsed_us;
    if (elapsed_us > dps310_diag.max_us)
    {
      dps310_diag.max_us = elapsed_us;
    }

    if (status == DPS__SUCCEEDED)
    {
      dps310_diag.ok++;
      dps310_diag.last_success_ms = millis();
    }
    else if (status == DPS__FAIL_UNFINISHED)
    {
      dps310_diag.unfinished++;
    }
    else if (status == DPS__FAIL_OVERFLOW)
    {
      dps310_diag.overflow++;
    }
    else
    {
      dps310_diag.other_fail++;
    }
  }

  void recordDps310Sample(float pressure_pa, float raw_alt_m)
  {
    dps310_diag.sample_count++;

    const float pressure_delta = pressure_pa - dps310_diag.pressure_mean_pa;
    dps310_diag.pressure_mean_pa += pressure_delta / static_cast<float>(dps310_diag.sample_count);
    const float pressure_delta2 = pressure_pa - dps310_diag.pressure_mean_pa;
    dps310_diag.pressure_m2_pa += pressure_delta * pressure_delta2;

    const float raw_alt_delta = raw_alt_m - dps310_diag.raw_alt_mean_m;
    dps310_diag.raw_alt_mean_m += raw_alt_delta / static_cast<float>(dps310_diag.sample_count);
    const float raw_alt_delta2 = raw_alt_m - dps310_diag.raw_alt_mean_m;
    dps310_diag.raw_alt_m2_m += raw_alt_delta * raw_alt_delta2;
  }

  void printDps310DiagIfDue()
  {
    static uint32_t last_print_ms = 0;
    const uint32_t now_ms = millis();
    if (now_ms - last_print_ms < 1000)
    {
      return;
    }
    last_print_ms = now_ms;

    const uint32_t total = dps310_diag.ok + dps310_diag.unfinished +
                           dps310_diag.overflow + dps310_diag.other_fail;
    const uint32_t avg_us = total > 0 ? dps310_diag.sum_us / total : 0;
    const uint32_t stale_ms = dps310_diag.last_success_ms > 0
                                  ? now_ms - dps310_diag.last_success_ms
                                  : now_ms;
    const float pressure_std_pa = dps310_diag.sample_count > 1
                                      ? sqrtf(dps310_diag.pressure_m2_pa /
                                              static_cast<float>(dps310_diag.sample_count - 1U))
                                      : 0.0f;
    const float raw_alt_std_m = dps310_diag.sample_count > 1
                                    ? sqrtf(dps310_diag.raw_alt_m2_m /
                                            static_cast<float>(dps310_diag.sample_count - 1U))
                                    : 0.0f;

    BFS_PROFILE_SERIAL.print("[DPS310] ok=");
    BFS_PROFILE_SERIAL.print(dps310_diag.ok);
    BFS_PROFILE_SERIAL.print(" unfin=");
    BFS_PROFILE_SERIAL.print(dps310_diag.unfinished);
    BFS_PROFILE_SERIAL.print(" ovf=");
    BFS_PROFILE_SERIAL.print(dps310_diag.overflow);
    BFS_PROFILE_SERIAL.print(" other=");
    BFS_PROFILE_SERIAL.print(dps310_diag.other_fail);
    BFS_PROFILE_SERIAL.print(" flush_ovf=");
    BFS_PROFILE_SERIAL.print(dps310_diag.flush_overflow);
    BFS_PROFILE_SERIAL.print(" flush_unk=");
    BFS_PROFILE_SERIAL.print(dps310_diag.flush_unknown);
    BFS_PROFILE_SERIAL.print(" avg_us=");
    BFS_PROFILE_SERIAL.print(avg_us);
    BFS_PROFILE_SERIAL.print(" max_us=");
    BFS_PROFILE_SERIAL.print(dps310_diag.max_us);
    BFS_PROFILE_SERIAL.print(" stale_ms=");
    BFS_PROFILE_SERIAL.print(stale_ms);
    BFS_PROFILE_SERIAL.print(" status=");
    BFS_PROFILE_SERIAL.print(dps310_diag.last_status);
    BFS_PROFILE_SERIAL.print(" pstd=");
    BFS_PROFILE_SERIAL.print(pressure_std_pa, 3);
    BFS_PROFILE_SERIAL.print(" astd=");
    BFS_PROFILE_SERIAL.print(raw_alt_std_m, 4);
    BFS_PROFILE_SERIAL.print(" raw_alt=");
    BFS_PROFILE_SERIAL.print(baro_altitude_raw, 3);
    BFS_PROFILE_SERIAL.print(" alt=");
    BFS_PROFILE_SERIAL.print(baro_altitude, 3);
    BFS_PROFILE_SERIAL.print(" pressure=");
    BFS_PROFILE_SERIAL.print(pressure, 2);
    BFS_PROFILE_SERIAL.print(" temp=");
    BFS_PROFILE_SERIAL.print(temperature, 2);
#ifdef BFS_EKF_BARO_ALTITUDE_UPDATE
      BFS_PROFILE_SERIAL.print(" b_innov=");
      BFS_PROFILE_SERIAL.print(baro_ekf_innovation_m, 3);
      BFS_PROFILE_SERIAL.print(" b_nis_r=");
      BFS_PROFILE_SERIAL.print(baro_ekf_test_ratio, 2);
      BFS_PROFILE_SERIAL.print(" b_fused=");
      BFS_PROFILE_SERIAL.print(baro_ekf_fused);
#endif
    BFS_PROFILE_SERIAL.println();

    if (dps310_diag.ok == 0 && dps310_diag.last_status == DPS__FAIL_UNFINISHED)
    {
      printDps310RegisterSnapshot("no_pressure");
    }

    dps310_diag = Dps310Diag{};
  }
#endif
}
// =============================================================================
// UART7 光流传感器自动识别（LQS48 / MTF-02P Micolink / MTF-02P AUTO 版 MAVLink APM）
// =============================================================================

namespace
{
  constexpr uint32_t FLOW_DETECT_WINDOW_MS = 400;            // 每轮 LQS48/Micolink 方案扫描时间
  constexpr uint32_t FLOW_DETECT_MAVLINK_WINDOW_MS = 800;    // AUTO 版需要先收到心跳再输出，给更长窗口
  constexpr uint32_t FLOW_DETECT_HEARTBEAT_MS = 100;         // MAVLink 心跳周期
  constexpr uint32_t FLOW_DETECT_STALE_MS = 1500;            // 已识别传感器连续无帧超过该值则重新扫描
  constexpr uint32_t FLOW_LQS48_BAUD_FAST = 921600;          // 现役 LQS48 波特率
  constexpr uint32_t FLOW_LQS48_BAUD_SLOW = 115200;          // 兼容 LQS48 被配置为 115200 的情形
  constexpr uint32_t FLOW_MTF02P_BAUD = 115200;              // MTF-02P 固定 115200

  enum class FlowDetectPhase : uint8_t
  {
    LQS48_921600 = 0,       // LQS48 @ 921600
    MTF_MSP_115200,         // MTF-02P INAV/MSP 模式，AUTO 版上电最常见输出，路径放前
    MTF_MICOLINK_115200,    // MTF-02P Micolink @ 115200（微空助手切换后输出）
    MTF_MAVLINK_115200,     // MTF-02P AUTO 版在收到 ArduPilot 心跳后走 MAVLink 输出
    LQS48_115200,           // LQS48 @ 115200
    MTF_MICOLINK_921600,    // MTF-02P Micolink @ 921600（少见，兜底）
    PHASE_COUNT
  };

  FlowDetectPhase s_detect_phase = FlowDetectPhase::LQS48_921600;
  uint32_t s_phase_start_ms = 0;
  uint32_t s_last_heartbeat_ms = 0;
  uint32_t s_last_detect_log_ms = 0;
  uint32_t s_detect_loop_count = 0;
  uint32_t s_mav_flow_frames = 0;
  uint32_t s_mav_dist_frames = 0;
  uint32_t s_uart_rx_avail_sum = 0;  // 每 tick 在解析前看到的 UART7 可读字节数累计
  uint32_t s_uart_rx_read_sum = 0;   // MAVLink 泵实际读走的字节数累计
  bool s_flow_dbg_pause = false;     // DBG flowdump 期间暂停自动识别，避免抢 RX 字节

  // ---- MSPv2 (INAV/Betaflight) 光流解析状态 ----
  enum class MspFlowState : uint8_t
  {
    WAIT_DOLLAR = 0,
    WAIT_X,
    WAIT_DIR,
    WAIT_FLAG,
    WAIT_CMD_L,
    WAIT_CMD_H,
    WAIT_SIZE_L,
    WAIT_SIZE_H,
    PAYLOAD,
    CHECKSUM,
  };
  MspFlowState s_msp_state = MspFlowState::WAIT_DOLLAR;
  uint8_t s_msp_flag = 0;
  uint8_t s_msp_cmd_h = 0;
  uint8_t s_msp_cmd_l = 0;
  uint8_t s_msp_size_h = 0;
  uint8_t s_msp_size_l = 0;
  uint16_t s_msp_size = 0; // MSPv2 长度字段是 16bit，超 255 的帧必须能正确拒绝
  uint16_t s_msp_payload_idx = 0;
  uint8_t s_msp_payload[32] = {0};
  uint8_t s_msp_crc = 0;
  uint16_t s_msp_msg_id = 0;

  uint32_t s_msp_flow_frame_count = 0;
  uint32_t s_msp_range_frame_count = 0;
  int32_t s_msp_distance_mm = -1;    // 最近一次激光测距结果（负值=无效）
  uint8_t s_msp_range_quality = 0;
  uint32_t s_msp_range_last_ms = 0;  // 最近一次收到测距帧的系统时刻（缓存过期判定）
  uint32_t s_msp_flow_last_ms = 0;
  bool s_msp_has_flow_last = false;

  const char *flowSensorTypeName(FlowSensorType type)
  {
    switch (type)
    {
    case FLOW_SENSOR_LQS48:
      return "LQS48";
    case FLOW_SENSOR_MTF02P_MICOLINK:
      return "MTF02P Micolink";
    case FLOW_SENSOR_MTF02P_MAVLINK_APM:
      return "MTF02P MAVLink-APM";
    case FLOW_SENSOR_MTF02P_MSP:
      return "MTF02P MSP";
    case FLOW_SENSOR_NONE:
    default:
      return "None";
    }
  }

  void flowSensorConfigureUart(uint32_t baud)
  {
    if (flow_sensor_baud == baud && flow_sensor_baud != 0)
    {
      return;
    }

    // STM32duino HardwareSerial 支持运行时 end/begin 重配置波特率。
    // 切换波特率后清空一次接收缓冲，避免上一波特率残留下乱码数据。
    opticalFlowSerial.end();
    opticalFlowSerial.begin(baud);
    flow_sensor_baud = baud;
    while (opticalFlowSerial.available() > 0)
    {
      opticalFlowSerial.read();
    }
  }

  // 前向声明：MSP 有效帧处理中需要调用 flowSensorLock 锁定传感器
  void flowSensorLock(FlowSensorType type, uint32_t baud);

  uint8_t flowSensorMspCrc8Update(uint8_t crc, uint8_t byte)
  {
    crc ^= byte;
    for (uint8_t i = 0; i < 8; i++)
    {
      if (crc & 0x80)
      {
        crc = static_cast<uint8_t>((crc << 1) ^ 0xD5);
      }
      else
      {
        crc = static_cast<uint8_t>(crc << 1);
      }
    }
    return crc;
  }

  void flowSensorResetMspParser()
  {
    s_msp_state = MspFlowState::WAIT_DOLLAR;
    s_msp_flag = 0;
    s_msp_cmd_l = 0;
    s_msp_cmd_h = 0;
    s_msp_size_l = 0;
    s_msp_size_h = 0;
    s_msp_size = 0;
    s_msp_payload_idx = 0;
    s_msp_crc = 0;
    s_msp_msg_id = 0;
  }

  // MSP2_SENSOR_RANGEFINDER_LIDARMT = 0x1F01, payload: quality(1) + distanceMm(int32)
  void flowSensorApplyMspRangePayload()
  {
    if (s_msp_msg_id != 0x1F01 || s_msp_size != 5)
    {
      return;
    }

    s_msp_range_quality = s_msp_payload[0];
    int32_t distance_mm_raw = 0;
    memcpy(&distance_mm_raw, &s_msp_payload[1], sizeof(distance_mm_raw));
    s_msp_distance_mm = (distance_mm_raw >= 0) ? distance_mm_raw : -1;
    s_msp_range_last_ms = millis();
    s_msp_range_frame_count++;

    // 测距帧是测距通道的存活证据（链路重识别判定用双通道取新），
    // 不刷新 last_update_ms——那是光流（速度）通道专属新鲜度，
    // 否则光流帧停流后冻结速度会被 EKF 融合当新测量反复注入。
    flow_data.last_range_update_ms = s_msp_range_last_ms;

    if (active_flow_sensor != FLOW_SENSOR_MTF02P_MSP)
    {
      flowSensorLock(FLOW_SENSOR_MTF02P_MSP, FLOW_MTF02P_BAUD);
    }
  }

  // MSP2_SENSOR_OPTICALFLOW_MT = 0x1F02, payload: quality(1) + motionX(int32) + motionY(int32)
  void flowSensorApplyMspFlowPayload()
  {
    if (s_msp_msg_id != 0x1F02 || s_msp_size != 9)
    {
      return;
    }

    uint8_t quality = s_msp_payload[0];
    int32_t motion_x = 0;
    int32_t motion_y = 0;
    memcpy(&motion_x, &s_msp_payload[1], sizeof(motion_x));
    memcpy(&motion_y, &s_msp_payload[5], sizeof(motion_y));

    const uint32_t now_ms = millis();
    float dt_s = 0.02f; // 首帧默认 50Hz 周期
    if (s_msp_has_flow_last)
    {
      dt_s = static_cast<float>(now_ms - s_msp_flow_last_ms) / 1000.0f;
    }
    s_msp_flow_last_ms = now_ms;
    s_msp_has_flow_last = true;
    s_msp_flow_frame_count++;

    if (active_flow_sensor != FLOW_SENSOR_MTF02P_MSP)
    {
      flowSensorLock(FLOW_SENSOR_MTF02P_MSP, FLOW_MTF02P_BAUD);
    }

    // 测距缓存过期：激光帧停流后不得永远沿用最后一次斜距（高度变化会让
    // 速度刻度全错），超过光流陈旧窗后退回 estimated_height 反投影路径。
    if (s_msp_distance_mm >= 0 &&
        (now_ms - s_msp_range_last_ms) > kFlowDataStaleTimeoutMs)
    {
      s_msp_distance_mm = -1;
    }

    if (dt_s > 0.001f && dt_s < 1.0f)
    {
      // 同 Betaflight mtOpticalflowReceiveNewData：motion / 200 / dt 得到 rad/s
      const float flow_gyro_x = static_cast<float>(motion_x) / 200.0f / dt_s;
      const float flow_gyro_y = static_cast<float>(motion_y) / 200.0f / dt_s;

      // 距离源优先取 MTF 自带激光测距（沿光轴斜距），不可用时退回
      // estimated_height/cos(tilt) 反投影，与 LQS48 路径保持同一物理口径。
      float slant_m = 0.0f;
      if (s_msp_distance_mm > 0)
      {
        slant_m = static_cast<float>(s_msp_distance_mm) / 1000.0f;
      }
      else
      {
        float qx = AHRS_Packet.Qx;
        float qy = AHRS_Packet.Qy;
        float cos_tilt = 1.0f - 2.0f * (qx * qx + qy * qy);
        cos_tilt = fabsf(cos_tilt);
        if (cos_tilt < 0.1f)
        {
          cos_tilt = 0.1f;
        }
        if (estimated_height > 0.08f)
        {
          slant_m = estimated_height / cos_tilt;
        }
      }

      // 高度/测距有效性独立更新：测距数据有效就记录，不因 8cm 光流最小工作距离而丢弃。
      float qx = AHRS_Packet.Qx;
      float qy = AHRS_Packet.Qy;
      float cos_tilt = 1.0f - 2.0f * (qx * qx + qy * qy);
      if (s_msp_distance_mm > 0)
      {
        float vertical_m = slant_m * fabsf(cos_tilt);
        flow_data.distance_m = flowDistanceMedianFilter.filter(vertical_m);
        flow_data.distance_m = flowDistanceFilter.filter(flow_data.distance_m);
        flow_data.is_range_valid = true;
        flow_data.range_quality = s_msp_range_quality;
        // 距离来自测距帧缓存：新鲜度沿用测距帧时刻，不按本光流帧刷新。
        flow_data.last_range_update_ms = s_msp_range_last_ms;
      }
      else
      {
        flow_data.is_range_valid = false;
      }

      if (slant_m > 0.08f && slant_m <= 6.0f)
      {
        const float v_total_x = flow_gyro_x * slant_m;
        const float v_total_y = flow_gyro_y * slant_m;

        const float imu_omega_x = current_omega_dps_body_filtered.x * DEG_TO_RAD;
        const float imu_omega_y = current_omega_dps_body_filtered.y * DEG_TO_RAD;
        const float v_rot_x = slant_m * imu_omega_y;
        const float v_rot_y = -slant_m * imu_omega_x;

        const float v_trans_x = v_total_x - v_rot_x;
        const float v_trans_y = v_total_y - v_rot_y;

        flow_data.velocity_x_mps = flowVelXFilter.filter(v_trans_x);
        flow_data.velocity_y_mps = flowVelYFilter.filter(v_trans_y);
      }
      else
      {
        flow_data.velocity_x_mps = 0.0f;
        flow_data.velocity_y_mps = 0.0f;
      }

      // is_flow_valid 语义 = “当前速度可用于导航融合”。
      // 高度低于 8cm 光流工作下限、测距无效、或 quality==0 时，宁可输出零速，
      // 也不能让 EKF 光流融合把“无有效速度”误当“零速观测”。
      const bool velocity_usable = (quality > 0 && slant_m > 0.08f && slant_m <= 6.0f);
      flow_data.is_flow_valid = velocity_usable;
      if (!velocity_usable)
      {
        flow_data.velocity_x_mps = 0.0f;
        flow_data.velocity_y_mps = 0.0f;
      }
      flow_data.flow_quality = quality;
    }
    else
    {
      flow_data.velocity_x_mps = 0.0f;
      flow_data.velocity_y_mps = 0.0f;
      flow_data.is_flow_valid = false;
      flow_data.flow_quality = 0;
    }

    flow_data.timestamp_ms = now_ms;
    flow_data.last_update_ms = now_ms;
  }

  void flowSensorFeedMspByte(uint8_t byte)
  {
    switch (s_msp_state)
    {
    case MspFlowState::WAIT_DOLLAR:
      if (byte == 0x24)
      {
        s_msp_state = MspFlowState::WAIT_X;
      }
      break;

    case MspFlowState::WAIT_X:
      if (byte == 0x58)
      {
        s_msp_state = MspFlowState::WAIT_DIR;
      }
      else
      {
        s_msp_state = MspFlowState::WAIT_DOLLAR;
      }
      break;

    case MspFlowState::WAIT_DIR:
      if (byte == 0x3C || byte == 0x3E)
      {
        s_msp_state = MspFlowState::WAIT_FLAG;
        s_msp_crc = 0;
      }
      else
      {
        s_msp_state = MspFlowState::WAIT_DOLLAR;
      }
      break;

    case MspFlowState::WAIT_FLAG:
      s_msp_flag = byte;
      s_msp_crc = flowSensorMspCrc8Update(s_msp_crc, byte);
      s_msp_state = MspFlowState::WAIT_CMD_L;
      break;

    case MspFlowState::WAIT_CMD_L:
      s_msp_cmd_l = byte;
      s_msp_crc = flowSensorMspCrc8Update(s_msp_crc, byte);
      s_msp_state = MspFlowState::WAIT_CMD_H;
      break;

    case MspFlowState::WAIT_CMD_H:
      s_msp_cmd_h = byte;
      s_msp_crc = flowSensorMspCrc8Update(s_msp_crc, byte);
      s_msp_state = MspFlowState::WAIT_SIZE_L;
      break;

    case MspFlowState::WAIT_SIZE_L:
      s_msp_size_l = byte;
      s_msp_crc = flowSensorMspCrc8Update(s_msp_crc, byte);
      s_msp_state = MspFlowState::WAIT_SIZE_H;
      break;

    case MspFlowState::WAIT_SIZE_H:
      s_msp_size_h = byte;
      s_msp_crc = flowSensorMspCrc8Update(s_msp_crc, byte);
      s_msp_size = static_cast<uint16_t>(s_msp_size_l | (static_cast<uint16_t>(s_msp_size_h) << 8));
      s_msp_msg_id = static_cast<uint16_t>(s_msp_cmd_l | (static_cast<uint16_t>(s_msp_cmd_h) << 8));
      if (s_msp_size > sizeof(s_msp_payload))
      {
        s_msp_state = MspFlowState::WAIT_DOLLAR;
      }
      else if (s_msp_size == 0)
      {
        s_msp_state = MspFlowState::CHECKSUM;
      }
      else
      {
        s_msp_payload_idx = 0;
        s_msp_state = MspFlowState::PAYLOAD;
      }
      break;

    case MspFlowState::PAYLOAD:
      s_msp_payload[s_msp_payload_idx++] = byte;
      s_msp_crc = flowSensorMspCrc8Update(s_msp_crc, byte);
      if (s_msp_payload_idx >= s_msp_size)
      {
        s_msp_state = MspFlowState::CHECKSUM;
      }
      break;

    case MspFlowState::CHECKSUM:
      if (s_msp_crc == byte)
      {
        if (s_msp_msg_id == 0x1F01)
        {
          flowSensorApplyMspRangePayload();
        }
        else if (s_msp_msg_id == 0x1F02)
        {
          flowSensorApplyMspFlowPayload();
        }
      }
      s_msp_state = MspFlowState::WAIT_DOLLAR;
      break;

    default:
      flowSensorResetMspParser();
      break;
    }
  }

  void flowSensorPumpMsp()
  {
    while (opticalFlowSerial.available() > 0)
    {
      flowSensorFeedMspByte(opticalFlowSerial.read());
    }
  }

  void flowSensorLock(FlowSensorType type, uint32_t baud)
  {
    flowSensorConfigureUart(baud);

    active_flow_sensor = type;
    flow_data.is_flow_valid = false;
    flow_data.is_range_valid = false;
    flow_data.velocity_x_mps = 0.0f;
    flow_data.velocity_y_mps = 0.0f;
    flow_data.distance_m = 0.0f;
    flow_data.flow_quality = 0;
    flow_data.range_quality = 0;
    flow_data.last_update_ms = millis();
    flow_data.last_range_update_ms = millis();

    flowDistanceFilter.initialize(0.0f);
    flowVelXFilter.initialize(0.0f);
    flowVelYFilter.initialize(0.0f);

    Serial8.print("[OpticalFlow] locked: ");
    Serial8.print(flowSensorTypeName(type));
    Serial8.print(" @ ");
    Serial8.print(baud);
    Serial8.println();
  }

  void flowSensorSendMavlinkHeartbeat(uint32_t now_ms)
  {
    if (now_ms - s_last_heartbeat_ms < FLOW_DETECT_HEARTBEAT_MS)
    {
      return;
    }
    s_last_heartbeat_ms = now_ms;

    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg,
                               MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA,
                               MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, 0, MAV_STATE_ACTIVE);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    if (opticalFlowSerial.availableForWrite() >= len)
    {
      opticalFlowSerial.write(buf, len);
    }
  }

  // MTF-02P 在 AUTO/APM 输出模式下，直接给 m/s 级光流速度（已做角速度补偿），
  // 因此无需再走 Micolink 路径的速度/旋转补偿流程。
  void flowSensorDecodeMavlinkMessage(const mavlink_message_t &msg)
  {
    if (msg.msgid == MAVLINK_MSG_ID_OPTICAL_FLOW)
    {
      mavlink_optical_flow_t flow;
      mavlink_msg_optical_flow_decode(&msg, &flow);

      if (active_flow_sensor != FLOW_SENSOR_MTF02P_MAVLINK_APM)
      {
        flowSensorLock(FLOW_SENSOR_MTF02P_MAVLINK_APM, FLOW_MTF02P_BAUD);
      }

      if (flow.quality > 0)
      {
        flow_data.velocity_x_mps = flowVelXFilter.filter(flow.flow_comp_m_x);
        flow_data.velocity_y_mps = flowVelYFilter.filter(flow.flow_comp_m_y);
        flow_data.is_flow_valid = true;
        flow_data.flow_quality = flow.quality;
      }
      else
      {
        flow_data.velocity_x_mps = 0.0f;
        flow_data.velocity_y_mps = 0.0f;
        flow_data.is_flow_valid = false;
        flow_data.flow_quality = 0;
      }

      if (flow.ground_distance > 0.0f)
      {
        float qx = AHRS_Packet.Qx;
        float qy = AHRS_Packet.Qy;
        float cos_tilt = 1.0f - 2.0f * (qx * qx + qy * qy);
        float vertical_m = flow.ground_distance * fabsf(cos_tilt);
        flow_data.distance_m = flowDistanceMedianFilter.filter(vertical_m);
        flow_data.distance_m = flowDistanceFilter.filter(flow_data.distance_m);
        flow_data.is_range_valid = true;
        flow_data.range_quality = flow.quality;
      }

      flow_data.timestamp_ms = millis();
      flow_data.last_update_ms = millis();
      s_mav_flow_frames++;
    }
    else if (msg.msgid == MAVLINK_MSG_ID_DISTANCE_SENSOR)
    {
      mavlink_distance_sensor_t dist;
      mavlink_msg_distance_sensor_decode(&msg, &dist);

      if (active_flow_sensor != FLOW_SENSOR_MTF02P_MAVLINK_APM)
      {
        flowSensorLock(FLOW_SENSOR_MTF02P_MAVLINK_APM, FLOW_MTF02P_BAUD);
      }

      if (dist.current_distance > 0 && dist.current_distance != 65535U)
      {
        float slant_m = static_cast<float>(dist.current_distance) / 100.0f;
        float qx = AHRS_Packet.Qx;
        float qy = AHRS_Packet.Qy;
        float cos_tilt = 1.0f - 2.0f * (qx * qx + qy * qy);
        float vertical_m = slant_m * fabsf(cos_tilt);
        flow_data.distance_m = flowDistanceMedianFilter.filter(vertical_m);
        flow_data.distance_m = flowDistanceFilter.filter(flow_data.distance_m);
        flow_data.is_range_valid = true;
        flow_data.range_quality = dist.signal_quality;
      }

      flow_data.last_update_ms = millis();
      s_mav_dist_frames++;
    }
  }

  void flowSensorPumpMavlink()
  {
    while (opticalFlowSerial.available() > 0)
    {
      uint8_t byte = opticalFlowSerial.read();
      s_uart_rx_read_sum++;
      mavlink_message_t msg;
      mavlink_status_t status;
      if (mavlink_parse_char(MAVLINK_COMM_2, byte, &msg, &status) == MAVLINK_FRAMING_OK)
      {
        flowSensorDecodeMavlinkMessage(msg);
      }
    }
  }

  void flowSensorRestartDetection()
  {
    active_flow_sensor = FLOW_SENSOR_NONE;
    flow_data.is_flow_valid = false;
    flow_data.is_range_valid = false;
    flow_data.velocity_x_mps = 0.0f;
    flow_data.velocity_y_mps = 0.0f;
    flow_data.distance_m = 0.0f;
    flow_data.flow_quality = 0;
    flow_data.range_quality = 0;
    flow_data.last_update_ms = 0;
    flow_data.last_range_update_ms = 0;

    flowDistanceFilter.initialize(0.0f);
    flowVelXFilter.initialize(0.0f);
    flowVelYFilter.initialize(0.0f);

    // MSP 解析器与测距缓存一并复位，避免跨会话残留（旧 dt 基准/旧斜距）。
    flowSensorResetMspParser();
    s_msp_distance_mm = -1;
    s_msp_range_last_ms = 0;
    s_msp_flow_last_ms = 0;
    s_msp_has_flow_last = false;

    s_detect_phase = FlowDetectPhase::LQS48_921600;
    s_phase_start_ms = 0;
    s_last_heartbeat_ms = 0;

    Serial8.println("[OpticalFlow] re-detect");
  }

  void flowSensorNextPhase(uint32_t now_ms)
  {
    uint8_t next = (static_cast<uint8_t>(s_detect_phase) + 1U) % static_cast<uint8_t>(FlowDetectPhase::PHASE_COUNT);
    s_detect_phase = static_cast<FlowDetectPhase>(next);
    s_phase_start_ms = now_ms;
  }

  void flowSensorLogDetectScan(uint32_t now_ms)
  {
    if (now_ms - s_last_detect_log_ms < 2000)
    {
      return;
    }
    s_last_detect_log_ms = now_ms;
    Serial8.print("[OpticalFlow] scanning loop=");
    Serial8.print(s_detect_loop_count);
    Serial8.print(" phase=");
    Serial8.print(static_cast<uint16_t>(s_detect_phase));
    Serial8.println();
  }

  void flowSensorDetectTick()
  {
    const uint32_t now_ms = millis();

    if (s_phase_start_ms == 0)
    {
      s_phase_start_ms = now_ms;
      flowSensorConfigureUart(FLOW_LQS48_BAUD_FAST);
    }

    s_uart_rx_avail_sum += static_cast<uint32_t>(opticalFlowSerial.available());

    switch (s_detect_phase)
    {
    case FlowDetectPhase::LQS48_921600:
      flowSensorConfigureUart(FLOW_LQS48_BAUD_FAST);
      if (lqs48Sensor.update())
      {
        flowSensorLock(FLOW_SENSOR_LQS48, FLOW_LQS48_BAUD_FAST);
      }
      break;

    case FlowDetectPhase::MTF_MICOLINK_115200:
      flowSensorConfigureUart(FLOW_MTF02P_BAUD);
      flowSensorSendMavlinkHeartbeat(now_ms); // 同时唤醒 AUTO 版模块，为下个 MAVLink 窗口产出数据
      if (mtf02pSensor.update())
      {
        flowSensorLock(FLOW_SENSOR_MTF02P_MICOLINK, FLOW_MTF02P_BAUD);
      }
      break;

    case FlowDetectPhase::MTF_MAVLINK_115200:
      flowSensorConfigureUart(FLOW_MTF02P_BAUD);
      flowSensorSendMavlinkHeartbeat(now_ms);
      flowSensorPumpMavlink(); // 内部收到首帧 OPTICAL_FLOW/DISTANCE_SENSOR 后自动锁定
      break;

    case FlowDetectPhase::MTF_MSP_115200:
      flowSensorConfigureUart(FLOW_MTF02P_BAUD);
      flowSensorPumpMsp(); // MSPv2 1F01/1F02 帧首包后自动锁定
      break;

    case FlowDetectPhase::LQS48_115200:
      flowSensorConfigureUart(FLOW_LQS48_BAUD_SLOW);
      if (lqs48Sensor.update())
      {
        flowSensorLock(FLOW_SENSOR_LQS48, FLOW_LQS48_BAUD_SLOW);
      }
      break;

    case FlowDetectPhase::MTF_MICOLINK_921600:
      flowSensorConfigureUart(FLOW_LQS48_BAUD_FAST);
      if (mtf02pSensor.update())
      {
        flowSensorLock(FLOW_SENSOR_MTF02P_MICOLINK, FLOW_LQS48_BAUD_FAST);
      }
      break;

    case FlowDetectPhase::PHASE_COUNT:
    default:
      s_detect_phase = FlowDetectPhase::LQS48_921600;
      s_phase_start_ms = now_ms;
      break;
    }

    if (active_flow_sensor != FLOW_SENSOR_NONE)
    {
      return;
    }

    const uint32_t window_ms = (s_detect_phase == FlowDetectPhase::MTF_MAVLINK_115200)
                                   ? FLOW_DETECT_MAVLINK_WINDOW_MS
                                   : FLOW_DETECT_WINDOW_MS;
    if (now_ms - s_phase_start_ms >= window_ms)
    {
      flowSensorNextPhase(now_ms);
      s_detect_loop_count++;
      flowSensorLogDetectScan(now_ms);
    }
  }
} // namespace

// ---- 光流自动识别诊断 getter（DBG `flow` 命令调用）----
uint8_t flowSensorGetDetectPhase()
{
  return static_cast<uint8_t>(s_detect_phase);
}

uint32_t flowSensorGetDetectLoopCount()
{
  return s_detect_loop_count;
}

uint32_t flowSensorGetMavFlowFrames()
{
  return s_mav_flow_frames;
}

uint32_t flowSensorGetMavDistFrames()
{
  return s_mav_dist_frames;
}

uint32_t flowSensorGetMspFlowFrames()
{
  return s_msp_flow_frame_count;
}

uint32_t flowSensorGetMspRangeFrames()
{
  return s_msp_range_frame_count;
}

uint32_t flowSensorGetUartRxAvailSum()
{
  return s_uart_rx_avail_sum;
}

uint32_t flowSensorGetUartRxReadSum()
{
  return s_uart_rx_read_sum;
}

// ------------------------------------------------------------------
// DBG `flowdump`：暂停自动识别，独占读取 UART7 指定时长并统计帧头/字节样本。
// 用于没有示波器时判断“模块到底有没有出数据、出的是哪种协议”。
// ------------------------------------------------------------------
void flowSensorDebugDump(HardwareSerial &out, uint32_t duration_ms)
{
  if (duration_ms > 2000)
  {
    duration_ms = 2000;
  }
  if (duration_ms < 50)
  {
    duration_ms = 50;
  }

  // MTF-02P 固定在 115200；flowdump 强制切到该波特率抓样本。
  // 记住原波特率，dump 结束复原——否则已锁定的 921600 传感器要靠 1.5s
  // 无帧超时 + 一轮重扫（最长 ~2.8s）才能恢复，光流空窗合计 4-5s。
  const uint32_t prev_baud = flow_sensor_baud;
  flowSensorConfigureUart(115200);
  s_flow_dbg_pause = true;

  uint32_t total = 0;
  uint32_t cnt_ab = 0, cnt_fa = 0, cnt_ef = 0, cnt_fd = 0, cnt_fe = 0, cnt_55 = 0;
  uint8_t samples[64];
  uint8_t sample_count = 0;

  const uint32_t start_ms = millis();
  while (millis() - start_ms < duration_ms)
  {
    while (opticalFlowSerial.available() > 0)
    {
      const uint8_t byte = opticalFlowSerial.read();
      total++;
      if (byte == 0xAB) cnt_ab++;
      else if (byte == 0xFA) cnt_fa++;
      else if (byte == 0xEF) cnt_ef++;
      else if (byte == 0xFD) cnt_fd++;
      else if (byte == 0xFE) cnt_fe++;
      else if (byte == 0x55) cnt_55++;
      if (sample_count < sizeof(samples))
      {
        samples[sample_count++] = byte;
      }
    }
  }

  s_flow_dbg_pause = false;

  // 复原 dump 前的波特率，并把双通道存活时刻刷新到当前：dump 本身最长 2s 已
  // 超过 1.5s 无帧重识别窗口，不刷新的话锁定的传感器会被立刻误判掉线。
  if (prev_baud != 0 && prev_baud != 115200)
  {
    flowSensorConfigureUart(prev_baud);
  }
  flow_data.last_update_ms = millis();
  flow_data.last_range_update_ms = millis();

  out.println(F("[DBG] flowdump @115200:"));
  out.print(F("  total=")); out.println(total);
  out.print(F("  header counts: AB=")); out.print(cnt_ab);
  out.print(F(" FA=")); out.print(cnt_fa);
  out.print(F(" EF=")); out.print(cnt_ef);
  out.print(F(" FD=")); out.print(cnt_fd);
  out.print(F(" FE=")); out.print(cnt_fe);
  out.print(F(" 55=")); out.println(cnt_55);
  out.print(F("  samples[0.."));
  out.print(sample_count);
  out.print(F("]: "));
  for (uint8_t i = 0; i < sample_count; i++)
  {
    if (samples[i] < 0x10) out.print('0');
    out.print(samples[i], HEX);
    out.print(' ');
  }
  out.println();
}

int initDPS310()
{
  pinMode(DPS310_ADDR_SEL, OUTPUT);
  digitalWrite(DPS310_ADDR_SEL, HIGH); // 设置地址选择为 0x77

  IIC2.begin(); // 自定义 SCL 和 SDA 引脚已在对象创建时指定
  const int reset_status = resetDps310OnBus();
#ifndef BFS_DPS310_DIAG
  (void)reset_status;
#endif

  // 初始化 DPS310
  Dps310Sensor.begin(IIC2, 0x77); // 设置 DPS310 I2C 地址
  //  注意: Dps310Sensor.begin() 是 void 函数，没有返回值
  // DPS310 支持 HS-mode，但当前 STM32duino Wire 仅实现到 1MHz Fm+ timing；
  // 驱动 begin() 会重新 begin 总线，时钟必须在其后设置。
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C2);
  IIC2.setClock(1000000);
#ifdef BFS_DPS310_DIAG
  BFS_PROFILE_SERIAL.print("[DPS310_RESET] status=");
  BFS_PROFILE_SERIAL.println(reset_status);
  printDps310RegisterSnapshot("after_begin");
#endif

#ifndef BFS_DPS310_PRESSURE_ONLY_CONT
  // 温度测量速率 (取值范围 0 到 7)
  // 每秒进行 2^temp_mr 次温度测量
  int16_t temp_mr = BFS_DPS310_TEMP_MEAS_RATE;

  // 温度过采样率 (取值范围 0 到 7)
  // 每个温度测量结果基于 2^temp_osr 次内部温度测量
  // 温度仅用于压力补偿, 不参与高度动态响应; 降低速率可减少 FIFO/I2C 负载。
  int16_t temp_osr = BFS_DPS310_TEMP_OSR;
#endif

  // 压力测量速率 (取值范围 0 到 7)
  // 每秒进行 2^prs_mr 次压力测量
  int16_t prs_mr = BFS_DPS310_PRESS_MEAS_RATE;

  // 压力过采样率 (取值范围 0 到 7)
  // 每个压力测量结果基于 2^prs_osr 次内部压力测量
  // 更高的值可以提高精度
  int16_t prs_osr = BFS_DPS310_PRESS_OSR;

  // 启动连续测量模式
#ifdef BFS_DPS310_PRESSURE_ONLY_CONT
  // 仅用于气压计延迟 profile：去掉 FIFO 中的温度样本, 用初始化阶段的温度补偿压力。
  // 正式飞行默认仍使用温度+压力连续测量，避免长时间温漂补偿失效。
  int ret = Dps310Sensor.startMeasurePressureCont(prs_mr, prs_osr);
#else
  int ret = Dps310Sensor.startMeasureBothCont(temp_mr, temp_osr, prs_mr, prs_osr);
#endif
  if (ret != 0)
  {
#ifdef BFS_DPS310_DIAG
    printDps310InitDiag(ret, 0, ret);
    printDps310RegisterSnapshot("start_fail");
#endif
    return ret;
  }
#ifdef BFS_DPS310_DIAG
  printDps310RegisterSnapshot("after_start");
#endif

  int init_count = 0;
  int16_t init_read_status = DPS__FAIL_UNFINISHED;
  // 读取初始数据
  while ((init_read_status = Dps310Sensor.getLatestResults(temperature_raw, pressure_raw)) != 0)
  {
    init_count++;
    if ((init_count % 50) == 0)
    {
      Dps310Sensor.flushFIFO(); // 周期性清空 FIFO 缓冲区
    }
    if (init_count >= DPS310_INIT_MAX_ATTEMPTS)
    {
      Serial8.println("[Error] DPS310 init data timeout.");
#ifdef BFS_DPS310_DIAG
      printDps310InitDiag(ret, init_count, init_read_status);
      printDps310RegisterSnapshot("init_timeout");
#endif
      return DPS310_INIT_DATA_TIMEOUT_ERROR;
    }
    digitalToggle(LED_green);
    delay(10);
  }
  temperature = temperature_raw;
  pressure = pressure_raw;
#ifdef BFS_DPS310_DIAG
  printDps310InitDiag(ret, init_count, init_read_status);
#endif

  // 保存启动点的气压绝对高度，外部只发布相对此点的高度。
  baro_altitude_offset = Dps310Sensor.calculateAltitude(pressure, temperature);
  baro_altitude_raw = 0.0f;
  baro_altitude = 0.0f;

  // 后续输入是相对高度，滤波器也必须从相对零点启动。
  baro_altitude_filter.initialize(0.0f);
  temperature_filter.initialize(temperature);
  pressure_filter.initialize(pressure);

  Dps310Sensor.flushFIFO(); // 清空 FIFO 缓冲区
  return 0;
}

bool readDPS310Data()
{
  // 获取最新的温度和压力
#ifdef BFS_DPS310_DIAG
  const uint32_t read_start_us = micros();
#endif
  int16_t result = Dps310Sensor.getLatestResults(temperature_raw, pressure_raw);
  dps310_last_read_status = result;
#ifdef BFS_DPS310_DIAG
  const uint32_t elapsed_us = micros() - read_start_us;
  recordDps310Read(result, elapsed_us);
#endif
  if (result == 0)
  {
    temperature = temperature_filter.filter(temperature_raw);
    pressure = pressure_filter.filter(pressure_raw);

    // 计算海拔高度 - 使用考虑实时温度的完整公式，精度比简化公式提升约1~2米
    // 原始数据用于诊断统计
    baro_altitude_raw = Dps310Sensor.calculateAltitude(pressure_raw, temperature_raw) - baro_altitude_offset;
#ifdef BFS_DPS310_DIAG
    recordDps310Sample(pressure_raw, baro_altitude_raw);
#endif
    // 滤波后数据用于控制 - 先滤波气压和温度，再计算高度，避免非线性失真
    float calculated_altitude = Dps310Sensor.calculateAltitude(pressure, temperature) - baro_altitude_offset;
    baro_altitude = baro_altitude_filter.filter(calculated_altitude); // 【已修改】将计算值送入滤波器
    return true;
  }
  else
  {
    // Serial8.print("Failed to get latest DPS310 results, code: "); Serial8.println(result); // 可选调试
    return false; // 读取失败
  }
}

bool resetBaroAltitudeReference()
{
  const float absolute_altitude_m =
      Dps310Sensor.calculateAltitude(pressure, temperature);
  if (!std::isfinite(absolute_altitude_m))
  {
    return false;
  }

  baro_altitude_offset = absolute_altitude_m;
  baro_altitude_raw = 0.0f;
  baro_altitude = 0.0f;
  baro_altitude_filter.initialize(0.0f);
  return true;
}

void handleDPS310()
{
  dps310_read_success = readDPS310Data(); // 读取并处理DPS310数据

  if (!dps310_read_success)
  {
    // digitalWrite(LED_yellow, HIGH); // 点亮黄灯指示DPS310读取问题
    if (dps310::shouldFlushFifoAfterReadStatus(dps310_last_read_status))
    {
      Dps310Sensor.flushFIFO(); // 仅对真实 FIFO/通信异常做恢复，暂无新数据不清空。
#ifdef BFS_DPS310_DIAG
      if (dps310_last_read_status == DPS__FAIL_OVERFLOW)
        dps310_diag.flush_overflow++;
      else
        dps310_diag.flush_unknown++;
#endif
    }
  }
  else
  {
    // 如果IMU也成功，则熄灭黄灯
    // if (imu_read_success) digitalWrite(LED_yellow, LOW);
    // digitalWrite(LED_yellow, LOW); // 简化：只要DPS310成功就尝试熄灭黄灯
  }
#ifdef BFS_DPS310_DIAG
  printDps310DiagIfDue();
#endif
}

void getFilteredTVCAngles(float &angle1, float &angle2)
{
  // 读取TVC角度传感器的原始模拟值并转换为角度
  float raw_angle1 = getServoAngle(SERVO5_PIN);  // SERVO5_PIN 连接到TVC传感器1
  float raw_angle2 = -getServoAngle(SERVO6_PIN); // SERVO6_PIN 连接到TVC传感器2, 原始读数取反

  // 应用互补滤波器对角度值进行滤波
  angle1 = angleSensor1Filter.filter(raw_angle1);
  angle2 = angleSensor2Filter.filter(raw_angle2);
}

void handleAngleSensors()
{
  float filteredSensor1Angle, filteredSensor2Angle;                 // 存储滤波后的TVC传感器角度
  getFilteredTVCAngles(filteredSensor1Angle, filteredSensor2Angle); // 获取角度值

  unsigned long currentTime = millis(); // 获取当前运行时间（毫秒）

  // 通过Type-C串口（Serial8）打印调试数据，格式：
  // 时间,舵机1输出,舵机2输出,传感器1角度,传感器2角度,TVC1目标摆角,TVC2目标摆角,0
  Serial8.print(currentTime);
  Serial8.print(",");
  Serial8.print(ch1_output); // TVC舵机1的控制输出百分比
  Serial8.print(",");
  Serial8.print(ch2_output); // TVC舵机2的控制输出百分比
  Serial8.print(",");
  Serial8.print(filteredSensor1Angle, 2); // 滤波后的传感器1角度，保留2位小数
  Serial8.print(",");
  Serial8.print(filteredSensor2Angle, 2); // 滤波后的传感器2角度，保留2位小数
  Serial8.print(",");
  Serial8.print(tvcTargetAngle1, 2); // TVC舵机1的目标摆角，保留2位小数
  Serial8.print(",");
  Serial8.print(tvcTargetAngle2, 2); // TVC舵机2的目标摆角，保留2位小数
  Serial8.print(",");
  Serial8.print(0); // 占位符或备用数据
  Serial8.println();
}

void handleMTF02PFlow()
{
  // 调用库的更新函数，它会从串口读取字节并驱动内部状态机进行解析。
  // 只有当成功接收并校验通过一帧完整的新数据时，此函数才返回true。
  if (mtf02pSensor.update())
  {
    // ========================================================================
    // --- Part 1: 高度处理 (测距仪读数 -> 垂直高度) ---
    // ========================================================================

    // 首先检查测距数据本身是否被传感器标记为有效。
    flow_data.is_range_valid = mtf02pSensor.isRangeDataValid();
    if (flow_data.is_range_valid)
    {
      // --- 步骤 1.1: 获取原始输入数据 ---
      // 从传感器获取原始斜距读数，并将其从毫米(mm)单位转换为米(m)单位。
      // 这对应于补偿公式中的斜距 S。
      float slant_distance_m = static_cast<float>(mtf02pSensor.getDistance_mm()) / 1000.0f;

      // 从全局AHRS数据包中获取当前的姿态四元数 q_b^n = [w, x, y, z]。
      // 这个四元数描述了从导航坐标系(NED)到机体坐标系(FRD)的旋转。
      // 我们只需要它的 x 和 y 分量来进行高度补偿。
      float qx = AHRS_Packet.Qx;
      float qy = AHRS_Packet.Qy;

      // --- 步骤 1.2: 执行姿态倾斜补偿 ---
      // 应用精确的四元数补偿公式: H = S * (1 - 2*(x^2 + y^2))
      // 补偿因子 (1 - 2*(qx*qx + qy*qy)) 在数学上等价于机体Z轴在导航系垂直轴上的投影长度，
      // 即飞机整体倾斜角度的余弦值 cos(tilt)。
      float height_compensation_factor = 1.0f - 2.0f * (qx * qx + qy * qy);

      // 安全性检查：在极端姿态（如接近90度倾斜）下，补偿因子可能因浮点误差变为负数。
      // 物理上高度总是正值，因此我们使用浮点绝对值函数 fabsf() 来确保结果的正确性。
      // 这对应于数学推导中的 H = S * |(L_n)_z|。
      height_compensation_factor = fabsf(height_compensation_factor);

      // 用斜距乘以补偿因子，计算出最终的、精确的对地垂直高度 H。
      float vertical_height_m = slant_distance_m * height_compensation_factor;

      // --- 步骤 1.3: 应用滤波器并更新全局数据结构 ---
      // 将经过补偿的垂直高度通过滤波器处理，然后存入全局的 flow_data 结构体中。
      flow_data.distance_m = flowDistanceMedianFilter.filter(vertical_height_m);
      flow_data.distance_m = flowDistanceFilter.filter(flow_data.distance_m);
      flow_data.range_quality = mtf02pSensor.getSignalStrength();
      flow_data.last_range_update_ms = millis();

      // --- 状态估计更新 ---
      // updateEstimatedVerticalVelocity(); // 更新垂直速度估计值
      // handleVerticalEstimation(); // 更新垂直状态估计值，使用卡尔曼滤波器
    }
    else
    {
      // 如果传感器报告测距数据无效，则将全局数据清零，并重置滤波器，
      // 以防止控制系统使用过时或错误的数据。
      flow_data.distance_m = 0.0f;
      flow_data.range_quality = 0;
      flowDistanceFilter.initialize(0.0f); // 重置滤波器
    }

    // ========================================================================
    // --- Part 2: 速度处理 (总视在速度 -> 纯平移速度) ---
    // ========================================================================

    // 检查光流数据本身是否有效，并且必须在有效的测距高度下进行速度解算。
    flow_data.is_flow_valid = mtf02pSensor.isFlowDataValid();
    // 设置一个最小高度阈值（如5cm）可以避免在地面上或极低高度时产生无意义的读数。
    // 注意：这里使用 flow_data.distance_m，它已经是滤波后的高度。
    // 光流最小工作距离 8cm (手册)；MTF-02P 测距上限 6m，超出量程的异常读数不可用于速度解算。
    if (flow_data.is_flow_valid && flow_data.is_range_valid &&
        flow_data.distance_m > 0.08f && flow_data.distance_m <= 6.0f)
    {
      // --- 步骤 2.1: 计算总视在速度 (V_total) ---
      // 获取归一化的光流速度 (单位: cm/s @ 1m 高度)。
      int16_t flow_vel_x_norm = mtf02pSensor.getFlowVelX_cms();
      int16_t flow_vel_y_norm = mtf02pSensor.getFlowVelY_cms();

      // 使用补偿后的垂直高度 `flow_data.distance_m` 将归一化速度解算为实际的机体坐标系速度。
      // 这是未经旋转补偿的总视在速度。
      // 注意单位转换：cm/s -> m/s (除以100)。
      float v_total_x = (static_cast<float>(flow_vel_x_norm) / 100.0f) * flow_data.distance_m;
      float v_total_y = (static_cast<float>(flow_vel_y_norm) / 100.0f) * flow_data.distance_m;

      // --- 步骤 2.2: 获取IMU角速度 (ω) ---
      // 从全局变量获取经过滤波后的机体坐标系角速度 (单位: deg/s)。
      // 使用滤波后的数据可以使补偿过程更平滑，减少高频噪声的影响。
      // `IMU_Packet` 中存储的是原始的 rad/s，但我们之前在PID控制中已经计算并存储了滤波后的 deg/s 值。
      // 这里我们直接使用 `current_omega_dps_body_filtered`。
      float omega_x_dps = current_omega_dps_body_filtered.x;
      float omega_y_dps = current_omega_dps_body_filtered.y;

      // 将角速度从 度/秒 (deg/s) 转换为 弧度/秒 (rad/s)，因为物理公式使用弧度单位。
      float omega_x_rads = omega_x_dps * DEG_TO_RAD;
      float omega_y_rads = omega_y_dps * DEG_TO_RAD;

      // --- 步骤 2.3: 计算旋转引起的视在速度 (V_rotation) ---
      // 根据物理模型 V_rot = [H*ω_y, -H*ω_x] 计算旋转分量。
      // 俯仰角速度(ω_y)产生X轴视在速度，滚转角速度(ω_x)产生Y轴视在速度。
      float v_rotation_x = flow_data.distance_m * omega_y_rads;
      float v_rotation_y = -flow_data.distance_m * omega_x_rads;

      // --- 步骤 2.4: 计算纯净的平移速度 (V_translation) ---
      // 从总视在速度中减去由旋转引起的分量，得到飞行器真正的对地平移速度。
      // V_translation = V_total - V_rotation
      float v_translation_x = v_total_x - v_rotation_x;
      float v_translation_y = v_total_y - v_rotation_y;

      // --- 步骤 2.5: 应用滤波器并更新全局数据结构 ---
      // 将最终解算出的、纯净的平移速度通过滤波器处理，然后存入全局数据结构。
      flow_data.velocity_x_mps = flowVelXFilter.filter(v_translation_x);
      flow_data.velocity_y_mps = flowVelYFilter.filter(v_translation_y);
      flow_data.flow_quality = mtf02pSensor.getFlowQuality();
    }
    else
    {
      // 如果光流数据无效或高度条件不满足，则将速度和质量清零，并重置滤波器。
      flow_data.velocity_x_mps = 0.0f;
      flow_data.velocity_y_mps = 0.0f;
      flow_data.flow_quality = 0;
      flowVelXFilter.initialize(0.0f); // 重置滤波器
      flowVelYFilter.initialize(0.0f); // 重置滤波器
    }

    // --- Part 3: 更新时间戳 ---
    // 保存传感器内部的时间戳，可用于数据同步或有效性判断。
    flow_data.timestamp_ms = mtf02pSensor.getTimestamp_ms();
    // 记录本控制器收到有效帧的系统时刻，供超时失效判断。
    flow_data.last_update_ms = millis();
  }
}

void handleLQS48Flow()
{
  // 1. 驱动层解析
  // 必须调用 update() 驱动状态机
  if (lqs48Sensor.update())
  {
    // 获取原始数据包
    LQS48_Data_t raw_data = lqs48Sensor.getData();

    // ========================================================================
    // --- Part 1: 高度处理 (仅用于更新 flow_data 供记录，不用于计算) ---
    // ========================================================================
    // 虽然我们计算速度不用它，但为了数据记录和调试，还是把光流高度解算出来
    if (lqs48Sensor.isHeightValid())
    {
      float raw_dist = (float)raw_data.height_mm / 1000.0f;
      // 简单的倾斜补偿用于记录
      float qx = AHRS_Packet.Qx;
      float qy = AHRS_Packet.Qy;
      float tilt_cos = 1.0f - 2.0f * (qx * qx + qy * qy);
      flow_data.distance_m = raw_dist * fabsf(tilt_cos);
      flow_data.is_range_valid = true;
      flow_data.range_quality = raw_data.quality;
      flow_data.last_range_update_ms = millis();
    }
    else
    {
      flow_data.distance_m = 0.0f;
      flow_data.is_range_valid = false;
      flow_data.range_quality = 0;
    }

    // ========================================================================
    // --- Part 2: 速度处理 (核心逻辑) ---
    // ========================================================================

    // 1. 检查光流有效性
    // 注意：这里我们不再强依赖 isHeightValid，因为我们用的是融合高度
    flow_data.is_flow_valid = lqs48Sensor.isFlowValid();

    // 2. 获取时间间隔 dt (秒)
    float dt_s = 0.0f;
    if (raw_data.dt_flow_ms > 0)
    {
      dt_s = (float)raw_data.dt_flow_ms / 1000.0f;
    }

    // 3. 获取用于计算的距离 (关键步骤)
    // 使用全局融合高度 estimated_height (垂直高度)
    // 并将其逆向投影为斜距: Slant = Vertical / cos(tilt)
    float calc_distance_m = 0.0f;

    // 计算 cos(tilt)
    float qx = AHRS_Packet.Qx;
    float qy = AHRS_Packet.Qy;
    float cos_tilt = 1.0f - 2.0f * (qx * qx + qy * qy);
    cos_tilt = fabsf(cos_tilt);
    if (cos_tilt < 0.1f)
      cos_tilt = 0.1f; // 防止除以0 (极端倾斜保护)

    // 只有当融合高度在有效范围内 (例如 > 5cm) 才计算速度
    if (estimated_height > 0.05f)
    {
      calc_distance_m = estimated_height / cos_tilt;
    }

    // 开始解算
    if (flow_data.is_flow_valid && dt_s > 0.001f && calc_distance_m > 0.25f)
    {
      // A. 计算原始光流角速度 (rad/s)
      // 直接用本帧的角增量除以时间
      float flow_gyro_x = raw_data.flow_x_rad / dt_s;
      float flow_gyro_y = raw_data.flow_y_rad / dt_s;

      // B. 计算总视在速度 (V = ω * D_slant)
      // 使用逆向投影得到的斜距
      float v_total_x = flow_gyro_x * calc_distance_m;
      float v_total_y = flow_gyro_y * calc_distance_m;

      // C. 旋转补偿
      // 获取 IMU 角速度 (rad/s)
      float imu_omega_x = current_omega_dps_body_filtered.x * DEG_TO_RAD;
      float imu_omega_y = current_omega_dps_body_filtered.y * DEG_TO_RAD;

      // 计算旋转分量 (V_rot = ω_imu * D_slant)
      // 同样使用斜距，保证物理模型一致
      float v_rot_x = calc_distance_m * imu_omega_y;
      float v_rot_y = -calc_distance_m * imu_omega_x;

      // D. 得到纯平移速度
      float v_trans_x = v_total_x - v_rot_x;
      float v_trans_y = v_total_y - v_rot_y;

      // E. 滤波输出
      // 因为是单帧原始数据计算，噪声可能稍大，依赖滤波平滑
      flow_data.velocity_x_mps = flowVelXFilter.filter(v_trans_x);
      flow_data.velocity_y_mps = flowVelYFilter.filter(v_trans_y);

      flow_data.flow_quality = raw_data.quality;
    }
    else
    {
      // 条件不满足，速度归零
      flow_data.velocity_x_mps = 0.0f;
      flow_data.velocity_y_mps = 0.0f;
      flow_data.flow_quality = 0;
    }

    // 更新时间戳
    flow_data.timestamp_ms = millis();
    flow_data.last_update_ms = millis();
  }
}
// -----------------------------------------------------------------------------
// UART7 光流传感器统一调度入口
// -----------------------------------------------------------------------------
void handleOpticalFlowSensor()
{
  if (s_flow_dbg_pause)
  {
    return; // DBG flowdump 正在独占读取 UART7
  }

  const uint32_t now_ms = millis();

  // 链路存活 = 光流帧或测距帧任一新鲜（两通道独立计时；MSP/MAVLink 的测距与
  // 光流是独立帧，单通道停流不应误判整链断线，也不应让冻结数据冒充新鲜）。
  const uint32_t flow_age_ms = now_ms - flow_data.last_update_ms;
  const uint32_t range_age_ms = now_ms - flow_data.last_range_update_ms;
  if (active_flow_sensor != FLOW_SENSOR_NONE &&
      flow_age_ms > FLOW_DETECT_STALE_MS && range_age_ms > FLOW_DETECT_STALE_MS)
  {
    flowSensorRestartDetection();
  }

  // 通道级陈旧清扫：某通道停流超过陈旧窗后清对应有效标志，
  // 防止冻结的速度/距离被下游消费者（EKF 融合/垂直速度差分）当新鲜值使用。
  if (active_flow_sensor != FLOW_SENSOR_NONE)
  {
    if (flow_age_ms > kFlowDataStaleTimeoutMs && flow_data.is_flow_valid)
    {
      flow_data.is_flow_valid = false;
    }
    if (range_age_ms > kFlowDataStaleTimeoutMs && flow_data.is_range_valid)
    {
      flow_data.is_range_valid = false;
    }
  }

  if (active_flow_sensor == FLOW_SENSOR_NONE)
  {
    flowSensorDetectTick();
    return;
  }

  switch (active_flow_sensor)
  {
  case FLOW_SENSOR_LQS48:
    handleLQS48Flow();
    break;

  case FLOW_SENSOR_MTF02P_MICOLINK:
    handleMTF02PFlow();
    break;

  case FLOW_SENSOR_MTF02P_MAVLINK_APM:
    // AUTO 版模块需要持续收到 ArduPilot 风格心跳，才保持 MAVLink APM 输出。
    flowSensorSendMavlinkHeartbeat(now_ms);
    flowSensorPumpMavlink();
    break;

  case FLOW_SENSOR_MTF02P_MSP:
    flowSensorPumpMsp();
    break;

  case FLOW_SENSOR_NONE:
  default:
    flowSensorRestartDetection();
    break;
  }
}

void handleMTF02PFlowTelemetry()
{
  // 安全检查：如果光流传感器串口未初始化或不可用，则不执行打印，避免程序崩溃

  // --- 1. 获取当前系统时间戳 ---
  unsigned long t = millis(); // 获取当前运行时间 (毫秒)

  // --- 2. 获取光流测距传感器的原始数据 ---
  uint32_t raw_mm = mtf02pSensor.getDistance_mm(); // 原始斜距 (毫米)
  int16_t raw_x = mtf02pSensor.getFlowVelX_cms();  // 原始X轴光流速度 (cm/s @ 1m高度)
  int16_t raw_y = mtf02pSensor.getFlowVelY_cms();  // 原始Y轴光流速度 (cm/s @ 1m高度)

  // --- 3. 获取经过处理后的光流数据 (来自全局结构体 flow_data) ---
  float H = flow_data.distance_m;      // 经过姿态倾斜补偿后的垂直高度 (米)
  float vx = flow_data.velocity_x_mps; // 经过旋转补偿后的纯X轴平移速度 (米/秒)
  float vy = flow_data.velocity_y_mps; // 经过旋转补偿后的纯Y轴平移速度 (米/秒)

  // --- 4. 计算旋转补偿的中间量 (用于调试验证) ---
  // 获取IMU的滚转和俯仰角速度 (来自IMU_Packet，单位为 rad/s)
  // 注意：这里直接使用了 icm_gyro_x 和 icm_gyro_y，它们在 handleICM42688 中被更新
  // 并且是机体坐标系下的角速度，单位为 rad/s。
  float w_x = icm_gyro_x; // IMU滚转角速度 (rad/s)
  float w_y = icm_gyro_y; // IMU俯仰角速度 (rad/s)

  // 根据公式 V_rotation = [H*ω_y, -H*ω_x] 计算由旋转引起的视在速度分量
  // 注意：符号必须与 handleMTF02PFlow() 中的旋转补偿公式保持一致。
  float v_rot_x = H * w_y;  // 由俯仰旋转引起的X轴视在速度 (米/秒)
  float v_rot_y = -H * w_x; // 由滚转旋转引起的Y轴视在速度 (米/秒)

  // --- 5. 打印一行 CSV 格式的数据到 Serial8 ---
  // 字段 [0]: 系统运行时间戳 (毫秒)
  Serial8.print(t);
  Serial8.print(',');

  // 字段 [1]: 激光测距仪原始斜距 (毫米)
  Serial8.print(raw_mm);
  Serial8.print(',');

  // 字段 [2]: 光流传感器原始X轴速度 (cm/s @ 1m高度)
  Serial8.print(raw_x);
  Serial8.print(',');

  // 字段 [3]: 光流传感器原始Y轴速度 (cm/s @ 1m高度)
  Serial8.print(raw_y);
  Serial8.print(',');

  // 字段 [4]: 测距数据是否有效 (布尔值: 0或1)
  Serial8.print(flow_data.is_range_valid);
  Serial8.print(',');

  // 字段 [5]: 姿态倾斜补偿因子 (cos(倾斜角)，无量纲)
  // 计算公式: 1 - 2*(Qx^2 + Qy^2)，其中Qx, Qy是姿态四元数的虚部
  Serial8.print(1.0f - 2.0f * (AHRS_Packet.Qx * AHRS_Packet.Qx + AHRS_Packet.Qy * AHRS_Packet.Qy), 4); // 保留4位小数
  Serial8.print(',');

  // 字段 [6]: 经过姿态倾斜补偿后的垂直高度 (米)
  Serial8.print(H, 4); // 保留4位小数
  Serial8.print(',');

  // 字段 [7]: 光流数据是否有效 (布尔值: 0或1)
  Serial8.print(flow_data.is_flow_valid);
  Serial8.print(',');

  // 字段 [8]: IMU滚转角速度 (弧度/秒)
  Serial8.print(w_x, 4); // 保留4位小数
  Serial8.print(',');

  // 字段 [9]: IMU俯仰角速度 (弧度/秒)
  Serial8.print(w_y, 4); // 保留4位小数
  Serial8.print(',');

  // 字段 [10]: 由旋转引起的X轴视在速度分量 (米/秒)
  Serial8.print(v_rot_x, 4); // 保留4位小数
  Serial8.print(',');

  // 字段 [11]: 由旋转引起的Y轴视在速度分量 (米/秒)
  Serial8.print(v_rot_y, 4); // 保留4位小数
  Serial8.print(',');

  // 字段 [12]: 经过旋转补偿后的纯X轴平移速度 (米/秒)
  Serial8.print(vx, 4); // 保留4位小数
  Serial8.print(',');

  // 字段 [13]: 经过旋转补偿后的纯Y轴平移速度 (米/秒)
  Serial8.print(vy, 4); // 保留4位小数，并换行结束当前行
  Serial8.println();
}

void handleLQS48FlowTelemetry()
{
  // 安全检查：如果光流传感器串口未初始化或不可用，则不执行打印，避免程序崩溃

  // --- 1. 获取当前系统时间戳 ---
  unsigned long t = millis(); // 获取当前运行时间 (毫秒)

  // --- 2. 获取LQS48光流测距传感器的原始数据 ---
  LQS48_Data_t raw_data = lqs48Sensor.getData();
  uint32_t raw_height_mm = raw_data.height_mm; // 原始高度 (毫米)
  float raw_flow_x_rad = raw_data.flow_x_rad;  // 原始X轴角增量 (弧度)
  float raw_flow_y_rad = raw_data.flow_y_rad;  // 原始Y轴角增量 (弧度)
  uint8_t flow_dt_ms = raw_data.dt_flow_ms;    // 光流数据时间间隔 (毫秒)
  uint8_t quality = raw_data.quality;          // 数据质量标志
  uint8_t lux = raw_data.lux;                  // 环境光照强度

  // --- 3. 获取经过处理后的光流数据 (来自全局结构体 flow_data) ---
  float H = flow_data.distance_m;      // 经过姿态倾斜补偿后的垂直高度 (米)
  float vx = flow_data.velocity_x_mps; // 经过旋转补偿后的纯X轴平移速度 (米/秒)
  float vy = flow_data.velocity_y_mps; // 经过旋转补偿后的纯Y轴平移速度 (米/秒)

  // --- 4. 计算旋转补偿和其他中间量 (用于调试验证) ---
  // 获取IMU的滚转和俯仰角速度 (来自IMU_Packet，单位为 rad/s)
  float w_x = icm_gyro_x; // IMU滚转角速度 (rad/s)
  float w_y = icm_gyro_y; // IMU俯仰角速度 (rad/s)

  // 计算倾斜补偿因子
  float cos_tilt = 1.0f - 2.0f * (AHRS_Packet.Qx * AHRS_Packet.Qx + AHRS_Packet.Qy * AHRS_Packet.Qy);
  cos_tilt = fabsf(cos_tilt);
  if (cos_tilt < 0.1f)
    cos_tilt = 0.1f;

  // 计算逆向投影的斜距
  float slant_distance_m = 0.0f;
  if (estimated_height > 0.05f)
  {
    slant_distance_m = estimated_height / cos_tilt;
  }

  // 计算由旋转引起的视在速度分量
  float v_rot_x = slant_distance_m * w_y;  // 由俯仰旋转引起的X轴视在速度分量
  float v_rot_y = -slant_distance_m * w_x; // 由滚转旋转引起的Y轴视在速度分量

  // --- 5. 打印一行 CSV 格式的数据到 Serial8 ---
  // 字段 [0]: 系统运行时间戳 (毫秒)
  Serial8.print(t);
  Serial8.print(',');

  // 字段 [1]: 激光测距仪原始高度 (毫米)
  Serial8.print(raw_height_mm);
  Serial8.print(',');

  // 字段 [2]: 光流传感器原始X轴角增量 (弧度)
  Serial8.print(raw_flow_x_rad, 6);
  Serial8.print(',');

  // 字段 [3]: 光流传感器原始Y轴角增量 (弧度)
  Serial8.print(raw_flow_y_rad, 6);
  Serial8.print(',');

  // 字段 [4]: 光流数据时间间隔 (毫秒)
  Serial8.print(flow_dt_ms);
  Serial8.print(',');

  // 字段 [5]: 光流数据是否有效 (布尔值: 0或1)
  Serial8.print(lqs48Sensor.isFlowValid());
  Serial8.print(',');

  // 字段 [6]: 高度数据是否有效 (布尔值: 0或1)
  Serial8.print(lqs48Sensor.isHeightValid());
  Serial8.print(',');

  // 字段 [7]: 数据质量标志 (0/1/2/3/23)
  Serial8.print(quality);
  Serial8.print(',');

  // 字段 [8]: 环境光照强度 (0~100)
  Serial8.print(lux);
  Serial8.print(',');

  // 字段 [9]: 姿态倾斜补偿因子 (cos(倾斜角)，无量纲)
  Serial8.print(cos_tilt, 4);
  Serial8.print(',');

  // 字段 [10]: 经过姿态倾斜补偿后的垂直高度 (米)
  Serial8.print(H, 4);
  Serial8.print(',');

  // 字段 [11]: IMU滚转角速度 (弧度/秒)
  Serial8.print(w_x, 4);
  Serial8.print(',');

  // 字段 [12]: IMU俯仰角速度 (弧度/秒)
  Serial8.print(w_y, 4);
  Serial8.print(',');

  // 字段 [13]: 逆向投影的斜距 (米)
  Serial8.print(slant_distance_m, 4);
  Serial8.print(',');

  // 字段 [14]: 由旋转引起的X轴视在速度分量 (米/秒)
  Serial8.print(v_rot_x, 4);
  Serial8.print(',');

  // 字段 [15]: 由旋转引起的Y轴视在速度分量 (米/秒)
  Serial8.print(v_rot_y, 4);
  Serial8.print(',');

  // 字段 [16]: 经过旋转补偿后的纯X轴平移速度 (米/秒)
  Serial8.print(vx, 4);
  Serial8.print(',');

  // 字段 [17]: 经过旋转补偿后的纯Y轴平移速度 (米/秒)
  Serial8.print(vy, 4);
  Serial8.println();
}
