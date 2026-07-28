#include "sensor_peripheral.h"
#include "math_utils.h"
#include "Dps310FifoPolicy.h"

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

void handleOpticalFlow()
{
  // 调用库的更新函数，它会从串口读取字节并驱动内部状态机进行解析。
  // 只有当成功接收并校验通过一帧完整的新数据时，此函数才返回true。
  if (opticalFlowSensor.update())
  {
    // ========================================================================
    // --- Part 1: 高度处理 (测距仪读数 -> 垂直高度) ---
    // ========================================================================

    // 首先检查测距数据本身是否被传感器标记为有效。
    flow_data.is_range_valid = opticalFlowSensor.isRangeDataValid();
    if (flow_data.is_range_valid)
    {
      // --- 步骤 1.1: 获取原始输入数据 ---
      // 从传感器获取原始斜距读数，并将其从毫米(mm)单位转换为米(m)单位。
      // 这对应于补偿公式中的斜距 S。
      float slant_distance_m = static_cast<float>(opticalFlowSensor.getDistance_mm()) / 1000.0f;

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
      flow_data.range_quality = opticalFlowSensor.getSignalStrength();

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
    flow_data.is_flow_valid = opticalFlowSensor.isFlowDataValid();
    // 设置一个最小高度阈值（如5cm）可以避免在地面上或极低高度时产生无意义的读数。
    // 注意：这里使用 flow_data.distance_m，它已经是滤波后的高度。
    if (flow_data.is_flow_valid && flow_data.is_range_valid && flow_data.distance_m > 0.05f)
    {
      // --- 步骤 2.1: 计算总视在速度 (V_total) ---
      // 获取归一化的光流速度 (单位: cm/s @ 1m 高度)。
      int16_t flow_vel_x_norm = opticalFlowSensor.getFlowVelX_cms();
      int16_t flow_vel_y_norm = opticalFlowSensor.getFlowVelY_cms();

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
      flow_data.flow_quality = opticalFlowSensor.getFlowQuality();
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
    flow_data.timestamp_ms = opticalFlowSensor.getTimestamp_ms();
  }
}

void handleLQS48Flow()
{
  // 1. 驱动层解析
  // 必须调用 update() 驱动状态机
  if (opticalFlowSensorLQS48.update())
  {
    // 获取原始数据包
    LQS48_Data_t raw_data = opticalFlowSensorLQS48.getData();

    // ========================================================================
    // --- Part 1: 高度处理 (仅用于更新 flow_data 供记录，不用于计算) ---
    // ========================================================================
    // 虽然我们计算速度不用它，但为了数据记录和调试，还是把光流高度解算出来
    if (opticalFlowSensorLQS48.isHeightValid())
    {
      float raw_dist = (float)raw_data.height_mm / 1000.0f;
      // 简单的倾斜补偿用于记录
      float qx = AHRS_Packet.Qx;
      float qy = AHRS_Packet.Qy;
      float tilt_cos = 1.0f - 2.0f * (qx * qx + qy * qy);
      flow_data.distance_m = raw_dist * fabsf(tilt_cos);
      flow_data.is_range_valid = true;
      flow_data.range_quality = raw_data.quality;
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
    flow_data.is_flow_valid = opticalFlowSensorLQS48.isFlowValid();

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
  }
}

void handleFlowTelemetry()
{
  // 安全检查：如果光流传感器串口未初始化或不可用，则不执行打印，避免程序崩溃

  // --- 1. 获取当前系统时间戳 ---
  unsigned long t = millis(); // 获取当前运行时间 (毫秒)

  // --- 2. 获取光流测距传感器的原始数据 ---
  uint32_t raw_mm = opticalFlowSensor.getDistance_mm(); // 原始斜距 (毫米)
  int16_t raw_x = opticalFlowSensor.getFlowVelX_cms();  // 原始X轴光流速度 (cm/s @ 1m高度)
  int16_t raw_y = opticalFlowSensor.getFlowVelY_cms();  // 原始Y轴光流速度 (cm/s @ 1m高度)

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

  // 根据公式 V_rotation = [-H*ω_y, H*ω_x] 计算由旋转引起的视在速度分量
  float v_rot_x = -H * w_y; // 由俯仰旋转引起的X轴视在速度 (米/秒)
  float v_rot_y = H * w_x;  // 由滚转旋转引起的Y轴视在速度 (米/秒)

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
  LQS48_Data_t raw_data = opticalFlowSensorLQS48.getData();
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
  Serial8.print(opticalFlowSensorLQS48.isFlowValid());
  Serial8.print(',');

  // 字段 [6]: 高度数据是否有效 (布尔值: 0或1)
  Serial8.print(opticalFlowSensorLQS48.isHeightValid());
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
