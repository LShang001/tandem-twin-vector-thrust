#include "ICM45686_FlightAdapter.h"

#include <math.h>

// UI 寄存器六轴数据使用 int16 全量程编码，换算时统一除以 32768。
static constexpr float kRawFullScale = 32768.0f;

// 接近 int16 极值时认为数据已经贴近量程边界；留一点余量避免只在 32767 才报警。
static constexpr int16_t kRawSaturationThreshold = 32000;

ICM45686FlightAdapter::ICM45686FlightAdapter(ICM456xx &imu_ref) : imu(imu_ref) {}

/*
 * 初始化飞控 IMU 读取链路。
 *
 * 设计意图：
 * - 把底层 begin()/startAccel()/startGyro() 固定放在一个入口里，避免飞控主程序漏掉某一步。
 * - 每次重新 begin() 都清空统计值，因为 ODR/FSR 或总线状态已经重新开始。
 * - 不在这里做轴向旋转或零偏估计，保证驱动层只负责“拿到可信原始数据并换算单位”。
 */
int ICM45686FlightAdapter::begin(const ICM45686FlightConfig &config)
{
  // 保存配置副本，后续 readSample() 换算单位和 fresh 判断都依赖它。
  cfg = config;
  resetStats();

  // 底层 begin() 会初始化总线回调、复位芯片并校验 WHO_AM_I。
  int rc = imu.begin();
  stat.last_status = rc;
  if (rc != INV_IMU_OK) {
    started = false;
    return rc;
  }

  // 旧整数 API 仍可用于飞控参数表；小数 ODR/FSR 场景应直接使用底层强类型 API。
  rc |= imu.startAccel(cfg.accel_odr_hz, cfg.accel_fsr_g);
  rc |= imu.startGyro(cfg.gyro_odr_hz, cfg.gyro_fsr_dps);
  stat.last_status = rc;
  started = (rc == INV_IMU_OK);
  return rc;
}

/*
 * 读取并解析一帧 IMU 数据。
 *
 * 输出策略：
 * - 函数入口先把 sample 清零，确保失败返回时上层不会误用旧样本。
 * - 只有底层读取成功后才更新时间戳、统计量和 fresh 标志。
 * - dt 统计只基于成功样本，避免通信错误把调度统计污染成异常大值。
 */
int ICM45686FlightAdapter::readSample(ICM45686FlightSample &sample)
{
  // 清空输出结构，失败路径保持确定状态。
  sample = ICM45686FlightSample {};
  if (!started) {
    stat.read_errors++;
    stat.last_status = INV_IMU_ERROR;
    return INV_IMU_ERROR;
  }

  // UI 寄存器路径一次性读取 accel/gyro/temp，适合飞控主循环轮询。
  inv_imu_sensor_data_t raw {};
  const int rc = imu.getDataFromRegisters(raw);
  stat.last_status = rc;
  if (rc != INV_IMU_OK) {
    stat.read_errors++;
    return rc;
  }

  sample.timestamp_us = micros();
  sample.raw_temp = raw.temp_data;
  for (uint8_t axis = 0; axis < 3; axis++) {
    // 保留原始值，便于后续做标定、日志回放、饱和和总线异常检测。
    sample.raw_accel[axis] = raw.accel_data[axis];
    sample.raw_gyro[axis] = raw.gyro_data[axis];

    // 根据当前量程配置换算到飞控常用物理单位。
    sample.accel_g[axis] = rawAccelToG(raw.accel_data[axis], cfg.accel_fsr_g);
    sample.gyro_dps[axis] = rawGyroToDps(raw.gyro_data[axis], cfg.gyro_fsr_dps);

    // 任一轴贴近原始满量程都记录为饱和，提示量程或机械振动需要检查。
    sample.saturated = sample.saturated || nearRawLimit(raw.accel_data[axis]) ||
                       nearRawLimit(raw.gyro_data[axis]);
  }

  // 温度和加速度模长是 bring-up 阶段最直观的健康检查量。
  sample.temperature_c = rawTempToC(raw.temp_data);
  sample.accel_norm_g = sqrtf(sample.accel_g[0] * sample.accel_g[0] +
                              sample.accel_g[1] * sample.accel_g[1] +
                              sample.accel_g[2] * sample.accel_g[2]);

  if (stat.last_sample_us != 0) {
    // micros() 无符号减法天然支持正常回绕场景，只要间隔远小于 2^32 us。
    const uint32_t dt_us = sample.timestamp_us - stat.last_sample_us;
    stat.dt_min_us = min(stat.dt_min_us, dt_us);
    stat.dt_max_us = max(stat.dt_max_us, dt_us);
    stat.dt_sum_us += dt_us;
  }
  stat.last_sample_us = sample.timestamp_us;
  stat.successful_samples++;
  if (sample.saturated) {
    stat.saturated_samples++;
  }
  sample.fresh = true;
  return INV_IMU_OK;
}

/*
 * 停止六轴输出。
 *
 * 该函数只关闭 accel/gyro，不清空配置；如果后续重新 begin()，统计会被重置。
 */
int ICM45686FlightAdapter::stop()
{
  int rc = INV_IMU_OK;
  if (started) {
    rc |= imu.stopAccel();
    rc |= imu.stopGyro();
  }
  started = false;
  stat.last_status = rc;
  return rc;
}

const ICM45686FlightConfig &ICM45686FlightAdapter::config() const
{
  return cfg;
}

const ICM45686FlightStats &ICM45686FlightAdapter::stats() const
{
  return stat;
}

void ICM45686FlightAdapter::resetStats()
{
  // 结构体默认值里 dt_min_us=UINT32_MAX，因此直接赋默认构造值即可。
  stat = ICM45686FlightStats {};
}

bool ICM45686FlightAdapter::hasFreshSample(uint32_t now_us) const
{
  if (stat.last_sample_us == 0) {
    return false;
  }
  // 使用无符号减法，兼容 micros() 回绕后的短时间窗口判断。
  return (now_us - stat.last_sample_us) <= cfg.stale_after_us;
}

float ICM45686FlightAdapter::rawAccelToG(int16_t raw, uint16_t fsr_g)
{
  // raw=32768 理论上对应正满量程；int16 正端最大为 32767，因此用 32768 做对称缩放。
  return static_cast<float>(raw) * static_cast<float>(fsr_g) / kRawFullScale;
}

float ICM45686FlightAdapter::rawGyroToDps(int16_t raw, uint16_t fsr_dps)
{
  // 陀螺寄存器缩放方式与加速度一致，只是物理单位从 g 换成 deg/s。
  return static_cast<float>(raw) * static_cast<float>(fsr_dps) / kRawFullScale;
}

float ICM45686FlightAdapter::rawTempToC(int16_t raw)
{
  // ICM45686 UI 温度寄存器公式来自手册：25 degC + TEMP_DATA / 128。
  return 25.0f + static_cast<float>(raw) / 128.0f;
}

bool ICM45686FlightAdapter::nearRawLimit(int16_t raw)
{
  // 只做快速阈值判断，不做滤波；连续饱和样本由上层根据 stats() 判定。
  return raw >= kRawSaturationThreshold || raw <= -kRawSaturationThreshold;
}
