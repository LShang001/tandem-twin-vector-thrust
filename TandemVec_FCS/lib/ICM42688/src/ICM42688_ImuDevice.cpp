/**
 * @file    ICM42688_ImuDevice.cpp
 * @module  传感器库 / ICM-42688-P（ImuAbstraction 后端实现）
 * @brief   ICM42688 → ImuDevice 适配实现
 * @details 档位映射显式枚举；ODR 按"不小于目标值的最接近档"选择，并通过
 *          actualAccelOdrHz()/actualGyroOdrHz() 回真实生效值。
 *
 * @theory  见 ICM42688_ImuDevice.h 的 @theory 段。
 * @ref     lib/ICM42688/src/registers.h、lib/ImuAbstraction/src/ImuDevice.h
 * @frame   N/A
 * @units   accX/Y/Z = g；gyrX/Y/Z = deg/s
 * @note    ★ 行为等价性（2026-10-31 逐项核对，与改造前 sensor_imu.cpp 对比）：
 *          1. 调用顺序一致：begin → LPF(gyro) → LPF(accel) → AccelFS →
 *             AccelODR → GyroFS → GyroODR。
 *          2. 配置值一致：LPF 陀螺 FREQ_100HZ / 加速度 FREQ_50HZ、
 *             AccelFS gpm8、GyroFS dps2000、ODR 均 odr2k（精确命中）。
 *          3. **begin() 的错误码原样透传**（如 -3 WHO_AM_I 不匹配、-4 PWR_MGMT0
 *             写失败），不压缩成统一错误码——上层若依赖具体错误码定位问题，
 *             行为与改造前一致。
 *          4. 不重复做 WHO_AM_I 校验：ICM42688::begin() 内部已校验并返回 -3，
 *             额外校验既冗余又会让错误码从 -3 变成自定义值。
 * @see     ICM42688_ImuDevice.h
 */
#include "ICM42688_ImuDevice.h"

namespace
{
/// ImuLpfBw 档位 → ICM42688 的固定标称带宽 (Hz)。
/// 对应关系：Bw50Hz=0 ↔ FREQ_50HZ ... Bw400Hz=3 ↔ FREQ_400HZ。
float lpfBwToHz(ImuLpfBw bw)
{
  switch (bw)
  {
  case ImuLpfBw::Bw50Hz:  return 50.0f;
  case ImuLpfBw::Bw100Hz: return 100.0f;
  case ImuLpfBw::Bw200Hz: return 200.0f;
  case ImuLpfBw::Bw400Hz: return 400.0f;
  }
  return 100.0f;
}
} // namespace

int ICM42688_ImuDevice::begin(const ImuConfig &cfg)
{
  // begin() 内部已包含：SPI/CS 初始化 → reset() → WHO_AM_I 校验(失败 -3) →
  // PWR_MGMT0 使能 accel/gyro LN 模式(失败 -4)。错误码原样透传。
  const int rs = imu_.begin();
  if (rs < 0)
  {
    err_ = "ICM42688 begin() failed";
    return rs;
  }

  // LPF 延后到 ODR 配置之后（选档依赖实际 ODR，见下方）。

  ICM42688::AccelFS accelFs;
  if (!accelFsrToEnum(cfg.accelFsrG, accelFs))
  {
    err_ = "ICM42688 unsupported accel FSR";
    return -2;
  }
  ICM42688::ODR accelOdr;
  uint16_t accelOdrHz = 0;
  if (!odrToEnum(cfg.accelOdrHz, accelOdr, accelOdrHz))
  {
    err_ = "ICM42688 unsupported accel ODR";
    return -3;
  }
  ICM42688::GyroFS gyroFs;
  if (!gyroFsrToEnum(cfg.gyroFsrDps, gyroFs))
  {
    err_ = "ICM42688 unsupported gyro FSR";
    return -4;
  }
  ICM42688::ODR gyroOdr;
  uint16_t gyroOdrHz = 0;
  if (!odrToEnum(cfg.gyroOdrHz, gyroOdr, gyroOdrHz))
  {
    err_ = "ICM42688 unsupported gyro ODR";
    return -5;
  }

  imu_.setAccelFS(accelFs);
  imu_.setAccelODR(accelOdr);
  imu_.setGyroFS(gyroFs);
  imu_.setGyroODR(gyroOdr);

  // ★2026-11-01 手册核对修复：ICM-42688-P 的 UI 低通在 GYRO_ACCEL_CONFIG0(0x52)：
  //   陀螺 bit3:0、加计 bit7:4，编码 0=ODR/2 … 7=ODR/256；旧实现把档位写进
  //   GYRO_CONFIG0/ACCEL_CONFIG0 的 ODR 字段，实际从未生效（还被随后的 ODR 覆盖）。
  //   LPF 必须在 ODR 之后配，这里按实际 ODR 选最接近目标带宽的档并回报真实值。
  const uint8_t gyroBw = ICM42688::selectLpfCode(gyroOdrHz, lpfBwToHz(cfg.gyroLpf));
  const uint8_t accelBw = ICM42688::selectLpfCode(accelOdrHz, lpfBwToHz(cfg.accelLpf));
  if (imu_.setLowPassFilterBw(gyroBw, true) < 0 ||
      imu_.setLowPassFilterBw(accelBw, false) < 0)
  {
    err_ = "ICM42688 LPF write failed";
    return -6;
  }
  actualGyroLpfHz_ = static_cast<float>(imu_.actualLpfHz(gyroOdrHz, gyroBw));
  actualAccelLpfHz_ = static_cast<float>(imu_.actualLpfHz(accelOdrHz, accelBw));

  actualAccelOdr_ = accelOdrHz;
  actualGyroOdr_ = gyroOdrHz;
  err_ = "";
  return 0;
}

int ICM42688_ImuDevice::readWhoAmI(uint8_t &value)
{
  // 走公开探测接口：内部已做 setBank(0) 读 0x75，并把读失败与"读到 0xFF"区分开。
  return imu_.readWhoAmIValue(value);
}

bool ICM42688_ImuDevice::accelFsrToEnum(uint8_t g, ICM42688::AccelFS &out)
{
  switch (g)
  {
  case 2:  out = ICM42688::AccelFS::gpm2;  return true;
  case 4:  out = ICM42688::AccelFS::gpm4;  return true;
  case 8:  out = ICM42688::AccelFS::gpm8;  return true;
  case 16: out = ICM42688::AccelFS::gpm16; return true;
  default: return false;
  }
}

bool ICM42688_ImuDevice::gyroFsrToEnum(uint16_t dps, ICM42688::GyroFS &out)
{
  switch (dps)
  {
  case 125:  out = ICM42688::GyroFS::dps125;  return true;
  case 250:  out = ICM42688::GyroFS::dps250;  return true;
  case 500:  out = ICM42688::GyroFS::dps500;  return true;
  case 1000: out = ICM42688::GyroFS::dps1000; return true;
  case 2000: out = ICM42688::GyroFS::dps2000; return true;
  default:   return false;
  }
}

bool ICM42688_ImuDevice::odrToEnum(uint16_t hz, ICM42688::ODR &out, uint16_t &actualHz)
{
  // 档位表升序；选 ≥目标值的最接近档，避免换型后采样率低于轮询率导致
  // 读到大量重复样本。只列飞控可能用到的档（12.5/25/1.5625 等低速档不开放）。
  struct Entry { uint16_t hz; ICM42688::ODR odr; };
  static const Entry kTable[] = {
      {50, ICM42688::ODR::odr50},
      {100, ICM42688::ODR::odr100},
      {200, ICM42688::ODR::odr200},
      {500, ICM42688::ODR::odr500},
      {1000, ICM42688::ODR::odr1k},
      {2000, ICM42688::ODR::odr2k},
      {4000, ICM42688::ODR::odr4k},
      {8000, ICM42688::ODR::odr8k},
      {16000, ICM42688::ODR::odr16k},
      {32000, ICM42688::ODR::odr32k},
  };
  const size_t n = sizeof(kTable) / sizeof(kTable[0]);

  // 精确命中优先。
  for (size_t i = 0; i < n; ++i)
  {
    if (kTable[i].hz == hz)
    {
      out = kTable[i].odr;
      actualHz = kTable[i].hz;
      return true;
    }
  }
  // 否则取第一个 ≥ hz 的档。
  for (size_t i = 0; i < n; ++i)
  {
    if (kTable[i].hz > hz)
    {
      out = kTable[i].odr;
      actualHz = kTable[i].hz;
      return true;
    }
  }
  // 超出最高档：取最高档（并让上层通过 actualHz 观察到偏差）。
  out = kTable[n - 1].odr;
  actualHz = kTable[n - 1].hz;
  return true;
}
