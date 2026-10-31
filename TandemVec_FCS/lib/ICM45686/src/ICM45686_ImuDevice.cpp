/**
 * @file    ICM45686_ImuDevice.cpp
 * @module  传感器库 / ICM-45686（ImuAbstraction 后端实现）
 * @brief   ICM45686 → ImuDevice 适配实现
 * @details ODR 按"≥目标值的最接近档"选择（ICM45686 无 2000Hz 档，会落到
 *          3200Hz），并回真实生效值；LPF 按实际 ODR 换算成 ODR/2^n 档。
 *          原始值换算复用库内辅助函数，不在上层重复实现手册公式。
 *
 * @theory  见 ICM45686_ImuDevice.h 的 @theory 段（DS-000489 v1.1 §9.5/§13/
 *          §14/§16/§20/§21）。
 * @ref     lib/ICM45686/src/imu/inv_imu_defs.h（枚举与位域）、
 *          lib/ICM45686/src/imu/inv_imu_driver.c（LPF 读-改-写实现）
 * @frame   N/A
 * @units   accX/Y/Z = g；gyrX/Y/Z = deg/s
 * @note    ★ dataReady() 语义（2026-10-31 自查修正后）：与 ICM42688 的
 *          getAGT() 对齐——先读 INT1_STATUS0(0x19) 的 bit2 = INT1_STATUS_DRDY
 *          做门控，无新样本返回 0，有新样本才 burst 读数据寄存器并返回 1。
 *          初版曾写成"每次调用无条件取一帧"，在 2kHz 轮询 + 3200Hz ODR 下会
 *          反复返回同一帧，而上层仍按 micros() 差分累进 imu_delta_*，
 *          等于重复样本被当新样本积分、污染 EKF —— 这是必须避免的。
 *          注意 DRDY 是 bit2，bit3 是同寄存器的 AUX1_DRDY（不同通道）。
 * @see     ICM45686_ImuDevice.h
 */
#include "ICM45686_ImuDevice.h"

namespace
{
/// 手册 DS-000489 v1.1 §16.24：INT1_STATUS0 寄存器地址（R/C 读清）。
constexpr uint8_t kInt1Status0Reg = 0x19;
/// int1_status_drdy 位于 bit2（bit3 是 AUX1_DRDY，勿混用）。
constexpr uint8_t kInt1StatusDrdyMask = 0x04;
} // namespace

int ICM45686_ImuDevice::begin(const ImuConfig &cfg)
{
  // begin() 内部（inv_imu_adv_init，见 imu/inv_imu_driver_advanced.c）已完成：
  //   等待 3ms 供电稳定 → 配置串行接口 → 读并校验 WHO_AM_I（失败返回
  //   INV_IMU_ERROR）→ 软复位 → 默认配置。错误码原样透传，与 42688 后端一致。
  const int rs = imu_.begin();
  if (rs != 0)
  {
    err_ = "ICM45686 begin() failed";
    return rs;
  }

  ICM45686AccelFsr accelFs;
  if (!accelFsrToEnum(cfg.accelFsrG, accelFs))
  {
    err_ = "ICM45686 unsupported accel FSR";
    return -2;
  }
  ICM45686AccelOdr accelOdr;
  uint16_t accelOdrHz = 0;
  if (!accelOdrToEnum(cfg.accelOdrHz, accelOdr, accelOdrHz))
  {
    err_ = "ICM45686 unsupported accel ODR";
    return -3;
  }
  ICM45686GyroFsr gyroFs;
  if (!gyroFsrToEnum(cfg.gyroFsrDps, gyroFs))
  {
    err_ = "ICM45686 unsupported gyro FSR";
    return -4;
  }
  ICM45686GyroOdr gyroOdr;
  uint16_t gyroOdrHz = 0;
  if (!gyroOdrToEnum(cfg.gyroOdrHz, gyroOdr, gyroOdrHz))
  {
    err_ = "ICM45686 unsupported gyro ODR";
    return -5;
  }

  // ★ 错误处置（2026-10-31 自查修正）：不能写 `rc |= step()`。
  //   底层错误码是负数（INV_IMU_ERROR=-1 / TRANSPORT=-3 / TIMEOUT=-4 ...），
  //   负数按位或会互相污染（例如 -3 | -4 = -1），最终无法判断是哪一步失败，
  //   甚至可能把"两个错误"折成 -1 这种无意义值。这里逐步检查并立即返回
  //   带步骤号的错误码，便于上板时定位。
  int step = imu_.startAccel(accelOdr, accelFs);
  if (step != 0)
  {
    err_ = "ICM45686 startAccel failed";
    return -20;
  }
  step = imu_.startGyro(gyroOdr, gyroFs);
  if (step != 0)
  {
    err_ = "ICM45686 startGyro failed";
    return -21;
  }

  // LPF：ImuLpfBw 是"目标绝对 Hz"语义，而 ICM45686 只有 ODR/2^n 倍率档，
  // 故按**实际生效的 ODR** 换算成最接近的 2^n 档（手册 §20/§21）。
  // 同时记录实际截止频率：低 ODR + 高目标带宽时会被芯片能力压到远低于目标
  // （如 ODR=200Hz + 目标 400Hz → 只能 50Hz），必须可观测，详见头文件说明。
  const uint8_t gyroShift = lpfShiftForTarget(gyroOdrHz, lpfTargetHz(cfg.gyroLpf));
  const uint8_t accelShift = lpfShiftForTarget(accelOdrHz, lpfTargetHz(cfg.accelLpf));
  actualGyroLpfHz_ = (gyroShift == 0U)
                         ? 0.0f
                         : static_cast<float>(gyroOdrHz) / static_cast<float>(1U << gyroShift);
  actualAccelLpfHz_ = (accelShift == 0U)
                          ? 0.0f
                          : static_cast<float>(accelOdrHz) / static_cast<float>(1U << accelShift);
  step = imu_.setLowPassFilterGyro(gyroLpfEnum(gyroShift));
  if (step != 0)
  {
    err_ = "ICM45686 gyro LPF write failed";
    return -22;
  }
  step = imu_.setLowPassFilterAccel(accelLpfEnum(accelShift));
  if (step != 0)
  {
    err_ = "ICM45686 accel LPF write failed";
    return -23;
  }

  // 记住当前 FSR：raw 换算需要枚举（库内 rawRegisterAccelToG 按 FSR 缩放）。
  accelFsr_ = accelFs;
  gyroFsr_ = gyroFs;
  actualAccelOdr_ = accelOdrHz;
  actualGyroOdr_ = gyroOdrHz;
  started_ = true;
  err_ = "";
  return 0;
}

int ICM45686_ImuDevice::dataReady()
{
  if (!started_)
  {
    return 0;
  }

  // ★ 2026-10-31 自查修正：补 DRDY 检查，使语义与 ICM42688 的 getAGT() 对齐。
  //
  // 手册依据 DS-000489 v1.1 §16.24 INT1_STATUS0（reg 0x19，R/C 读清）：
  //   bit2 = INT1_STATUS_DRDY（UI 数据就绪）
  //   bit3 = INT1_STATUS_AUX1_DRDY（注意：这是 AUX1 通道，**不是** UI，不要用错）
  // 位域顺序见 lib/ICM45686/src/imu/inv_imu_regmap_be.h 的 int1_status0_t：
  //   reset_done(7) aux1_agc_rdy(6) ap_agc_rdy(5) ap_fsync(4)
  //   aux1_drdy(3)  drdy(2)         fifo_ths(1)   fifo_full(0)
  //
  // 为什么必须检查：本方法被 2kHz 调度调用，而 ODR 只有 2000/3200Hz。若不做
  // DRDY 门控，每次调用都会把**同一帧**寄存器值再读一遍并返回 1，上层
  // readIMUData() 会用 micros() 差分算出真实 dt 并累进 imu_delta_*，
  // 等于把重复样本当成新样本积分 —— 这会污染 EKF 的速度/角度增量。
  // 加上门控后，无新数据时返回 0，与 ICM42688 getAGT() 行为一致。
  uint8_t status = 0;
  if (imu_.readRegister(kInt1Status0Reg, status) != 0)
  {
    return 0; // 总线读失败：按"无新数据"处理，避免上层读到陈旧值
  }
  if ((status & kInt1StatusDrdyMask) == 0)
  {
    return 0; // 无新样本
  }

  inv_imu_sensor_data_t d;
  if (imu_.getDataFromRegisters(d) != 0)
  {
    return 0;
  }
  last_ = d;
  return 1;
}

int ICM45686_ImuDevice::readWhoAmI(uint8_t &value)
{
  return imu_.readWhoAmI(value);
}

float ICM45686_ImuDevice::convertAccel(int idx) const
{
  // 手册 §15.1：原始值为 int16 满量程编码，按当前 FSR 线性缩放到 g。
  return ICM456xx::rawRegisterAccelToG(last_.accel_data[idx], accelFsr_);
}

float ICM45686_ImuDevice::convertGyro(int idx) const
{
  return ICM456xx::rawRegisterGyroToDps(last_.gyro_data[idx], gyroFsr_);
}

bool ICM45686_ImuDevice::accelFsrToEnum(uint8_t g, ICM45686AccelFsr &out)
{
  switch (g)
  {
  case 2:  out = ICM45686AccelFsr::G2;  return true;
  case 4:  out = ICM45686AccelFsr::G4;  return true;
  case 8:  out = ICM45686AccelFsr::G8;  return true;
  case 16: out = ICM45686AccelFsr::G16; return true;
  default: return false;
  }
}

bool ICM45686_ImuDevice::gyroFsrToEnum(uint16_t dps, ICM45686GyroFsr &out)
{
  // 手册 §2/§3：陀螺 FSR 支持 15.625..4000dps；这里只开放与 ICM42688 相同的
  // 常用档（125/250/500/1000/2000），保证换型时配置可原样复用。
  switch (dps)
  {
  case 125:  out = ICM45686GyroFsr::Dps125;  return true;
  case 250:  out = ICM45686GyroFsr::Dps250;  return true;
  case 500:  out = ICM45686GyroFsr::Dps500;  return true;
  case 1000: out = ICM45686GyroFsr::Dps1000; return true;
  case 2000: out = ICM45686GyroFsr::Dps2000; return true;
  default:   return false;
  }
}

bool ICM45686_ImuDevice::accelOdrToEnum(uint16_t hz, ICM45686AccelOdr &out, uint16_t &actualHz)
{
  // ICM45686 的 ODR 序列是 2 倍档（...800/1600/3200/6400），**没有 2000Hz**。
  // 因此目标 2000Hz 会落到 3200Hz —— 保证 ≥ 目标，避免欠采样。
  struct Entry { uint16_t hz; ICM45686AccelOdr odr; };
  static const Entry kTable[] = {
      {50, ICM45686AccelOdr::Hz50},
      {100, ICM45686AccelOdr::Hz100},
      {200, ICM45686AccelOdr::Hz200},
      {400, ICM45686AccelOdr::Hz400},
      {800, ICM45686AccelOdr::Hz800},
      {1600, ICM45686AccelOdr::Hz1600},
      {3200, ICM45686AccelOdr::Hz3200},
      {6400, ICM45686AccelOdr::Hz6400},
  };
  const size_t n = sizeof(kTable) / sizeof(kTable[0]);

  for (size_t i = 0; i < n; ++i)
  {
    if (kTable[i].hz == hz)
    {
      out = kTable[i].odr;
      actualHz = kTable[i].hz;
      return true;
    }
  }
  for (size_t i = 0; i < n; ++i)
  {
    if (kTable[i].hz > hz)
    {
      out = kTable[i].odr;
      actualHz = kTable[i].hz;
      return true;
    }
  }
  out = kTable[n - 1].odr;
  actualHz = kTable[n - 1].hz;
  return true;
}

bool ICM45686_ImuDevice::gyroOdrToEnum(uint16_t hz, ICM45686GyroOdr &out, uint16_t &actualHz)
{
  struct Entry { uint16_t hz; ICM45686GyroOdr odr; };
  static const Entry kTable[] = {
      {50, ICM45686GyroOdr::Hz50},
      {100, ICM45686GyroOdr::Hz100},
      {200, ICM45686GyroOdr::Hz200},
      {400, ICM45686GyroOdr::Hz400},
      {800, ICM45686GyroOdr::Hz800},
      {1600, ICM45686GyroOdr::Hz1600},
      {3200, ICM45686GyroOdr::Hz3200},
      {6400, ICM45686GyroOdr::Hz6400},
  };
  const size_t n = sizeof(kTable) / sizeof(kTable[0]);

  for (size_t i = 0; i < n; ++i)
  {
    if (kTable[i].hz == hz)
    {
      out = kTable[i].odr;
      actualHz = kTable[i].hz;
      return true;
    }
  }
  for (size_t i = 0; i < n; ++i)
  {
    if (kTable[i].hz > hz)
    {
      out = kTable[i].odr;
      actualHz = kTable[i].hz;
      return true;
    }
  }
  out = kTable[n - 1].odr;
  actualHz = kTable[n - 1].hz;
  return true;
}

uint8_t ICM45686_ImuDevice::lpfShiftForTarget(uint16_t odrHz, float targetHz)
{
  if (targetHz <= 0.0f || odrHz == 0U)
  {
    return 0U; // 旁路
  }
  // 手册 §20/§21：LPFBW_SEL 只有 ODR/4 .. ODR/128 六档 + 旁路，
  // 因此只能取最接近目标的 2^n 档。
  uint8_t bestShift = 2U;
  float bestErr = 1.0e30f;
  for (uint8_t shift = 2U; shift <= 7U; ++shift)
  {
    const float cutoff = static_cast<float>(odrHz) / static_cast<float>(1U << shift);
    const float err = (cutoff > targetHz) ? (cutoff - targetHz) : (targetHz - cutoff);
    if (err < bestErr)
    {
      bestErr = err;
      bestShift = shift;
    }
  }
  return bestShift;
}

float ICM45686_ImuDevice::lpfTargetHz(ImuLpfBw bw)
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

ICM45686GyroLpfBw ICM45686_ImuDevice::gyroLpfEnum(uint8_t shift)
{
  // 手册 §20 IPREG_SYS1_REG_172 GYRO_UI_LPFBW_SEL 位域取值。
  switch (shift)
  {
  case 2:  return ICM45686GyroLpfBw::Div4;
  case 3:  return ICM45686GyroLpfBw::Div8;
  case 4:  return ICM45686GyroLpfBw::Div16;
  case 5:  return ICM45686GyroLpfBw::Div32;
  case 6:  return ICM45686GyroLpfBw::Div64;
  case 7:  return ICM45686GyroLpfBw::Div128;
  default: return ICM45686GyroLpfBw::Bypass;
  }
}

ICM45686AccelLpfBw ICM45686_ImuDevice::accelLpfEnum(uint8_t shift)
{
  // 手册 §21 IPREG_SYS2_REG_131 ACCEL_UI_LPFBW_SEL 位域取值。
  switch (shift)
  {
  case 2:  return ICM45686AccelLpfBw::Div4;
  case 3:  return ICM45686AccelLpfBw::Div8;
  case 4:  return ICM45686AccelLpfBw::Div16;
  case 5:  return ICM45686AccelLpfBw::Div32;
  case 6:  return ICM45686AccelLpfBw::Div64;
  case 7:  return ICM45686AccelLpfBw::Div128;
  default: return ICM45686AccelLpfBw::Bypass;
  }
}
