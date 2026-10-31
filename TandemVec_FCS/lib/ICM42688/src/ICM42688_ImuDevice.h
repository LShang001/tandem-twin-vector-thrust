/**
 * @file    ICM42688_ImuDevice.h
 * @module  传感器库 / ICM-42688-P（ImuAbstraction 后端）
 * @brief   把 ICM42688 驱动适配到型号无关接口 ImuDevice
 * @details 位于本芯片自己的库内，因此"传感器相关代码都在传感器库里"。
 *          只做档位映射与转发，不做坐标变换、不做校准/滤波/EKF。
 *
 * @theory  寄存器依据（ICM-42688-P 数据手册；本仓库未存该手册全文，
 *          实现依据为 lib/ICM42688/src/registers.h 的常量与库内注释）：
 *          - WHO_AM_I：reg 0x75（UB0_REG_WHO_AM_I），期望 0x47；
 *            读取前必须切 bank0（REG_BANK_SEL=0x76）。
 *          - LPF：GYRO_CONFIG0(0x1C)/ACCEL_CONFIG0(0x1B) 低 4 位为 DLPF_CFG，
 *            取值为固定标称带宽档（50/100/200/400Hz）。
 *            注意这与 ICM-45686 的"ODR/2^n 倍率"语义**不同**。
 *
 * @ref     lib/ICM42688/README.md、lib/ImuAbstraction/src/ImuDevice.h
 * @frame   N/A（仅转发芯片原生轴向数据）
 * @units   accX/Y/Z = g；gyrX/Y/Z = deg/s
 * @note    ★ 行为保持：本后端的档位映射与改造前 sensor_imu.cpp 里的直接调用
 *          逐项等价（accel ±8g / 2kHz、gyro ±2000dps / 2kHz、LPF 陀螺 100Hz +
 *          加速度 50Hz）。默认 ImuConfig 也按这组值设定，因此现役硬件的
 *          采样配置不变。
 * @see     lib/ImuAbstraction/src/ImuDevice.h、src/imu_factory.cpp
 */
#pragma once

#include "ImuDevice.h"
#include "ICM42688.h"

/**
 * @brief ICM-42688-P 的 ImuDevice 后端。
 *
 * begin() 错误码：
 *   -1   底层 begin() 失败（SPI 不通）
 *   -2   不支持的加速度量程
 *   -3   不支持的加速度 ODR
 *   -4   不支持的陀螺量程
 *   -5   不支持的陀螺 ODR
 *   -10  WHO_AM_I 不匹配（焊错芯片 / 总线异常）
 */
class ICM42688_ImuDevice : public ImuDevice
{
public:
  /// WHO_AM_I 期望值（reg 0x75 = 0x47）。
  /// 注：ICM42688 库里的 WHO_AM_I 常量位于 protected 区，这里独立定义一份，
  /// 便于组合式封装直接引用。
  static constexpr uint8_t kExpectedWhoAmI = 0x47;

  ICM42688_ImuDevice(SPIClass &spi, uint8_t csPin, uint32_t spiHz)
      : imu_(spi, csPin, spiHz) {}

  ImuModel model() const override { return ImuModel::ICM42688P; }
  const char *modelName() const override { return "ICM-42688-P"; }

  int begin(const ImuConfig &cfg) override;

  int dataReady() override { return imu_.getAGT(); }

  float accX() override { return imu_.accX(); }
  float accY() override { return imu_.accY(); }
  float accZ() override { return imu_.accZ(); }
  float gyrX() override { return imu_.gyrX(); }
  float gyrY() override { return imu_.gyrY(); }
  float gyrZ() override { return imu_.gyrZ(); }

  /// 芯片温度 (°C)：底层驱动在每次读取数据时更新 _t，未读到样本时返回 NAN。
  float dieTempC() override { return imu_.temp(); }

  int readWhoAmI(uint8_t &value) override;

  uint16_t actualAccelOdrHz() const override { return actualAccelOdr_; }
  uint16_t actualGyroOdrHz() const override { return actualGyroOdr_; }

  /**
   * @brief 实际生效的陀螺低通截止频率 (Hz)。
   *
   * ICM42688 的 DLPF 是**固定标称带宽档**（50/100/200/400Hz），不像 ICM45686
   * 那样随 ODR 缩放，因此本值基本就是配置值。保留该接口是为了让上层能
   * **型号无关地**打印/比较滤波状态（与 ICM45686 后端的同名接口对称）。
   *
   * @note 手册标称值是典型值，实际 -3dB 点与 ODR 相关，这里只回报标称档位。
   */
  float actualGyroLpfHz() const { return actualGyroLpfHz_; }

  /// 实际生效的加速度低通截止频率 (Hz)。语义同 actualGyroLpfHz()。
  float actualAccelLpfHz() const { return actualAccelLpfHz_; }

  const char *lastError() const override { return err_; }

  /// 底层驱动引用，供需要高级能力（FIFO 等）的上层使用；常规路径不要直接用。
  ICM42688 &raw() { return imu_; }

private:
  static bool accelFsrToEnum(uint8_t g, ICM42688::AccelFS &out);
  static bool gyroFsrToEnum(uint16_t dps, ICM42688::GyroFS &out);
  /// 取 ≥目标值的最接近档；无更高档则取最高档。查不到返回 false。
  static bool odrToEnum(uint16_t hz, ICM42688::ODR &out, uint16_t &actualHz);

  ICM42688 imu_;
  uint16_t actualAccelOdr_ = 0;
  uint16_t actualGyroOdr_ = 0;
  float actualAccelLpfHz_ = 0.0f;
  float actualGyroLpfHz_ = 0.0f;
  const char *err_ = "";
};
