/**
 * @file    ICM45686_ImuDevice.h
 * @module  传感器库 / ICM-45686（ImuAbstraction 后端）
 * @brief   把 ICM45686（官方 TDK C driver + Arduino 封装）适配到 ImuDevice
 * @details 位于本芯片自己的库内。负责：档位映射、LPF 的 ODR/2^n 换算、
 *          原始值→物理量换算、WHO_AM_I 校验。不做坐标变换/校准/滤波/EKF。
 *
 * @theory  ★ 全部依据官方手册 DS-000489 ICM-45686 v1.1
 *          （lib/ICM45686/doc/DS-000489 ICM-45686 v1.1.pdf，MinerU 全文在
 *           lib/ICM45686/doc/mineru_.../ 下）。逐条对应：
 *
 *          1) WHO_AM_I：§16 寄存器表，reg **0x72**，复位值/期望值 **0xE9**
 *             （库内 INV_IMU_WHOAMI=0xE9）。注意与 ICM42688 的 0x75/0x47 不同，
 *             因此两者可以靠单一地址读取区分，不存在探测歧义。
 *
 *          2) SPI 读时序：§9.5 "SPI Operational Features" —— 数据 MSB first、
 *             SCLK 上升沿锁存、下降沿切换、首字节 bit7=1 表示读、
 *             后续为寄存器地址、无 dummy 字节、上限 24MHz(VDDIO≥1.71V)。
 *             与 ICM42688 完全一致 ⇒ 同一组 SPI 引脚/时序可复用。
 *
 *          3) 数据端序：§14 —— 默认 Little Endian；寄存器表中的字节描述按
 *             Big Endian 书写。库内 inv_imu_get_endianness() 在初始化时读取
 *             SREG_CTRL(0x67) 的 SREG_DATA_ENDIAN_SEL 并据此用
 *             FORMAT_16_BITS_DATA 组装 int16，故上层拿到的已是 MCU 本地序。
 *
 *          4) 数据寄存器：§15.1/§16 —— ACCEL_DATA_X1_UI(0x00) 起连续排布
 *             accel(6B)/gyro(6B)/temp(2B)，可一次 burst 读出（库已实现）。
 *
 *          5) LPF：§20 IPREG_SYS1_REG_172（GYRO_UI_LPFBW_SEL，bit[2:0]）、
 *             §21 IPREG_SYS2_REG_131（ACCEL_UI_LPFBW_SEL，bit[2:0]），
 *             取值为 **ODR/2^n 倍率**（Div4..Div128 或旁路），不是绝对 Hz。
 *             间接地址：IPREG_SYS1 基址 0xA400 + 0xAC = **0xA4AC**；
 *                       IPREG_SYS2 基址 0xA500 + 0x83 = **0xA583**（§13.4 规则 c/d）。
 *
 *          6) 间接寄存器访问：§13 —— 必须单次 burst 写 {IREG_ADDR_15_8,
 *             IREG_ADDR_7_0, IREG_DATA}；两次 IREG 访问之间至少 **4µs**（§13.3）；
 *             读为 prefetch 语义。库内 read_mreg/write_mreg 已按此实现
 *             （每次访问前 sleep_us(4)），本层直接复用不在上层重拼序列。
 *
 * @ref     DS-000489 v1.1 §9.5/§13/§14/§16/§20/§21；
 *          lib/ICM45686/doc/ICM45686_手册对照审查报告.md
 * @frame   N/A（仅转发芯片原生轴向数据）
 * @units   accX/Y/Z = g；gyrX/Y/Z = deg/s
 * @note    ★ 本后端是新增接线，**未经上板验证**。已验证：编译通过、寄存器
 *          地址/位域与手册一致、SPI 时序与 ICM42688 兼容、库内端序与 4µs
 *          间隔处理符合手册。未验证：真实数据质量、ODR 稳定性、LPF 实际
 *          截止频率、与现有校准/Madgwick/EKF 链路的闭环表现。
 *          换板首次上电必须先做：WHO_AM_I 读回 0xE9 → 静止 |acc|≈1g →
 *          单轴翻转极性核对 → 陀螺零偏量级；确认无误再考虑接通动力。
 * @see     lib/ImuAbstraction/src/ImuDevice.h、src/imu_factory.cpp
 */
#pragma once

#include "ImuDevice.h"
#include "ICM45686.h"

/**
 * @brief ICM-45686 的 ImuDevice 后端。
 *
 * begin() 错误码：
 *   -1   底层 begin() 失败
 *   -2   不支持的加速度量程
 *   -3   不支持的加速度 ODR
 *   -4   不支持的陀螺量程
 *   -5   不支持的陀螺 ODR
 *   -10  WHO_AM_I 不匹配（期望 0xE9）
 *   -11  FSR/ODR/LPF 配置写入失败
 */
class ICM45686_ImuDevice : public ImuDevice
{
public:
  /// WHO_AM_I 期望值（手册 §16：reg 0x72 = 0xE9）
  static constexpr uint8_t kWhoAmI = 0xE9;

  ICM45686_ImuDevice(SPIClass &spi, uint8_t csPin, uint32_t spiHz)
      : imu_(spi, csPin, spiHz) {}

  ImuModel model() const override { return ImuModel::Icm45686; }
  const char *modelName() const override { return "ICM-45686"; }

  int begin(const ImuConfig &cfg) override;

  int dataReady() override;

  float accX() override { return convertAccel(0); }
  float accY() override { return convertAccel(1); }
  float accZ() override { return convertAccel(2); }
  float gyrX() override { return convertGyro(0); }
  float gyrY() override { return convertGyro(1); }
  float gyrZ() override { return convertGyro(2); }

  int readWhoAmI(uint8_t &value) override;

  /// 芯片温度 (°C)：用 dataReady() 最近一次读到的 UI 样本温度换算；
  /// 尚未读到有效样本时返回 NAN（上层用 std::isfinite 过滤）。
  float dieTempC() override;

  uint16_t actualAccelOdrHz() const override { return actualAccelOdr_; }
  uint16_t actualGyroOdrHz() const override { return actualGyroOdr_; }

  /**
   * @brief 实际生效的陀螺低通截止频率 (Hz)，0 表示旁路。
   *
   * ★ 为什么需要它：ICM45686 的 LPF 只有 ODR/2^n 档（Div4..Div128），
   *   当"目标带宽"相对 ODR 偏高时会被物理限制压到远低于目标。例如：
   *     ODR=200Hz + 目标 400Hz → 只能到 Div4 = 50Hz（比目标低 87.5%）
   *     ODR=200Hz + 目标 200Hz → 只能到 Div4 = 50Hz（低 75%）
   *   这类偏差是芯片能力限制、无法通过配置解决，但**必须可观测**，
   *   否则调滤波的人会以为设进去的就是生效值。上电日志应打印本值。
   */
  float actualGyroLpfHz() const { return actualGyroLpfHz_; }

  /// 实际生效的加速度低通截止频率 (Hz)，0 表示旁路。语义同 actualGyroLpfHz()。
  float actualAccelLpfHz() const { return actualAccelLpfHz_; }

  const char *lastError() const override { return err_; }

  /// 底层驱动引用，供需要 FIFO/APEX 等高级能力的上层使用。
  ICM456xx &raw() { return imu_; }

private:
  float convertAccel(int idx) const;
  float convertGyro(int idx) const;

  static bool accelFsrToEnum(uint8_t g, ICM45686AccelFsr &out);
  static bool gyroFsrToEnum(uint16_t dps, ICM45686GyroFsr &out);
  /// 取 ≥目标值的最接近档；无更高档则取最高档。查不到返回 false。
  static bool accelOdrToEnum(uint16_t hz, ICM45686AccelOdr &out, uint16_t &actualHz);
  static bool gyroOdrToEnum(uint16_t hz, ICM45686GyroOdr &out, uint16_t &actualHz);

  /**
   * @brief 将"目标绝对截止频率 + 实际 ODR"换算为 ODR/2^n 的位移量 n。
   * @param odrHz     实际输出数据率
   * @param targetHz  期望截止频率（来自 ImuLpfBw 档位）
   * @return n∈[2,7]；targetHz<=0 时返回 0 表示旁路
   * @note 手册 §20/§21 只提供 2^n 倍率档，没有固定 Hz 档，因此这里只能取最接近
   *       的档位——实际截止频率与目标值可能有偏差，属于芯片能力限制。
   */
  static uint8_t lpfShiftForTarget(uint16_t odrHz, float targetHz);

  /// ImuLpfBw 档位 → 目标截止频率 (Hz)。
  static float lpfTargetHz(ImuLpfBw bw);

  /// 位移量 n(0/2..7) → 陀螺 LPFBW 枚举（0 表示旁路）。
  static ICM45686GyroLpfBw gyroLpfEnum(uint8_t shift);

  /// 位移量 n(0/2..7) → 加速度 LPFBW 枚举（0 表示旁路）。
  static ICM45686AccelLpfBw accelLpfEnum(uint8_t shift);

  ICM456xx imu_;
  inv_imu_sensor_data_t last_{};
  ICM45686AccelFsr accelFsr_ = ICM45686AccelFsr::G8;
  ICM45686GyroFsr gyroFsr_ = ICM45686GyroFsr::Dps2000;
  uint16_t actualAccelOdr_ = 0;
  uint16_t actualGyroOdr_ = 0;
  float actualAccelLpfHz_ = 0.0f;
  float actualGyroLpfHz_ = 0.0f;
  bool started_ = false;
  /// 是否已从 UI 样本读到过温度（dieTempC() 的有效性判据）。
  bool temp_valid_ = false;
  const char *err_ = "";
};
