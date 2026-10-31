/*
 *
 * Copyright (c) [2020] by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */
 
#ifndef ICM456xx_H
#define ICM456xx_H

#include "Arduino.h"
#include "SPI.h"
#include "Wire.h"

extern "C" {
  #include "imu/inv_imu_driver_advanced.h"
  #include "imu/inv_imu_edmp.h"
  #include "imu/inv_imu_selftest.h"
  #include "Invn/InvError.h"
}

/*
 * ICM456xx 系列芯片类型。
 *
 * 本库默认面向 ICM45686，编译宏由 imu/inv_imu.h 给出。不同芯片族可用的 eDMP/APEX
 * 功能不同，公共 API 用条件编译暴露对应能力：
 * - A1: ICM45686，支持基础六轴、FIFO、APEX A 类事件、AUX I2CM/ICT1531 磁力计路径。
 * - A2/B1/C1: 部分型号额外支持 GAF 或 VocalVibDet。
 */
#define INV_TYPE_A1    0
#define INV_TYPE_A2    1
#define INV_TYPE_B1    2
#define INV_TYPE_C1    4

#if defined(ICM45605) || defined(ICM45686) || defined(ICM45688P)
  #define INV_DEVICE_TYPE INV_TYPE_A1
#elif  defined(ICM45605S) || defined(ICM45686S)
  #define INV_DEVICE_TYPE INV_TYPE_A2
#elif defined(ICM45606) || defined(ICM45687) || defined(ICM45685)
  #define INV_DEVICE_TYPE INV_TYPE_B1
#elif defined(ICM45689) || defined(ICM45608)
  #define INV_DEVICE_TYPE INV_TYPE_C1
#else
  #error "No ICM device macro defined. Please include an inv_imu.h."
#endif

#if (INV_DEVICE_TYPE == INV_TYPE_A2)
  #include "imu/inv_imu_edmp_gaf.h"
#endif

enum {
  ACCEL = 0,
  GYRO,
  MAG
};

// GAF 算法选择。注意：ICM45686(A1) 不编译 GAF 接口。
// algo 0: GRV，六轴 accel+gyro。
// algo 1: GMRV，六轴 accel+mag。
// algo 2: RV，九轴 accel+gyro+mag。
enum {
  ALGO_GRV = 0,
  ALGO_GMRV,
  ALGO_RV
};

enum {
  ICM456XX_APEX_TILT=0,
  ICM456XX_APEX_PEDOMETER,
  ICM456XX_APEX_TAP,
  ICM456XX_APEX_R2W,
  ICM456XX_APEX_B2S,
  ICM456XX_APEX_AID,
  ICM456XX_APEX_FF,
  ICM456XX_APEX_LOWG,
  ICM456XX_APEX_HIGHG,
  ICM456XX_APEX_MAX
};

// APEX 自动 ODR 选择标记。不要使用 NULL 作为枚举默认值，避免 C++ 编译器产生指针算术警告。
#define ICM456XX_APEX_ODR_AUTO   ((dmp_ext_sen_odr_cfg_apex_odr_t)-1)
#define ICM456XX_ACCEL_ODR_AUTO  ((accel_config0_accel_odr_t)-1)

// 中断回调。Arduino attachInterrupt() 会直接调用该函数，回调内只做置位/计数，不做 SPI/I2C 访问。
typedef void (*ICM456xx_irq_handler)(void);

// 加速度计 ODR，枚举名保留小数档位，避免旧 uint16_t API 把 12.5 Hz 写成 12 Hz 造成歧义。
enum class ICM45686AccelOdr : uint8_t {
  Hz1_5625 = ACCEL_CONFIG0_ACCEL_ODR_1_5625_HZ,
  Hz3_125  = ACCEL_CONFIG0_ACCEL_ODR_3_125_HZ,
  Hz6_25   = ACCEL_CONFIG0_ACCEL_ODR_6_25_HZ,
  Hz12_5   = ACCEL_CONFIG0_ACCEL_ODR_12_5_HZ,
  Hz25     = ACCEL_CONFIG0_ACCEL_ODR_25_HZ,
  Hz50     = ACCEL_CONFIG0_ACCEL_ODR_50_HZ,
  Hz100    = ACCEL_CONFIG0_ACCEL_ODR_100_HZ,
  Hz200    = ACCEL_CONFIG0_ACCEL_ODR_200_HZ,
  Hz400    = ACCEL_CONFIG0_ACCEL_ODR_400_HZ,
  Hz800    = ACCEL_CONFIG0_ACCEL_ODR_800_HZ,
  Hz1600   = ACCEL_CONFIG0_ACCEL_ODR_1600_HZ,
  Hz3200   = ACCEL_CONFIG0_ACCEL_ODR_3200_HZ,
  Hz6400   = ACCEL_CONFIG0_ACCEL_ODR_6400_HZ,
};

// 陀螺仪 ODR。ICM45686 的 accel/gyro ODR 位域相同，但保留两个类型可以防止误传参数。
enum class ICM45686GyroOdr : uint8_t {
  Hz1_5625 = GYRO_CONFIG0_GYRO_ODR_1_5625_HZ,
  Hz3_125  = GYRO_CONFIG0_GYRO_ODR_3_125_HZ,
  Hz6_25   = GYRO_CONFIG0_GYRO_ODR_6_25_HZ,
  Hz12_5   = GYRO_CONFIG0_GYRO_ODR_12_5_HZ,
  Hz25     = GYRO_CONFIG0_GYRO_ODR_25_HZ,
  Hz50     = GYRO_CONFIG0_GYRO_ODR_50_HZ,
  Hz100    = GYRO_CONFIG0_GYRO_ODR_100_HZ,
  Hz200    = GYRO_CONFIG0_GYRO_ODR_200_HZ,
  Hz400    = GYRO_CONFIG0_GYRO_ODR_400_HZ,
  Hz800    = GYRO_CONFIG0_GYRO_ODR_800_HZ,
  Hz1600   = GYRO_CONFIG0_GYRO_ODR_1600_HZ,
  Hz3200   = GYRO_CONFIG0_GYRO_ODR_3200_HZ,
  Hz6400   = GYRO_CONFIG0_GYRO_ODR_6400_HZ,
};

// 加速度计 UI 满量程。32 g 只有在底层宏 INV_IMU_HIGH_FSR_SUPPORTED 开启时才真正可用。
enum class ICM45686AccelFsr : uint8_t {
  G2  = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_2_G,
  G4  = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_4_G,
  G8  = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_8_G,
  G16 = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_16_G,
#if INV_IMU_HIGH_FSR_SUPPORTED
  G32 = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_32_G,
#endif
};

// 陀螺仪 UI 满量程。小数档位按手册精确命名，避免 15/31/62 这种近似整数造成误解。
enum class ICM45686GyroFsr : uint8_t {
  Dps15_625 = GYRO_CONFIG0_GYRO_UI_FS_SEL_15_625_DPS,
  Dps31_25  = GYRO_CONFIG0_GYRO_UI_FS_SEL_31_25_DPS,
  Dps62_5   = GYRO_CONFIG0_GYRO_UI_FS_SEL_62_5_DPS,
  Dps125    = GYRO_CONFIG0_GYRO_UI_FS_SEL_125_DPS,
  Dps250    = GYRO_CONFIG0_GYRO_UI_FS_SEL_250_DPS,
  Dps500    = GYRO_CONFIG0_GYRO_UI_FS_SEL_500_DPS,
  Dps1000   = GYRO_CONFIG0_GYRO_UI_FS_SEL_1000_DPS,
  Dps2000   = GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS,
#if INV_IMU_HIGH_FSR_SUPPORTED
  Dps4000   = GYRO_CONFIG0_GYRO_UI_FS_SEL_4000_DPS,
#endif
};

// ★ 2026-10-31 飞控接入新增：UI 通路低通滤波带宽档位。
//   手册依据 DS-000489 v1.1：
//   - GYRO_UI_LPFBW_SEL 在 IPREG_SYS1_REG_172(0xA4AC) bit[2:0]，档位为 ODR/4 ... ODR/128 或旁路；
//   - ACCEL_UI_LPFBW_SEL 在 IPREG_SYS2_REG_131(0xA583) bit[2:0]，同上。
//   注意这是 **相对 ODR 的倍率**，不是绝对 Hz——同一个 Div32 档在 ODR=3200Hz 时是 100Hz，
//   在 ODR=200Hz 时只有 6.25Hz。上层抽象层负责按目标绝对带宽换算档位。
enum class ICM45686GyroLpfBw : uint8_t {
  Bypass = IPREG_SYS1_REG_172_GYRO_UI_LPFBW_NO_FILTER,
  Div4   = IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_4,
  Div8   = IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_8,
  Div16  = IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_16,
  Div32  = IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_32,
  Div64  = IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_64,
  Div128 = IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_128,
};

enum class ICM45686AccelLpfBw : uint8_t {
  Bypass = IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_NO_FILTER,
  Div4   = IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4,
  Div8   = IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_8,
  Div16  = IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_16,
  Div32  = IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_32,
  Div64  = IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_64,
  Div128 = IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_128,
};

// FIFO 工作模式。名称直接对应 FIFO_CONFIG0，但对 Arduino 用户更直观。
enum class ICM45686FifoMode : uint8_t {
  Bypass   = FIFO_CONFIG0_FIFO_MODE_BYPASS,
  Stream   = FIFO_CONFIG0_FIFO_MODE_STREAM,
  Snapshot = FIFO_CONFIG0_FIFO_MODE_SNAPSHOT,
};

// FIFO 深度。ICM45686 在不开 APEX/eDMP 占用 SRAM 时可使用最大 FIFO；APEX 场景应使用 2KB。
enum class ICM45686FifoDepth : uint8_t {
  Depth2KB = FIFO_CONFIG0_FIFO_DEPTH_APEX,
  Depth8KB = FIFO_CONFIG0_FIFO_DEPTH_MAX,
};

// Wake on Motion 配置，阈值全部使用 mg，对外不再暴露容易误解的 1g/256 LSB。
struct ICM45686WomConfig {
  uint8_t intpin = 2;                              // 连接到 MCU 的 INT1 GPIO。
  ICM456xx_irq_handler handler = NULL;             // ISR 回调；为空时返回 BAD_ARG。
  uint16_t threshold_x_mg = 52;                    // X 轴阈值，单位 mg。
  uint16_t threshold_y_mg = 52;                    // Y 轴阈值，单位 mg。
  uint16_t threshold_z_mg = 52;                    // Z 轴阈值，单位 mg。
  bool use_or_mode = true;                         // true: 任一轴过阈值触发；false: 三轴都过阈值触发。
  uint8_t duration_samples = 1;                    // 连续过阈值样本数，合法值 1..4。
  ICM45686AccelOdr accel_odr = ICM45686AccelOdr::Hz50; // WOM 推荐 50 Hz/20 ms 左右的低功耗采样。
};

// FIFO 配置。默认值保持旧 enableFifoInterrupt() 行为：accel+gyro、16 字节帧、snapshot、最大 FIFO。
struct ICM45686FifoConfig {
  bool enable_accel = true;                        // FIFO 中写入加速度数据。
  bool enable_gyro = true;                         // FIFO 中写入陀螺仪数据。
  bool high_resolution_20bit = false;              // true 时使用 20-bit 高分辨率 FIFO 格式。
  bool timestamp_or_fsync = true;                  // 普通六轴 FIFO 建议保留 timestamp；FSYNC 开启后该字段变为 FSYNC 计数。
  uint16_t watermark = 10;                         // FIFO watermark，单位 frame。
  ICM45686FifoMode mode = ICM45686FifoMode::Snapshot;
  ICM45686FifoDepth depth = ICM45686FifoDepth::Depth8KB;
  uint16_t accel_decimation = 1;                   // FIFO 抽取倍率：1/2/4/.../4096。
  uint16_t gyro_decimation = 1;                    // FIFO 抽取倍率：1/2/4/.../4096。
};

/*
 * ICM456xx Arduino 封装。
 *
 * 设计定位：
 * - 该类负责把 Arduino SPI/I2C 传输接入 InvenSense C driver。
 * - 返回值沿用底层 INV_IMU_OK(0) / 负数错误码，调用者必须检查返回值。
 * - begin() 会重新初始化底层 driver 状态；在切换 FIFO/APEX/WOM 等复杂模式前可再次调用 begin()
 *   清理 eDMP/中断配置残留。
 *
 * 飞控注意：
 * - 该类保留部分全局静态回调状态，适合单 IMU 使用。多 IMU 或飞控主链路建议使用
 *   ICM45686_FlightAdapter 包装，并避免多个 ICM456xx 实例并发访问。
 * - APEX/FIFO 会改写中断和 eDMP 配置，不应和普通寄存器轮询模式混用。
 */
class ICM456xx {
  public:
    // 使用 I2C 通信，address_lsb=false 对应 0x68，true 对应 0x69；freq 会被限制在 100 kHz 到 1 MHz。
    ICM456xx(TwoWire &i2c,bool address_lsb, uint32_t freq);
    // 使用 I2C 通信，默认 400 kHz。
    ICM456xx(TwoWire &i2c,bool address_lsb);
    // 使用 SPI 4-wire 通信，chip_select_id 为 CS GPIO，freq 会被限制在 100 kHz 到 24 MHz。
    ICM456xx(SPIClass &spi,uint8_t chip_select_id, uint32_t freq);
    // 使用 SPI 4-wire 通信，默认 6 MHz，SPI mode 3。
    ICM456xx(SPIClass &spi,uint8_t chip_select_id);

    // 初始化通信接口和底层 driver。返回 0 表示成功；负数通常表示参数、传输或芯片初始化失败。
    int begin();

    // 启动加速度计。odr 单位 Hz，fsr 单位 g；不支持的值会回退到底层默认枚举。
    int startAccel(uint16_t odr, uint16_t fsr);

    // 启动加速度计，使用强类型枚举精确表达 1.5625/3.125/6.25/12.5 Hz 等手册档位。
    int startAccel(ICM45686AccelOdr odr, ICM45686AccelFsr fsr);

    // 启动陀螺仪。odr 单位 Hz，fsr 单位 dps；不支持的值会回退到底层默认枚举。
    int startGyro(uint16_t odr, uint16_t fsr);

    // 启动陀螺仪，使用强类型枚举精确表达 15.625/31.25/62.5 dps 等手册档位。
    int startGyro(ICM45686GyroOdr odr, ICM45686GyroFsr fsr);

    // 从 UI 寄存器读取一帧原始 accel/gyro/temp。飞控轮询路径优先使用该函数或 FlightAdapter。
    int getDataFromRegisters(inv_imu_sensor_data_t& data);

    // ★ 2026-10-31 飞控接入新增：设置 UI 通路低通滤波带宽（LN 模式）。
    //   本库原先只在底层 C driver 暴露 inv_imu_set_gyro_ln_bw()/inv_imu_set_accel_ln_bw()
    //   （走 IPREG_SYS1_REG_172 / IPREG_SYS2_REG_131 间接寄存器），Arduino 层无公开入口，
    //   导致上层无法把 ICM45686 的滤波配成与 ICM42688 等效的档位。此处补公开封装。
    //   ★ 与 ICM42688 的语义差异：ICM42688 的 DLPF 是固定标称带宽（50/100/200/400Hz），
    //     而 ICM45686 的 GYRO_UI_LPFBW_SEL/ACCEL_UI_LPFBW_SEL 是 **ODR/2^n 倍率档**，
    //     因此实际截止频率随 ODR 变化。切换型号后滤波特性需重新评估，不能直接沿用调参结论。
    //   返回值：0 成功（INV_IMU_OK），负数为底层错误码。
    int setLowPassFilterGyro(ICM45686GyroLpfBw bw);
    int setLowPassFilterAccel(ICM45686AccelLpfBw bw);

    // 读取 WHO_AM_I。用于初始化后或周期性健康检查，ICM45686 预期值为 0xE9。
    int readWhoAmI(uint8_t& who_am_i);

    // 受控寄存器读取。主要用于诊断和按手册核对配置；普通应用不建议频繁直接访问寄存器。
    int readRegister(uint8_t reg, uint8_t& value);
    int readRegisters(uint8_t reg, uint8_t *buffer, uint32_t len);

    // 受控寄存器写入。writeRegisterChecked() 会写后读回 mask 覆盖的位，适合关键配置校验。
    int writeRegister(uint8_t reg, uint8_t value);
    int writeRegisterChecked(uint8_t reg, uint8_t value, uint8_t mask=0xFF);

    // 配置 FIFO 但不绑定 MCU 中断，适合轮询 FIFO_COUNT/FIFO_DATA 的场景。
    int configureFifo(const ICM45686FifoConfig& config);

    // 配置 FIFO 和 INT1 阈值中断。handler 不能为空；ISR 内只做置位/计数，不要读总线。
    int enableFifoInterrupt(uint8_t intpin, ICM456xx_irq_handler handler, uint8_t fifo_watermark);
    int enableFifoInterrupt(uint8_t intpin, ICM456xx_irq_handler handler, const ICM45686FifoConfig& config);

    // 从 FIFO 读取一个 frame。调用前必须先 enableFifoInterrupt() 并启动 accel/gyro。
    int getDataFromFifo(inv_imu_fifo_data_t& data);
    // 已知要读取多少个 frame 时，使用一次总线 burst 批量取出并在 MCU 本地解析。
    // 这个接口主要给高频 FIFO 导航测试用，目标是减少“每帧一次 SPI 读 FIFO_DATA”的事务开销。
    // 前提是 FIFO frame 配置在整个批次内保持不变；如果后续切到更复杂的 FIFO 头格式，
    // 需要同步检查 decode 逻辑是否仍与底层 InvenSense C driver 一致。
    int getDataFromFifoBatch(inv_imu_fifo_data_t *data, uint16_t frame_count);
    int getFifoFrameCount(uint16_t& frame_count);
    int flushFifo(void);

    // 配置 UI DRDY 中断，适合后续飞控用中断驱动寄存器读取。默认输出到 INT1。
    int configureDataReadyInterrupt(uint8_t intpin, ICM456xx_irq_handler handler,
                                    inv_imu_int_num_t int_num=INV_IMU_INT1);

#if INV_IMU_INT2_PIN_SUPPORTED
    // 配置 INT2 引脚用途：INT2/FSYNC/CLKIN/DRDY_INTR。AUX 场景会影响 INT2 可用性，使用前请对照硬件连接。
    int configureInt2PinUsage(ioc_pad_scenario_ovrd_pads_int2_cfg_ovrd_val_t usage);
#endif

#if INV_IMU_FSYNC_SUPPORTED
    // 配置并启用/关闭 FSYNC 标记。sensor_tag 决定 FSYNC 事件标记到哪一路 UI 数据 LSB。
    int configureFsync(fsync_config0_ap_fsync_sel_t sensor_tag, bool enable=true);
#endif

    // 执行芯片自检。自检会复位/重配芯片，完成后需要重新 begin()/startAccel()/startGyro()。
    int runSelfTest(inv_imu_selftest_output_t& output, bool accel_enable=true, bool gyro_enable=true);

    // 常用单位换算工具。寄存器路径按用户配置 FSR 缩放；20-bit FIFO 按手册固定 ±32g/±4000dps 缩放。
    static constexpr uint8_t womThresholdMgToReg(uint16_t mg) {
      return (uint8_t)((((uint32_t)mg * 256UL + 500UL) / 1000UL) > 255UL ?
          255UL : (((uint32_t)mg * 256UL + 500UL) / 1000UL));
    }
    static float accelFsrToG(ICM45686AccelFsr fsr);
    static float gyroFsrToDps(ICM45686GyroFsr fsr);
    static float rawRegisterAccelToG(int16_t raw, ICM45686AccelFsr fsr);
    static float rawRegisterGyroToDps(int16_t raw, ICM45686GyroFsr fsr);
    static float rawRegisterTempToC(int16_t raw);
    static float rawFifoTempToC(int8_t raw);
    static float rawFifo20AccelToG(int32_t raw);
    static float rawFifo20GyroToDps(int32_t raw);

#if (INV_DEVICE_TYPE == INV_TYPE_A2) || (INV_DEVICE_TYPE == INV_TYPE_B1) || (INV_DEVICE_TYPE == INV_TYPE_C1)
    int startGaf(uint8_t intpin, ICM456xx_irq_handler handler, uint8_t algo);
    int getGafData(inv_imu_edmp_gaf_outputs_t& gaf_outputs);
    int getGaf_GRVData(float& quatW,float& quatX,float& quatY,float& quatZ);
	int getGaf_GMRVData(float& quatW, float& quatX, float& quatY, float& quatZ) {
        float _dummy;
        return getGaf_GMRVData(quatW, quatX, quatY, quatZ, _dummy);
    }
    int getGaf_GMRVData(float& quatW,float& quatX,float& quatY,float& quatZ,float& accuracy);
	int getGaf_RVData(float& quatW, float& quatX, float& quatY, float& quatZ) {
        float _dummy;
        return getGaf_RVData(quatW, quatX, quatY, quatZ, _dummy);
    }
    int getGaf_RVData(float& quatW,float& quatX,float& quatY,float& quatZ,float& accuracy);
    int getGaf_RMData(float& mX,float& mY, float& mZ);
    int getGaf_BiasData(int type, int& q12_X,int& q12_Y,int& q12_Z,int& accuracy);
    int getCalibratedMag(float& mX, float& mY, float& mZ);
    int getCalibratedAccel(float& aX, float& aY, float& aZ);
    int getCalibratedGyro(float& gX, float& gY, float& gZ);
#endif

#if (INV_DEVICE_TYPE == INV_TYPE_B1)
    int startVocalVibDet(uint8_t intpin, ICM456xx_irq_handler handler);
    int SetVocalVibDet_Dynthresh(uint32_t thresh);
    int GetVocalVibDet_nb_samples(void);
    void SetVocalVibDet_nb_samples(uint32_t sample);
    int CheckVocalVibDet_thresh(void);
    int GetVocalVibDet_thresh(void);
    void SetVocalVibDet_thresh(uint32_t thresh);
#endif

#if (INV_DEVICE_TYPE == INV_TYPE_A1) || (INV_DEVICE_TYPE == INV_TYPE_B1) || (INV_DEVICE_TYPE == INV_TYPE_C1)
    // 高级 FIFO 解析入口，主要服务 eDMP/AUX 外部传感器路径。
    int adv_getDataFromFifo(void);
    // 初始化内部 I2C master，默认外部目标为 ICT1531/兼容磁力计地址 0x1E。
    int setI2CM(void);
    // AUX pass-through，仅适合明确知道外部传感器寄存器协议的调试场景。
    int setI2CMPassThrough(void);
    // 通过内部 I2C master 从外部磁力计读取 1 字节。
    int getDataFromI2CM(uint8_t reg, uint8_t& data);
    // pass-through 模式下读取外部传感器寄存器。
    int getDataFromPassThrough(uint8_t reg, uint8_t& data);
#endif
#if (INV_DEVICE_TYPE == INV_TYPE_A1)
    // 配置外部磁力计数据进入 FIFO ES0 字段；需要 AUX 硬件实际连接。
    int setI2CM_FIFO(uint8_t intpin, ICM456xx_irq_handler handler);
    // 读取高级 FIFO 输出的 accel/gyro/external 数据。
    int getAdvDataFromFifo(int32_t *accel, int32_t *gyro, float *external);
    // 主动读取外部磁力计数据，mag 单位约为 uT。
    int getExternalMagData(float *mag);
    // 检查外部磁力计 whoami。
    int initMag(void);
    // 使能或关闭外部磁力计。
    int enableMag(int flag);
#endif
    // 关闭加速度计。
    int stopAccel(void);
    // 关闭陀螺仪。
    int stopGyro(void);

    // APEX Tilt 检测。handler 为空时只置位功能，稍后可统一 setApexInterrupt()+startAPEX()。
    int startTiltDetection(uint8_t intpin=2, ICM456xx_irq_handler handler=NULL);
    // APEX 计步器。
    int startPedometer(uint8_t intpin=2, ICM456xx_irq_handler handler=NULL);
    // APEX 自由落体检测。
    int startFreeFall(uint8_t intpin=2, ICM456xx_irq_handler handler=NULL);
    // APEX 高 g 检测。
    int startHighG(uint8_t intpin=2, ICM456xx_irq_handler handler=NULL);
    // APEX 低 g 检测。
    int startLowG(uint8_t intpin=2, ICM456xx_irq_handler handler=NULL);
    // 读取计步器事件。返回 1 表示有新步态事件，0 表示无事件，负数表示错误。
    int getPedometer(uint32_t& step_count, float& step_cadence, char*& activity);
    // Wake on Motion 低功耗唤醒路径。会重配 accel 低功耗和 WOM 中断。
    int startWakeOnMotion(uint8_t intpin, ICM456xx_irq_handler handler, int odr=ACCEL_CONFIG0_ACCEL_ODR_50_HZ);
    int startWakeOnMotion(const ICM45686WomConfig& config);
    // APEX Tap 检测。
    int startTap(uint8_t intpin=2, ICM456xx_irq_handler handler=NULL);
    // APEX Raise to Wake。
    int startRaiseToWake(uint8_t intpin=2, ICM456xx_irq_handler handler=NULL);
    // B2S/AID 不是 ICM45686(A1) 主要路径，保留给兼容芯片族。
    int startB2S(uint8_t intpin=2, ICM456xx_irq_handler handler=NULL);
    int startAID(uint8_t intpin=2, ICM456xx_irq_handler handler=NULL);

    // APEX 事件读取函数。多数函数返回 1 表示事件发生，0 表示无事件，负数表示未启用或错误。
    int getTilt(void);
    int getTap(uint8_t& tap_count, uint8_t& axis, uint8_t& direction);
    int getRaiseToWake(void);
    int getFreefall(uint32_t& duration_ms);
    int getHighG(void);
    int getLowG(void);
    int getAID_Human(void);
    int getAID_Device(void);
    int getB2S(void);

    // 刷新 APEX 中断状态缓存。
    int updateApex(void);
    // 读取 INT1_STATUS0 并返回 INV_FIFO_FULL 位（0=正常，1=FIFO 已满/溢出）。
    // 注意：INT1_STATUS0 是 R/C 寄存器，读后自动清除；调后如需同时获取其他状态位请在 updateApex() 前读取。
    int readFifoFull(void);
    // 使能 INT1_STATUS_EN_FIFO_FULL（写 INT1_CONFIG0 bit 0），
    // 确保 readFifoFull() 能在 FIFO 满时读到 1。仅影响状态位跟踪，不触发物理中断。
    int enableFifoFullStatus(void);
    // 统一配置 APEX 中断输出到 INT1。
    int setApexInterrupt(uint8_t intpin, ICM456xx_irq_handler handler);
    // 根据已启用的 APEX 功能启动 eDMP；默认 AUTO 会自动选择所需 ODR。
    int startAPEX(dmp_ext_sen_odr_cfg_apex_odr_t edmp_odr=ICM456XX_APEX_ODR_AUTO,
                      accel_config0_accel_odr_t accel_odr=ICM456XX_ACCEL_ODR_AUTO);

    inv_imu_edmp_int_state_t apex_status;
  	TwoWire *i2c;
	uint8_t i2c_address;
	SPIClass *spi;
	uint8_t chip_select_id;
    uint32_t interface_clock_hz;

  protected:
    inv_imu_device_t icm_driver;
    accel_config0_accel_odr_t accel_freq_to_param(uint16_t accel_freq_hz);
    gyro_config0_gyro_odr_t gyro_freq_to_param(uint16_t gyro_freq_hz);
    accel_config0_accel_ui_fs_sel_t accel_fsr_g_to_param(uint16_t accel_fsr_g);
    gyro_config0_gyro_ui_fs_sel_t gyro_fsr_dps_to_param(uint16_t gyro_fsr_dps);
    int setup_irq(uint8_t intpin, ICM456xx_irq_handler handler);
    int setup_irq(uint8_t intpin, ICM456xx_irq_handler handler, inv_imu_int_num_t int_num,
                  const inv_imu_int_state_t& it_conf);
    uint32_t step_cnt_ovflw;
    bool apex_enable[ICM456XX_APEX_MAX];
    dmp_ext_sen_odr_cfg_apex_odr_t apex_edmp_odr;
    accel_config0_accel_odr_t apex_accel_odr;
#if (INV_DEVICE_TYPE == INV_TYPE_B1)
    int32_t	nb_samples_before_decision=0; /* To compute delay before notifying VVD window's end */
    int32_t	vvd_thresh=0;
#endif

};

#endif // ICM456xx_H
