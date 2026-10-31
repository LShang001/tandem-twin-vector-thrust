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
 
#include "Arduino.h"
#include "ICM45686.h"

#if (INV_DEVICE_TYPE != INV_TYPE_A2)
#include "invn_mag.h"
#include "Ict1531x/Ict1531x.h"
/* 
 * Soft-iron matrix applied to mag data in EDMP
 * Align mag axis to IMU
 */
static int32_t soft_iron_matrix[3][3] = {
	{ (1 << 30), 0, 0 },
	{ 0, (1 << 30), 0 },
	{ 0, 0, (1 << 30) },
};
#endif

#if (INV_DEVICE_TYPE == INV_TYPE_A1)
#include "imu/inv_imu_edmp_compass.h"
#include "imu/inv_imu_edmp_wearable.h"
#endif

#if (INV_DEVICE_TYPE == INV_TYPE_C1)
#include "imu/inv_imu_edmp_extended_features.h"
#endif

static int i2c_write(void *context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);
static int i2c_read(void *context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen);
static int spi_write(void *context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);
static int spi_read(void *context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen);
static void sleep_us(uint32_t us);
static void sensor_event_cb(inv_imu_sensor_event_t *event);
static int16_t decode_fifo_int16(uint8_t endianness_data, const uint8_t *src);
static void decode_fifo_frame_from_buffer(inv_imu_device_t *driver,
                                          const uint8_t *frame,
                                          inv_imu_fifo_data_t *data);

// C driver 通过回调访问 Arduino I2C/SPI。回调的 context 指向 ICM456xx 实例，
// 因此总线对象、片选和频率都应从实例读取，不再使用全局频率。
// I2C
#define I2C_DEFAULT_CLOCK 400000
#define I2C_MAX_CLOCK 1000000
#define ICM456xx_I2C_ADDRESS 0x68
#define ARDUINO_I2C_BUFFER_LENGTH 32
// SPI
#define SPI_READ 0x80
#define SPI_DEFAULT_CLOCK 6000000
#define SPI_MAX_CLOCK 24000000
#define ICM456XX_I2C_READ_TIMEOUT_MS 20
#define ICM456XX_I2CM_DONE_TIMEOUT_MS 50

// AUX/I2CM FIFO 解析回调使用的缓存。底层事件回调不是对象感知的，
// 因此这里仍是静态缓存；单 IMU 使用安全，多 IMU 并发需要进一步重构回调上下文。
static int32_t accel_data[3] = {0, 0, 0};
static int32_t gyro_data[3] = {0, 0, 0};
static float mag_data[3] = {0.0f, 0.0f, 0.0f};
// 批量 FIFO 读取复用的临时缓冲。
// 这里先把整批 FIFO_DATA 连续搬到 MCU RAM，再按 frame size 原地解码，
// 这样可以把“寄存器寻址 + 片选拉低/拉高”的固定 SPI 开销从每帧一次降到每批一次。
static uint8_t fifo_burst_buffer[FIFO_MIRRORING_SIZE];

// WOM 寄存器阈值单位是 1g/256，不是 mg。对外 API 统一使用 mg，再由 womThresholdMgToReg() 换算。
#define DEFAULT_WOM_THS_MG     52U
#define DEFAULT_WOM_AID_THS_MG 24U


#define Q11_DIV (1<<11)
#define Q14_DIV (1<<14)
#define Q16_DIV (1<<16)
#define Q30_DIV (1<<30)

#define ACCEL_FSR_G        (4)
#define RAW_ACC_SCALE      (ACCEL_FSR_G /* gee */ * 2 /* / 32768 * (1<<16)) */)
#define GYRO_FSR_DPS       (2000)
#define RAW_GYR_SCALE      (GYRO_FSR_DPS /* dps */ * 2 /* / (1<<15) * (1<<16)) */)
#define RAW_MAG_SCALE      0.075
#define RAW_MAG_SCALE_Q16  4915  /* 0.075 * (1 << 16) */

static odr_decimate_config_accel_fifo_odr_dec_t accel_decimation_to_param(uint16_t decimation);
static odr_decimate_config_gyro_fifo_odr_dec_t gyro_decimation_to_param(uint16_t decimation);
static tmst_wom_config_wom_int_dur_t wom_duration_to_param(uint8_t duration_samples);


#if (INV_DEVICE_TYPE == INV_TYPE_A2) || (INV_DEVICE_TYPE == INV_TYPE_B1) || (INV_DEVICE_TYPE == INV_TYPE_C1)
// GAF output aggregartion
static inv_imu_edmp_gaf_outputs_t gaf_outputs_internal = {0};
static int gaf_status = 0;
static int32_t acc_bias_q16[3] = { 0, 0, 0 }; //customer bias
#endif

// This is used by the getDataFromFifo callback (not object aware), declared static
static inv_imu_device_t *icm_driver_ptr = NULL;

// I2C 构造函数：显式清空 SPI 侧状态，避免 begin() 误判接口类型。
ICM456xx::ICM456xx(TwoWire &i2c_ref,bool lsb, uint32_t freq) {
  i2c = &i2c_ref; 
  spi = NULL;
  chip_select_id = 0;
  i2c_address = ICM456xx_I2C_ADDRESS | (lsb ? 0x1 : 0);
  if ((freq <= I2C_MAX_CLOCK) && (freq >= 100000))
  {
    interface_clock_hz = freq;
  } else {
    interface_clock_hz = I2C_DEFAULT_CLOCK;
  }
}

// I2C 构造函数，使用 400 kHz 默认频率。
ICM456xx::ICM456xx(TwoWire &i2c_ref,bool lsb) {
  i2c = &i2c_ref; 
  spi = NULL;
  chip_select_id = 0;
  i2c_address = ICM456xx_I2C_ADDRESS | (lsb ? 0x1 : 0);
  interface_clock_hz = I2C_DEFAULT_CLOCK;
}

// SPI 构造函数：显式清空 I2C 侧状态，避免 begin() 误走 I2C。
ICM456xx::ICM456xx(SPIClass &spi_ref,uint8_t cs_id, uint32_t freq) {
  i2c = NULL;
  i2c_address = ICM456xx_I2C_ADDRESS;
  spi = &spi_ref;
  chip_select_id = cs_id; 
  if ((freq <= SPI_MAX_CLOCK) && (freq >= 100000))
  {
    interface_clock_hz = freq;
  } else {
    interface_clock_hz = SPI_DEFAULT_CLOCK;
  }
}

// SPI 构造函数，使用 6 MHz 默认频率。
ICM456xx::ICM456xx(SPIClass &spi_ref,uint8_t cs_id) {
  i2c = NULL;
  i2c_address = ICM456xx_I2C_ADDRESS;
  spi = &spi_ref;
  chip_select_id = cs_id; 
  interface_clock_hz = SPI_DEFAULT_CLOCK;
}

/*
 * 初始化芯片和底层 C driver。
 *
 * 关键步骤：
 * 1. 根据构造函数留下的 i2c/spi 指针判断当前通信接口。
 * 2. 把 Arduino 层读写函数注册给 InvenSense C driver。
 * 3. 清空 APEX 功能开关，避免上一次模式残留影响下一次初始化。
 * 4. 注册 FIFO/eDMP 事件回调并调用 inv_imu_adv_init() 完成 WHO_AM_I、复位和默认配置。
 *
 * 返回值：
 * - INV_IMU_OK(0)：初始化成功。
 * - INV_IMU_ERROR_BAD_ARG：构造对象没有有效 I2C/SPI 接口。
 * - 其他负数：底层通信、WHO_AM_I 或复位流程失败。
 */
int ICM456xx::begin() {
  int rc = 0;

  if (i2c != NULL) {
    i2c->begin();
    i2c->setClock(interface_clock_hz);
    icm_driver.transport.serif_type = UI_I2C;
    icm_driver.transport.read_reg  = i2c_read;
    icm_driver.transport.write_reg = i2c_write;
  } else if (spi != NULL) {
    spi->begin();
    pinMode(chip_select_id,OUTPUT);
    digitalWrite(chip_select_id,HIGH);
    icm_driver.transport.serif_type = UI_SPI4;
    icm_driver.transport.read_reg  = spi_read;
    icm_driver.transport.write_reg = spi_write;
  } else {
    return INV_IMU_ERROR_BAD_ARG;
  }

  icm_driver.transport.context	= (void *)this;
  icm_driver.transport.sleep_us = sleep_us;
  
  /* Disable APEX features */
  for(int i=0; i < ICM456XX_APEX_MAX; i++)
  {
    apex_enable[i] = false;
  }
  /* Set ODR to the minimum, APEX init will select the highest required ODR */
  apex_edmp_odr = DMP_EXT_SEN_ODR_CFG_APEX_ODR_25_HZ;
  apex_accel_odr = ACCEL_CONFIG0_ACCEL_ODR_1_5625_HZ;
  
  /* Init sensor event callback */
  ((inv_imu_adv_var_t *)&icm_driver.adv_var)->sensor_event_cb = sensor_event_cb;
  icm_driver_ptr = &icm_driver;

  sleep_us(3000);

  return inv_imu_adv_init(&icm_driver);
}

int ICM456xx::startAccel(uint16_t odr, uint16_t fsr) {
  int rc = 0;
  // 兼容旧示例的整数配置入口：不支持的 ODR/FSR 会在映射函数中回退到默认档位。
  rc |= inv_imu_set_accel_fsr(&icm_driver, accel_fsr_g_to_param(fsr));
  rc |= inv_imu_set_accel_frequency(&icm_driver, accel_freq_to_param(odr));
  // 飞控/高速读取默认使用低噪声 LN 模式；低功耗 WOM 另有专用路径。
  rc |= inv_imu_set_accel_mode(&icm_driver, PWR_MGMT0_ACCEL_MODE_LN);
  return rc;
}

int ICM456xx::startAccel(ICM45686AccelOdr odr, ICM45686AccelFsr fsr) {
  int rc = 0;
  // 强类型入口直接写入手册枚举值，避免 12.5 Hz 被整数 API 表达成 12 的歧义。
  rc |= inv_imu_set_accel_fsr(&icm_driver, (accel_config0_accel_ui_fs_sel_t)fsr);
  rc |= inv_imu_set_accel_frequency(&icm_driver, (accel_config0_accel_odr_t)odr);
  rc |= inv_imu_set_accel_mode(&icm_driver, PWR_MGMT0_ACCEL_MODE_LN);
  return rc;
}

int ICM456xx::startGyro(uint16_t odr, uint16_t fsr) {
  int rc = 0;
  // 兼容旧示例的整数配置入口：15/31/62 分别代表 15.625/31.25/62.5 dps。
  rc |= inv_imu_set_gyro_fsr(&icm_driver, gyro_fsr_dps_to_param(fsr));
  rc |= inv_imu_set_gyro_frequency(&icm_driver, gyro_freq_to_param(odr));
  rc |= inv_imu_set_gyro_mode(&icm_driver, PWR_MGMT0_GYRO_MODE_LN);
  return rc;
}

int ICM456xx::startGyro(ICM45686GyroOdr odr, ICM45686GyroFsr fsr) {
  int rc = 0;
  // 强类型入口直接传递底层寄存器枚举，适合高频读取和飞控参数固定化。
  rc |= inv_imu_set_gyro_fsr(&icm_driver, (gyro_config0_gyro_ui_fs_sel_t)fsr);
  rc |= inv_imu_set_gyro_frequency(&icm_driver, (gyro_config0_gyro_odr_t)odr);
  rc |= inv_imu_set_gyro_mode(&icm_driver, PWR_MGMT0_GYRO_MODE_LN);
  return rc;
}

int ICM456xx::stopAccel(void) {
  return inv_imu_set_accel_mode(&icm_driver, PWR_MGMT0_ACCEL_MODE_OFF);
}

int ICM456xx::stopGyro(void) {
  return inv_imu_set_gyro_mode(&icm_driver, PWR_MGMT0_GYRO_MODE_OFF);
}

int ICM456xx::getDataFromRegisters(inv_imu_sensor_data_t& data) {
    // UI 寄存器路径固定读取 accel/gyro/temp，延迟低、状态少，是飞控主循环的首选起点。
    return inv_imu_get_register_data(&icm_driver, &data);
}

int ICM456xx::readWhoAmI(uint8_t& who_am_i) {
  return inv_imu_get_who_am_i(&icm_driver, &who_am_i);
}

// ★ 2026-10-31 飞控接入新增：UI 通路 LPF 带宽设置。
//
// 手册依据 DS-000489 v1.1：
//   - GYRO_UI_LPFBW_SEL  → IPREG_SYS1_REG_172 (0xA4AC) bit[2:0]
//   - ACCEL_UI_LPFBW_SEL → IPREG_SYS2_REG_131 (0xA583) bit[2:0]
// 这两个是间接寄存器（16-bit MREG 地址），必须经 IREG_ADDR_15_8/IREG_ADDR_7_0
// 窗口访问；底层 inv_imu_read_reg/inv_imu_write_reg 会按地址 >0xFF 自动路由到
// read_mreg/write_mreg，并先做 AN-000364 禁访区检查。这里直接复用官方 driver 的
// inv_imu_set_gyro_ln_bw()/inv_imu_set_accel_ln_bw()，不自己拼地址序列。
//
// 读-改-写语义：两个函数内部都是「读出整个 IPREG 字节 → 只改 LPFBW 位域 → 写回」，
// 不会破坏同一寄存器里其它位（如 GYRO_OIS_HPF1_BYP）。这也是不能直接用
// writeRegister() 写死一个常量字节的原因。
int ICM456xx::setLowPassFilterGyro(ICM45686GyroLpfBw bw) {
  return inv_imu_set_gyro_ln_bw(
      &icm_driver, (ipreg_sys1_reg_172_gyro_ui_lpfbw_sel_t)bw);
}

int ICM456xx::setLowPassFilterAccel(ICM45686AccelLpfBw bw) {
  return inv_imu_set_accel_ln_bw(
      &icm_driver, (ipreg_sys2_reg_131_accel_ui_lpfbw_t)bw);
}

int ICM456xx::readRegister(uint8_t reg, uint8_t& value) {
  return readRegisters(reg, &value, 1);
}

int ICM456xx::readRegisters(uint8_t reg, uint8_t *buffer, uint32_t len) {
  if ((buffer == NULL) || (len == 0)) {
    return INV_IMU_ERROR_BAD_ARG;
  }
  // 只暴露 8-bit UI 寄存器直接读取，避免上层随意访问间接寄存器危险区域。
  return inv_imu_read_reg(&icm_driver.transport, reg, len, buffer);
}

int ICM456xx::writeRegister(uint8_t reg, uint8_t value) {
  return inv_imu_write_reg(&icm_driver.transport, reg, 1, &value);
}

int ICM456xx::writeRegisterChecked(uint8_t reg, uint8_t value, uint8_t mask) {
  // 先写入目标值，再读回 mask 覆盖的 bit；SPI 本身无法报告断线时，这类校验尤其重要。
  int rc = writeRegister(reg, value);
  if (rc != INV_IMU_OK) {
    return rc;
  }

  uint8_t readback = 0;
  rc = readRegister(reg, readback);
  if (rc != INV_IMU_OK) {
    return rc;
  }

  return ((readback & mask) == (value & mask)) ? INV_IMU_OK : INV_IMU_ERROR;
}

int ICM456xx::setup_irq(uint8_t intpin, ICM456xx_irq_handler handler,
                        inv_imu_int_num_t int_num, const inv_imu_int_state_t& it_conf)
{
  int rc = 0;
  // 默认使用高有效、脉冲、推挽输出，和诊断台/ESP32 GPIO 中断配置保持一致。
  const inv_imu_int_pin_config_t it_pins = {
    .int_polarity=INTX_CONFIG2_INTX_POLARITY_HIGH,
    .int_mode=INTX_CONFIG2_INTX_MODE_PULSE,
    .int_drive=INTX_CONFIG2_INTX_DRIVE_PP
  };
  if(handler == NULL) {
    return INV_IMU_ERROR_BAD_ARG;
  }
  pinMode(intpin,INPUT);
  // 先配置芯片内部 INTx 引脚，再配置中断源，最后才把 MCU GPIO 绑定到 ISR。
  rc |= inv_imu_set_pin_config_int(&icm_driver, int_num, &it_pins);
  rc |= inv_imu_set_config_int(&icm_driver, int_num, &it_conf);
  attachInterrupt(intpin,handler,HIGH);
  return rc;
}

int ICM456xx::setup_irq(uint8_t intpin, ICM456xx_irq_handler handler)
{
  inv_imu_int_state_t it_conf;
  memset(&it_conf, INV_IMU_DISABLE, sizeof(it_conf));
  it_conf.INV_FIFO_THS = INV_IMU_ENABLE;
  return setup_irq(intpin, handler, INV_IMU_INT1, it_conf);
}

static int inv_init_i2cm_bypass(inv_imu_device_t *s)
{
  int status = INV_IMU_OK;
  ioc_pad_scenario_aux_ovrd_t ioc_pad_scenario_aux_ovrd;

  // 强制 AUX1 进入 I2CM master 模式；pass-through 调试前必须先让引脚复用正确。
  status |=
    inv_imu_read_reg(s, IOC_PAD_SCENARIO_AUX_OVRD, 1, (uint8_t *)&ioc_pad_scenario_aux_ovrd);
  ioc_pad_scenario_aux_ovrd.aux1_mode_ovrd     = 1;
  ioc_pad_scenario_aux_ovrd.aux1_mode_ovrd_val = 2;
  status |=
    inv_imu_write_reg(s, IOC_PAD_SCENARIO_AUX_OVRD, 1, (uint8_t *)&ioc_pad_scenario_aux_ovrd);

  return status;
}

int ICM456xx::configureFifo(const ICM45686FifoConfig& config) {
  int rc = 0;
  // Arduino 层配置结构体只保留对用户有意义的字段，这里展开到底层 advanced FIFO 配置。
  const inv_imu_adv_fifo_config_t fifo_config = {
    .base_conf = {
      .gyro_en = (uint8_t)(config.enable_gyro ? INV_IMU_ENABLE : INV_IMU_DISABLE),
      .accel_en = (uint8_t)(config.enable_accel ? INV_IMU_ENABLE : INV_IMU_DISABLE),
      .hires_en = (uint8_t)(config.high_resolution_20bit ? INV_IMU_ENABLE : INV_IMU_DISABLE),
      .fifo_wm_th = config.watermark,
      .fifo_mode = (fifo_config0_fifo_mode_t)config.mode,
      .fifo_depth = (fifo_config0_fifo_depth_t)config.depth
    },
    // watermark 中断在 FIFO frame 数达到或超过阈值时触发，便于主循环批量读取。
    .fifo_wr_wm_gt_th = FIFO_CONFIG2_FIFO_WR_WM_EQ_OR_GT_TH,
    .tmst_fsync_en = (uint8_t)(config.timestamp_or_fsync ? INV_IMU_ENABLE : INV_IMU_DISABLE),
    // 普通六轴 FIFO 不启用外部传感器 ES0/ES1；AUX 磁力计路径由 setI2CM_FIFO() 单独配置。
    .es1_en = INV_IMU_DISABLE,
    .es0_en = INV_IMU_DISABLE,
    .es0_6b_9b = FIFO_CONFIG4_FIFO_ES0_6B,
    // 压缩 FIFO 会增加解析复杂度，飞控 bring-up 阶段默认关闭，优先保证 frame 格式稳定。
    .comp_en = INV_IMU_DISABLE,
    .comp_nc_flow_cfg = FIFO_CONFIG4_FIFO_COMP_NC_FLOW_CFG_DIS,
    .gyro_dec = gyro_decimation_to_param(config.gyro_decimation),
    .accel_dec = accel_decimation_to_param(config.accel_decimation)
  };

  if ((!config.enable_accel && !config.enable_gyro) || (config.watermark == 0)) {
    return INV_IMU_ERROR_BAD_ARG;
  }

  rc |= inv_imu_adv_set_fifo_config(&icm_driver, &fifo_config);
  return rc;
}

int ICM456xx::enableFifoInterrupt(uint8_t intpin, ICM456xx_irq_handler handler, uint8_t fifo_watermark) {
  // 旧接口只允许传 watermark；其余字段使用 ICM45686FifoConfig 默认值。
  ICM45686FifoConfig config;
  config.watermark = fifo_watermark;
  return enableFifoInterrupt(intpin, handler, config);
}

int ICM456xx::enableFifoInterrupt(uint8_t intpin, ICM456xx_irq_handler handler, const ICM45686FifoConfig& config) {
  int rc = 0;
  inv_imu_int_state_t it_conf;
  if(handler == NULL) {
    return INV_IMU_ERROR_BAD_ARG;
  }

  rc |= configureFifo(config);
  // FIFO 中断只打开 FIFO_THS，避免同时混入 DRDY/APEX/WOM 状态。
  memset(&it_conf, INV_IMU_DISABLE, sizeof(it_conf));
  it_conf.INV_FIFO_THS = INV_IMU_ENABLE;
  rc |= setup_irq(intpin, handler, INV_IMU_INT1, it_conf);
  return rc;
}


int ICM456xx::getDataFromFifo(inv_imu_fifo_data_t& data) {
  return inv_imu_get_fifo_frame(&icm_driver,&data);
}

int ICM456xx::getDataFromFifoBatch(inv_imu_fifo_data_t *data, uint16_t frame_count) {
  if ((data == NULL) || (frame_count == 0U)) {
    return INV_IMU_ERROR_BAD_ARG;
  }

  if ((icm_driver.fifo_frame_size == 0U) || (icm_driver.fifo_frame_size > 20U)) {
    return INV_IMU_ERROR;
  }

  const uint32_t bytes_to_read =
      static_cast<uint32_t>(frame_count) * icm_driver.fifo_frame_size;
  if (bytes_to_read > FIFO_MIRRORING_SIZE) {
    return INV_IMU_ERROR_BAD_ARG;
  }

  int rc = inv_imu_read_reg(&icm_driver, FIFO_DATA, bytes_to_read, fifo_burst_buffer);
  if (rc != INV_IMU_OK) {
    return rc;
  }

  for (uint16_t idx = 0; idx < frame_count; ++idx) {
    decode_fifo_frame_from_buffer(&icm_driver,
                                  &fifo_burst_buffer[idx * icm_driver.fifo_frame_size],
                                  &data[idx]);
  }
  return INV_IMU_OK;
}

int ICM456xx::getFifoFrameCount(uint16_t& frame_count) {
  return inv_imu_get_frame_count(&icm_driver, &frame_count);
}

int ICM456xx::flushFifo(void) {
  return inv_imu_flush_fifo(&icm_driver);
}

int ICM456xx::configureWom(const ICM45686WomConfig& config) {
  if (config.duration_samples < 1U || config.duration_samples > 4U) {
    return INV_IMU_ERROR_BAD_ARG;
  }
  // WOM 在低功耗下工作：先用推荐 ODR 启动加计（LN 模式），再写阈值/时长。
  int rc = startAccel(config.accel_odr, ICM45686AccelFsr::G4);
  const uint8_t x = womThresholdMgToReg(config.threshold_x_mg);
  const uint8_t y = womThresholdMgToReg(config.threshold_y_mg);
  const uint8_t z = womThresholdMgToReg(config.threshold_z_mg);
  const tmst_wom_config_wom_int_mode_t mode =
      config.use_or_mode ? TMST_WOM_CONFIG_WOM_INT_MODE_ORED
                         : TMST_WOM_CONFIG_WOM_INT_MODE_ANDED;
  const tmst_wom_config_wom_int_dur_t dur =
      static_cast<tmst_wom_config_wom_int_dur_t>(config.duration_samples - 1U);
  rc |= inv_imu_adv_configure_wom(&icm_driver, x, y, z, mode, dur);
  return rc;
}

int ICM456xx::enableWom() {
  return inv_imu_adv_enable_wom(&icm_driver);
}

int ICM456xx::disableWom() {
  return inv_imu_adv_disable_wom(&icm_driver);
}

int ICM456xx::readWomStatus(bool& x, bool& y, bool& z) {
  int1_status1_t st = {};
  const int rc = inv_imu_read_reg(&icm_driver, INT1_STATUS1, 1, (uint8_t *)&st);
  x = st.int1_status_wom_x != 0U;
  y = st.int1_status_wom_y != 0U;
  z = st.int1_status_wom_z != 0U;
  return rc;
}

int ICM456xx::enablePedometer() {
  return inv_imu_edmp_enable_pedometer(&icm_driver);
}

int ICM456xx::disablePedometer() {
  return inv_imu_edmp_disable_pedometer(&icm_driver);
}

int ICM456xx::enableTilt() {
  return inv_imu_edmp_enable_tilt(&icm_driver);
}

int ICM456xx::disableTilt() {
  return inv_imu_edmp_disable_tilt(&icm_driver);
}

int ICM456xx::readApexStatus(inv_imu_edmp_int_state_t& status) {
  return inv_imu_edmp_get_int_apex_status(&icm_driver, &status);
}

int ICM456xx::readPedometerSteps(uint16_t& steps) {
  // 手册 DS-000577：APEX_DATA0(0x31)=STEP_CNT[7:0]、APEX_DATA1(0x32)=STEP_CNT[15:8]。
  constexpr uint8_t kApexData0Reg = 0x31;
  uint8_t buf[2] = {0U, 0U};
  const int rc = inv_imu_read_reg(&icm_driver, kApexData0Reg, 2, buf);
  steps = static_cast<uint16_t>(buf[0] | (static_cast<uint16_t>(buf[1]) << 8U));
  return rc;
}

int ICM456xx::configureDataReadyInterrupt(uint8_t intpin, ICM456xx_irq_handler handler,
                                          inv_imu_int_num_t int_num) {
  inv_imu_int_state_t it_conf;
  // DRDY 中断用于“有新 UI 寄存器数据”提示，ISR 内只置位，主循环再调用 getDataFromRegisters()。
  memset(&it_conf, INV_IMU_DISABLE, sizeof(it_conf));
  it_conf.INV_UI_DRDY = INV_IMU_ENABLE;
  return setup_irq(intpin, handler, int_num, it_conf);
}

#if INV_IMU_INT2_PIN_SUPPORTED
int ICM456xx::configureInt2PinUsage(ioc_pad_scenario_ovrd_pads_int2_cfg_ovrd_val_t usage) {
  return inv_imu_adv_set_int2_pin_usage(&icm_driver, usage);
}
#endif

#if INV_IMU_FSYNC_SUPPORTED
int ICM456xx::configureFsync(fsync_config0_ap_fsync_sel_t sensor_tag, bool enable) {
  int rc = 0;
  // sensor_tag 决定 FSYNC 事件嵌入哪一路 UI 数据 LSB；具体选择需要结合上层同步方案。
  rc |= inv_imu_adv_configure_fsync_ap_tag(&icm_driver, sensor_tag);
  rc |= enable ? inv_imu_adv_enable_fsync(&icm_driver) : inv_imu_adv_disable_fsync(&icm_driver);
  return rc;
}
#endif

int ICM456xx::runSelfTest(inv_imu_selftest_output_t& output, bool accel_enable, bool gyro_enable) {
  inv_imu_selftest_parameters_t params;
  // 先从底层获取默认自检参数，再按调用者选择打开 accel/gyro。
  int rc = inv_imu_selftest_init_params(&icm_driver, &params);
  if (rc != INV_IMU_OK) {
    // 初始化参数失败时清空输出，避免上层误读旧结果。
    memset(&output, 0, sizeof(output));
    return rc;
  }
  params.accel_en = accel_enable ? INV_IMU_ENABLE : INV_IMU_DISABLE;
  params.gyro_en = gyro_enable ? INV_IMU_ENABLE : INV_IMU_DISABLE;
  return inv_imu_selftest(&icm_driver, &params, &output);
}

#if (INV_DEVICE_TYPE == INV_TYPE_A2) || (INV_DEVICE_TYPE == INV_TYPE_B1) || (INV_DEVICE_TYPE == INV_TYPE_C1)
int ICM456xx::startGaf(uint8_t intpin, ICM456xx_irq_handler handler, uint8_t algo)
{
  int rc = 0;
  int gyro_is_on = 1;
  int mag_is_on = 0;  
  inv_imu_edmp_gaf_parameters_t gaf_params;
  inv_imu_edmp_int_state_t apex_int_config;
  //int8_t clk_variation = 0;
  inv_imu_adv_fifo_config_t fifo_config = {
    .base_conf = {
      .gyro_en    = INV_IMU_DISABLE, // raw data are read from sensor registers
      .accel_en   = INV_IMU_DISABLE, // raw data are read from sensor registers      .fifo_mode  = FIFO_CONFIG0_FIFO_MODE_SNAPSHOT,
      .hires_en   = INV_IMU_DISABLE,
      .fifo_wm_th = 4,
      .fifo_mode = FIFO_CONFIG0_FIFO_MODE_SNAPSHOT,
    },
    .fifo_wr_wm_gt_th     = FIFO_CONFIG2_FIFO_WR_WM_EQ_OR_GT_TH,
    .tmst_fsync_en        = INV_IMU_DISABLE,
    .es1_en               = INV_IMU_DISABLE,
    .es0_en               = INV_IMU_ENABLE,
    .es0_6b_9b            = FIFO_CONFIG4_FIFO_ES0_9B,
    .comp_en              = INV_IMU_DISABLE,
    .comp_nc_flow_cfg     = FIFO_CONFIG4_FIFO_COMP_NC_FLOW_CFG_DIS,
    .gyro_dec             = ODR_DECIMATE_CONFIG_GYRO_FIFO_ODR_DEC_1,
    .accel_dec            = ODR_DECIMATE_CONFIG_ACCEL_FIFO_ODR_DEC_1
  };

  stopAccel();
  stopGyro();
  rc = inv_imu_edmp_disable(&icm_driver);

#if (INV_DEVICE_TYPE == INV_TYPE_C1)
  if(algo == ALGO_GRV) // enable GRV when enable 6-axis(AG)
  { 
    mag_is_on = 0;
  }
  else if(algo == ALGO_GMRV) // enable GMRV when enable 6-axis(AM)
  {
    mag_is_on = 1;
	gyro_is_on = 0;
  }
  else if(algo == ALGO_RV) // enable RV when enable 9-axis(AGM)
  {
    mag_is_on = 1;
	gyro_is_on = 1;
  }

  if (mag_is_on) {
  	rc = invn_mag_init(&icm_driver);

    if (rc != 0)
	{
      mag_is_on = 0;
	  return rc;
    }
  }
#endif

  memset(&gaf_outputs_internal, 0, sizeof(gaf_outputs_internal));
  
  rc |= inv_imu_edmp_set_frequency(&icm_driver, DMP_EXT_SEN_ODR_CFG_APEX_ODR_100_HZ);

#if (INV_DEVICE_TYPE == INV_TYPE_A2)
  fifo_config.base_conf.fifo_depth = FIFO_CONFIG0_FIFO_DEPTH_GAF;
  fifo_config.base_conf.fifo_wm_th = 1;
  fifo_config.es1_en			   = INV_IMU_ENABLE;
  fifo_config.es0_en			   = INV_IMU_ENABLE;

  /* Load APEX algorithm GAF in DMP RAM */
  rc |= inv_imu_edmp_gaf_init(&icm_driver);
    
  /* Configure GAF parameters */
  rc |= inv_imu_edmp_gaf_init_parameters(&icm_driver, &gaf_params);

  //gaf_params.clock_variation = clk_variation;
  gaf_params.pdr_us          = 10000;
  rc |= inv_imu_edmp_gaf_set_parameters(&icm_driver, &gaf_params);
#else
  fifo_config.base_conf.gyro_en    = INV_IMU_ENABLE;
  fifo_config.base_conf.accel_en   = INV_IMU_ENABLE;
  fifo_config.base_conf.fifo_depth = FIFO_CONFIG0_FIFO_DEPTH_APEX;
  fifo_config.base_conf.fifo_wm_th = 1;
  fifo_config.es1_en			   = INV_IMU_ENABLE;
  fifo_config.es0_en               = INV_IMU_ENABLE;
  fifo_config.tmst_fsync_en 	   = INV_IMU_ENABLE;
  fifo_config.es0_6b_9b            = FIFO_CONFIG4_FIFO_ES0_9B;

  rc |= inv_imu_edmp_init(&icm_driver);
  rc |= inv_imu_edmp_get_gaf_parameters(&icm_driver, &gaf_params);
#if (INV_DEVICE_TYPE == INV_TYPE_C1)
  gaf_params.pdr_us = 20000;
  gaf_params.run_spherical = true;
  if (mag_is_on)
    gaf_params.mag_dt_us = 20000;
  else
    gaf_params.mag_dt_us = 0;
#else
  gaf_params.pdr_us	= 10000;
#endif

  int32_t acc_bias_q16[3]= {0,0,0};
  int16_t gyr_bias_q16[3]= {0,0,0};
  int32_t mag_bias_q16[3]= {0,0,0};

  rc |= inv_imu_edmp_set_frequency(&icm_driver, DMP_EXT_SEN_ODR_CFG_APEX_ODR_100_HZ);
  rc |= inv_imu_set_accel_frequency(&icm_driver, ACCEL_CONFIG0_ACCEL_ODR_100_HZ);
  rc |= inv_imu_set_gyro_frequency(&icm_driver, GYRO_CONFIG0_GYRO_ODR_100_HZ);
  rc |= inv_imu_edmp_set_gaf_acc_bias(&icm_driver, acc_bias_q16);
  rc |= inv_imu_edmp_set_gaf_gyr_bias(&icm_driver, gyr_bias_q16, 0, 0);
  rc |= inv_imu_edmp_set_gaf_parameters(&icm_driver, &gaf_params);
#if (INV_DEVICE_TYPE == INV_TYPE_C1)
  rc |= inv_imu_edmp_set_gaf_mag_bias(&icm_driver, mag_bias_q16, 0);
  rc |= inv_imu_edmp_set_gaf_mode(&icm_driver, gyro_is_on, mag_is_on);

  if (mag_is_on)
  {
	/* Configure properly soft iron matrix in eDMP image */
	rc |= inv_imu_edmp_set_gaf_soft_iron_cor_matrix(&icm_driver, soft_iron_matrix);
	rc |= inv_imu_edmp_enable_gaf_soft_iron_cor(&icm_driver);
  }

  if(!gyro_is_on) // For GMRV
  {
    rc |= inv_imu_set_accel_mode(&icm_driver, PWR_MGMT0_ACCEL_MODE_LP);
	rc |= inv_imu_select_accel_lp_clk(&icm_driver, SMC_CONTROL_0_ACCEL_LP_CLK_RCOSC);
  }
  
  rc |= invn_mag_load_ram_image(&icm_driver, INVN_MAG_USECASE_IMG_OVER_SIF);
  rc |= inv_imu_edmp_start_gaf_fifo_push(&icm_driver);
#endif
  rc |= inv_imu_adv_disable_wom(&icm_driver);
  rc |= inv_imu_adv_reset_fifo(&icm_driver);
#endif

  if (rc != 0) {
    return rc;
  }
  rc |= startAccel(100,8);
  if(gyro_is_on)
    rc |= startGyro(100,2000);
  sleep_us(GYR_STARTUP_TIME_US);
  
  /* Only 2k FIFO is supported if GAF RAM image is loaded */
  rc |= inv_imu_adv_set_fifo_config(&icm_driver, &fifo_config);
  rc |= setup_irq(intpin,handler);

#if (INV_DEVICE_TYPE == INV_TYPE_A2)
  rc |= inv_imu_edmp_gaf_enable(&icm_driver);
#else
  rc |= inv_imu_edmp_enable_gaf(&icm_driver);
#endif
  rc |= inv_imu_edmp_enable(&icm_driver);
  return rc;
}

int ICM456xx::getGafData(inv_imu_edmp_gaf_outputs_t& gaf_outputs)
{
  int rc = 0;
  uint16_t fifo_count;
  uint8_t fifo_data[FIFO_MIRRORING_SIZE]; /* Memory where to store FIFO data */
  uint8_t count = 0;
  while((gaf_status != 1)&&(rc==0)&&(count++ <100))
  {
    rc |= inv_imu_adv_get_data_from_fifo(&icm_driver, fifo_data, &fifo_count);
    rc |= inv_imu_adv_parse_fifo_data(&icm_driver, fifo_data, fifo_count);
  }
  if(gaf_status == 1)
  {
    memcpy(&gaf_outputs, &gaf_outputs_internal, sizeof(inv_imu_edmp_gaf_outputs_t));
    gaf_status = 0;
  } else {
    memset(&gaf_outputs, 0, sizeof(inv_imu_edmp_gaf_outputs_t));
    return -1;
  }

  return 0;
}

int ICM456xx::getGaf_GRVData(float& quatW, float& quatX, float& quatY, float& quatZ)
{
  int rc = 0;
  inv_imu_edmp_gaf_outputs_t gaf_outputs;
  rc = getGafData(gaf_outputs);

#if (INV_DEVICE_TYPE == INV_TYPE_C1)
  if(gaf_outputs.grv_quat_valid && !rc)
    rc = 0;
#endif

#if (INV_DEVICE_TYPE == INV_TYPE_A2) || (INV_DEVICE_TYPE == INV_TYPE_B1) || (INV_DEVICE_TYPE == INV_TYPE_C1)
  if(rc == 0) {
    quatW = (float)gaf_outputs.grv_quat_q14[0] / (float)Q14_DIV;
    quatX = (float)gaf_outputs.grv_quat_q14[1] / (float)Q14_DIV;
    quatY = (float)gaf_outputs.grv_quat_q14[2] / (float)Q14_DIV;
    quatZ = (float)gaf_outputs.grv_quat_q14[3] / (float)Q14_DIV;
  } 
#else
  if(rc == 0) {
    quatW = (float)gaf_outputs.grv_quat_q30[0] / (float)Q30_DIV;
    quatX = (float)gaf_outputs.grv_quat_q30[1] / (float)Q30_DIV;
    quatY = (float)gaf_outputs.grv_quat_q30[2] / (float)Q30_DIV;
    quatZ = (float)gaf_outputs.grv_quat_q30[3] / (float)Q30_DIV;
  }
#endif
  else {
    quatW = 0;
    quatX = 0;
    quatY = 0;
    quatZ = 0;
  }
  return rc;
}

int ICM456xx::getGaf_GMRVData(float& quatW,float& quatX,float& quatY,float& quatZ, float& accuracy)
{
#if (INV_DEVICE_TYPE == INV_TYPE_C1)
  inv_imu_edmp_gaf_outputs_t gaf_outputs;

  accuracy = 0;
  getGafData(gaf_outputs);
  
  if(gaf_outputs.gmrv_quat_valid)
  {
    quatW = (int32_t)gaf_outputs.gmrv_quat_q14[0] / (float)Q14_DIV;
    quatX = (int32_t)gaf_outputs.gmrv_quat_q14[1] / (float)Q14_DIV;
    quatY = (int32_t)gaf_outputs.gmrv_quat_q14[2] / (float)Q14_DIV;
    quatZ = (int32_t)gaf_outputs.gmrv_quat_q14[3] / (float)Q14_DIV;
	if(gaf_outputs.gmrv_heading_valid)
		accuracy = (int32_t)gaf_outputs.gmrv_heading_q11 / (float)Q11_DIV;
  } else {
    quatW = 0;
    quatX = 0;
    quatY = 0;
    quatZ = 0;
  }
#endif
  return 0;
}

int ICM456xx::getGaf_RVData(float& quatW, float& quatX, float& quatY, float& quatZ, float& accuracy)
{
#if (INV_DEVICE_TYPE == INV_TYPE_C1)
  inv_imu_edmp_gaf_outputs_t gaf_outputs;

  accuracy = 0;
  getGafData(gaf_outputs);

  if(gaf_outputs.rv_quat_valid)
  {
    quatW = (int32_t)gaf_outputs.rv_quat_q14[0] / (float)Q14_DIV;
    quatX = (int32_t)gaf_outputs.rv_quat_q14[1] / (float)Q14_DIV;
    quatY = (int32_t)gaf_outputs.rv_quat_q14[2] / (float)Q14_DIV;
    quatZ = (int32_t)gaf_outputs.rv_quat_q14[3] / (float)Q14_DIV;
	if(gaf_outputs.rv_heading_valid)
		accuracy = (int32_t)gaf_outputs.rv_heading_q11 / (float)Q11_DIV;
  }else{
    quatW = 0;
    quatX = 0;
    quatY = 0;
    quatZ = 0;
  }
#endif
  return 0;
}

int ICM456xx::getGaf_RMData(float& mX, float& mY, float& mZ)
{
#if (INV_DEVICE_TYPE == INV_TYPE_C1)
  mX = (int32_t)gaf_outputs_internal.rmag[0] * RAW_MAG_SCALE;
  mY = (int32_t)gaf_outputs_internal.rmag[1] * RAW_MAG_SCALE;
  mZ = (int32_t)gaf_outputs_internal.rmag[2] * RAW_MAG_SCALE;
#endif
  return 0;
}

int ICM456xx::getGaf_BiasData(int type, int& q12_X,int& q12_Y,int& q12_Z,int& accuracy)
{
  switch(type)
  {
    case GYRO:
      q12_X = gaf_outputs_internal.gyr_bias_q12[0];
      q12_Y = gaf_outputs_internal.gyr_bias_q12[1];
      q12_Z = gaf_outputs_internal.gyr_bias_q12[2];
      accuracy = gaf_outputs_internal.gyr_accuracy_flag;
      break;
#if (INV_DEVICE_TYPE == INV_TYPE_C1)
    case MAG:
      q12_X = gaf_outputs_internal.mag_bias_q16[0] >> 4;
      q12_Y = gaf_outputs_internal.mag_bias_q16[1] >> 4;
      q12_Z = gaf_outputs_internal.mag_bias_q16[2] >> 4;
      accuracy = gaf_outputs_internal.mag_accuracy_flag;
      break;
#endif
    default:
      return -1;
  }

  return 0;
}

#endif /* INV_TYPE_A2 & INV_TYPE_C1 */

#if (INV_DEVICE_TYPE == INV_TYPE_B1)

/* Default matrix to be used: identity */
static int8_t mounting_matrix[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

int ICM456xx::startVocalVibDet(uint8_t intpin, ICM456xx_irq_handler handler)
{
  int rc = 0;
  inv_imu_edmp_int_state_t apex_int_config;
  inv_imu_edmp_vvd_parameters_t vvd_params;
  inv_imu_int_state_t int_config;
  inv_imu_int_pin_config_t	int_pin_config;

  pinMode(intpin,INPUT);
  attachInterrupt(intpin,handler,RISING);

  sleep_us(3000);

  int_pin_config.int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH;
  int_pin_config.int_mode     = INTX_CONFIG2_INTX_MODE_PULSE;
  int_pin_config.int_drive    = INTX_CONFIG2_INTX_DRIVE_PP;
  rc |= inv_imu_set_pin_config_int(&icm_driver, INV_IMU_INT1, &int_pin_config);

  /* Stop all sensors in case they were previously ON */
  rc |= inv_imu_edmp_disable(&icm_driver);
  rc |= inv_imu_set_accel_mode(&icm_driver, PWR_MGMT0_ACCEL_MODE_OFF);
  if (rc != 0) {
    return rc;
  }

  /* Set ODR */
  rc |= inv_imu_edmp_set_frequency(&icm_driver, DMP_EXT_SEN_ODR_CFG_APEX_ODR_800_HZ);
  rc |= inv_imu_set_accel_frequency(&icm_driver, ACCEL_CONFIG0_ACCEL_ODR_800_HZ);
  if (rc != 0) {
    return rc;
  }

  /* Set sensor FSR */
  rc |= inv_imu_set_accel_fsr(&icm_driver, ACCEL_CONFIG0_ACCEL_UI_FS_SEL_8_G);
  if (rc != 0) {
    return rc;
  }
  
  /* Initialize APEX for VVD */
  rc |= inv_imu_edmp_init(&icm_driver);
  rc |= inv_imu_edmp_get_vvd_parameters(&icm_driver, &vvd_params);

  if(nb_samples_before_decision)
    vvd_params.nbSamplesBeforeDecision = nb_samples_before_decision;
  else
    nb_samples_before_decision = vvd_params.nbSamplesBeforeDecision;

  if(vvd_thresh)
    vvd_params.dynEnvVVDThresh = vvd_thresh;
  else
    vvd_thresh = vvd_params.dynEnvVVDThresh;
  
  rc |= inv_imu_edmp_set_vvd_parameters(&icm_driver, &vvd_params);

  rc |= inv_imu_edmp_set_vvd_model(&icm_driver);
  if (rc != 0) {
    return rc;
  }
  
  /* Set accel in low-noise mode with no filter as ODR/2 is not reachable */
  rc |= inv_imu_set_accel_ln_bw(&icm_driver, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_NO_FILTER);
  rc |= inv_imu_set_accel_mode(&icm_driver, PWR_MGMT0_ACCEL_MODE_LN);
  if (rc != 0) {
    return rc;
  }
  
  /* Wait for accel startup time */
  sleep_us(ACC_STARTUP_TIME_US);
  
  /* Set EDMP mounting matrix */
  rc |= inv_imu_edmp_set_mounting_matrix(&icm_driver, mounting_matrix);
  if (rc != 0) {
    return rc;
  }
  
  memset(&int_config, INV_IMU_DISABLE, sizeof(int_config));
  int_config.INV_EDMP_EVENT = INV_IMU_ENABLE;

  rc |= inv_imu_set_config_int(&icm_driver, INV_IMU_INT1, &int_config);
  if (rc != 0) {
    return rc;
  }
  
  /* Enable VVD */
  rc |= inv_imu_edmp_enable_vvd(&icm_driver);
  if (rc != 0) {
    return rc;
  }
  
  /* Disable all APEX interrupt and enable only the one we need */
  memset(&apex_int_config, INV_IMU_DISABLE, sizeof(apex_int_config));
  apex_int_config.INV_VVD = INV_IMU_ENABLE;
  rc |= inv_imu_edmp_set_config_int_apex(&icm_driver, &apex_int_config);
  if (rc != 0) {
    return rc;
  }
  
  /* Enable EDMP */
  rc |= inv_imu_edmp_enable(&icm_driver);
  if (rc != 0) {
    return rc;
  }

  return rc;
}

int ICM456xx::GetVocalVibDet_thresh(void)
{
  return vvd_thresh;
}

void ICM456xx::SetVocalVibDet_thresh(uint32_t thresh)
{
  vvd_thresh = thresh;
}

int ICM456xx::SetVocalVibDet_Dynthresh(uint32_t thresh)
{
  return inv_imu_edmp_request_vvd_update_thresh(&icm_driver, thresh);
}

int ICM456xx::GetVocalVibDet_nb_samples(void)
{
  return nb_samples_before_decision;
}

void ICM456xx::SetVocalVibDet_nb_samples(uint32_t sample)
{
  nb_samples_before_decision = sample;
}

int ICM456xx::CheckVocalVibDet_thresh(void)
{
  uint8_t status_update=0;
  inv_imu_edmp_check_vvd_thresh(&icm_driver, &status_update);
  return status_update;
}

#endif

#if (INV_DEVICE_TYPE == INV_TYPE_A1) || (INV_DEVICE_TYPE == INV_TYPE_B1) || (INV_DEVICE_TYPE == INV_TYPE_C1)

#define MAG_I2C_ADDR 0x1E

static int wait_for_i2c_master_complete(inv_imu_device_t *icm_driver)
{
  const uint32_t start_ms = millis();
  inv_imu_int_state_t int_state;

  while (1) {
    // ICM 内部 I2C master 完成后会置位 INV_I2CM_DONE；未置位前不能读取结果缓存。
    inv_imu_get_int_status(icm_driver, INV_IMU_INT1, &int_state);
    if (int_state.INV_I2CM_DONE) {
      i2cm_status_t i2cm_status;

      // DONE 只表示一次事务结束，还需要继续检查 busy/timeout/SCL/SDA 等错误位。
      inv_imu_read_reg(icm_driver, I2CM_STATUS, 1, (uint8_t *)&i2cm_status);
      if (i2cm_status.i2cm_busy || i2cm_status.i2cm_timeout_err ||
          i2cm_status.i2cm_srst_err || i2cm_status.i2cm_scl_err ||
          i2cm_status.i2cm_sda_err)
        return -1;
      return 0;
    }
    if ((millis() - start_ms) >= ICM456XX_I2CM_DONE_TIMEOUT_MS) {
      // AUX 口未接磁力计或上拉异常时不能无限阻塞飞控/诊断台。
      return INV_IMU_ERROR_TIMEOUT;
    }
    delay(1);
  }

  return INV_IMU_ERROR_TIMEOUT;
}

int ICM456xx::setI2CM(void)
{
  int rc = 0;
  inv_imu_int_state_t int1_config;

  // 初始化 ICM45686 内部 I2C master，并打开 I2CM_DONE 中断状态位供轮询等待。
  rc = inv_imu_init_i2cm(&icm_driver);
  rc |= inv_imu_get_config_int(&icm_driver, INV_IMU_INT1, &int1_config);
  int1_config.INV_I2CM_DONE = INV_IMU_ENABLE;
  rc |= inv_imu_set_config_int(&icm_driver, INV_IMU_INT1, &int1_config);  
  return rc;
}

int ICM456xx::getDataFromI2CM(uint8_t reg, uint8_t& data)
{
  int rc = 0;
  // 通过 ICM45686 内部 I2C master 读取外部 ICT1531/兼容磁力计的 1 字节寄存器。
  inv_imu_i2c_master_cfg_t cfg = { .op_cnt   = 1,
                                   .i2c_addr = MAG_I2C_ADDR,
                                   .op 	  = { /* op[0] */
                                             {
                                               .r_n_w	   = 1,
                                               .reg_addr = reg,
                                               .len      = 1
                                  } } };
  
  rc |= inv_imu_configure_i2cm(&icm_driver, &cfg, NULL);
  // fast_mode=1 使用内部 I2CM 快速事务；事务完成仍通过 wait_for_i2c_master_complete() 判定。
  rc |= inv_imu_start_i2cm_ops(&icm_driver, 1 /* fast_mode */);
  rc |= wait_for_i2c_master_complete(&icm_driver);
  rc |= inv_imu_get_i2cm_data(&icm_driver, &data, 1);

  return rc;
}

int ICM456xx::setI2CMPassThrough(void)
{
  return inv_init_i2cm_bypass(&icm_driver);
}

int ICM456xx::getDataFromPassThrough(uint8_t reg, uint8_t& data)
{
  int rc = 0;
  // pass-through 直接把主接口临时指向外部磁力计地址，读完必须恢复 ICM 主地址。
  i2c_address = MAG_I2C_ADDR;
  rc = inv_imu_read_reg(&icm_driver, reg, 1, (uint8_t *)&data);
  i2c_address = ICM456xx_I2C_ADDRESS;
  return rc;
}

static uint8_t fifo_data[FIFO_MIRRORING_SIZE]; /* Memory where to store FIFO data */

int ICM456xx::adv_getDataFromFifo(void)
{
  uint16_t fifo_count = 0;
  // 高级 FIFO 路径会先批量取出 FIFO mirror，再交给 advanced parser 分发到 sensor_event_cb()。
  inv_imu_adv_get_data_from_fifo(&icm_driver, fifo_data, &fifo_count);
  inv_imu_adv_parse_fifo_data(&icm_driver, fifo_data, fifo_count);
  return 0;
}

#if (INV_DEVICE_TYPE == INV_TYPE_A1)
int ICM456xx::setI2CM_FIFO(uint8_t intpin, ICM456xx_irq_handler handler)
{
  int rc = 0;
  inv_imu_int_state_t int1_config;
  inv_imu_int_pin_config_t  int_pin_config;
  inv_imu_adv_fifo_config_t fifo_config;
  uint8_t wdata = 0x12;

  // ES0 配置两个 I2CM 操作：先读磁力计温度/三轴数据，再写模式寄存器触发下一次测量。
  inv_imu_i2c_master_cfg_t cfg = { .op_cnt	= 2,
                                   .i2c_addr = MAG_I2C_ADDR,
                                   .op	  = { /* op[0] */
                                            {
                                                 .r_n_w    = 1,
                                                 .reg_addr = ICT1531X_TEMP_DATA_MSB,
                                                 .len	   = 7
                                            },
                                            {
                                                 .r_n_w    = 0,
                                                 .reg_addr = ICT1531X_MODE_CTRL_REG,
                                                 .len	   = 1,
                                                 .wdata = &wdata
                                            }
                                  } };

  int_pin_config.int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH;
  int_pin_config.int_mode     = INTX_CONFIG2_INTX_MODE_PULSE;
  int_pin_config.int_drive    = INTX_CONFIG2_INTX_DRIVE_PP;
  rc |= inv_imu_set_pin_config_int(&icm_driver, INV_IMU_INT1, &int_pin_config);

  /* Interrupts configuration */
  memset(&int1_config, INV_IMU_DISABLE, sizeof(int1_config));
  int1_config.INV_FIFO_THS = INV_IMU_ENABLE;
  rc |= inv_imu_set_config_int(&icm_driver, INV_IMU_INT1, &int1_config);

  /* FIFO configuration */
  rc |= inv_imu_adv_get_fifo_config(&icm_driver, &fifo_config);
  // AUX/eDMP 会占用 SRAM，因此这里使用 APEX 兼容的 2KB FIFO 深度。
  fifo_config.base_conf.gyro_en    = 1;
  fifo_config.base_conf.accel_en   = 1;
  fifo_config.base_conf.hires_en   = 0;
  fifo_config.base_conf.fifo_depth = FIFO_CONFIG0_FIFO_DEPTH_APEX;
  fifo_config.base_conf.fifo_wm_th = 1;
  fifo_config.base_conf.fifo_mode  = FIFO_CONFIG0_FIFO_MODE_SNAPSHOT;
  fifo_config.tmst_fsync_en        = INV_IMU_ENABLE;
  fifo_config.fifo_wr_wm_gt_th     = FIFO_CONFIG2_FIFO_WR_WM_EQ_OR_GT_TH;
  fifo_config.es0_en               = 1;
  fifo_config.es1_en               = 0;
  /* eDMP image will read external data and output 6 bytes in FIFO ES0 frame bitfield */
  fifo_config.es0_6b_9b = FIFO_CONFIG4_FIFO_ES0_6B;
  rc |= inv_imu_adv_set_fifo_config(&icm_driver, &fifo_config);

  rc = invn_mag_init(&icm_driver);
  rc |= inv_imu_edmp_compass_set_frequency(&icm_driver, DMP_EXT_SEN_ODR_CFG_EXT_ODR_25_HZ);
  rc |= inv_imu_configure_i2cm(&icm_driver, &cfg, NULL);
  
  /* Run eDMP init and load RAM image */
  rc |= inv_imu_edmp_compass_init(&icm_driver);

  /* Configure eDMP to get compass data from external sensor */
  rc |= inv_imu_edmp_compass_enable_es(&icm_driver);
  
  /* Reset FIFO */
  rc |= inv_imu_adv_reset_fifo(&icm_driver);
  
  /* Enable eDMP */
  rc |= inv_imu_edmp_enable(&icm_driver);
  
  /* Let dmp see ODR ready on ISR1, ISR0 all off */
  rc |= inv_imu_edmp_unmask_int_src(&icm_driver, INV_IMU_EDMP_INT1, EDMP_INT_SRC_EXT_ODR_DRDY_MASK);

  rc |= setup_irq(intpin, handler);
  
  return rc;  
}

int ICM456xx::getAdvDataFromFifo(int32_t *accel, int32_t *gyro, float *external)
{
  int rc = 0;
  uint16_t fifo_count;
  const fifo_header_t *    header;
  const fifo_header2_t *   header2;

  rc |= inv_imu_adv_get_data_from_fifo(&icm_driver, fifo_data, &fifo_count);

  /*
   * 修正 ES0 valid 标志：
   * 某些底层版本在外部传感器 ES0 数据有效时没有正确置位 header2.es0_en。
   * 这里在解析前手动补 bit，保证 advanced parser 会把外部磁力计数据送到回调。
   */
  header  = (const fifo_header_t *)&fifo_data[0];
  if(header->bits.ext_header)
  {
    header2 = (const fifo_header2_t *)&fifo_data[1];    
    if (header2->bits.es0_en)
      fifo_data[1] |= (1 << 2);
  }

  rc |= inv_imu_adv_parse_fifo_data(&icm_driver, fifo_data, fifo_count);

  memcpy(accel, accel_data, sizeof(accel_data));
  memcpy(gyro, gyro_data, sizeof(gyro_data));
  memcpy(external, mag_data, sizeof(mag_data));

  return rc;
}

int ICM456xx::initMag(void)
{
  int ret =0;
  ret = invn_read_whoami(&icm_driver);
  return ret;
}

int ICM456xx::enableMag(int flag)
{
  int ret =0;
  ret = invn_mag_enable(flag);
  return ret;
}

int ICM456xx::getExternalMagData(float *mag)
{
  int rc = 0;
  uint8_t data[6];
  int16_t external_data[3];
   
  // 主动读取 ICT1531 三轴磁力计原始数据，适合 AUX bring-up，不依赖 FIFO/eDMP。
  inv_imu_i2c_master_cfg_t cfg = { .op_cnt   = 1,
                                   .i2c_addr = MAG_I2C_ADDR,
                                   .op 	  = { /* op[0] */
                                            {
                                               .r_n_w	   = 1,
                                               .reg_addr = ICT1531X_MAG_DATA_X_LSB,
                                               .len      = 6
                                 } } };
  
  rc |= inv_imu_configure_i2cm(&icm_driver, &cfg, NULL);
  rc |= inv_imu_start_i2cm_ops(&icm_driver, 1 /* fast_mode */);
  rc |= wait_for_i2c_master_complete(&icm_driver);
  rc |= inv_imu_get_i2cm_data(&icm_driver, data, 6);

  // ICT1531 数据为小端 int16，换算系数 RAW_MAG_SCALE 约为 0.075 uT/LSB。
  external_data[0] = (((int16_t)data[1]) << 8) + data[0];
  external_data[1] = (((int16_t)data[3]) << 8) + data[2];
  external_data[2] = (((int16_t)data[5]) << 8) + data[4];

  mag_data[0] = external_data[0] * RAW_MAG_SCALE;
  mag_data[1] = external_data[1] * RAW_MAG_SCALE;
  mag_data[2] = external_data[2] * RAW_MAG_SCALE;
  memcpy(mag, mag_data, sizeof(mag_data));

  return rc;
}

#endif /* INV_TYPE_A1 */
#endif /* INV_TYPE_A1 & INV_TYPE_C1 */

/*
 * FIFO/eDMP 事件回调。
 *
 * advanced parser 每解析出一类数据，就通过这个回调交给 Arduino 封装。当前实现把 accel/gyro/mag
 * 写入文件级静态缓存，因此仍是“单 IMU 友好，多 IMU 不隔离”的结构。后续如果要做双 IMU 冗余，
 * 需要把这些缓存搬到 ICM456xx 实例内部，并让底层回调携带对象上下文。
 */
static void sensor_event_cb(inv_imu_sensor_event_t *event)
{
  if(event->sensor_mask & (1 << INV_SENSOR_ACCEL))
  {
    accel_data[0] = event->accel[0];
    accel_data[1] = event->accel[1];
    accel_data[2] = event->accel[2];
  }
	
  if(event->sensor_mask & (1 << INV_SENSOR_GYRO))
  {
    gyro_data[0] = event->gyro[0];
    gyro_data[1] = event->gyro[1];
    gyro_data[2] = event->gyro[2];  
  }

#if (INV_DEVICE_TYPE == INV_TYPE_A2)
  if ((event->sensor_mask & (1 << INV_SENSOR_ES0)) &&
	  (event->sensor_mask & (1 << INV_SENSOR_ES1))) {
    inv_imu_sensor_data_t data;
    inv_imu_get_register_data(icm_driver_ptr, &data);

    accel_data[0] = data.accel_data[0];
    accel_data[1] = data.accel_data[1];
    accel_data[2] = data.accel_data[2];

    gyro_data[0] = data.gyro_data[0];
    gyro_data[1] = data.gyro_data[1];
    gyro_data[2] = data.gyro_data[2];
	
    gaf_status = inv_imu_edmp_gaf_build_outputs(icm_driver_ptr, (const uint8_t *)event->es0, 
                                               (const uint8_t *)event->es1, &gaf_outputs_internal);
    if(!gaf_status)
      gaf_status = 1;
  }
#elif (INV_DEVICE_TYPE == INV_TYPE_A1)
  if(event->sensor_mask & (1 << INV_SENSOR_ES0))
  {
    int16_t external_data[3];
	
    external_data[0] = (((int16_t)event->es0[1]) << 8) + event->es0[0];
    external_data[1] = (((int16_t)event->es0[3]) << 8) + event->es0[2];
    external_data[2] = (((int16_t)event->es0[5]) << 8) + event->es0[4];

    mag_data[0] = external_data[0] * RAW_MAG_SCALE;
    mag_data[1] = external_data[1] * RAW_MAG_SCALE;
    mag_data[2] = external_data[2] * RAW_MAG_SCALE;
  }

#elif (INV_DEVICE_TYPE == INV_TYPE_B1) || (INV_DEVICE_TYPE == INV_TYPE_C1)
  if ((event->sensor_mask & ((1 << INV_SENSOR_ES0) | (1 << INV_SENSOR_ES1))) == ((1 << INV_SENSOR_ES0) | (1 << INV_SENSOR_ES1)))
  {
    inv_imu_edmp_gaf_decode_fifo(icm_driver_ptr, (const uint8_t *)event->es0,
                                 (const uint8_t *)event->es1, &gaf_outputs_internal);
#if (INV_DEVICE_TYPE == INV_TYPE_C1)
    gaf_status = gaf_outputs_internal.frame_complete;
#else
    gaf_status = 1;
#endif
  }
#endif
}

#if (INV_DEVICE_TYPE == INV_TYPE_A2) || (INV_DEVICE_TYPE == INV_TYPE_B1) || (INV_DEVICE_TYPE == INV_TYPE_C1)

static void fixedpoint_to_float(const int32_t *in, float *out, const uint8_t fxp_shift,
                                const uint8_t dim)
{
	int   i;
	float scale = 1.f / (1 << fxp_shift);

	for (i = 0; i < dim; i++)
		out[i] = scale * in[i];
}


int ICM456xx::getCalibratedMag(float& mX, float& mY, float& mZ)
{
  int32_t raw_q16[3];
  float mag_ut[3];
#if (INV_DEVICE_TYPE == INV_TYPE_C1)
  if(gaf_outputs_internal.rmag_valid)
  {
    raw_q16[0] = (int32_t)gaf_outputs_internal.rmag[0] * RAW_MAG_SCALE_Q16;
    raw_q16[1] = (int32_t)gaf_outputs_internal.rmag[1] * RAW_MAG_SCALE_Q16;
    raw_q16[2] = (int32_t)gaf_outputs_internal.rmag[2] * RAW_MAG_SCALE_Q16;
  }
  
  if(gaf_outputs_internal.mag_bias_valid)
  {
    raw_q16[0] -= gaf_outputs_internal.mag_bias_q16[0];
    raw_q16[1] -= gaf_outputs_internal.mag_bias_q16[1];
    raw_q16[2] -= gaf_outputs_internal.mag_bias_q16[2];
  }

  fixedpoint_to_float(raw_q16, mag_ut, 16, 3);
  mX = mag_ut[0];
  mY = mag_ut[1];
  mZ = mag_ut[2];
#endif
  return 0;
}

int ICM456xx::getCalibratedAccel(float& aX, float& aY, float& aZ)
{
  int32_t raw_q16[3];
  float accel_ut[3];

  raw_q16[0] = (int32_t)accel_data[0] * RAW_ACC_SCALE - acc_bias_q16[0];
  raw_q16[1] = (int32_t)accel_data[1] * RAW_ACC_SCALE - acc_bias_q16[1];
  raw_q16[2] = (int32_t)accel_data[2] * RAW_ACC_SCALE - acc_bias_q16[2];

  fixedpoint_to_float(raw_q16, accel_ut, 16, 3);
  aX = accel_ut[0];
  aY = accel_ut[1];
  aZ = accel_ut[2];
	
  return 0;
}

int ICM456xx::getCalibratedGyro(float& gX, float& gY, float& gZ)
{
  int32_t raw_q16[3];
  float gyro_ut[3];

  raw_q16[0] = (int32_t)gyro_data[0] * RAW_GYR_SCALE;
  raw_q16[1] = (int32_t)gyro_data[1] * RAW_GYR_SCALE;
  raw_q16[2] = (int32_t)gyro_data[2] * RAW_GYR_SCALE;

#if (INV_DEVICE_TYPE == INV_TYPE_C1)
  if(gaf_outputs_internal.gyr_bias_valid)
#endif
  {
    raw_q16[0] -= (int32_t)gaf_outputs_internal.gyr_bias_q12[0] << 4;
    raw_q16[1] -= (int32_t)gaf_outputs_internal.gyr_bias_q12[1] << 4;
    raw_q16[2] -= (int32_t)gaf_outputs_internal.gyr_bias_q12[2] << 4;
  }

  fixedpoint_to_float(raw_q16, gyro_ut, 16, 3);
  gX = gyro_ut[0];
  gY = gyro_ut[1];
  gZ = gyro_ut[2];
  
  return 0;
}

#endif

int ICM456xx::setApexInterrupt(uint8_t intpin, ICM456xx_irq_handler handler)
{
  int rc = 0;
  inv_imu_int_state_t config_int;
  inv_imu_int_pin_config_t int_pin_config;
  inv_imu_edmp_int_state_t apex_int_config;

  if (handler == NULL)
    return 0;
  
  // APEX 事件通常用边沿触发；ISR 内仍然只应置位/计数，不应读 SPI/I2C。
  pinMode(intpin,INPUT);
  attachInterrupt(intpin,handler,RISING);
  
  /*
   * Configure interrupts pins
   * - Polarity High
   * - Pulse mode
   * - Push-Pull drive
   */
  int_pin_config.int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH;
  int_pin_config.int_mode     = INTX_CONFIG2_INTX_MODE_PULSE;
  int_pin_config.int_drive    = INTX_CONFIG2_INTX_DRIVE_PP;
  rc |= inv_imu_set_pin_config_int(&icm_driver, INV_IMU_INT1, &int_pin_config);
  
  /*
   * 主 UI 中断源只打开 EDMP_EVENT，避免 FIFO/WOM/DRDY 与 APEX 事件混到同一模式。
   * 如果后续要混用，需要建立更明确的模式状态机和状态读取顺序。
   */
  inv_imu_get_config_int(&icm_driver,INV_IMU_INT1, &config_int);
  config_int.INV_WOM_X = INV_IMU_DISABLE;
  config_int.INV_WOM_Y = INV_IMU_DISABLE;
  config_int.INV_WOM_Z = INV_IMU_DISABLE;
  config_int.INV_FIFO_THS = INV_IMU_DISABLE;
  config_int.INV_EDMP_EVENT = INV_IMU_ENABLE;
  inv_imu_set_config_int(&icm_driver,INV_IMU_INT1, &config_int);
        
  // 根据已通过 startXxx() 置位的 apex_enable[]，生成 eDMP 内部 APEX 中断掩码。
  memset(&apex_int_config, INV_IMU_DISABLE, sizeof(apex_int_config));
  if(apex_enable[ICM456XX_APEX_TAP])
    apex_int_config.INV_TAP = INV_IMU_ENABLE;
  if(apex_enable[ICM456XX_APEX_FF])
    apex_int_config.INV_FF = INV_IMU_ENABLE;
  if(apex_enable[ICM456XX_APEX_LOWG])
    apex_int_config.INV_LOWG = INV_IMU_ENABLE;
  if(apex_enable[ICM456XX_APEX_HIGHG])
    apex_int_config.INV_HIGHG = INV_IMU_ENABLE;
  if(apex_enable[ICM456XX_APEX_B2S])
  {
    apex_int_config.INV_B2S = INV_IMU_ENABLE;
    apex_int_config.INV_B2S_REV = INV_IMU_ENABLE;
  }
#if (INV_DEVICE_TYPE == INV_TYPE_A1) || (INV_DEVICE_TYPE == INV_TYPE_A2)
  if(apex_enable[ICM456XX_APEX_TILT])
    apex_int_config.INV_TILT_DET = INV_IMU_ENABLE;
  if(apex_enable[ICM456XX_APEX_PEDOMETER])
  {
    apex_int_config.INV_STEP_DET = INV_IMU_ENABLE;
    apex_int_config.INV_STEP_CNT_OVFL = INV_IMU_ENABLE;
  }
  if(apex_enable[ICM456XX_APEX_R2W])
  { 
    apex_int_config.INV_R2W = INV_IMU_ENABLE;
    apex_int_config.INV_R2W_SLEEP = INV_IMU_ENABLE;
  }
#elif (INV_DEVICE_TYPE == INV_TYPE_B1)
  if(apex_enable[ICM456XX_APEX_AID])
  {
    apex_int_config.INV_AID_DEVICE = INV_IMU_ENABLE;
    apex_int_config.INV_AID_HUMAN = INV_IMU_ENABLE;
  }
#endif
  rc |= inv_imu_edmp_set_config_int_apex(&icm_driver, &apex_int_config);

  return rc;
}

int ICM456xx::startAPEX(dmp_ext_sen_odr_cfg_apex_odr_t edmp_odr, accel_config0_accel_odr_t accel_odr)
{
  int rc = 0;

  /*
   * AUTO ODR 选择：
   * - 自由落体/高 g/低 g 需要更高加速度采样率，优先 400/800 Hz。
   * - Tap 需要较高时间分辨率，使用 400 Hz。
   * - Tilt/Pedometer/R2W 这类慢事件使用 50 Hz。
   * APEX 会重配 accel/eDMP ODR，因此不要在飞控主六轴读取模式中动态调用本函数。
   */
  if(edmp_odr == ICM456XX_APEX_ODR_AUTO && accel_odr == ICM456XX_ACCEL_ODR_AUTO)
  {
    if(apex_enable[ICM456XX_APEX_FF] || apex_enable[ICM456XX_APEX_LOWG] || apex_enable[ICM456XX_APEX_HIGHG])
    {
#if (INV_DEVICE_TYPE == INV_TYPE_A1) || (INV_DEVICE_TYPE == INV_TYPE_A2)
      edmp_odr = DMP_EXT_SEN_ODR_CFG_APEX_ODR_800_HZ;
      accel_odr = ACCEL_CONFIG0_ACCEL_ODR_800_HZ;
#else
      edmp_odr = DMP_EXT_SEN_ODR_CFG_APEX_ODR_400_HZ;
      accel_odr = ACCEL_CONFIG0_ACCEL_ODR_400_HZ;
#endif
    }
	else if(apex_enable[ICM456XX_APEX_TAP])
    {
      edmp_odr = DMP_EXT_SEN_ODR_CFG_APEX_ODR_400_HZ;
      accel_odr = ACCEL_CONFIG0_ACCEL_ODR_400_HZ;
    }
    else
    {
      edmp_odr = DMP_EXT_SEN_ODR_CFG_APEX_ODR_50_HZ;
      accel_odr = ACCEL_CONFIG0_ACCEL_ODR_50_HZ;
    }
  }
  
#if (INV_DEVICE_TYPE == INV_TYPE_C1) || (INV_DEVICE_TYPE == INV_TYPE_B1)
  inv_imu_edmp_powersave_parameters_t power_parameters;
  rc |= inv_imu_edmp_init(&icm_driver);
#else
  inv_imu_edmp_apex_parameters_t apex_parameters;
  // A1/A2 芯片使用 APEX 专用初始化入口，内部会准备 eDMP 参数区。
  rc |= inv_imu_edmp_init_apex(&icm_driver);
#endif

  // 多个 APEX 功能同时启用时，保留“要求最高”的 eDMP ODR 和 accel ODR。
  if(apex_edmp_odr < edmp_odr)
      apex_edmp_odr = edmp_odr;
  /* Accel ODR higher value means lower ODR */
  if(apex_accel_odr > accel_odr)
      apex_accel_odr = accel_odr;

  rc |= inv_imu_edmp_set_frequency(&icm_driver, apex_edmp_odr);
  rc |= inv_imu_set_accel_frequency(&icm_driver, apex_accel_odr);

  /* Set BW = ODR/4 */
  rc |= inv_imu_set_accel_ln_bw(&icm_driver, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4);

  /* Select WUOSC clock to have accel in ULP (lowest power mode) */
  rc |= inv_imu_select_accel_lp_clk(&icm_driver, SMC_CONTROL_0_ACCEL_LP_CLK_WUOSC);

  /* Set AVG to 1x */
  rc |= inv_imu_set_accel_lp_avg(&icm_driver, IPREG_SYS2_REG_129_ACCEL_LP_AVG_1);

  /*
   * 重新启动 APEX 前先关闭所有可能已开启的事件，避免上一次诊断模式残留。
   * 这也是为什么 APEX 模式与普通寄存器轮询/FIFO 模式应互斥使用。
   */
#if (INV_DEVICE_TYPE == INV_TYPE_A1) || (INV_DEVICE_TYPE == INV_TYPE_A2)
  rc |= inv_imu_edmp_disable_pedometer(&icm_driver);
  rc |= inv_imu_edmp_disable_tilt(&icm_driver);
  rc |= inv_imu_edmp_disable_r2w(&icm_driver);
#elif (INV_DEVICE_TYPE == INV_TYPE_B1)
  rc |= inv_imu_edmp_disable_b2s(&icm_driver);
  rc |= inv_imu_edmp_disable_aid(&icm_driver);
#endif
  rc |= inv_imu_edmp_disable_tap(&icm_driver);
  rc |= inv_imu_adv_disable_wom(&icm_driver);
  rc |= inv_imu_edmp_disable_ff(&icm_driver);
  rc |= inv_imu_edmp_disable(&icm_driver);

#if (INV_DEVICE_TYPE == INV_TYPE_B1) || (INV_DEVICE_TYPE == INV_TYPE_C1)
  rc |= inv_imu_edmp_recompute_decimation(&icm_driver);

  rc |= inv_imu_edmp_get_powersave_parameters(&icm_driver, &power_parameters);
  power_parameters.power_save_en = 0;
  rc |= inv_imu_edmp_set_powersave_parameters(&icm_driver, &power_parameters);
#else /* INV_TYPE_B1 & INV_TYPE_C1 */
  rc |= inv_imu_edmp_recompute_apex_decimation(&icm_driver);
  
  rc |= inv_imu_edmp_get_apex_parameters(&icm_driver, &apex_parameters);
  // 关闭 APEX power save，使诊断时事件响应更确定；低功耗产品可按需求再开放参数。
  apex_parameters.power_save_en = 0;
  rc |= inv_imu_edmp_set_apex_parameters(&icm_driver, &apex_parameters);
#endif /* INV_TYPE_A1 & INV_TYPE_A2*/

  // APEX 算法需要加速度计工作，使用 LN 模式保证算法输入噪声较低。
  rc |= inv_imu_set_accel_mode(&icm_driver, PWR_MGMT0_ACCEL_MODE_LN);

#if (INV_DEVICE_TYPE == INV_TYPE_B1) || (INV_DEVICE_TYPE == INV_TYPE_C1)
  if(apex_enable[ICM456XX_APEX_TAP])
  {
    inv_imu_edmp_tap_parameters_t tap_parameters;
    rc |= inv_imu_edmp_get_tap_parameters(&icm_driver, &tap_parameters);
    tap_parameters.tap_tmax			 = TAP_TMAX_400HZ;
    tap_parameters.tap_tmin			 = TAP_TMIN_400HZ;
    tap_parameters.tap_smudge_reject_th = TAP_SMUDGE_REJECT_THR_400HZ;
    tap_parameters.tap_max = 3; // we want to be able to detect triple tap
    tap_parameters.tap_min = 1; // we want to be able to detect single tap
    rc |= inv_imu_edmp_set_tap_parameters(&icm_driver, &tap_parameters);
    rc |= inv_imu_edmp_enable_tap(&icm_driver);
  }

  if(apex_enable[ICM456XX_APEX_B2S])
  {
    inv_imu_edmp_b2s_parameters_t b2s_params;
#if (INV_DEVICE_TYPE == INV_TYPE_B1)
    rc |= inv_imu_edmp_get_b2s_parameters(&icm_driver, &b2s_params);
    b2s_params.b2s_mounting_matrix = 0;
    rc |= inv_imu_edmp_set_b2s_parameters(&icm_driver, &b2s_params);
    rc |= inv_imu_edmp_enable_b2s(&icm_driver);
#else /* INV_TYPE_B1 */
    inv_imu_edmp_b2s_init_t usecase;
    rc |= inv_imu_edmp_b2s_init_over_sif(&icm_driver, &usecase);
    rc |= inv_imu_edmp_b2s_get_parameters(&icm_driver, &b2s_params, usecase);
    b2s_params.b2s_mounting_matrix = 0; /* identity matrix */
    rc |= inv_imu_edmp_b2s_set_parameters(&icm_driver, &b2s_params, usecase);
    rc |= inv_imu_edmp_b2s_enable(&icm_driver);
#endif /*INV_TYPE_C1*/
  }

#if (INV_DEVICE_TYPE == INV_TYPE_B1)
  if(apex_enable[ICM456XX_APEX_AID])
  {
    inv_imu_edmp_aid_parameters_t aid_params;
    rc |= inv_imu_edmp_get_aid_parameters(&icm_driver, &aid_params);
    /* AID device config */
    aid_params.aid_device.aid_update_win                 = 50;
    aid_params.aid_device.aid_alert_timer                = 600;
    aid_params.aid_device.aid_output_enable              = 7;
    aid_params.aid_device.aid_disable_multiple_interrupt = 1;
    /* AID human config */
    aid_params.aid_human.aid_update_win                 = 350;
    aid_params.aid_human.aid_alert_timer                = 3750;
    aid_params.aid_human.aid_output_enable              = 7;
    aid_params.aid_human.aid_disable_multiple_interrupt = 1;

    rc |= inv_imu_edmp_set_aid_parameters(&icm_driver, &aid_params);
    rc |= inv_imu_adv_configure_wom(&icm_driver, 6, 6, 6, // 6 * 1/256mg ~= 23.44mg for each axis
                                    TMST_WOM_CONFIG_WOM_INT_MODE_ORED,
                                    TMST_WOM_CONFIG_WOM_INT_DUR_1_SMPL);
    rc |= inv_imu_adv_enable_wom(&icm_driver);
    rc |= inv_imu_edmp_enable_aid(&icm_driver);
  }
#endif /* INV_TYPE_B1 */
#else /* INV_TYPE_B1 & INV_TYPE_C1*/
  if(apex_enable[ICM456XX_APEX_TILT])
  {
    rc |= inv_imu_edmp_enable_tilt(&icm_driver);
  }
  if(apex_enable[ICM456XX_APEX_PEDOMETER])
  {
    rc |= inv_imu_edmp_enable_pedometer(&icm_driver);
  }
  if(apex_enable[ICM456XX_APEX_R2W])
  {
    rc |= inv_imu_edmp_get_apex_parameters(&icm_driver, &apex_parameters);
    // R2W 睡眠超时使用底层参数单位，当前默认 6.4 s；后续可封装到参数结构体。
    apex_parameters.r2w_sleep_time_out = 6400; /* 6.4 s */
    rc |= inv_imu_edmp_set_apex_parameters(&icm_driver, &apex_parameters);
    rc |= inv_imu_edmp_enable_r2w(&icm_driver);
  }  
  if(apex_enable[ICM456XX_APEX_TAP])
  {
    rc |= inv_imu_edmp_get_apex_parameters(&icm_driver, &apex_parameters);
    // Tap 参数与 400 Hz ODR 匹配；如果未来开放 ODR 参数，这里也必须同步切换参数表。
    apex_parameters.tap_tmax             = TAP_TMAX_400HZ;
    apex_parameters.tap_tmin             = TAP_TMIN_400HZ;
    apex_parameters.tap_smudge_reject_th = TAP_SMUDGE_REJECT_THR_400HZ;
    rc |= inv_imu_edmp_set_apex_parameters(&icm_driver, &apex_parameters);
    rc |= inv_imu_edmp_enable_tap(&icm_driver);
  }
#endif /* INV_TYPE_A1 & INV_TYPE_A2*/
  if(apex_enable[ICM456XX_APEX_FF] || 
     apex_enable[ICM456XX_APEX_HIGHG] || 
     apex_enable[ICM456XX_APEX_LOWG])
  {
    rc |= inv_imu_edmp_enable_ff(&icm_driver);
  }

  rc |= inv_imu_edmp_enable(&icm_driver);
  rc |= inv_imu_adv_enable_accel_ln(&icm_driver);

  return rc;
}

int ICM456xx::startTiltDetection(uint8_t intpin, ICM456xx_irq_handler handler)
{
  int rc = 0;

#if (INV_DEVICE_TYPE == INV_TYPE_B1) || (INV_DEVICE_TYPE == INV_TYPE_C1)
  return 0;
#endif

  // 先记录功能请求；如果 handler 为空，调用者可以继续启用其他 APEX 功能后统一 startAPEX()。
  apex_enable[ICM456XX_APEX_TILT] = true;

  if(handler != NULL)
  {
    rc |= setApexInterrupt(intpin, handler);
    rc |= startAPEX(DMP_EXT_SEN_ODR_CFG_APEX_ODR_50_HZ,ACCEL_CONFIG0_ACCEL_ODR_50_HZ);
  }
  return rc;
}

int ICM456xx::startPedometer(uint8_t intpin, ICM456xx_irq_handler handler)
{
  int rc = 0;
  
#if (INV_DEVICE_TYPE == INV_TYPE_B1) || (INV_DEVICE_TYPE == INV_TYPE_C1)
	return 0;
#endif

  // 计步器是 eDMP 事件输出，不参与飞控姿态/导航主链路。
  apex_enable[ICM456XX_APEX_PEDOMETER] = true;

  // step_cnt 是 16-bit 输出，溢出次数在 Arduino 层累计到 step_cnt_ovflw。
  step_cnt_ovflw = 0;
  if(handler != NULL)
  {
    rc |= setApexInterrupt(intpin, handler);
    rc |= startAPEX(DMP_EXT_SEN_ODR_CFG_APEX_ODR_50_HZ,ACCEL_CONFIG0_ACCEL_ODR_50_HZ);
  }
  return rc;
}

int ICM456xx::startRaiseToWake(uint8_t intpin, ICM456xx_irq_handler handler)
{
  int rc = 0;

#if (INV_DEVICE_TYPE == INV_TYPE_B1) || (INV_DEVICE_TYPE == INV_TYPE_C1)
	return 0;
#endif

  apex_enable[ICM456XX_APEX_R2W] = true;

  if(handler != NULL)
  {
    rc |= setApexInterrupt(intpin, handler);
    rc |= startAPEX(DMP_EXT_SEN_ODR_CFG_APEX_ODR_100_HZ,ACCEL_CONFIG0_ACCEL_ODR_100_HZ);
  }
  return rc;
}

int ICM456xx::getTilt(void)
{
  if(apex_enable[ICM456XX_APEX_TILT] == false)
    return INV_ERROR;

#if (INV_DEVICE_TYPE == INV_TYPE_A1) || (INV_DEVICE_TYPE == INV_TYPE_A2)
  // 先刷新 APEX 状态缓存；读取到事件后清零，保证上层看到的是边沿式事件。
  updateApex();
  if (apex_status.INV_TILT_DET)
  {
    apex_status.INV_TILT_DET = 0;
    return true;
  }
  return false;
#endif
}

int ICM456xx::getPedometer(uint32_t& step_count, float& step_cadence, char*& activity)
{
  int rc = 0;

  if(apex_enable[ICM456XX_APEX_PEDOMETER] == false)
    return INV_ERROR;

#if (INV_DEVICE_TYPE == INV_TYPE_A1) || (INV_DEVICE_TYPE == INV_TYPE_A2)
  rc |= updateApex();

  /* Pedometer */
  if (apex_status.INV_STEP_CNT_OVFL) {
    // 底层步数计数溢出时只给状态位，Arduino 层自己扩展成 32-bit 计数。
    apex_status.INV_STEP_CNT_OVFL = 0;
    step_cnt_ovflw++;
  }
  if (apex_status.INV_STEP_DET) {
    inv_imu_edmp_pedometer_data_t ped_data;
    apex_status.INV_STEP_DET = 0;

    rc |= inv_imu_edmp_get_pedometer_data(&icm_driver, &ped_data);
    if (rc == INV_IMU_OK) {
      step_count = (uint32_t)ped_data.step_cnt + (step_cnt_ovflw * UINT16_MAX);  
      if(ped_data.step_cadence != 0)
      {
        // step_cadence 是底层算法周期量，这里换算成步频；该路径只用于诊断显示。
        step_cadence = 200 / (float)ped_data.step_cadence;
      } else {
        step_cadence = 0;
      }
      activity = ped_data.activity_class == INV_IMU_EDMP_RUN ?
                                   (char*)"Run" :
                                   (ped_data.activity_class == INV_IMU_EDMP_WALK ?
                                        (char*)"Walk" :
                                        /* INV_IMU_EDMP_UNKOWN */ (char*)"Unknown");
      return true;
    } else {
        return INV_ERROR;
    }
  } else {
    return false;
  }
#endif /* INV_TYPE_A1 & INV_TYPE_A2 */
}

int ICM456xx::getRaiseToWake(void)
{
  if(apex_enable[ICM456XX_APEX_R2W] == false)
    return INV_ERROR;

#if (INV_DEVICE_TYPE == INV_TYPE_A1) || (INV_DEVICE_TYPE == INV_TYPE_A2)
  updateApex();
  if (apex_status.INV_R2W)
  {
    apex_status.INV_R2W = 0;
    return 1;
  } else if (apex_status.INV_R2W_SLEEP) {
    apex_status.INV_R2W_SLEEP = 0;
    return 2;
  } else {
    return false;
  }
#endif
}

int ICM456xx::startFreeFall(uint8_t intpin, ICM456xx_irq_handler handler)
{
  int rc = 0;

  apex_enable[ICM456XX_APEX_FF] = true;

  if(handler != NULL)
  {
    rc |= setApexInterrupt(intpin, handler);
    rc |= startAPEX(DMP_EXT_SEN_ODR_CFG_APEX_ODR_400_HZ,ACCEL_CONFIG0_ACCEL_ODR_400_HZ);
  }
  return rc;
}

int ICM456xx::startHighG(uint8_t intpin, ICM456xx_irq_handler handler)
{
  int rc = 0;

  apex_enable[ICM456XX_APEX_HIGHG] = true;

  if(handler != NULL)
  {  
    rc |= setApexInterrupt(intpin, handler);
    rc |= startAPEX(DMP_EXT_SEN_ODR_CFG_APEX_ODR_400_HZ,ACCEL_CONFIG0_ACCEL_ODR_400_HZ);
  }
  return rc;
}

int ICM456xx::startLowG(uint8_t intpin, ICM456xx_irq_handler handler)
{
  int rc = 0;

  apex_enable[ICM456XX_APEX_LOWG] = true;

  if(handler != NULL)
  {
    rc |= setApexInterrupt(intpin, handler);
    rc |= startAPEX(DMP_EXT_SEN_ODR_CFG_APEX_ODR_400_HZ,ACCEL_CONFIG0_ACCEL_ODR_400_HZ);
  }
  return rc;
}

int ICM456xx::startWakeOnMotion(uint8_t intpin, ICM456xx_irq_handler handler, int odr)
{
  // 兼容旧整数接口：保留旧默认阈值和 AND 模式，但把 ODR 通过旧映射函数转成强类型枚举。
  ICM45686WomConfig config;
  config.intpin = intpin;
  config.handler = handler;
  config.threshold_x_mg = DEFAULT_WOM_THS_MG;
  config.threshold_y_mg = DEFAULT_WOM_THS_MG;
  config.threshold_z_mg = DEFAULT_WOM_THS_MG;
  config.use_or_mode = false;
  config.duration_samples = 1;
  config.accel_odr = (ICM45686AccelOdr)accel_freq_to_param((uint16_t)odr);
  return startWakeOnMotion(config);
}

int ICM456xx::startWakeOnMotion(const ICM45686WomConfig& config)
{
  int rc = 0;
  inv_imu_int_state_t config_int;

  if (config.handler == NULL) {
    return INV_IMU_ERROR_BAD_ARG;
  }

  // WOM 使用独立 X/Y/Z 中断源，不走 EDMP_EVENT，避免和 APEX 事件状态混用。
  memset(&config_int, INV_IMU_DISABLE, sizeof(config_int));
  config_int.INV_WOM_X = INV_IMU_ENABLE;
  config_int.INV_WOM_Y = INV_IMU_ENABLE;
  config_int.INV_WOM_Z = INV_IMU_ENABLE;
  config_int.INV_EDMP_EVENT = INV_IMU_DISABLE;
  rc |= setup_irq(config.intpin, config.handler, INV_IMU_INT1, config_int);

  /* Select WUOSC clock to have accel in ULP (lowest power mode) */
  rc |= inv_imu_select_accel_lp_clk(&icm_driver, SMC_CONTROL_0_ACCEL_LP_CLK_WUOSC);

  /* Set AVG to 1x */
  rc |= inv_imu_set_accel_lp_avg(&icm_driver, IPREG_SYS2_REG_129_ACCEL_LP_AVG_1);

  rc |= inv_imu_adv_configure_wom(&icm_driver,
          // 对外配置单位为 mg；底层寄存器单位为 1g/256，这里集中换算避免调用者误填。
          womThresholdMgToReg(config.threshold_x_mg),
          womThresholdMgToReg(config.threshold_y_mg),
          womThresholdMgToReg(config.threshold_z_mg),
          config.use_or_mode ? TMST_WOM_CONFIG_WOM_INT_MODE_ORED : TMST_WOM_CONFIG_WOM_INT_MODE_ANDED,
          wom_duration_to_param(config.duration_samples));
  rc |= inv_imu_adv_enable_wom(&icm_driver);
  rc |= inv_imu_set_accel_frequency(&icm_driver, (accel_config0_accel_odr_t)config.accel_odr);
  rc |= inv_imu_adv_enable_accel_lp(&icm_driver);

  return rc;
}

int ICM456xx::startTap(uint8_t intpin, ICM456xx_irq_handler handler)
{
  int rc = 0;

  apex_enable[ICM456XX_APEX_TAP] = true;

  if(handler != NULL)
  {
    rc |= setApexInterrupt(intpin, handler);
    rc |= startAPEX(DMP_EXT_SEN_ODR_CFG_APEX_ODR_400_HZ,ACCEL_CONFIG0_ACCEL_ODR_400_HZ);
  }
  return rc;
}

int ICM456xx::startB2S(uint8_t intpin, ICM456xx_irq_handler handler)
{
  int rc = 0;

  apex_enable[ICM456XX_APEX_B2S] = true;

  if(handler != NULL)
  {  
    rc |= setApexInterrupt(intpin, handler);
    rc |= startAPEX(DMP_EXT_SEN_ODR_CFG_APEX_ODR_400_HZ,ACCEL_CONFIG0_ACCEL_ODR_400_HZ);
  }
  return rc;
}

int ICM456xx::startAID(uint8_t intpin, ICM456xx_irq_handler handler)
{
  int rc = 0;

  apex_enable[ICM456XX_APEX_AID] = true;

  if(handler != NULL)
  {  
    rc |= setApexInterrupt(intpin, handler);
    rc |= startAPEX(DMP_EXT_SEN_ODR_CFG_APEX_ODR_400_HZ,ACCEL_CONFIG0_ACCEL_ODR_400_HZ);
  }
  return rc;
}


int ICM456xx::getTap(uint8_t& tap_count, uint8_t& axis, uint8_t& direction)
{
  int rc = 0;

  if(apex_enable[ICM456XX_APEX_TAP] == false)
    return INV_ERROR;

  rc |= updateApex();

  if (apex_status.INV_TAP) {
    inv_imu_edmp_tap_data_t tap_data;
    apex_status.INV_TAP = 0;
    rc |= inv_imu_edmp_get_tap_data(&icm_driver, &tap_data);
    tap_count = tap_data.num;
    axis = tap_data.axis;
    direction = tap_data.direction;
    return true;
  } else {
    return false;
  }
}

int ICM456xx::getFreefall(uint32_t& duration_ms)
{
  int rc = 0;

  if(apex_enable[ICM456XX_APEX_FF] == false)
    return INV_ERROR;

  rc |= updateApex();

  if (apex_status.INV_FF) {
	uint16_t duration;
    apex_status.INV_FF = 0;
    rc |= inv_imu_edmp_get_ff_data(&icm_driver, &duration);
	duration_ms = (duration * 2500) / 1000;
    return true;
  } else {
    return false;
  }
}

int ICM456xx::getHighG(void)
{
  int rc = 0;

  if(apex_enable[ICM456XX_APEX_HIGHG] == false)
    return INV_ERROR;

  rc |= updateApex();

  if (apex_status.INV_HIGHG) {
    apex_status.INV_HIGHG = 0;
    return true;
  } else {
    return false;
  }
}

int ICM456xx::getLowG(void)
{
  int rc = 0;

  if(apex_enable[ICM456XX_APEX_LOWG] == false)
    return INV_ERROR;

  rc |= updateApex();

  if (apex_status.INV_LOWG) {
    apex_status.INV_LOWG = 0;  	
    return true;
  } else {
    return false;
  }
}

int ICM456xx::getB2S(void)
{
  int rc = 0;

  if(apex_enable[ICM456XX_APEX_B2S] == false)
    return INV_ERROR;

  rc |= updateApex();

  if (apex_status.INV_B2S) {
    apex_status.INV_B2S = 0;
    return 1;
  } 

  if(apex_status.INV_B2S_REV) {
    apex_status.INV_B2S_REV= 0;
    return 2;
  }

  return false;
}

int ICM456xx::getAID_Device(void)
{
  int rc = 0;
  uint8_t output_state;

  if(apex_enable[ICM456XX_APEX_AID] == false)
    return INV_ERROR;

  rc |= updateApex();

#if (INV_DEVICE_TYPE == INV_TYPE_B1)
  if (apex_status.INV_AID_DEVICE) {
    apex_status.INV_AID_DEVICE = 0;
	rc |= inv_imu_edmp_get_aid_data_device(&icm_driver, &output_state);
	/*	Decision taken by the algorithm,
	    1 => activity is detected, 
	    2=> inactivity is detected, 
	    6=> inactivity and sedentary alert are detected */
    return output_state;
  }
#endif
  return false;
}

int ICM456xx::getAID_Human(void)
{
  int rc = 0;
  uint8_t output_state;

  if(apex_enable[ICM456XX_APEX_AID] == false)
    return INV_ERROR;

  rc |= updateApex();

#if (INV_DEVICE_TYPE == INV_TYPE_B1)
  if (apex_status.INV_AID_HUMAN) {
    apex_status.INV_AID_HUMAN = 0;
	rc |= inv_imu_edmp_get_aid_data_human(&icm_driver, &output_state);
	/*	Decision taken by the algorithm,
	    1 => activity is detected, 
	    2=> inactivity is detected, 
	    6=> inactivity and sedentary alert are detected */
    return output_state;
  }
#endif
  return false;
}

int ICM456xx::updateApex(void)
{
  int rc = 0;
  inv_imu_int_state_t      int_state;
  inv_imu_edmp_int_state_t apex_state = { 0 };

  /* Read interrupt status */
  rc |= inv_imu_get_int_status(&icm_driver, INV_IMU_INT1, &int_state);

  /*
   * 只有主中断状态表明有 EDMP_EVENT 时才读取 APEX 状态寄存器。
   * APEX 状态读出后用 OR 累积到 apex_status，避免多个事件在上层轮询前被覆盖。
   */
  if (int_state.INV_EDMP_EVENT) {
    /* Read APEX interrupt status */
    rc |= inv_imu_edmp_get_int_apex_status(&icm_driver, &apex_state);
#if (INV_DEVICE_TYPE == INV_TYPE_A1) || (INV_DEVICE_TYPE == INV_TYPE_A2)
    apex_status.INV_STEP_CNT_OVFL |= apex_state.INV_STEP_CNT_OVFL;
    apex_status.INV_STEP_DET |= apex_state.INV_STEP_DET;
    apex_status.INV_TILT_DET |= apex_state.INV_TILT_DET;
    apex_status.INV_R2W |= apex_state.INV_R2W;
    apex_status.INV_R2W_SLEEP |= apex_state.INV_R2W_SLEEP;
#elif (INV_DEVICE_TYPE == INV_TYPE_B1) || (INV_DEVICE_TYPE == INV_TYPE_C1)
    apex_status.INV_AID_DEVICE |= apex_state.INV_AID_DEVICE;
    apex_status.INV_AID_HUMAN  |= apex_state.INV_AID_HUMAN;
#endif
    apex_status.INV_FF      |= apex_state.INV_FF;
    apex_status.INV_HIGHG   |= apex_state.INV_HIGHG;
    apex_status.INV_LOWG    |= apex_state.INV_LOWG;
    apex_status.INV_TAP     |= apex_state.INV_TAP;
    apex_status.INV_B2S     |= apex_state.INV_B2S;
    apex_status.INV_B2S_REV |= apex_state.INV_B2S_REV;

  }
  return rc;
}

int ICM456xx::readFifoFull(void)
{
  inv_imu_int_state_t int_state;
  const int status = inv_imu_get_int_status(&icm_driver, INV_IMU_INT1, &int_state);
  if (status != INV_IMU_OK) {
    return status; // 负数错误码
  }
  return int_state.INV_FIFO_FULL;
}

int ICM456xx::enableFifoFullStatus(void)
{
  // INT1_CONFIG0 (0x16) bit 0: INT1_STATUS_EN_FIFO_FULL
  // 必须使能此位，INT1_STATUS0.FIFO_FULL 才会在 FIFO 满时被置位。
  // ICM45686 的 INT1_CONFIG2 复位值为 0x04（open-drain / pulse / active-low）。
  // 这里改为 push-pull / pulse / active-low，仅用于状态位锁存配置，不依赖物理中断引脚。
  uint8_t val = 0;
  int rc = readRegister(0x18, val);
  if (rc != INV_IMU_OK) return rc;
  // INT1_CONFIG2: bit2=INT1_DRIVE=0(push-pull), bit1=INT1_MODE=0(pulse), bit0=INT1_POLARITY=0(active-low)
  // 当前仅用于状态位跟踪，不依赖物理中断引脚，极性设置不影响 readFifoFull() 功能。
  val &= ~0x07;    // clear DRIVE/MODE/POLARITY → push-pull, pulse, active-low
  rc = writeRegister(0x18, val);
  if (rc != INV_IMU_OK) return rc;

  // INT1_CONFIG0: enable FIFO_FULL status only (bit 0)
  rc = readRegister(0x16, val);
  if (rc != INV_IMU_OK) return rc;
  val |= 0x01;     // set INT1_STATUS_EN_FIFO_FULL
  return writeRegister(0x16, val);
}


static int i2c_write(void *context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
  ICM456xx *instance = (ICM456xx *)context;

  if ((instance == NULL) || (instance->i2c == NULL)) {
    return INV_IMU_ERROR_BAD_ARG;
  }

  // Arduino Wire 写事务：先写寄存器地址，再连续写入数据 payload。
  instance->i2c->beginTransmission(instance->i2c_address);
  instance->i2c->write(reg);
  for(uint8_t i = 0; i < wlen; i++) {
    instance->i2c->write(wbuffer[i]);
  }
  const int tx_status = instance->i2c->endTransmission();
  // Wire 能报告 ACK/NACK 等事务状态，转换成底层统一错误码。
  return (tx_status == 0) ? INV_IMU_OK : INV_IMU_ERROR_TRANSPORT;
}

/*
 * I2C 读回调。
 *
 * ICM45686 的底层 C driver 可能请求超过 Arduino Wire 缓冲区的数据，因此这里按
 * ARDUINO_I2C_BUFFER_LENGTH 分片读取。每个分片都检查 requestFrom() 返回长度和 available()
 * 超时，避免设备掉线或线被拉低时永久阻塞。
 */
static int i2c_read(void *context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
  uint16_t offset = 0;
  ICM456xx *instance = (ICM456xx *)context;

  if ((instance == NULL) || (instance->i2c == NULL) || (rbuffer == NULL)) {
    return INV_IMU_ERROR_BAD_ARG;
  }

  instance->i2c->beginTransmission(instance->i2c_address);
  instance->i2c->write(reg);
  // repeated-start：保持总线事务连续，符合常见寄存器读协议。
  const int tx_status = instance->i2c->endTransmission(false);
  if (tx_status != 0) {
    return INV_IMU_ERROR_TRANSPORT;
  }

  while(offset < rlen)
  {
    uint16_t rx_bytes = 0;
    // Wire 单次读取长度不能超过底层缓冲区，长读必须分片。
    uint8_t length = ((rlen - offset) > ARDUINO_I2C_BUFFER_LENGTH) ? ARDUINO_I2C_BUFFER_LENGTH : (rlen - offset) ;
    rx_bytes = instance->i2c->requestFrom(instance->i2c_address, length);
    if (rx_bytes != length) {
      return INV_IMU_ERROR_TRANSPORT;
    }

    const uint32_t start_ms = millis();
    while (instance->i2c->available() < length) {
      // 防止 I2C 设备异常时 read() 等待不到足够字节而卡住主循环。
      if ((millis() - start_ms) >= ICM456XX_I2C_READ_TIMEOUT_MS) {
        return INV_IMU_ERROR_TIMEOUT;
      }
      delay(1);
    }

    for(uint8_t i = 0; i < length; i++) {
      rbuffer[offset+i] = instance->i2c->read();
    }
    offset += length;
  }

  return (offset == rlen) ? INV_IMU_OK : INV_IMU_ERROR_TRANSPORT;
}

static int spi_write(void *context, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
  ICM456xx *instance = (ICM456xx *)context;

  if ((instance == NULL) || (instance->spi == NULL)) {
    return INV_IMU_ERROR_BAD_ARG;
  }

  // ICM45686 主 SPI 使用 mode 3；频率来自对象实例，避免多个对象共享全局频率。
  instance->spi->beginTransaction(SPISettings(instance->interface_clock_hz, MSBFIRST, SPI_MODE3));
  digitalWrite(instance->chip_select_id,LOW);
  // 写事务最高位保持 0，随后连续发送 payload。
  instance->spi->transfer(reg);
  for(uint8_t i = 0; i < wlen; i++) {
    instance->spi->transfer(wbuffer[i]);
  }
  digitalWrite(instance->chip_select_id,HIGH);
  instance->spi->endTransaction();
  // Arduino SPI transfer() 通常没有错误返回，链路健康需要靠 WHO_AM_I/写后读回/数据异常检测。
  return 0;
}

/*
 * SPI 读回调。
 *
 * 读寄存器时地址最高位置 1，然后连续 clock 出 rlen 个字节。裸 SPI 无 ACK/NACK，CS、MISO
 * 断线等问题不会在这里直接报错，必须由上层通过 WHO_AM_I、配置读回和数据合理性判断发现。
 */
static int spi_read(void *context, uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
  ICM456xx *instance = (ICM456xx *)context;

  if ((instance == NULL) || (instance->spi == NULL) || (rbuffer == NULL)) {
    return INV_IMU_ERROR_BAD_ARG;
  }

  instance->spi->beginTransaction(SPISettings(instance->interface_clock_hz, MSBFIRST, SPI_MODE3));
  digitalWrite(instance->chip_select_id,LOW);
  // ICM45686 SPI 读标志为 bit7=1。
  instance->spi->transfer(reg | SPI_READ);
  instance->spi->transfer(rbuffer,rlen);
  digitalWrite(instance->chip_select_id,HIGH);
  instance->spi->endTransaction();
  return 0;
}

static int16_t decode_fifo_int16(uint8_t endianness_data, const uint8_t *src)
{
  if (endianness_data == SREG_CTRL_SREG_DATA_BIG_ENDIAN) {
    return static_cast<int16_t>((static_cast<uint16_t>(src[0]) << 8) | src[1]);
  }
  return static_cast<int16_t>((static_cast<uint16_t>(src[1]) << 8) | src[0]);
}

static void decode_fifo_frame_from_buffer(inv_imu_device_t *driver,
                                          const uint8_t *frame,
                                          inv_imu_fifo_data_t *data)
{
  // 这里复刻的是当前测试所用“原始 accel/gyro/temp/timestamp FIFO frame”的拆包逻辑，
  // 不是把底层 driver 的所有高级 FIFO 事件语义都重新实现一遍。
  // 因此后续如果启用 APEX/eDMP/外部传感器扩展 frame，必须回头核对这里的布局假设。
  data->header.Byte = frame[0];

  switch (driver->fifo_frame_size) {
    case 8:
      data->byte_8.sensor_data[0] = decode_fifo_int16(driver->endianness_data, &frame[1]);
      data->byte_8.sensor_data[1] = decode_fifo_int16(driver->endianness_data, &frame[3]);
      data->byte_8.sensor_data[2] = decode_fifo_int16(driver->endianness_data, &frame[5]);
      data->byte_8.temp_data = static_cast<int8_t>(frame[7]);
      break;
    case 16:
      data->byte_16.accel_data[0] = decode_fifo_int16(driver->endianness_data, &frame[1]);
      data->byte_16.accel_data[1] = decode_fifo_int16(driver->endianness_data, &frame[3]);
      data->byte_16.accel_data[2] = decode_fifo_int16(driver->endianness_data, &frame[5]);
      data->byte_16.gyro_data[0] = decode_fifo_int16(driver->endianness_data, &frame[7]);
      data->byte_16.gyro_data[1] = decode_fifo_int16(driver->endianness_data, &frame[9]);
      data->byte_16.gyro_data[2] = decode_fifo_int16(driver->endianness_data, &frame[11]);
      data->byte_16.temp_data = static_cast<int8_t>(frame[13]);
      data->byte_16.timestamp =
          static_cast<uint16_t>(decode_fifo_int16(driver->endianness_data, &frame[14]));
      break;
    case 20: {
      const int16_t accel_x = decode_fifo_int16(driver->endianness_data, &frame[1]);
      const int16_t accel_y = decode_fifo_int16(driver->endianness_data, &frame[3]);
      const int16_t accel_z = decode_fifo_int16(driver->endianness_data, &frame[5]);
      const int16_t gyro_x = decode_fifo_int16(driver->endianness_data, &frame[7]);
      const int16_t gyro_y = decode_fifo_int16(driver->endianness_data, &frame[9]);
      const int16_t gyro_z = decode_fifo_int16(driver->endianness_data, &frame[11]);
      data->byte_20.accel_data[0] = (static_cast<int32_t>(accel_x) << 4) | (frame[17] >> 4);
      data->byte_20.accel_data[1] = (static_cast<int32_t>(accel_y) << 4) | (frame[18] >> 4);
      data->byte_20.accel_data[2] = (static_cast<int32_t>(accel_z) << 4) | (frame[19] >> 4);
      data->byte_20.gyro_data[0] = (static_cast<int32_t>(gyro_x) << 4) | (frame[17] & 0x0F);
      data->byte_20.gyro_data[1] = (static_cast<int32_t>(gyro_y) << 4) | (frame[18] & 0x0F);
      data->byte_20.gyro_data[2] = (static_cast<int32_t>(gyro_z) << 4) | (frame[19] & 0x0F);
      data->byte_20.temp_data = decode_fifo_int16(driver->endianness_data, &frame[13]);
      data->byte_20.timestamp =
          static_cast<uint16_t>(decode_fifo_int16(driver->endianness_data, &frame[15]));
      break;
    }
    default:
      data->header.Byte = 0;
      break;
  }
}

accel_config0_accel_ui_fs_sel_t ICM456xx::accel_fsr_g_to_param(uint16_t accel_fsr_g) {
  accel_config0_accel_ui_fs_sel_t ret = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_16_G;

  switch(accel_fsr_g) {
  case 2:  ret = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_2_G;  break;
  case 4:  ret = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_4_G;  break;
  case 8:  ret = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_8_G;  break;
  case 16: ret = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_16_G; break;
#if INV_IMU_HIGH_FSR_SUPPORTED
  case 32: ret = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_32_G; break;
#endif
  default:
    /* Unknown accel FSR. Set to default 16G */
    break;
  }
  return ret;
}

gyro_config0_gyro_ui_fs_sel_t ICM456xx::gyro_fsr_dps_to_param(uint16_t gyro_fsr_dps) {
  gyro_config0_gyro_ui_fs_sel_t ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS;

  switch(gyro_fsr_dps) {
  // 兼容旧整数 API：这些整数不是精确物理值，而是手册小数量程的简写。
  case 15:  ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_15_625_DPS;  break;
  case 31:  ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_31_25_DPS;  break;
  case 62:  ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_62_5_DPS;  break;
  case 125:  ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_125_DPS;  break;
  case 250:  ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_250_DPS;  break;
  case 500:  ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_500_DPS;  break;
  case 1000: ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_1000_DPS; break;
  case 2000: ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS; break;
#if INV_IMU_HIGH_FSR_SUPPORTED
  case 4000: ret = GYRO_CONFIG0_GYRO_UI_FS_SEL_4000_DPS; break;
#endif
  default:
    /* Unknown gyro FSR. Set to default 2000dps" */
    break;
  }
  return ret;
}

accel_config0_accel_odr_t ICM456xx::accel_freq_to_param(uint16_t accel_freq_hz) {
  accel_config0_accel_odr_t ret = ACCEL_CONFIG0_ACCEL_ODR_100_HZ;

  switch(accel_freq_hz) {
  // 兼容旧整数 API：1/3/6/12 分别映射到 1.5625/3.125/6.25/12.5 Hz。
  case 1:    ret = ACCEL_CONFIG0_ACCEL_ODR_1_5625_HZ;  break;
  case 3:    ret = ACCEL_CONFIG0_ACCEL_ODR_3_125_HZ;  break;
  case 6:    ret = ACCEL_CONFIG0_ACCEL_ODR_6_25_HZ;  break;
  case 12:   ret = ACCEL_CONFIG0_ACCEL_ODR_12_5_HZ;  break;
  case 25:   ret = ACCEL_CONFIG0_ACCEL_ODR_25_HZ;  break;
  case 50:   ret = ACCEL_CONFIG0_ACCEL_ODR_50_HZ;  break;
  case 100:  ret = ACCEL_CONFIG0_ACCEL_ODR_100_HZ; break;
  case 200:  ret = ACCEL_CONFIG0_ACCEL_ODR_200_HZ; break;
  case 400:  ret = ACCEL_CONFIG0_ACCEL_ODR_400_HZ; break;
  case 800:  ret = ACCEL_CONFIG0_ACCEL_ODR_800_HZ; break;
  case 1600: ret = ACCEL_CONFIG0_ACCEL_ODR_1600_HZ;  break;
  case 3200: ret = ACCEL_CONFIG0_ACCEL_ODR_3200_HZ;  break;
  case 6400: ret = ACCEL_CONFIG0_ACCEL_ODR_6400_HZ;  break;
  default:
    /* Unknown accel frequency. Set to default 100Hz */
    break;
  }
  return ret;
}

gyro_config0_gyro_odr_t ICM456xx::gyro_freq_to_param(uint16_t gyro_freq_hz) {
  gyro_config0_gyro_odr_t ret = GYRO_CONFIG0_GYRO_ODR_100_HZ;

  switch(gyro_freq_hz) {
  // 兼容旧整数 API：1/3/6/12 分别映射到 1.5625/3.125/6.25/12.5 Hz。
  case 1:   ret = GYRO_CONFIG0_GYRO_ODR_1_5625_HZ;  break;
  case 3:   ret = GYRO_CONFIG0_GYRO_ODR_3_125_HZ;  break;
  case 6:   ret = GYRO_CONFIG0_GYRO_ODR_6_25_HZ;  break;
  case 12:   ret = GYRO_CONFIG0_GYRO_ODR_12_5_HZ;  break;
  case 25:   ret = GYRO_CONFIG0_GYRO_ODR_25_HZ;  break;
  case 50:   ret = GYRO_CONFIG0_GYRO_ODR_50_HZ;  break;
  case 100:  ret = GYRO_CONFIG0_GYRO_ODR_100_HZ; break;
  case 200:  ret = GYRO_CONFIG0_GYRO_ODR_200_HZ; break;
  case 400:  ret = GYRO_CONFIG0_GYRO_ODR_400_HZ; break;
  case 800:  ret = GYRO_CONFIG0_GYRO_ODR_800_HZ; break;
  case 1600: ret = GYRO_CONFIG0_GYRO_ODR_1600_HZ;  break;
  case 3200: ret = GYRO_CONFIG0_GYRO_ODR_3200_HZ;  break;
  case 6400: ret = GYRO_CONFIG0_GYRO_ODR_6400_HZ;  break;
  default:
    /* Unknown gyro ODR. Set to default 100Hz */
    break;
  }
  return ret;
}

float ICM456xx::accelFsrToG(ICM45686AccelFsr fsr) {
  switch (fsr) {
  case ICM45686AccelFsr::G2:  return 2.0f;
  case ICM45686AccelFsr::G4:  return 4.0f;
  case ICM45686AccelFsr::G8:  return 8.0f;
  case ICM45686AccelFsr::G16: return 16.0f;
#if INV_IMU_HIGH_FSR_SUPPORTED
  case ICM45686AccelFsr::G32: return 32.0f;
#endif
  default: return 16.0f;
  }
}

float ICM456xx::gyroFsrToDps(ICM45686GyroFsr fsr) {
  switch (fsr) {
  case ICM45686GyroFsr::Dps15_625: return 15.625f;
  case ICM45686GyroFsr::Dps31_25:  return 31.25f;
  case ICM45686GyroFsr::Dps62_5:   return 62.5f;
  case ICM45686GyroFsr::Dps125:    return 125.0f;
  case ICM45686GyroFsr::Dps250:    return 250.0f;
  case ICM45686GyroFsr::Dps500:    return 500.0f;
  case ICM45686GyroFsr::Dps1000:   return 1000.0f;
  case ICM45686GyroFsr::Dps2000:   return 2000.0f;
#if INV_IMU_HIGH_FSR_SUPPORTED
  case ICM45686GyroFsr::Dps4000:   return 4000.0f;
#endif
  default: return 2000.0f;
  }
}

float ICM456xx::rawRegisterAccelToG(int16_t raw, ICM45686AccelFsr fsr) {
  // UI 寄存器路径按当前用户配置 FSR 缩放，和 FIFO 20-bit 固定缩放不同。
  return (float)raw * accelFsrToG(fsr) / 32768.0f;
}

float ICM456xx::rawRegisterGyroToDps(int16_t raw, ICM45686GyroFsr fsr) {
  return (float)raw * gyroFsrToDps(fsr) / 32768.0f;
}

float ICM456xx::rawRegisterTempToC(int16_t raw) {
  return 25.0f + ((float)raw / 128.0f);
}

float ICM456xx::rawFifoTempToC(int8_t raw) {
  return 25.0f + ((float)raw / 2.0f);
}

float ICM456xx::rawFifo20AccelToG(int32_t raw) {
  // 20-bit FIFO 加速度按手册固定 ±32 g，分母为 2^19。
  return (float)raw * 32.0f / 524288.0f;
}

float ICM456xx::rawFifo20GyroToDps(int32_t raw) {
  // 20-bit FIFO 陀螺按手册固定 ±4000 dps，分母为 2^19。
  return (float)raw * 4000.0f / 524288.0f;
}

static odr_decimate_config_accel_fifo_odr_dec_t accel_decimation_to_param(uint16_t decimation) {
  switch (decimation) {
  case 1:    return ODR_DECIMATE_CONFIG_ACCEL_FIFO_ODR_DEC_1;
  case 2:    return ODR_DECIMATE_CONFIG_ACCEL_FIFO_ODR_DEC_2;
  case 4:    return ODR_DECIMATE_CONFIG_ACCEL_FIFO_ODR_DEC_4;
  case 8:    return ODR_DECIMATE_CONFIG_ACCEL_FIFO_ODR_DEC_8;
  case 16:   return ODR_DECIMATE_CONFIG_ACCEL_FIFO_ODR_DEC_16;
  case 32:   return ODR_DECIMATE_CONFIG_ACCEL_FIFO_ODR_DEC_32;
  case 64:   return ODR_DECIMATE_CONFIG_ACCEL_FIFO_ODR_DEC_64;
  case 128:  return ODR_DECIMATE_CONFIG_ACCEL_FIFO_ODR_DEC_128;
  case 256:  return ODR_DECIMATE_CONFIG_ACCEL_FIFO_ODR_DEC_256;
  case 512:  return ODR_DECIMATE_CONFIG_ACCEL_FIFO_ODR_DEC_512;
  case 1024: return ODR_DECIMATE_CONFIG_ACCEL_FIFO_ODR_DEC_1024;
  case 2048: return ODR_DECIMATE_CONFIG_ACCEL_FIFO_ODR_DEC_2048;
  case 4096: return ODR_DECIMATE_CONFIG_ACCEL_FIFO_ODR_DEC_4096;
  default:   return ODR_DECIMATE_CONFIG_ACCEL_FIFO_ODR_DEC_1;
  }
}

static odr_decimate_config_gyro_fifo_odr_dec_t gyro_decimation_to_param(uint16_t decimation) {
  switch (decimation) {
  case 1:    return ODR_DECIMATE_CONFIG_GYRO_FIFO_ODR_DEC_1;
  case 2:    return ODR_DECIMATE_CONFIG_GYRO_FIFO_ODR_DEC_2;
  case 4:    return ODR_DECIMATE_CONFIG_GYRO_FIFO_ODR_DEC_4;
  case 8:    return ODR_DECIMATE_CONFIG_GYRO_FIFO_ODR_DEC_8;
  case 16:   return ODR_DECIMATE_CONFIG_GYRO_FIFO_ODR_DEC_16;
  case 32:   return ODR_DECIMATE_CONFIG_GYRO_FIFO_ODR_DEC_32;
  case 64:   return ODR_DECIMATE_CONFIG_GYRO_FIFO_ODR_DEC_64;
  case 128:  return ODR_DECIMATE_CONFIG_GYRO_FIFO_ODR_DEC_128;
  case 256:  return ODR_DECIMATE_CONFIG_GYRO_FIFO_ODR_DEC_256;
  case 512:  return ODR_DECIMATE_CONFIG_GYRO_FIFO_ODR_DEC_512;
  case 1024: return ODR_DECIMATE_CONFIG_GYRO_FIFO_ODR_DEC_1024;
  case 2048: return ODR_DECIMATE_CONFIG_GYRO_FIFO_ODR_DEC_2048;
  case 4096: return ODR_DECIMATE_CONFIG_GYRO_FIFO_ODR_DEC_4096;
  default:   return ODR_DECIMATE_CONFIG_GYRO_FIFO_ODR_DEC_1;
  }
}

static tmst_wom_config_wom_int_dur_t wom_duration_to_param(uint8_t duration_samples) {
  switch (duration_samples) {
  case 1: return TMST_WOM_CONFIG_WOM_INT_DUR_1_SMPL;
  case 2: return TMST_WOM_CONFIG_WOM_INT_DUR_2_SMPL;
  case 3: return TMST_WOM_CONFIG_WOM_INT_DUR_3_SMPL;
  case 4: return TMST_WOM_CONFIG_WOM_INT_DUR_4_SMPL;
  default: return TMST_WOM_CONFIG_WOM_INT_DUR_1_SMPL;
  }
}

static void sleep_us(uint32_t us)
{
    delayMicroseconds(us);
}


