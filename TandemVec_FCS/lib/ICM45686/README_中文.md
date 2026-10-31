# ICM45686 Arduino/PlatformIO 自有驱动库说明

本文档面向后续飞控、传感器 bring-up、独立测试工程和其他 Arduino/PlatformIO 项目。当前库已经不再按“上游 vendor tree 不可修改”的方式维护，而是作为本项目自己的 ICM45686 驱动库使用。

## 1. 库定位

ICM45686 是 TDK InvenSense 的高性能六轴 IMU，提供三轴加速度、三轴角速度、温度、FIFO、APEX 事件检测和 AUX I2CM 外部传感器接口。本库提供两层接口：

| 层级 | 文件 | 适用场景 |
| --- | --- | --- |
| 底层 Arduino 封装 | `src/ICM45686.h/cpp` | 直接访问 `begin()`、`startAccel()`、`getDataFromRegisters()`、FIFO、APEX、I2CM 等原始能力 |
| 飞控适配层 | `src/ICM45686_FlightAdapter.h/cpp` | 飞控主循环需要稳定读取物理量、统计错误、判断数据新鲜度时优先使用 |

推荐策略：

- 新项目只需要六轴 IMU 数据：优先使用 `ICM45686_FlightAdapter`。
- 需要验证 FIFO、APEX、Wake on Motion 或 AUX 磁力计：使用底层 `ICM456xx` API。
- 后续接入飞控闭环前，必须先独立验证供电、通信、轴向、量程、采样周期、饱和情况和异常返回。

## 2. 当前库相对官方版本的改进

本库已经做了以下自有化改造：

- `library.properties` 改为自有库元数据，导出 `ICM45686.h` 和 `ICM45686_FlightAdapter.h`。
- `ICM456xx` 构造函数显式区分 SPI/I2C，避免未初始化指针导致 `begin()` 误判接口。
- 通信频率从全局静态变量改为实例字段 `interface_clock_hz`，避免多实例或重复构造时互相覆盖。
- I2C 读路径加入传输错误检查、`available()` 等待和超时，避免传感器掉线时无限阻塞。
- AUX I2CM 完成等待加入超时，避免外部磁力计未接或异常时卡死。
- 修复部分未初始化局部返回值和无用变量。
- `startAPEX()` 不再使用 `NULL` 作为枚举默认值，消除 C++ 指针算术警告。
- 新增 `ICM45686_FlightAdapter`，封装飞控常用的初始化、量纲转换、错误统计、饱和检测和数据新鲜度判断。
- 新增强类型 ODR/FSR API：`ICM45686AccelOdr`、`ICM45686GyroOdr`、`ICM45686AccelFsr`、`ICM45686GyroFsr`，避免 `1.5625 Hz`、`12.5 Hz`、`15.625 dps` 等手册小数档位被旧整数 API 表达成近似值。
- 新增公开寄存器诊断 API：`readWhoAmI()`、`readRegister()`、`readRegisters()`、`writeRegister()`、`writeRegisterChecked()`，便于按手册做配置读回和总线健康检查。
- 新增 `ICM45686FifoConfig`，可配置 FIFO 普通/20-bit 高分辨率格式、2KB/8KB 深度、watermark、stream/snapshot、timestamp/FSYNC 和 FIFO ODR decimation。
- 新增 `ICM45686WomConfig`，Wake on Motion 阈值对外统一使用 `mg`，库内按手册 `1g/256` 自动换算到寄存器 LSB。
- 新增 `configureDataReadyInterrupt()`、`runSelfTest()`，并在底层宏支持时公开 `configureInt2PinUsage()`、`configureFsync()`。
- 新增寄存器路径和 FIFO 路径的单位换算工具函数，VOFA+ 高频示例和后续飞控代码可以共用同一套缩放逻辑。

## 3. 硬件连接

### 3.1 SPI 连接

ICM45686 支持 SPI mode 3。本测试工程默认 ESP32-S3 引脚如下：

| 信号 | ESP32-S3 默认 GPIO | 说明 |
| --- | --- | --- |
| SCK | `12` | SPI 时钟 |
| MISO | `13` | IMU 到 MCU |
| MOSI | `11` | MCU 到 IMU |
| CS | `10` | 片选，低有效 |
| INT1 | `14` | FIFO/APEX/WOM 中断，可选 |
| 3V3 | 3.3 V | 按模块实际要求供电，不要误接 5 V |
| GND | GND | 必须共地 |

默认 SPI 频率为 `6000000` Hz。硬件走线短、模块质量好时可提高，但飞控项目应先在目标板上测量错误率和波形质量，不要直接拉到上限。

### 3.2 I2C 连接

I2C 默认地址为 `0x68`，`address_lsb=true` 时为 `0x69`。默认频率 `400000` Hz，允许 `100000` 到 `1000000` Hz。I2C 必须确认上拉电阻、线长和总线共用设备。

### 3.3 AUX I2CM/外部磁力计

库内 AUX 磁力计路径默认面向 ICT1531/兼容磁力计，地址为 `0x1E`。如果 AUX 口没有连接对应硬件，`setI2CM()`、`getDataFromI2CM()`、`getExternalMagData()` 返回错误是预期现象，不应误判为 ICM45686 主 SPI/I2C 通信失败。

## 4. PlatformIO 使用

把本库放在项目 `lib/ICM45686` 下，然后在源码中包含：

```cpp
#include "ICM45686.h"
#include "ICM45686_FlightAdapter.h"
```

典型 `platformio.ini` 环境示例：

```ini
[env:icm45686_spi_test]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
monitor_speed = 115200
build_flags =
    -D ICM45686_TEST_SPI_SCK=12
    -D ICM45686_TEST_SPI_MISO=13
    -D ICM45686_TEST_SPI_MOSI=11
    -D ICM45686_TEST_SPI_CS=10
    -D ICM45686_TEST_SPI_HZ=6000000
    -D ICM45686_TEST_INT_PIN=14
```

本仓库内已提供四个 ICM45686 独立测试/验证环境：

- `icm45686_spi_test`：最小 SPI 连通和寄存器轮询测试。
- `icm45686_spi_diagnostic`：串口命令式综合诊断台，覆盖寄存器流、FIFO、APEX、Wake on Motion、bias、外部 I2CM 预留测试。
- `icm45686_vofa_highrate`：高频原始 IMU 读取和 VOFA+ JustFloat 输出，默认 SPI `12 MHz`、串口 `2 Mbps`、读周期 `625 us`。
- `icm45686_api_compile_test`：只做编译期 API 覆盖，不访问硬件，用于防止公开接口回归。

## 5. 最小 SPI 读取示例

```cpp
#include <Arduino.h>
#include <SPI.h>
#include "ICM45686.h"

static ICM456xx imu(SPI, 10, 6000000);

void setup()
{
  Serial.begin(115200);
  SPI.begin(12, 13, 11, 10);

  int rc = imu.begin();
  Serial.print("begin=");
  Serial.println(rc);
  if (rc != 0) {
    return;
  }

  rc = imu.startAccel(ICM45686AccelOdr::Hz100, ICM45686AccelFsr::G16);
  Serial.print("startAccel=");
  Serial.println(rc);
  rc = imu.startGyro(ICM45686GyroOdr::Hz100, ICM45686GyroFsr::Dps2000);
  Serial.print("startGyro=");
  Serial.println(rc);
}

void loop()
{
  inv_imu_sensor_data_t raw {};
  int rc = imu.getDataFromRegisters(raw);
  if (rc != 0) {
    Serial.print("read error=");
    Serial.println(rc);
    delay(100);
    return;
  }

  const float ax_g = ICM456xx::rawRegisterAccelToG(raw.accel_data[0], ICM45686AccelFsr::G16);
  const float gx_dps = ICM456xx::rawRegisterGyroToDps(raw.gyro_data[0], ICM45686GyroFsr::Dps2000);

  Serial.print(raw.accel_data[0]);
  Serial.print(',');
  Serial.print(raw.accel_data[1]);
  Serial.print(',');
  Serial.print(raw.accel_data[2]);
  Serial.print(" ax_g=");
  Serial.print(ax_g, 6);
  Serial.print(" gyro=");
  Serial.print(raw.gyro_data[0]);
  Serial.print(',');
  Serial.print(raw.gyro_data[1]);
  Serial.print(',');
  Serial.print(raw.gyro_data[2]);
  Serial.print(" gx_dps=");
  Serial.println(gx_dps, 6);
  delay(10);
}
```

## 6. 飞控适配层推荐用法

飞控项目通常不希望主循环里散落原始寄存器结构、量纲换算和错误统计，因此推荐使用 `ICM45686_FlightAdapter`。

```cpp
#include <Arduino.h>
#include <SPI.h>
#include "ICM45686.h"
#include "ICM45686_FlightAdapter.h"

static ICM456xx imu(SPI, 10, 6000000);
static ICM45686FlightAdapter imu_fc(imu);

void setup()
{
  Serial.begin(115200);
  SPI.begin(12, 13, 11, 10);

  ICM45686FlightConfig cfg;
  cfg.accel_odr_hz = 200;
  cfg.gyro_odr_hz = 200;
  cfg.accel_fsr_g = 16;
  cfg.gyro_fsr_dps = 2000;
  cfg.stale_after_us = 10000;

  int rc = imu_fc.begin(cfg);
  Serial.print("imu_fc.begin=");
  Serial.println(rc);
}

void loop()
{
  ICM45686FlightSample sample;
  int rc = imu_fc.readSample(sample);
  if (rc != 0) {
    const ICM45686FlightStats &stats = imu_fc.stats();
    Serial.print("imu read error, count=");
    Serial.println(stats.read_errors);
    return;
  }

  if (!imu_fc.hasFreshSample(micros())) {
    Serial.println("imu stale");
    return;
  }

  Serial.print("ax_g=");
  Serial.print(sample.accel_g[0], 5);
  Serial.print(" gx_dps=");
  Serial.print(sample.gyro_dps[0], 5);
  Serial.print(" norm=");
  Serial.print(sample.accel_norm_g, 5);
  Serial.print(" saturated=");
  Serial.println(sample.saturated);
}
```

### 6.1 `ICM45686FlightConfig`

| 字段 | 默认值 | 说明 |
| --- | --- | --- |
| `accel_odr_hz` | `200` | 加速度计输出频率，底层会映射到芯片支持枚举 |
| `gyro_odr_hz` | `200` | 陀螺仪输出频率 |
| `accel_fsr_g` | `16` | 加速度量程，单位 g |
| `gyro_fsr_dps` | `2000` | 陀螺量程，单位 dps |
| `stale_after_us` | `10000` | 超过该时间没有成功样本即认为数据陈旧 |

### 6.2 `ICM45686FlightSample`

| 字段 | 说明 |
| --- | --- |
| `timestamp_us` | 成功读数后的 `micros()` 时间戳 |
| `raw_accel[3]` | 原始加速度计读数 |
| `raw_gyro[3]` | 原始陀螺读数 |
| `raw_temp` | 原始温度 |
| `accel_g[3]` | 已换算加速度，单位 g |
| `gyro_dps[3]` | 已换算角速度，单位 deg/s |
| `temperature_c` | 温度，单位摄氏度 |
| `accel_norm_g` | 加速度模长，静止时应接近 1 g |
| `saturated` | 任一原始轴接近满量程时置位 |
| `fresh` | 当前 `readSample()` 成功时置位 |

### 6.3 `ICM45686FlightStats`

| 字段 | 说明 |
| --- | --- |
| `successful_samples` | 成功样本数 |
| `read_errors` | 读取失败次数 |
| `saturated_samples` | 接近饱和样本数 |
| `last_status` | 最近一次底层返回值 |
| `last_sample_us` | 最近一次成功样本时间 |
| `dt_min_us` / `dt_max_us` / `dt_sum_us` | 样本间隔统计 |

## 7. 底层 API 总览

### 7.1 初始化与基础采样

| API | 功能 |
| --- | --- |
| `ICM456xx(TwoWire&, bool, uint32_t)` | I2C 构造 |
| `ICM456xx(SPIClass&, uint8_t, uint32_t)` | SPI 构造 |
| `begin()` | 初始化通信和底层 driver |
| `startAccel(uint16_t odr, uint16_t fsr)` | 兼容旧代码的整数 API；`1/3/6/12` 分别映射到手册 `1.5625/3.125/6.25/12.5 Hz` 档位 |
| `startGyro(uint16_t odr, uint16_t fsr)` | 兼容旧代码的整数 API；`15/31/62` 分别映射到手册 `15.625/31.25/62.5 dps` 档位 |
| `startAccel(ICM45686AccelOdr, ICM45686AccelFsr)` | 推荐新代码使用，精确表达手册 ODR/FSR 枚举 |
| `startGyro(ICM45686GyroOdr, ICM45686GyroFsr)` | 推荐新代码使用，精确表达手册 ODR/FSR 枚举 |
| `stopAccel()` | 关闭加速度计 |
| `stopGyro()` | 关闭陀螺仪 |
| `getDataFromRegisters(data)` | 从 UI 寄存器读取原始六轴和温度 |
| `readWhoAmI(who_am_i)` | 读取 `WHO_AM_I`，ICM45686 预期为 `0xE9` |
| `readRegister(reg, value)` / `readRegisters(reg, buffer, len)` | 受控寄存器读取，适合诊断和手册对照 |
| `writeRegister(reg, value)` / `writeRegisterChecked(reg, value, mask)` | 受控寄存器写入；`Checked` 会写后读回校验指定 bit |

支持 ODR：`1, 3, 6, 12, 25, 50, 100, 200, 400, 800, 1600, 3200, 6400` Hz。不支持的值会回退到默认 100 Hz。

支持加速度量程：`2, 4, 8, 16` g，部分芯片支持 `32` g。不支持的值回退到 16 g。

支持陀螺量程：`15, 31, 62, 125, 250, 500, 1000, 2000` dps，部分芯片支持 `4000` dps。不支持的值回退到 2000 dps。

### 7.2 强类型 ODR/FSR 与单位换算

推荐所有新测试和后续飞控代码使用强类型枚举，避免小数档位被旧整数 API 误解：

```cpp
imu.startAccel(ICM45686AccelOdr::Hz1600, ICM45686AccelFsr::G16);
imu.startGyro(ICM45686GyroOdr::Hz1600, ICM45686GyroFsr::Dps2000);
```

常用换算工具：

| API | 说明 |
| --- | --- |
| `accelFsrToG(fsr)` | 返回加速度 FSR 数值，单位 g |
| `gyroFsrToDps(fsr)` | 返回陀螺 FSR 数值，单位 dps |
| `rawRegisterAccelToG(raw, fsr)` | UI 寄存器原始加速度换算为 g |
| `rawRegisterGyroToDps(raw, fsr)` | UI 寄存器原始陀螺换算为 dps |
| `rawRegisterTempToC(raw)` | UI 寄存器温度换算为摄氏度，公式 `25 + raw / 128` |
| `rawFifoTempToC(raw)` | FIFO 温度换算为摄氏度，公式 `25 + raw / 2` |
| `rawFifo20AccelToG(raw)` | 20-bit FIFO 加速度换算，按手册固定 `±32 g` 缩放 |
| `rawFifo20GyroToDps(raw)` | 20-bit FIFO 陀螺换算，按手册固定 `±4000 dps` 缩放 |
| `womThresholdMgToReg(mg)` | WOM 阈值从 mg 换算到 `1g/256` 寄存器 LSB |

### 7.3 FIFO

| API | 功能 |
| --- | --- |
| `configureFifo(config)` | 只配置 FIFO，不绑定 MCU 中断，适合轮询 FIFO frame count |
| `enableFifoInterrupt(intpin, handler, watermark)` | 配置 FIFO 和 INT1 中断 |
| `enableFifoInterrupt(intpin, handler, config)` | 使用 `ICM45686FifoConfig` 配置 FIFO 和 INT1 中断 |
| `getFifoFrameCount(frame_count)` | 读取 FIFO 中已有 frame 数 |
| `getDataFromFifo(data)` | 读取一个 FIFO frame |
| `flushFifo()` | 清空 FIFO |

`ICM45686FifoConfig` 字段说明：

| 字段 | 说明 |
| --- | --- |
| `enable_accel` / `enable_gyro` | 是否写入加速度/陀螺数据 |
| `high_resolution_20bit` | 是否使用 20-bit 高分辨率 FIFO 格式 |
| `timestamp_or_fsync` | 是否把 timestamp/FSYNC 信息写入 FIFO |
| `watermark` | watermark，单位 frame；为 `0` 时返回参数错误 |
| `mode` | `Bypass`、`Stream` 或 `Snapshot` |
| `depth` | `Depth2KB` 或 `Depth8KB`；APEX/eDMP 占用 SRAM 时应优先用 2KB |
| `accel_decimation` / `gyro_decimation` | FIFO ODR 抽取倍率，支持 `1/2/4/.../4096` |

注意事项：

- `handler` 不能为空，否则返回错误。
- 中断回调只做计数或置位，不要在 ISR 内读 SPI/I2C。
- FIFO 会改写中断配置；切换回寄存器轮询前建议重新 `begin()`。
- FIFO 温度换算使用 `25 + temp / 2`，寄存器温度换算使用 `25 + temp / 128`。
- 20-bit FIFO 高分辨率帧不按当前 UI FSR 缩放，而是按手册固定 `±32 g`、`±4000 dps` 缩放。

### 7.4 中断、自检、FSYNC 与 INT2

| API | 功能 |
| --- | --- |
| `configureDataReadyInterrupt(intpin, handler, int_num)` | 配置 UI DRDY 中断，默认 INT1 |
| `runSelfTest(output, accel_enable, gyro_enable)` | 执行芯片自检，自检后需要重新初始化传感器工作模式 |
| `configureFsync(sensor_tag, enable)` | 在底层宏支持时配置 FSYNC 标记 |
| `configureInt2PinUsage(usage)` | 在底层宏支持时配置 INT2 引脚用途 |

注意事项：

- ISR 里只做 `volatile` 标志置位或计数，不要直接读 SPI/I2C。
- `runSelfTest()` 会重配芯片内部状态，完成后应重新 `begin()`、`startAccel()`、`startGyro()`，不要把自检后的状态直接接入飞控主循环。
- INT2 受 AUX1/FSYNC/CLKIN/DRDY 复用限制影响，使用前必须结合硬件原理图和手册确认。
- FSYNC 与 FIFO timestamp 字段共享部分数据路径，诊断时要明确当前输出的是 timestamp 还是 FSYNC 计数。

### 7.5 Wake on Motion

旧接口仍可用：

```cpp
imu.startWakeOnMotion(14, imuIntHandler, 50);
```

推荐新代码使用 `ICM45686WomConfig`：

```cpp
ICM45686WomConfig wom;
wom.intpin = 14;
wom.handler = imuIntHandler;
wom.threshold_x_mg = 52;
wom.threshold_y_mg = 52;
wom.threshold_z_mg = 80;
wom.use_or_mode = true;
wom.duration_samples = 2;
wom.accel_odr = ICM45686AccelOdr::Hz50;
imu.startWakeOnMotion(wom);
```

WOM 阈值按 `mg` 填写，库内使用 `womThresholdMgToReg()` 换算为手册定义的 `1g/256` 寄存器 LSB。例如 `52 mg` 会换算到约 `13 LSB`。

### 7.6 APEX 事件

ICM45686(A1) 可用的 APEX 事件包括：

- `startTiltDetection()` / `getTilt()`
- `startPedometer()` / `getPedometer()`
- `startRaiseToWake()` / `getRaiseToWake()`
- `startTap()` / `getTap()`
- `startFreeFall()` / `getFreefall()`
- `startHighG()` / `getHighG()`
- `startLowG()` / `getLowG()`
- `startWakeOnMotion()`

APEX 使用建议：

- 多个 APEX 事件可以先调用各 `startXxx()` 无 handler 版本置位，再统一调用 `setApexInterrupt()` 和 `startAPEX()`。
- APEX 会重配 eDMP、加速度 ODR、带宽、中断源和低功耗相关寄存器。
- 不要在飞控主控制环里动态频繁切换 APEX 模式。
- APEX 更适合地面诊断、低功耗唤醒、事件检测，不建议作为姿态或导航主数据源。

### 7.7 AUX I2CM 与外部磁力计

| API | 功能 |
| --- | --- |
| `setI2CM()` | 初始化 ICM45686 内部 I2C master |
| `getDataFromI2CM(reg, data)` | 从外部 ICT1531 地址 `0x1E` 读 1 字节 |
| `setI2CMPassThrough()` | AUX pass-through |
| `getDataFromPassThrough(reg, data)` | pass-through 读取 |
| `setI2CM_FIFO(intpin, handler)` | 配置外部数据进入 FIFO ES0 |
| `getAdvDataFromFifo(accel, gyro, external)` | 读取高级 FIFO 解析输出 |
| `getExternalMagData(mag)` | 读取外部磁力计并换算 uT |

注意事项：

- 当前外部磁力计路径是为 ICT1531/兼容硬件写的，不是通用 I2C 扫描器。
- 未接 AUX 硬件时会返回超时或传输错误。
- 磁力计数据进入导航前必须做硬铁、软铁、安装方向和干扰验证。

## 8. ICM45686(A1) 不支持或不建议依赖的能力

- GAF/四元数融合接口只在 A2/B1/C1 条件编译下可用，当前 ICM45686(A1) 不编译这些函数。
- VocalVibDet 是 B1-only，ICM45686(A1) 不可用。
- 官方风格底层仍有部分静态回调状态，不适合多个 IMU 实例并发。
- 库内 APEX 事件返回值不是所有函数都严格区分“无事件”和“底层读取失败”，飞控安全逻辑不要直接依赖事件函数作为唯一故障判断。

## 9. 飞控接入建议

接入飞控前按以下顺序验证：

1. `WHO_AM_I == 0xE9`。
2. `begin() == 0`。
3. `startAccel()`、`startGyro()` 返回 0。
4. 静止时加速度模长接近 1 g，陀螺均值接近 0 dps。
5. 手动旋转模块，确认轴向、符号、FRD/NED 坐标约定与飞控一致。
6. 用固定周期读取，确认 `dt_min_us`、`dt_max_us`、错误计数和饱和计数可接受。
7. 再考虑 FIFO 或中断读取路径。
8. 最后才接入姿态解算、导航和控制闭环。

飞控中建议保留以下保护：

- 连续读数失败计数。
- 数据陈旧检测。
- 原始值饱和检测。
- 加速度模长异常检测。
- 陀螺静止零偏估计和温漂观察。
- 启动阶段传感器健康检查失败时禁止解锁。

## 10. 常见故障排查

### 10.1 `WHO_AM_I` 不等于 `0xE9`

优先检查：

- 供电是否为模块允许电压。
- GND 是否共地。
- SPI mode 是否为 mode 3。
- CS 是否接对且空闲为高。
- MISO/MOSI 是否反接。
- 是否误用了 I2C 连接运行 SPI 程序。
- 模块是否并非 ICM45686。

### 10.2 `begin()` 返回负数

可能原因：

- 主通信失败。
- 芯片复位后等待时间不足。
- SPI/I2C 引脚或频率不匹配。
- 多设备共用总线时被其他设备拉低或占用。

处理建议：

- 降低 SPI/I2C 频率。
- 单独连接 IMU 做最小测试。
- 用逻辑分析仪确认 CS、SCK、MOSI、MISO。

### 10.3 FIFO 没有中断

检查：

- INT1 是否接到配置的 MCU GPIO。
- GPIO 是否支持外部中断。
- 中断极性是否被外部电路反相。
- ISR 是否只做简单置位。
- `enableFifoInterrupt()` 返回值是否为 0。

### 10.4 APEX 没有事件

检查：

- 是否调用了 `setApexInterrupt()` 和 `startAPEX()`。
- INT1 是否连接。
- 事件动作是否满足算法触发条件。
- 是否刚从其他模式切换，建议重新 `begin()` 后再配置 APEX。

### 10.5 外部磁力计读不到

检查：

- AUX 口是否连接 ICT1531/兼容磁力计。
- 地址是否为 `0x1E`。
- 外部磁力计供电和上拉是否正确。
- 当前程序是否只是想验证 IMU 主 SPI/I2C，如果是，可忽略 AUX 错误。

## 11. 本仓库测试命令

```powershell
pio run -e icm45686_api_compile_test
pio run -e icm45686_spi_test
pio run -e icm45686_spi_diagnostic
pio run -e icm45686_vofa_highrate
```

这四个命令只证明代码能在对应 PlatformIO 环境下编译通过；其中 `icm45686_api_compile_test` 明确不访问硬件。真实传感器通信、FIFO 中断、WOM、自检、VOFA+ 高频输出仍需要上板验证。

串口诊断台使用：

```powershell
pio device monitor -e icm45686_spi_diagnostic
```

上电后先输入：

```text
i
r
s
bias
```

如果 INT1 已接好，再测试：

```text
f
a
wom
selftest
```

## 12. 实机安全

IMU 测试本身不驱动执行机构，但后续飞控联调时必须保证推进系统安全：

- 传感器 bring-up 阶段不要接入电机、舵机或推进动力。
- 如果测试程序同时包含 PWM、FOC、电机或舵机输出，先断开动力电源。
- 控制闭环前必须确认 IMU 轴向和符号，否则姿态控制会反向。
- 自由飞行前必须经过编译检查、静态传感器检查、执行机构方向检查、约束条件低功率测试。

## 13. 文件维护约定

- `ICM45686.h/cpp`：底层 Arduino driver 封装，保留寄存器、FIFO、APEX、I2CM 能力。
- `ICM45686_FlightAdapter.h/cpp`：飞控主循环建议入口，保持轻量、确定性、无动态内存。
- `imu/`、`Ict1531x/`、`Invn/`：底层 C driver 和寄存器定义，可继续重构，但每次应小步提交并跑两个测试环境。
- `examples/`：Arduino 示例，可逐步中文化，但不要把示例逻辑混入核心库。

如果后续继续深度重构底层 C 文件，优先顺序建议为：

1. 通信传输层。
2. FIFO 配置和解析。
3. APEX/eDMP 配置。
4. I2CM/外部磁力计。
5. 寄存器表和纯定义文件。
