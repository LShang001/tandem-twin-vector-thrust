#include <Arduino.h>
#include <SPI.h>
#include <math.h>

#include "ICM45686.h"
#include "ICM45686_FlightAdapter.h"

/*
 * ICM45686 综合功能 SPI 示例
 * ==========================
 *
 * 这个示例是本库的“全功能入口示例”，目标不是最短代码，而是给后续项目提供一份可读、可裁剪、
 * 可直接验证的参考模板。它覆盖：
 *
 * 1. SPI 四线通信初始化和 WHO_AM_I 直接寄存器验证。
 * 2. 底层 ICM456xx API 的 begin/startAccel/startGyro/getDataFromRegisters。
 * 3. 飞控推荐入口 ICM45686FlightAdapter，包括单位换算、数据新鲜度、错误统计和饱和检测。
 * 4. 原始寄存器轮询流，便于对照 FlightAdapter 的输出。
 * 5. FIFO + INT1 中断读取，便于验证中断线、FIFO watermark 和 FIFO frame 格式。
 * 6. APEX 综合事件：Tilt、Pedometer、RaiseToWake、Tap、FreeFall、HighG、LowG。
 * 7. Wake on Motion 低功耗唤醒路径。
 * 8. AUX I2CM/外部磁力计预留测试，用于连接 ICT1531/兼容磁力计后验证。
 * 9. 静止偏置估计，用于飞控接入前观察陀螺零偏和 1 g 模长。
 *
 * 重要设计原则：
 *
 * - 复杂功能互相会改写 eDMP、中断、FIFO 和加速度计模式，所以本示例采用“串口命令切换模式”，
 *   不在 setup() 中一次性启用全部功能。
 * - ISR 里只做计数，不读 SPI/I2C。总线访问都放在 loop() 普通上下文里。
 * - 切换复杂模式时重新 begin()，尽量避免上一模式留下的 eDMP/FIFO/中断配置影响下一模式。
 * - 飞控主链路建议优先使用 ICM45686FlightAdapter；APEX/FIFO/WOM 更适合诊断或特定低功耗场景。
 *
 * 默认接线面向 ESP32-S3，可通过编译宏覆盖：
 *
 * - ICM45686_EXAMPLE_SPI_SCK   默认 12
 * - ICM45686_EXAMPLE_SPI_MISO  默认 13
 * - ICM45686_EXAMPLE_SPI_MOSI  默认 11
 * - ICM45686_EXAMPLE_SPI_CS    默认 10
 * - ICM45686_EXAMPLE_SPI_HZ    默认 6000000
 * - ICM45686_EXAMPLE_INT_PIN   默认 14，若未连接 INT1，可设为 -1
 *
 * 串口波特率：115200。
 */

#ifndef ICM45686_EXAMPLE_SPI_SCK
#define ICM45686_EXAMPLE_SPI_SCK 12
#endif

#ifndef ICM45686_EXAMPLE_SPI_MISO
#define ICM45686_EXAMPLE_SPI_MISO 13
#endif

#ifndef ICM45686_EXAMPLE_SPI_MOSI
#define ICM45686_EXAMPLE_SPI_MOSI 11
#endif

#ifndef ICM45686_EXAMPLE_SPI_CS
#define ICM45686_EXAMPLE_SPI_CS 10
#endif

#ifndef ICM45686_EXAMPLE_SPI_HZ
#define ICM45686_EXAMPLE_SPI_HZ 6000000
#endif

#ifndef ICM45686_EXAMPLE_INT_PIN
#define ICM45686_EXAMPLE_INT_PIN 14
#endif

// ICM45686 的 WHO_AM_I 寄存器地址和期望值。直接读寄存器可排除库初始化流程的干扰。
static constexpr uint8_t kWhoAmIRegister = 0x72;
static constexpr uint8_t kExpectedWhoAmI = 0xE9;
static constexpr uint8_t kSpiReadBit = 0x80;

// 本示例统一使用 16 g / 2000 dps，适合飞控 bring-up 早期避免轻易饱和。
static constexpr uint16_t kAccelFsrG = 16;
static constexpr uint16_t kGyroFsrDps = 2000;
static constexpr uint16_t kDefaultOdrHz = 200;
static constexpr uint8_t kFifoWatermark = 8;
static constexpr float kRawFullScale = 32768.0f;

// 串口输出频率。输出太快会影响 loop 周期，也会让串口监视器难以阅读。
static constexpr uint32_t kFlightPrintPeriodMs = 20;
static constexpr uint32_t kRegisterPrintPeriodMs = 20;
static constexpr uint32_t kStatsPrintPeriodMs = 3000;

enum class DemoMode : uint8_t {
  Idle,                 // 空闲状态：loop() 只处理串口命令，不主动读取 IMU。
  FlightAdapterStream,  // 飞控适配层连续输出：推荐后续飞控主循环优先参考。
  RegisterStream,       // 底层寄存器轮询输出：用于对照原始值、验证单位换算。
  FifoInterrupt,        // FIFO + INT1 中断输出：用于验证 FIFO 水位和中断线。
  ApexEvents,           // APEX 事件输出：用于验证 eDMP 内部事件检测。
  WakeOnMotion,         // Wake on Motion 输出：用于低功耗运动唤醒 bring-up。
  ExternalI2cm          // AUX/I2CM 外部传感器探测：当前只做一次性读数。
};

struct DemoStats {
  uint32_t register_samples = 0;    // 寄存器轮询模式成功读取的样本数。
  uint32_t flight_samples = 0;      // FlightAdapter 模式成功读取的样本数。
  uint32_t fifo_frames = 0;         // FIFO 模式成功解析的 frame 数，不是中断次数。
  uint32_t read_errors = 0;         // 任何读取函数返回非 0 时累加，用于判断总线或模式配置问题。
  uint32_t saturated_samples = 0;   // 原始 accel/gyro 接近 int16 满量程的样本数。
  uint32_t irq_snapshot = 0;        // 上一次 loop() 已处理到的中断计数快照。
  uint32_t last_sample_us = 0;      // 上一个有效样本的 micros()，用于估计采样间隔。
  uint32_t dt_min_us = UINT32_MAX;  // 有效样本间隔最小值，初始设为最大值便于第一次比较。
  uint32_t dt_max_us = 0;           // 有效样本间隔最大值。
  uint64_t dt_sum_us = 0;           // 有效样本间隔总和，用 64 位避免长时间运行溢出。
  float accel_norm_min_g = 1000.0f; // 加速度模长最小值，静止时应接近 1 g。
  float accel_norm_max_g = 0.0f;    // 加速度模长最大值，用于观察冲击、振动或轴向异常。
  float accel_norm_sum_g = 0.0f;    // 加速度模长累加值，用于输出平均模长。
  float gyro_abs_max_dps = 0.0f;    // 三轴陀螺绝对值最大值，用于快速观察运动强度。
};

// 底层 Arduino 封装对象：负责 SPI 传输、寄存器配置、FIFO/APEX/WOM 等完整驱动能力。
static ICM456xx imu(SPI, ICM45686_EXAMPLE_SPI_CS, ICM45686_EXAMPLE_SPI_HZ);
// 飞控适配层对象：只包装飞控主循环常用的六轴轮询、单位换算、错误统计和 fresh 判断。
static ICM45686FlightAdapter flight_imu(imu);

// Arduino 会为 .ino 生成部分函数原型，但复杂示例最好显式声明，避免不同构建器行为不一致。
static uint32_t takeIrqCount();
static void resetIrqCount();
static bool intPinEnabled();
static void resetStats();
static float rawAccelToG(int32_t raw);
static float rawGyroToDps(int32_t raw);
static float registerTempToC(int16_t raw);
static float fifoTempToC(int8_t raw);
static bool nearRawLimit(int32_t raw);
static void updateCommonStats(const float accel_g[3], const float gyro_dps[3], bool saturated);
static uint8_t readRegisterBySpi(uint8_t reg);
static void printReturnCode(const __FlashStringHelper *api_name, int status);
static void printWhoAmI();
static bool beginRawImu();
static bool startRawAccelGyro(uint16_t odr_hz);
static bool startFlightAdapter();
static void stopCurrentMode();
static bool enterRegisterStream();
static bool enterFifoInterruptMode();
static bool enterApexEventsMode();
static bool enterWakeOnMotionMode();
static bool runExternalI2cmProbe();
static void runStationaryBiasEstimate(uint16_t sample_target);
static void printFlightSample();
static void printRegisterSample();
static void printFifoFramesForOneIrq();
static void printApexEvents();
static void printStats();
static void printCapabilityNotes();
static void printHelp();
static void handleCommand(String command);
static void processSerial();
static void runActiveMode();

static volatile uint32_t irq_count = 0;              // ISR 和 loop() 共享，必须使用 volatile。
static DemoMode current_mode = DemoMode::Idle;       // 当前串口命令选择的运行模式。
static DemoStats stats;                              // 当前模式的样本统计，切换模式时清零。
static bool imu_ready = false;                       // true 表示底层 imu.begin() 已成功。
static bool flight_started = false;                  // true 表示 FlightAdapter 已启动，需要用 flight_imu.stop() 停止。
static uint32_t last_flight_print_ms = 0;            // FlightAdapter 输出节流时间戳。
static uint32_t last_register_print_ms = 0;          // 寄存器轮询输出节流时间戳。
static uint32_t last_stats_print_ms = 0;             // 统计信息输出节流时间戳。

/*
 * 中断回调函数。
 *
 * 调用来源：Arduino attachInterrupt()，由 enableFifoInterrupt()、setApexInterrupt()
 * 或 startWakeOnMotion() 间接注册。
 *
 * 设计约束：
 * - ISR 中绝对不要 Serial.print()，否则可能阻塞或破坏时序。
 * - ISR 中不要读 SPI/I2C，因为总线驱动通常不是中断安全的。
 * - 这里只做最小计数，loop() 再根据计数读取 FIFO/APEX/WOM 状态。
 */
void imuInterruptHandler()
{
  // 中断上下文内不要做 Serial.print()，也不要读 SPI/I2C。这里只记录事件次数。
  irq_count = irq_count + 1;
}

/*
 * 安全读取中断计数快照。
 *
 * 返回值：当前 irq_count 的一致性快照。
 *
 * noInterrupts()/interrupts() 的原因：
 * - irq_count 是 32 位变量，在部分 MCU 上一次读取不一定天然原子。
 * - 关中断时间只覆盖一次读变量，耗时极短，不会明显影响实时性。
 */
static uint32_t takeIrqCount()
{
  noInterrupts();
  const uint32_t value = irq_count;
  interrupts();
  return value;
}

/*
 * 清零中断计数。
 *
 * 典型调用时机：
 * - resetStats()。
 * - 模式切换后开始新的统计周期。
 */
static void resetIrqCount()
{
  noInterrupts();
  irq_count = 0;
  interrupts();
}

/*
 * 判断示例是否启用了 INT 引脚。
 *
 * 返回值：
 * - true：ICM45686_EXAMPLE_INT_PIN 为有效 GPIO，可运行 FIFO/APEX/WOM。
 * - false：宏被设置为 -1，跳过所有需要中断线的功能。
 */
static bool intPinEnabled()
{
  return ICM45686_EXAMPLE_INT_PIN >= 0;
}

/*
 * 重置当前模式统计。
 *
 * 注意：这里只清理示例侧统计变量，不会清理芯片内部 FIFO/eDMP 状态。
 * 芯片状态由 stopCurrentMode() 和后续 beginRawImu()/flight_imu.begin() 处理。
 */
static void resetStats()
{
  stats = DemoStats {};
  resetIrqCount();
}

/*
 * 把加速度原始值换算为 g。
 *
 * ICM45686 在 UI/FIFO 输出中使用有符号 16 位满量程表示：
 * - raw = 32768 约等于 +FSR。
 * - 当前示例 FSR 固定为 kAccelFsrG，即 16 g。
 */
static float rawAccelToG(int32_t raw)
{
  return static_cast<float>(raw) * static_cast<float>(kAccelFsrG) / kRawFullScale;
}

/*
 * 把陀螺原始值换算为 dps。
 *
 * 当前示例 FSR 固定为 kGyroFsrDps，即 2000 dps。若后续修改量程，
 * 必须同步修改 kGyroFsrDps，否则串口输出单位会错误。
 */
static float rawGyroToDps(int32_t raw)
{
  return static_cast<float>(raw) * static_cast<float>(kGyroFsrDps) / kRawFullScale;
}

/*
 * 把普通寄存器路径的温度原始值换算为摄氏度。
 *
 * 公式来自库中 FlightAdapter 的 rawTempToC()，这里重复写出是为了让
 * 底层寄存器示例不依赖 FlightAdapter 内部私有函数。
 */
static float registerTempToC(int16_t raw)
{
  return 25.0f + static_cast<float>(raw) / 128.0f;
}

/*
 * 把 FIFO frame 中的 int8 温度字段换算为摄氏度。
 *
 * FIFO 的温度字段宽度和普通寄存器路径不同，所以换算比例也不同。
 */
static float fifoTempToC(int8_t raw)
{
  return 25.0f + static_cast<float>(raw) / 2.0f;
}

/*
 * 判断原始值是否接近 int16 满量程。
 *
 * 这里用 32000 而不是 32767，是为了提前提示“快饱和”，给飞控量程选择留余量。
 */
static bool nearRawLimit(int32_t raw)
{
  return raw >= 32000 || raw <= -32000;
}

/*
 * 更新所有模式共用的统计量。
 *
 * 参数：
 * - accel_g：已换算为 g 的三轴加速度。
 * - gyro_dps：已换算为 dps 的三轴角速度。
 * - saturated：调用者已经判断出的满量程风险。
 *
 * 这些统计量用于上板 bring-up：
 * - dt_us 可观察 loop 输出节流和实际读取间隔。
 * - accel_norm_g 静止时应接近 1 g，可快速发现轴向、单位或量程配置错误。
 * - gyro_abs_max_dps 可快速发现强振动、误触碰或陀螺异常。
 */
static void updateCommonStats(const float accel_g[3], const float gyro_dps[3], bool saturated)
{
  // 使用 micros() 统计“成功样本之间”的时间差，不统计读取失败或串口阻塞时间。
  const uint32_t now_us = micros();
  if (stats.last_sample_us != 0) {
    const uint32_t dt_us = now_us - stats.last_sample_us;
    stats.dt_min_us = min(stats.dt_min_us, dt_us);
    stats.dt_max_us = max(stats.dt_max_us, dt_us);
    stats.dt_sum_us += dt_us;
  }
  stats.last_sample_us = now_us;

  // 加速度模长是最直观的 bring-up 指标：静止桌面上一般应接近 1 g。
  const float accel_norm = sqrtf(accel_g[0] * accel_g[0] +
                                 accel_g[1] * accel_g[1] +
                                 accel_g[2] * accel_g[2]);
  stats.accel_norm_min_g = min(stats.accel_norm_min_g, accel_norm);
  stats.accel_norm_max_g = max(stats.accel_norm_max_g, accel_norm);
  stats.accel_norm_sum_g += accel_norm;

  // 三轴陀螺取绝对值最大项，便于快速发现强振动、误触碰或机械安装问题。
  const float gyro_abs = max(max(fabsf(gyro_dps[0]), fabsf(gyro_dps[1])), fabsf(gyro_dps[2]));
  stats.gyro_abs_max_dps = max(stats.gyro_abs_max_dps, gyro_abs);
  if (saturated) {
    stats.saturated_samples++;
  }
}

/*
 * 使用 SPI 直接读取单个寄存器。
 *
 * 这个函数绕过 ICM456xx::begin() 和底层 driver，只做最小 SPI transaction，
 * 适合在 bring-up 最早期验证：
 * - SPI 引脚是否正确。
 * - CS 是否能拉低选中芯片。
 * - SPI mode 3 是否匹配芯片要求。
 * - MISO 是否能读回 WHO_AM_I。
 *
 * 参数：
 * - reg：7 位寄存器地址，不需要手动 OR 读标志。
 *
 * 返回值：寄存器原始 8 位值。
 */
static uint8_t readRegisterBySpi(uint8_t reg)
{
  // ICM45686 SPI UI 接口使用 mode 3；如果改成 mode 0，WHO_AM_I 常见表现是 0x00/0xFF/随机值。
  SPI.beginTransaction(SPISettings(ICM45686_EXAMPLE_SPI_HZ, MSBFIRST, SPI_MODE3));
  // CS 低电平开始一次 SPI 访问。
  digitalWrite(ICM45686_EXAMPLE_SPI_CS, LOW);
  // 地址最高位置 1 表示读；写寄存器时最高位应为 0，本示例只做读。
  SPI.transfer(reg | kSpiReadBit);
  // 第二个字节发送 dummy，同时从 MISO 收到寄存器值。
  const uint8_t value = SPI.transfer(0x00);
  // CS 拉高结束本次访问，随后释放 SPI 总线。
  digitalWrite(ICM45686_EXAMPLE_SPI_CS, HIGH);
  SPI.endTransaction();
  return value;
}

/*
 * 统一打印库 API 返回值。
 *
 * 本库沿用 InvenSense driver 风格：0 表示成功，负数通常表示参数、传输、
 * 配置或状态错误。示例所有关键 API 都打印返回值，避免“初始化失败但继续读数”。
 */
static void printReturnCode(const __FlashStringHelper *api_name, int status)
{
  Serial.print(api_name);
  Serial.print(F(" 返回值="));
  Serial.println(status);
}

/*
 * 读取并打印 WHO_AM_I。
 *
 * 这是 IMU bring-up 的第一步。若这里不等于 0xE9，后面的 begin()/FIFO/APEX
 * 都没有继续排查意义，应优先检查供电、GND、SPI 引脚、CS 和芯片型号。
 */
static void printWhoAmI()
{
  const uint8_t who_am_i = readRegisterBySpi(kWhoAmIRegister);
  Serial.print(F("WHO_AM_I 寄存器 0x72 = 0x"));
  if (who_am_i < 16) {
    Serial.print('0');
  }
  Serial.print(who_am_i, HEX);
  Serial.println(who_am_i == kExpectedWhoAmI ? F("，匹配 ICM45686") : F("，不匹配 0xE9"));
}

/*
 * 初始化底层 ICM456xx 驱动。
 *
 * 返回值：
 * - true：imu.begin() 返回 0，后续可以配置 accel/gyro/FIFO/APEX。
 * - false：底层初始化失败，当前模式入口应立即退出。
 *
 * 注意：本函数不会启动传感器输出，只做通信接口和底层 driver 初始化。
 */
static bool beginRawImu()
{
  printWhoAmI();
  const int status = imu.begin();
  printReturnCode(F("imu.begin()"), status);
  imu_ready = status == 0;
  if (!imu_ready) {
    Serial.println(F("IMU 初始化失败。先检查供电、GND、SPI 接线、CS、SPI mode 3 和芯片型号。"));
  }
  return imu_ready;
}

/*
 * 启动普通寄存器轮询所需的 accel 和 gyro。
 *
 * 参数：
 * - odr_hz：目标输出数据率，单位 Hz。底层会把常用 Hz 映射为芯片枚举。
 *
 * 返回值：
 * - true：加速度计和陀螺仪都启动成功。
 * - false：任意一路启动失败。
 */
static bool startRawAccelGyro(uint16_t odr_hz)
{
  // 如果当前还没有 begin() 成功，就先补做一次底层初始化。
  if (!imu_ready && !beginRawImu()) {
    return false;
  }

  // 飞控 bring-up 初期使用较大量程，避免轻微碰撞或振动就饱和。
  const int accel_status = imu.startAccel(odr_hz, kAccelFsrG);
  const int gyro_status = imu.startGyro(odr_hz, kGyroFsrDps);
  printReturnCode(F("imu.startAccel()"), accel_status);
  printReturnCode(F("imu.startGyro()"), gyro_status);
  // 给传感器模式切换留一点稳定时间，避免刚启动时读到无效或旧数据。
  delay(100);
  return accel_status == 0 && gyro_status == 0;
}

/*
 * 进入飞控适配层连续输出模式。
 *
 * 这是后续飞控主链路最推荐参考的路径：
 * - begin() 内部启动 accel/gyro。
 * - readSample() 输出 g、dps、摄氏度和饱和标志。
 * - stats()/hasFreshSample() 可用于上层故障诊断。
 *
 * 返回值：true 表示进入模式成功。
 */
static bool startFlightAdapter()
{
  // 切换模式前统一停止上一模式，避免 FIFO/APEX/WOM 残留配置影响普通轮询。
  stopCurrentMode();

  ICM45686FlightConfig cfg;
  cfg.accel_odr_hz = kDefaultOdrHz;
  cfg.gyro_odr_hz = kDefaultOdrHz;
  cfg.accel_fsr_g = kAccelFsrG;
  cfg.gyro_fsr_dps = kGyroFsrDps;
  cfg.stale_after_us = 10000;

  // 先直接读 WHO_AM_I，再走高级封装 begin()，便于区分“硬件通信失败”和“库初始化失败”。
  printWhoAmI();
  const int status = flight_imu.begin(cfg);
  printReturnCode(F("flight_imu.begin()"), status);
  if (status != 0) {
    flight_started = false;
    imu_ready = false;
    return false;
  }

  imu_ready = true;
  flight_started = true;
  resetStats();
  current_mode = DemoMode::FlightAdapterStream;
  Serial.println(F("进入飞控适配层流模式：输出已换算 g/dps/温度/新鲜度/饱和状态。"));
  return true;
}

/*
 * 停止当前模式。
 *
 * 设计目的：
 * - FlightAdapter 模式必须调用 flight_imu.stop()，因为其内部维护 started 状态。
 * - 原始寄存器模式只需要关闭 accel/gyro。
 * - FIFO/APEX/WOM 这类复杂模式在下一次 begin() 时会重新初始化底层状态。
 *
 * 注意：该函数不 detachInterrupt()，因为库内部统一管理中断绑定；本示例通过重置
 * irq_count 和 current_mode 避免旧中断被继续处理。
 */
static void stopCurrentMode()
{
  if (flight_started) {
    printReturnCode(F("flight_imu.stop()"), flight_imu.stop());
    flight_started = false;
  } else if (imu_ready) {
    printReturnCode(F("imu.stopAccel()"), imu.stopAccel());
    printReturnCode(F("imu.stopGyro()"), imu.stopGyro());
  }

  current_mode = DemoMode::Idle;
  resetStats();
  Serial.println(F("已停止当前模式。"));
}

/*
 * 进入底层寄存器轮询模式。
 *
 * 用途：
 * - 对照 FlightAdapter 的换算结果。
 * - 观察原始 int16 数据是否随姿态变化。
 * - 在不启用 FIFO/eDMP 的情况下验证最基础六轴数据路径。
 */
static bool enterRegisterStream()
{
  stopCurrentMode();
  if (!beginRawImu()) {
    return false;
  }
  if (!startRawAccelGyro(kDefaultOdrHz)) {
    return false;
  }
  resetStats();
  current_mode = DemoMode::RegisterStream;
  Serial.println(F("进入底层寄存器轮询模式：用于对照 FlightAdapter 输出和验证原始数据。"));
  return true;
}

/*
 * 进入 FIFO + INT1 中断模式。
 *
 * 调用前提：
 * - INT1 已连接到 ICM45686_EXAMPLE_INT_PIN。
 * - GPIO 必须支持 attachInterrupt()。
 *
 * 行为：
 * - 配置 FIFO watermark。
 * - 注册中断回调。
 * - 启动 accel/gyro，把样本推入 FIFO。
 */
static bool enterFifoInterruptMode()
{
  stopCurrentMode();
  if (!intPinEnabled()) {
    Serial.println(F("FIFO 模式需要连接 INT1。若未接中断线，请把 ICM45686_EXAMPLE_INT_PIN 设为实际 GPIO。"));
    return false;
  }
  if (!beginRawImu()) {
    return false;
  }

  // enableFifoInterrupt() 会配置 FIFO 和中断输出，watermark 表示 FIFO 累积多少帧后触发中断。
  const int fifo_status = imu.enableFifoInterrupt(ICM45686_EXAMPLE_INT_PIN, imuInterruptHandler, kFifoWatermark);
  printReturnCode(F("imu.enableFifoInterrupt()"), fifo_status);
  // FIFO 只是缓存通道，仍需要启动 accel/gyro，芯片才会持续产生 frame。
  const int accel_status = imu.startAccel(kDefaultOdrHz, kAccelFsrG);
  const int gyro_status = imu.startGyro(kDefaultOdrHz, kGyroFsrDps);
  printReturnCode(F("imu.startAccel()"), accel_status);
  printReturnCode(F("imu.startGyro()"), gyro_status);
  if (fifo_status != 0 || accel_status != 0 || gyro_status != 0) {
    return false;
  }

  resetStats();
  current_mode = DemoMode::FifoInterrupt;
  Serial.print(F("进入 FIFO 中断模式，INT GPIO="));
  Serial.print(ICM45686_EXAMPLE_INT_PIN);
  Serial.print(F("，watermark="));
  Serial.println(kFifoWatermark);
  return true;
}

/*
 * 进入 APEX 综合事件模式。
 *
 * APEX 是芯片内部 eDMP 的事件检测功能。本示例一次性启用多个 APEX A 类事件，
 * 目的是展示库能力和串口观察方法，而不是建议飞控主循环这样同时使用。
 *
 * 注意：
 * - APEX 会重配 eDMP、加速度计 ODR 和中断状态。
 * - 不能和普通六轴飞控轮询路径混在同一个实时闭环里使用。
 */
static bool enterApexEventsMode()
{
  stopCurrentMode();
  if (!intPinEnabled()) {
    Serial.println(F("APEX 事件模式需要连接 INT1。"));
    return false;
  }
  if (!beginRawImu()) {
    return false;
  }

  int status = 0;
  // 这些 startXxx() 在 handler 默认为空时主要是置位“要启用的 APEX 功能”。
  status |= imu.startTiltDetection();
  status |= imu.startPedometer();
  status |= imu.startRaiseToWake();
  status |= imu.startTap();
  status |= imu.startFreeFall();
  status |= imu.startHighG();
  status |= imu.startLowG();
  printReturnCode(F("APEX 功能置位"), status);

  // 统一把 APEX 事件路由到 INT1，然后 startAPEX() 真正启动 eDMP。
  const int irq_status = imu.setApexInterrupt(ICM45686_EXAMPLE_INT_PIN, imuInterruptHandler);
  const int apex_status = imu.startAPEX();
  printReturnCode(F("imu.setApexInterrupt()"), irq_status);
  printReturnCode(F("imu.startAPEX()"), apex_status);
  if (status != 0 || irq_status != 0 || apex_status != 0) {
    return false;
  }

  resetStats();
  current_mode = DemoMode::ApexEvents;
  Serial.println(F("进入 APEX 综合事件模式：移动、敲击或改变姿态后观察事件输出。"));
  Serial.println(F("注意：APEX 会重配 eDMP/中断/加速度 ODR，不要与飞控主轮询模式同时使用。"));
  return true;
}

/*
 * 进入 Wake on Motion 模式。
 *
 * 用途：
 * - 验证低功耗运动唤醒路径。
 * - 观察模块静止和移动时 INT1 中断计数变化。
 *
 * 注意：WOM 会改写加速度计低功耗相关配置，测试后切回普通模式时必须重新 begin()。
 */
static bool enterWakeOnMotionMode()
{
  stopCurrentMode();
  if (!intPinEnabled()) {
    Serial.println(F("Wake on Motion 需要连接 INT1。"));
    return false;
  }
  if (!beginRawImu()) {
    return false;
  }

  // 默认 ODR 由库函数默认参数决定；本示例重点验证中断唤醒链路。
  const int status = imu.startWakeOnMotion(ICM45686_EXAMPLE_INT_PIN, imuInterruptHandler);
  printReturnCode(F("imu.startWakeOnMotion()"), status);
  if (status != 0) {
    return false;
  }

  resetStats();
  current_mode = DemoMode::WakeOnMotion;
  Serial.println(F("进入 Wake on Motion 模式：移动模块时应产生中断计数。"));
  return true;
}

/*
 * 执行一次 AUX/I2CM 外部磁力计探测。
 *
 * 当前库默认外部目标是 ICT1531/兼容磁力计地址 0x1E。本函数不是连续模式，
 * 只做一次 setI2CM()、一次寄存器读取和一次 getExternalMagData()。
 *
 * 若 AUX 口没有接外部磁力计，返回错误是正常现象，不代表 ICM45686 主六轴不可用。
 */
static bool runExternalI2cmProbe()
{
  stopCurrentMode();
  if (!beginRawImu()) {
    return false;
  }

  current_mode = DemoMode::ExternalI2cm;
  Serial.println(F("外部 I2CM/AUX 测试：默认目标是 ICT1531/兼容磁力计地址 0x1E。"));
  Serial.println(F("如果 AUX 口没有接外部磁力计，下面返回错误或超时是正常现象。"));

  // setI2CM() 打开 ICM45686 内部 I2C master，而不是主控 ESP32 的 Wire 总线。
  printReturnCode(F("imu.setI2CM()"), imu.setI2CM());

  uint8_t data = 0;
  // 读取外部传感器 0x00 寄存器，用于快速判断 AUX 链路是否有响应。
  const int i2cm_status = imu.getDataFromI2CM(0x00, data);
  Serial.print(F("imu.getDataFromI2CM(0x00) 返回="));
  Serial.print(i2cm_status);
  Serial.print(F(" data=0x"));
  if (data < 16) {
    Serial.print('0');
  }
  Serial.println(data, HEX);

  float mag_uT[3] = {0.0f, 0.0f, 0.0f};
  // 如果外部磁力计初始化和数据路径正常，这里会输出约 uT 单位的三轴磁场。
  const int mag_status = imu.getExternalMagData(mag_uT);
  Serial.print(F("imu.getExternalMagData() 返回="));
  Serial.print(mag_status);
  Serial.print(F(" mag_uT="));
  Serial.print(mag_uT[0], 3);
  Serial.print(',');
  Serial.print(mag_uT[1], 3);
  Serial.print(',');
  Serial.println(mag_uT[2], 3);
  return true;
}

/*
 * 执行静止零偏估计。
 *
 * 参数：
 * - sample_target：希望累计的有效样本数。样本越多，平均值越稳定，但耗时越长。
 *
 * 输出含义：
 * - accel_avg_g：三轴加速度均值。模块静止时，某一个轴通常接近 +/-1 g，其余轴接近 0。
 * - accel_norm_avg_g：加速度模长均值。静止时应接近 1 g。
 * - gyro_bias_dps：三轴陀螺零偏均值。飞控接入前可据此判断零偏量级是否合理。
 *
 * 注意：
 * - 这里不把偏置写入芯片或库内部，只打印结果，避免示例程序隐式改变标定状态。
 * - 真正飞控项目应在自己的标定层保存和应用零偏。
 */
static void runStationaryBiasEstimate(uint16_t sample_target)
{
  // 偏置估计必须从普通六轴轮询路径开始，避免 FIFO/APEX/WOM 模式残留。
  stopCurrentMode();
  if (!beginRawImu()) {
    return;
  }
  if (!startRawAccelGyro(kDefaultOdrHz)) {
    return;
  }

  Serial.print(F("开始静止偏置估计，目标样本数="));
  Serial.print(sample_target);
  Serial.println(F("。测试期间请保持模块静止。"));

  // 使用 double 做累加，避免几百个样本相加时 float 精度损失影响平均值。
  double accel_sum[3] = {0.0, 0.0, 0.0};
  double gyro_sum[3] = {0.0, 0.0, 0.0};
  double accel_norm_sum = 0.0;
  uint16_t valid_count = 0;
  uint16_t error_count = 0;
  const uint32_t start_ms = millis();

  // 最多等待 15 秒，避免传感器异常时串口命令永久卡住。
  while (valid_count < sample_target && (millis() - start_ms) < 15000) {
    inv_imu_sensor_data_t raw {};
    const int status = imu.getDataFromRegisters(raw);
    if (status != 0) {
      // 单次读取失败不立即退出，累计错误数后继续尝试，便于观察偶发总线错误。
      error_count++;
      delay(5);
      continue;
    }

    // 原始 int16 数据先换算为物理单位，再参与平均，输出更容易直接用于飞控标定判断。
    const float accel_g[3] = {
        rawAccelToG(raw.accel_data[0]),
        rawAccelToG(raw.accel_data[1]),
        rawAccelToG(raw.accel_data[2]),
    };
    const float gyro_dps[3] = {
        rawGyroToDps(raw.gyro_data[0]),
        rawGyroToDps(raw.gyro_data[1]),
        rawGyroToDps(raw.gyro_data[2]),
    };

    for (uint8_t axis = 0; axis < 3; axis++) {
      // 三轴分别累加，后续除以 valid_count 得到每个轴的均值。
      accel_sum[axis] += accel_g[axis];
      gyro_sum[axis] += gyro_dps[axis];
    }
    // 加速度模长用于判断整体量纲是否正确，不依赖模块摆放方向。
    accel_norm_sum += sqrtf(accel_g[0] * accel_g[0] +
                            accel_g[1] * accel_g[1] +
                            accel_g[2] * accel_g[2]);
    valid_count++;
    // 5 ms 对应约 200 Hz 读取节奏，和本示例默认 ODR 对齐。
    delay(5);
  }

  if (valid_count == 0) {
    Serial.println(F("偏置估计失败：没有有效样本。"));
    return;
  }

  Serial.print(F("bias valid="));
  Serial.print(valid_count);
  Serial.print(F(" errors="));
  Serial.print(error_count);
  Serial.print(F(" accel_avg_g="));
  Serial.print(accel_sum[0] / valid_count, 6);
  Serial.print(',');
  Serial.print(accel_sum[1] / valid_count, 6);
  Serial.print(',');
  Serial.print(accel_sum[2] / valid_count, 6);
  Serial.print(F(" accel_norm_avg_g="));
  Serial.print(accel_norm_sum / valid_count, 6);
  Serial.print(F(" gyro_bias_dps="));
  Serial.print(gyro_sum[0] / valid_count, 6);
  Serial.print(',');
  Serial.print(gyro_sum[1] / valid_count, 6);
  Serial.print(',');
  Serial.println(gyro_sum[2] / valid_count, 6);
}

/*
 * 读取并打印一帧 FlightAdapter 样本。
 *
 * FlightAdapter 是飞控主链路推荐接口，调用者无需自己处理 raw->g/dps/degC 换算。
 * 该函数每次成功读取后都会同步更新示例统计量。
 */
static void printFlightSample()
{
  ICM45686FlightSample sample;
  const int status = flight_imu.readSample(sample);
  if (status != 0) {
    // 读取失败时只打印错误并返回，不使用旧样本，避免飞控示例误导为“数据仍有效”。
    stats.read_errors++;
    Serial.print(F("flight_imu.readSample() 失败，status="));
    Serial.println(status);
    return;
  }

  stats.flight_samples++;
  // FlightAdapter 已经给出物理单位和 saturated 标志，可直接进入公共统计。
  updateCommonStats(sample.accel_g, sample.gyro_dps, sample.saturated);

  // 输出一行紧凑的 CSV-like 文本，方便串口监视器观察，也方便后续复制到日志工具。
  Serial.print(F("flight t_us="));
  Serial.print(sample.timestamp_us);
  Serial.print(F(" accel_g="));
  Serial.print(sample.accel_g[0], 5);
  Serial.print(',');
  Serial.print(sample.accel_g[1], 5);
  Serial.print(',');
  Serial.print(sample.accel_g[2], 5);
  Serial.print(F(" gyro_dps="));
  Serial.print(sample.gyro_dps[0], 5);
  Serial.print(',');
  Serial.print(sample.gyro_dps[1], 5);
  Serial.print(',');
  Serial.print(sample.gyro_dps[2], 5);
  Serial.print(F(" norm_g="));
  Serial.print(sample.accel_norm_g, 5);
  Serial.print(F(" temp_C="));
  Serial.print(sample.temperature_c, 2);
  Serial.print(F(" saturated="));
  Serial.print(sample.saturated);
  Serial.print(F(" fresh="));
  // fresh 依据最后成功样本时间判断，可用于飞控上层传感器超时保护。
  Serial.println(flight_imu.hasFreshSample(micros()));
}

/*
 * 读取并打印一帧普通寄存器样本。
 *
 * 该路径直接调用 imu.getDataFromRegisters()，展示底层驱动返回的原始字段。
 * 当 FlightAdapter 输出异常时，可以用该模式判断问题在底层通信还是适配层换算。
 */
static void printRegisterSample()
{
  inv_imu_sensor_data_t raw {};
  const int status = imu.getDataFromRegisters(raw);
  if (status != 0) {
    // 非 0 返回值通常意味着传输失败、模式未启动或底层状态异常。
    stats.read_errors++;
    Serial.print(F("imu.getDataFromRegisters() 失败，status="));
    Serial.println(status);
    return;
  }

  // 手动换算为物理单位，便于和 FlightAdapter 模式逐项对照。
  const float accel_g[3] = {
      rawAccelToG(raw.accel_data[0]),
      rawAccelToG(raw.accel_data[1]),
      rawAccelToG(raw.accel_data[2]),
  };
  const float gyro_dps[3] = {
      rawGyroToDps(raw.gyro_data[0]),
      rawGyroToDps(raw.gyro_data[1]),
      rawGyroToDps(raw.gyro_data[2]),
  };
  // 六个原始通道任意一个接近满量程，都认为这一帧存在饱和风险。
  const bool saturated = nearRawLimit(raw.accel_data[0]) || nearRawLimit(raw.accel_data[1]) ||
                         nearRawLimit(raw.accel_data[2]) || nearRawLimit(raw.gyro_data[0]) ||
                         nearRawLimit(raw.gyro_data[1]) || nearRawLimit(raw.gyro_data[2]);

  stats.register_samples++;
  updateCommonStats(accel_g, gyro_dps, saturated);

  // 同时打印 raw 和换算后的物理量，便于排查量程、符号和轴向。
  Serial.print(F("reg raw_a="));
  Serial.print(raw.accel_data[0]);
  Serial.print(',');
  Serial.print(raw.accel_data[1]);
  Serial.print(',');
  Serial.print(raw.accel_data[2]);
  Serial.print(F(" raw_g="));
  Serial.print(raw.gyro_data[0]);
  Serial.print(',');
  Serial.print(raw.gyro_data[1]);
  Serial.print(',');
  Serial.print(raw.gyro_data[2]);
  Serial.print(F(" accel_g="));
  Serial.print(accel_g[0], 5);
  Serial.print(',');
  Serial.print(accel_g[1], 5);
  Serial.print(',');
  Serial.print(accel_g[2], 5);
  Serial.print(F(" gyro_dps="));
  Serial.print(gyro_dps[0], 5);
  Serial.print(',');
  Serial.print(gyro_dps[1], 5);
  Serial.print(',');
  Serial.print(gyro_dps[2], 5);
  Serial.print(F(" temp_C="));
  Serial.print(registerTempToC(raw.temp_data), 2);
  Serial.print(F(" saturated="));
  Serial.println(saturated);
}

/*
 * 根据一次 FIFO 中断读取 kFifoWatermark 帧。
 *
 * 调用前提：
 * - 已进入 FifoInterrupt 模式。
 * - loop() 观察到 irq_count 增加。
 *
 * 注意：
 * - watermark 只是“中断触发阈值”，实际 FIFO 中可能有更多或更少数据。
 * - 本示例按固定 kFifoWatermark 读取，是为了保持逻辑简单，便于验证中断和 frame 格式。
 * - 更严谨的飞控数据采集应读取 FIFO count 或使用驱动提供的完整 FIFO drain 逻辑。
 */
static void printFifoFramesForOneIrq()
{
  for (uint8_t frame_index = 0; frame_index < kFifoWatermark; frame_index++) {
    inv_imu_fifo_data_t frame {};
    const int status = imu.getDataFromFifo(frame);
    if (status != 0) {
      // FIFO 读取失败后立即退出当前中断的 drain，避免连续打印重复错误。
      stats.read_errors++;
      Serial.print(F("imu.getDataFromFifo() 失败，status="));
      Serial.println(status);
      return;
    }

    // ICM45686 普通 FIFO 六轴 frame 使用 byte_16 字段保存 accel/gyro/temp/timestamp。
    const float accel_g[3] = {
        rawAccelToG(frame.byte_16.accel_data[0]),
        rawAccelToG(frame.byte_16.accel_data[1]),
        rawAccelToG(frame.byte_16.accel_data[2]),
    };
    const float gyro_dps[3] = {
        rawGyroToDps(frame.byte_16.gyro_data[0]),
        rawGyroToDps(frame.byte_16.gyro_data[1]),
        rawGyroToDps(frame.byte_16.gyro_data[2]),
    };
    // FIFO 中同样检查原始满量程风险，避免高动态场景下误以为数据可信。
    const bool saturated = nearRawLimit(frame.byte_16.accel_data[0]) ||
                           nearRawLimit(frame.byte_16.accel_data[1]) ||
                           nearRawLimit(frame.byte_16.accel_data[2]) ||
                           nearRawLimit(frame.byte_16.gyro_data[0]) ||
                           nearRawLimit(frame.byte_16.gyro_data[1]) ||
                           nearRawLimit(frame.byte_16.gyro_data[2]);

    stats.fifo_frames++;
    updateCommonStats(accel_g, gyro_dps, saturated);

    // header 用于确认 frame 类型；timestamp 是芯片 FIFO 时间戳，不是主控 micros()。
    Serial.print(F("fifo header=0x"));
    if (frame.header.Byte < 16) {
      Serial.print('0');
    }
    Serial.print(frame.header.Byte, HEX);
    Serial.print(F(" ts="));
    Serial.print(frame.byte_16.timestamp);
    Serial.print(F(" accel_g="));
    Serial.print(accel_g[0], 5);
    Serial.print(',');
    Serial.print(accel_g[1], 5);
    Serial.print(',');
    Serial.print(accel_g[2], 5);
    Serial.print(F(" gyro_dps="));
    Serial.print(gyro_dps[0], 5);
    Serial.print(',');
    Serial.print(gyro_dps[1], 5);
    Serial.print(',');
    Serial.print(gyro_dps[2], 5);
    Serial.print(F(" temp_C="));
    Serial.print(fifoTempToC(frame.byte_16.temp_data), 2);
    Serial.print(F(" saturated="));
    Serial.println(saturated);
  }
}

/*
 * 读取并打印 APEX 事件状态。
 *
 * APEX 事件不是普通连续采样数据，而是芯片内部 eDMP 根据运动状态产生的离散事件。
 * loop() 只在 INT1 中断计数变化后调用本函数，减少无意义轮询。
 *
 * 返回值约定由库封装决定：
 * - 1：有对应事件。
 * - 0：当前没有该事件。
 * - 负数：功能未启用、状态异常或底层错误。
 */
static void printApexEvents()
{
  bool printed = false;

  // Tilt 事件用于检测设备姿态是否发生明显翻转或倾斜变化。
  if (imu.getTilt() == 1) {
    Serial.print(F("APEX Tilt "));
    printed = true;
  }

  // Pedometer 会返回步数、步频和活动状态字符串；这些更偏可穿戴场景，飞控一般只做能力验证。
  uint32_t step_count = 0;
  float step_cadence = 0.0f;
  char *activity = nullptr;
  if (imu.getPedometer(step_count, step_cadence, activity) == 1) {
    Serial.print(F("Pedometer step="));
    Serial.print(step_count);
    Serial.print(F(" cadence="));
    Serial.print(step_cadence, 3);
    Serial.print(F(" activity="));
    Serial.print(activity == nullptr ? "Unknown" : activity);
    Serial.print(' ');
    printed = true;
  }

  // RaiseToWake 返回 1 表示 wake，2 表示 sleep；这类事件也会依赖芯片内部状态机。
  const int r2w = imu.getRaiseToWake();
  if (r2w == 1) {
    Serial.print(F("R2W Wake "));
    printed = true;
  } else if (r2w == 2) {
    Serial.print(F("R2W Sleep "));
    printed = true;
  }

  // Tap 事件会给出敲击次数、轴和方向，适合验证 eDMP 中断链路是否工作。
  uint8_t tap_count = 0;
  uint8_t axis = 0;
  uint8_t direction = 0;
  if (imu.getTap(tap_count, axis, direction) == 1) {
    static const char *const kAxisName[] = {"X", "Y", "Z"};
    static const char *const kDirectionName[] = {"+", "-"};
    Serial.print(F("Tap count="));
    Serial.print(tap_count);
    Serial.print(F(" axis="));
    Serial.print(axis < 3 ? kAxisName[axis] : "?");
    Serial.print(F(" dir="));
    Serial.print(direction < 2 ? kDirectionName[direction] : "?");
    Serial.print(' ');
    printed = true;
  }

  // 自由落体事件给出持续时间，飞控项目中如果使用它，必须结合自己的安全逻辑二次判定。
  uint32_t freefall_ms = 0;
  if (imu.getFreefall(freefall_ms) == 1) {
    Serial.print(F("FreeFall ms="));
    Serial.print(freefall_ms);
    Serial.print(' ');
    printed = true;
  }

  // HighG/LowG 是阈值类事件，适合做冲击/失重诊断，不建议直接驱动控制闭环。
  if (imu.getHighG() == 1) {
    Serial.print(F("HighG "));
    printed = true;
  }
  if (imu.getLowG() == 1) {
    Serial.print(F("LowG "));
    printed = true;
  }

  // 多个事件可能在同一次中断后同时出现，统一在一行结束，串口日志更容易阅读。
  if (printed) {
    Serial.println();
  }
}

/*
 * 打印当前模式统计信息。
 *
 * 输出分两部分：
 * 1. 示例自己的跨模式统计：样本数、IRQ、错误数、饱和数、dt、加速度模长和陀螺最大值。
 * 2. FlightAdapter 内部统计：只在 fc 模式下有明显意义，但其他模式打印也方便确认其状态。
 */
static void printStats()
{
  // total_samples 只统计成功读到的连续数据样本；APEX/WOM 事件不计入这里。
  const uint32_t total_samples = stats.flight_samples + stats.register_samples + stats.fifo_frames;
  Serial.print(F("stats flight="));
  Serial.print(stats.flight_samples);
  Serial.print(F(" reg="));
  Serial.print(stats.register_samples);
  Serial.print(F(" fifo="));
  Serial.print(stats.fifo_frames);
  Serial.print(F(" irq="));
  Serial.print(takeIrqCount());
  Serial.print(F(" errors="));
  Serial.print(stats.read_errors);
  Serial.print(F(" saturated="));
  Serial.print(stats.saturated_samples);

  if (total_samples > 1) {
    // N 个样本只有 N-1 个间隔，所以平均 dt 用 total_samples - 1。
    const float avg_dt_us = static_cast<float>(stats.dt_sum_us) / static_cast<float>(total_samples - 1);
    Serial.print(F(" dt_us[min/avg/max]="));
    Serial.print(stats.dt_min_us);
    Serial.print('/');
    Serial.print(avg_dt_us, 1);
    Serial.print('/');
    Serial.print(stats.dt_max_us);
  }

  if (total_samples > 0) {
    // 加速度模长平均值静止时应接近 1 g；偏离太多时优先检查量程、单位和安装状态。
    Serial.print(F(" accel_norm_g[min/avg/max]="));
    Serial.print(stats.accel_norm_min_g, 5);
    Serial.print('/');
    Serial.print(stats.accel_norm_sum_g / total_samples, 5);
    Serial.print('/');
    Serial.print(stats.accel_norm_max_g, 5);
    Serial.print(F(" gyro_abs_max_dps="));
    Serial.print(stats.gyro_abs_max_dps, 5);
  }
  Serial.println();

  // FlightAdapter 自带统计只由 flight_imu.readSample() 更新，用来验证飞控推荐接口自己的错误计数。
  const ICM45686FlightStats &fc_stats = flight_imu.stats();
  Serial.print(F("flight_adapter_stats ok="));
  Serial.print(fc_stats.successful_samples);
  Serial.print(F(" read_errors="));
  Serial.print(fc_stats.read_errors);
  Serial.print(F(" last_status="));
  Serial.print(fc_stats.last_status);
  Serial.print(F(" saturated="));
  Serial.println(fc_stats.saturated_samples);
}

/*
 * 打印芯片和库的能力边界。
 *
 * 这个函数不访问硬件，只用于提醒后续项目调用者：
 * 哪些功能适合 ICM45686，哪些能力属于其他芯片族或不应放在飞控主链路里。
 */
static void printCapabilityNotes()
{
  Serial.println(F("能力边界说明："));
  Serial.println(F("- ICM45686 属于 INV_TYPE_A1，支持六轴、FIFO、APEX A 类事件、Wake on Motion、AUX I2CM。"));
  Serial.println(F("- GAF/四元数融合接口不在 ICM45686(A1) 宏下编译，VocalVibDet 是 B1-only。"));
  Serial.println(F("- AUX 磁力计路径默认目标为 ICT1531/兼容磁力计地址 0x1E。"));
  Serial.println(F("- 飞控主链路建议使用 FlightAdapter；APEX/FIFO/WOM 作为诊断或特定功能路径。"));
  Serial.println(F("- 切换复杂模式后建议重新 begin()，本示例已经在模式切换时这样做。"));
}

/*
 * 打印串口命令帮助。
 *
 * 本示例不在 setup() 自动启动复杂功能，而是通过命令进入不同模式，原因是：
 * - FIFO/APEX/WOM 会互相改写芯片内部配置。
 * - 分步命令更适合 bring-up：先 WHO_AM_I，再普通六轴，再中断类能力。
 */
static void printHelp()
{
  Serial.println();
  Serial.println(F("ICM45686 综合功能 SPI 示例命令"));
  Serial.println(F("h/help   打印帮助"));
  Serial.println(F("i        直接读 WHO_AM_I 并执行 imu.begin()"));
  Serial.println(F("fc       飞控适配层流：推荐后续飞控主循环参考"));
  Serial.println(F("reg      底层寄存器轮询流：原始值 + 手动单位换算"));
  Serial.println(F("fifo     FIFO + INT1 中断读取：需要连接 INT1"));
  Serial.println(F("apex     APEX 综合事件：Tilt/Pedometer/R2W/Tap/FreeFall/HighG/LowG"));
  Serial.println(F("wom      Wake on Motion：需要连接 INT1"));
  Serial.println(F("bias     静止偏置估计：观察陀螺零偏和 1 g 模长"));
  Serial.println(F("aux      AUX I2CM/外部磁力计预留测试"));
  Serial.println(F("s        打印统计信息"));
  Serial.println(F("note     打印芯片/库能力边界"));
  Serial.println(F("stop     停止当前模式"));
}

/*
 * 处理一条串口命令。
 *
 * 参数：
 * - command：从 Serial 读取到的一整行文本，函数内部会 trim 并转成小写。
 *
 * 设计说明：
 * - 所有模式切换函数都自行调用 stopCurrentMode()，所以命令分发处不需要重复停止。
 * - 未知命令会重新打印帮助，方便现场串口调试。
 */
static void handleCommand(String command)
{
  // 去掉行尾 \r\n 和前后空格，兼容不同串口监视器发送格式。
  command.trim();
  // 命令统一转小写，用户输入 HELP/Fc/Reg 也能识别。
  command.toLowerCase();
  if (command.length() == 0) {
    return;
  }

  // 基础信息和初始化。
  if (command == "h" || command == "help") {
    printHelp();
  } else if (command == "i") {
    beginRawImu();
  // 飞控推荐数据路径。
  } else if (command == "fc") {
    startFlightAdapter();
  // 底层寄存器和 FIFO 诊断路径。
  } else if (command == "reg") {
    enterRegisterStream();
  } else if (command == "fifo") {
    enterFifoInterruptMode();
  // 芯片内部 eDMP/APEX 事件路径。
  } else if (command == "apex") {
    enterApexEventsMode();
  } else if (command == "wom") {
    enterWakeOnMotionMode();
  // 一次性诊断命令。
  } else if (command == "bias") {
    runStationaryBiasEstimate(600);
  } else if (command == "aux") {
    runExternalI2cmProbe();
  } else if (command == "s") {
    printStats();
  } else if (command == "note") {
    printCapabilityNotes();
  } else if (command == "stop") {
    stopCurrentMode();
  } else {
    Serial.print(F("未知命令："));
    Serial.println(command);
    printHelp();
  }
}

/*
 * 处理串口输入。
 *
 * 当前实现采用 readStringUntil('\n')，优点是示例代码简单易读；缺点是如果串口监视器
 * 不发送换行，可能等待 Serial timeout。实际飞控主循环不要使用这种阻塞式命令解析。
 */
static void processSerial()
{
  if (!Serial.available()) {
    return;
  }
  handleCommand(Serial.readStringUntil('\n'));
}

/*
 * 执行当前活动模式。
 *
 * loop() 每次都会调用本函数。本函数只根据 current_mode 做少量工作：
 * - 连续流模式按固定周期打印样本，避免串口输出过快拖慢系统。
 * - 中断模式只在 irq_count 变化后读取 FIFO/APEX/WOM。
 * - 所有活动模式每 kStatsPrintPeriodMs 打印一次统计信息。
 */
static void runActiveMode()
{
  const uint32_t now_ms = millis();

  switch (current_mode) {
    case DemoMode::Idle:
      // 空闲模式不主动访问 IMU，让用户可以先看帮助或重新发命令。
      break;

    case DemoMode::FlightAdapterStream:
      if (now_ms - last_flight_print_ms >= kFlightPrintPeriodMs) {
        // 20 ms 输出一次，对应 50 Hz 串口显示；传感器本身仍按 kDefaultOdrHz 配置。
        last_flight_print_ms = now_ms;
        printFlightSample();
      }
      break;

    case DemoMode::RegisterStream:
      if (now_ms - last_register_print_ms >= kRegisterPrintPeriodMs) {
        // 原始寄存器流也节流到 50 Hz，避免 Serial.print() 成为主要负载。
        last_register_print_ms = now_ms;
        printRegisterSample();
      }
      break;

    case DemoMode::FifoInterrupt: {
      const uint32_t irq_now = takeIrqCount();
      if (irq_now != stats.irq_snapshot) {
        // 如果串口输出太慢，可能一次积累很多中断。这里最多处理 20 次，避免 loop() 长时间卡住。
        const uint32_t pending_irq = min<uint32_t>(irq_now - stats.irq_snapshot, 20);
        stats.irq_snapshot = irq_now;
        for (uint32_t i = 0; i < pending_irq; i++) {
          printFifoFramesForOneIrq();
        }
      }
      break;
    }

    case DemoMode::ApexEvents:
      if (takeIrqCount() != stats.irq_snapshot) {
        // 只在新中断后刷新事件状态，降低无事件时的 SPI 访问量。
        stats.irq_snapshot = takeIrqCount();
        printApexEvents();
      }
      break;

    case DemoMode::WakeOnMotion:
      if (takeIrqCount() != stats.irq_snapshot) {
        // WOM 示例只关心是否发生唤醒中断，因此打印累计中断数即可。
        stats.irq_snapshot = takeIrqCount();
        Serial.print(F("Wake on Motion IRQ count="));
        Serial.println(stats.irq_snapshot);
      }
      break;

    case DemoMode::ExternalI2cm:
      // AUX/I2CM 当前是一次性探测命令，不在 loop 中持续读取。
      break;
  }

  if (current_mode != DemoMode::Idle && now_ms - last_stats_print_ms >= kStatsPrintPeriodMs) {
    // 周期性统计用于观察长期稳定性，不需要用户反复输入 s 命令。
    last_stats_print_ms = now_ms;
    printStats();
  }
}

/*
 * Arduino 初始化入口。
 *
 * 本函数只完成三件事：
 * 1. 打开串口。
 * 2. 初始化 SPI 引脚和 CS 空闲电平。
 * 3. 打印接线、命令和安全提示。
 *
 * 注意：setup() 不自动调用 imu.begin()，这样用户可以先确认串口输出的接线宏是否正确。
 */
void setup()
{
  Serial.begin(115200);
  // 给 USB CDC 或串口监视器一点枚举/打开时间，避免启动信息被漏掉。
  delay(1500);

  // CS 空闲必须为高电平；如果上电阶段 CS 浮空，可能导致从机误认为有半截 SPI 命令。
  pinMode(ICM45686_EXAMPLE_SPI_CS, OUTPUT);
  digitalWrite(ICM45686_EXAMPLE_SPI_CS, HIGH);
  // ESP32 可指定 SPI 引脚；STM32/其他 Arduino 平台如果不支持这个重载，需要改为平台对应写法。
  SPI.begin(ICM45686_EXAMPLE_SPI_SCK, ICM45686_EXAMPLE_SPI_MISO, ICM45686_EXAMPLE_SPI_MOSI,
            ICM45686_EXAMPLE_SPI_CS);

  Serial.println();
  Serial.println(F("ICM45686 综合功能 SPI 示例"));
  Serial.print(F("SPI SCK="));
  Serial.print(ICM45686_EXAMPLE_SPI_SCK);
  Serial.print(F(" MISO="));
  Serial.print(ICM45686_EXAMPLE_SPI_MISO);
  Serial.print(F(" MOSI="));
  Serial.print(ICM45686_EXAMPLE_SPI_MOSI);
  Serial.print(F(" CS="));
  Serial.print(ICM45686_EXAMPLE_SPI_CS);
  Serial.print(F(" Hz="));
  Serial.print(ICM45686_EXAMPLE_SPI_HZ);
  Serial.print(F(" mode=3 INT="));
  Serial.println(ICM45686_EXAMPLE_INT_PIN);

  printHelp();
  Serial.println(F("建议执行顺序：i -> fc -> s -> bias；接好 INT1 后再执行 fifo/apex/wom。"));
  Serial.println(F("本示例不会自动上传或驱动任何执行机构；后续接入飞控/电机前请断开动力电源。"));
}

/*
 * Arduino 主循环。
 *
 * 结构刻意保持简单：
 * - 先处理用户命令，允许随时 stop 或切换模式。
 * - 再执行当前活动模式的数据读取或事件处理。
 *
 * 飞控项目中应把传感器读取放进确定周期的任务调度器，本示例为了可读性使用普通 loop()。
 */
void loop()
{
  processSerial();
  runActiveMode();
}
