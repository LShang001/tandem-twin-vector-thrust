/**
 * @file    Arduino.h
 * @module  固件/宿主 SIL 桩（fw-tests：Arduino 框架替身）
 * @brief   提供真实固件源码在宿主机编译所需的 Arduino/STM32 类型、宏与空外设
 * @details 被测：无（替身层）。真实固件 `src/flight_control.cpp`、`src/state_data.cpp`
 *          通过 `-Itest_host/fw_sil/stub` 优先命中本文件，而不是 PlatformIO 的 Arduino 核。
 *          输入：固件源码里的 #include <Arduino.h>、引脚宏、Serial/SPI 调用；
 *          处理：只给类型、常量与空实现；不模拟时序、中断、PWM 与寄存器；
 *          输出：可编译的头文件。
 * @theory  N/A（框架替身，无算法）
 * @frame   N/A
 * @units   N/A
 * @note    与真实 Arduino 的差异：millis()/micros() 由 SIL 主程序维护的虚拟时间推进，
 *          引脚宏只是编号，digitalWrite 等为空实现——因此外设时序、中断延迟、
 *          PWM 分辨率（SIL 另行按 0.1% 量化近似）都不能由本桩代表。
 * @see     test_host/stub/Arduino.h（普通回归用的更小替身）、
 *          TandemVec_FCS/test_host/fw_sil/README.md
 */
#pragma once
// ============================================================
// Host SIL Arduino stub — 让真实固件源码在宿主机编译
// 只提供类型/常量/空实现；不模拟外设行为（SIL 自己注入状态）
// ============================================================
#include <cmath>
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cstdio>
#include <string>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Arduino 的 Print 进制常量：固件用 Serial8.print(v, HEX) 打印寄存器值。
// 宿主沿用 Arduino 的取值，避免源码里出现条件分支。
#ifndef DEC
#define DEC 10
#endif
#ifndef HEX
#define HEX 16
#endif
#ifndef OCT
#define OCT 8
#endif
#ifndef BIN
#define BIN 2
#ifndef SERIAL_8N1
#define SERIAL_8N1 0x06
#endif

// HAL/CMSIS 类型与寄存器替身：通信层/主程序会直接访问 GPIOD->BSRR、UART4->CR1、
// NVIC_* 与 F_CPU。详见同目录 stm32h7xx_hal.h；那里是纯内存实例，不做硬件访问。
#include "stm32h7xx_hal.h"
#endif

#ifndef PI
#define PI 3.14159265358979323846f
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.29577951308232f
#endif
#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295f
#endif
#ifndef HIGH
#define HIGH 1
#define LOW 0
#endif
// Minimal STM32 pin names used by state_data/board config
#define PA0 0
#define PA1 1
#define PA2 2
#define PA3 3
#define PA4 4
#define PA5 5
#define PA6 6
#define PA7 7
#define PA8 8
#define PA9 9
#define PA10 10
#define PA11 11
#define PA12 12
#define PA13 13
#define PA14 14
#define PA15 15
#define PB0 16
#define PB1 17
#define PB2 18
#define PB3 19
#define PB4 20
#define PB5 21
#define PB6 22
#define PB7 23
#define PB8 24
#define PB9 25
#define PB10 26
#define PB11 27
#define PB12 28
#define PB13 29
#define PB14 30
#define PB15 31
#define PC0 32
#define PC1 33
#define PC2 34
#define PC3 35
#define PC4 36
#define PC5 37
#define PC6 38
#define PC7 39
#define PC8 40
#define PC9 41
#define PC10 42
#define PC11 43
#define PC12 44
#define PC13 45
#define PC14 46
#define PC15 47
#define PD0 48
#define PD1 49
#define PD2 50
#define PD3 51
#define PD4 52
#define PD5 53
#define PD6 54
#define PD7 55
#define PD8 56
#define PD9 57
#define PD10 58
#define PD11 59
#define PD12 60
#define PD13 61
#define PD14 62
#define PD15 63
#define PE0 64
#define PE1 65
#define PE2 66
#define PE3 67
#define PE4 68
#define PE5 69
#define PE6 70
#define PE7 71
#define PE8 72
#define PE9 73
#define PE10 74
#define PE11 75
#define PE12 76
#define PE13 77
#define PE14 78
#define PE15 79
#define PF0 80
#define PF1 81
#define PF2 82
#define PF3 83
#define PF4 84
#define PF5 85
#define PF6 86
#define PF7 87
#define PF8 88
#define PF9 89
#define PF10 90
#define PF11 91
#define PF12 92
#define PF13 93
#define PF14 94
#define PF15 95
#define PG0 96
#define PG1 97
#define PG2 98
#define PG3 99
#define PG4 100
#define PG5 101
#define PG6 102
#define PG7 103
#define PG8 104
#define PG9 105
#define PG10 106
#define PG11 107
#define PG12 108
#define PG13 109
#define PG14 110
#define PG15 111
#define PH0 112
#define PH1 113
#define PH2 114
#define PH3 115
#define PH4 116
#define PH5 117
#define PH6 118
#define PH7 119
#define PH8 120
#define PH9 121
#define PH10 122
#define PH11 123
#define PH12 124
#define PH13 125
#define PH14 126
#define PH15 127
#define PI0 128
#define PI1 129
#define PI2 130
#define PI3 131
#define PI4 132
#define PI5 133
#define PI6 134
#define PI7 135
#define PI8 136
#define PI9 137
#define PI10 138
#define PI11 139
#define PI12 140
#define PI13 141
#define PI14 142
#define PI15 143
#define PJ0 144
#define PJ1 145
#define PJ2 146
#define PJ3 147
#define PJ4 148
#define PJ5 149
#define PJ6 150
#define PJ7 151
#define PJ8 152
#define PJ9 153
#define PJ10 154
#define PJ11 155
#define PJ12 156
#define PJ13 157
#define PJ14 158
#define PJ15 159
#define PK0 160
#define PK1 161
#define PK2 162
#define PK3 163
#define PK4 164
#define PK5 165
#define PK6 166
#define PK7 167
#define PK8 168
#define PK9 169
#define PK10 170
#define PK11 171
#define PK12 172
#define PK13 173
#define PK14 174
#define PK15 175
#define TIM8 8
#ifndef INPUT
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#endif

using std::isnan;
using std::isinf;

class Print {
public:
  virtual ~Print() {}
  virtual size_t write(uint8_t) { return 1; }
  virtual size_t write(const uint8_t*, size_t n) { return n; }
  size_t print(const char*) { return 0; }
  size_t print(char) { return 0; }
  size_t print(int) { return 0; }
  size_t print(unsigned int) { return 0; }
  size_t print(long) { return 0; }
  size_t print(unsigned long) { return 0; }
  size_t print(float) { return 0; }
  size_t print(double) { return 0; }
  size_t print(float, int) { return 0; }
  size_t print(double, int) { return 0; }
  // 显式整数+进制重载：固件用 print(reg, HEX) 打印寄存器，缺整数版本时
  // 只有 (float,int)/(double,int) 可选，会导致 overloaded 'print(uint32_t, int)' ambiguous。
  size_t print(int, int) { return 0; }
  size_t print(unsigned int, int) { return 0; }
  size_t print(long, int) { return 0; }
  size_t print(unsigned long, int) { return 0; }
  size_t println(unsigned long, int) { return 0; }
  size_t println(float, int) { return 0; }
  size_t println(double, int) { return 0; }
  size_t println() { return 0; }
  size_t println(int, int) { return 0; }
  size_t println(unsigned int, int) { return 0; }
  size_t println(long, int) { return 0; }
  template <typename T> size_t println(T) { return 0; }
  template <typename T> size_t print(const T&) { return 0; }
};

class Stream : public Print {
public:
  virtual int available() { return 0; }
  virtual int read() { return -1; }
  virtual int peek() { return -1; }
  virtual void flush() {}
  /// 通信层用 `Stream*` 做非阻塞发送判据（AnoComProtocol.h 直接调用）。
  /// 宿主"串口"是内存缓冲，返回足够大的常量即可。
  virtual int availableForWrite() { return 4096; }
  using Print::write;
  virtual size_t write(uint8_t b) { return Print::write(b); }
  size_t write(const uint8_t* b, size_t n) override { return Print::write(b, n); }

  /// 按字节读满 len 或耗尽缓存（通信层读 GCS/上位机命令时使用）。
  /// 走虚函数 available/read，因此对 HardwareSerial 的环形缓冲同样有效。
  size_t readBytes(uint8_t* buf, size_t len) {
    size_t n = 0;
    while (n < len) {
      if (available() <= 0) break;
      const int c = read();
      if (c < 0) break;
      buf[n++] = static_cast<uint8_t>(c);
    }
    return n;
  }
  size_t readBytes(char* buf, size_t len) {
    return readBytes(reinterpret_cast<uint8_t*>(buf), len);
  }
};

// Arduino Core 3.0 将串口具体类命名为 Uart；宿主替身沿用 HardwareSerial 实现。
namespace arduino { using ::Print; }

// Arduino STM32 HardwareTimer 的格式选择常量（真实核里来自 HardwareTimer.h）。
// 全链路数字孪生要链接真实 src/task_scheduler.cpp，其 setupTimer() 与统计打印
// 都按这两个常量解释 overflow，故宿主必须提供同值定义。
#ifndef HERTZ_FORMAT
#define HERTZ_FORMAT 1
#endif
#ifndef TICK_FORMAT
#define TICK_FORMAT 0
#endif

// Arduino 的 F() 把字面量放进 Flash；宿主没有 Harvard 架构，直接退化为指针。
// 真实 src/task_scheduler.cpp 的统计打印用 out.println(F("[DBG] ..."))，
// 缺此宏会编译失败（错误信息会误导成 units.h 里的 TempUnit::F）。
#ifndef F
#define F(literal) (literal)
#endif

/**
 * @brief 定时器替身：记录配置与溢出回调，由全链路主程序按仿真时间手动触发
 * @details 真实固件 setupTimer() 只做三件事——设频率、挂中断、启动；
 *          宿主侧把这三项记下来，`silTick()` 等价于 TIM8 溢出中断触发一次。
 *          这样 task_scheduler.cpp 本体（tick 累加、到期判定、相位、丢拍统计）
 *          一行不改地运行，被替换的只有「谁按 156.25 µs 触发」这一层。
 * @note    `silRunning()` 用于断言 setupTimer() 确实启动过；未启动时 silTick() 不回调。
 */
class HardwareTimer {
public:
  HardwareTimer() {}
  HardwareTimer(int) {}
  void pause() { running_ = false; }
  void resume() { running_ = true; }
  void setOverflow(unsigned long value, int format) {
    overflow_ = value;
    hertz_format_ = (format == HERTZ_FORMAT);
  }
  void attachInterrupt(void (*cb)()) { cb_ = cb; }

  uint32_t getTimerClkFreq() { return 64000000UL; }
  uint32_t getPrescaleFactor() { return 0; }
  uint32_t getOverflow(int format) {
    if (format != HERTZ_FORMAT) return 0;
    return hertz_format_ ? static_cast<uint32_t>(overflow_) : 0;
  }

  /// 仿真时基推进一拍（等价真实 TIM8 溢出中断）；未 resume 时不触发。
  void silTick() {
    if (running_ && cb_ != nullptr) cb_();
  }
  bool silRunning() const { return running_; }
  bool silHertzFormat() const { return hertz_format_; }
  unsigned long silOverflow() const { return overflow_; }

private:
  void (*cb_)() = nullptr;
  unsigned long overflow_ = 0;
  bool running_ = false;
  bool hertz_format_ = true;
};
// 仿真虚拟时钟：由 SIL 主循环按仿真时间推进；未设置时保持 0，兼容旧 SIL。
extern uint32_t g_sil_time_us;

/**
 * @brief `delay()` 的时间推进钩子（全链路数字孪生专用）
 *
 * @details 真实固件在 setup 阶段用 `delay()` 等超时窗口，例如 DETA100 探测：
 *          `while (millis() - t0 < TIMEOUT) { ...; delay(1); }`。
 *          宿主的 `millis()` 由虚拟时钟给值，若 `delay()` 不推进时钟，该循环永不退出。
 *          因此把"推进多少时间"外置成钩子：默认 nullptr（既有 SIL 行为完全不变），
 *          全链路主程序安装后按 156.25 µs 步进虚拟时间并触发调度器 tick。
 * @note    固件里 `delay()` 期间 TIM8 中断照常运行，所以推进时也必须触发 tick，
 *          否则 setup 阶段的定时器语义与固件不同。
 */
using SilDelayHook = void (*)(uint32_t us);
extern SilDelayHook g_sil_delay_hook;

class HardwareSerial : public Stream {
public:
  HardwareSerial() {}
  HardwareSerial(int, int) {}
  void begin(unsigned long) {}
  void begin(unsigned long, int) {}
  void begin(unsigned long, uint32_t /*config*/) {}
  void end() {}
  operator bool() const { return true; }
  /// 发送缓冲剩余空间。宿主的"串口"是内存环形缓冲，几乎无限，
  /// 因此返回一个足够大的常量——通信层用它做非阻塞发送判据。
  int availableForWrite() { return 4096; }
  /// 固件里 `Serial4.getHandle()` 用于读回 UART 寄存器（FIFO/IRQ 配置）。
  /// 宿主返回一个静态句柄，寄存器访问落在 stm32h7xx_hal.h 的内存实例上。
  UART_HandleTypeDef *getHandle() { static UART_HandleTypeDef h{}; h.Instance = nullptr; return &h; }

  // ---- 发送字节捕获（全链路数字孪生：遥测帧契约验证）----
  // 默认行为不变（仍然返回 1/n），只是把写出的字节同时记进缓冲。
  // 这样"固件真实构造出来的帧"可以直接导出，再按协议解析回原值——
  // 比读源码推断字段偏移可靠得多。
  void silClearTx() { txLen_ = 0; }
  size_t silTxLen() const { return txLen_; }
  const uint8_t *silTxBuf() const { return txBuf_; }
  static constexpr size_t kSilTxCap = 8192;

  size_t write(uint8_t v) override {
    if (txLen_ < kSilTxCap) txBuf_[txLen_++] = v;
    return 1;
  }
  size_t write(const uint8_t *b, size_t n) override {
    for (size_t i = 0; i < n && txLen_ < kSilTxCap; ++i) txBuf_[txLen_++] = b[i];
    return n;
  }
  using Print::write;

  int available() override { return static_cast<int>(count_); }
  int read() override {
    if (count_ == 0) return -1;
    const uint8_t v = buf_[head_];
    head_ = (head_ + 1) % sizeof(buf_);
    --count_;
    return v;
  }
  int peek() override { return count_ > 0 ? buf_[head_] : -1; }
  void clear() { head_ = tail_ = count_ = 0; }
  // SIL 注入：模拟 UART 收到的字节（环形缓冲，满则丢弃最新字节）。
  void feed(const uint8_t *data, size_t n) {
    while (n-- > 0) {
      if (count_ >= sizeof(buf_)) break;
      buf_[tail_] = *data++;
      tail_ = (tail_ + 1) % sizeof(buf_);
      ++count_;
    }
  }

private:
  uint8_t buf_[4096] = {};
  size_t head_ = 0;
  size_t tail_ = 0;
  size_t count_ = 0;
  // 发送字节捕获缓冲（见上方 silTx* 接口）。默认行为不变，仅记录。
  uint8_t txBuf_[kSilTxCap] = {};
  size_t txLen_ = 0;
};

using Uart = HardwareSerial;

extern HardwareSerial Serial;
extern HardwareSerial Serial1;
extern HardwareSerial Serial2;
extern HardwareSerial Serial3;
extern HardwareSerial Serial4;
extern HardwareSerial Serial5;
extern HardwareSerial Serial6;
extern HardwareSerial Serial7;
extern HardwareSerial Serial8;

inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t, uint8_t) {}
inline int digitalRead(uint8_t) { return 0; }
inline void analogWrite(uint8_t, int) {}
inline int analogRead(uint8_t) { return 0; }
// 与 STM32/Arduino 一致返回 32 bit，避免 Linux 64 bit unsigned long 掩盖
// micros() 71.6 min 回绕与 dt_s 模运算路径。
inline uint32_t millis() { return g_sil_time_us / 1000U; }
inline uint32_t micros() { return g_sil_time_us; }
// delay() 默认不动虚拟时钟（既有 SIL 依赖这一点）；安装 g_sil_delay_hook 后
// 才推进时间，供全链路模式让 setup() 里的超时等待自然到期。
inline void delay(unsigned long ms) {
  if (g_sil_delay_hook != nullptr) {
    g_sil_delay_hook(static_cast<uint32_t>(ms) * 1000U);
  }
}
inline void delayMicroseconds(unsigned int) {}

// ---- setup() 用到的 CMSIS/STM32 符号（全链路 main.cpp 编译所需）----
// 只保证固件源码在宿主通过编译；不产生任何寄存器或引脚效果。
// 名称与真实 CMSIS/HAL 保持一致，避免源码里出现条件分支。

#ifndef TIM_CHANNEL_4
#define TIM_CHANNEL_4 0x00000004U
#endif

inline void digitalToggle(int) {}
inline void analogWriteFrequency(unsigned long) {}
inline void analogWriteFrequency(int, unsigned long) {}
inline void analogWriteResolution(int) {}
inline void analogReadResolution(int) {}
inline void yield() {}
// 宿主单线程：临界区为空实现；真实中断并发不在 SIL 覆盖范围。
inline void noInterrupts() {}
inline void interrupts() {}

template <typename T> static T constrain(T v, T lo, T hi) {
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}
template <typename T> static T min(T a, T b) { return a < b ? a : b; }
template <typename T> static T max(T a, T b) { return a > b ? a : b; }
template <typename T> static T abs(T a) { return a < 0 ? -a : a; }
using std::sqrt;
using std::sin;
using std::cos;
using std::atan2;
using std::fabs;
using std::pow;

// 仅让固件里的 String 成员声明/简单用法编译
class String : public std::string {
public:
  String() {}
  String(const char* s) : std::string(s) {}
  String(const std::string& s) : std::string(s) {}
  String(double, int) {}
  String(int, int) {}
};
