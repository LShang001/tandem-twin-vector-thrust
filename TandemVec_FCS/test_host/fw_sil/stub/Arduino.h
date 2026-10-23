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
  size_t println() { return 0; }
  template <typename T> size_t println(T) { return 0; }
  template <typename T> size_t print(const T&) { return 0; }
};

class Stream : public Print {
public:
  virtual int available() { return 0; }
  virtual int read() { return -1; }
  virtual int peek() { return -1; }
  virtual void flush() {}
  using Print::write;
  virtual size_t write(uint8_t b) { return Print::write(b); }
  size_t write(const uint8_t* b, size_t n) override { return Print::write(b, n); }
};

// Arduino Core 3.0 将串口具体类命名为 Uart；宿主替身沿用 HardwareSerial 实现。
namespace arduino { using ::Print; }

class HardwareTimer {
public:
  HardwareTimer() {}
  HardwareTimer(int) {}
  void pause() {}
  void resume() {}
  void setOverflow(unsigned long, int) {}
  void attachInterrupt(void (*)()) {}
};
// 仿真虚拟时钟：由 SIL 主循环按仿真时间推进；未设置时保持 0，兼容旧 SIL。
extern uint32_t g_sil_time_us;

class HardwareSerial : public Stream {
public:
  HardwareSerial() {}
  HardwareSerial(int, int) {}
  void begin(unsigned long) {}
  void begin(unsigned long, int) {}
  void end() {}
  operator bool() const { return true; }

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
inline void delay(unsigned long) {}
inline void delayMicroseconds(unsigned int) {}
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
