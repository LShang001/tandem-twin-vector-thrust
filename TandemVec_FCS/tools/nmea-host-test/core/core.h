// core/core.h — 非 Arduino 主机环境的最小兼容层（ubx.h 平台分支的原设计）
// 提供 HardwareSerial 兼容类（测试喂字节）+ 可控时钟 + Arduino 常用宏。
#pragma once

#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

// ---- 可控时钟：测试用全局变量推进，模拟时间流逝 ----
extern uint32_t g_sim_millis;
extern uint32_t g_sim_micros;
inline uint32_t millis() { return g_sim_millis; }
inline uint32_t micros() { return g_sim_micros; }
inline void delay(uint32_t ms)
{
  g_sim_millis += ms;
  g_sim_micros += ms * 1000U;
}

#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

// ---- HardwareSerial 兼容层：available/read 从内部 FIFO 取字节 ----
class HardwareSerial
{
public:
  void begin(int32_t baud) { begin_baud_ = baud; }
  void flush() {}
  int32_t begin_baud() const { return begin_baud_; }
  int available() const { return static_cast<int>(tail_ - head_); }
  int read() { return head_ < tail_ ? buf_[head_++] : -1; }
  void clear() { head_ = tail_ = 0; }
  // ---- 测试辅助：向 FIFO 喂字节 ----
  void feed(const void *data, size_t n)
  {
    const uint8_t *p = static_cast<const uint8_t *>(data);
    while (n-- > 0 && tail_ < sizeof(buf_))
    {
      buf_[tail_++] = *p++;
    }
  }
  void feed(const char *s) { feed(s, strlen(s)); }

private:
  uint8_t buf_[4096];
  size_t head_ = 0;
  size_t tail_ = 0;
  int32_t begin_baud_ = 0;
};
