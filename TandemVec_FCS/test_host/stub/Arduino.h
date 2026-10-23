/**
 * @file    Arduino.h
 * @module  固件/宿主回归桩（fw-tests：Arduino 框架最小替身）
 * @brief   把 #include <Arduino.h> 转发到标准库，使平台相关头文件可在宿主机编译
 * @details 被测：无（替身层）。VerticalKF.h / VerticalKF_2State.h / HorizontalKF.h /
 *          Ws2812AppStatus.h 等只用到 isnan、fmaxf、PI、pow、Stream 基类与 constrain，
 *          这些在标准库或本桩中提供。
 *          输入：测试文件的 #include <Arduino.h>；处理：只做声明与宏转发；
 *          输出：可在 g++ 下编译的头文件。
 * @theory  N/A（框架替身，无算法）
 * @frame   N/A
 * @units   N/A
 * @note    与真实 Arduino 的差异：无 millis()/micros() 时间基准、无引脚 I/O、
 *          无 Serial 实现；`constrain` 在部分测试里被宏覆盖，故本桩用 #ifndef 保护。
 *          仅当 -Itest_host/stub 排在真实 Arduino 核之前时生效，不影响 pio 固件编译。
 * @see     test_host/fw_sil/stub/Arduino.h（SIL 用的更大替身）、test_host/README.md
 */
#ifndef TEST_HOST_ARDUINO_STUB_H
#define TEST_HOST_ARDUINO_STUB_H

#include <math.h>
#include <string.h>
#include <cstdint>
#include <stddef.h>

// Arduino 框架常量（Ws2812AppStatus.h 呼吸相位计算使用）
#ifndef PI
#define PI 3.1415926535897932384626433832795f
#endif

// ★ 2026-08-10 COMM-001：AnoComProtocol.h 需要 Stream 基类
//   （仅使用 available/read/write 三个成员，host 测试 mock 之）
class Stream
{
public:
    virtual int available() = 0;
    virtual int read() = 0;
    virtual size_t write(const uint8_t *buf, size_t len) = 0;
};

// Arduino 的 constrain（AnoComProtocol.cpp sendIMUData 等缩放使用）
// 部分测试文件自带 #define constrain 宏（宏优先于模板）——仅当未定义宏时提供模板
#ifndef constrain
template <typename T>
static T constrain(T v, T lo, T hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}
#endif

#endif // TEST_HOST_ARDUINO_STUB_H
