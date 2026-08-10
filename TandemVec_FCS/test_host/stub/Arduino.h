// 宿主机测试用 Arduino.h 极简桩
// VerticalKF.h / VerticalKF_2State.h / HorizontalKF.h / Ws2812AppStatus.h 等
// 仅 #include <Arduino.h> 后使用 isnan / fmaxf / PI / pow 等符号，
// 这些在标准 <math.h> / <cmath> 中已提供（PI 为 Arduino 同名宏）。
// 本桩仅转发到标准库，使这些平台相关头文件可在宿主机用 g++ 编译。
// 注意：仅在 test_host/ 宿主机测试编译路径下通过 -Itest_host/stub 优先命中，
// 不影响 PlatformIO 固件编译（固件使用真实的 Arduino.h）。
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
