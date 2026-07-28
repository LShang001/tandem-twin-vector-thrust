// 宿主机测试用 Arduino.h 极简桩
// VerticalKF.h / VerticalKF_2State.h / HorizontalKF.h 仅 #include <Arduino.h> 后使用
// isnan / fmaxf 等符号，这些在标准 <math.h> / <cmath> 中已提供。
// 本桩仅转发到标准库，使这些平台相关头文件可在宿主机用 g++ 编译。
// 注意：仅在 test_host/ 宿主机测试编译路径下通过 -Itest_host/stub 优先命中，
// 不影响 PlatformIO 固件编译（固件使用真实的 Arduino.h）。
#ifndef TEST_HOST_ARDUINO_STUB_H
#define TEST_HOST_ARDUINO_STUB_H

#include <math.h>
#include <string.h>
#include <cstdint>

#endif // TEST_HOST_ARDUINO_STUB_H
