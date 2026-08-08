// 宿主机测试用 WS2812Driver 极简 mock（仅覆盖 Ws2812AppStatus.h 用到的接口）
// 记录最近一次 setPixelColor 的像素值与 show 调用次数，供测试断言。
// 与 test_host/stub/Arduino.h 同模式：仅在 -Itest_host/stub 优先命中，
// 不影响 PlatformIO 固件编译（固件使用 lib/WS2812Driver 真实驱动）。
#ifndef TEST_HOST_WS2812_STUB_H
#define TEST_HOST_WS2812_STUB_H

#include <cstdint>

class WS2812Driver {
public:
    enum ColorOrder { ORDER_GRB };

    // 记录最近一次 setPixelColor 的像素值 / show 次数（测试断言用）
    static uint8_t last_r, last_g, last_b;
    static int show_count;

    WS2812Driver(uint32_t pin, uint16_t numPixels, ColorOrder order = ORDER_GRB)
        : _numPixels(numPixels) { (void)pin; (void)order; }

    void setPixelColor(uint16_t i, uint8_t r, uint8_t g, uint8_t b) {
        (void)i;
        last_r = r; last_g = g; last_b = b;
    }
    void show() { ++show_count; }
    void clear() { last_r = last_g = last_b = 0; }
    uint16_t numPixels() const { return _numPixels; }

    // 重置记录（每个测试用例前调用）
    static void resetLog() { last_r = last_g = last_b = 0; show_count = 0; }

private:
    uint16_t _numPixels;
};

#endif // TEST_HOST_WS2812_STUB_H
