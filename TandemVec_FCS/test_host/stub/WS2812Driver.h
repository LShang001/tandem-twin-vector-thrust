/**
 * @file    WS2812Driver.h
 * @module  固件/宿主回归桩（fw-tests：WS2812 灯带驱动替身）
 * @brief   记录最近一次像素值与 show() 次数，供灯效状态机测试断言
 * @details 被测：无（替身层）。只覆盖 Ws2812AppStatus.h 用到的接口：
 *          setPixelColor / show / clear / numPixels。
 *          输入：灯效状态机下发的 RGB；处理：写入静态记录字段；
 *          输出：通过 `last_r/last_g/last_b`、`show_count` 暴露给测试。
 * @theory  N/A（外设替身）
 * @frame   N/A
 * @units   RGB 分量 0–255 整数，无物理单位
 * @note    与真实驱动的差异：无 BSRR/DMA 时序、无颜色顺序换算、无视窗刷新；
 *          因此本桩只能验证“状态机想显示什么颜色”，不能验证灯带电气时序。
 * @see     TandemVec_FCS/test_host/test_led_status.cpp、include/Ws2812AppStatus.h
 */
#ifndef TEST_HOST_WS2812_STUB_H
#define TEST_HOST_WS2812_STUB_H
// 宿主机回归用 mock：只记录像素值与 show 调用次数，不驱动真实灯带。
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
