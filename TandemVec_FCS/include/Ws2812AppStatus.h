/*
 * Ws2812AppStatus.h - 飞控应用层状态机（依赖 WS2812Driver 通用库）
 *
 * 与 StatusLED (PE5 单色绿灯) 并行运行，提供 RGB 状态指示：
 * 1. 待机/锁定 (Standby):   蓝色呼吸灯
 * 2. 解锁/飞行 (Armed):      绿色慢闪 (5Hz)
 * 3. 校准状态 (Calibrating): 黄色快闪 (10Hz)
 * 4. 故障/低压 (Fault):      红色快闪 (10Hz) — 通过 setFault() 触发
 *
 * 设计：本类只包含"业务状态机"（什么状态显示什么颜色/效果），
 * 像素驱动由通用库 WS2812Driver 负责（bitbang / H7 DMA 双方案）。
 * 状态机逻辑继承自原 WS2812Status.h（2026-08-08 库化拆分）。
 */

#ifndef WS2812_APP_STATUS_H
#define WS2812_APP_STATUS_H

#include <Arduino.h>
#include <WS2812Driver.h>

class Ws2812AppStatus {
private:
    WS2812Driver &_strip;    // 引用通用驱动（由外部持有）

    // 闪烁模式定义（与 StatusLED 一致：校准 > 解锁 > 待机）
    enum Mode {
        MODE_BREATHING,   // 呼吸 (待机)   — 蓝色
        MODE_BLINK_SLOW,  // 慢闪 (解锁)   — 绿色 5Hz
        MODE_BLINK_FAST   // 快闪 (校准/故障) — 黄/红色 10Hz
    } _current_mode;

    bool _fault;

    // 状态颜色定义 (R, G, B)
    static const uint8_t COLOR_STANDBY_R = 0,   COLOR_STANDBY_G = 0,   COLOR_STANDBY_B = 255;   // 蓝
    static const uint8_t COLOR_ARMED_R   = 0,   COLOR_ARMED_G   = 255, COLOR_ARMED_B   = 0;     // 绿
    static const uint8_t COLOR_CALIB_R   = 255, COLOR_CALIB_G   = 255, COLOR_CALIB_B   = 0;     // 黄
    static const uint8_t COLOR_FAULT_R   = 255, COLOR_FAULT_G   = 0,   COLOR_FAULT_B   = 0;     // 红

public:
    // 构造函数：绑定驱动引用（单颗 LED，像素 0）
    Ws2812AppStatus(WS2812Driver &strip)
        : _strip(strip), _current_mode(MODE_BREATHING), _fault(false) {}

    // 设置故障状态（外部调用，优先级高于校准/解锁）
    void setFault(bool fault) { _fault = fault; }

    /**
     * @brief 更新LED状态（每 10ms 调用一次）
     * @param is_calibrating 是否正在校准IMU
     * @param is_armed       是否已解锁
     *
     * 优先级：故障 > 校准 > 解锁 > 待机
     */
    void update(bool is_calibrating, bool is_armed) {
        // --- 1. 判定当前模式优先级 ---
        if (_fault) {
            _current_mode = MODE_BLINK_FAST;
        } else if (is_calibrating) {
            _current_mode = MODE_BLINK_FAST;
        } else if (is_armed) {
            _current_mode = MODE_BLINK_SLOW;
        } else {
            _current_mode = MODE_BREATHING;
        }

        // --- 2. 执行对应模式的灯光效果 ---
        unsigned long current_time = millis();

        switch (_current_mode) {
            case MODE_BREATHING:
                handleBreathing(current_time);
                break;
            case MODE_BLINK_SLOW:
                handleBlinking(current_time, 100); // 100ms 翻转 = 5Hz
                break;
            case MODE_BLINK_FAST:
                handleBlinking(current_time, 50);  // 50ms 翻转 = 10Hz
                break;
        }
    }

private:
    // 处理呼吸灯效果（正弦波 gamma 2.0，与 StatusLED 一致）
    void handleBreathing(unsigned long t) {
        const float period = 2500.0f;  // 呼吸周期 2.5s
        float phase = (t % (int)period) / period * 2 * PI;
        float brightness = pow((sin(phase) + 1.0f) / 2.0f, 2.0f);
        brightness *= 0.35f;  // 35% 亮度上限（板载灯避免刺眼）

        uint8_t b = (uint8_t)(COLOR_STANDBY_B * brightness);
        _strip.setPixelColor(0, 0, 0, b);
        _strip.show();
    }

    // 处理闪烁效果（按当前模式选取颜色）
    void handleBlinking(unsigned long t, int interval) {
        uint8_t r, g, b;
        if (_fault) {
            r = COLOR_FAULT_R; g = COLOR_FAULT_G; b = COLOR_FAULT_B;
        } else if (_current_mode == MODE_BLINK_SLOW) {
            r = COLOR_ARMED_R; g = COLOR_ARMED_G; b = COLOR_ARMED_B;
        } else {
            r = COLOR_CALIB_R; g = COLOR_CALIB_G; b = COLOR_CALIB_B;
        }
        r >>= 1; g >>= 1; b >>= 1;  // 降亮度

        if ((t / interval) % 2 == 0) {
            _strip.setPixelColor(0, r, g, b);   // 亮
        } else {
            _strip.setPixelColor(0, 0, 0, 0);   // 灭
        }
        _strip.show();
    }
};

#endif
