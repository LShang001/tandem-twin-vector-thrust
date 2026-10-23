/**
 * @file    StatusLED.h
 * @module  固件/基础设施（单色状态灯驱动，PE5 绿灯）
 * @brief   封装单色 LED 的呼吸/慢闪/快闪三种效果，供灯效任务调用
 * @details 输入：点灯模式（待机呼吸 / 解锁慢闪 1Hz / 校准快闪 10Hz）；
 *          处理：模式 → 相位累加 → 占空比；
 *          输出：GPIO 电平（PE5）。
 *          不变量：非阻塞——闪烁全部用相位累加实现，任务内禁止 delay()。
 * @theory  N/A（外设驱动）
 * @ref     docs/电路拓扑参考.md（指示灯引脚）、AGENTS.md 踩坑记录（上电无信号需门控）
 * @frame   N/A
 * @units   闪烁频率 Hz；相位为调用次数计数（无物理单位）
 * @note    与 RGB 状态灯（Ws2812AppStatus，PD15）并行：单色灯只表“运行/校准/故障”，
 *          细节状态由 WS2812 呈现；优先级判定在 communication.cpp handleStatusLedTask。
 * @see     include/Ws2812AppStatus.h、src/communication.cpp（handleStatusLedTask）
 */

#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <Arduino.h>

class StatusLED {
private:
    int _pin;
    
    // 定义闪烁模式
    enum Mode {
        MODE_BREATHING,   // 呼吸 (待机)
        MODE_BLINK_SLOW,  // 慢闪 (解锁)
        MODE_BLINK_FAST   // 快闪 (校准)
    } _current_mode;

    // PWM 分辨率 (对应主程序中的设置)
    const int _pwm_max = 65535; 

public:
    // 构造函数
    StatusLED(int pin) : _pin(pin), _current_mode(MODE_BREATHING) {}

    // 初始化
    void begin() {
        pinMode(_pin, OUTPUT);
    }

    /**
     * @brief 更新LED状态
     * @param is_calibrating 是否正在校准IMU
     * @param is_armed       是否已解锁
     */
    void update(bool is_calibrating, bool is_armed) {
        // --- 1. 判定当前模式优先级 ---
        // 优先级：校准 > 解锁 > 待机
        if (is_calibrating) {
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
                handleBlinking(current_time, 200); // 200ms 翻转一次 = 5Hz
                break;
            case MODE_BLINK_FAST:
                handleBlinking(current_time, 50);  // 50ms 翻转一次 = 10Hz
                break;
        }
    }

private:
    // 处理呼吸灯效果 (使用正弦波模拟)
    void handleBreathing(unsigned long t) {
        // 呼吸周期约 2.5秒
        const float period = 2500.0f; 
        // 计算相位 (0 ~ 2PI)
        float phase = (t % (int)period) / period * 2 * PI;
        
        // 计算亮度: (sin(x) + 1) / 2 将范围映射到 0~1
        // 使用 pow(..., 2.0) 让暗部停留时间更长，呼吸感更真实
        float brightness = pow((sin(phase) + 1.0f) / 2.0f, 2.0f);
        
        // 输出 PWM
        int pwm_val = (int)(brightness * _pwm_max);
        analogWrite(_pin, pwm_val);
    }

    // 处理闪烁效果
    void handleBlinking(unsigned long t, int interval) {
        // 简单的方波逻辑
        if ((t / interval) % 2 == 0) {
            // 亮 (全亮度)
            analogWrite(_pin, _pwm_max); 
            // 如果引脚不支持PWM，可以用 digitalWrite(_pin, HIGH);
        } else {
            // 灭
            analogWrite(_pin, 0);
            // 如果引脚不支持PWM，可以用 digitalWrite(_pin, LOW);
        }
    }
};

#endif
