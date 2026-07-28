/*
 * StatusLED.h - 智能状态指示灯控制模块
 * 
 * 功能：
 * 根据无人机当前状态控制 LED 的闪烁模式：
 * 1. 待机/锁定状态 (Standby): 呼吸灯效果 (Breathing)
 * 2. 解锁/飞行状态 (Armed): 慢闪 (1Hz)
 * 3. 校准状态 (Calibrating): 快闪 (10Hz)
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
