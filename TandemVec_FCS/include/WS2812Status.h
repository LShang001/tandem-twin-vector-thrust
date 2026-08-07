/*
 * WS2812Status.h - RGB 状态指示灯控制模块 (WS2812B-Mini, PD15)
 *
 * 与 StatusLED (PE5 单色绿灯) 并行运行，提供 RGB 增强状态指示：
 * 1. 待机/锁定 (Standby):   蓝色呼吸灯
 * 2. 解锁/飞行 (Armed):      绿色慢闪 (5Hz)
 * 3. 校准状态 (Calibrating): 黄色快闪 (10Hz)
 * 4. 故障/低压 (Fault):      红色快闪 (10Hz) — 通过 setFault() 触发
 *
 * 驱动方式：自实现 DWT 周期计数器 bitbang（不依赖 Adafruit NeoPixel 库的 show()）。
 *
 * 为什么不用 Adafruit NeoPixel 库的 show()：
 *   库的 __arm__ bitbang 路径用 volatile uint8_t* 指向 stm32duino 的 BSRR 寄存器，
 *   *set=1 只写 BSRR 低8位 → 只能操作 GPIO 0-7，PD15 (bit15) 够不着。
 *   这是 NeoPixel 1.15.x 在 stm32duino 上对高位引脚的已知限制。
 *   本文件自实现 show()，用正确的 BSRR 位掩码 (1<<15) 直接写 32 位寄存器。
 *
 * 时序：STM32H743 @ 480MHz，DWT 分辨率 2.08ns。
 *   单颗 LED 刷新 = 24bit × 1.25µs = 30µs，show() 期间 noInterrupts()。
 *   2kHz 调度中断极轻量(遍历任务表置flag ≈ 1-2µs)，30µs 抖动可吸收。
 *
 * 硬件背景：PD15 = TIM4_CH4，H743 上该通道无 DMA 请求，
 * 故放弃 TIM PWM+DMA，改用 DWT bitbang（详见 docs/memory WS2812 调研记录）。
 */

#ifndef WS2812_STATUS_H
#define WS2812_STATUS_H

#include <Arduino.h>

// WS2812 时序参数（480MHz Cortex-M7，3-cycle 循环校准）
// WS2812 800kHz: 每位 1.25µs, T0H≈0.4µs, T1H≈0.8µs

class WS2812Status {
private:
    int _pin;
    PinName _pinName;
    GPIO_TypeDef *_port;
    uint32_t _setMask;   // BSRR 置位掩码 (1 << pin_bit)
    uint32_t _resetMask; // BSRR 清零掩码 (1 << (pin_bit + 16))

    // GRB 像素缓存（单颗 = 3 字节）
    uint8_t _pixels[3];

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
    // 构造函数
    WS2812Status(int pin) : _pin(pin), _pinName(digitalPinToPinName(pin)) {
        _port = get_GPIO_Port(STM_PORT(_pinName));
        uint32_t pinBit = STM_GPIO_PIN(_pinName);  // 0-15
        _setMask = 1UL << pinBit;           // BSRR 低16位 = SET
        _resetMask = 1UL << (pinBit + 16);  // BSRR 高16位 = RESET
        memset(_pixels, 0, sizeof(_pixels));
    }

    // 初始化
    void begin() {
        pinMode(_pin, OUTPUT);
        digitalWrite(_pin, LOW);
        show();      // 初始全灭
    }

    // 设置故障状态（外部调用，优先级高于校准/解锁）
    void setFault(bool fault) { _fault = fault; }

    /**
     * @brief 更新LED状态
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
                handleBlinking(current_time, 100); // 5Hz
                break;
            case MODE_BLINK_FAST:
                handleBlinking(current_time, 50);  // 10Hz
                break;
        }
    }

private:
    // 呼吸灯效果（正弦波 gamma 2.0，与 StatusLED 一致）
    void handleBreathing(unsigned long t) {
        const float period = 2500.0f;
        float phase = (t % (int)period) / period * 2 * PI;
        float brightness = pow((sin(phase) + 1.0f) / 2.0f, 2.0f);

        // 按亮度缩放蓝色（限制最大亮度避免刺眼）
        brightness *= 0.35f;  // 35% 上限
        uint8_t b = (uint8_t)(COLOR_STANDBY_B * brightness);
        setColor(0, 0, b);
    }

    // 闪烁效果
    void handleBlinking(unsigned long t, int interval) {
        uint8_t r, g, b;
        if (_fault) {
            r = COLOR_FAULT_R; g = COLOR_FAULT_G; b = COLOR_FAULT_B;
        } else if (_current_mode == MODE_BLINK_SLOW) {
            r = COLOR_ARMED_R; g = COLOR_ARMED_G; b = COLOR_ARMED_B;
        } else {
            r = COLOR_CALIB_R; g = COLOR_CALIB_G; b = COLOR_CALIB_B;
        }

        // 降低亮度（板载灯避免刺眼）
        r >>= 1; g >>= 1; b >>= 1;

        if ((t / interval) % 2 == 0) {
            setColor(r, g, b);   // 亮
        } else {
            setColor(0, 0, 0);   // 灭
        }
    }

    // 设置 GRB 像素并刷新
    void setColor(uint8_t r, uint8_t g, uint8_t b) {
        _pixels[0] = g;  // WS2812 像素顺序: G-R-B
        _pixels[1] = r;
        _pixels[2] = b;
        show();
    }

    // --- 核心驱动：NOP 循环 bitbang（不依赖 DWT，纯指令周期，最可靠）---
    //
    // 为什么不用 DWT CYCCNT：
    //   -O3 下编译器对 dwt_getCycles() 内联后，关中断 + 调试 halt 场景下
    //   CYCCNT 计数行为不稳定，导致位等待循环死锁。
    //   NOP 循环完全靠 CPU 流水线周期，无外设依赖。
    //
    // Cortex-M7 @ 480MHz 时序参数（每 NOP 约 1 cycle，循环开销已计入）：
    //   经校准的延时循环宏，每个循环体 ~3 cycle（subs + bne + nop）

    // 精确延时：参数为"3-cycle 循环圈数"
    static inline void __attribute__((always_inline))
    delayLoops(uint32_t loops) {
        __asm volatile(
            "1: subs %0, %0, #1 \n"
            "   bne 1b          \n"
            : "+r"(loops) : : "cc"
        );
    }

    void show() {
        noInterrupts();

        // 复位：拉低 >50µs（50µs × 480MHz / 3 ≈ 8000 圈）
        _port->BSRR = _resetMask;
        delayLoops(8000);

        // 逐位发送（MSB first），3 字节 GRB
        for (uint8_t i = 0; i < 3; i++) {
            uint8_t pix = _pixels[i];
            for (uint8_t mask = 0x80; mask; mask >>= 1) {
                // 拉高
                _port->BSRR = _setMask;

                // 保持高电平：T1H≈0.8µs 或 T0H≈0.4µs
                // 480MHz / 3 cycle/loop ≈ 160 loops/µs
                if (pix & mask) {
                    delayLoops(110);  // ~0.69µs T1H（扣 BSRR 写入开销）
                } else {
                    delayLoops(45);   // ~0.28µs T0H
                }

                // 拉低
                _port->BSRR = _resetMask;

                // 位剩余低电平时间：1.25µs - T_H
                if (pix & mask) {
                    delayLoops(55);   // ~0.34µs T1L
                } else {
                    delayLoops(120);  // ~0.75µs T0L
                }
            }
        }

        // 帧结束复位
        _port->BSRR = _resetMask;
        interrupts();
    }
};

#endif
