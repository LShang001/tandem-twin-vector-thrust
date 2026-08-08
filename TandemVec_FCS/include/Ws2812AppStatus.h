/*
 * Ws2812AppStatus.h - 飞控应用层状态机（依赖 WS2812Driver 通用库）
 *
 * 与 StatusLED (PE5 单色绿灯) 并行运行，提供 RGB 状态指示。
 * 2026-08-08 扩展：FcsLedState 模式状态机覆盖——
 *   BOOT 上电自检 / FAULT 故障 / FAILSAFE 空中失控 / CALIBRATING 校准 /
 *   NO_RECEIVER 上电无信号 / LINK_DOWN 地面断链 / LOW_LQ 弱信号 /
 *   LOW_VOLTAGE 低压 / ARMED_* 解锁按飞行模式呼吸 / STANDBY 待机
 *
 * 设计：本类只负责"效果渲染"（状态 → 颜色/节奏），状态判定在
 * communication.cpp::handleStatusLedTask 组装（业务优先级）。
 * 像素驱动由通用库 WS2812Driver 负责（bitbang / H7 DMA 双方案）。
 */

#ifndef WS2812_APP_STATUS_H
#define WS2812_APP_STATUS_H

#include <Arduino.h>
#include <WS2812Driver.h>

/**
 * @brief 飞控状态 → LED 效果枚举（按业务优先级由 handleStatusLedTask 组装）
 *
 * 优先级（高→低）：
 *   FAULT > FAILSAFE > CALIBRATING > NO_RECEIVER > LINK_DOWN
 *   > LOW_LQ > LOW_VOLTAGE > ARMED_*(模式) > STANDBY
 */
enum class FcsLedState {
    BOOT,          // 上电自检：白→蓝→绿 渐变 1.2s（首次 update 自动播放）
    FAULT,         // 故障：红快闪 10Hz（setFault 触发）
    FAILSAFE,      // 空中失控：红 SOS 双闪（100亮/100灭/100亮/600灭 ms）
    CALIBRATING,   // IMU 校准：黄快闪 10Hz
    NO_RECEIVER,   // 上电无信号（从未建链）：红慢闪 1Hz
    LINK_DOWN,     // 地面断链（曾建链）：品红慢闪 ~0.8Hz
    LOW_LQ,        // 弱信号（LQ<40%）：橙慢闪 1Hz
    LOW_VOLTAGE,   // 低压：红慢呼吸 2s
    ARMED_MANUAL,  // 解锁+手动模式：绿呼吸 2s
    ARMED_ALT,     // 解锁+高度保持：青呼吸 2s
    ARMED_POS,     // 解锁+定点：白呼吸 2s
    ARMED_GUIDED,  // 解锁+制导（上位机）：紫呼吸 2s
    STANDBY        // 待机（链路正常未解锁）：蓝呼吸 2.5s
};

class Ws2812AppStatus {
private:
    WS2812Driver &_strip;    // 引用通用驱动（由外部持有）

    // 调试故障标志（wsfault 命令触发，优先级最高，覆盖组装的状态）
    bool _fault;

    // BOOT 序列状态（一次性，上电首次 update 自动播放）
    unsigned long _boot_remaining_ms;  // 剩余时长；0 = 已播完
    unsigned long _boot_start_ms;      // BOOT 起点（绝对时间，防任务抖动漂移）
    bool _boot_started;

    // ===== 状态颜色定义 (R, G, B) =====
    static const uint8_t RED_R   = 255, RED_G   = 0,   RED_B   = 0;    // 红：故障/失控/无信号/低压
    static const uint8_t YEL_R   = 255, YEL_G   = 255, YEL_B   = 0;    // 黄：校准
    static const uint8_t ORG_R   = 255, ORG_G   = 120, ORG_B   = 0;    // 橙：弱信号
    static const uint8_t GRN_R   = 0,   GRN_G   = 255, GRN_B   = 0;    // 绿：手动/上电自检
    static const uint8_t CYN_R   = 0,   CYN_G   = 255, CYN_B   = 255;  // 青：高度保持
    static const uint8_t WHT_R   = 255, WHT_G   = 255, WHT_B   = 255;  // 白：定点
    static const uint8_t PPL_R   = 170, PPL_G   = 0,   PPL_B   = 255;  // 紫：制导
    static const uint8_t MAG_R   = 255, MAG_G   = 0,   MAG_B   = 180;  // 品红：地面断链
    static const uint8_t BLU_R   = 0,   BLU_G   = 0,   BLU_B   = 255;  // 蓝：待机

public:
    // 构造函数：绑定驱动引用（单颗 LED，像素 0）；BOOT 序列 1.2s
    Ws2812AppStatus(WS2812Driver &strip)
        : _strip(strip), _fault(false),
          _boot_remaining_ms(BOOT_DURATION_MS), _boot_start_ms(0), _boot_started(false) {}

    // 设置故障状态（调试命令 wsfault；优先级最高，覆盖一切）
    void setFault(bool fault) { _fault = fault; }

    /**
     * @brief 更新LED状态（每 10ms 调用一次）
     * @param state 业务状态（handleStatusLedTask 按优先级组装）
     * @param now_ms 当前毫秒时间戳（单测可注入，防任务抖动漂移）
     *
     * BOOT 序列在首次调用时自动播放并屏蔽正常状态机；
     * setFault 覆盖 state（调试命令语义）。
     */
    void update(FcsLedState state, unsigned long now_ms) {
        // --- 0. 上电自检序列（一次性 1.2s） ---
        if (_boot_remaining_ms > 0)
        {
            if (!_boot_started)
            {
                _boot_started = true;
                _boot_start_ms = now_ms;
            }
            renderBoot(now_ms - _boot_start_ms);
            if (now_ms - _boot_start_ms >= BOOT_DURATION_MS)
            {
                _boot_remaining_ms = 0;   // 播完，进入正常状态机
                _boot_started = false;
            }
            return;
        }

        // --- 1. 调试故障覆盖 ---
        if (_fault)
        {
            state = FcsLedState::FAULT;
        }

        // --- 2. 按状态渲染对应效果 ---
        switch (state)
        {
            case FcsLedState::FAULT:
                renderBlinking(RED_R, RED_G, RED_B, now_ms, 50);      // 红快闪 10Hz
                break;
            case FcsLedState::FAILSAFE:
                renderSosBlink(now_ms);                                // 红 SOS 双闪
                break;
            case FcsLedState::CALIBRATING:
                renderBlinking(YEL_R, YEL_G, YEL_B, now_ms, 50);      // 黄快闪 10Hz
                break;
            case FcsLedState::NO_RECEIVER:
                renderBlinking(RED_R, RED_G, RED_B, now_ms, 500);     // 红慢闪 1Hz
                break;
            case FcsLedState::LINK_DOWN:
                renderBlinking(MAG_R, MAG_G, MAG_B, now_ms, 600);     // 品红慢闪
                break;
            case FcsLedState::LOW_LQ:
                renderBlinking(ORG_R, ORG_G, ORG_B, now_ms, 500);     // 橙慢闪 1Hz
                break;
            case FcsLedState::LOW_VOLTAGE:
                renderBreathing(RED_R, RED_G, RED_B, now_ms, 2000, 0.40f);  // 红慢呼吸
                break;
            case FcsLedState::ARMED_MANUAL:
                renderBreathing(GRN_R, GRN_G, GRN_B, now_ms, 2000, 0.50f);  // 绿呼吸
                break;
            case FcsLedState::ARMED_ALT:
                renderBreathing(CYN_R, CYN_G, CYN_B, now_ms, 2000, 0.50f);  // 青呼吸
                break;
            case FcsLedState::ARMED_POS:
                renderBreathing(WHT_R, WHT_G, WHT_B, now_ms, 2000, 0.50f);  // 白呼吸
                break;
            case FcsLedState::ARMED_GUIDED:
                renderBreathing(PPL_R, PPL_G, PPL_B, now_ms, 2000, 0.50f);  // 紫呼吸
                break;
            case FcsLedState::STANDBY:
            default:
                renderBreathing(BLU_R, BLU_G, BLU_B, now_ms, 2500, 0.35f);  // 蓝呼吸
                break;
        }
    }

private:
    static const unsigned long BOOT_DURATION_MS = 1200;   // 上电自检总时长
    static const unsigned long BOOT_SEGMENT_MS  = 400;    // 单色段时长（白→蓝→绿）

    // 处理呼吸效果（正弦波 gamma 2.0；参数化颜色/周期/亮度上限）
    void renderBreathing(uint8_t r, uint8_t g, uint8_t b,
                         unsigned long t, unsigned long period, float max_brightness) {
        float phase = (t % period) / (float)period * 2 * PI;
        float brightness = pow((sin(phase) + 1.0f) / 2.0f, 2.0f);
        brightness *= max_brightness;

        _strip.setPixelColor(0, (uint8_t)(r * brightness),
                                (uint8_t)(g * brightness),
                                (uint8_t)(b * brightness));
        _strip.show();
    }

    // 处理闪烁效果（方波：亮/灭各 half_period；亮度减半防刺眼）
    void renderBlinking(uint8_t r, uint8_t g, uint8_t b,
                        unsigned long t, unsigned long half_period) {
        if ((t / half_period) % 2 == 0)
        {
            _strip.setPixelColor(0, r >> 1, g >> 1, b >> 1);   // 亮
        }
        else
        {
            _strip.setPixelColor(0, 0, 0, 0);                  // 灭
        }
        _strip.show();
    }

    // 空中失控 SOS 双闪：亮100/灭100/亮100/灭600 ms（周期 900ms，全亮突出紧急）
    void renderSosBlink(unsigned long t) {
        const unsigned long CYCLE_MS = 900;
        unsigned long phase = t % CYCLE_MS;
        bool on = (phase < 100) || (phase >= 200 && phase < 300);
        _strip.setPixelColor(0, on ? RED_R : 0, on ? RED_G : 0, on ? RED_B : 0);
        _strip.show();
    }

    // 上电自检：白→蓝→绿 三段，每段 400ms 内呼吸一次（确认驱动与像素正常）
    void renderBoot(unsigned long t) {
        if (t >= BOOT_DURATION_MS)
        {
            _strip.setPixelColor(0, 0, 0, 0);
            _strip.show();
            return;
        }
        unsigned seg = t / BOOT_SEGMENT_MS;          // 0 白 / 1 蓝 / 2 绿
        unsigned long seg_t = t % BOOT_SEGMENT_MS;   // 段内时间

        float phase = (seg_t / (float)BOOT_SEGMENT_MS) * 2 * PI;
        float br = pow((sin(phase) + 1.0f) / 2.0f, 2.0f);
        br *= 0.5f;  // 50% 亮度上限

        switch (seg)
        {
            case 0:  _strip.setPixelColor(0, (uint8_t)(WHT_R * br), (uint8_t)(WHT_G * br), (uint8_t)(WHT_B * br)); break;
            case 1:  _strip.setPixelColor(0, (uint8_t)(BLU_R * br), (uint8_t)(BLU_G * br), (uint8_t)(BLU_B * br)); break;
            default: _strip.setPixelColor(0, (uint8_t)(GRN_R * br), (uint8_t)(GRN_G * br), (uint8_t)(GRN_B * br)); break;
        }
        _strip.show();
    }
};

#endif
