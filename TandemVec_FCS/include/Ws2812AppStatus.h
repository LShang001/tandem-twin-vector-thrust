/**
 * @file    Ws2812AppStatus.h
 * @module  固件/基础设施（PD15 RGB 状态灯）
 * @brief   选择状态灯主色与提示，按切入时间渲染并提交变化帧。
 * @details 输入为已生效的飞行模式色、链路/导航标志和电池采样；渲染层独立计时，
 *          发送失败后保留重试机会。所有判定仅供显示。
 * @theory  N/A（状态显示）
 * @ref     docs/registers/固件调试工具链-2026-11-01.md
 * @frame   N/A
 * @units   时间 ms、电压 mV、RGB 0–255
 * @see     src/communication.cpp::handleStatusLedTask
 */
#pragma once

#include <Arduino.h>
#include <WS2812Driver.h>
#include <cmath>
#include <cstdint>

enum class FcsLedState {
    BOOT, FAULT, FAILSAFE, CALIBRATING, NO_RECEIVER, LINK_DOWN, LOW_LQ,
    LOW_VOLTAGE, BAT_CRITICAL, EKF_NOT_CONVERGED, GNSS_NO_FIX,
    ARMED_MANUAL, ARMED_ALT, ARMED_POS, ARMED_GUIDED, STANDBY, NONE
};

/// 稳定的 DBG 名称，主状态与普通提示共用。
inline const char *fcsLedStateName(FcsLedState state) {
    switch (state) {
        case FcsLedState::BOOT: return "BOOT";
        case FcsLedState::FAULT: return "FAULT";
        case FcsLedState::FAILSAFE: return "FAILSAFE";
        case FcsLedState::CALIBRATING: return "CALIBRATING";
        case FcsLedState::NO_RECEIVER: return "NO_RECEIVER";
        case FcsLedState::LINK_DOWN: return "LINK_DOWN";
        case FcsLedState::LOW_LQ: return "LOW_LQ";
        case FcsLedState::LOW_VOLTAGE: return "LOW_VOLTAGE";
        case FcsLedState::BAT_CRITICAL: return "BAT_CRITICAL";
        case FcsLedState::EKF_NOT_CONVERGED: return "EKF_NOT_CONVERGED";
        case FcsLedState::GNSS_NO_FIX: return "GNSS_NO_FIX";
        case FcsLedState::ARMED_MANUAL: return "ARMED_MANUAL";
        case FcsLedState::ARMED_ALT: return "ARMED_ALT";
        case FcsLedState::ARMED_POS: return "ARMED_POS";
        case FcsLedState::ARMED_GUIDED: return "ARMED_GUIDED";
        case FcsLedState::STANDBY: return "STANDBY";
        default: return "NONE";
    }
}

/// 单灯显示同时携带主状态和可短时叠加的提示。
struct FcsLedIndication {
    FcsLedState primary = FcsLedState::STANDBY;
    FcsLedState advisory = FcsLedState::NONE;
};

/// 显示输入由通信任务在同一拍采集，电池与 LQ 的有效性各自判断。
struct FcsLedInputs {
    bool armed = false;
    bool calibrating = false;
    bool link_up = false;
    bool ever_linked = false;
    bool failsafe = false;
    bool crsf_link_stats_seen = false;
    bool nav_ready_on_ground = false;
    bool gnss_fix = false;
    uint8_t link_quality = 100;
    uint32_t link_stats_ms = 0;
    float battery_mv = 0.0f;
    FcsLedState actual_mode = FcsLedState::ARMED_MANUAL;
};

class FcsLedPolicy {
public:
    static constexpr uint32_t kLinkStatsMaxAgeMs = 1000;
    static constexpr unsigned kLowMvPerCell = 3700;
    static constexpr unsigned kCriticalMvPerCell = 3500;
    static constexpr unsigned kRecoveryMvPerCell = 100;

    /// 串数由机型构建提供，0 表示没有可用的电池告警配置。
    explicit FcsLedPolicy(unsigned battery_cells) : _battery_cells(battery_cells) {}

    /// 处理电池迟滞并按现场优先级输出主状态，再挑选一个普通提示。
    FcsLedIndication select(const FcsLedInputs &in, uint32_t now_ms) {
        updateBattery(in.battery_mv);
        if (in.failsafe) return {FcsLedState::FAILSAFE};
        if (_battery == FcsLedState::BAT_CRITICAL) return {_battery};
        if (in.calibrating) return {FcsLedState::CALIBRATING};
        if (!in.link_up) {
            return {in.ever_linked ? FcsLedState::LINK_DOWN : FcsLedState::NO_RECEIVER};
        }

        FcsLedState advisory = _battery;
        if (in.crsf_link_stats_seen && now_ms - in.link_stats_ms <= kLinkStatsMaxAgeMs &&
            in.link_quality < 40 && advisory == FcsLedState::NONE) {
            advisory = FcsLedState::LOW_LQ;
        }
        if (advisory == FcsLedState::NONE && !in.armed && !in.nav_ready_on_ground) {
            advisory = FcsLedState::EKF_NOT_CONVERGED;
        }
        if (advisory == FcsLedState::NONE && !in.gnss_fix) {
            advisory = FcsLedState::GNSS_NO_FIX;
        }
        if (in.armed) return {in.actual_mode, advisory};
        return {advisory == FcsLedState::NONE ? FcsLedState::STANDBY : advisory};
    }

    FcsLedState batteryState() const { return _battery; }

private:
    unsigned _battery_cells;
    FcsLedState _battery = FcsLedState::NONE;

    /// 进入阈值立即生效，恢复多留每芯 100 mV，跨级恢复仍按低压阈值判定。
    void updateBattery(float mv) {
        if (_battery_cells == 0 || !std::isfinite(mv) || mv <= 0.0f) {
            _battery = FcsLedState::NONE;
            return;
        }
        const float critical = kCriticalMvPerCell * _battery_cells;
        const float low = kLowMvPerCell * _battery_cells;
        const float hysteresis = kRecoveryMvPerCell * _battery_cells;
        if (mv < critical || (_battery == FcsLedState::BAT_CRITICAL && mv < critical + hysteresis)) {
            _battery = FcsLedState::BAT_CRITICAL;
        } else if (mv < low || (_battery != FcsLedState::NONE && mv < low + hysteresis)) {
            _battery = FcsLedState::LOW_VOLTAGE;
        } else {
            _battery = FcsLedState::NONE;
        }
    }
};

class Ws2812AppStatus {
public:
    explicit Ws2812AppStatus(WS2812Driver &strip) : _strip(strip) {}

    /// 首拍采用新状态的相位零点。紧急告警、校准和已解锁模式可结束上电演示。
    void update(FcsLedIndication indication, uint32_t now_ms) {
        const FcsLedState state = indication.primary;
        if (_boot_pending) {
            if (!_boot_started) {
                _boot_started = true;
                _boot_start_ms = now_ms;
            }
            const uint32_t elapsed = now_ms - _boot_start_ms;
            if (elapsed < kBootMs && !interruptsBoot(state)) {
                const uint32_t section = elapsed / kBootSegmentMs;
                const uint32_t color = section == 0 ? kWhite : (section == 1 ? kBlue : kGreen);
                _rendered = FcsLedState::BOOT;
                present(scale(color, bootBreath(elapsed % kBootSegmentMs)));
                return;
            }
            _boot_pending = false;
        }

        if (state != _primary) {
            _primary = state;
            _primary_start_ms = now_ms;
        }
        if (indication.advisory != _advisory) {
            _advisory = indication.advisory;
            _advisory_start_ms = now_ms;
        }
        const uint32_t phase = now_ms - _primary_start_ms;
        _rendered = _primary;
        if (isArmedMode(_primary) && _advisory != FcsLedState::NONE &&
            (now_ms - _advisory_start_ms) % 2000U < 150U) {
            _rendered = _advisory;
            present(_advisory == FcsLedState::GNSS_NO_FIX ? kBlue : scale(colorFor(_advisory), 0.5f));
            return;
        }
        present(render(_primary, phase));
    }

    /// 调试和自动灯效共用同一提交缓存，失败后下一拍重试当前目标。
    bool present(uint32_t rgb) {
        if (_sent_valid && _last_rgb == rgb) return true;
        _strip.setPixelColor(0, static_cast<uint8_t>(rgb >> 16),
                               static_cast<uint8_t>(rgb >> 8), static_cast<uint8_t>(rgb));
        _sent_valid = _strip.show();
        if (_sent_valid) _last_rgb = rgb;
        return _sent_valid;
    }

    /// GPIO 或驱动模式改变后重发一次，保证恢复自动灯效时不会跳过同色帧。
    void invalidateFrame() { _sent_valid = false; }
    FcsLedState renderedState() const { return _rendered; }
    uint32_t lastRgb() const { return _last_rgb; }
    bool frameValid() const { return _sent_valid; }

    /// 高优先级状态可抢占调试输出。
    static bool isEmergency(FcsLedState state) {
        return state == FcsLedState::FAILSAFE || state == FcsLedState::BAT_CRITICAL;
    }

private:
    static constexpr uint32_t kBootMs = 1200, kBootSegmentMs = 400;
    static constexpr uint32_t kRed = 0xFF0000, kYellow = 0xFFFF00, kOrange = 0xFF7800;
    static constexpr uint32_t kGreen = 0x00FF00, kCyan = 0x00FFFF, kWhite = 0xFFFFFF;
    static constexpr uint32_t kPurple = 0xAA00FF, kMagenta = 0xFF00B4, kBlue = 0x0000FF;
    WS2812Driver &_strip;
    bool _boot_pending = true, _boot_started = false, _sent_valid = false;
    uint32_t _boot_start_ms = 0, _primary_start_ms = 0, _advisory_start_ms = 0;
    uint32_t _last_rgb = 0;
    FcsLedState _primary = FcsLedState::NONE, _advisory = FcsLedState::NONE;
    FcsLedState _rendered = FcsLedState::BOOT;

    /// 连续枚举段包含四个已生效模式，辅助提示不得进入此段。
    static bool isArmedMode(FcsLedState state) {
        return state >= FcsLedState::ARMED_MANUAL && state <= FcsLedState::ARMED_GUIDED;
    }

    static bool interruptsBoot(FcsLedState state) {
        return isEmergency(state) || state == FcsLedState::FAULT ||
               state == FcsLedState::CALIBRATING || isArmedMode(state);
    }

    /// 由色相和节奏共同区分状态。
    static uint32_t colorFor(FcsLedState state) {
        switch (state) {
            case FcsLedState::FAILSAFE:
            case FcsLedState::FAULT:
            case FcsLedState::NO_RECEIVER:
            case FcsLedState::LOW_VOLTAGE:
            case FcsLedState::BAT_CRITICAL: return kRed;
            case FcsLedState::CALIBRATING: return kYellow;
            case FcsLedState::LINK_DOWN: return kMagenta;
            case FcsLedState::LOW_LQ: return kOrange;
            case FcsLedState::ARMED_MANUAL: return kGreen;
            case FcsLedState::ARMED_ALT: return kCyan;
            case FcsLedState::ARMED_POS: return kWhite;
            case FcsLedState::ARMED_GUIDED:
            case FcsLedState::EKF_NOT_CONVERGED: return kPurple;
            default: return kBlue;
        }
    }

    /// 非线性亮度曲线，相位零点为峰值：告警切入当拍即亮，不从暗相开始。
    static float breath(uint32_t t, uint32_t period, float minimum, float maximum) {
        const float wave = 0.5f + 0.5f * std::cos(2.0f * static_cast<float>(PI) *
                                                   static_cast<float>(t % period) / period);
        return minimum + (maximum - minimum) * wave * wave;
    }

    /// 上电演示每段两端暗、中间亮，避免换色时亮度阶跃。
    static float bootBreath(uint32_t t) {
        const float wave = std::sin(static_cast<float>(PI) *
                                    static_cast<float>(t) / static_cast<float>(kBootSegmentMs));
        return 0.5f * wave * wave;
    }

    /// 分通道缩放，保持色相。
    static uint32_t scale(uint32_t rgb, float brightness) {
        const auto r = static_cast<uint32_t>(((rgb >> 16) & 255U) * brightness);
        const auto g = static_cast<uint32_t>(((rgb >> 8) & 255U) * brightness);
        const auto b = static_cast<uint32_t>((rgb & 255U) * brightness);
        return (r << 16) | (g << 8) | b;
    }

    static uint32_t blink(uint32_t rgb, uint32_t t, uint32_t half_period) {
        return (t / half_period) % 2U == 0 ? scale(rgb, 0.5f) : 0U;
    }

    /// 失控双闪与其他告警保持各自节奏，正常模式以连续呼吸显示。
    static uint32_t render(FcsLedState state, uint32_t t) {
        const uint32_t color = colorFor(state);
        if (isArmedMode(state)) return scale(color, breath(t, 2000, 0.15f, 0.5f));
        switch (state) {
            case FcsLedState::FAILSAFE: {
                const uint32_t phase = t % 900U;
                return (phase < 100U || (phase >= 200U && phase < 300U)) ? kRed : 0U;
            }
            case FcsLedState::FAULT:
            case FcsLedState::CALIBRATING: return blink(color, t, 50);
            case FcsLedState::NO_RECEIVER:
            case FcsLedState::LOW_LQ: return blink(color, t, 500);
            case FcsLedState::LINK_DOWN: return blink(color, t, 600);
            case FcsLedState::BAT_CRITICAL: return blink(color, t, 25);
            case FcsLedState::LOW_VOLTAGE: return scale(color, breath(t, 2000, 0.0f, 0.4f));
            case FcsLedState::EKF_NOT_CONVERGED: return scale(color, breath(t, 1200, 0.05f, 0.3f));
            case FcsLedState::GNSS_NO_FIX: return scale((t / 300U) % 2U == 0 ? kBlue : kPurple, 0.5f);
            default: return scale(kBlue, breath(t, 2500, 0.0f, 0.35f));
        }
    }
};
