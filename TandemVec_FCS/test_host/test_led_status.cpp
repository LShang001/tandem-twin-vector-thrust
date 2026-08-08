// 宿主机回归测试：Ws2812AppStatus 灯效状态机（include/Ws2812AppStatus.h）
// 编译运行：g++ -std=c++17 -Itest_host/stub -Iinclude test_host/test_led_status.cpp -o test_host/bin/led_status && ./test_host/bin/led_status
//
// 验证（2026-08-08 灯效扩展）：
//   - BOOT 上电自检序列：白→蓝→绿 三段 400ms，1.2s 后进入正常状态机
//   - 13 个 FcsLedState 的颜色与节奏（呼吸/慢闪/快闪/SOS 双闪时基）
//   - setFault 调试命令覆盖优先级
//   - 每次 update 必渲染（show() 调用）
// 通过 -Itest_host/stub 用极简 WS2812Driver mock 替代真实驱动。
#include "../include/Ws2812AppStatus.h"

#include <cstdio>
#include <string>

// mock 静态成员定义（stub/WS2812Driver.h）
uint8_t WS2812Driver::last_r = 0;
uint8_t WS2812Driver::last_g = 0;
uint8_t WS2812Driver::last_b = 0;
int WS2812Driver::show_count = 0;

static int g_fail_count = 0;
static void check(bool cond, const std::string &name)
{
  std::printf(cond ? "[PASS] %s\n" : "[FAIL] %s\n", name.c_str());
  if (!cond) ++g_fail_count;
}

// 便捷断言：当前像素 == (r,g,b)
static void expectRgb(uint8_t r, uint8_t g, uint8_t b, const std::string &name)
{
  check(WS2812Driver::last_r == r && WS2812Driver::last_g == g && WS2812Driver::last_b == b,
        name + " (" + std::to_string(WS2812Driver::last_r) + "," +
        std::to_string(WS2812Driver::last_g) + "," + std::to_string(WS2812Driver::last_b) + ")");
}

// 播完 BOOT 自检序列（1.2s，每 10ms 一帧，最后一次 t=1200 清零退出）
static void playBoot(Ws2812AppStatus &led)
{
  for (unsigned long t = 0; t <= 1200; t += 10)
  {
    led.update(FcsLedState::STANDBY, t);
  }
}

int main()
{
  const unsigned long T0 = 2000;  // BOOT(1.2s) 播完后的正常状态机起点

  // ============ 1. BOOT 上电自检序列（白→蓝→绿 各 400ms） ============
  {
    WS2812Driver strip(30, 1);
    Ws2812AppStatus led(strip);

    led.update(FcsLedState::STANDBY, 0);     // 首次调用：白段起始（呼吸 0.25×0.5 亮度）
    check(WS2812Driver::last_r > 0 && WS2812Driver::last_g > 0 && WS2812Driver::last_b > 0,
          "BOOT 段0 = 白色 (t=0)");
    led.update(FcsLedState::STANDBY, 200);   // 白段峰值附近
    check(WS2812Driver::last_r > 0 && WS2812Driver::last_g > 0 && WS2812Driver::last_b > 0,
          "BOOT 段0 白色持续 (t=200)");
    led.update(FcsLedState::STANDBY, 400);   // 边界 → 段1 蓝
    check(WS2812Driver::last_r == 0 && WS2812Driver::last_g == 0 && WS2812Driver::last_b > 0,
          "BOOT 段1 = 纯蓝 (t=400)");
    led.update(FcsLedState::STANDBY, 800);   // 边界 → 段2 绿
    check(WS2812Driver::last_r == 0 && WS2812Driver::last_g > 0 && WS2812Driver::last_b == 0,
          "BOOT 段2 = 纯绿 (t=800)");
    led.update(FcsLedState::STANDBY, 1200);  // 1.2s 播完瞬间：全灭
    expectRgb(0, 0, 0, "BOOT 播完瞬间全灭 (t=1200)");
    led.update(FcsLedState::STANDBY, T0 + 500);  // 正常状态机接管（2500ms 蓝呼吸亮区）
    check(WS2812Driver::last_r == 0 && WS2812Driver::last_g == 0 && WS2812Driver::last_b > 0,
          "BOOT 结束后进入 STANDBY 蓝呼吸");
  }

  // ============ 2. 各状态颜色正确性（BOOT 已播完） ============
  {
    WS2812Driver strip(30, 1);
    Ws2812AppStatus led(strip);
    playBoot(led);                               // 播完 BOOT

    // 呼吸相位统一取段内峰值（相位=π/2）便于断言主色
    const unsigned long t_peak = T0 + 500;

    led.update(FcsLedState::STANDBY, t_peak);
    check(WS2812Driver::last_r == 0 && WS2812Driver::last_g == 0 && WS2812Driver::last_b > 0,
          "STANDBY = 蓝呼吸");

    led.update(FcsLedState::ARMED_MANUAL, t_peak);
    check(WS2812Driver::last_r == 0 && WS2812Driver::last_g > 0 && WS2812Driver::last_b == 0,
          "ARMED_MANUAL = 绿呼吸");

    led.update(FcsLedState::ARMED_ALT, t_peak);
    check(WS2812Driver::last_r == 0 && WS2812Driver::last_g > 0 && WS2812Driver::last_b > 0,
          "ARMED_ALT = 青呼吸 (G+B)");

    led.update(FcsLedState::ARMED_POS, t_peak);
    check(WS2812Driver::last_r > 0 && WS2812Driver::last_g > 0 && WS2812Driver::last_b > 0,
          "ARMED_POS = 白呼吸 (R+G+B)");

    led.update(FcsLedState::ARMED_GUIDED, t_peak);
    check(WS2812Driver::last_r > 0 && WS2812Driver::last_g == 0 && WS2812Driver::last_b > 0 &&
          WS2812Driver::last_r < WS2812Driver::last_b,
          "ARMED_GUIDED = 紫呼吸 (R+B, R<B)");

    led.update(FcsLedState::LINK_DOWN, t_peak);
    check(WS2812Driver::last_r > 0 && WS2812Driver::last_g == 0 && WS2812Driver::last_b > 0 &&
          WS2812Driver::last_b > WS2812Driver::last_r * 0.5f,
          "LINK_DOWN = 品红慢闪 (R+B, B 较强)");

    led.update(FcsLedState::LOW_LQ, T0);     // 闪烁亮相（2000/500 为偶数）
    check(WS2812Driver::last_r > 0 && WS2812Driver::last_g > 0 && WS2812Driver::last_b == 0 &&
          WS2812Driver::last_r > WS2812Driver::last_g,
          "LOW_LQ = 橙慢闪 (R>G, 无 B)");

    led.update(FcsLedState::LOW_VOLTAGE, t_peak);
    check(WS2812Driver::last_r > 0 && WS2812Driver::last_g == 0 && WS2812Driver::last_b == 0,
          "LOW_VOLTAGE = 红慢呼吸");

    led.update(FcsLedState::CALIBRATING, t_peak);
    check(WS2812Driver::last_r > 0 && WS2812Driver::last_g > 0 && WS2812Driver::last_b == 0 &&
          WS2812Driver::last_r == WS2812Driver::last_g,
          "CALIBRATING = 黄快闪 (R==G)");
  }

  // ============ 3. 闪烁/SOS 节奏（时基） ============
  {
    WS2812Driver strip(30, 1);
    Ws2812AppStatus led(strip);
    playBoot(led);                               // 播完 BOOT

    // NO_RECEIVER 红慢闪 1Hz（半周期 500ms）：亮 500ms / 灭 500ms
    led.update(FcsLedState::NO_RECEIVER, T0);
    expectRgb(127, 0, 0, "NO_RECEIVER t=T0 亮 (半亮红)");
    led.update(FcsLedState::NO_RECEIVER, T0 + 700);
    expectRgb(0, 0, 0, "NO_RECEIVER t=T0+700 灭");
    led.update(FcsLedState::NO_RECEIVER, T0 + 1000);
    expectRgb(127, 0, 0, "NO_RECEIVER t=T0+1000 再亮");

    // LINK_DOWN 品红慢闪（半周期 600ms）
    led.update(FcsLedState::LINK_DOWN, T0 + 400);
    check(WS2812Driver::last_r > 0 && WS2812Driver::last_b > 0, "LINK_DOWN t=T0+400 亮");
    led.update(FcsLedState::LINK_DOWN, T0 + 1000);
    expectRgb(0, 0, 0, "LINK_DOWN t=T0+1000 灭");

    // CALIBRATING 黄快闪 10Hz（半周期 50ms）
    led.update(FcsLedState::CALIBRATING, T0);
    expectRgb(127, 127, 0, "CALIBRATING t=T0 亮 (半亮黄)");
    led.update(FcsLedState::CALIBRATING, T0 + 50);
    expectRgb(0, 0, 0, "CALIBRATING t=T0+50 灭");
    led.update(FcsLedState::CALIBRATING, T0 + 100);
    expectRgb(127, 127, 0, "CALIBRATING t=T0+100 再亮");

    // FAILSAFE SOS 双闪（亮100/灭100/亮100/灭600，周期900ms，全亮红）
    // 用相对时间基准（900 的整数倍相位起点，避免 T0 偏移周期）
    led.update(FcsLedState::FAILSAFE, 0);
    expectRgb(255, 0, 0, "FAILSAFE t=0 第一亮 (全亮红)");
    led.update(FcsLedState::FAILSAFE, 150);
    expectRgb(0, 0, 0, "FAILSAFE t=150 灭");
    led.update(FcsLedState::FAILSAFE, 250);
    expectRgb(255, 0, 0, "FAILSAFE t=250 第二亮");
    led.update(FcsLedState::FAILSAFE, 350);
    expectRgb(0, 0, 0, "FAILSAFE t=350 长灭");
    led.update(FcsLedState::FAILSAFE, 900);
    expectRgb(255, 0, 0, "FAILSAFE 周期回绕后仍双闪 (t=900)");

    // FAULT 红快闪 10Hz（半周期 50ms）
    led.update(FcsLedState::FAULT, T0);
    expectRgb(127, 0, 0, "FAULT t=T0 亮 (半亮红)");
    led.update(FcsLedState::FAULT, T0 + 50);
    expectRgb(0, 0, 0, "FAULT t=T0+50 灭");
  }

  // ============ 4. 呼吸亮度随时间变化 + 亮度上限 ============
  {
    WS2812Driver strip(30, 1);
    Ws2812AppStatus led(strip);
    playBoot(led);                               // 播完 BOOT

    led.update(FcsLedState::ARMED_MANUAL, T0);         // 相位 0（亮度 0.25×0.5）
    uint8_t dim = WS2812Driver::last_g;
    led.update(FcsLedState::ARMED_MANUAL, T0 + 500);   // 相位 π/2（峰值 0.5）
    uint8_t bright = WS2812Driver::last_g;
    check(bright > dim, "绿呼吸亮度随时间变化 (峰值 > 谷值)");
    check(bright <= 127, "绿呼吸峰值亮度 ≤ 50% (127)");

    led.update(FcsLedState::STANDBY, T0 + 500);
    check(WS2812Driver::last_b <= 89, "蓝呼吸峰值亮度 ≤ 35% (89)");

    led.update(FcsLedState::LOW_VOLTAGE, T0 + 500);
    check(WS2812Driver::last_r <= 102, "低压红呼吸峰值亮度 ≤ 40% (102)");
  }

  // ============ 5. setFault 覆盖优先级（wsfault 调试命令） ============
  {
    WS2812Driver strip(30, 1);
    Ws2812AppStatus led(strip);
    playBoot(led);                               // 播完 BOOT

    led.setFault(true);
    led.update(FcsLedState::ARMED_MANUAL, T0);    // 故障覆盖解锁绿呼吸
    expectRgb(127, 0, 0, "setFault(true) 覆盖 ARMED → 红快闪");
    led.update(FcsLedState::STANDBY, T0 + 50);
    expectRgb(0, 0, 0, "故障红快闪灭相");

    led.setFault(false);
    led.update(FcsLedState::STANDBY, T0 + 500);   // 2500ms 蓝呼吸亮区
    check(WS2812Driver::last_r == 0 && WS2812Driver::last_g == 0 && WS2812Driver::last_b > 0,
          "setFault(false) 恢复待机蓝呼吸");
  }

  // ============ 6. 每次 update 必渲染（show 调用计数） ============
  {
    WS2812Driver strip(30, 1);
    Ws2812AppStatus led(strip);
    const int before = WS2812Driver::show_count;
    led.update(FcsLedState::STANDBY, 0);
    led.update(FcsLedState::STANDBY, 100);
    led.update(FcsLedState::ARMED_MANUAL, T0);
    led.update(FcsLedState::FAILSAFE, T0);
    check(WS2812Driver::show_count == before + 4, "每次 update 均调用 show()");
  }

  std::printf("\n==== LED 状态机测试结束: %s (%d fail) ====\n",
              g_fail_count == 0 ? "ALL PASS" : "FAILED", g_fail_count);
  return g_fail_count == 0 ? 0 : 1;
}
