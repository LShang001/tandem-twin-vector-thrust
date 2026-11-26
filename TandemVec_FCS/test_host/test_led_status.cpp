/**
 * @file    test_led_status.cpp
 * @module  固件/宿主回归测试（fw-tests：WS2812 灯效）
 * @brief   验证状态优先级、告警切入相位、模式色相、电池迟滞与发送去重。
 * @details 被测：`include/Ws2812AppStatus.h`（FcsLedPolicy + Ws2812AppStatus）。
 *          输入：可控时钟与业务快照；输出：像素颜色与 show 调用次数。
 * @theory  N/A（灯效人机界面）；时基为设计约定，不是物理模型。
 * @frame   N/A
 * @units   时间 ms；电压 mV；RGB 0–255
 * @note    与真实驱动的差异：桩只记录像素与 show 次数，不产生时序。
 *          因此本测试只回答“想显示什么、什么时候想显示”，不证明灯带电气正确。
 * @test    S1 BOOT 三段；S2 告警打断自检；S3 告警切入无暗相；S4 模式色相；
 *          S5 已解锁提示穿插；S6 电池迟滞；S7 LQ 新鲜度；S8 地面就绪；
 *          S9 颜色去重；S10 发送失败重试；S11 无告警电压源
 * @see     TandemVec_FCS/include/Ws2812AppStatus.h、test_host/stub/WS2812Driver.h
 */
#include "Ws2812AppStatus.h"

#include <cstdio>
#include <string>

// 桩的静态成员定义（test_host/stub/WS2812Driver.h）
uint8_t WS2812Driver::last_r = 0;
uint8_t WS2812Driver::last_g = 0;
uint8_t WS2812Driver::last_b = 0;
int WS2812Driver::show_count = 0;
bool WS2812Driver::send_failed = false;

static int g_fail_count = 0;
static void check(bool cond, const std::string &name)
{
  std::printf(cond ? "[PASS] %s\n" : "[FAIL] %s\n", name.c_str());
  if (!cond) ++g_fail_count;
}

// 断言当前像素等于给定值。
static void expectRgb(uint8_t r, uint8_t g, uint8_t b, const std::string &name)
{
  check(WS2812Driver::last_r == r && WS2812Driver::last_g == g && WS2812Driver::last_b == b,
        name + " (" + std::to_string(WS2812Driver::last_r) + "," +
            std::to_string(WS2812Driver::last_g) + "," +
            std::to_string(WS2812Driver::last_b) + ")");
}

// 只判主色分量是否存在，用于呼吸/闪烁相位不确定的时刻。
static void expectHasRed(const std::string &name)
{
  check(WS2812Driver::last_r > 0 && WS2812Driver::last_g == 0 && WS2812Driver::last_b == 0,
        name + " (" + std::to_string(WS2812Driver::last_r) + "," +
            std::to_string(WS2812Driver::last_g) + "," +
            std::to_string(WS2812Driver::last_b) + ")");
}

// 构造已建链、已收敛、有 3D 定位、未解锁的稳定地面快照。
static FcsLedInputs groundInputs()
{
  FcsLedInputs in;
  in.link_up = true;
  in.ever_linked = true;
  in.nav_ready_on_ground = true;
  in.gnss_fix = true;
  in.crsf_link_stats_seen = true;
  in.link_quality = 100;
  in.link_stats_ms = 0;
  in.battery_mv = 12600.0f;
  in.actual_mode = FcsLedState::ARMED_MANUAL;
  return in;
}

// 播完 1.2 s 上电自检，返回自检后的时钟。
static uint32_t playBoot(Ws2812AppStatus &led)
{
  const uint32_t end = 1200;
  for (uint32_t t = 0; t <= end; t += 10) {
    led.update({FcsLedState::STANDBY}, t);
  }
  return end;
}

int main()
{
  // ============ 1. BOOT 白→蓝→绿 三段 ============
  {
    WS2812Driver strip(30, 1);
    Ws2812AppStatus led(strip);
    led.update({FcsLedState::STANDBY}, 0);
    expectRgb(0, 0, 0, "BOOT 段0 起点暗相 (t=0)");
    led.update({FcsLedState::STANDBY}, 200);
    check(WS2812Driver::last_r > 0 && WS2812Driver::last_b > 0, "BOOT 段0 白持续 (t=200 段中点)");
    led.update({FcsLedState::STANDBY}, 500);
    check(WS2812Driver::last_r == 0 && WS2812Driver::last_g == 0 && WS2812Driver::last_b > 0,
          "BOOT 段1 蓝 (t=500 段中点)");
    led.update({FcsLedState::STANDBY}, 900);
    check(WS2812Driver::last_r == 0 && WS2812Driver::last_b == 0 && WS2812Driver::last_g > 0,
          "BOOT 段2 绿 (t=900 段中点)");
    led.update({FcsLedState::STANDBY}, 1200);
    const uint32_t t = playBoot(led);
    led.update({FcsLedState::STANDBY}, t + 500);
    check(WS2812Driver::last_r == 0 && WS2812Driver::last_g == 0 && WS2812Driver::last_b > 0,
          "BOOT 结束后进入 STANDBY 蓝");
  }

  // ============ 2. 紧急告警打断上电自检 ============
  {
    WS2812Driver strip(30, 1);
    Ws2812AppStatus led(strip);
    led.update({FcsLedState::FAILSAFE}, 0);
    expectHasRed("失控首拍即显示红，不等自检播完");
    led.update({FcsLedState::FAILSAFE}, 80);
    expectHasRed("失控第一亮持续 (t=80，100ms 窗口内)");
    led.update({FcsLedState::FAILSAFE}, 150);
    expectRgb(0, 0, 0, "失控双闪第二段灭相 (t=150)");
  }

  // ============ 3. 告警从亮相切入，不带暗相延迟 ============
  {
    WS2812Driver strip(30, 1);
    Ws2812AppStatus led(strip);
    playBoot(led);
    // 先让 STANDBY 跑到暗相，再在同一时刻切失控。
    led.update({FcsLedState::STANDBY}, 2600);
    led.update({FcsLedState::FAILSAFE}, 2600);
    expectHasRed("失控切入当拍即亮（相位自零点起算）");
    led.update({FcsLedState::BAT_CRITICAL}, 5000);
    check(WS2812Driver::last_r > 0, "电池危险切入当拍即亮");
  }

  // ============ 4. 四种实际模式色相 ============
  {
    WS2812Driver strip(30, 1);
    Ws2812AppStatus led(strip);
    const uint32_t t0 = playBoot(led);
    FcsLedInputs in = groundInputs();
    in.armed = true;

    in.actual_mode = FcsLedState::ARMED_MANUAL;
    led.update(FcsLedPolicy(3).select(in, t0), t0);
    check(WS2812Driver::last_r == 0 && WS2812Driver::last_g > 0 && WS2812Driver::last_b == 0,
          "ARMED_MANUAL 绿");

    in.actual_mode = FcsLedState::ARMED_ALT;
    led.update(FcsLedPolicy(3).select(in, t0), t0);
    check(WS2812Driver::last_r == 0 && WS2812Driver::last_g > 0 && WS2812Driver::last_b > 0,
          "ARMED_ALT 青");

    in.actual_mode = FcsLedState::ARMED_POS;
    led.update(FcsLedPolicy(3).select(in, t0), t0);
    check(WS2812Driver::last_r > 0 && WS2812Driver::last_g > 0 && WS2812Driver::last_b > 0,
          "ARMED_POS 白");

    in.actual_mode = FcsLedState::ARMED_GUIDED;
    led.update(FcsLedPolicy(3).select(in, t0), t0);
    check(WS2812Driver::last_r > 0 && WS2812Driver::last_b > WS2812Driver::last_r,
          "ARMED_GUIDED 紫 (B>R)");
  }

  // ============ 5. 已解锁的普通提示短闪穿插，仍能认出模式 ============
  {
    WS2812Driver strip(30, 1);
    Ws2812AppStatus led(strip);
    const uint32_t t0 = playBoot(led);
    FcsLedPolicy policy(3);
    FcsLedInputs in = groundInputs();
    in.armed = true;
    in.gnss_fix = false;                    // 无 3D 定位 → 提示位
    in.actual_mode = FcsLedState::ARMED_POS;

    const auto ind = policy.select(in, t0);
    check(ind.primary == FcsLedState::ARMED_POS && ind.advisory == FcsLedState::GNSS_NO_FIX,
          "无定位时主色仍报实际模式，定位降为提示");

    led.update(ind, t0);                    // 提示窗内
    check(WS2812Driver::last_b > 0 && WS2812Driver::last_g == 0,
          "提示窗内显示蓝色提示，不覆盖为纯告警");

    led.update(ind, t0 + 400);              // 提示窗后回到模式色
    check(WS2812Driver::last_r > 0 && WS2812Driver::last_g > 0 && WS2812Driver::last_b > 0,
          "提示窗后恢复模式白呼吸");
  }

  // ============ 6. 电池迟滞：进入即报，恢复留余量 ============
  {
    FcsLedPolicy policy(3);                 // 3S：低压 11.1 V、危险 10.5 V、恢复 +0.3 V
    FcsLedInputs in = groundInputs();

    in.battery_mv = 12600.0f;
    check(policy.select(in, 0).primary == FcsLedState::STANDBY, "12.6 V 无电池告警");

    in.battery_mv = 11000.0f;
    check(policy.select(in, 0).primary == FcsLedState::LOW_VOLTAGE, "11.0 V 进入低压");

    in.battery_mv = 11200.0f;               // 已高于低压但未到恢复线
    check(policy.select(in, 0).primary == FcsLedState::LOW_VOLTAGE, "11.2 V 未到恢复线仍报低压");

    in.battery_mv = 11500.0f;
    check(policy.select(in, 0).primary == FcsLedState::STANDBY, "11.5 V 越过恢复线解除");

    in.battery_mv = 10400.0f;
    check(policy.select(in, 0).primary == FcsLedState::BAT_CRITICAL, "10.4 V 直接越级报危险");

    in.battery_mv = 10600.0f;
    check(policy.select(in, 0).primary == FcsLedState::BAT_CRITICAL, "10.6 V 危险态保持");

    in.battery_mv = 10900.0f;
    check(policy.select(in, 0).primary == FcsLedState::LOW_VOLTAGE, "10.9 V 退到低压");

    FcsLedPolicy no_battery(0);             // 共轴构建无电压采样
    in.battery_mv = 9000.0f;
    check(no_battery.select(in, 0).primary == FcsLedState::STANDBY,
          "串数 0 时不报电池告警");
  }

  // ============ 7. LQ 只在新鲜时参与判定 ============
  {
    FcsLedPolicy policy(3);
    FcsLedInputs in = groundInputs();
    in.link_quality = 10;

    in.crsf_link_stats_seen = true;
    in.link_stats_ms = 5000;
    check(policy.select(in, 5500).primary == FcsLedState::LOW_LQ, "LQ=10 新鲜时报弱信号");

    // MAVLink 模式或未收到过统计帧时，初值/陈旧值不得驱动弱信号灯。
    in.crsf_link_stats_seen = false;
    check(policy.select(in, 5500).primary == FcsLedState::STANDBY,
          "无 CRSF 统计（MAVLink 模式）时不报弱信号");

    in.crsf_link_stats_seen = true;
    in.link_quality = 0;
    check(policy.select(in, 9000).primary == FcsLedState::STANDBY,
          "统计帧 4 s 过期后不再报弱信号（不被陈旧 0 值误判）");
  }

  // ============ 8. 地面就绪判据含收敛，不只初始化标志 ============
  {
    FcsLedPolicy policy(3);
    FcsLedInputs in = groundInputs();
    in.nav_ready_on_ground = false;
    check(policy.select(in, 0).primary == FcsLedState::EKF_NOT_CONVERGED,
          "初始化完成但未收敛时仍显示待收敛");

    in.nav_ready_on_ground = true;
    check(policy.select(in, 0).primary == FcsLedState::STANDBY, "就绪后进入待机");

    in.armed = true;
    in.nav_ready_on_ground = false;
    check(policy.select(in, 0).primary == FcsLedState::ARMED_MANUAL,
          "已解锁时提示不抢占模式主色");
  }

  // ============ 9. 上电无信号与地面断链可区分 ============
  {
    FcsLedPolicy policy(3);
    FcsLedInputs in = groundInputs();
    in.link_up = false;
    in.ever_linked = false;
    check(policy.select(in, 0).primary == FcsLedState::NO_RECEIVER, "从未建链 = 无接收机");

    in.ever_linked = true;
    check(policy.select(in, 0).primary == FcsLedState::LINK_DOWN, "曾建链 = 地面断链");

    in.failsafe = true;
    check(policy.select(in, 0).primary == FcsLedState::FAILSAFE, "空中失控优先于断链");
  }

  // ============ 10. 同色不重复提交，驱动失败保留重试 ============
  {
    WS2812Driver strip(30, 1);
    Ws2812AppStatus led(strip);
    playBoot(led);

    // 校准快闪的亮相窗内颜色恒定，应只提交一次。
    const int before = WS2812Driver::show_count;
    led.update({FcsLedState::CALIBRATING}, 2000);
    const int after_first = WS2812Driver::show_count;
    led.update({FcsLedState::CALIBRATING}, 2010);
    led.update({FcsLedState::CALIBRATING}, 2030);
    check(WS2812Driver::show_count == after_first,
          "同色连续拍只提交一次（首拍 " + std::to_string(after_first - before) + " 次）");

    // 呼吸一个完整周期，8 位量化后的同色拍必须合并，帧数显著低于 100 Hz。
    const int before_breath = WS2812Driver::show_count;
    for (uint32_t t = 3000; t < 5500; t += 10) {
      led.update({FcsLedState::STANDBY}, t);
    }
    const int breath_frames = WS2812Driver::show_count - before_breath;
    std::printf("[INFO] STANDBY 呼吸 2.5 s 实际提交 %d 帧（100 Hz 直发为 250 帧）\n",
                breath_frames);
    check(breath_frames < 250, "呼吸态按颜色去重，帧数低于逐拍直发");

    WS2812Driver::send_failed = true;
    led.invalidateFrame();
    led.update({FcsLedState::STANDBY}, 5600);
    check(!led.frameValid(), "发送失败时帧标记为无效");
    WS2812Driver::send_failed = false;
    const int before_retry = WS2812Driver::show_count;
    led.update({FcsLedState::STANDBY}, 5610);
    check(WS2812Driver::show_count == before_retry + 1 && led.frameValid(),
          "恢复后下一拍重试同色帧");
  }

  // ============ 11. 渲染状态与主状态可观测（供 wsstat） ============
  {
    WS2812Driver strip(30, 1);
    Ws2812AppStatus led(strip);
    playBoot(led);
    led.update({FcsLedState::ARMED_ALT}, 2000);
    check(led.renderedState() == FcsLedState::ARMED_ALT, "renderedState 反映当前主色");
    check(fcsLedStateName(FcsLedState::ARMED_ALT) == std::string("ARMED_ALT"),
          "状态名可读");
    check(Ws2812AppStatus::isEmergency(FcsLedState::BAT_CRITICAL) &&
          Ws2812AppStatus::isEmergency(FcsLedState::FAILSAFE) &&
          !Ws2812AppStatus::isEmergency(FcsLedState::LOW_LQ),
          "紧急状态集合只含失控与电池危险");
  }

  std::printf("\n==== LED 状态机测试结束: %s (%d fail) ====\n",
              g_fail_count == 0 ? "ALL PASS" : "FAILED", g_fail_count);
  return g_fail_count == 0 ? 0 : 1;
}