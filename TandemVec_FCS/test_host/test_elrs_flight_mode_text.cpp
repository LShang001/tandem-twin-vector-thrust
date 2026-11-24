/**
 * @file    test_elrs_flight_mode_text.cpp
 * @module  固件/宿主回归测试（fw-tests：ELRS 飞行模式回传文本）
 * @brief   锁定 FLIGHT_MODE 回传文本的不变量：实际模式、解锁状态、降级标记、13 字符上限
 * @details 被测：`include/ElrsFlightModeText.h`（固件 `sendElrsFlightModeData()` 与
 *          本测试调用**同一份**实现，不是按源码重写）。
 *          输入：解锁状态 × 四种实际模式 × 三个降级标志的穷举，外加缓冲边界；
 *          处理：buildFlightModeText；
 *          输出：PASS/FAIL 计数，main 返回非零即失败。
 *          编译：g++ -std=c++17 -Iinclude test_host/test_elrs_flight_mode_text.cpp -o bin/elrsfmt
 * @theory  回传文本 = 未解锁 ? "DISARM" : 实际模式名 (+ "-DEG" 若本拍降级)
 *          @ref CTL-001 §8.6；EdgeTX crossfire.cpp::FLIGHT_MODE_ID
 * @frame   N/A
 * @units   N/A
 * @note    本测试锁定 2026-11-24 的修改：此前按 CH7 通道值推算模式，制导超时回落
 *          MANUAL、航向不可观降级 ALTHOLD 时遥控器仍显示通道指示的模式，且未解锁
 *          也显示 MANUAL。EdgeTX 只显示该帧的前 13 个字符，文本不得超长。
 * @test    T1 未解锁恒 DISARM；T2 已解锁模式名；T3 降级标记；T4 长度上限；
 *          T5 缓冲边界；T6 未知枚举；T7 与 ControlMode 同序
 * @see     include/ElrsFlightModeText.h、src/communication.cpp::sendElrsFlightModeData
 */
#include <cstdio>
#include <cstring>

#include "ElrsFlightModeText.h"

static int g_fail = 0;
static int g_pass = 0;
#define CHECK(cond, msg)                                                        \
  do                                                                            \
  {                                                                             \
    if (!(cond))                                                                \
    {                                                                           \
      std::printf("FAIL: %s (line %d)\n", msg, __LINE__);                       \
      g_fail++;                                                                 \
    }                                                                           \
    else                                                                        \
    {                                                                           \
      g_pass++;                                                                 \
    }                                                                           \
  } while (0)

namespace elrs = bfs::elrs;

namespace
{
const elrs::Mode kModes[] = {elrs::Mode::kManual, elrs::Mode::kAutoAltitude,
                             elrs::Mode::kAutoPosition, elrs::Mode::kGuided};
}  // namespace

// T1：未解锁时任何模式、任何降级标志都只显示 DISARM（旧实现此时显示 MANUAL）。
static void testDisarmed()
{
  const bool flags[] = {false, true};
  for (elrs::Mode mode : kModes)
  {
    for (bool heading : flags)
    {
      for (bool gnss : flags)
      {
        for (bool guidance : flags)
        {
          elrs::FlightModeInput in{};
          in.armed = false;
          in.actual_mode = mode;
          in.degraded_heading = heading;
          in.degraded_gnss = gnss;
          in.degraded_guidance = guidance;
          char text[elrs::kFlightModeFieldChars + 1U] = {};
          elrs::buildFlightModeText(in, text, sizeof(text));
          CHECK(std::strcmp(text, "DISARM") == 0, "未解锁必须显示 DISARM");
        }
      }
    }
  }
}

// T2：已解锁且无降级，文本与实际模式一一对应。
static void testArmedModeNames()
{
  const char *expected[] = {"MANUAL", "ALTHOLD", "POSHOLD", "GUIDED"};
  for (std::size_t i = 0U; i < sizeof(kModes) / sizeof(kModes[0]); ++i)
  {
    elrs::FlightModeInput in{};
    in.armed = true;
    in.actual_mode = kModes[i];
    char text[elrs::kFlightModeFieldChars + 1U] = {};
    elrs::buildFlightModeText(in, text, sizeof(text));
    CHECK(std::strcmp(text, expected[i]) == 0, "已解锁模式名与实际模式对应");
    CHECK(std::strcmp(elrs::modeName(kModes[i]), expected[i]) == 0, "modeName 与实际模式对应");
  }
}

// T3：三个降级标志各自、任意组合都追加 -DEG；最长的 "ALTHOLD-DEG" 仍在可显示范围内。
static void testDegradedSuffix()
{
  const bool flags[] = {false, true};
  for (elrs::Mode mode : kModes)
  {
    for (bool heading : flags)
    {
      for (bool gnss : flags)
      {
        for (bool guidance : flags)
        {
          const bool any_degraded = heading || gnss || guidance;
          elrs::FlightModeInput in{};
          in.armed = true;
          in.actual_mode = mode;
          in.degraded_heading = heading;
          in.degraded_gnss = gnss;
          in.degraded_guidance = guidance;
          char text[elrs::kFlightModeFieldChars + 1U] = {};
          elrs::buildFlightModeText(in, text, sizeof(text));

          char expected[elrs::kFlightModeFieldChars + 1U] = {};
          std::strncpy(expected, elrs::modeName(mode), sizeof(expected) - 1U);
          if (any_degraded)
          {
            std::strncat(expected, elrs::kDegradedSuffix,
                         sizeof(expected) - std::strlen(expected) - 1U);
          }
          CHECK(std::strcmp(text, expected) == 0, "降级标记按标志追加");
        }
      }
    }
  }

  elrs::FlightModeInput worst{};
  worst.armed = true;
  worst.actual_mode = elrs::Mode::kAutoAltitude;
  worst.degraded_heading = true;
  worst.degraded_gnss = true;
  worst.degraded_guidance = true;
  char text[elrs::kFlightModeFieldChars + 1U] = {};
  elrs::buildFlightModeText(worst, text, sizeof(text));
  CHECK(std::strcmp(text, "ALTHOLD-DEG") == 0, "最长文本为 ALTHOLD-DEG");
  CHECK(std::strlen(text) == 11U, "最长文本 11 字符");
}

// T4：穷举下文本始终以 '\0' 结尾，且不超过 EdgeTX 的 13 字符显示上限与 16 字符净荷。
static void testLengthInvariant()
{
  const bool flags[] = {false, true};
  for (bool armed : flags)
  {
    for (elrs::Mode mode : kModes)
    {
      for (bool heading : flags)
      {
        for (bool gnss : flags)
        {
          for (bool guidance : flags)
          {
            elrs::FlightModeInput in{};
            in.armed = armed;
            in.actual_mode = mode;
            in.degraded_heading = heading;
            in.degraded_gnss = gnss;
            in.degraded_guidance = guidance;
            char text[elrs::kFlightModeFieldChars + 1U] = {};
            elrs::buildFlightModeText(in, text, sizeof(text));
            CHECK(std::strlen(text) <= elrs::kMaxVisibleChars, "文本不超过 13 字符");
            CHECK(std::strlen(text) <= elrs::kFlightModeFieldChars, "文本不超过 16 字符净荷");
          }
        }
      }
    }
  }
}

// T5：缓冲边界——容量 0 不写；容量 1 只写结尾符；容量不足时截断且不越界。
static void testBufferEdges()
{
  elrs::FlightModeInput in{};
  in.armed = true;
  in.actual_mode = elrs::Mode::kAutoAltitude;
  in.degraded_heading = true;

  elrs::buildFlightModeText(in, nullptr, 0U);  // 不得崩溃

  char guard[8];
  std::memset(guard, 0xA5, sizeof(guard));
  elrs::buildFlightModeText(in, guard, 0U);  // 容量 0：一个字节都不写
  for (char c : guard)
  {
    CHECK(static_cast<unsigned char>(c) == 0xA5U, "容量 0 不写入");
  }

  elrs::buildFlightModeText(in, guard + 2U, 1U);  // 容量 1：只写结尾符
  CHECK(guard[2] == '\0', "容量 1 只写结尾符");
  CHECK(static_cast<unsigned char>(guard[0]) == 0xA5U, "容量 1 不写缓冲之前的字节");
  CHECK(static_cast<unsigned char>(guard[3]) == 0xA5U, "容量 1 不越界");

  for (std::size_t cap = 2U; cap <= 12U; ++cap)
  {
    char small[16];
    std::memset(small, 0x5A, sizeof(small));
    elrs::buildFlightModeText(in, small, cap);
    CHECK(small[cap - 1U] == '\0' || std::strlen(small) < cap, "截断后仍以结尾符结束");
    CHECK(std::strlen(small) < cap, "可见字符数小于容量");
    CHECK(static_cast<unsigned char>(small[cap]) == 0x5AU, "不越界写");
  }
}

// T6：未知枚举值不得显示成某个真实模式。
static void testUnknownMode()
{
  elrs::FlightModeInput in{};
  in.armed = true;
  in.actual_mode = static_cast<elrs::Mode>(99);
  char text[elrs::kFlightModeFieldChars + 1U] = {};
  elrs::buildFlightModeText(in, text, sizeof(text));
  CHECK(std::strcmp(text, "AUTO") == 0, "未知枚举显示 AUTO");
}

// T7：Mode 与 ControlMode 同序（MANUAL / AUTO_POSITION / AUTO_ALTITUDE / GUIDED）。
static void testEnumOrder()
{
  static_assert(static_cast<std::int32_t>(elrs::Mode::kManual) == 0, "Mode::kManual");
  static_assert(static_cast<std::int32_t>(elrs::Mode::kAutoPosition) == 1, "Mode::kAutoPosition");
  static_assert(static_cast<std::int32_t>(elrs::Mode::kAutoAltitude) == 2, "Mode::kAutoAltitude");
  static_assert(static_cast<std::int32_t>(elrs::Mode::kGuided) == 3, "Mode::kGuided");
  static_assert(elrs::kMaxVisibleChars == 13U, "EdgeTX 显示上限");
  static_assert((sizeof(elrs::kDegradedSuffix) - 1U) == 4U, "降级标记长度");
  g_pass += 6;
}

int main()
{
  testDisarmed();
  testArmedModeNames();
  testDegradedSuffix();
  testLengthInvariant();
  testBufferEdges();
  testUnknownMode();
  testEnumOrder();

  std::printf("ELRS 飞行模式文本：PASS %d, FAIL %d\n", g_pass, g_fail);
  if (g_fail != 0)
  {
    std::printf("FAIL: 存在失败项\n");
    return 1;
  }
  return 0;
}
