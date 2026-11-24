/**
 * @file    ElrsFlightModeText.h
 * @module  固件/通信遥测（纯平台无关文本生成）
 * @brief   ELRS 回传的飞行模式文本：实际模式 + 解锁状态 + 降级标记
 * @details CRSF FLIGHT_MODE 帧的 16 字符净荷由本模块生成，固件 `sendElrsFlightModeData()`
 *          与宿主测试调用**同一份**代码：
 *            未解锁 → `DISARM`；
 *            已解锁 → `MANUAL` / `ALTHOLD` / `POSHOLD` / `GUIDED`（实际生效模式）；
 *            本拍发生模式降级时追加 `-DEG`。
 *          取实际模式而不是按模式通道推算：`determine_control_mode()` 在航向不可观
 *          （POSHOLD/GUIDED→ALTHOLD）、GNSS 长失联（同上，opt-in 构建）、制导指令超时
 *          （GUIDED→MANUAL）三种情况下会改写模式，按通道推算会报出并未生效的模式；
 *          旧实现未解锁时也显示 `MANUAL`，飞手无法从回传分辨是否已解锁。
 * @frame  N/A
 * @units  N/A
 *
 * 不变量：
 *   - 输出以 '\0' 结尾，可见字符数 ≤ kMaxVisibleChars（13）：EdgeTX 把 FLIGHT_MODE 帧
 *     按 min(16, 帧长字节) 截断，第 3..15 字节之外的内容不显示；
 *   - 未解锁时恒为 `DISARM`，与降级标志无关；
 *   - `-DEG` 之外，文本与实际模式一一对应；未知枚举值显示 `AUTO`。
 *
 * @ref  CTL-001 §8.6（模式与安全联锁）；
 *       EdgeTX radio/src/telemetry/crossfire.cpp::FLIGHT_MODE_ID（文本截断规则）
 * @note 本文件不得包含 Arduino/串口/全局状态依赖，宿主测试直接编译。`Mode` 与
 *       `state_data.h` 的 `ControlMode` 同序，由 `src/communication.cpp` 的 static_assert
 *       守住。
 */
#ifndef BFS_ELRS_FLIGHT_MODE_TEXT_H
#define BFS_ELRS_FLIGHT_MODE_TEXT_H

#include <cstddef>
#include <cstdint>

namespace bfs {
namespace elrs {

/** EdgeTX 对 FLIGHT_MODE 帧可显示的字符数上限（不含结尾 '\0'） */
constexpr std::size_t kMaxVisibleChars = 13;

/** CRSF FLIGHT_MODE 净荷长度（字符数） */
constexpr std::size_t kFlightModeFieldChars = 16;

/** 未解锁文本 */
constexpr char kDisarmedText[] = "DISARM";

/** 降级标记：模式在本拍被飞控改写时追加在模式名之后 */
constexpr char kDegradedSuffix[] = "-DEG";

/** 与 state_data.h 的 ControlMode 同序 */
enum class Mode : std::int32_t
{
  kManual = 0,
  kAutoPosition = 1,
  kAutoAltitude = 2,
  kGuided = 3
};

/** 文本生成输入（全部由调用点从全局量取好，本模块不读全局量） */
struct FlightModeInput {
  bool armed;             /**< 实际解锁状态（g_is_unlocked） */
  Mode actual_mode;       /**< 实际生效模式（g_current_flight_mode） */
  bool degraded_heading;  /**< 航向不可观导致的降级（g_mode_degraded_heading） */
  bool degraded_gnss;     /**< GNSS 长失联导致的降级（g_mode_degraded_gnss） */
  bool degraded_guidance; /**< 制导指令超时导致 GUIDED 回落 MANUAL */
};

/** 模式名，不含降级标记；未知枚举值返回 "AUTO" */
inline const char *modeName(Mode mode)
{
  switch (mode)
  {
  case Mode::kManual:
    return "MANUAL";
  case Mode::kAutoAltitude:
    return "ALTHOLD";
  case Mode::kAutoPosition:
    return "POSHOLD";
  case Mode::kGuided:
    return "GUIDED";
  default:
    return "AUTO";
  }
}

/**
 * @brief 生成 FLIGHT_MODE 帧的文本
 * @pre   out 指向至少 cap 字节的可写缓冲
 * @post  out 以 '\0' 结尾；缓冲不足时截断，不越界写
 */
inline void buildFlightModeText(const FlightModeInput &in, char *out, std::size_t cap)
{
  if ((out == nullptr) || (cap == 0U))
  {
    return;
  }
  out[0] = '\0';
  if (cap == 1U)
  {
    return;
  }

  const char *head = in.armed ? modeName(in.actual_mode) : kDisarmedText;
  std::size_t len = 0U;
  while ((head[len] != '\0') && ((len + 1U) < cap))
  {
    out[len] = head[len];
    ++len;
  }

  const bool degraded = in.degraded_heading || in.degraded_gnss || in.degraded_guidance;
  if (in.armed && degraded)
  {
    for (std::size_t i = 0U; (kDegradedSuffix[i] != '\0') && ((len + 1U) < cap); ++i)
    {
      out[len] = kDegradedSuffix[i];
      ++len;
    }
  }
  out[len] = '\0';
}

}  // namespace elrs
}  // namespace bfs

#endif  // BFS_ELRS_FLIGHT_MODE_TEXT_H
