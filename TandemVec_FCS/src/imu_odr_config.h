/**
 * @file    imu_odr_config.h
 * @brief   IMU 采样率的唯一真源 + 型号支持列表 + 编译期严格校验
 *
 * @note    ★ 2026-11-10 立此文件的原因：
 *          改采样率之前散落在四处（ODR 配置 / 排空 kTargetRateHz / 批量帧 dt /
 *          ISR 相位累加），全部靠手写数值维护。2026-11-10 排查"控制响应慢"时，
 *          根因正是 IMU FIFO 排空长期欠量（控制吃 73ms 前的数据），而欠量的来源
 *          就是这几处数值不一致 + 后端"吸附"到相邻档位的隐式行为。
 *          现在四处全部由 kImuOdrHz 派生，且目标值必须在型号支持列表内，否则
 *          编译期直接报错；后端也改成严格匹配，不再吸附。
 *
 * @note    切换采样率的两种方式：
 *          1) 改本文件里对应型号的默认值（kImuOdrHz 的默认分支）
 *          2) 编译期覆盖：`pio run -e <env> -DBFS_IMU_ODR_HZ=800`
 *          不在支持列表内的值会触发 static_assert 失败，不会烧出跑错档位的固件。
 *
 * @note    为什么只开放整数档：ICM-45686 支持 1.5625/3.125/6.25/12.5 等小数档，
 *          但对飞控毫无意义（控制环 200Hz，最低用 50Hz 已有 4 倍过采样），
 *          且小数无法用 uint16_t 精确表达，强行支持只会引入取整歧义。
 *
 * @note    采样率是编译期常量，与 env 绑定，不做运行时切换——运行中改 ODR 会牵动
 *          调度任务周期、EKF 子样划分、增量缓冲深度，风险远大于收益。
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

namespace bfs_imu_odr
{

/** @brief 当前编译目标 IMU 型号支持的采样率档位（整数 Hz，升序）。 */
#if defined(FCS_IMU_MODEL_ICM45686)
// ICM-45686 驱动枚举：1.5625/3.125/6.25/12.5/25/50/100/200/400/800/1600/3200/6400
// 这里只列整数档；6.4k/3.2k/1.6k/800 是手册 LN（低噪声）档。
constexpr uint16_t kSupportedHz[] = {50, 100, 200, 400, 800, 1600, 3200, 6400};
#else
// ICM-42688 驱动当前开放集（无 1600 档，这也是当年"传 1600 被吸附到 2000"的由来）。
constexpr uint16_t kSupportedHz[] = {50, 100, 200, 500, 1000, 2000, 4000, 8000, 16000, 32000};
#endif

constexpr size_t kSupportedCount = sizeof(kSupportedHz) / sizeof(kSupportedHz[0]);

/** @brief 递归实现，C++11 兼容的 constexpr 查找（constexpr 函数体内不允许循环）。 */
constexpr bool supportedImpl(uint16_t hz, const uint16_t *list, size_t n, size_t i)
{
  return (i >= n) ? false : ((list[i] == hz) || supportedImpl(hz, list, n, i + 1));
}

/** @brief 判断某采样率是否受当前型号支持。 */
constexpr bool supported(uint16_t hz)
{
  return supportedImpl(hz, kSupportedHz, kSupportedCount, 0);
}

/** @brief 目标采样率：默认值按型号取实飞验证过的档，可用 -DBFS_IMU_ODR_HZ 覆盖。 */
#ifndef BFS_IMU_ODR_HZ
#if defined(FCS_IMU_MODEL_ICM45686)
constexpr uint16_t kOdrHz = 1600; // 2026-11-10 实飞确认（实测 1683Hz，芯片 RC 时钟 +5%）
#else
constexpr uint16_t kOdrHz = 2000; // 与 ICM-42688 后端精确匹配（该型号无 1600 档）
#endif
#else
constexpr uint16_t kOdrHz = static_cast<uint16_t>(BFS_IMU_ODR_HZ);
#endif

/** @brief 编译期把关：目标值必须在支持列表内，否则不允许出固件。 */
static_assert(supported(kOdrHz),
              "BFS_IMU_ODR_HZ / kOdrHz 不在该 IMU 型号支持的采样率档位内；"
              "请查阅 imu_odr_config.h 的 kSupportedHz 列表");

/**
 * @brief 编译期把关：采样率不能超过调度时基（当前 6400Hz）。
 *
 * ISR 由 TIM8 以 kSchedulerTickHz 触发、按相位累加抽点取帧，一拍的采样率上限
 * 就是时基本身。超过时基（例如 42688 的 8000/16000/32000 档）会让单拍需要取
 * 多帧，现逻辑只做一次相位扣减，必然持续积压——这正是 2026-11-10 延迟事故的
 * 同一种失败模式。要支持更高采样率，必须先提高 kSchedulerTickHz（会影响所有
 * 任务的周期精度），属于独立改动，不在采样率切换范围内。
 */
static_assert(kOdrHz <= 6400U,
              "所选采样率超过调度时基 6400Hz：需先提高 kSchedulerTickHz 才能使用该档");

} // namespace bfs_imu_odr
