/**
 * @file    imu_odr_config.h
 * @module  sensor-imu
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

#include "task_scheduler_interval.h" // scheduler::kSchedulerTickHz（ISR 相位累加的分母）

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

/**
 * @brief ISR 取数轮询频率（≠ 芯片 ODR）——按"整除时基 + 不低于 1.5×ODR"自动选档
 *
 * 三条约束（缺一不可）：
 *
 * ① **必须整除调度时基**。ISR 由 TIM8 以 `scheduler::kSchedulerTickHz` 触发，
 *    取数依相位累加抽点：`s_imu_phase += kIsrPollHz; if (>= 时基) 取一帧`。
 *    整除时每 N tick 严格取一次；不整除会在 N/N+1 拍之间交替，形成长周期节奏抖动。
 *
 * ② **必须不低于 1.5 × kOdrHz**。`readFifo` 每次最多交出一包，所以 ISR 的帧率上限
 *    就是 ISR 频率。芯片 ODR 由 RC 晶振决定，实测偏高（ICM-45686 标称 1600 实跑
 *    1683，+5%）；ISR 若只按标称值取数，差的帧会沉进 FIFO/ring。2026-11-11 实测
 *    ISR=1600 时：芯片产 1656 帧/s、ISR 只能交 1600 帧/s，`ring_size=4`、
 *    `imu_age` 从 412µs 恶化到 2650µs，且 `readIMUData` 每拍要多补帧、CPU 从
 *    34.9% 升到 45.1%——**排空只解决 FIFO 深度，解决不了取数次数不足**。
 *    取 1.5× 留 42% 余量（相对 +5% 的晶振偏差），再低就可能贴边。
 *
 * ③ **不超过调度时基**。一拍最多抽一次，超过时基物理上做不到（旧逻辑只做一次
 *    相位扣减，必然持续积压——2026-11-10 延迟事故的同一种失败模式）。
 *
 * 候选表是 6400 = 2^8 × 5^2 的 2 的幂因数序列，与 IMU 的 ODR 档位同构。
 * 落点：
 *   - 45686@1600 → 2400 下限 → **3200**（2.0×，实飞验证档）
 *   - 42688@2000 → 3000 下限 → **3200**（1.6×；不能取 2×=4000，因为不整除 6400）
 *   - 800 → 1600、400 → 800、1000 → 1600、500 → 800
 *   - 3200 → 6400（2.0×）
 *   - 6400 → 9600 下限，无档可用 → 编译期报错（该档需要更高时基）
 *
 * 历史：2026-11-11 曾把 kIsrPollHz 硬编码为 `2 × kOdrHz` 并断言整除时基。
 * 该写法只对 45686 的档位成立（1600→3200 整除），对 42688 的 2000→4000 不整除，
 * 导致纵列/共轴/单涵道全部 42688 env 编译失败（2026-11-12 修复）。
 * 时间轴由芯片 FIFO 时间戳差分驱动，与 ISR 频率无关；ISR 频率只决定"多久查一次水位"。
 */
constexpr uint32_t kSchedulerTick = scheduler::kSchedulerTickHz;

/** @brief ISR 轮询频率候选（均为 kSchedulerTick 的因数，升序）。 */
constexpr uint16_t kIsrPollCandidates[] = {100, 200, 400, 800, 1600, 3200, 6400};
constexpr size_t kIsrPollCandidateCount =
    sizeof(kIsrPollCandidates) / sizeof(kIsrPollCandidates[0]);

/** @brief 取数余量：ISR 频率至少为 ODR 的 3/2。 */
constexpr uint16_t kIsrRatioNum = 3;
constexpr uint16_t kIsrRatioDen = 2;

/** @brief ISR 频率下限：ceil(1.5 × odrHz)。 */
constexpr uint16_t minIsrPollHz(uint16_t odrHz)
{
  return static_cast<uint16_t>(
      (static_cast<uint32_t>(odrHz) * kIsrRatioNum + kIsrRatioDen - 1U) / kIsrRatioDen);
}

/** @brief 递归取候选表中 ≥ target 的最小档；无解返回 0（由 static_assert 拦截）。 */
constexpr uint16_t selectIsrPollImpl(uint16_t target, size_t i)
{
  return (i >= kIsrPollCandidateCount)
             ? 0
             : ((kIsrPollCandidates[i] >= target)
                    ? kIsrPollCandidates[i]
                    : selectIsrPollImpl(target, i + 1));
}

/** @brief 按 ODR 选 ISR 轮询频率。 */
constexpr uint16_t selectIsrPollHz(uint16_t odrHz)
{
  return selectIsrPollImpl(minIsrPollHz(odrHz), 0);
}

constexpr uint16_t kIsrPollHz = selectIsrPollHz(kOdrHz);

static_assert(kIsrPollHz != 0U,
              "没有可用的 ISR 轮询档：该 ODR 要求的取数频率超过调度时基 6400Hz；"
              "需先提高 kSchedulerTickHz（会影响所有任务周期），不能靠改采样率绕过");
static_assert(kIsrPollHz <= kSchedulerTick,
              "ISR 轮询频率超过调度时基：一拍最多抽一次，超出即持续积压");
static_assert(kSchedulerTick % kIsrPollHz == 0U,
              "ISR 轮询频率必须整除调度时基，否则相位累加出现长周期节奏抖动");
static_assert(kIsrPollHz >= minIsrPollHz(kOdrHz),
              "ISR 轮询频率低于 1.5×ODR：芯片 ODR 有 +5% 晶振偏差，取数次数不足会让"
              "FIFO/ring 稳定积压、控制吃旧数据（2026-11-11 实测）");

} // namespace bfs_imu_odr
