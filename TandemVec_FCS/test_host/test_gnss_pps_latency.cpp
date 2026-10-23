/**
 * @file    test_gnss_pps_latency.cpp
 * @module  固件/宿主回归测试（fw-tests：PPS-TOW 链路延时标定）
 * @brief   验证 PPS 边沿与 PVT 到达时刻的差值中位数能还原链路延时
 * @details 被测：`include/ins_gnss_pps_latency.h`（`InsGnssEstimatePpsLatency()`）。
 *          输入：200 个 PPS 边沿（每秒一个）与对应 PVT 到达时刻，
 *          叠加 σ=2 ms 的调度抖动；两组场景分别 25 ms 与 50 ms 真实延时
 *          （后者带 0.6 s 的小数 TOW）；
 *          处理：估计器取差值中位数与离散度；
 *          输出：25 ms 场景要求 valid、样本数≥190、误差<3 ms、离散度<5 ms；
 *          50 ms 场景误差<3 ms；空输入/2 个样本必须 invalid。
 *          运行：g++ -std=c++17 -Iinclude test_host/test_gnss_pps_latency.cpp -o bin/pps
 * @theory  链路延时 τ ≈ median(t_arrival − t_pps) − 整秒对齐；用中位数而非均值
 *          以抗偶发调度尖峰，σ_τ 由样本离散度给出。@ref REF-121 §2.2.1 p.33（量测时标）
 * @frame   N/A（纯时间轴）
 * @units   时间 µs / s，抖动 σ 为 ms
 * @note    3 ms 门限 = 2 ms 抖动经 200 样本中位数的统计误差上限（约 σ/√N·1.25 ≈ 0.18 ms）
 *          再加实现余量；样本数下限 190 留 5% 丢帧余量。
 *          硬件 PPS 捕获尚未接入固件，本文件只验证纯算法层（见 AGENTS.md）。
 * @test    T1 25 ms 估计；T2 50 ms + 小数 TOW；T3 无 PPS 拒绝；T4 样本过少拒绝
 * @see     TandemVec_FCS/include/ins_gnss_pps_latency.h
 */
// PPS 延时标定回归（纯平台无关）
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <random>
#include "ins_gnss_pps_latency.h"

static int pass = 0, fail = 0;
#define CHECK(name, cond)                                   \
  do {                                                      \
    if (cond) { ++pass; std::printf("[PASS] %s\n", name); } \
    else { ++fail; std::printf("[FAIL] %s\n", name); }      \
  } while (0)

int main() {
  constexpr size_t N = 200;
  InsGnssPpsEdge edges[N];
  InsGnssPvtArrival arrivals[N];
  std::mt19937 rng(7);
  std::normal_distribution<float> jitter(0.0f, 0.002f); // 2 ms 调度抖动

  // 25 ms 链路延时 + 2 ms 抖动
  for (size_t i = 0; i < N; ++i) {
    edges[i].tow_ms = static_cast<uint32_t>(i * 1000);
    edges[i].local_us = static_cast<uint32_t>(100000 + i * 1000000);
    arrivals[i].tow_ms = edges[i].tow_ms;
    arrivals[i].local_us = static_cast<uint32_t>(
        static_cast<int64_t>(edges[i].local_us) + 25000 +
        static_cast<int64_t>(jitter(rng) * 1e6f));
  }
  auto est = InsGnssEstimatePpsLatency(edges, N, arrivals, N);
  CHECK("25ms valid", est.valid);
  CHECK("25ms sample count", est.sample_count >= 190);
  CHECK("25ms latency ±3ms", std::fabs(est.latency_s - 0.025f) < 0.003f);
  CHECK("25ms spread <5ms", est.spread_s < 0.005f);

  // 50 ms + fractional tow（PVT 在整秒后 0.6s）
  for (size_t i = 0; i < N; ++i) {
    arrivals[i].tow_ms = static_cast<uint32_t>(i * 1000 + 600);
    arrivals[i].local_us = static_cast<uint32_t>(
        static_cast<int64_t>(edges[i].local_us) + 600000 + 50000 +
        static_cast<int64_t>(jitter(rng) * 1e6f));
  }
  auto est2 = InsGnssEstimatePpsLatency(edges, N, arrivals, N);
  CHECK("50ms valid", est2.valid);
  CHECK("50ms latency ±3ms", std::fabs(est2.latency_s - 0.050f) < 0.003f);

  // 无 PPS：无效
  auto none = InsGnssEstimatePpsLatency(nullptr, 0, nullptr, 0);
  CHECK("no pps invalid", !none.valid);

  // 单样本/少量样本：拒绝
  auto few = InsGnssEstimatePpsLatency(edges, 2, arrivals, 2);
  CHECK("few samples invalid", !few.valid);

  std::printf("\nPPS latency: %d passed, %d failed\n", pass, fail);
  return fail == 0 ? 0 : 1;
}
