/**
 * @file    sil_sensor_model.h
 * @module  固件/宿主 SIL（fw-tests：传感器真实度与 UBX 帧生成）
 * @brief   为真实固件 SIL 提供 2 kHz IMU 误差模型、抗混叠振动和 UBX NAV-PVT/EOE 帧
 * @details 被测：无（本文件是被测固件的外部传感器模型）。
 *          输入：plant 真值角速率/比力、振动幅值/频率、IMU 配置；
 *          处理：白噪声（按噪声密度×等效带宽）→ 零偏不稳定性（一阶 Gauss-Markov）
 *          → 振动（多频/单频，可选 ODR 前抗混叠衰减）→ 尺度/安装失准 →
 *          16 bit 量化 → 量程饱和；
 *          输出：`ImuSample`（FRD 机体系 rad/s、m/s²）与 UBX 二进制帧。
 * @theory  ICM-42688 标称噪声密度、16 bit 量化；振动混叠按连续域 8 阶 Butterworth
 *          （fc=80 Hz）幅频响应先衰减再采样，避免 199.9 Hz 单频伪装成准直流。
 *          零偏不稳定性用 OU 过程 `db=-b·dt/τ+σ·sqrt(1-e^(-2dt/τ))·N(0,1)`。
 * @frame   FRD 机体系（x 前 / y 右 / z 下）；NED 真值由调用方转换后传入。
 * @units   角速率 rad/s，比力 m/s²，振动 g，频率 Hz，时间 s。
 * @note    UBX 帧布局按 u-blox M8 NAV-PVT(92 B)/NAV-EOE(4 B) 固定偏移生成；
 *          解析正确性由真实 `lib/ublox-main` parser 验证，不在本文件重复实现。
 * @see     TandemVec_FCS/test_host/fw_sil/fw_sil_env_main.cpp、
 *          TandemVec_FCS/test_host/fw_sil/README.md
 */
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <random>

namespace sil
{

constexpr float kGravity = 9.79f;  // 与 TandemVec_Config.h 一致（项目唯一事实源）

/* 8 阶 Butterworth 幅频响应；用于模拟 IMU 模拟前端在 ODR 前的抗混叠衰减。
 * SIL 的 2 kHz ODR 对应 fc≈0.4·ODR=800 Hz；200 Hz MC 台才用 80 Hz。 */
inline float AntiAliasGain(float f_hz, float fc_hz = 800.0f, int order = 8)
{
  const float r = f_hz / fc_hz;
  return 1.0f / std::sqrt(1.0f + std::pow(r, 2.0f * static_cast<float>(order)));
}

struct ImuConfig
{
  float gyro_noise_density_dps = 0.0035f;  // dps/√Hz
  float accel_noise_density_ug = 70.0f;    // µg/√Hz
  float gyro_bias_dps[3] = {0.17f, -0.11f, 0.23f};
  float accel_bias_mg[3] = {5.1f, -3.1f, 8.2f};
  float gyro_bias_instability_dps = 0.01f;
  float accel_bias_instability_mg = 0.5f;
  float bias_tau_s = 200.0f;
  float gyro_scale = 0.002f;
  float accel_scale = 0.005f;
  float misalign = 0.0015f;
  float gyro_vib_dps = 0.8f;  // 80 Hz 陀螺振动耦合幅值（dps，随 vib_g 缩放）
  float full_scale_gyro_dps = 2000.0f;
  float full_scale_accel_g = 16.0f;
  float effective_bandwidth_hz = 100.0f;  // 噪声密度折算到输出带宽
  // 抗混叠截止频率应按 ODR 取（2 kHz ODR → 0.4·ODR=800 Hz），
  // 不能沿用 200 Hz MC 台的 80 Hz，否则会把 120 Hz 振动误衰减成 0.04。
  float anti_alias_cutoff_hz = 800.0f;
};

struct ImuSample
{
  float accel[3] = {0.0f, 0.0f, 0.0f};
  float gyro[3] = {0.0f, 0.0f, 0.0f};
};

class ImuModel
{
public:
  ImuModel() = default;
  explicit ImuModel(const ImuConfig &cfg, uint32_t seed)
      : cfg_(cfg), rng_(seed) { Reset(); }

  void Reset()
  {
    for (int i = 0; i < 3; ++i)
    {
      inst_gyro_[i] = cfg_.gyro_bias_instability_dps * 0.017453292f;
      inst_accel_[i] = cfg_.accel_bias_instability_mg * 0.001f * kGravity;
    }
    initialized_ = true;
  }

  /*
   * 生成一个 2 kHz IMU 样本。
   * @param true_omega_body 真值角速率 [rad/s]
   * @param true_accel_body 真值比力 [m/s²]（不含重力，地面静止时为 (0,0,-g)）
   * @param t_s 当前仿真时刻 [s]
   * @param dt_s 采样周期 [s]
   * @param vib_g 振动幅值 [g]（0 表示无振动）
   * @param vib_hz 单频振动频率 [Hz]；vib_mode=1 时忽略
   * @param vib_mode 0=单频；1=40/80/120 Hz 多频
   * @param anti_alias 是否在采样前按 Butterworth 幅频响应衰减
   */
  ImuSample Step(const float true_omega_body[3], const float true_accel_body[3],
                 float t_s, float dt_s, float vib_g, float vib_hz,
                 int vib_mode, bool anti_alias)
  {
    if (!initialized_) Reset();
    std::normal_distribution<float> n01(0.0f, 1.0f);

    // 零偏不稳定性：一阶 Gauss-Markov（离散精确解）。
    const float a = std::exp(-dt_s / cfg_.bias_tau_s);
    const float q = std::sqrt(std::max(0.0f, 1.0f - a * a));
    for (int i = 0; i < 3; ++i)
    {
      inst_gyro_[i] = a * inst_gyro_[i] +
                      q * cfg_.gyro_bias_instability_dps * 0.017453292f * n01(rng_);
      inst_accel_[i] = a * inst_accel_[i] +
                       q * cfg_.accel_bias_instability_mg * 0.001f * kGravity * n01(rng_);
    }

    const float gyro_sigma =
        cfg_.gyro_noise_density_dps * 0.017453292f *
        std::sqrt(cfg_.effective_bandwidth_hz);
    const float accel_sigma =
        cfg_.accel_noise_density_ug * 1.0e-6f * kGravity *
        std::sqrt(cfg_.effective_bandwidth_hz);

    float g[3], a_mps2[3];
    for (int i = 0; i < 3; ++i)
    {
      g[i] = true_omega_body[i] +
             cfg_.gyro_bias_dps[i] * 0.017453292f + inst_gyro_[i] +
             gyro_sigma * n01(rng_);
      a_mps2[i] = true_accel_body[i] +
                  cfg_.accel_bias_mg[i] * 0.001f * kGravity + inst_accel_[i] +
                  accel_sigma * n01(rng_);
    }

    // 振动：按幅频响应先衰减再叠加（数字滤波对已采样假直流失效）。
    float vib = 0.0f;
    if (vib_mode == 1)
    {
      const float w = 2.0f * 3.14159265f * t_s;
      float a40 = 0.70f, a80 = 0.45f, a120 = 0.25f;
      if (anti_alias)
      {
        a40 *= AntiAliasGain(40.0f, cfg_.anti_alias_cutoff_hz);
        a80 *= AntiAliasGain(80.0f, cfg_.anti_alias_cutoff_hz);
        a120 *= AntiAliasGain(120.0f, cfg_.anti_alias_cutoff_hz);
      }
      vib = vib_g * kGravity * (a40 * std::sin(w * 40.0f) +
                                a80 * std::sin(w * 80.0f + 1.1f) +
                                a120 * std::sin(w * 120.0f + 2.2f));
    }
    else
    {
      float amp = vib_g * kGravity;
      if (anti_alias) amp *= AntiAliasGain(vib_hz, cfg_.anti_alias_cutoff_hz);
      vib = amp * std::sin(2.0f * 3.14159265f * vib_hz * t_s);
    }
    a_mps2[0] += vib;
    a_mps2[2] += 0.5f * vib;
    const float gyro_vib = cfg_.gyro_vib_dps * vib_g * 0.017453292f *
                           std::sin(2.0f * 3.14159265f * 80.0f * t_s);
    g[0] += 0.3f * gyro_vib;
    g[1] += 0.2f * gyro_vib;
    g[2] += gyro_vib;

    // 尺度/安装失准 + 16 bit 量化 + 量程饱和。
    // gyro_scale/accel_scale 是相对误差（0.002=+0.2%），不是绝对增益。
    const float m = cfg_.misalign;
    const float gs = 1.0f + cfg_.gyro_scale;
    const float as = 1.0f + cfg_.accel_scale;
    const float gm[3] = {
        gs * g[0] + m * g[1] - m * g[2],
        -m * g[0] + gs * g[1] + m * g[2],
        m * g[0] - m * g[1] + gs * g[2]};
    const float am[3] = {
        as * a_mps2[0] + m * a_mps2[1] - m * a_mps2[2],
        -m * a_mps2[0] + as * a_mps2[1] + m * a_mps2[2],
        m * a_mps2[0] - m * a_mps2[1] + as * a_mps2[2]};

    const float gyro_lsb = 2.0f * cfg_.full_scale_gyro_dps /
                           65536.0f * 0.017453292f;
    const float accel_lsb = 2.0f * cfg_.full_scale_accel_g * kGravity /
                            65536.0f;
    const float gyro_limit = cfg_.full_scale_gyro_dps * 0.017453292f;
    const float accel_limit = cfg_.full_scale_accel_g * kGravity;

    ImuSample out;
    for (int i = 0; i < 3; ++i)
    {
      float gv = std::round(gm[i] / gyro_lsb) * gyro_lsb;
      float av = std::round(am[i] / accel_lsb) * accel_lsb;
      gv = std::fmax(-gyro_limit, std::fmin(gyro_limit, gv));
      av = std::fmax(-accel_limit, std::fmin(accel_limit, av));
      out.gyro[i] = gv;
      out.accel[i] = av;
    }
    return out;
  }

private:
  ImuConfig cfg_{};
  std::mt19937 rng_{1};
  float inst_gyro_[3] = {0.0f, 0.0f, 0.0f};
  float inst_accel_[3] = {0.0f, 0.0f, 0.0f};
  bool initialized_ = false;
};

/* ---------------- UBX 帧生成（NAV-PVT / NAV-EOE） ---------------- */

struct UbxPvtFields
{
  uint32_t itow = 0;
  int16_t year = 2026;
  uint8_t month = 10;
  uint8_t day = 24;
  uint8_t hour = 0;
  uint8_t minute = 0;
  uint8_t second = 0;
  uint8_t valid = 0x07;   // validDate|validTime|fullyResolved
  uint8_t fix_type = 3;   // 3D
  uint8_t flags = 0x01;   // gnssFixOK
  uint8_t flags2 = 0xE0;  // confirmed date/time/avail
  uint8_t num_sv = 14;
  int32_t lon_1e7 = 0;
  int32_t lat_1e7 = 0;
  int32_t height_mm = 0;
  int32_t h_msl_mm = 0;
  uint32_t h_acc_mm = 1000;
  uint32_t v_acc_mm = 1500;
  int32_t vel_n_mmps = 0;
  int32_t vel_e_mmps = 0;
  int32_t vel_d_mmps = 0;
  int32_t g_speed_mmps = 0;
  uint32_t s_acc_mmps = 100;
  uint16_t p_dop_1e2 = 120;
};

inline void PutU1(uint8_t *&p, uint8_t v) { *p++ = v; }
inline void PutU2(uint8_t *&p, uint16_t v)
{
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p += 2;
}
inline void PutU4(uint8_t *&p, uint32_t v)
{
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
  p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
  p += 4;
}
inline void PutI4(uint8_t *&p, int32_t v) { PutU4(p, static_cast<uint32_t>(v)); }
inline void PutI2(uint8_t *&p, int16_t v) { PutU2(p, static_cast<uint16_t>(v)); }

inline size_t FinishUbxFrame(uint8_t cls, uint8_t id,
                             const uint8_t *payload, uint16_t len,
                             uint8_t *out, size_t cap)
{
  const size_t total = 8U + len;
  if (out == nullptr || cap < total) return 0U;
  size_t w = 0;
  out[w++] = 0xB5;
  out[w++] = 0x62;
  out[w++] = cls;
  out[w++] = id;
  out[w++] = static_cast<uint8_t>(len & 0xFF);
  out[w++] = static_cast<uint8_t>((len >> 8) & 0xFF);
  if (len > 0U && payload != nullptr)
  {
    std::memcpy(&out[w], payload, len);
    w += len;
  }
  uint8_t ck_a = 0, ck_b = 0;
  for (size_t i = 2; i < w; ++i)
  {
    ck_a = static_cast<uint8_t>(ck_a + out[i]);
    ck_b = static_cast<uint8_t>(ck_b + ck_a);
  }
  out[w++] = ck_a;
  out[w++] = ck_b;
  return w;
}

/* 生成 UBX-NAV-PVT（class=0x01 id=0x07 len=92），返回帧长度。 */
inline size_t BuildNavPvt(const UbxPvtFields &f, uint8_t *out, size_t cap)
{
  uint8_t payload[92] = {};
  uint8_t *p = payload;
  PutU4(p, f.itow);
  PutU2(p, static_cast<uint16_t>(f.year));
  PutU1(p, f.month);
  PutU1(p, f.day);
  PutU1(p, f.hour);
  PutU1(p, f.minute);
  PutU1(p, f.second);
  PutU1(p, f.valid);
  PutU4(p, 30000000U);           // tAcc 30 ms
  PutI4(p, 0);                   // nano
  PutU1(p, f.fix_type);
  PutU1(p, f.flags);
  PutU1(p, f.flags2);
  PutU1(p, f.num_sv);
  PutI4(p, f.lon_1e7);
  PutI4(p, f.lat_1e7);
  PutI4(p, f.height_mm);
  PutI4(p, f.h_msl_mm);
  PutU4(p, f.h_acc_mm);
  PutU4(p, f.v_acc_mm);
  PutI4(p, f.vel_n_mmps);
  PutI4(p, f.vel_e_mmps);
  PutI4(p, f.vel_d_mmps);
  PutI4(p, f.g_speed_mmps);
  PutI4(p, 0);                   // headMot
  PutU4(p, f.s_acc_mmps);
  PutU4(p, 500000U);             // headAcc 5 deg
  PutU2(p, f.p_dop_1e2);
  PutU2(p, 0);                   // flags3
  PutU4(p, 0);                   // reserved0[0..3]
  PutI4(p, 0);                   // headVeh
  PutI2(p, 0);                   // magDec
  PutU2(p, 0);                   // magAcc
  return FinishUbxFrame(0x01, 0x07, payload, sizeof(payload), out, cap);
}

/* 生成 UBX-NAV-EOE（class=0x01 id=0x61 len=4），返回帧长度。 */
inline size_t BuildNavEoe(uint32_t itow, uint8_t *out, size_t cap)
{
  uint8_t payload[4];
  uint8_t *p = payload;
  PutU4(p, itow);
  return FinishUbxFrame(0x01, 0x61, payload, sizeof(payload), out, cap);
}

} // namespace sil
