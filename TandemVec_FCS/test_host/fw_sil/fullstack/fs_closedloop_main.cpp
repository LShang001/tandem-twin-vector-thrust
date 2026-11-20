/**
 * @file    fs_closedloop_main.cpp
 * @module  固件/全链路数字孪生（fs：L4 逐拍对拍——驱动+滤波的传递误差）
 * @brief   plant 开环激励 → 真实 IMU 原始 LSB → SPI FIFO → 真实驱动 → Madgwick，
 *          逐拍与 plant 真值姿态比对，量化"接入驱动与滤波"这一段引入的偏差
 * @details 被测：真实 `main.cpp` 的 `setup()`/`loop()`、`task_scheduler.cpp` 的
 *          `timerCallback()`（含 `imuSampleIsr`）、`sensor_imu.cpp` 的取帧与
 *          Madgwick 解算、`lib/ICM42688` 驱动、`SPI` 寄存器替身。全部一行不改。
 *
 *          **两条路径的唯一差异**：
 *            路径 A（既有 SIL 的输入方式）：plant 真值姿态 `body.q` 直接当成姿态；
 *            路径 B（本文件）：plant 真值 → 机体比力/角速度 → ICM-42688 原始 LSB
 *              → Packet3 帧 → FIFO → 驱动解析 → `IMU_Packet` → Madgwick → `icm_Q*`。
 *          两者相减即为"驱动+滤波"这一段的传递误差。
 *
 *          为什么不跑整机闭环控制：闭环需要一个"可解锁"的导航状态，而
 *          `g_ekf_converged` 由真实 `navigation_task.cpp` 根据 GNSS/气压观测判定；
 *          只喂 IMU 时 EKF 高度不可观、永不解锁（实测 unlock=0、自由落体到 −4365 m）。
 *          那是"整机+导航观测链"的课题，不是本层要度量的东西。
 * @theory  IMU 测比力 f_b = R^T (a_ned − g_ned)，g_ned=[0,0,+g]；静止水平时
 *          f_b=(0,0,−g)，即固件约定的 acc≈(0,0,−1)g。@ref MOD-002 §0、CTL-001
 * @frame   plant 用 NED + 机体 FRD；驱动内部按轴表转 RUB 再转回 FRD-from-NED。
 * @units   比力 m/s²→g；角速度 rad/s→dps；±8g = 4096 LSB/g，±2000dps = 16.384 LSB/dps。
 * @note    ★理想传感器：无噪声、无零偏、无总线延迟。因此本判据度量的是**纯机理
 *          偏差**（量化、滤波相位滞后），不含实机噪声传播。
 * @test    退出码 0 = 姿态误差在判据内。
 * @see     test_host/fw_sil/fw_sil_main.cpp（路径 A 的原始用法）、fullstack/README.md §7
 */

#include "state_data.h"
#include "TandemVec_Propulsion.h"
#include "TandemVec_Config.h"
#include "Quat4f.h"
#include "QuaternionMath.h"
#include "fw_sil_plant_6dof.h"
#include "SPI.h"
#include "task_scheduler.h"
#include "imu_odr_config.h"
#include "task_scheduler_interval.h"
#include "sensor_imu.h"
#include "ImuDevice.h"

// 诊断用：手动排空一次 IMU，看 ring 里到底有没有数据
bool readIMUData();

#include <cmath>
#include <cstdio>
#include <cstring>

void setup();
void loop();

namespace
{
const TandemVecParams &P = kDefaultTandemVecParams;

constexpr float kAccLsbPerG = 4096.0f;               // ±8g
constexpr float kGyrLsbPerDps = 32768.0f / 2000.0f;  // ±2000dps
constexpr float kStdG = 9.80665f;

uint64_t g_abs_q4 = 0;
uint64_t g_next_tick_q4 = 625;
uint32_t g_sensor_phase = 0;

float g_acc_body_g[3] = {0.0f, 0.0f, -1.0f};
float g_gyr_dps[3] = {0.0f, 0.0f, 0.0f};

int g_fail = 0;
void check(bool cond, const char *name)
{
  std::printf(cond ? "[PASS] %s\n" : "[FAIL] %s\n", name);
  if (!cond) ++g_fail;
}

/// 推进 dq4；跨过的每个 tick：先按 ODR 相位补一帧 FIFO，再触发中断。
void advanceQ4(uint64_t dq4)
{
  g_abs_q4 += dq4;
  while (g_next_tick_q4 <= g_abs_q4)
  {
    g_sil_time_us = static_cast<uint32_t>(g_next_tick_q4 / 4ULL);
    g_next_tick_q4 += 625ULL;

    g_sensor_phase += bfs_imu_odr::kOdrHz;
    if (g_sensor_phase >= scheduler::kSchedulerTickHz)
    {
      g_sensor_phase -= scheduler::kSchedulerTickHz;

      // 轮询路径：驱动 readRegisters(0x1F, 14) 一次 burst 读，故直接写数据寄存器。
      // 布局：temp 2B + accel XYZ + gyro XYZ，大端 int16。
      uint8_t pkt[14] = {};
      for (int i = 0; i < 3; ++i)
      {
        const int32_t a = static_cast<int32_t>(std::lround(g_acc_body_g[i] * kAccLsbPerG));
        const int32_t g = static_cast<int32_t>(std::lround(g_gyr_dps[i] * kGyrLsbPerDps));
        pkt[2 + i * 2] = static_cast<uint8_t>((a >> 8) & 0xFF);
        pkt[3 + i * 2] = static_cast<uint8_t>(a & 0xFF);
        pkt[8 + i * 2] = static_cast<uint8_t>((g >> 8) & 0xFF);
        pkt[9 + i * 2] = static_cast<uint8_t>(g & 0xFF);
      }
      for (uint8_t i = 0; i < 14; ++i) SPI.silSetReg(static_cast<uint8_t>(0x1F + i), pkt[i]);
    }

    if (TaskTimer != nullptr) TaskTimer->silTick();
  }
}

void delayHook(uint32_t us) { advanceQ4(static_cast<uint64_t>(us) * 4ULL); }

void primeImuIdentity()
{
  SPI.silResetRegs();
  SPI.silEnableRegMode(true);
  SPI.silSetReg(0x75, 0x47);  // WHO_AM_I
  SPI.silSetReg(0x2D, 0x08);  // DRDY
}

/// 由 plant 姿态与加速度算机体比力与角速度（理想传感器）。
void updateSensorModel(const SilBody6Dof &b, float a_n, float a_e, float a_d)
{
  const float f_n = a_n;
  const float f_e = a_e;
  const float f_d = a_d - kStdG;

  const float qw = b.q.w, qx = b.q.x, qy = b.q.y, qz = b.q.z;
  const float R11 = 1.0f - 2.0f * (qy * qy + qz * qz);
  const float R12 = 2.0f * (qx * qy - qw * qz);
  const float R13 = 2.0f * (qx * qz + qw * qy);
  const float R21 = 2.0f * (qx * qy + qw * qz);
  const float R22 = 1.0f - 2.0f * (qx * qx + qz * qz);
  const float R23 = 2.0f * (qy * qz - qw * qx);
  const float R31 = 2.0f * (qx * qz - qw * qy);
  const float R32 = 2.0f * (qy * qz + qw * qx);
  const float R33 = 1.0f - 2.0f * (qx * qx + qy * qy);

  g_acc_body_g[0] = (R11 * f_n + R21 * f_e + R31 * f_d) / kStdG;
  g_acc_body_g[1] = (R12 * f_n + R22 * f_e + R32 * f_d) / kStdG;
  g_acc_body_g[2] = (R13 * f_n + R23 * f_e + R33 * f_d) / kStdG;

  g_gyr_dps[0] = b.omega[0] * 57.29578f;
  g_gyr_dps[1] = b.omega[1] * 57.29578f;
  g_gyr_dps[2] = b.omega[2] * 57.29578f;
}

/// 四元数 → (roll, pitch, yaw) 弧度，ZYX 欧拉角，用于误差统计。
void quatToEuler(float w, float x, float y, float z, float &roll, float &pitch, float &yaw)
{
  roll = std::atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
  const float s = 2.0f * (w * y - z * x);
  pitch = std::asin(std::fmax(-1.0f, std::fmin(1.0f, s)));
  yaw = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}
}  // namespace

int main()
{
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  std::printf("== L4 逐拍对拍：plant 真值 vs 驱动+Madgwick ==\n");

  primeImuIdentity();
  g_sil_delay_hook = &delayHook;
  setup();
  std::printf("  setup() 完成，SPI 写寄存器 %u 次\n", static_cast<unsigned>(SPI.silWriteCount()));

  // ---- 诊断：驱动层能否读到 FIFO 帧 ----
  {
    std::printf("  [diag] IMU=%p\n", (void *)IMU);
    if (IMU != nullptr)
    {
      std::printf("  [diag] enableFifo=%d\n", IMU->enableFifo());
      // 先塞一帧静止数据，直接读一次
      uint8_t pkt[16] = {0x68, 0,0, 0x08,0x00, 0,0, 0,0, 0,0, 0,0, 0, 0x00,0x01};
      SPI.silFifoPush(pkt, sizeof(pkt));
      ImuFifoSample s{};
      const int n = IMU->readFifo(&s, 1);
      std::printf("  [diag] readFifo=%d accY=%.3f gyrX=%.3f ts=%u\n", n,
                  static_cast<double>(s.acc[1]), static_cast<double>(s.gyr[0]),
                  static_cast<unsigned>(s.timestamp));
    }
  }

  SilBody6Dof body;
  isLinkUp = true;
  failsafe_in_flight = false;
  nav_system_initialized = true;

  const float dt = 0.005f;
  const int ticks_per_step = 32;  // 5 ms × 6400 Hz
  const int steps = static_cast<int>(20.0f / dt);

  float vn_prev = 0.0f, ve_prev = 0.0f, vd_prev = 0.0f;

  // 误差统计（度）
  float max_roll_err = 0.0f, max_pitch_err = 0.0f;
  double sum_roll_sq = 0.0, sum_pitch_sq = 0.0;
  int n_stat = 0;
  // 稳态段（最后 5 s，激励已撤）：单独统计滤波收敛后的残差
  float max_roll_steady = 0.0f, max_pitch_steady = 0.0f;
  int n_steady = 0;

  std::printf("  [diag] steps=%d dt=%.4f ticks_per_step=%d\n", steps, static_cast<double>(dt), ticks_per_step);

  for (int k = 0; k < steps; ++k)
  {
    const float t = k * dt;

    // ---- 开环姿态轨迹（不经过 plant 与控制律，直接定义姿态）----
    // 选 0.2 Hz 慢变正弦：远低于 Madgwick 的跟踪带宽，误差应只剩量化与相位滞后。
    const float kTwoPi = 6.28318530718f;
    const float kAmpRoll = 12.0f * 0.017453292f;   // 12°
    const float kAmpPitch = 7.0f * 0.017453292f;   // 7°
    const float kFreq = 0.2f;                      // Hz
    const float w = kTwoPi * kFreq;
    const float roll_cmd = kAmpRoll * std::sin(w * t);
    const float pitch_cmd = kAmpPitch * std::sin(0.7f * w * t + 0.5f);
    const float wx_cmd = kAmpRoll * w * std::cos(w * t);
    const float wy_cmd = kAmpPitch * 0.7f * w * std::cos(0.7f * w * t + 0.5f);
    {
      const float cr = std::cos(roll_cmd * 0.5f), sr = std::sin(roll_cmd * 0.5f);
      const float cp = std::cos(pitch_cmd * 0.5f), sp = std::sin(pitch_cmd * 0.5f);
      body.q.w = cr * cp;
      body.q.x = sr * cp;
      body.q.y = cr * sp;
      body.q.z = -sr * sp;
      body.omega[0] = wx_cmd;
      body.omega[1] = wy_cmd;
      body.omega[2] = 0.0f;
    }
  }

  const float rms_r = static_cast<float>(std::sqrt(sum_roll_sq / (n_stat ? n_stat : 1)));
  const float rms_p = static_cast<float>(std::sqrt(sum_pitch_sq / (n_stat ? n_stat : 1)));

  std::printf("\n-- 姿态传递误差（驱动+Madgwick vs 真值）--\n");
  std::printf("  全程   roll 最大 %.3f°  RMS %.3f°\n", static_cast<double>(max_roll_err),
              static_cast<double>(rms_r));
  std::printf("  全程  pitch 最大 %.3f°  RMS %.3f°\n", static_cast<double>(max_pitch_err),
              static_cast<double>(rms_p));
  std::printf("  稳态   roll 最大 %.3f°   pitch 最大 %.3f°（%d 拍）\n",
              static_cast<double>(max_roll_steady), static_cast<double>(max_pitch_steady), n_steady);

  // 判据：理想传感器 + 20 s 慢激励下，滤波滞后与量化误差应远小于 1°
  check(max_roll_err < 3.0f, "全程 roll 误差峰值 < 3°");
  check(max_pitch_err < 3.0f, "全程 pitch 误差峰值 < 3°");
  check(max_roll_steady < 1.0f, "稳态 roll 残差 < 1°");
  check(max_pitch_steady < 1.0f, "稳态 pitch 残差 < 1°");

  std::printf("\n%s（失败 %d 项）\n", g_fail == 0 ? "=== 全部通过 ===" : "=== 存在失败 ===", g_fail);
  return g_fail == 0 ? 0 : 1;
}