// 共轴控制器宿主机回归（α-native 单一路径，2026-10-31 旧 δ 环移除后）
// 覆盖：零输入、极性、RATE 旁路外环、速率阻尼、手动 TVC、执行机构限幅、
//       偏航权限、NaN、6DOF 简化闭环、杆量曲线、RATE 内环积分。
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "CoaxialAllocator.h"
#include "CoaxialVtvControl.h"
#include "QuaternionMath.h"
#include "RcCurve.h"

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.29577951308232
#endif

namespace
{
int g_fail = 0;

#define CHECK(cond, msg)                                                        \
  do                                                                            \
  {                                                                             \
    if (!(cond))                                                                \
    {                                                                           \
      std::printf("FAIL: %s (line %d)\n", msg, __LINE__);                       \
      g_fail++;                                                                 \
    }                                                                           \
  } while (0)

bool nearf(const float a, const float b, const float tol = 1e-3f)
{
  return std::fabs(a - b) <= tol;
}

// 与固件同源的标称物理量：T=8 N、l=0.15 m、I=0.05/0.08、k_yaw=5e-4 N·m/µs。
constexpr float kLever = 0.15f;
constexpr float kThrust = 8.0f;
constexpr float kIx = 0.05f;
constexpr float kIy = 0.05f;
constexpr float kIz = 0.08f;
constexpr float kYawNmPerUs = 5.0e-4f;
constexpr float kDeg2Rad = 0.017453292519943295f;
const float kGBaseRoll = kLever * kThrust / kIx;      // (rad/s²)/rad
const float kGBasePitch = kLever * kThrust / kIy;
const float kGRoll = kGBaseRoll * kDeg2Rad;           // (rad/s²)/deg
const float kGPitch = kGBasePitch * kDeg2Rad;
const float kGYaw = kYawNmPerUs / kIz;                   // (rad/s²)/µs
const float kAlphaLimit = kGBaseRoll * std::sin(10.0f * kDeg2Rad);
const float kYawLimit = kGYaw * 180.0f;

bfs::CoaxialVtvInput baseInput()
{
  bfs::CoaxialVtvInput in;
  in.attitude_active = true;
  in.attitude_integral_active = true;
  in.rate_integral_active = true;
  in.yaw_active = true;
  in.yaw_integral_active = true;
  in.dt_s = 0.005f;
  in.alpha_per_deg_roll = kGRoll;
  in.alpha_per_deg_pitch = kGPitch;
  in.alpha_per_us_yaw = kGYaw;
  in.alpha_limit_roll = kAlphaLimit;
  in.alpha_limit_pitch = kAlphaLimit;
  in.alpha_limit_yaw = kYawLimit;
  return in;
}
} // namespace

int main()
{
  // T1 零误差/零速率：α 与差速均为 0。
  {
    bfs::CoaxialVtvController c;
    const auto out = c.update(baseInput());
    CHECK(out.valid && out.alpha_valid, "T1 有效");
    CHECK(nearf(out.alpha_roll_radps2, 0.0f) &&
              nearf(out.alpha_pitch_radps2, 0.0f) &&
              nearf(out.alpha_yaw_radps2, 0.0f),
          "T1 零输入零输出");
  }

  // T2 ATTITUDE 串级：正姿态误差 → 正角加速度指令。
  {
    bfs::CoaxialVtvController c;
    auto in = baseInput();
    in.error_roll_deg = 5.0f;
    in.error_pitch_deg = 5.0f;
    const auto out = c.update(in);
    CHECK(out.alpha_roll_radps2 > 0.0f, "T2 roll 极性");
    CHECK(out.alpha_pitch_radps2 > 0.0f, "T2 pitch 极性");
  }

  // T2b RATE_MODE 跳过姿态外环，速率目标直接进内环。
  {
    bfs::CoaxialVtvController c;
    auto in = baseInput();
    in.rate_mode = true;
    in.roll_rate_target_dps = 30.0f;
    in.pitch_rate_target_dps = -20.0f;
    const auto out = c.update(in);
    CHECK(out.alpha_roll_radps2 > 0.0f, "T2b rate roll 极性");
    CHECK(out.alpha_pitch_radps2 < 0.0f, "T2b rate pitch 极性");
  }

  // T3 速率阻尼：零误差、正角速率 → 负 α。
  {
    bfs::CoaxialVtvController c;
    auto in = baseInput();
    in.rate_roll_dps = 20.0f;
    in.rate_pitch_dps = 20.0f;
    const auto out = c.update(in);
    CHECK(out.alpha_roll_radps2 < 0.0f, "T3 roll 速率阻尼");
    CHECK(out.alpha_pitch_radps2 < 0.0f, "T3 pitch 速率阻尼");
  }

  // T4 手动 TVC：仍输出 δ（alpha_valid=false），首拍 α0.25 滤波。
  {
    bfs::CoaxialVtvController c;
    auto in = baseInput();
    in.manual_tvc = true;
    in.manual_roll_deg = 4.0f;
    in.manual_pitch_deg = -3.0f;
    const auto first = c.update(in);
    CHECK(!first.alpha_valid, "T4 手动不走 α");
    CHECK(nearf(first.roll_correction_deg, 1.0f), "T4 手动 roll 首拍 1.0");
    CHECK(nearf(first.pitch_correction_deg, -0.75f), "T4 手动 pitch 首拍 -0.75");
    bfs::CoaxialVtvOutput out = first;
    for (int i = 0; i < 40; ++i)
    {
      out = c.update(in);
    }
    CHECK(nearf(out.roll_correction_deg, 4.0f), "T4 手动 roll 收敛");
    CHECK(nearf(out.pitch_correction_deg, -3.0f), "T4 手动 pitch 收敛");
    CHECK(nearf(out.yaw_diff_us, 0.0f), "T4 手动 yaw=0");
    auto normal = baseInput();
    normal.error_roll_deg = 5.0f;
    const auto out2 = c.update(normal);
    CHECK(out2.alpha_valid && out2.alpha_roll_radps2 >= 0.0f,
          "T4 切回自动可恢复");
  }

  // T5 执行机构全行程饱和：α 钳到 g·sin(10°)，饱和位。
  {
    bfs::CoaxialVtvController c;
    auto in = baseInput();
    in.error_roll_deg = 1000.0f;
    in.error_pitch_deg = 1000.0f;
    bfs::CoaxialVtvOutput out;
    for (int i = 0; i < 200; ++i)
    {
      out = c.update(in);
    }
    CHECK(nearf(out.alpha_roll_radps2, kAlphaLimit, 1e-4f), "T5 roll α 限幅");
    CHECK(nearf(out.alpha_pitch_radps2, kAlphaLimit, 1e-4f), "T5 pitch α 限幅");
    CHECK(out.saturated, "T5 饱和标记");
  }

  // T6 偏航：正/负目标符号正确，超权限被 ±k_yaw·180 限幅。
  {
    bfs::CoaxialVtvController c;
    auto in = baseInput();
    in.yaw_rate_target_dps = 30.0f;
    bfs::CoaxialVtvOutput out;
    for (int i = 0; i < 20; ++i)
    {
      out = c.update(in);
    }
    CHECK(out.alpha_yaw_radps2 > 0.0f, "T6 正偏航目标");
    in.yaw_rate_target_dps = -30.0f;
    for (int i = 0; i < 20; ++i)
    {
      out = c.update(in);
    }
    CHECK(out.alpha_yaw_radps2 < 0.0f, "T6 负偏航目标");
    in.yaw_rate_target_dps = 1000.0f;
    for (int i = 0; i < 200; ++i)
    {
      out = c.update(in);
    }
    CHECK(std::fabs(out.alpha_yaw_radps2) <= kYawLimit + 1e-5f, "T6 偏航限幅");
  }

  // T7 非有限输入：invalid 且输出回零。
  {
    bfs::CoaxialVtvController c;
    auto in = baseInput();
    in.error_roll_deg = std::nanf("");
    const auto out = c.update(in);
    CHECK(!out.valid && !out.alpha_valid, "T7 NaN 无效");
    CHECK(nearf(out.alpha_roll_radps2, 0.0f) &&
              nearf(out.alpha_yaw_radps2, 0.0f),
          "T7 NaN 回零");
  }

  // T8 简化 6DOF 闭环：α 直接作为力矩（M=I·α，小角精确）。
  {
    bfs::CoaxialVtvController c;
    const double dt = 0.005;
    const double m = 1.5;
    const double g = 9.80665;
    Quaternion q = eulerToQuaternion(5.0 * DEG_TO_RAD, 3.0 * DEG_TO_RAD, 0.0);
    Vector3 omega{0.0, 0.0, 10.0 * DEG_TO_RAD};
    Vector3 vel{0.0, 0.0, 0.0};
    Vector3 pos{0.0, 0.0, 0.0};
    const double Ix = kIx, Iy = kIy, Iz = kIz;
    const double lever = kLever;
    const double thrust_n = m * g;  // T8 悬停推力，避免持续掉高
    const Quaternion q_target; // identity

    double max_abs_roll = 0.0, max_abs_pitch = 0.0;
    for (int i = 0; i < 2000; ++i)
    {
      const Quaternion q_err =
          quaternionMultiply(quaternionConjugate(q), q_target);
      const float sign_qw = (q_err.w >= 0.0) ? 1.0f : -1.0f;
      const float qv = std::sqrt(static_cast<float>(q_err.x * q_err.x +
                                                   q_err.y * q_err.y +
                                                   q_err.z * q_err.z));
      const float scale =
          (qv > 0.25f)
              ? 2.0f * std::atan2(qv, std::fabs(static_cast<float>(q_err.w))) /
                    qv * static_cast<float>(RAD_TO_DEG)
              : 2.0f * static_cast<float>(RAD_TO_DEG);
      bfs::CoaxialVtvInput in = baseInput();
      in.dt_s = static_cast<float>(dt);
      in.error_roll_deg = sign_qw * static_cast<float>(q_err.x) * scale;
      in.error_pitch_deg = sign_qw * static_cast<float>(q_err.y) * scale;
      in.rate_roll_dps = static_cast<float>(omega.x * RAD_TO_DEG);
      in.rate_pitch_dps = static_cast<float>(omega.y * RAD_TO_DEG);
      in.rate_yaw_dps = static_cast<float>(omega.z * RAD_TO_DEG);
      in.yaw_rate_target_dps = 0.0f;
      const auto out = c.update(in);
      if (!out.alpha_valid)
      {
        std::printf("FAIL: T8 controller invalid\n");
        g_fail++;
        break;
      }

      const double mx = Ix * out.alpha_roll_radps2;
      const double my = Iy * out.alpha_pitch_radps2;
      const double mz = Iz * out.alpha_yaw_radps2;

      // 经典 RK4（2026-10-31 用户要求：仿真不用 Euler）。控制器输出在本拍内保持，
      // 刚体转动/四元数/平动都用四 stage 积分，q 每步归一。
      struct RkState
      {
        double wx, wy, wz;
        Quaternion q;
        double vx, vy, vz;
        double px, py, pz;
      };
      struct RkDeriv
      {
        double dwx, dwy, dwz;
        Quaternion dq;
        double dvx, dvy, dvz;
        double dpx, dpy, dpz;
      };
      auto deriv = [&](const RkState &st) {
        RkDeriv d;
        const double wx = st.wx, wy = st.wy, wz = st.wz;
        d.dwx = (mx - (Iz - Iy) * wy * wz) / Ix;
        d.dwy = (my - (Ix - Iz) * wz * wx) / Iy;
        d.dwz = (mz - (Iy - Ix) * wx * wy) / Iz;
        d.dq = quaternionMultiply(
            st.q, Quaternion(0.0, 0.5 * wx, 0.5 * wy, 0.5 * wz));
        const Vector3 f_body{0.0, 0.0, -thrust_n};
        const Vector3 f_ned = rotateVector(st.q, f_body);
        d.dvx = f_ned.x / m;
        d.dvy = f_ned.y / m;
        d.dvz = f_ned.z / m + g;
        d.dpx = st.vx;
        d.dpy = st.vy;
        d.dpz = st.vz;
        return d;
      };
      auto addStage = [&](const RkState &st, const RkDeriv &d, double h) {
        RkState o = st;
        o.wx += d.dwx * h;
        o.wy += d.dwy * h;
        o.wz += d.dwz * h;
        o.q.w += d.dq.w * h;
        o.q.x += d.dq.x * h;
        o.q.y += d.dq.y * h;
        o.q.z += d.dq.z * h;
        o.q = normalizeQuaternion(o.q);
        o.vx += d.dvx * h;
        o.vy += d.dvy * h;
        o.vz += d.dvz * h;
        o.px += d.dpx * h;
        o.py += d.dpy * h;
        o.pz += d.dpz * h;
        return o;
      };

      RkState st0{omega.x, omega.y, omega.z, q,
                  vel.x, vel.y, vel.z, pos.x, pos.y, pos.z};
      const RkDeriv k1 = deriv(st0);
      const RkDeriv k2 = deriv(addStage(st0, k1, 0.5 * dt));
      const RkDeriv k3 = deriv(addStage(st0, k2, 0.5 * dt));
      const RkDeriv k4 = deriv(addStage(st0, k3, dt));

      RkState st1 = st0;
      const double w = dt / 6.0;
      st1.wx += w * (k1.dwx + 2 * k2.dwx + 2 * k3.dwx + k4.dwx);
      st1.wy += w * (k1.dwy + 2 * k2.dwy + 2 * k3.dwy + k4.dwy);
      st1.wz += w * (k1.dwz + 2 * k2.dwz + 2 * k3.dwz + k4.dwz);
      st1.q.w += w * (k1.dq.w + 2 * k2.dq.w + 2 * k3.dq.w + k4.dq.w);
      st1.q.x += w * (k1.dq.x + 2 * k2.dq.x + 2 * k3.dq.x + k4.dq.x);
      st1.q.y += w * (k1.dq.y + 2 * k2.dq.y + 2 * k3.dq.y + k4.dq.y);
      st1.q.z += w * (k1.dq.z + 2 * k2.dq.z + 2 * k3.dq.z + k4.dq.z);
      st1.q = normalizeQuaternion(st1.q);
      st1.vx += w * (k1.dvx + 2 * k2.dvx + 2 * k3.dvx + k4.dvx);
      st1.vy += w * (k1.dvy + 2 * k2.dvy + 2 * k3.dvy + k4.dvy);
      st1.vz += w * (k1.dvz + 2 * k2.dvz + 2 * k3.dvz + k4.dvz);
      st1.px += w * (k1.dpx + 2 * k2.dpx + 2 * k3.dpx + k4.dpx);
      st1.py += w * (k1.dpy + 2 * k2.dpy + 2 * k3.dpy + k4.dpy);
      st1.pz += w * (k1.dpz + 2 * k2.dpz + 2 * k3.dpz + k4.dpz);

      omega.x = st1.wx;
      omega.y = st1.wy;
      omega.z = st1.wz;
      q = st1.q;
      vel.x = st1.vx;
      vel.y = st1.vy;
      vel.z = st1.vz;
      pos.x = st1.px;
      pos.y = st1.py;
      pos.z = st1.pz;

      float roll, pitch, yaw;
      quaternionToEuler(q, roll, pitch, yaw);
      max_abs_roll = std::fmax(max_abs_roll, std::fabs(roll * RAD_TO_DEG));
      max_abs_pitch = std::fmax(max_abs_pitch, std::fabs(pitch * RAD_TO_DEG));
    }
    float roll, pitch, yaw;
    quaternionToEuler(q, roll, pitch, yaw);
    std::printf("T8 final roll=%.3f pitch=%.3f yawrate=%.3f dps pos=(%.2f,%.2f,%.2f)\n",
                roll * RAD_TO_DEG, pitch * RAD_TO_DEG, omega.z * RAD_TO_DEG,
                pos.x, pos.y, pos.z);
    CHECK(std::isfinite(roll) && std::isfinite(pitch) && std::isfinite(yaw),
          "T8 有限");
    CHECK(std::fabs(roll * RAD_TO_DEG) < 2.0f, "T8 滚转收敛");
    CHECK(std::fabs(pitch * RAD_TO_DEG) < 2.0f, "T8 俯仰收敛");
    CHECK(std::fabs(omega.z * RAD_TO_DEG) < 5.0f, "T8 偏航速率收敛");
    CHECK(max_abs_roll < 30.0f && max_abs_pitch < 30.0f, "T8 姿态有界");
    CHECK(std::fabs(pos.z) < 3.0f, "T8 高度有界");
  }

  // T9 杆量→目标角速率曲线（TandemVec rcRateCurve，600 dps 封顶）。
  {
    const float kRate = 0.5f;
    const float kExpo = 0.25f;
    const float kSuper = 0.7f;
    const float kMax = 600.0f;
    CHECK(nearf(bfs::coaxialVtvStickToRateDps(988.0f, kRate, kExpo, kSuper, kMax, false),
                -333.3333f, 0.01f), "T9 988→-333.33 dps");
    CHECK(nearf(bfs::coaxialVtvStickToRateDps(2012.0f, kRate, kExpo, kSuper, kMax, false),
                333.3333f, 0.01f), "T9 2012→+333.33 dps");
    CHECK(nearf(bfs::coaxialVtvStickToRateDps(1500.0f, kRate, kExpo, kSuper, kMax, false),
                0.0f), "T9 中位→0");
    CHECK(nearf(bfs::coaxialVtvStickToRateDps(2012.0f, kRate, kExpo, kSuper, kMax, true),
                -333.3333f, 0.01f), "T9 俯仰反向");
    CHECK(nearf(bfs::coaxialVtvStickToRateDps(1900.0f, kRate, kExpo, kSuper, kMax, false),
                139.2478f, 0.01f), "T9 曲线中点");
    CHECK(nearf(bfs::coaxialVtvStickToRateDps(2100.0f, kRate, kExpo, kSuper, kMax, false),
                600.0f, 0.01f), "T9 越界封顶");
    CHECK(nearf(bfs::coaxialVtvStickToRateDps(1900.0f, kRate, kExpo, 0.55f, kMax, false),
                115.1882f, 0.01f), "T9 偏航 super=0.55");
  }

  // T10 RATE 内环积分跨拍累积：两模式都使能；断开积分时只剩 P。
  {
    bfs::CoaxialVtvController a;
    bfs::CoaxialVtvController b;
    auto in = baseInput();
    in.rate_mode = true;
    in.roll_rate_target_dps = 30.0f;
    in.rate_roll_dps = 0.0f;
    bfs::CoaxialVtvOutput oa;
    bfs::CoaxialVtvOutput ob;
    for (int i = 0; i < 60; ++i)
    {
      oa = a.update(in);
      in.rate_integral_active = false;
      ob = b.update(in);
      in.rate_integral_active = true;
    }
    CHECK(oa.alpha_roll_radps2 > ob.alpha_roll_radps2 + 0.15f,
          "T10 RATE 内环积分持续累积");
    CHECK(oa.alpha_roll_radps2 > kGRoll * 7.5f &&
              oa.alpha_roll_radps2 < kGRoll * 10.2f,
          "T10 积分累积叠加 P 分量（未饱和）");
    CHECK(ob.alpha_roll_radps2 > 0.0f, "T10 无积分仍有 P 响应");
  }

  if (g_fail == 0)
  {
    std::printf("=== test_coaxial_vtv_control: 全部通过 ===\n");
    return 0;
  }
  std::printf("=== test_coaxial_vtv_control: %d 项失败 ===\n", g_fail);
  return 1;
}
