/**
 * @file    ins_param_align.h
 * @brief   书本参数辨识法精对准（秦永元《惯性导航》§9.9.2）。
 *
 * 纯平台无关（不依赖 Arduino/STM32），宿主机可直接回归。
 *
 * 原理：
 *   静基座下速度误差是时间的三次曲线：
 *     ΔV_E = a1E·t + a2E·t² + a3E·t³
 *     ΔV_N = a1N·t + a2N·t² + a3N·t³
 *   用最小二乘拟合 a1/a2/a3 后反解失准角初值与等效漂移：
 *     φ_E0 = a1N/g,  φ_N0 = −a1E/g
 *     u_E = 2a2N/g,  u_N = −2a2E/g,
 *     u_U = −6a3N/(g·ω·cosL) − (2a2E/g)·tanL
 *     φ_U0 = φ_N0·tanL − u_E/(ω·cosL)     （无 ε_E 修正，误差 ε_E/(ω·cosL)）
 *   双位置（绕 z 转 90°）可测 ε_IN = ε_IIE，进而修正 φ_U0（式 9.9.43c/9.9.49）。
 *
 * 用法：
 *   1) 上位在静止确认后每个导航周期调用 Update(t, 速度东/北)；
 *   2) Solve() 输出失准角、u 与双位置修正量；
 *   3) 用 Solve 结果对姿态阵做一次性修正（调用方负责）。
 */
#ifndef INS_PARAM_ALIGN_H
#define INS_PARAM_ALIGN_H

#include <cmath>
#include <cstdint>

class InsParamAlign
{
public:
  static constexpr int kMaxSamples = 1200;  // 600 s @ 2 Hz

  struct Sample
  {
    float t_s = 0.0f;
    float dv_e_mps = 0.0f;
    float dv_n_mps = 0.0f;
  };

  struct Result
  {
    bool valid = false;
    int sample_count = 0;
    float phi_e_rad = 0.0f;  // 东向失准角
    float phi_n_rad = 0.0f;  // 北向失准角
    float phi_u_rad = 0.0f;  // 方位失准角（无 ε_E 修正）
    float u_e = 0.0f;
    float u_n = 0.0f;
    float u_u = 0.0f;
    float a[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};  // a1E,a2E,a3E,a1N,a2N,a3N
    float fit_residual_rms = 0.0f;  // 拟合残差（m/s）
  };

  void Reset()
  {
    count_ = 0;
  }

  int count() const { return count_; }
  bool full() const { return count_ >= kMaxSamples; }

  /// 追加一个静止速度误差样本（t 为对准开始后的秒数）。
  bool Update(const float t_s, const float dv_e_mps, const float dv_n_mps)
  {
    if (count_ >= kMaxSamples || !std::isfinite(t_s) ||
        !std::isfinite(dv_e_mps) || !std::isfinite(dv_n_mps) || t_s <= 0.0f)
    {
      return false;
    }
    samples_[count_].t_s = t_s;
    samples_[count_].dv_e_mps = dv_e_mps;
    samples_[count_].dv_n_mps = dv_n_mps;
    ++count_;
    return true;
  }

  /**
   * 解算 a1/a2/a3 并反解失准角。
   * @param lat_rad   对准点纬度
   * @param g_mps2    当地重力
   * @param omega_ie  地球自转角速度（rad/s）
   * @param min_samples 最小样本数（默认 120 = 60 s @2 Hz）
   */
  Result Solve(const float lat_rad, const float g_mps2,
               const float omega_ie = 7.292115e-5f,
               const int min_samples = 120) const
  {
    Result r;
    if (count_ < min_samples || !std::isfinite(lat_rad) ||
        !std::isfinite(g_mps2) || g_mps2 <= 0.0f)
    {
      return r;
    }
    // 正规方程 H^T H（3x3）与 H^T y（两方向）
    double HtH[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
    double HtYe[3] = {0.0, 0.0, 0.0};
    double HtYn[3] = {0.0, 0.0, 0.0};
    for (int i = 0; i < count_; ++i)
    {
      const double t = samples_[i].t_s;
      const double h[3] = {t, t * t, t * t * t};
      for (int a = 0; a < 3; ++a)
      {
        for (int b = 0; b < 3; ++b)
        {
          HtH[a][b] += h[a] * h[b];
        }
        HtYe[a] += h[a] * samples_[i].dv_e_mps;
        HtYn[a] += h[a] * samples_[i].dv_n_mps;
      }
    }
    double inv[3][3];
    if (!Invert3(HtH, inv))
    {
      return r;
    }
    double ae[3];
    double an[3];
    for (int a = 0; a < 3; ++a)
    {
      ae[a] = inv[a][0] * HtYe[0] + inv[a][1] * HtYe[1] + inv[a][2] * HtYe[2];
      an[a] = inv[a][0] * HtYn[0] + inv[a][1] * HtYn[1] + inv[a][2] * HtYn[2];
    }
    // 残差（拟合质量）
    double res2 = 0.0;
    for (int i = 0; i < count_; ++i)
    {
      const double t = samples_[i].t_s;
      const double h[3] = {t, t * t, t * t * t};
      const double ye = h[0] * ae[0] + h[1] * ae[1] + h[2] * ae[2];
      const double yn = h[0] * an[0] + h[1] * an[1] + h[2] * an[2];
      const double de = ye - samples_[i].dv_e_mps;
      const double dn = yn - samples_[i].dv_n_mps;
      res2 += de * de + dn * dn;
    }
    r.fit_residual_rms = static_cast<float>(
        std::sqrt(res2 / (2.0 * static_cast<double>(count_))));

    const double g = g_mps2;
    const double w = omega_ie;
    const double cos_l = std::cos(static_cast<double>(lat_rad));
    const double tan_l = std::tan(static_cast<double>(lat_rad));
    if (std::fabs(cos_l) < 1e-6)
    {
      return r;  // 极区不适用
    }
    r.a[0] = static_cast<float>(ae[0]);
    r.a[1] = static_cast<float>(ae[1]);
    r.a[2] = static_cast<float>(ae[2]);
    r.a[3] = static_cast<float>(an[0]);
    r.a[4] = static_cast<float>(an[1]);
    r.a[5] = static_cast<float>(an[2]);
    r.u_e = static_cast<float>(2.0 * an[1] / g);
    r.u_n = static_cast<float>(-2.0 * ae[1] / g);
    r.u_u = static_cast<float>(-6.0 * an[2] / (g * w * cos_l) -
                               (2.0 * ae[1] / g) * tan_l);
    r.phi_e_rad = static_cast<float>(an[0] / g);
    r.phi_n_rad = static_cast<float>(-ae[0] / g);
    r.phi_u_rad = static_cast<float>(
        r.phi_n_rad * tan_l - r.u_e / (w * cos_l));
    r.sample_count = count_;
    r.valid = true;
    return r;
  }

  /**
   * 双位置修正：用第一位置测出的 ε_IN（= 第二位置 ε_E，式 9.9.49）修正方位。
   * @param r        第二位置的解算结果
   * @param eps_in   第一位置测得的等效北向漂移（rad/s，由式 9.9.50）
   */
  static void ApplyDualPosition(Result *r, const float eps_in_radps,
                                const float lat_rad,
                                const float omega_ie = 7.292115e-5f)
  {
    if (r == nullptr || !r->valid || !std::isfinite(eps_in_radps))
    {
      return;
    }
    const double cos_l = std::cos(static_cast<double>(lat_rad));
    if (std::fabs(cos_l) < 1e-6)
    {
      return;
    }
    r->phi_u_rad = static_cast<float>(
        static_cast<double>(r->phi_u_rad) -
        static_cast<double>(eps_in_radps) / (omega_ie * cos_l));
  }

private:
  static bool Invert3(const double m[3][3], double inv[3][3])
  {
    const double det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                       m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                       m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    if (std::fabs(det) < 1e-30)
    {
      return false;
    }
    const double id = 1.0 / det;
    inv[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * id;
    inv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * id;
    inv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * id;
    inv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * id;
    inv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * id;
    inv[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * id;
    inv[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * id;
    inv[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * id;
    inv[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * id;
    return true;
  }

  Sample samples_[kMaxSamples];
  int count_ = 0;
};

#endif  // INS_PARAM_ALIGN_H
