/**
 * @file    test_gravity_models.cpp
 * @module   测试/宿主机
 * @brief   WGS84 正常重力 vs J2 引力模型对比。
 * @details 输入：纬度 0–90°（15° 步长）×椭球高 0–10 km（5 km 步长）网格；处理：
 *          Somigliana 正常重力（+3.086e-6·h 线性高度修正）与 J2 球坐标引力
 *          （补离心力项以同口径）逐点比对；输出：差值 [m/s²]、[mg] 与最大值。
 *          判据：最大差 ≤ 5 mg → 对 50 m 级导航足够；超过则判需换 J2/EGM96。
 * @frame   NED：只算重力幅值（大地纬度、椭球高）；无姿态与机体系转换。
 * @units   纬度 deg（内部转 rad）、高度 km/m、重力 m/s²、差值 mg
 *          （1 mg ≈ 0.0098 m/s²）。
 *
 * 目的：量化“高精度重力模型”相对 WGS84 正常重力的差异，判断当前
 * 机械编排的重力精度是否足够（对惯导位置漂移的影响）。
 *
 * 模型：
 *   1) Somigliana 正常重力（WGS84 椭球、大地纬度）：
 *        γ = 9.7803253359·(1+0.00193185265241·sin²φ)/√(1−0.00669437999013·sin²φ)
 *      高度修正：γ(h) = γ(1 − 2h/a·(1+f+m−2f·sin²φ)) 的常用简化可用
 *      γ − 3.086e-6·h（h 米，近地）。
 *   2) J2 引力（球坐标，相对地心）：
 *        g = GM/r²·[1 − 1.5·J2·(a/r)²·(3·sin²φ − 1)]
 *
 * 输出：纬度 0–90°、高度 0–10 km 网格上的差值（m/s² 与 mg）。
 */
#include <cmath>
#include <cstdio>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr double kGM = 3.986004418e14;   // m³/s²
constexpr double kA = 6378137.0;         // WGS84 长半轴 m
constexpr double kJ2 = 1.08263e-3;       // 二阶带谐系数
constexpr double kE2 = 0.00669437999013; // 第一偏心率平方
constexpr double kOmega = 7.292115e-5;    // 地球自转角速度 rad/s

// Somigliana 正常重力（m/s²）
double normalGravity(double lat_rad)
{
  const double s2 = std::sin(lat_rad) * std::sin(lat_rad);
  return 9.7803253359 * (1.0 + 0.00193185265241 * s2) /
         std::sqrt(1.0 - kE2 * s2);
}

// 大地纬度 → 地心纬度（球坐标引力模型必须用后者）
double geocentricLat(double lat_geodetic_rad)
{
  return std::atan((1.0 - kE2) * std::tan(lat_geodetic_rad));
}

// J2 引力（m/s²，球坐标，r 为地心距；纬度输入为大地纬度）
double j2Gravity(double lat_rad, double height_m)
{
  lat_rad = geocentricLat(lat_rad);
  const double r = kA + height_m;
  const double s2 = std::sin(lat_rad) * std::sin(lat_rad);
  const double g_j2 = (kGM / (r * r)) *
      (1.0 - 1.5 * kJ2 * (kA / r) * (kA / r) * (3.0 * s2 - 1.0));
  // 引力模型需加离心力才与“正常重力”（含离心力）同口径：
  // a_centrifugal = ω²·r·cos²φ（沿地心径向的等效分量，一阶近似）。
  const double c2 = 1.0 - s2;
  return g_j2 + kOmega * kOmega * r * c2;
}
} // namespace

int main()
{
  std::printf("== WGS84 正常重力 vs J2 引力 ==\n");
  std::printf("lat(deg)  h(km)   正常重力      J2 引力      差(m/s²)   差(mg)\n");

  double max_diff = 0.0;
  double max_diff_mg = 0.0;
  for (int lat_deg = 0; lat_deg <= 90; lat_deg += 15)
  {
    for (int h_km = 0; h_km <= 10; h_km += 5)
    {
      const double lat = static_cast<double>(lat_deg) * kDeg2Rad;
      const double g_norm = normalGravity(lat) - 3.086e-6 * h_km * 1000.0;
      const double g_j2 = j2Gravity(lat, h_km * 1000.0);
      const double diff = g_j2 - g_norm;
      const double diff_mg = diff / 9.80665 * 1.0e3;
      if (std::fabs(diff) > std::fabs(max_diff))
      {
        max_diff = diff;
        max_diff_mg = diff_mg;
      }
      std::printf("%6d %6d %12.6f %12.6f %12.6f %10.3f\n", lat_deg, h_km,
                  g_norm, g_j2, diff, diff_mg);
    }
  }

  std::printf("\n最大差值 %.6f m/s²（%.3f mg）\n", max_diff, max_diff_mg);
  std::printf("参考：1 mg ≈ 0.0098 m/s²，300 s 纯惯导由 1 mg 引起的速度误差"
              "≈ 3 m/s、位置误差≈ 440 m。\n");

  // 判据：最大差值 < 5 mg（1e-2 量级小量，对 50 m 级导航可忽略）；
  // 若超过 5 mg，说明正常重力模型不足，需要换 J2/EGM96。
  if (std::fabs(max_diff_mg) <= 5.0)
  {
    std::printf("结论：正常重力模型与 J2 差 %.3f mg，对 50 m 级导航足够。\n",
                max_diff_mg);
    return 0;
  }
  std::printf("结论：差值 %.3f mg 超过 5 mg，需换更高精度重力模型。\n",
              max_diff_mg);
  return 1;
}
