// 一次性设施核查（不入库）：导弹 SIL 的真值↔量测 LLA↔NED 往返是否恒等。
// 若往返误差 ≪ 观测到的 ~0.3 m 系统差，则 §4.17 的"补偿后位置反而一致变差"
// 不能归因于比较几何，只能归因于 plant 平面 NED 与 EKF 曲线 NED 的框架模型差。
#include <cstdio>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include "constants.h"
#include "transforms.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main()
{
  const Eigen::Vector3d home(28.2 * M_PI / 180.0, 112.9 * M_PI / 180.0, 50.0);
  double max_ne = 0.0, max_d = 0.0;
  // 代表性弹道：28° 仰角上升、末速 451 m/s、航程 16 km、最高点 1987 m
  const int n = 450;
  std::vector<Eigen::Vector3d> pts;
  for (int i = 0; i <= n; ++i)
  {
    const double f = i / double(n);
    const double down = 16000.0 * f;
    const double up = 1987.0 * 4.0 * f * (1.0 - f);   // 抛物近似
    pts.push_back(Eigen::Vector3d(down, 0.0, -up));   // NED: 北 down、天负
  }
  for (const auto &p : pts)
  {
    const Eigen::Vector3d lla = bfs::ned2lla(p, home, bfs::AngPosUnit::RAD);
    const Eigen::Vector3d back = bfs::lla2ned(lla, home, bfs::AngPosUnit::RAD);
    max_ne = std::max(max_ne, std::hypot(back.x() - p.x(), back.y() - p.y()));
    max_d = std::max(max_d, std::fabs(back.z() - p.z()));
  }
  std::printf("round-trip max: horizontal=%.3e m  down=%.3e m  (range 16 km, alt 1987 m)\n",
              max_ne, max_d);
  // 参照点差异：lla0 切平面 vs 各点当地水平面的夹角（曲率效应量级）
  const double curv_rad = 16000.0 / 6371000.0;
  std::printf("plant-flat vs curved-NED 量级: %.4f rad = %.2f deg, "
              "在 1987 m 高度上的 N/D 耦合 ≈ %.1f m\n",
              curv_rad, curv_rad * 180.0 / M_PI, curv_rad * 1987.0);
  std::printf("结论判据: 若 round-trip ≪ 0.3 m ⇒ 比较几何无罪\n");
  return 0;
}
