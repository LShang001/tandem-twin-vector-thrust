/*
 * 倾斜罗盘初始姿态对准 (TiltCompass)
 * ==================================
 *
 * 利用三轴加速度计测量重力方向和磁力计测量地磁场方向，直接解析出机体系
 * 的初始横滚、俯仰和航向角，是 EKF 初始化的核心调平/定向算法。
 *
 * 算法分三步，按 ZYX 次序依次计算：
 * 1. 俯仰 (Pitch)：由归一化加速度 X 轴分量通过 asin 直接获取。
 * 2. 横滚 (Roll)：扣除俯仰影响后用 Y 轴分量计算。
 * 3. 航向 (Yaw)：将磁矢量按 roll/pitch 旋转到水平面后再 atan2 取方位。
 *
 * 边界条件与限制：
 * - 要求加速度计当前无剧烈线加速度（静止或匀速），否则 roll/pitch 被运动污染。
 * - pitch = ±90° 时 cos(pitch) = 0，roll 和 yaw 进入万向节锁奇异区。
 * - 磁力计受硬铁/软铁干扰时会引入航向偏差；本函数不含磁校准算法。
 * - 无磁力计时可传入伪磁向量 (如 [1,0,0]) 得到非零航向初值以完成数值初始化，
 *   此时 yaw 不是绝对航向，只用于避免 TiltCompass 内部奇异。
 * - 返回值顺序为 [Yaw(0), Pitch(1), Roll(2)]，单位 rad。
 */
#ifndef NAVIGATION_SRC_TILT_COMPASS_H_  // NOLINT
#define NAVIGATION_SRC_TILT_COMPASS_H_

#include "eigen.h"  // NOLINT
#include "Eigen/Dense"

namespace bfs {

/*
 * @param accel  三轴加速度计读数，单位 m/s²。静止或匀速时方向与重力相反。
 * @param mag    三轴磁力计读数，内部归一化，只使用方向信息。
 * @return       [Yaw(0), Pitch(1), Roll(2)]，单位 rad。
 */
inline Eigen::Vector3f TiltCompass(const Eigen::Vector3f &accel,
                                   const Eigen::Vector3f &mag) {
  Eigen::Vector3f ypr;
  Eigen::Vector3f a = accel;
  Eigen::Vector3f m = mag;

  /* 归一化加速度和磁力计向量 */
  a.normalize();
  m.normalize();

  /* 俯仰角：asin(a_x)，FRD 机体系 X 轴（前方）分量反映俯仰倾斜程度 */
  ypr(1) = std::asin(a(0));

  /*
   * 横滚角：asin(-a_y / cos(pitch))
   * 负号来自 FRD 约定（右翼下沉 → Y 轴为正 → 横滚应为负）；
   * cos(pitch) 项补偿了大俯仰时 Y 轴灵敏度的衰减。
   */
  ypr(2) = std::asin(-a(1) / std::cos(ypr(1)));

  /*
   * 航向角：将磁矢量绕 X 轴旋转 -roll、再绕 Y 轴旋转 -pitch 回到水平面后，
   * 用 atan2 计算水平面内磁矢量方位角。下方是两次旋转的展开形式。
   * 输出范围 [-π, π]，0 rad 对应磁北。
   */
  ypr(0) = std::atan2(m(2) * std::sin(ypr(2)) - m(1) * std::cos(ypr(2)),
           m(0) * std::cos(ypr(1)) + m(1) * std::sin(ypr(1)) * std::sin(ypr(2))
           + m(2) * std::sin(ypr(1)) * std::cos(ypr(2)));

  return ypr;
}

}  // namespace bfs

#endif  // NAVIGATION_SRC_TILT_COMPASS_H_ NOLINT
