/**
 * @file    tilt_compass.h
 * @module  固件/导航（初始姿态对准/调平定向）
 * @brief   由重力方向与地磁方向解析出机体系初始横滚、俯仰与航向
 * @details 输入：机体系加速度计读数（比力，m/s²）与磁力计读数（任意单位）；
 *          处理：归一化 → 俯仰/横滚解析 → 磁矢量倾斜补偿后 atan2 求航向；
 *          输出：[Yaw, Pitch, Roll]（rad），作为 Ekf15State::Initialize() 的
 *          初始姿态。不变量：输入向量先归一化，故只使用方向信息；不做磁校准。
 * @theory  3-2-1 欧拉角调平 @ref REF-120 §9.9.1 p.323–324（粗对准：重力矢量定
 *          水平、磁矢量定方位）；倾斜补偿磁航向的坐标变换见 @ref REF-120 §9.2
 *          p.252–266（方向余弦阵与欧拉角关系）
 * @frame   输入为机体系 FRD（x 前 / y 右 / z 下）比力与磁场；输出为 NED 下的
 *          [Yaw, Pitch, Roll]，与全库 eul2quat()/quat2angle() 顺序一致
 * @units   输入加速度 m/s²，磁场任意单位（内部归一化）；输出 rad
 * @note    1) 要求静止或匀速（无显著线加速度），否则 roll/pitch 被运动污染；
 *          2) pitch=±90° 时 cos(pitch)=0，roll/yaw 落入万向节锁奇异区；
 *          3) 无磁力计时可传伪磁向量（如 (1,0,0)）得到确定性 0 航向，此时 yaw
 *          不是绝对航向，只用于避免解析式退化（见 Ekf15State 的 P0 航向修复）。
 * @see     ekf_15_state.h::Initialize()、docs/08-参考资料/notes/04-航向与磁辅助.md
 */
#ifndef NAVIGATION_SRC_TILT_COMPASS_H_  // NOLINT
#define NAVIGATION_SRC_TILT_COMPASS_H_

#include "eigen.h"  // NOLINT
#include "Eigen/Dense"

namespace bfs {

/**
 * @brief 由重力方向与磁方位解析初始 [Yaw, Pitch, Roll]
 * @param[in] accel 机体系 FRD 三轴加速度计读数 [m/s²]，静止时方向为 −g
 * @param[in] mag   机体系 FRD 三轴磁力计读数（任意单位，内部归一化）
 * @return [Yaw(0), Pitch(1), Roll(2)]，单位 rad；yaw ∈ (−π, π]
 * @formula pitch = asin(a_x/|a|)
 *          roll  = asin(−a_y/(|a|·cos pitch))
 *          yaw   = atan2(m_z sinφ − m_y cosφ,
 *                        m_x cosθ + m_y sinθ sinφ + m_z sinθ cosφ)
 * @ref REF-120 §9.9.1 p.323–324（重力/磁矢量粗对准）；
 *      REF-120 §9.2 p.252–266（欧拉角与方向余弦阵的 ZYX 关系）
 * @pre 载体静止或匀速直线运动；|accel| 明显大于零（Initialize 已做兜底）
 * @post 返回的 yaw 为磁北基准；存在磁偏角时与真北差一个常值偏置
 * @note 等价变换：秦书粗对准按地理系 ENU 推导，本项目为 NED/FRD，故
 *       pitch=asin(+a_x)、roll=asin(−a_y/cosθ) 的符号与 ENU 教材相反；
 *       水平静止时 f^b=(0,0,−g) 代入即得 θ=φ=0，与机头朝天悬停基态一致。
 * @note roll 的 asin 形式在 |−a_y/cosθ|>1（大倾角+振动）时会产生 NaN，
 *       调用方（Initialize）只传入经 SanitizeAccelForInit() 处理过的样本。
 * @test test_host/test_ekf_15state.cpp T1（水平静止：|roll|、|pitch| < 1e-3 rad）
 * @test test_host/test_ekf_bias_static.cpp（0.2 rad 初始航向不确定度设定路径）
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
