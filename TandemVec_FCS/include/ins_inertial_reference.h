/**
 * @file    ins_inertial_reference.h
 * @module  固件/导航（GNSS 延时估计的惯导参考轨迹）
 * @brief   用纯 IMU 递推一条不被 GNSS 修正的参考轨迹，供延时批处理对齐使用
 * @details 输入：机体系比力、机体系→NED 四元数、加计零偏、步长；
 *          处理：去零偏→旋转到 NED→加正常重力→一次积分得速度、二次得位置；
 *          输出：参考位置/速度（valid() 为 false 时调用方必须弃用）。
 *          不变量：不使用任何 GNSS 量测，因此参考轨迹与 GNSS 延时的估计互相独立。
 * @theory  比力方程 V̇^n = C^n_b·f^b + g^n（本模块不建哥氏/传输率项）
 *          @ref REF-120 §7.4 (7.4.6) p.189；
 *          位置速率 ḣ=-V_D 与 LLA 积分 @ref REF-120 §7.2/§8.1
 * @frame   f^b 为 FRD 机体系比力，输出 p/v 为 NED
 * @units   m/s²、m/s、m、s；重力常数 9.79 m/s² 与 `TandemVec_Config.h` 同一事实源
 * @note    积分用一阶欧拉（非双子样），仅用于毫秒级延时对齐的相对形状，
 *          不用于控制；忽略地球自转/传输率，短时（≤20 s）误差可忽略。
 * @see     `InsGnssDelayAutoEstimator`、`handleNavigationSystem()` 延时估计段
 */

#pragma once

// 纯 IMU 航位推算参考（用于延时批处理，不被 GNSS 修正）
#include <Eigen/Dense>
#include <cmath>

class InsInertialReference
{
public:
  void Reset(const Eigen::Vector3d &p_ned, const Eigen::Vector3d &v_ned)
  {
    p_ = p_ned;
    v_ = v_ned;
    initialized_ = true;
  }

  void Update(const Eigen::Vector3f &accel_body_mps2,
              const Eigen::Quaternionf &q_ned_from_body,
              const Eigen::Vector3f &accel_bias_body_mps2,
              float dt_s)
  {
    if (!initialized_ || !(dt_s > 1e-5f) ||
        !accel_body_mps2.allFinite() || !q_ned_from_body.coeffs().allFinite())
    {
      return;
    }
    dt_s = std::fmin(dt_s, 0.05f);
    const Eigen::Vector3f f = accel_body_mps2 - accel_bias_body_mps2;
    const Eigen::Vector3d a_ned =
        q_ned_from_body.cast<double>() * f.cast<double>() +
        Eigen::Vector3d(0.0, 0.0, 9.79);
    v_ += a_ned * dt_s;
    p_ += v_ * dt_s;
    if (!p_.allFinite() || !v_.allFinite())
    {
      initialized_ = false;
    }
  }

  bool valid() const { return initialized_; }
  const Eigen::Vector3d &pos() const { return p_; }
  const Eigen::Vector3d &vel() const { return v_; }

private:
  Eigen::Vector3d p_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_ = Eigen::Vector3d::Zero();
  bool initialized_ = false;
};
