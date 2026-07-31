/*
 * 15 状态误差状态卡尔曼滤波器 (Ekf15State)
 * =========================================
 *
 * 本文件是本导航库的核心，实现了完整的捷联惯导解算 (Strapdown INS) +
 * GNSS 组合导航 + 多种辅助量测的 15 状态扩展卡尔曼滤波器。
 *
 * 15 维误差状态向量定义（按 x_ 中的索引顺序）：
 *   0-2：   NED 位置误差 (m)
 *   3-5：   NED 速度误差 (m/s)
 *   6-8：   FRD 姿态误差角 (rad) —— 等效旋转向量 (error-state / 乘性四元数)
 *   9-11：  加速度计零偏误差 (m/s²)
 *   12-14： 陀螺零偏误差 (rad/s)
 *
 * 核心功能层级：
 * ┌─────────────────────────────────────────────────────────────┐
 * │                    Public API (上层调用)                     │
 * │  Initialize()    TimeUpdate()         MeasurementUpdate*()  │
 * │  静基座对准      单子样/双子样传播     GNSS / ZUPT / Gravity │
 * │                  + 协方差预测          / Attitude / Yaw /     │
 * │                                       StaticGyro / GSF       │
 * ├─────────────────────────────────────────────────────────────┤
 * │                  Private 实现 (内部数学)                     │
 * │  PropagateStateFromIncrements()  —— 四元数姿态推进 + 速度/   │
 * │    位置中点积分 + 圆锥补偿 + 划摇补偿 + 地球/科氏力补偿      │
 * │  InjectErrorState()              —— 误差状态注入全量状态     │
 * │  StabilizeCovariance()           —— P 矩阵对称化 + 特征值投影 │
 * │  ComputeEarthModelTerms()        —— WGS-84 地球模型缓存       │
 * │  RestoreSnapshot() / RepropagateFromDelayedSnapshot()        │
 * │                                  —— GNSS 延迟回放重传播       │
 * ├─────────────────────────────────────────────────────────────┤
 * │              编译期宏控制（性能/精度权衡）                   │
 * │  FAST_PROPAGATION          FAST_FIRST_ORDER_PHI              │
 * │  FAST_FIRST_ORDER_COVARIANCE  STRUCTURED_SIMPSON_QC          │
 * │  FAST_STABILIZE            FAST_STATIC_AID_UPDATE            │
 * │  DISABLE_GNSS_DELAY_REPLAY   PROFILE_ENABLED                 │
 * └─────────────────────────────────────────────────────────────┘
 *
 * 坐标系约定（与库内其它文件统一）：
 * - 机体系 FRD (前-右-下)，导航系 NED (北-东-地)
 * - 欧拉角序 [Yaw, Pitch, Roll]（ZYX Tait-Bryan）
 * - LLA 内部以 rad/m 存储，NED 速度 m/s
 *
 * GNSS 延迟回放机制：
 * - 每个 TimeUpdate 后记录完整状态快照 (InsSnapshot) 到环形缓冲。
 * - GNSS 量测到达时，按 GNSS_DELAY_S 从缓冲中找到对应的历史时刻状态，
 *   恢复到该时刻 → 执行卡尔曼更新 → 重放后续 IMU 输入推进到当前时刻。
 * - 缓冲大小 64 格，在 200 Hz 下可覆盖 320 ms 历史，远超 15 ms 典型延迟。
 * - 纯惯导测试可设 DISABLE_GNSS_DELAY_REPLAY=1 将缓冲缩为 1 格。
 *
 * 本文件在 2024-2026 年间经过大量本地重构，包括但不限于：
 * - GNSS 延迟回放（环缓冲 + 重传播）
 * - NIS/残差门控 (ZUPT / Gravity / Attitude / Yaw / StaticGyro)
 * - 协方差稳定化 (Gershgorin + LDLT + 特征值投影)
 * - 双子样 TimeUpdate + 快速传播路径
 * - 静止陀螺零偏专用量测 (StaticGyro bias-only update)
 * - GNSS 重捕获低增益融合 + 失联膨胀
 * - 主机回归测试接缝 (EKF_HOST_REGRESSION)
 */

#ifndef NAVIGATION_SRC_EKF_15_STATE_H_ // NOLINT
#define NAVIGATION_SRC_EKF_15_STATE_H_

#include "aid_source_status.h"                   // NOLINT
#include "constants.h"                           // NOLINT
#include "earth_model.h"                         // NOLINT
#include "transforms.h"                          // NOLINT
#include "utils.h"                               // NOLINT
#include "units.h"                               // NOLINT
#include "icm45686_ins_fast_covariance_blocks.h" // NOLINT
#include "tilt_compass.h"                        // NOLINT
#include "eigen.h"                               // NOLINT
#include "Eigen/Dense"
#include <algorithm>
#include <array>
#include <cmath> // std::isfinite 用于 EKF 输入与新息数值防御。
#include <limits>

#ifndef BFS_NAVIGATION_EMBEDDED_FAST_PROPAGATION
#define BFS_NAVIGATION_EMBEDDED_FAST_PROPAGATION 0
#endif

#ifndef BFS_NAVIGATION_EMBEDDED_FAST_STABILIZE
#define BFS_NAVIGATION_EMBEDDED_FAST_STABILIZE 0
#endif

#ifndef BFS_NAVIGATION_EMBEDDED_FAST_STATIC_AID_UPDATE
#define BFS_NAVIGATION_EMBEDDED_FAST_STATIC_AID_UPDATE 0
#endif

#ifndef BFS_NAVIGATION_EMBEDDED_FAST_FIRST_ORDER_PHI
#define BFS_NAVIGATION_EMBEDDED_FAST_FIRST_ORDER_PHI 0
#endif

#ifndef BFS_NAVIGATION_EMBEDDED_FAST_FIRST_ORDER_COVARIANCE
#define BFS_NAVIGATION_EMBEDDED_FAST_FIRST_ORDER_COVARIANCE 0
#endif

#ifndef BFS_NAVIGATION_EMBEDDED_STRUCTURED_SIMPSON_QC
#define BFS_NAVIGATION_EMBEDDED_STRUCTURED_SIMPSON_QC 1
#endif

/*
 * 完整 F 矩阵开关（默认 0 = 简化 F 矩阵，仅核心非零块）。
 *
 * 设为 1 时额外建模以下耦合项：
 *   F(V,V)  — 速度误差的哥氏力/传输速率耦合（3×3，9 项）
 *   F(P,P)  — 位置误差的速度/纬度/高度自耦合（3×3，5 项）
 *   F(Φ,V)  — 姿态误差的速度/曲率耦合（3×3，3 项）
 *
 * 适用边界：
 * - 短时飞行（<2 min）+ MEMS IMU + 高频 GNSS 修正：简化版已足够，升级收益 <0.1°
 * - 长航时（>5 min）或高速（>500 m/s）：建议开启以减少协方差模型失配
 * - 200 Hz 嵌入式路径：额外开销约 15 次 float 乘加（可忽略）
 */
#ifndef BFS_NAVIGATION_EMBEDDED_FULL_F_MATRIX
#define BFS_NAVIGATION_EMBEDDED_FULL_F_MATRIX 1
#endif

#ifndef BFS_NAVIGATION_EMBEDDED_PROFILE_ENABLED
#define BFS_NAVIGATION_EMBEDDED_PROFILE_ENABLED 0
#endif

#ifndef BFS_NAVIGATION_EMBEDDED_PROFILE_NOW_US
#define BFS_NAVIGATION_EMBEDDED_PROFILE_NOW_US() 0UL
#endif

#ifndef BFS_NAVIGATION_EMBEDDED_PROFILE_MARK
#define BFS_NAVIGATION_EMBEDDED_PROFILE_MARK(stage, elapsed_us) \
  do                                                            \
  {                                                             \
    (void)(stage);                                              \
    (void)(elapsed_us);                                         \
  } while (0)
#endif

#ifndef BFS_NAVIGATION_EMBEDDED_FULL_STABILIZE_PERIOD
#define BFS_NAVIGATION_EMBEDDED_FULL_STABILIZE_PERIOD 50U
#endif

#ifndef BFS_NAVIGATION_EMBEDDED_PROPAGATION_STABILIZE_DIVIDER
#define BFS_NAVIGATION_EMBEDDED_PROPAGATION_STABILIZE_DIVIDER 1U
#endif

#ifndef BFS_NAVIGATION_EMBEDDED_STATIC_AID_STABILIZE_DIVIDER
#define BFS_NAVIGATION_EMBEDDED_STATIC_AID_STABILIZE_DIVIDER 1U
#endif

/*
 * 无 GNSS 纯惯导或桌面 bring-up 场景下，不需要维护 GNSS 延迟回放历史缓冲。
 * 打开此开关后：
 * 1. TimeUpdate 不再维护历史快照和 GNSS 失联膨胀逻辑
 * 2. 历史缓冲区缩成 1 个槽位，显著减少 RAM 占用
 * 3. MeasurementUpdate 仍可编译，但会退化为“当前时刻直接更新”，不再做延迟回放
 *
 * 使用边界：
 * - 仅建议在“当前固件不会接收真实延迟 GNSS 量测”的纯惯导测试环境打开。
 * - 一旦要接入串口 GNSS 并使用真实位置/速度融合，应关闭该宏，恢复完整历史回溯能力。
 * - 因此这里是“环境级裁剪”，不是删除 GNSS 组合导航能力；默认值仍保持 0。
 */
#ifndef BFS_NAVIGATION_EMBEDDED_DISABLE_GNSS_DELAY_REPLAY
#define BFS_NAVIGATION_EMBEDDED_DISABLE_GNSS_DELAY_REPLAY 0
#endif

/*
 * GNSS 观测相对当前 IMU/EKF 时刻的默认回溯延迟，单位 s。
 *
 * 估算依据：
 * - u-blox UBX-NAV-PVT 的 iTOW 对应导航历元，不是 MCU 收到完整串口帧的时刻。
 * - NAV-PVT 总帧长约 100 字节；115200 8N1 下纯串口传输约 8.7 ms，
 *   230400/460800/921600 下更短。
 * - 当前 GNSS UART 计划使用 921600 baud，纯串口传输约 1 ms；预留接收机内部输出排队、
 *   MCU 串口缓存和主循环调度抖动后，15 ms 比原先 120 ms 更适合作为当前 200 Hz EKF
 *   的无 PPS 初始估计。
 *
 * 后续若接入 PPS/TIMEPULSE 或根据 iTOW 与 MCU micros() 做实测标定，应在 platformio.ini
 * 中覆盖该宏，而不是改 EKF 源码。
 */
#ifndef BFS_NAVIGATION_GNSS_DELAY_S
#define BFS_NAVIGATION_GNSS_DELAY_S 0.015f
#endif

namespace bfs
{

  class Ekf15State
  {
    // 历史状态快照，用于 GNSS 延迟量测后的状态重传播。
    struct InsSnapshot
    {
      Eigen::Vector3d lla;
      Eigen::Vector3f ned_vel;
      Eigen::Quaternionf quat;
      Eigen::Vector3f accel_bias;
      Eigen::Vector3f gyro_bias;
      Eigen::Vector3f previous_delta_theta;
      Eigen::Vector3f previous_delta_v;
      Eigen::Matrix<float, 15, 15> p;
      Eigen::Vector3f accel_input;
      Eigen::Vector3f gyro_input;
      Eigen::Vector3f delta_theta_1;
      Eigen::Vector3f delta_theta_2;
      Eigen::Vector3f delta_v_1;
      Eigen::Vector3f delta_v_2;
      float dt_s = 0.0f;
      bool has_two_sample_increment = false;
      bool has_velocity_update = false;
      Eigen::Vector3f velocity_meas = Eigen::Vector3f::Zero();
      float velocity_ne_noise = 0.0f;
      float velocity_d_noise = 0.0f;
      bool has_gravity_update = false;
      Eigen::Vector3f gravity_accel_meas = Eigen::Vector3f::Zero();
      float gravity_noise = 0.0f;
      bool has_attitude_update = false;
      Eigen::Vector3f attitude_ypr_meas = Eigen::Vector3f::Zero();
      float attitude_roll_pitch_noise = 0.0f;
      float attitude_yaw_noise = 0.0f;
      bool has_yaw_update = false;
      float yaw_meas = 0.0f;
      float yaw_noise = 0.0f;
      bool has_static_gyro_update = false;
      Eigen::Vector3f static_gyro_meas = Eigen::Vector3f::Zero();
      Eigen::Vector3f static_gyro_noise = Eigen::Vector3f::Zero();
      bool has_baro_altitude_update = false;
      float baro_altitude_m = 0.0f;
      float baro_noise_std_m = 0.0f;
    };

  public:
    Ekf15State() {}

    /* Sensor characteristics setters */
    inline void accel_std_mps2(const float val)
    {
      // IMU噪声标准差必须为有限正数，非法配置保持上一可信值。
      if (IsValidNoiseStd(val))
      {
        accel_std_mps2_ = val;
      }
    }
    inline void accel_markov_bias_std_mps2(const float val)
    {
      // 零偏随机游走标准差必须为有限正数，避免过程噪声矩阵出现NaN或负语义。
      if (IsValidNoiseStd(val))
      {
        accel_markov_bias_std_mps2_ = val;
      }
    }
    inline void accel_tau_s(const float val)
    {
      // Markov相关时间常数参与除法，必须为有限正数。
      if (IsValidNoiseStd(val))
      {
        accel_tau_s_ = val;
      }
    }
    inline void gyro_std_radps(const float val)
    {
      // 陀螺白噪声标准差必须为有限正数，非法配置保持上一可信值。
      if (IsValidNoiseStd(val))
      {
        gyro_std_radps_ = val;
      }
    }
    inline void gyro_markov_bias_std_radps(const float val)
    {
      // 陀螺零偏随机游走标准差必须为有限正数。
      if (IsValidNoiseStd(val))
      {
        gyro_markov_bias_std_radps_ = val;
      }
    }
    inline void gyro_tau_s(const float val)
    {
      // Markov相关时间常数参与除法，必须为有限正数。
      if (IsValidNoiseStd(val))
      {
        gyro_tau_s_ = val;
      }
    }
    inline void gnss_pos_ne_std_m(const float val)
    {
      // GNSS精度来自外部接收机，异常值保持上一可信配置，避免R矩阵被NaN/零噪声污染。
      if (IsValidNoiseStd(val))
      {
        gnss_pos_ne_std_m_ = val;
      }
    }
    inline void gnss_pos_d_std_m(const float val)
    {
      // GNSS精度来自外部接收机，异常值保持上一可信配置，避免R矩阵被NaN/零噪声污染。
      if (IsValidNoiseStd(val))
      {
        gnss_pos_d_std_m_ = val;
      }
    }
    inline void gnss_vel_ne_std_mps(const float val)
    {
      // GNSS精度来自外部接收机，异常值保持上一可信配置，避免R矩阵被NaN/零噪声污染。
      if (IsValidNoiseStd(val))
      {
        gnss_vel_ne_std_mps_ = val;
      }
    }
    inline void gnss_vel_d_std_mps(const float val)
    {
      // GNSS精度来自外部接收机，异常值保持上一可信配置，避免R矩阵被NaN/零噪声污染。
      if (IsValidNoiseStd(val))
      {
        gnss_vel_d_std_mps_ = val;
      }
    }
    inline void propagation_stabilize_divider(const uint32_t val)
    {
      // 运行时 divider 只允许正整数；1 表示每次传播后都做完整稳定化。
      propagation_stabilize_divider_ = std::max<uint32_t>(1U, val);
    }
    inline void static_aid_stabilize_divider(const uint32_t val)
    {
      // 静止辅助路径同样只接受正整数 divider；1 表示每次量测更新后都完整稳定化。
      static_aid_stabilize_divider_ = std::max<uint32_t>(1U, val);
    }
    inline uint32_t propagation_stabilize_divider() const
    {
      return propagation_stabilize_divider_;
    }
    inline uint32_t static_aid_stabilize_divider() const
    {
      return static_aid_stabilize_divider_;
    }

    /* Sensor characteristics getters */
    inline float accel_std_mps2() const
    {
      return accel_std_mps2_;
    }
    inline float accel_markov_bias_std_mps2() const
    {
      return accel_markov_bias_std_mps2_;
    }
    inline float accel_tau_s() const
    {
      return accel_tau_s_;
    }
    inline float gyro_std_radps() const
    {
      return gyro_std_radps_;
    }
    inline float gyro_markov_bias_std_radps() const
    {
      return gyro_markov_bias_std_radps_;
    }
    inline float gyro_tau_s() const
    {
      return gyro_tau_s_;
    }
    inline float gnss_pos_ne_std_m() const
    {
      return gnss_pos_ne_std_m_;
    }
    inline float gnss_pos_d_std_m() const
    {
      return gnss_pos_d_std_m_;
    }
    inline float gnss_vel_ne_std_mps() const
    {
      return gnss_vel_ne_std_mps_;
    }
    inline float gnss_vel_d_std_mps() const
    {
      return gnss_vel_d_std_mps_;
    }

    /* Initial covariance setters */
    inline void init_pos_err_std_m(const float val)
    {
      // 初始协方差标准差必须为有限正数，避免初始化P矩阵奇异或NaN。
      if (IsValidNoiseStd(val))
      {
        init_pos_err_std_m_ = val;
      }
    }
    inline void init_vel_err_std_mps(const float val)
    {
      // 初始协方差标准差必须为有限正数，避免初始化P矩阵奇异或NaN。
      if (IsValidNoiseStd(val))
      {
        init_vel_err_std_mps_ = val;
      }
    }
    inline void init_att_err_std_rad(const float val)
    {
      // 姿态初始不确定度必须为有限正数，过小或异常值都不写入。
      if (IsValidNoiseStd(val))
      {
        init_att_err_std_rad_ = val;
      }
    }
    inline void init_heading_err_std_rad(const float val)
    {
      // 航向初始不确定度必须为有限正数，避免P矩阵被Inf污染。
      if (IsValidNoiseStd(val))
      {
        init_heading_err_std_rad_ = val;
      }
    }
    inline void init_accel_bias_std_mps2(const float val)
    {
      // 加速度零偏初始不确定度必须为有限正数。
      if (IsValidNoiseStd(val))
      {
        init_accel_bias_std_mps2_ = val;
      }
    }
    inline void init_gyro_bias_std_radps(const float val)
    {
      // 陀螺零偏初始不确定度必须为有限正数。
      if (IsValidNoiseStd(val))
      {
        init_gyro_bias_std_radps_ = val;
      }
    }

    /* Initial covariance getters */
    inline float init_pos_err_std_m() const
    {
      return init_pos_err_std_m_;
    }
    inline float init_vel_err_std_mps() const
    {
      return init_vel_err_std_mps_;
    }
    inline float init_att_err_std_rad() const
    {
      return init_att_err_std_rad_;
    }
    inline float init_heading_err_std_rad() const
    {
      return init_heading_err_std_rad_;
    }
    inline float init_accel_bias_std_mps2() const
    {
      return init_accel_bias_std_mps2_;
    }
    inline float init_gyro_bias_std_radps() const
    {
      return init_gyro_bias_std_radps_;
    }

    /**
     * @brief 初始化 EKF 状态 (Initialization)
     *
     * 在系统启动时调用一次。负责构建初始的状态向量 x 和协方差矩阵 P，
     * 并设定过程噪声 Q 和观测噪声 R 的参数。
     * 同时执行初始对准（Alignment）：利用加速度计和磁力计确定初始姿态。
     *
     * @param accel   初始加速度计读数 (用于计算 Roll/Pitch 和初始化零偏)
     * @param gyro    初始陀螺仪读数 (用于初始化陀螺仪零偏)
     * @param mag     初始磁力计读数 (用于计算 Yaw)
     * @param ned_vel 初始速度 (通常为 0，除非是空中对准)
     * @param lla     初始位置 (纬度, 经度, 高度)
     */
    void Initialize(const Eigen::Vector3f &accel, const Eigen::Vector3f &gyro,
                    const Eigen::Vector3f &mag, const Eigen::Vector3f &ned_vel,
                    const Eigen::Vector3d &lla)
    {
      // 初始化入口也做传感器兜底，防止单帧坏数据把初始姿态、零偏或地理坐标置为NaN。
      const Eigen::Vector3f safe_accel = SanitizeAccelForInit(accel);
      const Eigen::Vector3f safe_gyro = SanitizeVectorOrZero(gyro);
      const Eigen::Vector3f safe_mag = SanitizeMagForInit(mag);
      const Eigen::Vector3f safe_ned_vel = SanitizeVectorOrZero(ned_vel);
      const Eigen::Vector3d safe_lla = SanitizeLlaForInit(lla);

      // 缓存初始化地点的 WGS-84 正常重力（供控制律/遥测消费）
      gravity_mps2_ = ComputeEarthModelTerms(safe_lla(0), safe_lla(2)).gravity_mps2;

      // 在 Initialize 函数的开头预分配内存
      // 使用固定数组保存历史状态，避免飞控运行时进行堆分配。
      buf_head_ = 0;
      buf_full_ = false;
      buf_count_ = 0;
      p_.setZero();
      q_.setZero();
      r_.setZero();
      rw_.setZero();
      gs_.setZero();
      fs_.setZero();
      phi_.setZero();
      s_.setZero();
      k_.setZero();
      y_.setZero();
      x_.setZero();
      time_since_gnss_update_s_ = 0.0f;
      gnss_reacquire_timer_s_ = 0.0f;

      // ----------------------------------------------------------------------
      // [步骤 1] 初始化观测矩阵 H (Observation Matrix)
      // 公式: z = H * x + v
      // ----------------------------------------------------------------------
      /*
       * H 矩阵描述了观测值 (GNSS) 与状态值 (INS) 之间的线性关系。
       * 标准观测为 [Pos_N, Pos_E, Pos_D, Vel_N, Vel_E, Vel_D]，
       * 因此必须映射完整 6 维位置和速度误差。
       */
      h_.setZero();
      h_.block(0, 0, 6, 6) = Eigen::Matrix<float, 6, 6>::Identity();

      // ----------------------------------------------------------------------
      // [步骤 2] 初始化过程噪声协方差矩阵 Rw (Process Noise Covariance)
      // 描述 IMU 传感器自身的噪声特性 (连续时间谱密度)
      // ----------------------------------------------------------------------

      /* 加速度计白噪声 (Velocity Random Walk) */
      rw_.block(0, 0, 3, 3) = accel_std_mps2_ * accel_std_mps2_ *
                              Eigen::Matrix<float, 3, 3>::Identity();

      /* 陀螺仪白噪声 (Angle Random Walk) */
      rw_.block(3, 3, 3, 3) = gyro_std_radps_ * gyro_std_radps_ *
                              Eigen::Matrix<float, 3, 3>::Identity();

      /* 加速度计零偏随机游走 (Bias Instability) */
      /* 公式来源于一阶高斯-马尔可夫过程的方差方程 */
      rw_.block(6, 6, 3, 3) = 2.0f * accel_markov_bias_std_mps2_ *
                              accel_markov_bias_std_mps2_ / accel_tau_s_ *
                              Eigen::Matrix<float, 3, 3>::Identity();

      /* 陀螺仪零偏随机游走 (Bias Instability) */
      rw_.block(9, 9, 3, 3) = 2.0f * gyro_markov_bias_std_radps_ *
                              gyro_markov_bias_std_radps_ / gyro_tau_s_ *
                              Eigen::Matrix<float, 3, 3>::Identity();

      // ----------------------------------------------------------------------
      // [步骤 3] 初始化观测噪声协方差矩阵 R (Observation Noise Covariance)
      // 描述 GPS 测量的不确定度
      // ----------------------------------------------------------------------
      UpdateGnssNoiseMatrix();

      // ----------------------------------------------------------------------
      // [步骤 4] 初始化状态协方差矩阵 P (Initial Covariance Estimate)
      // 描述我们对“初始状态值”有多大的信心 (值越大越不自信)
      // ----------------------------------------------------------------------

      /* 初始位置误差协方差 */
      p_.block(0, 0, 3, 3) = init_pos_err_std_m_ * init_pos_err_std_m_ *
                             Eigen::Matrix<float, 3, 3>::Identity();

      /* 初始速度误差协方差 */
      p_.block(3, 3, 3, 3) = init_vel_err_std_mps_ * init_vel_err_std_mps_ *
                             Eigen::Matrix<float, 3, 3>::Identity();

      /* 初始水平姿态 (Roll/Pitch) 误差协方差 (由加速度计对准精度决定) */
      p_.block(6, 6, 2, 2) = init_att_err_std_rad_ * init_att_err_std_rad_ *
                             Eigen::Matrix<float, 2, 2>::Identity();

      /* 初始航向 (Yaw) 误差协方差 (由磁力计对准精度决定) */
      p_(8, 8) = init_heading_err_std_rad_ * init_heading_err_std_rad_;

      /* 初始加速度零偏误差协方差 */
      p_.block(9, 9, 3, 3) = init_accel_bias_std_mps2_ *
                             init_accel_bias_std_mps2_ *
                             Eigen::Matrix<float, 3, 3>::Identity();

      /* 初始陀螺仪零偏误差协方差 */
      p_.block(12, 12, 3, 3) = init_gyro_bias_std_radps_ *
                               init_gyro_bias_std_radps_ *
                               Eigen::Matrix<float, 3, 3>::Identity();

      // ----------------------------------------------------------------------
      // [步骤 5] 初始化状态转移矩阵中的零偏部分 (Markov Bias Matrices)
      // ----------------------------------------------------------------------
      /*
       * 定义零偏随时间的衰减特性 (High-pass filter characteristic in error state).
       * F_bias = -1/tau * I
       */
      accel_markov_bias_ = -1.0f / accel_tau_s_ *
                           Eigen::Matrix<float, 3, 3>::Identity();
      gyro_markov_bias_ = -1.0f / gyro_tau_s_ *
                          Eigen::Matrix<float, 3, 3>::Identity();
      RefreshStateTransitionStaticBlocks();

      // ----------------------------------------------------------------------
      // [步骤 6] 初始化状态向量 (State Initialization)
      // ----------------------------------------------------------------------

      /* 1. 位置和速度直接使用传入的初始值 (通常来自 GPS) */
      ins_lla_rad_m_ = safe_lla;
      ins_ned_vel_mps_ = safe_ned_vel;

      /*
       * 2. 计算“去偏后”的初始加速度计数据。
       * 注意：accel_bias_mps2_ 保持默认值(0)，因为静态加速度计读数包含重力，不能直接当零偏。
       */
      ins_accel_mps2_ = safe_accel - accel_bias_mps2_;
      previous_delta_theta_rad_.setZero();
      previous_delta_v_mps_.setZero();

      /*
       * 3. 初始姿态对准 (Alignment)
       * TiltCompass 算法利用：
       * - 加速度计 (重力矢量) -> 计算 Roll 和 Pitch (调平)
       * - 磁力计 -> 计算 Yaw (定向)
       */
      ins_ypr_rad_ = TiltCompass(safe_accel, safe_mag);

      /* 4. 将欧拉角转换为四元数，作为系统的初始姿态 */
      quat_ = eul2quat(ins_ypr_rad_);

      /*
       * 5. 初始化陀螺零偏。
       * 实机静止时陀螺仍会测到地球自转；若把完整读数当零偏扣掉，
       * 后续又在捷联传播中补偿导航系转动，就会在无GNSS静止/低动态时产生慢姿态漂移。
       */
      // 初始化阶段就把姿态矩阵缓存下来，后续传播可以直接复用上一时刻结果。
      t_b2ned = quat2dcm(quat_).transpose();
      const Eigen::Vector3f nav_frame_rate_ned =
          earthrate(safe_lla(0), bfs::AngPosUnit::RAD).cast<float>() +
          navrate(safe_ned_vel.cast<double>(), safe_lla, bfs::AngPosUnit::RAD)
              .cast<float>();
      const Eigen::Vector3f expected_physical_gyro_body =
          t_b2ned.transpose() * nav_frame_rate_ned;
      gyro_bias_radps_ = safe_gyro - expected_physical_gyro_body;
      ins_gyro_radps_ = safe_gyro - gyro_bias_radps_;
    }

    /**
     * @brief 执行时间更新 (预测步骤)
     *
     * 利用 IMU 数据进行捷联惯导解算 (Strapdown INS Mechanics)，推算下一时刻的状态。
     * 同时基于误差状态运动学方程，预测协方差矩阵 P 的变化。
     *
     * @param accel 原始加速度计读数 (m/s^2)
     * @param gyro  原始陀螺仪读数 (rad/s)
     * @param dt_s  时间间隔 (秒)
     */
    void TimeUpdate(const Eigen::Vector3f &accel, const Eigen::Vector3f &gyro,
                    const float dt_s)
    {
      if (!IsFiniteVector(accel) || !IsFiniteVector(gyro))
      {
        // 运行期IMU单帧异常直接丢弃，避免NaN/Inf污染状态和GNSS延迟历史缓冲。
        return;
      }
      if (!IsPlausibleImuSample(accel, gyro))
      {
        // 有限但物理离谱的IMU峰值同样丢弃，避免单帧坏数据污染状态和历史缓冲。
        return;
      }
      if (!IsValidTimeStep(dt_s))
      {
        // 异常时间步直接丢弃，避免反向积分或长步长污染状态和GNSS延迟缓冲。
        return;
      }

      PropagateState(accel, gyro, dt_s);
      last_valid_dt_s_ = dt_s;
#if !BFS_NAVIGATION_EMBEDDED_DISABLE_GNSS_DELAY_REPLAY
      const bool was_in_gnss_outage = time_since_gnss_update_s_ >= GNSS_REACQUIRE_MIN_OUTAGE_S;
      time_since_gnss_update_s_ = std::min(time_since_gnss_update_s_ + dt_s,
                                           GNSS_OUTAGE_TIME_CAP_S);
      if (!was_in_gnss_outage)
      {
        gnss_reacquire_timer_s_ = std::max(0.0f, gnss_reacquire_timer_s_ - dt_s);
      }

      GrowGnssOutageCovariance(dt_s);
      RecordSnapshot(buf_head_, accel, gyro, dt_s);
      state_buffer_[buf_head_].has_two_sample_increment = false;

      buf_head_++;
      if (buf_head_ >= BUFFER_SIZE)
      {
        buf_head_ = 0;
        buf_full_ = true; // 标记缓冲区已满，可以安全回溯了
      }
      if (buf_count_ < BUFFER_SIZE)
      {
        buf_count_++;
      }
#endif
    }

    /**
     * @brief 使用同一导航周期内的两个IMU子样增量进行预测。
     *
     * @param delta_theta_1_rad 前半周期原始机体系角增量(rad)。
     * @param delta_theta_2_rad 后半周期原始机体系角增量(rad)。
     * @param delta_v_1_mps 前半周期原始机体系速度增量(m/s)。
     * @param delta_v_2_mps 后半周期原始机体系速度增量(m/s)。
     * @param dt_s 两个子样合计的导航周期时间。
     */
    void TimeUpdateTwoSample(const Eigen::Vector3f &delta_theta_1_rad,
                             const Eigen::Vector3f &delta_theta_2_rad,
                             const Eigen::Vector3f &delta_v_1_mps,
                             const Eigen::Vector3f &delta_v_2_mps,
                             const float dt_s)
    {
      if (!IsFiniteVector(delta_theta_1_rad) || !IsFiniteVector(delta_theta_2_rad) ||
          !IsFiniteVector(delta_v_1_mps) || !IsFiniteVector(delta_v_2_mps))
      {
        return;
      }
      if (!IsValidTimeStep(dt_s))
      {
        return;
      }
      const Eigen::Vector3f avg_raw_gyro_radps =
          (delta_theta_1_rad + delta_theta_2_rad) / dt_s;
      const Eigen::Vector3f avg_raw_accel_mps2 =
          (delta_v_1_mps + delta_v_2_mps) / dt_s;
      if (!IsPlausibleImuSample(avg_raw_accel_mps2, avg_raw_gyro_radps))
      {
        return;
      }

      const float half_dt_s = 0.5f * dt_s;
      const Eigen::Vector3f corrected_delta_theta_1_rad =
          delta_theta_1_rad - gyro_bias_radps_ * half_dt_s;
      const Eigen::Vector3f corrected_delta_theta_2_rad =
          delta_theta_2_rad - gyro_bias_radps_ * half_dt_s;
      const Eigen::Vector3f corrected_delta_v_1_mps =
          delta_v_1_mps - accel_bias_mps2_ * half_dt_s;
      const Eigen::Vector3f corrected_delta_v_2_mps =
          delta_v_2_mps - accel_bias_mps2_ * half_dt_s;
      const Eigen::Vector3f avg_corrected_gyro_radps =
          (corrected_delta_theta_1_rad + corrected_delta_theta_2_rad) / dt_s;
      const Eigen::Vector3f avg_corrected_accel_mps2 =
          (corrected_delta_v_1_mps + corrected_delta_v_2_mps) / dt_s;

      PropagateStateFromIncrements(avg_corrected_accel_mps2,
                                   avg_corrected_gyro_radps,
                                   corrected_delta_theta_1_rad,
                                   corrected_delta_theta_2_rad,
                                   corrected_delta_v_1_mps,
                                   corrected_delta_v_2_mps,
                                   dt_s,
                                   true);
      last_valid_dt_s_ = dt_s;
#if !BFS_NAVIGATION_EMBEDDED_DISABLE_GNSS_DELAY_REPLAY
      const bool was_in_gnss_outage = time_since_gnss_update_s_ >= GNSS_REACQUIRE_MIN_OUTAGE_S;
      time_since_gnss_update_s_ = std::min(time_since_gnss_update_s_ + dt_s,
                                           GNSS_OUTAGE_TIME_CAP_S);
      if (!was_in_gnss_outage)
      {
        gnss_reacquire_timer_s_ = std::max(0.0f, gnss_reacquire_timer_s_ - dt_s);
      }

      GrowGnssOutageCovariance(dt_s);
      RecordSnapshot(buf_head_, avg_raw_accel_mps2, avg_raw_gyro_radps, dt_s);
      state_buffer_[buf_head_].has_two_sample_increment = true;
      state_buffer_[buf_head_].delta_theta_1 = delta_theta_1_rad;
      state_buffer_[buf_head_].delta_theta_2 = delta_theta_2_rad;
      state_buffer_[buf_head_].delta_v_1 = delta_v_1_mps;
      state_buffer_[buf_head_].delta_v_2 = delta_v_2_mps;

      buf_head_++;
      if (buf_head_ >= BUFFER_SIZE)
      {
        buf_head_ = 0;
        buf_full_ = true;
      }
      if (buf_count_ < BUFFER_SIZE)
      {
        buf_count_++;
      }
#endif
    }

    /**
     * @brief 执行量测更新 (Measurement Update)
     *
     * 融合 GNSS (GPS) 数据来修正 INS 的累积误差，考虑GNSS固定延迟回溯修正。
     * 包含标准的卡尔曼滤波 5 个步骤：
     * 1. 计算残差 (y)
     * 2. 计算新息协方差 (S)
     * 3. 计算卡尔曼增益 (K)
     * 4. 更新状态协方差矩阵 (P)
     * 5. 更新状态向量 (x) 并反馈修正系统
     *
     * @param gnss_vel  GNSS 测量的速度 (NED坐标系, m/s)
     * @param gnss_lla  GNSS 测量到的位置 (纬度rad, 经度rad, 高度m)
     * @param dt_avg_s  EKF的平均运行周期，用于外部调用有效性检查。
     *                  实际延迟回溯优先使用历史快照中的真实dt累计，避免主循环抖动选错历史状态。
     */
    void MeasurementUpdate(const Eigen::Vector3f &gnss_vel,
                           const Eigen::Vector3d &gnss_lla,
                           const float dt_avg_s)
    {
      (void)MeasurementUpdateDetailed(gnss_vel, gnss_lla, dt_avg_s);
    }

    /**
     * @brief 带诊断结果的 GNSS 位置/速度量测更新。
     *
     * 该接口与 MeasurementUpdate() 使用完全相同的滤波数学，但会返回输入有效性、
     * 是否进入 EKF、是否真正通过 NIS 门控并完成融合。上层 VOFA/AidSource 诊断必须
     * 使用这个结果，不能在调用后无条件标记 GNSS fused。四参数重载允许上层传入
     * 当前 epoch 映射后的真实观测年龄；三参数版本保留固定延迟兼容路径。
     */
    MeasurementUpdateResult MeasurementUpdateDetailed(const Eigen::Vector3f &gnss_vel,
                                                       const Eigen::Vector3d &gnss_lla,
                                                       const float dt_avg_s)
    {
      return MeasurementUpdateDetailed(gnss_vel, gnss_lla, dt_avg_s,
                                       GNSS_DELAY_S, false);
    }

    MeasurementUpdateResult MeasurementUpdateDetailed(const Eigen::Vector3f &gnss_vel,
                                                       const Eigen::Vector3d &gnss_lla,
                                                       const float dt_avg_s,
                                                       const float measurement_age_s)
    {
      return MeasurementUpdateDetailed(gnss_vel, gnss_lla, dt_avg_s,
                                       measurement_age_s, true);
    }

    MeasurementUpdateResult MeasurementUpdateDetailed(const Eigen::Vector3f &gnss_vel,
                                                       const Eigen::Vector3d &gnss_lla,
                                                       const float dt_avg_s,
                                                       const float measurement_age_s,
                                                       const bool require_delayed_snapshot)
    {
      MeasurementUpdateResult result;
      if (!IsFiniteVector(gnss_vel) || !IsValidLlaForEarthModel(gnss_lla) ||
          !IsValidTimeStep(dt_avg_s) || !std::isfinite(measurement_age_s) ||
          measurement_age_s < 0.0f)
      {
        // GNSS量测或周期异常时直接忽略，避免恢复历史状态后无法安全重传播。
        return result;
      }
      result.input_valid = true;
      result.attempted = true;

      // 按历史快照真实dt累计回溯到GNSS量测时刻，主循环周期抖动时仍能选到正确历史状态。
      Eigen::Vector3d hist_lla;
      Eigen::Vector3f hist_vel;
      int hist_idx = -1;
#if BFS_NAVIGATION_EMBEDDED_DISABLE_GNSS_DELAY_REPLAY
      const bool can_repropagate = false;
#else
      const bool can_repropagate = GetDelayedSnapshotIndexByAge(measurement_age_s, &hist_idx);
#endif

      if (require_delayed_snapshot && measurement_age_s > 0.0f &&
          !can_repropagate)
      {
        // 带真实时间戳的旧量测若已超出历史窗口，不能静默退化成当前时刻融合。
        return result;
      }

      if (can_repropagate)
      {
        RestoreSnapshot(state_buffer_[hist_idx]);
        hist_lla = ins_lla_rad_m_;
        hist_vel = ins_ned_vel_mps_;
      }
      else
      {
        // 启动阶段保护：缓冲区数据不足时降级为当前时刻更新。
        hist_lla = ins_lla_rad_m_;
        hist_vel = ins_ned_vel_mps_;
      }

      // ----------------------------------------------------------------------
      // [步骤 1] 计算测量残差 (Innovation / Residual)
      // 公式: y = z - h(x)
      // ----------------------------------------------------------------------

      /*
       * 计算位置误差 (前3维):
       * 将 GNSS 测得的 LLA (经纬高) 与当前 INS 估计的 LLA 之间的差异，
       * 转换为 NED 坐标系下的距离误差 (单位: 米)。
       * lla2ned 是一个大地测量学函数，计算两点间的北向、东向、地向距离。
       * gnss_lla 与 hist_lla 均为弧度，必须显式传 RAD——若按 DEG 解释，
       * 新息会被缩小 π/180 倍且旋转矩阵建在错误纬度上。
       */
      y_.segment(0, 3) =
          lla2ned(gnss_lla, hist_lla, bfs::AngPosUnit::RAD).cast<float>();

      /*
       * 计算速度误差 (后3维):
       * 直接计算 GNSS 速度与 INS 估计速度的差值。
       */
      y_.segment(3, 3) = gnss_vel - hist_vel;
      result.innovation_norm = y_.norm();

      // ----------------------------------------------------------------------
      // [步骤 2] 计算新息协方差 (Innovation Covariance)
      // 公式: S = H * P * H^T + R
      // ----------------------------------------------------------------------
      /*
       * h_: 观测矩阵 (6x15)，将状态空间映射到观测空间。
       * p_: 预测的状态协方差矩阵 (由 TimeUpdate 得到)。
       * r_: 观测噪声协方差矩阵 (代表 GPS 的不确定度)。
       */
      UpdateGnssNoiseMatrix();
      Eigen::Matrix<float, 6, 6> gnss_update_noise = r_;

      // 长时间失联后的首帧重捕获不能只依赖 NIS：失联膨胀后的 P 可能很大，
      // 十几米级假位置台阶也会得到很小的 NIS。先检查残差是否与接收机自报精度相称。
      if (time_since_gnss_update_s_ >= GNSS_REACQUIRE_MIN_OUTAGE_S &&
          !IsGnssReacquisitionResidualPlausible())
      {
        if (can_repropagate)
        {
          RepropagateFromDelayedSnapshot(hist_idx);
        }
        return result;
      }

      Eigen::Matrix<float, 6, 6> s_inv;
      float nis_value = 0.0f;
      if (!ComputeGnssInnovationStats(gnss_update_noise, &s_inv, &nis_value))
      {
        // S矩阵病态时拒绝本帧GNSS更新，避免直接反演产生坏增益。
        if (can_repropagate)
        {
          RepropagateFromDelayedSnapshot(hist_idx);
        }
        return result;
      }
      result.test_ratio = std::isfinite(nis_value)
                              ? nis_value / GNSS_NIS_REJECT_THRESHOLD
                              : 0.0f;
      if (nis_value > GNSS_NIS_REJECT_THRESHOLD)
      {
        Eigen::Matrix<float, 6, 6> conservative_noise;
        const bool use_conservative_reacquisition =
            BuildConservativeGnssReacquisitionNoise(&conservative_noise);
        if (use_conservative_reacquisition &&
            ComputeGnssInnovationStats(conservative_noise, &s_inv, &nis_value) &&
            nis_value <= GNSS_REACQUIRE_NIS_REJECT_THRESHOLD)
        {
          // GNSS长时间失联后，合理的数米级残差不应被普通NIS门限永久拒绝。
          // 使用膨胀R低增益重捕获，避免单帧跳变，同时让位置/速度重新收敛。
          gnss_update_noise = conservative_noise;
          result.test_ratio = std::isfinite(nis_value)
                                  ? nis_value / GNSS_REACQUIRE_NIS_REJECT_THRESHOLD
                                  : 0.0f;
        }
        else
        {
          // GNSS新息远超当前协方差可解释范围且不满足重捕获条件时判为离群值。
          if (can_repropagate)
          {
            RepropagateFromDelayedSnapshot(hist_idx);
          }
          return result;
        }
      }

      // ----------------------------------------------------------------------
      // [步骤 3] 计算卡尔曼增益 (Kalman Gain)
      // 公式: K = P * H^T * S^-1
      // ----------------------------------------------------------------------
      /*
       * 卡尔曼增益决定了我们多大程度上相信“观测值”(GPS)，以及多大程度上相信“预测值”(INS)。
       * 如果 S (观测不确定性) 很大，K 就会变小。
       */
      k_ = p_ * h_.transpose() * s_inv;

      // ----------------------------------------------------------------------
      // [步骤 4] 更新状态协方差矩阵 P (Covariance Update)
      // 使用 "Joseph Form" (约瑟夫形式) 以保证数值稳定性和对称性
      // 公式: P = (I - KH) * P * (I - KH)^T + K * R * K^T
      // ----------------------------------------------------------------------
      /*
       * 相比简单的 P = (I - KH)P，Joseph 形式虽然计算量稍大，
       * 但能保证 P 矩阵始终是对称正定的，防止滤波器发散。
       */
      p_ = (Eigen::Matrix<float, 15, 15>::Identity() - k_ * h_) * p_ *
               (Eigen::Matrix<float, 15, 15>::Identity() - k_ * h_).transpose() +
           k_ * gnss_update_noise * k_.transpose();
      StabilizeCovariance();

      // ----------------------------------------------------------------------
      // [步骤 5] 计算误差状态向量 (Error State Calculation)
      // 公式: dx = K * y
      // ----------------------------------------------------------------------
      /*
       * x_ 在这里存储的是“误差状态” (delta state)，而不是全量状态。
       * 包含了：位置误差、速度误差、姿态误差、加速度零偏误差、陀螺仪零偏误差。
       */
      x_ = k_ * y_;

      InjectErrorState();

      if (can_repropagate)
      {
        OverwriteSnapshotState(hist_idx);
        RepropagateFromDelayedSnapshot(hist_idx);
      }
      else
      {
        const int latest_idx = LatestSnapshotIndex();
        if (latest_idx >= 0)
        {
          OverwriteSnapshotState(latest_idx);
        }
      }
      if (time_since_gnss_update_s_ >= GNSS_REACQUIRE_MIN_OUTAGE_S)
      {
        gnss_reacquire_timer_s_ = GNSS_REACQUIRE_WINDOW_S;
      }
      time_since_gnss_update_s_ = 0.0f;
      result.fused = true;
      return result;
    }

    /**
     * @brief 仅速度量测更新，用于无 GNSS 静止零速修正 (ZUPT)。
     *
     * 该接口只观测 NED 速度，不伪造位置观测，避免把假原点位置误差注入姿态和零偏。
     *
     * @param measured_vel_ned NED 速度观测值，静止时通常为 (0,0,0)。
     * @param vel_ne_std_mps 水平速度观测标准差。
     * @param vel_d_std_mps 垂直速度观测标准差。
     * @return true 表示该速度量测通过了残差/NIS检查并已实际融合；false 表示被拒绝或输入无效。
     */
    bool MeasurementUpdateVelocity(const Eigen::Vector3f &measured_vel_ned,
                                   const float vel_ne_std_mps,
                                   const float vel_d_std_mps)
    {
      return MeasurementUpdateVelocityDetailed(measured_vel_ned,
                                               vel_ne_std_mps,
                                               vel_d_std_mps)
          .fused;
    }

    /**
     * @brief 带诊断结果的速度量测更新。
     *
     * 与 MeasurementUpdateVelocity() 使用同一套滤波数学，但额外返回输入是否有效、
     * 是否尝试融合、新息模长和门控比，方便上层 AidSource 统计或串口诊断。
     */
    MeasurementUpdateResult MeasurementUpdateVelocityDetailed(
        const Eigen::Vector3f &measured_vel_ned,
        const float vel_ne_std_mps,
        const float vel_d_std_mps)
    {
      MeasurementUpdateResult result;
      if (!IsFiniteVector(measured_vel_ned) ||
          !IsValidNoiseStd(vel_ne_std_mps) ||
          !IsValidNoiseStd(vel_d_std_mps))
      {
        // 零速/速度量测异常时不更新，也不记录到延迟回放事件。
        return result;
      }
      result.input_valid = true;
      result = ApplyVelocityUpdateDetailed(measured_vel_ned,
                                           vel_ne_std_mps,
                                           vel_d_std_mps);
      if (result.fused)
      {
        RecordVelocityUpdateOnLatestSnapshot(measured_vel_ned, vel_ne_std_mps, vel_d_std_mps);
      }
      return result;
    }

    /**
     * @brief 加速度重力方向量测更新，用于无 GNSS 时约束横滚和俯仰。
     *
     * 原始 TimeUpdate 只是用加速度递推速度/位置，不会持续用重力方向修正姿态。
     * 该接口把加速度计在低动态或静止时测得的重力方向作为单位矢量观测，
     * 形成类似互补滤波的 roll/pitch 闭环。航向角不由重力观测约束。
     *
     * @param measured_accel_mps2 当前机体系 FRD 加速度计读数，单位 m/s^2。
     * @param accel_dir_noise_rad 重力方向观测噪声，近似等价于姿态角噪声。
     * @return true 表示该重力量测通过了残差/NIS检查并已实际融合；false 表示被拒绝或输入无效。
     */
    bool MeasurementUpdateGravity(const Eigen::Vector3f &measured_accel_mps2,
                                  const float accel_dir_noise_rad)
    {
      return MeasurementUpdateGravityDetailed(measured_accel_mps2,
                                              accel_dir_noise_rad)
          .fused;
    }

    /**
     * @brief 带诊断结果的重力方向量测更新。
     *
     * 返回值用于区分“输入无效”“进入 EKF 但被门控拒绝”和“实际融合成功”，
     * 便于后续把静止辅助调度和组合导航观测状态从测试程序抽到库层。
     */
    MeasurementUpdateResult MeasurementUpdateGravityDetailed(
        const Eigen::Vector3f &measured_accel_mps2,
        const float accel_dir_noise_rad)
    {
      MeasurementUpdateResult result;
      if (!IsFiniteVector(measured_accel_mps2) || !IsValidNoiseStd(accel_dir_noise_rad))
      {
        // 加速度方向或噪声异常时不把坏重力方向写入姿态闭环。
        return result;
      }
      result.input_valid = true;
      result = ApplyGravityUpdateDetailed(measured_accel_mps2, accel_dir_noise_rad);
      if (result.fused)
      {
        RecordGravityUpdateOnLatestSnapshot(measured_accel_mps2, accel_dir_noise_rad);
      }
      return result;
    }

    /**
     * @brief 静止角速度量测更新，用于长静止时约束陀螺零偏和航向漂移。
     *
     * 重力方向只能约束 roll/pitch，无法直接观测陀螺零偏；静止且 GNSS/磁航向不可用时，
     * 未收敛的 gyro_bias 会通过积分变成姿态漂移。该接口只把“静止时应测到的地球/导航系
     * 角速度 + gyro_bias”作为三轴陀螺零偏量测，不直接把残差注入姿态。姿态漂移会在后续
     * 传播中因为零偏被修正而停止继续扩大；roll/pitch 仍由 Gravity 负责低频对齐。
     *
     * @param measured_gyro_radps 当前机体系 FRD 原始角速度，单位 rad/s，不能预扣静止均值。
     * @param gyro_noise_radps 静止角速度量测标准差，单位 rad/s。
     * @return true 表示量测通过残差/NIS 检查并实际融合。
     */
    bool MeasurementUpdateStaticGyro(const Eigen::Vector3f &measured_gyro_radps,
                                     const float gyro_noise_radps)
    {
      return MeasurementUpdateStaticGyroDetailed(measured_gyro_radps,
                                                 gyro_noise_radps)
          .fused;
    }

    bool MeasurementUpdateStaticGyro(const Eigen::Vector3f &measured_gyro_radps,
                                     const Eigen::Vector3f &gyro_noise_radps)
    {
      return MeasurementUpdateStaticGyroDetailed(measured_gyro_radps,
                                                 gyro_noise_radps)
          .fused;
    }

    /**
     * @brief 带诊断结果的静止角速度量测更新。
     *
     * 该量测只应由上层静止检测确认后调用。它使用三轴静止角速度残差估计 gyro_bias，
     * 但不直接回写姿态误差；roll/pitch 的绝对方向仍由 Gravity 量测负责，yaw 在无磁/
     * 无 GNSS 航向时不当作绝对可观状态，只尽量阻止零偏继续积分成漂移。
     */
    MeasurementUpdateResult MeasurementUpdateStaticGyroDetailed(
        const Eigen::Vector3f &measured_gyro_radps,
        const float gyro_noise_radps)
    {
      return MeasurementUpdateStaticGyroDetailed(
          measured_gyro_radps,
          (Eigen::Vector3f() << gyro_noise_radps, gyro_noise_radps, gyro_noise_radps)
              .finished());
    }

    MeasurementUpdateResult MeasurementUpdateStaticGyroDetailed(
        const Eigen::Vector3f &measured_gyro_radps,
        const Eigen::Vector3f &gyro_noise_radps)
    {
      MeasurementUpdateResult result;
      if (!IsFiniteVector(measured_gyro_radps) ||
          !IsValidStaticGyroNoiseVector(gyro_noise_radps))
      {
        return result;
      }
      result.input_valid = true;
      result = ApplyStaticGyroUpdateDetailed(measured_gyro_radps, gyro_noise_radps);
      if (result.fused)
      {
        RecordStaticGyroUpdateOnLatestSnapshot(measured_gyro_radps, gyro_noise_radps);
      }
      return result;
    }

    /**
     * @brief 姿态量测更新，用于无 GNSS 时约束 EKF 姿态不发散。
     *
     * 量测向量顺序与库内部保持一致: [Yaw, Pitch, Roll]。该接口适合融合
     * Madgwick/Mahony 等 IMU AHRS 输出，作为 EKF 纯陀螺积分姿态的低频约束。
     *
     * @param measured_ypr_rad 姿态量测 [Yaw, Pitch, Roll]，单位 rad。
     * @param roll_pitch_noise_rad 横滚/俯仰量测噪声标准差。
     * @param yaw_noise_rad 航向量测噪声标准差。
     */
    void MeasurementUpdateAttitude(const Eigen::Vector3f &measured_ypr_rad,
                                   const float roll_pitch_noise_rad,
                                   const float yaw_noise_rad)
    {
      (void)MeasurementUpdateAttitudeDetailed(measured_ypr_rad,
                                              roll_pitch_noise_rad,
                                              yaw_noise_rad);
    }

    /**
     * @brief 带诊断结果的姿态量测更新。
     *
     * 姿态量测通常来自 Madgwick/Mahony/外部 AHRS。详细结果可以输出姿态残差、
     * NIS 门控比，并作为低动态姿态辅助是否过强的调参依据。
     */
    MeasurementUpdateResult MeasurementUpdateAttitudeDetailed(
        const Eigen::Vector3f &measured_ypr_rad,
        const float roll_pitch_noise_rad,
        const float yaw_noise_rad)
    {
      MeasurementUpdateResult result;
      if (!IsFiniteVector(measured_ypr_rad) ||
          !IsValidNoiseStd(roll_pitch_noise_rad) ||
          !IsValidNoiseStd(yaw_noise_rad))
      {
        // AHRS姿态量测异常时直接丢弃，避免角度环绕或矩阵更新进入NaN。
        return result;
      }
      result.input_valid = true;
      result = ApplyAttitudeUpdateDetailed(measured_ypr_rad,
                                           roll_pitch_noise_rad,
                                           yaw_noise_rad);
      if (result.fused)
      {
        RecordAttitudeUpdateOnLatestSnapshot(measured_ypr_rad, roll_pitch_noise_rad, yaw_noise_rad);
      }
      return result;
    }

    /**
     * @brief 标量航向观测更新 (Scalar Yaw Measurement Update)
     *
     * 专门用于处理单轴航向观测（如双矢量法计算出的航向、双天线航向或磁罗盘航向）。
     * 相比通用的矩阵更新，此函数针对 H = [0...0, 1, 0...0] 进行了稀疏性优化，
     * 极大地减少了计算量，适合高频调用。
     *
     * @param measured_yaw_rad 观测到的绝对航向角 (单位: 弧度, 范围: -PI ~ PI)
     * @param measurement_noise_rad 观测噪声标准差 (单位: 弧度)
     */
    void MeasurementUpdateYaw(const float measured_yaw_rad, const float measurement_noise_rad)
    {
      (void)MeasurementUpdateYawDetailed(measured_yaw_rad, measurement_noise_rad);
    }

    /**
     * @brief 带诊断结果的标量航向量测更新。
     *
     * 后续接 EKF-GSF yaw、双天线航向或磁航向时，上层可以直接用该结果统计航向
     * 观测的接受率、残差和门控比。
     */
    MeasurementUpdateResult MeasurementUpdateYawDetailed(
        const float measured_yaw_rad,
        const float measurement_noise_rad)
    {
      MeasurementUpdateResult result;
      if (!std::isfinite(measured_yaw_rad) || !IsValidNoiseStd(measurement_noise_rad))
      {
        // 航向量测异常时不更新，避免WrapAngleRad处理Inf导致死循环。
        return result;
      }
      result.input_valid = true;
      result = ApplyYawUpdateDetailed(measured_yaw_rad, measurement_noise_rad);
      if (result.fused)
      {
        RecordYawUpdateOnLatestSnapshot(measured_yaw_rad, measurement_noise_rad);
      }
      return result;
    }

    /**
     * @brief 直接重锚定绝对位置和速度，用于无GNSS假原点启动后首次获得真实GNSS。
     *
     * 该接口不重置姿态、IMU零偏和协方差中的姿态/零偏部分，只把地理位置、NED速度
     * 切换到可信GNSS基准，避免几百公里级假原点残差被GNSS门控永久拒绝。
     */
    void ResetPositionVelocityToGnss(const Eigen::Vector3f &gnss_vel,
                                     const Eigen::Vector3d &gnss_lla)
    {
      if (!IsFiniteVector(gnss_vel) || !IsValidLlaForEarthModel(gnss_lla))
      {
        return;
      }

      ins_lla_rad_m_ = gnss_lla;
      ins_ned_vel_mps_ = gnss_vel;
      p_.block(0, 0, 3, 3) = init_pos_err_std_m_ * init_pos_err_std_m_ *
                             Eigen::Matrix3f::Identity();
      p_.block(3, 3, 3, 3) = init_vel_err_std_mps_ * init_vel_err_std_mps_ *
                             Eigen::Matrix3f::Identity();
      StabilizeCovariance();
      time_since_gnss_update_s_ = 0.0f;
      gnss_reacquire_timer_s_ = 0.0f;

      ResetHistoryBuffer();
    }

    // ---- EKF 全量状态读取 (只读 getter) ----
    inline Eigen::Vector3f accel_bias_mps2() const { return accel_bias_mps2_; }
    inline Eigen::Vector3f gyro_bias_radps() const { return gyro_bias_radps_; }
    inline Eigen::Vector3f accel_mps2() const { return ins_accel_mps2_; }
    inline Eigen::Vector3f gyro_radps() const { return ins_gyro_radps_; }
    inline Eigen::Vector3f ned_vel_mps() const { return ins_ned_vel_mps_; }
    inline float yaw_rad() const
    {
      EnsureEulerAnglesUpToDate();
      return ins_ypr_rad_(0);
    }
    inline float pitch_rad() const
    {
      EnsureEulerAnglesUpToDate();
      return ins_ypr_rad_(1);
    }
    inline float roll_rad() const
    {
      EnsureEulerAnglesUpToDate();
      return ins_ypr_rad_(2);
    }
    inline Eigen::Vector3d lla_rad_m() const { return ins_lla_rad_m_; }
    // 直接获取 EKF 估计的四元数姿态 (FRD 机体系 → NED 导航系)
    inline Eigen::Quaternionf quat() const { return quat_; }

    // 当前 WGS-84 正常重力（Somigliana，随经纬高更新；未初始化时为默认标准值 9.80665）
    inline double gravity_mps2() const { return gravity_mps2_; }
    inline double lat_rad() const { return ins_lla_rad_m_(0); }
    inline double lon_rad() const { return ins_lla_rad_m_(1); }
    inline double alt_m() const { return ins_lla_rad_m_(2); }
    inline bool covariance_is_healthy() const
    {
      // 只读健康检查供主机仿真和调试使用：不改状态，只验证P矩阵没有进入数值失效区。
      constexpr float SYMMETRY_TOLERANCE = 1.0e-3f;
      constexpr float MIN_VARIANCE_TOLERANCE = -1.0e-6f;
      for (int row = 0; row < 15; ++row)
      {
        const float diagonal = p_(row, row);
        if (!std::isfinite(diagonal) || diagonal < MIN_VARIANCE_TOLERANCE)
        {
          return false;
        }
        for (int col = row + 1; col < 15; ++col)
        {
          const float upper = p_(row, col);
          const float lower = p_(col, row);
          if (!std::isfinite(upper) || !std::isfinite(lower))
          {
            return false;
          }
          if (std::fabs(upper - lower) > SYMMETRY_TOLERANCE)
          {
            return false;
          }
        }
      }
      return true;
    }
    inline bool covariance_is_positive_semidefinite() const
    {
      // 主机仿真用严格检查：对称且特征值非负才说明P矩阵仍是合法协方差。
      if (!covariance_is_healthy())
      {
        return false;
      }
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 15, 15>> eigensolver(p_);
      if (eigensolver.info() != Eigen::Success)
      {
        return false;
      }
      const Eigen::Matrix<float, 15, 1> eigenvalues = eigensolver.eigenvalues();
      const float trace_scale = std::max(std::fabs(p_.trace()), eigenvalues.cwiseAbs().maxCoeff());
      const float min_eigenvalue_tolerance =
          -std::max(MIN_COVARIANCE_PSD_ABSOLUTE_TOLERANCE,
                    trace_scale * RELATIVE_COVARIANCE_PSD_TOLERANCE);
      return eigenvalues.minCoeff() >= min_eigenvalue_tolerance;
    }
    inline float covariance_min_eigenvalue() const
    {
      // 主机仿真/调参记录用：返回P矩阵最小特征值，便于定位协方差传播退化程度。
      if (!covariance_is_healthy())
      {
        return -std::numeric_limits<float>::infinity();
      }
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 15, 15>> eigensolver(p_);
      if (eigensolver.info() != Eigen::Success)
      {
        return -std::numeric_limits<float>::infinity();
      }
      return eigensolver.eigenvalues().minCoeff();
    }
    inline float covariance_trace() const
    {
      // 主机仿真/调参记录用：返回P矩阵迹，用于评估协方差数值尺度。
      return p_.trace();
    }
    inline float covariance_max_abs_coeff() const
    {
      // 主机仿真/调参记录用：返回P矩阵最大绝对元素，用于判断PSD退化是否来自病态尺度。
      return p_.cwiseAbs().maxCoeff();
    }

#ifdef EKF_HOST_REGRESSION
    inline float covariance_coeff_for_test(const int row, const int col) const
    {
      if (row < 0 || row >= 15 || col < 0 || col >= 15)
      {
        return std::numeric_limits<float>::quiet_NaN();
      }
      return p_(row, col);
    }

    inline float state_transition_coeff_for_test(const int row, const int col) const
    {
      if (row < 0 || row >= 15 || col < 0 || col >= 15)
      {
        return std::numeric_limits<float>::quiet_NaN();
      }
      return phi_(row, col);
    }

    /*
     * 主机回归专用：暴露连续时间状态雅可比 fs_(row,col)。
     * fs_ 在 PropagateStateFromIncrements() 中按当前 IMU 输入和地理项重建，
     * 主机测试可借此验证 F(V,V) 哥氏力、F(P,P) 自耦合、F(Φ,V)=-C^b_n*M
     * 等关键非零块的符号、坐标系和数值与公式严格一致。
     */
    inline float fs_coeff_for_test(const int row, const int col) const
    {
      if (row < 0 || row >= 15 || col < 0 || col >= 15)
      {
        return std::numeric_limits<float>::quiet_NaN();
      }
      return fs_(row, col);
    }

    /*
     * 主机回归专用：暴露 t_b2ned (C^n_b)，用于 F(Φ,V)=-(t_b2ned^T)*M 公式核对。
     */
    inline float t_b2ned_coeff_for_test(const int row, const int col) const
    {
      if (row < 0 || row >= 3 || col < 0 || col >= 3)
      {
        return std::numeric_limits<float>::quiet_NaN();
      }
      return t_b2ned(row, col);
    }

    /*
     * 主机回归专用：暴露 NED 速度 ins_ned_vel_mps_(idx)，验证双子样划摇/圆锥
     * 补偿后的姿态-速度推进结果是否与 IMU 增量手算一致。
     */
    inline float ned_vel_for_test(const int idx) const
    {
      if (idx < 0 || idx >= 3)
      {
        return std::numeric_limits<float>::quiet_NaN();
      }
      return ins_ned_vel_mps_(idx);
    }

    /*
     * 主机回归专用：暴露当前姿态四元数 (w,x,y,z by idx 0..3)，
     * 用于对照圆锥补偿后的等效旋转向量是否与 (2/3) θ_1×θ_2 修正后一致。
     */
    inline float quat_for_test(const int idx) const
    {
      if (idx < 0 || idx >= 4)
      {
        return std::numeric_limits<float>::quiet_NaN();
      }
      switch (idx)
      {
      case 0:
        return quat_.w();
      case 1:
        return quat_.x();
      case 2:
        return quat_.y();
      case 3:
        return quat_.z();
      default:
        return std::numeric_limits<float>::quiet_NaN();
      }
    }
#endif

  private:
    static bool IsValidNoiseStd(const float std_value)
    {
      // 所有量测噪声标准差必须有限且为正，防止R矩阵奇异或被NaN污染。
      return std::isfinite(std_value) && (std_value > 0.0f);
    }

    static float ClampMeasurementNoiseStd(const float std_value)
    {
      // 外部精度可能给出极小正数；滤波内部用下限避免R矩阵数值奇异。
      if (!IsValidNoiseStd(std_value))
      {
        return MIN_MEASUREMENT_NOISE_STD;
      }
      return std::max(std_value, MIN_MEASUREMENT_NOISE_STD);
    }

    static float ClampStaticGyroNoiseStd(const float std_value)
    {
      /*
       * 静止陀螺量测单位是 rad/s，不是姿态角 rad 或速度 m/s。全局量测噪声下限
       * 0.01 对它过大，会让 0.002~0.004 rad/s 这类实测调参完全失效，因此这里用
       * 独立下限，只防止 R 接近奇异。
       */
      if (!IsValidNoiseStd(std_value))
      {
        return MIN_STATIC_GYRO_MEASUREMENT_NOISE_STD;
      }
      return std::max(std_value, MIN_STATIC_GYRO_MEASUREMENT_NOISE_STD);
    }

    static bool IsValidStaticGyroNoiseVector(const Eigen::Vector3f &std_value)
    {
      // 三轴静止陀螺量测允许不同噪声，但每个轴都必须是有限正数。
      return IsValidNoiseStd(std_value(0)) &&
             IsValidNoiseStd(std_value(1)) &&
             IsValidNoiseStd(std_value(2));
    }

    static Eigen::Vector3f ClampStaticGyroNoiseStd(const Eigen::Vector3f &std_value)
    {
      /*
       * 轴向噪声用于现场权衡：X/Y 轴过强会影响 roll/pitch，Z 轴则主要压 yaw 漂移。
       * 每轴仍套独立下限，避免任一 R 对角线接近 0。
       */
      return (Eigen::Vector3f() << ClampStaticGyroNoiseStd(std_value(0)),
              ClampStaticGyroNoiseStd(std_value(1)),
              ClampStaticGyroNoiseStd(std_value(2)))
          .finished();
    }

    static bool IsFiniteVector(const Eigen::Vector3f &vec)
    {
      // 对外部float量测做统一有限值检查，避免NaN/Inf进入滤波矩阵。
      return std::isfinite(vec(0)) && std::isfinite(vec(1)) && std::isfinite(vec(2));
    }

    static bool IsFiniteVector(const Eigen::Vector3d &vec)
    {
      // 对外部double位置量测做统一有限值检查，避免NaN/Inf进入地理坐标换算。
      return std::isfinite(vec(0)) && std::isfinite(vec(1)) && std::isfinite(vec(2));
    }

    template <int Size>
    static bool IsFiniteMatrix(const Eigen::Matrix<float, Size, Size> &matrix)
    {
      // 矩阵反演前后都做有限值检查，避免Inf/NaN通过卡尔曼增益扩散。
      for (int row = 0; row < Size; ++row)
      {
        for (int col = 0; col < Size; ++col)
        {
          if (!std::isfinite(matrix(row, col)))
          {
            return false;
          }
        }
      }
      return true;
    }

    template <int Size>
    static bool InvertSymmetricPositiveDefiniteMatrix(
        const Eigen::Matrix<float, Size, Size> &matrix,
        Eigen::Matrix<float, Size, Size> *inverse)
    {
      // S矩阵理论上应为对称正定；用LDLT求解比直接inverse更稳，并能检测病态矩阵。
      if (inverse == nullptr || !IsFiniteMatrix(matrix))
      {
        return false;
      }
      Eigen::LDLT<Eigen::Matrix<float, Size, Size>> ldlt(matrix);
      if (ldlt.info() != Eigen::Success || !ldlt.isPositive())
      {
        return false;
      }
      *inverse = ldlt.solve(Eigen::Matrix<float, Size, Size>::Identity());
      return IsFiniteMatrix(*inverse);
    }

    bool ComputeGnssInnovationStats(const Eigen::Matrix<float, 6, 6> &measurement_noise,
                                    Eigen::Matrix<float, 6, 6> *s_inv,
                                    float *nis_value)
    {
      if (s_inv == nullptr || nis_value == nullptr || !IsFiniteMatrix(measurement_noise))
      {
        return false;
      }
      s_ = h_ * p_ * h_.transpose() + measurement_noise;
      if (!InvertSymmetricPositiveDefiniteMatrix(s_, s_inv))
      {
        return false;
      }
      const Eigen::Matrix<float, 1, 1> nis = y_.transpose() * (*s_inv) * y_;
      if (!std::isfinite(nis(0, 0)))
      {
        return false;
      }
      *nis_value = nis(0, 0);
      return true;
    }

    bool IsGnssReacquisitionResidualPlausible() const
    {
      const float pos_residual_ne_m = y_.segment(0, 2).norm();
      const float pos_residual_d_m = std::fabs(y_(2));
      const float vel_residual_ne_mps = y_.segment(3, 2).norm();
      const float vel_residual_d_mps = std::fabs(y_(5));
      if (!std::isfinite(pos_residual_ne_m) || !std::isfinite(pos_residual_d_m) ||
          !std::isfinite(vel_residual_ne_mps) || !std::isfinite(vel_residual_d_mps))
      {
        return false;
      }

      const float safe_pos_ne_std_m = ClampMeasurementNoiseStd(gnss_pos_ne_std_m_);
      const float safe_pos_d_std_m = ClampMeasurementNoiseStd(gnss_pos_d_std_m_);
      const float pos_ne_residual_limit_m =
          std::min(GNSS_REACQUIRE_MAX_POS_NE_RESIDUAL_M,
                   std::max(GNSS_REACQUIRE_MIN_POS_NE_RESIDUAL_GATE_M,
                            safe_pos_ne_std_m * GNSS_REACQUIRE_RESIDUAL_SIGMA_GATE));
      const float pos_d_residual_limit_m =
          std::min(GNSS_REACQUIRE_MAX_POS_D_RESIDUAL_M,
                   std::max(GNSS_REACQUIRE_MIN_POS_D_RESIDUAL_GATE_M,
                            safe_pos_d_std_m * GNSS_REACQUIRE_RESIDUAL_SIGMA_GATE));

      return (pos_residual_ne_m <= pos_ne_residual_limit_m) &&
             (pos_residual_d_m <= pos_d_residual_limit_m) &&
             (vel_residual_ne_mps <= GNSS_REACQUIRE_MAX_VEL_NE_RESIDUAL_MPS) &&
             (vel_residual_d_mps <= GNSS_REACQUIRE_MAX_VEL_D_RESIDUAL_MPS);
    }

    bool BuildConservativeGnssReacquisitionNoise(Eigen::Matrix<float, 6, 6> *noise) const
    {
      if (noise == nullptr || !IsFiniteMatrix(r_))
      {
        return false;
      }

      const bool has_gnss_outage =
          (time_since_gnss_update_s_ >= GNSS_REACQUIRE_MIN_OUTAGE_S) ||
          (gnss_reacquire_timer_s_ > 0.0f);
      const bool residual_is_plausible = IsGnssReacquisitionResidualPlausible();
      if (!has_gnss_outage || !residual_is_plausible)
      {
        return false;
      }

      *noise = r_;
      (*noise)(0, 0) += GNSS_REACQUIRE_POS_NE_EXTRA_STD_M * GNSS_REACQUIRE_POS_NE_EXTRA_STD_M;
      (*noise)(1, 1) += GNSS_REACQUIRE_POS_NE_EXTRA_STD_M * GNSS_REACQUIRE_POS_NE_EXTRA_STD_M;
      (*noise)(2, 2) += GNSS_REACQUIRE_POS_D_EXTRA_STD_M * GNSS_REACQUIRE_POS_D_EXTRA_STD_M;
      (*noise)(3, 3) += GNSS_REACQUIRE_VEL_NE_EXTRA_STD_MPS * GNSS_REACQUIRE_VEL_NE_EXTRA_STD_MPS;
      (*noise)(4, 4) += GNSS_REACQUIRE_VEL_NE_EXTRA_STD_MPS * GNSS_REACQUIRE_VEL_NE_EXTRA_STD_MPS;
      (*noise)(5, 5) += GNSS_REACQUIRE_VEL_D_EXTRA_STD_MPS * GNSS_REACQUIRE_VEL_D_EXTRA_STD_MPS;
      return true;
    }

    static Eigen::Vector3f SanitizeVectorOrZero(const Eigen::Vector3f &vec)
    {
      // 通用float向量兜底：坏速度或坏陀螺输入按静止处理，优先保证初始化有限。
      if (IsFiniteVector(vec))
      {
        return vec;
      }
      return Eigen::Vector3f::Zero();
    }

    static Eigen::Vector3f SanitizeAccelForInit(const Eigen::Vector3f &accel)
    {
      // 初始化调平必须依赖有效重力方向；坏加表或近零向量时采用机体系静止重力默认值。
      if (IsFiniteVector(accel) && accel.norm() > 1.0f)
      {
        return accel;
      }
      return (Eigen::Vector3f() << 0.0f, 0.0f, -G_MPS2).finished();
    }

    static Eigen::Vector3f SanitizeMagForInit(const Eigen::Vector3f &mag)
    {
      // 初始航向需要有效磁场方向；坏磁力计或零磁场时采用机头朝北的保守默认值。
      if (IsFiniteVector(mag) && mag.norm() > 1.0e-6f)
      {
        return mag;
      }
      return (Eigen::Vector3f() << 1.0f, 0.0f, 0.0f).finished();
    }

    static Eigen::Vector3d SanitizeLlaForInit(const Eigen::Vector3d &lla)
    {
      // 没有可靠GNSS或LLA异常时使用长沙附近伪原点，保证地球模型计算有有限输入。
      if (IsValidLlaForEarthModel(lla))
      {
        return lla;
      }
      return (Eigen::Vector3d() << 28.2 * 3.14159265358979323846 / 180.0,
              112.9 * 3.14159265358979323846 / 180.0,
              50.0)
          .finished();
    }

    static bool IsValidLlaForEarthModel(const Eigen::Vector3d &lla)
    {
      // 地球模型要求纬度远离极点，经度在[-pi, pi]附近，高度在飞控可用范围内。
      constexpr double MIN_COS_LAT = 1.0e-3;
      constexpr double MAX_ABS_ALT_M = 100000.0;
      if (!IsFiniteVector(lla))
      {
        return false;
      }
      if (std::fabs(lla(0)) >= (0.5 * 3.14159265358979323846 - MIN_COS_LAT))
      {
        return false;
      }
      if (std::fabs(lla(1)) > (3.14159265358979323846 + 1.0e-6))
      {
        return false;
      }
      if (std::fabs(lla(2)) > MAX_ABS_ALT_M)
      {
        return false;
      }
      return true;
    }

    static bool IsPlausibleImuSample(const Eigen::Vector3f &accel,
                                     const Eigen::Vector3f &gyro)
    {
      // ICM42688配置为±8g、±2000dps；这里给少量裕度，拒绝明显越界的有限坏帧。
      return (accel.norm() <= MAX_PLAUSIBLE_ACCEL_MPS2) &&
             (gyro.norm() <= MAX_PLAUSIBLE_GYRO_RADPS);
    }

    static bool IsValidTimeStep(const float dt_s)
    {
      // 正常导航周期约为5ms；100ms以上通常代表调度卡顿或时间戳异常，不应参与积分。
      return std::isfinite(dt_s) && (dt_s > 0.0f) && (dt_s <= MAX_TIME_UPDATE_DT_S);
    }

    void GrowGnssOutageCovariance(const float dt_s)
    {
      if (time_since_gnss_update_s_ < GNSS_REACQUIRE_MIN_OUTAGE_S)
      {
        return;
      }

      // GNSS长时间失联后，绝对位置/速度误差通常比IMU白噪声传播更快增长。
      // 这里只膨胀可由GNSS直接观测的位置和速度协方差，不触碰姿态闭环。
      const float outage_scale =
          std::min(time_since_gnss_update_s_ / GNSS_REACQUIRE_MIN_OUTAGE_S, 4.0f);
      const float pos_growth_ne = GNSS_OUTAGE_POS_NE_GROWTH_MPS *
                                  GNSS_OUTAGE_POS_NE_GROWTH_MPS * dt_s * outage_scale;
      const float pos_growth_d = GNSS_OUTAGE_POS_D_GROWTH_MPS *
                                 GNSS_OUTAGE_POS_D_GROWTH_MPS * dt_s * outage_scale;
      const float vel_growth_ne = GNSS_OUTAGE_VEL_NE_GROWTH_MPS2 *
                                  GNSS_OUTAGE_VEL_NE_GROWTH_MPS2 * dt_s * outage_scale;
      const float vel_growth_d = GNSS_OUTAGE_VEL_D_GROWTH_MPS2 *
                                 GNSS_OUTAGE_VEL_D_GROWTH_MPS2 * dt_s * outage_scale;

      p_(0, 0) += pos_growth_ne;
      p_(1, 1) += pos_growth_ne;
      p_(2, 2) += pos_growth_d;
      p_(3, 3) += vel_growth_ne;
      p_(4, 4) += vel_growth_ne;
      p_(5, 5) += vel_growth_d;
      StabilizeCovariance();
    }

    static float WrapAngleRad(float angle_rad)
    {
      if (!std::isfinite(angle_rad))
      {
        // 非有限角度没有可定义的环绕结果，返回0避免调用方卡死在while循环。
        return 0.0f;
      }
      // fmod + 单次归位代替不定次 while：耗时确定，且对极端大角度输入不会卡循环。
      const float PI_F = 3.14159265358979323846f;
      const float TWO_PI_F = 2.0f * PI_F;
      angle_rad = std::fmod(angle_rad, TWO_PI_F);
      if (angle_rad > PI_F)
      {
        angle_rad -= TWO_PI_F;
      }
      else if (angle_rad < -PI_F)
      {
        angle_rad += TWO_PI_F;
      }
      return angle_rad;
    }

    static Eigen::Quaternionf BuildDeltaQuatFromRotationVector(const Eigen::Vector3f &delta_theta_rad)
    {
      const float delta_angle_rad = delta_theta_rad.norm();
      Eigen::Quaternionf delta_quat;
      if (delta_angle_rad > 1.0e-6f)
      {
        // 旋转向量转四元数使用指数映射；IMU传播和误差状态反馈共用，避免高角度时一阶近似低估转角。
        const float half_angle_rad = 0.5f * delta_angle_rad;
        const float vector_scale = std::sin(half_angle_rad) / delta_angle_rad;
        delta_quat.w() = std::cos(half_angle_rad);
        delta_quat.x() = vector_scale * delta_theta_rad(0);
        delta_quat.y() = vector_scale * delta_theta_rad(1);
        delta_quat.z() = vector_scale * delta_theta_rad(2);
      }
      else
      {
        // 极小角度退化到一阶近似，避免除零并保持静止/低速更新的数值连续性。
        delta_quat.w() = 1.0f;
        delta_quat.x() = 0.5f * delta_theta_rad(0);
        delta_quat.y() = 0.5f * delta_theta_rad(1);
        delta_quat.z() = 0.5f * delta_theta_rad(2);
      }
      return delta_quat.normalized();
    }

    void StabilizeCovariance()
    {
      propagation_stabilize_skip_count_ = 0U;
      static_aid_stabilize_skip_count_ = 0U;
      // 理论上Joseph更新和Phi*P*Phi^T+Q会保持P为PSD；单精度和强耦合高动态下仍可能出现负特征值。
      // 常规路径只做对称化和LDLT快速检查；只有矩阵可疑时才做特征值投影，避免在200Hz任务中每步分解15x15矩阵。
      const float gershgorin_lower_bound =
          SymmetrizeAndComputeGershgorinLowerBound(&p_);
#if BFS_NAVIGATION_EMBEDDED_FAST_STABILIZE
      /*
       * ESP32 实机快速稳定化：
       * 原始路径每次都对 15x15 协方差做 LDLT 检查，必要时还做特征分解。静止桌面测试
       * 实测 TimeUpdate 中这部分开销很可观。fast 模式下每步仍执行对称化、有限值检查和
       * 对角方差下限保护；只有发现明显异常，或到达周期性完整检查点时，才进入完整
       * LDLT/特征值投影路径。
       *
       * 该宏默认关闭，只给 ICM45686 bring-up 测试压榨 ESP32-S3 性能。若用于飞控闭环，
       * 必须用高动态数据重新验证 PSD、NIS、状态漂移和故障恢复行为。
       */
      covariance_stabilize_count_++;
      bool require_full_check =
          (BFS_NAVIGATION_EMBEDDED_FULL_STABILIZE_PERIOD > 0U) &&
          ((covariance_stabilize_count_ %
            BFS_NAVIGATION_EMBEDDED_FULL_STABILIZE_PERIOD) == 0U);
      for (int row = 0; row < 15; ++row)
      {
        for (int col = 0; col < 15; ++col)
        {
          if (!std::isfinite(p_(row, col)))
          {
            require_full_check = true;
            p_(row, col) = (row == col) ? MIN_COVARIANCE_EIGENVALUE : 0.0f;
          }
        }
        if (p_(row, row) < MIN_COVARIANCE_EIGENVALUE)
        {
          require_full_check = true;
          p_(row, row) = MIN_COVARIANCE_EIGENVALUE;
        }
      }
      if (!require_full_check &&
          gershgorin_lower_bound >= MIN_COVARIANCE_ALLOWED_EIGENVALUE)
      {
        return;
      }
#endif
      /*
       * 对当前纯惯导台架路径，绝大多数 P 矩阵都健康。Gershgorin 下界一旦非负，就已经足够证明
       * 协方差为 PSD，不必再做 15x15 LDLT。这样可以明显降低常规稳定化路径的平均耗时。
       */
      if (std::isfinite(gershgorin_lower_bound) &&
          gershgorin_lower_bound >= MIN_COVARIANCE_ALLOWED_EIGENVALUE)
      {
        return;
      }
      Eigen::LDLT<Eigen::Matrix<float, 15, 15>> ldlt(p_);
      if (ldlt.info() == Eigen::Success && ldlt.isPositive())
      {
        /*
         * Gershgorin 下界只是“保守快速判据”：
         * - 下界非负时，可直接证明矩阵为 PSD，前面已经提前返回。
         * - 下界为负时，并不代表矩阵真的坏了，只说明 Gershgorin 判据过于保守。
         *
         * 这时如果 LDLT 仍确认当前 P 矩阵正定，就没必要再走更昂贵的特征分解。
         * 之前这里错误地又附带了 Gershgorin 条件，导致很多“LDLT 已经通过”的正常情形
         * 仍会继续落入 SelfAdjointEigenSolver，白白增加 200 Hz 热路径上的 CPU 开销。
         */
        return;
      }
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 15, 15>> eigensolver(p_);
      if (eigensolver.info() != Eigen::Success)
      {
        // 特征分解失败时无法安全投影，退化为最小对角抖动，至少保持对角线为正。
        p_ += MIN_COVARIANCE_EIGENVALUE * Eigen::Matrix<float, 15, 15>::Identity();
        return;
      }

      Eigen::Matrix<float, 15, 1> eigenvalues = eigensolver.eigenvalues();
      if (eigenvalues.minCoeff() >= 0.0f)
      {
        return;
      }
      const float projection_floor =
          std::max(MIN_COVARIANCE_EIGENVALUE,
                   eigenvalues.maxCoeff() * RELATIVE_COVARIANCE_NEGATIVE_EIGEN_FLOOR);
      for (int idx = 0; idx < 15; ++idx)
      {
        if (eigenvalues(idx) < projection_floor)
        {
          // 投影只修复负特征值或接近零的数值毛刺；相对地板取很小值，避免位置大方差把姿态/零偏模态抬爆。
          eigenvalues(idx) = projection_floor;
        }
      }
      p_ = eigensolver.eigenvectors() * eigenvalues.asDiagonal() *
           eigensolver.eigenvectors().transpose();
      p_ = 0.5f * (p_ + p_.transpose().eval());
    }

    void StabilizeCovarianceAfterPropagation()
    {
      if (propagation_stabilize_divider_ <= 1U)
      {
        StabilizeCovariance();
      }
      else
      {
        propagation_stabilize_skip_count_++;
        bool require_full_stabilize = false;
        for (int idx = 0; idx < 15; ++idx)
        {
          const float diag = p_(idx, idx);
          if (!std::isfinite(diag) || diag < MIN_COVARIANCE_ALLOWED_EIGENVALUE)
          {
            require_full_stabilize = true;
            break;
          }
        }
        if (!require_full_stabilize &&
            propagation_stabilize_skip_count_ <
                propagation_stabilize_divider_)
        {
          return;
        }
        StabilizeCovariance();
      }
    }

    void StabilizeCovarianceAfterStaticAidUpdate()
    {
      if (static_aid_stabilize_divider_ <= 1U)
      {
        StabilizeCovariance();
      }
      else
      {
        /*
         * 只对静止辅助量测的协方差稳定化做独立降频：
         * - 传播路径已有单独 divider，不复用计数器，避免两条热路径互相锁相。
         * - GNSS/姿态/航向等其余量测仍保持每次完整稳定化。
         * - 一旦对角线出现非有限值或负方差，立即退回完整稳定化。
         */
        static_aid_stabilize_skip_count_++;
        bool require_full_stabilize = false;
        for (int idx = 0; idx < 15; ++idx)
        {
          const float diag = p_(idx, idx);
          if (!std::isfinite(diag) || diag < MIN_COVARIANCE_ALLOWED_EIGENVALUE)
          {
            require_full_stabilize = true;
            break;
          }
        }
        if (!require_full_stabilize &&
            static_aid_stabilize_skip_count_ <
                static_aid_stabilize_divider_)
        {
          return;
        }
        StabilizeCovariance();
      }
    }

    static float GershgorinLowerBound(const Eigen::Matrix<float, 15, 15> &matrix)
    {
      // Gershgorin下界是保守的快速PSD筛查：下界非负时可直接认为矩阵正定，不需要特征分解。
      float lower_bound = std::numeric_limits<float>::infinity();
      for (int row = 0; row < 15; ++row)
      {
        float offdiag_sum = 0.0f;
        for (int col = 0; col < 15; ++col)
        {
          if (col != row)
          {
            offdiag_sum += std::fabs(matrix(row, col));
          }
        }
        lower_bound = std::min(lower_bound, matrix(row, row) - offdiag_sum);
      }
      return lower_bound;
    }

    static float SymmetrizeAndComputeGershgorinLowerBound(Eigen::Matrix<float, 15, 15> *matrix)
    {
      float row_offdiag_abs_sum[15] = {};
      for (int row = 0; row < 15; ++row)
      {
        float diag = (*matrix)(row, row);
        if (!std::isfinite(diag))
        {
          diag = MIN_COVARIANCE_EIGENVALUE;
          (*matrix)(row, row) = diag;
        }
        for (int col = row + 1; col < 15; ++col)
        {
          float upper = (*matrix)(row, col);
          float lower = (*matrix)(col, row);
          if (!std::isfinite(upper))
          {
            upper = 0.0f;
          }
          if (!std::isfinite(lower))
          {
            lower = 0.0f;
          }
          const float avg = 0.5f * (upper + lower);
          (*matrix)(row, col) = avg;
          (*matrix)(col, row) = avg;
          const float abs_avg = std::fabs(avg);
          row_offdiag_abs_sum[row] += abs_avg;
          row_offdiag_abs_sum[col] += abs_avg;
        }
      }

      float lower_bound = std::numeric_limits<float>::infinity();
      for (int row = 0; row < 15; ++row)
      {
        lower_bound = std::min(lower_bound,
                               (*matrix)(row, row) - row_offdiag_abs_sum[row]);
      }
      return lower_bound;
    }

    struct EarthModelTerms
    {
      double lat_rad = 0.0;
      double alt_m = 0.0;
      double sin_lat = 0.0;
      double cos_lat = 1.0;
      double tan_lat = 0.0;
      double rn_m = SEMI_MAJOR_AXIS_LENGTH_M;
      double rm_m = SEMI_MAJOR_AXIS_LENGTH_M * (1.0 - ECC2);
      double mean_radius_m = SEMI_MAJOR_AXIS_LENGTH_M;
      double gravity_mps2 = 9.80665;
      Eigen::Vector3d earth_rate_ned_radps = Eigen::Vector3d::Zero();
    };

    struct LlaRateGeometryTerms
    {
      double alt_m = 0.0;
      double cos_lat = 1.0;
      double rn_m = SEMI_MAJOR_AXIS_LENGTH_M;
      double rm_m = SEMI_MAJOR_AXIS_LENGTH_M * (1.0 - ECC2);
    };

    static LlaRateGeometryTerms ComputeLlaRateGeometryTerms(const double lat_rad,
                                                            const double alt_m)
    {
      LlaRateGeometryTerms terms;
      terms.alt_m = alt_m;
      terms.cos_lat = std::cos(lat_rad);
      const double sin_lat = std::sin(lat_rad);
      const double one_minus_e2_sin2 =
          std::max(1.0e-12, 1.0 - ECC2 * sin_lat * sin_lat);
      const double sqrt_term = std::sqrt(one_minus_e2_sin2);
      terms.rn_m = SEMI_MAJOR_AXIS_LENGTH_M / sqrt_term;
      terms.rm_m = SEMI_MAJOR_AXIS_LENGTH_M * (1.0 - ECC2) /
                   (one_minus_e2_sin2 * sqrt_term);
      return terms;
    }

    static EarthModelTerms ComputeEarthModelTerms(const double lat_rad,
                                                  const double alt_m)
    {
      EarthModelTerms terms;
      terms.lat_rad = lat_rad;
      terms.alt_m = alt_m;
      terms.sin_lat = std::sin(lat_rad);
      terms.cos_lat = std::cos(lat_rad);
      const double safe_cos_lat =
          (std::fabs(terms.cos_lat) > 1.0e-9) ? terms.cos_lat
                                              : ((terms.cos_lat >= 0.0) ? 1.0e-9 : -1.0e-9);
      terms.tan_lat = terms.sin_lat / safe_cos_lat;

      const double one_minus_e2_sin2 =
          std::max(1.0e-12, 1.0 - ECC2 * terms.sin_lat * terms.sin_lat);
      const double sqrt_term = std::sqrt(one_minus_e2_sin2);
      terms.rn_m = SEMI_MAJOR_AXIS_LENGTH_M / sqrt_term;
      terms.rm_m = SEMI_MAJOR_AXIS_LENGTH_M * (1.0 - ECC2) /
                   (one_minus_e2_sin2 * sqrt_term);
      terms.mean_radius_m = std::sqrt(terms.rn_m * terms.rm_m);

      const double sin_2lat = 2.0 * terms.sin_lat * terms.cos_lat;
      // Somigliana sin²2φ 系数与 earth_model.h::normalgravity_mps2() 保持一致
      // （5.898e-6 来自 WGS-84/GRS-80 椭球面重力闭式系数；早期 5.9e-6 是同源四舍五入近似）。
      const double g0 = 9.780318 *
                        (1.0 + 5.3024e-3 * terms.sin_lat * terms.sin_lat -
                         5.898e-6 * sin_2lat * sin_2lat);
      /*
       * 高度修正统一使用反平方形式：
       *   g(h) = g0 / (1 + h / R)^2
       * 对 h < 0 的一阶展开应为 g0 * (1 - 2h / R)，不能写成 g0 * (1 + h / R)。
       * EKF 内部缓存项必须与 earth_model.h 的 normalgravity_mps2() 保持一致。
       */
      const double gravity_scale = 1.0 + alt_m / terms.mean_radius_m;
      terms.gravity_mps2 = g0 / (gravity_scale * gravity_scale);

      terms.earth_rate_ned_radps(0) = WE_RADPS * terms.cos_lat;
      terms.earth_rate_ned_radps(1) = 0.0;
      terms.earth_rate_ned_radps(2) = -WE_RADPS * terms.sin_lat;
      return terms;
    }

    static Eigen::Vector3d ComputeLlaRateFromTerms(const Eigen::Vector3d &ned_vel,
                                                   const EarthModelTerms &terms)
    {
      Eigen::Vector3d lla_dot = Eigen::Vector3d::Zero();
      const double safe_cos_lat =
          (std::fabs(terms.cos_lat) > 1.0e-9) ? terms.cos_lat
                                              : ((terms.cos_lat >= 0.0) ? 1.0e-9 : -1.0e-9);
      lla_dot(0) = ned_vel(0) / (terms.rm_m + terms.alt_m);
      lla_dot(1) = ned_vel(1) / ((terms.rn_m + terms.alt_m) * safe_cos_lat);
      lla_dot(2) = -ned_vel(2);
      return lla_dot;
    }

    static Eigen::Vector3d ComputeLlaRateFromTerms(const Eigen::Vector3d &ned_vel,
                                                   const LlaRateGeometryTerms &terms)
    {
      Eigen::Vector3d lla_dot = Eigen::Vector3d::Zero();
      const double safe_cos_lat =
          (std::fabs(terms.cos_lat) > 1.0e-9) ? terms.cos_lat
                                              : ((terms.cos_lat >= 0.0) ? 1.0e-9 : -1.0e-9);
      lla_dot(0) = ned_vel(0) / (terms.rm_m + terms.alt_m);
      lla_dot(1) = ned_vel(1) / ((terms.rn_m + terms.alt_m) * safe_cos_lat);
      lla_dot(2) = -ned_vel(2);
      return lla_dot;
    }

    static Eigen::Vector3d ComputeNavRateFromTerms(const Eigen::Vector3d &ned_vel,
                                                   const EarthModelTerms &terms)
    {
      Eigen::Vector3d nav_rate = Eigen::Vector3d::Zero();
      const double rn_plus_alt = terms.rn_m + terms.alt_m;
      const double rm_plus_alt = terms.rm_m + terms.alt_m;
      nav_rate(0) = ned_vel(1) / rn_plus_alt;
      nav_rate(1) = -ned_vel(0) / rm_plus_alt;
      nav_rate(2) = -ned_vel(1) * terms.tan_lat / rn_plus_alt;
      return nav_rate;
    }

    inline void RefreshStateTransitionStaticBlocks()
    {
      /*
       * Fs 的稀疏结构固定，常量块也固定：
       * - pos<-vel 的单位阵
       * - att<-gyro_bias 的负单位阵
       * - accel / gyro 零偏的 Markov 衰减块
       *
       * 因此这些块初始化一次即可。传播步里只需要覆盖真正随样本变化的动态块，
       * 避免每帧 setZero() 后再把同样的常量块写一遍。
       */
      fs_.setZero();
      fs_.block(0, 3, 3, 3) = Eigen::Matrix<float, 3, 3>::Identity();
      fs_.block(6, 12, 3, 3) = -Eigen::Matrix<float, 3, 3>::Identity();
      fs_.block(9, 9, 3, 3) = accel_markov_bias_;
      fs_.block(12, 12, 3, 3) = gyro_markov_bias_;
    }

    inline void MarkEulerAnglesDirty()
    {
      ypr_dirty_ = true;
    }

    inline void MarkEulerAnglesClean()
    {
      ypr_dirty_ = false;
    }

    inline void UpdateEulerAnglesFromQuat()
    {
      ins_ypr_rad_ = quat2angle(quat_);
      MarkEulerAnglesClean();
    }

    inline void EnsureEulerAnglesUpToDate() const
    {
      if (!ypr_dirty_)
      {
        return;
      }
      auto *self = const_cast<Ekf15State *>(this);
      self->UpdateEulerAnglesFromQuat();
    }

    void PropagateState(const Eigen::Vector3f &accel, const Eigen::Vector3f &gyro,
                        const float dt_s)
    {
      ins_accel_mps2_ = accel - accel_bias_mps2_;
      ins_gyro_radps_ = gyro - gyro_bias_radps_;
      const Eigen::Vector3f delta_theta_rad = ins_gyro_radps_ * dt_s;
      const Eigen::Vector3f delta_v_body_mps = ins_accel_mps2_ * dt_s;
      PropagateStateFromIncrements(ins_accel_mps2_,
                                   ins_gyro_radps_,
                                   delta_theta_rad,
                                   Eigen::Vector3f::Zero(),
                                   delta_v_body_mps,
                                   Eigen::Vector3f::Zero(),
                                   dt_s,
                                   false);
    }

    void PropagateStateFromIncrements(const Eigen::Vector3f &avg_accel_mps2,
                                      const Eigen::Vector3f &avg_gyro_radps,
                                      const Eigen::Vector3f &delta_theta_1_rad,
                                      const Eigen::Vector3f &delta_theta_2_rad,
                                      const Eigen::Vector3f &delta_v_1_mps,
                                      const Eigen::Vector3f &delta_v_2_mps,
                                      const float dt_s,
                                      const bool use_two_sample)
    {
      // 统一的捷联惯导预测：普通 TimeUpdate、双子样 TimeUpdate 和 GNSS 延迟重传播都走同一套动力学。
      ins_accel_mps2_ = avg_accel_mps2;
      ins_gyro_radps_ = avg_gyro_radps;
      Eigen::Vector3f delta_theta_rad = delta_theta_1_rad + delta_theta_2_rad;
      Eigen::Vector3f delta_v_body_mps = delta_v_1_mps + delta_v_2_mps;
      const Eigen::Vector3f coning_delta_theta_rad =
          use_two_sample
              ? (delta_theta_rad +
                 (2.0f / 3.0f) * delta_theta_1_rad.cross(delta_theta_2_rad))
              : (delta_theta_rad +
                 (1.0f / 12.0f) * previous_delta_theta_rad_.cross(delta_theta_rad));
      const Eigen::Vector3f sculling_delta_v_body_mps =
          use_two_sample
              ? (delta_v_body_mps +
                 0.5f * delta_theta_rad.cross(delta_v_body_mps) +
                 (2.0f / 3.0f) *
                     (delta_theta_1_rad.cross(delta_v_2_mps) +
                      delta_v_1_mps.cross(delta_theta_2_rad)))
              : (delta_v_body_mps +
                 0.5f * delta_theta_rad.cross(delta_v_body_mps) +
                 (1.0f / 12.0f) *
                     (previous_delta_theta_rad_.cross(delta_v_body_mps) +
                      previous_delta_v_mps_.cross(delta_theta_rad)));

      const Eigen::Quaternionf quat_prev = quat_;
      // 上一时刻的机体系到NED方向余弦矩阵在类里已缓存，直接复用可少一次 quat2dcm()。
      const Eigen::Matrix3f t_b2ned_prev = t_b2ned;
      const Eigen::Vector3f vel_prev_ned = ins_ned_vel_mps_;
      const Eigen::Vector3d lla_prev_rad_m = ins_lla_rad_m_;
      const Eigen::Quaternionf delta_quat_mid =
          BuildDeltaQuatFromRotationVector(coning_delta_theta_rad * 0.5f);
      const Eigen::Quaternionf quat_mid = (quat_prev * delta_quat_mid).normalized();
      const Eigen::Matrix3f t_b2ned_mid = quat2dcm(quat_mid).transpose();

#if BFS_NAVIGATION_EMBEDDED_PROFILE_ENABLED
      const uint32_t profile_geodesy_start_us = BFS_NAVIGATION_EMBEDDED_PROFILE_NOW_US();
#endif
      /*
       * 上一时刻这里只需要地理曲率半径和 cos(lat) 来求 LLA rate，
       * 不需要重力、地球自转率、平均半径这些更重的项。
       * 因此把“前一时刻的 LLA rate 几何项”拆成轻量路径，避免每帧多做一遍完整地球模型。
       */
      const LlaRateGeometryTerms prev_terms =
          ComputeLlaRateGeometryTerms(lla_prev_rad_m(0), lla_prev_rad_m(2));
      const Eigen::Vector3d lla_rate_prev_rad_mps =
          ComputeLlaRateFromTerms(vel_prev_ned.cast<double>(), prev_terms);

      // 速度用划摇补偿后的机体系速度增量推进；重力使用中点位置的WGS84正常重力。
      const Eigen::Vector3d lla_mid_rad_m =
          lla_prev_rad_m + (0.5 * static_cast<double>(dt_s) * lla_rate_prev_rad_mps);
      const EarthModelTerms mid_terms =
          ComputeEarthModelTerms(lla_mid_rad_m(0), lla_mid_rad_m(2));
      const float gravity_mid_mps2 = static_cast<float>(mid_terms.gravity_mps2);
      gravity_mps2_ = mid_terms.gravity_mps2; // 缓存当地重力供外部消费（控制律/遥测）
      const Eigen::Vector3f gravity_ned_mid =
          (Eigen::Vector3f() << 0.0f, 0.0f, gravity_mid_mps2).finished();
      const Eigen::Vector3f earth_rate_mid_ned =
          mid_terms.earth_rate_ned_radps.cast<float>();
      const Eigen::Vector3f nav_rate_mid_ned =
          ComputeNavRateFromTerms(vel_prev_ned.cast<double>(), mid_terms).cast<float>();
      const Eigen::Vector3f coriolis_accel_mid_ned =
          -((2.0f * earth_rate_mid_ned + nav_rate_mid_ned).cross(vel_prev_ned));
      const Eigen::Vector3f nav_frame_delta_theta_rad =
          -(earth_rate_mid_ned + nav_rate_mid_ned) * dt_s;
#if BFS_NAVIGATION_EMBEDDED_PROFILE_ENABLED
      const uint32_t profile_geodesy_end_us = BFS_NAVIGATION_EMBEDDED_PROFILE_NOW_US();
      BFS_NAVIGATION_EMBEDDED_PROFILE_MARK(
          1U, profile_geodesy_end_us - profile_geodesy_start_us);
      const uint32_t profile_mechanics_start_us = profile_geodesy_end_us;
#endif

      delta_quat_ = BuildDeltaQuatFromRotationVector(coning_delta_theta_rad);
      const Eigen::Quaternionf nav_frame_delta_quat =
          BuildDeltaQuatFromRotationVector(nav_frame_delta_theta_rad);
      quat_ = (nav_frame_delta_quat * quat_ * delta_quat_).normalized();

      if (quat_.w() < 0)
      {
        quat_ = Eigen::Quaternionf(-quat_.w(), -quat_.x(), -quat_.y(),
                                   -quat_.z());
      }

      MarkEulerAnglesDirty();
      t_b2ned = quat2dcm(quat_).transpose();
      const Eigen::Vector3f delta_v_ned =
          t_b2ned_prev * sculling_delta_v_body_mps +
          dt_s * (gravity_ned_mid + coriolis_accel_mid_ned);
      ins_ned_vel_mps_ = vel_prev_ned + delta_v_ned;

      // 位置用速度中点推进；经纬高积分保持double中间量，减少短时纯惯导位置误差。
      const Eigen::Vector3f vel_mid_ned = vel_prev_ned + 0.5f * delta_v_ned;
      const Eigen::Vector3d lla_rate_mid_rad_mps =
          ComputeLlaRateFromTerms(vel_mid_ned.cast<double>(), mid_terms);
      ins_lla_rad_m_ =
          lla_prev_rad_m + (static_cast<double>(dt_s) * lla_rate_mid_rad_mps);
#if BFS_NAVIGATION_EMBEDDED_PROFILE_ENABLED
      const uint32_t profile_mechanics_end_us = BFS_NAVIGATION_EMBEDDED_PROFILE_NOW_US();
      BFS_NAVIGATION_EMBEDDED_PROFILE_MARK(
          2U, profile_mechanics_end_us - profile_mechanics_start_us);
      const uint32_t profile_linearization_start_us = profile_mechanics_end_us;
#endif

#if !(BFS_NAVIGATION_EMBEDDED_FAST_PROPAGATION &&             \
      BFS_NAVIGATION_EMBEDDED_FAST_FIRST_ORDER_COVARIANCE) && \
    !BFS_NAVIGATION_EMBEDDED_STRUCTURED_SIMPSON_QC
      gs_.setZero();
#endif
      // NED的D向下为正，D误差增大等价高度降低，重力下向加速度随之增大。
      const float mean_radius_mid_m = static_cast<float>(mid_terms.mean_radius_m);
      fs_(5, 2) = 2.0f * gravity_mid_mps2 / mean_radius_mid_m;
      fs_.block(3, 6, 3, 3) = -t_b2ned_mid * Skew(ins_accel_mps2_);
      fs_.block(3, 9, 3, 3) = -t_b2ned_mid;
      fs_.block(6, 6, 3, 3) = -Skew(ins_gyro_radps_);

#if BFS_NAVIGATION_EMBEDDED_FULL_F_MATRIX
      /*
       * 完整 F 矩阵扩展：哥氏力/传输速率耦合 + 位置自耦合 + 姿态-速度耦合。
       *
       * 对照 KF-GINS (gi_engine.cpp:184-278) 的 21×21 完整 F 矩阵，
       * 这里补充 15×15 子空间中被简化的非零块。
       *
       * 变量命名遵循 NED [North, East, Down] 约定：
       *   Vn=vel_prev_ned(0), Ve=vel_prev_ned(1), Vd=vel_prev_ned(2)
       *   φ=lat, h=alt, Rm=rm_mid, Rn=rn_mid, ωe=earth_rate
       */
      const float vn = vel_prev_ned(0);
      const float ve = vel_prev_ned(1);
      const float vd = vel_prev_ned(2);
      const float sin_lat = static_cast<float>(mid_terms.sin_lat);
      const float cos_lat = static_cast<float>(mid_terms.cos_lat);
      const float tan_lat = static_cast<float>(mid_terms.tan_lat);
      const float rmh = static_cast<float>(mid_terms.rm_m) + static_cast<float>(mid_terms.alt_m);
      const float rnh = static_cast<float>(mid_terms.rn_m) + static_cast<float>(mid_terms.alt_m);
      const float omega_e = static_cast<float>(WE_RADPS);

      // ── F(V,V)：速度误差的哥氏力/传输速率自耦合（3×3）──
      // 物理含义：NED 速度误差之间的哥氏力和离心力耦合。
      // 对照 KF-GINS gi_engine.cpp:241-250。速度误差始终在导航系定义，与姿态误差坐标系无关。
      // F(V_N, V_N): Vd/(Rm+h)  传输速率对角项
      fs_(3, 3) = vd / rmh;
      // F(V,N←E): -2(ωe·sinφ + Ve·tanφ/(Rn+h))  哥氏力
      fs_(3, 4) = -2.0f * (omega_e * sin_lat + ve * tan_lat / rnh);
      // F(V,N←D): Vn/(Rm+h)  传输速率
      fs_(3, 5) = vn / rmh;
      // F(V,E←N): 2ωe·sinφ + Ve·tanφ/(Rn+h)  哥氏力
      fs_(4, 3) = 2.0f * omega_e * sin_lat + ve * tan_lat / rnh;
      // F(V_E, V_E): (Vd + Vn·tanφ)/(Rn+h)  传输速率对角项
      fs_(4, 4) = (vd + vn * tan_lat) / rnh;
      // F(V,E←D): 2ωe·cosφ + Ve/(Rn+h)  哥氏力
      fs_(4, 5) = 2.0f * omega_e * cos_lat + ve / rnh;
      // F(V,D←N): -2Vn/(Rm+h)  传输速率
      fs_(5, 3) = -2.0f * vn / rmh;
      // F(V,D←E): -2(ωe·cosφ + Ve/(Rn+h))  哥氏力
      fs_(5, 4) = -2.0f * (omega_e * cos_lat + ve / rnh);

      // ── F(P,P)：位置误差的速度/纬度/高度自耦合（3×3）──
      // 物理含义：位置误差率对位置误差本身的偏导。
      // 对照 KF-GINS gi_engine.cpp:219-225。
      // F(N,N): -Vd/(Rm+h)
      fs_(0, 0) = -vd / rmh;
      // F(N,D): Vn/(Rm+h)
      fs_(0, 2) = vn / rmh;
      // F(E,N): Ve·tanφ/(Rn+h)
      fs_(1, 0) = ve * tan_lat / rnh;
      // F(E,E): -(Vd + Vn·tanφ)/(Rn+h)
      fs_(1, 1) = -(vd + vn * tan_lat) / rnh;
      // F(E,D): Ve/(Rn+h)
      fs_(1, 2) = ve / rnh;

      // ── F(Φ,V)：姿态误差的速度/曲率耦合（3×3）──
      //
      // 本 EKF 的姿态误差定义在体系（body frame）：状态反馈为 quat_ = quat_ * Exp(δβ^b)。
      // 体系误差角的动力学包含 transport rate 项：
      //   δβ̇^b ⊃ -C^b_n · δω_en^n = -C^b_n · M · δv^n
      // 其中 M 是 transport rate 雅可比：
      //   M = [0,        1/(Rn+h),   0        ]
      //       [-1/(Rm+h), 0,          0        ]
      //       [0,        -tanφ/(Rn+h), 0       ]
      // 因此 F(β, V) = -C^b_n · M = -(t_b2ned^T) · M。
      //
      // 对照：KF-GINS 使用导航系误差（左乘 q = Exp(δφ^n) * q），其 F(Φ,V) = +M（无旋转）。
      // 当姿态水平（C^b_n = I）时，两者差一个负号：F(β,V) = -M vs F(φ,V) = +M。
      //
      // t_b2ned = C^n_b（body→nav），t_b2ned^T = C^b_n（nav→body）。
      Eigen::Matrix3f transport_jacobian = Eigen::Matrix3f::Zero();
      transport_jacobian(0, 1) = 1.0f / rnh;
      transport_jacobian(1, 0) = -1.0f / rmh;
      transport_jacobian(2, 1) = -tan_lat / rnh;
      fs_.block<3, 3>(6, 3) = -t_b2ned_mid.transpose() * transport_jacobian;
#endif // BFS_NAVIGATION_EMBEDDED_FULL_F_MATRIX

      const Eigen::Matrix<float, 15, 15> fs_dt = fs_ * dt_s;
#if BFS_NAVIGATION_EMBEDDED_FAST_FIRST_ORDER_PHI
      /*
       * 纯惯导桌面静止 bring-up 的进一步压性能选项：
       * - 直接使用一阶状态转移 Phi ≈ I + F dt
       * - 省掉一次 15x15 矩阵乘法，主要目标是压低 prop_lin_mean
       *
       * 适用边界：
       * - dt 很小（这里 5 ms）且主要是静止/低动态硬件验证
       * - 不应直接推广到高动态或飞控闭环场景；若后续要用于真实飞行，
       *   需要重新验证协方差一致性、状态漂移和量测门控行为。
       */
#if !(BFS_NAVIGATION_EMBEDDED_FAST_PROPAGATION &&             \
      BFS_NAVIGATION_EMBEDDED_FAST_FIRST_ORDER_COVARIANCE) || \
    defined(EKF_HOST_REGRESSION)
      // 当前嵌入式快路径若直接走 P += Fdt*P + P*(Fdt)^T + Q，则本帧 phi_ 不再参与后续计算；
      // 这里只在“仍会用到 phi_”或主机回归需要读出 phi_”时才构造它。
      phi_ = Eigen::Matrix<float, 15, 15>::Identity() + fs_dt;
#endif
#else
      phi_ = Eigen::Matrix<float, 15, 15>::Identity() +
             fs_dt + 0.5f * fs_dt * fs_dt;
#endif

      /*
       * 当前 pure-INS ESP32 性能环境同时打开了：
       * - FAST_PROPAGATION
       * - FAST_FIRST_ORDER_COVARIANCE
       *
       * 在这组宏组合下，协方差传播不会再走 Gs * Rw * Gs^T 或 double Qd 离散化路径，
       * 因此本帧构造 gs_ 只是额外的 15x12 清零与 4 个 block 写入，不参与任何后续计算。
       * 这里直接按编译期条件跳过，避免在 200 Hz 主链路里做无效矩阵写入。
       *
       * 若未来关闭 FAST_FIRST_ORDER_COVARIANCE 或 FAST_PROPAGATION，编译期会自动回到
       * 原有 gs_ 构造路径，不改变其它场景的滤波数学。
       */
#if !(BFS_NAVIGATION_EMBEDDED_FAST_PROPAGATION &&             \
      BFS_NAVIGATION_EMBEDDED_FAST_FIRST_ORDER_COVARIANCE) && \
    !BFS_NAVIGATION_EMBEDDED_STRUCTURED_SIMPSON_QC
      gs_.block(3, 0, 3, 3) = -t_b2ned_mid;
      gs_.block(6, 3, 3, 3) = -Eigen::Matrix<float, 3, 3>::Identity();
      gs_.block(9, 6, 3, 3) = Eigen::Matrix<float, 3, 3>::Identity();
      gs_.block(12, 9, 3, 3) = Eigen::Matrix<float, 3, 3>::Identity();
#endif
#if BFS_NAVIGATION_EMBEDDED_PROFILE_ENABLED
      const uint32_t profile_linearization_end_us =
          BFS_NAVIGATION_EMBEDDED_PROFILE_NOW_US();
      BFS_NAVIGATION_EMBEDDED_PROFILE_MARK(
          3U, profile_linearization_end_us - profile_linearization_start_us);
      const uint32_t profile_covariance_start_us = profile_linearization_end_us;
#endif

#if BFS_NAVIGATION_EMBEDDED_FAST_PROPAGATION
      /*
       * ESP32 实机快速路径：
       * 原始路径在每个 TimeUpdate 中使用多组 double 15x15 / 15x12 矩阵乘法，
       * 在 ESP32-S3 上实测会带来很高的栈占用和约 23~24 ms 的单步耗时。
       *
       * 本宏默认关闭，只有 bring-up / 桌面静止测试环境显式开启。开启后：
       * - 协方差传播改用 float 中间量，避免 Xtensa 上软件 double 乘法成为主瓶颈。
       * - 过程噪声使用一阶近似 Qd ≈ G Rw G^T dt，适合当前 10~25 ms 级别、
       *   无 GNSS、静止/低动态的硬件连通与惯导链路验证。
       * - 飞控闭环或高动态任务若要使用该路径，必须重新做数值一致性和动态验证。
       */
      /*
       * 当前 15 状态模型下，Gs/Rw 的结构是固定稀疏块：
       * - accel 白噪声只驱动速度误差块
       * - gyro 白噪声只驱动姿态误差块
       * - accel/gyro bias 随机游走只驱动各自零偏块
       * 并且 Rw 各 3x3 子块都是标量 * I，t_b2ned_mid 又是正交矩阵，因此
       *   t_b2ned_mid * (sigma^2 I) * t_b2ned_mid^T = sigma^2 I
       * 可以直接把 Q 写成 4 个对角 3x3 子块，避免每帧做一整套
       * 15x12 * 12x12 * 12x15 的稠密矩阵乘法。
       *
       * 这条快路径与当前 Gs/Rw 构造完全等价；若未来过程噪声模型引入非对角相关项，
       * 必须同步回退或重写这里的结构化展开。
       */
      const float accel_rw_var = accel_std_mps2_ * accel_std_mps2_ * dt_s;
      const float gyro_rw_var = gyro_std_radps_ * gyro_std_radps_ * dt_s;
      const float accel_bias_rw_var =
          (2.0f * accel_markov_bias_std_mps2_ * accel_markov_bias_std_mps2_ /
           accel_tau_s_) *
          dt_s;
      const float gyro_bias_rw_var =
          (2.0f * gyro_markov_bias_std_radps_ * gyro_markov_bias_std_radps_ /
           gyro_tau_s_) *
          dt_s;

#if BFS_NAVIGATION_EMBEDDED_FAST_FIRST_ORDER_COVARIANCE
      /*
       * 在一阶 Phi ≈ I + Fdt 的同一前提下，协方差传播进一步近似为：
       *   P(k+1) ≈ P + Fdt*P + P*(Fdt)^T + Q
       *
       * 由于 P 设计上始终保持对称，(Fdt*P)^T 即可复用为 P*(Fdt)^T，
       * 因此这里只需要一次 15x15 乘法，而不再做完整的
       *   Phi * P * Phi^T
       * 两次稠密矩阵乘法。
       *
       * 适用边界与 fast first-order Phi 相同：仅用于 5 ms 级别、静止/低动态的
       * 纯惯导 bring-up / 桌面压性能环境。若用于高动态或正式飞行，需要重新验证
       * 协方差一致性和量测门控行为。
       */
      const Eigen::Matrix<float, 15, 15> fs_p =
          Icm45686FirstOrderCovarianceDeltaFromBlocks(
              p_,
              fs_.block<3, 3>(3, 6),
              fs_.block<3, 3>(3, 9),
              fs_.block<3, 3>(6, 6),
              fs_.block<3, 3>(9, 9),
              fs_.block<3, 3>(12, 12),
              fs_(5, 2),
              dt_s
#if BFS_NAVIGATION_EMBEDDED_FULL_F_MATRIX
              ,
              fs_.block<3, 3>(3, 3),   // F(V,V): 哥氏力/传输速率
              fs_.block<3, 3>(0, 0),   // F(P,P): 位置自耦合
              fs_.block<3, 3>(6, 3)    // F(Φ,V): 姿态-速度曲率耦合
#endif
              );
      p_ += fs_p;
      p_ += fs_p.transpose().eval();
      /*
       * 对当前快路径，Q 本身就是 4 组互不耦合的 3x3 对角块。
       * 直接把这 12 个对角增量写回 P，和先构造完整 Q 再执行 P += Q 完全等价，
       * 但可以省掉一次 15x15 清零、4 次 block/diagonal 填充以及一次整矩阵加法。
       */
      p_(3, 3) += accel_rw_var;
      p_(4, 4) += accel_rw_var;
      p_(5, 5) += accel_rw_var;
      p_(6, 6) += gyro_rw_var;
      p_(7, 7) += gyro_rw_var;
      p_(8, 8) += gyro_rw_var;
      p_(9, 9) += accel_bias_rw_var;
      p_(10, 10) += accel_bias_rw_var;
      p_(11, 11) += accel_bias_rw_var;
      p_(12, 12) += gyro_bias_rw_var;
      p_(13, 13) += gyro_bias_rw_var;
      p_(14, 14) += gyro_bias_rw_var;
#else
      q_.setZero();
      q_.block<3, 3>(3, 3).diagonal().setConstant(accel_rw_var);
      q_.block<3, 3>(6, 6).diagonal().setConstant(gyro_rw_var);
      q_.block<3, 3>(9, 9).diagonal().setConstant(accel_bias_rw_var);
      q_.block<3, 3>(12, 12).diagonal().setConstant(gyro_bias_rw_var);
      const Eigen::Matrix<float, 15, 15> phi_p = phi_ * p_;
      p_.noalias() = phi_p * phi_.transpose();
      p_ += q_;
#endif
#else
      // 离散过程噪声必须自身保持正半定；使用二阶状态转移的Simpson积分近似
      // Qd = ∫ Phi(t) G Rw G^T Phi(t)^T dt，比零阶保持更接近高频子步传播。
      const Eigen::Matrix<double, 15, 15> identity_d =
          Eigen::Matrix<double, 15, 15>::Identity();
      const Eigen::Matrix<double, 15, 15> fs_d = fs_.cast<double>();
#if BFS_NAVIGATION_EMBEDDED_STRUCTURED_SIMPSON_QC
      /*
       * 高精度 Simpson 路径的等价性能优化：
       * 当前连续过程噪声结构固定为 4 个互不相关的三轴白噪声块：
       * - 加速度白噪声经 -Cbn 驱动速度误差
       * - 陀螺白噪声经 -I 驱动态度误差
       * - 加速度零偏随机游走经 I 驱动加速度零偏误差
       * - 陀螺零偏随机游走经 I 驱动陀螺零偏误差
       *
       * 因为 Cbn 是正交矩阵，Cbn * (sigma^2 I) * Cbn^T 仍等于 sigma^2 I。
       * 所以 Gs * Rw * Gs^T 与下面 4 个 3x3 对角块完全等价。
       * 这里仍继续使用 double Simpson Qd 和 double P 传播，只省掉固定稀疏矩阵的稠密乘法。
       */
      const Eigen::Matrix<double, 15, 15> qc_d =
          Icm45686StructuredContinuousQc(
              static_cast<double>(accel_std_mps2_),
              static_cast<double>(gyro_std_radps_),
              static_cast<double>(accel_markov_bias_std_mps2_),
              static_cast<double>(accel_tau_s_),
              static_cast<double>(gyro_markov_bias_std_radps_),
              static_cast<double>(gyro_tau_s_));
#else
      const Eigen::Matrix<double, 15, 12> gs_d = gs_.cast<double>();
      const Eigen::Matrix<double, 12, 12> rw_d = rw_.cast<double>();
      const Eigen::Matrix<double, 15, 15> qc_d = gs_d * rw_d * gs_d.transpose();
#endif
      const auto phi_at_time = [&](const double tau_s)
      {
        const Eigen::Matrix<double, 15, 15> f_tau = fs_d * tau_s;
        return identity_d + f_tau + 0.5 * f_tau * f_tau;
      };
      const double dt_d = static_cast<double>(dt_s);
      const Eigen::Matrix<double, 15, 15> phi_half_d = phi_at_time(0.5 * dt_d);
      const Eigen::Matrix<double, 15, 15> phi_full_d = phi_.cast<double>();
      const Eigen::Matrix<double, 15, 15> q_d =
          (dt_d / 6.0) *
          (qc_d +
           4.0 * (phi_half_d * qc_d * phi_half_d.transpose()) +
           (phi_full_d * qc_d * phi_full_d.transpose()));
      q_ = q_d.cast<float>();
      q_ = 0.5f * (q_ + q_.transpose().eval());

      // 协方差传播使用double中间量，降低15x15矩阵乘法在高动态大协方差下的舍入误差。
      const Eigen::Matrix<double, 15, 15> phi_d = phi_.cast<double>();
      const Eigen::Matrix<double, 15, 15> p_d = p_.cast<double>();
      p_ = (phi_d * p_d * phi_d.transpose() + q_d).cast<float>();
#endif
#if BFS_NAVIGATION_EMBEDDED_PROFILE_ENABLED
      const uint32_t profile_covariance_end_us =
          BFS_NAVIGATION_EMBEDDED_PROFILE_NOW_US();
      BFS_NAVIGATION_EMBEDDED_PROFILE_MARK(
          4U, profile_covariance_end_us - profile_covariance_start_us);
      const uint32_t profile_stabilize_start_us = profile_covariance_end_us;
#endif
      StabilizeCovarianceAfterPropagation();
#if BFS_NAVIGATION_EMBEDDED_PROFILE_ENABLED
      const uint32_t profile_stabilize_end_us =
          BFS_NAVIGATION_EMBEDDED_PROFILE_NOW_US();
      BFS_NAVIGATION_EMBEDDED_PROFILE_MARK(
          5U, profile_stabilize_end_us - profile_stabilize_start_us);
#endif
      previous_delta_theta_rad_ = use_two_sample ? delta_theta_2_rad : delta_theta_rad;
      previous_delta_v_mps_ = use_two_sample ? delta_v_2_mps : delta_v_body_mps;
    }

    void InjectErrorState()
    {
      const double lat_before_rad = ins_lla_rad_m_(0);
      const double alt_before_m = ins_lla_rad_m_(2);
      const double denom = fabs(1.0 - (ECC2 * sin(lat_before_rad) *
                                       sin(lat_before_rad)));
      double sqrt_denom = std::sqrt(denom);
      double Rns = SEMI_MAJOR_AXIS_LENGTH_M * (1 - ECC2) /
                   (denom * sqrt_denom);
      double Rew = SEMI_MAJOR_AXIS_LENGTH_M / sqrt_denom;

      // 位置误差注入的三个分量统一使用更新前的纬度/高度计算比例尺，避免分量顺序带来二阶耦合误差。
      const double delta_lat_rad = x_(0) / (Rns + alt_before_m);
      const double delta_lon_rad = x_(1) / (Rew + alt_before_m) / cos(lat_before_rad);
      ins_lla_rad_m_(0) += delta_lat_rad;
      ins_lla_rad_m_(1) += delta_lon_rad;
      ins_lla_rad_m_(2) -= x_(2);

      ins_ned_vel_mps_ += x_.segment(3, 3);

      // 误差状态姿态反馈同样使用指数映射，避免AHRS/重力量测一次性修正较大倾角时低估修正量。
      delta_quat_ = BuildDeltaQuatFromRotationVector(x_.segment(6, 3));
      quat_ = (quat_ * delta_quat_).normalized();
      if (quat_.w() < 0)
      {
        quat_ = Eigen::Quaternionf(-quat_.w(), -quat_.x(), -quat_.y(),
                                   -quat_.z());
      }
      MarkEulerAnglesDirty();
      t_b2ned = quat2dcm(quat_).transpose();

      accel_bias_mps2_ += x_.segment(9, 3);
      gyro_bias_radps_ += x_.segment(12, 3);

      ins_accel_mps2_ -= x_.segment(9, 3);
      ins_gyro_radps_ -= x_.segment(12, 3);

      x_.setZero();
    }

    bool ApplyVelocityUpdate(const Eigen::Vector3f &measured_vel_ned,
                             const float vel_ne_std_mps,
                             const float vel_d_std_mps)
    {
      return ApplyVelocityUpdateDetailed(measured_vel_ned,
                                         vel_ne_std_mps,
                                         vel_d_std_mps)
          .fused;
    }

    MeasurementUpdateResult ApplyVelocityUpdateDetailed(
        const Eigen::Vector3f &measured_vel_ned,
        const float vel_ne_std_mps,
        const float vel_d_std_mps)
    {
      // 纯速度量测的实际卡尔曼更新；公开接口和延迟重放共用，避免两套逻辑漂移。
      MeasurementUpdateResult result;
      result.input_valid = true;
      result.attempted = true;
      Eigen::Matrix3f r_vel = Eigen::Matrix3f::Zero();
      const float safe_vel_ne_std_mps = ClampMeasurementNoiseStd(vel_ne_std_mps);
      const float safe_vel_d_std_mps = ClampMeasurementNoiseStd(vel_d_std_mps);
      r_vel(0, 0) = safe_vel_ne_std_mps * safe_vel_ne_std_mps;
      r_vel(1, 1) = safe_vel_ne_std_mps * safe_vel_ne_std_mps;
      r_vel(2, 2) = safe_vel_d_std_mps * safe_vel_d_std_mps;

      Eigen::Vector3f y_vel = measured_vel_ned - ins_ned_vel_mps_;
      result.innovation_norm = y_vel.norm();
      const bool zero_velocity_observation = measured_vel_ned.norm() < ZERO_VELOCITY_MEAS_NORM_MPS;
      if (zero_velocity_observation && y_vel.norm() > ZERO_VELOCITY_RESIDUAL_REJECT_MPS)
      {
        // ZUPT只应在上层静止确认可靠时使用；若误触发时当前速度已达数m/s，底层直接拒绝。
        return result;
      }
      if (y_vel.norm() > VELOCITY_RESIDUAL_REJECT_MPS)
      {
        // 该接口主要用于ZUPT/辅助速度约束，数米每秒级残差应交给GNSS主更新而不是强行融合。
        return result;
      }
#if BFS_NAVIGATION_EMBEDDED_FAST_STATIC_AID_UPDATE
      /*
       * ESP32 实机瘦身路径：
       * 速度量测 H 只有速度误差块为 I，因此 S = Pvv + R，K = P[:,v] * S^-1。
       * 这里避免构造 3x15 H 和 15x15 Joseph 中间矩阵，协方差用等价的
       * P = P - K*S*K^T 更新，然后仍进入 StabilizeCovariance() 做对称化和PSD检查。
       *
       * 该宏默认关闭。它适合当前 ICM45686 静止 ZUPT 性能测试；若用于飞控闭环或
       * 高动态速度量测，需要重新验证数值稳定性、NIS 拒绝率和故障恢复行为。
       */
      Eigen::Matrix3f s_vel = p_.block<3, 3>(3, 3) + r_vel;
      Eigen::Matrix3f s_vel_inv;
      if (!InvertSymmetricPositiveDefiniteMatrix(s_vel, &s_vel_inv))
      {
        // S矩阵病态时拒绝该速度量测，避免坏增益破坏协方差。
        return result;
      }
      const Eigen::Matrix<float, 1, 1> nis_vel = y_vel.transpose() * s_vel_inv * y_vel;
      result.test_ratio = std::isfinite(nis_vel(0, 0))
                              ? nis_vel(0, 0) / VELOCITY_NIS_REJECT_THRESHOLD
                              : 0.0f;
      if (!std::isfinite(nis_vel(0, 0)) || nis_vel(0, 0) > VELOCITY_NIS_REJECT_THRESHOLD)
      {
        // 速度/ZUPT量测若与当前状态严重不一致，拒绝单帧离群值，避免速度被拉飞。
        return result;
      }
      Eigen::Matrix<float, 15, 3> k_vel = p_.block<15, 3>(0, 3) * s_vel_inv;
      p_ -= (k_vel * s_vel * k_vel.transpose()).eval();
      StabilizeCovarianceAfterStaticAidUpdate();

      x_ = k_vel * y_vel;
      InjectErrorState();
      result.fused = true;
      return result;
#else
      Eigen::Matrix<float, 3, 15> h_vel = Eigen::Matrix<float, 3, 15>::Zero();
      h_vel.block(0, 3, 3, 3) = Eigen::Matrix3f::Identity();
      Eigen::Matrix3f s_vel = h_vel * p_ * h_vel.transpose() + r_vel;
      Eigen::Matrix3f s_vel_inv;
      if (!InvertSymmetricPositiveDefiniteMatrix(s_vel, &s_vel_inv))
      {
        // S矩阵病态时拒绝该速度量测，避免坏增益破坏协方差。
        return result;
      }
      const Eigen::Matrix<float, 1, 1> nis_vel = y_vel.transpose() * s_vel_inv * y_vel;
      result.test_ratio = std::isfinite(nis_vel(0, 0))
                              ? nis_vel(0, 0) / VELOCITY_NIS_REJECT_THRESHOLD
                              : 0.0f;
      if (!std::isfinite(nis_vel(0, 0)) || nis_vel(0, 0) > VELOCITY_NIS_REJECT_THRESHOLD)
      {
        // 速度/ZUPT量测若与当前状态严重不一致，拒绝单帧离群值，避免速度被拉飞。
        return result;
      }
      Eigen::Matrix<float, 15, 3> k_vel = p_ * h_vel.transpose() * s_vel_inv;

      Eigen::Matrix<float, 15, 15> identity = Eigen::Matrix<float, 15, 15>::Identity();
      p_ = (identity - k_vel * h_vel) * p_ * (identity - k_vel * h_vel).transpose() +
           k_vel * r_vel * k_vel.transpose();
      StabilizeCovarianceAfterStaticAidUpdate();

      x_ = k_vel * y_vel;
      InjectErrorState();
      result.fused = true;
      return result;
#endif
    }

    bool ApplyGravityUpdate(const Eigen::Vector3f &measured_accel_mps2,
                            const float accel_dir_noise_rad)
    {
      return ApplyGravityUpdateDetailed(measured_accel_mps2,
                                        accel_dir_noise_rad)
          .fused;
    }

    MeasurementUpdateResult ApplyGravityUpdateDetailed(
        const Eigen::Vector3f &measured_accel_mps2,
        const float accel_dir_noise_rad)
    {
      // 重力方向量测只在加速度有效时执行；返回值用于决定是否记录到延迟重放事件。
      MeasurementUpdateResult result;
      result.input_valid = true;
      result.attempted = true;
      if (measured_accel_mps2.norm() < 1.0f)
      {
        return result;
      }

      /*
       * Gravity 辅助只应约束“当前姿态下的重力方向”，不能把加速度计零偏也当成
       * 姿态倾角来修。运行期 EKF 已经显式建模 accel_bias_mps2_，因此这里先用
       * 当前零偏估计扣除原始加速度，再取方向：
       * - ZUPT/传播负责继续估计 accel_bias；
       * - Gravity 使用扣偏后的比力方向修 roll/pitch；
       * - 若继续用 raw accel 方向，水平加速度零偏会被误解释成倾角，并通过姿态-零偏
       *   交叉协方差进一步污染 gyro_bias，长静止时表现为 yaw 慢性漂移。
       */
      const Eigen::Vector3f compensated_accel_mps2 =
          measured_accel_mps2 - accel_bias_mps2_;
      if (compensated_accel_mps2.norm() < 1.0f)
      {
        return result;
      }
      const Eigen::Vector3f measured_dir = compensated_accel_mps2.normalized();
      const Eigen::Vector3f gravity_specific_force_ned =
          (Eigen::Vector3f() << 0.0f, 0.0f, -1.0f).finished();
      const Eigen::Matrix3f t_ned2b = quat2dcm(quat_);
      const Eigen::Vector3f predicted_dir = t_ned2b * gravity_specific_force_ned;

      /*
       * 姿态误差 δβ 定义在体系（右乘 q_true = q_nom ⊗ Exp(δβ^b)），重力方向预测
       *   predicted_dir(δβ) = (I - [δβ×]) · predicted_dir_nom
       *                     = predicted_dir_nom + [predicted_dir_nom×] · δβ
       * 因此 H 对体系姿态误差的雅可比为
       *   H_att = +Skew(predicted_dir) = +Skew(t_ned2b · g_sf_ned)
       * 利用恒等式 Skew(R·v) = R·Skew(v)·R^T，等价于
       *   H_att = t_ned2b · Skew(g_sf_ned) · t_b2ned
       * 早期实现漏乘右端 t_b2ned（= t_ned2b^T），相当于把体系误差当成导航系误差
       * 来线性化：水平时 t_b2ned≈I 不暴露，有倾角时 H 误差达 O(sin tilt)。
       */
      const Eigen::Matrix3f h_grav_att = Skew(predicted_dir);

      const float safe_accel_dir_noise_rad = ClampMeasurementNoiseStd(accel_dir_noise_rad);
      Eigen::Matrix3f r_grav = safe_accel_dir_noise_rad * safe_accel_dir_noise_rad *
                               Eigen::Matrix3f::Identity();

      Eigen::Vector3f y_grav = measured_dir - predicted_dir;
      result.innovation_norm = y_grav.norm();
#if BFS_NAVIGATION_EMBEDDED_FAST_STATIC_AID_UPDATE
      /*
       * ESP32 实机瘦身路径：
       * 重力方向量测只观测姿态误差块，H 只有 [6:8] 三列非零。直接用姿态协方差块
       * 计算 S，并用 P[:,att] * H_att^T * S^-1 求 K，可少掉大部分动态尺寸矩阵乘法。
       * 协方差同样采用 P = P - K*S*K^T，并在随后做统一稳定化。
       */
      Eigen::Matrix3f s_grav =
          h_grav_att * p_.block<3, 3>(6, 6) * h_grav_att.transpose() + r_grav;
      Eigen::Matrix3f s_grav_inv;
      if (!InvertSymmetricPositiveDefiniteMatrix(s_grav, &s_grav_inv))
      {
        // S矩阵病态时拒绝该重力量测，保持当前惯导姿态。
        return result;
      }
      const Eigen::Matrix<float, 1, 1> nis_grav = y_grav.transpose() * s_grav_inv * y_grav;
      result.test_ratio = std::isfinite(nis_grav(0, 0))
                              ? nis_grav(0, 0) / GRAVITY_NIS_REJECT_THRESHOLD
                              : 0.0f;
      if (!std::isfinite(nis_grav(0, 0)) || nis_grav(0, 0) > GRAVITY_NIS_REJECT_THRESHOLD)
      {
        // 重力方向量测离群时不修姿态，防止机动加速度或坏AHRS把姿态拉偏。
        return result;
      }
      Eigen::Matrix<float, 15, 3> k_grav =
          p_.block<15, 3>(0, 6) * h_grav_att.transpose() * s_grav_inv;
      p_ -= (k_grav * s_grav * k_grav.transpose()).eval();
      StabilizeCovarianceAfterStaticAidUpdate();

      x_ = k_grav * y_grav;
      InjectErrorState();
      result.fused = true;
      return result;
#else
      Eigen::Matrix<float, 3, 15> h_grav = Eigen::Matrix<float, 3, 15>::Zero();
      h_grav.block(0, 6, 3, 3) = h_grav_att;
      Eigen::Matrix3f s_grav = h_grav * p_ * h_grav.transpose() + r_grav;
      Eigen::Matrix3f s_grav_inv;
      if (!InvertSymmetricPositiveDefiniteMatrix(s_grav, &s_grav_inv))
      {
        // S矩阵病态时拒绝该重力量测，保持当前惯导姿态。
        return result;
      }
      const Eigen::Matrix<float, 1, 1> nis_grav = y_grav.transpose() * s_grav_inv * y_grav;
      result.test_ratio = std::isfinite(nis_grav(0, 0))
                              ? nis_grav(0, 0) / GRAVITY_NIS_REJECT_THRESHOLD
                              : 0.0f;
      if (!std::isfinite(nis_grav(0, 0)) || nis_grav(0, 0) > GRAVITY_NIS_REJECT_THRESHOLD)
      {
        // 重力方向量测离群时不修姿态，防止机动加速度或坏AHRS把姿态拉偏。
        return result;
      }
      Eigen::Matrix<float, 15, 3> k_grav = p_ * h_grav.transpose() * s_grav_inv;

      Eigen::Matrix<float, 15, 15> identity = Eigen::Matrix<float, 15, 15>::Identity();
      p_ = (identity - k_grav * h_grav) * p_ * (identity - k_grav * h_grav).transpose() +
           k_grav * r_grav * k_grav.transpose();
      StabilizeCovarianceAfterStaticAidUpdate();

      x_ = k_grav * y_grav;
      InjectErrorState();
      result.fused = true;
      return result;
#endif
    }

    bool ApplyStaticGyroUpdate(const Eigen::Vector3f &measured_gyro_radps,
                               const float gyro_noise_radps)
    {
      return ApplyStaticGyroUpdateDetailed(measured_gyro_radps,
                                           gyro_noise_radps)
          .fused;
    }

    bool ApplyStaticGyroUpdate(const Eigen::Vector3f &measured_gyro_radps,
                               const Eigen::Vector3f &gyro_noise_radps)
    {
      return ApplyStaticGyroUpdateDetailed(measured_gyro_radps,
                                           gyro_noise_radps)
          .fused;
    }

    MeasurementUpdateResult ApplyStaticGyroUpdateDetailed(
        const Eigen::Vector3f &measured_gyro_radps,
        const float gyro_noise_radps)
    {
      return ApplyStaticGyroUpdateDetailed(
          measured_gyro_radps,
          (Eigen::Vector3f() << gyro_noise_radps, gyro_noise_radps, gyro_noise_radps)
              .finished());
    }

    MeasurementUpdateResult ApplyStaticGyroUpdateDetailed(
        const Eigen::Vector3f &measured_gyro_radps,
        const Eigen::Vector3f &gyro_noise_radps)
    {
      MeasurementUpdateResult result;
      result.input_valid = true;
      result.attempted = true;

      /*
       * 静止时原始陀螺读数由两部分组成：
       *   measured_gyro_body = expected_nav_frame_rate_body + gyro_bias_body
       *
       * 这里的 expected_nav_frame_rate_body 与 Initialize() 中的定义一致，包含地球自转
       * 和当前 NED 导航系转动。桌面静止时 navrate 接近 0，但仍保留完整表达式，避免
       * 以后把该接口用于带 GNSS 速度的静止/低速场景时符号漂移。
       */
      const Eigen::Vector3f nav_frame_rate_ned =
          earthrate(ins_lla_rad_m_(0), bfs::AngPosUnit::RAD).cast<float>() +
          navrate(ins_ned_vel_mps_.cast<double>(),
                  ins_lla_rad_m_,
                  bfs::AngPosUnit::RAD)
              .cast<float>();
      const Eigen::Vector3f expected_physical_gyro_body =
          t_b2ned.transpose() * nav_frame_rate_ned;
      const Eigen::Vector3f predicted_gyro_body =
          expected_physical_gyro_body + gyro_bias_radps_;
      const Eigen::Vector3f gyro_residual_body =
          measured_gyro_radps - predicted_gyro_body;
      result.innovation_norm = gyro_residual_body.norm();
      if (gyro_residual_body.norm() > STATIC_GYRO_RESIDUAL_REJECT_RADPS)
      {
        // 静止角速度残差过大通常表示上层误判静止或传感器异常，不能强行改零偏。
        return result;
      }

      const Eigen::Vector3f safe_gyro_noise_radps =
          ClampStaticGyroNoiseStd(gyro_noise_radps);
      const Eigen::Matrix3f r_gyro =
          safe_gyro_noise_radps.array().square().matrix().asDiagonal();
      Eigen::Matrix3f s_gyro = p_.block<3, 3>(12, 12) + r_gyro;
      Eigen::Matrix3f s_gyro_inv;
      if (!InvertSymmetricPositiveDefiniteMatrix(s_gyro, &s_gyro_inv))
      {
        // 零偏子块病态时拒绝本次量测，保持当前协方差。
        return result;
      }
      const Eigen::Matrix<float, 1, 1> nis_gyro =
          gyro_residual_body.transpose() * s_gyro_inv * gyro_residual_body;
      result.test_ratio = std::isfinite(nis_gyro(0, 0))
                              ? nis_gyro(0, 0) / STATIC_GYRO_NIS_REJECT_THRESHOLD
                              : 0.0f;
      if (!std::isfinite(nis_gyro(0, 0)) ||
          nis_gyro(0, 0) > STATIC_GYRO_NIS_REJECT_THRESHOLD)
      {
        return result;
      }

      /*
       * 静止角速度是“零偏量测”，不是绝对姿态量测。前一版使用完整
       * K=P*H^T*S^-1，会经姿态-零偏交叉协方差直接改姿态；硬件 5 分钟样本证明
       * 这会和 Gravity 的 roll/pitch 闭环互相拉扯，短时 yaw 好看但长时回退。
       *
       * 这里故意只更新 gyro_bias 三维子块：
       * - p_bg 用标准 Kalman/Joseph 形式收缩；
       * - 与其它状态的交叉协方差按同一个右乘因子缩放，保持矩阵一致；
       * - 不把残差直接写入姿态，姿态只会在后续 IMU 传播中因零偏更准而停止继续漂移。
       */
      const Eigen::Matrix3f p_bg = p_.block<3, 3>(12, 12);
      const Eigen::Matrix3f k_bg = p_bg * s_gyro_inv;
      gyro_bias_radps_ += k_bg * gyro_residual_body;
      const Eigen::Matrix3f identity3 = Eigen::Matrix3f::Identity();
      const Eigen::Matrix3f p_bg_updated =
          (identity3 - k_bg) * p_bg * (identity3 - k_bg).transpose() +
          k_bg * r_gyro * k_bg.transpose();
      p_.block<12, 3>(0, 12) *= (identity3 - k_bg).transpose();
      p_.block<3, 12>(12, 0) = p_.block<12, 3>(0, 12).transpose();
      p_.block<3, 3>(12, 12) = p_bg_updated;
      StabilizeCovarianceAfterStaticAidUpdate();

      ins_gyro_radps_ = measured_gyro_radps - gyro_bias_radps_;
      result.fused = true;
      return result;
    }

    bool ApplyAttitudeUpdate(const Eigen::Vector3f &measured_ypr_rad,
                             const float roll_pitch_noise_rad,
                             const float yaw_noise_rad)
    {
      return ApplyAttitudeUpdateDetailed(measured_ypr_rad,
                                         roll_pitch_noise_rad,
                                         yaw_noise_rad)
          .fused;
    }

    MeasurementUpdateResult ApplyAttitudeUpdateDetailed(
        const Eigen::Vector3f &measured_ypr_rad,
        const float roll_pitch_noise_rad,
        const float yaw_noise_rad)
    {
      // 姿态量测更新的实际实现；公开调用会记录事件，GNSS延迟回放只重放事件。
      MeasurementUpdateResult result;
      result.input_valid = true;
      result.attempted = true;
      Eigen::Matrix<float, 3, 15> h_att = Eigen::Matrix<float, 3, 15>::Zero();
      h_att.block(0, 6, 3, 3) = Eigen::Matrix3f::Identity();

      Eigen::Matrix3f r_att = Eigen::Matrix3f::Zero();
      const float safe_roll_pitch_noise_rad = ClampMeasurementNoiseStd(roll_pitch_noise_rad);
      const float safe_yaw_noise_rad = ClampMeasurementNoiseStd(yaw_noise_rad);
      r_att(0, 0) = safe_roll_pitch_noise_rad * safe_roll_pitch_noise_rad;
      r_att(1, 1) = safe_roll_pitch_noise_rad * safe_roll_pitch_noise_rad;
      r_att(2, 2) = safe_yaw_noise_rad * safe_yaw_noise_rad;

      const Eigen::Quaternionf measured_quat = eul2quat(measured_ypr_rad);
      Eigen::Quaternionf error_quat = quat_.conjugate() * measured_quat;
      if (error_quat.w() < 0.0f)
      {
        error_quat = Eigen::Quaternionf(-error_quat.w(), -error_quat.x(),
                                        -error_quat.y(), -error_quat.z());
      }
      Eigen::Vector3f y_att;
      const Eigen::Vector3f error_vec(error_quat.x(), error_quat.y(), error_quat.z());
      const float error_vec_norm = error_vec.norm();
      if (error_vec_norm > 1.0e-6f)
      {
        // 用四元数对数映射得到完整旋转向量，避免0.5rad级姿态修正被2*q_vec一阶近似低估。
        const float error_angle_rad = 2.0f * std::atan2(error_vec_norm, error_quat.w());
        y_att = error_vec * (error_angle_rad / error_vec_norm);
      }
      else
      {
        // 极小角度下对数映射退化为2*q_vec，保持数值连续。
        y_att = 2.0f * error_vec;
      }
      if (y_att.norm() > ATTITUDE_RESIDUAL_REJECT_RAD)
      {
        // 姿态辅助量测出现大角度跳变时先按四元数误差拒绝，避免初始大协方差过度放行。
        result.innovation_norm = y_att.norm();
        return result;
      }
      result.innovation_norm = y_att.norm();

      Eigen::Matrix3f s_att = h_att * p_ * h_att.transpose() + r_att;
      Eigen::Matrix3f s_att_inv;
      if (!InvertSymmetricPositiveDefiniteMatrix(s_att, &s_att_inv))
      {
        // S矩阵病态时拒绝该姿态量测，避免AHRS辅助破坏协方差。
        return result;
      }
      const Eigen::Matrix<float, 1, 1> nis_att = y_att.transpose() * s_att_inv * y_att;
      result.test_ratio = std::isfinite(nis_att(0, 0))
                              ? nis_att(0, 0) / ATTITUDE_NIS_REJECT_THRESHOLD
                              : 0.0f;
      if (!std::isfinite(nis_att(0, 0)) || nis_att(0, 0) > ATTITUDE_NIS_REJECT_THRESHOLD)
      {
        // AHRS姿态量测若出现大跳变，按离群处理，不写入延迟重放事件。
        return result;
      }
      Eigen::Matrix<float, 15, 3> k_att = p_ * h_att.transpose() * s_att_inv;

      Eigen::Matrix<float, 15, 15> identity = Eigen::Matrix<float, 15, 15>::Identity();
      p_ = (identity - k_att * h_att) * p_ * (identity - k_att * h_att).transpose() +
           k_att * r_att * k_att.transpose();
      StabilizeCovariance();

      x_ = k_att * y_att;
      InjectErrorState();
      result.fused = true;
      return result;
    }

    bool ApplyYawUpdate(const float measured_yaw_rad, const float measurement_noise_rad)
    {
      return ApplyYawUpdateDetailed(measured_yaw_rad, measurement_noise_rad).fused;
    }

    MeasurementUpdateResult ApplyYawUpdateDetailed(
        const float measured_yaw_rad,
        const float measurement_noise_rad)
    {
      // 航向标量量测的实际实现，保持与原接口相同的稀疏更新路径。
      MeasurementUpdateResult result;
      result.input_valid = true;
      result.attempted = true;
      float y = WrapAngleRad(measured_yaw_rad - yaw_rad());
      result.innovation_norm = std::fabs(y);
      if (std::fabs(y) > YAW_RESIDUAL_REJECT_RAD)
      {
        // 航向辅助量测出现大角度跳变时直接拒绝，避免初始大航向协方差过度放行。
        return result;
      }
      const float safe_measurement_noise_rad = ClampMeasurementNoiseStd(measurement_noise_rad);
      float r = safe_measurement_noise_rad * safe_measurement_noise_rad;
      float s = p_(8, 8) + r;
      if (s < 1e-6f)
        s = 1e-6f;
      const float nis_yaw = y * y / s;
      result.test_ratio = std::isfinite(nis_yaw)
                              ? nis_yaw / YAW_NIS_REJECT_THRESHOLD
                              : 0.0f;
      if (!std::isfinite(nis_yaw) || nis_yaw > YAW_NIS_REJECT_THRESHOLD)
      {
        // 航向标量量测离群时拒绝，避免双矢量/磁航向瞬时跳变拉飞yaw。
        return result;
      }

      Eigen::Matrix<float, 1, 15> h_yaw = Eigen::Matrix<float, 1, 15>::Zero();
      h_yaw(0, 8) = 1.0f;
      Eigen::Matrix<float, 15, 1> k = p_ * h_yaw.transpose() / s;
      Eigen::Matrix<float, 15, 15> identity = Eigen::Matrix<float, 15, 15>::Identity();
      p_ = (identity - k * h_yaw) * p_ * (identity - k * h_yaw).transpose() +
           k * r * k.transpose();
      StabilizeCovariance();

      // x_ 在 InjectErrorState() 末尾必清零，这里与其它量测更新统一用直接赋值，
      // 消除“x_ 可能残留上次误差”的误读。
      x_ = k * y;
      InjectErrorState();
      result.fused = true;
      return result;
    }

  public:
    /**
     * @brief 气压高度标量量测更新 (对标 GNSS 位置 D 分量更新)。
     *
     * 对标 GNSS MeasurementUpdateDetailed 中位置部分: 创新项使用 lla2ned 计算 NED-D 距离,
     * H 矩阵仅观测位置误差状态 D 分量 (x_[2]), 其余状态通过 P 矩阵耦合获得间接修正。
     *
     * 此接口无零偏状态——若 QNH 与标准大气压偏差较大 (数米至数十米),
     * 会与 GNSS 绝对高度基准产生冲突。仅在 GNSS 不可用、或需要观察气压辅助效果时启用。
     *
     * @param baro_altitude_m 气压计测量的海拔高度 (天向为正, 单位 m)。
     * @param baro_noise_std_m 气压高度观测噪声标准差 (单位 m), 典型值 0.35~1.5m。
     * @return true 表示量测通过残差/NIS 检查并实际融合。
     */
    bool MeasurementUpdateBaroAltitude(const float baro_altitude_m,
                                       const float baro_noise_std_m)
    {
      return MeasurementUpdateBaroAltitudeDetailed(baro_altitude_m,
                                                   baro_noise_std_m)
          .fused;
    }

    /**
     * @brief 带诊断结果的气压高度标量量测更新。
     */
    MeasurementUpdateResult MeasurementUpdateBaroAltitudeDetailed(
        const float baro_altitude_m,
        const float baro_noise_std_m)
    {
      MeasurementUpdateResult result;
      if (!std::isfinite(baro_altitude_m) || !IsValidNoiseStd(baro_noise_std_m))
      {
        return result;
      }
      result.input_valid = true;
      result = ApplyBaroAltitudeUpdateDetailed(baro_altitude_m, baro_noise_std_m);
      if (result.fused)
      {
        RecordBaroAltitudeUpdateOnLatestSnapshot(baro_altitude_m,
                                                 baro_noise_std_m);
      }
      return result;
    }

  private:
    MeasurementUpdateResult ApplyBaroAltitudeUpdateDetailed(
        const float baro_altitude_m,
        const float baro_noise_std_m)
    {
      // 对标 GNSS 位置 D 分量观测: 创新项 = NED D 方向位移 (已预测 -> 观测)
      // y = lla2ned(baro_lla, pred_lla).z()
      // 当 baro 高于 EKF: y < 0 (NED D 为负 = 向上), 经 K*y 修正后高度增加。
      MeasurementUpdateResult result;
      result.input_valid = true;
      result.attempted = true;

      const Eigen::Vector3d baro_lla(ins_lla_rad_m_(0), ins_lla_rad_m_(1),
                                     static_cast<double>(baro_altitude_m));
      const Eigen::Vector3d ned_diff = lla2ned(baro_lla, ins_lla_rad_m_,
                                               bfs::AngPosUnit::RAD);
      const float y = static_cast<float>(ned_diff(2)); // D 分量 = 创新项
      result.innovation_norm = std::fabs(y);

      // 硬残差门限: 拒绝超过 50m 的高度跳变 (QNH 突变 / 传感器故障)
      if (std::fabs(y) > BARO_ALTITUDE_RESIDUAL_REJECT_M)
      {
        return result;
      }

      const float safe_noise_std_m = ClampMeasurementNoiseStd(baro_noise_std_m);
      const float r = safe_noise_std_m * safe_noise_std_m;
      float s = p_(2, 2) + r; // P[2][2] = 位置误差 D 分量方差
      if (s < 1e-9f) s = 1e-9f;

      const float nis_baro = y * y / s;
      result.test_ratio = std::isfinite(nis_baro)
                              ? nis_baro / BARO_ALTITUDE_NIS_REJECT_THRESHOLD
                              : 0.0f;
      if (!std::isfinite(nis_baro) || nis_baro > BARO_ALTITUDE_NIS_REJECT_THRESHOLD)
      {
        return result;
      }

      // 标量卡尔曼更新: H = [0, 0, 1, 0, ...] (1x15, 观测位置误差 D 分量)
      Eigen::Matrix<float, 1, 15> h_baro = Eigen::Matrix<float, 1, 15>::Zero();
      h_baro(0, 2) = 1.0f;
      // K = P * H^T / S
      const Eigen::Matrix<float, 15, 1> k = p_ * h_baro.transpose() / s;

      // Joseph 形式协方差更新: P = (I-KH) * P * (I-KH)^T + K * R * K^T
      Eigen::Matrix<float, 15, 15> identity = Eigen::Matrix<float, 15, 15>::Identity();
      p_ = (identity - k * h_baro) * p_ * (identity - k * h_baro).transpose() +
           k * r * k.transpose();
      StabilizeCovariance();

      x_ = k * y;
      InjectErrorState();
      result.fused = true;
      return result;
    }

  public:
    void UpdateGnssNoiseMatrix()
    {
      r_.setZero();
      const float safe_gnss_pos_ne_std_m = ClampMeasurementNoiseStd(gnss_pos_ne_std_m_);
      const float safe_gnss_pos_d_std_m = ClampMeasurementNoiseStd(gnss_pos_d_std_m_);
      const float safe_gnss_vel_ne_std_mps = ClampMeasurementNoiseStd(gnss_vel_ne_std_mps_);
      const float safe_gnss_vel_d_std_mps = ClampMeasurementNoiseStd(gnss_vel_d_std_mps_);
      r_.block(0, 0, 2, 2) = safe_gnss_pos_ne_std_m * safe_gnss_pos_ne_std_m *
                             Eigen::Matrix<float, 2, 2>::Identity();
      r_(2, 2) = safe_gnss_pos_d_std_m * safe_gnss_pos_d_std_m;
      r_.block(3, 3, 2, 2) = safe_gnss_vel_ne_std_mps * safe_gnss_vel_ne_std_mps *
                             Eigen::Matrix<float, 2, 2>::Identity();
      r_(5, 5) = safe_gnss_vel_d_std_mps * safe_gnss_vel_d_std_mps;
    }

    void RecordSnapshot(const int idx, const Eigen::Vector3f &accel,
                        const Eigen::Vector3f &gyro, const float dt_s)
    {
      // 记录传播后的完整状态和本步 IMU 输入，供延迟 GNSS 更新后从历史点重放。
      state_buffer_[idx].lla = ins_lla_rad_m_;
      state_buffer_[idx].ned_vel = ins_ned_vel_mps_;
      state_buffer_[idx].quat = quat_;
      state_buffer_[idx].accel_bias = accel_bias_mps2_;
      state_buffer_[idx].gyro_bias = gyro_bias_radps_;
      state_buffer_[idx].previous_delta_theta = previous_delta_theta_rad_;
      state_buffer_[idx].previous_delta_v = previous_delta_v_mps_;
      state_buffer_[idx].p = p_;
      state_buffer_[idx].accel_input = accel;
      state_buffer_[idx].gyro_input = gyro;
      state_buffer_[idx].dt_s = dt_s;
      ClearReplayEvents(state_buffer_[idx]);
    }

    void ClearReplayEvents(InsSnapshot &snapshot)
    {
      // 每个 TimeUpdate 只对应当前导航步的非GNSS量测事件，覆盖旧环形槽位时必须清空。
      snapshot.has_velocity_update = false;
      snapshot.has_gravity_update = false;
      snapshot.has_attitude_update = false;
      snapshot.has_yaw_update = false;
      snapshot.has_static_gyro_update = false;
      snapshot.has_baro_altitude_update = false;
    }

    void ResetHistoryBuffer()
    {
      // 绝对位置重锚定后，旧历史快照仍处于旧原点，不能再参与GNSS延迟回放。
      for (int idx = 0; idx < BUFFER_SIZE; ++idx)
      {
        ResetSnapshot(state_buffer_[idx]);
      }
      buf_head_ = 0;
      buf_full_ = false;
      buf_count_ = 0;
      // 首快照 dt 使用最近一次通过校验的真实导航周期，而不是硬编码 5 ms：
      // 57.14 Hz 高精度环境的周期是 17.5 ms，硬编码会让重锚定后第一次
      // GNSS 延迟回溯按错误时间对齐历史状态。
      RecordSnapshot(buf_head_, ins_accel_mps2_, ins_gyro_radps_, last_valid_dt_s_);
      buf_head_ = 1;
      buf_count_ = 1;
    }

    void ResetSnapshot(InsSnapshot &snapshot)
    {
      snapshot.lla = Eigen::Vector3d::Zero();
      snapshot.ned_vel = Eigen::Vector3f::Zero();
      snapshot.quat = Eigen::Quaternionf::Identity();
      snapshot.accel_bias = Eigen::Vector3f::Zero();
      snapshot.gyro_bias = Eigen::Vector3f::Zero();
      snapshot.previous_delta_theta = Eigen::Vector3f::Zero();
      snapshot.previous_delta_v = Eigen::Vector3f::Zero();
      snapshot.p.setZero();
      snapshot.accel_input = Eigen::Vector3f::Zero();
      snapshot.gyro_input = Eigen::Vector3f::Zero();
      snapshot.delta_theta_1 = Eigen::Vector3f::Zero();
      snapshot.delta_theta_2 = Eigen::Vector3f::Zero();
      snapshot.delta_v_1 = Eigen::Vector3f::Zero();
      snapshot.delta_v_2 = Eigen::Vector3f::Zero();
      snapshot.dt_s = 0.0f;
      snapshot.has_two_sample_increment = false;
      snapshot.baro_altitude_m = 0.0f;
      snapshot.baro_noise_std_m = 0.0f;
      ClearReplayEvents(snapshot);
    }

    int LatestSnapshotIndex() const
    {
      if (buf_count_ <= 0)
      {
        return -1;
      }
      return (buf_head_ - 1 + BUFFER_SIZE) % BUFFER_SIZE;
    }

    void RecordVelocityUpdateOnLatestSnapshot(const Eigen::Vector3f &measured_vel_ned,
                                              const float vel_ne_std_mps,
                                              const float vel_d_std_mps)
    {
      const int idx = LatestSnapshotIndex();
      if (idx < 0)
      {
        return;
      }
      state_buffer_[idx].has_velocity_update = true;
      state_buffer_[idx].velocity_meas = measured_vel_ned;
      state_buffer_[idx].velocity_ne_noise = vel_ne_std_mps;
      state_buffer_[idx].velocity_d_noise = vel_d_std_mps;
      OverwriteSnapshotState(idx);
    }

    void RecordGravityUpdateOnLatestSnapshot(const Eigen::Vector3f &measured_accel_mps2,
                                             const float accel_dir_noise_rad)
    {
      const int idx = LatestSnapshotIndex();
      if (idx < 0)
      {
        return;
      }
      state_buffer_[idx].has_gravity_update = true;
      state_buffer_[idx].gravity_accel_meas = measured_accel_mps2;
      state_buffer_[idx].gravity_noise = accel_dir_noise_rad;
      OverwriteSnapshotState(idx);
    }

    void RecordAttitudeUpdateOnLatestSnapshot(const Eigen::Vector3f &measured_ypr_rad,
                                              const float roll_pitch_noise_rad,
                                              const float yaw_noise_rad)
    {
      const int idx = LatestSnapshotIndex();
      if (idx < 0)
      {
        return;
      }
      state_buffer_[idx].has_attitude_update = true;
      state_buffer_[idx].attitude_ypr_meas = measured_ypr_rad;
      state_buffer_[idx].attitude_roll_pitch_noise = roll_pitch_noise_rad;
      state_buffer_[idx].attitude_yaw_noise = yaw_noise_rad;
      OverwriteSnapshotState(idx);
    }

    void RecordYawUpdateOnLatestSnapshot(const float measured_yaw_rad,
                                         const float measurement_noise_rad)
    {
      const int idx = LatestSnapshotIndex();
      if (idx < 0)
      {
        return;
      }
      state_buffer_[idx].has_yaw_update = true;
      state_buffer_[idx].yaw_meas = measured_yaw_rad;
      state_buffer_[idx].yaw_noise = measurement_noise_rad;
      OverwriteSnapshotState(idx);
    }

    void RecordStaticGyroUpdateOnLatestSnapshot(const Eigen::Vector3f &measured_gyro_radps,
                                                const Eigen::Vector3f &gyro_noise_radps)
    {
      const int idx = LatestSnapshotIndex();
      if (idx < 0)
      {
        return;
      }
      state_buffer_[idx].has_static_gyro_update = true;
      state_buffer_[idx].static_gyro_meas = measured_gyro_radps;
      state_buffer_[idx].static_gyro_noise = gyro_noise_radps;
      OverwriteSnapshotState(idx);
    }

    void RecordBaroAltitudeUpdateOnLatestSnapshot(const float baro_altitude_m,
                                                  const float baro_noise_std_m)
    {
      const int idx = LatestSnapshotIndex();
      if (idx < 0)
      {
        return;
      }
      state_buffer_[idx].has_baro_altitude_update = true;
      state_buffer_[idx].baro_altitude_m = baro_altitude_m;
      state_buffer_[idx].baro_noise_std_m = baro_noise_std_m;
      OverwriteSnapshotState(idx);
    }

    void OverwriteSnapshotState(const int idx)
    {
      // 量测后更新快照的状态/协方差，使后续GNSS延迟回放从一致状态继续。
      state_buffer_[idx].lla = ins_lla_rad_m_;
      state_buffer_[idx].ned_vel = ins_ned_vel_mps_;
      state_buffer_[idx].quat = quat_;
      state_buffer_[idx].accel_bias = accel_bias_mps2_;
      state_buffer_[idx].gyro_bias = gyro_bias_radps_;
      state_buffer_[idx].previous_delta_theta = previous_delta_theta_rad_;
      state_buffer_[idx].previous_delta_v = previous_delta_v_mps_;
      state_buffer_[idx].p = p_;
    }

    void CopyReplayEvents(InsSnapshot *dst, const InsSnapshot &src)
    {
      // 保留该导航步原本发生过的非GNSS量测事件，供后续延迟更新继续重放。
      dst->has_velocity_update = src.has_velocity_update;
      dst->velocity_meas = src.velocity_meas;
      dst->velocity_ne_noise = src.velocity_ne_noise;
      dst->velocity_d_noise = src.velocity_d_noise;
      dst->has_gravity_update = src.has_gravity_update;
      dst->gravity_accel_meas = src.gravity_accel_meas;
      dst->gravity_noise = src.gravity_noise;
      dst->has_attitude_update = src.has_attitude_update;
      dst->attitude_ypr_meas = src.attitude_ypr_meas;
      dst->attitude_roll_pitch_noise = src.attitude_roll_pitch_noise;
      dst->attitude_yaw_noise = src.attitude_yaw_noise;
      dst->has_yaw_update = src.has_yaw_update;
      dst->yaw_meas = src.yaw_meas;
      dst->yaw_noise = src.yaw_noise;
      dst->has_static_gyro_update = src.has_static_gyro_update;
      dst->static_gyro_meas = src.static_gyro_meas;
      dst->static_gyro_noise = src.static_gyro_noise;
      dst->has_baro_altitude_update = src.has_baro_altitude_update;
      dst->baro_altitude_m = src.baro_altitude_m;
      dst->baro_noise_std_m = src.baro_noise_std_m;
    }

    void ReplayNonGnssUpdates(const InsSnapshot &snapshot)
    {
      // 按主任务执行顺序重放同一导航步里的非GNSS量测：姿态/陀螺零偏约束先于ZUPT。
      if (snapshot.has_gravity_update)
      {
        ApplyGravityUpdate(snapshot.gravity_accel_meas, snapshot.gravity_noise);
      }
      if (snapshot.has_attitude_update)
      {
        ApplyAttitudeUpdate(snapshot.attitude_ypr_meas,
                            snapshot.attitude_roll_pitch_noise,
                            snapshot.attitude_yaw_noise);
      }
      if (snapshot.has_yaw_update)
      {
        ApplyYawUpdate(snapshot.yaw_meas, snapshot.yaw_noise);
      }
      if (snapshot.has_static_gyro_update)
      {
        ApplyStaticGyroUpdate(snapshot.static_gyro_meas, snapshot.static_gyro_noise);
      }
      if (snapshot.has_velocity_update)
      {
        ApplyVelocityUpdate(snapshot.velocity_meas,
                            snapshot.velocity_ne_noise,
                            snapshot.velocity_d_noise);
      }
      if (snapshot.has_baro_altitude_update)
      {
        ApplyBaroAltitudeUpdateDetailed(snapshot.baro_altitude_m,
                                        snapshot.baro_noise_std_m);
      }
    }

    bool GetDelayedSnapshotIndexByAge(const float delay_s, int *idx) const
    {
      // 按快照dt反向累计真实时间，而不是用单帧dt估计步数，适应飞控任务调度抖动。
      if (!std::isfinite(delay_s) || delay_s < 0.0f || idx == nullptr)
      {
        return false;
      }
      const int latest_idx = LatestSnapshotIndex();
      if (latest_idx < 0)
      {
        return false;
      }
      if (delay_s <= 0.0f)
      {
        *idx = latest_idx;
        return true;
      }
      if (buf_count_ <= 1)
      {
        return false;
      }

      float accumulated_s = 0.0f;
      float best_error_s = delay_s;
      int best_idx = -1;
      int cursor = latest_idx;
      for (int steps_back = 1; steps_back < buf_count_; ++steps_back)
      {
        const float segment_dt_s = state_buffer_[cursor].dt_s;
        if (!IsValidTimeStep(segment_dt_s))
        {
          // 历史dt异常说明缓冲区不能可靠对齐GNSS时间，降级为当前时刻更新。
          return false;
        }
        accumulated_s += segment_dt_s;
        const int candidate_idx = (cursor - 1 + BUFFER_SIZE) % BUFFER_SIZE;
        const float error_s = std::fabs(accumulated_s - delay_s);
        if (error_s <= best_error_s)
        {
          best_error_s = error_s;
          best_idx = candidate_idx;
        }
        if (accumulated_s >= delay_s)
        {
          break;
        }
        cursor = candidate_idx;
      }

      if (accumulated_s < delay_s || best_idx < 0)
      {
        return false;
      }
      *idx = best_idx;
      return true;
    }

    void RestoreSnapshot(const InsSnapshot &snapshot)
    {
      // 回到 GNSS 量测实际对应的历史时刻，再执行卡尔曼更新。
      ins_lla_rad_m_ = snapshot.lla;
      ins_ned_vel_mps_ = snapshot.ned_vel;
      quat_ = snapshot.quat.normalized();
      MarkEulerAnglesDirty();
      t_b2ned = quat2dcm(quat_).transpose();
      accel_bias_mps2_ = snapshot.accel_bias;
      gyro_bias_radps_ = snapshot.gyro_bias;
      previous_delta_theta_rad_ = snapshot.previous_delta_theta;
      previous_delta_v_mps_ = snapshot.previous_delta_v;
      p_ = snapshot.p;
      StabilizeCovariance();
    }

    void RepropagateFromDelayedSnapshot(const int hist_idx)
    {
      // 历史时刻完成量测修正后，重放之后的 IMU 输入，把状态重新推进到当前时刻。
      int steps = (buf_head_ - 1 - hist_idx + BUFFER_SIZE) % BUFFER_SIZE;
      if (steps <= 0 || steps >= BUFFER_SIZE)
      {
        return;
      }

      int replay_indices[BUFFER_SIZE];
      for (int i = 0; i < steps; ++i)
      {
        replay_indices[i] = (hist_idx + 1 + i) % BUFFER_SIZE;
      }

      for (int i = 0; i < steps; ++i)
      {
        const InsSnapshot replay = state_buffer_[replay_indices[i]];
        if (!IsValidTimeStep(replay.dt_s))
        {
          // 历史槽位若被异常dt污染则跳过，防止延迟重传播再次放大坏数据。
          continue;
        }
        if (replay.has_two_sample_increment)
        {
          const float half_dt_s = 0.5f * replay.dt_s;
          const Eigen::Vector3f corrected_delta_theta_1_rad =
              replay.delta_theta_1 - gyro_bias_radps_ * half_dt_s;
          const Eigen::Vector3f corrected_delta_theta_2_rad =
              replay.delta_theta_2 - gyro_bias_radps_ * half_dt_s;
          const Eigen::Vector3f corrected_delta_v_1_mps =
              replay.delta_v_1 - accel_bias_mps2_ * half_dt_s;
          const Eigen::Vector3f corrected_delta_v_2_mps =
              replay.delta_v_2 - accel_bias_mps2_ * half_dt_s;
          const Eigen::Vector3f avg_corrected_gyro_radps =
              (corrected_delta_theta_1_rad + corrected_delta_theta_2_rad) /
              replay.dt_s;
          const Eigen::Vector3f avg_corrected_accel_mps2 =
              (corrected_delta_v_1_mps + corrected_delta_v_2_mps) / replay.dt_s;
          PropagateStateFromIncrements(avg_corrected_accel_mps2,
                                       avg_corrected_gyro_radps,
                                       corrected_delta_theta_1_rad,
                                       corrected_delta_theta_2_rad,
                                       corrected_delta_v_1_mps,
                                       corrected_delta_v_2_mps,
                                       replay.dt_s,
                                       true);
        }
        else
        {
          PropagateState(replay.accel_input, replay.gyro_input, replay.dt_s);
        }
        ReplayNonGnssUpdates(replay);
        RecordSnapshot(replay_indices[i], replay.accel_input, replay.gyro_input, replay.dt_s);
        state_buffer_[replay_indices[i]].has_two_sample_increment =
            replay.has_two_sample_increment;
        state_buffer_[replay_indices[i]].delta_theta_1 = replay.delta_theta_1;
        state_buffer_[replay_indices[i]].delta_theta_2 = replay.delta_theta_2;
        state_buffer_[replay_indices[i]].delta_v_1 = replay.delta_v_1;
        state_buffer_[replay_indices[i]].delta_v_2 = replay.delta_v_2;
        CopyReplayEvents(&state_buffer_[replay_indices[i]], replay);
      }
    }

    // ==================================================================
    // GNSS 延迟回放历史环形缓冲区
    // ==================================================================
    //
    // 设计动机：GNSS 接收机输出的 LLA/速度对应的是"导航历元时刻"的状态，
    // 不是 MCU 收到串口字节的时刻。以 u-blox UBX-NAV-PVT 为例：
    // - 串口传输延迟 (921600 baud, ~100 B): ~1 ms
    // - 接收机内部输出排队 + MCU 缓冲 + 主循环调度抖动: 数 ms 量级
    // - 当前默认 GNSS_DELAY_S = 0.015 s (15 ms) 覆盖了这些延迟来源。
    //
    // 工作机制：
    // 1. 每个 TimeUpdate 后在环形缓冲中记录完整状态快照 (InsSnapshot)。
    // 2. GNSS 量测到达时，按 GNSS_DELAY_S 从缓冲中搜索最匹配的历史时刻。
    // 3. 恢复到该时刻 → 执行卡尔曼更新 (MeasurementUpdateDetailed)。
    // 4. 重放历史时刻到当前时刻之间的所有 IMU 输入，将状态推回"现在"。
    //
    // 缓冲区大小：
    // 200 Hz 导航时 64 格可覆盖 320 ms 历史，远超 15 ms 典型延迟，
    // 为 500 Hz + PPS 延迟精确定标留有充分余量。
    // 纯惯导测试时 DISABLE_GNSS_DELAY_REPLAY=1 将缓冲缩为 1 格，关闭重放链路。

    static const int BUFFER_SIZE =
#if BFS_NAVIGATION_EMBEDDED_DISABLE_GNSS_DELAY_REPLAY
        1;
#else
        64;
#endif

    /* 固定大小环形历史状态缓冲，栈/静态区分配，避免飞控运行期堆操作。 */
    std::array<InsSnapshot, BUFFER_SIZE> state_buffer_;

    int buf_head_ = 0;      // 环形缓冲当前写入位置索引
    bool buf_full_ = false; // 首次填满后置 true，此后所有历史都在缓冲内
    int buf_count_ = 0;     // 当前已写入的有效快照数量 (≤ BUFFER_SIZE)

    /* GNSS 固定回溯延迟，单位 s。默认由编译宏 BFS_NAVIGATION_GNSS_DELAY_S 配置,
       PPS/TIMEPULSE 实测标定后可在 platformio.ini 中覆盖该宏。 */
    const float GNSS_DELAY_S = BFS_NAVIGATION_GNSS_DELAY_S;
    // 6维GNSS位置/速度量测的NIS门限；略高于99.9%卡方门限，保留机动容差并拒绝明显跳点。
    static constexpr float GNSS_NIS_REJECT_THRESHOLD = 30.0f;
    // 长时间失联后的GNSS重捕获门限：只对物理残差合理的量测启用膨胀R低增益融合。
    static constexpr float GNSS_REACQUIRE_MIN_OUTAGE_S = 2.0f;
    static constexpr float GNSS_REACQUIRE_WINDOW_S = 6.0f;
    static constexpr float GNSS_REACQUIRE_NIS_REJECT_THRESHOLD = 80.0f;
    static constexpr float GNSS_REACQUIRE_MAX_POS_NE_RESIDUAL_M = 25.0f;
    static constexpr float GNSS_REACQUIRE_MAX_POS_D_RESIDUAL_M = 35.0f;
    static constexpr float GNSS_REACQUIRE_MAX_VEL_NE_RESIDUAL_MPS = 6.0f;
    static constexpr float GNSS_REACQUIRE_MAX_VEL_D_RESIDUAL_MPS = 5.0f;
    // 重捕获残差还必须与接收机自报精度相称，避免低质量恢复窗口接受十几米级假台阶。
    static constexpr float GNSS_REACQUIRE_RESIDUAL_SIGMA_GATE = 3.0f;
    static constexpr float GNSS_REACQUIRE_MIN_POS_NE_RESIDUAL_GATE_M = 6.0f;
    static constexpr float GNSS_REACQUIRE_MIN_POS_D_RESIDUAL_GATE_M = 8.0f;
    static constexpr float GNSS_REACQUIRE_POS_NE_EXTRA_STD_M = 2.0f;
    static constexpr float GNSS_REACQUIRE_POS_D_EXTRA_STD_M = 4.0f;
    static constexpr float GNSS_REACQUIRE_VEL_NE_EXTRA_STD_MPS = 0.5f;
    static constexpr float GNSS_REACQUIRE_VEL_D_EXTRA_STD_MPS = 0.7f;
    static constexpr float GNSS_OUTAGE_TIME_CAP_S = 60.0f;
    static constexpr float GNSS_OUTAGE_POS_NE_GROWTH_MPS = 1.2f;
    static constexpr float GNSS_OUTAGE_POS_D_GROWTH_MPS = 1.8f;
    static constexpr float GNSS_OUTAGE_VEL_NE_GROWTH_MPS2 = 0.35f;
    static constexpr float GNSS_OUTAGE_VEL_D_GROWTH_MPS2 = 0.50f;
    // 3维速度量测NIS门限；ZUPT需要容纳短时惯导漂移，只拒绝数米每秒级明显速度跳点。
    static constexpr float VELOCITY_NIS_REJECT_THRESHOLD = 1000.0f;
    // 速度辅助接口的物理残差门限；容纳静止确认后的较大惯导漂移，只拒绝十几米每秒级荒谬跳点。
    static constexpr float VELOCITY_RESIDUAL_REJECT_MPS = 8.0f;
    // 零速修正专用门限；误静止确认时不允许把高速运动强行拉成静止。
    static constexpr float ZERO_VELOCITY_MEAS_NORM_MPS = 0.30f;
    static constexpr float ZERO_VELOCITY_RESIDUAL_REJECT_MPS = 5.5f;
    // 3维重力方向量测NIS门限；拒绝方向突变，避免机动加速度被当成重力。
    static constexpr float GRAVITY_NIS_REJECT_THRESHOLD = 18.0f;
    // 静止角速度量测门限；用于三轴陀螺零偏闭环，只允许小残差修正。
    static constexpr float STATIC_GYRO_NIS_REJECT_THRESHOLD = 24.0f;
    static constexpr float STATIC_GYRO_RESIDUAL_REJECT_RADPS = 0.05f;
    // 3维姿态量测NIS门限；拒绝AHRS姿态瞬时跳变。
    static constexpr float ATTITUDE_NIS_REJECT_THRESHOLD = 18.0f;
    // 姿态辅助量测物理残差门限；保留小角度修正，拒绝约45度以上的单帧跳变。
    static constexpr float ATTITUDE_RESIDUAL_REJECT_RAD = 0.8f;
    // 1维航向量测NIS门限；高于99.9%卡方门限，保留真实小修正但拒绝大跳变。
    static constexpr float YAW_NIS_REJECT_THRESHOLD = 12.0f;
    // 航向辅助量测物理残差门限；保留中小幅航向修正，拒绝约57度以上的单帧跳变。
    static constexpr float YAW_RESIDUAL_REJECT_RAD = 1.0f;
    // 气压高度标量量测 NIS 门限；χ²(1) 99.99% 分位数 ≈ 15.1，取 16 为 4σ 等效。
    static constexpr float BARO_ALTITUDE_NIS_REJECT_THRESHOLD = 16.0f;
    // 气压高度硬残差门限；拒绝超过 50m 的高度跳变 (QNH 突变 / 传感器故障)。
    static constexpr float BARO_ALTITUDE_RESIDUAL_REJECT_M = 50.0f;
    // TimeUpdate单步最大允许积分时间；超过该值时宁可丢弃该步，也不把异常时间戳写入状态。
    static constexpr float MAX_TIME_UPDATE_DT_S = 0.1f;
    // 协方差PSD投影下限，只用于数值稳定化，不作为真实不确定度调参通道。
    static constexpr float MIN_COVARIANCE_EIGENVALUE = 1.0e-9f;
    static constexpr float MIN_COVARIANCE_ALLOWED_EIGENVALUE = -1.0e-5f;
    static constexpr float MIN_COVARIANCE_PSD_ABSOLUTE_TOLERANCE = 1.0e-5f;
    static constexpr float RELATIVE_COVARIANCE_PSD_TOLERANCE = 1.0e-7f;
    static constexpr float RELATIVE_COVARIANCE_NEGATIVE_EIGEN_FLOOR = 1.0e-10f;
    // 运行期IMU物理合理性门限；超过传感器量程很多的有限值视为坏帧。
    static constexpr float MAX_PLAUSIBLE_ACCEL_MPS2 = 120.0f;
    static constexpr float MAX_PLAUSIBLE_GYRO_RADPS = 40.0f;
    // 量测噪声标准差下限；外部精度再小也不能让R矩阵接近奇异。
    static constexpr float MIN_MEASUREMENT_NOISE_STD = 1.0e-2f;
    static constexpr float MIN_STATIC_GYRO_MEASUREMENT_NOISE_STD = 1.0e-5f;

    // ==================================================================
    // 传感器噪声模型参数 —— 加速度计和陀螺仪使用一阶高斯-马尔可夫
    // (Gauss-Markov) 过程建模白噪声 + 零偏随机游走。
    // 默认值仅为编译期兜底，实机构建时由 platformio.ini 的 -D 宏覆盖。
    // ==================================================================

    /* 加速度计白噪声标准差 (Velocity Random Walk)，单位 m/s² */
    float accel_std_mps2_ = 0.05f;

    /* 加速度计零偏一阶 Markov 过程驱动噪声标准差，单位 m/s²。
       物理含义：零偏随时间缓慢随机游走的强度。 */
    float accel_markov_bias_std_mps2_ = 0.01f;

    /* 加速度计零偏 Markov 过程相关时间常数，单位 s。
       τ 越大 → 零偏变化越慢 → 长时稳定性越好但短时修正越保守。 */
    float accel_tau_s_ = 100.0f;

    /* 加速度计零偏状态转移矩阵 (Fs 块)：
       F_accel_bias = (-1/τ) * I₃
       描述零偏随时间衰减回零的速率（高通特性）。 */
    Eigen::Matrix3f accel_markov_bias_ = -1.0f / accel_tau_s_ *
                                         Eigen::Matrix<float, 3, 3>::Identity();

    /* 陀螺仪白噪声标准差 (Angle Random Walk, ARW)，单位 rad/s */
    float gyro_std_radps_ = 0.00175f;

    /* 陀螺仪零偏一阶 Markov 过程驱动噪声标准差，单位 rad/s */
    float gyro_markov_bias_std_radps_ = 0.00025f;

    /* 陀螺仪零偏 Markov 过程相关时间常数，单位 s */
    float gyro_tau_s_ = 50.0f;

    /* 陀螺仪零偏状态转移矩阵 (Fs 块)：F_gyro_bias = (-1/τ) * I₃ */
    Eigen::Matrix3f gyro_markov_bias_ = -1.0f / gyro_tau_s_ *
                                        Eigen::Matrix<float, 3, 3>::Identity();

    /* GNSS 水平位置 (N/E) 量测噪声标准差，单位 m。
       在 UpdateGnssNoiseMatrix() 中平方后填入 R(0,0) 和 R(1,1)。 */
    float gnss_pos_ne_std_m_ = 3.0f;

    /* GNSS 垂直位置 (D) 量测噪声标准差，单位 m。
       垂直精度通常低于水平精度，默认取水平的两倍。 */
    float gnss_pos_d_std_m_ = 6.0f;

    /* GNSS 水平速度 (N/E) 量测噪声标准差，单位 m/s */
    float gnss_vel_ne_std_mps_ = 0.5f;

    /* GNSS 垂直速度 (D) 量测噪声标准差，单位 m/s */
    float gnss_vel_d_std_mps_ = 1.0f;

    // ==================================================================
    // 初始协方差矩阵 P₀ 参数 —— 描述 EKF 对初始状态的不确定度。
    // 值越大 → EKF 越信任初始量测，收敛越快但过渡段振荡越大。
    // ==================================================================

    /* 初始位置误差标准差，单位 m */
    float init_pos_err_std_m_ = 10.0f;

    /* 初始速度误差标准差，单位 m/s */
    float init_vel_err_std_mps_ = 1.0f;

    /* 初始姿态 (roll/pitch) 误差标准差，单位 rad，约 20°。
       来自静基座加速度计调平的不确定度。 */
    float init_att_err_std_rad_ = 0.34906f;

    /* 初始航向 (yaw) 误差标准差，单位 rad，约 180° (≈ π)。
       无磁力计/GNSS 航向时使用大不确定度，表示"完全不知道初始航向"。 */
    float init_heading_err_std_rad_ = 3.14159f;

    /* 初始加速度计零偏不确定度标准差，单位 m/s²，约 0.1g */
    float init_accel_bias_std_mps2_ = 0.9810f;

    /* 初始陀螺零偏不确定度标准差，单位 rad/s，约 1°/s */
    float init_gyro_bias_std_radps_ = 0.01745f;

    // ==================================================================
    // 卡尔曼滤波器核心矩阵
    // ==================================================================

    /* 观测矩阵 H (6×15)：将 15 维状态空间映射到 6 维 GNSS 观测空间。
       H[0:5, 0:5] = I₆，即前 6 个误差状态 (位置+速度) 被 GNSS 直接观测。 */
    Eigen::Matrix<float, 6, 15> h_ = Eigen::Matrix<float, 6, 15>::Zero();

    /* GNSS 6 维观测噪声协方差矩阵 R (6×6)，由 UpdateGnssNoiseMatrix() 动态更新。 */
    Eigen::Matrix<float, 6, 6> r_ = Eigen::Matrix<float, 6, 6>::Zero();

    /* IMU 传感器连续噪声协方差矩阵 Rw (12×12)：
       前 3 维：加速度白噪声 (m²/s⁴·s = (m/s²)²)
       中 3 维：陀螺白噪声 (rad²/s²·s = (rad/s)²)
       后 6 维：加速度/陀螺零偏 Markov 驱动噪声 (2σ²/τ) */
    Eigen::Matrix<float, 12, 12> rw_ = Eigen::Matrix<float, 12, 12>::Zero();

    /* 噪声输入矩阵 Gs (15×12)：将 12 维权值噪声映射到 15 维状态空间。
       结构为固定稀疏块（速度←加速度、姿态←陀螺、零偏←自身）。 */
    Eigen::Matrix<float, 15, 12> gs_ = Eigen::Matrix<float, 15, 12>::Zero();

    /* 新息协方差矩阵 S (6×6)：S = H·P·Hᵀ + R
       在 ComputeGnssInnovationStats() 中计算，LDLT 分解求逆。 */
    Eigen::Matrix<float, 6, 6> s_ = Eigen::Matrix<float, 6, 6>::Zero();

    /* 状态估计协方差矩阵 P (15×15)。
       这是 EKF 的核心不确定度度量：对角元素为各状态的方差，
       非对角元素为状态间协方差（耦合关系）。 */
    Eigen::Matrix<float, 15, 15> p_ = Eigen::Matrix<float, 15, 15>::Zero();

    /* 快速稳定化路径计数器，用于周期性触发完整 LDLT/特征分解检查。 */
    uint32_t covariance_stabilize_count_ = 0;

    /* 传播路径稳定化跳步计数：每跳 propagation_stabilize_divider_-1 个传播周期
       才做一次完整稳定化，在确认静止长时场景中用于减频削峰。 */
    uint32_t propagation_stabilize_skip_count_ = 0;

    /* 静止辅助量测路径稳定化跳步计数：独立于传播路径，只用于 ZUPT/Gravity/
       StaticGyro 量测更新后的稳定化减频。 */
    uint32_t static_aid_stabilize_skip_count_ = 0;

    /* 传播路径运行时稳定化分频系数。
       默认继承编译期宏，但允许上层 (updateRuntimeStabilizePolicy) 在
       确认静止长驻留时动态放大，降低桌面静止场景的 CPU 峰值。 */
    uint32_t propagation_stabilize_divider_ =
        std::max<uint32_t>(1U, static_cast<uint32_t>(
                                   BFS_NAVIGATION_EMBEDDED_PROPAGATION_STABILIZE_DIVIDER));

    /* 静止辅助路径运行时稳定化分频系数。
       同样允许按工况动态调整，与传播 divider 独立以避免锁相。 */
    uint32_t static_aid_stabilize_divider_ =
        std::max<uint32_t>(1U, static_cast<uint32_t>(
                                   BFS_NAVIGATION_EMBEDDED_STATIC_AID_STABILIZE_DIVIDER));

    /* 离散过程噪声协方差矩阵 Q (15×15)。
       在 PropagateStateFromIncrements() 末尾由连续噪声 Rw 离散化得到，
       形式取决于编译宏：快速 float 路径用对角注入，高精度路径用 Simpson Qd。 */
    Eigen::Matrix<float, 15, 15> q_ = Eigen::Matrix<float, 15, 15>::Zero();

    /* 卡尔曼增益矩阵 K (15×6)：K = P·Hᵀ·S⁻¹
       决定观测对状态的修正权重——S 大 (观测不可信) → K 小 → 倾向保持预测。 */
    Eigen::Matrix<float, 15, 6> k_ = Eigen::Matrix<float, 15, 6>::Zero();

    /* 连续时间系统矩阵 Fs (15×15)：误差状态动力学的线性化 Jacobian。
       结构包含固定块 (pos←vel I₃, bias←Markov 衰减) 和动态块
       (速度←比力斜对称、姿态←角速度斜对称、高度通道重力梯度)。 */
    Eigen::Matrix<float, 15, 15> fs_ = Eigen::Matrix<float, 15, 15>::Zero();

    /* 离散时间状态转移矩阵 Φ (15×15)：由 Fs 经矩阵指数展开得到。
       二阶展开 Φ ≈ I + F·dt + ½(F·dt)²，一阶快速路径用 Φ ≈ I + F·dt。 */
    Eigen::Matrix<float, 15, 15> phi_ = Eigen::Matrix<float, 15, 15>::Zero();

    /* 新息向量 y (6×1)：y = z - h(x)，即 GNSS 观测与 EKF 预测的残差。
       前 3 维为 NED 位置差 (m)，后 3 维为 NED 速度差 (m/s)。 */
    Eigen::Matrix<float, 6, 1> y_ = Eigen::Matrix<float, 6, 1>::Zero();

    /* 误差状态向量 x (15×1)：存储卡尔曼更新计算出的误差状态修正量。
       在 InjectErrorState() 中注入全量状态后清零。
       顺序：[pos_err(3), vel_err(3), att_err(3), accel_bias_err(3), gyro_bias_err(3)] */
    Eigen::Matrix<float, 15, 1> x_ = Eigen::Matrix<float, 15, 1>::Zero();

    /* 距上次 GNSS 成功融合的累计时间，单位 s。
       用于触发失联重捕获逻辑（> GNSS_REACQUIRE_MIN_OUTAGE_S 时激活低增益融合窗口）。 */
    float time_since_gnss_update_s_ = 0.0f;

    /* GNSS 重捕获保守融合窗口的剩余计时，单位 s。
       一次成功重捕获后设置窗口宽度，窗口内继续使用膨胀 R 防止后续帧再被普通门限拒绝。 */
    float gnss_reacquire_timer_s_ = 0.0f;

    /* 最近一次通过所有校验的成功导航周期，单位 s。
       用于 ResetHistoryBuffer() 为首快照填充正确的 dt，避免硬编码周期值。 */
    float last_valid_dt_s_ = 0.005f;
    double gravity_mps2_ = 9.80665; // WGS-84 正常重力缓存（Somigliana）

    // ==================================================================
    // 运行期状态变量
    // ==================================================================

    /* 机体系→NED 系方向余弦矩阵的转置 (= DCM 的转置 = NED→机体系旋转矩阵)。
       在 Initialize() 和每次姿态更新后同步刷新，传播中直接复用避免重复 quat2dcm()。 */
    Eigen::Matrix3f t_b2ned;

    /* EKF 估计的加速度计零偏 (m/s²)，持续被 ZUPT/GNSS 量测修正。
       传播时从原始加速度读数中扣除，使比力输入为"去偏后"的物理量。 */
    Eigen::Vector3f accel_bias_mps2_ = Eigen::Vector3f::Zero();

    /* EKF 估计的陀螺零偏 (rad/s)，持续被静态标定/StaticGyro/GNSS 修正。
       传播时从原始角速度读数中扣除，使角速度输入不含常值偏差。 */
    Eigen::Vector3f gyro_bias_radps_;

    /* 归一化加速度向量缓存（历史遗留，当前主传播路径不再使用）。 */
    Eigen::Vector3f accel_norm_mps2_;

    /* 归一化磁力计向量缓存（历史遗留，当前主传播路径不再使用）。 */
    Eigen::Vector3f mag_norm_mps2_;

    /* 姿态更新四元数增量，由 BuildDeltaQuatFromRotationVector() 计算。
       同时用于传播 (角增量→四元数乘法) 和误差状态注入 (姿态修正量→四元数)。 */
    Eigen::Quaternionf delta_quat_;

    /* 当前 EKF 估计的四元数姿态 (FRD 机体系相对 NED 导航系)。
       每次传播和量测修正后保持归一化。w<0 时整体取反以保持唯一表示。 */
    Eigen::Quaternionf quat_;

    /* 当前帧 IMU 去偏后的加速度 (机体系 FRD, m/s²)，
       同时用于 INS 传播和 VOFA/诊断输出。 */
    Eigen::Vector3f ins_accel_mps2_;

    /* 当前帧 IMU 去偏后的角速度 (机体系 FRD, rad/s) */
    Eigen::Vector3f ins_gyro_radps_;

    /* 上一导航步的角增量 (rad)，用于流式圆锥补偿 (sculling)。
       双子样路径取 delta_theta_2，单子样路径取总 delta_theta。 */
    Eigen::Vector3f previous_delta_theta_rad_ = Eigen::Vector3f::Zero();

    /* 上一导航步的速度增量 (m/s)，用于流式划摇补偿 (sculling)。 */
    Eigen::Vector3f previous_delta_v_mps_ = Eigen::Vector3f::Zero();

    /* 当前 EKF 欧拉角 [Yaw(0), Pitch(1), Roll(2)]，单位 rad。
       由四元数惰性求值 (EnsureEulerAnglesUpToDate) 按需更新，避免每帧做 quat2angle()。 */
    Eigen::Vector3f ins_ypr_rad_;

    /* 欧拉角脏标志：true 表示四元数已更新但欧拉角尚未重新计算。
       由 MarkEulerAnglesDirty() / MarkEulerAnglesClean() 管理。 */
    bool ypr_dirty_ = false;

    /* 当前 EKF NED 速度 (m/s)，[北向, 东向, 地向(向下为正)] */
    Eigen::Vector3f ins_ned_vel_mps_;

    /* 当前 EKF 大地坐标 LLA (纬度 rad, 经度 rad, WGS-84 椭球高 m)。
       使用 double 存储以保证绝对位置精度 (float32 在 ~113° 经度附近约 0.75 m 一跳)。 */
    Eigen::Vector3d ins_lla_rad_m_;
  };

} // namespace bfs

#endif // NAVIGATION_SRC_EKF_15_STATE_H_ NOLINT
