/**
 * @file fw_algo_api.h
 * @module fw-nav/EKF 宿主直调 C ABI
 * @brief 固件算法宿主直调接口（C ABI）：让仿真/调试程序直接驱动固件 Ekf15State。
 *
 * @details 背景：此前 sim/ 下的 Python 研究脚本为了做参数与场景扫描，重写了一份
 *          15 态 ESKF（sim/ekf_core.py，800+ 行）。两份实现各改各的，任何固件
 *          修复都要在 Python 里再复刻一次，结论还可能不一致。
 *          本接口把固件 lib/navigation-main/src/ekf_15_state.h 的真实实现编译成
 *          共享库，用稳定的 C ABI 暴露给 Python(ctypes)/MATLAB/宿主程序，
 *          使"仿真里跑的算法"就是"固件里烧的算法"。
 *
 * @par 设计约束
 *      - 只做薄封装：不改变任何滤波数学、门限或默认值，参数透传。
 *      - C ABI：只用固定宽度标量/数组，不暴露 Eigen/STL，便于跨语言加载。
 *      - 单线程、无全局状态：每个句柄持有独立 Ekf15State，可并行多实例。
 *      - 版本化：FW_ALGO_ABI_VERSION 变更才允许破坏兼容。
 *
 * @par 典型用法（Python）
 *      @code{.py}
 *      import fwapi
 *      ekf = fwapi.Ekf()
 *      ekf.initialize(accel, gyro, mag, vel_ned, lla, yaw_offset_rad=0.0)
 *      ekf.time_update_two_sample(dth1, dth2, dv1, dv2, dt)
 *      r = ekf.gnss_update(vel_ned, lla, dt_avg, age_s, require_snapshot=False)
 *      st = ekf.state()            # 导航状态 + 协方差对角 + 计数器
 *      @endcode
 *
 * @frame 输入/输出沿用固件约定：NED 导航系、FRD 机体系，四元数 (w,x,y,z)。
 * @units SI：m、m/s、m/s²、rad、rad/s、s；LLA 为 (纬度 rad, 经度 rad, 椭球高 m)。
 *
 * @note 本头文件只用于宿主侧（test_host/），不参与固件镜像编译。
 * @see  TandemVec_FCS/lib/navigation-main/src/ekf_15_state.h
 *       TandemVec-EKF-Report/sim/fwapi/
 */
#ifndef FW_ALGO_API_H
#define FW_ALGO_API_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32)
#define FW_API __declspec(dllexport)
#else
#define FW_API __attribute__((visibility("default")))
#endif

/** ABI 版本：结构体布局或语义发生不兼容变化时 +1。 */
#define FW_ALGO_ABI_VERSION 1

/** 协方差维度（15 态 ESKF）。 */
#define FW_EKF_STATE_DIM 15

/** 不透明 EKF 句柄。 */
typedef struct fw_ekf fw_ekf;

/**
 * @brief 初始化参数（一次性）。
 * @note mag 允许全 0（无磁初始化）；此时航向由 yaw_offset_rad 指定。
 */
typedef struct
{
  float accel_mps2[3];   /**< 机体系比力初值 [m/s²] */
  float gyro_radps[3];   /**< 机体系角速度初值 [rad/s] */
  float mag_ut[3];       /**< 机体系磁场初值 [µT]，无磁填 0 */
  float vel_ned_mps[3];  /**< NED 速度初值 [m/s] */
  double lla_rad_m[3];   /**< 初始位置（纬度 rad, 经度 rad, 椭球高 m） */
  float yaw_offset_rad;  /**< 初始航向偏置 [rad]（真北→磁北修正/手动对准角） */
} fw_ekf_init_t;

/**
 * @brief 单次量测更新结果（与固件 MeasurementUpdateResult 一一对应）。
 */
typedef struct
{
  int32_t input_valid;        /**< 输入有限且物理有效 */
  int32_t attempted;          /**< 进入 EKF 更新流程 */
  int32_t fused;              /**< 实际完成状态/协方差更新 */
  int32_t reacquired;         /**< 长时间失联后的自动重锚 */
  int32_t outlier_rejected;   /**< 大跳点被野值门拒绝 */
  int32_t subset_position;    /**< 位置子集更新融合 */
  int32_t subset_velocity;    /**< 速度子集更新融合 */
  float nis_pos3;             /**< 位置子集 NIS */
  float nis_vel3;             /**< 速度子集 NIS */
  float innovation_norm;      /**< 新息模长 */
  float test_ratio;           /**< NIS/门限 */
  float yaw_correction_rad;   /**< 磁更新期望航向修正（限幅前） */
  int32_t yaw_correction_clamped; /**< 航向修正是否被限幅 */
} fw_update_result;

/**
 * @brief 导航状态快照（只读）。
 * @note cov_diag 为 P 的对角线，顺序与固件误差状态一致：
 *       [0:3]=位置(m²) [3:6]=速度(m²/s²) [6:9]=失准角(rad²)
 *       [9:12]=加计零偏 [12:15]=陀螺零偏。
 */
typedef struct
{
  double lla_rad_m[3];
  float vel_ned_mps[3];
  float quat_wxyz[4];
  float euler_ypr_rad[3];      /**< (yaw, pitch, roll) */
  float accel_bias_mps2[3];
  float gyro_bias_radps[3];
  float yaw_sigma_rad;         /**< P(8,8) 的 1σ */
  float tilt_sigma_rad;        /**< P(6,6)+P(7,7) 的 1σ */
  float gravity_mps2;          /**< 初始化点 WGS-84 正常重力 */
  float cov_diag[FW_EKF_STATE_DIM];

  uint32_t gnss_reacquire_reset_count;   /**< 硬重锚次数 */
  uint32_t gnss_velocity_reanchor_count; /**< 速度重锚次数 */
  uint32_t gnss_outlier_reject_count;    /**< GNSS 野值拒绝累计 */
  uint32_t mag_fused_count;              /**< 磁矢量融合累计 */
  uint32_t mag_reject_count;             /**< 磁矢量拒绝累计 */
  uint32_t vel_subset_count;             /**< 速度子集软恢复累计 */

  float last_gnss_resid_pos_ne_m;   /**< 最近一次 GNSS 水平位置残差 */
  float last_gnss_resid_vel_ne_mps; /**< 最近一次 GNSS 水平速度残差 */
  int32_t last_gnss_gate_pos;       /**< 最近一次位置野值门是否触发 */
  int32_t last_gnss_gate_vel;       /**< 最近一次速度野值门是否触发 */
  float time_since_gnss_s;          /**< 距上次 GNSS 融合的时间 */
} fw_nav_state;

/* ---- 生命周期 ---- */

/** @brief ABI 版本（与头文件 FW_ALGO_ABI_VERSION 比对，防止加载到旧库）。 */
FW_API int32_t fw_algo_abi_version(void);

/** @brief 构建信息（编译器、标量 Eigen 开关、编译时间），用于证据可追溯。 */
FW_API const char *fw_algo_build_info(void);

/** @brief 创建 EKF 实例；失败返回 NULL。调用方负责 fw_ekf_destroy。 */
FW_API fw_ekf *fw_ekf_create(void);

/** @brief 销毁实例；传 NULL 安全。 */
FW_API void fw_ekf_destroy(fw_ekf *h);

/** @brief 初始化（对应 Ekf15State::Initialize）。返回 0 成功。 */
FW_API int32_t fw_ekf_init(fw_ekf *h, const fw_ekf_init_t *init);

/* ---- 传播 ---- */

/** @brief 单子样传播：accel/gyro 为已去零偏的机体系量（对应 TimeUpdate）。 */
FW_API void fw_ekf_time_update(fw_ekf *h, const float accel_mps2[3],
                               const float gyro_radps[3], float dt_s);

/**
 * @brief 双子样传播（对应 TimeUpdateTwoSample，零偏在内部按 half_dt 扣除）。
 * @param dth1,dth2 两个子样角增量 [rad]
 * @param dv1,dv2   两个子样速度增量 [m/s]（含重力，未去零偏）
 */
FW_API void fw_ekf_time_update_two_sample(fw_ekf *h,
                                          const float dth1[3], const float dth2[3],
                                          const float dv1[3], const float dv2[3],
                                          float dt_s);

/* ---- 量测更新 ---- */

/**
 * @brief GNSS 位置/速度 6 维更新（含回放、NIS 门控、子集回退、持久重锚）。
 * @param measurement_age_s 观测年龄 [s]，0 表示当前
 * @param require_delayed_snapshot 非 0 时强制命中历史快照（固件生产口径）
 */
FW_API fw_update_result fw_ekf_gnss_update(fw_ekf *h,
                                           const float vel_ned_mps[3],
                                           const double lla_rad_m[3],
                                           float dt_avg_s,
                                           float measurement_age_s,
                                           int32_t require_delayed_snapshot);

/** @brief 磁矢量更新（对应 MeasurementUpdateMagVectorDetailed）。 */
FW_API fw_update_result fw_ekf_mag_update(fw_ekf *h,
                                          const float mag_body_ut[3],
                                          const float mag_ned_ut[3],
                                          float noise_ut);

/** @brief 气压绝对高度更新（对应 MeasurementUpdateBaroAltitudeDetailed）。 */
FW_API fw_update_result fw_ekf_baro_update(fw_ekf *h, float altitude_m,
                                           float noise_std_m);

/** @brief 外部测速辅助（光流/视觉；对应 MeasurementUpdateAidingVelocityDetailed）。 */
FW_API fw_update_result fw_ekf_aiding_velocity_update(fw_ekf *h,
                                                      const float vel_ned_mps[3],
                                                      float vel_ne_std_mps,
                                                      float vel_d_std_mps);

/** @brief 加速度重力方向更新（约束 roll/pitch）。 */
FW_API fw_update_result fw_ekf_gravity_update(fw_ekf *h, const float accel_mps2[3],
                                              float noise_rad);

/** @brief 静止角速度更新（bias-only，只应由静止检测确认后调用）。 */
FW_API fw_update_result fw_ekf_static_gyro_update(fw_ekf *h,
                                                  const float gyro_radps[3],
                                                  float noise_radps);

/** @brief 航向标量更新。 */
FW_API fw_update_result fw_ekf_yaw_update(fw_ekf *h, float yaw_rad, float noise_rad);

/* ---- 复位/重锚 ---- */

/** @brief 位置/速度重置到 GNSS 基准（对应 ResetPositionVelocityToGnss）。 */
FW_API void fw_ekf_reset_position_velocity(fw_ekf *h, const float vel_ned_mps[3],
                                           const double lla_rad_m[3]);

/** @brief 仅速度重置到外部测速源（对应 ResetVelocityToExternal）。 */
FW_API void fw_ekf_reset_velocity(fw_ekf *h, const float vel_ned_mps[3],
                                  float vel_std_mps);

/* ---- 配置（透传到固件 setter，默认值与固件一致） ---- */

/**
 * @brief GNSS 门控配置。
 * @param consider_heading 非 0 时抑制 GNSS 修航向（P1-5 相位门）
 * @param reanchor_allowed 非 0 允许持久失配后的硬重锚
 * @param vertical_reanchor_allowed 非 0 允许垂直方向参与重锚
 */
FW_API void fw_ekf_set_gnss_gates(fw_ekf *h, int32_t consider_heading,
                                  int32_t reanchor_allowed,
                                  int32_t vertical_reanchor_allowed);

/**
 * @brief 速度子集软恢复配置。
 * @param enabled 非 0 启用
 * @param max_pos_resid_m 触发时允许的最大位置残差（默认 25 m）
 * @param max_fires_per_epoch 每完整更新历元最多触发次数（默认 1，255=不限）
 */
FW_API void fw_ekf_set_velocity_subset(fw_ekf *h, int32_t enabled,
                                       float max_pos_resid_m,
                                       int32_t max_fires_per_epoch);

/** @brief IMU 合理值门限（默认 120 m/s² / 40 rad/s，对应 12.2 g / 2292 °/s）。 */
FW_API void fw_ekf_set_imu_limits(fw_ekf *h, float accel_mps2, float gyro_radps);

/** @brief 加计过程噪声缩放（振动自适应用；默认 1.0）。 */
FW_API void fw_ekf_set_q_accel_scale(fw_ekf *h, float scale);

/** @brief GNSS 天线相对 IMU 的机体系杆臂 [m]（默认 0）。 */
FW_API void fw_ekf_set_lever_arm(fw_ekf *h, const float lever_body_m[3]);

/* ---- 读取 ---- */

/** @brief 拷贝当前导航状态；h 或 out 为空时不做任何事。 */
FW_API void fw_ekf_get_state(fw_ekf *h, fw_nav_state *out);

/**
 * @brief 诊断信息以 JSON 文本输出（单行，便于 JSONL 落盘）。
 * @param buf 输出缓冲；cap 为容量
 * @return 写入长度（不含结尾 0）；缓冲不足返回 -1 并写空串
 */
FW_API int32_t fw_ekf_diag_json(fw_ekf *h, char *buf, int32_t cap);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* FW_ALGO_API_H */
