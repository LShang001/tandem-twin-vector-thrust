/**
 * @file    fw_nav_api.h
 * @module  fw-nav/固件核心宿主直调 C ABI（导航步进）
 * @brief   让宿主直接把导航任务的**每拍输入**灌进固件 `handleNavigationSystem()`，
 *          读回 15 态 EKF 的导航状态；导航算法（传播/门控/重锚/航向门控）全部来自
 *          固件原码，宿主不再复刻。
 *
 * @details 边界（与固件内部一致，不是新抽象）：
 *          - **输入 = `navigation_task.cpp` 在 200 Hz tick 上消费的外部量**：
 *            聚合后的 IMU 均值/双子样增量、`backup_ahrs_*`（由 sensor_imu.cpp 的
 *            Madgwick 产生）、UBX 历元（本库内部经真实解析器入队）、气压高度、
 *            磁矢量、光流、解锁/油门标志。
 *          - **不包含** `sensor_imu.cpp` 的 2 kHz 软件低通与 Madgwick：那一层与
 *            硬件（ICM-42688/SPI、时间戳）耦合，不在共享核心里。要复现生产口径的
 *            滤波，请在调用方做（SIL 用 `sil::ImuLpfApply` + 真实 Madgwick，与
 *            固件同一参数）；本 ABI 的 `ahrs_quat_wxyz` 就是那一层的输出。
 *
 * @par 可扩展性约定（同 fw_gnc_api.h）
 *      1) 版本化结构：前两个字段固定 `struct_size`/`abi_version`，只能尾部追加。
 *      2) 能力查询：`fw_core_caps().nav_abi_version` 报告本子 ABI 版本。
 *      3) 时间由调用方推进：每次 step 把 `g_sil_time_us` 前移 `dt_s`（固件用
 *         `micros()` 做历元年龄/超时判定）。
 *
 * @par 设计约束
 *      - 单例：导航任务持有全局 EKF 与解析器，一个进程一个实例；`fw_nav_create()`
 *        幂等、不隔离状态。
 *      - `fw_nav_reset()` 清空宿主缓冲与计数器并重建 EKF；home/原点由第一个
 *        GNSS 历元建立（与生产一致）。
 *      - 本文件只用于宿主侧（test_host/），不参与固件镜像编译。
 *
 * @frame  NED 导航系 / FRD 机体系；四元数 FRD←NED，欧拉角 Z-Y'-X''、theta=−asin(R13)
 * @units  SI：m、m/s、m/s²、rad/s、deg（仅 `euler_rpy_deg`/`lla_deg` 用度）
 * @see     src/navigation_task.cpp、src/state_data.h、lib/ublox-main/src/ubx.h、
 *          test_host/fw_sil/sil_sensor_model.h（UBX 帧生成）、CTL-004
 */
#ifndef FW_NAV_API_H
#define FW_NAV_API_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#if defined(_WIN32)
#define FW_NAV_API __declspec(dllexport)
#else
#define FW_NAV_API __attribute__((visibility("default")))
#endif

/** 导航步进 ABI 版本：结构体布局或输入映射语义不兼容变化时 +1。 */
#define FW_NAV_ABI_VERSION 1
/** 每拍最多携带的双子样增量条数（固件 NAV_IMU_DELTA_BUFFER_SIZE=64，这里够 5 ms@2 kHz×4）。 */
#define FW_NAV_MAX_DELTA_SAMPLES 16

/** 不透明导航句柄（进程内单例语义）。 */
typedef struct fw_nav fw_nav;

/**
 * @brief 每拍输入（200 Hz）。
 * @note 可选量用 `*_valid` 门控；未接入的量把 valid 置 0，固件走"无该量测"分支。
 *       `delta_count=0` 时只用均值（固件走单子样传播）；否则 |
 *       mean·dt 应与 Σ(delta) 一致（调用方保证，见 @details）。
 */
typedef struct
{
  uint32_t struct_size;   /**< 入参：调用方结构大小 */
  uint32_t abi_version;   /**< 入参：必须 == FW_NAV_ABI_VERSION */
  float dt_s;             /**< 本拍时长 [s]（SIL/固件 0.005） */

  /* ---- IMU 聚合（导航增量，已滤波、已减安装矩阵，FRD 机体系）----
   * 用**和**而不是均值：固件内部 `acc_sum/count` 只有在"和"层面才逐位可复现
   * （先除后乘会引入末位差，逐拍累积成可观测的轨迹分叉）。`sample_count=1`
   * 时"和"即"本拍单样本值"。 */
  float accel_sum_body_mps2[3];   /**< 本拍 Σ比力 [m/s²]（count 个 2 kHz 样本之和） */
  float gyro_sum_body_radps[3];   /**< 本拍 Σ角速率 [rad/s] */
  int32_t sample_count;           /**< 聚合的 2 kHz 样本数（≥1） */
  int32_t delta_count;            /**< 双子样增量条数（0=只用均值，≤FW_NAV_MAX_DELTA_SAMPLES） */
  float delta_theta_rad[FW_NAV_MAX_DELTA_SAMPLES][3]; /**< 各子样 Σω·dt [rad] */
  float delta_vel_mps[FW_NAV_MAX_DELTA_SAMPLES][3];   /**< 各子样 Σa·dt [m/s] */
  float delta_dt_s[FW_NAV_MAX_DELTA_SAMPLES];         /**< 各子样时长 [s] */

  /* ---- AHRS（sensor_imu.cpp 的 Madgwick 输出，FRD←NED）---- */
  float ahrs_quat_wxyz[4];  /**< 四元数 w,x,y,z */
  int32_t ahrs_valid;

  /* ---- GNSS（本拍推入一个新历元；库内部生成 NAV-PVT/EOE 走真实解析器）----
   * 数值用 **UBX 线格式整数**（1e-7 deg / mm / mm·s⁻¹ / mm 精度）而不是 float：
   * 纬度 28° 的 float32 分辨率约 0.2 m、经度 112° 约 0.75 m，会在量测入口
   * 就损失定位精度（逐拍对拍表现为 ~0.2 m 的固定偏差）。 */
  int32_t gnss_valid;
  int32_t gnss_lat_1e7;     /**< 纬度 [1e-7 deg] */
  int32_t gnss_lon_1e7;     /**< 经度 [1e-7 deg] */
  int32_t gnss_alt_mm;      /**< 椭球高 [mm] */
  int32_t gnss_vel_ned_mmps[3];/**< NED 速度 [mm/s] */
  uint32_t gnss_h_acc_mm;   /**< 水平精度 [mm] */
  uint32_t gnss_v_acc_mm;   /**< 垂直精度 [mm/s] */
  uint32_t gnss_s_acc_mmps; /**< 速度精度 [mm/s] */
  uint32_t gnss_itow_ms;    /**< iTOW [ms]（去重用，需单调） */

  /* ---- 气压 / 磁 / 光流 ---- */
  float baro_alt_up_m;      /**< 气压高度（相对起飞点，向上为正）[m] */
  int32_t baro_valid;
  float mag_body_ut[3];     /**< 机体系磁场 [µT] */
  int32_t mag_valid;
  float flow_vx_mps;        /**< 光流机体系前向速度 [m/s] */
  float flow_vy_mps;        /**< 光流机体系右向速度 [m/s] */
  float flow_range_m;       /**< 测距高度 [m] */
  int32_t flow_quality;     /**< 0–100（0=失效） */
  int32_t range_quality;    /**< 0–100 */
  int32_t flow_valid;       /**< **事件**：本拍更新光流状态（新样本或显式失效）；
                             *   0 表示本拍不碰 flow_data（保持上次状态与时间戳） */

  /* ---- 飞行状态（导航用：ZUPT/静止陀螺/健康门控）---- */
  int32_t unlocked;         /**< 对应固件 g_is_unlocked */
  float throttle_pct;       /**< 油门百分比（静止陀螺在 ≥3% 时跳过融合） */
} fw_nav_input_t;

/** @brief 每拍输出：EKF 导航状态 + 门控/健康状态。 */
typedef struct
{
  uint32_t struct_size;   /**< 入参：调用方结构大小 */
  uint32_t abi_version;   /**< 出参：FW_NAV_ABI_VERSION */

  float quat_wxyz[4];     /**< EKF 姿态 FRD←NED */
  float euler_rpy_deg[3]; /**< roll/pitch/yaw [deg]（theta=−asin(R13)） */
  double lla_deg[3];      /**< 纬度/经度 [deg]、椭球高 [m]（double：float32 在 112° 只有 ~0.75 m 分辨率） */
  float pos_ned_m[3];     /**< 相对导航原点 NED 位置 [m] */
  float vel_ned_mps[3];   /**< NED 速度 [m/s] */
  float alt_up_m;         /**< 相对起飞点高度（向上为正）[m] */

  float accel_bias_mps2[3];/**< 加计零偏估计 [m/s²] */
  float gyro_bias_radps[3];/**< 陀螺零偏估计 [rad/s] */
  float yaw_sigma_deg;     /**< 航向 1σ [deg]（P(8,8)） */
  float tilt_sigma_deg;    /**< 水平姿态 1σ [deg] */

  int32_t initialized;     /**< 导航已初始化（原点已建立） */
  int32_t gnss_fix3d;      /**< 当前 UBX fix ≥ 3D */
  uint32_t gnss_epoch_count;      /**< 累计入队历元数 */
  uint32_t gnss_fix3d_count;      /**< 质量通过的 3D fix 数 */
  uint32_t gnss_quality_fail;     /**< 质量门限拒绝数 */
  uint32_t gnss_duplicate;        /**< 重复 iTOW 丢弃数 */
  float gnss_age_s;        /**< 最近历元的真实回放年龄 [s] */
  int32_t baro_fused;      /**< 最近一次气压更新是否融合 */
  float baro_innov_m;      /**< 最近一次气压新息 [m] */
  int32_t heading_aiding;  /**< 航向辅助可用（磁矢量新鲜） */
  int32_t pos_degraded;    /**< 位置保持已降级（航向不可观） */
  int32_t imu_health;      /**< InsImuHealth 状态（0=Ok/1=Warning/2=Failed） */
  uint32_t imu_health_flags;/**< InsImuHealth flags 位图 */
  uint32_t gnss_queued;    /**< 解析器队列里的完整历元数（未消费会累加） */
  uint32_t gnss_pvt;       /**< 解析到的 NAV-PVT 帧数 */
  uint32_t gnss_eoe;       /**< 解析到的 NAV-EOE 帧数 */
  uint32_t gnss_overflow;  /**< 解析器队列溢出数 */
  uint32_t last_epoch_tow_ms;/**< 最近一个原始历元的 iTOW [ms]（与注入序列对账用） */
  uint32_t steps;          /**< 已推进的拍数 */
  /* ---- 观测专用（尾部追加，abi_version 不变；未定义 EKF_NAV_TELEMETRY 时
   *      为 NaN）：EKF 协方差对角。逐拍对拍要复算卡尔曼增益 K=PHᵀ(HPHᵀ+R)⁻¹
   *      必须能看到 P；该宏只加只读访问器、不参与滤波数学（见 run_all.sh）。 */
  float pos_cov_m2[3];     /**< P(n,n)=位置方差 [m²]，序 NED */
  float vel_cov_m2[3];     /**< P(v,v)=速度方差 [m²/s²]，序 NED */
} fw_nav_output_t;

/** @brief 创建/取用单例导航句柄。`out` 非空。返回 0 成功。 */
FW_NAV_API int32_t fw_nav_create(fw_nav **out);
/** @brief 销毁句柄（清空单例指针；固件全局状态保留到下次 create）。 */
FW_NAV_API void fw_nav_destroy(fw_nav *handle);
/** @brief 清空宿主缓冲/计数器并重建 EKF 状态（原点由首个 GNSS 历元重建）。 */
FW_NAV_API int32_t fw_nav_reset(fw_nav *handle);
/**
 * @brief 推进一拍导航并读回状态。
 * @return 0 成功；1 空指针；2 ABI 版本不符；3 输入结构过小；4 输出结构过小；
 *         5 输入非有限或 dt≤0
 */
FW_NAV_API int32_t fw_nav_step(fw_nav *handle, const fw_nav_input_t *in,
                               fw_nav_output_t *out);

/**
 * @brief 控制层武装重定基（起飞点原点切换）——把固件解锁沿的记账暴露给宿主。
 * @details SIL 侧由 `flight_control.cpp` 的解锁沿执行同一组动作：
 *          `setNavigationOriginFromLlaRad(当前原点)` + 清零 `relative_*` /
 *          `INS_GNSS_Packet.location_*` / `estimated_height` / `fused_*`。这些是
 *          **导航层状态**，但触发时机在控制层（武装），所以不在 `fw_nav_step` 里。
 *          宿主若只重放 `fw_nav_step`，解锁后 `pos_ned_m`/`alt_up_m` 会与固件差
 *          一个**常量**（= 解锁前 EKF 相对首帧 GNSS 原点的漂移，随 seed 变化；
 *          CTL-004 §6.2 的 0.53 m 残差即由此主导）。调用方在检测到解锁边沿时
 *          用固件**实际使用的原点**调一次本函数，参考系即重新对齐。
 * @param origin_lla_deg 固件 `origin_lat_deg/lon_deg/alt_m`（度/度/米）。
 * @return 0 成功；1 空指针；5 参数非有限。
 * @note 不触碰 EKF 协方差/状态，也不改滤波算式；只改参考系与对外记账。
 * @warning 必须在**同一拍的 `fw_nav_step` 之前**调用，时序与 SIL 的
 *          "控制任务先于/同拍于导航任务"一致。
 */
FW_NAV_API int32_t fw_nav_rebase_origin(fw_nav *handle,
                                        const double origin_lla_deg[3]);
#ifdef __cplusplus
}
#endif

#endif  /* FW_NAV_API_H */
