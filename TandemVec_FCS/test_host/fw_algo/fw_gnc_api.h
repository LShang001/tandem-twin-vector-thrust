/**
 * @file    fw_gnc_api.h
 * @module  fw-control/固件核心宿主直调 C ABI（控制链 + 能力/参数）
 * @brief   让宿主仿真直接驱动固件 `runGNCExecutive()`（模式机+制导+姿态/速率环+分配+混控），
 *          并提供能力查询与参数注册表访问；控制律全部来自固件原码，不在仿真侧重写。
 *
 * @details 背景：此前 Python 侧（`TandemVec-EKF-Report/sim/hifi/hifi_core.py` 的
 *          `FirmwareControl`）自带一份"复刻固件"的姿态/速率/分配实现，任何固件修复
 *          （nav-raw、速率环量纲迁移、轴置换）都要在 Python 再写一遍。本 ABI 让仿真与
 *          固件共用同一份目标代码：输入=导航状态+遥控+目标，输出=舵机 PWM/分配诊断。
 *
 * @par 可扩展性约定（新增字段/新增消费者按这三条走）
 *      1) **版本化结构**：所有入/出结构前两个字段固定为 `struct_size` 与 `abi_version`；
 *         加字段只能追加在尾部，调用方填自己的 `struct_size`，库只写自己已知的长度。
 *      2) **能力查询**：`fw_core_caps()` 回报各子 ABI 版本、机型、参数数量、编译期特性位
 *         与 build id；实验脚本据此判"这条特性到底编进去了没有"，不靠猜。
 *      3) **参数走注册表**：`fw_param_*` 直接复用固件 `ano_params.cpp` 的 133 项注册表
 *         （与 GCS 0xE0–0xE3、MAVLink PARAM_SET 同一份），不在 ABI 里另立参数表。
 *
 * @par 设计约束
 *      - 薄封装：不改变任何控制律、限幅、默认参数。
 *      - 单例：固件控制层持有全局/函数静态状态（PID、滤波器），一个进程只能有一个实例；
 *        `fw_gnc_create()` 幂等，不隔离状态。
 *      - 角速度输入必须是**已去偏**的实测角速度：本库内 `nav_ekf` 不运行（零偏恒 0），
 *        调用方负责 `omega_meas − gyro_bias`，与控制链内部
 *        `current_omega_dps_body_filtered − nav_ekf.gyro_bias` 的总效果一致。
 *      - 时间由调用方推进：每次 step 把虚拟时钟 `g_sil_time_us` 前移 `dt_s`。
 *
 * @frame   NED 导航系 / FRD 机体系；四元数 FRD←NED，欧拉角 Z-Y'-X''、theta=−asin(R13)
 * @units  SI：m、m/s、m/s²、rad、rad/s、s；遥控 PWM 输入 µs（988–2012）；
 *          执行器输出为百分比（0–100）；摆角 deg
 * @note   本头文件只用于宿主侧（test_host/），不参与固件镜像编译。
 * @see     src/flight_control.cpp、src/state_data.h、src/ano_params.cpp、
 *          fw_algo_api.h、fw_ctrl_api.cpp
 */
#ifndef FW_GNC_API_H
#define FW_GNC_API_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#if defined(_WIN32)
#define FW_GNC_API __declspec(dllexport)
#else
#define FW_GNC_API __attribute__((visibility("default")))
#endif

/** 固件核心 ABI 版本（能力查询结构）。 */
#define FW_CORE_ABI_VERSION 1
/** 控制链步进 ABI 版本：结构体布局或输入映射语义不兼容变化时 +1。 */
#define FW_GNC_ABI_VERSION 1

/** 不透明控制链句柄（进程内单例语义）。 */
typedef struct fw_gnc fw_gnc;

/** 机型标识（编译期选定，见 `fw_core_caps_t.vehicle_profile`）。 */
enum
{
  FW_VEHICLE_TANDEMVEC = 0,
  FW_VEHICLE_COAXIAL_VTVL = 1,
  FW_VEHICLE_LIQUID_ROCKET = 2,
};

/** 编译期特性位（`fw_core_caps_t.feature_bits`）。 */
enum
{
  FW_FEAT_RAW_IMU_INCREMENT = 1u << 0,   /**< BFS_NAVIGATION_RAW_IMU_INCREMENT */
  FW_FEAT_GNSS_OUTAGE_VEL_HOLD = 1u << 1,/**< BFS_GNSS_OUTAGE_VELOCITY_HOLD */
  FW_FEAT_GNSS_OUTAGE_ALT_HOLD = 1u << 2,/**< BFS_GNSS_OUTAGE_ALT_HOLD */
  FW_FEAT_LONG_OUTAGE_DESCENT = 1u << 3, /**< BFS_LONG_GNSS_OUTAGE_DESCENT */
  FW_FEAT_BARO_HEALTH_GATE = 1u << 4,    /**< BFS_BARO_HEALTH_GATE */
  FW_FEAT_GNSS_VERTICAL_GATE = 1u << 5,  /**< BFS_GNSS_VERTICAL_REANCHOR_GATE */
  FW_FEAT_GNC_AVAILABLE = 1u << 6,       /**< 本库含控制链（未定义则该库只有 EKF/分配） */
};

/**
 * @brief 能力查询结果（只读）。
 * @note 调用方把 `struct_size` 填成自己的结构大小；库只写它已知的长度。
 */
typedef struct
{
  uint32_t struct_size;      /**< 调用方结构大小（入参） */
  uint32_t core_abi_version; /**< FW_CORE_ABI_VERSION */
  uint32_t ekf_abi_version;  /**< FW_ALGO_ABI_VERSION */
  uint32_t ctrl_abi_version; /**< 控制分配 ABI 版本（见 fw_ctrl_api.cpp） */
  uint32_t gnc_abi_version;  /**< FW_GNC_ABI_VERSION */
  uint32_t param_count;      /**< 参数注册表项数（= ANO_PARAMS_COUNT） */
  uint32_t vehicle_profile;  /**< FW_VEHICLE_* */
  uint32_t feature_bits;     /**< FW_FEAT_* 位或 */
  char build_id[64];         /**< 构建标识（-DFW_BUILD_ID，缺省 "dev"） */
} fw_core_caps_t;

/**
 * @brief 每拍输入。
 * @note 未使用的遥控通道请填 988（低档）或 1500（中位），不要填 0。
 */
typedef struct
{
  uint32_t struct_size;      /**< 入参：调用方结构大小 */
  uint32_t abi_version;      /**< 入参：必须 == FW_GNC_ABI_VERSION */
  float quat_wxyz[4];        /**< 姿态四元数 FRD←NED（w,x,y,z） */
  float omega_body_radps[3]; /**< 机体系角速度，**已去偏** [rad/s] */
  float vel_ned_mps[3];      /**< NED 速度 [m/s] */
  float pos_ned_m[3];        /**< 相对导航原点的 NED 位置 [m] */
  float alt_up_m;            /**< 相对起飞点高度（向上为正）[m] */
  uint16_t rc_pwm[16];       /**< 遥控通道原始 PWM [µs]，CH1..CH16 */
  int32_t link_up;           /**< 链路有效（对应固件 isLinkUp） */
  int32_t failsafe_in_flight;/**< 空中失控保护（对应 failsafe_in_flight） */
  float target_alt_m;        /**< 定高/定点目标高度（相对起飞点）[m] */
  float dt_s;                /**< 本拍步长 [s]，≤0 视为非法输入直接返回 */
} fw_gnc_input_t;

/** @brief 每拍输出（全部为固件本拍实际写出的量）。 */
typedef struct
{
  uint32_t struct_size;                  /**< 入参：调用方缓冲大小（≥ sizeof） */
  uint32_t abi_version;                  /**< 出参：FW_GNC_ABI_VERSION */
  float ch1_pct, ch2_pct, ch3_pct, ch4_pct; /**< 执行器输出 [%] 0–100（固件混控层原值，与遥测 0xF1 同源） */
  int32_t flight_mode;                   /**< 0=MANUAL 1=AUTO_POSITION 2=AUTO_ALTITUDE 3=GUIDED */
  int32_t unlocked;                      /**< 解锁位 */
  float tvc_upper_deg;                   /**< 上摆座角指令 [deg]（滚转） */
  float tvc_lower_deg;                   /**< 下摆座角指令 [deg]（俯仰） */
  float omega_ref_dps[3];                /**< 外环目标角速率 [deg/s] */
  float alpha_ref_dps2[3];               /**< 内环角加速度指令 [deg/s²] */
  float M_cmd[3];                        /**< 分配前力矩 [N·m] */
  float delta_f_deg, delta_t_deg, dw;    /**< 分配输出（摆角 deg / 差速归一化） */
  int32_t mode_degraded_heading;         /**< 航向门控降级（AUTO_POSITION→AUTO_ALTITUDE） */
  int32_t mode_degraded_gnss;            /**< GNSS 失联降级（同上，失联/光流失效触发） */
  int32_t failsafe_in_flight;            /**< 本拍失控保护状态 */
  uint32_t actuator_fault_bits;          /**< bit0 舵机卡死 / bit1 推力损失 / bit2 请求受控下降 */
} fw_gnc_output_t;

/** @brief 取能力描述；`out->struct_size` 由调用方预填。 */
FW_GNC_API void fw_core_caps(fw_core_caps_t *out);

/** @brief 创建/取回进程内唯一实例（幂等）。 */
FW_GNC_API fw_gnc *fw_gnc_create(void);

/** @brief 释放实例（单例；之后可再次 create）。 */
FW_GNC_API void fw_gnc_destroy(fw_gnc *handle);

/**
 * @brief 推进一拍固件控制链。
 * @param[in]  handle 句柄（必须非空）
 * @param[in]  in     本拍输入（`struct_size`/`abi_version` 必须匹配）
 * @param[out] out    本拍输出（`struct_size` 预填缓冲大小）
 * @return 0=成功；1=空指针；2=ABI 版本不匹配；3=输入结构过小；4=输出缓冲过小；
 *         5=数值非法/步长非正（此时不改状态）
 */
FW_GNC_API int32_t fw_gnc_step(fw_gnc *handle, const fw_gnc_input_t *in,
                               fw_gnc_output_t *out);

/** @brief 参数注册表项数（= ANO_PARAMS_COUNT）。 */
FW_GNC_API uint32_t fw_param_count(void);

/** @brief 参数 ID → 名称；越界返回 NULL。 */
FW_GNC_API const char *fw_param_name(uint32_t id);

/** @brief 名称 → 参数 ID；未找到返回 -1。支持 ≤15B 前缀（MAVLink 语义）。 */
FW_GNC_API int32_t fw_param_find(const char *name);

/** @brief 读参数（统一转 float）；成功返回 0。 */
FW_GNC_API int32_t fw_param_get(uint32_t id, float *out);

/** @brief 写参数（拒绝 NaN/Inf；写后 applyFlightCtrlParams 无扰同步）；成功返回 0。 */
FW_GNC_API int32_t fw_param_set(uint32_t id, float value);

#ifdef __cplusplus
}
#endif

#endif /* FW_GNC_API_H */
