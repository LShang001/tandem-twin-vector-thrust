/**
 * @file    fw_sil_env_main.cpp
 * @module  固件/宿主 SIL（fw-tests：真实固件 + 6DOF + 传感器/环境注入）
 * @brief   在真实固件控制链外挂 6DOF 动力学、传感器模型与风扰/故障注入
 * @details 被测：真实固件 `src/flight_control.cpp`（控制链）与
 *          `lib/navigation-main/src/ekf_15_state.h`（`--ekf` 时）。
 *          输入：命令行开关——`--manual` 手动阶跃、`--ekf` 用真实 EKF 估计、
 *          `--fault 3|4|5` 单发推力损失/上摆卡死/双发推力损失、`--fault-at T` 注入时刻（默认 8 s）、
 *          `--loss F` 推力保留比例（默认 0.70）、`--alt T` 定高目标（默认 10 m）、
 *          `--seconds T` 仿真时长（默认 40 s）、
 *          `--mask N` 覆盖 `inertia_comp_mask`（0=关前馈 / 3=全开，用于 C-02 A/B）、
 *          `--coupled` 同时 roll/pitch 激励 + 油门脉冲、`--flow` 光流测速、
 *          `--flow-delay N` 光流延迟、`--mag` 磁航向、`--seed`、`--no-wind`；
 *          `--real-nav` 下另有 `--mag-noise/--mag-disturb/--mag-dropout/
 *          --mag-cal-invalid`、`--imu-fault nan|stuck`/`--imu-dropout A:B`、
 *          `--gnss-consistency-reject`、`--trace-csv PATH`（200 Hz 控制级
 *          CSV：真值/EKF 姿态、陀螺、角速率指令、位置/目标、磁 kick/门控、
 *          模式与降级标志；仅测试设施）、`--trace-fast PATH`
 *          `--trace-fast-window A:B`（2 kHz 短窗 plant 角速率与 IMU 原始
 *          样本，用于区分高频模态/混叠；仅测试设施）、`--pos-leg N:E`
 *          （进入 AUTO_POSITION 时叠加前飞段目标偏移，仅 SIL）、
 *          `--pos-hold-at T` `--fwd-stick A:B`（定高前飞后再切定点）、
 *          `--takeoff-at T`（起飞时刻，默认 3 s；验证地面等待不计窗口）、
 *          `--initial-yaw-deg D`（plant 初始真值航向，验证手动对准）、
 *          `--static-calib-s S`（地面静止校准时长，默认 SIL 0=跳过）、
 *          `--ekf-q-scale S`（EKF 加速度过程噪声缩放，研究用）、
 *          `--calib-hold T`（前 T 秒电机不出力、固件油门 0，研究静置收敛）、
 *          `--wind-window A:B`（覆盖侧风时段）、`--bias-z-max R`
 *          （无磁离地陀螺零偏 z 限幅，0=关闭）；
 *          共轴 L1 保真度开关（仅 `-DFW_SIL_COAXIAL`）：
 *          `--coax-realistic`（预设）、`--coax-motor-tau S`、`--coax-servo-rate DPS`、
 *          `--coax-ground-mu MU`、`--coax-servojam roll|pitch` + `--coax-servojam-at T`、
 *          `--coax-motor-loss1|2 F`；失配/分配器激励（task-11）：
 *          `--coax-ix IX`、`--coax-inertia-scale F`（三轴惯量乘子）、
 *          `--coax-thrust-scale F`（推力效能）、`--coax-yaw-scale F`（差速偏航）、
 *          `--coax-alloc-mode 0|1`（0=legacy 1=α+分配器；固件字段落地后生效，
 *          缺字段只警告不崩）；`--rc-profile PATH` 灌入 CSV 遥控激励表
 *          （表头 `t,roll_us,pitch_us,yaw_us,throttle_us,mode_us,att_mode_us`，
 *          行间线性插值、超末行保持）；缺省/零值=改动前的既有行为（逐位一致），
 *          详见 README「共轴 plant 保真度开关」；
 *          处理：本文件提供 6DOF 刚体、电机/舵机动态、IMU/GNSS/气压计注入、
 *          20–25 s 侧风 3 m/s、地面约束与 0.1% PWM 量化；
 *          输出：每 1 s 打印高度/估计高度/位置/姿态/舵机/电机/GNSS 有效位
 *          与 F3/F4 标志；结束打印 max/rms 姿态、故障标志、final_alt。
 *          运行：BIN_DIR=<临时目录> bash test_host/fw_sil/build_and_run_env.sh <开关>
 * @theory  6DOF 刚体：`I·ω̇ = M − ω×Iω`、`v̇ = F/m + g`；
 *          推进力矩/推力来自 `computeWrench()`（kT/kQ 口径）。
 *          @ref THY-002 §2–3、@ref CTL-001 §4（控制链）
 * @frame   NED（位置/速度/风）+ FRD（角速率/比力）；`--ekf` 时 EKF 内部保持 WGS84 LLA
 * @units   高度 m，姿态 deg，舵机 PWM 百分比，电机转速 rad/s，风 m/s，时间 s
 * @note    与实机差异：传感器为注入模型 + 白噪声，无真实振动谱、无电机磁干扰、
 *          无 GNSS 多径；`--mag`/`mag_*` profile 才启用磁路径，基线默认关闭；
 *          `--flow` 是理想光流，不能当实机验证。40 s `--pos-hold` 的横滚/航向摆动
 *          在无磁基线也存在，属既有位置保持-偏航耦合边界，不是磁更新引入。
 * @test    S1 定高（直注/EKF）闭环；S2 --manual 阶跃；S3 --fault 3/4 故障降级；
 *          S4 --flow 光流辅助；S5 --seed 多 seed 稳定性（run_matrix.sh）；
 *          S6 --coupled + --mask 0/3 前馈 A/B（C-02）；S7 --fault 3 的 F3 标志（C-01）
 * @see     TandemVec_FCS/test_host/fw_sil/README.md、run_matrix.sh、run_acceptance.sh
 */
// ============================================================
// 真实源码：src/flight_control.cpp + src/state_data.cpp；
// 本文件只提供 plant、传感器注入与环境，不复制固件控制律。
// ============================================================
#include "state_data.h"
#include "flight_control.h"
#include "TandemVec_Propulsion.h"
#include "TandemVec_Config.h"
#include "Quat4f.h"
#include "QuaternionMath.h"
#include "ekf_15_state.h"
#include "transforms.h"
#include "mag_reference.h"
#include <Eigen/Dense>
#ifdef FW_SIL_REAL_NAV
#include "communication.h"
#include "navigation_task.h"
#include "sil_sensor_model.h"
#include "missile_plant.h"
#include "sil_imu_filter.h"
#include "ins_vibration_hf.h"
#include "sil_vibration_isolator.h"
#include "NavImuIncrementSelect.h"
#include "MadgwickAHRS.h"
#include "math_utils.h"
#endif

#include <cmath>
#include <cstdio>
#include <string>
#include <cstring>
#include <limits>
#include <random>
#include <array>
// type_traits/utility：--coax-alloc-mode 的字段探测模板（SFINAE）无条件需要，
// 不随 EKF_NAV_TELEMETRY 开关。
#include <type_traits>
#include <utility>
#ifdef EKF_NAV_TELEMETRY
#include <vector>
#include <algorithm>
#endif

// 200 Hz 控制级 CSV 追踪（仅 SIL 测试设施；默认关闭）。
static const char *g_trace_csv_path = nullptr;
static FILE *g_trace_csv_file = nullptr;
// 控制链对拍轨迹（仅 SIL）：逐拍记录喂给 runGNCExecutive 的输入与它写出的输出，
// 供 fwapi.Gnc 用同一序列重放比对（证明直调 ABI 与 SIL 控制前状态一致）。
static const char *g_trace_ctrl_path = nullptr;
static FILE *g_trace_ctrl_file = nullptr;
// 导航步进对拍轨迹（仅 SIL）：逐拍记录 handleNavigationSystem() **调用前**的
// 全部外部输入（聚合 IMU/AHRS/GNSS/气压/磁/光流/标志）与调用后的 EKF 状态，
// 供 fwapi.Nav 用同一序列重放（证明 fw_nav_* 的注入映射与 SIL 一致）。
static const char *g_trace_nav_path = nullptr;
static FILE *g_trace_nav_file = nullptr;
// 本拍是否向解析器推入了 GNSS 历元 / 磁样本 / 光流样本（都不是每拍发生）。
static bool g_navtick_gnss = false;
static float g_navtick_gnss_lla[3] = {0.0f, 0.0f, 0.0f};
static float g_navtick_gnss_vel[3] = {0.0f, 0.0f, 0.0f};
static float g_navtick_gnss_acc[3] = {0.0f, 0.0f, 0.0f};
static uint32_t g_navtick_gnss_itow = 0;
// SIL 实际喂进 UBX 解析器的**整数**量（lat/lon 1e-7 deg、alt mm、vel mm/s）与
// 造出它们的 double LLA。上面 3 个 float32 度列在 28°/112° 只有 0.1–0.4 m 的
// 分辨率：重放若用 float32 度反算整数，量测入口就不是 SIL 的那个数（CTL-004 §6.2）。
static int32_t g_navtick_gnss_pos_i[3] = {0, 0, 0};
static int32_t g_navtick_gnss_vel_i[3] = {0, 0, 0};
static uint32_t g_navtick_gnss_acc_i[3] = {0, 0, 0};
static double g_navtick_gnss_lla_d[3] = {0.0, 0.0, 0.0};
static bool g_navtick_mag = false;
static float g_navtick_mag_body[3] = {0.0f, 0.0f, 0.0f};
static bool g_navtick_flow = false;
static float g_navtick_flow_v[2] = {0.0f, 0.0f};
static float g_navtick_flow_range = 0.0f;
static int g_navtick_flow_q = 0;
static int g_navtick_range_q = 0;
// 2 kHz 短窗追踪：仅记录 [start,end) 内的 plant 角速率与 IMU 原始样本。
static const char *g_trace_fast_path = nullptr;
static FILE *g_trace_fast_file = nullptr;
static float g_trace_fast_start_s = 24.0f;
static float g_trace_fast_end_s = 26.0f;
// 6DOF 积分子步数：默认 1 保持旧行为；>1 用于诊断显式欧拉近 Nyquist 数值振荡。
static int g_plant_substeps = 1;
// 6DOF 导弹弹体模式：真实固件导航任务 + 导弹真值/传感器；不跑 TandemVec 控制。
static bool g_missile_mode = false;
// 导航在环：把 EKF 姿态/陀螺估计喂给弹体理想自驾，默认关（保持真值闭环）。
static bool g_missile_nav_loop = false;
/* --gnss-lever：写入 kFlightCtrlParams.gnss_lever_*，navigation_task 每拍下发给
   EKF 做天线杆臂补偿（默认 0 = 旧行为）。与 --gnss-lever-inject 一致时应抵消。 */
static float g_gnss_lever_param[3] = {0.0f, 0.0f, 0.0f};
/* 最近一次注入的 GNSS 量测（原点相对 NED）与实际杆臂偏移，供 200 Hz 追踪
   直接核对注入符号——不要用真值姿态反推。 */
static float g_sil_gnss_meas_ned[3] = {0.0f, 0.0f, 0.0f};
static float g_sil_gnss_lever_ned[3] = {0.0f, 0.0f, 0.0f};
static Quat4f g_ms_att_est{1.0f, 0.0f, 0.0f, 0.0f};
static float g_ms_rate_est[3] = {0.0f, 0.0f, 0.0f};
static std::string g_imu_grade = "icm";  // ideal/icm/low/adverse（导弹 SIL 研究档位）
// 导弹 plant 包线扫描（默认 5000 N/120 kg/10 s → 6.4 g/451 m/s）。
static float g_ms_thrust_n = 5000.0f;
static float g_ms_mass_kg = 120.0f;
static float g_ms_burn_s = 10.0f;
// 生产 ICM-42688 配 ±8 g（sensor_imu.cpp）；16 g 仅作量程敏感性研究。
static float g_imu_fs_g = 8.0f;
// §4.24ak 姿态注入设施：把 IMU 三轴整体旋转指定角度（等效轴失准），用于验证
// "姿态误差 × 比力" 对位置误差的定量关系；默认不设置（按 grade 的原值）。
static bool g_imu_misalign_set = false;
static float g_imu_misalign_deg = 0.0f;
// 振动起始时刻（SIL）：0=从 t=0 起振；>0 用于先干净对准再叠加振动。
static float g_vib_start_s = 0.0f;
// 弹载飞控减震座（SIL 研究）：0=未装；>0 为固有频率 Hz；阻尼比默认 0.15。
static float g_vib_iso_fn_hz = 0.0f;
static float g_vib_iso_zeta = 0.15f;
// 生产 sensor_imu.cpp 的软件低通（陀螺 α=0.15/加计 α=0.02/0.05）默认启用；
// --no-imu-lpf 仅用于 A/B 旧口径。
static bool g_imu_lpf_enable = true;
// 生产 AHRS（Madgwick，2 kHz，滤波后机体系→FLU）默认启用；
// --no-sil-ahrs 仅用于复现旧 SIL 的 backup_ahrs=0 口径。
static bool g_sil_madgwick_enable = true;
// 研究开关：导航增量绕过软件 LPF（AHRS/控制仍用滤波值），
// 用于评估“EKF 用未滤波增量”的收益；默认 false 与生产一致。
static bool g_nav_raw_imu = false;
// 研究设施：与 `--nav-raw-imu` 配套。IMU 故障窗及其后 5 s 内，导航增量退回
// 滤波值（= 默认生产口径），用于检验"冻结格 +67.5% 的代价来自未滤波台阶直接进
// 导航积分"这一机理假设（报告 §4.24h/§4.24m）。默认 false；单独给出时对
// 非 raw 臂完全惰性（增量本来就取滤波值）。
static bool g_nav_raw_fault_fallback = false;
// 锁存键选择：false=用注入故障窗（研究捷径）；true=用固件 IMU 健康 Failed 信号
// （= 固件化时真正可用的触发源，用它先测"会不会在高过载/振动格误触发而静默吃掉收益"）。
static bool g_nav_raw_fb_use_health = false;
// 机制判别用：把窗键回退的**起点**推迟这么多秒（终点仍是"注入结束 + 5 s"），
// 用于隔离"健康键优于窗键"是否只因为 Failed 确认延迟让切换点后移（报告 §4.24m）。
static float g_nav_raw_fb_start_delay_s = 0.0f;
// 反证用：把后持时长从 5 s 改成 H 秒（H 小 => 在 LPF 仍带冻结值时就切回，
// 预测台阶重新出现）。报告 §4.24m-W 组的反向检验。
static float g_nav_raw_fb_hold_s = 5.0f;
static uint32_t g_nav_raw_fb_frames = 0;  // 自证据：回退真实发生的帧数
static uint32_t g_nav_bias_frames = 0;    // 自证据：导航加速度偏置注入生效的帧数
// 起飞时刻（仅 SIL）：默认 3 s；用于验证地面等待不消耗无磁 20 s 窗口。
static float g_takeoff_at_s = 3.0f;
// 仅 SIL：初始真值航向偏置 [deg]，用于验证起飞前手动航向对准。
static float g_initial_yaw_deg = 0.0f;
// 仅 SIL：地面静置校准段（电机不出力、固件油门置 0）。
static float g_calib_hold_s = 0.0f;

#ifdef BFS_RATE_ACCEL_FB
extern void setRateAccelFeedbackGains(float rp_gain, float yaw_gain,
                                      float tau_s);
#endif
#ifdef BFS_RATE_PHASE_LEAD
extern void setRatePhaseLeadGains(float rp_gain, float yaw_gain, float tau_s);
#endif

static const TandemVecParams &P = kDefaultTandemVecParams;
static const ServoConfig &SC = kDefaultServoConfig;
static bfs::Ekf15State g_ekf;
static bool g_use_ekf = false;
static int g_flow_delay = 1;  // 10 Hz 样本数
static bool g_use_mag_yaw = false;
static float g_mag_yaw_jump_rad = 0.0f; // 测试用：单次磁航向修正上限
static float g_flow_scale = 0.95f; // 在线标定后的光流尺度
static const Eigen::Vector3d kHomeLla(28.2 * M_PI / 180.0,
                                      112.9 * M_PI / 180.0, 50.0);

#ifdef FW_SIL_COAXIAL
// 共轴 L1 plant 参数（CLI 可覆盖）；默认值见 CTL-003/VER-002。
static float g_coax_mass_kg = 0.84f;
static float g_coax_ix = 0.05f;
static float g_coax_iy = 0.05f;
static float g_coax_iz = 0.08f;
static float g_coax_max_thrust_n = 14.0f;
static float g_coax_lever_m = 0.15f;
static float g_coax_yaw_nm_per_norm = 0.0005f;
static float g_coax_motor_loss1 = 1.0f;
static float g_coax_motor_loss2 = 1.0f;

// —— 可切保真度开关（2026-10-30，task-5）——
// 语义约定：**未指定 = 沿用改动前的 L1 行为**（执行器一阶、舵机 300°/s
// 速率限幅与 0.1% PWM 量化本来就在 2 kHz 内环/控制帧里，属基线而非新增）。
// 这里用 -1 作“未指定”哨兵，显式 0 才表示“无限快 / 无摩擦”，从而保证
// 缺省路径的浮点运算与改动前逐位一致。
// 气动阻尼按用户指示不在本 L1 plant 范围内（VTVL 低速段），wind 仍是
// 直接等效加速度力（原语义不动）。
static float g_coax_motor_tau_s = -1.0f;    // <0 未指定→0.28 s；0=立即跟踪
static float g_coax_servo_rate_dps = -1.0f; // <0 未指定→300 °/s；0=无限速率
static float g_coax_ground_mu = 0.0f;       // 地面水平摩擦系数，0=无（默认）
static float g_coax_cli_loss1 = 1.0f;       // 前/尾电机长期损失保留比例
static float g_coax_cli_loss2 = 1.0f;
static int g_coax_servojam = 0;             // 0=无 1=roll(前摆) 2=pitch(尾摆)
static float g_coax_servojam_at_s = -1.0f;  // <0 → 复用 --fault-at

// —— 分配器/失配激励开关（2026-10-31，task-11）——
// 语义：默认值 = 改动前的 plant 数值（Ix 0.05、三轴惯量系数 1、推力 1、
// 差速偏航系数 1、alloc_mode 0=legacy），保证缺省路径逐位不变；
// ×1.0f 的乘法在 IEEE 下精确，不引入舍入差。以上都只改 SIL plant 的
// 力/力矩与惯量，不改固件控制律输出。
static float g_coax_inertia_scale = 1.0f;  // 三轴惯量统一乘子 [0.25,4]
static float g_coax_thrust_scale = 1.0f;   // 实际推力/指令推力 [0.5,1.5]
static float g_coax_yaw_scale = 1.0f;      // 差速偏航力矩乘子 [0.25,4]
static int g_coax_alloc_mode = 0;          // 0=legacy 1=α+分配器（写固件参数）

// —— --rc-profile：CSV 遥控激励表（task-11 追加硬需求）——
// 首行表头固定为 `t,roll_us,pitch_us,yaw_us,throttle_us,mode_us,att_mode_us`；
// t 为 SIL 秒、其余列微秒。50 Hz 遥控节拍按行间线性插值取值，t 超过末行则
// 保持末行；行数上限 4096（40 s 任务按 10 ms 采样也只需 4000 行）。表只改
// 摇杆与 CH7(mode)/CH9(att_mode) 的注入值，CH5 解锁与链路时序仍走原逻辑。
struct RcProfileRow
{
  double t;
  uint16_t roll, pitch, yaw, thr, mode, att;
};
static constexpr int kRcProfileMaxRows = 4096;
static RcProfileRow g_rc_profile[kRcProfileMaxRows];
static int g_rc_profile_rows = 0;

static bool rcProfileLoad(const char *path)
{
  std::FILE *f = std::fopen(path, "r");
  if (f == nullptr)
  {
    std::fprintf(stderr, "[sil] --rc-profile 无法打开文件 %s\n", path);
    return false;
  }
  char line[256] = {0};
  if (std::fgets(line, sizeof(line), f) == nullptr)
  {
    std::fprintf(stderr, "[sil] --rc-profile %s 为空（缺表头）\n", path);
    std::fclose(f);
    return false;
  }
  int n = static_cast<int>(std::strlen(line));
  while (n > 0 && (line[n - 1] == '\n' || line[n - 1] == '\r'))
  {
    line[--n] = '\0';
  }
  const char *expect =
      "t,roll_us,pitch_us,yaw_us,throttle_us,mode_us,att_mode_us";
  if (std::strcmp(line, expect) != 0)
  {
    std::fprintf(stderr, "[sil] --rc-profile 表头不符：期望 \"%s\"，实际 \"%s\"\n",
                 expect, line);
    std::fclose(f);
    return false;
  }
  double last_t = -1.0;
  while (std::fgets(line, sizeof(line), f) != nullptr)
  {
    n = static_cast<int>(std::strlen(line));
    while (n > 0 && (line[n - 1] == '\n' || line[n - 1] == '\r'))
    {
      line[--n] = '\0';
    }
    if (n == 0)
    {
      continue;  // 允许空行
    }
    if (g_rc_profile_rows >= kRcProfileMaxRows)
    {
      std::fprintf(stderr, "[sil] --rc-profile 数据行超过上限 %d\n",
                   kRcProfileMaxRows);
      std::fclose(f);
      return false;
    }
    RcProfileRow r{};
    int c[6] = {0, 0, 0, 0, 0, 0};
    if (std::sscanf(line, "%lf,%d,%d,%d,%d,%d,%d", &r.t, &c[0], &c[1],
                    &c[2], &c[3], &c[4], &c[5]) != 7)
    {
      std::fprintf(stderr, "[sil] --rc-profile 第 %d 行格式非法（需 7 列）：%s\n",
                   g_rc_profile_rows + 1, line);
      std::fclose(f);
      return false;
    }
    // t 非负、有限、单调不减；各 PWM 列范围按任务书校验（mode_us 1000–2000，
    // 其余 988–2012，含 CH9 att_mode_us）。
    if (!(r.t >= 0.0) || r.t > 1.0e6 || r.t < last_t)
    {
      std::fprintf(stderr,
                   "[sil] --rc-profile 第 %d 行 t=%g 非法（需 0≤t 且单调不减，"
                   "上一行 t=%g）\n", g_rc_profile_rows + 1, r.t, last_t);
      std::fclose(f);
      return false;
    }
    const int lim_lo = 988, lim_hi = 2012;
    for (int j = 0; j < 6; ++j)
    {
      const int lo = (j == 4) ? 1000 : lim_lo;   // mode_us 允许 1000–2000
      const int hi = (j == 4) ? 2000 : lim_hi;
      if (c[j] < lo || c[j] > hi)
      {
        std::fprintf(stderr,
                     "[sil] --rc-profile 第 %d 行第 %d 列=%d 越界 [%d,%d]\n",
                     g_rc_profile_rows + 1, j + 2, c[j], lo, hi);
        std::fclose(f);
        return false;
      }
    }
    r.roll = static_cast<uint16_t>(c[0]);
    r.pitch = static_cast<uint16_t>(c[1]);
    r.yaw = static_cast<uint16_t>(c[2]);
    r.thr = static_cast<uint16_t>(c[3]);
    r.mode = static_cast<uint16_t>(c[4]);
    r.att = static_cast<uint16_t>(c[5]);
    last_t = r.t;
    g_rc_profile[g_rc_profile_rows++] = r;
  }
  std::fclose(f);
  if (g_rc_profile_rows == 0)
  {
    std::fprintf(stderr, "[sil] --rc-profile %s 没有数据行\n", path);
    return false;
  }
  std::fprintf(stderr, "[sil] --rc-profile 载入 %d 行（t=%.3f…%.3f s）\n",
               g_rc_profile_rows, g_rc_profile[0].t,
               g_rc_profile[g_rc_profile_rows - 1].t);
  return true;
}

// 按 SIL 时间 t 插值取 PWM；cursor 为单调游标（稳定时间基准，只前进）。
static void rcProfileAt(double t, int &cursor,
                        uint16_t &roll, uint16_t &pitch, uint16_t &yaw,
                        uint16_t &thr, uint16_t &mode, uint16_t &att)
{
  if (cursor < 0) cursor = 0;
  if (cursor >= g_rc_profile_rows) cursor = g_rc_profile_rows - 1;
  while (cursor + 1 < g_rc_profile_rows &&
         g_rc_profile[cursor + 1].t <= t)
  {
    ++cursor;
  }
  const RcProfileRow &a = g_rc_profile[cursor];
  const RcProfileRow &b =
      g_rc_profile[(cursor + 1 < g_rc_profile_rows) ? cursor + 1 : cursor];
  const double span = b.t - a.t;
  double w = 0.0;
  if (t > a.t && span > 1.0e-12)
  {
    w = (t - a.t) / span;
    if (w > 1.0) w = 1.0;  // 超出末行 → 保持末行
  }
  const auto lerp = [w](uint16_t x, uint16_t y) {
    return static_cast<uint16_t>(
        std::lround(static_cast<double>(x) +
                    (static_cast<double>(y) - static_cast<double>(x)) * w));
  };
  roll = lerp(a.roll, b.roll);
  pitch = lerp(a.pitch, b.pitch);
  yaw = lerp(a.yaw, b.yaw);
  thr = lerp(a.thr, b.thr);
  mode = lerp(a.mode, b.mode);
  att = lerp(a.att, b.att);
}
#endif

struct Plant {
  Quat4f q;
  float omega[3];      // body rad/s
  float vel[3];        // NED m/s
  float pos[3];        // NED m
  float wind[3];       // NED m/s (currently force-equivalent)
  float acc_ned[3];    // last NED inertial acceleration
  float force_ned[3];  // last NED thrust/force (no gravity)
  float motor_wf, motor_wt;
  float servo_f, servo_t; // rad actual
  bool on_ground;

  Plant() : q(1,0,0,0), omega{0,0,0}, vel{0,0,0}, pos{0,0,0},
            wind{0,0,0}, acc_ned{0,0,0}, force_ned{0,0,0}, motor_wf(0), motor_wt(0),
            servo_f(0), servo_t(0), on_ground(true) {}

  void body_to_ned(const float v[3], float out[3]) const {
    const float w=q.w, x=q.x, y=q.y, z=q.z;
    out[0] = (1-2*(y*y+z*z))*v[0] + 2*(x*y-w*z)*v[1] + 2*(x*z+w*y)*v[2];
    out[1] = 2*(x*y+w*z)*v[0] + (1-2*(x*x+z*z))*v[1] + 2*(y*z-w*x)*v[2];
    out[2] = 2*(x*z-w*y)*v[0] + 2*(y*z+w*x)*v[1] + (1-2*(x*x+y*y))*v[2];
  }
  void ned_to_body(const float v[3], float out[3]) const {
    const float w=q.w, x=q.x, y=q.y, z=q.z;
    out[0] = (1-2*(y*y+z*z))*v[0] + 2*(x*y+w*z)*v[1] + 2*(x*z-w*y)*v[2];
    out[1] = 2*(x*y-w*z)*v[0] + (1-2*(x*x+z*z))*v[1] + 2*(y*z+w*x)*v[2];
    out[2] = 2*(x*z+w*y)*v[0] + 2*(y*z-w*x)*v[1] + (1-2*(x*x+y*y))*v[2];
  }
  void euler(float &roll, float &pitch, float &yaw) const {
    Quaternion qq{q.w,q.x,q.y,q.z};
    quaternionToEuler(qq, roll, pitch, yaw);
  }
  // ★ 本 plant 只有垂直地面约束：没有气动阻尼；地面水平摩擦仅在
  // --coax-ground-mu>0 时启用（默认关闭），缺省落地后仍可滑行/翻滚。
  // 水平速度只能被推力改变，落地后仍可滑行/翻滚。长失联 hold/pos 与
  // 落地后位移是**上界**，不是实机漂移预测（2026-10-26，§7.11）。
  void step(const SixDOFWrench &w, float dt) {
    // model -> body: F=(Fz', Fy', -Fx'), M=(-Mz', My', -Mx')
    const float Fb[3] = {w.Fz, w.Fy, -w.Fx};
    const float M[3] = {-w.Mz, w.My, -w.Mx};

#ifdef FW_SIL_COAXIAL
    // 共轴 L1 参数可由 CLI 覆盖（默认见 g_coax_*）；不是实机辨识值。
    // --coax-inertia-scale：三轴惯量统一乘子（默认 1.0f，×1.0 精确）。
    // --coax-ix 只改 Ix，与 --coax-iy 相互独立。
    const float Ixx = g_coax_ix * g_coax_inertia_scale;
    const float Iyy = g_coax_iy * g_coax_inertia_scale;
    const float Izz = g_coax_iz * g_coax_inertia_scale;
    const float mass = g_coax_mass_kg;
#else
    const float Ixx = P.Ix, Iyy = P.Iy, Izz = P.Iz;
    const float mass = P.m;
#endif
    const int n_sub = g_plant_substeps > 0 ? g_plant_substeps : 1;
    const float h = dt / static_cast<float>(n_sub);
    for (int ss = 0; ss < n_sub; ++ss)
    {
      const float p=omega[0], qq=omega[1], r=omega[2];
      omega[0] += (M[0] - (Izz-Iyy)*qq*r) / Ixx * h;
      omega[1] += (M[1] - (Ixx-Izz)*r*p) / Iyy * h;
      omega[2] += (M[2] - (Iyy-Ixx)*p*qq) / Izz * h;

      const float wx=omega[0], wy=omega[1], wz=omega[2];
      Quat4f dq{-0.5f*(q.x*wx+q.y*wy+q.z*wz),
                0.5f*(q.w*wx+q.y*wz-q.z*wy),
                0.5f*(q.w*wy-q.x*wz+q.z*wx),
                0.5f*(q.w*wz+q.x*wy-q.y*wx)};
      q.w+=dq.w*h; q.x+=dq.x*h; q.y+=dq.y*h; q.z+=dq.z*h;
      q=qNorm(q);

      float Fn[3];
      body_to_ned(Fb, Fn);
      const float ax=(Fn[0]+wind[0])/mass;
      const float ay=(Fn[1]+wind[1])/mass;
      const float az=(Fn[2]+wind[2])/mass + P.g;
      acc_ned[0]=ax; acc_ned[1]=ay; acc_ned[2]=az;
      force_ned[0]=Fn[0]+wind[0]; force_ned[1]=Fn[1]+wind[1]; force_ned[2]=Fn[2]+wind[2];
      vel[0]+=ax*h; vel[1]+=ay*h; vel[2]+=az*h;
      pos[0]+=vel[0]*h; pos[1]+=vel[1]*h; pos[2]+=vel[2]*h;

      if (pos[2] >= 0.0f) { // NED down: ground at 0
        pos[2]=0.0f;
        if (vel[2] > 0.0f) vel[2]=0.0f;
        on_ground = true;
#ifdef FW_SIL_COAXIAL
        // 地面水平库仑摩擦（--coax-ground-mu，默认 0 = 完全跳过 → 逐位不变）。
        // L1 假设：地面对机体的法向支撑力≈m·g，动摩擦加速度 μ·g，方向与
        // 触地水平速度相反；每子步按 Δv=μ·g·h 比例消减，越过零点直接置零
        // （不产生反冲/静摩擦黏滞）。只作用于已触地段，不动飞行段。
        if (g_coax_ground_mu > 0.0f)
        {
          const float dv = g_coax_ground_mu * P.g * h;
          const float vh = std::sqrt(vel[0]*vel[0] + vel[1]*vel[1]);
          if (vh > 0.0f)
          {
            const float k = (vh > dv) ? (vh - dv) / vh : 0.0f;
            vel[0] *= k;
            vel[1] *= k;
          }
        }
#endif
      } else {
        on_ground = false;
      }
    }
  }
};

#ifdef FW_SIL_COAXIAL
// 共轴双桨近似 wrench：两电机平均转速→总推力，差速→偏航力矩，
// 两轴舵机修正角→推力方向与滚转/俯仰力矩。L1 集总模型，无真实桨距/反扭。
static SixDOFWrench computeCoaxialWrench(const Plant &p)
{
  // 长期电机损失（--coax-motor-loss1/2）与故障注入的瞬时 loss 串联；
  // 缺省 1.0f 的乘法在 IEEE 下精确等于原值，不改变基线结果。
  const float wf = p.motor_wf * g_coax_motor_loss1 * g_coax_cli_loss1;
  const float wt = p.motor_wt * g_coax_motor_loss2 * g_coax_cli_loss2;
  const float avg = 0.5f * (wf + wt) / P.wMax;
  const float diff = (wf - wt) / P.wMax;
  // --coax-thrust-scale：推力效能失配（默认 1.0f，×1.0 精确不改基线），
  // 只缩放 plant 实际力/力矩，不动固件输出。
  const float thrust = std::fmax(0.0f, std::fmin(1.0f, avg)) *
                       g_coax_max_thrust_n * g_coax_thrust_scale;
  const float roll = p.servo_f;
  const float pitch = p.servo_t;
  SixDOFWrench w;
  w.Fx = thrust * std::cos(roll) * std::cos(pitch);
  w.Fy = thrust * std::sin(roll);
  w.Fz = thrust * std::sin(pitch);
  // --coax-yaw-scale：差速偏航力矩乘子（默认 1.0f，×1.0 精确不改基线）
  w.Mx = g_coax_yaw_nm_per_norm * g_coax_yaw_scale * diff;      // body Mz = -w.Mx（差速反向）
  w.My = g_coax_lever_m * thrust * std::sin(pitch); // body My
  w.Mz = -g_coax_lever_m * thrust * std::sin(roll); // body Mx = -w.Mz
  return w;
}

// —— 分配器模式接线（task-11）——
// 固件侧 CoaxialCtrlParams.alloc_mode（0=legacy 1=α+分配器）由 task-10 分批
// 落地；这里做编译期字段探测（SFINAE）：字段未出现时只打印警告、不阻止启动
// （--coax-alloc-mode 1 仍可照常跑），字段落地后自动接线、无需再改本文件。
template <typename T, typename = void>
struct CoaxParamsHasAllocMode : std::false_type {};
template <typename T>
struct CoaxParamsHasAllocMode<
    T, std::void_t<decltype(std::declval<T &>().alloc_mode)>>
    : std::true_type {};

template <typename CoaxT>
static void applyCoaxAllocModeTo(CoaxT &coax, int mode)
{
  if constexpr (CoaxParamsHasAllocMode<CoaxT>::value)
  {
    coax.alloc_mode = static_cast<uint8_t>(mode & 0x01);
    std::fprintf(stderr, "[sil] 固件参数 coax.alloc_mode=%d（0=legacy 1=α+分配器）\n",
                 mode);
  }
  else
  {
    std::fprintf(stderr, "[sil] ★ 警告：固件参数尚无 coax.alloc_mode 字段，"
                         "--coax-alloc-mode %d 本次未生效（字段落地后自动接线）\n",
                 mode);
  }
}
#endif

struct Sensors {
  float gnss_n, gnss_e, gnss_d;
  float gnss_vn, gnss_ve, gnss_vd;
  float baro_alt;
  float gyro[3];
  float accel[3];
  float mag_yaw;
  bool gnss_valid;
  float flow_vn, flow_ve;
  bool flow_valid;
  int flow_quality;
  std::array<float, 32> flow_hist_vn{};
  std::array<float, 32> flow_hist_ve{};
  Sensors() : gnss_n(0),gnss_e(0),gnss_d(0),gnss_vn(0),gnss_ve(0),gnss_vd(0),
              baro_alt(0), gyro{0,0,0}, accel{0,0,0}, mag_yaw(0),
              gnss_valid(true), flow_vn(0), flow_ve(0), flow_valid(false),
              flow_quality(100) {}
};

// yaw/att_mode 为 --rc-profile 追加的可选列：缺省 yaw=1500、att_mode=1000，
// 与改动前写死的值完全相同（不传参的旧调用点行为逐位不变）。
static void set_rc(uint16_t throttle, uint16_t mode, bool unlock,
                   uint16_t roll = 1500, uint16_t pitch = 1500,
                   uint16_t yaw = 1500, uint16_t att_mode = 1000) {
  for (int i=0;i<9;++i) raw_rc_values[i]=1500;
  raw_rc_values[0]=roll; raw_rc_values[1]=pitch;
  raw_rc_values[2]=throttle; raw_rc_values[3]=yaw;
  raw_rc_values[4]=unlock?1800:1000;
  raw_rc_values[5]=1800;
  raw_rc_values[6]=mode; raw_rc_values[7]=1000; raw_rc_values[8]=att_mode;
}

static void update_sensors(Plant &p, Sensors &s, std::mt19937 &rng, float t) {
  std::normal_distribution<float> gn(0.0f, 1.0f);
  // IMU: gyro + bias + noise; accel specific force in body
  const float gyro_bias[3] = {0.003f, -0.002f, 0.004f};
  for (int i=0;i<3;++i) s.gyro[i] = p.omega[i] + gyro_bias[i] + 0.0015f*gn(rng);
  const float accel_bias[3] = {0.05f, -0.03f, 0.08f};
  // 比力：地面静止时地面反力支撑，f_ned=(0,0,-g)；
  // 飞行时 f_ned=推力/质量（不含重力）。
  float f_ned[3];
  if (p.on_ground) {
    f_ned[0]=0; f_ned[1]=0; f_ned[2]=-P.g;
  } else {
    f_ned[0]=p.force_ned[0]/P.m;
    f_ned[1]=p.force_ned[1]/P.m;
    f_ned[2]=p.force_ned[2]/P.m;
  }
  float a_body[3];
  p.ned_to_body(f_ned, a_body);
  for (int i=0;i<3;++i) s.accel[i] = a_body[i] + accel_bias[i] + 0.05f*gn(rng);

  // GNSS 10 Hz, noise + 20-25s outage
  const bool outage = (t > 20.0f && t < 25.0f);
  s.gnss_valid = !outage;
  static int last_gnss_tick = -1;
  const int gnss_tick = int(t * 10.0f + 0.5f);
  if (!outage && gnss_tick != last_gnss_tick) {
    last_gnss_tick = gnss_tick;
    s.gnss_n = p.pos[0] + 0.8f*gn(rng);
    s.gnss_e = p.pos[1] + 0.8f*gn(rng);
    s.gnss_d = p.pos[2] + 1.2f*gn(rng);
    s.gnss_vn = p.vel[0] + 0.1f*gn(rng);
    s.gnss_ve = p.vel[1] + 0.1f*gn(rng);
    s.gnss_vd = p.vel[2] + 0.15f*gn(rng);
  }
  // Baro 25 Hz, noise + slow drift
  static int last_baro_tick = -1;
  const int baro_tick = int(t * 25.0f + 0.5f);
  if (baro_tick != last_baro_tick) {
    last_baro_tick = baro_tick;
    s.baro_alt = -p.pos[2] + 0.25f*gn(rng) + 0.02f*t;
  }
  // Mag yaw (truth + noise)
  float r,pitch,yaw; p.euler(r,pitch,yaw);
  s.mag_yaw = yaw + 0.005f*gn(rng);
  // 光流质量门控：大姿态角时地面纹理/尺度失效，质量降 0
  s.flow_quality = (fabsf(r) < 0.17f && fabsf(pitch) < 0.17f) ? 100 : 0;

  // 光流水平速度（真值+噪声，10 Hz；连续可用；无高度/垂直）
  static int last_flow_tick = -1;
  const int ft = int(t * 10.0f + 0.5f);
  if (ft != last_flow_tick) {
    last_flow_tick = ft;
    // 机体水平速度（真实光流是机体系，含 0.95 尺度误差），
    // 在采样时刻用 EKF 姿态转 NED 后存入延迟队列。这样延迟补偿的是
    // “测量时刻的 NED 速度”，避免用当前姿态旋转旧机体测量。
    float vb[3];
    p.ned_to_body(p.vel, vb);
    // GNSS 有效时在线估计光流尺度（真实固件也有双矢量测速/尺度标定）
    if (s.gnss_valid && !outage) {
      const float sf = std::hypot(vb[0], vb[1]);
      const float sg = std::hypot(s.gnss_vn, s.gnss_ve);
      if (sf > 1.0f && sg > 1.0f) {
        const float ratio = std::max(0.7f, std::min(1.3f, sg / sf));
        g_flow_scale = 0.98f * g_flow_scale + 0.02f * ratio;
      }
    }
    const float fvx = g_flow_scale * vb[0] + 0.12f * gn(rng);
    const float fvy = g_flow_scale * vb[1] + 0.12f * gn(rng);
    const Eigen::Vector3f vn = g_ekf.quat().toRotationMatrix() *
                               Eigen::Vector3f(fvx, fvy, 0.0f);
    const int hi = ft % 32;
    s.flow_hist_vn[hi] = vn(0);
    s.flow_hist_ve[hi] = vn(1);
    // 10 Hz 链路 0.1 s 延迟（1 个样本）
    const int di = (hi - g_flow_delay + 32) % 32;
    s.flow_vn = s.flow_hist_vn[di];
    s.flow_ve = s.flow_hist_ve[di];
    s.flow_valid = (ft > 0);
  }
}

static void feed_firmware(const Plant &p, const Sensors &s) {
  if (!g_use_ekf) {
    // 直注模式：AHRS/导航量由传感器直接注入（没有 EKF）
    Quaternion qq{p.q.w,p.q.x,p.q.y,p.q.z};
    float r,pitch,yaw; quaternionToEuler(qq,r,pitch,yaw);
    AHRS_Packet.Roll=r; AHRS_Packet.Pitch=pitch; AHRS_Packet.Heading=s.mag_yaw;
    AHRS_Packet.Qw=p.q.w; AHRS_Packet.Qx=p.q.x;
    AHRS_Packet.Qy=p.q.y; AHRS_Packet.Qz=p.q.z;
    icm_gyro_x=s.gyro[0]; icm_gyro_y=s.gyro[1]; icm_gyro_z=s.gyro[2];
    current_omega_dps_body_filtered=Vector3(s.gyro[0]*57.29578f,
                                             s.gyro[1]*57.29578f,
                                             s.gyro[2]*57.29578f);
    relative_north=s.gnss_n; relative_east=s.gnss_e;
    relative_down=-s.baro_alt;
    estimated_height=s.baro_alt;
    fused_north_pos=s.gnss_n; fused_east_pos=s.gnss_e;
    fused_north_vel=s.gnss_vn; fused_east_vel=s.gnss_ve;
    INS_GNSS_Packet.location_north=s.gnss_n;
    INS_GNSS_Packet.location_east=s.gnss_e;
    INS_GNSS_Packet.location_down=s.gnss_d;
    INS_GNSS_Packet.velocity_north=s.gnss_vn;
    INS_GNSS_Packet.velocity_east=s.gnss_ve;
    INS_GNSS_Packet.velocity_down=s.gnss_vd;
    return;
  }
  // 真实 EKF 输出（姿态、位置、速度）
  const Eigen::Vector3f v = g_ekf.ned_vel_mps();
  const Eigen::Vector3d lla(g_ekf.lat_rad(), g_ekf.lon_rad(), g_ekf.alt_m());
  const Eigen::Vector3d rel = bfs::lla2ned(lla, kHomeLla);
  const Eigen::Quaternionf q = g_ekf.quat();
  const float roll = g_ekf.roll_rad(), pitch = g_ekf.pitch_rad(), yaw = g_ekf.yaw_rad();

  AHRS_Packet.Roll = roll;
  AHRS_Packet.Pitch = pitch;
  AHRS_Packet.Heading = yaw;
  AHRS_Packet.Qw = q.w(); AHRS_Packet.Qx = q.x();
  AHRS_Packet.Qy = q.y(); AHRS_Packet.Qz = q.z();

  // 速率环用原始陀螺（与固件一致），姿态用 EKF
  icm_gyro_x = s.gyro[0];
  icm_gyro_y = s.gyro[1];
  icm_gyro_z = s.gyro[2];
  current_omega_dps_body_filtered = Vector3(s.gyro[0] * 57.29578f,
                                            s.gyro[1] * 57.29578f,
                                            s.gyro[2] * 57.29578f);

  // EKF 姿态 + 传感器高度/位置（诊断用混合：先验证高度链，不是最终 EKF 全闭环）
  relative_north = s.gnss_n;
  relative_east = s.gnss_e;
  relative_down = -s.baro_alt;
  estimated_height = s.baro_alt;
  fused_north_pos = rel(0);
  fused_east_pos = rel(1);
  fused_north_vel = v(0);
  fused_east_vel = v(1);

  INS_GNSS_Packet.location_north = rel(0);
  INS_GNSS_Packet.location_east = rel(1);
  INS_GNSS_Packet.location_down = rel(2);
  INS_GNSS_Packet.velocity_north = v(0);
  INS_GNSS_Packet.velocity_east = v(1);
  INS_GNSS_Packet.velocity_down = v(2);
}

#ifdef FW_SIL_REAL_NAV
// ============================================================
// 真实导航任务 SIL（--real-nav）：2 kHz IMU → 真实 handleNavigationSystem
// ============================================================
struct RealNavConfig
{
  bool enabled = false;
  int profile = 1;  // 0 clean / 1 nominal / 2 degraded
  float vib_g = 1.0f;       // 全油门振动幅值 [g]，按 (rpm/wMax)^2 缩放
  float vib_hz = 80.0f;
  int vib_mode = 0;
  bool anti_alias = true;
  float gnss_latency_s = 0.020f;
  float gnss_rate_hz = 10.0f;
  float gnss_pos_ne_std = 0.8f;
  float gnss_pos_d_std = 1.2f;
  float gnss_vel_ne_std = 0.10f;
  float gnss_vel_d_std = 0.15f;
  // 有色 GNSS 误差开关：真实多径/电离层误差在时间上相关，EKF 的 R 按白噪声
  // 建模；关闭后只剩白噪声，用于同二进制隔离“有色误差导致协方差过度自信”。
  bool gnss_colored = true;
  // 仅缩放 UBX 自报精度（h_acc/v_acc/s_acc），不改变实际噪声；用于测试
  // EKF 的 R 是否应覆盖有色误差（1.0=当前固件口径）。
  float gnss_r_scale = 1.0f;
  /* GNSS 天线杆臂【注入】（机体系 [m]，天线相对 IMU 原点）。非 0 时 UBX 报告
     的是天线相位中心的位置/速度：p+=C_b^n r、v+=C_b^n(ω×r)。
     固件侧的补偿参数是 kFlightCtrlParams.gnss_lever_*（--gnss-lever）；
     两者一致时应完全抵消，不一致/为 0 时留下未建模米级与米/秒级偏置。 */
  float gnss_lever_inject[3] = {0.0f, 0.0f, 0.0f};
  // 研究设施（仅 SIL）：导航专用加速度偏置台阶。只加在喂给导航的 IMU 样本上，
  // 不改弹体真值、不改控制输入 ⇒ 用来直接检验"速度误差越过物理门后是否必然锁死"
  // （报告 §9.4 #3 缺的那一次直接注入）。窗口 [start,end)，单位 m/s²，机体系。
  float nav_accel_bias_mps2[3] = {0.0f, 0.0f, 0.0f};
  float nav_accel_bias_start_s = -1.0f;
  float nav_accel_bias_end_s = -1.0f;
  float outage_start = 20.0f;
  float outage_end = 25.0f;
  float outlier_prob = 0.0f;
  bool gnss_spoof = false;
  float spoof_offset_m = 20.0f;
  float spoof_ramp_mps = 0.0f;
  // 垂直 spoof：正值表示“报得更高”（pd 减小），与水平 spoof 共用 start。
  float spoof_offset_z_m = 0.0f;
  float spoof_ramp_z_mps = 0.0f;
  float spoof_start_s = 15.0f;
  float baro_noise = 0.25f;
  float baro_drift = 0.02f;
  bool baro_lag = true;
  float wind_force_mps2 = 1.0f;  // 侧风等效加速度（默认 20–25 s）
  float wind_start_s = 20.0f;
  float wind_end_s = 25.0f;
  bool flow = false;
  int flow_delay = 1;
  float flow_dropout_start = 1.0e9f;
  float flow_dropout_end = 1.0e9f;
  bool pos_hold = false;
  float pos_hold_at = 5.0f;   // SIL 测试用：延迟切 AUTO_POSITION [s]
  float fwd_stick_start = 1.0e9f; // SIL 测试用：前飞俯仰杆窗口
  float fwd_stick_end = 1.0e9f;
  bool rc_loss = false;
  float rc_loss_start = 15.0f;
  float rc_loss_end = 17.0f;
  float imu_drop_prob = 0.0f;
  int imu_fault_mode = 0;  // 0 none / 1 stuck / 2 NaN
  float imu_fault_start = 20.0f;
  float imu_fault_end = 21.0f;
  float imu_dropout_start = 1.0e9f;
  float imu_dropout_end = 1.0e9f;
  bool mag = false;
  bool gnss_consistency_reject = false;
  float mag_noise_ut = 0.5f;
  float mag_disturb_ut = 0.0f; // NED 正弦干扰幅值；0=干净
  float mag_dropout_start = 1.0e9f;
  float mag_dropout_end = 1.0e9f;
  bool mag_cal_invalid = false;
};

static void applyRealNavProfile(RealNavConfig &c, const char *name)
{
  if (std::strcmp(name, "clean") == 0)
  {
    c.profile = 0;
    c.vib_g = 0.0f;
    c.gnss_latency_s = 0.015f;
    c.gnss_pos_ne_std = 0.5f;
    c.gnss_pos_d_std = 0.8f;
    c.gnss_vel_ne_std = 0.05f;
    c.gnss_vel_d_std = 0.08f;
    c.outage_start = 1.0e9f;
    c.outage_end = 1.0e9f;
    c.outlier_prob = 0.0f;
    c.baro_noise = 0.10f;
    c.baro_drift = 0.0f;
    c.baro_lag = false;
    c.wind_force_mps2 = 0.0f;
    c.flow = false;
    c.rc_loss = false;
    c.imu_drop_prob = 0.0f;
  }
  else if (std::strcmp(name, "nomag_degraded") == 0)
  {
    // 无磁观测性降级场景：环境用 clean，但不注入磁；由 HeadingAidingGate
    // 强制 AUTO_POSITION→AUTO_ALTITUDE。不要复用 nominal 的 1g 振动/风，
    // 否则把“降级后自由漂移”误当成控制发散。
    applyRealNavProfile(c, "clean");
    c.mag = false;
  }
  else if (std::strcmp(name, "degraded") == 0)
  {
    c.profile = 2;
    c.vib_g = 2.0f;
    c.vib_hz = 120.0f;
    c.gnss_latency_s = 0.050f;
    c.gnss_pos_ne_std = 2.5f;
    c.gnss_pos_d_std = 4.0f;
    c.gnss_vel_ne_std = 0.20f;
    c.gnss_vel_d_std = 0.30f;
    // 传感器/环境退化门：5 s 失联 + 1.5 m/s² 阵风；RC 链路故障单独用
    // --rc-loss 注入，不能把“命令 disarm”混进传感器退化验收。
    c.outage_start = 20.0f;
    c.outage_end = 25.0f;
    c.outlier_prob = 0.02f;
    c.baro_noise = 0.50f;
    c.baro_drift = 0.10f;
    c.baro_lag = true;
    c.wind_force_mps2 = 1.5f;
    c.flow = true;
    c.rc_loss = false;
    c.rc_loss_start = 20.0f;
    c.rc_loss_end = 22.0f;
    c.imu_drop_prob = 0.001f;
  }
  else if (std::strcmp(name, "mag_clean") == 0)
  {
    applyRealNavProfile(c, "clean");
    c.mag = true;
    c.mag_noise_ut = 0.3f;
    c.mag_disturb_ut = 0.0f;
  }
  else if (std::strcmp(name, "mag_nominal") == 0)
  {
    applyRealNavProfile(c, "nominal");
    c.mag = true;
    c.mag_noise_ut = 0.6f;
    c.mag_disturb_ut = 0.0f;
  }
  else if (std::strcmp(name, "mag_gate") == 0)
  {
    // 与 mag_nominal 同一传感器环境；验收只看“偏航振荡门控降级 + 有界”。
    applyRealNavProfile(c, "mag_nominal");
  }
  else if (std::strcmp(name, "mag_disturbed") == 0)
  {
    applyRealNavProfile(c, "nominal");
    c.mag = true;
    c.mag_noise_ut = 0.8f;
    c.mag_disturb_ut = 12.0f;
  }
  else  // nominal
  {
    c.profile = 1;
    c.vib_g = 1.0f;
    c.vib_hz = 80.0f;
    c.gnss_latency_s = 0.020f;
    c.gnss_pos_ne_std = 0.8f;
    c.gnss_pos_d_std = 1.2f;
    c.gnss_vel_ne_std = 0.10f;
    c.gnss_vel_d_std = 0.15f;
    c.outage_start = 20.0f;
    c.outage_end = 25.0f;
    c.outlier_prob = 0.0f;
    c.baro_noise = 0.25f;
    c.baro_drift = 0.02f;
    c.baro_lag = true;
    c.wind_force_mps2 = 1.0f;
    // 室外 5 s GNSS 失联 + 侧风时，没有速度外部参照就无法保持水平位置；
    // nominal 启用光流测速（有该传感器的机型），否则只能验收定高。
    c.flow = true;
    c.rc_loss = false;
    c.imu_drop_prob = 0.0f;
  }
}

struct TruthSample
{
  float pos[3];
  float vel[3];
  /* 杆臂注入需要【量测时刻】的姿态与体角速率：GNSS 报告的是天线相位中心，
     p_ant = p_imu + C_b^n r、v_ant = v_imu + C_b^n(ω×r)。必须用与 pos/vel
     同一时刻的 q/ω，否则链路延迟 + 高角速率会混进注入误差本身。 */
  float q[4];
  float omega[3];
};

static void accumulateImuSample(const sil::ImuSample &s, float dt_s)
{
  acc_sum_x += s.accel[0];
  acc_sum_y += s.accel[1];
  acc_sum_z += s.accel[2];
  gyro_sum_x += s.gyro[0];
  gyro_sum_y += s.gyro[1];
  gyro_sum_z += s.gyro[2];
  ++imu_sample_count;

  if (imu_delta_sample_count < NAV_IMU_DELTA_BUFFER_SIZE)
  {
    const int idx = imu_delta_sample_count;
    imu_delta_theta_x[idx] = s.gyro[0] * dt_s;
    imu_delta_theta_y[idx] = s.gyro[1] * dt_s;
    imu_delta_theta_z[idx] = s.gyro[2] * dt_s;
    imu_delta_v_x[idx] = s.accel[0] * dt_s;
    imu_delta_v_y[idx] = s.accel[1] * dt_s;
    imu_delta_v_z[idx] = s.accel[2] * dt_s;
    imu_delta_dt_s[idx] = dt_s;
    imu_delta_sample_count++;
    imu_delta_time_sum_s += dt_s;
  }
  else
  {
    imu_delta_overflow = true;
  }
}

static int runRealNavLoop(const RealNavConfig &cfg, int seed, float seconds,
                          float target_alt, bool manual, bool coupled, int fault,
                          float fault_at, float loss, int mask, bool wind_on)
{
  Plant p;
  sil::MissileParams ms_params;
  ms_params.thrust_n = g_ms_thrust_n;
  ms_params.mass_kg = g_ms_mass_kg;
  ms_params.burn_s = g_ms_burn_s;
  sil::MissilePlant missile(ms_params);
  if (g_missile_mode)
  {
    const auto &ms = missile.state();
    for (int a = 0; a < 3; ++a) {
      p.pos[a] = ms.pos[a]; p.vel[a] = ms.vel[a];
      p.omega[a] = ms.omega[a]; p.force_ned[a] = ms.force_ned[a];
    }
    p.q = ms.q;
    p.on_ground = false;
  }
  if (!g_missile_mode && g_initial_yaw_deg != 0.0f)
  {
    const float h = 0.5f * g_initial_yaw_deg * 0.017453292f;
    p.q = Quat4f(std::cos(h), 0.0f, 0.0f, std::sin(h));
  }
  std::mt19937 rng(seed);
  sil::ImuConfig imu_cfg;
  if (g_imu_grade == "ideal") {
    imu_cfg.gyro_noise_density_dps = 1e-5f;
    imu_cfg.accel_noise_density_ug = 1e-3f;
    for (int i = 0; i < 3; ++i) {
      imu_cfg.gyro_bias_dps[i] = 0.0f;
      imu_cfg.accel_bias_mg[i] = 0.0f;
    }
    imu_cfg.gyro_bias_instability_dps = 0.0f;
    imu_cfg.accel_bias_instability_mg = 0.0f;
    imu_cfg.gyro_scale = 0.0f;
    imu_cfg.accel_scale = 0.0f;
    imu_cfg.misalign = 0.0f;
    imu_cfg.gyro_vib_dps = 0.0f;
  } else if (g_imu_grade == "low") {
    for (int i = 0; i < 3; ++i) {
      imu_cfg.gyro_bias_dps[i] *= 2.0f;
      imu_cfg.accel_bias_mg[i] *= 2.0f;
    }
    imu_cfg.gyro_noise_density_dps *= 2.0f;
    imu_cfg.accel_noise_density_ug *= 2.0f;
    imu_cfg.gyro_scale *= 2.0f;
    imu_cfg.accel_scale *= 2.0f;
    imu_cfg.misalign *= 2.0f;
  } else if (g_imu_grade == "adverse") {
    for (int i = 0; i < 3; ++i) {
      imu_cfg.gyro_bias_dps[i] *= 5.0f;
      imu_cfg.accel_bias_mg[i] *= 5.0f;
    }
    imu_cfg.gyro_noise_density_dps *= 3.0f;
    imu_cfg.accel_noise_density_ug *= 3.0f;
    imu_cfg.gyro_scale *= 5.0f;
    imu_cfg.accel_scale *= 5.0f;
    imu_cfg.misalign *= 5.0f;
  }
  if (g_imu_misalign_set)
  {
    imu_cfg.misalign = g_imu_misalign_deg * static_cast<float>(M_PI) / 180.0f;
  }
  imu_cfg.full_scale_accel_g = g_imu_fs_g;
  sil::ImuModel imu(imu_cfg, static_cast<uint32_t>(seed) * 7919U + 11U);
  sil::ImuLpfState imu_lpf;
  // 生产 AHRS：sensor_imu.cpp 的 Madgwick（2 kHz，滤波后的机体系样本）。
  // 之前 SIL 未链接该库，backup_ahrs_* 恒为 0，等于把“重力把俯仰拉平”
  // 人为固定在 0°；这里接真实库，让 AHRS 量测误差由算法自己产生。
  Madgwick madgwick_sil;
  bool madgwick_sil_init = false;

  // 让固件自己完成 EKF 初始化/原点设置；不要沿用旧 SIL 的强制初始化。
  nav_system_initialized = false;
  is_origin_lla_set = false;
  nav_data_source = NavDataSource::INTERNAL;
  deta100_online = false;
  isLinkUp = true;
  failsafe_in_flight = false;
  g_sil_time_us = 0;
  const float dt_ctrl = 0.005f;
  const int sub_steps = 10;
  const float dt_imu = dt_ctrl / static_cast<float>(sub_steps);
  const int steps = static_cast<int>(seconds / dt_ctrl);
  const int kHist = 256;
  std::array<TruthSample, kHist> hist{};
  std::normal_distribution<float> n01(0.0f, 1.0f);

  Eigen::Vector3f gnss_corr = Eigen::Vector3f::Zero();
  float baro_state = 0.0f;
  bool baro_init = false;
  std::array<float, 32> flow_hist_vn{};
  std::array<float, 32> flow_hist_ve{};
  float servo_target_f = 0.0f, servo_target_t = 0.0f;
  // 舵机卡死锁存（每次 run 独立，避免跨场景串扰）
  bool coax_jam_latched = false;
  float coax_jam_rad = 0.0f;
  float motor_target_wf = 0.0f, motor_target_wt = 0.0f;

  bool alt_cmd = false;
  float max_roll = 0.0f, max_pitch = 0.0f;
  float max_mag_kick = 0.0f;
  float max_mag_kick_rp = 0.0f;
  bool imu_fault_detected_any = false;
  bool gnss_degrade_seen = false;
  // 落地时刻指标：把“飞行段行为”与落地后无摩擦滑行分开，避免把滑行
  // 位移当成飞行漂移（SIL plant 无地面水平摩擦，2026-10-26）。
  bool airborne_seen = false;
  bool landed = false;
  float land_t = -1.0f;
  float land_hold = -1.0f;
  float land_pos_err = -1.0f;
  float land_roll_deg = 0.0f;
  float max_roll_preland = 0.0f;
  // 接地垂直速度（NED 向下为正）与接地前最大下降速度：下降策略的
  // 触地硬度指标，不能只看落地时间/位移（2026-10-26 下降速率权衡）。
  float land_vz_down = -1.0f;
  float max_desc_vz_down = 0.0f;
  // 失联起始瞬间的真值高度：脚本用 --alt 设的是目标高度，爬升未完成时
  // 与实际失联高度不是一回事（2026-10-26 下降速率扫描发现的台架口径坑）。
  float alt_at_outage = -1.0f;
  bool gnss_outage_prev = false;
  double sum_roll2 = 0.0, sum_pitch2 = 0.0;
  double sum_pos2 = 0.0, max_pos = 0.0;
  double max_gnss_age = 0.0;  // 全程 GNSS 年龄峰值，用作失联注入的自证据
  int n_att = 0, n_pos = 0;
  double sum_nav_alt = 0.0;
  int n_nav_alt = 0;
  float hold_target[2] = {0.0f, 0.0f};
  bool hold_set = false;
  bool rc_lost_prev = false;
  float pending_imu_dt = 0.0f;  // 丢样时把时间并到下一个被接受的样本
  sil::ImuSample last_imu;
  bool have_last_imu = false;
  const float kImuNaN = std::numeric_limits<float>::quiet_NaN();
  double sum_hold2 = 0.0, max_hold = 0.0;
  double sum_hold_end2 = 0.0;
  double sum_nav_att2 = 0.0, sum_nav_vel2 = 0.0;
  double max_nav_att = 0.0;
  // 分轴姿态误差（deg）：四元数总误差无法区分“航向不可观”与“横滚/俯仰
  // 在过载段发散”，导弹 SIL 需要按轴归因（2026-10-27）。
  double sum_nav_roll2 = 0.0, sum_nav_pitch2 = 0.0, sum_nav_yaw2 = 0.0;
  double max_nav_roll = 0.0, max_nav_pitch = 0.0, max_nav_yaw = 0.0;
  int n_nav_att = 0;
  int n_hold = 0, n_hold_end = 0;
#ifdef EKF_NAV_TELEMETRY
  // 闭环协方差一致性观测（仅 SIL 测试设施）：以 plant 真值为参照计算
  // NEES = e^T P^-1 e / 3，判断滤波器的自报不确定度是否可信。
  std::vector<float> nees_pos_samples, nees_vel_samples;
  Eigen::Vector3d nees_origin_last = Eigen::Vector3d::Zero();
  Eigen::Vector3d nees_truth_origin = Eigen::Vector3d::Zero();
  bool nees_origin_valid = false;
  float nees_dbg_ep = 0.0f, nees_dbg_sig = 0.0f, nees_dbg_val = -1.0f;
#endif

  if (g_trace_csv_path != nullptr)
  {
    g_trace_csv_file = std::fopen(g_trace_csv_path, "w");
    if (g_trace_csv_file == nullptr)
    {
      std::fprintf(stderr, "[trace] cannot open %s\n", g_trace_csv_path);
    }
    else
    {
      std::fprintf(
          g_trace_csv_file,
          "t,truth_roll,truth_pitch,truth_yaw,ekf_roll,ekf_pitch,ekf_yaw,"
          "gyro_x,gyro_y,gyro_z,omega_ref_x,omega_ref_y,omega_ref_z,"
          "ekf_n,ekf_e,true_n,true_e,hold_n,hold_e,"
          "mag_kick,mag_kick_rp,mag_gate_reason,mode,reanchor,"
          "heading_aiding,pos_degraded,ch1,ch2,ch3,ch4,yaw_sigma,heading_mode,"
          "gyro_bias_z,mag_age,heading_age,"
          "truth_alt,ctl_alt,target_alt,truth_vz,ekf_vz,ekf_ba_z,vfk_vz,"
          "gbx_dps,gby_dps,abx_mps2,aby_mps2,baro_innov_m,baro_fused,"
          "ekf_vn,ekf_ve,ekf_tilt_sigma,gnss_resid_pos_ne,gnss_resid_vel_ne,"
          "gate_pos,gate_vel,vel_sub_rec,"
          "msr_n,msr_e,msr_d,lv_n,lv_e,lv_d,"
          "ad_tau_ms,ad_sharp,ad_valid,ad_rej,ad_ng,ad_er,ad_app,ad_blk,"
          "vib_q,vib_rms_g,vib_sug,vib_gq,vib_gyr_dps,vib_high,vib_hf_g,"
          "origin_alt_m,ekf_alt_m,baro_alt_m,p_dd_std_m,reacq_cnt\n");
    }
  }
  if (g_trace_ctrl_path != nullptr)
  {
    g_trace_ctrl_file = std::fopen(g_trace_ctrl_path, "w");
    if (g_trace_ctrl_file == nullptr)
    {
      std::fprintf(stderr, "[trace-ctrl] cannot open %s\n", g_trace_ctrl_path);
    }
    else
    {
      std::fprintf(
          g_trace_ctrl_file,
          "t,quat_w,quat_x,quat_y,quat_z,wx_dps,wy_dps,wz_dps,"
          "rc0,rc1,rc2,rc3,rc4,rc5,rc6,rc7,rc8,alt_up,target_alt,link,fs,"
          "mode,ch1,ch2,ch3,ch4,delta_f,delta_t,dw,or0,or1,or2,ar0,ar1,ar2,"
          "rn,re,rd,vn,ve,vd,roll,pitch,heading,gx_rps,gy_rps,gz_rps,"
          "hdg_ok,pos_deg,gnss_age,sfup,cos_tilt,thr_cmd,alt_err,tgt_vz\n");
    }
  }
  if (g_trace_nav_path != nullptr)
  {
    g_trace_nav_file = std::fopen(g_trace_nav_path, "w");
    if (g_trace_nav_file == nullptr)
    {
      std::fprintf(stderr, "[trace-nav] cannot open %s\n", g_trace_nav_path);
    }
    else
    {
      std::fprintf(
          g_trace_nav_file,
          "t,acc_sum_x,acc_sum_y,acc_sum_z,gyro_sum_x,gyro_sum_y,gyro_sum_z,"
          "sample_count,delta_count,delta_dt_sum,"
          "ahrs_w,ahrs_x,ahrs_y,ahrs_z,"
          "gnss_valid,gnss_lat,gnss_lon,gnss_alt,gnss_vn,gnss_ve,gnss_vd,"
          "gnss_hacc,gnss_vacc,gnss_sacc,gnss_itow,"
          "baro,baro_valid,mag_x,mag_y,mag_z,mag_valid,"
          "flow_vx,flow_vy,flow_range,flow_q,range_q,flow_valid,"
          "unlocked,throttle,"
          "quat_w,quat_x,quat_y,quat_z,roll,pitch,heading,"
          "lla_lat,lla_lon,lla_alt,rn,re,rd,vn,ve,vd,alt_up,"
          "bax,bay,baz,bgx,bgy,bgz,init,fix3d,gnss_age,baro_fused,"
          "hdg_aid,pos_deg,imu_health,baro_innov,gnss_epoch,gnss_fix3d_cnt"
          ",d0tx,d0ty,d0tz,d0vx,d0vy,d0vz,d0dt"
          ",d1tx,d1ty,d1tz,d1vx,d1vy,d1vz,d1dt"
          ",d2tx,d2ty,d2tz,d2vx,d2vy,d2vz,d2dt"
          ",d3tx,d3ty,d3tz,d3vx,d3vy,d3vz,d3dt"
          ",d4tx,d4ty,d4tz,d4vx,d4vy,d4vz,d4dt"
          ",d5tx,d5ty,d5tz,d5vx,d5vy,d5vz,d5dt"
          ",d6tx,d6ty,d6tz,d6vx,d6vy,d6vz,d6dt"
          ",d7tx,d7ty,d7tz,d7vx,d7vy,d7vz,d7dt"
          ",d8tx,d8ty,d8tz,d8vx,d8vy,d8vz,d8dt"
          ",d9tx,d9ty,d9tz,d9vx,d9vy,d9vz,d9dt,t_us"
          // 对拍专用列（尾部追加，旧读取器按列名取值不受影响）：SIL 实际注入的
          // UBX 整数、造出它们的 double LLA，以及调用后的 EKF 协方差对角。
          ",gnss_lat_1e7,gnss_lon_1e7,gnss_alt_mm"
          ",gnss_vn_mmps,gnss_ve_mmps,gnss_vd_mmps"
          ",gnss_hacc_mm,gnss_vacc_mm,gnss_sacc_mmps"
          ",gnss_lat_d,gnss_lon_d,gnss_alt_d"
          ",p_pn,p_pe,p_pd,p_vn,p_ve,p_vd"
          // 控制层武装重定基见证：固件导航原点（度/米）。宿主在解锁边沿用它调
          // `fw_nav_rebase_origin()` 才能与 SIL 同参考系（CTL-004 §6.2）。
          ",origin_lat_d,origin_lon_d,origin_alt_d\n");
    }
  }
  if (g_trace_fast_path != nullptr)
  {
    g_trace_fast_file = std::fopen(g_trace_fast_path, "w");
    if (g_trace_fast_file == nullptr)
    {
      std::fprintf(stderr, "[trace-fast] cannot open %s\n", g_trace_fast_path);
    }
    else
    {
      std::fprintf(g_trace_fast_file,
                   "t,plant_wx,plant_wy,plant_wz,"
                   "gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z\n");
    }
  }

  // --rc-profile 的单调游标：作为 profile 时间的稳定基准，仅在 50 Hz 节拍推进；
  // 没有 profile 时完全不参与任何计算（缺省路径逐位不变）。
  int rc_prof_cursor = 0;
  for (int k = 0; k < steps; ++k)
  {
    const float t = k * dt_ctrl;
    g_sil_time_us = static_cast<uint32_t>(t * 1.0e6f);

    // 真值历史（用于模拟 GNSS 链路延迟；按控制帧 5 ms 存储）。
    const int hi = k % kHist;
    hist[hi].pos[0] = p.pos[0]; hist[hi].pos[1] = p.pos[1]; hist[hi].pos[2] = p.pos[2];
    hist[hi].vel[0] = p.vel[0]; hist[hi].vel[1] = p.vel[1]; hist[hi].vel[2] = p.vel[2];
    hist[hi].q[0] = p.q.w; hist[hi].q[1] = p.q.x;
    hist[hi].q[2] = p.q.y; hist[hi].q[3] = p.q.z;
    hist[hi].omega[0] = p.omega[0]; hist[hi].omega[1] = p.omega[1];
    hist[hi].omega[2] = p.omega[2];

    // RC 50 Hz：链路丢失走真实 linkDownCallback（飞行中保持解锁 + AUTO_ALTITUDE），
    // 不再用“把 CH5 拉低”模拟——那会让固件执行 disarm 而不是 failsafe。
    const bool rc_lost = cfg.rc_loss &&
                         (t >= cfg.rc_loss_start && t < cfg.rc_loss_end);
    if (rc_lost && !rc_lost_prev)
    {
      linkDownCallback();
    }
    else if (!rc_lost && rc_lost_prev)
    {
      linkUpCallback();
    }
    rc_lost_prev = rc_lost;
    if (!rc_lost && (k % 4) == 0)
    {
      if (g_rc_profile_rows > 0)
      {
        // --rc-profile：按 CSV 插值覆盖四根摇杆与 CH7(mode)/CH9(att_mode)；
        // CH5 解锁时序仍沿用 t>=0.5 s，CH6/CH8 保持 set_rc 原值。
        uint16_t prof_roll = 1500, prof_pitch = 1500, prof_yaw = 1500;
        uint16_t prof_thr = 1000, prof_mode = 1000, prof_att = 1000;
        rcProfileAt(static_cast<double>(t), rc_prof_cursor,
                    prof_roll, prof_pitch, prof_yaw, prof_thr, prof_mode,
                    prof_att);
        set_rc(prof_thr, prof_mode, t >= 0.5f, prof_roll, prof_pitch,
               prof_yaw, prof_att);
      }
      else if (coupled)
      {
        const uint16_t roll = (t > 10 && t < 12) ? 1530 : ((t > 12 && t < 14) ? 1470 : 1500);
        const uint16_t pitch = (t > 10 && t < 12) ? 1470 : ((t > 12 && t < 14) ? 1530 : 1500);
        set_rc(1500, 1000, t >= 0.5f, roll, pitch);
      }
      else if (manual)
      {
        const uint16_t roll = (t > 10 && t < 15) ? 1600 : 1500;
        const uint16_t pitch = (t > 20 && t < 25) ? 1600 : 1500;
        set_rc((t < 1) ? 1000 : 1500, 1000, t >= 0.5f, roll, pitch);
      }
      else
      {
        const uint16_t thr = (t < 1.0f) ? 1000 : ((t >= 2.0f && t < 5.0f) ? 1600 : 1500);
        // 1700 落在 AUTO_POSITION 档（<1750 不触发 GUIDED 轨迹规划），
        // 1800 会因 ignition 打开被判成 GUIDED，超时后退回 MANUAL。
        const uint16_t mode = (cfg.pos_hold && t >= cfg.pos_hold_at)
                                  ? 1700
                                  : ((t < 2) ? 1300 : 1500);
        uint16_t roll = 1500;
        uint16_t pitch = 1500;
        if (t >= cfg.fwd_stick_start && t < cfg.fwd_stick_end)
        {
          pitch = 1600;  // SIL 前飞段：定高模式下压杆建立速度和倾斜
        }
        set_rc(thr, mode, t >= 0.5f, roll, pitch);
      }
    }

    // GNSS 可配置刷新率（默认 10 Hz）：真实 UBX NAV-PVT+EOE 注入 Serial4。
    const bool gnss_outage = (t >= cfg.outage_start && t < cfg.outage_end);
    if (gnss_outage && !gnss_outage_prev) alt_at_outage = -p.pos[2];
    gnss_outage_prev = gnss_outage;
    const float gnss_rate_hz = std::fmax(1.0f, std::fmin(100.0f, cfg.gnss_rate_hz));
    const int gnss_div =
        std::max(1, static_cast<int>(std::lround(200.0f / gnss_rate_hz)));
    if (!gnss_outage && (k % gnss_div) == 0 && t >= 0.1f)
    {
      const float t_meas = t - cfg.gnss_latency_s;
      int j = k - static_cast<int>(std::lround(cfg.gnss_latency_s / dt_ctrl));
      if (j < 0) j = 0;
      const TruthSample &h = hist[j % kHist];

      const float gnss_dt_s = 1.0f / gnss_rate_hz;
      const float a_corr = std::exp(-gnss_dt_s / 20.0f);
      const float q_corr = std::sqrt(std::max(0.0f, 1.0f - a_corr * a_corr));
      // 无论启用与否都消耗相同的随机数，保证白噪声臂与有色臂逐 epoch
      // 共用同一白噪声序列，只差 20 s 相关分量本身。
      gnss_corr(0) = a_corr * gnss_corr(0) + q_corr * 0.5f * n01(rng);
      gnss_corr(1) = a_corr * gnss_corr(1) + q_corr * 0.5f * n01(rng);
      gnss_corr(2) = a_corr * gnss_corr(2) + q_corr * 0.8f * n01(rng);
      if (!cfg.gnss_colored)
      {
        gnss_corr.setZero();
      }

      float pn = h.pos[0] + gnss_corr(0) + cfg.gnss_pos_ne_std * n01(rng);
      float pe = h.pos[1] + gnss_corr(1) + cfg.gnss_pos_ne_std * n01(rng);
      float pd = h.pos[2] + gnss_corr(2) + cfg.gnss_pos_d_std * n01(rng);
      float vn = h.vel[0] + cfg.gnss_vel_ne_std * n01(rng);
      float ve = h.vel[1] + cfg.gnss_vel_ne_std * n01(rng);
      float vd = h.vel[2] + cfg.gnss_vel_d_std * n01(rng);

      // 杆臂注入：把 IMU 原点真值换算到天线相位中心，用【量测时刻】的姿态与
      // 体角速率（与固件延迟快照同口径），偏移本身是确定量、不叠加噪声。
      g_sil_gnss_lever_ned[0] = g_sil_gnss_lever_ned[1] =
          g_sil_gnss_lever_ned[2] = 0.0f;
      if (cfg.gnss_lever_inject[0] != 0.0f || cfg.gnss_lever_inject[1] != 0.0f ||
          cfg.gnss_lever_inject[2] != 0.0f)
      {
        const Quat4f qh{h.q[0], h.q[1], h.q[2], h.q[3]};
        const float *r = cfg.gnss_lever_inject;
        const float ocr_body[3] = {
            h.omega[1] * r[2] - h.omega[2] * r[1],
            h.omega[2] * r[0] - h.omega[0] * r[2],
            h.omega[0] * r[1] - h.omega[1] * r[0]};
        float r_ned[3], ocr_ned[3];
        sil::BodyToNed(qh, r, r_ned);
        sil::BodyToNed(qh, ocr_body, ocr_ned);
        g_sil_gnss_lever_ned[0] = r_ned[0];
        g_sil_gnss_lever_ned[1] = r_ned[1];
        g_sil_gnss_lever_ned[2] = r_ned[2];
        pn += r_ned[0];
        pe += r_ned[1];
        pd += r_ned[2];
        vn += ocr_ned[0];
        ve += ocr_ned[1];
        vd += ocr_ned[2];
      }

      bool outlier = false;
      if (cfg.outlier_prob > 0.0f)
      {
        std::uniform_real_distribution<float> u01(0.0f, 1.0f);
        if (u01(rng) < cfg.outlier_prob)
        {
          outlier = true;
          pn += 50.0f; pe -= 40.0f; pd += 30.0f;
          vn += 12.0f; ve -= 10.0f;
        }
      }
      // GNSS spoof：位置偏移但自报精度仍然良好（h_acc 正常），用于检验
      // 30 m 野值门/NIS 能否识别“慢偏/小台阶”欺骗；不是普通野值。
      if (cfg.gnss_spoof && t >= cfg.spoof_start_s)
      {
        const float dt_spoof = t - cfg.spoof_start_s;
        const float off = cfg.spoof_offset_m + cfg.spoof_ramp_mps * dt_spoof;
        pn += off;
        pe += 0.6f * off;
        vn += cfg.spoof_ramp_mps;
        ve += 0.6f * cfg.spoof_ramp_mps;
        const float off_z = cfg.spoof_offset_z_m + cfg.spoof_ramp_z_mps * dt_spoof;
        pd -= off_z;  // 正值=报得更高（NED down 减小）
        vd -= cfg.spoof_ramp_z_mps;
      }

      const Eigen::Vector3d lla = bfs::ned2lla(
          Eigen::Vector3d(pn, pe, pd), kHomeLla, bfs::AngPosUnit::RAD);
      g_sil_gnss_meas_ned[0] = pn;
      g_sil_gnss_meas_ned[1] = pe;
      g_sil_gnss_meas_ned[2] = pd;
      sil::UbxPvtFields f;
      f.itow = 100000U + static_cast<uint32_t>(t_meas * 1000.0f);
      f.lon_1e7 = static_cast<int32_t>(std::llround(lla(1) * 57.29577951308232 * 1.0e7));
      f.lat_1e7 = static_cast<int32_t>(std::llround(lla(0) * 57.29577951308232 * 1.0e7));
      f.height_mm = static_cast<int32_t>(std::llround(lla(2) * 1000.0));
      f.h_msl_mm = f.height_mm;
      f.vel_n_mmps = static_cast<int32_t>(std::llround(vn * 1000.0f));
      f.vel_e_mmps = static_cast<int32_t>(std::llround(ve * 1000.0f));
      f.vel_d_mmps = static_cast<int32_t>(std::llround(vd * 1000.0f));
      f.g_speed_mmps = static_cast<int32_t>(std::llround(std::hypot(vn, ve) * 1000.0f));
      f.fix_type = 3;
      f.num_sv = outlier ? 7 : 14;
      f.h_acc_mm = outlier ? 20000U : static_cast<uint32_t>(
          cfg.gnss_r_scale * std::max(0.5f, 0.8f + 0.5f * gnss_corr.head<2>().norm()) *
          1000.0f);
      f.v_acc_mm = outlier ? 30000U : static_cast<uint32_t>(
          cfg.gnss_r_scale * std::max(1.0f, 1.5f + 0.8f * std::fabs(gnss_corr(2))) *
          1000.0f);
      f.s_acc_mmps = outlier ? 5000U : static_cast<uint32_t>(
          cfg.gnss_r_scale * std::max(0.05f, 0.08f + 0.05f * gnss_corr.head<2>().norm()) *
          1000.0f);
      f.p_dop_1e2 = outlier ? 500U : static_cast<uint16_t>(
          100 + 20 * std::min(4.0f, gnss_corr.norm()));

      uint8_t frame[128];
      size_t n = sil::BuildNavPvt(f, frame, sizeof(frame));
      Serial4.feed(frame, n);
      n = sil::BuildNavEoe(f.itow, frame, sizeof(frame));
      Serial4.feed(frame, n);
      last_gnss_data_ms = millis();
      g_navtick_gnss = true;
      g_navtick_gnss_lla[0] = lla(0) * 57.29577951308232;
      g_navtick_gnss_lla[1] = lla(1) * 57.29577951308232;
      g_navtick_gnss_lla[2] = lla(2);
      g_navtick_gnss_vel[0] = vn;
      g_navtick_gnss_vel[1] = ve;
      g_navtick_gnss_vel[2] = vd;
      g_navtick_gnss_acc[0] = f.h_acc_mm / 1000.0f;
      g_navtick_gnss_acc[1] = f.v_acc_mm / 1000.0f;
      g_navtick_gnss_acc[2] = f.s_acc_mmps / 1000.0f;
      g_navtick_gnss_itow = f.itow;
      // 对拍用：落盘的必须是**进入 UBX 的整数**本身，而不是 float32 度反算值。
      g_navtick_gnss_pos_i[0] = f.lat_1e7;
      g_navtick_gnss_pos_i[1] = f.lon_1e7;
      g_navtick_gnss_pos_i[2] = f.height_mm;
      g_navtick_gnss_vel_i[0] = f.vel_n_mmps;
      g_navtick_gnss_vel_i[1] = f.vel_e_mmps;
      g_navtick_gnss_vel_i[2] = f.vel_d_mmps;
      g_navtick_gnss_acc_i[0] = f.h_acc_mm;
      g_navtick_gnss_acc_i[1] = f.v_acc_mm;
      g_navtick_gnss_acc_i[2] = f.s_acc_mmps;
      g_navtick_gnss_lla_d[0] = lla(0) * 57.29577951308232;
      g_navtick_gnss_lla_d[1] = lla(1) * 57.29577951308232;
      g_navtick_gnss_lla_d[2] = lla(2);
    }
    // 每帧泵一次 UART backlog；真实解析器负责帧同步/校验/epoch 排队。
    ubx.Pump(512, micros());

    // 气压计 25 Hz：一阶滞后 + 噪声 + 漂移 + 1 cm 量化。
    if ((k % 8) == 0)
    {
      // 导弹模式从离地 50 m 起始；导航原点在起飞点，气压高度需减去该偏移。
      const float true_alt = -p.pos[2] - (g_missile_mode ? 50.0f : 0.0f);
      if (!baro_init) { baro_state = true_alt; baro_init = true; }
      if (cfg.baro_lag)
        baro_state += (true_alt - baro_state) * std::fmin(1.0f, 0.04f / 0.15f);
      else
        baro_state = true_alt;
      float b = baro_state + cfg.baro_noise * n01(rng) + cfg.baro_drift * t;
      b = std::round(b / 0.01f) * 0.01f;
      baro_altitude_raw = b;
      baro_altitude = b;
    }

    // 光流 10 Hz：机体系速度 + 尺度/噪声 + 姿态质量门控 + 链路延迟。
    if ((k % 20) == 0)
    {
      const bool flow_dropout =
          (t >= cfg.flow_dropout_start && t < cfg.flow_dropout_end);
      if (cfg.flow && !flow_dropout)
      {
        float vb[3];
        p.ned_to_body(p.vel, vb);
        float r_att, pitch_att, yaw_att;
        p.euler(r_att, pitch_att, yaw_att);
        const bool quality_ok = (std::fabs(r_att) < 0.35f && std::fabs(pitch_att) < 0.35f);
        const float fx = 0.95f * vb[0] + 0.12f * n01(rng);
        const float fy = 0.95f * vb[1] + 0.12f * n01(rng);
        const int fhi = (k / 20) % 32;
        flow_hist_vn[fhi] = fx;
        flow_hist_ve[fhi] = fy;
        const int fdi = (fhi - cfg.flow_delay + 32) % 32;
        flow_data.velocity_x_mps = flow_hist_vn[fdi];
        flow_data.velocity_y_mps = flow_hist_ve[fdi];
        flow_data.flow_quality = quality_ok ? 100 : 0;
        flow_data.is_flow_valid = quality_ok;
        flow_data.timestamp_ms = millis();
        flow_data.last_update_ms = millis();
        flow_data.distance_m = -p.pos[2] + 0.05f * n01(rng);
        flow_data.range_quality = 90;
        flow_data.is_range_valid = true;
        flow_data.last_range_update_ms = millis();
        g_navtick_flow = true;
        g_navtick_flow_v[0] = flow_data.velocity_x_mps;
        g_navtick_flow_v[1] = flow_data.velocity_y_mps;
        g_navtick_flow_range = flow_data.distance_m;
        g_navtick_flow_q = 100;
        g_navtick_range_q = 90;
      }
      else
      {
        flow_data.is_flow_valid = false;
        flow_data.is_range_valid = false;
        g_navtick_flow = true;   // 显式失效也是一次"光流状态更新"
      }
    }

    if (cfg.pos_hold && !hold_set && t >= 5.0f)
    {
      hold_target[0] = p.pos[0];
      hold_target[1] = p.pos[1];
      hold_set = true;
    }

    // 磁力计注入（生产路径）：按真值姿态把站点参考场旋到机体系，
    // 加噪声/硬铁残差/慢变干扰后写入 MagSensor；导航任务自行做门控与 EKF 融合。
    // 100 Hz 注入（2 kHz 内环每 2 拍一次），更接近真实 MagSensor 轮询率。
    if (cfg.mag)
    {
      const bool mag_dropout =
          t >= cfg.mag_dropout_start && t <= cfg.mag_dropout_end;
      if (!mag_dropout && (k % 2) == 0)
      {
        const Eigen::Quaternionf q_true(p.q.w, p.q.x, p.q.y, p.q.z);
        Eigen::Vector3f mag_ned(
            bfs::kDefaultMagReference.field_ut[0],
            bfs::kDefaultMagReference.field_ut[1],
            bfs::kDefaultMagReference.field_ut[2]);
        if (cfg.mag_disturb_ut > 0.0f)
        {
          // NED 慢变干扰：同时破坏模长/倾角/方位，模拟电机/外部磁源。
          mag_ned.x() += cfg.mag_disturb_ut *
                         std::sin(2.0f * 3.14159265f * t / 7.0f + 0.3f);
          mag_ned.y() += cfg.mag_disturb_ut *
                         std::cos(2.0f * 3.14159265f * t / 11.0f + 0.7f);
          mag_ned.z() += 0.5f * cfg.mag_disturb_ut *
                         std::sin(2.0f * 3.14159265f * t / 13.0f);
        }
        Eigen::Vector3f mag_body = q_true.conjugate() * mag_ned;
        mag_body.x() += cfg.mag_noise_ut * n01(rng);
        mag_body.y() += cfg.mag_noise_ut * n01(rng);
        mag_body.z() += cfg.mag_noise_ut * n01(rng);
        magSensor.silInjectFrdSample(mag_body.data(), micros(),
                                     !cfg.mag_cal_invalid);
        g_navtick_mag = true;
        g_navtick_mag_body[0] = mag_body.x();
        g_navtick_mag_body[1] = mag_body.y();
        g_navtick_mag_body[2] = mag_body.z();
      }
    }

    // 真实导航任务 + 真实 GNC。导航任务消费 IMU 聚合量和 UBX epoch。
    const bool calib_hold = (t < g_calib_hold_s);
    if (calib_hold)
    {
      throttlePercent = 0.0f; // 校准段固件看到零油门，静止陀螺/ZUPT 可用
    }
    // 导弹没有 TandemVec 控制，但飞行中固件应按“已解锁 + 有油门”运行：
    // 否则 IMU 健康监测的冻结检测（unlocked && throttle≥3%）永远不武装，
    // SIL 会把冻结故障误测成“检测不到”。1 s 发射支撑后置为飞行状态。
    if (g_missile_mode)
    {
      const bool launched = (t >= 1.0f);
      g_is_unlocked = launched;
      throttlePercent = launched ? 50.0f : 0.0f;
    }
    // 导航对拍轨迹：聚合缓冲在 handleNavigationSystem() 内被清零，必须**调用前**
    // 快照；本拍 GNSS/磁/光流的推送标志由上面的注入分支置位。
    float navtick_inputs[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    float navtick_ahrs[4] = {backup_ahrs_Qw, backup_ahrs_Qx,
                             backup_ahrs_Qy, backup_ahrs_Qz};
    const uint32_t navtick_now_us = micros();
    static constexpr int kNavTraceDeltas = 10;
    float navtick_dth[kNavTraceDeltas][3] = {};
    float navtick_dv[kNavTraceDeltas][3] = {};
    float navtick_ddt[kNavTraceDeltas] = {};
    if (g_trace_nav_file != nullptr)
    {
      navtick_inputs[0] = acc_sum_x;   navtick_inputs[1] = acc_sum_y;
      navtick_inputs[2] = acc_sum_z;   navtick_inputs[3] = gyro_sum_x;
      navtick_inputs[4] = gyro_sum_y;  navtick_inputs[5] = gyro_sum_z;
      navtick_inputs[6] = static_cast<float>(imu_sample_count);
      navtick_inputs[7] = static_cast<float>(imu_delta_sample_count);
      navtick_inputs[8] = imu_delta_time_sum_s;
      const int nd = (imu_delta_sample_count < kNavTraceDeltas)
                         ? imu_delta_sample_count : kNavTraceDeltas;
      for (int i = 0; i < nd; ++i)
      {
        navtick_dth[i][0] = imu_delta_theta_x[i];
        navtick_dth[i][1] = imu_delta_theta_y[i];
        navtick_dth[i][2] = imu_delta_theta_z[i];
        navtick_dv[i][0] = imu_delta_v_x[i];
        navtick_dv[i][1] = imu_delta_v_y[i];
        navtick_dv[i][2] = imu_delta_v_z[i];
        navtick_ddt[i] = imu_delta_dt_s[i];
      }
    }
    handleNavigationSystem();
    if (g_trace_nav_file != nullptr)
    {
      const Eigen::Quaternionf qe = nav_ekf.quat();
      const float tr = nav_ekf.roll_rad() * 57.2957795f;
      const float tp = nav_ekf.pitch_rad() * 57.2957795f;
      const float ty = nav_ekf.yaw_rad() * 57.2957795f;
      const Eigen::Vector3d lla = nav_ekf.lla_rad_m();
      const Eigen::Vector3f vel = nav_ekf.ned_vel_mps();
      const Eigen::Vector3f ba = nav_ekf.accel_bias_mps2();
      const Eigen::Vector3f bg = nav_ekf.gyro_bias_radps();
      std::fprintf(
          g_trace_nav_file,
          "%.4f,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%d,%.9g,"
          "%.9f,%.9f,%.9f,%.9f,"
          "%d,%.9f,%.9f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%u,"
          "%.6f,%d,%.6f,%.6f,%.6f,%d,"
          "%.6f,%.6f,%.6f,%d,%d,%d,%d,%.4f,"
          "%.9f,%.9f,%.9f,%.9f,%.6f,%.6f,%.6f,"
          "%.9f,%.9f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
          "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%d,%.4f,%d,%d,%d,%d,%.4f,%u,%u",
          t,
          navtick_inputs[0], navtick_inputs[1], navtick_inputs[2],
          navtick_inputs[3], navtick_inputs[4], navtick_inputs[5],
          static_cast<int>(navtick_inputs[6]),
          static_cast<int>(navtick_inputs[7]), navtick_inputs[8],
          navtick_ahrs[0], navtick_ahrs[1], navtick_ahrs[2], navtick_ahrs[3],
          g_navtick_gnss ? 1 : 0,
          g_navtick_gnss_lla[0], g_navtick_gnss_lla[1], g_navtick_gnss_lla[2],
          g_navtick_gnss_vel[0], g_navtick_gnss_vel[1], g_navtick_gnss_vel[2],
          g_navtick_gnss_acc[0], g_navtick_gnss_acc[1], g_navtick_gnss_acc[2],
          g_navtick_gnss_itow,
          baro_altitude, 1,
          g_navtick_mag_body[0], g_navtick_mag_body[1], g_navtick_mag_body[2],
          g_navtick_mag ? 1 : 0,
          flow_data.velocity_x_mps, flow_data.velocity_y_mps,
          flow_data.distance_m,
          static_cast<int>(flow_data.flow_quality),
          static_cast<int>(flow_data.range_quality),
          flow_data.is_flow_valid ? 1 : 0,
          g_is_unlocked ? 1 : 0, throttlePercent,
          qe.w(), qe.x(), qe.y(), qe.z(), tr, tp, ty,
          lla(0) * 57.29577951308232, lla(1) * 57.29577951308232, lla(2),
          relative_north, relative_east, relative_down,
          vel.x(), vel.y(), vel.z(), estimated_height,
          ba.x(), ba.y(), ba.z(), bg.x(), bg.y(), bg.z(),
          nav_system_initialized ? 1 : 0,
          (ubx.fix() >= bfs::Ubx::FIX_3D) ? 1 : 0,
          gnss_status.last_measurement_age_s,
          baro_ekf_fused ? 1 : 0,
          g_heading_hold_allowed ? 1 : 0, g_pos_hold_degraded ? 1 : 0,
          static_cast<int>(g_imu_health_state),
          baro_ekf_innovation_m, gnss_status.epoch_count,
          gnss_status.fix3d_count);
      for (int i = 0; i < kNavTraceDeltas; ++i)
      {
        std::fprintf(g_trace_nav_file,
                     ",%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",
                     navtick_dth[i][0], navtick_dth[i][1], navtick_dth[i][2],
                     navtick_dv[i][0], navtick_dv[i][1], navtick_dv[i][2],
                     navtick_ddt[i]);
      }
      // t_us 与对拍专用列同行：先打 t_us（不换行），下面的追加块负责换行。
      std::fprintf(g_trace_nav_file, ",%u", navtick_now_us);
      // 对拍专用列：见证量（注入整数 / double LLA）与调用后 P 对角。取调用后，
      // 与上面的状态列同一时刻；gnss_valid=0 的行整数列沿用上一历元，必须忽略。
      // P 对角只在 `-DEKF_NAV_TELEMETRY` 构建可用（该宏只加只读协方差访问器、
      // 不改滤波数学，见 run_all.sh）；未开时写 nan 而不是 0，避免被当成真值。
      double navtick_pdiag[6];
      for (int i = 0; i < 6; ++i)
      {
        navtick_pdiag[i] = std::numeric_limits<double>::quiet_NaN();
      }
#ifdef EKF_NAV_TELEMETRY
      for (int i = 0; i < 6; ++i)
      {
        navtick_pdiag[i] = static_cast<double>(nav_ekf.covariance_coeff(i, i));
      }
#endif
      std::fprintf(
          g_trace_nav_file,
          ",%d,%d,%d,%d,%d,%d,%u,%u,%u,%.12f,%.12f,%.12f,"
          "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",
          static_cast<int>(g_navtick_gnss_pos_i[0]),
          static_cast<int>(g_navtick_gnss_pos_i[1]),
          static_cast<int>(g_navtick_gnss_pos_i[2]),
          static_cast<int>(g_navtick_gnss_vel_i[0]),
          static_cast<int>(g_navtick_gnss_vel_i[1]),
          static_cast<int>(g_navtick_gnss_vel_i[2]),
          static_cast<unsigned>(g_navtick_gnss_acc_i[0]),
          static_cast<unsigned>(g_navtick_gnss_acc_i[1]),
          static_cast<unsigned>(g_navtick_gnss_acc_i[2]),
          g_navtick_gnss_lla_d[0], g_navtick_gnss_lla_d[1],
          g_navtick_gnss_lla_d[2],
          navtick_pdiag[0], navtick_pdiag[1], navtick_pdiag[2],
          navtick_pdiag[3], navtick_pdiag[4], navtick_pdiag[5]);
      std::fprintf(g_trace_nav_file, ",%.9f,%.9f,%.6f\n",
                   origin_lat_deg, origin_lon_deg, origin_alt_m);
      g_navtick_gnss = false;
      g_navtick_mag = false;
      g_navtick_flow = false;
    }
    // 失联注入的自证据：`gnss_age`（= last_measurement_age_s）是量测本身的年龄、
    // 且取末帧，实测三档都只有 0.025 s，**不能**用来判断失联是否发生；
    // 真正反映"多久没吃到 GNSS"的是 aid_tracker 的 GNSS 年龄，记其峰值。
    const float aid_gnss_age =
        aid_tracker.status(bfs::AidSource::Gnss).age_s;
    // aid_tracker 在首次记录前把 age_s 初始化为 FLT_MAX 哨兵，必须排除，
    // 否则峰值恒为 3.4e38、判据永远"有效"。
    if (aid_gnss_age > 0.0f && aid_gnss_age < 60.0f &&
        aid_gnss_age > max_gnss_age)
    {
      max_gnss_age = aid_gnss_age;
    }
    if (g_missile_mode && g_missile_nav_loop && g_sil_nav_debug_initialized)
    {
      const Eigen::Quaternionf qe = nav_ekf.quat();
      g_ms_att_est = Quat4f{qe.w(), qe.x(), qe.y(), qe.z()};
      const Eigen::Vector3f bg = nav_ekf.gyro_bias_radps();
      g_ms_rate_est[0] = g_sil_nav_debug_gyro[0] - bg.x();
      g_ms_rate_est[1] = g_sil_nav_debug_gyro[1] - bg.y();
      g_ms_rate_est[2] = g_sil_nav_debug_gyro[2] - bg.z();
    }
    if (!calib_hold && !g_missile_mode)
    {
      // 校准段不跑控制：否则位置/姿态积分器在地面长时间饱和，起飞瞬间
      // 直接翻车，会把“控制器饱和”误当成 EKF 不收敛。
      // 导弹模式由弹体自带自动驾驶，固件只跑导航，不跑 TandemVec 控制。
      // 对拍轨迹必须记录**调用前**的 target_altitude：固件的垂直 loiter 状态机
      // 会在调用内修改它，若把调用后的值当输入重放，会晚一拍并双重推进。
      const float target_alt_pre_call = target_altitude;
      runGNCExecutive();
      if (g_trace_ctrl_file != nullptr)
      {
        std::fprintf(
            g_trace_ctrl_file,
            "%.4f,%.9f,%.9f,%.9f,%.9f,%.6f,%.6f,%.6f,"
            "%u,%u,%u,%u,%u,%u,%u,%u,%u,%.4f,%.4f,%d,%d,"
            "%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
            "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
            "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
            "%.6f,%.6f,%.6f,%.9f,%.9f,%.9f,%d,%d,%.4f,%.6f,%.6f,%.4f,%.4f,%.4f\n",
            t, AHRS_Packet.Qw, AHRS_Packet.Qx, AHRS_Packet.Qy, AHRS_Packet.Qz,
            current_omega_dps_body_filtered.x, current_omega_dps_body_filtered.y,
            current_omega_dps_body_filtered.z,
            raw_rc_values[0], raw_rc_values[1], raw_rc_values[2], raw_rc_values[3],
            raw_rc_values[4], raw_rc_values[5], raw_rc_values[6], raw_rc_values[7],
            raw_rc_values[8], estimated_height, target_alt_pre_call,
            isLinkUp ? 1 : 0, failsafe_in_flight ? 1 : 0,
            static_cast<int>(g_current_flight_mode),
            ch1_output, ch2_output, ch3_output, ch4_output,
            gnc_tel.delta_f_deg, gnc_tel.delta_t_deg, gnc_tel.dw,
            gnc_tel.omega_ref_dps[0], gnc_tel.omega_ref_dps[1], gnc_tel.omega_ref_dps[2],
            gnc_tel.alpha_ref_dps2[0], gnc_tel.alpha_ref_dps2[1],
            gnc_tel.alpha_ref_dps2[2],
            relative_north, relative_east, relative_down,
            INS_GNSS_Packet.velocity_north, INS_GNSS_Packet.velocity_east,
            INS_GNSS_Packet.velocity_down,
            AHRS_Packet.Roll, AHRS_Packet.Pitch, AHRS_Packet.Heading,
            static_cast<double>(icm_gyro_x) -
                (nav_system_initialized ? nav_ekf.gyro_bias_radps().x() : 0.0f),
            static_cast<double>(icm_gyro_y) -
                (nav_system_initialized ? nav_ekf.gyro_bias_radps().y() : 0.0f),
            static_cast<double>(icm_gyro_z) -
                (nav_system_initialized ? nav_ekf.gyro_bias_radps().z() : 0.0f),
            g_heading_hold_allowed ? 1 : 0, g_pos_hold_degraded ? 1 : 0,
            static_cast<double>(g_gnss_outage_age_s),
            static_cast<double>(gnc_tel.specific_force_up_mps2),
            static_cast<double>(gnc_tel.thrust_cos_tilt),
            static_cast<double>(gnc_tel.throttle_cmd_pct),
            static_cast<double>(gnc_tel.alt_error_m),
            static_cast<double>(gnc_tel.target_vz_mps));
      }
    }
    if (g_sil_nav_debug_imu_failed)
    {
      imu_fault_detected_any = true;
    }
    if (g_mode_degraded_gnss)
    {
      gnss_degrade_seen = true;
    }
    if (std::fabs(mag_status.last_kick_deg) > max_mag_kick)
    {
      max_mag_kick = std::fabs(mag_status.last_kick_deg);
    }
    if (mag_status.last_kick_rp_deg > max_mag_kick_rp)
    {
      max_mag_kick_rp = mag_status.last_kick_rp_deg;
    }

    if (g_trace_csv_file != nullptr)
    {
      float tr = 0.0f, tp = 0.0f, ty = 0.0f;
      p.euler(tr, tp, ty);
      // 位置 D 协方差（仅遥测二进制可读；默认构建恒 -1，保持列集合固定）
#ifdef EKF_NAV_TELEMETRY
      const double p_dd_std_trace = std::sqrt(std::fabs(nav_ekf.covariance_coeff(2, 2)));
#else
      const double p_dd_std_trace = -1.0;
#endif
      std::fprintf(
          g_trace_csv_file,
          "%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
          "%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,"
          "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
          "%.3f,%.3f,%u,%d,%d,%d,%d,%.1f,%.1f,%.1f,%.1f,%.2f,%u,%.4f,"
          "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.3f,"
          "%.4f,%.4f,%.4f,%.4f,%.3f,%d,"
          "%.2f,%.2f,%.4f,%.2f,%.2f,%d,%d,%u,"
          "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.4f,%d,%u,%u,%u,%u,%u,"
          "%.3f,%.3f,%.3f,%.4f,%.2f,%d,%.4f,"
          "%.3f,%.3f,%.3f,%.3f,%u\n",
          t, tr * 57.29578f, tp * 57.29578f, ty * 57.29578f,
          nav_ekf.roll_rad() * 57.29578f, nav_ekf.pitch_rad() * 57.29578f,
          nav_ekf.yaw_rad() * 57.29578f, g_sil_nav_debug_gyro[0],
          g_sil_nav_debug_gyro[1], g_sil_nav_debug_gyro[2],
          gnc_tel.omega_ref_dps[0], gnc_tel.omega_ref_dps[1],
          gnc_tel.omega_ref_dps[2], relative_north, relative_east,
          p.pos[0], p.pos[1], hold_target[0], hold_target[1],
          mag_status.last_kick_deg, mag_status.last_kick_rp_deg,
          static_cast<unsigned>(mag_status.last_gate_reason),
          static_cast<int>(g_current_flight_mode),
          g_pos_hold_reanchor ? 1 : 0, g_heading_aiding_valid ? 1 : 0,
          g_mode_degraded_heading ? 1 : 0, ch1_output, ch2_output,
          ch3_output, ch4_output, nav_ekf.yaw_sigma_rad() * 57.29578f,
          static_cast<unsigned>(g_heading_aid_mode),
          nav_ekf.gyro_bias_radps().z(),
          aid_tracker.status(bfs::AidSource::Mag).age_s,
          g_heading_aiding_age_s,
          -p.pos[2], -relative_down, target_altitude, -p.vel[2],
          -INS_GNSS_Packet.velocity_down, ekf_accel_bias_z_mps2,
          vfk_velocity,
          nav_ekf.gyro_bias_radps().x() * 57.29578f,
          nav_ekf.gyro_bias_radps().y() * 57.29578f,
          nav_ekf.accel_bias_mps2().x(),
          nav_ekf.accel_bias_mps2().y(),
          baro_ekf_innovation_m, baro_ekf_fused ? 1 : 0,
          nav_ekf.ned_vel_mps().x(), nav_ekf.ned_vel_mps().y(),
          nav_ekf.tilt_sigma_rad() * 57.29578f,
          nav_ekf.gnss_resid_pos_ne_m(), nav_ekf.gnss_resid_vel_ne_mps(),
          nav_ekf.gnss_gate_pos_outlier() ? 1 : 0,
          nav_ekf.gnss_gate_vel_outlier() ? 1 : 0,
          static_cast<unsigned>(nav_ekf.gnss_velocity_subset_recovery_count()),
          g_sil_gnss_meas_ned[0], g_sil_gnss_meas_ned[1],
          g_sil_gnss_meas_ned[2], g_sil_gnss_lever_ned[0],
          g_sil_gnss_lever_ned[1], g_sil_gnss_lever_ned[2],
          static_cast<double>(gnss_status.auto_delay_tau_s * 1000.0f),
          static_cast<double>(gnss_status.auto_delay_sharpness),
          gnss_status.auto_delay_valid ? 1 : 0,
          static_cast<unsigned>(gnss_status.auto_delay_reject),
          static_cast<unsigned>(gnss_status.auto_delay_n_gnss),
          static_cast<unsigned>(gnss_status.auto_delay_n_erased),
          static_cast<unsigned>(gnss_status.auto_delay_applied_count),
          static_cast<unsigned>(gnss_status.auto_delay_apply_block),
          static_cast<double>(g_sil_nav_debug_vib[0]),
          static_cast<double>(g_sil_nav_debug_vib[1]),
          static_cast<double>(g_sil_nav_debug_vib[2]),
          static_cast<double>(g_sil_nav_debug_vib[3]),
          static_cast<double>(gnc_tel.vib_gyro_rms_dps),
          gnc_tel.vib_high ? 1 : 0,
          static_cast<double>(g_ins_hf_vibration.last().rms_g),
          origin_alt_m, nav_ekf.alt_m(),
          static_cast<double>(baro_altitude),
          p_dd_std_trace,
          nav_ekf.gnss_reacquire_reset_count());
    }

    if (!manual && !alt_cmd && t >= g_takeoff_at_s)
    {
      target_altitude = target_alt;
      vertical_loiter_state = ALTITUDE_HOVERING;
      alt_cmd = true;
    }

    // 控制输出 -> 执行器目标（量化在控制帧，动态在 2 kHz 内环）。
#ifdef FW_SIL_COAXIAL
    // 共轴：ch1/ch2 = 33.33–66.67% → ±15° 修正角；ch3/ch4 = 电机百分比。
    // mapCoaxial: roll_corr/half → 舵机百分比增大；pitch 取反。
    const float coax_roll_corr = (ch1_output - 50.0f) * 15.0f / 16.6667f;
    const float coax_pitch_corr = -(ch2_output - 50.0f) * 15.0f / 16.6667f;
    const float q = 0.001f * 15.0f * 2.0f * 0.017453292f; // 0.1% × 15°
    servo_target_f = std::round(coax_roll_corr * 0.017453292f / q) * q;
    servo_target_t = std::round(coax_pitch_corr * 0.017453292f / q) * q;
#else
    const float upper = (ch1_output - (50 + SC.zero_roll_pct)) * SC.half_travel_deg /
                        (50 * (SC.teeth_gimbal / SC.teeth_servo) * SC.dir_roll) * 0.017453292f;
    const float lower = (ch2_output - (50 + SC.zero_pitch_pct)) * SC.half_travel_deg /
                        (50 * (SC.teeth_gimbal / SC.teeth_servo) * SC.dir_pitch) * 0.017453292f;
    const float q = 0.001f * SC.half_travel_deg * 2.0f;
    servo_target_f = std::round(upper / q) * q;
    servo_target_t = std::round(lower / q) * q;
#endif
    motor_target_wf = (ch3_output / 100.0f) * P.wMax;
    motor_target_wt = (ch4_output / 100.0f) * P.wMax;
    if (t < g_calib_hold_s)
    {
      motor_target_wf = 0.0f;
      motor_target_wt = 0.0f;
    }

    // 2 kHz 内环：执行器动态 + plant + IMU 采样。
    for (int s = 0; s < sub_steps; ++s)
    {
      const float ts = t + static_cast<float>(s) * dt_imu;
      g_sil_time_us = static_cast<uint32_t>(ts * 1.0e6f);

      if (g_missile_mode)
      {
        if (g_missile_nav_loop && g_sil_nav_debug_initialized) {
          missile.Step(dt_imu, &g_ms_att_est, g_ms_rate_est);
        } else {
          missile.Step(dt_imu);
        }
        const auto &ms = missile.state();
        for (int a = 0; a < 3; ++a) {
          p.pos[a] = ms.pos[a]; p.vel[a] = ms.vel[a];
          p.omega[a] = ms.omega[a]; p.force_ned[a] = ms.force_ned[a];
          p.acc_ned[a] = ms.acc_ned[a];
        }
        p.q = ms.q;
        p.on_ground = false;
      }
      else
      {
      // 执行器动态（缺省=改动前的硬编码值：舵机 τ=0.05 s + 300 °/s 速率限幅、
      // 电机 τ=0.28 s）。--coax-motor-tau / --coax-servo-rate 只在显式给出时
      // 覆盖；τ=0 / rate=0 表示理想器件（仅作对照臂，不是实机值）。
#ifdef FW_SIL_COAXIAL
      const float servo_rate_dps =
          (g_coax_servo_rate_dps >= 0.0f) ? g_coax_servo_rate_dps : 300.0f;
      const float motor_tau_s =
          (g_coax_motor_tau_s >= 0.0f) ? g_coax_motor_tau_s : 0.28f;
#else
      const float servo_rate_dps = 300.0f;
      const float motor_tau_s = 0.28f;
#endif
      const float rate_limit = (servo_rate_dps > 0.0f)
                                   ? servo_rate_dps * 0.017453292f * dt_imu
                                   : 1.0e9f;  // 0 = 无限速率
      const float servo_step_f = std::fmax(-rate_limit, std::fmin(rate_limit,
          (servo_target_f - p.servo_f) * std::fmin(1.0f, dt_imu / 0.05f)));
      const float servo_step_t = std::fmax(-rate_limit, std::fmin(rate_limit,
          (servo_target_t - p.servo_t) * std::fmin(1.0f, dt_imu / 0.05f)));
      p.servo_f += servo_step_f;
      p.servo_t += servo_step_t;
      const float motor_alpha = (motor_tau_s > 0.0f)
                                    ? std::fmin(1.0f, dt_imu / motor_tau_s)
                                    : 1.0f;
      p.motor_wf += (motor_target_wf - p.motor_wf) * motor_alpha;
      p.motor_wt += (motor_target_wt - p.motor_wt) * motor_alpha;

#ifdef FW_SIL_COAXIAL
      // 舵机卡死（--coax-servojam roll|pitch）：到达卡死时刻后锁存**当时的
      // 实际角度**并强制保持（齿轮咬死/连杆卡滞的一阶近似）；缺省 0 跳过。
      // 与 --fault 4（前摆 8° 硬偏置）同时启用时，下方 fault 4 覆盖 roll，
      // 即故障注入优先级更高（文档已注明）。
      if (g_coax_servojam != 0)
      {
        const float jam_at = (g_coax_servojam_at_s >= 0.0f)
                                 ? g_coax_servojam_at_s : fault_at;
        if (t >= jam_at)
        {
          if (!coax_jam_latched)
          {
            coax_jam_latched = true;
            coax_jam_rad = (g_coax_servojam == 1) ? p.servo_f : p.servo_t;
          }
          if (g_coax_servojam == 1) p.servo_f = coax_jam_rad;
          else                      p.servo_t = coax_jam_rad;
        }
      }
#endif

      if (fault == 4 && t > fault_at) p.servo_f = 8.0f * 0.017453292f;
      p.wind[1] = (wind_on && t > cfg.wind_start_s && t < cfg.wind_end_s)
                       ? cfg.wind_force_mps2 * P.m
                       : 0.0f;

#ifdef FW_SIL_COAXIAL
      g_coax_motor_loss1 = 1.0f;
      g_coax_motor_loss2 = 1.0f;
      if (t > fault_at && (fault == 3 || fault == 5))
      {
        g_coax_motor_loss1 = loss;
        if (fault == 5) g_coax_motor_loss2 = loss;
      }
      p.step(computeCoaxialWrench(p), dt_imu);
#else
      PropulsionState ps{p.motor_wf, p.motor_wt, p.servo_f, p.servo_t};
      if ((fault == 3 || fault == 5) && t > fault_at)
      {
        ps.wf *= loss;
        if (fault == 5) ps.wt *= loss;
      }
      p.step(computeWrench(ps, P), dt_imu);
#endif
      }  // !g_missile_mode

      float f_ned[3];
      if (p.on_ground)
      {
        f_ned[0] = 0.0f; f_ned[1] = 0.0f; f_ned[2] = -P.g;
      }
      else
      {
        const float mass_used = g_missile_mode ? missile.state().mass_kg : P.m;
        f_ned[0] = p.force_ned[0] / mass_used;
        f_ned[1] = p.force_ned[1] / mass_used;
        f_ned[2] = p.force_ned[2] / mass_used;
      }
      float f_body[3];
      p.ned_to_body(f_ned, f_body);

      const float rpm_ratio = std::fmax(p.motor_wf, p.motor_wt) / P.wMax;
      const float vib_scale = g_missile_mode ? 1.0f : rpm_ratio * rpm_ratio;
      std::uniform_real_distribution<float> u01(0.0f, 1.0f);
      const bool imu_dropout_window =
          t >= cfg.imu_dropout_start && t <= cfg.imu_dropout_end;
      const bool drop = imu_dropout_window ||
                        ((cfg.imu_drop_prob > 0.0f) &&
                         (u01(rng) < cfg.imu_drop_prob));
      const bool imu_fault =
          (cfg.imu_fault_mode != 0) &&
          (t >= cfg.imu_fault_start && t < cfg.imu_fault_end);
      if (drop)
      {
        pending_imu_dt += dt_imu;
      }
      else
      {
        const float sample_dt = dt_imu + pending_imu_dt;
        pending_imu_dt = 0.0f;
        sil::ImuSample sample;
        if (imu_fault && cfg.imu_fault_mode == 1 && have_last_imu)
        {
          sample = last_imu;  // 传感器输出冻结
        }
        else
        {
          const float omega[3] = {p.omega[0], p.omega[1], p.omega[2]};
          float vib_now =
              (ts >= g_vib_start_s) ? cfg.vib_g * vib_scale : 0.0f;
          if (vib_now > 0.0f && g_vib_iso_fn_hz > 0.0f)
          {
            // 多频模式（40/80/120 Hz）用 80 Hz 代表频率做幅值缩放；
            // 单频模式按实际激励频率计算。真实减震座应按频谱积分。
            const float vib_f = (cfg.vib_mode == 1) ? 80.0f : cfg.vib_hz;
            vib_now *= sil::VibrationTransmissibility(vib_f, g_vib_iso_fn_hz,
                                                      g_vib_iso_zeta);
          }
          sample = imu.Step(
              omega, f_body, ts, sample_dt, vib_now, cfg.vib_hz,
              cfg.vib_mode, cfg.anti_alias);
          if (g_trace_fast_file != nullptr && ts >= g_trace_fast_start_s &&
              ts < g_trace_fast_end_s)
          {
            std::fprintf(g_trace_fast_file,
                         "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                         ts, omega[0], omega[1], omega[2], sample.gyro[0],
                         sample.gyro[1], sample.gyro[2], sample.accel[0],
                         sample.accel[1], sample.accel[2]);
          }
          if (imu_fault && cfg.imu_fault_mode == 2)
          {
            for (int a = 0; a < 3; ++a)
            {
              sample.accel[a] = kImuNaN;
              sample.gyro[a] = kImuNaN;
            }
          }
          else
          {
            last_imu = sample;
            have_last_imu = true;
          }
        }
        // 生产 sensor_imu.cpp 在 2 kHz 样本上做一阶软件低通；非有限样本不更新
        // 滤波器状态（否则 NaN 会永久污染后续恢复）。
        const sil::ImuSample sample_raw = sample;
        if (g_imu_lpf_enable && sample.accel[0] == sample.accel[0] &&
            sample.accel[1] == sample.accel[1] && sample.accel[2] == sample.accel[2] &&
            sample.gyro[0] == sample.gyro[0] && sample.gyro[1] == sample.gyro[1] &&
            sample.gyro[2] == sample.gyro[2])
        {
          sample = sil::ImuLpfApply(imu_lpf, sample);
        }
        // 高频振动通道：与生产 `sensor_imu.cpp` 步骤 4.5 同口径——用软件低通
        // 前/后之差（补高通）测 80–120 Hz 机械振动；SIL 样本单位 m/s²，换算成 g。
        {
          constexpr float kGn = 9.80665f;
          const float hf_raw_g[3] = {sample_raw.accel[0] / kGn,
                                     sample_raw.accel[1] / kGn,
                                     sample_raw.accel[2] / kGn};
          const float hf_lp_g[3] = {sample.accel[0] / kGn, sample.accel[1] / kGn,
                                    sample.accel[2] / kGn};
          g_ins_hf_vibration.Update(hf_raw_g, hf_lp_g, sample_dt);
        }
        // 生产 sensor_imu.cpp 顺序：滤波后的机体系 FRD → Madgwick 输入系
        // (x, -y, -z) → updateIMU(dps, g) → NED/FRD 欧拉 → backup_ahrs_*。
        if (g_sil_madgwick_enable &&
            sample.accel[0] == sample.accel[0] &&
            sample.accel[1] == sample.accel[1] &&
            sample.accel[2] == sample.accel[2] &&
            sample.gyro[0] == sample.gyro[0] &&
            sample.gyro[1] == sample.gyro[1] &&
            sample.gyro[2] == sample.gyro[2])
        {
          constexpr double kGravity = 9.80665;
          constexpr double kRadToDeg = 57.29577951308232;
          const double ax_m = sample.accel[0] / kGravity;
          const double ay_m = -sample.accel[1] / kGravity;
          const double az_m = -sample.accel[2] / kGravity;
          const double gx_m = sample.gyro[0] * kRadToDeg;
          const double gy_m = -sample.gyro[1] * kRadToDeg;
          const double gz_m = -sample.gyro[2] * kRadToDeg;
          if (!madgwick_sil_init)
          {
            madgwick_sil.initializeFromAccelerometer(ax_m, ay_m, az_m);
            madgwick_sil_init = true;
          }
          madgwick_sil.updateIMU(gx_m, gy_m, gz_m, ax_m, ay_m, az_m, sample_dt);
          double ahrs_roll = 0.0, ahrs_pitch = 0.0, ahrs_yaw = 0.0;
          madgwick_sil.getEulerAngles_NED_FRD(ahrs_roll, ahrs_pitch, ahrs_yaw);
          backup_ahrs_roll = static_cast<float>(ahrs_roll);
          backup_ahrs_pitch = static_cast<float>(ahrs_pitch);
          backup_ahrs_yaw = wrapAnglePi(static_cast<float>(ahrs_yaw) +
                                        ahrs_yaw_correction_rad);
          const Quaternion ahrs_q = eulerToQuaternion(
              static_cast<double>(backup_ahrs_roll),
              static_cast<double>(backup_ahrs_pitch),
              static_cast<double>(backup_ahrs_yaw));
          backup_ahrs_Qw = ahrs_q.w;
          backup_ahrs_Qx = ahrs_q.x;
          backup_ahrs_Qy = ahrs_q.y;
          backup_ahrs_Qz = ahrs_q.z;
        }
        // 导航增量源：默认取滤波值（生产口径）；`--nav-raw-imu` 改取未滤波值；
        // 再叠加 `--nav-raw-fault-fallback` 则在故障期内退回滤波值。
        // 锁存键两种：注入故障窗（研究捷径）或固件健康状态 Failed（= 固件化时真正拿得到的信号）。
        // 健康键分支直接复用固件模块 `NavImuIncrementSelect.h`（与 `sensor_imu.cpp`
        // 里 `BFS_NAVIGATION_RAW_IMU_INCREMENT` 打开后的调用完全同一份代码），
        // 语义与原先的内联锁存逐拍等价：Failed 拍置满 10 000、其后非 Failed 拍递减。
        static bfs::NavImuIncrementSelector s_nav_fb_sel(
            true, bfs::NavImuIncrementSelector::kDefaultHoldTicks);
        bool fault_latched = false;
        if (g_nav_raw_fb_use_health)
        {
          fault_latched =
              s_nav_fb_sel.select(g_sil_nav_debug_imu_failed) == bfs::NavImuSource::Filtered;
        }
        else
        {
          fault_latched = (cfg.imu_fault_mode != 0) &&
                          (t >= cfg.imu_fault_start + g_nav_raw_fb_start_delay_s) &&
                          (t < cfg.imu_fault_end + g_nav_raw_fb_hold_s);
        }
        const bool use_raw_nav = g_nav_raw_imu &&
                                 !(g_nav_raw_fault_fallback && fault_latched);
        if (g_nav_raw_imu && g_nav_raw_fault_fallback && fault_latched)
        {
          ++g_nav_raw_fb_frames;
        }
        sil::ImuSample nav_sample = use_raw_nav ? sample_raw : sample;
        if (cfg.nav_accel_bias_start_s >= 0.0f &&
            t >= cfg.nav_accel_bias_start_s && t < cfg.nav_accel_bias_end_s)
        {
          for (int a = 0; a < 3; ++a)
          {
            nav_sample.accel[a] += cfg.nav_accel_bias_mps2[a];
          }
          ++g_nav_bias_frames;
        }
        accumulateImuSample(nav_sample, sample_dt);
        icm_gyro_x = sample.gyro[0];
        icm_gyro_y = sample.gyro[1];
        icm_gyro_z = sample.gyro[2];
        icm_accel_x = sample.accel[0];
        icm_accel_y = sample.accel[1];
        icm_accel_z = sample.accel[2];
      }
    }

    // 每帧统计（避免 1 Hz 抽样漏掉尖峰）；打印仍保持 1 Hz。
    float r = 0.0f, pitch = 0.0f, yaw = 0.0f;
    p.euler(r, pitch, yaw);
    if (std::fabs(r * 57.29578f) > max_roll) max_roll = std::fabs(r * 57.29578f);
    if (!landed && std::fabs(r * 57.29578f) > max_roll_preland)
    {
      max_roll_preland = std::fabs(r * 57.29578f);
    }
    if (!airborne_seen && -p.pos[2] > 1.0f)
    {
      airborne_seen = true;
    }
    if (airborne_seen && !landed && p.vel[2] > max_desc_vz_down)
    {
      max_desc_vz_down = p.vel[2];
    }
    // 首次从 >1 m 降到 <0.5 m 记为落地；land_* 是落地瞬间的快照。
    if (airborne_seen && !landed && -p.pos[2] < 0.5f)
    {
      landed = true;
      land_t = t;
      land_vz_down = p.vel[2];
      land_roll_deg = std::fabs(r * 57.29578f);
      if (hold_set)
      {
        land_hold = std::hypot(double(p.pos[0]) - double(hold_target[0]),
                               double(p.pos[1]) - double(hold_target[1]));
      }
      land_pos_err = std::hypot(double(relative_north) - double(p.pos[0]),
                                double(relative_east) - double(p.pos[1]));
    }
    if (std::fabs(pitch * 57.29578f) > max_pitch) max_pitch = std::fabs(pitch * 57.29578f);
    if (t > 9.0f)
    {
      sum_roll2 += double(r * 57.29578f) * double(r * 57.29578f);
      sum_pitch2 += double(pitch * 57.29578f) * double(pitch * 57.29578f);
      ++n_att;
    }
    if (nav_system_initialized && t > 10.0f)
    {
      const double en = double(relative_north) - double(p.pos[0]);
      const double ee = double(relative_east) - double(p.pos[1]);
      const double e2 = en * en + ee * ee;
      sum_pos2 += e2;
      if (std::sqrt(e2) > max_pos) max_pos = std::sqrt(e2);
      ++n_pos;
      sum_nav_alt += nav_ekf.alt_m() - origin_alt_m;
      ++n_nav_alt;
      const Eigen::Quaternionf qe = nav_ekf.quat();
      const Eigen::Quaternionf qt(p.q.w, p.q.x, p.q.y, p.q.z);
      const float qdot = std::fabs(std::clamp(qe.dot(qt), -1.0f, 1.0f));
      const float att_err = 2.0f * std::acos(qdot);
      const Eigen::Vector3f vel_err =
          nav_ekf.ned_vel_mps() - Eigen::Vector3f(p.vel[0], p.vel[1], p.vel[2]);
      sum_nav_att2 += static_cast<double>(att_err) * att_err;
      sum_nav_vel2 += vel_err.squaredNorm();
      max_nav_att = std::max(max_nav_att, static_cast<double>(att_err));
      // 欧拉角差：航向先包到 ±180°，避免 ±π 跳变把 RMSE 拉爆。
      float dyaw = nav_ekf.yaw_rad() - yaw;
      while (dyaw > 3.14159265f) dyaw -= 6.2831853f;
      while (dyaw < -3.14159265f) dyaw += 6.2831853f;
      const double er = double(nav_ekf.roll_rad() - r) * 57.29578;
      const double ep = double(nav_ekf.pitch_rad() - pitch) * 57.29578;
      const double ey = double(dyaw) * 57.29578;
      sum_nav_roll2 += er * er;
      sum_nav_pitch2 += ep * ep;
      sum_nav_yaw2 += ey * ey;
      max_nav_roll = std::max(max_nav_roll, std::fabs(er));
      max_nav_pitch = std::max(max_nav_pitch, std::fabs(ep));
      max_nav_yaw = std::max(max_nav_yaw, std::fabs(ey));
      ++n_nav_att;
    }
#ifdef EKF_NAV_TELEMETRY
    // 原点变化即重取真值基准（包括 t<=10 s 的起飞前/爬升段），
    // 否则 10 s 才取样会把已爬升的高度当成恒定位置偏差。
    if (nav_system_initialized && is_origin_lla_set)
    {
      const Eigen::Vector3d origin_lla(origin_lat_rad, origin_lon_rad, origin_alt_m);
      if (!nees_origin_valid || (origin_lla - nees_origin_last).norm() > 1e-9)
      {
        nees_origin_valid = true;
        nees_origin_last = origin_lla;
        nees_truth_origin = Eigen::Vector3d(p.pos[0], p.pos[1], p.pos[2]);
      }
    }
    // 10 Hz 采样、跳过前 10 s 收敛段。
    if (nav_system_initialized && is_origin_lla_set && t > 10.0f && (k % 20) == 0)
    {
      const Eigen::Vector3d origin_lla(origin_lat_rad, origin_lon_rad, origin_alt_m);
      const Eigen::Vector3d est = bfs::lla2ned(
          nav_ekf.lla_rad_m(), origin_lla, bfs::AngPosUnit::RAD);
      const Eigen::Vector3d truth = Eigen::Vector3d(p.pos[0], p.pos[1], p.pos[2]) -
                                    nees_truth_origin;
      const Eigen::Vector3f ep = (est - truth).cast<float>();
      const Eigen::Vector3f ev = nav_ekf.ned_vel_mps() -
                                 Eigen::Vector3f(p.vel[0], p.vel[1], p.vel[2]);
      Eigen::Matrix3f Pp, Pv;
      for (int rr = 0; rr < 3; ++rr)
      {
        for (int cc = 0; cc < 3; ++cc)
        {
          Pp(rr, cc) = nav_ekf.covariance_coeff(rr, cc);
          Pv(rr, cc) = nav_ekf.covariance_coeff(3 + rr, 3 + cc);
        }
      }
      const Eigen::Vector3f wp = Pp.ldlt().solve(ep);
      const Eigen::Vector3f wv = Pv.ldlt().solve(ev);
      const float nees_p = ep.dot(wp) / 3.0f;
      const float nees_v = ev.dot(wv) / 3.0f;
      if (std::isfinite(nees_p) && nees_p >= 0.0f) nees_pos_samples.push_back(nees_p);
      if (std::isfinite(nees_v) && nees_v >= 0.0f) nees_vel_samples.push_back(nees_v);
      nees_dbg_ep = ep.norm();
      nees_dbg_sig = std::sqrt(std::fmax(0.0f, (Pp(0, 0) + Pp(1, 1) + Pp(2, 2)) / 3.0f));
      nees_dbg_val = nees_p;
    }
#endif
    if (hold_set && t > 10.0f)
    {
      const double hx = double(p.pos[0]) - double(hold_target[0]);
      const double hy = double(p.pos[1]) - double(hold_target[1]);
      const double h2 = hx * hx + hy * hy;
      sum_hold2 += h2;
      if (std::sqrt(h2) > max_hold) max_hold = std::sqrt(h2);
      ++n_hold;
      if (t > (seconds - 5.0f))
      {
        sum_hold_end2 += h2;
        ++n_hold_end;
      }
    }

    // 1 Hz 诊断打印。
    if ((k % 200) == 0)
    {
      const long age_ms = (last_gnss_data_ms > 0)
                              ? static_cast<long>(millis() - last_gnss_data_ms)
                              : -1;
      Eigen::Vector3d obs_rel = Eigen::Vector3d::Zero();
      if (gnss_status.has_obs && is_origin_lla_set)
      {
        Eigen::Vector3d origin_lla(origin_lat_rad, origin_lon_rad, origin_alt_m);
        obs_rel = bfs::lla2ned(gnss_status.last_obs_lla_rad_m, origin_lla,
                               bfs::AngPosUnit::RAD);
      }
      std::printf("t=%5.1f alt=%7.2f nav_alt=%7.2f pos=(%6.2f,%6.2f) truth=(%6.2f,%6.2f) "
                  "obs=(%6.2f,%6.2f) nis=%.2f ep=%u pvt=%u eoe=%u pend=%u "
                  "roll=%6.2f pitch=%6.2f yaw=%6.2f navyaw=%6.2f "
                  "servo=(%5.1f,%5.1f) motor=(%5.1f,%5.1f) "
                  "nav=%d fix=%d gnss_age=%ld imu=%d acc=(%6.2f,%6.2f,%6.2f) "
                  "gyro=(%6.3f,%6.3f,%6.3f) st=%d/%.2f/%u epoch=%d initd=%d "
                  "mk=%.2f mg=%u\n",
                  t, -p.pos[2],
                  nav_system_initialized ? (nav_ekf.alt_m() - origin_alt_m) : 0.0,
                  relative_north, relative_east, p.pos[0], p.pos[1],
                  obs_rel(0), obs_rel(1), gnss_status.last_test_ratio,
                  gnss_status.epoch_count, gnss_status.parser_nav_pvt_count,
                  gnss_status.parser_nav_eoe_count, gnss_status.pending_epoch_count,
                  r * 57.29578f, pitch * 57.29578f, yaw * 57.29578f,
                  nav_system_initialized ? nav_ekf.yaw_rad() * 57.29578f : 0.0f,
                  ch1_output, ch2_output, ch3_output, ch4_output,
                  nav_system_initialized ? 1 : 0,
                  static_cast<int>(ubx.fix()), age_ms,
                  g_sil_nav_debug_imu_count,
                  g_sil_nav_debug_accel[0], g_sil_nav_debug_accel[1], g_sil_nav_debug_accel[2],
                  g_sil_nav_debug_gyro[0], g_sil_nav_debug_gyro[1], g_sil_nav_debug_gyro[2],
                  g_sil_nav_debug_static ? 1 : 0,
                  g_sil_nav_debug_static_confidence,
                  g_sil_nav_debug_static_frames,
                  g_sil_nav_debug_gnss_epoch ? 1 : 0,
                  g_sil_nav_debug_initialized ? 1 : 0,
                  mag_status.last_kick_deg,
                  static_cast<unsigned>(mag_status.last_gate_reason));
      if (g_missile_mode) {
        const auto &ms = missile.state();
        float rpy_r, rpy_p, rpy_y;
        {
          Quaternion qd{ms.q.w, ms.q.x, ms.q.y, ms.q.z};
          quaternionToEuler(qd, rpy_r, rpy_p, rpy_y);
        }
        std::printf("[ms] t=%5.2f alt=%7.1f pitch=%6.1f alpha=%6.1f qbar=%8.0f T=%5.0f v=%6.1f imp=%d\n",
                    ms.time_s, -ms.pos[2], rpy_p * 57.3f, ms.alpha_rad * 57.3f,
                    ms.qbar_pa, ms.thrust_n,
                    std::sqrt(ms.vel[0]*ms.vel[0]+ms.vel[1]*ms.vel[1]+ms.vel[2]*ms.vel[2]),
                    ms.impacted ? 1 : 0);
      }
#ifdef EKF_NAV_TELEMETRY
      std::printf("[nees] t=%5.1f ep3d=%6.2f sig_rms=%6.2f nees=%7.2f\n",
                  t, static_cast<double>(nees_dbg_ep),
                  static_cast<double>(nees_dbg_sig),
                  static_cast<double>(nees_dbg_val));
#endif
    }
  }

  const double rms_roll = n_att ? std::sqrt(sum_roll2 / n_att) : 0.0;
  const double rms_pitch = n_att ? std::sqrt(sum_pitch2 / n_att) : 0.0;
#ifdef EKF_NAV_TELEMETRY
  auto nees_quantile = [](std::vector<float> v, const double q) -> double {
    if (v.empty()) return -1.0;
    std::sort(v.begin(), v.end());
    const double pos = q * (static_cast<double>(v.size()) - 1.0);
    const size_t i = static_cast<size_t>(pos);
    const double frac = pos - static_cast<double>(i);
    const size_t j = std::min(i + 1, v.size() - 1);
    return static_cast<double>(v[i]) * (1.0 - frac) +
           static_cast<double>(v[j]) * frac;
  };
  char nees_json[384];
  if (!nees_pos_samples.empty() && !nees_vel_samples.empty())
  {
    std::snprintf(nees_json, sizeof(nees_json),
                  ",\"nees_pos_med\":%.3f,\"nees_pos_p95\":%.3f,"
                  "\"nees_pos_max\":%.3f,\"nees_vel_med\":%.3f,"
                  "\"nees_vel_p95\":%.3f,\"nees_vel_max\":%.3f,\"nees_n\":%d",
                  nees_quantile(nees_pos_samples, 0.5),
                  nees_quantile(nees_pos_samples, 0.95),
                  nees_quantile(nees_pos_samples, 1.0),
                  nees_quantile(nees_vel_samples, 0.5),
                  nees_quantile(nees_vel_samples, 0.95),
                  nees_quantile(nees_vel_samples, 1.0),
                  static_cast<int>(nees_pos_samples.size()));
  }
  else
  {
    nees_json[0] = '\0';
  }
#else
  const char *nees_json = "";
#endif
  const double pos_rms = n_pos ? std::sqrt(sum_pos2 / n_pos) : -1.0;
  const double nav_alt_mean = n_nav_alt ? (sum_nav_alt / n_nav_alt) : 0.0;
  const double hold_rms = n_hold ? std::sqrt(sum_hold2 / n_hold) : -1.0;
  const double hold_end_rms = n_hold_end ? std::sqrt(sum_hold_end2 / n_hold_end) : -1.0;
  std::printf("[sil_real] profile=%d seed=%d final_alt=%.2f nav_alt_mean=%.2f "
              "pos_rms=%.2f pos_max=%.2f hold_rms=%.2f hold_max=%.2f hold_end_rms=%.2f "
              "max_roll=%.2f max_pitch=%.2f rms_roll=%.2f rms_pitch=%.2f "
              "nav_init=%d fix3d=%u quality_fail=%u tow_mismatch=%u duplicate=%u "
              "reacquire_resets=%u horiz_reacq=%u vel_sub_rec=%u "
              "outlier_rejects=%u mag_fused=%u mag_rej=%u "
              "mag_gate=%u mag_cal=%d mag_norm=%.2f mag_hdg=%.2f "
              "grav_att=%u grav_fused=%u att_att=%u att_fused=%u "
              "mag_kick=%.2f mag_kickrp=%.2f "
              "imu_state=%d imu_frz=%.2f imu_drop=%.2f imu_failed=%d "
              "imu_detected=%d imu_inv=%d imu_flags=%u pre_st=%d pre_ok=%d "
              "pre_acc=%.2f pre_astd=%.3f pre_gyr=%.3f pre_gstd=%.3f "
              "gnss_susp=%d gnss_crej=%u gnss_cres=%.3f\n",
              cfg.profile, seed, -p.pos[2], nav_alt_mean, pos_rms, max_pos,
              hold_rms, max_hold, hold_end_rms, max_roll, max_pitch, rms_roll, rms_pitch,
              nav_system_initialized ? 1 : 0,
              gnss_status.fix3d_count, gnss_status.quality_fail_count,
              gnss_status.tow_mismatch_count, gnss_status.duplicate_itow_count,
              nav_ekf.gnss_reacquire_reset_count(),
              nav_ekf.gnss_horizontal_reanchor_count(),
              nav_ekf.gnss_velocity_subset_recovery_count(),
              nav_ekf.gnss_outlier_reject_count(),
              nav_ekf.mag_vector_fused_count(),
              nav_ekf.mag_vector_reject_count(),
              mag_status.gate_rejected, mag_status.cal_valid ? 1 : 0,
              mag_status.norm_ut, mag_status.heading_error_deg,
              aid_tracker.status(bfs::AidSource::Gravity).attempt_count,
              aid_tracker.status(bfs::AidSource::Gravity).fuse_count,
              aid_tracker.status(bfs::AidSource::Attitude).attempt_count,
              aid_tracker.status(bfs::AidSource::Attitude).fuse_count,
              max_mag_kick, max_mag_kick_rp,
              static_cast<int>(g_imu_health_state), g_imu_health_frozen_s,
              g_imu_health_no_sample_s, g_sil_nav_debug_imu_failed ? 1 : 0,
              imu_fault_detected_any ? 1 : 0,
              g_sil_nav_debug_imu_invalid, g_imu_health_flags,
              g_sil_pre_stable ? 1 : 0, g_sil_pre_ok ? 1 : 0,
              g_sil_pre_accel_norm, g_sil_pre_accel_std,
              g_sil_pre_gyro_norm, g_sil_pre_gyro_std,
              gnss_status.consistency_suspect ? 1 : 0,
              gnss_status.consistency_reject_count,
              gnss_status.consistency_accel_res);
  std::printf("SIL_REAL_JSON {\"profile\":%d,\"seed\":%d,\"final_alt\":%.3f,"
              "\"nav_alt_mean\":%.3f,\"pos_rms\":%.3f,\"pos_max\":%.3f,"
              "\"hold_rms\":%.3f,\"hold_max\":%.3f,\"hold_end_rms\":%.3f,"
              "\"max_roll\":%.3f,"
              "\"max_pitch\":%.3f,\"rms_roll\":%.3f,\"rms_pitch\":%.3f,"
              "\"nav_init\":%d,\"fix3d\":%u,\"gnss_age\":%.3f,"
              "\"gnss_stale_peak_s\":%.3f,"
              "\"gnss_tmap_invalid\":%u,\"quality_fail\":%u,"
              "\"tow_mismatch\":%u,\"duplicate\":%u,\"reacquire_resets\":%u,"
              "\"horiz_reacq\":%u,\"vel_sub_rec\":%u,"
              "\"outlier_rejects\":%u,\"mag_fused\":%u,\"mag_rej\":%u,"
              "\"mag_gate\":%u,\"mag_cal\":%d,\"mag_norm\":%.3f,"
              "\"att_att\":%u,\"att_fused\":%u,"
              "\"mag_hdg\":%.3f,\"imu_state\":%d,\"imu_frz\":%.3f,"
              "\"imu_drop\":%.3f,\"imu_failed\":%d,\"imu_detected\":%d,"
              "\"imu_inv\":%d,\"imu_flags\":%u,"
              "\"pre_stable\":%d,\"pre_ok\":%d,\"pre_acc\":%.2f,"
              "\"pre_astd\":%.3f,\"pre_gyr\":%.3f,\"pre_gstd\":%.3f,"
              "\"gnss_susp\":%d,\"gnss_crej\":%u,\"gnss_cres\":%.3f,"
              "\"vgsus\":%d,\"vgfr\":%u,"
              "\"bh_sus\":%d,\"bh_flags\":%u,\"bh_skips\":%u,\"bh_rate\":%.3f,"
              "\"bh_fcum\":%u,\"bh_nsus\":%u,"
              "\"heading_aiding\":%d,\"heading_age\":%.2f,"
              "\"f3\":%d,\"f4\":%d,\"pos_degraded\":%d,"
              "\"mode_degraded_gnss\":%d,"
              "\"land_t\":%.2f,\"land_hold\":%.2f,\"land_pos_err\":%.2f,"
              "\"land_roll\":%.2f,\"max_roll_preland\":%.2f,"
              "\"land_vz_down\":%.3f,\"max_desc_vz_down\":%.3f,"
              "\"alt_at_outage\":%.2f,"
              "\"baro_vel_attempts\":%u,\"baro_vel_fused\":%u,"
              "\"baro_vel_rejected\":%u,"
              "\"ad_tau_ms\":%.1f,\"ad_sharp\":%.4f,\"ad_valid\":%d,"
              "\"ad_resid\":%.3f,"
              "\"ad_rej\":%u,\"ad_ng\":%u,\"ad_er\":%u,\"ad_smax\":%.1f,"
              "\"ad_app\":%u,\"ad_blk\":%u,"
              "\"ms_vmax\":%.1f,\"ms_altmax\":%.1f,\"ms_gmax\":%.2f,"
              "\"ms_rate_max\":%.1f,\"ms_qbar_max\":%.0f,"
              "\"ms_impact_t\":%.2f,"
              "\"ms_final_n\":%.1f,\"ms_final_e\":%.1f,\"ms_final_d\":%.1f,"
              "\"nav_att_rmse\":%.3f,\"nav_att_max\":%.3f,"
              "\"nav_roll_rmse\":%.3f,\"nav_roll_max\":%.3f,"
              "\"nav_pitch_rmse\":%.3f,\"nav_pitch_max\":%.3f,"
              "\"nav_yaw_rmse\":%.3f,\"nav_yaw_max\":%.3f,"
              "\"nav_vel_rms\":%.4f,\"nrfb\":%u,\"nav_bias_frames\":%u,"
              "\"coax_motor_tau_s\":%.3f,\"coax_servo_rate_dps\":%.1f,"
              "\"coax_ground_mu\":%.3f,\"coax_servojam\":%d,"
              "\"coax_loss1\":%.3f,\"coax_loss2\":%.3f,\"coax_ix\":%.4f,\"coax_inertia_scale\":%.3f,\"coax_thrust_scale\":%.3f,\"coax_yaw_scale\":%.3f,\"coax_alloc_mode\":%d,\"rc_profile_rows\":%d,\"rc_profile_t_first\":%.3f,\"rc_profile_t_last\":%.3f%s}\n",
              cfg.profile, seed, -p.pos[2], nav_alt_mean, pos_rms, max_pos,
              hold_rms, max_hold, hold_end_rms, max_roll, max_pitch, rms_roll, rms_pitch,
              nav_system_initialized ? 1 : 0,
              gnss_status.fix3d_count,
              gnss_status.last_measurement_age_s,
              max_gnss_age,
              gnss_status.time_mapping_invalid_count,
              gnss_status.quality_fail_count,
              gnss_status.tow_mismatch_count, gnss_status.duplicate_itow_count,
              nav_ekf.gnss_reacquire_reset_count(),
              nav_ekf.gnss_horizontal_reanchor_count(),
              nav_ekf.gnss_velocity_subset_recovery_count(),
              nav_ekf.gnss_outlier_reject_count(),
              nav_ekf.mag_vector_fused_count(),
              nav_ekf.mag_vector_reject_count(),
              mag_status.gate_rejected, mag_status.cal_valid ? 1 : 0,
              mag_status.norm_ut,
              aid_tracker.status(bfs::AidSource::Attitude).attempt_count,
              aid_tracker.status(bfs::AidSource::Attitude).fuse_count,
              mag_status.heading_error_deg,
              static_cast<int>(g_imu_health_state), g_imu_health_frozen_s,
              g_imu_health_no_sample_s, g_sil_nav_debug_imu_failed ? 1 : 0,
              imu_fault_detected_any ? 1 : 0,
              g_sil_nav_debug_imu_invalid, g_imu_health_flags,
              g_sil_pre_stable ? 1 : 0, g_sil_pre_ok ? 1 : 0,
              g_sil_pre_accel_norm, g_sil_pre_accel_std,
              g_sil_pre_gyro_norm, g_sil_pre_gyro_std,
              gnss_status.consistency_suspect ? 1 : 0,
              gnss_status.consistency_reject_count,
              gnss_status.consistency_accel_res,
              g_gnss_vert_suspect ? 1 : 0,
              g_gnss_vert_gate_frames,
              g_baro_health_suspect ? 1 : 0,
              g_baro_health_flags,
              g_baro_health_skips,
              static_cast<double>(g_baro_health_drift_mps),
              g_baro_health_flags_cum,
              g_baro_health_sus_count,
              g_heading_aiding_valid ? 1 : 0,
              static_cast<double>(g_heading_aiding_age_s),
              g_actuator_thrust_loss ? 1 : 0,
              g_actuator_servo_jam ? 1 : 0,
              g_mode_degraded_heading ? 1 : 0,
              gnss_degrade_seen ? 1 : 0,
              static_cast<double>(land_t),
              static_cast<double>(land_hold),
              static_cast<double>(land_pos_err),
              static_cast<double>(land_roll_deg),
              static_cast<double>(max_roll_preland),
              static_cast<double>(land_vz_down),
              static_cast<double>(max_desc_vz_down),
              static_cast<double>(alt_at_outage),
              g_baro_vel_aid_attempts, g_baro_vel_aid_fused,
              g_baro_vel_aid_rejected,
              static_cast<double>(gnss_status.auto_delay_tau_s * 1000.0f),
              static_cast<double>(gnss_status.auto_delay_sharpness),
              gnss_status.auto_delay_valid ? 1 : 0,
              static_cast<double>(gnss_status.auto_delay_residual_mps),
              static_cast<unsigned>(gnss_status.auto_delay_reject),
              static_cast<unsigned>(gnss_status.auto_delay_n_gnss),
              static_cast<unsigned>(gnss_status.auto_delay_n_erased),
              static_cast<double>(gnss_status.auto_delay_speed_max_mps),
              static_cast<unsigned>(gnss_status.auto_delay_applied_count),
              static_cast<unsigned>(gnss_status.auto_delay_apply_block),
              static_cast<double>(missile.state().max_speed_mps),
              static_cast<double>(missile.state().max_alt_m),
              static_cast<double>(missile.state().max_load_g),
              static_cast<double>(missile.state().max_rate_dps),
              static_cast<double>(missile.state().max_qbar_pa),
              static_cast<double>(missile.state().impact_t),
              static_cast<double>(p.pos[0]), static_cast<double>(p.pos[1]),
              static_cast<double>(p.pos[2]),
              n_nav_att ? std::sqrt(sum_nav_att2 / n_nav_att) * 57.29578 : -1.0,
              max_nav_att * 57.29578,
              n_nav_att ? std::sqrt(sum_nav_roll2 / n_nav_att) : -1.0,
              max_nav_roll,
              n_nav_att ? std::sqrt(sum_nav_pitch2 / n_nav_att) : -1.0,
              max_nav_pitch,
              n_nav_att ? std::sqrt(sum_nav_yaw2 / n_nav_att) : -1.0,
              max_nav_yaw,
              n_nav_att ? std::sqrt(sum_nav_vel2 / n_nav_att) : -1.0,
              static_cast<unsigned>(g_nav_raw_fb_frames),
              static_cast<unsigned>(g_nav_bias_frames),
#ifdef FW_SIL_COAXIAL
              static_cast<double>((g_coax_motor_tau_s >= 0.0f) ? g_coax_motor_tau_s : 0.28f),
              static_cast<double>((g_coax_servo_rate_dps >= 0.0f) ? g_coax_servo_rate_dps : 300.0f),
              static_cast<double>(g_coax_ground_mu),
              g_coax_servojam,
              static_cast<double>(g_coax_cli_loss1),
              static_cast<double>(g_coax_cli_loss2),
              static_cast<double>(g_coax_ix),
              static_cast<double>(g_coax_inertia_scale),
              static_cast<double>(g_coax_thrust_scale),
              static_cast<double>(g_coax_yaw_scale),
              g_coax_alloc_mode,
              g_rc_profile_rows,
              (g_rc_profile_rows > 0) ? g_rc_profile[0].t : 0.0,
              (g_rc_profile_rows > 0)
                  ? g_rc_profile[g_rc_profile_rows - 1].t
                  : 0.0,
#else
              0.28, 300.0, 0.0, 0, 1.0, 1.0, 0.05, 1.0, 1.0, 1.0, 0,
              0, 0.0, 0.0,
#endif
              nees_json);
  if (g_trace_csv_file != nullptr)
  {
    std::fclose(g_trace_csv_file);
    g_trace_csv_file = nullptr;
  }
  if (g_trace_ctrl_file != nullptr)
  {
    std::fclose(g_trace_ctrl_file);
    g_trace_ctrl_file = nullptr;
  }
  if (g_trace_nav_file != nullptr)
  {
    std::fclose(g_trace_nav_file);
    g_trace_nav_file = nullptr;
  }
  if (g_trace_fast_file != nullptr)
  {
    std::fclose(g_trace_fast_file);
    g_trace_fast_file = nullptr;
  }
  return 0;
}
#endif  // FW_SIL_REAL_NAV

int main(int argc, char** argv) {
  int seed=1;
  bool manual=false;
  bool coupled=false;   // 同时 roll/pitch 激励 + 油门脉冲，用于 C-02 前馈 A/B
  int fault=0; // 0=none, 3=thrust loss(单发), 4=servo jam, 5=both motors loss
  float fault_at=8.0f;  // 故障注入时刻 [s]（默认 8；定高需先爬升时可设 20）
  float loss=0.70f;     // fault 3/5 的推力保留比例（0.9=损失10%）
  float vib_hf_enter_g=-1.0f; // 高频振动否决进入门限 [g]（-1=用宏默认 0.15）
  float target_alt=10.0f; // AUTO_ALTITUDE 目标高度 [m]（F3 验证可调高）
  float seconds=40.0f;    // 仿真时长 [s]
  int mask=0x03;        // inertia_comp_mask，0 关闭前馈 / 3 全开（C-02 A/B）
  float gain_scale=1.0f; // 测试用：att/rate kp 缩放（默认 1.0，不影响生产）
  float rate_gain_scale=1.0f; // 测试用：仅 rate kp 缩放（覆盖 gain_scale）
  float att_gain_scale=1.0f;  // 测试用：仅 att kp 缩放（覆盖 gain_scale）
  float ang_alpha=-1.0f;      // 测试用：姿态外环输出滤波 alpha（<0 用参数默认）
  int heading_aid_mode=0;     // 0=磁连续；1=磁仅初始对准；2=无磁手动对准
  float heading_align_deg=0.0f; // 无磁/初始对准的机头真北航向 [deg]
  float pos_leg_n_m=0.0f;     // SIL 前飞段目标北向偏移 [m]
  float pos_leg_e_m=0.0f;     // SIL 前飞段目标东向偏移 [m]
  bool rate_gain_scale_set=false;
  bool att_gain_scale_set=false;
  bool scale_ki=false;   // 同时缩放 rate ki
#ifdef BFS_RATE_ACCEL_FB
  float aaf_gain=-1.0f;      // 测试用：AAF 滚转/俯仰增益覆盖（<0 用编译期默认）
  float aaf_yaw_gain=-1.0f;  // 测试用：AAF 偏航增益覆盖（可设 0 关闭偏航）
  float aaf_tau=-1.0f;       // 测试用：差分低通时间常数覆盖（<0 用默认 0.02 s）
#endif
#ifdef BFS_RATE_PHASE_LEAD
  float pl_gain=-1.0f;
  float pl_yaw_gain=-1.0f;
  float pl_tau=-1.0f;
#endif
  bool flow=false;
  bool wind_on=true;    // --no-wind 关闭 20–25 s 侧风（F3 洁净场景用）
#ifdef FW_SIL_REAL_NAV
  RealNavConfig real_cfg;
#endif
  for (int i=1;i<argc;++i) {
    if (std::strcmp(argv[i],"--manual")==0) manual=true;
    else if (std::strcmp(argv[i],"--coupled")==0) coupled=true;
    else if (std::strcmp(argv[i],"--ekf")==0) g_use_ekf=true;
    else if (std::strcmp(argv[i],"--no-wind")==0) wind_on=false;
    else if (std::strcmp(argv[i],"--fault")==0 && i+1<argc) fault=std::atoi(argv[++i]);
    else if (std::strcmp(argv[i],"--fault-at")==0 && i+1<argc) fault_at=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--loss")==0 && i+1<argc) loss=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--alt")==0 && i+1<argc) target_alt=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--seconds")==0 && i+1<argc) seconds=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--mask")==0 && i+1<argc) mask=std::atoi(argv[++i]);
    else if (std::strcmp(argv[i],"--gain-scale")==0 && i+1<argc) gain_scale=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--rate-gain-scale")==0 && i+1<argc)
    {
      rate_gain_scale=std::atof(argv[++i]);
      rate_gain_scale_set=true;
    }
    else if (std::strcmp(argv[i],"--att-gain-scale")==0 && i+1<argc)
    {
      att_gain_scale=std::atof(argv[++i]);
      att_gain_scale_set=true;
    }
    else if (std::strcmp(argv[i],"--ang-alpha")==0 && i+1<argc)
      ang_alpha=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--heading-aid-mode")==0 && i+1<argc)
      heading_aid_mode=std::atoi(argv[++i]);
    else if (std::strcmp(argv[i],"--heading-align-deg")==0 && i+1<argc)
      heading_align_deg=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--calib-hold")==0 && i+1<argc)
      g_calib_hold_s=static_cast<float>(std::atof(argv[++i]));
    else if (std::strcmp(argv[i],"--ekf-q-scale")==0 && i+1<argc)
      g_ekf_q_accel_scale=static_cast<float>(std::atof(argv[++i]));
    else if (std::strcmp(argv[i],"--static-calib-s")==0 && i+1<argc)
      g_static_calib_required_s=static_cast<float>(std::atof(argv[++i]));
    else if (std::strcmp(argv[i],"--bias-z-max")==0 && i+1<argc)
      g_nomag_bias_z_max_radps=std::atof(argv[++i]); // 0=关闭限幅（消融用）
    else if (std::strcmp(argv[i],"--pos-leg")==0 && i+1<argc)
      std::sscanf(argv[++i], "%f:%f", &pos_leg_n_m, &pos_leg_e_m);
    else if (std::strcmp(argv[i],"--scale-ki")==0) scale_ki=true;
#ifdef BFS_RATE_ACCEL_FB
    else if (std::strcmp(argv[i],"--aaf-gain")==0 && i+1<argc) aaf_gain=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--aaf-yaw-gain")==0 && i+1<argc) aaf_yaw_gain=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--aaf-tau")==0 && i+1<argc) aaf_tau=std::atof(argv[++i]);
#endif
#ifdef BFS_RATE_PHASE_LEAD
    else if (std::strcmp(argv[i],"--pl-gain")==0 && i+1<argc) pl_gain=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--pl-yaw-gain")==0 && i+1<argc) pl_yaw_gain=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--pl-tau")==0 && i+1<argc) pl_tau=std::atof(argv[++i]);
#endif
    else if (std::strcmp(argv[i],"--flow")==0) flow=true;
    else if (std::strcmp(argv[i],"--flow-delay")==0 && i+1<argc) g_flow_delay=std::atoi(argv[++i]);
    else if (std::strcmp(argv[i],"--flow-dropout")==0 && i+1<argc)
      std::sscanf(argv[++i], "%f:%f", &real_cfg.flow_dropout_start,
                  &real_cfg.flow_dropout_end);
    else if (std::strcmp(argv[i],"--mag")==0) g_use_mag_yaw=true;
    else if (std::strcmp(argv[i],"--seed")==0 && i+1<argc) seed=std::atoi(argv[++i]);
#ifdef FW_SIL_REAL_NAV
    else if (std::strcmp(argv[i],"--mag-noise")==0 && i+1<argc) real_cfg.mag_noise_ut=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--mag-yaw-jump")==0 && i+1<argc)
      g_mag_yaw_jump_rad = std::atof(argv[++i]) * 0.0174532925f;
    else if (std::strcmp(argv[i],"--mag-disturb")==0 && i+1<argc) real_cfg.mag_disturb_ut=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--mag-dropout")==0 && i+1<argc)
      std::sscanf(argv[++i], "%f:%f", &real_cfg.mag_dropout_start,
                  &real_cfg.mag_dropout_end);
    else if (std::strcmp(argv[i],"--mag-cal-invalid")==0) real_cfg.mag_cal_invalid=true;
    else if (std::strcmp(argv[i],"--real-nav")==0) real_cfg.enabled=true;
    else if (std::strcmp(argv[i],"--trace-csv")==0 && i+1<argc)
      g_trace_csv_path = argv[++i];
    else if (std::strcmp(argv[i],"--trace-nav")==0 && i+1<argc)
      g_trace_nav_path = argv[++i];
    else if (std::strcmp(argv[i],"--trace-ctrl")==0 && i+1<argc)
      g_trace_ctrl_path = argv[++i];
    else if (std::strcmp(argv[i],"--trace-fast")==0 && i+1<argc)
      g_trace_fast_path = argv[++i];
    else if (std::strcmp(argv[i],"--trace-fast-window")==0 && i+1<argc)
      std::sscanf(argv[++i], "%f:%f", &g_trace_fast_start_s,
                  &g_trace_fast_end_s);
    else if (std::strcmp(argv[i],"--plant-substeps")==0 && i+1<argc)
      g_plant_substeps = std::atoi(argv[++i]);
    else if (std::strcmp(argv[i],"--missile")==0)      g_missile_mode = true;
    else if (std::strcmp(argv[i],"--imu-grade")==0 && i+1<argc)
      g_imu_grade = argv[++i];
    else if (std::strcmp(argv[i],"--ms-thrust")==0 && i+1<argc)
      g_ms_thrust_n = std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--ms-mass")==0 && i+1<argc)
      g_ms_mass_kg = std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--ms-burn")==0 && i+1<argc)
      g_ms_burn_s = std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--missile-nav-loop")==0)
      g_missile_nav_loop = true;
    else if (std::strcmp(argv[i],"--imu-fs-g")==0 && i+1<argc)
      g_imu_fs_g = std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--imu-misalign-deg")==0 && i+1<argc)
    {
      g_imu_misalign_deg = std::atof(argv[++i]);
      g_imu_misalign_set = true;
    }
    else if (std::strcmp(argv[i],"--imu-health-accel-range")==0 && i+1<argc)
      g_sil_imu_health_accel_range_mps2 = std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--no-imu-lpf")==0)
      g_imu_lpf_enable = false;
    else if (std::strcmp(argv[i],"--no-sil-ahrs")==0)
      g_sil_madgwick_enable = false;
    else if (std::strcmp(argv[i],"--nav-raw-imu")==0)
      g_nav_raw_imu = true;
    else if (std::strcmp(argv[i],"--nav-raw-fault-fallback")==0)
      g_nav_raw_fault_fallback = true;
    else if (std::strcmp(argv[i],"--nav-raw-fb-health")==0)
      g_nav_raw_fb_use_health = true;
    else if (std::strcmp(argv[i],"--nav-raw-fb-delay")==0 && i+1<argc)
      g_nav_raw_fb_start_delay_s = std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--nav-raw-fb-hold")==0 && i+1<argc)
      g_nav_raw_fb_hold_s = std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--vib-start")==0 && i+1<argc)
      g_vib_start_s = std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--gyro-q-cap")==0 && i+1<argc)
      g_sil_gyro_q_cap_radps = std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--delay-deadband")==0 && i+1<argc)
      g_sil_delay_deadband_s = std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--vib-veto")==0 && i+1<argc)
      g_sil_delay_vib_veto = std::atoi(argv[++i]);
    else if (std::strcmp(argv[i],"--vib-hf-source")==0 && i+1<argc)
      g_sil_vib_hf_source = std::atoi(argv[++i]);
    else if (std::strcmp(argv[i],"--vib-hf-enter")==0 && i+1<argc)
      vib_hf_enter_g = std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--delay-max-gap")==0 && i+1<argc)
      g_sil_delay_max_gap_s = std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--delay-max-resid")==0 && i+1<argc)
      g_sil_delay_max_resid_mps = std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--delay-apply-lpf")==0 && i+1<argc)
      g_sil_delay_apply_lpf_s = std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--delay-apply-freeze")==0 && i+1<argc)
      g_sil_delay_apply_freeze = std::atoi(argv[++i]);
    else if (std::strcmp(argv[i],"--delay-grid-max-ms")==0 && i+1<argc)
      g_sil_delay_grid_max_ms = std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--delay-cap-ms")==0 && i+1<argc)
      g_sil_delay_cap_ms = std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--vib-iso-fn")==0 && i+1<argc)
      g_vib_iso_fn_hz = std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--vib-iso-zeta")==0 && i+1<argc)
      g_vib_iso_zeta = std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--sat-q-inflate")==0 && i+1<argc)
    {
      ++i;
#ifdef BFS_IMU_SATURATION_Q_INFLATE
      g_sil_imu_sat_q_inflate_mps2 =
          static_cast<float>(std::atof(argv[i]));
#endif
    }
    else if (std::strcmp(argv[i],"--sat-q-enter")==0 && i+1<argc)
    {
      ++i;
#ifdef BFS_IMU_SATURATION_Q_INFLATE
      g_sil_imu_sat_q_enter_s = static_cast<float>(std::atof(argv[i]));
#endif
    }
    else if (std::strcmp(argv[i],"--sat-q-release")==0 && i+1<argc)
    {
      ++i;
#ifdef BFS_IMU_SATURATION_Q_INFLATE
      g_sil_imu_sat_q_release_s = static_cast<float>(std::atof(argv[i]));
#endif
    }
    else if (std::strcmp(argv[i],"--profile")==0 && i+1<argc)
      applyRealNavProfile(real_cfg, argv[++i]);
#ifdef FW_SIL_COAXIAL
    else if (std::strcmp(argv[i],"--coax-mass")==0 && i+1<argc)
      g_coax_mass_kg = std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--coax-thrust")==0 && i+1<argc)
      g_coax_max_thrust_n = std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--coax-lever")==0 && i+1<argc)
      g_coax_lever_m = std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--coax-iy")==0 && i+1<argc)
      g_coax_iy = std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--coax-iz")==0 && i+1<argc)
      g_coax_iz = std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--coax-yawk")==0 && i+1<argc)
      g_coax_yaw_nm_per_norm = std::atof(argv[++i]);
    // —— 分配器/失配激励（task-11，2026-10-31）——
    // 均为 plant 侧激励，不写固件增益；缺省值 = 改动前数值（逐位一致）。
    else if (std::strcmp(argv[i],"--coax-ix")==0 && i+1<argc)
      g_coax_ix = static_cast<float>(std::atof(argv[++i]));  // 独立于 --coax-iy
    else if (std::strcmp(argv[i],"--coax-inertia-scale")==0 && i+1<argc)
      g_coax_inertia_scale = static_cast<float>(std::atof(argv[++i]));
    else if (std::strcmp(argv[i],"--coax-thrust-scale")==0 && i+1<argc)
      g_coax_thrust_scale = static_cast<float>(std::atof(argv[++i]));
    else if (std::strcmp(argv[i],"--coax-yaw-scale")==0 && i+1<argc)
      g_coax_yaw_scale = static_cast<float>(std::atof(argv[++i]));
    else if (std::strcmp(argv[i],"--coax-alloc-mode")==0 && i+1<argc)
    {
      const int m = std::atoi(argv[++i]);
      if (m != 0 && m != 1)
      {
        std::fprintf(stderr,
                     "[sil] --coax-alloc-mode 只接受 0(legacy)|1(α+分配器)，"
                     "收到 %d\n", m);
        return 2;
      }
      g_coax_alloc_mode = m;
    }
    // --rc-profile：CSV 遥控激励表（行间线性插值，覆盖摇杆 + CH7/CH9）
    else if (std::strcmp(argv[i],"--rc-profile")==0 && i+1<argc)
    {
      if (!rcProfileLoad(argv[++i]))
      {
        return 2;
      }
    }
    // —— 可切保真度开关（task-5，2026-10-30）——
    // 预设按 argv 顺序“就地”应用：--coax-realistic 之后显式给的参数覆盖预设，
    // 之前给的会被预设覆盖（与命令行选项顺序语义一致）。
    else if (std::strcmp(argv[i],"--coax-realistic")==0)
    {
      // L1 建议值（非实机辨识值；物理含义与依据见 README「共轴 plant 保真度开关」）：
      // 电机一阶 τ=0.28 s、舵机 300 °/s + 0.1% PWM 量化、地面 μ=0.5。
      // 注意：执行器动态与 0.1% 量化在改动前就硬编码在 SIL 中，属基线；
      // 预设真正新增的只有“落地后水平摩擦”。
      g_coax_motor_tau_s = 0.28f;
      g_coax_servo_rate_dps = 300.0f;
      g_coax_ground_mu = 0.5f;
    }
    else if (std::strcmp(argv[i],"--coax-motor-tau")==0 && i+1<argc)
    {
      const float v = static_cast<float>(std::atof(argv[++i]));
      if (v < 0.0f)
      {
        std::fprintf(stderr, "[sil] --coax-motor-tau 需 ≥0，收到 %g\n", v);
        return 2;
      }
      g_coax_motor_tau_s = v;  // 0 = 立即跟踪（理想电机）
    }
    else if (std::strcmp(argv[i],"--coax-servo-rate")==0 && i+1<argc)
    {
      const float v = static_cast<float>(std::atof(argv[++i]));
      if (v < 0.0f)
      {
        std::fprintf(stderr, "[sil] --coax-servo-rate 需 ≥0，收到 %g\n", v);
        return 2;
      }
      g_coax_servo_rate_dps = v;  // 0 = 无限速率（理想舵机）
    }
    else if (std::strcmp(argv[i],"--coax-ground-mu")==0 && i+1<argc)
      g_coax_ground_mu = static_cast<float>(std::atof(argv[++i]));
    else if (std::strcmp(argv[i],"--coax-motor-loss1")==0 && i+1<argc)
      g_coax_cli_loss1 = static_cast<float>(std::atof(argv[++i]));
    else if (std::strcmp(argv[i],"--coax-motor-loss2")==0 && i+1<argc)
      g_coax_cli_loss2 = static_cast<float>(std::atof(argv[++i]));
    else if (std::strcmp(argv[i],"--coax-servojam")==0 && i+1<argc)
    {
      const char *m = argv[++i];
      if (std::strcmp(m, "roll") == 0) g_coax_servojam = 1;       // 前摆/上摆
      else if (std::strcmp(m, "pitch") == 0) g_coax_servojam = 2;  // 尾摆/下摆
      else if (std::strcmp(m, "none") == 0) g_coax_servojam = 0;
      else
      {
        std::fprintf(stderr,
                     "[sil] --coax-servojam 只接受 roll|pitch|none，收到 %s\n", m);
        return 2;
      }
    }
    else if (std::strcmp(argv[i],"--coax-servojam-at")==0 && i+1<argc)
    {
      const float v = static_cast<float>(std::atof(argv[++i]));
      if (v < 0.0f)
      {
        std::fprintf(stderr, "[sil] --coax-servojam-at 需 ≥0，收到 %g\n", v);
        return 2;
      }
      g_coax_servojam_at_s = v;
    }
#endif
    else if (std::strcmp(argv[i],"--vib")==0 && i+1<argc) real_cfg.vib_g=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--vib-hz")==0 && i+1<argc) real_cfg.vib_hz=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--vib-mode")==0 && i+1<argc) real_cfg.vib_mode=std::atoi(argv[++i]);
    else if (std::strcmp(argv[i],"--no-anti-alias")==0) real_cfg.anti_alias=false;
    else if (std::strcmp(argv[i],"--anti-alias")==0) real_cfg.anti_alias=true;
    else if (std::strcmp(argv[i],"--gnss-latency-ms")==0 && i+1<argc)
      real_cfg.gnss_latency_s=std::atof(argv[++i])*0.001f;
    else if (std::strcmp(argv[i],"--gnss-rate-hz")==0 && i+1<argc)
      real_cfg.gnss_rate_hz=static_cast<float>(std::atof(argv[++i]));
    else if (std::strcmp(argv[i],"--gnss-pos-std")==0 && i+1<argc)
      real_cfg.gnss_pos_ne_std=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--gnss-vel-std")==0 && i+1<argc)
      real_cfg.gnss_vel_ne_std=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--gnss-white")==0)
      real_cfg.gnss_colored=false;
    else if (std::strcmp(argv[i],"--gnss-vel-subset")==0 && i+1<argc)
      g_gnss_vel_subset_recovery_enable = std::atoi(argv[++i]) != 0;
    else if (std::strcmp(argv[i],"--gnss-vel-subset-posres")==0 && i+1<argc)
      g_gnss_vel_subset_max_pos_resid_m = std::atof(argv[++i]);  // §4.24q 同二进制 A/B
    else if (std::strcmp(argv[i],"--gnss-vel-subset-maxfires")==0 && i+1<argc)
      g_gnss_vel_subset_max_fires_per_epoch = std::atoi(argv[++i]);  // §4.24w 同二进制 A/B
    else if (std::strcmp(argv[i],"--gnss-v-gate")==0 && i+1<argc)
      g_gnss_v_gate_enable = std::atoi(argv[++i]) != 0;
    else if (std::strcmp(argv[i],"--gnss-r-scale")==0 && i+1<argc)
      real_cfg.gnss_r_scale=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--gnss-lever-inject")==0 && i+1<argc)
    {
      // 天线相对 IMU 的机体系杆臂 [m]（SIL 注入侧，逗号分隔）
      if (std::sscanf(argv[++i], "%f,%f,%f", real_cfg.gnss_lever_inject,
                      real_cfg.gnss_lever_inject + 1,
                      real_cfg.gnss_lever_inject + 2) != 3)
      {
        std::fprintf(stderr, "[sil] bad --gnss-lever-inject: %s\n", argv[i]);
      }
    }
    else if (std::strcmp(argv[i],"--gnss-lever")==0 && i+1<argc)
    {
      // 固件补偿参数（kFlightCtrlParams.gnss_lever_*，逗号分隔）
      if (std::sscanf(argv[++i], "%f,%f,%f", &g_gnss_lever_param[0],
                      &g_gnss_lever_param[1], &g_gnss_lever_param[2]) != 3)
      {
        std::fprintf(stderr, "[sil] bad --gnss-lever: %s\n", argv[i]);
      }
    }
    else if (std::strcmp(argv[i],"--gnss-outlier")==0 && i+1<argc)
      real_cfg.outlier_prob=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--gnss-spoof")==0 && i+1<argc)
    {
      real_cfg.gnss_spoof = true;
      real_cfg.spoof_offset_m = std::atof(argv[++i]);
    }
    else if (std::strcmp(argv[i],"--gnss-spoof-ramp")==0 && i+1<argc)
      real_cfg.spoof_ramp_mps=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--gnss-spoof-start")==0 && i+1<argc)
      real_cfg.spoof_start_s=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--gnss-spoof-z")==0 && i+1<argc)
    {
      real_cfg.gnss_spoof = true;
      real_cfg.spoof_offset_z_m=std::atof(argv[++i]);
    }
    else if (std::strcmp(argv[i],"--gnss-spoof-z-ramp")==0 && i+1<argc)
    {
      real_cfg.gnss_spoof = true;
      real_cfg.spoof_ramp_z_mps=std::atof(argv[++i]);
    }
    else if (std::strcmp(argv[i],"--gnss-outage")==0 && i+1<argc)
      std::sscanf(argv[++i], "%f:%f", &real_cfg.outage_start, &real_cfg.outage_end);
    else if (std::strcmp(argv[i],"--baro-noise")==0 && i+1<argc)
      real_cfg.baro_noise=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--baro-drift")==0 && i+1<argc)
      real_cfg.baro_drift=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--baro-vel-aid")==0 && i+1<argc)
      g_baro_vel_aid_enable = std::atoi(argv[++i]) != 0;
    else if (std::strcmp(argv[i],"--baro-health-gate")==0 && i+1<argc)
      g_baro_health_gate_enable = std::atoi(argv[++i]) != 0;
    else if (std::strcmp(argv[i],"--flare")==0 && i+1<argc)
      g_flare_enable = std::atoi(argv[++i]) != 0;
    else if (std::strcmp(argv[i],"--wind-force")==0 && i+1<argc)
      real_cfg.wind_force_mps2=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--initial-yaw-deg")==0 && i+1<argc)
      g_initial_yaw_deg=static_cast<float>(std::atof(argv[++i]));
    else if (std::strcmp(argv[i],"--takeoff-at")==0 && i+1<argc)
      g_takeoff_at_s=static_cast<float>(std::atof(argv[++i]));
    else if (std::strcmp(argv[i],"--wind-window")==0 && i+1<argc)
      std::sscanf(argv[++i], "%f:%f", &real_cfg.wind_start_s,
                  &real_cfg.wind_end_s);
    else if (std::strcmp(argv[i],"--no-baro-lag")==0) real_cfg.baro_lag=false;
    else if (std::strcmp(argv[i],"--nav-accel-bias-step")==0 && i+1<argc)
    {
      if (std::sscanf(argv[++i], "%f:%f:%f:%f:%f",
                      &real_cfg.nav_accel_bias_start_s,
                      &real_cfg.nav_accel_bias_end_s,
                      &real_cfg.nav_accel_bias_mps2[0],
                      &real_cfg.nav_accel_bias_mps2[1],
                      &real_cfg.nav_accel_bias_mps2[2]) != 5)
      {
        std::fprintf(stderr, "[sil] --nav-accel-bias-step 需要 t0:t1:ax:ay:az\n");
        return 2;
      }
    }
    else if (std::strcmp(argv[i],"--pos-hold")==0) real_cfg.pos_hold=true;
    else if (std::strcmp(argv[i],"--pos-hold-at")==0 && i+1<argc)
      real_cfg.pos_hold_at=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--fwd-stick")==0 && i+1<argc)
      std::sscanf(argv[++i], "%f:%f", &real_cfg.fwd_stick_start,
                  &real_cfg.fwd_stick_end);
    else if (std::strcmp(argv[i],"--no-flow")==0) real_cfg.flow=false;
    else if (std::strcmp(argv[i],"--rc-loss")==0 && i+1<argc)
    {
      real_cfg.rc_loss=true;
      std::sscanf(argv[++i], "%f:%f", &real_cfg.rc_loss_start, &real_cfg.rc_loss_end);
    }
    else if (std::strcmp(argv[i],"--imu-drop")==0 && i+1<argc)
      real_cfg.imu_drop_prob=std::atof(argv[++i]);
    else if (std::strcmp(argv[i],"--imu-fault")==0 && i+1<argc)
    {
      const char *m = argv[++i];
      real_cfg.imu_fault_mode = (std::strcmp(m, "stuck") == 0) ? 1 :
                                ((std::strcmp(m, "nan") == 0) ? 2 : 0);
    }
    else if (std::strcmp(argv[i],"--imu-fault-window")==0 && i+1<argc)
      std::sscanf(argv[++i], "%f:%f", &real_cfg.imu_fault_start,
                  &real_cfg.imu_fault_end);
    else if (std::strcmp(argv[i],"--imu-dropout")==0 && i+1<argc)
      std::sscanf(argv[++i], "%f:%f", &real_cfg.imu_dropout_start,
                  &real_cfg.imu_dropout_end);
    else if (std::strcmp(argv[i],"--gnss-consistency-reject")==0)
      real_cfg.gnss_consistency_reject=true;
#endif
    else
    {
      // 静默忽略未知参数会让"以为注入了故障、其实没有"的对照臂产出假证据
      // （实测：`--loss 5` 被当成推力保留比例 500% 且需 `--fault 3/5` 才生效，
      //  三组"失联"臂因此与洁净臂逐位相同）。一律硬失败。
      std::fprintf(stderr, "[sil] 未识别的参数: %s\n", argv[i]);
      return 2;
    }
  }
  if (loss <= 0.0f || loss > 1.0f)
  {
    std::fprintf(stderr,
                 "[sil] --loss 是推力保留比例 (0,1]，收到 %g；"
                 "GNSS 失联请用 --gnss-outage A:B\n", loss);
    return 2;
  }
#ifdef FW_SIL_COAXIAL
  // 共轴保真度开关范围校验：非法值立即失败，避免静默产出“以为开了、其实
  // 没开”的对照臂（与 --loss 校验同思路）。
  if (g_coax_cli_loss1 < 0.0f || g_coax_cli_loss1 > 1.0f ||
      g_coax_cli_loss2 < 0.0f || g_coax_cli_loss2 > 1.0f)
  {
    std::fprintf(stderr, "[sil] --coax-motor-loss1/2 是推力保留比例 [0,1]，"
                         "收到 %g / %g\n", g_coax_cli_loss1, g_coax_cli_loss2);
    return 2;
  }
  if (g_coax_ground_mu < 0.0f || g_coax_ground_mu > 5.0f)
  {
    std::fprintf(stderr, "[sil] --coax-ground-mu 应在 [0,5]，收到 %g\n",
                 g_coax_ground_mu);
    return 2;
  }
  if (g_coax_motor_tau_s > 5.0f || g_coax_servo_rate_dps > 2000.0f)
  {
    std::fprintf(stderr, "[sil] --coax-motor-tau 应 ≤5 s、--coax-servo-rate 应 "
                         "≤2000 °/s，收到 %g s / %g °/s\n",
                 g_coax_motor_tau_s, g_coax_servo_rate_dps);
    return 2;
  }
  // task-11：失配激励参数范围（超出物理合理区间的输入直接拒绝，rc=2）
  if (g_coax_ix <= 0.0f || g_coax_ix > 5.0f)
  {
    std::fprintf(stderr, "[sil] --coax-ix 应在 (0,5] kg·m²，收到 %g\n",
                 g_coax_ix);
    return 2;
  }
  if (g_coax_inertia_scale < 0.25f || g_coax_inertia_scale > 4.0f)
  {
    std::fprintf(stderr, "[sil] --coax-inertia-scale 应在 [0.25,4]，收到 %g\n",
                 g_coax_inertia_scale);
    return 2;
  }
  if (g_coax_thrust_scale < 0.5f || g_coax_thrust_scale > 1.5f)
  {
    std::fprintf(stderr, "[sil] --coax-thrust-scale 应在 [0.5,1.5]，收到 %g\n",
                 g_coax_thrust_scale);
    return 2;
  }
  if (g_coax_yaw_scale < 0.25f || g_coax_yaw_scale > 4.0f)
  {
    std::fprintf(stderr, "[sil] --coax-yaw-scale 应在 [0.25,4]，收到 %g\n",
                 g_coax_yaw_scale);
    return 2;
  }
#endif
  // 高频振动否决门限扫描（同二进制 runtime A/B）：只改进入门限，释放门限按比例缩放。
  if (vib_hf_enter_g > 0.0f)
  {
    g_ins_hf_vibration.SetHysteresis(vib_hf_enter_g, -1.0f);
  }
  const float rate_scale = rate_gain_scale_set ? rate_gain_scale : gain_scale;  const float att_scale = att_gain_scale_set ? att_gain_scale : gain_scale;
  if (att_scale != 1.0f || rate_scale != 1.0f)
  {
    auto scale_pid = [&](auto &q, float s) {
      q.kp *= s;
      if (scale_ki) q.ki *= s;
    };
    scale_pid(kFlightCtrlParams.att_roll, att_scale);
    scale_pid(kFlightCtrlParams.att_pitch, att_scale);
    scale_pid(kFlightCtrlParams.att_yaw, att_scale);
    scale_pid(kFlightCtrlParams.rate_roll, rate_scale);
    scale_pid(kFlightCtrlParams.rate_pitch, rate_scale);
    scale_pid(kFlightCtrlParams.rate_yaw, rate_scale);
  }
  kFlightCtrlParams.gnss_lever_x_m = g_gnss_lever_param[0];
  kFlightCtrlParams.gnss_lever_y_m = g_gnss_lever_param[1];
  kFlightCtrlParams.gnss_lever_z_m = g_gnss_lever_param[2];
#ifdef BFS_RATE_ACCEL_FB
  if (aaf_gain >= 0.0f)
  {
    setRateAccelFeedbackGains(aaf_gain,
                              (aaf_yaw_gain >= 0.0f) ? aaf_yaw_gain : aaf_gain,
                              aaf_tau);
  }
#endif
#ifdef BFS_RATE_PHASE_LEAD
  if (pl_gain >= 0.0f)
  {
    setRatePhaseLeadGains(pl_gain,
                          (pl_yaw_gain >= 0.0f) ? pl_yaw_gain : pl_gain,
                          pl_tau);
  }
#endif
  if (g_mag_yaw_jump_rad > 0.0f)
  {
    nav_ekf.mag_yaw_jump_limit_rad(g_mag_yaw_jump_rad);
  }
  g_heading_aid_mode = static_cast<uint8_t>(heading_aid_mode);
  g_heading_align_deg = heading_align_deg;
  // 飞行段回归默认跳过地面 30 s 静置；测校准门控时显式传 --static-calib-s 30。
  g_static_calib_required_s = 0.0f;
  g_sil_pos_leg_n_m = pos_leg_n_m;
  g_sil_pos_leg_e_m = pos_leg_e_m;
  if (ang_alpha >= 0.0f)
  {
    rollAngleOutputFilter.setAlpha(ang_alpha);
    pitchAngleOutputFilter.setAlpha(ang_alpha);
  }
#ifdef FW_SIL_REAL_NAV
  if (real_cfg.enabled)
  {
    real_cfg.flow = real_cfg.flow || flow;
    real_cfg.flow_delay = g_flow_delay;
    real_cfg.mag = real_cfg.mag || g_use_mag_yaw;
    g_gnss_consistency_reject_enabled = real_cfg.gnss_consistency_reject;
    kFlightCtrlParams.inertia_comp_mask = static_cast<uint8_t>(mask & 0x03);
#ifdef FW_SIL_COAXIAL
    // alloc_mode 默认 0=legacy 等于固件默认值：0 时不写、不打印，保证缺省
    // 输出/行为逐位不变；只有显式请求非 legacy 模式才写固件参数。
    if (g_coax_alloc_mode != 0)
    {
      applyCoaxAllocModeTo(kFlightCtrlParams.coax, g_coax_alloc_mode);
    }
#endif
    if (coupled) manual = true;
    return runRealNavLoop(real_cfg, seed, seconds, target_alt, manual, coupled,
                          fault, fault_at, loss, mask, wind_on);
  }
#endif
  // 运行时覆盖前馈掩码（仅宿主机 SIL；生产固件由 0xE1 参数写入）
  kFlightCtrlParams.inertia_comp_mask = static_cast<uint8_t>(mask & 0x03);
  if (coupled) manual = true;  // 耦合激励走手动杆，便于直接看姿态响应
  Plant p; Sensors s; std::mt19937 rng(seed);
  g_ekf.Initialize(Eigen::Vector3f(0.0f, 0.0f, -P.g),
                   Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(),
                   Eigen::Vector3f::Zero(), kHomeLla);
  isLinkUp=true; failsafe_in_flight=false;
  nav_system_initialized=true; is_origin_lla_set=true;
  origin_lat_deg=28.2; origin_lon_deg=112.9; origin_alt_m=50.0;

  const float dt=0.005f;
  const int steps=int(seconds/dt);
  bool alt_cmd=false;
  float max_roll=0,max_pitch=0;
  double sum_roll2=0.0, sum_pitch2=0.0; int n_att=0;
  int last_gnss_update=-1, last_baro_update=-1, last_flow_update=-1;
  // --coax-servojam 的卡死锁存（legacy 演示路径，与 --real-nav 路径互不影响）
  bool coax_jam_latched = false;
  float coax_jam_rad = 0.0f;

  for (int k=0;k<steps;++k) {
    const float t=k*dt;
    if (coupled) {
      // 温和耦合激励：10–12 s 右滚 + 抬头，12–14 s 反向；小幅（±30 µs），
      // 让姿态保持在线性区，比较 mask=0/3 的跨轴响应。
      const uint16_t roll = (t>10 && t<12) ? 1530 : ((t>12 && t<14) ? 1470 : 1500);
      const uint16_t pitch = (t>10 && t<12) ? 1470 : ((t>12 && t<14) ? 1530 : 1500);
      set_rc(1500,1000,t>=0.5f,roll,pitch);
    } else if (manual) {
      const uint16_t roll=(t>10 && t<15)?1600:1500;
      const uint16_t pitch=(t>20 && t<25)?1600:1500;
      set_rc((t<1)?1000:1500,1000,t>=0.5f,roll,pitch);
    } else {
      const uint16_t thr = (t < 1.0f) ? 1000 : ((t >= 2.0f && t < 5.0f) ? 1600 : 1500);
      set_rc(thr,(t<2)?1300:1500,t>=0.5f);
    }
    update_sensors(p,s,rng,t);

    // 真实 EKF：IMU 传播 + GNSS/气压量测（--ekf 实验模式）
    if (g_use_ekf) g_ekf.TimeUpdate(Eigen::Vector3f(s.accel[0], s.accel[1], s.accel[2]),
                     Eigen::Vector3f(s.gyro[0], s.gyro[1], s.gyro[2]), dt);
    const int gt = int(t * 10.0f + 0.5f);
    if (g_use_ekf && s.gnss_valid && gt != last_gnss_update) {
      last_gnss_update = gt;
      const Eigen::Vector3d gnss_lla = bfs::ned2lla(
          Eigen::Vector3d(s.gnss_n, s.gnss_e, s.gnss_d), kHomeLla);
      g_ekf.MeasurementUpdate(
          Eigen::Vector3f(s.gnss_vn, s.gnss_ve, s.gnss_vd), gnss_lla, dt);
    }
    const int ft = int(t * 10.0f + 0.5f);
    if (g_use_ekf && g_use_mag_yaw && ft != last_flow_update) {
      // 磁航向（真实 EKF yaw 观测；默认关闭，见 README：多 seed 会失稳）
      g_ekf.MeasurementUpdateYaw(s.mag_yaw, 0.02f);
    }
    if (g_use_ekf && flow && s.flow_valid && s.flow_quality >= 50 &&
        ft != last_flow_update) {
      last_flow_update = ft;
      // 光流速度：只更新 N/E，D 填当前估计并给大噪声
      const Eigen::Vector3f vcur = g_ekf.ned_vel_mps();
      // 机体系光流速度 -> NED（用当前 EKF 姿态）；D 填当前估计并给大噪声
      g_ekf.MeasurementUpdateAidingVelocityDetailed(
          Eigen::Vector3f(s.flow_vn, s.flow_ve, vcur(2)), 0.20f, 1.0e3f);
    }
    const int bt = int(t * 25.0f + 0.5f);
    if (g_use_ekf && bt != last_baro_update) {
      last_baro_update = bt;
      g_ekf.MeasurementUpdateBaroAltitude(
          static_cast<float>(kHomeLla(2) + s.baro_alt), 0.3f);
    }
    feed_firmware(p,s);
    runGNCExecutive();
    if (!manual && !alt_cmd && t >= g_takeoff_at_s) {
      target_altitude = target_alt;
      vertical_loiter_state = ALTITUDE_HOVERING;
      alt_cmd = true;
    }

    // 舵机一阶 + 速率限幅（缺省 300°/s）+ 0.1% PWM 量化（物理执行器约束）
#ifdef FW_SIL_COAXIAL
    const float servo_rate_dps =
        (g_coax_servo_rate_dps >= 0.0f) ? g_coax_servo_rate_dps : 300.0f;
    const float motor_tau_s =
        (g_coax_motor_tau_s >= 0.0f) ? g_coax_motor_tau_s : 0.28f;
#else
    const float servo_rate_dps = 300.0f;
    const float motor_tau_s = 0.28f;
#endif
    const float rate_limit = (servo_rate_dps > 0.0f)
                                 ? servo_rate_dps * 0.017453292f * dt
                                 : 1.0e9f;  // 0 = 无限速率
#ifdef FW_SIL_COAXIAL
    const float coax_roll_corr = (ch1_output - 50.0f) * 15.0f / 16.6667f;
    const float coax_pitch_corr = -(ch2_output - 50.0f) * 15.0f / 16.6667f;
    float u_cmd = coax_roll_corr * 0.017453292f;
    float l_cmd = coax_pitch_corr * 0.017453292f;
    const float q = 0.001f * 15.0f * 2.0f * 0.017453292f;
    u_cmd = std::round(u_cmd / q) * q;
    l_cmd = std::round(l_cmd / q) * q;
#else
    const float upper=(ch1_output-(50+SC.zero_roll_pct))*SC.half_travel_deg/
      (50*(SC.teeth_gimbal/SC.teeth_servo)*SC.dir_roll)*0.017453292f;
    const float lower=(ch2_output-(50+SC.zero_pitch_pct))*SC.half_travel_deg/
      (50*(SC.teeth_gimbal/SC.teeth_servo)*SC.dir_pitch)*0.017453292f;
    float u_cmd = upper;
    float l_cmd = lower;
    // 0.1% PWM -> 角度量化近似
    u_cmd = std::round(u_cmd / (0.001f * SC.half_travel_deg * 2.0f)) *
            (0.001f * SC.half_travel_deg * 2.0f);
    l_cmd = std::round(l_cmd / (0.001f * SC.half_travel_deg * 2.0f)) *
            (0.001f * SC.half_travel_deg * 2.0f);
#endif
    const float du = std::fmaxf(-rate_limit, std::fminf(rate_limit, u_cmd - p.servo_f));
    const float dl = std::fmaxf(-rate_limit, std::fminf(rate_limit, l_cmd - p.servo_t));
    p.servo_f += du * fminf(dt/0.05f,1.0f);
    p.servo_t += dl * fminf(dt/0.05f,1.0f);

    const float wf_t=(ch3_output/100)*P.wMax, wt_t=(ch4_output/100)*P.wMax;
    const float motor_alpha_legacy =
        (motor_tau_s > 0.0f) ? fminf(dt/motor_tau_s,1.0f) : 1.0f;
    p.motor_wf += (wf_t-p.motor_wf)*motor_alpha_legacy;
    p.motor_wt += (wt_t-p.motor_wt)*motor_alpha_legacy;

#ifdef FW_SIL_COAXIAL
    // 舵机卡死（--coax-servojam）：到达卡死时刻后锁存当时实际角并保持；
    // 下方 F4 会覆盖 roll，即故障注入优先级更高。
    if (g_coax_servojam != 0)
    {
      const float jam_at = (g_coax_servojam_at_s >= 0.0f)
                               ? g_coax_servojam_at_s : fault_at;
      if (t >= jam_at)
      {
        if (!coax_jam_latched)
        {
          coax_jam_latched = true;
          coax_jam_rad = (g_coax_servojam == 1) ? p.servo_f : p.servo_t;
        }
        if (g_coax_servojam == 1) p.servo_f = coax_jam_rad;
        else                      p.servo_t = coax_jam_rad;
      }
    }
#endif
    // 故障注入：F4 上摆卡死（冻结在 +8°）；F3/5 的推力损失在构造 PropulsionState 后
    // 乘到“有效转速”上（只作用于本拍 wrench，不改变电机状态，避免逐拍复利）。
    if (fault == 4 && t > fault_at) {
      p.servo_f = 8.0f * 0.017453292f;
    }

    // 阵风：20-25s 侧风 3 m/s（以 NED 侧向力体现）；--no-wind 时关闭
    p.wind[1]=(wind_on && t>20 && t<25)?3.0f*P.m:0.0f;
#ifdef FW_SIL_COAXIAL
    g_coax_motor_loss1 = 1.0f;
    g_coax_motor_loss2 = 1.0f;
    if (t > fault_at && (fault == 3 || fault == 5)) {
      g_coax_motor_loss1 = loss;
      if (fault == 5) g_coax_motor_loss2 = loss;
    }
    SixDOFWrench wr=computeCoaxialWrench(p);
#else
    PropulsionState ps{p.motor_wf,p.motor_wt,p.servo_f,p.servo_t};
    // 故障注入：fault 3 = 单发（前）推力保留 loss；fault 5 = 双发同时损失
    if ((fault == 3 || fault == 5) && t > fault_at) {
      ps.wf *= loss;
      if (fault == 5) ps.wt *= loss;
    }
    SixDOFWrench wr=computeWrench(ps,P);
#endif
    p.step(wr,dt);

    if ((k%200)==0) {
      float r,pitch,yaw; p.euler(r,pitch,yaw);
      if (fabsf(r*57.29578f)>max_roll) max_roll=fabsf(r*57.29578f);
      if (fabsf(pitch*57.29578f)>max_pitch) max_pitch=fabsf(pitch*57.29578f);
      if (t > 9.0f) {
        sum_roll2 += double(r*57.29578f)*double(r*57.29578f);
        sum_pitch2 += double(pitch*57.29578f)*double(pitch*57.29578f);
        ++n_att;
      }
      std::printf("t=%5.1f alt=%7.2f ekf_alt=%7.2f pos=(%6.2f,%6.2f) roll=%6.2f pitch=%6.2f "
                  "servo=(%5.1f,%5.1f) motor=(%5.1f,%5.1f) gnss=%d f3=%d f4=%d mask=%d\n",
                  t,-p.pos[2],g_ekf.alt_m()-kHomeLla(2),p.pos[0],p.pos[1],r*57.29578f,pitch*57.29578f,
                  ch1_output,ch2_output,ch3_output,ch4_output,(int)s.gnss_valid,
                  (int)g_actuator_thrust_loss,(int)g_actuator_servo_jam,(int)kFlightCtrlParams.inertia_comp_mask);
    }
  }
  const double rms_roll = n_att ? std::sqrt(sum_roll2/n_att) : 0.0;
  const double rms_pitch = n_att ? std::sqrt(sum_pitch2/n_att) : 0.0;
  std::printf("[fw_sil_env] seed=%d manual=%d coupled=%d fault=%d mask=%d "
              "max_roll=%.2f max_pitch=%.2f rms_roll=%.2f rms_pitch=%.2f "
              "f3=%d f4=%d final_alt=%.2f\n",
              seed,(int)manual,(int)coupled,fault,(int)kFlightCtrlParams.inertia_comp_mask,
              max_roll,max_pitch,rms_roll,rms_pitch,
              (int)g_actuator_thrust_loss,(int)g_actuator_servo_jam,-p.pos[2]);
  return 0;
}
