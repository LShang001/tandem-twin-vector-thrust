/**
 * @file    flight_control.cpp
 * @module  固件/控制（GNC 执行器：模式判定、姿态/速率环、控制分配、执行器 mix）
 * @brief   把遥控输入与导航状态转换为舵机摆角、双电机转速与油门
 * @details 输入：RC 摇杆、导航姿态/角速率、参数表 kFlightCtrlParams；
 *          处理：runGNCExecutive 每 200 Hz 严格按序执行
 *            层0 输入与模式判定 → 姿态外环 → 角速率内环 → 控制分配（B 矩阵）
 *            → 执行器 mix → 限幅与安全门控 → 故障诊断 → 在线辨识（只观测）；
 *          输出：上摆 δf / 下摆 δt 舵机 PWM% 与双电机油门%。
 *          不变量：任何分支的输出都先经 dMax / dwMax / ±MAX_CORRECTION /
 *            0–100% 限幅才落地；锁定态电机恒 0%。
 * @theory  SAS 三通道与直接映射 @ref CTL-001 §3–§5；
 *          差速开方分配 @ref CTL-001 §7、@ref PROP-001 §6；
 *          B_true 控制分配 @ref THY-003 §3、@ref TandemVec-Paper/ch/05-allocation.tex (eq:effectiveness_matrix)；
 *          惯量逆解交叉耦合 @ref THY-003 §1.2；
 *          电机 τm 一阶滞后 @ref THY-002 §4、@ref PROP-001 §3；
 *          FPV 摇杆曲线 @ref TandemVec-Paper/ch/07-control.tex (eq:rc_curve)；
 *          执行器轴映射（悬停构型）@ref AGENTS.md 踩坑记录
 * @frame   固件"悬停存档系"：x_b=滚转轴、y_b=俯仰轴、z_b=推力轴（悬停竖直指地，
 *          即机头朝天时机身纵轴）；导航量 NED（x 北 / y 东 / z 下）。
 *          与论文/MOD-002 模型系差一个纯旋转置换，轴名不可跨文档混读。
 * @units   控制器角度域统一 deg、deg/s、deg/s²；仅在写 ControlOutputs_t 时
 *          经 ControlUnits::dps2ToRadps2 转 rad/s² 进入惯量逆解；
 *          位置 m、m/s、m/s²；PWM 988–2012 µs 或 0–100%；时间 s、毫秒 _ms。
 * @redline 执行器轴映射、速率环量纲、差速公式 ωf=ω0√(1+Δω)、SAS 摆角限幅
 *          （逐条见 docs/registers/行为保持红线清单.md）
 * @note    速率环量纲 2026-08-11 迁移完成：roll/pitch kp=16.042818 s⁻¹、
 *          yaw kp=11.459156 s⁻¹、ki=5.729578 / 4.297184 s⁻²。外部参数文件若
 *          仍写旧混合域 0.28/0.20/0.10，物理增益会低 57.3 倍，不得导入。
 * @see     flight_control.h、FlightCtrlParams.h、TandemVec_ControlAllocation.h、
 *          CTL-001 §8.6（仿真↔固件对照表）
 * @test    test_host/test_flight_control_axis.cpp（T16–T18 轴映射/前馈/航向跟踪）；
 *          test_host/fw_sil/（真实源码 SIL 定高/阶跃）
 */
#include "flight_control.h"
#include "ins_altitude_reference.h"
#include "math_utils.h"
#include "navigation_task.h"
#include "QuaternionMath.h"
#include "sensor_peripheral.h"
// 纵列双发矢量推力：推进正向映射 + 差速分配 + 物理模型逆解控制分配
#include "TandemVec_Propulsion.h"
#include "TandemVec_Config.h"
#include "TandemVec_ControlAllocation.h"
// 在线参数辨识（★ 纯观测模式：只估计并遥测，不修改任何增益）
#include "TandemVec_OnlineID.h"
// 惯量逆解交叉耦合前馈（★2026-08-10 通用层：ω×(I·ω) + ω×h，仿真同构）
#include "InertiaDecoupling.h"
#include "RcCurve.h"    // FPV 摇杆曲线（RATE_MODE，★2026-08-11）
#include "ControlUnits.h"
#include "ActuatorFaultMonitor.h" // 执行器故障检测（2026-10，只读诊断）
#include "ActuatorOutputMap.h"    // 机型输出表 → 舵机/电调/阀门（通用层）
#include "ThrustAllocation.h"     // 期望天向比力 → 总推力/油门（机型推进层）
#include "TandemVecAllocator.h"   // 纵列双发 α/油门 → 摆角/差速（机型分配器）
#include "mavlink_bridge.h"    // MAVLink STATUSTEXT 事件桥接（2026-08-10）

#include <cmath>

// ═══════════════════════════════════════════════════════════════════════════
// 【层0】RC 输入与模式判定
//   process_control_inputs → determine_control_mode → manage_pid_integrals
//   职责：PWM 通道 → 语义化输入（解锁/点火/手动TVC/姿态子模式）；
//         模式档位 → ControlMode；按模式/油门动态使能各环积分。
//   安全约定：一切开关判定先过 isLinkUp 门控（上电 raw_rc 全 0 会误判低档，
//     见 AGENTS.md 踩坑记录）；锁定/断链时电机强制 0。
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief 处理遥控器输入与开关状态，完成解锁上升沿的一次性初始化
 *
 * 职责：① 读 raw_rc_values（PWM 988–2012 µs）；② 开关量去抖动与语义化；
 * ③ 燃料液位双级滤波；④ 同步状态标志；⑤ 解锁上升沿重置控制目标/PID/
 * 滤波器/卡尔曼/原点；⑥ 起飞点相对位置清零；⑦ 燃料阀与点火 MOS 输出。
 *
 * @param[out] inputs 控制输入结构体（通道原始值 + 开关状态），全部覆盖写
 * @pre  raw_rc_values[] 已由通信层更新（无链路时为 0，故必须配合 isLinkUp）
 * @post 所有开关量已去抖；解锁上升沿后的控制器状态与起飞点均为零初值
 * @note  解锁上升沿的重置范围（目标高度/位置、12 个 PID、7 个滤波器、
 *        HorizontalKF、气压参考）必须保持"全清"——漏一项会让新架次继承旧积分。
 * @ref   CTL-001 §8.6（模式与门控）；见 AGENTS.md 踩坑记录
 * @units 输入 PWM µs（988–2012，1500 中位）；内部相对位置 m（NED）
 * @frame 通道语义按 FRD 摇杆约定；相对位置 NED（北/东/下）
 */
void process_control_inputs(ControlInputs_t &inputs)
{
  // ========================================================================
  // 步骤 1: 读取遥控器通道原始值 (PWM 范围 988-2012)
  // ========================================================================
  // 通道 1 (Roll): 左右摇杆，988=最左, 1500=中位, 2012=最右
  inputs.roll_raw = raw_rc_values[0];
  // 通道 2 (Pitch): 前后摇杆，988=最前(下俯), 1500=中位, 2012=最后(上仰)
  inputs.pitch_raw = raw_rc_values[1];
  // 通道 3 (Throttle): 油门摇杆，988=最低, 1500=中位, 2012=最高
  inputs.throttle_raw = raw_rc_values[2];
  // 通道 4 (Yaw): 偏航摇杆，988=最左(逆时针), 1500=中位, 2012=最右(顺时针)
  inputs.yaw_raw = raw_rc_values[3];
  // 通道 7 (Mode): 三档飞行模式开关
  //   < 1300: 手动模式 (MANUAL)
  //   1300-1650: 高度保持 (AUTO_ALTITUDE)
  //   > 1650: 定点/制导 (AUTO_POSITION / GUIDED)
  inputs.mode_channel_val = raw_rc_values[6];

  // ========================================================================
  // 步骤 2: 开关状态处理 (带中值滤波去抖动)
  // ========================================================================
  // 通道 5 (Arm/Disarm): 解锁开关
  //   > 1500: 已解锁 (Armed) - 允许电机输出
  //   <= 1500: 已锁定 (Disarmed) - 电机锁定
  // 使用 7 点中值滤波去除机械开关抖动
  // 失控保护(failsafe_in_flight)绕过解锁通道阈值判定，避免断链时 1500中值导致的空中锁定
  inputs.is_unlocked = failsafe_in_flight || (isLinkUp && (unlockMedian.filter(static_cast<float>(raw_rc_values[4])) > 1500.0f));

  // 通道 6 (Ignition): 点火开关
  //   > 1500: 允许点火 (点火 MOS 管通电)
  //   <= 1500: 禁止点火
  // 使用 5 点中值滤波
  inputs.is_ignition_enabled = (ignitionMedian.filter(static_cast<float>(raw_rc_values[5])) > 1500.0f);

  // 通道 8 (TVC Manual): TVC 手动/自动切换开关
  //   < 1200: 手动 TVC 模式 (摇杆直接控制舵机，旁路姿态控制器)
  //   >= 1200: 自动 TVC 模式 (姿态控制器输出舵机角度)
  // ★ 2026-08-08 上电/断链安全门控：无链路时强制自动分支（isLinkUp=false）。
  //   根因：raw_rc_values 初始化全 0 → CH8=0<1200 → is_manual_tvc=true →
  //   手动 TVC 旁路分支把舵机钳到满偏 ±15°（PWM 72%/28%），直到首帧通道
  //   数据到达。门控后无链时落入全锁定分支（舵机 1500us 中位 + 电机停）。
  //   链路正常时手动 TVC 标定功能不受影响；断链后同样兜底。
  // ★2026-08-10 手动TVC开关反逻辑（用户需求）：低位（默认）= 飞控自稳介入，
//   仅高位（>1750）才是手动 TVC 演示/标定模式——避免忘记打回自动导致意外手动旁路。
//   上电无信号 raw_rc 全 0 → 低位 → is_manual_tvc=false → 锁定分支兜底（安全）。
inputs.is_manual_tvc = isLinkUp && (raw_rc_values[7] > 1750);

  // 通道 9 (Attitude Mode): 姿态控制模式选择
  //   < 1500: 角度模式 (ATTITUDE_MODE) - 摇杆控制目标角度
  //   >= 1500: 角速率模式 (RATE_MODE) - 摇杆直接控制目标角速率
  inputs.attitude_mode = (raw_rc_values[8] < 1500) ? ATTITUDE_MODE : RATE_MODE;

  // ========================================================================
  // 步骤 3: 读取燃料液位传感器
  // ========================================================================
  // FUEL_PIN (PC9) 连接浮球式液位开关
  // HIGH (1): 液位充足 (浮球上升触发)
  // LOW (0): 液位不足 (浮球下降触发)
  int rawFuel = digitalRead(FUEL_PIN);                     // 读取原始数字电平
  float medianFuel = fuelMedianFilter.filter(rawFuel);     // 9点中值滤波去除触点抖动
  float smoothFuel = fuelLowpassFilter.filter(medianFuel); // 低通滤波 (alpha=0.02) 进一步平滑
  fuelOK = (smoothFuel > 0.5f);                            // 阈值判定: 平滑值 > 0.5 认为液位充足

  // ========================================================================
  // 步骤 4: 同步状态标志
  // ========================================================================
  isDatalogging = inputs.is_unlocked; // 解锁时启动黑匣子数据记录
  // 轨迹规划模式: 模式通道 > 1750 (高档) 且点火开关开启
  // 这意味着只有在制导模式档位且点火开关打开时，才允许接收上位机轨迹规划指令
  trajectoryPlanningStarted = inputs.mode_channel_val > 1750 && inputs.is_ignition_enabled;

  // ========================================================================
  // 步骤 5: 解锁上升沿处理 (一次性初始化)
  // ========================================================================
  // 使用静态变量检测解锁状态的上升沿 (false -> true)
  // 每次解锁时执行一次完整的系统重置，确保从干净状态开始飞行
  static bool last_unlocked_state = false;

  if (inputs.is_unlocked && !last_unlocked_state) // 检测上升沿
  {
    // --- 5.1 重置物理目标设定点 ---
    // 将所有控制目标设为当前状态，防止解锁瞬间因目标值与实际值不匹配导致跳变
    target_altitude = 0.0f;              // 新的起飞点原点建立后，高度目标从 0 m 开始
    targetNorth = 0.0f;                 // 北向位置目标 = 0 (相对位置)
    targetEast = 0.0f;                  // 东向位置目标 = 0
    target_accel_z_up_global = 0.0f;    // 垂直加速度目标 = 0

    // --- 5.2 重置所有 PID 控制器 ---
    // 清除积分项和微分项缓存，防止历史数据干扰新的飞行
    // 姿态轴 (Roll/Pitch/Yaw)
    rollAnglePID.reset();  // Roll 角度外环
    pitchAnglePID.reset(); // Pitch 角度外环
    yawAnglePID.reset();   // Yaw 角度外环（新增）
    rollRatePID.reset();   // Roll 角速率内环
    pitchRatePID.reset();  // Pitch 角速率内环
    yawRatePID.reset();    // Yaw 角速率内环
    // 位置与垂直轴
    altitudePositionPController.reset();
    altitudeVelocityPIDController.reset();
    northPosPID.reset();
    eastPosPID.reset();
    northVelPID.reset();
    eastVelPID.reset();

    // 3. 初始化所有控制流滤波器 (防止滤波器初始阶跃)
    thrustCompN_filter.initialize(0.0f);
    thrustCompE_filter.initialize(0.0f);
    rollAngleOutputFilter.initialize(0.0f);
    pitchAngleOutputFilter.initialize(0.0f);
    rollOutputFilter.initialize(0.0f);
    pitchOutputFilter.initialize(0.0f);

    // 4. 重置状态估计器 (KF/EKF)
    kf_north.reset();
    kf_east.reset();

    // 5. 设置地理原点 (Home Point)
    // EKF 必须在静止检测完成后才能提供有效 LLA；未初始化时跳过，
    // 原点将在 navigation_task EKF 初始化或首次 GNSS 重锚定时补设。
    if (nav_data_source == NavDataSource::DETA100 && deta100_online &&
        Status_Packet.filter_status.gnss_fix_status >= DETA100_GPS_FIX_TYPE_3D_FIX &&
        std::isfinite(Geodetic_Pos_Packet.latitude) &&
        std::isfinite(Geodetic_Pos_Packet.longitude) &&
        std::isfinite(Geodetic_Pos_Packet.height) &&
        Geodetic_Pos_Packet.latitude != 0.0 &&
        Geodetic_Pos_Packet.longitude != 0.0)
    {
      setNavigationOriginFromLlaRad(Geodetic_Pos_Packet.latitude,
                                    Geodetic_Pos_Packet.longitude,
                                    Geodetic_Pos_Packet.height);
      is_origin_lla_set = true;
      Serial8.println("[Arm] Home Point set from DETA100.");
      mavlinkSendStatustext(MAV_SEVERITY_INFO, "Armed - Home Point set");
    }
    else if (nav_system_initialized)
    {
      setNavigationOriginFromLlaRad(nav_ekf.lla_rad_m()[0],
                                    nav_ekf.lla_rad_m()[1],
                                    nav_ekf.lla_rad_m()[2]);
      is_origin_lla_set = true;
      Serial8.println("Home Point Set! Lat: " + String(origin_lat_deg, 7) + ", Lon: " + String(origin_lon_deg, 7));
    }
    else
    {
      Serial8.println("[Arm] EKF not initialized, defer Home Point to EKF init.");
      mavlinkSendStatustext(MAV_SEVERITY_WARNING, "Armed - EKF not ready, HP deferred");
    }
    // 气压计只保留起飞点相对高度语义，不再混入激光测距离地高度。
    if (resetBaroAltitudeReference())
    {
      Serial8.println("[Arm] Barometer takeoff reference reset to 0 m.");
    }
    else
    {
      Serial8.println("[Arm] Barometer reference reset skipped: invalid sample.");
    }

    // 起飞点是控制坐标原点，所有对外相对位置和高度立即清零。
    relative_north = 0.0f;
    relative_east = 0.0f;
    relative_down = 0.0f;
    INS_GNSS_Packet.location_north = 0.0f;
    INS_GNSS_Packet.location_east = 0.0f;
    INS_GNSS_Packet.location_down = 0.0f;
    estimated_height = 0.0f;
    fused_north_pos = 0.0f;
    fused_east_pos = 0.0f;
    fused_north_vel = 0.0f;
    fused_east_vel = 0.0f;

    Serial8.println("[HorizontalKF] Filters Reset on Arming.");
  }

  // 更新 last_unlocked_state，为下一个循环检测上升沿做准备
  last_unlocked_state = inputs.is_unlocked;

  // 实时计算相对位移：本函数不在这里积分/差分
  // ① relative_north/east/down 由导航层统一写入（EKF 相对起飞点 NED），
  //    历史上这里曾有两套本地实现（经纬度→ENU 位移、FDI location_* 直接差分），
  //    两者都会与导航层争抢同一组全局量，已删除（git 历史保留）。
  // ② 本函数只负责"原点未建立时兜底清零"，防止解锁前残留上一架次的值。
  if (is_origin_lla_set)
  {
    // 原点有效：relative_* 直接使用导航层本拍写入值，不做二次换算。
  }
  else
  {
    // 如果原点未设置，相对位置应为0
    relative_north = 0.0f;
    relative_east = 0.0f;
    relative_down = 0.0f; // 确保相对地向位置也清零
  }

  // 辅助执行机构控制 (燃料阀门、点火)
  SetServoPos(inputs.is_manual_tvc ? 100.0f : 0.0f, SERVO7_PIN); // 控制燃料阀门 (SERVO7)
  // 点火控制 (ignition 引脚)
  // 条件：必须已解锁 (isUnlocked 为 true) 且点火开关已打开 (isIgnitionEnabled 为 true)
  digitalWrite(ignition, (inputs.is_unlocked && inputs.is_ignition_enabled) ? HIGH : LOW);
}

/**
 * @brief 由模式通道档位判定飞行模式，并处理 GUIDED 超时与航向不可观降级
 * @param[in] inputs 控制输入（只用 mode_channel_val；解锁门控在 getControlMode 内部）
 * @return ControlMode（MANUAL / AUTO_ALTITUDE / AUTO_POSITION / GUIDED）
 * @pre  CH7 模式通道已读入；GUIDED 模式要求上位机持续刷新制导指令
 * @note  ① 超时门限 GUIDANCE_CMD_TIMEOUT_MS = 500 ms：制导链一断就退回 MANUAL，
 *           属于安全降级路径，任何改动都必须保留（不得为"连得上"而放宽）。
 *        ② 航向不可观降级（2026-10）：`g_pos_hold_degraded` 由导航层
 *           HeadingAidingGate 置位（磁矢量超过 2 s 无成功融合；YawGsf 只诊断，
 *           不解除门控）。
 *           AUTO_POSITION/GUIDED 都依赖绝对航向构造水平推力方向，航向不可观时
 *           继续位置闭环会追逐漂移的航向估计（AGENTS.md EKF-in-loop 根因）。
 *           此时强制降为 AUTO_ALTITUDE：保留高度环，放弃水平位置闭环；
 *           摇杆仍可控制姿态。磁源恢复并连续 3 s 新鲜后由导航层解除门控。
 *           无绝对航向源（未装磁/未标定/磁长时间受扰）不是可飞行状态，
 *           该降级是安全边界，不是可选功能。
 * @ref   CTL-001 §8.6；CTL-003 §2；AGENTS.md「EKF 进控制闭环 10/10 FAIL」
 * @units millis() 毫秒
 */
ControlMode determine_control_mode(const ControlInputs_t &inputs)
{
  ControlMode mode = getControlMode(inputs.mode_channel_val); // getControlMode内部已处理未解锁情况

  // GUIDED 模式的超时故障保护
  if (mode == GUIDED && (millis() - last_guidance_command_millis > GUIDANCE_CMD_TIMEOUT_MS))
  {
    mode = MANUAL; // 超时则强制切换到手动模式
                   // AnoCom.sendText("Guidance Timeout! Switching to MANUAL.");
  }

  g_mode_degraded_heading = false;
  if (g_pos_hold_degraded &&
      (mode == AUTO_POSITION || mode == GUIDED))
  {
    mode = AUTO_ALTITUDE;
    g_mode_degraded_heading = true;
  }
  return mode;
}

/**
 * @brief 按模式/油门/手动旁路状态动态使能各 PID 环的积分项
 * @param[in] inputs 控制输入（解锁、手动TVC、油门、姿态子模式）
 * @param[in] mode   当前飞行模式
 * @return 无；就地调用各 PID 的 setIntegralEnable()
 * @pre  在每拍控制律之前调用（runGNCExecutive 步骤 3）
 * @note  ① 姿态外环积分仅在角度模式 + 有动力时使能；
 *        ② 航向外环积分恒禁用（悬停倾斜误差会积分饱和，且航向源与磁解耦，
 *          见 manage_pid_integrals 内 yawAnglePID 那一行）；
 *        ③ 速率内环积分只要姿态控制激活就使能（消除配平力矩）；
 *        ④ 垂直速度环积分只在定高/定点/制导且非手动旁路时使能。
 *        setIntegralEnable 内部做无扰切换（从当前输出反推积分初值），
 *        因此频繁开关不会在输出上产生阶跃。
 * @ref   PositionPID.h（无扰切换/积分状态钳位）
 */
void manage_pid_integrals(const ControlInputs_t &inputs, ControlMode mode)
{
  // 判断油门是否有效（大于1100表示有动力）
  bool throttle_active = inputs.throttle_raw > 1100;

  // 判断姿态控制是否激活：需满足解锁、非手动TVC、油门有效三个条件
  bool attitude_control_active = inputs.is_unlocked && !inputs.is_manual_tvc && throttle_active;

  // 姿态角PID积分项控制（Roll/Pitch/Yaw 角度模式启用；Yaw 外环保守禁用积分，
  // 仅比例+微分——避免悬停倾斜误差的积分饱和，与磁航向源解耦）
  rollAnglePID.setIntegralEnable(attitude_control_active && (inputs.attitude_mode == ATTITUDE_MODE));
  pitchAnglePID.setIntegralEnable(attitude_control_active && (inputs.attitude_mode == ATTITUDE_MODE));
  yawAnglePID.setIntegralEnable(false);  // Yaw 姿态外环启用但不用积分（2026-08-02 VTOL 构型）

  // 姿态角速度PID积分项控制（姿态控制激活时启用）
  rollRatePID.setIntegralEnable(attitude_control_active);
  pitchRatePID.setIntegralEnable(attitude_control_active);
  yawRatePID.setIntegralEnable(attitude_control_active);

  // --- 垂直速度PID积分项控制 ---
  // 积分项应在以下情况启用：
  // 1. 处于 AUTO_ALTITUDE 或 AUTO_POSITION 或 GUIDED 模式。
  // 2. 姿态控制激活 (解锁、非手动TVC、油门有效)。
  // 3. 垂直Loiter状态为 ALTITUDE_HOVERING (高度保持) 或 ALTITUDE_MOVING (垂直速度控制)。
  //    实际上，只要处于 AUTO_ALTITUDE/AUTO_POSITION/GUIDED 模式且姿态控制激活，垂直速度环就应该工作，
  //    其积分项也应该启用以消除静差。
  altitudeVelocityPIDController.setIntegralEnable(
      (mode == AUTO_ALTITUDE || mode == AUTO_POSITION || mode == GUIDED) && inputs.is_unlocked && !inputs.is_manual_tvc);
}

/**
 * @brief 垂直通道：模式 → 天向加速度指令 → 期望天向比力（机型无关边界）
 * @param[in]  inputs  控制输入（手动模式下油门摇杆直通）
 * @param[in]  mode    当前飞行模式；AUTO_* / GUIDED 走闭环，其余手动油门
 * @param[out] outputs 自动模式写 specific_force_up_mps2/thrust_cos_tilt/
 *                     throttle_direct；手动模式写直通油门。throttle_percent
 *                     的自动值由 mix 层机型推力映射回填。
 * @pre  高度/垂直速度来自导航层（起飞点 NED 原点，向上为正）
 * @formula a_up = g + a_z_cmd；cos_tilt = max(|R33|, 0.5)；
 *          输出比力 a_up，机型层再用 F_total = m·a_up/cos_tilt 映射执行器
 * @ref  PROP-001 §2、见 CTL-001 §8.4（串级定高与倾角补偿）；
 *       @ref MOD-002 §1.2（cos_tilt 用 R33，不是 R13）；CTL-003 §2
 * @units 高度 m、垂直速度 m/s、比力 m/s²；PID 步长 0.005 s
 * @note  ★ 2026-10-24：本函数不再读取 initial_mass/MAX_THRUST，也不做油门映射。
 *        质量与最大推力属于机型推进层（ThrustAllocation），否则控制层会把
 *        TandemVec 的推进能力泄漏到共轴/火箭路径。
 *        倾角补偿用 |R33| = |1−2(qx²+qy²)|：存档系 z_b=推力轴，R33 即推力轴在
 *        NED 竖直方向的投影。若误用 R13（x_b 投影），悬停时 R13≈0 → 钳到 0.5
 *        → 油门需求凭空 ×2（2026-08-07 已踩，勿改回）。
 */
void compute_thrust_control(const ControlInputs_t &inputs, ControlMode mode, ControlOutputs_t &outputs)
{
  // 控制高度统一使用起飞点 NED 原点，转换为天向为正。
  const float current_altitude_for_control =
      InsRelativeHeightUpFromNedDown(relative_down);

  // 获取当前垂直速度 (向上为正)
  // 注意：DETA100 输出的 velocity_down 是 NED 坐标系，向下为正
  // 控制器期望向上为正，所以取负值
  const float currentVerticalVelocity =
      InsVerticalVelocityUpFromNedDown(INS_GNSS_Packet.velocity_down);

  // ==========================================================================================
  // [逻辑分支 A] 自动油门控制类模式 (定高 / 定点 / 制导)
  // ==========================================================================================
  if (mode == AUTO_ALTITUDE || mode == AUTO_POSITION || mode == GUIDED) // 自动高度模式或自动定点模式都采用自动油门控制
  {
    // --- 1. 模式初始化与状态保持 ---
    if (!autoAltitudeMode)
    {
      autoAltitudeMode = true;
      // 首次进入自动高度/定点模式时，将当前高度设为目标高度
      target_altitude = current_altitude_for_control;
      altitudePositionPController.reset();
      altitudeVelocityPIDController.reset();
      vertical_loiter_state = ALTITUDE_HOVERING; // 初始状态为悬停
    }

    // 定义目标天向运动加速度变量 (m/s^2)
    float target_acceleration_z = 0.0f;

    // --- 2. 分模式计算加速度指令 ---

    if (mode == GUIDED)
    {
      // [GUIDED 模式核心逻辑]
      // 直接使用上位机下发的垂直加速度指令 (ENU坐标系，Up为正)
      // 注意：这里假设 guidance_accel_U_cmd 是运动学加速度 (Kinematic Accel)，不包含抵消重力的 1g
      // 如果上位机发 0，代表垂直匀速运动；如果上位机发 >0，代表向上加速。
      target_acceleration_z = guidance_accel_U_cmd;

      // [关键]：这里不调用 altitudeVelocityPIDController.compute()
      // 也不调用 reset()。PID 对象的内部状态（尤其是积分项 I_term）保持不变（冻结）。
      // 这样切回 Auto 模式时，积分项依然有效，可立即补偿电池电压下降。

      // 更新目标高度为当前高度，防止切回 Auto 模式瞬间因位置误差产生跳变
      target_altitude = current_altitude_for_control;
      target_vertical_velocity = currentVerticalVelocity;
    }

    else
    {
      // [AUTO_ALTITUDE / AUTO_POSITION 模式逻辑]

      // --- 目标高度/垂直速度更新 (来自遥控器摇杆) ---
      // 1. 将油门摇杆值映射到0-100的百分比，与原始逻辑保持一致。
      float throttle_mapped_percent = mapFloat(inputs.throttle_raw, 988.0f, 2012.0f, 0.0f, 100.0f);

      // 2. 定义控制参数，使用已有的全局常量。
      const float THROTTLE_CENTER = 50.0f;
      const float ALTITUDE_RATE_CMD_DEADZONE = 10.0f; // 油门杆中位死区 (百分比)
      const float MAX_TARGET_ALTITUDE_RATE = 1.0f;    // 摇杆满舵时，目标高度的最大变化速度 (1.0 m/s)

      // 判断油门摇杆是否在中心死区内
      bool throttle_stick_in_deadzone = (fabsf(throttle_mapped_percent - THROTTLE_CENTER) < ALTITUDE_RATE_CMD_DEADZONE);

      // --- 垂直 Loiter 状态机逻辑 ---
      if (throttle_stick_in_deadzone)
      {
        /*
         * [ 垂直制导优化：高度锁定点物理预测 ]
         * 原理：
         * 1. 向上刹车加速度 a_z_up 受限于电机最大推力与重力之差。
         * 2. 引入安全系数，预留出垂直速度环 PID 的调节空间。
         */

        // --- 垂直物理参数 ---
        const float MAX_UPWARD_ACCEL = 4.0f;       // 假设最大推力下，向上加速的极限约为 4.0 m/s^2 (约0.5g增量)
        const float MAX_DOWNWARD_ACCEL = 5.0f;     // 向下减速(刹车)时，为了平稳，限制减速度为 5.0 m/s^2
        const float VERTICAL_SAFETY_FACTOR = 1.2f; // 垂直安全系数

        if (vertical_loiter_state == ALTITUDE_MOVING)
        {
          // 切换到悬停状态瞬间执行预测
          altitudePositionPController.reset();

          float brake_dist_z = 0.0f;

          if (currentVerticalVelocity > 0.02f)
          {
            // 情况 A: 正在上升，松杆锁定
            // 使用最大向下加速度（减少推力）来刹车
            brake_dist_z = (currentVerticalVelocity * currentVerticalVelocity) / (2.0f * MAX_DOWNWARD_ACCEL);
          }
          else if (currentVerticalVelocity < -0.02f)
          {
            // 情况 B: 正在下降，松杆锁定
            // 使用最大向上加速度（增加推力）来刹车
            brake_dist_z = -(currentVerticalVelocity * currentVerticalVelocity) / (2.0f * MAX_UPWARD_ACCEL);
          }

          // 应用安全系数并进行限幅
          brake_dist_z *= VERTICAL_SAFETY_FACTOR;
          brake_dist_z = constrain(brake_dist_z, -3.0f, 3.0f); // 垂直预测点限制在 3 米内

          // 锁定新的目标高度点
          target_altitude = current_altitude_for_control + brake_dist_z;

          // 状态切换
          vertical_loiter_state = ALTITUDE_HOVERING;
        }

        // 外环：计算高度误差 -> 目标垂直速度
        float altitude_error = target_altitude - current_altitude_for_control;
        float raw_target_vertical_velocity = altitudePositionPController.compute(altitude_error, 0, 0.005f);
        // 目标垂直速度进行滤波，使指令变化更平滑
        target_vertical_velocity = altitude_rate_target_filter.filter(raw_target_vertical_velocity);

        // (可选) 对目标垂直速度进行限幅，防止位置环输出过大
        target_vertical_velocity = constrain(target_vertical_velocity, -MAX_TARGET_ALTITUDE_RATE, MAX_TARGET_ALTITUDE_RATE);
      }
      else
      {
        // --- 移动状态 (ALTITUDE_MOVING) ---
        vertical_loiter_state = ALTITUDE_MOVING; // 更新状态为移动

        // 计算目标高度的变化率，精确复现原始的分段映射逻辑。
        float target_altitude_change_rate = 0.0f; // 初始化为0

        // 如果油门在上升区 (中心点 + 死区之上)
        if (throttle_mapped_percent > THROTTLE_CENTER + ALTITUDE_RATE_CMD_DEADZONE)
        {
          // 将摇杆在 (死区边缘, 100] 的行程，映射到 (0, MAX_TARGET_ALTITUDE_RATE] m/s 的目标速度
          target_altitude_change_rate = mapFloat(throttle_mapped_percent,
                                                 THROTTLE_CENTER + ALTITUDE_RATE_CMD_DEADZONE, 100.0f,
                                                 0.0f, MAX_TARGET_ALTITUDE_RATE);
        }
        // 如果油门在下降区 (中心点 - 死区之下)
        else if (throttle_mapped_percent < THROTTLE_CENTER - ALTITUDE_RATE_CMD_DEADZONE)
        {
          // 将摇杆在 [0, 死区边缘) 的行程，映射到 [-MAX_TARGET_ALTITUDE_RATE, 0) m/s 的目标速度
          target_altitude_change_rate = mapFloat(throttle_mapped_percent,
                                                 0.0f, THROTTLE_CENTER - ALTITUDE_RATE_CMD_DEADZONE,
                                                 -MAX_TARGET_ALTITUDE_RATE, 0.0f);
        }
        // 如果在死区内，target_altitude_change_rate 保持为 0.0f。

        // 对目标垂直速度进行滤波，使指令变化更平滑
        altitudeRateTarget = altitude_rate_target_filter.filter(target_altitude_change_rate);

        // 直接将摇杆映射的目标垂直速度作为内环的目标
        target_vertical_velocity = altitudeRateTarget;

        // 重置高度位置环PID，因为此时不使用高度位置环
        altitudePositionPController.reset();
      }

      // --- 内环：计算速度误差 -> 目标垂直加速度 (Target Acceleration) ---
      // PID控制器的输出物理含义为“期望加速度(m/s^2)”
      // 这里的 target_acceleration_z 是相对于悬停状态的增量加速度
      target_acceleration_z = altitudeVelocityPIDController.computeDerivativeOnMeasurement(target_vertical_velocity, currentVerticalVelocity, 0.005f);
    }

    // --- 3. 输出期望天向比力（控制层边界，机型无关） ---
    // 这里的 target_acceleration_z 要么来自 PID (AUTO模式)，要么来自上位机 (GUIDED模式)
    // 总天向比力 = 抵消重力(1g) + 运动加速度；重力项用 EKF 当地值
    // ekf_gravity_mps2（Somigliana，未初始化时回退 9.81）。
    // ★ 本层不乘质量、不除 MAX_THRUST：质量/最大推力/推力曲线属于机型层，
    //   由 mix_and_output_commands() 的 ThrustAllocation 完成。
    outputs.specific_force_up_mps2 = ekf_gravity_mps2 + target_acceleration_z;

    // 倾斜推力补偿的几何量：当机体倾斜时，为保持相同天向分力，总推力必须增大。
    // 计算倾斜角的余弦值 (cos_tilt)
    // ★ 与原始 VTVL 实飞存档版一致（2026-08-07 恢复）：
    // 存档系 z_b = 推力轴（机头朝天时指向地面，推力沿 -z_b 朝上），
    // 故垂直分量 = z_b 在 NED 垂直方向的投影 R33 = 1-2(qx²+qy²)。
    // 悬停静止 roll=pitch=0 → q=Rz(Heading) → qx=qy=0 → R33=1 ✓ 无需补偿。
    // ⚠️ 2026-08-07 曾因误改轴映射（x_b 竖直）而改用 R13，恢复映射后必须回退，
    //    否则悬停时 R13≈0 → 钳位 0.5 → 油门需求×2。
    float qx = AHRS_Packet.Qx, qy = AHRS_Packet.Qy;
    float cos_tilt = 1.0f - 2.0f * (qx * qx + qy * qy);  // R33
    cos_tilt = fabsf(cos_tilt);                          // 机头朝上/朝下方向无关

    // 安全保护：限制最大补偿角度，防止倾斜过大时推力暴增导致失控
    // 例如限制在 60度 (cos(60)=0.5)，即推力最多增加到原来的2倍
    cos_tilt = fmaxf(cos_tilt, 0.5f);
    outputs.thrust_cos_tilt = cos_tilt;

    outputs.throttle_direct = false;
    outputs.throttle_percent = 0.0f; // 机型层映射后回填，供遥测/辨识/故障诊断

    target_accel_z_up_global = target_acceleration_z; // 更新天向运动的目标加速度全局变量
  }

  // ==========================================================================================
  // [逻辑分支 B] 手动模式
  // ==========================================================================================
  else
  { // MANUAL 模式
    // --- 手动油门 ---
    autoAltitudeMode = false;
    altitudePositionPController.reset();
    altitudeVelocityPIDController.reset();     // 手动模式下重置 PID，积分清零
    vertical_loiter_state = ALTITUDE_HOVERING; // 退出模式时重置状态机
    outputs.throttle_percent = mapFloat(inputs.throttle_raw, 988.0f, 2012.0f, 0.0f, 100.0f);
    outputs.specific_force_up_mps2 = 0.0f;
    outputs.thrust_cos_tilt = 1.0f;
    outputs.throttle_direct = true;
  }
  // 注意：throttlePercent 全局量在 mix_and_output_commands() 末尾统一回填；
  // 自动模式下本函数不再提前给出油门，避免控制层提前完成机型推力逆解。
}

/**
 * @brief 初始化水平位置保持链（位置环 → 速度环）的限幅/滤波/积分界
 * @pre  首次进入 AUTO_POSITION 时调用；参数全部取自 kFlightCtrlParams
 * @post 位置/速度环 PID 清零并配好限幅；thrust_comp 滤波与 HorizontalKF 归零
 * @note  位置环输入 m、输出 m/s（限幅 ±kPosCtrlMaxSpeedCmd）；
 *        速度环输入 m/s、输出 m/s²（限幅 ±kMaxAccelCmd）。
 *        积分界用 int_limit（输出量纲），实际积分状态由 PositionPID 反推钳位。
 * @ref   CTL-001 §8.4（串级定高/水平环）；见 PositionPID.h
 * @units m、m/s、m/s²；PID 步长 0.005 s（200 Hz）
 */
void initPositionHold()
{
  // --- 1. 目标位置初始化 ---
  // Loiter 目标点是动态设置的：进入定点模式时由 handlePositionControl
  // 把 targetNorth/targetEast 锁到当前点，故这里无需预置。

  // --- 2. 位置环PID初始化 (输入: m, 输出: m/s) ---
  // ★ 2026-08-08 C路径重构：限幅/积分/滤波统一读自 kFlightCtrlParams（§4.0）
  northPosPID.reset();
  eastPosPID.reset();
  // 设置位置环输出限幅 (最大目标速度)
  northPosPID.setOutputLimits(kFlightCtrlParams.pos_n.out_min, kFlightCtrlParams.pos_n.out_max);
  eastPosPID.setOutputLimits(kFlightCtrlParams.pos_e.out_min, kFlightCtrlParams.pos_e.out_max);
  // 设置位置环的微分滤波系数
  northPosPID.setFilterCoefficient(kFlightCtrlParams.pos_n.filter_alpha);
  eastPosPID.setFilterCoefficient(kFlightCtrlParams.pos_e.filter_alpha);
  // 设置积分限幅
  northPosPID.setIntegralLimit(kFlightCtrlParams.pos_n.int_limit);
  eastPosPID.setIntegralLimit(kFlightCtrlParams.pos_e.int_limit);

  // --- 3. 速度环PID初始化 (输入: m/s, 输出: m/s^2) ---
  northVelPID.reset();
  eastVelPID.reset();

  // 配置 PID 输出限幅 (现在输出的是物理加速度)
  northVelPID.setOutputLimits(kFlightCtrlParams.vel_n.out_min, kFlightCtrlParams.vel_n.out_max);
  eastVelPID.setOutputLimits(kFlightCtrlParams.vel_e.out_min, kFlightCtrlParams.vel_e.out_max);

  // 配置积分限幅 (通常设为最大输出的一半或更小)
  northVelPID.setIntegralLimit(kFlightCtrlParams.vel_n.int_limit);
  eastVelPID.setIntegralLimit(kFlightCtrlParams.vel_e.int_limit);
  // 设置速度环的微分滤波系数
  northVelPID.setFilterCoefficient(kFlightCtrlParams.vel_n.filter_alpha);
  eastVelPID.setFilterCoefficient(kFlightCtrlParams.vel_e.filter_alpha);

  // --- 4. 状态标志与输出初始化 ---
  positionHoldEnabled = true; // 标记已初始化
  thrust_comp_N = 0.0f;       // 清零初始输出
  thrust_comp_E = 0.0f;

  // *** 新增：初始化并重置滤波器 ***
  thrustCompN_filter.initialize(0.0f);
  thrustCompE_filter.initialize(0.0f);

  // --- 新增：重置水平卡尔曼滤波器 ---
  kf_north.reset();
  kf_east.reset();
}

/**
 * @brief 水平定点：Loiter 状态机 + 位置/速度串级 → 目标倾斜推力分量
 * @param[in] roll_rc_raw  滚转通道原始 PWM（988–2012 µs）
 * @param[in] pitch_rc_raw 俯仰通道原始 PWM（988–2012 µs）
 * @return 无；更新全局 thrust_comp_N / thrust_comp_E（单位推力矢量水平分量）
 * @pre  仅 AUTO_POSITION / AUTO_ALTITUDE 有实际作用，其他模式复位并返回
 * @formula 刹车点 s = v·|v| / (2·a_max)·k_safe，a_max = g·tan(15°)；
 *          速度环 a = kp·Δv + ki∫Δv + kd·d(−v)/dt；
 *          加速度圆限幅 |a| ≤ MAX_ACCEL_CMD；推力分量 = a_i / |[a_n,a_e,g+a_z]|
 * @ref  CTL-001 §8.4；见 THY-003 §1（牛顿–欧拉）
 * @units 位置 m、速度 m/s、加速度 m/s²、倾角 rad；PID 步长 0.005 s
 * @note  北向由 pitch 摇杆、东向由 roll 摇杆映射（体轴映射见 AGENTS.md 低速悬停
 *        闭环口径踩坑记录，2026-10-21 修正；旧实现把东向塞进偏航会漂）。
 *        刹车预测点限幅 ±5 m，防止速度估计异常把目标点设到数公里外。
 */
void handlePositionControl(float roll_rc_raw, float pitch_rc_raw)
{
  // --- 1. 模式检查与初始化 ---
  ControlMode currentMode = getControlMode(raw_rc_values[6]);
  if (currentMode != AUTO_POSITION && currentMode != AUTO_ALTITUDE)
  {
    if (positionHoldEnabled)
    {
      positionHoldEnabled = false;
      northPosPID.reset();
      eastPosPID.reset();
      northVelPID.reset();
      eastVelPID.reset();
      thrust_comp_N = 0.0f;
      thrust_comp_E = 0.0f;
      loiter_state = LOITER_HOVERING; // 退出模式时重置状态机
    }
    return;
  }

  // 如果是首次进入自动位置模式，则执行初始化
  if (!positionHoldEnabled)
  {
    // 调用初始化函数，该函数应设置目标位置为当前位置，并配置PID参数
    initPositionHold();
    // 首次进入时，将当前位置设为目标悬停点
    targetNorth = relative_north;
    targetEast = relative_east;
    loiter_state = LOITER_HOVERING; // 初始状态为悬停
  }

  // --- 2. 数据采集 (NED坐标系) ---
  // 使用全局计算好的相对位置
  float currentNorth = relative_north;
  float currentEast = relative_east;
  // 速度已包含光流与卫导的组合切换逻辑，本函数不再做数据源选择。
  // 历史遗留：这里曾按 flow_data.is_flow_valid 直接替换速度、或把机体系光流
  // 速度 bodyToNed 到 NED，两套分支均已并入导航层 EKF（光流/测距是两条独立
  // 新鲜度通道，见 AGENTS.md 光流踩坑记录），故不再保留本地换算。
  float currentVelNorth = INS_GNSS_Packet.velocity_north;
  float currentVelEast = INS_GNSS_Packet.velocity_east;

  // 判断摇杆是否在中心死区内
  bool sticks_in_deadzone = (fabsf(roll_rc_raw - 1500.0f) < RC_LOITER_DEADZONE) &&
                            (fabsf(pitch_rc_raw - 1500.0f) < RC_LOITER_DEADZONE);

  // sticks_in_deadzone = false; // 内环测试时取消注释，将摇杆死区设为0，进入速度控制模式
  // 目标速度变量，将由状态机决定其值
  targetVelNorth = 0.0f;
  targetVelEast = 0.0f;

  // --- 3. Loiter 状态机逻辑 ---
  if (sticks_in_deadzone)
  {
    /*
     * [ 物理制导优化：基于最大倾角限制的刹车点预测 ]
     * 原理：
     * 1. 在维持高度平衡的前提下，最大水平减速度 a_max = G * tan(max_tilt)。
     * 2. 根据运动学方程 v^2 = 2as，最短制动距离 s = v^2 / (2a)。
     * 3. 引入安全系数 (BRAKE_SAFETY_FACTOR)，为位置环 PID 留出调节余量，防止舵机打满。
     */

    // --- 物理参数定义 ---
    const float MAX_TILT_DEG = 15.0f;       // 系统允许的最大控制倾角 (度)
    const float BRAKE_SAFETY_FACTOR = 1.4f; // 安全系数：1.4表示预测点比理论极限远40%，增加平滑度
    // 重力加速度用 EKF 当地值 ekf_gravity_mps2（= G_ACCEL_CONST 回退，来源 aircraft-model.json）

    // 1. 计算当前倾角限制下的最大物理减速度 (m/s^2)
    // a_max 约为 2.62 m/s^2 (在 15度 倾角时)
    const float a_max = ekf_gravity_mps2 * tanf(MAX_TILT_DEG * DEG_TO_RAD);

    if (loiter_state == LOITER_MOVING)
    {
      // --- 垂直通道状态清理 ---
      // 切换到悬停前，重置位置环积分，防止之前的移动误差干扰刹车动作
      northPosPID.reset();
      eastPosPID.reset();

      // 2. 计算北向 (North) 预测刹车位移
      // 使用 v * |v| 保持速度方向，并计算 v^2 / (2 * a_max)
      // 引入安全系数 BRAKE_SAFETY_FACTOR 以获得更平滑的减速曲线
      float brake_dist_n = (currentVelNorth * fabsf(currentVelNorth)) / (2.0f * a_max) * BRAKE_SAFETY_FACTOR;

      // 3. 计算东向 (East) 预测刹车位移
      float brake_dist_e = (currentVelEast * fabsf(currentVelEast)) / (2.0f * a_max) * BRAKE_SAFETY_FACTOR;

      // 4. 预测点距离限幅 (故障保护)
      // 防止速度估算异常导致预测点被设在数公里之外
      const float MAX_BRAKE_DIST = 5.0f; // 最大允许预测 5 米外的停止点
      brake_dist_n = constrain(brake_dist_n, -MAX_BRAKE_DIST, MAX_BRAKE_DIST);
      brake_dist_e = constrain(brake_dist_e, -MAX_BRAKE_DIST, MAX_BRAKE_DIST);

      // 5. 锁定目标点：当前位置 + 物理预测位移
      targetNorth = currentNorth + brake_dist_n;
      targetEast = currentEast + brake_dist_e;

      // 打印调试信息（可选，用于地面站调参）
      // Serial8.printf("Brake Locked! Vel: %.2f, Dist: %.2f\n", currentVelNorth, brake_dist_n);
    }

    // 状态机切换为悬停状态
    loiter_state = LOITER_HOVERING;

    // 运行位置环，计算修正速度
    float northError = targetNorth - currentNorth;
    float eastError = targetEast - currentEast;
    targetVelNorth = northPosPID.computeWithExternalDerivative(northError, 0, currentVelNorth, 0.005f);
    targetVelEast = eastPosPID.computeWithExternalDerivative(eastError, 0, currentVelEast, 0.005f);
  }
  else
  {
    // --- 移动状态 (MOVING) ---
    loiter_state = LOITER_MOVING; // 更新状态为移动
                                  // 将摇杆输入映射为机体坐标系下的期望速度，并考虑死区
    // 定义摇杆中心值和最大偏差
    const float RC_CENTER = 1500.0f;
    const float RC_MAX_DEVIATION = 512.0f; // 2012 - 1500 = 512, 1500 - 988 = 512

    float body_vel_x = 0.0f; // 初始化为0，如果在死区内则保持为0
    // Pitch杆控制机体X轴速度 (向前/向后)
    if (pitch_rc_raw > RC_CENTER + RC_LOITER_DEADZONE)
    {
      // 摇杆在正向死区外：将 (死区边缘, 最大值] 映射到 (0, MAX_LOITER_SPEED_CMD]
      body_vel_x = mapFloat(pitch_rc_raw, RC_CENTER + RC_LOITER_DEADZONE, RC_CENTER + RC_MAX_DEVIATION, 0.0f, MAX_LOITER_SPEED_CMD);
    }
    else if (pitch_rc_raw < RC_CENTER - RC_LOITER_DEADZONE)
    {
      // 摇杆在负向死区外：将 [最小值, 死区边缘) 映射到 [-MAX_LOITER_SPEED_CMD, 0)
      body_vel_x = mapFloat(pitch_rc_raw, RC_CENTER - RC_MAX_DEVIATION, RC_CENTER - RC_LOITER_DEADZONE, -MAX_LOITER_SPEED_CMD, 0.0f);
    }
    // 如果摇杆在死区内 (RC_CENTER - RC_LOITER_DEADZONE <= pitch_rc_raw <= RC_CENTER + RC_LOITER_DEADZONE)，
    // body_vel_x 保持其初始值 0.0f。

    float body_vel_y = 0.0f; // 初始化为0，如果在死区内则保持为0
    // Roll杆控制机体Y轴速度 (向右/向左)
    if (roll_rc_raw > RC_CENTER + RC_LOITER_DEADZONE)
    {
      // 摇杆在正向死区外：将 (死区边缘, 最大值] 映射到 (0, MAX_LOITER_SPEED_CMD]
      body_vel_y = mapFloat(roll_rc_raw, RC_CENTER + RC_LOITER_DEADZONE, RC_CENTER + RC_MAX_DEVIATION, 0.0f, MAX_LOITER_SPEED_CMD);
    }
    else if (roll_rc_raw < RC_CENTER - RC_LOITER_DEADZONE)
    {
      // 摇杆在负向死区外：将 [最小值, 死区边缘) 映射到 [-MAX_LOITER_SPEED_CMD, 0)
      body_vel_y = mapFloat(roll_rc_raw, RC_CENTER - RC_MAX_DEVIATION, RC_CENTER - RC_LOITER_DEADZONE, -MAX_LOITER_SPEED_CMD, 0.0f);
    }
    // 如果摇杆在死区内，body_vel_y 保持其初始值 0.0f。

    // 将机体坐标系下的期望速度，通过当前偏航角，旋转到导航坐标系(NED)
    float yaw_rad = AHRS_Packet.Heading;
    bodyToNed(body_vel_x, body_vel_y, yaw_rad, targetVelNorth, targetVelEast);

    // 在移动模式下，位置环积分项清零，防止积分饱和
    northPosPID.reset();
    eastPosPID.reset();
  }

  // --- 4. 核心优化：速度环 -> 输出物理加速度 (Target Acceleration) ---
  // 物理意义: Kp * (V_target - V_current) = 期望的加速度 (m/s^2)
  // 这里的 PID 输出直接就是 a_x 和 a_y
  float target_accel_n = northVelPID.computeDerivativeOnMeasurement(targetVelNorth, currentVelNorth, 0.005f);
  float target_accel_e = eastVelPID.computeDerivativeOnMeasurement(targetVelEast, currentVelEast, 0.005f);

  // --- 5. 加速度矢量圆形限幅 ---
  // 避免 "方形限幅" (即 N 和 E 分别限幅) 导致斜向加速时总加速度过大
  float accel_mag_sq = target_accel_n * target_accel_n + target_accel_e * target_accel_e;
  float max_accel_sq = MAX_ACCEL_CMD * MAX_ACCEL_CMD;

  if (accel_mag_sq > max_accel_sq)
  {
    float scale = MAX_ACCEL_CMD / sqrtf(accel_mag_sq);
    target_accel_n *= scale;
    target_accel_e *= scale;
  }

  // ========================================================================
  // --- 6. 物理模型逆解：目标加速度矢量 -> 推力方向矢量 ---
  // ========================================================================
  // 目标: 构造一个单位推力矢量 T_unit，使得其产生的合力满足 F = m * a。
  //
  // 物理推导 (天向/向上视角):
  // 1. 抵消重力加速度所需的基准天向加速度 = ekf_gravity_mps2 (约 9.79~9.81, EKF 当地值)
  // 2. 高度环输出的目标天向加速度增量 = target_accel_z_up_global (向上为正)
  // 3. 所需的总天向比力 (Total Vertical Specific Force) = ekf_gravity_mps2 + target_accel_z_up_global
  //    (注：即使在悬停状态，电机也必须提供 1g 的天向比力)
  //
  // 4. 水平向比力需求即为目标水平加速度: [target_accel_n, target_accel_e]
  //
  // 5. 三轴总比力矢量模长 (Total Force Norm):
  //    Norm = sqrt( Accel_N^2 + Accel_E^2 + (G + Accel_Up)^2 )

  // 计算三轴合成比力矢量的模长
  // 引入 target_accel_z_up_global 实现了水平环对垂直环动态的实时感知
  float total_z_force_demand = ekf_gravity_mps2 + target_accel_z_up_global;
  float total_force_norm = sqrtf(target_accel_n * target_accel_n +
                                 target_accel_e * target_accel_e +
                                 total_z_force_demand * total_z_force_demand);

  // 安全保护：防止自由落体状态或计算异常导致的除零错误
  if (total_force_norm < 0.5f)
    total_force_norm = 0.5f;

  // 计算单位推力矢量在水平方向的分量 (即目标倾角的 sin 值)
  // 核心逻辑：当垂直推力需求增加时，total_force_norm 增大，水平分量会自动“收缩”，
  // 从而在不改变总推力模长的情况下，精确维持目标的水平加速度。
  float raw_thrust_comp_N = target_accel_n / total_force_norm;
  float raw_thrust_comp_E = target_accel_e / total_force_norm;

  // --- 7. 滤波输出 ---
  // 对目标矢量进行低通滤波，使姿态变化更柔和，减少电机/舵机抖动
  thrust_comp_N = thrustCompN_filter.filter(raw_thrust_comp_N);
  thrust_comp_E = thrustCompE_filter.filter(raw_thrust_comp_E);

  // 最终的安全检查 (防止数值误差导致 sin(theta) > 1)
  // 实际上前面的加速度限幅已经保证了这一点，这里是双重保险
  float thrust_comp_mag_sq = thrust_comp_N * thrust_comp_N + thrust_comp_E * thrust_comp_E;
  if (thrust_comp_mag_sq > POS_CTRL_MAX_THRUST_COMP * POS_CTRL_MAX_THRUST_COMP)
  {
    float scale = POS_CTRL_MAX_THRUST_COMP / sqrtf(thrust_comp_mag_sq);
    thrust_comp_N *= scale;
    thrust_comp_E *= scale;
  }
  // 经过这一步，thrust_comp_N/E 就是最终要传递给姿态生成函数的输入。
  // constructTiltTargetQuaternion 函数内部会处理从这些分量到完整单位矢量的构造。
}

/**
 * @brief 姿态目标生成：定点/制导走推力矢量逆解，其余模式走摇杆或速率直通
 * @param[in]  inputs  控制输入（姿态子模式、摇杆、航向）
 * @param[in]  mode    当前飞行模式
 * @param[out] q_target 目标姿态四元数（机体系→NED，分量 {w,x,y,z}）
 * @pre  姿态量 AHRS_Packet 已更新；AUTO_POSITION 时 thrust_comp_N/E 已由
 *       handlePositionControl 本拍算好
 * @formula q_target = q_tilt ⊗ Rz(Heading)（定点/制导，先倾斜再叠航向）；
 *          姿态模式 q_target = Rz(Heading) ⊗ Rxy(rollTarget, pitchTarget)；
 *          倾角来自 constructTiltTargetQuaternion(comp_N, comp_E)
 * @ref  MOD-002 §3（四元数右乘/分量顺序）；见 CTL-001 §8.6
 * @units 角度 rad（摇杆目标角先乘 DEG_TO_RAD）；杆量 PWM µs
 * @note  ① 存档系 z_b=推力轴 → 悬停静止 roll=pitch=0，直接叠 Rz(Heading) 即可，
 *          不加"悬停基态补偿"，天然避开欧拉奇异点（2026-08-07 恢复）。
 *        ② 定点模式切回时把 positionHoldEnabled 清掉，强制重锁当前点，
 *          否则会自主飞回上一段悬停点（2026-08-10 修复，见函数内注释）。
 */
void generate_attitude_target(const ControlInputs_t &inputs, ControlMode mode, Quaternion &q_target)
{
  if (mode == AUTO_POSITION)
  {
    // ★ 2026-08-10 全面审查修复：每次从其他模式切入 AUTO_POSITION 时重新锁当前点。
    //   handlePositionControl 内部的复位分支（L565）不可达——函数只在 AUTO_POSITION
    //   被调用，内部 currentMode 恒为 AUTO_POSITION，positionHoldEnabled 永不复位，
    //   目标点残留 → 从手动/定高切回定点时飞行器自动飞回旧悬停点（无操纵自主机动）。
    static ControlMode s_prev_mode = MANUAL;
    if (s_prev_mode != AUTO_POSITION)
      positionHoldEnabled = false;  // 触发 handlePositionControl 内重新 init + 锁当前点
    s_prev_mode = mode;
    handlePositionControl(inputs.roll_raw, inputs.pitch_raw);
    // `handlePositionControl` 更新全局的 thrust_comp_N/E
  }
  else if (mode == GUIDED)
  {
    // --- 制导计算机输出期望物理加速度 (Target Acceleration) ---
    float target_accel_n = guidance_accel_N_cmd;
    float target_accel_e = guidance_accel_E_cmd;

    // --- 加速度矢量圆形限幅 ---
    // 避免 "方形限幅" (即 N 和 E 分别限幅) 导致斜向加速时总加速度过大
    float accel_mag_sq = target_accel_n * target_accel_n + target_accel_e * target_accel_e;
    float max_accel_sq = MAX_ACCEL_CMD * MAX_ACCEL_CMD;

    if (accel_mag_sq > max_accel_sq)
    {
      float scale = MAX_ACCEL_CMD / sqrtf(accel_mag_sq);
      target_accel_n *= scale;
      target_accel_e *= scale;
    }

    // --- GUIDED模式也必须使用全比力归一化 ---
    // 此时 target_accel_z_up_global 已经被 compute_thrust_control 更新为 guidance_accel_U_cmd
    // 重力项用 EKF 当地值 ekf_gravity_mps2（Somigliana，未初始化时回退 9.81）
    float total_z_demand = ekf_gravity_mps2 + target_accel_z_up_global;

    float total_force_norm = sqrtf(target_accel_n * target_accel_n +
                                   target_accel_e * target_accel_e +
                                   total_z_demand * total_z_demand);

    // 计算单位推力矢量分量
    float raw_thrust_comp_N = target_accel_n / total_force_norm;
    float raw_thrust_comp_E = target_accel_e / total_force_norm;

    // --- 滤波输出 ---
    // GUIDED 模式刻意不做低通：制导计算机已给出平滑的加速度指令，
    // 再滤波只会增加相位滞后（AUTO_POSITION 分支才用 thrustComp 滤波器）。
    thrust_comp_N = raw_thrust_comp_N;
    thrust_comp_E = raw_thrust_comp_E;

    // 最终的安全检查 (防止数值误差导致 sin(theta) > 1)
    // 实际上前面的加速度限幅已经保证了这一点，这里是双重保险
    float thrust_comp_mag_sq = thrust_comp_N * thrust_comp_N + thrust_comp_E * thrust_comp_E;
    if (thrust_comp_mag_sq > POS_CTRL_MAX_THRUST_COMP * POS_CTRL_MAX_THRUST_COMP)
    {
      float scale = POS_CTRL_MAX_THRUST_COMP / sqrtf(thrust_comp_mag_sq);
      thrust_comp_N *= scale;
      thrust_comp_E *= scale;
    }

    // 重置内置位置PID
    if (positionHoldEnabled)
    {
      northPosPID.reset();
      eastPosPID.reset();
      northVelPID.reset();
      eastVelPID.reset();
    }
  }

  if (mode == AUTO_POSITION || mode == GUIDED)
  {
    // 计算推力矢量控制的目标姿态四元数
    // 该过程分为两个步骤：
    // 1. 根据北向和东向推力分量计算倾斜四元数
    // 2. 将倾斜四元数与当前航向四元数结合，得到最终目标姿态

    // 用于存储倾斜目标姿态的四元数（仅包含滚转和俯仰，不含偏航）
    Quaternion q_tilt_target;

    // 根据推力分量计算倾斜目标四元数
    // thrust_comp_N: 北向推力分量（控制俯仰）
    // thrust_comp_E: 东向推力分量（控制滚转）
    if (constructTiltTargetQuaternion(thrust_comp_N, thrust_comp_E, q_tilt_target))
    {
      // ★ 与原始 VTVL 实飞存档版完全一致（2026-08-07 恢复）：
      // 存档系 z_b = 推力轴 → 悬停静止 roll=pitch=0，绕 z_b 即世界航向，
      // 故"保持当前航向"就是标准 Rz(Heading)，无需悬停基态补偿。
      Quaternion q_yaw_base = eulerToQuaternion(0.0f, 0.0f, AHRS_Packet.Heading);

      // 将倾斜四元数与航向四元数相乘，得到完整的目标姿态
      q_target = quaternionMultiply(q_tilt_target, q_yaw_base);

      // 归一化目标四元数，确保单位四元数特性
      q_target = normalizeQuaternion(q_target);
    }
    else
    {
      // 如果倾斜四元数计算失败（如推力过小），则保持当前实际姿态作为目标姿态，这通常发生在推力不足或计算异常的情况下
      q_target = {AHRS_Packet.Qw, AHRS_Packet.Qx, AHRS_Packet.Qy, AHRS_Packet.Qz};
    }

    // 将目标姿态四元数转换为欧拉角
    float yawTarget;
    quaternionToEuler(q_target, rollTarget, pitchTarget, yawTarget);
  }
  else
  { // MANUAL 模式和定高模式 - 手动遥控姿态角度飞行模式

    // 根据遥控器的姿态模式开关选择控制方式
    if (inputs.attitude_mode == ATTITUDE_MODE)
    {
      // 姿态模式：直接控制飞行器姿态角
      // 将遥控器摇杆值（988-2012）映射为姿态角命令（-MAX_ANGLE_COMMAND 到 +MAX_ANGLE_COMMAND）

      // 俯仰角目标值：遥控器前后摇杆控制
      // ★ 2026-08-10 全面审查修复：注释与代码符号统一（此前注释"前推→负值"与代码
      //   相反——代码 988→+MAX 在 θ̇≈−q 约定下即低头，实机行为正确，注释是回归诱因）
      // 前推摇杆 -> 正值（机头下俯，θ̇≈−q 下 q>0=低头）
      // 后拉摇杆 -> 负值（机头上仰）
      pitchTarget = mapFloat(inputs.pitch_raw, 988.0f, 2012.0f, MAX_ANGLE_COMMAND, -MAX_ANGLE_COMMAND);

      // 滚转角目标值：遥控器左右摇杆控制
      // 左打摇杆 -> 负值（左滚）
      // 右打摇杆 -> 正值（右滚）
      rollTarget = mapFloat(inputs.roll_raw, 988.0f, 2012.0f, -MAX_ANGLE_COMMAND, MAX_ANGLE_COMMAND);

      // ★ 与原始 VTVL 实飞存档版完全一致（2026-08-07 恢复）：
      // 机体系 z_b = 推力轴（机头朝天时指向地面）→ 悬停静止解算 roll=pitch=0，
      // 故"保持当前航向 + 摇杆倾斜"就是标准 Rz(Heading)⊗Rxy(roll,pitch)，
      // 无需悬停基态补偿、天然避开欧拉奇异点。
      Quaternion q_yaw_base = eulerToQuaternion(0.0f, 0.0f, AHRS_Packet.Heading);

      // 构建机体倾斜四元数（仅滚转/俯仰，无偏航）
      Quaternion q_tilt_body = eulerToQuaternion(rollTarget * DEG_TO_RAD, pitchTarget * DEG_TO_RAD, 0.0f);

      // 合成最终目标姿态四元数
      // 先应用机体倾斜，再应用悬停基态+航向旋转
      q_target = quaternionMultiply(q_yaw_base, q_tilt_body);

      // 归一化目标四元数，确保数学正确性
      q_target = normalizeQuaternion(q_target);
    }
    else
    { // RATE_MODE - 角速度控制模式

      // 角速度模式：直接控制飞行器角速度
      // 目标姿态保持当前实际姿态，通过角速度PID控制器实现遥控控制
      q_target = {AHRS_Packet.Qw, AHRS_Packet.Qx, AHRS_Packet.Qy, AHRS_Packet.Qz};

      // 目标姿态保持当前实际姿态，因为角速度模式下不使用姿态角控制
      rollTarget = AHRS_Packet.Roll;
      pitchTarget = AHRS_Packet.Pitch;
    }
  }
}

// ★ 2026-08-09 yaw 航向锁（ratchet hold）状态：
//   摇杆偏离 → 角速度指令（原行为）+ 持续刷新保持参考（回中即锁当时航向）；
//   摇杆回中 → 航向短弧误差 × att_yaw.kp → 速率指令。
//   解锁瞬间参考 = 当前航向（防旧参考导致起飞猛转）。
static bool  s_yaw_hold_armed = false;
static float s_yaw_hold_ref_rad = 0.0f;

// ═══════════════════════════════════════════════════════════════════════════
// 【控制链】姿态外环 → 角速率内环（roll / pitch / yaw 三通道）
//   execute_attitude_controller：四元数误差 → 目标角速率 → 角加速度指令
//   execute_yaw_controller     ：航向锁（ratchet hold）→ 目标角速率 → 角加速度
//   单位口径：外环 deg→deg/s，内环 deg/s→deg/s²；只在写 ControlOutputs_t 时
//   转 rad/s²。见下方"角速率内环量纲"红线块。
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief 滚转/俯仰姿态控制：姿态外环（角度模式）→ 角速率内环 → α 指令
 * @param[in]  inputs  控制输入（解锁、手动TVC旁路、姿态子模式、摇杆）
 * @param[in]  q_target 目标姿态四元数（{w,x,y,z}，机体系→NED）
 * @param[out] outputs 写 alpha_roll_radps2 / alpha_pitch_radps2 [rad/s²]
 * @pre  陀螺已减 EKF 零偏；姿态控制门控 = is_unlocked && !is_manual_tvc
 * @formula 外环 Δω_ref = kp_a·Δθ + ki_a∫Δθ − kd_a·ω（输出限幅 ±80 deg/s）；
 *          内环 α_ref = kp_r·(ω_ref−ω) + ki_r∫(ω_ref−ω)（deg/s²）；kd=0
 * @ref  CTL-001 §4–§5（SAS 三通道/极性）；见 MOD-002 §3（四元数误差右乘）
 * @units 角度 deg、角速率 deg/s、角加速度 deg/s²（仅写输出时转 rad/s²）
 * @note  ① 姿态误差角取 2·atan2(|q_vec|, |q_w|)，大误差下精确、小误差退化为
 *          2·q_vec；q_w 取正保证最短路径。
 *        ② 姿态外环的 D 项用外部导数（陀螺），传参语义为 d(input)/dt，
 *          PositionPID 内部取负作误差导数。
 *        ③ RATE_MODE 走 FPV 曲线直通目标角速率，并复位外环/内环 PID，
 *          避免模式切换时携带旧积分。
 */
static Vector3 getSanitizedBodyGyro();

void execute_attitude_controller(const ControlInputs_t &inputs, const Quaternion &q_target, ControlOutputs_t &outputs)
{
  // 获取当前角速率
  Vector3 current_omega_rps_body = getSanitizedBodyGyro();
  // （getSanitizedBodyGyro 定义在混控段之前，见文件下方静态函数）
  // ★ 2026-08-09 陀螺动态零偏补偿："飘"根因修复——EKF 实时估计的零偏
  //   （温度漂移 0.2-0.4°/s，初值不确定度 0.4°/s）此前只用于日志，控制
  //   用静态校准原始值 → 速率模式持续旋转（每分钟 12-24°）。补偿后速率
  //   反馈与 EKF 姿态同一参考系。仅 EKF 初始化后补偿（收敛前估计不可靠）。
  if (nav_system_initialized)
  {
    const Eigen::Vector3f gb = nav_ekf.gyro_bias_radps();
    current_omega_rps_body.x -= gb(0);
    current_omega_rps_body.y -= gb(1);
    current_omega_rps_body.z -= gb(2);
  }
  // 滤波链（2026-08-09 审计后近全关：α1=0.4 / 二级直通——物理减震底座已隔离振动）
  current_omega_dps_body_filtered = {
      rollSpeedFilter2.filter(rollSpeedFilter.filter(current_omega_rps_body.x * RAD_TO_DEG)),
      pitchSpeedFilter2.filter(pitchSpeedFilter.filter(current_omega_rps_body.y * RAD_TO_DEG)),
      yawSpeedFilter2.filter(yawSpeedFilter.filter(current_omega_rps_body.z * RAD_TO_DEG))};

  bool is_attitude_control_active = inputs.is_unlocked && !inputs.is_manual_tvc;
  if (is_attitude_control_active)
  {
    // 姿态模式外环控制：姿态角 -> 目标角速度
    // 该部分为外环PID控制器，将姿态角误差转换为目标角速度
    if (inputs.attitude_mode == ATTITUDE_MODE)
    {
      // 获取当前实际姿态四元数（从AHRS传感器数据）
      // 包含当前飞行器实际的滚转、俯仰、偏航信息
      Quaternion q_current = {AHRS_Packet.Qw, AHRS_Packet.Qx, AHRS_Packet.Qy, AHRS_Packet.Qz};

      // 计算当前姿态到目标姿态的误差四元数
      // 通过四元数共轭和乘法得到姿态误差，表示从当前姿态到目标姿态的旋转
      // q_error = q_current⁻¹ * q_target
      Quaternion q_error = quaternionMultiply(quaternionConjugate(q_current), q_target);

      // 确保四元数符号一致性（选择最短路径旋转）
      // 四元数q和-q表示相同的旋转，但插值方向相反
      // 通过选择w分量为正的版本，确保旋转路径最短
      float sign_qw = (q_error.w >= 0.0f) ? 1.0f : -1.0f;

      // ── 姿态外环（四元数误差 → 目标角速率）────────────────────────────
      // 体轴映射（存档系，★以 mix 层实码为准）：滚转=绕 x_b、俯仰=绕 y_b、
      // 偏航=绕 z_b=推力轴（悬停时竖直，即世界航向）。
      // 故误差四元数分量：q_err.x → 滚转外环（上摆 δf）、q_err.y → 俯仰外环
      // （下摆 δt）、q_err.z → 航向（由 execute_yaw_controller 统一处理）。
      // ⚠️ 旧注释"x_b=机身纵轴=推力轴"把论文模型系（x'=推力轴）与固件存档系
      //   （z_b=推力轴）混读，已废止：本固件滚转永远指"绕 x_b"。
      // 使用完整四元数向量模长，确保多轴同时存在误差时 atan2 参数正确
      // 原代码仅用 sqrt(y²+z²)，当偏航误差（x分量）不为零时会低估模长，
      // 导致进入 atan2 分支的判断偏低，large-angle精确缩放系数偏大
      float q_vec_norm = sqrtf(q_error.x * q_error.x +
                               q_error.y * q_error.y +
                               q_error.z * q_error.z);
      float precise_scale;
      if (q_vec_norm > 0.25f)
      {
        precise_scale = 2.0f * atan2f(q_vec_norm, fabsf(q_error.w)) / q_vec_norm * RAD_TO_DEG;
      }
      else
      {
        precise_scale = 2.0f * RAD_TO_DEG;
      }
      gnc_tel.error_deg[0] = sign_qw * q_error.x * precise_scale;  // 滚转误差 = 体轴x分量
      gnc_tel.error_deg[1] = sign_qw * q_error.y * precise_scale;

      // 外环：滚转外部导数取 omega.x（体轴x = FRD 滚转速率）
      // 传参为 d(input)/dt 语义：input=0 常数，误差导数 = -omega；
      // computeWithExternalDerivative 内部取 -derivative 作误差导数，故此处传 +omega。
      gnc_tel.omega_ref_dps[0] = rollAnglePID.computeWithExternalDerivative(gnc_tel.error_deg[0], 0, current_omega_dps_body_filtered.x, 0.005f);
      gnc_tel.omega_ref_dps[1] = pitchAnglePID.computeWithExternalDerivative(gnc_tel.error_deg[1], 0, current_omega_dps_body_filtered.y, 0.005f);

      // ★ yaw 摇杆不在此处处理：存档约定下 yaw 恒为角速度指令，
      //   由 execute_yaw_controller 统一映射（两种模式一致）。

      // 限制目标角速度在安全范围内
      // 防止PID输出过大导致危险动作
      gnc_tel.omega_ref_dps[0] = constrain(gnc_tel.omega_ref_dps[0], -MAX_TARGET_RATE, MAX_TARGET_RATE);
      gnc_tel.omega_ref_dps[1] = constrain(gnc_tel.omega_ref_dps[1], -MAX_TARGET_RATE, MAX_TARGET_RATE);
    }
    else
    { // RATE_MODE —— 摇杆直接生成目标角速率（★2026-08-11 FPV 摇杆曲线接入）
      //   roll 摇杆 → 绕 x_b（上摆通道）；pitch 摇杆 → 绕 y_b（下摆通道）
      //   yaw 摇杆 → 绕 z_b（差速），由 execute_yaw_controller 处理
      //   曲线：归一化 → 死区 → expo（中心）→ rc_rate（灵敏度）→ super（边缘）
      //   满杆角速度 = rcRate×200/(1−super)，默认 roll/pitch 333°/s、yaw 222°/s
      gnc_tel.omega_ref_dps[0] = rcRateCurve(
          rcNormFromUs(inputs.roll_raw),
          kFlightCtrlParams.rc_rate, kFlightCtrlParams.rc_expo[0],
          kFlightCtrlParams.rc_super[0], MAX_MANUAL_rollRATE);
      gnc_tel.omega_ref_dps[1] = -rcRateCurve(
          rcNormFromUs(inputs.pitch_raw),
          kFlightCtrlParams.rc_rate, kFlightCtrlParams.rc_expo[1],
          kFlightCtrlParams.rc_super[1], MAX_MANUAL_pitchRATE); // 推杆=低头负角速度（取反）
      rollAnglePID.reset();
      pitchAnglePID.reset();
      rollRatePID.reset();
      pitchRatePID.reset();
    }

    // 滤波目标角速率
    gnc_tel.omega_ref_dps[0] = rollAngleOutputFilter.filter(gnc_tel.omega_ref_dps[0]);
    gnc_tel.omega_ref_dps[1] = pitchAngleOutputFilter.filter(gnc_tel.omega_ref_dps[1]);

    // ── 角速率内环（角速率误差 → 角加速度指令）────────────────────────
    // 内环统一角度域：角速率误差(deg/s) → 角加速度指令(deg/s²)。
    // 仅在写入 ControlOutputs 时转换为 rad/s²，供惯量逆解使用。
    //
    // ── 角速率内环量纲（2026-08-11 统一，行为红线）──────────────────────
    // 公式：α_ref[deg/s²] = kp[s⁻¹]·(ω_ref−ω)[deg/s] + ki[s⁻²]·∫(ω_ref−ω)dt
    // 现值：kp roll/pitch 16.042818、yaw 11.459156；ki roll/pitch 5.729578、
    //       yaw 4.297184（2026-08-12 下调 25%）；kd 全轴 0。
    // 来源：@ref AGENTS.md 踩坑记录"速率环量纲已统一"
    // 实现：PositionPID::computeDerivativeOnMeasurement 输出 deg/s²，
    //       再经 ControlUnits::dps2ToRadps2 转 rad/s² 写 outputs。
    // 警示：旧"混合域"参数 0.28/0.20/0.10 只有乘 180/π 才与本组等价——
    //       若外部参数文件（0xE1 / 上位机 / 旧存档）继续写旧值，物理增益
    //       会低 57.3 倍；同理 kd 若要从 0 调非零，也必须先 ×57.3。
    //       另：PositionPID 的 D 项按"每拍差分"实现（未除 dt），kd≠0 时
    //       数值量纲隐含 200 Hz，不能当连续域微分用。
    // ────────────────────────────────────────────────────────────────────
    gnc_tel.alpha_ref_dps2[0] = rollRatePID.computeDerivativeOnMeasurement(
        gnc_tel.omega_ref_dps[0], current_omega_dps_body_filtered.x, 0.005f);
    // ★ 2026-08-10 通读修复：原误用 rollRatePID（复制粘贴）——pitchRatePID.compute
    //   从未被调用 → pitch 积分恒 0、且 roll/pitch 共享同一 PID 实例的积分状态互相污染；
    //   在线辨识 i_term[1]=pitchRatePID.getIntegral() 因此恒 0（pitch CG/配平力矩辨识失效）。
    //   迁移前 kp 相同（0.28）掩盖了比例项错误，实机调参稳定期未被发现。修复后 pitch 积分独立。
    gnc_tel.alpha_ref_dps2[1] = pitchRatePID.computeDerivativeOnMeasurement(
        gnc_tel.omega_ref_dps[1], current_omega_dps_body_filtered.y, 0.005f);
    gnc_tel.alpha_ref_dps2[0] = rollOutputFilter.filter(gnc_tel.alpha_ref_dps2[0]);
    gnc_tel.alpha_ref_dps2[1] = pitchOutputFilter.filter(gnc_tel.alpha_ref_dps2[1]);
    outputs.alpha_roll_radps2 = ControlUnits::dps2ToRadps2(gnc_tel.alpha_ref_dps2[0]);
    outputs.alpha_pitch_radps2 = ControlUnits::dps2ToRadps2(gnc_tel.alpha_ref_dps2[1]);
  }
  else
  { // 姿态控制未激活（手动TVC旁路）
    rollAnglePID.reset();
    pitchAnglePID.reset();
    rollRatePID.reset();
    pitchRatePID.reset();
    gnc_tel.omega_ref_dps[0] = 0.0f;
    gnc_tel.omega_ref_dps[1] = 0.0f;
    // 手动TVC：alpha清零，mix层走RC旁路直接控制舵机
    gnc_tel.alpha_ref_dps2[0] = 0.0f;
    gnc_tel.alpha_ref_dps2[1] = 0.0f;
    outputs.alpha_roll_radps2  = 0.0f;
    outputs.alpha_pitch_radps2 = 0.0f;
  }

  // 遥测（★ 2026-08-08 C路径重构：控制链中间量收拢进 gnc_tel §4.0b）
  gnc_tel.alpha_ref_radps2[0] = outputs.alpha_roll_radps2;
  gnc_tel.alpha_ref_radps2[1] = outputs.alpha_pitch_radps2;
}

/**
 * @brief 航向通道：摇杆角速率指令 + 回中航向锁（ratchet hold）→ α_yaw 指令
 * @param[in]  inputs   控制输入（yaw 摇杆、解锁状态）
 * @param[in]  q_target 目标姿态四元数（本控制器不消费：航向参考由 Euler 航向锁
 *                      维护；保留形参只为与 execute_attitude_controller 同签名）
 * @param[out] outputs  写 alpha_yaw_radps2 [rad/s²]
 * @pre  航向取 AHRS_Packet.Heading（NED）；门控 is_unlocked && !is_manual_tvc
 * @formula 杆量偏离：ω_ref = rcRateCurve(Δus/512, rc_rate, expo[2], super[2], 600)
 *          杆量回中：ω_ref = clamp(att_yaw.kp · Δψ, ±att_yaw.int_limit)，
 *          Δψ = 短弧航向误差（±π 归一化）；内环 α_yaw = kp_r·(ω_ref−ω_z)+ki_r∫
 * @ref  CTL-001 §8.6（ratchet hold 与仿真对照）；
 *       @ref TandemVec-Paper/ch/07-control.tex (eq:rc_curve)
 * @units 角度 rad、角速率 deg/s、角加速度 deg/s²；死区 40 µs；限幅 ±24 deg/s
 * @note  ① 偏航摇杆在 ATTITUDE_MODE / RATE_MODE 下都是"角速度指令"（与原始
 *          存档版一致）；回中后的航向保持由本函数的锁逻辑提供，**不是**
 *          q_err 姿态外环——旧注释"ATTITUDE_MODE 走 q_err.z 姿态外环"与实码
 *          不符，已废止（2026-08-09 起为 ratchet hold）。
 *        ② 解锁瞬间以当前航向为参考（防旧参考导致起飞猛转）。
 *        ③ 差速通道受电机 τm=0.28 s 限带，航向环带宽必须保守（CTL-001 §8.6）。
 *        ④ 2026-10-24：`g_pos_hold_degraded`（HeadingAidingGate，无新鲜磁/
 *           YawGsf 航向）时禁用 ratchet hold，回中只给 ω_ref=0；否则会追逐
 *           不可观且漂移的 AHRS 航向，SIL 无磁 nominal 已复现 yaw 自旋。
 * @test test_host/test_flight_control_axis.cpp T17（航向指令跟踪与转角验证）
 */
void execute_yaw_controller(const ControlInputs_t &inputs,
                             const Quaternion      &q_target,
                             ControlOutputs_t      &outputs)
{
  bool attitude_ctrl_active = inputs.is_unlocked && !inputs.is_manual_tvc;

  if (attitude_ctrl_active)
  {
    // ★ 与原始 VTVL 实飞存档版完全一致（2026-08-07 恢复）：
    // 存档系 z_b = 推力轴 → 绕 z_b 即航向（差速通道）。
    // yaw 摇杆**恒为角速度指令**（ATTITUDE_MODE / RATE_MODE 行为相同）：
    // 摇杆偏离 = 角速度指令；回中 = 锁航向（★2026-08-09 ratchet hold，
    // 短弧误差 × att_yaw.kp，限幅 att_yaw.int_limit）。
    // 存档版"松杆即停"由航向锁替代——安全上更优（回中不漂）。
    // 解锁瞬间参考 = 当前航向。
    if (inputs.is_unlocked && !s_yaw_hold_armed)
    {
      s_yaw_hold_ref_rad = AHRS_Packet.Heading;
      s_yaw_hold_armed = true;
    }
    if (!inputs.is_unlocked)
    {
      s_yaw_hold_armed = false;
    }
    const float yaw_stick_us = inputs.yaw_raw - 1500.0f;
    const float YAW_STICK_DEADBAND_US = 40.0f;   // 回中死区（杆位抖动）
    // ★ 2026-10-24 航向不可观时禁止 heading hold：
    //   无磁/无 GNSS 航向时 AHRS_Packet.Heading 的绝对误差不可观测，ratchet
    //   hold 会持续追一个漂移的参考（SIL 无磁 nominal 出现 yaw 自旋）。
    //   失效期间回中只做角速率=0（速率阻尼），不锁绝对航向；摇杆仍可控制 yaw。
    //   磁源恢复且门控解除后，重新以当前航向为参考。位置保持降级与 yaw hold
    //   降级共用同一个观测性门控，避免两处判据漂移。
    const bool heading_hold_allowed = !g_pos_hold_degraded;
    if (!heading_hold_allowed)
    {
      s_yaw_hold_armed = false;
    }
    if (fabsf(yaw_stick_us) > YAW_STICK_DEADBAND_US)
    {
      // ★ 2026-08-11 FPV 摇杆曲线接入（yaw 通道，expo/super 独立参数）
      gnc_tel.omega_ref_dps[2] = rcRateCurve(
          yaw_stick_us / 512.0f,
          kFlightCtrlParams.rc_rate, kFlightCtrlParams.rc_expo[2],
          kFlightCtrlParams.rc_super[2], MAX_MANUAL_yawRATE);
      if (heading_hold_allowed)
      {
        s_yaw_hold_ref_rad = AHRS_Packet.Heading;  // 持续刷新：回中即锁当时航向
      }
    }
    else if (heading_hold_allowed)
    {
      // 回中：航向短弧误差（ref−current，与 roll/pitch 误差约定一致）→ 速率指令
      float yaw_err = s_yaw_hold_ref_rad - AHRS_Packet.Heading;
      while (yaw_err > M_PI)  yaw_err -= 2.0f * (float)M_PI;
      while (yaw_err < -M_PI) yaw_err += 2.0f * (float)M_PI;
      gnc_tel.omega_ref_dps[2] = constrain(
          kFlightCtrlParams.att_yaw.kp * yaw_err * RAD_TO_DEG,
          -kFlightCtrlParams.att_yaw.int_limit,
           kFlightCtrlParams.att_yaw.int_limit);
    }
    else
    {
      // 航向不可观：回中即零角速率（仅保留 yaw 速率阻尼）。
      gnc_tel.omega_ref_dps[2] = 0.0f;
    }
    yawAnglePID.reset();  // 航向不用 PID 外环（自定义 hold 逻辑，增益取自 att_yaw 参数）

    gnc_tel.omega_ref_dps[2] = yawAngleOutputFilter.filter(gnc_tel.omega_ref_dps[2]);

    // 内环统一角度域：速率误差(deg/s) → 角加速度指令(deg/s²)。
    gnc_tel.alpha_ref_dps2[2] = yawRatePID.computeDerivativeOnMeasurement(
        gnc_tel.omega_ref_dps[2], current_omega_dps_body_filtered.z, 0.005f);
    gnc_tel.alpha_ref_dps2[2] = yawOutputFilter.filter(gnc_tel.alpha_ref_dps2[2]);
    outputs.alpha_yaw_radps2 = ControlUnits::dps2ToRadps2(gnc_tel.alpha_ref_dps2[2]);
  }
  else
  {
    yawAnglePID.reset();
    yawRatePID.reset();
    gnc_tel.omega_ref_dps[2] = 0.0f;
    gnc_tel.alpha_ref_dps2[2] = 0.0f;
    outputs.alpha_yaw_radps2 = 0.0f;  // 手动TVC由mix层RC旁路处理；锁定时保持零
  }

  // 遥测（★ 2026-08-08 C路径重构：控制链中间量收拢进 gnc_tel §4.0b）
  gnc_tel.alpha_ref_radps2[2] = outputs.alpha_yaw_radps2;
}

/**
 * @brief 垂直推力机型映射 + 控制分配（层1–2）+ 执行器 mix（层3）+ 限幅/安全门控
 * @param[in] inputs  控制输入（解锁、手动TVC、四通道摇杆）
 * @param[in,out] outputs 控制律输出（specific_force_up_mps2、thrust_cos_tilt、
 *                    alpha_roll/alpha_pitch/alpha_yaw [rad/s²]）；本层回填
 *                    最终 throttle_percent 供遥测/辨识/故障诊断
 * @return 无；直接写舵机/电机 PWM，并更新 gnc_tel 执行器段与 g_tvc_* 观测量
 * @pre  上层已给出 α 指令与期望天向比力；本层是整条链唯一写执行器的地方
 * @formula ⓪ 自动模式 F_total = m·a_up/cos_tilt，thr% = F_total·100/F_max；
 *          ① 惯量逆解 M = I·α（+ 交叉耦合前馈），经轴置换进模型系；
 *          ② 差速通道增益调度 k = min((w0_eff/w_hover)², 1)；
 *          ③ B_true 分配 u = B_true⁻¹·M_cmd（3×3 Cramer 逆，近奇异退 FULL_B）；
 *          ④ 差速开方分配 ωf,t = w0·√(1±Δω)，再钳到 wMax；
 *          ⑤ 舵机 PWM% = 50 + (摆角·传动比·方向)/半行程×50
 * @ref  THY-003 §3（控制分配/B⁺）；见 TandemVec-Paper/ch/05-allocation.tex
 *       (eq:direct_mapping、eq:Btrue_allocation)；
 *       CTL-001 §7/§8.4（差速分配/垂直推力）、§8.6（固件链）；见 PROP-001 §5–§6；
 *       CTL-003 §2（垂直推力属于机型推进层）
 * @units α [rad/s²]、比力 [m/s²]、M [N·m]、摆角 [rad]（转 deg 后进舵机映射）、
 *        Δω 无量纲、转速 [rad/s]、PWM 0–100%
 * @note  本层三条硬性安全边界（删任一条都会改变实机行为）：
 *        ① 锁定态电机强制 0%（只有舵机允许随摇杆摆动做地面标定）；
 *        ② 零油门门控（throttle < 5%）把 δf/δt/Δω 一并归零，保持"零油门→执行器归中"；
 *        ③ allocateDifferential 必须用**真实指令油门 w0**（它决定实际转速），
 *           B 矩阵工作点才用观测转速 w0_eff——两者混用会把电机顶上 30% 转速。
 *        ④ 垂直映射只在自动模式执行（throttle_direct=false）；手动 RC 直通不得
 *           再乘质量/除 cos_tilt，否则零油门/地面模式油门会被改写。
 */
// ★ 2026-08-07：差速回路增益调度系数，供在线辨识修正命令量使用。
// mix 层（步骤8）写入 → update_online_identification（步骤9）读取，
// 同一 GNC 拍内顺序执行，数据一致。
static float s_yaw_gain_sched = 1.0f;

/*
 * 控制侧陀螺有限值守卫：传感器故障/NaN 时保持上一有效值，避免 NaN 进入
 * 姿态/速率环和混控。只做安全兜底，不替代 IMU 驱动层的健康检查。
 */
static Vector3 getSanitizedBodyGyro()
{
  static float last_x = 0.0f, last_y = 0.0f, last_z = 0.0f;
  if (std::isfinite(icm_gyro_x)) last_x = icm_gyro_x;
  if (std::isfinite(icm_gyro_y)) last_y = icm_gyro_y;
  if (std::isfinite(icm_gyro_z)) last_z = icm_gyro_z;
  return Vector3{last_x, last_y, last_z};
}

void mix_and_output_commands(const ControlInputs_t &inputs, ControlOutputs_t &outputs)
{
  const TandemVecParams &P = kDefaultTandemVecParams;
  const Vector3 gyro_safe = getSanitizedBodyGyro();
  const float MOTOR_PWM_MIN = 1000.0f;
  const float MOTOR_PWM_MAX = 2000.0f;
  (void)MOTOR_PWM_MIN; (void)MOTOR_PWM_MAX;  // 保留常量供调试/注释引用，消除编译器警告

  // BTRUE 策略需要上一拍执行器状态（★2026-08-11 术语修正：这是"上一拍工作点
  //  Jacobian"（绝对指令形式，准线性/增益调度），不是 INDI——无增量累加、
  //  不用实测角加速度；本固件为 NDI 式架构：模型逆 + 前馈对消 + 在线 B）
  // 首拍全零 → B_true = B_full（零摆角名义点），行为安全
  static PropulsionState prev_prop_state = {0.0f, 0.0f, 0.0f, 0.0f};
  static TandemVecAllocatorState tandem_alloc_state;

  float upper_gimbal_deg = 0.0f;
  float lower_gimbal_deg  = 0.0f;
  float motor1_pct       = 0.0f;   // 直接用百分比，避免 us→pct 双重转换
  float motor2_pct       = 0.0f;

  // ── 垂直推力机型映射（控制层只给期望天向比力）────────────────────────
  // 自动模式：specific_force_up_mps2 是天向比力（含 1g），这里才乘质量、除
  // cos_tilt、限幅并映射到油门。手动模式：throttle_direct=true，直接沿用 RC。
  // ★ 这是 TandemVec 的推进能力口径；共轴/火箭应替换为各自的推力曲线/阀门映射。
  float final_throttle_pct = outputs.throttle_percent;
  if (!outputs.throttle_direct)
  {
    bfs::ThrustAllocationInput thrust_in;
    thrust_in.specific_force_up_mps2 = outputs.specific_force_up_mps2;
    thrust_in.thrust_axis_up_cos = outputs.thrust_cos_tilt;
    thrust_in.mass_kg = initial_mass;
    thrust_in.max_total_thrust_n = MAX_THRUST;
    const bfs::ThrustAllocationOutput thrust_out =
        bfs::allocateLinearThrustThrottle(thrust_in);
    final_throttle_pct = thrust_out.valid ? thrust_out.throttle_pct : 0.0f;
  }

  if (!inputs.is_unlocked)
  {
    // ★2026-08-10 锁定状态（任何模式）：摇杆直通舵机摆动（地面标定摆座方向），
    //   电机绝不转——原逻辑仅手动TVC模式锁定可摆、自动模式锁定舵机中位，
    //   用户需求改为锁定即可摆，不必切手动 TVC。安全红线保持：锁定电机强制最低。
    lower_gimbal_deg  = mapFloat(inputs.pitch_raw, 988.0f, 2012.0f,
                                -MAX_CORRECTION, MAX_CORRECTION);
    // ★ 2026-08-08 上摆(roll)映射取反：自控链路 Mz=-Iz·alpha_roll（负号）+
    //   分配器 df=Mz/(a·T0)（正系数）→ front 摆角与 alpha_roll 反号；
    //   手动直通映射须与自控最终摆角方向一致（右打摇杆→正摆角）。
    upper_gimbal_deg = mapFloat(inputs.roll_raw,  988.0f, 2012.0f,
                                MAX_CORRECTION, -MAX_CORRECTION);
    motor1_pct = 0.0f;
    motor2_pct = 0.0f;
    // 差速/饱和标记同步（AnoCom 0x40 帧全模式显示）；重置 BTRUE 工作点（防 stale）
    gnc_tel.dw = 0.0f;
    gnc_tel.alloc_sat[0] = gnc_tel.alloc_sat[1] = gnc_tel.alloc_sat[2] = false;
    prev_prop_state = {0.0f, 0.0f, 0.0f, 0.0f};
    tandem_alloc_state = TandemVecAllocatorState{};
  }
  else if (inputs.is_manual_tvc)
  {
    // ---- 解锁 + 手动TVC（CH8 高位）：RC 直接控制舵机/差速，用于演示/地面标定 ----
    // ★2026-08-10 开关反逻辑后此分支仅在解锁 + 高位时进入（锁定已在上方处理）
    lower_gimbal_deg  = mapFloat(inputs.pitch_raw, 988.0f, 2012.0f,
                                -MAX_CORRECTION, MAX_CORRECTION);
    upper_gimbal_deg = mapFloat(inputs.roll_raw,  988.0f, 2012.0f,
                                MAX_CORRECTION, -MAX_CORRECTION);
    float w0 = (final_throttle_pct / 100.0f) * P.wMax;
    float manual_dw = mapFloat((inputs.yaw_raw - 1500.0f), -512.0f, 512.0f,
                               -P.dwMax, P.dwMax);
    auto diff = allocateDifferential(w0, manual_dw, P);
    motor1_pct = constrain(mapFloat(diff.wf_target, 0.0f, P.wMax, 0.0f, 100.0f), 0.0f, 100.0f);
    motor2_pct = constrain(mapFloat(diff.wt_target, 0.0f, P.wMax, 0.0f, 100.0f), 0.0f, 100.0f);
    // 差速/饱和标记同步（AnoCom 0x40 帧全模式显示）
    gnc_tel.dw = manual_dw;
    gnc_tel.alloc_sat[0] = gnc_tel.alloc_sat[1] = gnc_tel.alloc_sat[2] = false;
  }
  else
  {
    // ---- 姿态控制模式：物理模型逆解控制分配（无倾角保护，支持大机动）----
    {
#if GYRO_DIRECT_TEST
      // ================================================================
      // 陀螺直通测试模式（2026-08-07 调试）：绕过外环姿态环 + B 矩阵分配，
      // 陀螺角速度 × 负反馈增益 直接驱动执行器，验证三通道反馈方向。
      //   下摆(绕y_b): δt = +K·ω_y  （物理 δt>0→My<0，负斜率）
      //   上摆(绕z_b): δf = -K·ω_z  （物理 δf>0→Mz>0，正斜率）
      //   差速(绕x_b): Δω = +K·ω_x  （物理 Δω>0→Mx<0，负斜率）
      // 增益：0.5 deg per (deg/s)，30°/s 时触达 ±15° 摆角限幅
      // 2026-08-07 实机第4轮：方向全对但超调严重 → 大幅降 P（0.5→0.1）
      // ⚠️ 本块是**调试死路径**（GYRO_DIRECT_TEST=0 时不参与编译）：
      //   上面的轴名沿用历史模型系叫法（上摆"绕 z_b"），与固件通道命名
      //   （上摆=滚转=体轴 x）不是一套；符号以本块代码为准，且代码里
      //   upper 取 +K·ω_z（与上一行注释的 -K·ω_z 写法不一致，见核查记录
      //   P2-8）。正式飞行永远走 #else 的 BTRUE 分配，勿照抄本块。
      const float GYRO_K = 0.1f;
      const float R2D = 57.29578f;
      lower_gimbal_deg  = constrain( GYRO_K * gyro_safe.y * R2D, -15.0f, 15.0f);
      upper_gimbal_deg = constrain( GYRO_K * gyro_safe.z * R2D, -15.0f, 15.0f);
      float w0 = (final_throttle_pct / 100.0f) * P.wMax;
      float dw = constrain( GYRO_K * gyro_safe.x * R2D * 0.05f, -P.dwMax, P.dwMax);
      auto diff = allocateDifferential(w0, dw, P);
      motor1_pct = constrain(mapFloat(diff.wf_target, 0.0f, P.wMax, 0.0f, 100.0f), 0.0f, 100.0f);
      motor2_pct = constrain(mapFloat(diff.wt_target, 0.0f, P.wMax, 0.0f, 100.0f), 0.0f, 100.0f);
      prev_prop_state = {0.0f, 0.0f, 0.0f, 0.0f};
#else
      // 机型分配器边界：控制层至此只给 α_cmd + 油门；本段是唯一进入
      // TandemVec 惯量/几何/BTRUE/差速的机型逆解。共轴/火箭替换 allocator，
      // 不需要改姿态/速率/制导层。
      TandemVecAllocatorInput alloc_in;
      alloc_in.alpha_roll_radps2 = outputs.alpha_roll_radps2;
      alloc_in.alpha_pitch_radps2 = outputs.alpha_pitch_radps2;
      alloc_in.alpha_yaw_radps2 = outputs.alpha_yaw_radps2;
      alloc_in.throttle_pct = final_throttle_pct;
      float gyro_body[3] = {static_cast<float>(gyro_safe.x),
                            static_cast<float>(gyro_safe.y),
                            static_cast<float>(gyro_safe.z)};
      if (nav_system_initialized)
      {
        const Eigen::Vector3f gb = nav_ekf.gyro_bias_radps();
        gyro_body[0] -= gb(0);
        gyro_body[1] -= gb(1);
        gyro_body[2] -= gb(2);
      }
      alloc_in.gyro_body_radps[0] = gyro_body[0];
      alloc_in.gyro_body_radps[1] = gyro_body[1];
      alloc_in.gyro_body_radps[2] = gyro_body[2];
      alloc_in.inertia_comp_mask = kFlightCtrlParams.inertia_comp_mask;
      alloc_in.dt_s = 0.005f;
      const TandemVecAllocatorOutput alloc =
          allocateTandemVec(alloc_in, tandem_alloc_state, P);
      tandem_alloc_state = alloc.next;
      // 保持既有 AnoVars/在线辨识观测口径（原 g_wf_est/g_wt_est 在 mix 内更新）。
      g_wf_est = tandem_alloc_state.wf_est;
      g_wt_est = tandem_alloc_state.wt_est;

      if (!alloc.valid)
      {
        lower_gimbal_deg = 0.0f;
        upper_gimbal_deg = 0.0f;
        motor1_pct = 0.0f;
        motor2_pct = 0.0f;
        s_yaw_gain_sched = 1.0f;
        gnc_tel.M_cmd[0] = gnc_tel.M_cmd[1] = gnc_tel.M_cmd[2] = 0.0f;
        gnc_tel.M_ff[0] = gnc_tel.M_ff[1] = gnc_tel.M_ff[2] = 0.0f;
        gnc_tel.w0_eff = 0.0f;
        gnc_tel.yaw_gain_sched = 1.0f;
        gnc_tel.delta_f_deg = 0.0f;
        gnc_tel.delta_t_deg = 0.0f;
        gnc_tel.dw = 0.0f;
        gnc_tel.alloc_sat[0] = gnc_tel.alloc_sat[1] =
            gnc_tel.alloc_sat[2] = false;
      }
      else
      {
        lower_gimbal_deg = alloc.delta_t_rad * RAD_TO_DEG;
        upper_gimbal_deg = alloc.delta_f_rad * RAD_TO_DEG;
        motor1_pct = alloc.motor1_pct;
        motor2_pct = alloc.motor2_pct;
        s_yaw_gain_sched = alloc.yaw_gain_sched;
        gnc_tel.M_cmd[0] = alloc.M_cmd[0];
        gnc_tel.M_cmd[1] = alloc.M_cmd[1];
        gnc_tel.M_cmd[2] = alloc.M_cmd[2];
        gnc_tel.M_ff[0] = alloc.M_ff[0];
        gnc_tel.M_ff[1] = alloc.M_ff[1];
        gnc_tel.M_ff[2] = alloc.M_ff[2];
        gnc_tel.w0_eff = alloc.w0_eff;
        gnc_tel.yaw_gain_sched = alloc.yaw_gain_sched;
        gnc_tel.delta_f_deg = upper_gimbal_deg;
        gnc_tel.delta_t_deg = lower_gimbal_deg;
        gnc_tel.dw = alloc.dw;
        gnc_tel.alloc_sat[0] = alloc.sat_delta_f;
        gnc_tel.alloc_sat[1] = alloc.sat_delta_t;
        gnc_tel.alloc_sat[2] = alloc.sat_dw;
      }
#endif  // GYRO_DIRECT_TEST
    }
  }

  // ================================================================
  // 层3：执行器输出 — 通用输出表映射（当前机型 kTandemVecProfile）
  // ----------------------------------------------------------------
  // 旧 mix 的齿轮/方向/中位/半行程公式已收口到 ActuatorOutputMap 的
  // TandemVecOutputCalibration；后续机型只需替换 VehicleProfile 输出表。
  // 硬件槽位 → GPIO 映射仍留在本层，通用层不依赖 Arduino。
  // 限幅与安全门控：舵机/电调钳位 0–100%；锁定/零油门由上层分支保证。
  // 修改方向 / 中位 / 行程：仍编辑 TandemVec_Config.h §ServoConfig。
  // ================================================================
  const ServoConfig& SC = kDefaultServoConfig;
  bfs::TandemVecOutputCalibration output_cal;
  output_cal.gear_ratio = SC.teeth_gimbal / SC.teeth_servo;
  output_cal.half_travel_deg = SC.half_travel_deg;
  output_cal.dir_roll = SC.dir_roll;
  output_cal.dir_pitch = SC.dir_pitch;
  output_cal.zero_roll_pct = SC.zero_roll_pct;
  output_cal.zero_pitch_pct = SC.zero_pitch_pct;
  const bfs::OutputCommandSet actuator_outputs = bfs::mapTandemVecOutputs(
      upper_gimbal_deg, lower_gimbal_deg, motor1_pct, motor2_pct,
      output_cal);
  for (uint8_t i = 0; i < actuator_outputs.count; ++i)
  {
    const bfs::OutputCommand &cmd = actuator_outputs.commands[i];
    switch (cmd.hardware_slot)
    {
    case 0: // 上摆/滚转舵机
      SetServoPos(cmd.value, TVC_ROLL_SERVO_PIN);
      ch1_output = cmd.value;
      break;
    case 1: // 下摆/俯仰舵机
      SetServoPos(cmd.value, TVC_PITCH_SERVO_PIN);
      ch2_output = cmd.value;
      break;
    case 2: // 前电机
      SetServoPos(cmd.value, MOTOR1_PIN);
      ch3_output = cmd.value;
      break;
    case 3: // 尾电机
      SetServoPos(cmd.value, MOTOR2_PIN);
      ch4_output = cmd.value;
      break;
    default:
      break;
    }
  }

  // 执行器指令物理量纲（全模式统一捕获）→ AnoCom 0x40 帧 / 上位机 TVC 显示
  g_tvc_upper_deg = upper_gimbal_deg;
  g_tvc_lower_deg  = lower_gimbal_deg;

  // 本拍最终油门（手动直通或机型推力映射）回填到控制输出与 CRSF/导航观测。
  // 必须在 mix 返回前完成：步骤 8b 故障诊断与步骤 9 在线辨识都读该值。
  outputs.throttle_percent = final_throttle_pct;
  throttlePercent = final_throttle_pct;
}
/**
 * @brief 在线参数辨识更新 — ★ 纯观测模式，不改变任何控制行为
 *
 * 作用：估计惯量比 b = I_nominal/I_actual、总扰动 d、重心偏移 CG，
 *       结果写入 id_* 全局量供地面站与黑匣子记录。
 *
 * 【安全性】本函数只读控制器状态，不写任何增益、限幅或输出。
 *   即使辨识结果完全错误，也不影响飞行 —— 这是"第一步：只观测不闭环"。
 *   待多架次数据确认 b_est 稳定后，才考虑人工调整增益（第二步）。
 *
 * 轴序约定：[0]=roll(滚转/上摆) [1]=pitch(俯仰/下摆) [2]=yaw(偏航/差速)，
 *   与 outputs.alpha_* 及 current_omega_dps_body_filtered 的 FRD 映射对应：
 *   滚转←omega.x、俯仰←omega.y、偏航←omega.z。
 * @param[in] inputs  控制输入（解锁/手动旁路门控）
 * @param[in] outputs 本拍控制输出（α 指令 [rad/s²]、油门 [0,100]）
 * @pre  必须在 mix 层之后调用：α 与油门此时才是本拍最终值
 * @formula y = φᵀθ + ε，y=ω̇（陀螺数值微分），φ=[α_cmd, 1]ᵀ，θ=[b, d]ᵀ；
 *          FFRLS 递推，激励门控 σ² = 0.95σ² + 0.05α_cmd² > 4 才更新
 * @ref  TandemVec_OnlineID.h；见 TandemVec-Paper/ch/16-adrc-onlineid.tex
 *       (eq:rls_update、eq:excitation、eq:gain_scheduling)
 * @units α [rad/s²]；ω 传入 deg/s（OnlineID 内部转 rad/s 再微分）；dt 0.005 s
 * @note  差速通道的 α 指令必须乘 s_yaw_gain_sched 才是"实际执行量"，否则
 *        辨识会把调度衰减误判成"命令大、响应小"，b_est 系统性偏小。
 */
static void update_online_identification(const ControlInputs_t &inputs,
                                         const ControlOutputs_t &outputs)
{
  static OnlineID s_online_id;
  static bool     s_was_unlocked = false;

  // 解锁上升沿：清空跨架次残留（协方差、微分状态、CG 估计）
  if (inputs.is_unlocked && !s_was_unlocked) s_online_id.reset();
  s_was_unlocked = inputs.is_unlocked;

  // 仅在姿态控制实际工作时辨识：手动TVC旁路或锁定状态下
  // alpha_cmd 恒为 0，没有激励，辨识无意义。
  if (!inputs.is_unlocked || inputs.is_manual_tvc) return;

  // 命令角加速度（内环输出，rad/s²）
  // ★ 2026-08-07：差速通道（alpha_yaw → Mx'）在 mix 层被增益调度缩放
  //   (w0/w_hover)²，实际执行的力矩 ≠ 内环原始输出。辨识必须用**实际
  //   执行量**，否则会误判"命令了却没达到" → 惯量比 b_est 系统性偏小。
  //   摆座通道（roll/pitch）未做调度，原样传入。
  const float alpha_cmd[3] = { outputs.alpha_roll_radps2,
                               outputs.alpha_pitch_radps2,
                               outputs.alpha_yaw_radps2 * s_yaw_gain_sched };

  // 实测角速率（deg/s）— OnlineID 内部转 rad/s 再求导
  // Vector3 分量为 double，需显式转 float（花括号初始化不允许隐式收窄）
  const float omega_dps[3] = { static_cast<float>(current_omega_dps_body_filtered.x),   // 滚转
                               static_cast<float>(current_omega_dps_body_filtered.y),   // 俯仰
                               static_cast<float>(current_omega_dps_body_filtered.z) }; // 偏航

  // 内环积分项 I_term = ki × integral (rad/s²)，用于提取配平力矩→CG偏移
  const float i_term[3] = {
      ControlUnits::dps2ToRadps2(rollRatePID.getKi()  * rollRatePID.getIntegral()),
      ControlUnits::dps2ToRadps2(pitchRatePID.getKi() * pitchRatePID.getIntegral()),
      ControlUnits::dps2ToRadps2(yawRatePID.getKi()   * yawRatePID.getIntegral()) };

  s_online_id.step(alpha_cmd, omega_dps, i_term,
                   outputs.throttle_percent, 0.005f);   // GNC 固定 200Hz

  // ---- 导出遥测（只写 id_* 全局量，不回写控制参数）----
  for (int i = 0; i < 3; ++i)
  {
    id_b_est[i]   = s_online_id.b_est[i];
    id_d_est[i]   = s_online_id.d_est[i];
    id_excited[i] = (s_online_id.alpha_var[i] > 4.0f);
  }
  id_cg_mm = s_online_id.cg_est_mm;

  // 建议增益（仅供地面站显示与离线复算，**未写回 PID**）
  id_kp_suggest[0] = s_online_id.adaptKpR(rollRatePID.getKp(),  0);
  id_kp_suggest[1] = s_online_id.adaptKpR(pitchRatePID.getKp(), 1);
  id_kp_suggest[2] = s_online_id.adaptKpR(yawRatePID.getKp(),   2);
}

/**
 * @brief 执行器故障只读诊断（2026-10）：F3 单发推力损失 / F4 舵机卡死
 *
 * 输入来自本拍最终执行器指令、姿态误差和垂直速度误差（**测得−目标**）；
 * 只写 g_actuator_* 诊断标志，不改变油门/舵机输出。
 * 注意：这只是检测层，受控下降/故障重构尚未接入（标志仅供遥测/地面站）。
 *
 * 边界（两类的判据与能力上限不同，不能混写）：
 *   F3 推力损失：油门 > 75% 且垂直速度误差 < −1.0 m/s 持续 1.0 s；
 *     仅 AUTO_ALTITUDE / AUTO_POSITION / GUIDED 有效（手动模式无有效垂直目标）；
 *     动作建议 ThrustBoost（抬油门），实际执行由控制层决定；定高模式下
 *     高度环本身可补偿 ~30% 推力损失（仿真 PASS），不等同"故障重构"。
 *   F4 舵机卡死：|姿态误差| > 25° 且 |摆角指令| ≥ 90% dMax 持续 0.5 s；
 *     动作建议 ControlledDescent（有界下降）。前摆 δ_f 是滚转**唯一**主控，
 *     卡死属硬件单点，无法靠控制律恢复（见 notes/08-F4 单舵机卡死能力边界）。
 *
 * 2026-10-23 修正：F3 垂直速度误差旧实现写成"目标−当前"，与监测器
 *   "下沉为负"的约定相反，真实下沉时永不触发；现统一走
 *   `bfs::thrustLossVerticalVelocityError(measured, target)`。
 *
 * @param[in] inputs  控制输入（解锁门控：未解锁直接清零诊断状态）
 * @param[in] outputs 本拍控制输出（油门百分比）
 * @param[in] mode    当前飞行模式（F3 仅在自动垂直模式启用）
 * @pre  必须在 mix 层之后调用（要用本拍最终 gnc_tel.delta_*_deg）
 * @ref  ActuatorFaultMonitor.h；见 docs/08-参考资料/notes/08-F4单舵机卡死能力边界.md
 * @units 姿态误差 deg、摆角占空比（|cmd|/dMax 无量纲）、油门 [0,1]、
 *        垂直速度误差 m/s、时间 s
 */
static bfs::ActuatorFaultMonitor s_actuator_fault_monitor;
static void updateActuatorFaultDiagnostics(const ControlInputs_t &inputs,
                                           const ControlOutputs_t &outputs,
                                           const ControlMode mode)
{
  if (!inputs.is_unlocked)
  {
    s_actuator_fault_monitor.reset();
    g_actuator_servo_jam = false;
    g_actuator_thrust_loss = false;
    g_actuator_controlled_descent_request = false;
    return;
  }

  const float dmax_deg = kDefaultTandemVecParams.dMax * 57.2957795f;
  const float df_frac = (dmax_deg > 1e-3f)
                            ? (gnc_tel.delta_f_deg / dmax_deg)
                            : 0.0f;
  const float dt_frac = (dmax_deg > 1e-3f)
                            ? (gnc_tel.delta_t_deg / dmax_deg)
                            : 0.0f;
  // ── F3 判据输入（2026-10-23 修正）────────────────────────────────────
  // 公式：e_v = v_up,meas − v_up,target（下沉为负）@ref CTL-002 §3
  // 历史缺陷：旧代码写成 target − measured，真实下沉时 e_v>0，F3 永不触发；
  //   反向爬升超目标且油门>75% 持续 1s 时会误报 ThrustBoost。
  // 手动模式：target_vertical_velocity 只在 AUTO_*/GUIDED 更新，手动下是上次
  //   自动模式的陈旧值 → 不允许 F3 参与判定（thrust_detection_enabled=false），
  //   否则会把陈旧目标当成“期望下沉”而误判/漏判。
  // 符号契约已收口到 thrustLossVerticalVelocityError()，调用方不得自行相减。
  const float current_v_up =
      InsVerticalVelocityUpFromNedDown(INS_GNSS_Packet.velocity_down);
  const float vertical_vel_err =
      bfs::thrustLossVerticalVelocityError(current_v_up, target_vertical_velocity);
  const bool f3_enabled = (mode == AUTO_ALTITUDE || mode == AUTO_POSITION ||
                           mode == GUIDED);
  s_actuator_fault_monitor.update(gnc_tel.error_deg[0],
                                  gnc_tel.error_deg[1],
                                  df_frac,
                                  dt_frac,
                                  outputs.throttle_percent / 100.0f,
                                  vertical_vel_err,
                                  0.005f, // 200 Hz 导航/控制周期
                                  f3_enabled);
  g_actuator_servo_jam = s_actuator_fault_monitor.servo_jam();
  g_actuator_thrust_loss = s_actuator_fault_monitor.thrust_loss();
  g_actuator_controlled_descent_request =
      (s_actuator_fault_monitor.action() ==
       bfs::ActuatorFaultAction::ControlledDescent);
}

/**
 * @brief GNC（制导、导航与控制）核心调度执行器 [频率: 200 Hz]
 *
 * 执行流程（顺序不可交换，每步依赖上一步的全局量）:
 *
 * 1. 状态管理 (State Management):
 *    - 判定飞行模式 (MANUAL / AUTO_ALT / AUTO_POS / GUIDED)；
 *    - 处理解锁安全联锁、原点复位、积分项动态使能及 GUIDED 超时保护。
 *
 * 2. 制导律计算 (Guidance Law):
 *    - 垂直通道: 闭环计算高度与速度误差，生成垂直推力需求（油门 %）；
 *    - 水平通道: 闭环计算位置与速度误差，生成水平推力分量；
 *    - 目标合成: 期望加速度 → 推力方向 → 目标姿态四元数 (q_target)。
 *
 * 3. 控制律执行 (Control Law):
 *    - 姿态外环: 姿态误差四元数 → 目标角速率 (deg/s)；
 *    - 角速率内环: 角速率误差 → 角加速度指令 (deg/s²)；
 *    - 航向通道: 摇杆角速率 + 回中航向锁 → 差速通道 α_yaw。
 *
 * 4. 动力混控 (Mixer & Output):
 *    - 惯量逆解 → 轴置换 → B_true 分配 → 摆角/差速 → 舵机与电调 PWM。
 *
 * 5. 诊断与观测: 执行器故障只读诊断（F3/F4）+ 在线辨识（不写回增益）。
 *
 * @pre  由 200 Hz 任务调度器调用；本拍内完成，禁止阻塞（含串口/Flash 写）
 * @post 执行器 PWM 已刷新；gnc_tel 控制链中间量已填好供遥测
 * @ref  CTL-001 §8.6；见 THY-003 §3
 * @units 每拍固定 dt = 0.005 s；角度域 deg/deg/s/deg/s²，写执行器前转 rad
 * @note  步骤 8 必须在步骤 6/7 之后、步骤 9 之前：辨识要用本拍最终 α 与油门。
 */

void runGNCExecutive()
{
  // --- 1. 数据准备 ---
  ControlInputs_t controller_inputs;
  ControlOutputs_t controller_outputs = {0}; // 初始化输出结构体
  Quaternion q_target_attitude;              // 目标姿态四元数

  // --- 2. 控制流程 ---
  // 步骤 1: 处理遥控输入、开关状态，并处理解锁时的原点设置
  process_control_inputs(controller_inputs);

  // 步骤 2: 决定当前飞行模式，并处理GUIDED模式的超时故障保护
  ControlMode current_mode = determine_control_mode(controller_inputs);
  g_current_flight_mode = current_mode; // 暴露到全局供 handleAnoCom 遥测发送
  g_is_unlocked = controller_inputs.is_unlocked; // 暴露到全局供 handleAnoCom 遥测发送

  // 步骤 3: 根据模式和状态，管理所有PID控制器的积分项
  manage_pid_integrals(controller_inputs, current_mode);

  // 步骤 4: 计算推力控制，得到油门百分比
  compute_thrust_control(controller_inputs, current_mode, controller_outputs);

  // 步骤 5: 根据模式和输入，生成目标姿态四元数
  generate_attitude_target(controller_inputs, current_mode, q_target_attitude);

  // 步骤 6: 执行Roll/Pitch姿态控制器，计算TVC舵机修正量
  execute_attitude_controller(controller_inputs, q_target_attitude, controller_outputs);

  // 步骤 7: 执行偏航控制器 → 差速通道 Δω（悬停时绕推力轴 = 世界航向）
  //   注：旧注释"→ 上摆角 δ_f"是轴置换前的语义，与 mix 层实码不符，已废止。
  execute_yaw_controller(controller_inputs, q_target_attitude, controller_outputs);

  // 步骤 8: 将所有计算出的控制量混合，并输出到执行机构
  mix_and_output_commands(controller_inputs, controller_outputs);

  // 步骤 8b: 执行器故障只读诊断（不改变输出）
  updateActuatorFaultDiagnostics(controller_inputs, controller_outputs, current_mode);

  // 步骤 9: 在线参数辨识 — ★ 纯观测，只写 id_* 遥测量，不改任何增益
  //   放在混控之后：此时 alpha_* 与 throttle_percent 均为本拍最终值。
  update_online_identification(controller_inputs, controller_outputs);

  // --- 3. 清理工作 ---
  // 清除新指令标志，确保每个新指令只被处理一次
  if (new_guidance_command_received)
  {
    new_guidance_command_received = false;
  }

} // end of runGNCExecutive()
