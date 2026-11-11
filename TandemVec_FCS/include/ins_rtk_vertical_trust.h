/**
 * @file    ins_rtk_vertical_trust.h
 * @module  固件/导航（RTK 垂直可信仲裁）
 * @brief   仲裁「高精度 RTK 固定解接管高度」：停气压观测 + 取消 GNSS 垂直位置/速度噪声地板
 * @details 输入：GNSS 新鲜度、fix 是否==RTK_FIXED(6)、接收机垂直精度 vAcc、
 *          垂直 spoof 疑似、本周期 dt；处理：四条件全满足连续 confirm_s 才进入
 *          可信态（进入迟滞），任一条件破坏立即退出（安全侧快速回退）；可信态下
 *          垂直位置地板从 floor_default 渐降到 floor_rtk、垂直速度地板从
 *          floor_vel_default 渐降到 floor_vel_rtk（两者目标均=0，即取消地板，
 *          vAcc/s_acc 原样进 R），退出立即回升各自默认；输出：trusted（停气压
 *          判据）与 floor_m/floor_vel_mps（送入 GNSS 动态权重的垂直位置/速度地板）。
 *          停气压与降地板共用同一确认态，杜绝「气压停了、地板还是 3 m」的最差组合。
 * @theory  R 应按量测实际噪声水平给定（REF-121 §2.2.1 p.33）；RTK 固定解短基线
 *          下整周模糊度固定、大部分误差被差分消掉，接收机 vAcc 可信，故可取消
 *          地板。但切换瞬间 EKF 高度仍停在气压锚定值，与 RTK 真值可差米级，R 骤降
 *          会使 NIS=rᵀS⁻¹r 暴涨被 χ² 门拒收（REF-121 §7.3.1 p.255），故地板必须
 *          渐降而非阶跃，让高度被逐帧拉向 RTK、新息始终在门内。
 * @frame   N/A（垂直通道噪声仲裁，与 NED/FRD 轴定义无关）
 * @units   精度/位置地板 m；速度地板 m/s；时长 s；fix 为 UBX Fix 枚举（FIX_RTK_FIXED=6）
 * @note    与 ins_gnss_dynamic_weight.h「R 地板不可收紧」的一般规则不冲突：那条
 *          针对单点/浮点解（接收机自报精度偏乐观，收紧地板→接受率暴跌）；本仲裁
 *          只在 fix==6 + vAcc 门 + 新鲜度 + 迟滞全满足的 RTK 固定解 regime 内取消
 *          地板，且渐降过渡。vAcc=0 是 NMEA 兜底的「未知」而非「超准」，必须 >0。
 * @see     kGnssDwCfg、navigation_task.cpp 气压更新块与 GNSS 动态权重块、
 *          test_host/test_rtk_vertical_trust.cpp
 */

#pragma once

#include <cstdint>

namespace bfs {

struct RtkVerticalTrustConfig {
  float max_v_acc_m         = 0.1f;   // vAcc 上限：>0 且 ≤ 此值才算高精度（F9P 好条件 ≈0.02 m）
  float confirm_s           = 1.0f;   // 进入可信态所需连续满足时长（进入迟滞）
  float floor_default_m     = 3.0f;   // 非可信态垂直位置地板（须与 kGnssDwCfg.floor_pos_d_m 一致）
  float floor_rtk_m         = 0.0f;   // 可信态位置目标地板：0=取消，vAcc 原样进 R
  float floor_vel_default_mps = 0.25f; // 非可信态垂直速度地板（须与 kGnssDwCfg.floor_vel_d_mps 一致）
  float floor_vel_rtk_mps   = 0.0f;   // 可信态速度目标地板：0=取消，s_acc 原样进 R
  float ramp_s              = 3.0f;   // 地板渐降时长（防 NIS 门拒收）；≤0 表示立即到位
  float max_dt_s            = 0.25f;  // 单周期 dt 上限（防调度间隙造成累计/渐降跳变）
};

struct RtkVerticalTrustState {
  float confirm_accum_s = 0.0f;   // 连续满足累计时长
  bool  trusted         = false;  // 可信态（停气压 + 降地板）
  float floor_m         = 3.0f;   // 当前有效垂直位置地板（MakeState 初始化为 floor_default）
  float floor_vel_mps   = 0.25f;  // 当前有效垂直速度地板（MakeState 初始化为 floor_vel_default）
};

// 用配置初始化状态，保证两地板起点 = 各自默认（避免上电首帧误信 GNSS 垂直）。
inline RtkVerticalTrustState RtkVerticalTrustMakeState(const RtkVerticalTrustConfig &cfg)
{
  RtkVerticalTrustState s;
  s.confirm_accum_s = 0.0f;
  s.trusted = false;
  s.floor_m = cfg.floor_default_m;
  s.floor_vel_mps = cfg.floor_vel_default_mps;
  return s;
}

// 单地板线性渐降：从 def 向 rtk 以 (def−rtk)/ramp_s 的速率下降并钳到 rtk；
// ramp_s≤0 立即到位。current 为当前地板值，返回渐降后的地板。
inline float RtkVerticalTrustRampFloor(float current, float def, float rtk,
                                       float ramp_s, float dt_s)
{
  if (ramp_s > 0.0f)
  {
    const float rate = (def - rtk) / ramp_s;
    current -= rate * dt_s;
    if (current < rtk) current = rtk;
    return current;
  }
  return rtk;
}

/**
 * @brief   每气压周期（25 Hz）调用一次，更新可信态与垂直位置/速度地板
 * @details 输入：状态/配置 + 四条件（新鲜、fix==6、vAcc 门、无 spoof）+ dt；
 *          处理：迟滞累计 → 可信判定 → 两地板渐降/回升；输出：trusted。
 *          不变量：非可信态 floor_m/floor_vel_mps 恒等于各自默认；trusted 落下时
 *          confirm_accum_s 清零，再进入须重新累计满 confirm_s。
 * @formula 迟滞：cond 真则 accum+=dt，accum≥confirm_s 置 trusted；cond 假则
 *          accum=0、trusted=false。地板：trusted 时各自
 *          floor=max(floor_rtk, floor−(floor_default−floor_rtk)/ramp_s·dt)；
 *          否则 floor=floor_default。dt 先钳到 [0, max_dt_s]。
 * @ref     REF-121 §7.3.1 p.255（NIS/χ² 门控，渐降避免切换瞬态拒收）
 * @param[in,out] s              仲裁状态
 * @param[in]     cfg            配置
 * @param[in]     gnss_fresh     GNSS 数据是否新鲜（断流即 false）
 * @param[in]     fix_is_rtk_fixed  fix 是否精确==FIX_RTK_FIXED(6)
 * @param[in]     v_acc_m        接收机自报垂直精度 (m)；≤0 视为未知
 * @param[in]     vert_suspect   垂直 spoof 疑似（与重锚门互锁）
 * @param[in]     dt_s           本周期时长 (s)
 * @return    本周期 trusted（是否停气压）
 */
inline bool RtkVerticalTrustUpdate(RtkVerticalTrustState &s,
                                   const RtkVerticalTrustConfig &cfg,
                                   bool gnss_fresh, bool fix_is_rtk_fixed,
                                   float v_acc_m, bool vert_suspect, float dt_s)
{
  if (dt_s < 0.0f) dt_s = 0.0f;
  if (dt_s > cfg.max_dt_s) dt_s = cfg.max_dt_s;

  // vAcc=0/负 是「未知」（NMEA 兜底恒 0），不能当高精度；须 >0 且 ≤门限。
  const bool v_acc_ok = (v_acc_m > 0.0f) && (v_acc_m <= cfg.max_v_acc_m);
  const bool cond = gnss_fresh && fix_is_rtk_fixed && v_acc_ok && !vert_suspect;

  if (cond)
  {
    s.confirm_accum_s += dt_s;
    if (s.confirm_accum_s >= cfg.confirm_s) s.trusted = true;
  }
  else
  {
    // 安全侧快速回退：任一条件破坏立即退出，再进入须重新迟滞确认。
    s.confirm_accum_s = 0.0f;
    s.trusted = false;
  }

  if (s.trusted)
  {
    // 渐降：让 EKF 高度/垂速被逐帧拉向 RTK，新息始终在 NIS 门内（防骤降拒收）。
    // 位置与速度地板共用 ramp_s，同步渐降到位。
    s.floor_m = RtkVerticalTrustRampFloor(
        s.floor_m, cfg.floor_default_m, cfg.floor_rtk_m, cfg.ramp_s, dt_s);
    s.floor_vel_mps = RtkVerticalTrustRampFloor(
        s.floor_vel_mps, cfg.floor_vel_default_mps, cfg.floor_vel_rtk_mps,
        cfg.ramp_s, dt_s);
  }
  else
  {
    // 抬 R 恒安全（只会降低信任、不会拒收），立即回升各自默认地板。
    s.floor_m = cfg.floor_default_m;
    s.floor_vel_mps = cfg.floor_vel_default_mps;
  }
  return s.trusted;
}

}  // namespace bfs