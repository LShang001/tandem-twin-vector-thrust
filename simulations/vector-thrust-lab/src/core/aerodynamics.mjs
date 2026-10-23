/**
 * @file    aerodynamics.mjs
 * @module  仿真 core/气动力
 * @brief   风轴系气动力/力矩（升力、阻力、侧力、静稳定与阻尼）→ 机体系
 * @details 输入：机体系速度 v、机体角速度 ω、参数表、气动开关；
 *          处理：动压/迎角/侧滑角 → 风轴升阻力与三轴气动矩 → 投影到机体系；
 *          输出：机体系力分量 {aX,Y,aZ} 与力矩 {Mx,My,Mz}，可写入遥测 sink。
 *          不变量：aeroOn=false 时力与力矩恒为零，但空速/动压遥测仍更新。
 * @theory  动压 q̄=½ρV²、L=q̄S·CLa·α、D=q̄S(CD0+CDk·CL²) @ref MOD-001 §6；
 *          气动矩 Clb/Clp、Cm0/Cma/Cmq、Cnb/Cnr @ref MOD-001 §6；
 *          风轴→机体系投影 @ref THY-003 §1
 * @frame   输入速度为机体系 FRD；内部在风轴系算力，再绕 y_b 转 −α 投影回机体系
 * @units   V [m/s]、q̄ [Pa]、α/β [rad]、力 [N]、力矩 [N·m]
 * @redline 空速下限 vMin=0.5 m/s 仅用于分母保护（见行为保持红线清单）
 * @note 本模块不含螺旋桨滑流、失速与非定常效应（线性小迎角模型，@ref MOD-001 §11）
 * @see     dynamics.mjs（RK4 每阶段用中间状态重算气动）、tests/aerodynamics.test.mjs
 */
import { clamp } from './math.mjs';

// ── 气动模型（风轴系）──────────────────────────────────────────────
// 公式：q̄=½ρV²；L=q̄S·CLa·α；D=q̄S(CD0+CDk·CL²)；Y=q̄S·CYb·β
//       Mx=q̄Sb̄(Clb·β+Clp·p̂)、My=q̄Sc̄(Cm0+Cma·α+Cmq·q̂)、Mz=q̄Sb̄(Cnb·β+Cnr·r̂)
// 符号：α=atan2(w,u)（抬头为正）、β=asin(v/V)、p̂=p·b̄/(2V) 等为无量纲角速率
// 来源：@ref MOD-001 §6（模型形式）、@ref THY-003 §1（刚体方程）
// 实现：`computeAeroState` 一段；风轴力再投影 aX=L·sinα−D·cosα、aZ=−L·cosα−D·sinα
// 等价变换：无（线性小迎角模型；教材 ENU 与本项目 NED 仅在符号约定层，见 MOD-002 §2）
// 数值证据：@test tests/aerodynamics.test.mjs（升力方向、aX 解析式 1e-9、Cmq 阻尼符号）
// ─────────────────────────────────────────────────────────────────
// @formula α=atan2(w,u)，β=asin(v/max(V,vMin))，q̄=½ρV²；
//          aX=L·sinα−D·cosα，aZ=−L·cosα−D·sinα
// @note 动压用真实 V（零速为 0），只有 β 与无量纲角速率的分母用 vSafe=max(V,vMin)
//       ——低速保护不会凭空造力，这是"关掉气动可复现角速度自由积分"的前提
// @test tests/aerodynamics.test.mjs（开关归零、升力方向、β/Clb 符号、零速保护）
/**
 * @brief 纯状态气动计算：给定机体系速度/角速度返回气动力与力矩
 * @param vVec 机体系速度 [u,v,w] [m/s]；@param wVec 机体角速度 [p,q,r] [rad/s]
 * @param {Object} P 参数表；@param {boolean} [aeroOn] 气动开关；@param {Object} [sink] 遥测缓存
 * @returns {{aX,Y,aZ,Mx,My,Mz}} 机体系力 [N] / 力矩 [N·m]；@ref MOD-001 §6
 */
export function computeAeroState(vVec, wVec, P, aeroOn = true, sink = null) {
  const u = vVec.x, v = vVec.y, wv = vVec.z;
  const V = Math.hypot(u, v, wv);
  const vSafe = Math.max(V, P.vMin);
  const al = Math.atan2(wv, u);                        // 迎角 α（抬头为正：w>0 表示来流从下方）
  const be = Math.asin(clamp(v / vSafe, -1, 1));       // 侧滑角 β（vSafe 防 V→0 除零）
  const qb = 0.5 * P.rho * V * V;                      // 动压（真实空速，低速不加料）
  let aX = 0, Y = 0, aZ = 0, Mx = 0, My = 0, Mz = 0;
  if (aeroOn) {
    const CL = P.CLa * al;
    const L = qb * P.Sw * CL;                          // 升力（沿风轴 −z_w）
    const D = qb * P.Sw * (P.CD0 + P.CDk * CL * CL);   // 阻力（零升+诱导）
    Y = qb * P.Sw * P.CYb * be;                        // 侧力（风轴 y_w → 机体系 y 近似）
    aX = L * Math.sin(al) - D * Math.cos(al);
    aZ = -L * Math.cos(al) - D * Math.sin(al);
    // 气动矩: 静稳定项 + 阻尼导数项（无量纲角速率）
    const pH = wVec.x * P.bspan / (2 * vSafe), qH = wVec.y * P.cbar / (2 * vSafe), rH = wVec.z * P.bspan / (2 * vSafe);
    Mx = qb * P.Sw * P.bspan * (P.Clb * be + P.Clp * pH);
    My = qb * P.Sw * P.cbar * (P.Cm0 + P.Cma * al + P.Cmq * qH);
    Mz = qb * P.Sw * P.bspan * (P.Cnb * be + P.Cnr * rH);
  }
  if (sink) {
    sink.V = V; sink.qbar = qb; sink.alpha = al; sink.beta = be;
    sink.Mx = Mx; sink.My = My; sink.Mz = Mz;
  }
  return { aX, Y, aZ, Mx, My, Mz };
}

// @note 本函数只在子步开始调用一次（供 SAS/遥测看到一致状态）；RK4 各阶段用
//       computeAeroState 拿中间状态重算，两条路径共用同一模型，不允许分叉
// @test tests/aerodynamics.test.mjs（computeAero 与解析式 1e-9 比对）
/**
 * @brief 仿真状态接口：按当前状态算气动力并刷新 sim.aero 遥测
 * @param {Object} sim 仿真状态；@param {Object} P 参数表
 * @returns {{aX,Y,aZ}} 机体系气动力分量 [N]（不含力矩）；@ref MOD-001 §6
 */
export function computeAero(sim, P) {
  const { S, F, aero } = sim;
  const out = computeAeroState(F.vel, S.omega, P, S.aero, aero);
  return { aX: out.aX, Y: out.Y, aZ: out.aZ };
}
