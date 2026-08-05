// ============================================================
//  空气动力（风轴系 → 机体系）
// ============================================================
import { clamp } from './math.mjs';

// 纯状态气动计算：v（机体系速度）、w（机体角速度）→ 气动力/力矩
// 供 RK4 各阶段用中间状态重算（对齐 Python rk4_step 的 aero_forces 逐阶段重算），
// 也供 computeAero 遥测调用。aeroOn=false 时贡献为零（无翼/裸物理复现）。
// sink 可选：写入遥测对象 {V, qbar, alpha, beta}（由 computeAero 传入 sim.aero）
export function computeAeroState(vVec, wVec, P, aeroOn = true, sink = null) {
  const u = vVec.x, v = vVec.y, wv = vVec.z;
  const V = Math.hypot(u, v, wv);
  const vSafe = Math.max(V, P.vMin);
  const al = Math.atan2(wv, u);                        // 迎角 α
  const be = Math.asin(clamp(v / vSafe, -1, 1));       // 侧滑角 β
  const qb = 0.5 * P.rho * V * V;                      // 动压
  let aX = 0, Y = 0, aZ = 0, Mx = 0, My = 0, Mz = 0;
  if (aeroOn) {
    const CL = P.CLa * al;
    const L = qb * P.Sw * CL;                          // 升力
    const D = qb * P.Sw * (P.CD0 + P.CDk * CL * CL);   // 阻力（零升+诱导）
    Y = qb * P.Sw * P.CYb * be;                        // 侧力
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

// 仿真状态接口（遥测/子步初显示）：包装纯函数，返回机体系气动力分量增量 {aX, Y, aZ}
export function computeAero(sim, P) {
  const { S, F, aero } = sim;
  const out = computeAeroState(F.vel, S.omega, P, S.aero, aero);
  return { aX: out.aX, Y: out.Y, aZ: out.aZ };
}
