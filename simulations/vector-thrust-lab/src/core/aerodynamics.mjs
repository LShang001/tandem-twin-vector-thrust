// ============================================================
//  空气动力（风轴系 → 机体系）
// ============================================================
import { clamp } from './math.mjs';

// 计算空速/动压/迎角/侧滑角与气动力/力矩
// aero 开关关闭时贡献为零（可复现"角速度自由积分"的裸物理）
// 返回机体系气动力分量增量 {aX, Y, aZ}
export function computeAero(sim, P) {
  const { S, F, aero } = sim;
  const u = F.vel.x, v = F.vel.y, wv = F.vel.z;
  const V = Math.hypot(u, v, wv);
  const vSafe = Math.max(V, P.vMin);
  const al = Math.atan2(wv, u);                        // 迎角 α
  const be = Math.asin(clamp(v / vSafe, -1, 1));       // 侧滑角 β
  const qb = 0.5 * P.rho * V * V;                      // 动压
  aero.V = V; aero.qbar = qb; aero.alpha = al; aero.beta = be;
  aero.Mx = 0; aero.My = 0; aero.Mz = 0;
  let aX = 0, Y = 0, aZ = 0;
  if (S.aero) {
    const CL = P.CLa * al;
    const L = qb * P.Sw * CL;                          // 升力
    const D = qb * P.Sw * (P.CD0 + P.CDk * CL * CL);   // 阻力（零升+诱导）
    Y = qb * P.Sw * P.CYb * be;                        // 侧力
    aX = L * Math.sin(al) - D * Math.cos(al);
    aZ = -L * Math.cos(al) - D * Math.sin(al);
    // 气动矩: 静稳定项 + 阻尼导数项（无量纲角速率）
    const pH = S.omega.x * P.bspan / (2 * vSafe), qH = S.omega.y * P.cbar / (2 * vSafe), rH = S.omega.z * P.bspan / (2 * vSafe);
    aero.Mx = qb * P.Sw * P.bspan * (P.Clb * be + P.Clp * pH);
    aero.My = qb * P.Sw * P.cbar * (P.Cm0 + P.Cma * al + P.Cmq * qH);
    aero.Mz = qb * P.Sw * P.bspan * (P.Cnb * be + P.Cnr * rH);
  }
  return { aX, Y, aZ };
}
