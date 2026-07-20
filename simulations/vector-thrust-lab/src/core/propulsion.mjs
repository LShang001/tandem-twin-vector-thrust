// ============================================================
//  动力装置：电机一阶惯性 + 电磁反扭 + 推力矢量六维映射
// ============================================================

// 更新转速、推力、反扭矩与机体系六维力/力矩
// 返回摆角三角函数供转动方程的转子角动量项使用
export function stepPropulsion(sim, P, dt) {
  const { S, dyn } = sim;
  const dtC = S.dtAct, dfC = S.dfAct, dwC = S.dwAct;

  // 差速分配: ω_f²+ω_t² ≈ 常数
  const w0 = S.thr * P.wMax;
  const wfT = w0 * Math.sqrt(Math.max(0, 1 + dwC));
  const wtT = w0 * Math.sqrt(Math.max(0, 1 - dwC));
  S.wf += (wfT - S.wf) * Math.min(dt / P.tauM, 1);
  S.wt += (wtT - S.wt) * Math.min(dt / P.tauM, 1);
  const dWf = (S.wf - sim.prevWf) / Math.max(dt, 1e-4);
  const dWt = (S.wt - sim.prevWt) / Math.max(dt, 1e-4);
  sim.prevWf = S.wf; sim.prevWt = S.wt;

  const Tf = P.kT * S.wf * S.wf, Tt = P.kT * S.wt * S.wt;
  const Qf = P.kQ * S.wf * S.wf + P.Jp * dWf;   // 传给机体的反扭矩 = 电磁扭矩
  const Qt = P.kQ * S.wt * S.wt + P.Jp * dWt;
  const cf = Math.cos(dfC), sf = Math.sin(dfC);
  const ct = Math.cos(dtC), st = Math.sin(dtC);

  dyn.Tf = Tf; dyn.Tt = Tt; dyn.Qf = Qf; dyn.Qt = Qt;
  dyn.Fx = Tf * cf + Tt * ct;
  dyn.Fy = Tf * sf;
  dyn.Fz = -Tt * st;
  dyn.Mx = -Qf * cf + Qt * ct;                  // 滚转: 反扭差
  dyn.My = -P.b * Tt * st - Qf * sf;            // 俯仰: 尾摆 + 耦合
  dyn.Mz = P.a * Tf * sf - Qt * st;             // 偏航: 前摆 + 耦合
  return { cf, sf, ct, st };
}
