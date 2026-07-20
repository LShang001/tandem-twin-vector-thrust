// ============================================================
//  遥测快照：浏览器层（HUD / 渲染）读取的数据视图
// ============================================================

export function getTelemetry(sim) {
  const { S, F, dyn, aero } = sim;
  return {
    forces: { ...dyn },
    aero: { ...aero },
    rotors: { wf: S.wf, wt: S.wt },
    actuators: { dtAct: S.dtAct, dfAct: S.dfAct, dwAct: S.dwAct },
    commands: { thr: S.thr, dt: S.dt, df: S.df, dw: S.dw },
    flight: {
      pos: { ...F.pos },
      vel: { ...F.vel },
      vWorld: { ...F.vWorld },
      euler: { ...F.euler },
      quat: { ...S.quat },
      omega: { ...S.omega },
    },
    flags: { sas: S.sas, aero: S.aero },
    time: S.time,
  };
}
