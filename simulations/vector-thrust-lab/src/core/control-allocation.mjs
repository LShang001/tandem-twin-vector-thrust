// ============================================================
//  B_true — 在线控制效能 Jacobian（控制分配）
//  对齐 Python core.control_effectiveness / 论文 ch/10 / 固件
//  TandemVec_ControlAllocation.h（三端同构）
//
//  B 矩阵：行 = [Mx, My, Mz]（力矩增量），列 = [Δdw, Δδt, Δδf]（执行器增量）
//  B[i,j] = ∂M_i/∂u_j 在当前工作点 (ω0, δf, δt, dw) 的解析偏导
//  （ω0 = 当前油门基准转速；Tf/Tt、Qf/Qt 用差速线性化 T0·(1±dw)）
//
//  用法（纯控制分配，非 INDI）：
//    内环：M_des = diag(I)·K·(ωdes − ω)（目标力矩，对角独立）
//    分配：Δu = B⁻¹·(M_des − M_cur) → u += Δu（限幅）
// ============================================================

// 3×3 矩阵求逆（伴随矩阵法，det≠0）
export function inv3(m) {
  const [a, b, c] = m[0], [d, e, f] = m[1], [g, h, i] = m[2];
  const det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
  if (!Number.isFinite(det) || Math.abs(det) < 1e-12) return null;   // 奇异（低油门）
  const inv = 1 / det;
  return [
    [(e * i - f * h) * inv, (c * h - b * i) * inv, (b * f - c * e) * inv],
    [(f * g - d * i) * inv, (a * i - c * g) * inv, (c * d - a * f) * inv],
    [(d * h - e * g) * inv, (b * g - a * h) * inv, (a * e - b * d) * inv],
  ];
}

// 在线 Jacobian：B = ∂[Mx,My,Mz]/∂[dw,δt,δf]（解析偏导，忽略 Jp·ω̇ 瞬态项）
export function computeBTrue(omega0, delta_f, delta_t, dw, P) {
  const T0 = P.kT * omega0 * omega0;      // 基准推力
  const tau0 = P.kQ * omega0 * omega0;    // 基准反扭
  const Tf = T0 * (1 + dw), Tt = T0 * (1 - dw);
  const tauf = tau0 * (1 + dw), taut = tau0 * (1 - dw);
  const sf = Math.sin(delta_f), cf = Math.cos(delta_f);
  const st = Math.sin(delta_t), ct = Math.cos(delta_t);
  // 行 [Mx, My, Mz] × 列 [dw, dt, df]
  return [
    [-tau0 * (cf + ct), -taut * st, tauf * sf],        // Mx
    [P.b * T0 * st - tau0 * sf, -P.b * Tt * ct, -tauf * cf],  // My
    [P.a * T0 * sf + tau0 * st, -taut * ct, P.a * Tf * cf],   // Mz
  ];
}
