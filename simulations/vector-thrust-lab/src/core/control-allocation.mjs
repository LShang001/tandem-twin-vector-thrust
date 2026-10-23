/**
 * @file    control-allocation.mjs
 * @module  仿真 core/控制分配
 * @brief   B_true 在线控制效能 Jacobian 与 3×3 求逆（悬停增量分配用）
 * @details 输入：当前工作点 (ω0, δf, δt, Δω) 与参数表；
 *          处理：按解析偏导组装 B=∂[Mx,My,Mz]/∂[Δdw,Δδt,Δδf]，供 B⁻¹ 增量分配；
 *          输出：3×3 矩阵 / 逆矩阵（奇异时 null）。
 *          不变量：B 为"准稳态"效率矩阵——忽略 Jp·ω̇ 瞬态反扭与气动/陀螺项。
 * @theory  力矩映射 M=Σ[r×F + s·τ·n] @ref THY-003 §2.1；
 *          摆角雅可比 ∂M/∂δ @ref THY-003 §2.2；
 *          线性化分配 Δu = B⁻¹·ΔM @ref THY-003 §3.2、@ref CTL-001 §8.5
 * @frame   机体系 FRD；列序固定 [Δdw(Δω), Δδt(δt), Δδf(δf)]
 * @units   ω0 [rad/s]、δ [rad]、Δω [-]，B 元素单位 N·m 每执行器单位
 * @redline 列序/行序是与固件 TandemVec_ControlAllocation.h 的接口约定，改动即失配
 * @note 三端同构：Python core.control_effectiveness、固件 TandemVec_ControlAllocation.h
 * @see     control.mjs（applyVtolHover 的 B_true 分支）、tests/control-allocation.test.mjs
 */

// @formula A⁻¹ = adj(A)/det(A)，det 阈值 1e-12
// @ref THY-003 §3.3（几何奇异/权限退化）
// @note 返回 null 而非抛异常/NaN：调用方（悬停分配）据此回退对角映射；
//       低油门时 B∝ω0² 必然近奇异，这是设计内的工况而非错误
// @test tests/control-allocation.test.mjs「低油门奇异防护」「B⁻¹·B = I」
/**
 * @brief 3×3 矩阵求逆（伴随矩阵法）
 * @param {number[][]} m 3×3 矩阵（行优先）
 * @returns {number[][]|null} 逆矩阵；行列式近零或非有限时返回 null
 */
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

// ── B_true：推进力矩对执行器增量的解析 Jacobian ────────────────────
// 公式：B[i][j] = ∂M_i/∂u_j，u=[Δω, δt, δf]
// 符号：T0=kT·ω0²、tau0=kQ·ω0² 为工作点基准；Tf=T0(1+Δω)、Tt=T0(1−Δω)
// 来源：@ref THY-003 §2.2（雅可比解析链）、@ref CTL-001 §8.5（三端同构）
// 实现：`computeBTrue` 返回 3×3；偏导直接对六维映射求导（无差分，无迭代）
// 等价变换：差速开方分配使 ωf²=ω0²(1+Δω) ⇒ T/Q 对 Δω 严格线性，故 B 的 dw 列是精确偏导
//          （不是小 Δω 近似）；钳位 wMax 后该线性在边界失效（分配需限幅兜底）
// 数值证据：@test tests/control-allocation.test.mjs「与数值差分一致 < 1e-6」
// ─────────────────────────────────────────────────────────────────
// @formula T0=kT·ω0²、tau0=kQ·ω0²；Tf=T0(1+Δω)、Tt=T0(1−Δω)、τf/τt 同构
// @ref THY-003 §2.2；@ref CTL-001 §8.5
// @note 忽略项：Jp·ω̇ 瞬态反扭、气动力矩、ω×h 陀螺项——分配层只负责执行器，
//       外扰与刚体耦合由内环闭环抑制（与 CTL-001 §8.5 口径一致）
// @note Δω 列含 −tau0(cf+ct)：零摆角时退化为 −2·kQ·ω0²（CTL-001 §7.4 的线性效率）
// @test tests/control-allocation.test.mjs（差分一致性、Python 同构、分配精度）
/**
 * @brief 组装当前工作点的控制效能矩阵 B = ∂[Mx,My,Mz]/∂[Δdw,Δδt,Δδf]
 * @param omega0 基准转速 [rad/s]；@param delta_f 前摆角 [rad]
 * @param delta_t 尾摆角 [rad]；@param dw 差速指令 [-]；@param {Object} P 参数表
 * @returns {number[][]} 3×3 B 矩阵，行 [Mx,My,Mz]、列 [dw,δt,δf]
 */
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
