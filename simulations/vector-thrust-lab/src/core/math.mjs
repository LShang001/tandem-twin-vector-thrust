/**
 * @file    math.mjs
 * @module  仿真 core/姿态数学
 * @brief   四元数、旋转与欧拉角提取的纯函数集合
 * @details 输入：机体系/惯性系矢量与单位四元数 {x,y,z,w}；
 *          处理：标量钳位、四元数乘法/共轭/归一化、三明治积旋转、3-2-1 欧拉角提取；
 *          输出：新对象（不修改入参），被 dynamics/control/state 复用。
 *          不变量：core 层零 Three.js/DOM 依赖，Node 可独立单测；
 *          q 的语义固定为"机体系→惯性系"（MOD-002 §3.2）。
 * @theory  四元数代数与三明治积 @ref THY-001 §6；
 *          姿态运动学右乘式 q̇ = ½·q⊗(0,ω_b) @ref THY-001 §7；
 *          四元数→DCM 展开 @ref THY-001 §8；
 *          3-2-1 欧拉角提取（本项目锁定式）@ref MOD-002 §4
 * @frame   机体系 FRD → 惯性系 NED（z 向下）
 * @units   角度 rad，角速度 rad/s；本文件不做 deg 换算
 * @redline theta = −asin(R13) 为行为保持红线（@ref MOD-002 §4.3）
 * @see     dynamics.mjs、control.mjs、tests/math.test.mjs
 */

// @note 限幅类红线的统一实现：SAS 摆角/差速、α/β 定义域都走它；不处理 NaN
// @test tests/math.test.mjs「clamp 边界」
/**
 * @brief 把标量钳位到闭区间 [lo, hi]
 * @param {number} x 输入标量；@param {number} lo 下限；@param {number} hi 上限
 * @returns {number} 钳位结果（NaN 原样传播）
 */
export const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));

/**
 * @brief 构造三维矢量对象
 * @param {number} [x=0] x 分量；@param {number} [y=0] y 分量；@param {number} [z=0] z 分量
 * @returns {{x:number,y:number,z:number}} 矢量对象
 * @note 无参调用给零矢量；单位由使用处决定（位置 m、速度 m/s、力 N）
 */
export const vec3 = (x = 0, y = 0, z = 0) => ({ x, y, z });

/**
 * @brief 构造四元数对象
 * @param {number} [x=0] 虚部 i；@param {number} [y=0] 虚部 j；@param {number} [z=0] 虚部 k
 * @param {number} [w=1] 实部（默认 1 ⇒ 单位元）
 * @returns {{x:number,y:number,z:number,w:number}} 四元数
 */
export const quat = (x = 0, y = 0, z = 0, w = 1) => ({ x, y, z, w });

// ── 四元数旋转（三明治积）──────────────────────────────────────────
// 公式：v' = q ⊗ ṽ ⊗ q*，ṽ = (0, v)
// 符号：q 单位四元数（机体系→惯性系）；v/v' 待旋转/旋转后矢量
// 来源：@ref THY-001 §6（与罗德里格斯公式 v'=v·cosθ+n(n·v)(1−cosθ)+(n×v)·sinθ 等价）
// 实现：本函数展开为 t = 2(q×v)，v' = v + q_w·t + q×t —— 免构造中间四元数，少 2 次分配
// 等价变换：无（教材与项目同为右手系、同一四元数定义）
// 数值证据：tests/math.test.mjs「绕 z 轴旋转 90°」「旋转再逆旋转还原向量」
// ─────────────────────────────────────────────────────────────────
/**
 * @brief 用单位四元数旋转三维矢量：v' = q⊗ṽ⊗q*
 * @param v 待旋转矢量；@param q 单位四元数（须已归一化，体系→惯性系）
 * @returns {{x,y,z}} 旋转后的矢量（同单位）；@formula t=2(q×v)，v'=v+q_w·t+q×t
 * @ref THY-001 §6；@note 与 THREE.Vector3.applyQuaternion 逐项一致；@test tests/math.test.mjs
 */
export function rotateVecByQuat(v, q) {
  const { x: vx, y: vy, z: vz } = v;
  const { x: qx, y: qy, z: qz, w: qw } = q;
  // t = 2 × cross(q.xyz, v)
  const tx = 2 * (qy * vz - qz * vy);
  const ty = 2 * (qz * vx - qx * vz);
  const tz = 2 * (qx * vy - qy * vx);
  return {
    x: vx + qw * tx + qy * tz - qz * ty,
    y: vy + qw * ty + qz * tx - qx * tz,
    z: vz + qw * tz + qx * ty - qy * tx,
  };
}

/**
 * @brief 单位四元数求逆（= 共轭）
 * @param q 单位四元数；@returns {{x,y,z,w}} 共轭 (−x,−y,−z,w)
 * @formula q⁻¹ = q* = (−v, q₀)，仅当 |q| = 1；@ref THY-001 §6
 * @note 非单位四元数的真逆为 q* 除以 |q|²，本函数不补该除法（调用点均已保范）
 */
export const quatInvert = (q) => ({ x: -q.x, y: -q.y, z: -q.z, w: q.w });

// a ⊗ b（THREE.Quaternion.multiply 的分量顺序）
/**
 * @brief 四元数 Hamilton 乘 a⊗b（旋转复合：先 b 后 a）
 * @param a 左乘四元数；@param b 右乘四元数；@returns {{x,y,z,w}} a⊗b
 * @formula (p₀q₀−v_p·v_q, p₀v_q+q₀v_p+v_p×v_q)；@ref THY-001 §6
 * @note 分量顺序对齐 THREE.Quaternion.multiply；姿态积分固定右乘 q⊗ω̃（体系角速度）
 */
export function quatMultiply(a, b) {
  return {
    x: a.x * b.w + a.w * b.x + a.y * b.z - a.z * b.y,
    y: a.y * b.w + a.w * b.y + a.z * b.x - a.x * b.z,
    z: a.z * b.w + a.w * b.z + a.x * b.y - a.y * b.x,
    w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
  };
}

/**
 * @brief 四元数归一化（保范投影）
 * @param q 任意四元数；@returns {{x,y,z,w}} 单位四元数（零输入回退单位元）
 * @formula q ← q/‖q‖；@ref THY-001 §9（积分漂移与每步归一化）
 * @note 零输入回退 (0,0,0,1) 而非 NaN——RK4 中间点若抵消，不应污染整条状态链
 */
export function quatNormalize(q) {
  const l = Math.hypot(q.x, q.y, q.z, q.w);
  if (l === 0) return { x: 0, y: 0, z: 0, w: 1 };
  return { x: q.x / l, y: q.y / l, z: q.z / l, w: q.w / l };
}

// ── 3-2-1 欧拉角提取（本项目锁定约定，勿按教材"修正"）──────────────
// 公式：phi = atan2(R23,R33)，theta = −asin(R13)，psi = atan2(R12,R11)
// 符号：R_ij 为机体系→惯性系 DCM 元素；theta 的负号是本项目约定（非标准 3-2-1）
// 来源：@ref MOD-002 §4.1（公式）与 §4.3（行为锁定）；DCM 展开 @ref THY-001 §8
// 实现：本函数一次算出 m11/m12/m13/m23/m33 五个元素，省去构造整矩阵
// 等价变换：三式合起来等于"对 R^T 做标准 ZYX 分解"⇒ 三个角都是逆旋转角，
//           单轴正转 a 时 phi/theta/psi 读数均为 −a（phi、psi 同样取负，见 MOD-002 §4.2 注）
// 数值证据：@test tests/math.test.mjs「纯俯仰四元数 → theta = -a0」
// ─────────────────────────────────────────────────────────────────
/**
 * @brief 由单位四元数提取 3-2-1 欧拉角（项目锁定式）
 * @param q 单位四元数（机体系→惯性系）；@returns {{phi,theta,psi}} 显示角 [rad]
 * @formula phi=atan2(R23,R33)、theta=−asin(R13)、psi=atan2(R12,R11)；@ref MOD-002 §4.1
 * @note R13 先 clamp 到 [−1,1] 防浮点越界 NaN；返回值是 HUD/SAS 共用的"逆旋转角"
 */
export function eulerFromQuat(q) {
  const { x, y, z, w } = q;
  const x2 = x + x, y2 = y + y, z2 = z + z;
  const xx = x * x2, xy = x * y2, xz = x * z2;
  const yy = y * y2, yz = y * z2, zz = z * z2;
  const wx = w * x2, wy = w * y2, wz = w * z2;
  const m11 = 1 - (yy + zz);
  const m12 = xy - wz;
  const m13 = xz + wy;
  const m23 = yz - wx;
  const m33 = 1 - (xx + yy);
  return {
    phi: Math.atan2(m23, m33),
    theta: -Math.asin(clamp(m13, -1, 1)),
    psi: Math.atan2(m12, m11),
  };
}
