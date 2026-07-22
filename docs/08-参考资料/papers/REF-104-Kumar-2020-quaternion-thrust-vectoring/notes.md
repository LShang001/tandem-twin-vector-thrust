# 文献笔记：Quaternion Feedback Based Autonomous Control of a Quadcopter UAV with Thrust Vectoring Rotors

- **编号**：REF-104
- **作者**：Rumit Kumar, Mahathi Bhargavapuri, Aditya M. Deshpande, Siddharth Sridhar, Kelly Cohen, Manish Kumar
- **年份**：2020
- **来源**：American Control Conference (ACC) 2020，6 页
- **arxiv**：[2006.15686](https://arxiv.org/abs/2006.15686)
- **关联项目模块**：§3 推力矢量映射、§4 四元数姿态运动学、§5 控制分配、§7 SAS 控制律设计

---

## 核心内容

1. **平台**：四旋翼 UAV，每个旋翼可通过舵机绕单轴倾斜（tilt-rotor），形成过驱动系统（8 执行器控制 6 自由度）
2. **姿态控制**：四元数反馈（quaternion state feedback），避免欧拉角万向锁
   - 四元数运动学 `\dot{q} = ½ q ⊗ [0, p, q, r]^T`——与项目 `eq:quaternion_kinematics` 完全一致
   - 从四元数提取欧拉角用 asin（与项目的 θ = -asin(R₁₃) 不同的符号约定，但原理相同）
3. **控制分配**：在悬停点处用 Taylor 展开线性化力矩方程（忽略 Coriolis 项 Ω×IΩ），推导控制分配矩阵（式 28，8×7）
   - 这是**数学推导**的控制分配，非几何设计——与项目"正交摆轴天然解耦"思路形成对比
4. **Lyapunov 稳定性**：对姿态误差四元数 + 角速率误差构造 Lyapunov 函数，证明闭环全局渐近稳定
5. **外环位置控制**：PID 生成期望加速度 → 通过向量对齐计算期望四元数 → 输入内环姿态控制器

## 与项目的关系

| 维度 | 本文 (REF-104) | 本项目的纵列双发构型 |
|------|---------------|---------------------|
| 平台 | 四旋翼，每旋翼可单轴倾斜 | 固定翼，双电机沿纵轴布置 |
| 执行器数量 | 8（4 电机转速 + 4 倾转角） | 4（2 电机转速 + 2 摆角） |
| 驱动类型 | **过驱动**（8→6） | **恰驱动**（4→4） |
| 控制分配方法 | 悬停点线性化 → 8×7 矩阵 | 正交摆轴几何解耦 → 对角近似 + 交叉项补偿 |
| 四元数用法 | 误差四元数反馈 + Lyapunov | 姿态运动学积分 + DCM 提取欧拉角 |
| 姿态控制律 | `τ = -k_Q·ε - k_Ω·Ω_err`（四元数域） | `δ = k_q·q - k_θ·Δθ`（欧拉角域 SAS） |

### 可借鉴的方法

- **Lyapunov 稳定性分析**：本文 §III-B 的姿态误差四元数 Lyapunov 推导可以适配到项目的四元数反馈控制律设计中（项目当前是欧拉角域 SAS，未在四元数域做稳定性分析）
- **误差四元数定义**：`q_err = q_des* ⊗ q`，这是四元数反馈的标准做法，如果项目未来升级到四元数域控制可参考
- **控制分配推导方法**：Taylor 展开线性化思路与项目 §5 的线性化效能矩阵推导一致，可交叉验证

### 关键差异（不适用之处）

- 本文的"过驱动 + 线性化控制分配"范式与项目的"几何设计换取算法简洁"哲学相反——项目刻意避免在线矩阵求逆
- 本文忽略 Coriolis 项（Ω×IΩ）简化推导，项目保留完整牛-欧方程
- 本文是四旋翼悬停基准，项目是固定翼巡航基准——配平点完全不同

## 关键摘录

> "quaternion state feedback is utilized to compute the control commands for the UAV motors while avoiding the gimbal lock condition experienced by Euler angle based controllers."

> "The moment equations are linearized using small perturbation theory or Taylor's series expansion to derive the necessary control allocation."

> "Lyapunov stability analysis of the attitude controller is shown to prove global stability."

> 式 (10)：悬停点处 `[q₀,q₁,q₂,q₃] = [1,0,0,0]` 简化了四元数运动学——`q̇ ≈ ½[0,p,q,r]^T`

---

## 引用建议

在项目文档/论文中可按以下场景引用：

- 在 §7（控制律设计）讨论四元数反馈 vs 欧拉角反馈时，引为四元数域姿态控制的工程实现范例
- 在 §5（控制分配）讨论过驱动 vs 恰驱动控制分配方法时，作为对比参照
- 在 §6（稳定性分析）讨论 Lyapunov 方法时，作为可复现的分析模板
