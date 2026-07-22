# 文献笔记：Full Envelope Control of Over-Actuated Fixed-Wing Vectored Thrust eVTOL

- **编号**：REF-105
- **作者**：Emmanuel Enenakpogbe, James F. Whidborne, Linghai Lu (Cranfield University)
- **年份**：2024
- **来源**：Aerospace (MDPI), 11, 979，期刊论文（扩展自 UKACC CONTROL 2024 会议论文）
- **DOI**：[10.3390/aerospace11120979](https://doi.org/10.3390/aerospace11120979)
- **关联项目模块**：§3 推力矢量映射、§5 控制分配、§7 SAS 控制律设计

---

## 核心内容

1. **平台**：Lilium Jet 风格的 7 座 eVTOL——鸭式布局、36 个涵道风扇（EDF）、8 组倾转控制面
2. **控制器架构**：两级级联
   - 内环：经典 NDI（Nonlinear Dynamic Inversion）——对消机体非线性动力学，使系统表现为纯积分器
   - 外环：PID 线性控制器——位置/姿态跟踪
   - 统一框架：无需增益调度，全包线（悬停→过渡→巡航→反向过渡）有效
3. **控制分配核心创新**：将推力矢量执行器的**极坐标模型（推力大小+倾角）转换为笛卡尔坐标（x/z分量）**
   - 转换后执行器映射变为**线性** → 可直接用经典线性 QP 优化
   - 非线性被转移到执行器限幅的计算中（极坐标限幅→笛卡尔限幅的映射）
   - 用 Active Set 加权最小二乘 QP 求解，支持通道优先级和饱和约束
4. **6DoF 仿真验证**：垂直起飞带航向变化、前向过渡、爬升/下降、协调转弯、垂直着陆
5. **实时性验证**：CA 迭代次数 <20 次，不产生执行器状态跳变

## 与项目的关系

| 维度 | 本文 (REF-105) | 本项目 |
|------|---------------|--------|
| 平台 | 36 EDF 鸭式 eVTOL，8 组倾转控制面 | 2 电机固定翼，2 单轴摆座 |
| 驱动类型 | **重度过驱动**（36 EDF → 8 虚拟控制 → 6 DoF） | **恰驱动**（4 执行器 → 4 自由度） |
| 高层控制 | NDI（对消非线性 + 纯积分器） | SAS 增稳（角速率阻尼 + 姿态 PI） |
| 控制分配核心 | 极坐标→笛卡尔坐标线性化 + Active Set QP | 正交摆轴几何解耦 → 对角近似 |
| 执行器限幅处理 | 内嵌在 QP 约束中 | 独立硬限幅 + 积分钳位 |
| 鲁棒性 | 依赖精确模型（NDI 的已知缺陷） | 反馈结构天然鲁棒（待量化） |

### 可借鉴的方法

- **极坐标→笛卡尔坐标的转换技巧**：如果我们未来增加执行器（如引入气动舵面），执行器模型会变非线性。本文的"把非线性从映射矩阵移到限幅计算"的思路可以直接套用
- **Active Set QP 的通道优先级加权**：$\mathbf{W}_v$ 矩阵对不同控制通道（滚转>俯仰>偏航）赋不同权重——如果我们的耦合项需要更精细的处理，这个框架是现成方案
- **统一控制器无模式切换**：NDI 的全包线能力值得关注——如果我们的飞行包线扩展到含失速/大迎角，当前 SAS 的线性假设会失效

### 关键差异（不适用之处）

- 本文的 CA 方案解决的是"8 个虚拟控制量→36 个 EDF 指令"的分配，计算量远大于我们的"3→3"对角映射
- NDI 需要精确的机载模型（OBM）来做对消——我们作为概念级项目没有这个条件
- 36 个 EDF 的机械复杂度与我们的"2 电机+2 单轴摆座"是完全不同的设计哲学

## 关键摘录

> "The control allocation scheme uses a novel architecture, which transfers the nonlinearity in the vectored thrust effector model formulation to the computation of the actuator limits by converting the effector model from polar to rectangular form, thus allowing the use of classical control allocation linear optimisation technique."

> "The maximum number of CA iterations 'CA Iteration Number' are all below 20 when the AS CCA is active. Therefore, the CA algorithm finds the optimal solution quickly."

> "NDI relies on an accurate model of the system, which is not possible and hence not inherently robust to model mismatches and external disturbances."

## 引用建议

- 在 THY-004 §5（控制分配）讨论过驱动 vs 恰驱动时，作为"过驱动+在线优化"范式的代表
- 在 §8（讨论）比较控制架构时，NDI+CA vs SAS+直接映射是两种不同的复杂度-鲁棒性权衡
- 极坐标→笛卡尔坐标转换技巧可在未来引入气动舵面后的混合控制分配中使用
