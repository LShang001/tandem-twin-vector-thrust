式(3.10.13)两边对 $b_{i}^{r}$ 取偏导数,得

$$
\begin{array}{r l} \boldsymbol {r} _ {i, k / k - 1} & = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {r} _ {i, k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} + \frac {\partial}{\partial b _ {i} ^ {r}} (\boldsymbol {\Gamma} _ {k - 1} ^ {r} \boldsymbol {Q} _ {k - 1} ^ {r} \boldsymbol {\Gamma} _ {k - 1} ^ {r \mathrm{T}}) \Bigg | _ {\boldsymbol {b} ^ {r} = \boldsymbol {b}} + \\ & \quad \boldsymbol {\Phi} _ {k / k - 1} \boldsymbol {P} _ {k - 1} ^ {\mathrm{T}} \frac {\partial \boldsymbol {\Phi} _ {k , k - 1} ^ {\mathrm{T}}}{\partial b _ {i} ^ {r}} \Bigg | _ {\boldsymbol {b} ^ {r} = \boldsymbol {b}} + \frac {\partial \boldsymbol {\Phi} _ {k , k - 1} ^ {r}}{\partial b _ {k} ^ {i}} \Bigg | _ {\boldsymbol {b} ^ {r} = \boldsymbol {b}} \boldsymbol {P} _ {k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \end{array}\tag{3.10.21}
$$

式中

$$
\boldsymbol {r} _ {i, 0} = \frac {\partial \boldsymbol {P} _ {0} ^ {P}}{\partial b _ {i} ^ {r}} \Bigg | _ {\boldsymbol {b} ^ {r} = \boldsymbol {b}}\tag{3.10.22}
$$

可以看出,小尺度灵敏度的求解不需要真实系统和量测的具体参数值,只需要模型的参数值。但如果真实的参数值与模型的参数值相差很大,而这又是不了解真实系统的经常有的情况,则小尺度灵敏度就不足以表征实际的灵敏度。因此,小尺度灵敏度的应用有一定的局限性。

## 3.10.2 连续系统的卡尔曼滤波误差分析一般方法

## 1. 方差误差分析

设真实系统和量测为

$$
\dot {\boldsymbol {X}} ^ {r} (t) = \boldsymbol {F} ^ {r} (t) \boldsymbol {X} ^ {r} (t) + \boldsymbol {G} ^ {r} (t) \boldsymbol {w} ^ {r} (t)\tag{3.10.23}
$$

$$
\mathbf {Z} (t) = \mathbf {H} ^ {r} (t) \mathbf {X} ^ {r} (t) + \mathbf {v} ^ {r} (t)\tag{3.10.24}
$$

而滤波器设计中采用的系统模型和量测模型为

$$
\dot {\boldsymbol {X}} (t) = \boldsymbol {F} (t) \boldsymbol {X} (t) + \boldsymbol {G} (t) \boldsymbol {w} (t)\tag{3.10.25}
$$

$$
\mathbf {Z} (t) = \mathbf {H} (t) \mathbf {X} (t) + \mathbf {v} (t)\tag{3.10.26}
$$

上述诸方程中的噪声统计特性与离散系统中的说明类似,这里不再详述。根据模型,滤波计算方程为

令

$$
\begin{array}{r l} \dot {\hat {X}} (t) & = F (t) \hat {X} (t) + \bar {K} (t) [ Z (t) - H (t) \hat {X} (t) ] \\ & \quad \tilde {X} ^ {P} (t) \stackrel {\text { def }} {=} X ^ {r} (t) - \hat {X} (t) \\ & \quad \Delta F (t) \stackrel {\text { def }} {=} F ^ {r} (t) - F (t) \\ & \quad \Delta H (t) \stackrel {\text { def }} {=} H ^ {r} (t) - H (t) \end{array}\tag{3.10.27}
$$

则

$$
\begin{array}{r l} \widetilde {\boldsymbol {X}} ^ {p} (t) & = [ \boldsymbol {F} (t) - \overline {{\boldsymbol {K}}} (t) \boldsymbol {H} (t) ] \widetilde {\boldsymbol {X}} ^ {p} (t) + [ \Delta \boldsymbol {F} (t) - \overline {{\boldsymbol {K}}} (t) \Delta \boldsymbol {H} (t) ] \boldsymbol {X} ^ {r} (t) + \\ & \quad \boldsymbol {G} ^ {r} (t) \boldsymbol {w} ^ {r} (t) - \overline {{\boldsymbol {K}}} (t) \boldsymbol {v} ^ {r} (t) \end{array}\tag{3.10.28}
$$

实际估计误差 $\widetilde{\mathbf{X}}^P (t)$ 的微分方程中还包含状态 $X^{r}(t)$ 。因此，为了求 $\widetilde{\mathbf{X}}^P (t)$ 的均方阵 $P^P (t)$ ，定义一个新的状态向量

$$
\boldsymbol {X} ^ {a} (t) \stackrel {\text { def }} {=} \left[ \begin{array}{c} \widetilde {\boldsymbol {X}} ^ {P} (t) \\ \boldsymbol {X} ^ {r} (t) \end{array} \right]
$$

从求 $X^{a}(t)$ 的均方阵求出 $\widetilde{X}^{P}(t)$ 的均方阵。 $X^{a}(t)$ 的状态方程为

$$
\dot {\boldsymbol {X}} ^ {a} = \boldsymbol {F} ^ {a} (t) \boldsymbol {X} ^ {a} (t) + \boldsymbol {w} ^ {a} (t)\tag{3.10.29}
$$

式中

$$
\boldsymbol {F} ^ {a} (t) = \left[ \begin{array}{c c} \boldsymbol {F} (t) - \bar {\boldsymbol {K}} (t) \boldsymbol {H} (t) & \Delta \boldsymbol {F} (t) - \bar {\boldsymbol {K}} (t) \Delta \boldsymbol {H} (t) \\ \boldsymbol {0} & \boldsymbol {F} ^ {r} (t) \end{array} \right]
$$

$$
\boldsymbol {w} ^ {a} (t) = \left[ \begin{array}{c} \boldsymbol {G} ^ {r} (t) \boldsymbol {w} ^ {r} (t) - \bar {\boldsymbol {K}} (t) \boldsymbol {v} ^ {r} (t) \\ \boldsymbol {G} ^ {r} (t) \boldsymbol {w} ^ {r} (t) \end{array} \right]
$$

从上式知 $w^{a}(t)$ 是白噪声向量, 其协方差阵为

$$
\operatorname{Cov} \left[ w ^ {a} (t), w ^ {a} (\tau) \right] = E \left[ w ^ {a} (t) w ^ {a T} (\tau) \right] = T (t) \delta (t - \tau)\tag{3.10.30}
$$

式中

$$
\boldsymbol {T} = \left[ \begin{array}{c c} \boldsymbol {G} ^ {r} (t) \boldsymbol {Q} ^ {r} (t) \boldsymbol {G} ^ {r \mathrm{T}} (t) + \bar {\boldsymbol {K}} (t) \boldsymbol {R} ^ {r} (t) \bar {\boldsymbol {K}} ^ {\mathrm{T}} (t) & \boldsymbol {G} ^ {r} (t) \boldsymbol {Q} ^ {r} (t) \boldsymbol {G} ^ {r \mathrm{T}} (t) \\ \boldsymbol {G} ^ {r} (t) \boldsymbol {Q} ^ {r} (t) \boldsymbol {G} ^ {r \mathrm{T}} (t) & \boldsymbol {G} ^ {r} (t) \boldsymbol {Q} ^ {r} (t) \boldsymbol {G} ^ {r \mathrm{T}} (t) \end{array} \right]\tag{3.10.31}
$$

令

$$
\boldsymbol {P} ^ {a} (t) = E \left[ \boldsymbol {X} ^ {a} (t) \boldsymbol {X} ^ {a T} (t) \right]
$$

则

$$
\begin{array}{r l} \dot {\mathbf {P}} ^ {a} (t) & = \frac {\mathrm{d}}{\mathrm{d} t} E [ \mathbf {X} ^ {a} (t) \mathbf {X} ^ {a \mathrm{T}} (t) ] = \\ & E [ \dot {\mathbf {X}} ^ {a} (t) \mathbf {X} ^ {a \mathrm{T}} (t) + \mathbf {X} ^ {a} (t) \dot {\mathbf {X}} ^ {a \mathrm{T}} (t) ] = \\ & E [ \mathbf {F} ^ {a} (t) \mathbf {X} ^ {a} (t) \mathbf {X} ^ {a \mathrm{T}} (t) + \mathbf {w} ^ {a} (t) \mathbf {X} ^ {a \mathrm{T}} (t) + \mathbf {X} ^ {a} (t) \mathbf {X} ^ {a \mathrm{T}} (t) \mathbf {F} ^ {a \mathrm{T}} (t) + \\ & \mathbf {X} ^ {a} (t) \mathbf {w} ^ {a \mathrm{T}} (t) ] = \mathbf {F} ^ {a} (t) \mathbf {P} ^ {a} (t) + \mathbf {P} ^ {a} (t) \mathbf {F} ^ {a \mathrm{T}} (t) + \\ & E [ \mathbf {w} ^ {a} (t) \mathbf {X} ^ {a \mathrm{T}} (t) + \mathbf {X} ^ {a} (t) \mathbf {w} ^ {a \mathrm{T}} (t) ] \end{array} \tag {3}\tag{3.10.32}
$$

为了求出上式中的均值项,需从式(3.10.29)中解出 $X^{a}(t)$ 。即

$$
\boldsymbol {X} ^ {a} (t) = \boldsymbol {\Phi} ^ {a} (t, t _ {0}) \boldsymbol {X} ^ {a} (t _ {0}) + \int_ {t _ {0}} ^ {t} \boldsymbol {\Phi} ^ {a} (t, \tau) \boldsymbol {w} ^ {a} (\tau) \mathrm{d} \tau\tag{3.10.33}
$$

其中一步转移阵 $\Phi^{a}(t,\tau)$ 由下述微分方程确定

$$
\left\{ \begin{array}{l} \frac {\mathrm{d}}{\mathrm{d} t} [ \boldsymbol {\Phi} ^ {a} (t, \tau) ] = \mathbf {F} ^ {a} (t) \boldsymbol {\Phi} ^ {a} (t, \tau) \\ \boldsymbol {\Phi} ^ {a} (t, t) = \mathbf {I} \end{array} \right.
$$

当 $t \geqslant t_0$ 时， $w^a(t)$ 与 $X^a(t_0)$ 不相关。所以

$$
\begin{array}{r l} & {E [ w ^ {a} (t) X ^ {a \mathrm{T}} (t) + X ^ {a} (t) w ^ {a \mathrm{T}} (t) ] =} \\ & {\int_ {t _ {0}} ^ {t} E [ w ^ {a} (t) w ^ {a \mathrm{T}} (\tau) ] \Phi^ {a \mathrm{T}} (t, \tau) \mathrm{d} \tau + \int_ {t _ {0}} ^ {t} \Phi^ {a} (t, \tau) E [ w ^ {a} (\tau) W ^ {a \mathrm{T}} (t) ] \mathrm{d} \tau =} \\ & {\int_ {t _ {0}} ^ {t} T (t) \Phi^ {a \mathrm{T}} (t, \tau) \delta (t - \tau) \mathrm{d} \tau + \int_ {t _ {0}} ^ {t} \Phi^ {a} (t, \tau) T (\tau) \delta (t - \tau) \mathrm{d} \tau} \end{array}
$$

为求上式中的积分,取

$$
R _ {c} (t - \tau) = \left\{ \begin{array}{l l} \frac {1}{\Delta t}, & t - \frac {\Delta t}{2} \leqslant \tau \leqslant t + \frac {\Delta t}{2} \\ 0, & \text {其余} t \end{array} \right.
$$

则

$$
\begin{array}{r l} \delta (t - \tau) & = \lim _ {\Delta t \to 0} R _ {c} (t - \tau) \\ \int_ {t _ {0}} ^ {t} T (t) \Phi^ {a \mathrm{T}} (t, \tau) \delta (t - \tau) \mathrm{d} \tau & = T (t) \int_ {t _ {0}} ^ {t} \Phi^ {a \mathrm{T}} (t, \tau) \lim _ {\Delta t \to 0} R _ {c} (t - \tau) \mathrm{d} \tau = \\ & T (t) \lim _ {\Delta t \to 0} \int_ {t - \frac {\Delta t}{2}} ^ {t} \Phi^ {a \mathrm{T}} (t, \tau) \frac {1}{\Delta t} \mathrm{d} \tau = \\ & T (t) I \cdot \frac {1}{\Delta (t)} (t - t + \frac {\Delta t}{2}) = \frac {1}{2} T (t) \end{array}
$$

同理可得

$$
\int_ {t _ {0}} ^ {t} \pmb {\Phi} ^ {a} (t, \tau) \pmb {T} (\tau) \delta (t - \tau) \mathrm{d} \tau = \frac {1}{2} \pmb {T} (t)
$$

所以式 $(3.10.32)$ 成

$$
\dot {\boldsymbol {P}} ^ {a} (t) = \boldsymbol {F} ^ {a} (t) \boldsymbol {P} ^ {a} (t) + \boldsymbol {P} ^ {a} (t) \boldsymbol {F} ^ {a T} (t) + \boldsymbol {T} (t)\tag{3.10.34}
$$

令

$$
\boldsymbol {P} ^ {a} (t) = \left[ \begin{array}{c c} \boldsymbol {P} ^ {P} (t) & \boldsymbol {C} ^ {\mathrm{T}} (t) \\ \boldsymbol {C} (t) & \boldsymbol {A} (t) \end{array} \right]
$$

式中

$$
\left. \begin{array}{l} \boldsymbol {P} ^ {P} (t) \stackrel {\text { def }} {=} E [ \widetilde {\boldsymbol {X}} ^ {P} (t) \widetilde {\boldsymbol {X}} ^ {P T} ] \\ \boldsymbol {C} (t) \stackrel {\text { def }} {=} E [ \boldsymbol {X} ^ {r} (t) \widetilde {\boldsymbol {X}} ^ {P T} (t) \\ \boldsymbol {A} (t) \stackrel {\text { def }} {=} E [ \boldsymbol {X} ^ {r} (t) \boldsymbol {X} ^ {r T} (t) ] \end{array} \right\}\tag{3.10.35}
$$

则式 $(3.10.34)$ 可写成

$$
\begin{array}{r l} \left[ \begin{array}{c c} \dot {\boldsymbol {P}} ^ {P} (t) & \dot {\boldsymbol {C}} ^ {\mathrm{T}} (t) \\ \dot {\boldsymbol {C}} (t) & \dot {\boldsymbol {A}} (t) \end{array} \right] = & \\ & \left[ \begin{array}{c c} \boldsymbol {F} (t) - \overline {{\boldsymbol {K}}} (t) \boldsymbol {H} (t) & \Delta \boldsymbol {F} (t) - \overline {{\boldsymbol {K}}} (t) \Delta \boldsymbol {H} (t) \\ \boldsymbol {0} & \boldsymbol {F} ^ {r} (t) \end{array} \right] \left[ \begin{array}{c c} \boldsymbol {P} ^ {P} (t) & \boldsymbol {C} ^ {\mathrm{T}} (t) \\ \boldsymbol {C} (t) & \boldsymbol {A} (t) \end{array} \right] + \\ & \left[ \begin{array}{c c} \boldsymbol {P} ^ {P} (t) & \boldsymbol {C} ^ {\mathrm{T}} (t) \\ \boldsymbol {C} (t) & \boldsymbol {A} (t) \end{array} \right] \left[ \begin{array}{c c} [ \boldsymbol {F} (t) - \overline {{\boldsymbol {K}}} (t) \boldsymbol {H} (t) ] ^ {\mathrm{T}} & \boldsymbol {0} \\ [ \Delta \boldsymbol {F} (t) - \overline {{\boldsymbol {K}}} (t) \Delta \boldsymbol {H} (t) ] ^ {\mathrm{T}} & \boldsymbol {F} ^ {r \mathrm{T}} (t) \end{array} \right] + \\ & \left[ \begin{array}{c c} \boldsymbol {G} ^ {r} (t) \boldsymbol {Q} ^ {r} (t) \boldsymbol {G} ^ {r \mathrm{T}} (t) + \overline {{\boldsymbol {K}}} (t) \boldsymbol {R} ^ {r} (t) \overline {{\boldsymbol {K}}} ^ {\mathrm{T}} (t) & \boldsymbol {G} ^ {r} (t) \boldsymbol {Q} ^ {r} (t) \boldsymbol {G} ^ {r \mathrm{T}} (t) \\ \boldsymbol {G} ^ {r} (t) \boldsymbol {Q} ^ {r} (t) \boldsymbol {G} ^ {r \mathrm{T}} (t) & \boldsymbol {G} ^ {r} (t) \boldsymbol {Q} ^ {r} (t) \boldsymbol {G} ^ {r \mathrm{T}} (t) \end{array} \right] \end{array}
$$

从上式得

$$
\begin{array}{r l} & {\dot {\mathbf {P}} ^ {P} (t) = [ \mathbf {F} (t) - \bar {\mathbf {K}} (t) \mathbf {H} (t) ] \mathbf {P} ^ {P} (t) + \mathbf {P} ^ {P} (t) [ \mathbf {F} (t) - \bar {\mathbf {K}} (t) \mathbf {H} (t) ] ^ {\mathrm{T}} +} \\ & {\qquad [ \Delta \mathbf {F} (t) - \bar {\mathbf {K}} (t) \Delta \mathbf {H} (t) ] \mathbf {C} (t) + \mathbf {C} ^ {\mathrm{T}} (t) [ \Delta \mathbf {F} (t) - \bar {\mathbf {K}} (t) \Delta \mathbf {H} (t) ] ^ {\mathrm{T}} +} \\ & {\qquad \mathbf {G} ^ {r} (t) \mathbf {Q} ^ {r} (t) \mathbf {G} ^ {r \mathrm{T}} (t) + \bar {\mathbf {K}} (t) \mathbf {R} ^ {r} (t) \bar {\mathbf {K}} ^ {\mathrm{T}} (t)} \\ & {\dot {\mathbf {C}} (t) = \mathbf {F} ^ {r} (t) \mathbf {C} (t) + \mathbf {C} (t) [ \mathbf {F} (t) - \bar {\mathbf {K}} (t) \mathbf {H} (t) ] ^ {\mathrm{T}} +} \\ & {\qquad \mathbf {A} (t) [ \Delta \mathbf {F} (t) - \bar {\mathbf {K}} (t) \Delta \mathbf {H} (t) ] ^ {\mathrm{T}} + \mathbf {G} ^ {r} (t) \mathbf {Q} ^ {r} (t) \mathbf {G} ^ {r \mathrm{T}} (t)} \\ & {\dot {\mathbf {A}} (T) = \mathbf {F} ^ {r} (t) \mathbf {A} (t) + \mathbf {A} (t) \mathbf {F} ^ {r \mathrm{T}} (t) + \mathbf {G} ^ {r} (t) \mathbf {Q} ^ {r} (t) \mathbf {G} ^ {r \mathrm{T}} (t)} \end{array}\tag{3.10.36}
$$

(3.10.37)

(3.10.38)

各均方阵的初始值为

$$
\begin{array}{r l} & \boldsymbol {P} ^ {P} (0) = E \{\left[ \boldsymbol {X} ^ {r} (0) - \hat {\boldsymbol {X}} (0) \right] \left[ \boldsymbol {X} ^ {r} (0) - \hat {\boldsymbol {X}} (0) \right] ^ {\mathrm{T}} \} \\ & \boldsymbol {A} (0) = E \left[ \boldsymbol {X} ^ {r} (0) \boldsymbol {X} ^ {r \mathrm{T}} (0) \right] \\ & \boldsymbol {C} (0) = E \{\boldsymbol {X} ^ {r} (0) \left[ \boldsymbol {X} ^ {r} (0) - \hat {\boldsymbol {X}} (0) \right] ^ {\mathrm{T}} \} \end{array}
$$

从式(3.10.38)、式(3.10.37)和式(3.10.36)可以依次解出 $A(t)$ , $C(t)$ , 最后解出 $P^{P}(t)$ 。

## 2. 灵敏度分析

与离散系统相同,大尺度灵敏度为

$$
\mathbf {S} _ {i} (t) \stackrel {\mathrm{def}} {=} \frac {\mathbf {P} ^ {P} (t) - \mathbf {P} (t)}{\Delta b _ {i}}\tag{3.10.39}
$$

小尺度灵敏度为

$$
\boldsymbol {r} _ {i} (t) \stackrel {\text { def }} {=} \frac {\partial \boldsymbol {P} ^ {p} (t)}{\partial b _ {i} ^ {r}} \Bigg | _ {\boldsymbol {b} ^ {r} = \boldsymbol {b}}\tag{3.10.40}
$$

对式(3.10.36)两边求偏导数,并交换微分次序,再令 $b^{r}=b$ ,则可得小尺度灵敏度的微分方程

$$
\begin{array}{r l} \dot {\boldsymbol {r}} _ {i} (t) & = [ \boldsymbol {F} (t) - \overline {{\boldsymbol {K}}} (t) \boldsymbol {H} (t) ] \boldsymbol {r} _ {i} (t) + \boldsymbol {r} _ {i} (t) [ \boldsymbol {F} (t) - \overline {{\boldsymbol {K}}} (t) \boldsymbol {H} (t) ] ^ {\mathrm{T}} + \\ & \frac {\partial [ \boldsymbol {F} ^ {r} (t) - \overline {{\boldsymbol {K}}} (t) \boldsymbol {H} ^ {r} (t) ]}{\partial b _ {i} ^ {r}} \Bigg | _ {b ^ {r} = b} \boldsymbol {P} (t) + \boldsymbol {P} (t) \frac {\partial [ \boldsymbol {F} ^ {r} (t) - \overline {{\boldsymbol {K}}} (t) \boldsymbol {H} ^ {r} (t) ] ^ {\mathrm{T}}}{\partial b _ {i} ^ {r}} \Bigg | _ {b ^ {r} = b} + \\ & \frac {\partial [ \boldsymbol {G} ^ {r} (t) \boldsymbol {Q} ^ {r} (t) \boldsymbol {G} ^ {\mathrm{T}} (t) ]}{\partial b _ {i} ^ {r}} \Bigg | _ {b ^ {r} = b} + \overline {{\boldsymbol {K}}} (t) \frac {\partial \boldsymbol {R} ^ {r} (t)}{\partial b _ {i} ^ {r}} \Bigg | _ {b ^ {r} = b} \overline {{\boldsymbol {K}}} (t) \end{array}\tag{3.10.41}
$$

下面举一简单例子说明求解灵敏度的过程。

例3-7 设飞行器作等高飞行，希望的飞行高度为 $H_0$ ，但实际的飞行高度为 $H^r$ 。用高度表监测飞行高度，高度测量值为 $M^r = h^r H^r + v^r$ ，其中 $E[v^r(t)] = 0, E[v^r(t)v^r(\tau)] = r^r\delta (t - \tau)$ 。取高度偏离值为状态，即 $X^r = H^r - H_0$ 。假定 $E[X^r(0)] = 0, E\{[X^r(0)]^2\} = C_{x_0}$ ，则真实的系统方程和量测方程为

$$
\begin{array}{l} \dot {X} ^ {r} (t) = 0 \\ Z (t) = h ^ {r} X ^ {r} (t) + v ^ {r} (t) \end{array}
$$

而在设计卡尔曼滤器时,选取的系统模型和量测模型为

$$
\begin{array}{l} \dot {X} (t) = 0 \\ Z (t) = h X (t) + v (t) \end{array}
$$

其中 $E[v(t)]=0, E[v(t)v(\tau)]=r\delta(t-\tau)$ ，且 $r \neq r^{r}, h \neq h^{r}$ 。

试对卡尔曼滤波器作关于 r、h 和 $C_{X_{0}}$ 的灵敏度分析。

解 按连续型卡尔曼滤波方程设计滤波器。根据式(2.3.21)，滤波的估计均方误差方程为

即

$$
\frac {\mathrm{d} P (t)}{\mathrm{d} t} = \frac {- h ^ {2} P ^ {2} (t)}{r}   \frac {1}{P ^ {2} (t)} \mathrm{d} P (t) = - \frac {h ^ {2}}{r} \mathrm{d} t
$$

对该式从0到 $t$ 积分，得

$$
P (t) = \frac {r P (0)}{h ^ {2} P (0) t + r}\tag{1}
$$

根据式 $(2.3.20)$ 得滤波增益为

$$
\overline {{{K}}} (t) = \frac {h P (0)}{h ^ {2} P (0) t + r}\tag{2}
$$

又由式(3.10.36)，实际的估计均方误差 $P^{P}(t)$ 方程为

$$
\dot {P} ^ {P} (t) = - \frac {2 h ^ {2} P (0)}{h ^ {2} P (0) t + r} P ^ {P} (t) - \frac {2 \Delta h h P (0)}{h ^ {2} P (0) t + r} C (t) + \left[ \frac {h P (0)}{h ^ {2} P (0) t + r} \right] ^ {2} r ^ {r}\tag{3}
$$

式中

$$
\begin{array}{r l} & \Delta h = h ^ {r} - h \\ & C (t) = E [ X ^ {r} (t) \widetilde {X} ^ {p} (t) ] \end{array}
$$

根据式(3.10.37)， $C(t)$ 满足方程

$$
\dot {C} (t) = - \frac {h ^ {2} P (0)}{h ^ {2} P (0) t + r} C (t) - \frac {h P (0) \Delta h}{h ^ {2} P (0) t + r} A (t)\tag{4}
$$

式中

$$
A (t) = E \{\left[ X ^ {r} (t) \right] ^ {2} \}
$$

又根据式(3.10.38)，A 满足方程

$$
\dot {A} (t) = 0\tag{5}
$$

因为 $E[X^{r}(0)]=0$ ，在滤波器设计中取 $E[X(0)]=0$ ，所以有

$$
\begin{array}{r l} C (0) & = E [ X ^ {r} (0) \widetilde {X} ^ {P} (0) ] = E \{X ^ {r} (0) [ X ^ {r} (0) - \hat {X} (0) ] \} = \\ & \quad E \{[ X ^ {r} (0) ] ^ {2} \} - \hat {X} (0) E [ X ^ {r} (0) ] = C _ {X} (0) \end{array}
$$

$$
A (0) = E \{\left[ X ^ {r} (0) \right] ^ {2} \} = C _ {X} (0)
$$

$$
\begin{array}{r l} P ^ {P} (0) & = E \{[ X ^ {r} (0) - \hat {X} (0) ] ^ {2} \} = E \{(X ^ {r} (0) - E [ X (0) ]) ^ {2} \} = \\ & E \{[ X ^ {r} (0) ] ^ {2} \} = C _ {X} (0) \end{array}
$$

所以由式(5)得

$$
A (t) = C _ {X} (0)
$$

将 $A(t)$ 代入式(4)，则式(4)成为一阶非齐次线性微分方程

$$
\dot {y} (t) + p (t) y (t) = q (t)
$$

其通解为

$$
y (t) = \mathrm{e} ^ {- \int p (t) \mathrm{d} t} [ \int q (t) \mathrm{e} ^ {\int p (t) \mathrm{d} t} \mathrm{d} t + a ]
$$

其中 a 为待定常数。由式(4)的具体参数, 可得

$$
C (t) = \frac {1}{h ^ {2} P (0) t + r} [ - h P (0) \Delta h C _ {X} (0) t + a ]\tag{6}
$$

当 $t = 0$ 时

$$
C (0) = C _ {X} (0) = \frac {a}{r}
$$

所以

$$
a = r C _ {X} (0)
$$

$$
C (t) = \frac {1}{h ^ {2} P (0) t + r} [ - h P (0) \Delta h C _ {X} (0) t + r C _ {X} (0) ]\tag{7}
$$

将 $C(t)$ 代入式(3)，同样求解一阶非齐次线性微分方程，得

$$
\begin{array}{r l} P ^ {P} (t) = & \frac {1}{[ h ^ {2} P (0) t + r ] ^ {2}} \{- \Delta h ^ {2} h ^ {2} P ^ {2} (0) C _ {X} (0) t ^ {2} + [ h ^ {2} P ^ {2} (0) r ^ {r} - \\ & 2 \Delta h h P (0) C _ {X} (0) r ] t + r ^ {2} C _ {X} (0) \} \end{array}\tag{8}
$$

应用式 $(3.10.39)$ 得大尺度灵敏度

$$
S _ {C _ {X} (0)} (t) = \left\{\frac {h ^ {2} P ^ {2} (0) r t + r ^ {2} C _ {X} (0)}{\left[ h ^ {2} P (0) t + r \right] ^ {2}} - \frac {r P (0)}{h ^ {2} P (0) t + r} \right\} \left[ C _ {X} (0) - P (0) \right] ^ {- 1} =
$$

$$
\frac {r ^ {2}}{\left[ h ^ {2} P (0) t + r \right] ^ {2}}\tag{9}
$$

$$
S _ {r} (t) = \left\{\frac {h ^ {2} P ^ {2} (0) r ^ {r} t + r ^ {2} P (0)}{[ h ^ {2} P (0) t + r ] ^ {2}} - \frac {r P (0)}{h ^ {2} P (0) t + r} \right\} (r ^ {r} - r) ^ {- 1} =\tag{10}
$$

$$
S _ {h} (t) = \frac {h ^ {2} \Delta h P ^ {3} (0) t ^ {2} - 2 P ^ {2} (0) r h t}{[ h ^ {2} P (0) t + r ] ^ {2}}\tag{11}
$$

应用式(3.10.41)可求得小尺度灵敏度。也可以根据定义直接从式(8)求取

$$
r _ {C _ {X} (0)} (t) = \frac {\partial P ^ {P} (t)}{\partial C _ {X} (0)} \Bigg | _ {C _ {X} (0) = P (0)} = \frac {r ^ {2}}{[ h ^ {2} P (0) t + r ] ^ {2}}\tag{12}
$$

$$
r _ {r} (t) = \frac {\partial P ^ {P} (t)}{\partial r} \Bigg | _ {r ^ {r} = r} = \frac {h ^ {2} P ^ {2} (0) t}{[ h ^ {2} P (0) t + r ] ^ {2}}\tag{13}
$$

$$
r _ {h} (t) = \frac {\partial P ^ {P} (t)}{\partial h} \Bigg | _ {h ^ {r} = h} = - \frac {2 h P ^ {2} (0) r t}{[ h ^ {2} P (0) t + r ] ^ {2}}\tag{14}
$$

式(9)～式(14)都是当 $P(0)\neq P^{P}(0),r\neq r^{r},h\neq h^{r}$ 时的灵敏度，它们反映了当 $P(0)$ ，r和h取得不准确时，实际滤波均方误差受到影响的程度。从诸表达式可看出， $S_{C_{X}(0)}(t)$ 与 $C_{X}(0)$ 无关，所以 $S_{C_{X}(0)}(t)=r_{C_{X}(0)}(t)$ 。同样 $S_{r}(t)$ 与 $r_{r}(t)$ 也相同，仅 $S_{h}(t)$ 与 $r_{h}(t)$ 不同。

在 $P(0)$ 与 $P^{P}(0)$ （即 $C_{X}(0)$ ）不同，r 与 $r^{r}$ 不同或 h 与 $h^{r}$ 不同的情况下，从两种灵敏度表达式可以看出对实际的估计均方误差的影响，也可看出模型的这些参数 $(P(0), r \text{ 与 } h)$ 与这种影响的关系。从灵敏度表达式还可看出，这种影响随时间而变化。当 t 相当大时， $P(0)$ 不同于 $C_{X}(0)$ 的影响以及 r 不同于 $r^{r}$ 的影响将逐渐消失，而 h 不同于 $h^{r}$ 的影响却始终存在。根据式（1）和式（8）（或式（11）），有

$$
\begin{array}{r l} & \lim _ {t \to \infty} P (t) = 0 \\ & \lim _ {t \to \infty} P ^ {P} (t) = \frac {(\Delta h) ^ {2} C _ {X} (0)}{h ^ {2}} \\ & \lim _ {t \to \infty} S _ {h} (t) = \frac {\Delta h P (0)}{h ^ {2}} \end{array}
$$

或

上述诸式说明:随着滤波时间的增大,计算的估计均方误差趋于零,而实际的估计均方误差却因模型的量测系数 h 有误差而并不趋于零。

方差误差分析和灵敏度分析在设计次优滤波器时，经常用来计算实际的估计均方误差或某些参数的灵敏度，以判别次优滤波器的滤波性能，了解系统参数变化对滤波性能的影响程度，从而更好地选择次优滤波器的结构和参数。必须指出，不论是实际估计均方误差的计算方程，还是灵敏度的计算方程，都是在真实系统为线性系统的前提下获得的。对非线性的真实系统，这些计算方程就不适用了。对这类系统，可以采用计算真实状态估计的方法进行误差分析，即根据真实的系统模型、一组初始状态的随机样本数据，计算出一组状态和量测的动态过程值，并根据量测值和采用的滤波器计算出估计的动态过程值并与真实状态比较，得出一组估计误差的动态过程值。按照上述方法，用不同的随机样本值，可以得出若干组估计误差的动态过程值。取其均方值，得到实际的估计均方误差。这种方法通常称作蒙特-卡洛仿真法。采用的随机样本数越多，所得到的实际的估计均方误差就越准确，但计算量也就越大。

## 3.10.3 模型正确时滤波误差上界分析

设系统和量测模型正确,即对离散系统,有

$$
\boldsymbol {\Phi} _ {k, k - 1} = \boldsymbol {\Phi} _ {k, k - 1} ^ {r}, \quad \boldsymbol {\Gamma} _ {k} = \boldsymbol {\Gamma} _ {l} ^ {r}, \quad \boldsymbol {H} _ {k} = \boldsymbol {H} _ {k} ^ {r}
$$

对连续系统，有

$$
\boldsymbol {F} (t) = \boldsymbol {F} ^ {r} (t), \quad \boldsymbol {G} (t) = \boldsymbol {G} ^ {r} (t), \quad \boldsymbol {H} (t) = \boldsymbol {H} ^ {r} (t)
$$

并设系统噪声方差阵为 $Q_{k}$ ，量测噪声方差阵为 $R_{k}, P_{0}^{P} = Var[X_{0}^{r}]$ ，则根据式（3.10.12）和式（3.10.13），反映真实估计误差和一步预测误差的均方阵为

$$
\boldsymbol {P} _ {k} ^ {P} = (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) \boldsymbol {P} _ {k / k - 1} ^ {P} (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) ^ {\mathrm{T}} + \boldsymbol {K} _ {k} \boldsymbol {R} _ {k} ^ {r} \boldsymbol {K} _ {k} ^ {\mathrm{T}}\tag{3.10.42}
$$

$$
\boldsymbol {P} _ {k / k - 1} ^ {p} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {P} _ {k - 1} ^ {P} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {Q} _ {k - 1} ^ {r} \boldsymbol {\Gamma} _ {k - 1} ^ {\mathrm{T}}\tag{3.10.43}
$$

设计卡尔曼滤波器时,系统模型和量测模型虽然选取正确,但系统噪声方差阵,量测噪声方差阵及初始均方误差阵选取有误,即

$$
\begin{array}{l} {Q _ {k} \neq Q _ {k} ^ {r}} \\ {R _ {k} \neq R _ {k} ^ {r}} \\ {P _ {0} \neq P _ {0} ^ {P} = \mathrm{Var} [ X _ {0} ^ {r} ]} \end{array}
$$

则根据式 $(2.2.4e)$ 和式 $(2.2.4c)$ ，得滤波计算中获得的均方误差阵为

$$
\boldsymbol {P} _ {k} = \left(\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}\right) \boldsymbol {P} _ {k / k - 1} \left(\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}\right) ^ {\mathrm{T}} + \boldsymbol {K} _ {k} \boldsymbol {R} _ {k} \boldsymbol {K} _ {k} ^ {\mathrm{T}}\tag{3.10.44}
$$

$$
\boldsymbol {P} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {P} _ {k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {Q} _ {k - 1} \boldsymbol {\Gamma} _ {k - 1} ^ {\mathrm{T}}\tag{3.10.45}
$$

式(3.10.44)与式(3.10.42)等式两边相减,式(3.10.45)与式(3.10.43)等式两边也相减,并令

$$
\begin{array}{r} \Delta P _ {k} = P _ {k} - P _ {k} ^ {P} \\ \Delta P _ {k / k - 1} = P _ {k / k - 1} - P _ {k / k - 1} ^ {P} \end{array}
$$

得

$$
\begin{array}{r l} \Delta P _ {k} & = (I - K _ {k} H _ {k}) \Delta P _ {k / k - 1} (I - K _ {k} H _ {k}) ^ {\mathrm{T}} + K _ {k} (R _ {k} - R _ {k} ^ {r}) K _ {k} ^ {\mathrm{T}} \\ \Delta P _ {k / k - 1} & = \Phi_ {k, k - 1} \Delta P _ {k - 1} \Phi_ {k, k - 1} ^ {\mathrm{T}} + \Gamma_ {k - 1} (Q _ {k - 1} - Q _ {k - 1} ^ {r}) \Gamma_ {k - 1} ^ {\mathrm{T}} \end{array}\tag{3.10.46}
$$

(3.10.47)

从上述两式可看出，如果滤波器设计中，选择参数时使 $Q_{k} \geqslant Q_{k}^{r}$ 和 $R_{k} \geqslant R_{k}^{r}$ ，则只要 $\Delta P_{k-1} \geqslant 0$ ，必有 $\Delta P_{k/k-1} \geqslant 0$ ，因而也有 $\Delta P_{k} \geqslant 0$ 。根据数学归纳法，可得到如下结论：

在只有 $P_0, Q_k$ 和 $R_k$ 有误差的情况下，如果选取

$$
\left. \begin{array}{l} \boldsymbol {P} _ {0} \geqslant \boldsymbol {P} _ {0} ^ {P} \\ \boldsymbol {Q} _ {k} \geqslant \boldsymbol {Q} _ {k} ^ {r} \\ \boldsymbol {R} _ {k} \geqslant \boldsymbol {R} _ {k} ^ {r} \end{array} \right\}\tag{3.10.48}
$$

则必有

$$
\boldsymbol {P} _ {k} \geqslant \boldsymbol {P} _ {k} ^ {P}
$$

反之，如果选取

$$
\begin{array}{l} \boldsymbol {P} _ {0} \leqslant \boldsymbol {P} _ {0} ^ {P} \\ \boldsymbol {Q} _ {k} \leqslant \boldsymbol {Q} _ {k} ^ {r} \\ \boldsymbol {R} _ {k} \leqslant \boldsymbol {R} _ {k} ^ {r} \end{array}
$$

则必有

$$
\boldsymbol {P} _ {k} \leqslant \boldsymbol {P} _ {k} ^ {P}
$$

因此可进一步得出如下结论：

在只有 $P_0, Q_k$ 和 $R_k$ 有误差的情况下，如果按式(3.10.48)选取滤波参数，并且系统是一致完全可控和一致完全可观测的，则 $P_k$ 有一致的有限上界。因为 $P_k^P \leqslant P_k$ ，所以 $P_k^P$ 一定有一致的有限上界。

以上结论有助于选择模型的噪声方程阵。例如， $P_{0}, Q_{k}$ 和 $R_{k}$ 若无法精确获得，而只知道它们可能的取值范围，则可以采用它们可能的较大值，也就是保守值。这样，只要滤波计算中获得的估计均方误差能满足设计要求，那么实际滤波误差的均方误差必定能满足要求。另外也可以适当选取 $Q_{k}$ 和 $R_{k}$ ，如果系统模型和量测模型满足一致完全可控和一致完全可观测，则 $P_{k}$ 必然有界，而 $P_{k}^{P} \leqslant P_{k}$ 。所以，这种保守设计在一定意义上可以防止实际的估计均方误差阵的

发散。

连续系统也有相同的结论,这里就不再介绍了。

## 3.11 $H_{\infty}$ 滤波 $^{[119]}$

设 $X_{k}$ 为被估计状态,相应的系统方程和量测方程为

$$
\boldsymbol {X} _ {k} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {X} _ {k - 1} + \boldsymbol {W} _ {k - 1}\tag{3.11.1a}
$$

$$
\mathbf {Z} _ {k} = \mathbf {H} _ {k} \mathbf {X} _ {k} + \mathbf {V} _ {k}\tag{3.11.1b}
$$

式中， $W_{k}$ 和 $V_{k}$ 为独立的零均值高斯白噪声，方差阵分别为 $Q_{k}$ 和 $R_{k}$ 。

若 $\Phi_{k,k - 1},H_k,Q_k,R_k$ 和 $X_0$ 的先验信息准确已知，则采用标准卡尔曼算法可以获得 $X_{k}$ 的最小方差估计，估计精度优于任何其它估计。如果并不确切知道 $Q_{k},R_{k}$ 和 $X_0$ 的准确先验信息，例如对系统的不确定性了解不清，则在设计卡尔曼滤波器时应适当增大 $Q_{k}$ 的取值，以增大对实时量测量的利用权重，同时降低对一步预测的利用权重，此法俗称为调谐。但是调谐存在诸多盲目性，无法确定 $Q_{k}$ 究竟增大到多少才能使估计精度达到最佳。并且如果 $\mathbf{W}_k$ 和 $\mathbf{V}_k$ 不是白噪声，或者存在偏置量，则卡尔曼滤波效果会严重恶化，甚至发散。 $H_{\infty}$ 滤波是解决上述问题的有效算法。对 $H_{\infty}$ 滤波的研究始于20世纪80年代[131-133]，MikeGrimble在1988年提出了根据噪声与估计误差之比的上界来设计状态估计器频率响应的方法，最先提出了 $H_{\infty}$ 滤波[134]，但其研究是在频域内进行的，存在诸多设计上的不便。之后的研究都是在时域内进行的，简化了研究工作，并与卡尔曼滤波建立起了内在联系[135-138]。

$H_{\infty}$ 滤波的设计思路是: 在 $Q_{k}, R_{k}$ 和 $P_{0}$ 未知的情况下, 将 $W_{k}, V_{k}, X_{0}$ 的不确定性对估计精度的影响降低到最低程度, 使滤波器在最恶劣的条件下估计误差达到最小。所以 $H_{\infty}$ 滤波可理解成系统存在严重干扰条件下的最优滤波, 滤波鲁棒性是其最显著特点。下面, 根据这一思路详细介绍 $H_{\infty}$ 滤波算法。

## 3.11.1 约束条件下的极值求解

## 1. 静态约束条件下的极值求解

设有某标量函数 $J(X,W)$ ，其中 X 为 n 维向量，W 为 m 维向量， $f(X,W)=0$ 为 n 维向量方程，要求确定出在 $f(X,W)=0$ 约束条件下， $J(X,W)$ 的极值以及对应于该极值的 $X=X^{*}$ 和 $W=W^{*}$ ，习惯上将 $X^{*}$ 和 $W^{*}$ 称为 $J(X,W)$ 的驻点。解决约束条件下极值问题的有效方法是拉格朗日乘数法。具体方法简述如下。

构造增广代价函数

$$
J _ {a} = J + \lambda^ {T} f\tag{3.11.2}
$$

式中， $\lambda$ 为拉格朗日乘数，为 n 维待定向量。

由(3.11.2)式得

$$
\frac {\partial J _ {a}}{\partial \boldsymbol {X} ^ {\mathrm{T}}} = \frac {\partial J}{\partial \boldsymbol {X} ^ {\mathrm{T}}} + \boldsymbol {\lambda} ^ {\mathrm{T}} \frac {\partial f}{\partial \boldsymbol {X} ^ {\mathrm{T}}}\tag{3.11.3a}
$$

$$
\frac {\partial J _ {a}}{\partial \boldsymbol {W} ^ {\mathrm{T}}} = \frac {\partial J}{\partial \boldsymbol {W} ^ {\mathrm{T}}} + \boldsymbol {\lambda} ^ {\mathrm{T}} \frac {\partial \boldsymbol {f}}{\partial \boldsymbol {W} ^ {\mathrm{T}}}\tag{3.11.3b}
$$

$$
\frac {\partial J _ {a}}{\partial \lambda} = f\tag{3.11.3c}
$$

为求解 $J_{a}$ 的驻点，令上述诸式等于零，

$$
\boldsymbol {\lambda} ^ {\mathrm{T}} = - \frac {\partial J}{\partial \boldsymbol {X} ^ {\mathrm{T}}} \left(\frac {\partial \boldsymbol {f}}{\partial \boldsymbol {X} ^ {\mathrm{T}}}\right) ^ {- 1}\tag{3.11.4a}
$$

$$
\frac {\partial J}{\partial \boldsymbol {W} ^ {\mathrm{T}}} - \frac {\partial J}{\partial \boldsymbol {X} ^ {\mathrm{T}}} \left(\frac {\partial \boldsymbol {f}}{\partial \boldsymbol {X} ^ {\mathrm{T}}}\right) ^ {- 1} \frac {\partial \boldsymbol {f}}{\partial \boldsymbol {W} ^ {\mathrm{T}}} = \mathbf {0}\tag{3.11.4b}
$$

$$
f = 0\tag{3.11.4c}
$$

式(3.11.4)即为驻点应满足的必要条件,该式共有 $2n+m$ 个分量方程,联立求解这些方程,可以确定出驻点 $X^{*},W^{*}$ 和拉格朗日乘数 $\lambda$ ,代入(3.11.2)式可确定出 $J_{a}$ 的极值。

式(3.11.4)说明,拉格朗日乘数法将约束条件下的极值求解转化为无约束极值求解,约束方程在式(3.11.3)中被用作求取驻点的已知条件。

例3-8 确定出在 $x - 3 = 0$ 约束条件下 $J(x,u) = \frac{x^2}{2} + xu + u^2 + u$ 的极小值。

解 可采用直观分析法确定该条件极值,也可采用拉格朗日乘数法。

直观分析法

由约束方程得 $x = 3$ ，则 $J(x,u) = \frac{9}{2} +4u + u^2$ ，当 $u = -2$ 时， $J(x,u)$ 达到最小值 $\frac{1}{2}$ ，即 $J_{\mathrm{min}} = \frac{1}{2}$ 。

拉格朗日乘数法

$$
J _ {a} = J + \lambda f = \frac {x ^ {2}}{2} + x u + u ^ {2} + u + \lambda (x - 3)
$$

$$
\frac {\partial J _ {a}}{\partial x} = x + u + \lambda = 0
$$

$$
\frac {\partial J _ {a}}{\partial u} = x + 2 u + 1 = 0
$$

$$
\frac {\partial J _ {a}}{\partial \lambda} = x - 3 = 0
$$

联立求解上述三式，得 $x = 3, u = -2, \lambda = -1, J_{\min} = \frac{1}{2}$ 。

两种方法结果相同。

2. 动态约束条件下的极值求解

设有时变系统

$$
\boldsymbol {X} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {X} _ {k} + \boldsymbol {W} _ {k} \quad k = 0, 1, \dots , N - 1
$$

要求在该 N 个约束方程条件下, 确定下述标量代价函数的极值

$$
J = \psi (\mathbf {X} _ {0}) + \sum_ {k = 0} ^ {N - 1} L _ {k}\tag{3.11.5}
$$

其中 $\psi$ 为 $X_{0}$ 的已知函数， $L_{k}$ 为 $X_{k}$ 和 $W_{k}$ 的已知函数。

构造增广代价函数

$$
J _ {a} = J + \sum_ {k = 0} ^ {N - 1} \boldsymbol {\lambda} _ {k + 1} ^ {\mathrm{T}} (\boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {X} _ {k} + \boldsymbol {W} _ {k} - \boldsymbol {X} _ {k + 1})\tag{3.11.6}
$$

式 $(3.11.5)$ 代入上式,得:

$$
\begin{array}{r l} J _ {a} & = \psi (\mathbf {X} _ {0}) + \sum_ {k = 0} ^ {N - 1} [ L _ {k} + \boldsymbol {\lambda} _ {k + 1} ^ {\mathrm{T}} (\boldsymbol {\Phi} _ {k + 1, k} \mathbf {X} _ {k} + \mathbf {W} _ {k} - \mathbf {X} _ {k + 1}) ] = \\ & \psi (\mathbf {X} _ {0}) + \sum_ {k = 0} ^ {N - 1} [ L _ {k} + \boldsymbol {\lambda} _ {k + 1} ^ {\mathrm{T}} (\boldsymbol {\Phi} _ {k + 1, k} \mathbf {X} _ {k} + \mathbf {W} _ {k}) ] - \sum_ {k = 0} ^ {N - 1} \boldsymbol {\lambda} _ {k + 1} ^ {\mathrm{T}} \mathbf {X} _ {k + 1} = \\ & \psi (\mathbf {X} _ {0}) + \sum_ {k = 0} ^ {N - 1} \mathcal {H} _ {k} - \sum_ {k = 0} ^ {N} \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \mathbf {X} _ {k} + \boldsymbol {\lambda} _ {0} ^ {\mathrm{T}} \mathbf {X} _ {0} = \\ & \psi (\mathbf {X} _ {0}) + \sum_ {k = 0} ^ {N - 1} (\mathcal {H} _ {k} - \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \mathbf {X} _ {k}) - \boldsymbol {\lambda} _ {N} ^ {\mathrm{T}} \mathbf {X} _ {N} + \boldsymbol {\lambda} _ {0} ^ {\mathrm{T}} \mathbf {X} _ {0} \end{array}\tag{3.11.7}
$$

其中

$$
\mathcal {H} _ {k} = L _ {k} + \lambda_ {k + 1} ^ {\mathrm{T}} (\boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {X} _ {k} + \boldsymbol {W} _ {k})\tag{3.11.8}
$$

驻点必须满足的方程为

$$
\frac {\partial J _ {u}}{\partial \mathbf {X} _ {k} ^ {\mathrm{T}}} = \mathbf {0} \quad k = 0, 1, \dots , N\tag{3.11.9a}
$$

$$
\frac {\partial J _ {a}}{\partial \boldsymbol {W} _ {k} ^ {\mathrm{T}}} = \mathbf {0} \quad k = 0, 1, \dots , N - 1\tag{3.11.9b}
$$

$$
\frac {\partial J _ {a}}{\partial \lambda_ {k}} = 0 \quad k = 0, 1, \dots , N\tag{3.11.9c}
$$

由式(3.11.7)和式(3.11.8)，式(3.11.9a)和式(3.11.9b)可写成

$$
\lambda_ {0} ^ {\mathrm{T}} + \frac {\partial \psi (\mathbf {X} _ {0})}{\partial \mathbf {X} _ {0} ^ {\mathrm{T}}} = \mathbf {0}\tag{3.11.10a}
$$

$$
- \lambda_ {N} ^ {\mathrm{T}} = 0\tag{3.11.10b}
$$

$$
\frac {\partial \mathcal {H} _ {k}}{\partial \mathbf {X} _ {k} ^ {\mathrm{T}}} = \pmb {\lambda} _ {k} ^ {\mathrm{T}} \quad k = 1, 2, \dots , N - 1\tag{3.11.10c}
$$

$$
\frac {\partial \mathcal {H} _ {k}}{\partial \pmb {W} _ {k} ^ {\mathrm{T}}} = \pmb {0} \quad k = 0, 1, 2, \dots , N - 1\tag{3.11.10d}
$$

式 $(3.11.9c)$ 即为由不同时刻的系统方程确定的约束方程,分析如下:

由式 $(3.11.7)$ ，

$$
\frac {\partial J _ {a}}{\partial \lambda_ {N}} = - X _ {N} + \frac {\partial \mathcal {H} _ {N - 1}}{\partial \lambda_ {N}} = - X _ {N} + \Phi_ {N, N - 1} X _ {N - 1} + W _ {N - 1} = 0
$$

此即 $t_{N}$ 时刻的系统方程所确定的约束方程。

当 $k=1,2,\cdots,N-1$ 时，

$$
\frac {\partial J _ {a}}{\partial \lambda_ {k}} = - X _ {k} + \frac {\partial \mathcal {H} _ {k - 1}}{\partial \lambda_ {k}} = - X _ {k} + \Phi_ {k, k - 1} X _ {k - 1} + W _ {k - 1} = 0
$$

此即 $k=1,2,\cdots,N-1$ 时刻的系统方程确定的约束方程。

说明：由式(3.11.7)知， $\lambda_0^{\mathrm{T}}X_0$ 实质上并不存在，无论 $\lambda_0$ 任意取值，在该式中， $-\lambda_0^{\mathrm{T}}X_0$ 和 $+\lambda_0^{\mathrm{T}}X_0$ 总互相抵消。所以，只要 $\lambda_0$ 的取值能使式(3.11.7)右侧第二项对 $X_0$ 的偏导数为零，即可确保式(3.11.10a)成立。3.11.2节对此作了验证。

## 3.11.2 $H_{\infty}$ 滤波中的极小值

1. $H_{\infty}$ 滤波的代价函数

设系统方程和量测方程为

$$
\begin{array}{r} \mathbf {X} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} \mathbf {X} _ {k} + \mathbf {W} _ {k} \\ \mathbf {Z} _ {k} = \mathbf {H} _ {k} \mathbf {X} _ {k} + \mathbf {V} _ {k} \end{array}
$$

式中， $W_{k}$ 和 $V_{k}$ 的特性未知，它们有可能是随机噪声，但均值不为零，也有可能是确定性干扰。在这种情况下，卡尔曼滤波是无法解决估计问题的。

假设需要估计的量是状态变量的某种线性组合：

$$
\mathbf {y} _ {k} = \mathbf {T} _ {k} \mathbf {X} _ {k}
$$

其中 $T_{k}$ 满秩。

实现 $H_{\infty}$ 滤波的代价函数为

$$
J _ {1} = \frac {\sum_ {k = 0} ^ {N - 1} \| \mathbf {y} _ {k} - \hat {\mathbf {y}} _ {k} \| _ {\mathbf {s} _ {k}} ^ {2}}{\| \mathbf {X} _ {0} - \hat {\mathbf {X}} _ {0} \| _ {\mathbf {P} _ {0} ^ {- 1}} ^ {2} + \sum_ {k = 0} ^ {N - 1} (\| \mathbf {W} _ {k} \| _ {\mathbf {Q} _ {k} ^ {- 1}} ^ {2} + \| \mathbf {V} _ {k} \| _ {\mathbf {R} _ {k} ^ {- 1}} ^ {2})}\tag{3.11.11}
$$

式中， $\| \mathbf{y}_k - \hat{\mathbf{y}}_k \|_{S_k}^2 = (\mathbf{y}_k - \hat{\mathbf{y}}_k)^{\mathrm{T}} S_k (\mathbf{y}_k - \hat{\mathbf{y}}_k)$ ，即向量 $\mathbf{y}_k - \hat{\mathbf{y}}_k$ 的加权内积， $S_k$ 为该向量的协方差阵，其余向量的加权内积说明同上。

假设代价函数的上界为 $\frac{1}{\theta}$ , 即

$$
J _ {1} <   \frac {1}{\theta}\tag{3.11.12}
$$

式中 $\theta$ 为 $H_{\infty}$ 滤波器的性能上界参数，由设计者选定。

由式(3.11.12)可重新定义代价函数

$$
J = - \frac {1}{\theta} \| \boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0} \| _ {\boldsymbol {P} _ {0} ^ {- 1}} ^ {2} + \sum_ {k = 0} ^ {N - 1} \left[ \| \boldsymbol {y} _ {k} - \hat {\boldsymbol {y}} _ {k} \| _ {\boldsymbol {s} _ {k}} ^ {2} - \frac {1}{\theta} \left(\| \boldsymbol {W} _ {k} \| _ {\boldsymbol {Q} _ {k} ^ {- 1}} ^ {2} + \| \boldsymbol {V} _ {k} \| _ {\boldsymbol {R} _ {k} ^ {- 1}} ^ {2}\right) \right] <   1\tag{3.11.13}
$$

$H_{\infty}$ 滤波所要满足的指标是: 在系统方程 $X_{k+1} = \Phi_{k+1,k} X_k + W_k (k = 0, 1, 2 \cdots, N-1)$ 所确定的约束条件下, 所确定的估计值 $\hat{y}_k$ 使 J 达到最小。(3.11.13) 式中, 分母上各向量的加权内积所用的加权阵分别为 $P_0, Q_k, R_k$ 的逆, 如果选择滤波初值时的误差很大, 系统的不确定性非常严重, 量测误差很大, 则这些阵的逆会很小, 相应的向量加权内积也会很小, 结果是使 J 增大。 $H_{\infty}$ 滤波的设计思路是在 $P_0, Q_k, R_k$ 达到上界时, 寻求相应对策, 使 J 达到最小, 即在干扰极大化背景下的估计误差最小化

$$
J ^ {*} = \min _ {\hat {y} _ {k}} \max _ {(W _ {k}, V _ {k}, X _ {0})} J
$$

由于 $\hat{\mathbf{y}}_k = \mathbf{T}_k\hat{\mathbf{X}}_k,\mathbf{V}_k = \mathbf{Z}_k - \mathbf{H}_k\mathbf{X}_k$ ，所以

$$
\left\| \mathbf {V} _ {k} \right\| _ {\mathbf {R} _ {k} ^ {- 1}} ^ {2} = \left\| \mathbf {Z} _ {k} - \mathbf {H} _ {k} \mathbf {X} _ {k} \right\| _ {\mathbf {R} _ {k} ^ {- 1}} ^ {2}\tag{3.11.14}
$$

$$
\left\| \mathbf {y} _ {k} - \hat {\mathbf {y}} _ {k} \right\| _ {S _ {k}} ^ {2} = \left(\mathbf {X} _ {k} - \hat {\mathbf {X}} _ {k}\right) ^ {\mathrm{T}} \mathbf {T} _ {k} ^ {\mathrm{T}} \mathbf {S} _ {k} \mathbf {T} _ {k} \left(\mathbf {X} _ {k} - \hat {\mathbf {X}} _ {k}\right) = \left\| \mathbf {X} _ {k} - \hat {\mathbf {X}} _ {k} \right\| _ {S _ {k}} ^ {2}\tag{3.11.15}
$$

其中

$$
\widetilde {\boldsymbol {S}} _ {k} = \boldsymbol {T} _ {k} ^ {\mathrm{T}} \boldsymbol {S} _ {k} \boldsymbol {T} _ {k}\tag{3.11.16}
$$

式(3.11.14)、式(3.11.15)代入式(3.11.13)，得

$$
J = - \frac {1}{\theta} \| \boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0} \| _ {\boldsymbol {P} _ {0} ^ {- 1}} ^ {2} + \sum_ {k = 0} ^ {N - 1} \left[ \| \boldsymbol {X} _ {k} - \hat {\boldsymbol {X}} _ {k} \| _ {\dot {\boldsymbol {S}} _ {k}} ^ {2} - \frac {1}{\theta} \left(\| \boldsymbol {W} _ {k} \| _ {\boldsymbol {Q} _ {k} ^ {- 1}} ^ {2} + \| \boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {X} _ {k} \| _ {\boldsymbol {R} _ {k} ^ {- 1}} ^ {2}\right) \right] =
$$

$$
\psi (\pmb {X} _ {0}) + \sum_ {k = 0} ^ {N - 1} L _ {k}\tag{3.11.17}
$$

式中

$$
\psi (\pmb {X} _ {0}) = - \frac {1}{\theta} \| \pmb {X} _ {0} - \hat {\pmb {X}} _ {0} \| _ {\pmb {P} _ {0} ^ {- 1}} ^ {2}\tag{3.11.18}
$$

$$
L _ {k} = \left\| \mathbf {X} _ {k} - \hat {\mathbf {X}} _ {k} \right\| _ {S _ {k}} ^ {2} - \frac {1}{\theta} (\left\| \mathbf {W} _ {k} \right\| _ {Q _ {k} ^ {- 1}} ^ {2} + \left\| \mathbf {Z} _ {k} - \mathbf {H} _ {k} \mathbf {X} _ {k} \right\| _ {\mathbf {R} _ {k} ^ {- 1}} ^ {2})\tag{3.11.19}
$$

构造增广代价函数

$$
J _ {a} = J + \sum_ {k = 0} ^ {N - 1} \frac {2 \lambda_ {k + 1} ^ {\mathrm{T}}}{\theta} (\boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {X} _ {k} + \boldsymbol {W} _ {k} - \boldsymbol {X} _ {k + 1})\tag{3.11.20}
$$

式中， $\frac{2\lambda_{k+1}^{T}}{\theta}$ 为拉格朗日乘数，与(3.11.6)式相比，仅差比例系数 $\frac{2}{\theta}$ ，并不影响极值分析。

式 $(3.11.17)$ 代入式 $(3.11.20)$

$$
J _ {a} = \psi (\mathbf {X} _ {0}) + \sum_ {k = 0} ^ {N - 1} L _ {k} + \sum_ {k = 0} ^ {N - 1} \frac {2 \pmb {\lambda} _ {k + 1} ^ {\mathrm{T}}}{\theta} (\pmb {\Phi} _ {k + 1, k} \mathbf {X} _ {k} + \mathbf {W} _ {k} - \mathbf {X} _ {k + 1}) =
$$

$$
\psi \left(\boldsymbol {X} _ {0}\right) + \sum_ {k = 0} ^ {N - 1} \left[ L _ {k} + \frac {2 \boldsymbol {\lambda} _ {k + 1} ^ {\mathrm{T}}}{\theta} \left(\boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {X} _ {k} + \boldsymbol {W} _ {k}\right) \right] - \sum_ {k = 0} ^ {N} \frac {2 \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}}}{\theta} \boldsymbol {X} _ {k} + \frac {2 \boldsymbol {\lambda} _ {0} ^ {\mathrm{T}}}{\theta} \boldsymbol {X} _ {0} =
$$

$$
\psi (\mathbf {X} _ {0}) + \sum_ {k = 0} ^ {N - 1} \left(\mathcal {H} _ {k} - \frac {2 \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}}}{\theta} \mathbf {X} _ {k}\right) - \frac {2 \boldsymbol {\lambda} _ {N} ^ {\mathrm{T}}}{\theta} \mathbf {X} _ {N} + \frac {2 \boldsymbol {\lambda} _ {0} ^ {\mathrm{T}}}{\theta} \mathbf {X} _ {0}\tag{3.11.21}
$$

式中

$$
\mathcal {H} _ {k} = L _ {k} + \frac {2 \pmb {\lambda} _ {k + 1} ^ {\mathrm{T}}}{\theta} (\pmb {\Phi} _ {k + 1, k} \pmb {X} _ {k} + \pmb {W} _ {k})\tag{3.11.22}
$$

2. 求解代价函数由 $X_0$ 和 $W_k$ 确定的驻点

根据式 $(3.11.21)$ ，驻点必须满足的方程为

$$
\frac {2 \pmb {\lambda} _ {0} ^ {\mathrm{T}}}{\theta} + \frac {\partial \psi (\pmb {X} _ {0})}{\partial \pmb {X} _ {0} ^ {\mathrm{T}}} = \pmb {0}\tag{3.11.23a}
$$

$$
\frac {\boldsymbol {\lambda} _ {N} ^ {\mathrm{T}}}{\theta} = \mathbf {0}\tag{3.11.23b}
$$

$$
\frac {\partial \mathcal {H} _ {k}}{\partial W _ {k} ^ {\mathrm{T}}} = 0 \quad k = 0, 1, 2, \dots , N - 1\tag{3.11.23c}
$$

$$
\frac {\partial \mathcal {H} _ {k}}{\partial \mathbf {X} _ {k} ^ {\mathrm{T}}} = \frac {2 \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}}}{\theta} k = 0, 1, 2, \dots , N - 1\tag{3.11.23d}
$$

为分析方便，先计算 $\frac{\partial\parallel\mathbf{X}_0 - \hat{\mathbf{X}}_0\parallel^2\mathbf{P}_0^{-1}}{\partial\mathbf{X}_0^{\mathrm{T}}}$ 。记

$$
\boldsymbol {C} = (\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}) ^ {\mathrm{T}}, \quad \boldsymbol {D} = \boldsymbol {P} _ {0} ^ {- 1} (\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0})
$$

则

$$
\frac {\partial \parallel \boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0} \parallel_ {\boldsymbol {P} _ {0} ^ {- 1}} ^ {2}}{\partial \boldsymbol {X} _ {0} ^ {\mathrm{T}}} = \frac {\partial (\boldsymbol {C D})}{\partial \boldsymbol {X} _ {0} ^ {\mathrm{T}}}
$$

根据附录 A 中式(A-7)

$$
\frac {\partial (\boldsymbol {C D})}{\partial \boldsymbol {X} _ {0} ^ {\mathrm{T}}} = \frac {\partial \boldsymbol {C}}{\partial \boldsymbol {X} _ {0} ^ {\mathrm{T}}} (\boldsymbol {I} _ {q \times q} \otimes \boldsymbol {D}) + (\boldsymbol {I} _ {p \times p} \otimes \boldsymbol {C}) \frac {\partial \boldsymbol {D}}{\partial \boldsymbol {X} ^ {\mathrm{T}}}
$$

式中， $\otimes$ 为克朗尼克积，p,q分别为 $X_{0}^{T}$ 的行数和列数，即p=1,q=n。

$$
\frac {\partial \mathbf {C}}{\partial \mathbf {X} _ {0} ^ {\mathrm{T}}} (\mathbf {I} _ {n \times n} \otimes \mathbf {D}) = \frac {\partial \mathbf {X} _ {0} ^ {\mathrm{T}}}{\partial \mathbf {X} _ {0} ^ {\mathrm{T}}} (\mathbf {I} _ {n \times n} \otimes \mathbf {D}) =
$$

$$
\left[ \begin{array}{l l l l} \boldsymbol {e} _ {1} ^ {\mathrm{T}} & \boldsymbol {e} _ {2} ^ {\mathrm{T}} & \dots & \boldsymbol {e} _ {n} ^ {\mathrm{T}} \end{array} \right] \left[ \begin{array}{c c c} \boldsymbol {D} & & \boldsymbol {0} \\ & \ddots & \\ \boldsymbol {0} & & \boldsymbol {D} \end{array} \right] = \left[ \begin{array}{l l l l} \boldsymbol {e} _ {1} ^ {\mathrm{T}} \boldsymbol {D} & \boldsymbol {e} _ {2} ^ {\mathrm{T}} \boldsymbol {D} & \dots & \boldsymbol {e} _ {n} ^ {\mathrm{T}} \boldsymbol {D} \end{array} \right] =
$$

$$
\left[ \boldsymbol {e} _ {1} ^ {\mathrm{T}} \boldsymbol {P} _ {0} ^ {- 1} \left(\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}\right) \quad \boldsymbol {e} _ {2} ^ {\mathrm{T}} \boldsymbol {P} _ {0} ^ {- 1} \left(\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}\right) \quad \dots \quad \boldsymbol {e} _ {n} ^ {\mathrm{T}} \boldsymbol {P} _ {0} ^ {- 1} \left(\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}\right) \right]
$$

式中， $e_{i}$ 为第 i 行元为 1 的单位向量。

记 $\boldsymbol{P}_{0}^{-1}(\boldsymbol{X}_{0}-\hat{\boldsymbol{X}}_{0})=\left[p_{1}\quad p_{2}\quad\cdots\quad p_{n}\right]^{\mathrm{T}}$ ，并注意到 $P_{0}^{-1}$ 为对称阵，则

$$
\boldsymbol {e} _ {i} ^ {\mathrm{T}} \boldsymbol {P} _ {0} ^ {- 1} (\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}) = p _ {i}
$$

$$
\frac {\partial \boldsymbol {C}}{\partial \boldsymbol {X} _ {0} ^ {\mathrm{T}}} (\boldsymbol {I} _ {n \times n} \otimes \boldsymbol {D}) = [ p _ {1} \quad p _ {2} \quad \dots \quad p _ {n} ] = [ \boldsymbol {P} _ {0} ^ {- 1} (\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}) ] ^ {\mathrm{T}} = (\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}) ^ {\mathrm{T}} \boldsymbol {P} _ {0} ^ {- 1}
$$

$$
\left(\boldsymbol {I} _ {p \times p} \otimes \boldsymbol {C}\right) \frac {\partial \boldsymbol {D}}{\partial \boldsymbol {X} _ {0} ^ {\mathrm{T}}} = \left(\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}\right) ^ {\mathrm{T}} \frac {\partial \left[ \boldsymbol {P} _ {0} ^ {- 1} \left(\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}\right) \right]}{\partial \boldsymbol {X} _ {0} ^ {\mathrm{T}}} = \left(\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}\right) ^ {\mathrm{T}} \boldsymbol {P} _ {0} ^ {- 1}
$$

所以

$$
\frac {\partial \parallel \boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0} \parallel_ {\boldsymbol {P} _ {0} ^ {- 1}} ^ {2}}{\partial \boldsymbol {X} _ {0} ^ {\mathrm{T}}} = 2 (\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}) ^ {\mathrm{T}} \boldsymbol {P} _ {0} ^ {- 1}
$$

按同样的分析方法可得其余加权内积的偏微分。

式(3.11.18)代入式(3.11.23a)，并用上述所推关系代入，得

$$
\frac {2 \pmb {\lambda} _ {0} ^ {\mathrm{T}}}{\theta} - \frac {2}{\theta} (\pmb {X} _ {0} - \hat {\pmb {X}} _ {0}) ^ {\mathrm{T}} \pmb {P} _ {0} ^ {- 1} = \pmb {0}
$$

即

$$
\frac {2 \pmb {\lambda} _ {0}}{\theta} - \frac {2}{\theta} \pmb {P} _ {0} ^ {- 1} (\pmb {X} _ {0} - \hat {\pmb {X}} _ {0}) = \pmb {0}
$$

所以

$$
\boldsymbol {X} _ {0} = \hat {\boldsymbol {X}} _ {0} + \boldsymbol {P} _ {0} \lambda_ {0}\tag{3.11.24}
$$

由式(3.11.23b)，得

$$
\lambda_ {N} = 0\tag{3.11.25}
$$

由式(3.11.19)、式(3.11.22)和式(3.11.23c)，得

$$
\frac {2}{\theta} \boldsymbol {\lambda} _ {k + 1} ^ {\mathrm{T}} - \frac {2}{\theta} \boldsymbol {W} _ {k} ^ {\mathrm{T}} \boldsymbol {Q} _ {k} ^ {- 1} = \mathbf {0}
$$

即

$$
\frac {2}{\theta} \lambda_ {k + 1} - \frac {2}{\theta} Q _ {k} ^ {- 1} W _ {k} = 0
$$

所以

$$
W _ {k} = Q _ {k} \lambda_ {k + 1}\tag{3.11.26}
$$

系统方程可写成

$$
\boldsymbol {X} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {X} _ {k} + Q _ {k} \lambda_ {k + 1}\tag{3.11.27}
$$

由式 $(3.11.23d)$ 得

$$
\frac {2}{\theta} \boldsymbol {\lambda} _ {k + 1} ^ {\mathrm{T}} \boldsymbol {\Phi} _ {k + 1, k} + 2 (\boldsymbol {X} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \bar {\boldsymbol {S}} _ {k} - \frac {2}{\theta} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {X} _ {k}) ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} (- \boldsymbol {H} _ {k}) = \frac {2 \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}}}{\theta}
$$

即

$$
\begin{array}{r l} \frac {\boldsymbol {\lambda} _ {k}}{\theta} & = \frac {1}{\theta} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} \boldsymbol {\lambda} _ {k + 1} + \tilde {\boldsymbol {S}} _ {k} (\boldsymbol {X} _ {k} - \hat {\boldsymbol {X}} _ {k}) + \frac {1}{\theta} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {X} _ {k}) \\ \boldsymbol {\lambda} _ {k} & = \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} \boldsymbol {\lambda} _ {k + 1} + \theta \tilde {\boldsymbol {S}} _ {k} (\boldsymbol {X} _ {k} - \hat {\boldsymbol {X}} _ {k}) + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {X} _ {k}) \end{array}\tag{3.11.28}
$$

为了验证式(3.11.10a)和式(3.11.23a)的正确性,考察式(3.11.21)第二项对 $X_{0}$ 的偏导数:

$$
\frac {\partial}{\partial \mathbf {X} _ {0} ^ {\mathrm{T}}} (\mathcal {H} _ {0} - 2 \frac {\boldsymbol {\lambda} _ {0} ^ {\mathrm{T}}}{\theta} \mathbf {X} _ {0}) = \frac {\partial L _ {0}}{\partial \mathbf {X} _ {0} ^ {\mathrm{T}}} + \frac {2 \boldsymbol {\lambda} _ {1} ^ {\mathrm{T}}}{\theta} \boldsymbol {\Phi} _ {1, 0} - \frac {2 \boldsymbol {\lambda} _ {0} ^ {\mathrm{T}}}{\theta} =
$$

$$
2 \left(\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}\right) ^ {\mathrm{T}} \tilde {\boldsymbol {S}} _ {0} - \frac {1}{\theta} \cdot 2 \left(\boldsymbol {Z} _ {0} - \boldsymbol {H} _ {0} \boldsymbol {X} _ {0}\right) ^ {\mathrm{T}} \boldsymbol {R} _ {0} ^ {- 1} (- \boldsymbol {H} _ {0}) + \frac {2 \boldsymbol {\lambda} _ {1} ^ {\mathrm{T}}}{\theta} \boldsymbol {\Phi} _ {1, 0} - \frac {2 \boldsymbol {\lambda} _ {0} ^ {\mathrm{T}}}{\theta} =
$$

$$
\frac {2}{\theta} \left\{\left[ \boldsymbol {\Phi} _ {1, 0} ^ {\mathrm{T}} \boldsymbol {\lambda} _ {1} + \theta \tilde {\boldsymbol {S}} _ {0} (\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}) + \boldsymbol {H} _ {0} ^ {\mathrm{T}} \boldsymbol {R} _ {0} ^ {- 1} (\boldsymbol {Z} _ {0} - \boldsymbol {H} _ {0} \boldsymbol {X} _ {0}) \right] - \boldsymbol {\lambda} _ {0} \right\} ^ {\mathrm{T}}
$$

而由式(3.11.28)，当 $k = 0$ 时，有：

$$
\boldsymbol {\lambda} _ {0} = \boldsymbol {\Phi} _ {1, 0} ^ {\mathrm{T}} \boldsymbol {\lambda} _ {1} + \theta \tilde {\boldsymbol {S}} _ {0} (\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}) + \boldsymbol {H} _ {0} ^ {\mathrm{T}} \boldsymbol {R} _ {0} ^ {- 1} (\boldsymbol {Z} _ {0} - \boldsymbol {H} _ {0} \boldsymbol {X} _ {0})
$$

该式代入上式，得：

$$
\frac {\partial}{\partial \mathbf {X} _ {0} ^ {\mathrm{T}}} (\mathcal {H} _ {0} - \frac {2 \boldsymbol {\lambda} _ {0} ^ {\mathrm{T}}}{\theta} \mathbf {X} _ {0}) = \mathbf {0}
$$

所以式 $(3.11.23a)$ ，即式 $(3.11.10a)$ 成立。

式(3.11.24)为 $t_{0}$ 时刻真实状态与 $\mu_{0}=\hat{X}_{0}$ 间的关系,为便于推导,假设 $t_{k}$ 时刻也存在此关系

$$
\boldsymbol {X} _ {k} = \boldsymbol {\mu} _ {k} + \boldsymbol {P} _ {k} \lambda_ {k}\tag{3.11.29}
$$

式中 $\mu_{k}$ 和 $P_{k}$ 待定。

式 $(3.11.29)$ 分别代入式 $(3.11.27)$ 和式 $(3.11.28)$ ，得

$$
\begin{array}{r l} \boldsymbol {\mu} _ {k + 1} + \boldsymbol {P} _ {k + 1} \lambda_ {k + 1} & = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {\mu} _ {k} + \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k} \lambda_ {k} + Q _ {k} \lambda_ {k + 1} \\ \boldsymbol {\lambda} _ {k} & = \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} \boldsymbol {\lambda} _ {k + 1} + \theta \tilde {\boldsymbol {S}} _ {k} (\boldsymbol {\mu} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - \hat {\boldsymbol {X}} _ {k}) + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} [ \boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} (\boldsymbol {\mu} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k}) ] \end{array}\tag{3.11.30}
$$

(3.11.31)

从式(3.11.31)可解得

$$
\boldsymbol {\lambda} _ {k} = \left(\boldsymbol {I} - \theta \tilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k}\right) ^ {- 1} \left[ \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} \boldsymbol {\lambda} _ {k + 1} + \theta \tilde {\boldsymbol {S}} _ {k} \left(\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}\right) + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \left(\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}\right) \right]\tag{3.11.32}
$$

式 $(3.11.32)$ 代入式 $(3.11.30)$ ，得

$$
\begin{array}{r l} \boldsymbol {\mu} _ {k + 1} + \boldsymbol {P} _ {k + 1} \boldsymbol {\lambda} _ {k + 1} & = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {\mu} _ {k} + \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k} (\boldsymbol {I} - \theta \tilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k}) ^ {- 1} [ \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} \boldsymbol {\lambda} _ {k + 1} + \\ & \quad \theta \tilde {\boldsymbol {S}} _ {k} (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) ] + \boldsymbol {Q} _ {k} \boldsymbol {\lambda} _ {k + 1} \end{array}
$$

即

$$
\begin{array}{r l} \boldsymbol {\mu} _ {k + 1} - \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {\mu} _ {k} - \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k} (\boldsymbol {I} - \theta \tilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k}) ^ {- 1} [ \theta \tilde {\boldsymbol {S}} _ {k} (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) + \\ & \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) ] = \\ & [ - \boldsymbol {P} _ {k + 1} + \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k} (\boldsymbol {I} - \theta \tilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \left. \boldsymbol {P} _ {k}\right) ^ {- 1} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + \boldsymbol {Q} _ {k}) ] \lambda_ {k + 1} \end{array}\tag{3.11.33}
$$

式 $(3.11.33)$ 左右两边均为零时等式仍然成立，左边为零时，有

$$
\boldsymbol {\mu} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {\mu} _ {k} + \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k} (\boldsymbol {I} - \theta \tilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k}) ^ {- 1} [ \theta \tilde {\boldsymbol {S}} _ {k} (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) ]\tag{3.11.34}
$$

其中

$$
\boldsymbol {\mu} _ {0} = \hat {\boldsymbol {X}} _ {0}\tag{3.11.35}
$$

右边为零时，有

$$
\boldsymbol {P} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k} (\boldsymbol {I} - \theta \tilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k}) ^ {- 1} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + \boldsymbol {Q} _ {k}\tag{3.11.36}
$$

记

$$
\widetilde {\boldsymbol {P}} _ {k} = \boldsymbol {P} _ {k} (\boldsymbol {I} - \theta \widetilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k}) ^ {- 1} = (\boldsymbol {P} _ {k} ^ {- 1} - \theta \widetilde {\boldsymbol {S}} _ {k} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}) ^ {- 1}\tag{3.11.37}
$$

则

$$
\boldsymbol {P} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} \widetilde {\boldsymbol {P}} _ {k} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + \boldsymbol {Q} _ {k}
$$

(3.11.38)

根据上述分析, 代价函数 J 由 $X_{0}$ 和 $W_{k}$ 确定的驻点按式(3.11.24), 式(3.11.26), 式(3.11.25) 计算

$$
\begin{array}{r l} & X _ {0} = \hat {X} _ {0} + P _ {0} \lambda_ {0} \\ & W _ {k} = Q _ {k} \lambda_ {k + 1} \\ & \lambda_ {N} = 0 \end{array}
$$

$\lambda_{k}(k=N-1,N-2,\cdots,1,0)$ 按式(3.11.32)、式(3.11.36)和式(3.11.34)确定：

$$
\boldsymbol {\lambda} _ {k} = \left(\boldsymbol {I} - \theta \tilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k}\right) ^ {- 1} \left[ \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} \boldsymbol {\lambda} _ {k + 1} + \theta \tilde {\boldsymbol {S}} _ {k} (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) \right]
$$

$$
\begin{array}{r l} & {\pmb {P} _ {k + 1} = \pmb {\Phi} _ {k + 1, k} \pmb {P} _ {k} (\pmb {I} - \theta \tilde {\pmb {S}} _ {k} \pmb {P} _ {k} + \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H} _ {k} \pmb {P} _ {k}) ^ {- 1} \pmb {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + \pmb {Q} _ {k}} \\ & {\qquad \pmb {\mu} _ {0} = \hat {\pmb {X}} _ {0}} \\ & {\pmb {\mu} _ {k + 1} = \pmb {\Phi} _ {k + 1, k} \pmb {\mu} _ {k} + \pmb {\Phi} _ {k + 1, k} \pmb {P} _ {k} (\pmb {I} - \theta \tilde {\pmb {S}} _ {k} \pmb {P} _ {k} + \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H} _ {k} \P_ {k}) ^ {- 1} [ \theta \tilde {\pmb {S}} _ {k} (\pmb {\mu} _ {k} - \hat {\pmb {X}} _ {k}) +} \\ & {\qquad \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} (\pmb {Z} _ {k} - \pmb {H} _ {k} \pmb {\mu} _ {k}) ]} \end{array}
$$

其中求解 $\lambda_{k}$ 为逆序递推，求解 $\pmb{\mu}_{k}$ 为顺序递推，初值分别按式(3.11.25)和式(3.11.35)确定。确定出 $\lambda_0$ 和 $\lambda_{k + 1}$ 后，即可按式(3.11.24）和式(3.11.26）确定驻点 $X_0^*$ 和 $W_{k}^{*}$ ，这说明(3.11.29）式所作的假设是成立的。

3. 求解代价函数由 $\hat{X}_{k}$ 和 $Z_{k}$ 确定的驻点

由式 $(3.11.24)$ 和式 $(3.11.29)$ ，得

$$
\lambda_ {0} = P _ {0} ^ {- 1} (X _ {0} - \hat {X} _ {0})\tag{3.11.39}
$$

$$
\boldsymbol {\lambda} _ {k} = \boldsymbol {P} _ {k} ^ {- 1} (\boldsymbol {X} _ {k} - \boldsymbol {\mu} _ {k})\tag{3.11.40}
$$

所以

$$
\| \boldsymbol {\lambda} _ {0} \| _ {\boldsymbol {P} _ {0}} ^ {2} = \boldsymbol {\lambda} _ {0} ^ {\mathrm{T}} \boldsymbol {P} _ {0} \boldsymbol {\lambda} _ {0} = (\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}) ^ {\mathrm{T}} \boldsymbol {P} _ {0} ^ {- 1} \boldsymbol {P} _ {0} \boldsymbol {P} _ {0} ^ {- 1} (\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}) = \| \boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0} \| _ {\boldsymbol {P} _ {0} ^ {- 1}} ^ {2}\tag{3.11.41}
$$

式 $(3.11.41)$ 和式 $(3.11.29)$ 代入式 $(3.11.17)$ ，得

$$
\begin{array}{r l} & {J = - \frac {1}{\theta} \| \pmb {\lambda} _ {0} \| _ {\pmb {P} _ {0}} ^ {2} +  \sum_ {k = 0} ^ {N - 1} \Big [ \| \pmb {\mu} _ {k} + \pmb {P} _ {k} \pmb {\lambda} _ {k} - \hat {\pmb {X}} _ {k} \| _ {\pmb {S} _ {k}} ^ {2} - \frac {1}{\theta} (\| \pmb {W} _ {k} \| _ {\pmb {Q} _ {k} ^ {- 1}} ^ {2} + \| \pmb {Z} _ {k} - \pmb {H} _ {k} (\pmb {\mu} _ {k} + \pmb {P} _ {k} \pmb {\lambda} _ {k}) \| _ {\pmb {R} _ {k} ^ {- 1}} ^ {2}) \Big ]} \\ & {\text {式中} \| \pmb {W} _ {k} \| _ {\pmb {Q} _ {k} ^ {- 1}} ^ {2} = \pmb {W} _ {k} ^ {\mathrm{T}} \pmb {Q} _ {k} ^ {- 1} \pmb {W} _ {k}.} \end{array}
$$

应用式 $(3.11.26)$ 可计算得

$$
\| \boldsymbol {W} _ {k} \| _ {Q _ {k} ^ {- 1}} ^ {2} = \boldsymbol {\lambda} _ {k + 1} ^ {\mathrm{T}} Q _ {k} Q _ {k} ^ {- 1} Q _ {k} \boldsymbol {\lambda} _ {k + 1} = \boldsymbol {\lambda} _ {k + 1} ^ {\mathrm{T}} Q _ {k} \boldsymbol {\lambda} _ {k + 1} = \| \boldsymbol {\lambda} _ {k + 1} \| _ {Q _ {k}} ^ {2}\tag{3.11.42}
$$

所以

$$
\begin{array}{l} J = - \frac {1}{\theta} \| \boldsymbol {\lambda} _ {0} \| _ {\boldsymbol {P} _ {0}} ^ {2} + \sum_ {k = 0} ^ {N - 1} [ \| \boldsymbol {\mu} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - \hat {\boldsymbol {X}} _ {k} \| _ {\hat {\boldsymbol {S}} _ {k}} ^ {2} - \\ \frac {1}{\theta} \| \boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} (\boldsymbol {\mu} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k}) \| _ {\boldsymbol {R} _ {k} ^ {- 1}} ^ {2} ] - \frac {1}{\theta} \sum_ {k = 0} ^ {N - 1} \| \boldsymbol {\lambda} _ {k + 1} \| _ {\mathcal {Q} _ {k}} ^ {2} \end{array}
$$

由于

$$
\begin{array}{r l} \sum_ {k = 0} ^ {N} \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - \sum_ {k = 0} ^ {N - 1} \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} & = \boldsymbol {\lambda} _ {0} ^ {\mathrm{T}} \boldsymbol {P} _ {0} \boldsymbol {\lambda} _ {0} + \sum_ {k = 1} ^ {N} \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - \sum_ {k = 0} ^ {N - 1} \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} = \\ & \| \boldsymbol {\lambda} _ {0} \| _ {\boldsymbol {P} _ {0}} ^ {2} + \sum_ {k = 0} ^ {N - 1} (\boldsymbol {\lambda} _ {k + 1} ^ {\mathrm{T}} \boldsymbol {P} _ {k + 1} \boldsymbol {\lambda} _ {k + 1} - \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k}) \end{array}\tag{3.11.43}
$$

而根据式 $(3.11.25)$

$$
\sum_ {k = 0} ^ {N} \lambda_ {k} ^ {\mathrm{T}} P _ {k} \lambda_ {k} - \sum_ {k = 0} ^ {N - 1} \lambda_ {k} ^ {\mathrm{T}} P _ {k} \lambda_ {k} = \lambda_ {N} ^ {\mathrm{T}} P _ {N} \lambda_ {N} + \sum_ {k = 0} ^ {N - 1} (\lambda_ {k} ^ {\mathrm{T}} P _ {k} \lambda_ {k} - \lambda_ {k} ^ {\mathrm{T}} P _ {k} \lambda_ {k}) = 0
$$

所以

$$
- \frac {1}{\theta} \| \boldsymbol {\lambda} _ {0} \| _ {\boldsymbol {P} _ {0}} ^ {2} = \frac {1}{\theta} \sum_ {k = 0} ^ {N - 1} (\boldsymbol {\lambda} _ {k + 1} ^ {\mathrm{T}} \boldsymbol {P} _ {k + 1} \boldsymbol {\lambda} _ {k + 1} - \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k})
$$

上式代入式(3.11.43)，由于 J 为标量，所以有

$$
\begin{array}{l} J = \sum_ {k = 0} ^ {N - 1} \left[ \| \boldsymbol {\mu} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - \hat {\boldsymbol {X}} _ {k} \| _ {\tilde {\mathbf {s}} _ {k}} ^ {2} - \frac {1}{\theta} \| \boldsymbol {\lambda} _ {k + 1} \| _ {\tilde {\mathbf {Q}} _ {k}} ^ {2} + \frac {1}{\theta} (\boldsymbol {\lambda} _ {k + 1} ^ {\mathrm{T}} \boldsymbol {P} _ {k + 1} \boldsymbol {\lambda} _ {k + 1} - \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k}) - \right. \\ \left. \frac {1}{\theta} \| \boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} (\boldsymbol {\mu} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k}) \| _ {\tilde {\mathbf {R}} _ {k} ^ {- 1}} ^ {2} \right] = \sum_ {k = 0} ^ {N - 1} \left[ \| \boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k} \| _ {\tilde {\mathbf {s}} _ {k}} ^ {2} + 2 (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \tilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} + \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \tilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} + \right. \\ \left. \frac {1}{\theta} \| \boldsymbol {\lambda} _ {k + 1} \| _ {(P _ {k + 1} - Q _ {k})} ^ {2} - \frac {1}{\theta} \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - \frac {1}{\theta} \| \boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k} \| _ {\tilde {\mathbf {R}} _ {k} ^ {- 1}} ^ {2} + \right. \\ \left. \frac {2}{\theta} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - \frac {1}{\theta} \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} \right] \end{array}
$$

考察上述和式中的第四项,根据式(3.11.38),

$$
\begin{array}{r l} \left\| \boldsymbol {\lambda} _ {k + 1} \right\| _ {(P _ {k + 1} - Q _ {k})} ^ {2} & = \boldsymbol {\lambda} _ {k + 1} ^ {\mathrm{T}} (\boldsymbol {P} _ {k + 1} - Q _ {k}) \boldsymbol {\lambda} _ {k + 1} = \boldsymbol {\lambda} _ {k + 1} ^ {\mathrm{T}} \left(\boldsymbol {\Phi} _ {k + 1, k} \widetilde {\boldsymbol {P}} _ {k} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + Q _ {k} - Q _ {k}\right) \boldsymbol {\lambda} _ {k + 1} = \\ & \quad \boldsymbol {\lambda} _ {k + 1} ^ {\mathrm{T}} \boldsymbol {\Phi} _ {k + 1, k} \widetilde {\boldsymbol {P}} _ {k} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} \boldsymbol {\lambda} _ {k + 1} \end{array} \tag {3.}\tag{3.11.45}
$$

式中， $\tilde{P}_k$ 由式(3.11.37)确定。

又由式(3.11.31)，得

$$
\boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} \boldsymbol {\lambda} _ {k + 1} = \boldsymbol {\lambda} _ {k} - \theta \tilde {\boldsymbol {S}} _ {k} (\boldsymbol {\mu} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - \hat {\boldsymbol {X}} _ {k}) - \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} [ \boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} (\boldsymbol {\mu} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k}) ]
$$

将该式代入式(3.11.45)，所以

$$
\begin{array}{r l} \left\| \boldsymbol {\lambda} _ {k + 1} \right\| _ {(P _ {k + 1} - Q _ {k})} ^ {2} & = \{\boldsymbol {\lambda} _ {k} - \theta \tilde {\boldsymbol {S}} _ {k} (\boldsymbol {\mu} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - \hat {\boldsymbol {X}} _ {k}) - \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} [ \boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} (\boldsymbol {\mu} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k}) ] \} ^ {\mathrm{T}} \widetilde {\boldsymbol {P}} _ {k} \{\boldsymbol {\lambda} _ {k} - \\ & \quad \theta \tilde {\boldsymbol {S}} _ {k} (\boldsymbol {\mu} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - \hat {\boldsymbol {X}} _ {k}) - \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} [ \boldsymbol {Z} _ {h} - \boldsymbol {H} _ {k} (\boldsymbol {\mu} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k}) ] \} = \\ & \quad \{\boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} (\boldsymbol {I} - \theta \boldsymbol {P} _ {k} \tilde {\boldsymbol {S}} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}) - \theta (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \tilde {\boldsymbol {S}} _ {k} - \\ & \quad (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}) \widehat {\boldsymbol {P}} _ {k} \{\boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} (\boldsymbol {I} - \theta \boldsymbol {P} _ {k} \tilde {\boldsymbol {S}} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}) = \\ & \quad \theta (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \tilde {\boldsymbol {S}} _ {k} - (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}) ^ {\mathrm{T}} \end{array}
$$

由式 $(3.11.37)$

$$
\boldsymbol {I} - \theta \boldsymbol {P} _ {k} \tilde {\boldsymbol {S}} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} = \boldsymbol {P} _ {k} \tilde {\boldsymbol {P}} _ {k} ^ {- 1}
$$

所以

$$
\begin{array}{r l} \left\| \boldsymbol {\lambda} _ {k + 1} \right\| _ {(P _ {k + 1} - Q _ {k})} ^ {2} & = \left[ \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \widetilde {\boldsymbol {P}} _ {k} ^ {- 1} - \theta (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \widetilde {\boldsymbol {S}} _ {k} - (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \right] \widetilde {\boldsymbol {P}} _ {k} \\ & \left[ \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \widetilde {\boldsymbol {P}} _ {k} ^ {- 1} - \theta (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \widetilde {\boldsymbol {S}} _ {k} - (\boldsymbol {Z} k - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \right] ^ {\mathrm{T}} = \\ & \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \widetilde {\boldsymbol {P}} _ {k} ^ {- 1} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - \theta (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \widetilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - \\ & \theta \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \widetilde {\boldsymbol {S}} _ {k} (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) + \theta^ {2} (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \widetilde {\boldsymbol {S}} _ {k} \widetilde {\boldsymbol {P}} _ {k} \widetilde {\boldsymbol {S}} _ {k} (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) + \\ & \theta (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \boldsymbol {\mu} _ {k}) ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} \mathbf {H} _ {k} \widetilde {\mathbf {P}} _ {k} \widetilde {\mathbf {S}} _ {k} (\boldsymbol {\mu} _ {k} - \hat {\mathbf {X}} _ {k}) - \boldsymbol {\lambda} ^ {\mathrm{T}} \mathbf {P} _ {k} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \boldsymbol {\mu} _ {k}) + \\ & \theta (\boldsymbol {\mu} _ {k} - \hat {\mathbf {X}} _ {k}) ^ {\mathrm{T}} \widetilde {\mathbf {S}} _ {k} \widetilde {\mathbf {P}} _ {k} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \boldsymbol {\mu} _ {k}) + \\ & (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \boldsymbol {\mu} _ {k}) ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} \mathbf {H} _ {k} \widetilde {\mathbf {P}} _ {k} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \boldsymbol {\mu} _ {k}) \end{array}
$$

该式中各项都是标量,转置后不变,所以

$$
\begin{array}{r l} \left\| \boldsymbol {\lambda} _ {k + 1} \right\| _ {(P _ {k + 1} - Q _ {k})} ^ {2} & = \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \widetilde {\boldsymbol {P}} _ {k} ^ {- 1} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - 2 \theta (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \widetilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - 2 (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} + \\ & \quad \theta^ {2} (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \widetilde {\boldsymbol {S}} _ {k} \widetilde {\boldsymbol {P}} _ {k} \widetilde {\boldsymbol {S}} _ {k} (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) + 2 \theta (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \widetilde {\boldsymbol {S}} _ {k} \widetilde {\boldsymbol {P}} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} (\boldsymbol {Z} _ {k} - \\ & \quad \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) + (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \widetilde {\boldsymbol {P}} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) \end{array}
$$

由式 $(3.11.37)$ ，得

$$
\widetilde {\boldsymbol {P}} _ {k} ^ {- 1} = \boldsymbol {P} _ {k} ^ {- 1} (\boldsymbol {I} - \theta \boldsymbol {P} _ {k} \widetilde {\boldsymbol {S}} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k})\tag{3.11.47}
$$

所以

$$
\begin{array}{r l} \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \widetilde {\boldsymbol {P}} _ {k} ^ {- 1} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} & = \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} (\boldsymbol {I} - \theta \boldsymbol {P} _ {k} \widetilde {\boldsymbol {S}} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}) \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} = \\ & \quad \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - \theta \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \widetilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} + \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} \end{array}
$$

代入式 $(3.11.46)$ ，得

$$
\begin{array}{r l} \parallel \boldsymbol {\lambda} _ {k + 1} \parallel_ {(P _ {k + 1} - Q _ {k})} ^ {2} & = \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - \theta \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \widetilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} + \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - \\ & 2 \theta (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \tilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - 2 (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} + \\ & \theta^ {2} (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \tilde {\boldsymbol {S}} _ {k} \tilde {\boldsymbol {P}} _ {k} \tilde {\boldsymbol {S}} _ {k} (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) + 2 \theta (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \tilde {\boldsymbol {S}} _ {k} \tilde {\boldsymbol {P}} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} (\boldsymbol {Z} _ {k} - \\ & \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) + (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \tilde {\boldsymbol {P}} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) \end{array}\tag{3.11.48}
$$

式 $(3.11.48)$ 代入式 $(3.11.44)$ ，得

$$
\begin{array}{l} J = \sum_ {k = 0} ^ {N - 1} \left[ (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \tilde {\boldsymbol {S}} _ {k} (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) - \frac {1}{\theta} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) + \right. \\ \left. \theta (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \tilde {\boldsymbol {S}} _ {k} \tilde {\boldsymbol {P}} _ {k} \tilde {\boldsymbol {S}} _ {k} (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) + 2 (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \tilde {\boldsymbol {S}} _ {k} \tilde {\boldsymbol {P}} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) + \right. \\ \left. \frac {1}{\theta} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \tilde {\boldsymbol {P}} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) \right] = \\ \sum_ {k = 0} ^ {N - 1} \left[ (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} (\tilde {\boldsymbol {S}} _ {k} + \theta \tilde {\boldsymbol {S}} _ {k} \tilde {\boldsymbol {P}} _ {k} \tilde {\boldsymbol {S}} _ {k}) (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) + 2 (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \tilde {\boldsymbol {S}} _ {k} \tilde {\boldsymbol {P}} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {K} _ {k} ^ {- 1} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) + \right. \\ \left. \frac {1}{\theta} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) ^ {\mathrm{T}} (\boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \tilde {\boldsymbol {P}} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} - \boldsymbol {R} _ {k} ^ {- 1}) (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) \right] \\ \frac {\partial J}{\partial \hat {\boldsymbol {X}} _ {k}} = 2 (\tilde {\boldsymbol {S}} _ {k} + \theta \tilde {\boldsymbol {S}} _ {k} \tilde {\boldsymbol {P}} _ {k} \tilde {\boldsymbol {S}} _ {k}) (\hat {\boldsymbol {X}} _ {k} - \boldsymbol {\mu} _ {k}) + 2 \tilde {\boldsymbol {S}} _ {k} \tilde {\boldsymbol {P}} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} (\boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k} - \boldsymbol Z _ {k}) = 0 \\ \frac {\partial J}{\partial Z _ {k}} = \frac 2{\theta} (\boldsymbol R _ {k} ^ {- 1} \boldsymbol H _ {k} \tilde {\boldsymbol P} _ {k} \boldsymbol H _ {k} ^ {\mathrm{T}} \boldsymbol R _ {k} ^ {- 1} - \boldsymbol R _ {k} ^ {- 1}) (\boldsymbol Z _ {k} - \boldsymbol H _ {k} \boldsymbol {\mu} _ {k}) + 2 \boldsymbol R _ {k} ^ {- 1} \boldsymbol H _ {k} \tilde {\boldsymbol P} _ {k} \tilde {\boldsymbol S} _ {k} (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol X} _ {k}) = 0 \end{array}\tag{3.11.50}
$$

从式 $(3.11.49)$ 和式 $(3.11.50)$ 可得

$$
\hat {X} _ {k} = \mu_ {k}\tag{3.11.51a}
$$

$$
\mathbf {Z} _ {k} = \mathbf {H} _ {k} \boldsymbol {\mu} _ {k}\tag{3.11.51b}
$$

根据式 $(3.11.49)$ 和式 $(3.11.37)$

$$
\frac {\partial^ {2} J}{\partial \hat {\boldsymbol {X}} _ {k} \partial \hat {\boldsymbol {X}} _ {k} ^ {\mathrm{T}}} = 2 (\tilde {\boldsymbol {S}} _ {k} + \theta \tilde {\boldsymbol {S}} _ {k} \tilde {\boldsymbol {P}} _ {k} \tilde {\boldsymbol {S}} _ {k}) = 2 [ \tilde {\boldsymbol {S}} _ {k} + \theta \tilde {\boldsymbol {S}} _ {k} (\boldsymbol {P} _ {k} ^ {- 1} - \theta \tilde {\boldsymbol {S}} _ {k} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}) ^ {- 1} \tilde {\boldsymbol {S}} _ {k} ]
$$

从上式可看出，只要 $\theta$ 取得足够小，则总能保证 $\frac{\partial^2J}{\partial\hat{X}_k\partial\hat{X}_k^{\mathrm{T}}}$ 正定，即

$$
\boldsymbol {P} _ {k} ^ {- 1} - \theta \tilde {\boldsymbol {S}} _ {k} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} > 0\tag{3.11.52}
$$

从而使 $\hat{X}_{k}=\mu_{k}$ 时代价函数 J 达到最小。由(3.11.12)式知， $\theta$ 取得足够小意味着代价函数 $J_{1}$ 的上界定得较宽，因此参数 $\theta$ 与代价函数的上界有关，应该定得适当小，定得过大，则在 $\hat{X}_{k}=\mu_{k}$ 处代价函数 J 会出现最大值。

式 $(3.11.51)$ 代入式 $(3.11.34)$ ，得

其中

$$
\begin{array}{r l} & {\hat {\mathbf {X}} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} \hat {\mathbf {X}} _ {k} + \boldsymbol {\Phi} _ {k + 1, k} \mathbf {K} _ {k} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k})} \\ & {\mathbf {K} _ {k} = \mathbf {P} _ {k} (\mathbf {I} - \theta \tilde {\mathbf {S}} _ {k} \mathbf {P} _ {k} + \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} \mathbf {H} _ {k} \mathbf {P} _ {k}) ^ {- 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1}} \end{array}\tag{3.11.53}
$$

(3.11.54)

根据式 $(3.11.50)$

$$
\frac {\partial^ {2} J}{\partial \mathbf {Z} _ {k} \partial \mathbf {Z} _ {k} ^ {\mathrm{T}}} = \frac {2}{\theta} (\mathbf {R} _ {k} ^ {- 1} \mathbf {H} _ {k} \widetilde {\mathbf {P}} _ {k} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} - \mathbf {R} _ {k} ^ {- 1}) = \frac {2}{\theta} \mathbf {R} _ {k} ^ {- 1} (\mathbf {H} _ {k} \widetilde {\mathbf {P}} _ {k} \mathbf {H} _ {k} ^ {\mathrm{T}} - \mathbf {R} _ {k}) \mathbf {R} _ {k} ^ {- 1}
$$

从上式可看出, 如果 $R_{k}$ 取得过大, 使 $H_{k}\widetilde{P}_{k}H_{k}^{T}-R_{k}$ 为负定阵, 则在 $Z_{k}=H_{k}\mu_{k}$ 处代价函数 J 会出现极大值, 要使之成为极小值, 需有

$$
\boldsymbol {H} _ {k} \widetilde {\boldsymbol {P}} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} - \boldsymbol {R} _ {k} > 0
$$

式 $(3.11.37)$ 代入上式，得

$$
\boldsymbol {H} _ {k} \boldsymbol {P} _ {k} (\boldsymbol {I} - \theta \tilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k}) ^ {- 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} - \boldsymbol {R} _ {k} > 0\tag{3.11.55}
$$

## 3.11.3 $H_{\infty}$ 滤波算法

1. 离散型 $H_{\infty}$ 滤波算法

综合以上讨论,整理出离散型 $H_{\infty}$ 滤波算法如下。

(1) 滤波处理对象

$$
\begin{array}{r l} \mathbf {X} _ {k + 1} & = \boldsymbol {\Phi} _ {k + 1, k} \mathbf {X} _ {k} + \mathbf {W} _ {k} \\ \mathbf {Z} _ {k} & = \mathbf {H} _ {k} \mathbf {X} _ {k} + \mathbf {V} _ {k} \\ \mathbf {y} _ {k} & = \mathbf {T} _ {k} \mathbf {X} _ {k} \end{array}
$$

式中， $y_{k}$ 是被估计量， $W_{k}$ 和 $V_{k}$ 不一定是白噪声。

(2) 取代价函数

$$
J _ {1} = \frac {\sum_ {k = 0} ^ {N - 1} \| \mathbf {y} _ {k} - \hat {\mathbf {y}} _ {k} \| _ {s _ {k}} ^ {2}}{\| \mathbf {X} _ {0} - \hat {\mathbf {X}} _ {0} \| _ {P _ {0} ^ {- 1}} ^ {2} + \sum_ {k = 0} ^ {N - 1} (\| \mathbf {W} _ {k} \| _ {Q _ {k} ^ {- 1}} ^ {2} + \| \mathbf {V} _ {k} \| _ {R _ {k} ^ {- 1}} ^ {2})}\tag{3.11.11}
$$

选取 $P_{0}, Q_{k}, R_{k}$ 和 $S_{k}$ 时，必须保证这些矩阵对称和正定。

(3) 滤波算法

$$
\tilde {\boldsymbol {S}} _ {k} = \boldsymbol {T} _ {k} ^ {\mathrm{T}} \boldsymbol {S} _ {k} \boldsymbol {T} _ {k}\tag{3.11.16}
$$

$$
\hat {\mathbf {X}} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} \hat {\mathbf {X}} _ {k} + \boldsymbol {\Phi} _ {k + 1, k} \mathbf {K} _ {k} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k})\tag{3.11.53}
$$

$$
\boldsymbol {K} _ {k} = \boldsymbol {P} _ {k} (\boldsymbol {I} - \theta \tilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k}) ^ {- 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1}\tag{3.11.54}
$$

$$
\boldsymbol {P} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k} (\boldsymbol {I} - \theta \tilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k}) ^ {- 1} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + \boldsymbol {Q} _ {k}\tag{3.11.36}
$$

式中 $k=0,1,2,\cdots$ 。

选取 $\theta$ 时需保证：

$$
\begin{array}{r} {\pmb {P} _ {k} ^ {- 1} - \theta \tilde {\pmb {S}} _ {k} + \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H} _ {k} > 0} \\ {\pmb {H} _ {k} \pmb {P} _ {k} (\pmb {I} - \theta \tilde {\pmb {S}} _ {k} \pmb {P} _ {k} + \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H} _ {k} \pmb {P} _ {k}) ^ {- 1} \pmb {H} _ {k} ^ {\mathrm{T}} - \pmb {R} _ {k} > 0} \end{array}\tag{3.11.52}
$$

(3.11.55)

2. 连续型 $H_{\infty}$ 滤波算法

设有连续型系统方程和量测方程

$$
\begin{array}{r l} \dot {\mathbf {X}} (t) & = \mathbf {F} (t) \mathbf {X} (t) + \mathbf {B} (t) \mathbf {u} (t) + \mathbf {W} (t) \\ \mathbf {Z} (t) & = \mathbf {H} (t) \mathbf {X} (t) + \mathbf {V} (t) \end{array}
$$

被估计量为

$$
\mathbf {y} (t) = \mathbf {T} (t) \mathbf {X} (t)
$$

代价函数选为

$$
J _ {1} = \frac {\int_ {0} ^ {T} \| \mathbf {y} (t) - \hat {\mathbf {y}} (t) \| _ {s} ^ {2} \mathrm{d} t}{\| \mathbf {X} (0) - \hat {\mathbf {X}} (0) \| _ {P _ {0} ^ {- 1}} ^ {2} + \int_ {0} ^ {T} (\| \mathbf {W} \| _ {q ^ {- 1}} ^ {2} + \| \mathbf {V} \| _ {r ^ {- 1}} ^ {2}) \mathrm{d} t}\tag{3.11.56}
$$

对应的 $H_{\infty}$ 滤波方程为

$$
\dot {\boldsymbol {P}} (t) = \boldsymbol {F} (t) \boldsymbol {P} (t) + \boldsymbol {P} (t) \boldsymbol {F} ^ {\mathrm{T}} (t) + \boldsymbol {q} - \boldsymbol {K} (t) \boldsymbol {H} (t) \boldsymbol {P} (t) + \theta \boldsymbol {P} (t) \boldsymbol {T} ^ {\mathrm{T}} (t) \boldsymbol {S T} (t) \boldsymbol {P} (t)\tag{3.11.57a}
$$

$$
\boldsymbol {K} (t) = \boldsymbol {P} (t) \boldsymbol {H} (t) \boldsymbol {r} ^ {- 1}\tag{3.11.57b}
$$

$$
\dot {\hat {\mathbf {X}}} (t) = \mathbf {F} (t) \hat {\mathbf {X}} (t) + \mathbf {B} (t) \mathbf {u} (t) + \mathbf {K} (t) [ \mathbf {Z} (t) - \mathbf {H} (t) \hat {\mathbf {X}} (t) ]\tag{3.11.57c}
$$

$$
\hat {\mathbf {y}} (t) = \mathbf {T} (t) \hat {\mathbf {X}} (t)\tag{3.11.57d}
$$

3. $H_{\infty}$ 滤波与卡尔曼滤波的比较

为了揭示卡尔曼滤波与 $H_{\infty}$ 滤波的内在联系，对卡尔曼滤波一步预测基本方程（见 2.2.4 节）作恒等变形。

$$
\hat {\boldsymbol {X}} _ {k + 1 / k} = \boldsymbol {\Phi} _ {k + 1, k} \hat {\boldsymbol {X}} _ {k / k - 1} + \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {K} _ {k} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \hat {\boldsymbol {X}} _ {k / k - 1})\tag{3.11.58a}
$$

$$
\boldsymbol {K} _ {k} = \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}) ^ {- 1}\tag{3.11.58b}
$$

$$
\boldsymbol {P} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {P} _ {k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} + \boldsymbol {Q} _ {k - 1}\tag{3.11.58c}
$$

$$
\boldsymbol {P} _ {k} = (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) \boldsymbol {P} _ {k / k - 1}\tag{3.11.58d}
$$

上述方程对应的滤波对象如式 $(3.11.1)$ 所列。

根据矩阵反演公式，

$$
\left(\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}\right) ^ {- 1} = \boldsymbol {R} _ {k} ^ {- 1} - \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \left(\boldsymbol {I} + \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}\right) ^ {- 1} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1}\tag{3.11.59}
$$

所以

$$
\begin{array}{r l} & {\mathbf {K} _ {k} = \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} (\mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} + \mathbf {R} _ {k}) ^ {- 1} =} \\ & {\quad \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} - \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} \mathbf {H} _ {k} (\mathbf {I} + \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} \mathbf {H} _ {k}) ^ {- 1} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} =} \\ & {\quad [ \mathbf {I} - \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} \mathbf {H} _ {k} (\mathbf {I} + \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R } _ {k} ^ {- 1} \mathbf {H} _ {k}) ^ {- 1} ] \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} =} \\ & {\quad [ (\mathbf {I} + \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} \mathbf {H} _ {k}) - \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} \mathbf {H} _ {k}) ] (\mathbf {I} + \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} \mathbf {H} _ {k}) ^ {- 1} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathcal {R} _ {k} ^ {- 1}} \\ & {\quad (\mathbf {I} + \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} \mathbf {H} _ {k}) ^ {- 1} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbb {R} _ {k} ^ {- 1}} \\ & {\quad P _ {k + 1 / k} = \boldsymbol {\Phi_ {k + 1, k}} P _ {k} \boldsymbol {\Phi_ {k + 1, k}} + Q _ {k} =} \\ & {\quad \boldsymbol {\Phi_ {k + 1, k}} (\mathbf {I - K _ {k}} H _ {k}) P _ {k / k - 1} \boldsymbol {\Phi_ {k + 1, k}} + Q _ {k} =} \\ & {\quad \boldsymbol {\Phi_ {k + 1, k}} P _ {k / k - 1} \boldsymbol {\Phi_ {k + 1, k}} - \boldsymbol {\Phi_ {k + 1, k}} K _ {k} H _ {k} P _ {k / k - 1} \boldsymbol {\Phi_ {k + 1, k}} + Q _ {k} =} \\ & {\quad \boldsymbol {\Phi_ {k + 1, k}} P _ {k / k - 1} \boldsymbol {\Phi_ {k + 1, k}} - \boldsymbol {\Phi_ {k + 1, k}} (\mathbf {(I + P _ {_ k / k - 1}} H _ {_ k} ^ {\mathrm{T}} R _ {_ k} ^ {- 1} H _ {_ k}) ^ {- 1} P _ {_ k / k - 1} H _ {_ k} ^ {\mathrm{T}} R _ {_ k} ^ {- 1} H _ {_ k} P _ {_ k / k - 1} \boldsymbol {\Phi_ {k + 1, k}} + Q _ {_ k =}} \\ & {\quad \boldsymbol {\Phi_ {k + 1, k}} P _ {_ k / k - 1} \boldsymbol {\Phi_ {k + 1, k}} - \boldsymbol {\Phi_ {k + 1, k}} (P _ {_ k / k - 1} ^ {- 1} + H _ {_ k} ^ {\mathrm{T}} R _ {_ k} ^ {- 1} H _ {_ k}) ^ {- 1} H _ {_ k} ^ {\mathrm{T}} R _ {_ k} ^ {- 1} H _ {_ k} P _ {_ k / k - 1} \boldsymbol {\Phi_ {k + 1, k}} + Q _ {_ k}} \end{array}
$$

对上式第二项中的求逆应用矩阵反演公式,得

$$
\begin{array}{r l} \boldsymbol {P} _ {k + 1 / k} & = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} - \boldsymbol {\Phi} _ {k + 1, k} [ \boldsymbol {P} _ {k / k - 1} - \\ & \quad \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {R} _ {k} + \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}}) ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} ] \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + Q _ {k} = \\ & \quad \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k / k - 1} [ \boldsymbol {I} - \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {R} _ {k} + \\ & \quad \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}}) ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} ] \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + Q _ {k} = \\ & \quad \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k / k - 1} \Pi_ {k} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + Q _ {k} \end{array}\tag{3.11.61}
$$

式中

$$
\boldsymbol {\Pi} _ {k} = \boldsymbol {I} - \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \left(\boldsymbol {R} _ {k} + \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}}\right) ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1}
$$

式中的 $(\pmb{R}_k + \pmb{H}_k\pmb{P}_{k / k - 1}\pmb{H}_k^{\mathrm{T}})^{-1}$ 采用矩阵求逆反演公式求解。

$$
\begin{array}{r l} \boldsymbol {\Pi} _ {k} & = \boldsymbol {I} - \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \left[ \boldsymbol {R} _ {k} ^ {- 1} - \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} (\boldsymbol {P} _ {k / k - 1} ^ {- 1} + \right. \\ & \left. \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}) ^ {- 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \right] \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} = \end{array}
$$

$$
\begin{array}{r l} & {\pmb {I} - \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H} _ {k} \pmb {P} _ {k / k - 1} + (\pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H} _ {k} \pmb {P} _ {k / k - 1}) ^ {2} =} \\ & {\pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H} _ {k} (\pmb {P} _ {k / k - 1} ^ {- 1} + \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H} _ {k}) ^ {- 1} (\pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H} _ {k} \pmb {P} _ {k / k - 1}) ^ {2} =} \\ & {\pmb {I} - \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H} _ {k} \pmb {P} _ {k / k - 1} + (\pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H _ {k}} \pmb {P} _ {k / k - 1}) ^ {2} =} \\ & {\pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H} _ {k} \pmb {P} _ {k / k - 1} (\pmb {I} + \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H _ {k}} \pmb {P} _ {k / k - 1}) ^ {- 1} (\pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H _ {k}} \pmb {P} _ {k / k - 1}) ^ {2} =} \\ & {\pmb {I} - \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H _ {k}} \pmb {P} _ {k / k - 1} + (\pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H _ {k}} \pmb {P} _ {k / k - 1}) ^ {2}} \\ & {\left(\pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H _ {k}} \pmb {P} _ {k / k - 1}\right) ^ {3} (\pmb {I} + \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R _ {k}} ^ {- 1} \pmb {H _ {k}} \pmb {P} _ {k / k - 1}) ^ {- 1}} \\ & {\left[ (\pmb {I} + \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R _ {k}} ^ {- 1} \pmb {H _ {k}} \pmb {P _ {k / k - 1}}) - \pmb {H _ {k}} ^ {\mathrm{T}} \pmb {R _ {k}} ^ {- 1} \pmb {H _ {k}} \pmb {P _ {k / k - 1}} (\pmb {I} + \pmb {H _ {k}} ^ {\mathrm{T}} \pmb {R _ {k}} ^ {- 1} \pmb {H _ {k}} \pmb {P _ {k / k - 1}}) + \right.} \\ & {\left. (\pmb {H _ {k}} ^ {\mathrm{T}} \pmb {R _ {k}} ^ {- 1} \pmb {H _ {k}} \pmb {P _ {k / k - 1}}) ^ {2} (\pmb {I} + \pmb {H _ {k}} ^ {\mathrm{T}} \pmb {R _ {k}} ^ {- 1} \pmb {H _ {k}} \pmb {P _ {k / k - 1}}) - \right.} \\ & \left. (\pmb {(H _ {{\bf k}}} ^ {\mathrm{T}} \pmb {{R_ {{\bf k}}} ^ {- 1}} \pmb {{H_ {{\bf k}}} P_ {{\bf k / k - 1}}}) ^ {- 3}\right) (\pmb {{I}} + \pmb {{H_ {{\bf k}}} ^ {\mathrm{T}} \pmb {{R_ {{\bf k}}} ^ {- 1}} \pmb {{H_ {{\bf k}}} P_ {{\bf k / k - 1}}}) ^ {- 1}} \\ & \left. (\pmb {{I}} + \pmb {{H_ {{\bf k}}} ^ {\mathrm{T}} \pmb {{R_ {{\bf k}}} ^ {- 1}} \pmb {{H_ {{\bf k}}} P_ {{\bf k / k - 1}}}) ^ {- 1}\right)} \end{array}\tag{3.11.62}
$$

式 $(3.11.62)$ 代入式 $(3.11.61)$ ，得

$$
\boldsymbol {P} _ {k + 1 / k} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k / k - 1} (\boldsymbol {I} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1}) ^ {- 1} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + \boldsymbol {Q} _ {k}\tag{3.11.63}
$$

由式 $(3.11.60)$ ，

$$
\begin{array}{r l} \mathbf {K} _ {k} & = (\mathbf {I} + \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} \mathbf {H} _ {k}) ^ {- 1} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} = \\ & \quad \mathbf {P} _ {k / k - 1} \mathbf {P} _ {k / k - 1} ^ {- 1} (\mathbf {I} + \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} \mathbf {H} _ {k}) ^ {- 1} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbb {R} _ {k} ^ {- 1} = \\ & \quad \mathbf {P} _ {k / k - 1} (\mathbf {P} _ {k / k - 1} + \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbb {R} _ {k} ^ {- 1} \mathbf {H} _ {k} \mathbf {P} _ {k / k - 1}) ^ {- 1} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbb {R} _ {k} ^ {- 1} = \\ & \quad \mathbf {P} _ {k / k - 1} (\mathbf {I} + \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbb {R} _ {k} ^ {- 1} \mathbf {H} _ {k} \mathbf {P} _ {k / k - 1}) ^ {- 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbb {R} _ {k} ^ {- 1} \end{array}\tag{3.11.64}
$$

比较式(3.11.53)与式(3.11.58a)，式(3.11.54)与式(3.11.64)，和式(3.11.36)与式(3.11.63)，可看出当 $\theta=0$ 时， $H_{\infty}$ 滤波与卡尔曼滤波的一步预测计算完全相同。

综上所述可归纳出如下结论：

（1）设计卡尔曼滤波器时，所使用的系统噪声、量测噪声和初始估计误差的先验信息是这些误差的方差阵 $Q_{k}, R_{k}, P_{0}$ ，而设计 $H_{\infty}$ 滤波器时使用的是这些误差的最大幅值信息。

(2) 卡尔曼滤波是 $H_{\infty}$ 滤波 $\theta=0$ 时的特例，对应的代价函数是无界的，这意味卡尔曼滤波器不能保证系统在所有条件下的滤波效果，如果设计中使用的 $Q_{k}, R_{k}, P_{0}$ 与实际情况不符，滤波效果不会达到最优，差别越大，滤波效果就越差。调谐能一定程度改善滤波效果，但调谐具有盲目性。而 $H_{\infty}$ 滤波通过 $\theta\tilde{S}_{k}P_{k}$ 项实现增大 $K_{k}$ 和 $P_{k+1}$ （见式（3.11.54）和式（3.11.36）），从而增大对新鲜量测的利用权重。并且这一过程是通过定量计算实现的。所以 $H_{\infty}$ 滤波可看做卡尔曼滤波的鲁棒型。

(3) $H_{\infty}$ 滤波中， $\frac{1}{\theta}$ 是代价函数的界， $\theta$ 越小，式(3.11.52)和式(3.11.55)越容易满足，滤波器设计越容易，但代价函数的界就越高，滤波器的潜在风险就越大。这意味着一旦滤波器工作不正常，滤波精度恶化就越严重。

(4) 若只需估计状态变量的线性组合 $y_{k}=T_{k}X_{k}$ ，则卡尔曼滤波先计算 $\hat{X}_{k}$ ，再计算 $\hat{y}_{k}=T_{k}\hat{X}_{k}$ ，滤波过程与 $T_{k}$ 无关。而 $H_{\infty}$ 滤波中 $T_{k}$ 参与了滤波计算，与滤波效果密切相关。

例3-9 设标量系统方程和量测方程为

$$
\begin{array}{r l} & X _ {k + 1} = X _ {k} + W _ {k} \\ & Z _ {k} = X _ {k} + V _ {k} \\ & Y _ {k} = X _ {k} \end{array}
$$

其中 $W_{k}$ 和 $V_{k}$ 均为随机量，且统计特性未知。确定出稳态 $H_{\infty}$ 滤波方程，用于估计 $X_{k}$ 。

解 选择 $Q = R = S = 1$ ，根据题意， $\Phi_{k+1,k} = 1, H_k = 1, T_k = 1$ 。由式(3.11.36)和式(3.11.54)，

$$
\begin{array}{r l} P _ {k + 1} & = \frac {P _ {k}}{1 - \theta P _ {k} + P _ {k}} + 1 = \frac {1 - \theta P _ {k} + 2 P _ {k}}{1 - \theta P _ {k} + P _ {k}} \\ K _ {k} & = \frac {P _ {k}}{1 - \theta P _ {k} + P _ {k}} \end{array}
$$

滤波达到稳态时， $P_{k + 1} = P_k = P$ ，所以有

$$
\begin{array}{l} P (1 - \theta P + P) = 1 - \theta P + 2 P \\ (1 - \theta) P ^ {2} + (\theta - 1) P - 1 = 0 \end{array}
$$

$$
P = \frac {1 - \theta \pm \sqrt {(\theta - 1) ^ {2} + 4 (1 - \theta)}}{2 (1 - \theta)} = \frac {1 - \theta \pm \sqrt {(\theta - 1) (\theta - 5)}}{2 (1 - \theta)}
$$

1) 当 $1 - \theta > 0$ ，即取 $\theta < 1$ 时， $P = \frac{1}{2} \pm \frac{1}{2} \sqrt{\frac{5 - \theta}{1 - \theta}}$

$$
P _ {+} = \frac {1}{2} \left(1 + \sqrt {\frac {5 - \theta}{1 - \theta}}\right) > 0\tag{1}
$$

$P_{-} = \frac{1}{2}\left(1 - \sqrt{\frac{5 - \theta}{1 - \theta}}\right) < 0$ ，不合题意，应舍去。

2) 当取 $1 < \theta < 5$ 时， $(\theta - 1)(\theta - 5) < 0$ 。 $\sqrt{(\theta - 1)(\theta - 5)}$ 不存在，P 无解。所以取 $1 < \theta < 5$ 不能满足要求。

3）当取 $\theta > 5$ 时， $P = \frac{1}{2} \pm \frac{1}{2}\left[-\frac{\sqrt{(\theta - 1)(\theta - 5)}}{\theta - 1}\right] = \frac{1}{2}\left(1 \mp \sqrt{\frac{\theta - 5}{\theta - 1}}\right)$ 。因为在 $\theta > 5$ 条件下， $\frac{\theta - 5}{\theta - 1} = 1 - \frac{4}{\theta - 1} < 1$ ，即 $\sqrt{\frac{\theta - 5}{\theta - 1}} < 1$ 。所以

$$
P _ {+} = \frac {1}{2} \left(1 - \sqrt {\frac {\theta - 5}{\theta - 1}}\right) > 0\tag{2}
$$

$$
P _ {-} = \frac {1}{2} \left(1 + \sqrt {\frac {\theta - 5}{\theta - 1}}\right) > 0\tag{3}
$$

又根据式(3.11.52)， $\theta$ 取值应满足： $\frac{1}{P}-\theta+1>0$ ，即 $\theta$ 应满足： $\theta<1+\frac{1}{P}$ 。若能确定出 $1+\frac{1}{P}$ 的上限。即可确定出 $\theta$ 取值时的允许上限。

对于解(1)，由于 $\theta < 1, \frac{5 - \theta}{1 - \theta}$ 取得最小值时， $P$ 也取得最小值。而 $\theta = -\infty$ 时， $\frac{5 - \theta}{1 - \theta}$ 达到最小，即

$$
\lim _ {\theta \rightarrow - \infty} \frac {5 - \theta}{1 - \theta} = \lim _ {\theta \rightarrow - \infty} \frac {\frac {5}{\theta} - 1}{\frac {1}{\theta} - 1} = 1
$$

此时，P=1 为最小值， $\frac{1}{P}$ 达到最大值 1。所以 $\theta$ 应满足 $\theta<1+\frac{1}{P}=2$ ，即根据式(3.11.52)的要求， $\theta$ 取值不能超过 2。所以只有情况 1)，即 $\theta<1$ 才满足 $H_{\infty}$ 滤波要求，而不能取 $\theta>5$ 。稳态滤波方程为

$$
\hat {X} _ {k + 1} = \hat {X} _ {k} + \overline {{{K}}} (Z _ {k} - \hat {X} _ {k})
$$

式中

$$
\overline {{{K}}} = \frac {P}{1 - \theta P + P}
$$

$$
P = \frac {1}{2} \left(1 + \sqrt {\frac {5 - \theta}{1 - \theta}}\right)
$$

$$
\theta <   1
$$

$$
\text { 习   题 }
$$

3-1 在量测噪声为有色噪声的条件下,如果连续系统中有已知控制量,即

$$
\dot {\boldsymbol {X}} (t) = \boldsymbol {F} (t) \boldsymbol {X} (t) + \boldsymbol {u} (t) + \boldsymbol {w} (t)
$$

试导出滤波方程。

3-2 设系统和量测为

$$
\begin{array}{r l} & {\left[ \begin{array}{l} {\dot {X} _ {1} (t)} \\ {\dot {X} _ {2} (t)} \\ {\dot {X} _ {3} (t)} \end{array} \right] = \left[ \begin{array}{l l l} {0} & {1} & {0} \\ {0} & {0} & {1} \\ {- 1} & {0} & {- 2} \end{array} \right] \left[ \begin{array}{l} {X _ {1} (t)} \\ {X _ {2} (t)} \\ {X _ {3} (t)} \end{array} \right] + \left[ \begin{array}{l} {0} \\ {0} \\ {1} \end{array} \right] w (t)} \\ & {\qquad Z _ {1} (t) = X _ {1} (t) + v (t)} \\ & {\qquad Z _ {2} (t) = X _ {2} (t)} \end{array}
$$

按如下已知条件分别列写出滤波方程：

(1) $w(t)$ 和 $v(t)$ 为互不相关的零均值白噪声,方差强度均为1。

(2) $w(t)$ 和 $v(t)$ 为零均值白噪声,但两者相关,互协方差强度为 $S(t)$ 。

(3) $v(t)$ 为零均值的有色噪声

$$
\dot {v} (t) = - v (t) + \xi (t)
$$

$w(t)$ 和 $\xi(t)$ 都是互不相关的零均值白噪声，方差强度均为 1, $v(0)$ 的方差为零。

$$
v (t) = \zeta (t) + \xi (t) \tag {4}
$$

式中

$$
\dot {\zeta} (t) = - \zeta (t) + \xi (t)
$$

$w(t)$ 和 $\xi(t)$ 都是互不相关的零均值白噪声，方差强度都为 1，且 $\zeta(0)$ 的方差为零。

3-3 设系统方程和量测方程为

式中

$$
\begin{array}{r l} & {\mathbf {X} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} \mathbf {X} _ {k} + \boldsymbol {\Gamma} _ {k} \mathbf {W} _ {k}} \\ & {\mathbf {Z} _ {k} = \mathbf {H} _ {k} \mathbf {X} _ {k} + \mathbf {V} _ {k} + \boldsymbol {\eta} _ {k}} \\ & {\mathbf {V} _ {k + 1} = \boldsymbol {\Psi} _ {k + 1, k} \mathbf {V} _ {k} + \boldsymbol {\xi} _ {k}} \\ & {E [ \mathbf {W} _ {k} ] = \mathbf {0}, \quad E [ \mathbf {W} _ {k} \mathbf {W} _ {j} ^ {\mathrm{T}} ] = Q _ {k} \delta_ {k j}} \\ & {E [ \boldsymbol {\eta} _ {k} ] = \mathbf {0}, \quad E [ \boldsymbol {\eta} _ {k} \boldsymbol {\eta} _ {j} ^ {\mathrm{T}} ] = \boldsymbol {\rho} _ {k} \delta_ {k j}} \\ & {E [ \boldsymbol {\xi} _ {k} ] = \mathbf {0}, \quad E [ \boldsymbol {\xi} _ {k} \boldsymbol {\xi} _ {j} ^ {\mathrm{T}} ] = R _ {k} \delta_ {k j}} \end{array}
$$

$W_{k}, \eta_{k}$ 和 $\xi_{k}$ 互不相关。

试分别用状态扩增法和量测扩增法列写出递推滤波方程，并比较两种滤波方法的特点。

3-4 设系统方程和量测方程为

$$
\begin{array}{r l} & \mathbf {X} _ {k} = \boldsymbol {\Phi} _ {k, k - 1} \mathbf {X} _ {k - 1} \\ & \mathbf {Z} _ {k} = \mathbf {H} _ {k} \mathbf {X} _ {k} + \mathbf {V} _ {k} \end{array}
$$

其中 $Z_{k}$ 为 m 维向量， $V_{k}$ 为零均值的白噪声序列，方差阵 $R_{k}$ 为对角阵，试列写出 k-1 至 k 步的序贯处理平方根滤波方程。

3-5 如果例 3-3 中, $H = [1 \quad 1]$ , 按基本卡尔曼滤波方程和平方根滤波方程分别计算出 $K_{k+1}$ 和 $P_{k+1}$ , 并比较两者的结果。

3-6 如果量测不是标量而是向量，且不采用序贯处理，则在平方根滤波的 Potter 算法中，量测更新将会遇到什么障碍？

3-7 设计滤波器时选用的系统模型和量测模型为

$$
\begin{array}{r} {\pmb {X} _ {k} = \pmb {\Phi} _ {k, k - 1} \pmb {X} _ {k - 1} + \pmb {\Gamma} _ {k - 1} \pmb {W} _ {k - 1}} \\ {\pmb {Z} _ {k} = \pmb {H} _ {k} \pmb {X} _ {k} + \pmb {V} _ {k}} \end{array}
$$

证明如下结论：

(1) 若 $\Phi_{k,k-1}, H_{k}, \Gamma_{k}, Q_{k}$ 和 $R_{k}$ 有误差，则即使 $\hat{X}_{0} = E[X_{0}^{r}] = m_{X_{0}}^{r}$ ，估计仍然是有偏的；

(2) 在上述条件下, 如果 $m_{X_{0}}^{r}=0$ , 则估计是无偏的;

(3) 在(1)的条件下, 若 $\Delta\Phi_{k,k-1}=0,\Delta H_{k}=0$ , 则估计是无偏的。

3-8 在模型正确的条件下，令 $P_{k}^{r}$ 表示用正确参数得到的滤波估计均方误差，并令

$$
\begin{array}{l} \Delta P _ {k} ^ {p} = P _ {k} ^ {p} - P _ {k} ^ {r} \\ \Delta P _ {k} ^ {r} = P _ {k} - P _ {k} ^ {r} \end{array}
$$

如果选取：

$$
Q _ {k} \geqslant Q _ {k} ^ {r}, R _ {k} \geqslant R _ {k} ^ {r}, P _ {0} ^ {p} \geqslant P _ {0} ^ {r}, P _ {0} \geqslant P _ {0} ^ {r}
$$

证明：

$$
\Delta P _ {k} ^ {p} \geqslant 0, \Delta P _ {k} ^ {r} \geqslant 0
$$

（注：这说明 $P_{k}^{r}$ 是 $P_{k}^{p}$ 和 $P_{k}$ 可能达到的最小值）

3-9 对式(3.10.13)两边求偏导数,试推导出式(3.10.21)。

3-10 试用式(3.10.41)求解出例3-7的小尺度灵敏度方程(12)、(13)、(14)。

3-11 设系统和量测为

$$
\left[ \begin{array}{c} \boldsymbol {X} _ {k + 1} ^ {1} \\ \boldsymbol {X} _ {k + 1} ^ {2} \end{array} \right] = \left[ \begin{array}{c c} \boldsymbol {\Phi} _ {k + 1, k} ^ {1 1} & \boldsymbol {\Phi} _ {k + 1, k} ^ {1 2} \\ \boldsymbol {0} & \boldsymbol {\Phi} _ {k + 1, k} ^ {2 2} \end{array} \right] \left[ \begin{array}{c} \boldsymbol {X} _ {k} ^ {1} \\ \boldsymbol {X} _ {k} ^ {2} \end{array} \right] + \left[ \begin{array}{c c} \boldsymbol {\Gamma} _ {k} ^ {1} & \boldsymbol {0} \\ \boldsymbol {0} & \boldsymbol {\Gamma} _ {k} ^ {2} \end{array} \right] \left[ \begin{array}{c} \boldsymbol {W} _ {k} ^ {1} \\ \boldsymbol {W} _ {k} ^ {2} \end{array} \right]
$$

$$
\mathbf {Z} _ {k} = \left[ \begin{array}{l l} \mathbf {H} _ {k} ^ {1} & \mathbf {0} \end{array} \right] \left[ \begin{array}{l} \mathbf {X} _ {k} ^ {1} \\ \mathbf {X} _ {k} ^ {2} \end{array} \right] + \mathbf {V} _ {k}
$$

其中， $W_{k}^{1}$ 、 $W_{k}^{2}$ 和 $V_{k}$ 都是互不相关的零均值白噪声序列， $W_{k}^{1}$ 的方差阵为 $Q_{k}^{1}$ ， $W_{k}^{2}$ 的方差阵为 $Q_{k}^{2}$ ， $V_{k}$ 的方差阵为 $R_{k}$ 。

仿照式(3.9.3)～(3.9.14)的推导，推导出仅估计 $X_{k}^{1}$ 的降阶滤波方程。

3-12 设 $R_{c}(t - \tau) = \begin{cases} \frac{1}{\Delta t}, & t \leqslant \tau \leqslant t + \Delta t \\ 0, & \text{其余 } t \end{cases}$

则 $\delta (t - \tau) = \lim_{\Delta t\to 0}R_c(t - \tau)$

利用这一关系式,证明:

$$
\int_ {t _ {0}} ^ {\mathrm{T}} \boldsymbol {T} (t) \boldsymbol {\Phi} ^ {a \mathrm{T}} (t, \tau) \delta (t - \tau) \mathrm{d} \tau + \int_ {t _ {0}} ^ {\mathrm{T}} \boldsymbol {\Phi} ^ {a} (t - \tau) \boldsymbol {T} (t) \delta (t - \tau) \mathrm{d} \tau = \boldsymbol {T} (t)
$$

式中 $\Phi^{a}(t,\tau)$ 为某一线性系统的转移阵， $T(t)$ 为某一矩阵。

3-13 直升机作定高悬停，悬停高度用机载无线电高度表作监测。设无线电高度表的采样间隔为 $T$ ，高度测量误差为零均值的白噪声序列，方差为 $R$ 。由于气流影响，直升机在定高值附近作快速上下颠簸，高度变化近似描述为： $X_{k} - X_{k - 1} = W_{k - 1}$ ，其中 $W_{k - 1}$ 为零均值白噪声，方差为 $Q$ 。

(1) 设计提取精确高度的卡尔曼滤波器；

(2) 确定出当滤波达到稳态时的滤波方程。

3-14 设标量系统方程和量测方程为

$$
\begin{array}{r l} & X _ {k + 1} = X _ {k} + W _ {k} \\ & Z _ {k} = h X _ {k} + V _ {k} \\ & Y _ {k} = X _ {k} \end{array}
$$

式中 $W_{k}$ 和 $V_{k}$ 均为随机量，且统计特性未知。确定出稳态 $H_{\infty}$ 滤波方程，用于估计 $X_{k}$ 。

## 第四章 卡尔曼滤波稳定性介绍

## 4.1 稳定性定义

稳定性是任何控制系统正常工作的基本要求。在经典控制理论中，稳定性是指系统受到某一扰动后恢复原有运动状态的能力，即如果系统受到有界扰动，不论扰动引起的初始偏差有多大，在扰动撤除后，系统都能以足够的准确度恢复到原始平稳状态，则这种系统是稳定的。显然这指的是系统的运动稳定性。而卡尔曼滤波理论中的稳定性指的是系统的平衡状态稳定性，即李雅普诺夫意义下的稳定性。下面介绍这种稳定性的详细定义。

设线性系统为

$$
\boldsymbol {X} _ {k} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {X} _ {k - 1} + \boldsymbol {u} _ {k - 1}\tag{4.1.1}
$$

并设 $X_{k}^{1}$ 和 $X_{k}^{2}$ 是系统对应于不同初始状态 $X_{0}^{1}$ 和 $X_{0}^{2}$ 在 k 时刻的状态。

定义1

如果任意给定 $\varepsilon > 0$ ，总可找到 $\delta = \delta(\varepsilon, t_{0}) > 0$ ，当 $\|X_{0}^{1} - X_{0}^{2}\| < \delta$ 时， $\|X_{k}^{1} - X_{k}^{2}\| < \varepsilon$ 恒成立，则称式（4.1.1）所示系统是稳定的。其中 $\|X\|$ 为向量 X 的范数，即 $\|X\| = \sqrt{x_{1}^{2} + x_{2}^{2} + \cdots + x_{n}^{2}}$ 。

定义2

如果任意给定 $\varepsilon > 0$ ，总可以找到 $\delta = \delta(\varepsilon)$ ，当 $\|X_0^1 - X_0^2\| < \delta$ 时， $\|X_k^1 - X_k^2\| < \varepsilon$ 恒成立，则称系统式(4.1.1)是一致稳定的。

稳定和一致稳定的差别在于后者的 $\delta$ 仅与 $\varepsilon$ 有关，而与 $t_0$ 无关。所谓一致是指 $t_0$ 而言的。对定常系统，不同的 $t_0$ 时刻，转移阵仍然是常阵。所以对定常系统，稳定和一致稳定是一样的。显然，对时变系统，稳定并不等于一致稳定，而一致稳定必然是稳定的。

定义3

若系统式(4.1.1)不但稳定,而且对任意初始状态 $X_{0}^{1}$ 和 $X_{0}^{2}$ ,总有

$$
\lim _ {t _ {k} \rightarrow \infty} \| \boldsymbol {X} _ {k} ^ {1} - \boldsymbol {X} _ {k} ^ {2} \| = 0
$$

即任意给定 $\mu > 0$ ，总可以找到 $T = T(\mu, t_0) > 0$ ，当 $t_k \geqslant t_0 + T$ 时， $\|X_k^1 - X_k^2\| < \mu$ 恒成立，则称系统式(4.1.1)是渐近稳定的。

定义4

若系统式(4.1.1)不但一致稳定,而且对任意初始状态 $X_{0}^{1}$ 和 $X_{0}^{2}$ ,总有

$$
\lim _ {t _ {k} \rightarrow \infty} \| \mathbf {X} _ {k} ^ {1} - \mathbf {X} _ {k} ^ {2} \| = 0
$$

即对任意给定的 $\mu > 0$ ，总可以找到 $T = T(\mu) > 0$ ，当 $t_k \geqslant t_0 + T$ 时， $\|X_k^1 - X_k^2\| < \mu$ 恒成立，则称系统式(4.1.1)是一致渐近稳定的。

同样，对定常系统，渐近稳定和一致渐近稳定两者等价。

卡尔曼滤波器实际上可看做类似式(4.1.1)所示线性系统。根据基本方程式(2.2.4)，状态估计为

$$
\begin{array}{r l} \hat {\mathbf {X}} _ {k} & = \boldsymbol {\Phi} _ {k, k - 1} \hat {\mathbf {X}} _ {k - 1} + \mathbf {K} _ {k} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \boldsymbol {\Phi} _ {k, k - 1} \hat {\mathbf {X}} _ {k - 1}) = \\ & (\mathbf {I} - \mathbf {K} _ {k} \mathbf {H} _ {k}) \boldsymbol {\Phi} _ {k, k - 1} \hat {\mathbf {X}} _ {k - 1} + \mathbf {K} _ {k} \mathbf {Z} _ {k} \end{array}\tag{4.1.2}
$$

该式描述了一个线性系统，系统的状态为 $\hat{X}_k$ ，一步转移阵为 $(I - K_kH_k)\Phi_{k,k - 1}$ ，系统的动态激励为 $K_{k}Z_{k}$ 。所以滤波稳定问题可视为线性系统的稳定问题。但是由于增益阵 $K_{k}$ 是时变的，它由估计的均方误差阵确定，所以式(4.1.2）所示系统是时变系统，根据一步转移阵 $(I - K_kH_k)\Phi_{k,k - 1}$ 来分析滤波稳定性是很不方便的。目前，一般都是直接根据被估计状态的系统方程和量测方程来判定滤波稳定性。

## 4.2 判别卡尔曼滤波稳定的充分条件

2.2.4 小节的定理 2.10 指出，当 $\hat{X}_{0}=E[X_{0}], P_{0}=Var[X_{0}]$ 时，卡尔曼滤波的估计从滤波开始起就是无偏的，且估计的均方误差是最小的。但是在工程实践中，常常得不到正确的上述初始状态的验前统计值。如果滤波初始值选取了其他值，这对滤波会产生什么影响，这就是滤波稳定性需要研究的问题。具体包括如下内容：

（1）随着滤波时间的增长，估计值 $\hat{X}_{k}$ （或 $\hat{X}(t)$ ）是否逐渐不受所选的初始估计值 $\hat{X}_{0}$ （或 $\hat{X}(0)$ ）的影响。

（2）随着滤波时间的增长，估计均方误差阵 $P_{k}$ （或 $P(t)$ ）是否逐渐不受初始均方误差阵 $P_{0}$ （或 $P(0)$ ）的影响。

如果随着滤波时间的增长, $\hat{X}_{k}$ (或 $\hat{X}(t)$ ) 和 $P_{k}$ (或 $P(t)$ ) 各自都逐渐不受其初值的影响,则滤波器是滤波稳定的。如果滤波器不是滤波稳定的,则估计是有偏的,估计均方误差也不是最小的。因此,滤波器是不是滤波稳定的,是滤波器能否正常工作的前提。

## 4.2.1 滤波稳定充分条件之一:系统随机可控和随机可观测

定理4.1

设连续系统的状态方程和测量方程为

$$
\dot {\boldsymbol {X}} (t) = \boldsymbol {F} (t) \boldsymbol {X} (t) + \boldsymbol {G} (t) \boldsymbol {w} (t)\tag{4.2.1}
$$

$$
\mathbf {Z} (t) = \mathbf {H} (t) \mathbf {X} (t) + \boldsymbol {v} (t)\tag{4.2.2}
$$

式中

$$
\left. \begin{array}{l} E [ w (t) ] = 0, E [ w (t) w ^ {\mathrm{T}} (\tau) ] = q (t) \delta (t - \tau) \\ E [ v (t) ] = 0, E [ v (t) v ^ {\mathrm{T}} (\tau) ] = r (t) \delta (t - \tau) \\ E [ w (t) v ^ {\mathrm{T}} (\tau) ] = 0 \end{array} \right\}\tag{4.2.3}
$$

如果系统一致完全随机可控和一致完全随机可观测，且 $q(t)$ 和 $r(t)$ 都是正定的，则卡尔曼滤波器是一致渐近稳定的。

连续系统一致完全随机可控指的是：

$$
\boldsymbol {\Lambda} (t, t - \sigma) \stackrel {\mathrm{def}} {=} \int_ {t - \sigma} ^ {t} \boldsymbol {\Phi} (t, \tau) \boldsymbol {G} (\tau) \boldsymbol {q} (\tau) \boldsymbol {G} ^ {\mathrm{T}} (\tau) \boldsymbol {\Phi} ^ {\mathrm{T}} (t, \tau) \mathrm{d} \tau > 0\tag{4.2.4}
$$

式中， $\boldsymbol{\Lambda}(t,t-\sigma)$ 为连续型随机可控阵， $\sigma$ 为与 t 无关的正数， $\Phi(t,\tau)$ 为系统从 $\tau$ 到 t 的转移矩

阵，>0 表示正定。

随机可控与可控的意义是类似的。后者是描述系统的确定性输入（或控制）影响系统状态的能力，而前者是描述系统的随机噪声影响系统状态的能力。将上述随机可控的判别式(4.2.4)与控制理论中介绍的可控判别式

$$
\boldsymbol {\Lambda} (t, t - \sigma) = \int_ {t - \sigma} ^ {t} \boldsymbol {\Phi} (t, \tau) \boldsymbol {E} (\tau) \boldsymbol {E} ^ {\mathrm{T}} (\tau) \boldsymbol {\Phi} ^ {\mathrm{T}} (t, \tau) \mathrm{d} \tau > 0\tag{4.2.5}
$$

相比较,可看出两种判别式的形式是相似的。区别仅在于:式(4.2.5)中 $E(t)$ 为确定性控制系统的控制阵,而式(4.2.4)中,与 $E(t)$ 位置相对应的 $G(t)$ 是噪声驱动阵,并且考虑了系统噪声方差强度阵 $q(t)$ 的影响。

连续系统一致完全随机可观测指的是

$$
\pmb {M} (t, t - \sigma) \stackrel {\mathrm{def}} {=} \int_ {t - \sigma} ^ {t} \pmb {\Phi} ^ {\mathrm{T}} (\tau , t) \pmb {H} ^ {\mathrm{T}} (\tau) \pmb {r} ^ {- 1} (\tau) \pmb {H} (\tau) \pmb {\Phi} (\tau , t) \mathrm{d} \tau > \mathbf {0}\tag{4.2.6}
$$

$M(t,t-\sigma)$ 为连续型随机可观测阵, $\sigma$ 为与 t 无关的正数。

随机可观测与可观测的意义更为相似。后者是描述无量测噪声时从量测中确定出状态的能力，而前者为描述从含有噪声误差的量测中估计状态的能力。确定性系统的可观测阵为

$$
\pmb {M} (t, t - \dot {\sigma}) \stackrel {\mathrm{def}} {=} \int_ {t - \sigma} ^ {t} \pmb {\Phi} ^ {\mathrm{T}} (\tau , t) \pmb {H} ^ {\mathrm{T}} (\tau) \pmb {H} (\tau) \pmb {\Phi} (\tau , t) \mathrm{d} \tau > 0\tag{4.2.7}
$$

比较式(4.2.6)和式(4.2.7)，可看出随机可观测阵比可观测阵多考虑了量测噪声方差强度阵 $r(t)$ 的影响， $r(t)$ 越大，则随机可观测性就越差。

定理 4.1 所述滤波稳定充分条件是最早由卡尔曼提出的 $^{[12]}$ 。随着卡尔曼滤波技术的发展，参考文献 $[63 \sim 65]$ 提出了离散系统滤波稳定的充分条件。该条件叙述如下：

定理4.2

设离散系统的状态方程和量测方程为

$$
\boldsymbol {X} _ {k} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {X} _ {k - 1} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {W} _ {k - 1}
$$

$$
\mathbf {Z} _ {k} = \mathbf {H} _ {k} \mathbf {X} _ {k} + \mathbf {V} _ {k}\tag{4.2.8}
$$

(4.2.9)

式中

$$
\left. \begin{array}{l} E [ \boldsymbol {W} _ {k} ] = \mathbf {0}, E [ \boldsymbol {W} _ {k} \boldsymbol {W} _ {j} ^ {\mathrm{T}} ] = Q _ {k} \delta_ {k j} \\ E [ \boldsymbol {V} _ {k} ] = \mathbf {0}, E [ \boldsymbol {V} _ {k} \boldsymbol {V} _ {j} ^ {\mathrm{T}} ] = R _ {k} \delta_ {k j} \\ E [ \boldsymbol {W} _ {k} \boldsymbol {V} _ {j} ^ {\mathrm{T}} ] = \mathbf {0} \end{array} \right\}\tag{4.2.10}
$$

如果系统一致完全随机可控和一致完全随机可观测，且 $Q_{k}$ 和 $R_{k}$ 都是正定的，则卡尔曼滤波器是一致渐近稳定的。

离散系统一致完全随机可控指的是

$$
\boldsymbol {\Lambda} (k, k - N + 1) = \sum_ {i = k - N + 1} ^ {k} \boldsymbol {\Phi} _ {k, i} \boldsymbol {\Gamma} _ {i - 1} \boldsymbol {Q} _ {i - 1} \boldsymbol {\Gamma} _ {i - 1} ^ {\mathrm{T}} \boldsymbol {\Phi} _ {k, i} ^ {\mathrm{T}} > \mathbf {0}\tag{4.2.11}
$$

式中， $\Lambda(k,k-N+1)$ 为离散型随机可控阵，N 为与 k 无关的正整数。

离散系统一致完全随机可观测指的是

$$
\boldsymbol {M} (k, k - N + 1) = \sum_ {i = k - N + 1} ^ {k} \boldsymbol {\Phi} _ {i, k} ^ {\mathrm{T}} \boldsymbol {H} _ {i} ^ {\mathrm{T}} \boldsymbol {R} _ {i} ^ {- 1} \boldsymbol {H} _ {i} \boldsymbol {\Phi} _ {i, k} > \mathbf {0}\tag{4.2.12}
$$

式中， $M(k,k-N+1)$ 为离散型随机可观测阵，N 为与 k 无关的正整数。

对定常系统,不论是离散的还是连续的,一致完全随机可控和一致完全随机可观测就是完

全随机可控和完全随机可观测。完全随机可控的判别式可简化为

$$
\operatorname{rank} \left[ F ^ {n - 1} G q ^ {\frac {1}{2}} \quad F ^ {n - 2} G q ^ {\frac {1}{2}} \quad \dots \quad G q ^ {\frac {1}{2}} \right] = n \quad (\text {连续系统})\tag{4.2.13}
$$

$$
\mathrm{rank} [ \Phi^ {n - 1} \Gamma Q ^ {\frac {1}{2}} \quad \Phi^ {n - 2} \Gamma Q ^ {\frac {1}{2}} \quad \dots \quad \Gamma Q ^ {\frac {1}{2}} ] = n \quad (\text {离散系统})\tag{4.2.14}
$$

如果 q 和 Q 正定, 则完全随机可控的判别式还可进一步简化为

$$
\operatorname{rank} \left[ \begin{array}{c c c c} \boldsymbol {F} ^ {n - 1} \boldsymbol {G} & \boldsymbol {F} ^ {n - 2} \boldsymbol {G} & \dots & \boldsymbol {G} \end{array} \right] = n (\text {连续系统})\tag{4.2.15}
$$

$$
\operatorname{rank} \left[ \begin{array}{c c c c} {\Phi^ {n - 1} \Gamma} & {\Phi^ {n - 2} \Gamma} & {\dots} & {\Gamma} \end{array} \right] = n (\text {离散系统})\tag{4.2.16}
$$

由于一般系统都有 r > 0, R > 0, 所以完全随机可观测的判别式可简化为

$$
\operatorname{rank} \left[ \begin{array}{c} {{H}} \\ {{H F}} \\ {{\vdots}} \\ {{H F ^ {n - 1}}} \end{array} \right] = n \quad (\text { 连续系统 })\tag{4.2.17}
$$

$$
\operatorname{rank} \left[ \begin{array}{c} {{H}} \\ {{H \Phi}} \\ {{\vdots}} \\ {{H \Phi^ {n - 1}}} \end{array} \right] = n \quad (\text {离散系统})\tag{4.2.18}
$$

以上诸式中，n 为系统的维数， $q^{\frac{1}{2}}$ 和 $Q^{\frac{1}{2}}$ 分别表示 q 和 Q 的平方根，即 $q = q^{\frac{1}{2}} q^{\frac{1}{2} T}, Q = Q^{\frac{1}{2}} Q^{\frac{1}{2} T}$ 。可以看出，定常系统的完全随机可观测的判别式与完全可观测的判别式是一样的。

对定常系统来说,如果滤波器是滤波稳定的,则均方误差阵不但渐近地与初值无关,而且逐渐地趋向稳态值。以离散系统为例,根据式(2.2.4d)、式(2.2.4c)及式(2.2.4e'),一步预测均方误差阵可写成

$$
\boldsymbol {P} _ {k + 1 / k} = \boldsymbol {\Phi} \left[ \boldsymbol {P} _ {k / k - 1} - \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} ^ {\mathrm{T}} \left(\boldsymbol {H} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} ^ {\mathrm{T}} + \boldsymbol {R}\right) ^ {- 1} \boldsymbol {H} \boldsymbol {P} _ {k / k - 1} \right] \boldsymbol {\Phi} ^ {\mathrm{T}} + \boldsymbol {\Gamma} Q \boldsymbol {\Gamma} ^ {\mathrm{T}}\tag{4.2.19}
$$

令一步预测均方误差阵的稳态值为 $\bar{P}$ ，则

$$
\overline {{P}} = \boldsymbol {\Phi} [ \overline {{P}} - \overline {{P}} \boldsymbol {H} ^ {\mathrm{T}} (\boldsymbol {H} \overline {{P}} \boldsymbol {H} ^ {\mathrm{T}} + \boldsymbol {R}) ^ {- 1} \boldsymbol {H} \overline {{P}} ] \boldsymbol {\Phi} ^ {\mathrm{T}} + \boldsymbol {\Gamma} Q \boldsymbol {\Gamma} ^ {\mathrm{T}}\tag{4.2.20}
$$

增益阵 $K_{k}$ 的稳态值为

$$
\boldsymbol {K} = \overline {{\boldsymbol {P}}} \boldsymbol {H} ^ {\mathrm{T}} (\boldsymbol {H} \overline {{\boldsymbol {P}}} \boldsymbol {H} ^ {\mathrm{T}} + \boldsymbol {R}) ^ {- 1}\tag{4.2.21}
$$

估计均方误差阵 $P_{k}$ 的稳态值为

$$
\boldsymbol {P} = (\boldsymbol {I} - \boldsymbol {K H}) \bar {\boldsymbol {P}}\tag{4.2.22}
$$

或

$$
\boldsymbol {P} = (\boldsymbol {I} - \boldsymbol {K H}) \boldsymbol {\Phi} \boldsymbol {P} \boldsymbol {\Phi} ^ {\mathrm{T}} + \boldsymbol {\Gamma} \boldsymbol {Q} \boldsymbol {\Gamma} ^ {\mathrm{T}}\tag{4.2.23}
$$

$$
\boldsymbol {K} = \boldsymbol {P H} ^ {\mathrm{T}} \boldsymbol {R} ^ {- 1}\tag{4.2.24}
$$

这时，用式(4.1.2)描述的滤波器达到稳态，滤波器系统的转移矩阵 $(I - K_{k}H_{k})\Phi_{k,k - 1}$ 达到稳态值

$$
\boldsymbol {\Psi} = (\boldsymbol {I} - \boldsymbol {K H}) \boldsymbol {\Phi}\tag{4.2.25}
$$

不论是连续系统还是离散系统，上述判别条件的证明基本上都是一样的，即在系统一致完全随机可控和一致完全随机可观测的条件下，首先证明 $P_{k}$ 或 $P(t)(k > 0,t > 0)$ 是正定的，并且有上界和下界，然后用 $P_{k}(P(t))$ 或其逆阵组成李雅普诺夫函数，证明该函数符合李雅普诺夫第二方法的一致渐近稳定条件。因此，滤波器是一致渐近稳定的。在滤波器是一致渐近稳定的条件下，可以很方便地证明 $P_{k}(P(t))$ 渐近地不受初值的影响。因为李雅普诺夫第二方法的稳定性条件是充分条件,所以这个判别条件也仅是充分条件。

例4-1 设系统和量测为

$$
\begin{array}{r} X _ {k + 1} = X _ {k} + W _ {k} \\ Z _ {k} = X _ {k} + V _ {k} \end{array}
$$

其中 $X_{k}$ 和 $Z_{k}$ 都是标量， $W_{k}$ 和 $V_{k}$ 为互不相关的零均值白噪声序列，方差分别 Q 和 R。判别滤波器是否滤波稳定。

解 从物理意义或从离散定常系统的判别式都可确定系统是随机可控和随机可观测的。从 $K_{k}$ 的计算过程来看

$$
\begin{array}{r l} & P _ {k / k - 1} = P _ {k - 1} + Q \\ & K _ {k} = \frac {P _ {k / k - 1}}{P _ {k / k - 1} + R} \end{array}
$$

随机可控(以 Q 表征)的意义是:不论 $P_{k-1}(k>0)$ 值大小,Q 始终保证 $P_{k/k-1}$ 有值,从而 $K_{k}$ 有值,使每步计算都能利用量测中的最新信息,修正旧估计,得到新的实时估计。即随机可控为估计提供了条件。而随机可观测(以 R 表征)说明量测提供了带有噪声的量测值,同样也为估计提供了条件。一般在 $P_{0}$ 取较大值的情况下,随着估计过程的进行, $P_{k-1}$ 是逐渐下降的(这说明估计在起作用)。下降的程度与 Q 和 R 有关。而 $P_{k/k-1}$ 却比 $P_{k-1}$ 大,增大的值与 Q 有关。因此,当下降值与增大值相当时, $K_{k}$ 值趋于稳态值,滤波器趋于稳态。从以上分析可以看出滤波器稳定与 Q,R 的关系。根据式(4.2.20)和式(4.2.21),本例稳态的 $P_{k/k-1}$ 和 $K_{k}$ 各为(当 Q=R=1 时)

可解得

$$
\begin{array}{r l} & {\overline {{{P}}} = \overline {{{P}}} - \frac {\overline {{{P}}} ^ {2}}{\overline {{{P}}} + 1} + 1} \\ & {\qquad \overline {{{P}}} = 1. 6 1 8} \\ & {K = \frac {1 . 6 1 8}{1 . 6 1 8 + 1} = 0. 6 1 8} \end{array}
$$

所以

利用随机可控和随机可观测作滤波稳定的判别条件是目前常采用的方法。这是因为：

（1）随机可控和随机可观测只须要利用系统的参数阵和噪声方差阵就可以直接进行计算判别，不必变换系统，方法比较方便；

(2) 很多系统都能满足这种判别条件。

但是毕竟还有不少系统(例如惯性导航系统)常不满足这种条件。而充分条件是判别滤波器稳定的上界条件。若满足充分条件,则可以肯定滤波器是稳定的;但若不满足充分条件,就不能下结论滤波器不稳定,只是不能肯定滤波器是稳定的。这就带来了判断模糊问题。可见充分条件越严,判断模糊问题就严重,而充分条件越宽,就越容易满足,判断模糊现象就越少。因此,人们提出了其他一些较宽的判别条件。

## 4.2.2 滤波稳定充分条件之二:系统随机可观测和推广形式的随机可控

系统完全随机可控这一条件意味着系统噪声必须对系统的所有状态起作用,但是有的系统并不满足这一条件。参考文献[69]提出用下式代替随机可控阵

$$
\begin{array}{r l} & {\overline {{{{\pmb \Lambda}}}} (t, t _ {0}) = \pmb \Phi (t, t _ {0}) \pmb P (0) \pmb \Phi^ {\mathrm{T}} (t, t _ {0}) +} \\ & {\qquad \int_ {t _ {0}} ^ {t} \pmb \Phi (t, \tau) \pmb G (\tau) \pmb q (\tau) \pmb G ^ {\mathrm{T}} (\tau) \pmb \Phi^ {\mathrm{T}} (t, \tau) \mathrm{d} \tau \quad (\text {连续系统})} \\ & {\overline {{{{\pmb \Lambda}}}} (k, k _ {0}) = \pmb \Phi (k, k _ {0}) \pmb P _ {0} \pmb \Phi^ {\mathrm{T}} (k, k _ {0}) +} \end{array}\tag{4.2.26}
$$

$$
\sum_ {i = k _ {0} + 1} ^ {k} \pmb {\Phi} (k, i) \pmb {\Gamma} _ {i - 1} \pmb {Q} _ {i - 1} \pmb {\Gamma} _ {i - 1} ^ {\mathrm{T}} \pmb {\Phi} ^ {\mathrm{T}} (k, i) \qquad (\text {离散系统})\tag{4.2.27}
$$

上述两式中， $\bar{\Lambda}$ 为推广形式的随机可控阵，右边第二项就是随机可控阵。

参考文献[69]证明了 $\overline{\Lambda}(t,t_0),\mathbf{P}(t)(\overline{\Lambda}(k,k_0),\mathbf{P}_k)$ 的零空间是相同的，因此，只要 $\overline{\Lambda}(t,t_0)$ 是非奇异的，则 $P(t)(P_k)$ 也是非奇异的。这就保证了 $P^{-1}(t)(P_k^{-1})$ 的存在。再加上系统一致完全可观测，证明了用 $P^{-1}(t)(P_k^{-1})$ 组成的李雅普诺夫函数符合渐近稳定条件。

离散系统滤波稳定的充分条件为：

## 定理4.3

如果离散系统是一致完全可观测的， $\bar{\Lambda}(k_{1},k_{0})$ 对某 $k_{1}$ 时刻是非奇异的，系统有关参数阵 $\Phi_{k,k-1},\Gamma_{k},H_{k},Q_{k}$ 和 $R_{k}^{-1}$ 有界，则卡尔曼滤波器是渐近稳定的。

上述充分条件有两个主要特点：

（1）推广形式的随机可控阵描述了系统随机可控的性质，同时还与滤波器的初始估计均方误差阵 $P_0$ 有关。如果 $P_0 > 0$ ，则因转移阵 $\Phi (k_{1},k_{0})$ 是满秩的，所以 $\overline{\Lambda} (k_1,k_0)$ 必然满秩，这就放宽了可控阵必须正定的条件。

(2) $\bar{\Lambda}(k_{1},k_{0})$ 是指某 $k_{1}$ 时刻的，它不具有一致性。因此滤波器仅满足渐近稳定条件。

系统一致完全随机可观测对滤波稳定的作用在上一小节中已经叙述过。如果 $P_{0} > 0$ ，则可以保证即使不存在系统噪声的条件下，仍可以尽量利用量测值来修正估计，这与系统噪声的作用是相似的。

## 4.2.3 滤波稳定充分条件之三:系统随机可稳定和随机可检测

## 定理4.4

如果线性定常离散系统是完全随机可稳定和完全随机可检测的,则卡尔曼滤波器是渐近稳定的。

下面详细介绍完全随机可稳定和完全随机可检测的含义。

设线性定常离散系统的系统方程和量测方程为

$$
\boldsymbol {X} _ {k} = \boldsymbol {\Phi} \boldsymbol {X} _ {k - 1} + \boldsymbol {\Gamma} \boldsymbol {W} _ {k - 1}\tag{4.2.28}
$$

$$
\mathbf {Z} _ {k} = \mathbf {H X} _ {k} + \mathbf {V} _ {k}\tag{4.2.29}
$$

且假设系统不是完全随机可控的。

## 1. 完全随机可稳定

所谓完全随机可稳定是指:对状态 $X_{k}$ 作满秩线性变换,将系统的可控部分和不可控部分分离开,如果不可控部分是稳定的,则系统就是随机可稳定的。

设

$$
\boldsymbol {X} _ {k} = \boldsymbol {T} _ {\mathrm{c}} \boldsymbol {X} _ {k} ^ {\mathrm{c}}
$$

(4.2.30)

式中， $T_{c}$ 是某一满秩变换矩阵， $X_{k}^{c}$ 是被分离为可控部分和不可控部分的状态。则式(4.2.28)和式(4.2.29)变成

即

记

$$
\begin{array}{r l} & T _ {\mathrm{c}} X _ {k} ^ {\mathrm{c}} = \Phi T _ {\mathrm{c}} X _ {k - 1} ^ {\mathrm{c}} + I W _ {k - 1} \\ & Z _ {k} = H T _ {\mathrm{c}} X _ {k} ^ {\mathrm{c}} + V _ {k} \\ & X _ {k} ^ {\mathrm{c}} = T _ {\mathrm{c}} ^ {- 1} \Phi T _ {\mathrm{c}} X _ {k - 1} ^ {\mathrm{c}} + T _ {\mathrm{c}} ^ {- 1} I W _ {k - 1} \\ & \Phi^ {\mathrm{c}} = T _ {\mathrm{c}} ^ {- 1} \Phi T _ {\mathrm{c}} \end{array}
$$

$$
\begin{array}{r} \pmb {\Gamma} ^ {c} = \pmb {T} _ {c} ^ {- 1} \pmb {\Gamma} \\ \pmb {H} ^ {c} = \pmb {H} \pmb {T} _ {c} \end{array}
$$

式中 $T_{c}$ 的选取准则是使

$$
\boldsymbol {\Phi} ^ {c} = \left[ \begin{array}{c c} \boldsymbol {\Phi} _ {1 1} ^ {c c} & \boldsymbol {\Phi} _ {1 2} ^ {c c} \\ \mathbf {0} & \boldsymbol {\Phi} _ {2 2} ^ {c c} \end{array} \right] \left. \begin{array}{l} \} n _ {c c} \\ \} n - n _ {c c} \end{array} \right.
$$

$$
\boldsymbol {\Gamma} ^ {\mathrm{c}} = \left[ \begin{array}{l} \boldsymbol {\Gamma} _ {1} ^ {\mathrm{c}} \\ \boldsymbol {0} \end{array} \right] \left\{ \begin{array}{l} n _ {\mathrm{c}} \\ n - n _ {\mathrm{c}} \end{array} \right.
$$

这样系统方程成

$$
\left[ \begin{array}{c} \boldsymbol {X} _ {k 1} ^ {\mathrm{c}} \\ \boldsymbol {X} _ {k 2} ^ {\mathrm{c}} \end{array} \right] = \left[ \begin{array}{c c} \boldsymbol {\Phi} _ {1 1} ^ {\mathrm{c}} & \boldsymbol {\Phi} _ {1 2} ^ {\mathrm{c}} \\ \boldsymbol {0} & \boldsymbol {\Phi} _ {2 2} ^ {\mathrm{c}} \end{array} \right] \left[ \begin{array}{c} \boldsymbol {X} _ {(k - 1) 1} ^ {\mathrm{c}} \\ \boldsymbol {X} _ {(k - 1) 2} ^ {\mathrm{c}} \end{array} \right] + \left[ \begin{array}{c} \boldsymbol {I} _ {1} ^ {\mathrm{c}} \\ \boldsymbol {0} \end{array} \right] \boldsymbol {W} _ {k - 1}
$$

即得

子系统1：

$$
\boldsymbol {X} _ {k 1} ^ {\mathrm{c}} = \boldsymbol {\Phi} _ {1 1} ^ {\mathrm{c}} \boldsymbol {X} _ {(k - 1) 1} ^ {\mathrm{c}} + \boldsymbol {\Phi} _ {1 2} ^ {\mathrm{c}} \boldsymbol {X} _ {(k - 1) 2} ^ {\mathrm{c}} + \boldsymbol {\Gamma} _ {1} ^ {\mathrm{c}} \boldsymbol {W} _ {k - 1}\tag{4.2.31}
$$

子系统2：

$$
\boldsymbol {X} _ {k 2} ^ {\mathrm{c}} = \boldsymbol {\Phi} _ {2 2} ^ {\mathrm{c}} \boldsymbol {X} _ {(k - 1) 2} ^ {\mathrm{c}}\tag{4.2.32}
$$

如果子系统 1 是完全随机可控的, 子系统 2 是渐近稳定的, 即其特征值 $\lambda_{i}$ 满足:

$$
\mid \lambda_ {i} (\Phi_ {2 2} ^ {\mathrm{c}}) \mid <   1 \quad i = 1, 2, \dots , n - n _ {\mathrm{c}}\tag{4.2.33}
$$

则称原系统式 $(4.2.28)$ 是完全随机可稳定的。

## 2. 完全随机可检测

所谓完全随机可检测是指:系统经满秩线性变换后被分离成可观测部分和不可观测部分,如果不可观测部分是稳定的,则称系统是完全随机可检测的。

设

$$
\boldsymbol {X} _ {k} = \boldsymbol {T} _ {\mathrm{o}} \boldsymbol {X} _ {k} ^ {\mathrm{o}}\tag{4.2.34}
$$

式中， $T_{0}$ 是某一满秩变换矩阵， $X_{k}^{0}$ 是被分离为可观测部分和不可观测部分的状态。则式(4.2.28)和式(4.2.29)成

$$
\begin{array}{r} \mathbf {T} _ {\mathrm{o}} \mathbf {X} _ {k} ^ {\mathrm{o}} = \boldsymbol {\Phi} \mathbf {T} _ {\mathrm{o}} \mathbf {X} _ {k - 1} ^ {\mathrm{o}} + \boldsymbol {\Gamma} \mathbf {W} _ {k - 1} \\ \mathbf {Z} _ {k} = \mathbf {H} \mathbf {T} _ {\mathrm{o}} \mathbf {X} _ {k} ^ {1} + \mathbf {V} _ {k} \end{array}
$$

即

$$
\boldsymbol {X} _ {k} ^ {\mathrm{o}} = \boldsymbol {\Phi} ^ {\mathrm{o}} \boldsymbol {X} _ {k - 1} ^ {\mathrm{o}} + \boldsymbol {\Gamma} ^ {\mathrm{o}} \boldsymbol {W} _ {k - 1}\tag{4.2.35}
$$

式中

$$
\mathbf {Z} _ {k} = \mathbf {H} ^ {\circ} \mathbf {X} _ {k} ^ {\circ} + \mathbf {V} _ {k}\tag{4.2.36}
$$

$$
\boldsymbol {\Phi} ^ {0} = \boldsymbol {T} _ {0} ^ {- 1} \boldsymbol {\Phi} \boldsymbol {T} _ {0}\tag{4.2.37}
$$

$$
\boldsymbol {\Gamma} ^ {0} = \boldsymbol {T} _ {0} ^ {- 1} \boldsymbol {\Gamma}\tag{4.2.38}
$$

$$
\boldsymbol {H} ^ {\mathrm{o}} = \boldsymbol {H} \boldsymbol {T} _ {\mathrm{o}}\tag{4.2.39}
$$

选择 $T_{0}$ 时使其满足：

$$
\boldsymbol {\Phi} ^ {0} = \left[ \begin{array}{c c} \boldsymbol {\Phi} _ {1 1} ^ {0} & \mathbf {0} \\ \boldsymbol {\Phi} _ {2 1} ^ {0} & \boldsymbol {\Phi} _ {2 2} ^ {0} \end{array} \right] \left. \right\} n _ {o}   \left. \right\} n - n _ {o}\tag{4.2.40}
$$

$$
\boldsymbol {\Gamma} ^ {0} = \left[ \begin{array}{l} \boldsymbol {\Gamma} _ {1} ^ {0} \\ \boldsymbol {\Gamma} _ {2} ^ {0} \end{array} \right] \left. \begin{array}{l} \} n _ {0} \\ \} n - n _ {0} \end{array} \right.\tag{4.2.41}
$$

$$
\begin{array}{r l} \boldsymbol {H} ^ {\circ} = & \left[ \underbrace {\boldsymbol {H} _ {1} ^ {\circ}} _ {n _ {\mathrm{o}}} \quad \underbrace {\boldsymbol {0}} _ {n - n _ {\mathrm{o}}} \right] \end{array}\tag{4.2.42}
$$

这样，式(4.2.28)和式(4.2.29)成

$$
\left[ \begin{array}{l} \boldsymbol {X} _ {k 1} ^ {\mathrm{o}} \\ \boldsymbol {X} _ {k 2} ^ {\mathrm{o}} \end{array} \right] = \left[ \begin{array}{c c} \boldsymbol {\Phi} _ {1 1} ^ {\mathrm{o}} & \boldsymbol {0} \\ \boldsymbol {\Phi} _ {2 1} ^ {\mathrm{o}} & \boldsymbol {\Phi} _ {2 2} ^ {\mathrm{o}} \end{array} \right] \left[ \begin{array}{l} \boldsymbol {X} _ {(k - 1) 1} ^ {\mathrm{o}} \\ \boldsymbol {X} _ {(k - 1) 2} ^ {\mathrm{o}} \end{array} \right] + \left[ \begin{array}{l} \boldsymbol {\Gamma} _ {1} ^ {\mathrm{o}} \\ \boldsymbol {\Gamma} _ {2} ^ {\mathrm{o}} \end{array} \right] \boldsymbol {W} _ {k - 1}
$$

$$
\mathbf {Z} _ {k} = \left[ \begin{array}{l l} \mathbf {H} _ {1} ^ {\mathrm{o}} & \mathbf {0} \end{array} \right] \left[ \begin{array}{l} \mathbf {X} _ {k 1} ^ {\mathrm{o}} \\ \mathbf {X} _ {k 2} ^ {\mathrm{o}} \end{array} \right] + \mathbf {V} _ {k}
$$

从上述两式可分离得

子系统1：

$$
\left. \begin{array}{l} \boldsymbol {X} _ {k 1} ^ {\mathrm{o}} = \boldsymbol {\Phi} _ {1 1} ^ {\mathrm{o}} \boldsymbol {X} _ {(k - 1) 1} ^ {\mathrm{o}} + \boldsymbol {\Gamma} _ {1} ^ {\mathrm{o}} \boldsymbol {W} _ {k - 1} \\ \boldsymbol {Z} _ {k} = \boldsymbol {H} _ {1} ^ {\mathrm{o}} \boldsymbol {X} _ {k 1} ^ {\mathrm{o}} + \boldsymbol {V} _ {k} \end{array} \right\}\tag{4.2.43}
$$

子系统2：

$$
\boldsymbol {X} _ {k 2} ^ {\mathrm{o}} = \boldsymbol {\Phi} _ {2 1} ^ {\mathrm{o}} \boldsymbol {X} _ {(k - 1) 1} ^ {\mathrm{o}} + \boldsymbol {\Phi} _ {2 2} ^ {\mathrm{o}} \boldsymbol {X} _ {(k - 1) 2} ^ {\mathrm{o}} + \boldsymbol {\Gamma} _ {2} ^ {\mathrm{o}} \boldsymbol {W} _ {k - 1}\tag{4.2.44}
$$

显然子系统 1 是独立子系统。

如果子系统 1 完全可观测, 子系统 2 虽然不可观测 ( $Z_{k}$ 只反映 $X_{k1}^{o}$ , 而不反映 $X_{k2}^{o}$ ), 但是渐近稳定的, 即其特征值 $\lambda_{i}$ 满足

$$
\mid \lambda_ {i} (\Phi_ {2 2} ^ {\mathrm{o}}) \mid <   1 \quad i = 1, 2, \dots , n - n _ {\mathrm{o}}\tag{4.2.45}
$$

则称原系统式(4.2.28)和式(4.2.29)是完全随机可检测的。

## 4.2.4 使用充分条件判别滤波稳定性的局限性

由于上述 3 个判别滤波稳定的条件都是充分条件,所以这些条件只能用来判别滤波是稳定的,而不能用来判别滤波是不稳定的。这就是说,如果满足判别条件,则可以肯定滤波是稳定的,但如果不满足这些判别条件,则不能肯定滤波是稳定的,但也不能肯定滤波是不稳定的,即存在判断模糊性。因此,使用上述充分条件判别滤波稳定性是有局限性的。下面通过举例说明之。

例4-2 设线性定常系统的系统方程和量测方程为

$$
\begin{array}{r} X _ {k} = \Phi X _ {k - 1} \\ Z _ {k} = X _ {k} + V _ {k} \end{array}
$$

其中 $X_{k}$ 和 $Z_{k}$ 都是标量， $V_{k}$ 为零均值的白噪声序列，方差为 R=1 。按：① $P_{0}=1$ ; ② $P_{0}=0$ ; ③ $P_{0}>0$ 三种情况判定系统的滤波稳定性，并与解析分析结果作比较。

解 本例中，n=1, H=1, Q=0, $\Gamma=0$ , R=1。

（1）采用充分条件之一，即完全随机可控和完全随机可观测条件判别之。

$$
\begin{array}{r l} \operatorname{rank} [ 0 ] & = 0 \neq n = 1 \\ \operatorname{rank} [ 1 ] & = 1 = n \end{array}
$$

所以系统是随机可观测的,但不是随机可控的,并不满足充分条件之一,因此无法确定滤波稳定,也不能肯定滤波不稳定。

(2) 采用充分条件之二, 即推广型的随机可控和随机可观测条件判别之。

$$
\Lambda (k, 0) = \Phi^ {2 k} P _ {0} + \sum_ {i = 1} ^ {k} \Phi^ {2 (k - i)} 0 = (\Phi^ {k}) ^ {2} P _ {0}
$$

若 $P_0 > 0$ ，即 $P_0 \neq 0$ ，则 $\Lambda(k,0) > 0$ ，即 $\operatorname{rank}[\Lambda(k,0)] = 1 = n$ ，上面已分析得系统是随机可观测的，所以满足充分条件二，可以判定当 $P_0 = 1$ 和 $P_0 > 0$ 时滤波稳定，但当 $P_0 = 0$ 时无法肯定

滤波是否稳定。

(3) 采用充分条件之三, 即完全随机可稳定和完全随机可检测条件判别之。

系统为一般系统,可看做由满秩变换后得到的两个子系统:

子系统 1 的状态恒为零, 它对任意的控制, 状态总能恢复到零, 所以可看做是完全可控系统。

子系统 2 的状态为 $X_{k}$ ，特征值满足：

$$
\mid \lambda - \Phi \mid = 0
$$

解得

$$
\lambda = \Phi
$$

所以，当 $|\lambda| = |\Phi| < 1$ 时，系统完全随机可稳定，并且由于原系统是随机可观测的，所以可肯定系统滤波渐近稳定。而当 $|\lambda| = |\Phi| \geqslant 1$ 时，则无法肯定滤波是否稳定。

(4) 根据解析分析结果判定滤波稳定性。

根据式(2.2.4b)、式(2.2.4c)和式(2.2.4e')得

$$
\begin{array}{r l} & P _ {k / k - 1} = \Phi^ {2} P _ {k - 1} \\ & K _ {k} = \frac {P _ {k / k - 1}}{P _ {k / k - 1} + 1} \\ & P _ {k} = (1 - K _ {k}) P _ {k / k - 1} = (1 - \frac {P _ {k / k - 1}}{P _ {k / k - 1} + 1}) P _ {k / k - 1} = K _ {k} \end{array}
$$

当 $k = 1$ 时

$$
\begin{array}{r l} & P _ {1 / 0} = \Phi^ {2} P _ {0} \\ & P _ {1} = K _ {1} = \frac {\Phi^ {2} P _ {0}}{\Phi^ {2} P _ {0} + 1} \end{array}
$$

当 $k = 2$ 时，

$$
\begin{array}{r l} & P _ {2 / 1} = \Phi^ {2} P _ {1} = \frac {\Phi^ {4} P _ {0}}{\Phi^ {2} P _ {0} + 1} \\ & P _ {2} = K _ {2} = \frac {\Phi^ {4} P _ {0}}{\Phi^ {2} P _ {0} + 1} (\frac {\Phi^ {4} P _ {0}}{\Phi^ {2} P _ {0} + 1} + 1) ^ {2} = \frac {\Phi^ {4} P _ {0}}{\Phi^ {4} P _ {0} + \Phi^ {2} P _ {0} + 1} \end{array}
$$

当 $k = 3$ 时

$$
\begin{array}{r l} & P _ {3 / 2} = \Phi^ {2} P _ {2} = \frac {\Phi^ {6} P _ {0}}{\Phi^ {4} P _ {0} + \Phi^ {2} P _ {0} + 1} \\ & P _ {3} = K _ {3} = \frac {\Phi^ {6} P _ {0}}{\Phi^ {4} P _ {0} + \Phi^ {2} P _ {0} + 1} (\frac {\Phi^ {6} P _ {0}}{\Phi^ {4} P _ {0} + \Phi^ {2} P _ {0} + 1} + 1) ^ {- 1} = \\ & \frac {\Phi^ {6} P _ {0}}{\Phi^ {6} P _ {0} + \Phi^ {4} P _ {0} + \Phi^ {2} P _ {0} + 1} \\ & \dots \dots \end{array}
$$

从上述结果可归纳出 $P_{k / k - 1}$ 和 $P_{k}$ 的通项公式为

$$
\begin{array}{r l} & P _ {k / k - 1} = \frac {\Phi^ {2 k} P _ {0}}{\Phi^ {2 (k - 1)} P _ {0} + \Phi^ {2 (k - 2)} P _ {0} + \cdots + \Phi^ {2} P _ {0} + 1} \\ & P _ {k} = K _ {k} = \frac {\Phi^ {2 k} P _ {0}}{\Phi^ {2 k} P _ {0} + \Phi^ {2 (k - 1)} P _ {0} + \cdots + \Phi^ {2} P _ {0} + 1} \end{array}
$$

1) 当 $|\Phi| \neq 1$ 时

$$
\begin{array}{r l} P _ {k} = K _ {k} & = \frac {\Phi^ {2 k} P _ {0}}{P _ {0} [ \Phi^ {2 k} + \Phi^ {2 (k - 1)} + \cdots + \Phi^ {2} + 1 ] + 1 - P _ {0}} = \frac {\Phi^ {2 k} P _ {0}}{P _ {0} \frac {1 - \Phi^ {2 (k + 1)}}{1 - \Phi^ {2}} + 1 - P _ {0}} = \\ & \frac {(1 - \Phi^ {2}) \Phi^ {2 k} P _ {0}}{[ 1 - \Phi^ {2 (k + 1)} ] P _ {0} + (1 - P _ {0}) (1 - \Phi^ {2})} \end{array}
$$

① 当 $|\Phi| < 1$ 时

$$
\lim _ {k \to \infty} P _ {k} = \lim _ {k \to \infty} K _ {k} = 0
$$

② 当 $|\Phi| > 1$ 时

$$
\lim _ {k \rightarrow \infty} P _ {k} = \lim _ {k \rightarrow \infty} K _ {k} = \lim _ {k \rightarrow \infty} \frac {(1 - \Phi^ {2}) P _ {0}}{(\Phi^ {- 2 k} - \Phi^ {2}) P _ {0} + (1 - P _ {0}) (1 - \Phi^ {2}) \Phi^ {- 2 k}} = 1 - \frac {1}{\Phi^ {2}}
$$

2) 当 $|\Phi| = 1$ 时

$$
\begin{array}{r l} & P _ {k} = K _ {k} = \frac {P _ {0}}{k P _ {0} + 1} \\ & \lim _ {k \to \infty} P _ {k} = \lim _ {k \to \infty} K _ {k} = 0 \end{array}
$$

从上述分析可看出，不管 $\Phi$ 为何值，也不管 $P_{0}$ 为何值， $P_{k}$ 随着 k 的增大逐渐不受 $P_{0}$ 的影响，当 $k \to \infty$ 时， $P_{k}$ 与 $P_{0}$ 无关。

下面再分析 $X_{k}$ 与 $X_0$ 的关系：

由式(2.2.4a)和式(2.2.4b)得

$$
\begin{array}{r l} & \hat {X} _ {k} = \Phi X _ {k - 1} + K _ {k} (Z _ {k} - \Phi \hat {X} _ {k - 1}) = \Phi (1 - K _ {k}) \hat {X} _ {k - 1} + K _ {k} Z _ {k} \\ & \hat {X} _ {k - 1} = \Phi (1 - K _ {k - 1}) \hat {X} _ {k - 2} + K _ {k - 1} Z _ {k - 1} \\ & \hat {X} _ {k - 2} = \Phi (1 - K _ {k - 2}) \hat {X} _ {k - 3} + K _ {k - 2} Z _ {k - 2} \\ & \dots \dots \\ & \hat {X} _ {1} = \Phi (1 - K _ {1}) \hat {X} _ {0} + K _ {1} Z _ {1} \end{array}
$$

所以

而

$$
\begin{array}{r l} & {\hat {X} _ {k} = \Phi (1 - K _ {k}) \Phi (1 - K _ {k - 1}) \Phi (1 - K _ {k - 2}) \dots \Phi (1 - K _ {1}) \hat {X} _ {0} +} \\ & {\quad (\text {由} Z _ {1}, Z _ {2}, \dots , Z _ {k} \text {组成的修正项}) =} \\ & {\quad \Phi^ {k} (1 - K _ {k}) (1 - K _ {k - 1}) \dots (1 - K _ {1}) \hat {X} _ {0} +} \\ & {\quad (\text {由} Z _ {1}, Z _ {2}, \dots , Z _ {k} \text {组成的修正项})} \\ & {1 - K _ {k} = \frac {\Phi^ {2 (k - 1)} P _ {0} + \Phi^ {2 (k - 2)} P _ {0} + \cdots + \Phi^ {2} P _ {0} + 1}{\Phi^ {2 k} P _ {0} + \Phi^ {2 (k - 1)} P _ {0} + \cdots + \Phi^ {2} P _ {0} + 1}} \\ & {1 - K _ {k - 1} = \frac {\Phi^ {2 (k - 2)} P _ {0} + \Phi^ {2 (k - 3)} P _ {0} + \cdots \Phi^ {2} P _ {0} + 1}{\Phi^ {2 (k - 1)} P _ {0} + \Phi^ {2 (k - 2)} P _ {0} + \cdots + \Phi^ {2} P _ {0} + 1}} \\ & {\qquad \dots \dots} \\ & {1 - K _ {1} = \frac {1}{\Phi^ {2} P _ {0} + 1}} \end{array}
$$

所以

$$
(1 - K _ {k}) (1 - K _ {k - 1}) \dots (1 - K _ {1}) = \frac {1}{\Phi^ {2 k} P _ {0} + \Phi^ {2 (k - 1)} P _ {0} + \cdots + \Phi^ {2} P _ {0} + 1}
$$

(1) 当 $P_{0} \neq 0$ 时

$$
\hat {X} _ {k} = \frac {\Phi^ {k}}{[ \Phi^ {2 k} + \Phi^ {2 (k - 1)} + \cdots + \Phi^ {2} + 1 ] P _ {0} + 1 - P _ {0}} \hat {X} _ {0} +
$$

(由 $Z_{1}, Z_{2}, \cdots, Z_{k}$ 组成的修正项)

1) 当 $|\Phi| \neq 1$ 时

$$
\begin{array}{r l} \hat {X} _ {k} & = \frac {\Phi^ {k}}{\frac {1 - \Phi^ {2 (k + 1)}}{1 - \Phi^ {2}} P _ {0} + 1 - P _ {0}} \hat {X} _ {0} + (Z _ {1}, Z _ {2}, \dots , Z _ {k} \text {组成的修正项}) = \\ & \frac {\Phi^ {k} (1 - \Phi^ {2})}{[ 1 - \Phi^ {2 (k + 1)} ] P _ {0} + (1 - P _ {0}) (1 - \Phi^ {2})} \hat {X} _ {0} + \\ & (Z _ {1}, Z _ {2}, \dots , Z _ {k} \text {组成的修正项}) \end{array}
$$

① 当 $|\Phi| < 1$ 时

$$
\lim _ {k \to \infty} \hat {X} _ {k} = 0 \cdot \hat {X} _ {0} + (Z _ {1}, Z _ {2}, \dots , Z _ {k} \text { 组成的修正项 })
$$

② 当 $|\Phi| > 1$ 时

$$
\begin{array}{r l} & {\underset {k \to \infty} {\lim} \hat {X} _ {k} = \underset {k \to \infty} {\lim} \frac {1 - \Phi^ {2}}{(\Phi^ {- k} - \Phi^ {k + 2}) P _ {0} - (1 - P _ {0}) (1 - \Phi^ {2}) \Phi^ {- k}} \hat {X} _ {0} +} \\ & {\qquad (Z _ {1}, Z _ {2}, \dots , Z _ {k} \text {组成的修正项}) =} \end{array}
$$

$0 \cdot \hat{X}_{0} + (Z_{1}, Z_{2}, \cdots, Z_{k}$ 组成的修正项）

2) 当 $|\Phi| = 1$ 时

$$
\begin{array}{r l} \hat {X} _ {k} & = \frac {1}{(k + 1) P _ {0} + 1 - P _ {0}} \hat {X} _ {0} + (Z _ {1}, Z _ {2}, \dots , Z _ {k} \text { 组成的修正项 }) \\ & \quad \lim _ {k \to \infty} \hat {X} _ {k} = 0 \cdot \hat {X} _ {0} + (Z _ {1}, Z _ {2}, \dots , Z _ {k} \text { 组成的修正项 }) \end{array}
$$

(2) 当 $P_{0}=0$ 时

$$
\hat {X} _ {k} = \Phi^ {k} \hat {X} _ {0} + (Z _ {1}, Z _ {2}, \dots , Z _ {k} \mathrm{组成的修正项})
$$

1) 当 $|\Phi| < 1$ 时

$$
\lim _ {k \to \infty} \hat {X} _ {k} = 0 \cdot \hat {X} _ {0} + (Z _ {1}, Z _ {2}, \dots , Z _ {k} \text { 组成的修正项 })
$$

2) 当 $|\Phi| = 1$ 时

$$
\lim _ {k \to \infty} \hat {X} _ {k} = \hat {X} _ {0} + (Z _ {1}, Z _ {2}, \dots , Z _ {k} \text { 组   成   的   修   正   项 })
$$

3) 当 $|\Phi| > 1$ 时

$$
\lim _ {k \to \infty} \hat {X} _ {k} = \infty \cdot \hat {X} _ {0} + (Z _ {1}, Z _ {2}, \dots , Z _ {k} \text { 组成的修正项 })
$$

从上述分析可看出，当 $P_0 > 0$ 或 $P_0 = 0$ 且 $|\Phi| < 1$ 时，随着滤波步数的增加， $\hat{X}_0$ 对 $\hat{X}_k$ 的影响将逐渐消失，而当 $P_0 = 0$ ，且 $|\Phi| \geqslant 1$ 时， $\hat{X}_0$ 对 $\hat{X}_k$ 的影响始终存在。

综合上述分析,可得四种方法对滤波稳定性的判别结果,如表 4.2.1 所列。

表 4.2.1 四种方法对滤波稳定性的判别结果

<table><tr><td rowspan="2">滤波稳定/条件方法</td><td colspan="3"> $P_0 > 0$ </td><td colspan="3"> $P_0 = 0$ </td></tr><tr><td> $|\Phi| < 1$ </td><td> $|\Phi| = 1$ </td><td> $|\Phi| > 1$ </td><td> $|\Phi| < 1$ </td><td> $|\Phi| = 1$ </td><td> $|\Phi| > 1$ </td></tr><tr><td>充分条件之一</td><td>?</td><td>?</td><td>?</td><td>?</td><td>?</td><td>?</td></tr><tr><td>充分条件之二</td><td>√</td><td>√</td><td>√</td><td>?</td><td>?</td><td>?</td></tr><tr><td>充分条件之三</td><td>√</td><td>?</td><td>?</td><td>√</td><td>?</td><td>?</td></tr><tr><td>解析分析</td><td>√</td><td>√</td><td>√</td><td>√</td><td>×</td><td>×</td></tr></table>

说明：？表示无法确定滤波稳定还是不稳定；

√ 表示确定滤波稳定；

× 表示确定滤波不稳定。

从表 4.2.1 可看出, 使用充分条件判别滤波稳定性是有局限性的, 当条件满足时, 可以肯定滤波稳定, 但当条件不满足时, 则无法肯定究竟滤波稳定还是不稳定。

## 4.3 适用于惯导系统的滤波稳定判别条件

4.2 节介绍的滤波稳定性判别条件都是充分条件,越是采用宽的条件来判别,越能接近真实地了解滤波器的稳定情况。据此,针对惯导系统的实际情况,提出如下滤波稳定判别条件:

## 定理4.5

如果系统是完全随机可检测的，且 $P_0 > 0(P(0) > 0)$ ，则卡尔曼滤波器是滤波稳定的。

下面对上述定理作简要证明。

设系统和量测如式(4.2.28)和式(4.2.29)所示,式中 $W_{k}$ 和 $V_{k}$ 是互相独立的零均质白噪声序列, $V_{k}$ 的方差阵为R,且 $P_{0}>0$ 。系统是完全随机可检测的,所以系统经满秩矩阵 $T_{0}$ 作相似变换后,得代数等价系统和量测为

$$
\boldsymbol {X} _ {k} ^ {\mathrm{o}} = \boldsymbol {\Phi} ^ {\mathrm{o}} \boldsymbol {X} _ {k - 1} ^ {\mathrm{o}} + \boldsymbol {\Gamma} ^ {\mathrm{o}} \boldsymbol {W} _ {k - 1}\tag{4.2.35}
$$

$$
\mathbf {Z} _ {k} = \mathbf {H} ^ {\mathrm{o}} \mathbf {X} _ {k} ^ {\mathrm{o}} + \mathbf {V} _ {k}\tag{4.2.36}
$$

即

子系统1：

$$
\left. \begin{array}{l} \boldsymbol {X} _ {k 1} ^ {\mathrm{o}} = \boldsymbol {\Phi} _ {1 1} ^ {\mathrm{o}} \boldsymbol {X} _ {(k - 1) 1} ^ {\mathrm{o}} + \boldsymbol {\Gamma} _ {1} ^ {\mathrm{o}} \boldsymbol {W} _ {k - 1} \\ \boldsymbol {Z} _ {k} = \boldsymbol {H} _ {1} ^ {\mathrm{o}} \boldsymbol {X} _ {k 1} ^ {\mathrm{o}} + \boldsymbol {V} _ {k} \end{array} \right\}\tag{4.2.43}
$$

子系统2：

$$
\boldsymbol {X} _ {k 2} ^ {\mathrm{o}} = \boldsymbol {\Phi} _ {2 1} ^ {\mathrm{o}} \boldsymbol {X} _ {(k - 1) 1} ^ {\mathrm{o}} + \boldsymbol {\Phi} _ {2 2} ^ {\mathrm{o}} \boldsymbol {X} _ {(k - 1) 2} ^ {\mathrm{o}} + \boldsymbol {\Gamma} _ {2} ^ {\mathrm{o}} \boldsymbol {W} _ {k - 1}\tag{4.2.44}
$$

详细变换关系见 4.2.3 节所述。其中子系统 1 是独立系统，该子系统随机可观测，它的状态 $X_{k1}^{o}$ 通过 $\Phi_{21}^{o}$ 影响子系统 2。子系统 2 本身是渐近稳定的。即

$$
\mid \lambda_ {i} (\Phi_ {2 2} ^ {\circ}) \mid <   1 \quad i = 1, 2, \dots , n - n _ {\circ}\tag{4.2.45}
$$

如果对系统作滤波估计,按式(4.1.2)有

$$
\hat {\boldsymbol {X}} _ {k} ^ {\mathrm{o}} = \left(\boldsymbol {I} - \boldsymbol {K} _ {k} ^ {\mathrm{o}} \boldsymbol {H} ^ {\mathrm{o}}\right) \boldsymbol {\Phi} ^ {\mathrm{o}} \hat {\boldsymbol {X}} _ {k - 1} ^ {\mathrm{o}} + \boldsymbol {K} _ {k} ^ {\mathrm{o}} \boldsymbol {Z} _ {k}\tag{4.3.1}
$$

令

$$
\boldsymbol {P} _ {k} ^ {\mathrm{o}} \stackrel {\text {def}} {=} \left[ \begin{array}{l l} \boldsymbol {P} _ {k 1 1} ^ {\mathrm{o}} & \boldsymbol {P} _ {k 1 2} ^ {\mathrm{o}} \\ \boldsymbol {P} _ {k 2 1} ^ {\mathrm{o}} & \boldsymbol {P} _ {k 2 2} ^ {\mathrm{o}} \end{array} \right] \left. \right\} n _ {\mathrm{o}}   \left. \right\} n - n _ {\mathrm{o}}
$$

根据式 $(2.2.4c')$ 有

$$
\mathbf {K} _ {k} ^ {\mathrm{o}} = \left[ \begin{array}{c} \mathbf {K} _ {k 1} ^ {\mathrm{o}} \\ \mathbf {K} _ {k 2} ^ {\mathrm{o}} \end{array} \right] = \left[ \begin{array}{c} \mathbf {P} _ {k 1 1} ^ {\mathrm{o}} \mathbf {H} _ {1} ^ {\mathrm{oT}} \mathbf {R} ^ {- 1} \\ \mathbf {P} _ {k 2 1} ^ {\mathrm{o}} \mathbf {H} _ {1} ^ {\mathrm{oT}} \mathbf {R} ^ {- 1} \end{array} \right] \left. \begin{array}{l} \} n _ {\mathrm{o}} \\ \} n - n _ {\mathrm{o}} \end{array} \right.
$$

$$
(I - K _ {k} ^ {\mathrm{o}} H ^ {\mathrm{o}}) \Phi^ {\mathrm{o}} = \left[ \begin{array}{c c} (I - K _ {k 1} ^ {\mathrm{o}} H _ {1} ^ {\mathrm{o}}) \Phi_ {1 1} ^ {\mathrm{o}} & 0 \\ \Phi_ {2 1} ^ {\mathrm{o}} - K _ {k 2} ^ {\mathrm{o}} H _ {1} ^ {\mathrm{o}} \Phi_ {1 1} ^ {\mathrm{o}} & \Phi_ {2 2} ^ {\mathrm{o}} \end{array} \right] \} n _ {\mathrm{o}}\tag{4.3.2}
$$

上式结合式(4.3.1)说明可以把滤波器看做是两个子系统。 $(I-K_{k1}^{o}H_{1}^{o})\Phi_{11}^{o}$ 表示子系统1和量测 $Z_{k}$ 组成的滤波器子系统1的转移矩阵；由于子系统1是随机可观测的，且 $(P_{0})_{11}>0$ ，根据滤波稳定充分条件之二，知滤波器子系统1是渐近稳定的。 $\Phi_{22}^{o}$ 也是滤波器子系统2的转移矩阵，所以滤波器子系统2也是渐近稳定的，它还接收滤波器子系统1的状态 $X_{k1}^{o}$ 通过 $(\Phi_{21}^{o}-$

$K_{k2}^{o}H_{1}^{o}\Phi_{11}^{o}$ ）的输入。因为滤波器子系统1是渐近稳定的，所以这个输入也不影响滤波器子系统2的渐近稳定性质。因此整个滤波器是渐近稳定的。

式(4.3.1)虽然表示系统式(4.2.35)和式(4.2.36)的滤波器,但根据代数等价系统的性质,同样可以证明式(4.2.28)和式(4.2.29)的滤波器是渐近稳定的。即根据式(4.2.34)有

$$
\begin{array}{r l} \boldsymbol {P} _ {k} ^ {\mathrm{o}} & = E \big [ (\boldsymbol {X} _ {k} ^ {\mathrm{o}} - \hat {\boldsymbol {X}} _ {k} ^ {\mathrm{o}}) (\boldsymbol {X} _ {k} ^ {\mathrm{o}} - \hat {\boldsymbol {X}} _ {k} ^ {\mathrm{o}}) ^ {\mathrm{T}} \big ] = \\ & \boldsymbol {T} _ {\mathrm{o}} ^ {- 1} E \big [ (\boldsymbol {X} _ {k} - \hat {\boldsymbol {X}} _ {k}) (\boldsymbol {X} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \big ] \boldsymbol {T} _ {\mathrm{o}} ^ {- \mathrm{T}} = \boldsymbol {T} _ {\mathrm{o}} ^ {- 1} \boldsymbol {P} _ {k} \boldsymbol {T} _ {\mathrm{o}} ^ {- \mathrm{T}} \end{array}\tag{4.3.3}
$$

根据式 $(4.3.3)$ 和式 $(4.2.39)$ 有

$$
\boldsymbol {K} _ {k} ^ {\mathrm{o}} = \boldsymbol {P} _ {k} ^ {\mathrm{o}} \boldsymbol {H} ^ {\mathrm{oT}} \boldsymbol {R} ^ {- 1} = \boldsymbol {T} _ {\mathrm{o}} ^ {- 1} \boldsymbol {P} _ {k} \boldsymbol {T} _ {\mathrm{o}} ^ {- \mathrm{T}} (\boldsymbol {H T} _ {\mathrm{o}}) ^ {\mathrm{T}} \boldsymbol {R} ^ {- 1} = \boldsymbol {T} _ {\mathrm{o}} ^ {- 1} \boldsymbol {P} _ {k} \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {R} ^ {- 1} = \boldsymbol {T} _ {\mathrm{o}} ^ {- 1} \boldsymbol {K} _ {k}\tag{4.3.4}
$$

式(4.3.3)和式(4.3.4)中的 $P_{k}$ 和 $K_{k}$ 就是原系统进行滤波的估计均方误差阵和增益矩阵。再将式(4.3.4)、式(4.2.37)和式(4.2.39)代入式(4.3.2)得

$$
(I - K _ {k} ^ {\circ} H ^ {\circ}) \Phi^ {\circ} = T _ {\mathrm{o}} ^ {- 1} (I - K _ {k} H) \Phi T _ {\mathrm{o}}\tag{4.3.5}
$$

上式说明,经变换后的系统的滤波器转移矩阵是原系统的滤波器转移矩阵的相似变换。根据系统相似变换后的特征值不变的性质,说明如果变换后系统的滤波器是渐近稳定的,则原系统的滤波器也是渐近稳定的。

下面证明 $P_{k}$ 渐近地不受 $P_{0}$ 的影响。

如果 $P_{k}^{1}$ 和 $P_{k}^{2}$ 各是两个不同初始估计均方误差阵 ( $P_{0}^{1}$ 和 $P_{0}^{2}$ ) 条件下的 k 时刻估计均方误差阵，令

$$
\begin{array}{l} \delta \boldsymbol {P} _ {k} = \boldsymbol {P} _ {k} ^ {1} - \boldsymbol {P} _ {k} ^ {2} \\ \delta \boldsymbol {P} _ {0} = \boldsymbol {P} _ {0} ^ {1} - \boldsymbol {P} _ {0} ^ {2} \end{array}
$$

利用矩阵求逆公式和式(2.2.4c)有

$$
\begin{array}{r l} & {(I - K _ {k} H) ^ {- 1} = [ I - P _ {k / k - 1} H ^ {\mathrm{T}} (H P _ {k / k - 1} H ^ {\mathrm{T}} + R) ^ {- 1} H ] ^ {- 1} =} \\ & {\qquad \qquad I + P _ {k / k - 1} H ^ {\mathrm{T}} (H P _ {k / k - 1} H ^ {\mathrm{T}} + R - H P _ {k / k - 1} H ^ {\mathrm{T}}) ^ {- 1} H =} \\ & {\qquad \qquad I + P _ {k / k - 1} H ^ {\mathrm{T}} R ^ {- 1} H} \end{array}\tag{4.3.6}
$$

利用式(4.3.6)和式 $(2.2.4e^{\prime})$ 以及对称矩阵相乘所得积如果是对称矩阵,则乘法满足交换律的性质,得

$$
\begin{array}{r l} & {(I - K _ {k} ^ {2} H) ^ {- 1} \delta P _ {k} (I - K _ {k} ^ {1} H) ^ {- T} = (I - K _ {k} ^ {2} H) ^ {- 1} [ (I - K _ {k} ^ {1} H) P _ {k / k - 1} ^ {1} -} \\ & {(I - K _ {k} ^ {2} H) P _ {k / k - 1} ^ {2} ] (I - K _ {k} ^ {1} H) ^ {- T} =} \\ & {(I - K _ {k} ^ {2} H) ^ {- 1} P _ {k / k - 1} ^ {1} - P _ {k / k - 1} ^ {2} (I - K _ {k} ^ {1} H) ^ {- T} =} \\ & {(I + P _ {k / k - 1} ^ {2} H ^ {\mathrm{T}} R ^ {- 1} H) P _ {k / k - 1} ^ {1} - P _ {k / k - 1} ^ {2} (I + P _ {k / k - 1} ^ {1} H ^ {\mathrm{T}} R ^ {- 1} H) ^ {\mathrm{T}} =} \\ & {P _ {k / k - 1} ^ {1} - P _ {k / k - 1} ^ {2} = \Phi (P _ {k - 1} ^ {1} - P _ {k - 1} ^ {2}) \Phi^ {\mathrm{T}} = \Phi \delta P _ {k - 1} \Phi^ {\mathrm{T}}} \end{array}
$$

所以

$$
\delta \boldsymbol {P} _ {k} = (\boldsymbol {I} - \boldsymbol {K} _ {k} ^ {2} \boldsymbol {H}) \boldsymbol {\Phi} \delta \boldsymbol {P} _ {k - 1} \boldsymbol {\Phi} ^ {\mathrm{T}} (\boldsymbol {I} - \boldsymbol {K} _ {k} ^ {1} \boldsymbol {H}) ^ {\mathrm{T}} = \boldsymbol {\Psi} _ {k, k - 1} ^ {2} \delta \boldsymbol {P} _ {k - 1} \boldsymbol {\Psi} _ {k, k - 1} ^ {\mathrm{T}}\tag{4.3.7}
$$

式中 $\Psi_{k,k - 1}^{1}$ 和 $\Psi_{k,k - 1}^{2}$ 各表示在 $P_0^1$ 和 $P_0^2$ 条件下的滤波器一步转移矩阵。利用式(4.3.7)有

$$
\delta \boldsymbol {P} _ {k} = \boldsymbol {\Psi} _ {k, 0} ^ {2} \delta \boldsymbol {P} _ {0} \boldsymbol {\Psi} _ {k, 0} ^ {1 T}\tag{4.3.8}
$$

式中 $\Psi_{k,0}^{1}$ 和 $\Psi_{k,0}^{2}$ 各表示在 $P_{0}^{1}$ 和 $P_{0}^{2}$ 条件下滤波器从零时刻到 k 时刻的转移矩阵。

因为滤波器是渐近稳定的,所以有

$$
\begin{array}{l} \lim _ {k \to \infty} \| \boldsymbol {\Psi} _ {k, 0} ^ {1} \| = 0 \\ \lim _ {k \to \infty} \| \boldsymbol {\Psi} _ {k, 0} ^ {2} \| = 0 \end{array}
$$

式中 $\| \cdot \|$ 为范数。因此有

$$
\lim _ {k \rightarrow \infty} \| \delta \boldsymbol {P} _ {k} \| = 0
$$

这就证明了 $P_{k}$ 渐近地不受 $P_{0}$ 的影响。

惯性导航系统采用卡尔曼滤波，常不满足随机可控或随机可稳定的条件。而状态向量的初值都是随机向量，初始估计均方误差阵的对角线元素一般都是不为零的正数，因此， $P_{0}>0$ 这一条件一般都能满足。这样，利用 $P_{0}>0$ 和随机可检测来判别滤波器是否滤波稳定，关键在于系统是否随机可检测。这种判别方法的特点还在于：即使系统不是随机可检测的，仍可以从不可检测的性质中看出不可检测的原因和对滤波稳定的影响，从而得出滤波器是否仍能使用的结论。

## 习题

4-1 设定常系统的系统噪声方差阵正定，证明：如果系统满足式(4.2.14)，则必满足式(4.2.16)，即定常系统随机可控阵中可免去 $Q^{\frac{1}{2}}$ 。

4-2 设时变系统的系统噪声方差阵正定，证明：如果系统满足下式

$$
\sum_ {i = k - N + 1} ^ {k} \boldsymbol {\Phi} _ {k, i} \boldsymbol {\Gamma} _ {i - 1} \boldsymbol {\Gamma} _ {i - 1} ^ {\mathrm{T}} \boldsymbol {\Phi} _ {k, i} ^ {\mathrm{T}} > \mathbf {0}
$$

则必满足式 $(4.2.11)$ ，即可用该式代替随机可控阵。

4-3 设系统和量测中各参数为： $\Phi = a, \Gamma = H = Q = R = 1$ ，求 $P$ 和 $K$ ，并证明：

$$
\mid (1 - K H) \Phi \mid <   1
$$

4-4 判别例2-15中系统是随机可观测的，并说明可观测的物理意义。

# 第五章 滤波系统的校正

## 5.1 概述

对确定性线性系统,如果控制的指标函数是二次型的,则可以求得最优控制规律。下面具体说明之。

设线性系统为

$$
\dot {\boldsymbol {X}} (t) = \boldsymbol {F} (t) \boldsymbol {X} (t) + \boldsymbol {E} (t) \boldsymbol {u} (t)\tag{5.1.1}
$$

式中， $X(t)$ 为 n 维状态向量， $u(t)$ 为 r 维控制向量。

给定初始条件 $X(0)$ ，要求选择最优控制向量 $u^{*}(t)$ ，使下列二次型指标函数为最小：

$$
J = \int_ {t _ {0}} ^ {t} [ \mathbf {X} ^ {\mathrm{T}} (t) \mathbf {Q} ^ {0} (t) \mathbf {X} (t) + \mathbf {u} ^ {\mathrm{T}} (t) \mathbf {R} ^ {0} (t) \mathbf {u} (t) ] \mathrm{d} t + \mathbf {X} ^ {\mathrm{T}} (t _ {f}) \mathbf {S X} (t _ {f})\tag{5.1.2}
$$

式中， $Q^{0}(t)$ 和 S 为 $n \times n$ 的权矩阵， $R^{0}(t)$ 为 $r \times r$ 的权矩阵。

二次型性能指标函数的物理意义可解释为: 如果 X 为误差向量, 则 $\boldsymbol{X}^{\mathrm{T}}(t_{f})\boldsymbol{S}\boldsymbol{X}(t_{f})$ 表示终端误差指标, 积分项中的两项各表示过程的误差指标和控制能量指标。要求 J 最小, 就是要终端误差、过程误差和控制量的二次型加权和为最小。权矩阵 $Q^{0}(t)$ , $R^{0}(t)$ 和 S 根据要求确定。 $Q^{0}(t)$ 和 $R^{0}(t)$ 的时变性表示不同时间的要求不同, 但要求 $Q^{0}(t)$ 和 S 是非负定阵, $R^{0}(t)$ 是正定阵。在很多实际问题中, 它们都是元素为正的对角阵。

用极大值原理或动态规划等方法都可以求解上述最优控制问题,得到最优控制规律为

$$
\boldsymbol {u} ^ {*} (t) = - \boldsymbol {R} ^ {0} (t) ^ {- 1} \boldsymbol {E} ^ {\mathrm{T}} (t) \boldsymbol {P} ^ {0} (t) \boldsymbol {X} (t)\tag{5.1.3}
$$

上式说明最优控制量 $\boldsymbol{u}^{*}(t)$ 与 $\boldsymbol{X}(t)$ 成线性关系。为求 $\boldsymbol{u}^{*}(t)$ ，必须先知道 $\boldsymbol{X}(t)$ 。式中 $\boldsymbol{P}^{0}(t)$ 是下列黎卡蒂方程的解：

$$
\dot {\boldsymbol {P}} ^ {0} (t) = - \boldsymbol {P} ^ {0} (t) \boldsymbol {F} (t) - \boldsymbol {F} ^ {\mathrm{T}} (t) \boldsymbol {P} ^ {0} (t) + \boldsymbol {P} ^ {0} (t) \boldsymbol {E} (t) \boldsymbol {R} ^ {0} (t) ^ {- 1} \boldsymbol {E} ^ {\mathrm{T}} (t) \boldsymbol {P} ^ {0} (t) - \boldsymbol {Q} ^ {0} (t)\tag{5.1.4}
$$

式中 $P^{0}(t)$ 为对称矩阵, 方程的边界条件为

$$
\boldsymbol {P} ^ {0} (t _ {f}) = \boldsymbol {S}
$$

在已知 $X(t)$ 的条件下,二次型性能指标最优控制问题最终归结为求解 $P^{0}(t)$ 的黎卡蒂非线性微分方程问题。

如果系统是随机过程,并且状态向量不能直接得到,而需要用卡尔曼滤波方法来估计,则最优控制问题和最优估计问题就交叉在一起了,解决的办法如下。

定理 5.1(分离定理)

对二次型性能指标的最优控制问题,若求解控制和估计以对方为已知条件,则可以分别按最优控制和最优估计两个独立问题来处理,即,求取最优控制时,可以认为状态变量是已知的,求取最优估计时,将控制项取作已知控制项,最后,用状态估计作为最优控制中的已知状态,就得到随机线性动态系统的最优控制。这种将最优控制与最优估计分别考虑的原理称为分

离定理。

## 5.2 离散系统的分离定理

设系统和量测为

$$
\boldsymbol {X} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {X} _ {k} + \boldsymbol {B} _ {k} \boldsymbol {u} _ {k} + \boldsymbol {W} _ {k}\tag{5.2.1}
$$

$$
\mathbf {Z} _ {k} = \mathbf {H} _ {k} \mathbf {X} _ {k} + \mathbf {V} _ {k}\tag{5.2.2}
$$

式中， $\{W_{k}\}$ 和 $\{V_{k}\}$ 为互不相关且均值为零的正态白噪声序列，方差分别为 $Q_{k}$ 和 $R_{k}$ 。状态向量 $X_{k}$ 的初值为 $X_{0}$ 。 $X_{0}$ 也服从正态分布，并与 $W_{k}$ 和 $V_{k}$ 互不相关。

由于过程随机,按式(5.1.2)确定指标函数是无意义的,所以随机线性动态系统的二次型性能指标采用集平均值,即 $u_{0}^{*},u_{1}^{*},\cdots,u_{N-1}^{*}$ 使

$$
J _ {N} = E \{\sum_ {k = 0} ^ {N - 1} [ X _ {k} ^ {\mathrm{T}} Q _ {k} ^ {0} X _ {k} + u _ {k} ^ {\mathrm{T}} R _ {k} ^ {0} u _ {k} ] + X _ {N} ^ {\mathrm{T}} S X _ {N} \}\tag{5.2.3}
$$

达到最小。下面用动态规划来求这个最优控制规律。

按动态规划,上述问题就是在初值为 $X_{0}$ 条件下求 N 级决策过程。选取 u 的方法是先找出最后一级选 $u_{N-1}$ 的规律,使这最后一级为最优然后逐级倒推,使每一级都最优,从而使整个过程都是最优的。

问题分两步讨论。首先假定可以得到 $X_{k}$ 值，推导出与 $X_{k}$ 成线性函数的最优控制规律，然后假定得不到 $X_{k}$ 值，将 $X_{k}$ 的最优估计代入控制规律中，同样证明在这最优估计的条件下是最优的。由此证明了分离定理。

1. 在可以得到状态向量 $X_{k}$ 值的条件下

在该条件下量测方程式(5.2.2)简化为

$$
\mathbf {Z} _ {k} = \mathbf {X} _ {k}
$$

$X_{0}$ 为随机向量。这里先令 $X_{0}$ 为随机样本空间内的某一个具体样本，则指标函数中的 $X_{0}$ 项可以不取均值而单独列写。将 $X_{0}$ 条件下的最小指标函数用 $W\{X_{0}\}$ 来表示，则

$$
\begin{array}{r l} W \{\mathbf {X} _ {0} \} & = \min _ {\mathbf {u} _ {0}, \dots , \mathbf {u} _ {N - 1}} \{\mathbf {X} _ {0} ^ {\mathrm{T}} \mathbf {Q} _ {0} ^ {0} \mathbf {X} _ {0} + \mathbf {u} _ {0} ^ {\mathrm{T}} \mathbf {R} _ {0} ^ {0} \mathbf {u} _ {0} + \\ & \quad E _ {\mathbf {X} _ {1}, \dots , \mathbf {X} _ {N}} [ (\sum_ {k = 1} ^ {N - 1} [ \mathbf {X} _ {k} ^ {\mathrm{T}} \mathbf {Q} _ {k} ^ {0} \mathbf {X} _ {k} + \mathbf {u} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {0} \mathbf {u} _ {k} ] + \mathbf {X} _ {N} ^ {\mathrm{T}} \mathbf {S X} _ {N}) | _ {\mathbf {x} _ {0}} ] \end{array}\tag{5.2.4}
$$

式中 $E[(\cdot)|_{X_{0}}]$ 表示在初值为 $X_{0}$ 条件下的关于 $X_{1},\cdots,X_{N}$ 的条件均值。 $X_{0}$ 为某一具体值，则根据控制规律确定的 $u_{0}$ 也是具体值，所以 $u_{0}$ 项也不须要取均值。

因为逐级都是最优的,所以如果满足式(5.2.4),则从 $k+1$ 级到N级过程也应是最优的,在 $X_{k}$ 为某一具体值的条件下, $k+1$ 级到N级的最小指标函数为

$$
\begin{array}{r l} W \{\boldsymbol {X} _ {k} \} & = \min _ {\boldsymbol {u} _ {k}, \dots , \boldsymbol {u} _ {n - 1}} \{\boldsymbol {X} _ {k} ^ {\mathrm{T}} \boldsymbol {Q} _ {k} ^ {0} \boldsymbol {X} _ {k} + \boldsymbol {u} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {0} \boldsymbol {u} _ {k} + \\ & E _ {\boldsymbol {x} _ {k + 1}, \dots , \boldsymbol {x} _ {N}} [ (\sum_ {i = k + 1} ^ {N - 1} [ \boldsymbol {X} _ {i} ^ {\mathrm{T}} \boldsymbol {Q} _ {i} ^ {0} \boldsymbol {X} _ {i} + \boldsymbol {u} _ {i} ^ {\mathrm{T}} \boldsymbol {R} _ {i} ^ {0} \boldsymbol {u} _ {i} ] + \boldsymbol {X} _ {N} ^ {\mathrm{T}} \boldsymbol {S X} _ {N}) | _ {\boldsymbol {x} _ {k}} ] \} = \\ & \min _ {\boldsymbol {u} _ {k}} \{\boldsymbol {X} _ {k} ^ {\mathrm{T}} \boldsymbol {Q} _ {k} ^ {0} \boldsymbol {X} _ {k} + \boldsymbol {u} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {0} \boldsymbol {u} _ {k} + \min _ {\boldsymbol {u} _ {k + 1}, \dots , \boldsymbol {u} _ {N - 1}} E _ {\boldsymbol {x} _ {k + 1}, \dots , \boldsymbol {x} _ {N}} [ (\sum_ {i = k + 1} ^ {N - 1} [ \boldsymbol {X} _ {i} ^ {\mathrm{T}} \boldsymbol {Q} _ {i} ^ {0} \boldsymbol {X} _ {i} + \boldsymbol {u} _ {i} ^ {- 1}, \dots , \boldsymbol {u} _ {N - 1}) | _ {\boldsymbol {x} _ {k}} ] ] \\ & \quad + \boldsymbol {X} _ {N} ^ {\mathrm{T}} \boldsymbol {S X} _ {N}) | _ {\boldsymbol {x} _ {k}} ] \quad k = 0, 1, 2, \dots , N - 1 \end{array}\tag{5.2.5}
$$

交换取极小值与求均值的次序，并对 $X_{k + 1}$ 单独求取均值，则上式可改写为

$$
\begin{array}{l} W \left\{\boldsymbol {X} _ {k} \right\} = \min _ {\boldsymbol {u} _ {k}} \left\{\boldsymbol {X} _ {k} ^ {\mathrm{T}} \boldsymbol {Q} _ {k} ^ {0} \boldsymbol {X} _ {k} + \boldsymbol {u} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {0} \boldsymbol {u} _ {k} + \right. \\ E \left[ \min _ {\boldsymbol {u} _ {k + 1}, \dots , \boldsymbol {u} _ {N - 1}} E \left[ \left(\sum_ {i = k + 1} ^ {N - 1} \left[ \boldsymbol {X} _ {i} ^ {\mathrm{T}} \boldsymbol {Q} _ {i} ^ {0} \boldsymbol {X} _ {i} + \boldsymbol {u} _ {i} ^ {\mathrm{T}} \boldsymbol {R} _ {i} ^ {0} \boldsymbol {u} _ {i} \right] + \boldsymbol {X} _ {N} ^ {\mathrm{T}} \boldsymbol {S X} _ {N}\right) | _ {\boldsymbol {x} _ {k + 1}} \right] | _ {\boldsymbol {x} _ {k}} \right] \end{array}
$$

上式中的最后一项为

$$
\begin{array}{l} \min _ {\boldsymbol {u} _ {k + 1}, \dots , \boldsymbol {u} _ {N - 1}} E [ (\sum_ {i = k + 1} ^ {N - 1} [ \boldsymbol {X} _ {i} ^ {\mathrm{T}} \boldsymbol {Q} _ {i} ^ {0} \boldsymbol {X} _ {i} + \boldsymbol {u} _ {i} ^ {\mathrm{T}} \boldsymbol {R} _ {i} ^ {0} \boldsymbol {u} _ {i} ] + \boldsymbol {X} _ {N} ^ {\mathrm{T}} \boldsymbol {S} \boldsymbol {X} _ {N}) | _ {\boldsymbol {x} _ {k + 1}} ] = \\ \min _ {\boldsymbol {u} _ {k + 1}, \dots , \boldsymbol {u} _ {N - 1}} \{\boldsymbol {X} _ {k + 1} ^ {\mathrm{T}} \boldsymbol {Q} _ {k + 1} ^ {0} \boldsymbol {X} _ {k + 1} + \boldsymbol {u} _ {k + 1} ^ {\mathrm{T}} \boldsymbol {R} _ {k + 1} ^ {0} \boldsymbol {u} _ {k + 1} + \\ E [ (\sum_ {i = k + 2} ^ {N - 1} [ \boldsymbol {X} _ {i} ^ {\mathrm{T}} \boldsymbol {Q} _ {i} ^ {0} \boldsymbol {X} _ {i} + \boldsymbol {u} _ {i} ^ {\mathrm{T}} \boldsymbol {R} _ {i} ^ {0} \boldsymbol {u} _ {i} ] + \boldsymbol {X} _ {N } ^ {\mathrm{T}} \boldsymbol {S} \boldsymbol {X} _ {N}) | _ {\boldsymbol {x} _ {k + 1}} ] \} = \\ W \{\boldsymbol {X} _ {k + 1} \} \end{array}
$$

上式推导中,利用了式(5.2.5)关系。

所以有

$$
W \left\{\boldsymbol {X} _ {k} \right\} = \min _ {\boldsymbol {u} _ {k}} \left\{\boldsymbol {X} _ {k} ^ {\mathrm{T}} \boldsymbol {Q} _ {k} ^ {0} \boldsymbol {X} _ {k} + \boldsymbol {u} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {0} \boldsymbol {u} _ {k} + E _ {\boldsymbol {X} _ {k + 1}} \left[ W \left\{\boldsymbol {X} _ {k + 1} \right\} \mid_ {\boldsymbol {X} _ {k}} \right] \right\}\tag{5.2.6}
$$

该式是动态规划基本递推关系式，即从 $W\{\mathbf{X}_{k + 1}\}$ 推出 $W\{\mathbf{X}_k\}$ 。而第 $N$ 级为

$$
\boldsymbol {W} \left\{\boldsymbol {X} _ {N} \right\} = \boldsymbol {X} _ {N} ^ {\mathrm{T}} \boldsymbol {S} \boldsymbol {X} _ {N}
$$

因此，计算顺序是从第 N 级一直算到第一级为止。

下面讨论式(5.2.6)解的求取。从该式可看出， $W\{X_{N}\}$ 是 $X_{N}$ 的二次型函数，第N-1级的 $W\{X_{N-1}\}$ 是 $X_{N-1}$ 和 $u_{N-1}$ 的二次型函数，而 $u_{N-1}$ 与 $X_{N-1}$ 和 $X_{N}$ 有关，后者是个确定的值。依此类推，可以认为第k级的 $W\{X_{k}\}$ 是 $X_{k}$ 的二次型函数，并与一个确定的值有关。因此可设

$$
W \left\{\boldsymbol {X} _ {k} \right\} = \boldsymbol {X} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} ^ {0} \boldsymbol {X} _ {k} + \eta_ {k}\tag{5.2.7}
$$

式中， $P_{k}^{0}$ 是 $n \times n$ 的非负定阵， $\eta_{k}$ 就是上述的确定值。根据上述关系，式(5.2.6)可改写成

$$
W \left\{\boldsymbol {X} _ {k} \right\} = \min _ {\boldsymbol {u} _ {k}} \left\{\boldsymbol {X} _ {k} ^ {\mathrm{T}} \boldsymbol {Q} _ {k} ^ {0} \boldsymbol {X} _ {k} + \boldsymbol {u} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {0} \boldsymbol {u} _ {k} + E _ {\boldsymbol {X} _ {k + 1}} \left[ \left(\boldsymbol {X} _ {k + 1} ^ {\mathrm{T}} \boldsymbol {P} _ {k + 1} ^ {0} \boldsymbol {X} _ {k + 1} + \eta_ {k + 1}\right) \mid_ {\boldsymbol {X} _ {k}}\right) \right\}\tag{5.2.8}
$$

将式(5.2.1)代入,由于状态初值和噪声都是正态分布的,所以 $X_{k}$ 也是正态分布的, $X_{k}$ 和 $W_{k}$ 互不相关就意味着互相独立,这就使有关 $W_{k}$ 的均值与条件 $X_{k}$ 无关,因此有

$$
\begin{array}{r l} & E _ {\mathbf {x} _ {k + 1}} \left\{\left(\mathbf {X} _ {k + 1} ^ {\mathrm{T}} \mathbf {P} _ {k + 1} ^ {0} \mathbf {X} _ {k + 1} + \eta_ {k + 1}\right) \mid_ {\mathbf {x} _ {k}} \right\} = \\ & E _ {\mathbf {x} _ {k}, \mathbf {w} _ {k}} \left\{\left[ \left(\boldsymbol {\Phi} _ {k + 1, k} \mathbf {X} _ {k} + \mathbf {B} _ {k} \mathbf {u} _ {k} + \mathbf {W} _ {k}\right) ^ {\mathrm{T}} \mathbf {P} _ {k + 1} ^ {0} \left(\boldsymbol {\Phi} _ {k + 1, k} \mathbf {X} _ {k} + \mathbf {B} _ {k} \mathbf {u} _ {k} + \mathbf {W} _ {k}\right) + \eta_ {k + 1} \right] \mid_ {\mathbf {x} _ {k}} \right\} = \\ & [ \boldsymbol {\Phi} _ {k + 1, k} \mathbf {X} _ {k} + \mathbf {B} _ {k} \mathbf {u} _ {k} ] ^ {\mathrm{T}} \mathbf {P} _ {k + 1} ^ {0} [ \boldsymbol {\Phi} _ {k + 1, k} \mathbf {X} _ {k} + \mathbf {B} _ {k} \mathbf {u} _ {k} ] + E [ \mathbf {W} _ {k} ^ {\mathrm{T}} \mathbf {P} _ {k + 1} ^ {0} \mathbf {W} _ {k} ] + \eta_ {k + 1} = \\ & [ \boldsymbol {\Phi} _ {k + 1, k} \mathbf {X} _ {k} + \mathbf {B} _ {k} \mathbf {u} _ {k} ] ^ {\mathrm{T}} \mathbf {P} _ {k + 1} ^ {0} [ \boldsymbol {\Phi} _ {k + 1, k} \mathbf {X} _ {k}, \mathbf {B} _ {k} \mathbf {U} _ {k} ] + \operatorname{tr} [ \mathbf {P} _ {k + 1} ^ {0}, \mathbf {Q} _ {k} ] + \eta_ {k + 1} \end{array} \tag {5.2.9}
$$

将式 $(5.2.9)$ 代入式 $(5.2.8)$ 得

$$
\begin{array}{r l} W \{\boldsymbol {X} _ {k} \} & = \min _ {\boldsymbol {u} _ {k}} \{\boldsymbol {X} _ {k} ^ {\mathrm{T}} \boldsymbol {Q} _ {k} ^ {0} \boldsymbol {X} _ {k} + \boldsymbol {u} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {0} \boldsymbol {u} _ {k} + [ \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {X} _ {k} + \boldsymbol {B} _ {k} \boldsymbol {u} _ {k} ] ^ {\mathrm{T}} \boldsymbol {P} _ {k + 1} ^ {0} [ \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {X} _ {k} + \boldsymbol {B} _ {k} \boldsymbol {u} _ {k} ] + \\ & \quad \operatorname{tr} [ \boldsymbol {P} _ {k + 1} ^ {0} \boldsymbol {Q} _ {k} ] + \eta_ {k + 1} \} = \min _ {\boldsymbol {u} _ {k}} \{\boldsymbol {X} _ {k} ^ {\mathrm{T}} [ \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} \boldsymbol {P} _ {k + 1} ^ {0} \boldsymbol {\Phi} _ {k + 1, k} + \boldsymbol {Q} _ {k} ^ {0} - \\ & \quad \boldsymbol {A} _ {k} ^ {\mathrm{T}} (\boldsymbol {R} _ {k} ^ {0} + \boldsymbol {B} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k + 1} ^ {0} \boldsymbol {B} _ {k}) \boldsymbol {A} _ {k} ] \boldsymbol {X} _ {k} + \\ & \quad (\boldsymbol {u} _ {k} + \boldsymbol {A} _ {k} \boldsymbol {X} _ {k}) ^ {\mathrm{T}} (\boldsymbol {R} _ {k} ^ {0} + \boldsymbol {B} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k + 1} ^ {0} \boldsymbol {B} _ {k}) (\boldsymbol {u} _ {k} + \boldsymbol {A} _ {k} \boldsymbol {X} _ {k}) + \operatorname{tr} [ \boldsymbol {P} _ {k + 1} ^ {0} \boldsymbol {Q} _ {k} ] \eta_ {k + 1} \}. \end{array} \tag {5.2.10}
$$

式中

$$
\boldsymbol {A} _ {k} = \left[ \boldsymbol {R} _ {k} ^ {0} + \boldsymbol {B} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k + 1} ^ {0} \boldsymbol {B} _ {k} \right] ^ {- 1} \boldsymbol {B} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k + 1} ^ {0} \boldsymbol {\Phi} _ {k + 1, k}\tag{5.2.11}
$$

因此，使指标函数为最小的 $u_{k}$ 为

$$
\boldsymbol {u} _ {k} ^ {*} = - \boldsymbol {A} _ {k} \boldsymbol {X} _ {k} = - \left[ \boldsymbol {R} _ {K} ^ {0} + \boldsymbol {B} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k + 1} \boldsymbol {B} _ {k} \right] ^ {- 1} \boldsymbol {B} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k + 1} ^ {0} \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {X} _ {k}\tag{5.2.12}
$$

因为 $R_{k}^{0}$ 是正定阵， $P_{k+1}^{0}$ 是非负定阵，所以上式中的逆阵存在。从该式可以看出，最优控制 $u_{k}^{*}$ 是状态 $X_{k}$ 的线性函数。

比较式(5.2.7)和式(5.2.10)可得

$$
\boldsymbol {P} _ {k} ^ {0} = \boldsymbol {Q} _ {k} ^ {0} + \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} \boldsymbol {P} _ {k + 1} ^ {0} \boldsymbol {\Phi} _ {k + 1, k} - \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} \boldsymbol {P} _ {k + 1} ^ {0} \boldsymbol {B} _ {k} [ \boldsymbol {R} _ {k} ^ {0} +
$$

$$
\left. \boldsymbol {B} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k + 1} ^ {0} \boldsymbol {B} _ {k} \right] ^ {- 1} \boldsymbol {B} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k + 1} ^ {0} \boldsymbol {\Phi} _ {k + 1, k}\tag{5.2.13}
$$

$$
\eta_ {k} = \mathrm{tr} [ P _ {k + 1} ^ {0} Q _ {k} ] + \eta_ {k + 1}\tag{5.2.14}
$$

式(5.2.13)为离散形式的黎卡蒂方程,它仅与参数阵 $\Phi_{k+1,k}$ , $B_{k}$ 以及指标函数中的权矩阵 $Q_{k}^{0}$ , $R_{k}^{0}$ 有关,与噪声的统计特性 $Q_{k}$ , $R_{k}$ 无关。 $P_{k}^{0}$ 的终端条件为 $P_{k}^{0}=S$ 。 $P_{k}^{0}$ 按式(5.2.13)从 N 级逐级计算至 $P_{N}^{0}$ 。式(5.2.14)的终端条件为 $\eta_{N}=0$ 。同样,从 N 级逐级计算 $\eta_{k}$ 。

算出 $P_{k+1}^{0}$ 后，按式(5.2.12) 算出 $u_{k}^{*}$ ，即可逐级计算得最优控制序列 $u_{N-1}^{*}, \cdots, u_{k}^{*}, \cdots, u_{1}^{*}, u_{0}^{*}$ 。

下面讨论最小指标函数的求取。因为 $W\{X_{0}\}$ 是在某一 $X_{0}$ 的条件下的最小指标函数，所以在随机意义下的最小指标函数就是要用 $W\{X_{0}\}$ 的均值来衡量，即

$$
J _ {N \min} = E [ W \{\mathbf {X} _ {0} \} ] = E [ \mathbf {X} _ {0} ^ {\mathrm{T}} \mathbf {P} _ {0} ^ {0} \mathbf {X} _ {0} + \eta_ {0} ] = E [ \mathbf {X} _ {0} ^ {\mathrm{T}} \mathbf {P} _ {0} ^ {0} \mathbf {X} _ {0} ] + \sum_ {k = 0} ^ {N - 1} \operatorname{tr} [ \mathbf {P} _ {k + 1} ^ {0} \mathbf {Q} _ {k} ]
$$

因

$$
E [ \boldsymbol {X} _ {0} ^ {\mathrm{T}} \boldsymbol {P} _ {0} ^ {0} \boldsymbol {X} _ {0} ] = \boldsymbol {m} _ {X _ {0}} ^ {\mathrm{T}} \boldsymbol {P} _ {0} ^ {0} \boldsymbol {m} _ {X _ {0}} + \operatorname{tr} [ \boldsymbol {P} _ {0} ^ {0} \boldsymbol {C} _ {X _ {0}} ]
$$

式中 $m_{X_{0}}$ 和 $C_{X_{0}}$ 各为 $X_{0}$ 的均值和方差。所以

$$
J _ {N \min} = m _ {X _ {0}} ^ {\mathrm{T}} P _ {0} ^ {0} m _ {X _ {0}} + \operatorname{tr} [ P _ {0} ^ {0} C _ {X _ {0}} ] + \sum_ {k = 0} ^ {N - 1} \operatorname{tr} [ P _ {k + 1} ^ {0} Q _ {k} ]\tag{5.2.15}
$$

2. 在不能得到状态向量 $X_{k}$ 的条件下

这时可通过最优滤波得到状态的最优估计。可以证明，最优控制规律仍然是式(5.2.12)，唯一的不同仅需将 $X_{k}$ 改为 $\hat{X}_{k}$ 。这时的最优是指在采用最优估计这一条件下的最优。

在一般情况下，量测 $Z_{k}$ 在 $t_{k}$ 时刻得到，而估计 $\hat{X}_{k}$ 的计算尚需要时间，因此， $t_{k}$ 时刻的最优估计取 $\hat{X}_{k/k-1}$ 。在这种条件下的最优控制为

$$
\boldsymbol {u} _ {k} ^ {*} = - \boldsymbol {A} _ {k} \hat {\boldsymbol {X}} _ {k / k - 1}\tag{5.2.16}
$$

根据卡尔曼滤波一步预测方程式(2.2.35)再考虑控制项,有

$$
\begin{array}{r l} \hat {\mathbf {X}} _ {k + 1 / k} & = \boldsymbol {\Phi} _ {k + 1, k} \hat {\mathbf {X}} _ {k / k - 1} + \boldsymbol {B} _ {k} \boldsymbol {u} _ {k} + \boldsymbol {K} _ {k} ^ {*} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1}) = \\ & \quad \boldsymbol {\Phi} _ {k + 1, k} \hat {\mathbf {X}} _ {k / k - 1} + \boldsymbol {B} _ {k} \boldsymbol {u} _ {k} + \boldsymbol {K} _ {k} ^ {*} (\boldsymbol {V} _ {k} + \boldsymbol {H} _ {k} \tilde {\mathbf {X}} _ {k / k - 1}) \end{array}\tag{5.2.17}
$$

比较式(5.2.17)与式(5.2.1)，可看出 $\hat{X}_{k+1/k}$ 和 $\hat{X}_{k/k-1}$ 各与 $X_{k+1}$ 和 $X_{k}$ 相对应，而 $K_{k}^{*}(V_{k}+H_{k}\hat{X}_{k/k-1})$ 可看做零均值的白噪声，所以两式具有相似的形式。

现将指标函数式(5.2.3)改写为 $\hat{\mathbf{X}}_{k / k - 1}$ 的形式。因为估计 $\hat{\mathbf{X}}_{k / k - 1}$ 与估计误差 $\tilde{\mathbf{X}}_{k / k - 1}$ 正交，即

$$
E [ \hat {\mathbf {X}} _ {k / k - 1} \tilde {\mathbf {X}} _ {k / k - 1} ^ {\mathrm{T}} ] = \mathbf {0}
$$

所以

$$
\begin{array}{r l} & J _ {N} = E \{\sum_ {k = 1} ^ {N - 1} [ (\hat {\mathbf {X}} _ {k / k - 1} + \tilde {\mathbf {X}} _ {k / k - 1}) ^ {\mathrm{T}} \boldsymbol {Q} _ {k} ^ {0} (\hat {\mathbf {X}} _ {k / k - 1} + \tilde {\mathbf {X}} _ {k / k - 1}) + \\ & \quad \boldsymbol {u} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {0} \boldsymbol {u} _ {k} ] + (\hat {\mathbf {X}} _ {N / N - 1} + \tilde {\mathbf {X}} _ {N / N - 1}) ^ {\mathrm{T}} \boldsymbol {S} (\hat {\mathbf {X}} _ {N / N - 1} + \tilde {\mathbf {X}} _ {N / N - 1}) \} = \end{array}
$$

$$
\begin{array}{l} E \{\sum_ {k = 0} ^ {N - 1} [ \hat {\mathbf {X}} _ {k / k - 1} ^ {\mathrm{T}} \mathbf {Q} _ {k} ^ {0} \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {u} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {0} \mathbf {u} _ {k} ] + \hat {\mathbf {X}} _ {N / N - 1} ^ {\mathrm{T}} \mathbf {S} \hat {\mathbf {X}} _ {N / N - 1} \} + \\ E \{\sum_ {k = 0} ^ {N - 1} \widetilde {\mathbf {X}} _ {k / k - 1} ^ {\mathrm{T}} \mathbf {Q} _ {k} ^ {0} \widetilde {\mathbf {X}} _ {k / k - 1} + \widetilde {\mathbf {X}} _ {N / N - 1} ^ {\mathrm{T}} \mathbf {S} \widetilde {\mathbf {X}} _ {N / N - 1} \} \stackrel {{\text {def}}} {{=}} J _ {N} (\hat {\mathbf {X}}) + J _ {N} (\widetilde {\mathbf {X}}) \end{array}\tag{5.2.18}
$$

$u_{k}$ 仅由 $\hat{X}_{k/k-1}$ 决定，与 $\hat{X}_{k/k-1}$ 无关。所以在选择 $u_{k}$ 时，只要使上式中 $J_{N}(\hat{X})$ 最小，就能使指标函数最小，而 $J_{N}(\hat{X})$ 与式(5.2.3)在形式上完全相同。因此，借用上一节讨论的结果，即可求得 $J_{N}(\hat{X})$ 最小的控制规律为

$$
\boldsymbol {u} _ {k} ^ {*} = - \boldsymbol {A} _ {k} \hat {\boldsymbol {X}} _ {k / k - 1}
$$

$A_{k}$ 的方程同式(5.2.11)，式中所用的 $P_{k}^{0}$ 也从式(5.2.13)求出。这就证明了分离定理，即求最优控制 $u_{k}$ 时，以一步预测 $\hat{X}_{k/k-1}$ 为已知条件；求最优估计 $\hat{X}_{k}$ 时，以已确定的 $u_{k}$ 为已知条件。

这种最优控制下的指标函数的求法如下：

滤波方程式(5.2.17)中的噪声 $K_{k}^{*}\left[V_{k}+H_{k}\widetilde{X}_{k/k-1}\right]$ 相当于式(5.2.1)中的系统噪声 $W_{k}$ 。该噪声的方差阵为

$$
Q _ {k} ^ {\prime} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {R} _ {k} + \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}}) ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}}\tag{5.2.19}
$$

用 $\xi_{k}$ 代替式(5.2.14)中的 $\eta_{k}$ ，则

$$
\xi_ {k} = \mathrm{tr} [ P _ {k + 1} ^ {0} Q _ {k} ^ {\prime} ] + \xi_ {k + 1}\tag{5.2.20}
$$

同样， $\xi_{k}$ 的终端条件为

$$
\xi_ {N} = 0
$$

按照式(5.2.20)的递推关系,可逐级算出 $\xi_{k}$ ,加上 $P_{k}^{0}$ ,可以得到

$$
W \left\{\hat {\boldsymbol {X}} _ {k / k - 1} \right\} = \hat {\boldsymbol {X}} _ {k / k - 1} ^ {\mathrm{T}} \boldsymbol {P} _ {k} ^ {0} \hat {\boldsymbol {X}} _ {k / k - 1} + \xi_ {k}\tag{5.2.21}
$$

根据式 $(5.2.18)$ 最小指标函数为

$$
\begin{array}{r l} J _ {N \min} & = \min J _ {N} (\hat {\mathbf {X}}) + \min J _ {N} (\tilde {\mathbf {X}}) = E [ W \{\hat {\mathbf {X}} _ {0} \} ] + \min J _ {N} (\tilde {\mathbf {X}}) = \\ & m _ {X _ {0}} ^ {\mathrm{T}} P _ {0} ^ {0} m _ {X _ {0}} + \operatorname{tr} [ P _ {0} ^ {0} P _ {0} ] + \sum_ {k = 0} ^ {N - 1} \operatorname{tr} [ P _ {k + 1} ^ {0} Q _ {k} ^ {\prime} ] + \sum_ {k = 0} ^ {N - 1} \operatorname{tr} [ Q _ {k} ^ {0} P _ {k / k - 1} ] + \\ & \operatorname{tr} [ S P _ {N / N - 1} ] \end{array}\tag{5.2.22}
$$

包括系统和滤波器的最优控制方块图如图5.2.1所示。图中未示出反馈阵 $A_{k}$ 的计算过程。

![](images/c5c999a0479e0b7323e2bb7ebf3e847550c97e7615e854feb251dcb1a06a6111.jpg)  
图5.2.1 基于最优估计的最优控制的方块图

需要指出的是:如果从得到量测后计算出估计和控制所需要的时间远比量测的时间间隔短,则可认为 $t_{k}$ 时刻能够得到最优估计 $\hat{X}_{k}$ 。这样,最优控制规律可改写为

$$
\boldsymbol {u} _ {k} ^ {*} = - \boldsymbol {A} _ {k} \hat {\boldsymbol {X}} _ {k}\tag{5.2.23}
$$

式中 $A_{k}$ 的计算及 $A_{k}$ 方程中 $P_{k+1}^{0}$ 的计算仍按式(5.2.11)和式(5.2.13)，仅最小指标函数稍有不同，这里就不再列写了。

以上讨论了二次型指标函数为最小要求下系统的最优控制规律。结论是:如果系统的状态可以得到,则根据式(5.2.12)和式(5.2.13)可以求出最优控制。如果系统状态不能得到,则状态可用最优估计代替,按式(5.2.16)(或式(5.2.23))和式(5.2.13)求出最优控制。最优估计和最优控制可以分别考虑的原理称为分离定理。

二次型指标是一种较为全面的要求，但指标中的权矩阵与控制质量没有直接的联系，在设计中不容易确定下来。另外，由于 $P_{k}^{0}$ 阵是从终端开始计算的，所以必须预先计算，并将数值存储在计算机中。如果控制级数N很大，则存储量是很大的。

## 5.3 连续系统的分离定理

设系统和量测为

$$
\dot {\boldsymbol {X}} (t) = \boldsymbol {F} (t) \boldsymbol {X} (t) + \boldsymbol {E} (t) \boldsymbol {u} (t) + \boldsymbol {w} (t)\tag{5.3.1}
$$

$$
\mathbf {Z} (t) = \mathbf {H} (t) \mathbf {X} (t) + \nu (t)\tag{5.3.2}
$$

式中 $w(t)$ 和 $v(t)$ 为互不相关的零均值正态白噪声，方差强度阵分别为 $q(t)$ 和 $r(t)$ ， $q(t)$ 为非负定阵， $r(t)$ 为正定阵，初始状态 X(0) 也服从正态分布，与 $w(t)$ ， $v(t)$ 不相关，均值为 $m_{X}(0)$ ，方差为 $C_{X}$ 。

指标函数为

$$
J = E \left\{\boldsymbol {X} ^ {\mathrm{T}} \left(t _ {N}\right) \boldsymbol {S X} \left(t _ {N}\right) + \int_ {t _ {0}} ^ {t _ {N}} \left[ \boldsymbol {X} ^ {\mathrm{T}} (t) \boldsymbol {Q} ^ {0} (t) \boldsymbol {X} (t) + \boldsymbol {u} ^ {\mathrm{T}} (t) \boldsymbol {R} ^ {0} (t) \boldsymbol {u} (t) \right] \mathrm{d} t \right\}\tag{5.3.3}
$$

式中各权矩阵的意义和要求都与离散系统的相同。最优控制的目的就是要选择 $\boldsymbol{u}^{*}(t)$ ，使指标函数 J 达到最小。

与离散系统一样，在不能得到状态的情况下，连续系统的最优控制同样可以与最优估计分开考虑，即作最优估计时可将控制量作为已知量处理，而最优控制虽然采用状态的估计，但控制规律却与采用状态本身的相同，这就是分离定理。现将最优控制的各方程列写如下，证明从略。式中各符号的意义可参看离散系统。

$$
\boldsymbol {u} ^ {*} (t) = - \boldsymbol {K} _ {c} (t) \hat {\boldsymbol {X}} (t) = - \boldsymbol {R} ^ {0} (t) ^ {- 1} \boldsymbol {E} ^ {\mathrm{T}} (t) \boldsymbol {P} ^ {0} (t) \hat {\boldsymbol {X}} (t)\tag{5.3.4}
$$

$$
\dot {\boldsymbol {P}} ^ {0} (t) = - \boldsymbol {P} ^ {0} (t) \boldsymbol {F} (t) - \boldsymbol {F} ^ {\mathrm{T}} (t) \boldsymbol {P} ^ {0} (t) + \boldsymbol {P} ^ {0} (t) \boldsymbol {E} (t) \boldsymbol {R} ^ {0} (t) ^ {- 1} \boldsymbol {E} ^ {\mathrm{T}} (t) \boldsymbol {P} ^ {0} (t) - \boldsymbol {Q} ^ {0} (t)\tag{5.3.5}
$$

$$
\boldsymbol {P} ^ {0} (t _ {N}) = \boldsymbol {S}
$$

$$
\dot {\boldsymbol {\xi}} (t) = - \operatorname{tr} \left[ \boldsymbol {P} (t) \boldsymbol {H} ^ {\mathrm{T}} (t) \boldsymbol {R} ^ {- 1} (t) \boldsymbol {H} (t) \boldsymbol {P} (t) \boldsymbol {P} ^ {0} (t) \right]\tag{5.3.6}
$$

$$
\xi (t _ {N}) = 0
$$

用估计和估计误差代替状态,则指标函数为

$$
J = E \{\int_ {t _ {0}} ^ {t _ {N}} [ \hat {\mathbf {X}} ^ {\mathrm{T}} (t) \mathbf {Q} ^ {0} (t) \hat {\mathbf {X}} (t) + \mathbf {u} ^ {\mathrm{T}} (t) \mathbf {R} ^ {0} (t) \mathbf {u} (t) ] \mathrm{d} t \} +
$$

$$
\begin{array}{r l} & E [ \hat {\mathbf {X}} ^ {\mathrm{T}} (t _ {N}) \mathbf {S} \hat {\mathbf {X}} (t _ {N}) ] + E \{\int_ {t _ {0}} ^ {t _ {N}} \tilde {\mathbf {X}} ^ {\mathrm{T}} (t) \mathbf {Q} ^ {0} (t) \tilde {\mathbf {X}} (t) \mathrm{d} t \} = \\ & E [ \tilde {\mathbf {X}} ^ {\mathrm{T}} (t _ {N}) \mathbf {S} \tilde {\mathbf {X}} (t _ {N}) ] \stackrel {\text {def}} {=} J (\hat {\mathbf {X}}) + J (\tilde {\mathbf {X}}) \\ & W \{\hat {\mathbf {X}} (t), t \} = \hat {\mathbf {X}} ^ {\mathrm{T}} (t) \mathbf {P} ^ {0} (t) \hat {\mathbf {X}} (t) + \xi (t) \\ & J _ {\min} = m _ {X} ^ {\mathrm{T}} (0) \mathbf {P} ^ {0} (0) m _ {X} (0) + \operatorname{tr} [ \mathbf {P} ^ {0} (0) \mathbf {P} (0) ] + \\ & \int_ {t _ {0}} ^ {t _ {N}} \operatorname{tr} [ \mathbf {P} (t) \mathbf {H} ^ {\mathrm{T}} (t) \mathbf {R} ^ {- 1} (t) \mathbf {H} (t) \mathbf {P} (t) \mathbf {P} ^ {0} (t) ] \mathrm{d} t + \\ & \int_ {t _ {0}} ^ {t _ {N}} \operatorname{tr} [ \mathbf {P} (t) \mathbf {Q} ^ {0} (t) ] \mathrm{d} t + \operatorname{tr} [ \mathbf {P} (t _ {N}) \mathbf {S} ] \\ & \mathbf {P} (0) = E \{[ \mathbf {X} (0) - \hat {\mathbf {X}} (0) ] [ \mathbf {X} (0) - \hat {\mathbf {X}} (0) ] ^ {\mathrm{T}} \} \end{array}\tag{5.3.7}
$$

(5.3.8)

(5.3.9)

## 5.4 离散系统的估计直接反馈控制

物理系统一般并不都是线性的，比如惯导系统、GPS导航系统、多普勒导航系统等都是非线性系统。若直接以导航参数作为状态，则对应的系统方程和量测方程都是非线性方程，直接按这些方程设计滤波器十分麻烦，必须采用非线性滤波方法。所以为避免不必要的繁琐，一般都以系统输出参量的误差作状态。而误差相对真实参量是小量，二阶及二阶以上的误差量乘积可视为高阶小量而忽略掉，这样，系统方程和量测方程都成为线性方程，滤波问题也就成为线性系统的滤波问题了。

由于滤波系统的状态都是误差量,所以控制的目的是消除这些误差量,以提高物理系统的精度,这就使以上介绍的控制规律的求取得以简化。对于平台式惯导系统,速度、位置等导航参量都是数字量,它们由导航计算机解算获得,所包含的误差可用一次性校正即脉冲校正的方法清除掉。而惯性平台的姿态角、方位角是物理参量,消除它们的误差角需要通过给相应陀螺施加指令来完成,而陀螺进动需要时间,所以对这些误差量的校正需要在时间过程里的连续控制。

## 5.4.1 由估计确定的脉冲校正

设卡尔曼滤波器的滤波周期为 T，获得量测后计算出估计值所需时间为 $T_{f}$ 。根据 $T_{f}$ 与 T 相比大小的不同，由估计值确定出脉冲校正量的方法也不同。

1. $T_{f}\ll T$

$T_{f} \ll T$ 意味着获得量测 $Z_{k}$ 后立即就能解得估计 $\hat{X}_{k}$ ，用 $\hat{X}_{k}$ 校正 $X_{k}$ 具有实时意义。记校正前的状态估计为 $\hat{X}_{k}^{-}$ ，校正后的状态估计为 $\hat{X}_{k}^{+}$ ，所加的脉冲校正控制为 $B_{k}^{P}u_{k}^{P}$ 。对于使用误差量作为状态变量的滤波系统来说，校正的目的是要消除这些被估计出来的误差量，即

$$
\hat {\boldsymbol {X}} _ {k} ^ {+} = \boldsymbol {B} _ {k} ^ {P} \boldsymbol {u} _ {k} ^ {P} + \hat {\boldsymbol {X}} _ {k} ^ {-} = \mathbf {0}\tag{5.4.1}
$$

具体做法是从导航系统的导航解中扣除这一误差的估计值。所以，脉冲校正使各误差量根据估计值各自消除，各误差量之间没有交叉影响，即

$$
\boldsymbol {B} _ {k} ^ {P} = \boldsymbol {I}\tag{5.4.2}
$$

因此，脉冲校正控制量为

$$
\boldsymbol {u} _ {k} ^ {P} = - \hat {\boldsymbol {X}} _ {k}\tag{5.4.3}
$$

由式 $(5.4.1)$ 得采用脉冲校正后的一步预测值和滤波值

$$
\hat {\boldsymbol {X}} _ {k + 1 / k} = \boldsymbol {\Phi} _ {k + 1, k} \hat {\boldsymbol {X}} _ {k} ^ {+} = \mathbf {0}
$$

$$
\hat {\mathbf {X}} _ {k + 1} ^ {-} = \hat {\mathbf {X}} _ {k + 1 / k} + \mathbf {K} _ {k + 1} (\mathbf {Z} _ {k + 1} - \mathbf {H} _ {k + 1} \hat {\mathbf {X}} _ {k + 1 / k}) = \mathbf {K} _ {k + 1} \mathbf {Z} _ {k + 1}
$$

亦即

$$
\hat {X} _ {k / k - 1} = 0\tag{5.4.4}
$$

$$
\hat {\mathbf {X}} _ {k} ^ {-} = \mathbf {K} _ {k} \mathbf {Z} _ {k}\tag{5.4.5}
$$

将式 $(5.4.5)$ 代入式 $(5.4.3)$ 得脉冲校正控制为

$$
\boldsymbol {u} _ {k} ^ {P} = - \boldsymbol {K} _ {k} \boldsymbol {Z} _ {k}\tag{5.4.6}
$$

2. $T_{f}$ 不是远小于 $T$

$T_{f}$ 不是远小于 $T$ 意味着获得量测 $\mathbf{Z}_k$ 后并不能立即就得到 $\hat{\mathbf{X}}_k$ ，而要经过较长的延迟才能得到。此时若仍用 $\hat{\mathbf{X}}_k$ 作为 $k$ 时刻导航解的校正控制量，则已失去了实时意义，所以只能利用 $k - 1$ 时刻的估计值根据一步转移关系计算得 $k$ 时刻的脉冲校正控制量

$$
\boldsymbol {u} _ {k} ^ {P} = - \boldsymbol {\Phi} _ {k, k - 1} \hat {\boldsymbol {X}} _ {k - 1}\tag{5.4.7}
$$

而

$$
\boldsymbol {\Phi} _ {k, k - 1} \hat {\boldsymbol {X}} _ {k - 1} = \hat {\boldsymbol {X}} _ {k / k - 1} ^ {-}
$$

所以

$$
\boldsymbol {u} _ {k} ^ {P} = - \hat {\boldsymbol {X}} _ {k / k - 1} ^ {-}
$$

这说明所加的控制实质上是消去 $\hat{X}_{k/k-1}^{-}$ ，经校正后的一步预测为

$$
\hat {X} _ {k / k - 1} ^ {+} = 0\tag{5.4.8}
$$

这样，滤波方程为

$$
\hat {\mathbf {X}} _ {k} = \mathbf {K} _ {k} \mathbf {Z} _ {k}\tag{5.4.9}
$$

## 5.4.2 由估计确定的持续校正

设物理系统满足的系统方程为

$$
\dot {\boldsymbol {X}} (t) = \boldsymbol {F} (t) \boldsymbol {X} (t) + \boldsymbol {E} (t) \boldsymbol {u} (t) + \boldsymbol {G} (t) \boldsymbol {w} (t)\tag{5.4.10}
$$

式中 $\boldsymbol{u}(t)$ 是连续控制量，它持续作用于系统。因为离散卡尔曼滤波只能得到滤波时间点上的估计值，所以由此确定的控制也只能是 $\boldsymbol{u}(t)$ 的采样值 $\boldsymbol{u}(t_{k})$ 。

这些采样值可根据卡尔曼滤波值来确定。

由于使用式 $(2.2.4)$ 基本滤波方程的需要,原连续系统作了离散化处理

$$
\boldsymbol {X} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {X} _ {k} + \int_ {t _ {k}} ^ {t _ {k + 1}} \boldsymbol {\Phi} _ {k + 1, t} \boldsymbol {E} (t) \boldsymbol {u} (t) \mathrm{d} t + \boldsymbol {\Gamma} _ {k} \boldsymbol {W} _ {k}
$$

(5.4.11)

若卡尔曼滤波器的滤波周期 T 较短，则在滤波间隔 $\left[t_{k}, t_{k+1}\right)$ 内， $\boldsymbol{u}(t)$ 可近似取作常值：

$$
\boldsymbol {u} (t) = \boldsymbol {u} _ {k} ^ {\mathrm{c}} \quad t _ {k} \leqslant t <   t _ {k + 1}\tag{5.4.12}
$$

式中 $\boldsymbol{u}_{k}^{c}=\boldsymbol{u}(t_{k})$ 是 $\boldsymbol{u}(t)$ 在 $t_{k}$ 时间点上的采样值。这样，式(5.4.11)可写成

$$
\boldsymbol {X} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {X} _ {k} + \int_ {t _ {k}} ^ {t _ {k + 1}} \boldsymbol {\Phi} _ {k + 1, t} \boldsymbol {E} (t) \mathrm{d} t \boldsymbol {u} _ {k} ^ {\mathrm{c}} + \boldsymbol {\Gamma} _ {k} \boldsymbol {W} _ {k}
$$

令

$$
\boldsymbol {B} _ {k} ^ {\mathrm{c}} = \int_ {t _ {k}} ^ {t _ {k + 1}} \boldsymbol {\Phi} _ {k + 1, t} \boldsymbol {E} (t) \mathrm{d} t\tag{5.4.13}
$$

则离散化后的系统为

$$
\boldsymbol {X} _ {k} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {X} _ {k - 1} + \boldsymbol {B} _ {k - 1} ^ {\mathrm{c}} \boldsymbol {u} _ {k - 1} ^ {\mathrm{c}} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {W} _ {k - 1}\tag{5.4.14}
$$

有两种可能的途径确定 $u_{k - 1}^{\mathrm{c}}$

(1) 根据状态估计值确定。式(5.4.14)对应的滤波方程为

$$
\begin{array}{r l} \hat {\mathbf {X}} _ {k} & = \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {K} _ {k} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1}) = \boldsymbol {\Phi} _ {k, k - 1} \hat {\mathbf {X}} _ {k - 1} + \mathbf {B} _ {k - 1} ^ {\mathrm{c}} \mathbf {u} _ {k - 1} ^ {\mathrm{c}} + \\ & \quad \mathbf {K} _ {k} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \boldsymbol {\Phi} _ {k, k - 1} \hat {\mathbf {X}} _ {k - 1} - \mathbf {H} _ {k} \mathbf {B} _ {k - 1} ^ {\mathrm{c}} \mathbf {u} _ {k - 1} ^ {\mathrm{c}}) \end{array}
$$

加入控制后，应使 $\hat{\pmb{X}}_k = \mathbf{0}$ ，所以

$$
\left(\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}\right) \boldsymbol {B} _ {k - 1} ^ {\mathrm{c}} \boldsymbol {u} _ {k - 1} ^ {\mathrm{c}} + \boldsymbol {\Phi} _ {k, k - 1} \hat {\boldsymbol {X}} _ {k - 1} - \boldsymbol {K} _ {k} \left(\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\Phi} _ {k, k - 1} \hat {\boldsymbol {X}} _ {k - 1}\right) = \boldsymbol {0}
$$

如果 $u_{k-1}^{c}$ 从上式中求取，则需要用到 $Z_{k}$ ，而 $u_{k-1}^{c}$ 是 $[t_{k-1}, t_{k})$ 内的控制量，而 $Z_{k}$ 是 $t_{k}$ 时刻的量测， $u_{k-1}^{c}$ 必须在 $t_{k-1}$ 时刻获得，而 $Z_{k}$ 只能在 $t_{k}$ 时刻获得，这说明由上述方程是无法确定 $u_{k-1}^{c}$ 的。

(2) 根据一步预测值确定。式(5.4.14) 对应的一步预测方程为

$$
\hat {\boldsymbol {X}} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \hat {\boldsymbol {X}} _ {k - 1} + \boldsymbol {B} _ {k - 1} ^ {\mathrm{c}} \boldsymbol {u} _ {k - 1} ^ {\mathrm{c}}\tag{5.4.15}
$$

加入控制后，应使 $\hat{X}_{k / k - 1} = 0$ ，所以有

$$
\boldsymbol {\Phi} _ {k, k - 1} \hat {\boldsymbol {X}} _ {k - 1} + \boldsymbol {B} _ {k - 1} ^ {\mathrm{c}} \boldsymbol {u} _ {k - 1} ^ {\mathrm{c}} = \boldsymbol {0}
$$

即

$$
\boldsymbol {B} _ {k} ^ {\mathrm{c}} \boldsymbol {u} _ {k} ^ {\mathrm{c}} = - \boldsymbol {\Phi} _ {k + 1, k} \hat {\boldsymbol {X}} _ {k}\tag{5.4.16}
$$

式中

$$
\pmb {B} _ {k} ^ {\mathrm{c}} = \int_ {t _ {k}} ^ {t _ {k + 1}} \pmb {\Phi} _ {k + 1, t} \pmb {E} (t) \mathrm{d} t
$$

由式(5.4.16)可确定出 $u_{k}^{c}$ ，这一控制在 $\left[t_{k},t_{k+1}\right)$ 时间段内加入系统，计算 $u_{k}^{c}$ 需要用到 $\hat{X}_{k}$ ，而 $\hat{X}_{k}$ 一般在 $t_{k}$ 时刻之后不久能获得，所以在时间的先后次序上是行得通的，按式(5.4.16)求取 $u_{k}^{c}$ 是可行的。此时，有

$$
\hat {\mathbf {X}} _ {k / k - 1} = \mathbf {0}\tag{5.4.17}
$$

$$
\hat {\mathbf {X}} _ {k} = \mathbf {K} _ {k} \mathbf {Z} _ {k}\tag{5.4.18}
$$

下面以速度组合导航系统中简化的惯导北向回路为例,说明脉冲校正和持续校正的具体求法。

例5-1 由多普勒雷达和惯导系统组成的速度组合导航系统中（取北-西-天地理坐标系为导航坐标系），在忽略地转率补偿量及有害加速度条件下，简化的惯导系统北向回路状态方程为

$$
\left[ \begin{array}{c} \dot {\Delta v _ {\mathrm{N}}} (t) \\ \dot {\varphi_ {\mathrm{W}}} (t) \end{array} \right] = \left[ \begin{array}{c c} 0 & - g \\ \frac {1}{R} & 0 \end{array} \right] \left[ \begin{array}{c} \Delta v _ {\mathrm{N}} (t) \\ \varphi_ {\mathrm{W}} (t) \end{array} \right] + \left[ \begin{array}{c} \nabla_ {\mathrm{N}} (t) \\ \varepsilon_ {\mathrm{W}} (t) \end{array} \right]
$$

求实现对惯导系统校正的控制量。

解 由于采用速度组合,量测量为两子系统的北向速度之差

$$
\begin{array}{r l} & Z _ {k} = [ v _ {\mathrm{N}} + \Delta v _ {\mathrm{N}} (t _ {k}) ] - [ v _ {\mathrm{N}} + \Delta v _ {\mathrm{ND}} (t _ {k}) ] + V _ {k} = \\ & \qquad \Delta v _ {\mathrm{N}} (t _ {k}) - \Delta v _ {\mathrm{ND}} (t _ {k}) + V _ {k} \end{array}
$$

式中， $v_{N}$ 为载体的真实速度； $\Delta v_{N}$ 和 $\Delta v_{ND}$ 分别为惯导和多普勒系统的速度误差； $V_{k}$ 为量测噪声。

设量测的采样周期为 $t_{k+1}-t_{k}=T$ 。考虑校正控制后，经离散化后惯导的状态方程为

$$
\left[ \begin{array}{c} \Delta v _ {k + 1} \\ \varphi_ {k + 1} \end{array} \right] = \left[ \begin{array}{c c} 1 & - g T \\ \frac {T}{R} & 1 \end{array} \right] \left[ \begin{array}{c} \Delta v _ {k} \\ \varphi_ {k} \end{array} \right] + \Gamma \left[ \begin{array}{c} \nabla_ {k} \\ \varepsilon_ {k} \end{array} \right] + B _ {k} u _ {k}
$$

为书写方便,上式中略去了下标 N 和 W。此外,在一步转移阵的计算中,略去了关于 T 的高次项,此处认为滤波周期较短。

显然，对 $\Delta v_{k}$ 的校正属于脉冲控制，而对平台水平姿态角的校正则属于持续性控制，因为控制平台的姿态角必须通过对陀螺施矩才能逐渐消除姿态误差。设两种控制分别为

脉冲控制

持续控制

$$
\begin{array}{c} \boldsymbol {B} _ {k} ^ {\mathrm{p}} \boldsymbol {u} _ {k} ^ {\mathrm{p}} \\ \boldsymbol {B} _ {k} ^ {\mathrm{c}} \boldsymbol {u} _ {k} ^ {\mathrm{c}} \end{array}
$$

由于仅对 $\Delta v_{k}$ 作脉冲校正, 所以由式(5.4.3)

$$
\boldsymbol {u} _ {k} ^ {\mathrm{p}} = \left[ \begin{array}{c} - \Delta \hat {v} _ {k} ^ {-} \\ 0 \end{array} \right]
$$

$\Delta v_{k}$ 经脉冲校正后， $\Delta\hat{v}_{k}^{+}=0$ 。

由式 $(5.4.16)$ ，持续校正控制应满足

$$
\boldsymbol {B} _ {k} ^ {\mathrm{c}} \boldsymbol {u} _ {k} ^ {\mathrm{c}} = - \boldsymbol {\Phi} _ {k + 1, k} \left[ \begin{array}{l} 0 \\ \hat {\varphi} _ {k} \end{array} \right]\tag{1}
$$

且速度校正和平台误差角校正是各自进行的，即(5.4.10)式中， $E(t)=I$ ，所以

$$
\boldsymbol {B} _ {k} ^ {\mathrm{c}} = \int_ {t _ {k}} ^ {t _ {k + 1}} \boldsymbol {\Phi} (t _ {k + 1}, t) \mathrm{d} t = \int_ {t _ {k}} ^ {t _ {k + 1}} \left[ \begin{array}{c c} 1 & - g (t _ {k + 1} - t) \\ \frac {t _ {k + 1} - t}{R} & 1 \end{array} \right] \mathrm{d} t = \left[ \begin{array}{c c} T & - \frac {g}{2} T ^ {2} \\ \frac {T ^ {2}}{2 R} & T \end{array} \right]\tag{2}
$$

将式(2)代入式(1)

$$
\left[ \begin{array}{c c} T & - \frac {g}{2} T ^ {2} \\ \frac {T ^ {2}}{2 R} & T \end{array} \right] \left[ \begin{array}{l} u _ {k} ^ {\mathrm{cv}} \\ u _ {k} ^ {\mathrm{cv}} \end{array} \right] = - \left[ \begin{array}{c c} 1 & - g T \\ \frac {T}{R} & 1 \end{array} \right] \left[ \begin{array}{l} 0 \\ \hat {\varphi} _ {k} \end{array} \right]
$$

即

$$
T u _ {k} ^ {\mathrm{cv}} - \frac {g T ^ {2}}{2} u _ {k} ^ {\mathrm{c} \varphi} = g T \hat {\varphi} _ {k}\tag{3}
$$

$$
\frac {T ^ {2}}{2 R} u _ {k} ^ {\mathrm{cv}} + T u _ {k} ^ {\mathrm{c} \varphi} = - \hat {\varphi} _ {k}\tag{4}
$$

由于 $\frac{T^2}{2R} \approx 0$ ，所以由式(4)得

$$
u _ {k} ^ {\mathrm{c} \varphi} = - \frac {\hat {\varphi} _ {k}}{T}
$$

将上式代入式(3)得

$$
u _ {k} ^ {\mathrm{cv}} = g \hat {\varphi} _ {k} - \frac {g T}{2} \frac {\hat {\varphi} _ {k}}{T} = \frac {g}{2} \hat {\varphi} _ {k}
$$

$u_{k}^{c\varphi}$ 和 $u_{k}^{cv}$ 在 $[t_{k}, t_{k+1})$ 时间段内始终连续地作用于惯导系统，其中 $u_{k}^{c\varphi}$ 以指令形式对陀螺施矩，使陀螺进动，通过平台系统的修正回路控制平台转动，以消除 $\hat{\varphi}_{k}$ 。当 $\hat{\varphi}_{k}^{+}=0$ 时，指令也为零。 $u_{k}^{cv}$ 以加速度补偿形式对加速度计输出作修正，也是持续施加的控制量。

从上述分析可以看出,脉冲校正无交叉耦合效应,持续校正却因为系统的转移特性存在交叉耦合效应。

## 5.5 连续系统的估计直接反馈控制

系统和量测同式(5.3.1)和式(5.3.2)。

连续系统的控制量一般都是控制状态的变化率。控制规律可以从离散系统的结果得到。由式(5.4.16)和式(5.4.18)有

$$
\boldsymbol {B} _ {k} ^ {\mathrm{c}} \boldsymbol {u} _ {k} ^ {\mathrm{c}} = - \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {K} _ {k} \boldsymbol {Z} _ {k}\tag{5.5.1}
$$

式中

$$
\boldsymbol {B} _ {k} ^ {\mathrm{c}} = \int_ {t _ {k}} ^ {t _ {k} + T} \boldsymbol {\Phi} _ {t _ {k} + T, t} \boldsymbol {E} (t) \mathrm{d} t
$$

式中 T 为滤波周期，即 $T = t_{k+1} - t_{k}$ 。

式(5.5.1)两边除以 T，并取 $T \rightarrow 0$ 时的极限

$$
\lim _ {T \to 0} \frac {\pmb {B} _ {k} ^ {\mathrm{c}}}{T} \pmb {u} _ {k} ^ {\mathrm{c}} = - \lim _ {T \to 0} \pmb {\Phi} _ {k + 1, k} \frac {\pmb {K} _ {k}}{T} \pmb {Z} _ {k}\tag{5.5.2}
$$

先考察式(5.5.2)的左侧。根据积分中值定理，由式(5.4.13)得

$$
\lim _ {T \rightarrow 0} \frac {\boldsymbol {B} _ {k} ^ {\mathrm{c}}}{T} = \lim _ {T \rightarrow 0} \frac {1}{T} \int_ {t _ {k}} ^ {t _ {k} + T} \boldsymbol {\Phi} _ {t _ {k} + T, t} \boldsymbol {E} (t) \mathrm{d} t = \lim _ {T \rightarrow 0} \frac {1}{T} \boldsymbol {\Phi} _ {t _ {k} + T, \xi} \boldsymbol {E} (\xi) \cdot T
$$

式中 $t_k \leqslant \xi \leqslant t_k + T$ 。当 $T \to 0$ 时， $\xi \to t_k, \Phi_{t_k + T, \xi} \to I$ ，而 $t_k$ 成为连续变量 $t, u_k^c$ 也成为连续控制量 $\boldsymbol{u}(t)$ ，所以

$$
\lim _ {T \to 0} \frac {\pmb {B} _ {k} ^ {\mathrm{c}}}{T} \pmb {u} _ {k} ^ {\mathrm{c}} = \pmb {E} (t) \pmb {u} (t)
$$

再考察式(5.5.2)的右侧。由式(2.3.14)

$$
\mathbf {K} _ {k} = \mathbf {K} (t _ {k - 1} + T) = \mathbf {P} (t _ {k - 1} + T) \mathbf {H} ^ {\mathrm{T}} (t _ {k - 1} + T) \left[ \frac {\mathbf {r} (t _ {k - 1} + T)}{T} \right] ^ {- 1}
$$

所以

$$
\begin{array}{r l} & {\underset {T \to 0} {\lim} \pmb {\Phi} _ {k + 1, k} \frac {\pmb {K} _ {k}}{T} \pmb {Z} _ {k} =} \\ & {\quad \underset {T \to 0} {\lim} \pmb {\Phi} _ {k + 1, k} \frac {1}{T} \pmb {P} (t _ {k - 1} + T) \pmb {H} ^ {\mathrm{T}} (t _ {k - 1} + T) \pmb {r} ^ {- 1} (t _ {k - 1} + T) T \pmb {Z} (t _ {k}) =} \\ & {\quad \pmb {I P (t)} \pmb {H} ^ {\mathrm{T}} (t) \pmb {r} ^ {- 1} (t) \pmb {Z} (t) = \pmb {P (t)} \pmb {H} ^ {\mathrm{T}} (t) \pmb {r} ^ {- 1} (t) \pmb {Z} (t)} \end{array}
$$

根据式 $(2.3.20)$

$$
\lim _ {T \rightarrow 0} \Phi_ {k + 1, k} \frac {\mathbf {K} _ {k}}{T} \mathbf {Z} _ {k} = \mathbf {K} (t) \mathbf {Z} (t)
$$

因此式 $(5.5.2)$ 成

$$
\boldsymbol {E} (t) \boldsymbol {u} (t) = - \boldsymbol {K} (t) \boldsymbol {Z} (t)\tag{5.5.3}
$$

这就是连续系统的估计直接反馈控制规律。与离散系统一样，控制可以直接从量测计算得到，如果各个状态都能单独控制，则 $E(t)$ 为单位阵。

## 习题

5-1 设 $X$ 为 $n$ 维随机向量，均值为 $m_X$ ，方差为 $C_X, P$ 为 $n \times n$ 的方阵，证明：

$$
E \left\{\boldsymbol {X} ^ {\mathrm{T}} \boldsymbol {P} \boldsymbol {X} \right\} = m _ {X} ^ {\mathrm{T}} \boldsymbol {P} m _ {X} + \operatorname{tr} \left[ \boldsymbol {P C} _ {X} \right]
$$

## 5-2 设系统为

$$
X _ {k + 1} = X _ {k} + u _ {k} + W _ {k}
$$

其中 $X$ 和 $u$ 都是标量， $\{W_k\}$ 是均值为零、方差为 $r$ 的正态白噪声序列，初始状态 $X_0$ 服从正态分布，均值为 $m_{X_0}$ ，方差为 $C_{X_0}$ ，指标函数是

$$
J _ {N} = E \{\sum_ {k = 0} ^ {N - 1} (X _ {k} ^ {2} + u _ {k} ^ {2}) + X _ {N} ^ {2} \}
$$

如果 $X_{k}$ 能够得到，求指标函数为极小的控制规律和最小指标函数。当 $N \to \infty$ 时，求极限控制规律。

## 5-3 设系统为

$$
X _ {k + 1} = \Phi X _ {k} + B u _ {k} + W _ {k}
$$

其中 X 和 u 都是标量， $\{W_{k}\}$ 为正态分布的零均值白噪声序列，初始状态 $X_{0}$ 服从正态分布，指标函数是

$$
J _ {N} = E \{\sum_ {k = 1} ^ {N} X _ {k} ^ {2} \}
$$

如果 $X_{k}$ 能够得到, 求指标函数为极小的控制规律。

5-4 例5-1中，如果 $\Delta v_{k}$ 并未施加脉冲校正控制，即 $\Delta \hat{v}_k$ 并未得到校正，试求持续校正控制，并列写出一步预测 $(\hat{\mathbf{X}}_{k + 1 / k})$ 方程和估计 $(\hat{\mathbf{X}}_{k + 1})$ 计算方程。

5-5 设 $X_{1}$ 和 $X_{2}$ 为某系统的误差量， $X_{1}$ 为数字量， $X_{2}$ 为物理量，满足如下方程：

$$
\begin{array}{l} \dot {X} _ {1} = a X _ {2} + w _ {1} \\ \dot {X} _ {2} = b X _ {1} + w _ {2} \end{array}
$$

对 $X_{1}$ 可作等周期测量, 测量周期为 $T(T \ll 1)$ 。 $t_{k}$ 时刻的量测量为

$$
Z _ {k} = X _ {1} (t _ {k}) + V _ {k}
$$

其中 a 和 b 均为常数， $w_{1}, w_{2}$ 和 $V_{k}$ 均为白噪声， $w_{1}$ 和 $w_{2}$ 的方差强度分别为 $q_{1}$ 和 $q_{2}, V_{k}$ 的方差为 R。

(1) 列写出以 $X_{1}$ 和 $X_{2}$ 为状态变量的系统方程；

(2) 求出对系统离散化的一步转移矩阵；

(3) 若卡尔曼滤波器对 $X_{1}$ 和 $X_{2}$ 在 $t_{k}$ 时刻的估计值为 $\hat{X}_{1}(t_{k})$ 和 $\hat{X}_{2}(t_{k})$ ，求对 $X_{1}$ 和 $X_{2}$ 的校正控制量，并说明该控制量施加的时间。

# 第六章 卡尔曼滤波的推广

第二章已经讨论了最优线性预测和卡尔曼滤波的基本方程，解决了三种最优估计类型中的最优预测 $(\hat{X}_{k / k - 1})$ 和最优滤波 $(\hat{X}_k)$ 两类问题。本章将在线性卡尔曼滤波理论基础上推广线性滤波的基本算法，解决第三种最优滤波类型——平滑和解决非线性系统卡尔曼滤波的算法。

## 6.1 最优线性平滑

## 6.1.1 最优线性平滑的物理意义

什么是平滑？如果已知量测值 $Z_{1}, Z_{2}, \cdots, Z_{k}$ ，要求找出 $X_{j}$ 的最优线性估值 $\hat{X}_{j/k}$ ，当 j < k 时，称为平滑。根据 k 和 j 的具体变化情况，最优平滑可分为三种类型。

## 1. 固定点平滑

令 $\bar{Z}_{k}=\left[Z_{1}^{T}\quad Z_{2}^{T}\quad\cdots\quad Z_{k}^{T}\right]^{T}$ 为 k 时刻内所有量测值组成的向量，则利用 $\bar{Z}_{k}$ 来估计 $0\sim k-1$ 时刻中某个固定时刻 $j(k=j+1,j+2,\cdots)$ 状态向量 $X_{j}$ 的这种平滑称为固定点平滑。平滑的输出为 $\hat{X}_{j/j+1},\hat{X}_{j/j+2},\cdots$ 。

在实际应用中,如果对某项实验或某个过程中某一时刻状态的估计显得特别重要时,常采用固定点平滑。例如利用观测人造卫星轨道的数据来估计其进入轨道时的初始状态。

## 2. 固定滞后平滑

利用 $\bar{Z}_{k}$ 来估计k-N时刻的状态 $\hat{X}_{k-N}$ ，N为某个确定的固定滞后值，这种平滑称为固定滞后平滑。平滑输出为 $\hat{X}_{k-N/k}(k=N,N+1,\cdots)$ 。它是一种在线估计的方法，只是估计的时间有延迟而已。

在许多通讯系统中,信号的传输一般都有延迟。如果信号估计的精度是要求的主要项目,则常采用固定滞后平滑。

## 3. 固定区间平滑

利用固定的时间区间 $(0,M]$ 中所得到的所有量测值 $\bar{Z}_{M}=[Z_{1}^{T}\quad Z_{2}^{T}\quad\cdots\quad Z_{M}^{T}]^{T}$ 来估计这个区间中每个时刻的状态 $X_{k}(k=1,2,\cdots,M)$ ，这种平滑称为固定区间平滑。平滑的输出是 $\hat{X}_{k/M}$ 。这种固定区间平滑在惯性导航系统中应用较多。

## 6.1.2 固定点平滑

## 1. 固定点平滑方程

利用卡尔曼滤波基本方程和扩充状态变量的方法来推导固定点平滑方程,推导过程比较简单。

设系统状态方程和量测方程为

$$
\boldsymbol {X} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {X} _ {k} + \boldsymbol {\Gamma} _ {k} \boldsymbol {W} _ {k}\tag{6.1.1}
$$

$$
\mathbf {Z} _ {k} = \mathbf {H} _ {k} \mathbf {X} _ {k} + \mathbf {V} _ {k}\tag{6.1.2}
$$

$\{W_{k}\}$ 和 $\{V_{k}\}$ 都是白噪声序列。如果以 j 表示某固定时刻，固定点平滑就是 $\hat{X}_{j/k}(k \geqslant j)$ 。

增加一个新的状态向量 $X_{k}^{a}$ ，其递推方程为

$$
\boldsymbol {X} _ {k + 1} ^ {a} = \boldsymbol {X} _ {k} ^ {a} \quad k \geqslant j\tag{6.1.3}
$$

初始值 $X_{j}^{a} = X_{j}$ ，则

$$
\boldsymbol {X} _ {k + 1} ^ {a} = \boldsymbol {X} _ {j}
$$

这意味着增加的这个状态向量虽然在形式上有线性递推关系，但实际上它就是固定点的状态向量 $X_{j}$ 。令 $k$ 时刻对 $k + 1$ 时刻的状态 $X_{k + 1}^{a}$ 的预测为 $\hat{X}_{k + 1 / k}^{a}$ ，则

$$
\hat {\boldsymbol {X}} _ {k + 1 / k} ^ {a} = \hat {\boldsymbol {X}} _ {j / k}\tag{6.1.4}
$$

令 $P_{k+1/k}^{aa}$ 表示平滑均方误差阵，则

$$
\mathbf {P} _ {k + 1 / k} ^ {a a} = \mathbf {P} _ {j / k} \stackrel {\mathrm{def}} {=} E [ (\mathbf {X} _ {j} - \hat {\mathbf {X}} _ {j / k}) (\mathbf {X} _ {j} - \hat {\mathbf {X}} _ {j / k}) ^ {\mathrm{T}} ]\tag{6.1.5}
$$

状态扩增后的系统和量测方程为

$$
\left[ \begin{array}{c} \boldsymbol {X} _ {k + 1} \\ \boldsymbol {X} _ {k + 1} ^ {a} \end{array} \right] = \left[ \begin{array}{c c} \boldsymbol {\Phi} _ {k + 1, k} & \boldsymbol {0} \\ \boldsymbol {0} & \boldsymbol {I} \end{array} \right] \left[ \begin{array}{c} \boldsymbol {X} _ {k} \\ \boldsymbol {X} _ {k} ^ {a} \end{array} \right] + \left[ \begin{array}{c} \boldsymbol {\Gamma} _ {k} \\ \boldsymbol {0} \end{array} \right] \boldsymbol {W} _ {k}\tag{6.1.6}
$$

$$
\mathbf {Z} _ {k} = \left[ \begin{array}{l l} \mathbf {H} _ {k} & \mathbf {0} \end{array} \right] \left[ \begin{array}{l} \mathbf {X} _ {k} \\ \mathbf {X} _ {k} ^ {a} \end{array} \right] + \mathbf {V} _ {k}\tag{6.1.7}
$$

当 k=j 时, 状态向量满足以下关系:

$$
\left[ \begin{array}{l l} \boldsymbol {X} _ {j} ^ {\mathrm{T}} & \boldsymbol {X} _ {j} ^ {a \mathrm{T}} \end{array} \right] ^ {\mathrm{T}} = \left[ \begin{array}{l l} \boldsymbol {X} _ {j} ^ {\mathrm{T}} & \boldsymbol {X} _ {j} ^ {\mathrm{T}} \end{array} \right] ^ {\mathrm{T}}
$$

将离散系统卡尔曼滤波一步预测基本方程应用于系统式(6.1.6)和量测式(6.1.7)，有

$$
\begin{array}{r l} \left[ \begin{array}{c} \hat {\mathbf {X}} _ {k + 1 / k} \\ \hat {\mathbf {X}} _ {k + 1 / k} ^ {a} \end{array} \right] & = \left[ \begin{array}{c c} \boldsymbol {\Phi} _ {k + 1, k} & \mathbf {0} \\ \mathbf {0} & \mathbf {I} \end{array} \right] \left[ \begin{array}{c} \hat {\mathbf {X}} _ {k / k} \\ \hat {\mathbf {X}} _ {k / k} ^ {a} \end{array} \right] = \\ & \left[ \begin{array}{c c} \boldsymbol {\Phi} _ {k + 1, k} & \mathbf {0} \\ \mathbf {0} & \mathbf {I} \end{array} \right] \left[ \left[ \begin{array}{c} \hat {\mathbf {X}} _ {k / k - 1} \\ \hat {\mathbf {X}} _ {k / k - 1} ^ {a} \end{array} \right] + \left[ \begin{array}{c} \mathbf {K} _ {k} \\ \mathbf {K} _ {k} ^ {a} \end{array} \right] (\mathbf {Z} _ {k} - [ \mathbf {H} _ {k}, \mathbf {0} ] [ \begin{array}{c} \hat {\mathbf {X}} _ {k / k - 1} \\ \hat {\mathbf {X}} _ {k / k - 1} ^ {a} \end{array} ]) \right] = \\ & \left[ \left[ \begin{array}{c c} \boldsymbol {\Phi} _ {k + 1, k} & \mathbf {0} \\ \mathbf {0} & \mathbf {I} \end{array} \right] - \left[ \begin{array}{c c} \mathbf {K} _ {k} ^ {*} \\ \mathbf {K} _ {k} ^ {a} \end{array} \right] [ \mathbf {H} _ {k}, \mathbf {0} ] \right] [ \begin{array}{c} \hat {\mathbf {X}} _ {k / k - 1} \\ \hat {\mathbf {X}} _ {k / k - 1} ^ {a} \end{array} ] + [ \begin{array}{c} \mathbf {K} _ {k} ^ {*} \\ \mathbf {K} _ {k} ^ {a} \end{array} ] \mathbf {Z} _ {k} \end{array}\tag{6.1.8}
$$

式中

当 k=j 时, 同样有初始条件

$$
\left[ \hat {\boldsymbol {X}} _ {j / j - 1} ^ {\mathrm{T}} \quad \hat {\boldsymbol {X}} _ {j / j - 1} ^ {a \mathrm{T}} \right] ^ {\mathrm{T}} = \left[ \hat {\boldsymbol {X}} _ {j / j - 1} ^ {\mathrm{T}} \quad \hat {\boldsymbol {X}} _ {j / j - 1} ^ {\mathrm{T}} \right] ^ {\mathrm{T}}
$$

值得注意的是，式（6.1.8）中的增益阵 $\left[K_{k}^{*T}K_{k}^{aT}\right]^{T}$ 相当于第二章中的一步预测方程的增益阵 $K_{k}^{*}$ 。

系统式(6.1.6)扩增后的状态估值均方误差阵 $P_{k+1/k}^{*a}$ 为

$$
\begin{array}{r l} \boldsymbol {P} _ {k + 1 / k} ^ {* a} & \stackrel {\text {def}} {=} E \left[ \left(\left[ \begin{array}{l} \boldsymbol {X} _ {k + 1} \\ \boldsymbol {X} _ {k + 1} ^ {a} \end{array} \right] - \left[ \begin{array}{l} \hat {\boldsymbol {X}} _ {k + 1 / k} \\ \hat {\boldsymbol {X}} _ {k + 1 / k} ^ {a} \end{array} \right]\right) \left(\left[ \begin{array}{l} \boldsymbol {X} _ {k + 1} ^ {\mathrm{T}} \\ \boldsymbol {X} _ {k + 1} ^ {a \mathrm{T}} \end{array} \right] - \left[ \begin{array}{l} \hat {\boldsymbol {X}} _ {k + 1 / k} ^ {\mathrm{T}} \\ \hat {\boldsymbol {X}} _ {k + 1 / k} ^ {a \mathrm{T}} \end{array} \right]\right) ^ {\mathrm{T}} \right] = \\ & E \left[ \left[ \begin{array}{l} \widetilde {\boldsymbol {X}} _ {k + 1 / k} \\ \widetilde {\boldsymbol {X}} _ {k + 1 / k} ^ {a} \end{array} \right] \left[ \begin{array}{l} \widetilde {\boldsymbol {X}} _ {k + 1 / k} ^ {\mathrm{T}} \\ \widetilde {\boldsymbol {X}} _ {k + 1 / k} ^ {a \mathrm{T}} \end{array} \right] ^ {\mathrm{T}} \right] = E \left[ \begin{array}{l l} \widetilde {\boldsymbol {X}} _ {k + 1 / k} \widetilde {\boldsymbol {X}} _ {k + 1 / k} ^ {\mathrm{T}} & \widetilde {\boldsymbol {X}} _ {k + 1 / k} \widetilde {\boldsymbol {X}} _ {k + 1 / k} ^ {a \mathrm{T}} \\ \widetilde {\boldsymbol {X}} _ {k + 1 / k} ^ {a} \widetilde {\boldsymbol {X}} _ {k + 1 / k} ^ {\mathrm{T}} & \widetilde {\boldsymbol {X}} _ {k + 1 / k} ^ {a} \widetilde {\boldsymbol {X}} _ {k + 1 / k} ^ {a \mathrm{T}} \end{array} \right] = \end{array}
$$

$$
\left[ \begin{array}{l l} \boldsymbol {P} _ {k + 1 / k} & \boldsymbol {P} _ {k + 1 / k} ^ {a T} \\ \boldsymbol {P} _ {k + 1 / k} ^ {a} & \boldsymbol {P} _ {k + 1 / k} ^ {a a} \end{array} \right]\tag{6.1.9}
$$

仿照一步预测基本方程中的 $P_{k+1/k}$ 阵得

$$
\begin{array}{r l} \left[ \begin{array}{c c} \boldsymbol {P} _ {k + 1 / k} & \boldsymbol {P} _ {k + 1 / k} ^ {a \mathrm{T}} \\ \boldsymbol {P} _ {k + 1 / k} ^ {a} & \boldsymbol {P} _ {k + 1 / k} ^ {a a} \end{array} \right] & = \left(\left[ \begin{array}{c c} \boldsymbol {\Phi} _ {k + 1, k} & \boldsymbol {0} \\ \boldsymbol {0} & \boldsymbol {I} \end{array} \right] - \left[ \begin{array}{c} \boldsymbol {K} _ {k} ^ {*} \\ \boldsymbol {K} _ {k} ^ {a} \end{array} \right] [ \boldsymbol {H} _ {k}, \boldsymbol {0} ]\right) \cdot \\ & \left[ \begin{array}{c c} \boldsymbol {P} _ {k / k - 1} & \boldsymbol {P} _ {k / k - 1} ^ {a \mathrm{T}} \\ \boldsymbol {P} _ {k / k - 1} ^ {a} & \boldsymbol {P} _ {k / k - 1} ^ {a a} \end{array} \right] \left[ \begin{array}{c c} \boldsymbol {\Phi} _ {k + 1, k} & \boldsymbol {0} \\ \boldsymbol {0} & \boldsymbol {I} \end{array} \right] ^ {\mathrm{T}} + \\ & \left[ \begin{array}{c} \boldsymbol {\Gamma} _ {k} \\ \boldsymbol {0} \end{array} \right] Q _ {k} [ \boldsymbol {\Gamma} _ {k} ^ {\mathrm{T}} \quad \boldsymbol {0} ] \end{array}
$$

考虑到均方误差阵的对称性,则上式转置后可得

$$
\begin{array}{r l} & {\left[ \begin{array}{l l} P _ {k + 1 / k} & P _ {k + 1 / k} ^ {a \mathrm{T}} \\ P _ {k + 1 / k} ^ {a} & P _ {k + 1 / k} ^ {a a} \end{array} \right] = \left[ \begin{array}{c c} \Phi_ {k + 1, k} & 0 \\ 0 & I \end{array} \right] \left[ \begin{array}{c c} P _ {k / k - 1} & P _ {k / k - 1} ^ {a \mathrm{T}} \\ P _ {k / k - 1} ^ {a} & P _ {k / k - 1} ^ {a a} \end{array} \right]} \\ & {\left(\left[ \begin{array}{c c} \Phi_ {k + 1, k} ^ {\mathrm{T}} & 0 \\ 0 & I \end{array} \right] - \left[ \begin{array}{c c} H _ {k} ^ {\mathrm{T}} \\ 0 \end{array} \right] \left[ \begin{array}{c c} K _ {k} ^ {* \mathrm{T}} & K _ {k} ^ {a \mathrm{T}} \end{array} \right]\right) +} \\ & {\left[ \begin{array}{l} \Gamma_ {k} \\ 0 \end{array} \right] Q _ {k} [ \Gamma_ {k} ^ {\mathrm{T}} & 0 ] =} \\ & {\left[ \begin{array}{c c} \Phi_ {k + 1, k} P _ {k / k - 1} & \Phi_ {k + 1, k} P _ {k / k - 1} ^ {a \mathrm{T}} \\ P _ {k / k - 1} ^ {a} & P _ {k / k - 1} ^ {a a} \end{array} \right] \left[ \begin{array}{c c} \Phi_ {k + 1, k} ^ {\mathrm{T}} - H _ {k} ^ {\mathrm{T}} K _ {k} ^ {* \mathrm{T}} & - H _ {k} ^ {\mathrm{T}} K _ {k} ^ {a \mathrm{T}} \\ 0 & I \end{array} \right] +} \\ & {\left[ \begin{array}{l l} \Gamma_ {k} Q _ {k} \Gamma_ {k} ^ {\mathrm{T}} & 0 \\ 0 & 0 \end{array} \right] =} \\ & {\left[ \begin{array}{l l} \Phi_ {k + 1 / k} P _ {k / k - 1} (\Phi_ {k + 1, k} ^ {\mathrm{T}} - H _ {k} ^ {\mathrm{T}} K _ {k} ^ {* \mathrm{T}}) & - \Phi_ {k + 1, k} P _ {k / k - 1} H _ {k} ^ {\mathrm{T}} K _ {k} ^ {a \mathrm{T}} + \Phi_ {k + 1, k} P _ {k / k - 1} ^ {a \mathrm{T}} \\ P _ {k / k - 1} ^ {a} (\Phi_ {k + 1, k} ^ {\mathrm{T}} - H _ {k} ^ {\mathrm{T}} K _ {k} ^ {* \mathrm{T}}) & - P _ {k / k - 1} ^ {a} H _ {k} ^ {\mathrm{T}} K _ {k} ^ {a \mathrm{T}} + P _ {k / k - 1} ^ {a a} \end{array} \right] +} \\ & {\left[ \begin{array}{l l} \Gamma_ {k} Q _ {k} \Gamma_ {k} ^ {\mathrm{T}} & 0 \\ 0 & 0 \end{array} \right]} \end{array}
$$

式中均方误差阵各元素初始条件为

$$
\left[ \begin{array}{l l} \boldsymbol {P} _ {j / j - 1} & \boldsymbol {P} _ {j / j - 1} ^ {a \mathrm{T}} \\ \boldsymbol {P} _ {j / j - 1} ^ {a} & \boldsymbol {P} _ {j / j - 1} ^ {a a} \end{array} \right] = \left[ \begin{array}{l l} \boldsymbol {P} _ {j / j - 1} & \boldsymbol {P} _ {j / j - 1} \\ \boldsymbol {P} _ {j / j - 1} & \boldsymbol {P} _ {j / j - 1} \end{array} \right]
$$

因为式(6.1.8)中的增益阵 $\left[K_{k}^{*T}\quad K_{k}^{aT}\right]^{T}$ 相当于一步预测方程中的 $K_{k}^{*}$ 阵，故可写成

$$
\begin{array}{r l} \left[ \begin{array}{c} \mathbf {K} _ {k} ^ {*} \\ \mathbf {K} _ {k} ^ {a} \end{array} \right] & = \left[ \begin{array}{c c} \boldsymbol {\Phi} _ {k + 1, k} & \mathbf {0} \\ \mathbf {0} & \mathbf {I} \end{array} \right] \left[ \begin{array}{c c} \mathbf {P} _ {k / k - 1} & \mathbf {P} _ {k / k - 1} ^ {a T} \\ \mathbf {P} _ {k / k - 1} ^ {a} & \mathbf {P} _ {k / k - 1} ^ {a a} \end{array} \right] \left[ \begin{array}{c} \mathbf {H} _ {k} ^ {T} \\ \mathbf {0} \end{array} \right] (\mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {T} + \mathbf {R} _ {k}) ^ {- 1} = \\ & \left[ \begin{array}{c} \boldsymbol {\Phi} _ {k + 1, k} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {T} \\ \mathbf {P} _ {k / k - 1} ^ {a} \mathbf {H} _ {k} ^ {T} \end{array} \right] (\mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {T} + \mathbf {R} _ {k}) ^ {- 1} \end{array}\tag{6.1.11}
$$

式(6.1.8)、式(6.1.10)和式(6.1.11)为状态扩增后的滤波方程。从形式上看，它们的阶次增加了一倍，但都可分解成 $n$ 阶原系统的滤波方程和固定点平滑方程。于是得固定点平滑方程为

$$
\hat {\mathbf {X}} _ {j / k} = \hat {\mathbf {X}} _ {j / k - 1} + \mathbf {K} _ {k} ^ {a} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1}) = \hat {\mathbf {X}} _ {j / k - 1} + \mathbf {K} _ {k} ^ {a} \widetilde {\mathbf {Z}} _ {k}\tag{6.1.12}
$$

$$
\boldsymbol {K} _ {k} ^ {a} = \boldsymbol {P} _ {k / k - 1} ^ {a} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}) ^ {- 1}\tag{6.1.13}
$$

$$
\boldsymbol {P} _ {k + 1 / k} ^ {a} = \boldsymbol {P} _ {k / k - 1} ^ {a} \left(\boldsymbol {\Phi} _ {k + 1, k} - \boldsymbol {K} _ {k} ^ {*} \boldsymbol {H} _ {k}\right) ^ {\mathrm{T}}\tag{6.1.14}
$$

式中状态向量初始值为 $\hat{X}_{j / j - 1},\pmb{P}_{k + 1 / k}^{a}$ 均方误差阵初始值为

$$
\boldsymbol {P} _ {j / j - 1} ^ {a} = \boldsymbol {P} _ {j / j - 1}
$$

整个平滑过程是:从0时刻开始,滤波器用滤波方程解算出 $P_{k+1/k}$ 和 $K_{k}^{*}$ ,并估计出 $\hat{X}_{k+1/k}$ ;当 $k\geqslant j$ 时,滤波器用平滑方程解算出 $P_{k+1/k}^{a}$ 和 $K_{k}^{a}$ ,从而解算出 $\hat{X}_{j/k}$ 。

2. 固定点平滑的几点说明

(1) 关于平滑均方误差阵 $P_{j/k}$ 。利用式(6.1.10) 可得到平滑的均方误差阵为

$$
\boldsymbol {P} _ {j / k} = \boldsymbol {P} _ {k + 1 / k} ^ {a a} = \boldsymbol {P} _ {k / k - 1} ^ {a a} - \boldsymbol {P} _ {k / k - 1} ^ {a} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {K} _ {k} ^ {a \mathrm{T}}
$$

或

$$
\boldsymbol {P} _ {j / k} = \boldsymbol {P} _ {j / k - 1} - \boldsymbol {P} _ {k / k - 1} ^ {a} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {K} _ {k} ^ {a \mathrm{T}}\tag{6.1.15}
$$

按平滑方程式(6.1.12)～式(6.1.14)的递推关系,可得固定点j处平滑值 $\hat{X}_{j/k}$ 的非递推形式的均方误差阵(平滑时刻 $i=j,j+1,j+2,\cdots,k$ )为

$$
\begin{array}{r l} & {\pmb {P} _ {j / k} = \pmb {P} _ {j / j - 1} - \sum_ {i = j} ^ {k} (\pmb {P} _ {i / i - 1} ^ {a} \pmb {H} _ {i} ^ {\mathrm{T}} \pmb {K} _ {i} ^ {a \mathrm{T}}) =} \\ & {\qquad \pmb {P} _ {j / j - 1} - \sum_ {i = j} ^ {k} [ \pmb {P} _ {i / i - 1} ^ {a} \pmb {H} _ {i} ^ {\mathrm{T}} (\pmb {H} _ {i} \pmb {P} _ {i / i - 1} \pmb {H} _ {i} ^ {\mathrm{T}} + \pmb {R} _ {i}) ^ {- 1} \pmb {H} _ {i} \pmb {P} _ {i / i - 1} ^ {a \mathrm{T}} ]} \end{array}\tag{6.1.16}
$$

由式(6.1.16)可知，对固定点 $j$ 的平滑均方误差阵 $\pmb{P}_{j / k}$ 比固定点 $j$ 的预测均方误差阵 $\pmb{P}_{j / j - 1}$ 要小，式中最后一项就是均方误差阵减小的值。 $k - j$ 越大，则利用的量测值越多，越能改善 $\hat{\pmb{X}}_{j / k}$ 的均方误差。

（2）从平滑方程可以看出，平滑增益阵 $K_{k}^{a}$ 和均方误差阵 $P_{k+1/k}^{a}$ 与量测无关，因此，可以单独进行计算。它们与滤波方程中的 $P_{k+1/k}$ 和 $K_{k}$ 具有同样性质。

（3）对定常系统， $H_{k}=H,\Phi_{k+1,k}=\Phi$ ，当滤波器趋于稳态时， $K_{k}^{*}=K^{*},P_{k+1/k}=\bar{P}$ ，则

$$
\mathbf {K} _ {k} ^ {a} = \mathbf {P} _ {k / k - 1} ^ {a} \mathbf {H} ^ {\mathrm{T}} (\mathbf {H} \bar {\mathbf {P}} \mathbf {H} ^ {\mathrm{T}} + \mathbf {R}) ^ {- 1}\tag{6.1.17}
$$

$$
\boldsymbol {P} _ {k / k - 1} ^ {a} = \boldsymbol {P} _ {k - 1 / k - 2} ^ {a} (\boldsymbol {\Phi} - \boldsymbol {K} ^ {*} \boldsymbol {H}) ^ {\mathrm{T}} = \overline {{\boldsymbol {P}}} (\boldsymbol {\Phi} ^ {\mathrm{T}} - \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {K} ^ {* \mathrm{T}}) ^ {k - j}\tag{6.1.18}
$$

$$
\boldsymbol {P} _ {j / k} = \overline {{\boldsymbol {P}}} - \sum_ {i = j} ^ {k} \left[ \boldsymbol {P} _ {i / i - 1} ^ {a} \boldsymbol {H} ^ {\mathrm{T}} (\boldsymbol {H} \overline {{\boldsymbol {P}}} \boldsymbol {H} ^ {\mathrm{T}} + \boldsymbol {R}) ^ {- 1} \boldsymbol {H} \boldsymbol {P} _ {i / i - 1} ^ {a \mathrm{T}} \right]\tag{6.1.19}
$$

对定常系统,虽然滤波器在稳态时也是定常的,但因为 $K_{k}^{a}$ 是时变的,所以,固定点平滑器是时变的。

从式(6.1.19)可粗略看出,每步的平滑均方误差都比前一步的小,而第k步减小的程度主要决定于 $P_{k/k-1}^{a}$ 。从式(6.1.18)可看出,如果滤波器是稳定的,则随着k的增大, $P_{k/k-1}^{a}$ 越来越小,它减小的速率由 $(\Phi-K^{*}H)$ 的特征值所决定。根据经验,平滑所需的间隔 $(k-j)$ 一般是滤波器时间常数(决定于 $(\Phi-K^{*}H)$ 的主要特征值)的2\~3倍为好。若间隔再长,进一步改善估计均方误差的作用就会很小。

（4）平滑改善估计均方误差的程度与 $Q_{k}$ 和 $R_{k}$ 有关，或者说与信噪比有关。 $R_{k}$ 越大，平滑改善滤波性能的作用就越小。如果系统噪声大，则从以后的量测值中可以更多地改善固定点状态的估计。如果量测噪声大，则以后的量测再多，估计的改善也不会太大。这一点也是作为是否采用平滑技术来改善估计的依据之一。

（5）如果需要平滑的是一部分状态变量或者是某些状态变量的线性组合 $Y_{k}$ ，有

$$
\mathbf {Y} _ {k} = \mathbf {L} _ {k} \mathbf {X} _ {k}
$$

而 $\mathbf{Y}_k$ 的维数 $p \leqslant n$ ，则

$$
\begin{array}{r} \hat {Y} _ {j / k} = L _ {j} \hat {X} _ {j / k} \\ \hat {Y} _ {j / k - 1} = L _ {j} \hat {X} _ {j / k - 1} \end{array}
$$

平滑器的阶数可以从 n 阶降到 p 阶。

## 6.1.3 固定滞后平滑[119]

设 N 为某一正整数， $t_{k}$ 为某一时刻，则 $t_{k-N}$ 相对 $t_{k}$ 滞后 N 步， $t_{k-N+1}$ 相对 $t_{k+1}$ 滞后 N 步。将 $\hat{X}_{k-N/k}=E^{*}\left[X_{k-N}/Z_{1}Z_{2}\cdots Z_{k}\right], k=N, N+1,\cdots$ 称为 k 时刻固定滞后 N 步的平滑值，该平滑值是 $X_{k-N}$ 基于 $Z_{1}Z_{2}\cdots Z_{k}$ 的最优估计，在 $t_{k}$ 时刻才能计算出。

固定滞后平滑的工程实例颇多,如侦察卫星所摄地面图像向地面站台转发的问题,假设卫星在 k-N 时刻拍摄的图像需经 N 步处理才能完成,也就是在 $t_{k}$ 时刻才能转发,图像信息的判读与图像拍摄时刻卫星的运动状态(姿态、角速度、速度等)的精确度有关。在 $t_{k-N}$ 至 $t_{k}$ 时间段内有 N 组卫星运动状态的量测量可利用,则可根据该 N 组量测量对 $t_{k-N}$ 时刻(即图像拍摄时刻)卫星的运动状态作精确估计,所用方法即为固定滞后平滑。

设系统方程和量测方程为

$$
\begin{array}{r} \boldsymbol {X} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {X} _ {k} + \boldsymbol {W} _ {k} \\ \boldsymbol {Z} _ {k} = \boldsymbol {H} _ {k} \boldsymbol {X} _ {k} + \boldsymbol {V} _ {k} \end{array}
$$

其中 $W_{k}$ 和 $V_{k}$ 为相互独立的零均值白噪声，方差阵分别为 $Q_{k}$ 和 $R_{k}$ 。

记 $X_{k}^{(i)}(i=0,1,\cdots,N)$ 为相对 $t_{k}$ 时刻滞后 i 步的状态，即

$$
\begin{array}{r l} & {\pmb {X} _ {k} ^ {(0)} = \pmb {X} _ {k}, \pmb {X} _ {k} ^ {(1)} = \pmb {X} _ {k - 1}, \dots , \pmb {X} _ {k} ^ {(N)} = \pmb {X} _ {k - N}} \\ & {\pmb {X} _ {k + 1} ^ {(0)} = \pmb {X} _ {k + 1}, \pmb {X} _ {k + 1} ^ {(1)} = \pmb {X} _ {k}, \dots , \pmb {X} _ {k + 1} ^ {(N)} = \pmb {X} _ {k - (N - 1)}, \pmb {X} _ {k + 1} ^ {(N + 1)} = \pmb {X} _ {k - N}} \end{array}
$$

因此有

$$
\begin{array}{r l} \boldsymbol {X} _ {k + 1} ^ {(0)} & = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {X} _ {k} ^ {(0)} + \boldsymbol {W} _ {k} \\ \boldsymbol {X} _ {k + 1} ^ {(1)} & = \boldsymbol {X} _ {k} = \boldsymbol {X} _ {k} ^ {(0)} \\ \boldsymbol {X} _ {k + 1} ^ {(2)} & = \boldsymbol {X} _ {k - 1} = \boldsymbol {X} _ {k} ^ {(1)} \\ & \dots \\ \boldsymbol {X} _ {k + 1} ^ {(N)} & = \boldsymbol {X} _ {k - (N - 1)} = \boldsymbol {X} _ {k} ^ {(N - 1)} \\ \boldsymbol {X} _ {k + 1} ^ {(N + 1)} & = \boldsymbol {X} _ {k - N} = \boldsymbol {X} _ {k} ^ {N} \end{array}
$$

记

$$
\bar {\boldsymbol {X}} _ {k + 1} = \left[ \begin{array}{c} \boldsymbol {X} _ {k + 1} ^ {(0)} \\ \boldsymbol {X} _ {k + 1} ^ {(1)} \\ \vdots \\ \boldsymbol {X} _ {k + 1} ^ {(N)} \\ \boldsymbol {X} _ {k + 1} ^ {(N + 1)} \end{array} \right] \quad \bar {\boldsymbol {X}} _ {k} = \left[ \begin{array}{c} \boldsymbol {X} _ {k} ^ {(0)} \\ \boldsymbol {X} _ {k} ^ {(1)} \\ \vdots \\ \boldsymbol {X} _ {k} ^ {(N)} \\ \boldsymbol {X} _ {k} ^ {(N + 1)} \end{array} \right]
$$

则按上述关系可列写出 $\bar{X}$ 的状态方程和量测方程

$$
\bar {X} _ {k + 1} = \bar {\Phi} _ {k + 1, k} \bar {X} _ {k} + \bar {\Gamma} _ {k} W _ {k}\tag{6.1.20}
$$

$$
\mathbf {Z} _ {k} = \overline {{\mathbf {H}}} _ {k} \overline {{\mathbf {X}}} _ {k} + \mathbf {V} _ {k}\tag{6.1.21}
$$

其中

$$
\overline {{{\boldsymbol {\Phi}}}} _ {k + 1, k} = \left[ \begin{array}{c c c c c c} \boldsymbol {\Phi} _ {k + 1, k} & \mathbf {0} & \mathbf {0} & \dots & \mathbf {0} & \mathbf {0} \\ \boldsymbol {I} & \mathbf {0} & \mathbf {0} & \dots & \mathbf {0} & \mathbf {0} \\ \mathbf {0} & \boldsymbol {I} & \mathbf {0} & \dots & \mathbf {0} & \mathbf {0} \\ & & \dots & & \\ \mathbf {0} & \mathbf {0} & \mathbf {0} & & \boldsymbol {I} & \mathbf {0} \end{array} \right], \quad \overline {{{\boldsymbol {\Gamma}}}} _ {k} = \left[ \begin{array}{c} \boldsymbol {I} \\ \mathbf {0} \\ \vdots \\ \mathbf {0} \end{array} \right], \quad \overline {{{\boldsymbol {H}}}} _ {k} = [ \boldsymbol {H} _ {k} \quad \mathbf {0} \quad \dots \quad \mathbf {0} ]
$$

根据式(2.2.38)、式(2.2.39)和式(2.2.40)一步预测基本方程，并考虑到一步预测均方误差阵为对称阵，得

$$
\overline {{{\boldsymbol {X}}}} _ {k + 1 / k} = \overline {{{\boldsymbol {\Phi}}}} _ {k + 1, k} \overline {{{\boldsymbol {X}}}} _ {k / k - 1} + \overline {{{\boldsymbol {K}}}} _ {k} (\mathbf {Z} _ {k} - \overline {{{\boldsymbol {H}}}} _ {k} \overline {{{\boldsymbol {X}}}} _ {k / k - 1})\tag{6.1.22a}
$$

$$
\overline {{{\boldsymbol {K}}}} _ {k} = \overline {{{\boldsymbol {\Phi}}}} _ {k + 1, k} \overline {{{\boldsymbol {P}}}} _ {k / k - 1} \overline {{{\boldsymbol {H}}}} _ {k} ^ {\mathrm{T}} (\overline {{{\boldsymbol {H}}}} _ {k} \overline {{{\boldsymbol {P}}}} _ {k / k - 1} \overline {{{\boldsymbol {H}}}} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}) ^ {- 1}\tag{6.1.22b}
$$

$$
\overline {{{P}}} _ {k / k - 1} = \overline {{{\Phi}}} _ {k + 1, k} \overline {{{P}}} _ {k / k - 1} (\overline {{{\Phi}}} _ {k + 1, k} ^ {\mathrm{T}} - \overline {{{H}}} _ {k} ^ {\mathrm{T}} \overline {{{K}}} _ {k} ^ {\mathrm{T}}) + \overline {{{\Gamma}}} _ {k} Q _ {k} \overline {{{\Gamma}}} _ {k} ^ {\mathrm{T}}\tag{6.1.22c}
$$

式 $(6.1.22b)$ 可写成

$$
\left[ \begin{array}{c} \overline {{\boldsymbol {K}}} _ {k} ^ {(0)} \\ \overline {{\boldsymbol {K}}} _ {k} ^ {(1)} \\ \dots \\ \overline {{\boldsymbol {K}}} _ {k} ^ {(N + 1)} \end{array} \right] = \left[ \begin{array}{c c c c c c} \boldsymbol {\Phi} _ {k + 1, k} & \mathbf {0} & \mathbf {0} & \dots & \mathbf {0} & \mathbf {0} \\ \boldsymbol {I} & \mathbf {0} & \mathbf {0} & \dots & \mathbf {0} & \mathbf {0} \\ \vdots & \vdots & \vdots & & \vdots & \vdots \\ \mathbf {0} & \mathbf {0} & \mathbf {0} & \dots & \boldsymbol {I} & \mathbf {0} \end{array} \right] \left[ \begin{array}{c c c c} \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, 0} & \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {1, 0} & \dots & \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {N + 1, 0} \\ \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, 1} & \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {1, 1} & \dots & \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {N + 1, 1} \\ \vdots & & & \vdots \\ \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, N + 1} & \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, N + 1} & \dots & \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {N + 1, N + 1} \end{array} \right] \left[ \begin{array}{c} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \\ \mathbf {0} \\ \vdots \\ \mathbf {0} \end{array} \right].
$$

$$
\left[\begin{array}{c c c c}\mathbf {H} _ {k}&\mathbf {0}&\dots&\mathbf {0}\end{array}\right]\left[\begin{array}{c c c c}\overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, 0}&\overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {1, 0}&\dots&\overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {N + 1, 0}\\\overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, 1}&\overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {1, 1}&\dots&\overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {N + 1, 1}\\\vdots&&&\vdots\\\overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, N + 1}&\overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {1, N + 1}&\dots&\overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {N + 1, N + 1}\end{array}\right]\left[\begin{array}{c}\boldsymbol {H} _ {k} ^ {\mathrm{T}}\\\mathbf {0}\\\vdots\\\mathbf {0}\end{array}\right] + \boldsymbol {R} _ {k}\left. \right) ^ {- 1}
$$

式中

$$
\overline {{{\boldsymbol {P}}}} _ {k / k - 1} ^ {j, i} = E \left[ \left(\boldsymbol {X} _ {k - j} - \hat {\boldsymbol {X}} _ {k - j / k - 1}\right) \left(\boldsymbol {X} _ {k - i} - \hat {\boldsymbol {X}} _ {k - i / k - 1}\right) ^ {\mathrm{T}} \right] \quad i, j = 0, 1, 2, \dots , N + 1.
$$

显然有

$$
\overline {{{{\boldsymbol {P}}}}} _ {k / k - 1} ^ {j, i} = (\overline {{{{\boldsymbol {P}}}}} _ {k / k - 1} ^ {i, j}) ^ {\mathrm{T}}
$$

当 $i = 0, j = 0$ 时，

$$
\overline {{{\boldsymbol {P}}}} _ {k / k - 1} ^ {0, 0} = E \left[ \left(\boldsymbol {X} _ {k} - \hat {\boldsymbol {X}} _ {k / k - 1}\right) \left(\boldsymbol {X} _ {k} - \hat {\boldsymbol {X}} _ {k / k - 1}\right) ^ {\mathrm{T}} \right] = \boldsymbol {P} _ {k / k - 1}\tag{6.1.23}
$$

为卡尔曼滤波的一步预测均方误差阵。

展开上式,得

$$
\left[ \begin{array}{c} \overline {{\boldsymbol {K}}} _ {k} ^ {(0)} \\ \overline {{\boldsymbol {K}}} _ {k} ^ {(1)} \\ \vdots \\ \overline {{\boldsymbol {K}}} _ {k} ^ {(N + 1)} \end{array} \right] = \left[ \begin{array}{c} \boldsymbol {\Phi} _ {k + 1, k} \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, 0} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \\ \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, 0} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \\ \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \\ \vdots \\ \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, N} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \end{array} \right] (\boldsymbol {H} _ {k} \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, 0} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}) ^ {- 1}\tag{6.1.24}
$$

式中

$$
\overline {{{\boldsymbol {K}}}} _ {k} ^ {(0)} = \boldsymbol {\Phi} _ {k + 1, k} \overline {{{\boldsymbol {P}}}} _ {k / k - 1} ^ {0, 0} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {H} _ {k} \overline {{{\boldsymbol {P}}}} _ {k / k - 1} ^ {0, 0} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}) ^ {- 1} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}) ^ {- 1}
$$

$$
\overline {{{\boldsymbol {K}}}} _ {k} ^ {(1)} = \overline {{{\boldsymbol {P}}}} _ {k / k - 1} ^ {0, 0} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \left(\boldsymbol {H} _ {k} \overline {{{\boldsymbol {P}}}} _ {k / k - 1} ^ {0, 0} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}\right) ^ {- 1} = \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \left(\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}\right) ^ {- 1}\tag{6.1.25}
$$

根据式 $(2.2.39)$ 和式 $(2.2.4c)$ ，知

$$
\overline {{{{\boldsymbol {K}}}}} _ {k} ^ {(0)} = \boldsymbol {K} _ {k} ^ {*}\tag{6.1.26}
$$

$$
\overline {{{{\boldsymbol {K}}}}} _ {k} ^ {(1)} = \boldsymbol {K} _ {k}\tag{6.1.27}
$$

即 $\bar{K}_{k}^{(0)}$ 为一步预测基本方程中的最佳增益， $\bar{K}_{k}^{(1)}$ 为卡尔曼滤波基本方程中的最佳增益。

式 $(6.1.24)$ 的其余增益为

$$
\overline {{{\boldsymbol {K}}}} _ {k} ^ {(2)} = \overline {{{\boldsymbol {P}}}} _ {k / k - 1} ^ {0, 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {H} _ {k} \overline {{{\boldsymbol {P}}}} _ {k / k - 1} ^ {0, 0} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}) ^ {- 1}\tag{6.1.28a}
$$

$$
\overline {{{{\boldsymbol {K}}}}} _ {k} ^ {(N + 1)} = \overline {{{{\boldsymbol {P}}}}} _ {k / k - 1} ^ {0, N} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {H} _ {k} \overline {{{{\boldsymbol {P}}}}} _ {k / k - 1} ^ {0, 0} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}) ^ {- 1}\tag{6.1.28b}
$$

根据式 $(6.1.25)$ 和式 $(6.1.28)$ ，可归纳出通项公式：

$$
\overline {{{\boldsymbol {K}}}} _ {k} ^ {(i)} = \overline {{{\boldsymbol {P}}}} _ {k / k - 1} ^ {0, i - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \left(\boldsymbol {H} _ {k} \overline {{{\boldsymbol {P}}}} _ {k / k - 1} ^ {0, 0} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}\right) ^ {- 1} \quad i = 1, 2, \dots , N + 1\tag{6.1.29}
$$

记

$$
\boldsymbol {M} = \left(\boldsymbol {H} _ {k} \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, 0} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}\right) ^ {- 1} \boldsymbol {H} _ {k}\tag{6.1.30}
$$

则(6.1.22c)式可以写成

$$
\left[ \begin{array}{c c c c c} \overline {{\boldsymbol {P}}} _ {k + 1 / k} ^ {0, 0} & \overline {{\boldsymbol {P}}} _ {k + 1 / k} ^ {1, 0} & \overline {{\boldsymbol {P}}} _ {k + 1 / k} ^ {2, 0} & \dots & \overline {{\boldsymbol {P}}} _ {k + 1 / k} ^ {N + 1, 0} \\ \overline {{\boldsymbol {P}}} _ {k + 1 / k} ^ {0, 1} & \overline {{\boldsymbol {P}}} _ {k + 1 / k} ^ {1, 1} & \overline {{\boldsymbol {P}}} _ {k + 1 / k} ^ {2, 1} & \dots & \overline {{\boldsymbol {P}}} _ {k + 1 / k} ^ {N + 1, 1} \\ \overline {{\boldsymbol {P}}} _ {k + 1 / k} ^ {0, 2} & \overline {{\boldsymbol {P}}} _ {k + 1 / k} ^ {1, 2} & \overline {{\boldsymbol {P}}} _ {k + 1 / k} ^ {2, 2} & \dots & \overline {{\boldsymbol {P}}} _ {k + 1 / k} ^ {N + 1, 2} \\ \vdots & \vdots & \vdots & & \vdots \\ \overline {{\boldsymbol {P}}} _ {k + 1 / k} ^ {0, N + 1} & \overline {{\boldsymbol {P}}} _ {k + 1 / k} ^ {1, N + 1} & \overline {{\boldsymbol {P}}} _ {k + 1 / k} ^ {2, N + 1} & \dots & \overline {{\boldsymbol {P}}} _ {k + 1 / k} ^ {N + 1, N + 1} \end{array} \right] =
$$

$$
\left[ \begin{array}{c c c c c} \boldsymbol {\Phi} _ {k + 1, k} \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, 0} & \boldsymbol {\Phi} _ {k + 1, k} \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {1, 0} & \boldsymbol {\Phi} _ {k + 1, k} \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {2, 0} & \dots & \boldsymbol {\Phi} _ {k + 1, k} \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {N + 1, 0} \\ \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, 0} & \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {1, 0} & \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {2, 0} & \dots & \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {N + 1, 0} \\ \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, 1} & \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {1, 1} & \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {2, 1} & \dots & \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {N + 1, 1} \\ \dots & \vdots & \vdots & & \vdots \\ \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, N} & \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {1, N} & \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {2, N} & \dots & \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {N + 1, N} \end{array} \right].
$$

$$
\left[ \begin{array}{c c c c c c} \left[ \begin{array}{c c c c c c} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} & \boldsymbol {I} & \boldsymbol {0} & \boldsymbol {0} & \dots & \boldsymbol {0} \\ \boldsymbol {0} & \boldsymbol {0} & \boldsymbol {I} & \boldsymbol {0} & \dots & \boldsymbol {0} \\ \boldsymbol {0} & \boldsymbol {0} & \boldsymbol {0} & \boldsymbol {I} & \dots & \boldsymbol {0} \\ \vdots & \vdots & \vdots & \vdots & & \vdots \\ \boldsymbol {0} & \boldsymbol {0} & \boldsymbol {0} & \boldsymbol {0} & \dots & \boldsymbol {I} \\ \boldsymbol {0} & \boldsymbol {0} & \boldsymbol {0} & \boldsymbol {0} & \dots & \boldsymbol {0} \end{array} \right] - \left[ \begin{array}{c} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \\ \boldsymbol {0} \\ \vdots \\ \boldsymbol {0} \end{array} \right] \left[ \begin{array}{c c c c c c} M \bar {P} _ {k / k - 1} ^ {0, 0} & \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} & M \bar {P} _ {k / k - 1} ^ {0, 0} & M \bar {P} _ {k / k - 1} ^ {0, 1} & \dots & M \bar {P} _ {k / k - 1} ^ {0, N} \end{array} \right] +
$$

$$
\left[ \begin{array}{c c c c} Q _ {k} & 0 & \dots & 0 \\ 0 & 0 & \dots & 0 \\ \vdots & \vdots & & \vdots \\ 0 & 0 & \dots & 0 \end{array} \right]\tag{6.1.31}
$$

记式 $(6.1.31)$ 等号右侧第一项括号内的值为A，即

$$
\mathbf {A} = \left[ \begin{array}{c c c c c c} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} & \mathbf {I} & \mathbf {0} & \mathbf {0} & \dots & \mathbf {0} \\ \mathbf {0} & \mathbf {0} & \mathbf {I} & \mathbf {0} & \dots & \mathbf {0} \\ \mathbf {0} & \mathbf {0} & \mathbf {0} & \mathbf {I} & \dots & \mathbf {0} \\ \vdots & \vdots & \vdots & \vdots & & \vdots \\ \mathbf {0} & \mathbf {0} & \mathbf {0} & \mathbf {0} & \dots & \mathbf {I} \\ \mathbf {0} & \mathbf {0} & \mathbf {0} & \mathbf {0} & \dots & \mathbf {0} \end{array} \right] -
$$

$$
\left[ \begin{array}{c c c c c} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {M} \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, 0} & \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} & \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {M} \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, 0} & \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {M} \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, 1} & \dots & \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {M} \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, N} \\ \boldsymbol {0} & & \boldsymbol {0} & \boldsymbol {0} & \dots & \boldsymbol {0} \\ \vdots & & \vdots & \vdots & & \vdots \\ \boldsymbol {0} & & \boldsymbol {0} & \boldsymbol {0} & \dots & \boldsymbol {0} \end{array} \right] =
$$

$$
\left[ \begin{array}{c c c c c} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} - \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {M} \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, 0} & \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} & \boldsymbol {I} - \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {M} \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, 0} & - \boldsymbol {H} _ {k} ^ {\mathrm{T}} \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, 1} & \dots & - \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {M} \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, 1} \\ \boldsymbol {0} & & \boldsymbol {I} & \dots & \boldsymbol {0} \\ \boldsymbol {0} & & \boldsymbol {0} & \dots & \boldsymbol {0} \\ \vdots & & \vdots & & \vdots \\ \boldsymbol {0} & & \boldsymbol {0} & \dots & \boldsymbol {I} \\ \boldsymbol {0} & & \boldsymbol {0} & \dots & \boldsymbol {0} \end{array} \right]
$$

列写出式(6.1.31)第一列及主对角元(矩阵)

$$
\overline {{{P}}} _ {k + 1 / k} ^ {0, 0} = \boldsymbol {\Phi} _ {k + 1, k} \overline {{{P}}} _ {k / k - 1} ^ {0, 0} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} - \boldsymbol {\Phi} _ {k + 1, k} \overline {{{P}}} _ {k / k - 1} ^ {0, 0} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {M} \overline {{{P}}} _ {k / k - 1} ^ {0, 0} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + Q _ {k}
$$

由式 $(6.1.30)$ 和式 $(6.1.24)$ ，

$$
\boldsymbol {M} \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, 0} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} = (\boldsymbol {H} _ {k} \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, 0} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}) ^ {- 1} \boldsymbol {H} _ {k} \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, 0} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} = (\overline {{\boldsymbol {K}}} _ {k} ^ {(0)}) ^ {\mathrm{T}}
$$

所以

$$
\overline {{{\boldsymbol {P}}}} _ {k + 1 / k} ^ {0, 0} = \boldsymbol {\Phi} _ {k + 1, k} \overline {{{\boldsymbol {P}}}} _ {k / k - 1} ^ {0, 0} (\boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} - \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\overline {{{\boldsymbol {K}}}} _ {k} ^ {(0)}) ^ {\mathrm{T}}) + \boldsymbol {Q} _ {k} = \boldsymbol {\Phi} _ {k + 1, k} \overline {{{\boldsymbol {P}}}} _ {k / k - 1} ^ {(0, 0)} (\boldsymbol {\Phi} _ {k + 1, k} - \overline {{{\boldsymbol {K}}}} _ {k} ^ {(0)} \boldsymbol {H}) ^ {\mathrm{T}} + \boldsymbol {Q} _ {k}\tag{6.1.32}
$$

$$
\begin{array}{l} \overline {{\boldsymbol {P}}} _ {k + 1 / k} ^ {0, 1} = \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, 0} (\boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} - \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {M} \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, 0} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}}) = \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, 0} (\boldsymbol {\Phi} _ {k + 1, k} - \overline {{\boldsymbol {K}}} _ {k} ^ {(0)} \boldsymbol {H} _ {k}) ^ {\mathrm{T}} \\ \overline {{\boldsymbol {P}}} _ {k + 1 / k} ^ {0, 2} = \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, 1} (\boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} - \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {M} \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, 0} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}}) = \overline {{\boldsymbol {p}}} _ {k / k - 1} ^ {0, 1} (\boldsymbol {\Phi} _ {k + 1, k} - \overline {{\boldsymbol {K}}} _ {k} ^ {(0)} \boldsymbol {H} _ {k}) ^ {\mathrm{T}} \end{array}
$$

$$
\begin{array}{r l} & {\overline {{{P}}} _ {k + 1 / k} ^ {0, N + 1} = \overline {{{P}}} _ {k / k - 1} ^ {0, N} (\boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} - \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {M} \overline {{{P}}} _ {k / k - 1} ^ {0, 0} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}}) = \overline {{{P}}} _ {k / k - 1} ^ {0, N} (\boldsymbol {\Phi} _ {k + 1, k} - \overline {{{K}}} _ {k} ^ {(0)} \boldsymbol {H} _ {k}) ^ {\mathrm{T}}} \\ & {\overline {{{P}}} _ {k + 1 / k} ^ {1, 1} = \overline {{{P}}} _ {k / k - 1} ^ {0, 0} (\boldsymbol {I} - \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {M} \overline {{{P}}} _ {k / k - 1} ^ {0, 0}) = \overline {{{P}}} _ {k / k - 1} ^ {0, 0} - \overline {{{P}}} _ {k / k - 1} ^ {0, 0} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {H} _ {k} \overline {{{P}}} _ {k / k - 1} ^ {0, 0} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}) ^ {- 1} \boldsymbol {H} _ {k} \overline {{{P}}} _ {k / k - 1} ^ {0, 0} =} \\ & {\overline {{{P}}} _ {k / k - 1} ^ {0, 0} - \overline {{{P}}} _ {k / k - 1} ^ {0, 0} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {K} _ {k} ^ {(1)}) ^ {\mathrm{T}}} \\ & {\overline {{{P}}} _ {k + 1 / k} ^ {2, 2} = - \overline {{{P}}} _ {k / k - 1} ^ {0, 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \overline {{{P}}} _ {k / k - 1} ^ {0, 1} + \overline {{{P}}} _ {k / k - 1} ^ {1, 1} = \overline {{{P}}} _ {k / k - 1} ^ {1, 1} - \overline {{{P}}} _ {k / k - 1} ^ {0, 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {H} _ {k} \overline {{{P}}} _ {k / k - 1} ^ {0, 0} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}) ^ {- 1} \boldsymbol {H} _ {k} \overline {{{P}}} _ {k / (k - 1)} ^ {0, 1} =} \\ & {\overline {{{P}}} _ {k / k - 1} ^ {1, 1} - \overline {{{P}}} _ {k / k - 1} ^ {0, 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {K} _ {k} ^ {(2)}) ^ {\mathrm{T}}} \end{array}
$$

$$
\begin{array}{r l} \overline {{\boldsymbol {P}}} _ {k + 1 / k} ^ {N + 1, N + 1} & = - \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, N} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {M} \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, N} + \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {N, N} = \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {N, N} - \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, N} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {H} _ {k} \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, 0} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}) ^ {- 1} \boldsymbol {H} _ {k} \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, N} = \\ & \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {N, N} - \overline {{\boldsymbol {P}}} _ {k / k - 1} ^ {0, N} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {K} _ {k} ^ {(N + 1)}) ^ {\mathrm{T}} \end{array}
$$

根据上述诸式,可归纳出通项公式

$$
\overline {{{\boldsymbol {P}}}} _ {k + 1 / k} ^ {0, i} = \overline {{{\boldsymbol {P}}}} _ {k / k - 1} ^ {0, i - 1} \left(\boldsymbol {\Phi} _ {k + 1, k} - \overline {{{\boldsymbol {K}}}} _ {k} ^ {(0)} \boldsymbol {H} _ {k}\right) ^ {\mathrm{T}}\tag{6.1.33}
$$

$$
\overline {{{\boldsymbol {P}}}} _ {k + 1 / k} ^ {i, i} = \overline {{{\boldsymbol {P}}}} _ {k / k - 1} ^ {i - 1, i - 1} - \overline {{{\boldsymbol {P}}}} _ {k / k - 1} ^ {0, i - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\overline {{{\boldsymbol {K}}}} _ {k} ^ {(i)}) ^ {\mathrm{T}}\tag{6.1.34}
$$

$$
i = 1, 2, \dots , N + 1
$$

由于

$$
\hat {\boldsymbol {X}} _ {k + 1 / k} ^ {(0)} = \hat {\boldsymbol {X}} _ {k + 1 / k}, \quad \hat {\boldsymbol {X}} _ {k + 1 / k} ^ {(1)} = \hat {\boldsymbol {X}} _ {k}, \quad \hat {\boldsymbol {X}} _ {k + 1 / k} ^ {(2)} = \hat {\boldsymbol {X}} _ {k - 1 / k}, \quad \dots , \quad \hat {\boldsymbol {X}} _ {k + 1 / k} ^ {(N + 1)} = \hat {\boldsymbol {X}} _ {k - N / k}
$$

$$
\hat {\boldsymbol {X}} _ {k / k - 1} ^ {(0)} = \hat {\boldsymbol {X}} _ {k / k - 1}, \quad \hat {\boldsymbol {X}} _ {k / k - 1} ^ {(1)} = \hat {\boldsymbol {X}} _ {k - 1}, \quad \dots , \quad \hat {\boldsymbol {X}} _ {k / k - 1} ^ {(N)} = \hat {\boldsymbol {X}} _ {k - N / k - 1}, \quad \hat {\boldsymbol {X}} _ {k / k - 1} ^ {(N + 1)} = \hat {\boldsymbol {X}} _ {k - (N + 1) / k}
$$

式中

$$
\mathbf {X} _ {k + 1 / k} ^ {(i)} = \mathbf {E} ^ {*} \left[ \mathbf {X} _ {k + 1} ^ {(i)} / \mathbf {Z} _ {1} \mathbf {Z} _ {2} \dots \mathbf {Z} _ {k} \right], \quad \mathbf {X} _ {k / k - 1} ^ {(i)} = \mathbf {E} ^ {*} \left[ \mathbf {X} _ {k} ^ {(i)} / \mathbf {Z} _ {1} \mathbf {Z} _ {2} \dots \mathbf {Z} _ {k - 1} \right]
$$

所以式 $(6.1.22a)$ 可写成

$$
\left[ \begin{array}{c} \hat {\boldsymbol {X}} _ {k + 1 / k} \\ \hat {\boldsymbol {X}} _ {k} \\ \hat {\boldsymbol {X}} _ {k - 1 / k} \\ \vdots \\ \hat {\boldsymbol {X}} _ {k - N / k} \end{array} \right] = \left[ \begin{array}{c c c c c c} \boldsymbol {\Phi} _ {k + 1, k} & \mathbf {0} & \mathbf {0} & \dots & \mathbf {0} & \mathbf {0} \\ \boldsymbol {I} & \mathbf {0} & \mathbf {0} & \dots & \mathbf {0} & \mathbf {0} \\ \mathbf {0} & \boldsymbol {I} & \mathbf {0} & \dots & \mathbf {0} & \mathbf {0} \\ \vdots & \vdots & \vdots & & \vdots & \vdots \\ \mathbf {0} & \mathbf {0} & \mathbf {0} & \dots & \boldsymbol {I} & \mathbf {0} \end{array} \right] \left[ \begin{array}{c} \hat {\boldsymbol {X}} _ {k / k - 1} \\ \hat {\boldsymbol {X}} _ {k - 1} \\ \hat {\boldsymbol {X}} _ {k - 2 / k - 1} \\ \vdots \\ \boldsymbol {X} _ {k - N / k - 1} \\ \boldsymbol {X} _ {k - (N + 1) / k - 1} \end{array} \right] + \left[ \begin{array}{c} \overline {{\boldsymbol {K}}} _ {k} ^ {(0)} \\ \overline {{\boldsymbol {K}}} _ {k} ^ {(1)} \\ \overline {{\boldsymbol {K}}} _ {k} ^ {(2)} \\ \vdots \\ \overline {{\boldsymbol {K}}} _ {k} ^ {(N + 1)} \end{array} \right] (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \hat {\boldsymbol {X}} _ {k / k - 1})
$$

将上式写成分矢量形式，并根据式 $（6.1.26）$ 和式 $（6.1.27）$ ，得

$$
\begin{array}{r l} \hat {\mathbf {X}} _ {k + 1 / k} & = \boldsymbol {\Phi} _ {k + 1, k} \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {K} _ {k} ^ {*} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1}) \\ \hat {\mathbf {X}} _ {k} & = \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {K} _ {k} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1}) \\ \hat {\mathbf {X}} _ {k - 1 / k} & = \hat {\mathbf {X}} _ {k - 1} + \overline {{\mathbf {K}}} _ {k} ^ {(2)} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1}) \\ \hat {\mathbf {X}} _ {k - 2 / k} & = \hat {\mathbf {X}} _ {k - 2 / k - 1} + \overline {{\mathbf {K}}} _ {k} ^ {(3)} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1}) \\ & \dots \\ \hat {\mathbf {X}} _ {k - N / k} & = \hat {\mathbf {X}} _ {k - N / k - 1} + \overline {{\mathbf {K}}} _ {k} ^ {(N + 1)} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1}) \end{array}
$$

根据上述诸式,可归纳出通项公式

$$
\hat {\boldsymbol {X}} _ {k - i / k} = \hat {\boldsymbol {X}} _ {k - i / k - 1} + \overline {{{\boldsymbol {K}}}} _ {k} ^ {(i + 1)} \left(\mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\boldsymbol {X}} _ {k / k - 1}\right) \quad i = 1, 2, \dots , N\tag{6.1.35}
$$

根据式(6.1.35)、式(6.1.29)、式(6.1.33)、式(6.1.34)、式(6.1.23)和式(6.1.26)，可整理出固定滞后平滑算法如下：

$$
\begin{array}{r l} \hat {\mathbf {X}} _ {k - i / k} & = \hat {\mathbf {X}} _ {k - i / k - 1} + \overline {{\mathbf {K}}} _ {k} ^ {(i + 1)} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1}) \\ \overline {{\mathbf {K}}} _ {k} ^ {(i)} & = \overline {{\mathbf {P}}} _ {k / k - 1} ^ {0, i - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} (\mathbf {H} _ {k} \overline {{\mathbf {P}}} _ {k / k - 1} ^ {0, 0} \mathbf {H} _ {k} ^ {\mathrm{T}} + \mathbf {R} _ {k}) ^ {- 1} \\ \overline {{\mathbf {P}}} _ {k + 1 / k} ^ {0, i} & = \overline {{\mathbf {P}}} _ {k / k - 1} ^ {0, i - 1} (\boldsymbol {\Phi} _ {k + 1, k} - \overline {{\mathbf {K}}} _ {k} ^ {(0)} \mathbf {H} _ {k}) ^ {\mathrm{T}} \\ \overline {{\mathbf {P}}} _ {k + 1 / k} ^ {i, i} & = \overline {{\mathbf {P}}} _ {k / k - 1} ^ {i - 1, i - 1} - \overline {{\mathbf {P}}} _ {k / k - 1} ^ {0, i - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} (\overline {{\mathbf {K}}} _ {k} ^ {(i)}) ^ {\mathrm{T}} \quad i = 1, 2, \dots , N \end{array}
$$

式中，

$$
\begin{array}{r l} & {\hat {X} _ {k - 1 / k - 1} = \hat {X} _ {k - 1}} \\ & {\overline {{{P}}} _ {k / k - 1} ^ {0, 0} = P _ {k / k - 1}} \\ & {\overline {{{K}}} _ {k} ^ {(0)} = K _ {k} ^ {*} = \Phi_ {k + 1, k} P _ {k / k - 1} H _ {k} ^ {\mathrm{T}} (H _ {k} P _ {k / k - 1} H _ {k} ^ {\mathrm{T}} + R _ {k}) ^ {- 1}.} \end{array}
$$

执行固定滞后平滑时，必须同时计算出 $K_{k}^{*}$ ， $P_{k/k-1}$ 和 $\hat{X}_{k/k-1}$ ，这些量按卡尔曼滤波一步预测基本方程计算。

## 6.1.4 固定区间平滑

在实际工作中,量测值都是在有限的时间间隔内得到的。利用这个时间间隔内所有的量测值来估计系统在这个时间内整个过程的状态变量,当然是一种合理和有效的平滑方法,这就是固定区间平滑。如果固定的时间间隔中一共有 N 个时刻,并以 m 表示这个时间间隔中的任一时刻,即 $0 \leqslant m \leqslant N$ ,则固定区间平滑估计就是 $\hat{X}_{m/N}, m = 0, 1, 2, \cdots, N - 1$ 。

## 1. 前向-后向平滑算法[119]

设固定区间为 $\left[t_{1},t_{N}\right]$ ，在 $t_{1},t_{2},\cdots,t_{N}$ 时刻相应的量测为 $Z_{1},Z_{2},\cdots,Z_{N}$ 。 $t_{m}$ 为该区间中的某一时间点。则

$$
\hat {\mathbf {X}} _ {m / N} = E ^ {*} \left[ \mathbf {X} _ {m} / \mathbf {Z} _ {1}, \mathbf {Z} _ {2} \dots , \mathbf {Z} _ {N} \right] = \mathbf {K} _ {f} \hat {\mathbf {X}} _ {f, m} + \mathbf {K} _ {b} \hat {\mathbf {X}} _ {b, m / m + 1}
$$

式中 $\hat{X}_{f,m}=E^{*}\left[X_{m}/Z_{1},Z_{2},\cdots,Z_{m}\right]$ ，为前向滤波结果； $\hat{X}_{b,m/m+1}=E\left[X_{m}/Z_{N},Z_{N-1},\cdots,Z_{m+1}\right]$ ，为后向滤波结果。

设 $\hat{\pmb{X}}_{f,m}$ 的均方误差阵为 $P_{f,m},\hat{X}_{b,m / m + 1}$ 的均方误差阵为 $P_{b,m / m + 1}$ ，即

$$
\begin{array}{r l} & {\hat {\mathbf {X}} _ {f, m} = \mathbf {X} _ {m} + \boldsymbol {\Delta} _ {f, m}} \\ & {\hat {\mathbf {X}} _ {b, m / m + 1} = \mathbf {X} _ {m} + \boldsymbol {\Delta} _ {b, m / m + 1}} \\ & {\left[ \begin{array}{l} {\hat {\mathbf {X}} _ {f, m}} \\ {\hat {\mathbf {X}} _ {b, m / m + 1}} \end{array} \right] = \left[ \begin{array}{l} {\boldsymbol {I}} \\ {\boldsymbol {I}} \end{array} \right] \mathbf {X} _ {m} + \left[ \begin{array}{l} {\boldsymbol {\Delta} _ {f, m}} \\ {\boldsymbol {\Delta} _ {b, m / m + 1}} \end{array} \right]} \end{array}
$$

其中 $\Delta_{f,m}$ 为前向滤波估计误差， $\Delta_{b,m/m+1}$ 为后向滤波估计误差。

记 $H=\begin{bmatrix}I\\ I\end{bmatrix},R_{m}=\begin{bmatrix}P_{f,m}&0\\ 0&P_{b,m/m+1}\end{bmatrix},Z_{m}=\begin{bmatrix}\hat{X}_{f,m}\\ \hat{X}_{b,m/m+1}\end{bmatrix}$ ，其中 I 为单位阵。根据马尔柯夫估计算法，即式(2.1.12)和式(2.1.13)：

$$
\begin{array}{l} \mathbf {s}, m = (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {R} _ {m} ^ {- 1} \boldsymbol {H}) ^ {- 1} \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {R} _ {m} ^ {- 1} \boldsymbol {Z} _ {m} = \\ \left(\left[ \begin{array}{c c} \boldsymbol {I} & \boldsymbol {I} \end{array} \right] \left[ \begin{array}{c c} \boldsymbol {P} _ {f, m} ^ {- 1} & \boldsymbol {0} \\ \boldsymbol {0} & \boldsymbol {P} _ {b, m / m + 1} ^ {- 1} \end{array} \right] \left[ \begin{array}{c} \boldsymbol {I} \\ \boldsymbol {I} \end{array} \right]\right) ^ {- 1} [ \boldsymbol {I} & \boldsymbol {I} ] \left[ \begin{array}{c c} \boldsymbol {P} _ {f, m} ^ {- 1} & \boldsymbol {0} \\ \boldsymbol {0} & \boldsymbol {P} _ {b, m / m + 1} ^ {- 1} \end{array} \right] \left[ \begin{array}{c} \hat {\boldsymbol {X}} _ {f, m} \\ \hat {\boldsymbol {X}} _ {b, m / m + 1} \end{array} \right] = \\ (\boldsymbol {P} _ {f, m} ^ {- 1} + \boldsymbol {P} _ {b, m / m + 1} ^ {- 1}) ^ {- 1} (\boldsymbol {P} _ {f, m} ^ {- 1} \hat {\boldsymbol {X}} _ {f, m} + \boldsymbol {P} _ {b, m / m + 1} ^ {- 1} \hat {\boldsymbol {X}} _ {b, m / m + 1}) = \\ (\boldsymbol {P} _ {f, m} ^ {- 1} + \boldsymbol {P} _ {b, m / m + 1} ^ {- 1}) ^ {- 1} \boldsymbol {P} _ {f, m} ^ {- 1} \hat {\boldsymbol {X}} _ {f, m} + (\boldsymbol {P} _ {f, m} ^ {- 1} + \boldsymbol {P} _ {b, m / m + 1} ^ {- 1}) ^ {- 1} \boldsymbol {P} _ {b, m / m + 1} ^ {- 1} \hat {\boldsymbol {X}} _ {b, m / m + 1} = \\ [ \boldsymbol {P} _ {f, m} ^ {- 1} (\boldsymbol {P} _ {b, m / m + 1} + \boldsymbol {P} _ {f, m}) \boldsymbol {P} _ {b, m / m + 1} ^ {- 1} ] ^ {- 1} \boldsymbol {P} _ {f, m} ^ {- 1} \hat {\boldsymbol {X}} _ {f, m} + \\ [ \boldsymbol {P} _ {b, m / m + 1} ^ {- 1} (\boldsymbol {P} _ {f, m} + \boldsymbol {P} _ {b, m / m + 1}) \boldsymbol {P} _ {f, m} ^ {- 1} ] ^ {- 1} \boldsymbol {P} _ {b, m / m + 1} ^ {- 1} \hat {\boldsymbol {X}} _ {b, m / m + 1} = \\ [ (\boldsymbol {P} _ {b, m / m + 1} + \boldsymbol {P} _ {f, m}) \boldsymbol {P} _ {b, m / m + 1} ^ {- 1} ] ^ {- 1} (\boldsymbol {P} _ {f, m} ^ {- 1}) ^ {- 1} \boldsymbol {P} _ {f, m} ^ {- 1} \hat {\boldsymbol {X}} _ {f, m} + \\ [ (\boldsymbol {P} _ {f, m} + \boldsymbol {P} _ {b, m / m + 1}) \boldsymbol {P} _ {f, m} ^ {- 1} ] ^ {- 1} (\boldsymbol {P} _ {b, m / m + 1} ^ {- 1}) ^ {- 1} \boldsymbol {P} _ {b, m / m + 1} ^ {- 1} \hat {\boldsymbol {X}} _ {b, m / m + 1} = \\ \boldsymbol {P} _ {b, m / m + 1} (\boldsymbol {P} _ {b, m / m + 1} + \boldsymbol {P} _ {f, m}) ^ {- 1} \hat {\boldsymbol {X}} _ {f, m} + \boldsymbol {P} _ {f, m} (\boldsymbol {P} _ {f, m} + \boldsymbol {P} _ {b, m / m + 1}) ^ {- 1} \hat {\boldsymbol {X}} _ {b, m / m + 1}. \end{array}\tag{6.1.36}
$$

估计的均方误差阵为

$$
\begin{array}{r l} & {\pmb {P} _ {s, m} = (\pmb {H} ^ {\mathrm{T}} \pmb {R} _ {m} ^ {- 1} \pmb {H}) ^ {- 1} = \left(\left[ \begin{array}{l l} {\pmb {I}} & {\pmb {I}} \end{array} \right] \left[ \begin{array}{c c} {\pmb {P} _ {f, m} ^ {- 1}} & {\pmb {0}} \\ {\pmb {0}} & {\pmb {P} _ {b, m / m + 1} ^ {- 1}} \end{array} \right] \left[ \begin{array}{l} {\pmb {I}} \\ {\pmb {I}} \end{array} \right]\right) ^ {- 1} =} \\ & {\qquad (\pmb {P} _ {f, m} ^ {- 1} + \pmb {P} _ {b, m / m + 1} ^ {- 1}) ^ {- 1} = (\pmb {I} _ {f, m} + \pmb {I} _ {b, m / m + 1}) ^ {- 1}} \end{array}\tag{6.1.37}
$$

其中 $I_{f,m}=P_{f,m}^{-1},I_{b,m/m+1}=P_{b,m/m+1}^{-1}$ 为信息矩阵。为书写方便，此处约定：凡用 I 表示的矩阵，带下标的为信息矩阵，不带下标的为单位矩阵。

设系统方程和量测方程为

$$
\begin{array}{r l} & {\mathbf {X} _ {k} = \pmb {\Phi} _ {k, k - 1} \mathbf {X} _ {k - 1} + \mathbf {W} _ {k - 1}} \\ & {\qquad \mathbf {Z} _ {k} = \mathbf {H} _ {k} \mathbf {X} _ {k} + \mathbf {V} _ {k}} \\ & {\mathbf {W} _ {k} \sim \mathbf {N} (\mathbf {0}, Q _ {k}), \quad \mathbf {V} _ {k} \sim \mathbf {N} (\mathbf {0}, R _ {k})} \end{array}
$$

则 $X_{m}$ 的前向滤波值由卡尔曼滤波基本方程确定：

$$
\begin{array}{l} \hat {\mathbf {X}} _ {f, k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \hat {\mathbf {X}} _ {f, k - 1} \\ \hat {\mathbf {X}} _ {f, k} = \hat {\mathbf {X}} _ {f, k / k - 1} + \mathbf {K} _ {f, k} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {f, k / k - 1}) \\ \mathbf {K} _ {f, k} = \mathbf {P} _ {f, k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} (\mathbf {H} _ {k} \mathbf {P} _ {f, k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} + \mathbf {R} _ {k}) ^ {- 1} \\ \mathbf {P} _ {f, k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \mathbf {P} _ {f, k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} + Q _ {k - 1} \\ \mathbf {P} _ {f, k} = (\mathbf {I} - \mathbf {K} _ {f, k} \mathbf {H} _ {k}) \mathbf {P} _ {f, k / k - 1} \\ k = 1, 2, \dots m \end{array}
$$

后向滤波从 k=N 开始。选取滤波初值 $\hat{X}_{b,N}$ 是十分盲目的，对应的均方误差阵 $P_{b,N}=\infty$ 。所以必须采用信息滤波方程，根据 3.4 节关于信息滤波的介绍：

$$
\boldsymbol {I} _ {b, k} = \boldsymbol {I} _ {b, k / k + 1} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}\tag{6.1.38a}
$$

$$
\mathbf {K} _ {b, k} = \mathbf {I} _ {b, k} ^ {- 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1}\tag{6.1.38b}
$$

$$
\hat {\mathbf {X}} _ {b, k} = \hat {\mathbf {X}} _ {b, k / k + 1} + \mathbf {K} _ {b, k} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {b, k / k + 1})\tag{6.1.38c}
$$

$$
\boldsymbol {I} _ {b, k - 1 / k} = \left[ \boldsymbol {\Phi} _ {k, k - 1} ^ {- 1} \boldsymbol {I} _ {b, k} ^ {- 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {- \mathrm{T}} + \boldsymbol {\Phi} _ {k, k - 1} ^ {- 1} Q _ {k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {- \mathrm{T}} \right] ^ {- 1} = \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \left[ \boldsymbol {I} _ {b, k} ^ {- 1} + Q _ {k - 1} \right] ^ {- 1} \boldsymbol {\Phi} _ {k, k - 1} =
$$

$$
\boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \left[ \boldsymbol {Q} _ {k - 1} ^ {- 1} - \boldsymbol {Q} _ {k - 1} ^ {- 1} \left(\boldsymbol {I} _ {b, k} + \boldsymbol {Q} _ {k - 1} ^ {- 1}\right) ^ {- 1} \boldsymbol {Q} _ {k - 1} ^ {- 1} \right] \boldsymbol {\Phi} _ {k, k - 1}\tag{6.1.38d}
$$

$$
\boldsymbol {X} _ {b, k - 1 / k} = \boldsymbol {\Phi} _ {k, k - 1} ^ {- 1} \hat {\boldsymbol {X}} _ {b, k}\tag{6.1.38e}
$$

式中，信息矩阵 $I_{b,k-1/k}=P_{b,k-1/k}^{-1};I_{b,k}=P_{b,k}^{-1};I_{b,N/N+1}=0,k=N,N-1,N-2,\cdots,m+1$ 。

以 $k=m+1$ 为后向滤波终了时刻的原因是: m 时刻的量测只能使用一次, $Z_{m}$ 已在前向滤波中使用过, 后向滤波中不可再用。

根据(6.1.36)式，

$$
\hat {\boldsymbol {X}} _ {s, m} = \boldsymbol {P} _ {b, m / m + 1} \left(\boldsymbol {P} _ {f, m} + \boldsymbol {P} _ {b, m / m + 1}\right) ^ {- 1} \hat {\boldsymbol {X}} _ {f, m} + \boldsymbol {P} _ {f, m} \left(\boldsymbol {P} _ {f, m} + \boldsymbol {P} _ {b, m / m + 1}\right) ^ {- 1} \hat {\boldsymbol {X}} _ {b, m / m + 1}
$$

应用求逆反演公式求解上式右侧第二项中的逆：

$$
\left(\boldsymbol {P} _ {b, m / m + 1} + \boldsymbol {P} _ {f, m}\right) ^ {- 1} = \left[ \boldsymbol {P} _ {b, m / m + 1} - (- \boldsymbol {I}) \cdot \boldsymbol {I} _ {f, m} ^ {- 1} \cdot \boldsymbol {I} \right] ^ {- 1}
$$

对应于

$$
\begin{array}{r l} & (A _ {1 1} - A _ {1 2} A _ {2 2} ^ {- 1} A _ {2 1}) ^ {- 1} = A _ {1 1} ^ {- 1} + A _ {1 1} ^ {- 1} A _ {1 2} (A _ {2 2} - A _ {2 1} A _ {1 1} ^ {- 1} A _ {1 2}) ^ {- 1} A _ {2 1} A _ {1 1} ^ {- 1} \\ & \quad A _ {1 1} = P _ {b, m / m + 1}, \quad A _ {1 2} = - I, \quad A _ {2 1} = I, \quad A _ {2 2} ^ {- 1} = I _ {f, m} ^ {- 1} = P _ {f, m} \\ & (P _ {b, m / m + 1} + P _ {f, m}) ^ {- 1} = I _ {b, m / m + 1} + I _ {b, m / m + 1} (- I) [ I _ {f, m} - I \cdot I _ {b, m / m + 1} (- I) ] ^ {- 1} I \cdot I _ {b, m / m + 1} = \\ & \quad I _ {b, m / m + 1} - I _ {b, m / m + 1} (I _ {f, m} + I _ {b, m / m + 1}) ^ {- 1} I _ {b, m / m + 1} \end{array}
$$

所以

$$
\begin{array}{r l} \hat {\mathbf {X}} _ {s, m} & = [ (\mathbf {P} _ {b, m / m + 1} + \mathbf {P} _ {f, m}) - \mathbf {P} _ {f, m} ] (\mathbf {P} _ {f, m} + \mathbf {P} _ {b, m / m + 1}) ^ {- 1} \hat {\mathbf {X}} _ {f, m} + \\ & \quad \mathbf {P} _ {f, m} [ \mathbf {I} _ {b, m / m + 1} - \mathbf {I} _ {b, m / m + 1} (\mathbf {I} _ {f, m} + \mathbf {I} _ {b, m / m + 1}) ^ {- 1} \mathbf {I} _ {b, m / m + 1} ] \hat {\mathbf {X}} _ {b, m / m + 1} = \\ & \quad [ \mathbf {I} - \mathbf {P} _ {f, m} (\mathbf {P} _ {f, m} + \mathbf {P} _ {b, m / m + 1}) ^ {- 1} ] \hat {\mathbf {X}} _ {f, m} + \\ & \quad \mathbf {P} _ {f, m} [ \mathbf {I} - \mathbf {I} _ {b, m / m + 1} (\mathbf {I} _ {f, m} + \mathbf {I} _ {b, m / m + 1}) ^ {- 1} ] \mathbf {I} _ {b, m / m + 1} \hat {\mathbf {X}} _ {b, m / m + 1} = \\ & \quad [ \mathbf {I} - \mathbf {P} _ {f, m} \mathbf {I} _ {b, m / m + 1} (\mathbf {I} + \mathbf {P} _ {f, m} \mathbf {I} _ {b, m / m + 1}) ^ {- 1} ] \hat {\mathbf {X}} _ {f, m} + \\ & \quad \mathbf {P} _ {f, m} [ \mathbf {I} - \mathbf {I} _ {b, m / m + 1} (\mathbf {I} + \mathbf {P} _ {f, m} \mathbf {I} _ {b, m / m + 1}) ^ {- 1} \mathbf {P} _ {f, m} ] \mathbf {I} _ {b, m / m + 1} \hat {\mathbf {X}} _ {b, m / m + 1} = \\ & \quad \mathbf {P} _ {f, m} [ \mathbf {I} - \mathbf {I} _ {b, m / m + 1} (\mathbf {I} + \mathbf {P} _ {f, m} \mathbf {I} _ {b, m / m + 1}) ^ {- 1} \mathbf {P} _ {f, m} ] \mathbf {J} _ {f, m} \hat {\mathbf {X}} _ {f, m} + \\ & \quad \mathbf {P} _ {f, m} [ \mathbf {I} - \mathbf {I} _ {b, m / m + 1} (\mathbf {I} + \mathbf {P} _ {f, m} \mathbf {I} _ {b, m / m + 1}) ^ {- 1} \mathcal P _ {f, m} ] \mathbf {J} _ {b, m / m + 1} \hat {\mathbf {X}} _ {b, m / m + 1}. \end{array}\tag{6.1.39}
$$

式 $(6.1.39)$ 中，

$$
\begin{array}{r l} & {\pmb {P} _ {f, m} [ \pmb {I} - \pmb {I} _ {b, m / m + 1} (\pmb {I} + \pmb {P} _ {f, m} \pmb {I} _ {b, m / m + 1}) ^ {- 1} \pmb {P} _ {f, m} ] =} \\ & {\qquad \pmb {P} _ {f, m} - \pmb {P} _ {f, m} \pmb {I} _ {b, m / m + 1} (\pmb {I} + \pmb {P} _ {f, m} \pmb {I} _ {b, m / m + 1}) ^ {- 1} \pmb {P} _ {f, m} =} \\ & {\qquad \pmb {P} _ {f, m} - \pmb {P} _ {f, m} (\pmb {I} _ {f, m} \pmb {P} _ {b, m / m + 1} + \pmb {I}) ^ {- 1} =} \\ & {\qquad [ \pmb {P} _ {f, m} (\pmb {I} _ {f, m} \pmb {P} _ {b, m / m + 1} + \pmb {I}) - \pmb {P} _ {f, m} ] (\pmb {I} _ {f, m} \pmb {P} _ {b, m / m + 1} + \pmb {I}) ^ {- 1} =} \\ & {\qquad \pmb {P} _ {b, m / m + 1} (\pmb {I} _ {f, m} \pmb {P} _ {b, m / m + 1} + \pmb {I}) ^ {- 1} = (\pmb {I} _ {f, m} + \pmb {I} _ {b, m / m + 1}) ^ {- 1} = \pmb {P} _ {s, m}} \end{array}\tag{6.1.40}
$$

式 $(6.1.40)$ 推导中，用到了关系式 $(6.1.37)$ 。

所以式 $(6.1.39)$ 可写成

$$
\hat {\boldsymbol {X}} _ {s, m} = \boldsymbol {P} _ {s, m} \left(\boldsymbol {I} _ {f, m} \hat {\boldsymbol {X}} _ {f, m} + \boldsymbol {I} _ {b, m / m + 1} \hat {\boldsymbol {X}} _ {b, m / m + 1}\right)\tag{6.1.41}
$$

式中， $P_{s,m}$ 为平滑的均方误差阵。

若按照式(6.1.38)作后向滤波，则滤波初值 $\hat{X}_{b,N}$ 的确定十分困难，为避免此问题，对式

(6.1.38) 稍加修改。

取

$$
\boldsymbol {S} _ {b, k} = \boldsymbol {P} _ {b, k} ^ {- 1} \hat {\boldsymbol {X}} _ {b, k} = \boldsymbol {I} _ {b, k} \hat {\boldsymbol {X}} _ {b, k}\tag{6.1.42a}
$$

$$
\boldsymbol {S} _ {b, k - 1 / k} = \boldsymbol {P} _ {b, k - 1 / k} ^ {- 1} \hat {\boldsymbol {X}} _ {b, k - 1 / k} = \boldsymbol {I} _ {b, k - 1 / k} \hat {\boldsymbol {X}} _ {b, k - 1 / k}\tag{6.1.42b}
$$

在选初值时，由于 $I_{b,N} = 0$ ，所以 $S_{b,N} = 0$ ，与 $\hat{X}_{b,N}$ 取值无关。

根据式(6.1.38a)、式(6.1.38c)和式(6.1.38b)，

$$
\begin{array}{r l} \mathbf {S} _ {b, k} & = \mathbf {I} _ {b, k} \hat {\mathbf {X}} _ {b, k} = \mathbf {I} _ {b, k} \hat {\mathbf {X}} _ {b, k / k + 1} + \mathbf {I} _ {b, k} \mathbf {K} _ {b, k} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {b, k / k + 1}) = \\ & \quad \mathbf {I} _ {b, k / k + 1} \hat {\mathbf {X}} _ {b, k / k + 1} + \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} \mathbf {H} _ {k} \hat {\mathbf {X}} _ {b, k / k + 1} + \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {b, k / k + 1}) = \\ & \quad \mathbf {S} _ {b, k / k + 1} + \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} \mathbf {Z} _ {k} \end{array}
$$

所以式 $(6.1.38)$ 可写成

$$
\boldsymbol {I} _ {b, k} = \boldsymbol {I} _ {b, k / k + 1} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}\tag{6.1.43a}
$$

$$
\boldsymbol {S} _ {b, k} = \boldsymbol {S} _ {b, k / k + 1} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {Z} _ {k}\tag{6.1.43b}
$$

$$
\boldsymbol {I} _ {b, k - 1 / k} = \left[ \boldsymbol {\Phi} _ {k, k - 1} ^ {- 1} \boldsymbol {I} _ {b, k} ^ {- 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {- T} + \boldsymbol {\Phi} _ {k, k - 1} ^ {- 1} Q _ {k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {- T} \right] ^ {- 1} = \boldsymbol {\Phi} _ {k, k - 1} ^ {T} \left(\boldsymbol {I} _ {b, k} ^ {- 1} + Q _ {k - 1}\right) ^ {- 1} \boldsymbol {\Phi} _ {k, k - 1} =
$$

$$
\boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \left[ \boldsymbol {Q} _ {k - 1} ^ {- 1} - \boldsymbol {Q} _ {k - 1} ^ {- 1} \left(\boldsymbol {I} _ {b, k} + \boldsymbol {Q} _ {k - 1} ^ {- 1}\right) ^ {- 1} \boldsymbol {Q} _ {k - 1} ^ {- 1} \right] \boldsymbol {\Phi} _ {k, k - 1}\tag{6.1.43c}
$$

$$
\boldsymbol {S} _ {b, k - 1 / k} = \boldsymbol {I} _ {b, k - 1 / k} \boldsymbol {\Phi} _ {k, k - 1} ^ {- 1} \boldsymbol {I} _ {b, k} ^ {- 1} \boldsymbol {S} _ {b, k}\tag{6.1.43d}
$$

$$
k = N, N - 1, \dots , m + 1
$$

当 $k=m+1$ 时，即可求得 $S_{b,m/m+1}$ ，此即 $I_{b,m/m+1}\hat{X}_{b,m/m+1}$ ，代入式(6.1.41)即可确定出 m 时刻的平滑值 $\hat{X}_{s,m}$ 。

## 2. RTS 算法 $^{[118]}$

在前向-后向平滑算法中，前向滤波和后向滤波是同时进行的，而后向滤波有诸多求逆，且并不符合人们的习惯。1965年，H.Rauch，F.Tung和C.Striebel提出了RTS算法，该算法取消了后向滤波过程，而前向滤波从 $t_{1}$ 开始一直执行到 $t_{N}$ ，记录下每次滤波结果，顺序处理完量测量 $Z_{1},Z_{2},\cdots,Z_{N}$ 后，逆序执行RTS平滑算法，获得需要的平滑值。该算法的优点是避免了后向滤波，提高了计算效率。下面在前向-后向滤波的基础上推导出RTS算法。

(1) RTS 协方差阵更新。由式(6.1.37)，平滑误差均方误差阵为

$$
\boldsymbol {P} _ {s, m} = \left(\boldsymbol {P} _ {f, m} ^ {- 1} + \boldsymbol {P} _ {b, m / m + 1} ^ {- 1}\right) ^ {- 1} = \left[ \boldsymbol {P} _ {f, m} ^ {- 1} - (- \boldsymbol {I}) \boldsymbol {P} _ {b, m / m + 1} ^ {- 1} \boldsymbol {I} \right] ^ {- 1}
$$

对应于求逆反演公式

$$
\begin{array}{r l} & (A _ {1 1} - A _ {1 2} A _ {2 2} ^ {- 1} A _ {2 1}) ^ {- 1} = A _ {1 1} ^ {- 1} + A _ {1 1} ^ {- 1} A _ {1 2} (A _ {2 2} - A _ {2 1} A _ {1 1} ^ {- 1} A _ {1 2}) ^ {- 1} A _ {2 1} A _ {1 1} ^ {- 1} \\ & \quad A _ {1 1} = P _ {f, m} ^ {- 1}, \quad A _ {1 2} = - I, \quad A _ {2 1} = I, \quad A _ {2 2} = P _ {b, m / m + 1} \end{array}
$$

所以

$$
\begin{array}{r l} \boldsymbol {P} _ {s, m} & = \boldsymbol {P} _ {f, m} + \boldsymbol {P} _ {f, m} (- \boldsymbol {I}) [ \boldsymbol {P} _ {b, m / m + 1} - \boldsymbol {P} _ {f, m} (- \boldsymbol {I}) ] ^ {- 1} \boldsymbol {P} _ {f, m} = \\ & \quad \boldsymbol {P} _ {f, m} - \boldsymbol {P} _ {f, m} (\boldsymbol {P} _ {f, m} + \boldsymbol {P} _ {b, m / m + 1}) ^ {- 1} \boldsymbol {P} _ {f, m} \end{array}\tag{6.1.44}
$$

由式 $(6.1.38d)$ ，应用求逆反演公式，得

$$
\boldsymbol {P} _ {b, m / m + 1} = \boldsymbol {\Phi} _ {m + 1, m} ^ {- 1} (\boldsymbol {P} _ {b, m + 1} + Q _ {m}) \boldsymbol {\Phi} _ {m + 1, m} ^ {- T}
$$

所以

$$
\begin{array}{r l} & {(P _ {f, m} + P _ {b, m / m + 1}) ^ {- 1} = [ P _ {f, m} + \Phi_ {m + 1, m} ^ {- 1} (P _ {b, m + 1} + Q _ {m}) \Phi_ {m + 1, m} ^ {- T} ] ^ {- 1} =} \\ & {\qquad [ \Phi_ {m + 1, m} ^ {- 1} \Phi_ {m + 1, m} P _ {f, m} \Phi_ {m + 1, m} ^ {T} \Phi_ {m + 1, m} ^ {- T} + \Phi_ {m + 1, m} ^ {- 1} (P _ {b, m + 1} + Q _ {m}) \Phi_ {m + 1, m} ^ {- T} ] ^ {- 1} =} \\ & {\qquad [ \Phi_ {m + 1, m} ^ {- 1} (\Phi_ {m + 1, m} P _ {f, m} \Phi_ {m + 1, m} ^ {T} + P _ {b, m + 1} + Q _ {m}) \Phi_ {m + 1, m} ^ {- T} ] ^ {- 1} =} \\ & {\qquad \Phi_ {m + 1, m} ^ {T} (\Phi_ {m + 1, m} P _ {f, m} \Phi_ {m + 1, m} ^ {T} + P _ {b, m + 1} + Q _ {m}) ^ {- 1} \Phi_ {m + 1, m} =} \end{array}
$$

$$
\boldsymbol {\Phi} _ {m + 1, m} ^ {\mathrm{T}} \left(\boldsymbol {P} _ {f, m + 1 / m} + \boldsymbol {P} _ {b, m + 1}\right) ^ {- 1} \boldsymbol {\Phi} _ {m + 1, m}\tag{6.1.45}
$$

由式 $(2.2.4e^{\prime\prime})$ 和式 $(6.1.38a)$ ，得

$$
\boldsymbol {I} _ {f, m} = \boldsymbol {I} _ {f, m / m - 1} + \boldsymbol {H} _ {m} ^ {\mathrm{T}} \boldsymbol {R} _ {m} ^ {- 1} \boldsymbol {H} _ {m}\tag{6.1.46a}
$$

$$
\boldsymbol {I} _ {b, m} = \boldsymbol {I} _ {b, m / m + 1} + \boldsymbol {H} _ {m} ^ {\mathrm{T}} \boldsymbol {R} _ {m} ^ {- 1} \boldsymbol {H} _ {m}\tag{6.1.46b}
$$

由式 $(6.1.43a)$ 和式 $(6.1.46)$ ，得

$$
\mathbf {I} _ {b, m + 1} = \mathbf {I} _ {b, m + 1 / m + 2} + \mathbf {I} _ {f, m + 1} - \mathbf {I} _ {f, m + 1 / m}
$$

即

$$
\mathbf {I} _ {f, m + 1} + \mathbf {I} _ {b, m + 1 / m + 2} = \mathbf {I} _ {b, m + 1} + \mathbf {I} _ {f, m + 1 / m}\tag{6.1.47}
$$

将式 $(6.1.47)$ 代入式 $(6.1.37)$ ，得

$$
\boldsymbol {P} _ {s, m + 1} = (\boldsymbol {I} _ {f, m + 1} + \boldsymbol {I} _ {b, m + 1 / m + 2}) ^ {- 1} = (\boldsymbol {I} _ {b, m + 1} + \boldsymbol {I} _ {f, m + 1 / m}) ^ {- 1}\tag{6.1.48}
$$

$$
\boldsymbol {P} _ {s, m + 1} ^ {- 1} = \boldsymbol {I} _ {b, m + 1} + \boldsymbol {I} _ {f, m + 1 / m}
$$

$$
\boldsymbol {P} _ {b, m + 1} = \left(\boldsymbol {P} _ {s, m + 1} ^ {- 1} - \boldsymbol {I} _ {f, m + 1 / m}\right) ^ {- 1}\tag{6.1.49}
$$

将式 $(6.1.49)$ 代入式 $(6.1.45)$ ，得

$$
\begin{array}{r l} & (\boldsymbol {P} _ {f, m} + \boldsymbol {P} _ {b, m / m + 1}) ^ {- 1} = \boldsymbol {\Phi} _ {m + 1, m} ^ {\mathrm{T}} [ \boldsymbol {P} _ {f, m + 1 / m} + (\boldsymbol {P} _ {s, m + 1} ^ {- 1} - \boldsymbol {I} _ {f, m + 1 / m}) ^ {- 1} ] ^ {- 1} \boldsymbol {\Phi} _ {m + 1, m} = \\ & \quad \boldsymbol {\Phi} _ {m + 1, m} ^ {\mathrm{T}} [ \boldsymbol {I} _ {f, m + 1 / m} ^ {- 1} \boldsymbol {I} _ {f, m + 1 / m} \boldsymbol {I} _ {f, m + 1 / m} ^ {- 1} + \boldsymbol {I} _ {f, m + 1 / m} ^ {- 1} \boldsymbol {I} _ {f, m + 1 / m} (\boldsymbol {P} _ {s, m + 1} ^ {- 1} - \boldsymbol {I} _ {f, m + 1 / m}) ^ {- 1}. \\ & \quad \boldsymbol {I} _ {f, m + 1 / m} \boldsymbol {I} _ {f, m + 1 / m} ^ {- 1} ] ^ {- 1} \boldsymbol {\Phi} _ {m + 1, m} = \boldsymbol {\Phi} _ {m + 1, m} ^ {\mathrm{T}} \boldsymbol {I} _ {f, m + 1 / m} [ \boldsymbol {I} _ {f, m + 1 / m} + \boldsymbol {I} _ {f, m + 1 / m} (\boldsymbol {P} _ {s, m + 1} ^ {- 1} - \boldsymbol {I} _ {f, m + 1 / m}) ^ {- 1} ] ^ {- 1} \boldsymbol {\Phi} _ {m + 1, m}. \end{array}
$$

该式方括号内的项应用矩阵求逆反演公式

$$
\begin{array}{r l} & {\mathbf {A} _ {1 1} ^ {- 1} + \mathbf {A} _ {1 1} ^ {- 1} \mathbf {A} _ {1 2} (\mathbf {A} _ {2 2} - \mathbf {A} _ {2 1} \mathbf {A} _ {1 1} ^ {- 1} \mathbf {A} _ {1 2}) ^ {- 1} \mathbf {A} _ {2 1} \mathbf {A} _ {1 1} ^ {- 1} = (\mathbf {A} _ {1 1} - \mathbf {A} _ {1 2} \mathbf {A} _ {2 2} ^ {- 1} \mathbf {A} _ {2 1}) ^ {- 1}} \\ & {\qquad \mathbf {A} _ {1 1} ^ {- 1} = \mathbf {I} _ {f, m + 1 / m}, \quad \mathbf {A} _ {1 2} = \mathbf {I}, \quad \mathbf {A} _ {2 2} = \mathbf {P} _ {s, m + 1} ^ {- 1}, \quad \mathbf {A} _ {2 1} = \mathbf {I}} \\ & {(\mathbf {P} _ {f, m} + \mathbf {P} _ {b, m / m + 1}) ^ {- 1} = \boldsymbol {\Phi} _ {m + 1, m} ^ {\mathrm{T}} \mathbf {I} _ {f, m + 1 / m} (\mathbf {P} _ {f, m + 1 / m} - \mathbf {P} _ {s, m + 1}) \mathbf {I} _ {f, m + 1 / m} \boldsymbol {\Phi} _ {m + 1, m}} \end{array}\tag{6.1.50}
$$

将式 $(6.1.50)$ 代入式 $(6.1.44)$ ，得

$$
\boldsymbol {P} _ {s, m} = \boldsymbol {P} _ {f, m} - \boldsymbol {P} _ {f, m} \boldsymbol {\Phi} _ {m + 1, m} ^ {\mathrm{T}} \boldsymbol {I} _ {f, m + 1 / m} (\boldsymbol {P} _ {f, m + 1 / m} - \boldsymbol {P} _ {s, m + 1}) \boldsymbol {I} _ {f, m + 1 / m} \boldsymbol {\Phi} _ {m + 1, m} \boldsymbol {P} _ {f, m}
$$

令

$$
\boldsymbol {K} _ {s, m} = \boldsymbol {P} _ {f, m} \boldsymbol {\Phi} _ {m + 1, m} ^ {\mathrm{T}} \boldsymbol {I} _ {f, m + 1 / m}\tag{6.1.51}
$$

则

$$
\boldsymbol {P} _ {s, m} = \boldsymbol {P} _ {f, m} - \boldsymbol {K} _ {s, m} (\boldsymbol {P} _ {f, m + 1 / m} - \boldsymbol {P} _ {s, m + 1}) \boldsymbol {K} _ {s, m} ^ {\mathrm{T}}\tag{6.1.52}
$$

(2)RTS 平滑值更新。为便于推导平滑值的逆序递推公式,先证明 5 个辅助关系式。
关系式 1:

$$
\boldsymbol {\Phi} _ {k, k - 1} ^ {- 1} \boldsymbol {Q} _ {k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {- \mathrm{T}} = \boldsymbol {\Phi} _ {k, k - 1} ^ {- 1} \boldsymbol {P} _ {f, k / k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {- \mathrm{T}} - \boldsymbol {P} _ {f, k - 1}\tag{6.1.53}
$$

证明:由卡尔曼滤波基本方程,

即

$$
\begin{array}{l} \boldsymbol {P} _ {f, k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {P} _ {f, k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} + \boldsymbol {Q} _ {k - 1} \\ \boldsymbol {Q} _ {k - 1} = \boldsymbol {P} _ {f, k / k - 1} - \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {P} _ {f, k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \end{array}
$$

上式两边分别左乘 $\Phi_{k,k-1}^{-1}$ ，右乘 $\Phi_{k,k-1}^{-T}$ ，即得式(6.1.53)。

关系式 2:

$$
\boldsymbol {P} _ {b, k} = (\boldsymbol {P} _ {f, k / k - 1} + \boldsymbol {P} _ {b, k}) \boldsymbol {I} _ {f, k / k - 1} \boldsymbol {P} _ {s, k}\tag{6.1.54}
$$

证明：由式(6.1.48)，

$$
\begin{array}{r l} & {\pmb {I} = (\pmb {I} _ {b, k} + \pmb {I} _ {f, k / k - 1}) \pmb {P} _ {s, k}} \\ & {\pmb {P} _ {b, k} = (\pmb {I} + \pmb {P} _ {b, k} \pmb {I} _ {f, k / k - 1}) \pmb {P} _ {s, k} = \pmb {P} _ {s, k} + \pmb {P} _ {b, k} \pmb {I} _ {f, k / k - 1} \pmb {P} _ {s, k} =} \\ & {\qquad \pmb {P} _ {f, k / k - 1} \pmb {I} _ {f, k / k - 1} \pmb {P} _ {s, k} + \pmb {P} _ {b, k} \pmb {I} _ {f, k / k - 1} \pmb {P} _ {s, k} =} \\ & {\qquad (\pmb {P} _ {f, k / k - 1} + \pmb {P} _ {b, k}) \pmb {I} _ {f, k / k - 1} \pmb {P} _ {s, k}} \end{array}
$$

关系式3：

$$
\boldsymbol {P} _ {f, k / k - 1} + \boldsymbol {P} _ {b, k} = \boldsymbol {\Phi} _ {k, k - 1} (\boldsymbol {P} _ {f, k - 1} + \boldsymbol {P} _ {b, k - 1 / k}) \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}}\tag{6.1.55}
$$

证明:由卡尔曼滤波基本方程和式(6.1.38d),

$$
\begin{array}{r l} & \boldsymbol {P} _ {f, k - 1} = \boldsymbol {\Phi} _ {k, k - 1} ^ {- 1} \boldsymbol {P} _ {f, k / k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {- \mathrm{T}} - \boldsymbol {\Phi} _ {k, k - 1} ^ {- 1} Q _ {k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {- \mathrm{T}} \\ & \boldsymbol {P} _ {b, k - 1 / k} = \boldsymbol {\Phi} _ {k, k - 1} ^ {- 1} \boldsymbol {P} _ {b, k} \boldsymbol {\Phi} _ {k, k - 1} ^ {- \mathrm{T}} + \boldsymbol {\Phi} _ {k, k - 1} ^ {- 1} Q _ {k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {- \mathrm{T}} \end{array}
$$

上述两式左、右分别相加，并整理得

$$
\begin{array}{l} \boldsymbol {P} _ {f, k - 1} + \boldsymbol {P} _ {b, k - 1 / k} = \boldsymbol {\Phi} _ {k, k - 1} ^ {- 1} (\boldsymbol {P} _ {f, k / k - 1} + \boldsymbol {P} _ {b, k}) \boldsymbol {\Phi} _ {k, k - 1} ^ {- T} \\ \boldsymbol {P} _ {f, k / k - 1} + \boldsymbol {P} _ {b, k} = \boldsymbol {\Phi} _ {k, k - 1} (\boldsymbol {P} _ {f, k - 1} + \boldsymbol {P} _ {b, k - 1 / k}) \boldsymbol {\Phi} _ {k, k - 1} ^ {T} \end{array}
$$

关系式 4:

$$
\hat {\boldsymbol {X}} _ {s, k} = \boldsymbol {P} _ {s, k} \boldsymbol {I} _ {f, k} \hat {\boldsymbol {X}} _ {f, k / k - 1} - \boldsymbol {P} _ {s, k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \hat {\boldsymbol {X}} _ {f, k / k - 1} + \boldsymbol {P} _ {s, k} \boldsymbol {S} _ {b, k}
$$

其中

(6.1.56)

$$
\boldsymbol {S} _ {b, k} = \boldsymbol {P} _ {b, k} ^ {- 1} \hat {\boldsymbol {X}} _ {b, k}\tag{6.1.57}
$$

证明：由式(6.1.43b)，

$$
\boldsymbol {S} _ {b, k / k + 1} = \boldsymbol {S} _ {b, k} - \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {Z} _ {k}
$$

又由式 $(6.1.42b)$ ，式 $(6.1.41)$ 可写成

$$
\hat {\boldsymbol {X}} _ {s, k} = \boldsymbol {P} _ {s, k} \boldsymbol {I} _ {f, k} \hat {\boldsymbol {X}} _ {f, k / k - 1} + \boldsymbol {P} _ {s, k} \boldsymbol {I} _ {f, k} \boldsymbol {K} _ {f, k} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \hat {\boldsymbol {X}} _ {f, k / k - 1}) + \boldsymbol {P} _ {s, k} \boldsymbol {S} _ {b, k} - \boldsymbol {P} _ {s, k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {Z} _ {k}
$$

由式 $(2.2.4c')$

$$
\boldsymbol {K} _ {f, k} = \boldsymbol {P} _ {f, k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1}
$$

所以

$$
\begin{array}{r l} \hat {\boldsymbol {X}} _ {s, k} & = \boldsymbol {P} _ {s, k} \boldsymbol {I} _ {f, k} \hat {\boldsymbol {X}} _ {f, k / k - 1} + \boldsymbol {P} _ {s, k} \boldsymbol {I} _ {f, k} \boldsymbol {P} _ {f, k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \hat {\boldsymbol {X}} _ {f, k / k - 1}) + \boldsymbol {P} _ {s, k} \boldsymbol {S} _ {b, k} - \boldsymbol {P} _ {s, k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {Z} _ {k} = \\ & \quad \boldsymbol {P} _ {s, k} \boldsymbol {I} _ {f, k} \hat {\boldsymbol {X}} _ {f, k / k - 1} + \boldsymbol {P} _ {s, k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \hat {\boldsymbol {X}} _ {f, k / k - 1}) + \boldsymbol {P} _ {s, k} \boldsymbol {S} _ {b, k} - {\boldsymbol {P}} _ {s, k} {\boldsymbol {H}} _ {k} ^ {\mathrm{T}} {\boldsymbol {R}} _ {k} ^ {- 1} {\boldsymbol {Z}} _ {k} = \\ & \quad \boldsymbol {P} _ {s, k} \boldsymbol {I} _ {f, k} \hat {\boldsymbol {X}} _ {f, k / k - 1} - {\boldsymbol {P}} _ {s, k} {\boldsymbol {H}} _ {k} ^ {\mathrm{T}} {\boldsymbol {R}} _ {k} ^ {- 1} {\boldsymbol {H}} _ {k} \hat {\boldsymbol {X}} _ {f, k / k - 1} + {\boldsymbol {P}} _ {s, k} {\boldsymbol {S}} _ {b, k} \end{array}
$$

关系式5：

$$
\left(\boldsymbol {P} _ {f, k - 1} + \boldsymbol {P} _ {b, k - 1 / k}\right) ^ {- 1} = \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \boldsymbol {I} _ {f, k / k - 1} \left(\boldsymbol {P} _ {f, k / k - 1} - \boldsymbol {P} _ {s, k}\right) \boldsymbol {I} _ {f, k / k - 1} \boldsymbol {\Phi} _ {k, k - 1}\tag{6.1.58}
$$

证明：由式（3.4.2）和式（6.1.38a），

$$
\boldsymbol {I} _ {f, k} = \boldsymbol {I} _ {f, k / k - 1} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}\tag{6.1.59a}
$$

$$
\boldsymbol {I} _ {b, k} = \boldsymbol {I} _ {b, k / k + 1} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}\tag{6.1.59b}
$$

由上两式，得

$$
\boldsymbol {I} _ {b, k} = \boldsymbol {I} _ {b, k / k + 1} + \boldsymbol {I} _ {f, k} - \boldsymbol {I} _ {f, k / k - 1} = [ (\boldsymbol {I} _ {b, k / k + 1} + \boldsymbol {I} _ {f, k}) ^ {- 1} ] ^ {- 1} - \boldsymbol {I} _ {f, k / k - 1}
$$

将式 $(6.1.37)$ 代入上式,得

$$
\boldsymbol {I} _ {b, k} = \boldsymbol {P} _ {s, k} ^ {- 1} - \boldsymbol {I} _ {f, k / k - 1}
$$

即

$$
\boldsymbol {P} _ {b, k} = (\boldsymbol {I} _ {s, k} - \boldsymbol {I} _ {f, k / k - 1}) ^ {- 1}
$$

将上式代入式(6.1.55)，得

$$
\boldsymbol {\Phi} _ {k, k - 1} \left(\boldsymbol {P} _ {f, k - 1} + \boldsymbol {P} _ {b, k - 1 / k}\right) \boldsymbol {\Phi} _ {k, k - 1} ^ {\top} = \boldsymbol {P} _ {f, k / k - 1} + \left(\boldsymbol {I} _ {s, k} - \boldsymbol {I} _ {f, k / k - 1}\right) ^ {- 1}
$$

上式两边求逆，

$$
\begin{array}{r l} & {\Phi_ {k, k - 1} ^ {- \mathrm{T}} (P _ {f, k - 1} + P _ {b, k - 1 / k}) ^ {- 1} \Phi_ {k, k - 1} ^ {- 1} = [ P _ {f, k / k - 1} + (I _ {s, k} - I _ {f, k / k - 1}) ^ {- 1} ] ^ {- 1}} \\ & {(P _ {f, k - 1} + P _ {b, k - 1 / k}) ^ {- 1} = \Phi_ {k, k - 1} ^ {\mathrm{T}} [ P _ {f, k / k - 1} + (I _ {s, k} - I _ {f, k / k - 1}) ^ {- 1} ] ^ {- 1} \Phi_ {k, k - 1} =} \\ & {\quad \Phi_ {k, k - 1} ^ {\mathrm{T}} [ P _ {f, k / k - 1} I _ {f, k / k - 1} P _ {f, k / k - 1} + P _ {f, k / k - 1} I _ {f, k / k - 1} \cdot} \\ & {\quad (I _ {s, k} - I _ {f, k / k - 1}) ^ {- 1} I _ {f, k / k - 1} P _ {f, k / k - 1} ] ^ {- 1} \Phi_ {k, k - 1} =} \\ & {\quad \Phi_ {k, k - 1} ^ {\mathrm{T}} I _ {f, k / k - 1} [ I _ {f, k / k - 1} + I _ {f, k / k - 1} (I _ {s, k} - I _ {f, k / k - 1}) ^ {- 1} I _ {f, k / k - 1} ] ^ {- 1} I _ {f, k / k - 1} \Phi_ {k, k - 1}} \end{array}
$$

对 $(I_{s,k}-I_{f,k/k-1})^{-1}$ 应用矩阵求逆反演公式，则上式可进一步化简，

$$
\begin{array}{r l} & {(P _ {f, k - 1} + P _ {b, k - 1 / k}) ^ {- 1} = \Phi_ {k, k - 1} ^ {\mathrm{T}} I _ {f, k / k - 1} [ I _ {f, k / k - 1} + I _ {f, k / k - 1} (- P _ {f, k / k - 1} -} \\ & {\quad P _ {f, k / k - 1} (- P _ {f, k / k - 1} + P _ {s, k}) ^ {- 1} P _ {f, k / k - 1}) I _ {f, k / k - 1} ] ^ {- 1} I _ {f, k / k - 1} \Phi_ {k, k - 1} =} \\ & {\quad \Phi_ {k, k - 1} ^ {\mathrm{T}} I _ {f, k / k - 1} [ I _ {f, k / k - 1} + (- I - (- P _ {f, k / k - 1} + P _ {s, k}) ^ {- 1} \cdot} \\ & {\quad P _ {f, k / k - 1}) I _ {f, k / k - 1} ] ^ {- 1} I _ {f, k / k - 1} \Phi_ {k, k - 1} =} \\ & {\quad \Phi_ {k, k - 1} ^ {\mathrm{T}} I _ {f, k / k - 1} [ I _ {f, k / k - 1 -} I _ {f, k / k - 1} - (- P _ {f, k / k - 1} + P _ {s, k}) ^ {- 1} ] ^ {- 1} I _ {f, k / k - 1} \Phi_ {k, k - 1} =} \\ & {\quad \Phi_ {k, k - 1} ^ {\mathrm{T}} I _ {f, k / k - 1} (P _ {f, k / k - 1} - P _ {s, k}) I _ {f, k / k - 1} \Phi_ {k, k - 1}} \end{array}
$$

有了上述 5 个关系式,便可方便地推导出平滑值与滤波值间的关系。

由式(6.1.43d)和式(6.1.43c)，

$$
\begin{array}{r l} S _ {b, k - 1 / k} & = I _ {b, k - 1 / k} \Phi_ {k, k - 1} ^ {- 1} P _ {b, k} S _ {b, k} = \\ & \Phi_ {k, k - 1} ^ {\mathrm{T}} [ Q _ {k - 1} ^ {- 1} - Q _ {k - 1} ^ {- 1} (I _ {b, k} + Q _ {k - 1} ^ {- 1}) ^ {- 1} Q _ {k - 1} ^ {- 1} ] \Phi_ {k, k - 1} \Phi_ {k, k - 1} ^ {- 1} P _ {b, k} S _ {b, k} = \\ & \Phi_ {k, k - 1} ^ {\mathrm{T}} Q _ {k - 1} ^ {- 1} [ I - (I _ {b, k} + Q _ {k - 1} ^ {- 1}) ^ {- 1} Q _ {k - 1} ^ {- 1} ] P _ {b, k} S _ {b, k} = \\ & \Phi_ {k, k - 1} ^ {\mathrm{T}} Q _ {k - 1} ^ {- 1} (I _ {b, k} + Q _ {k - 1} ^ {- 1}) ^ {- 1} (I _ {b, k} + Q _ {k - 1} ^ {- 1} - Q _ {k - 1} ^ {- 1}) P _ {b, k} S _ {b, k} = \\ & \Phi_ {k, k - 1} ^ {\mathrm{T}} Q _ {k - 1} ^ {- 1} (I _ {b, k} + Q _ {k - 1} ^ {- 1}) ^ {- 1} S _ {b, k} = \\ & \Phi_ {k, k - 1} ^ {\mathrm{T}} (I + I _ {b, k} Q _ {k - 1}) ^ {- 1} S _ {b, k} \end{array}\tag{6.1.60}
$$

式 $(6.1.60)$ 也可写成

$$
\left(\boldsymbol {I} + \boldsymbol {I} _ {b, k} Q _ {k - 1}\right) \boldsymbol {\Phi} _ {k, k - 1} ^ {- T} \boldsymbol {S} _ {b, k - 1 / k} = \boldsymbol {S} _ {b, k}
$$

两边同时左乘 $\Phi_{k,k-1}^{-1}P_{b,k}$ ，

$$
\boldsymbol {\Phi} _ {k, k - 1} ^ {- 1} \boldsymbol {P} _ {b, k} \boldsymbol {\Phi} _ {k, k - 1} ^ {- \mathrm{T}} \boldsymbol {S} _ {b, k - 1 / k} + \boldsymbol {\Phi} _ {k, k - 1} ^ {- 1} Q _ {k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {- \mathrm{T}} \boldsymbol {S} _ {b, k - 1 / k} = \boldsymbol {\Phi} _ {k, k - 1} ^ {- 1} \boldsymbol {P} _ {b, k} \boldsymbol {S} _ {b, k}
$$

将式 $(6.1.53)$ 代入上式，

$$
\begin{array}{r l} & {\Phi_ {k, k - 1} ^ {- 1} P _ {b, k} \Phi_ {k, k - 1} ^ {- \mathrm{T}} S _ {b, k - 1 / k} + \Phi_ {k, k - 1} ^ {- 1} P _ {f, k / k - 1} \Phi_ {k, k - 1} ^ {- \mathrm{T}} S _ {b, k - 1 / k} - P _ {f, k - 1} S _ {b, k - 1 / k} = \Phi_ {k, k - 1} ^ {- 1} P _ {b, k} S _ {b, k}} \\ & {\quad [ \Phi_ {k, k - 1} ^ {- 1} (P _ {b, k} + P _ {f, k / k - 1}) \Phi_ {k, k - 1} ^ {- \mathrm{T}} - P _ {f, k - 1} ] S _ {b, k - 1 / k} = \Phi_ {k, k - 1} ^ {- 1} P _ {b, k} S _ {b, k}} \\ & {\quad [ (P _ {b, k} + P _ {f, k / k - 1}) \Phi_ {k, k - 1} ^ {- \mathrm{T}} - \Phi_ {k, k - 1} P _ {f, k - 1} ] S _ {b, k - 1 / k} = P _ {b, k} S _ {b, k}} \end{array}
$$

再将式 $(6.1.54)$ 代入上式的右侧

$$
\left[ \left(\boldsymbol {P} _ {f, k / k - 1} + \boldsymbol {P} _ {b, k}\right) \boldsymbol {\Phi} _ {k, k - 1} ^ {- \mathrm{T}} - \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {P} _ {f, k - 1} \right] \boldsymbol {S} _ {b, k - 1 / k} = \left(\boldsymbol {P} _ {f, k / k - 1} + \boldsymbol {P} _ {b, k}\right) \boldsymbol {I} _ {f, k / k - 1} \boldsymbol {P} _ {s, k} \boldsymbol {S} _ {b, k}
$$

式(6.1.55)代入上式后，再左乘 $(P_{f,k-1}+P_{b,k-1/k})^{-1}\Phi_{k,k-1}^{-1}$ ，得

$$
\left[ \boldsymbol {I} - \left(\boldsymbol {P} _ {f, k - 1} + \boldsymbol {P} _ {b, k - 1 / k}\right) ^ {- 1} \boldsymbol {P} _ {f, k - 1} \right] \boldsymbol {S} _ {b, k - 1 / k} = \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \boldsymbol {I} _ {f, k / k - 1} \boldsymbol {P} _ {s, k} \boldsymbol {S} _ {b, k}
$$

式 $(6.1.58)$ 代入上式，

$$
\boldsymbol {S} _ {b, k - 1 / k} - \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \boldsymbol {I} _ {f, k / k - 1} (\boldsymbol {P} _ {f, k / k - 1} - \boldsymbol {P} _ {s, k}) \boldsymbol {I} _ {f, k / k - 1} \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {P} _ {f, k - 1} \boldsymbol {S} _ {b, k - 1 / k} = \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \boldsymbol {I} _ {f, k / k - 1} \boldsymbol {P} _ {s, k} \boldsymbol {S} _ {b, k}\tag{6.1.61}
$$

以 $\Phi_{k,k - 1}^{-1}\hat{X}_{f,k / k - 1}$ 右乘式(6.1.58）左、右两边，

$$
- \left(\boldsymbol {P} _ {f, k - 1} + \boldsymbol {P} _ {b, k - 1 / k}\right) ^ {- 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {- 1} \boldsymbol {X} _ {f, k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \boldsymbol {I} _ {f, k / k - 1} \left(\boldsymbol {P} _ {s, k} - \boldsymbol {P} _ {f, k / k - 1}\right) \boldsymbol {I} _ {f, k / k - 1} \hat {\boldsymbol {X}} _ {f, k / k - 1}\tag{6.1.62}
$$

式 $(6.1.61)$ 和式 $(6.1.62)$ 左、右两侧分别相加，得

$$
\begin{array}{r l} & {\pmb {S} _ {b, k - 1 / k} - \pmb {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \pmb {I} _ {f, k / k - 1} (\pmb {P} _ {f, k / k - 1} - \pmb {P} _ {s, k}) \pmb {I} _ {f, k / k - 1} \pmb {\Phi} _ {k, k - 1} \pmb {P} _ {f, k - 1} \pmb {S} _ {b, k - 1 / k} -} \\ & {\qquad (\pmb {P} _ {f, k - 1} + \pmb {P} _ {b, k - 1 / k}) ^ {- 1} \pmb {\Phi} _ {k, k - 1} ^ {- 1} \hat {\pmb {X}} _ {f, k / k - 1} =} \\ & {\qquad \pmb {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \pmb {I} _ {f, k / k - 1} \pmb {P} _ {s, k} \pmb {S} _ {b, k} + \pmb {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \pmb {I} _ {f, k / k - 1} (\pmb {P} _ {s, k} - \pmb {P} _ {f, k / k - 1}) \pmb {I} _ {f, k / k - 1} \hat {\pmb {X}} _ {f, k / k - 1}} \end{array}
$$

将(6.1.56)式确定的 $P_{s,k}S_{b,k}$ 代入上式,得

$$
\begin{array}{r l} & S _ {b, k - 1 / k} - \Phi_ {k, k - 1} ^ {\mathrm{T}} I _ {f, k / k - 1} (P _ {f, k / k - 1} - P _ {s, k}) I _ {f, k / k - 1} \Phi_ {k, k - 1} P _ {f, k - 1} S _ {b, k - 1 / k} - \\ & (P _ {f, k - 1} + P _ {b, k - 1 / k}) ^ {- 1} \Phi_ {k, k - 1} ^ {- 1} \hat {X} _ {f, k / k - 1} = \\ & \Phi_ {k, k - 1} ^ {\mathrm{T}} I _ {f, k / k - 1} \hat {X} _ {s, k} - \Phi_ {k, k - 1} ^ {\mathrm{T}} I _ {f, k / k - 1} P _ {s, k} I _ {f, k} \hat {X} _ {f, k / k - 1} + \\ & \Phi_ {k, k - 1} ^ {\mathrm{T}} I _ {f, k / k - 1} P _ {s, k} H _ {k} ^ {\mathrm{T}} R _ {k} ^ {- 1} H _ {k} \hat {X} _ {f, k / k - 1} + \\ & \Phi_ {k, k - 1} ^ {\mathrm{T}} I _ {f, k / k - 1} (P _ {s, k} - P _ {f, k / k - 1}) I _ {f, k / k - 1} \hat {X} _ {f, k / k - 1} \end{array}
$$

即

$$
\begin{array}{r l} & S _ {b, k - 1 / k} - \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \boldsymbol {I} _ {f, k / k - 1} (\boldsymbol {P} _ {f, k / k - 1} - \boldsymbol {P} _ {s, k}) \boldsymbol {I} _ {f, k / k - 1} \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {P} _ {f, k - 1} \boldsymbol {S} _ {b, k - 1 / k} + \\ & \left\{- (\boldsymbol {P} _ {f, k - 1} + \boldsymbol {P} _ {b, k - 1 / k}) ^ {- 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {- 1} + \left[ \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \boldsymbol {I} _ {f, k / k - 1} \boldsymbol {P} _ {s, k} \boldsymbol {I} _ {f, k} - \right. \right. \\ & \left. \left. \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \boldsymbol {I} _ {f, k / k - 1} \boldsymbol {P} _ {s, k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} - \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \boldsymbol {I} _ {f, k / k - 1} \boldsymbol {P} _ {s, k} \boldsymbol {I} _ {f, k / k - 1} \right] \right\} \hat {\boldsymbol {X}} _ {f, k / k - 1} = \\ & \left. \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \boldsymbol {I} _ {f, k / k - 1} (\hat {\boldsymbol {X}} _ {s, k} - \hat {\boldsymbol {X}} _ {f, k / k - 1}) \right. \end{array}\tag{6.1.63}
$$

根据式 $(6.1.59a)$ ,

$$
\boldsymbol {I} _ {f, k} - \boldsymbol {I} _ {f, k / k - 1} = \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}
$$

所以，式 $(6.1.63)$ 左边方括号内的值为

$$
\begin{array}{r l} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \boldsymbol {I} _ {f, k / k - 1} \boldsymbol {P} _ {s, k} \boldsymbol {I} _ {f, k} - \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \boldsymbol {I} _ {f, k / k - 1} \boldsymbol {P} _ {s, k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} - \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \boldsymbol {I} _ {f, k / k - 1} \boldsymbol {P} _ {s, k} \boldsymbol {I} _ {f, k / k - 1} = & \\ \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \boldsymbol {I} _ {f, k / k - 1} \boldsymbol {P} _ {s, k} (\boldsymbol {I} _ {f, k} - \boldsymbol {I} _ {f, k / k - 1}) - \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \boldsymbol {I} _ {f, k / k - 1} \boldsymbol {P} _ {s, k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} = & 0 \end{array}
$$

这样式(6.1.63)中大括号内的值可写为

$$
- \left(\boldsymbol {P} _ {f, k - 1} + \boldsymbol {P} _ {b, k - 1 / k}\right) ^ {- 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {- 1} = - \boldsymbol {I} _ {b, k - 1 / k} \left(\boldsymbol {I} + \boldsymbol {P} _ {f, k - 1} \boldsymbol {I} _ {b, k - 1 / k}\right) ^ {- 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {- 1}
$$

又由于 $\pmb{\Phi}_{k,k - 1}^{-1}\hat{\pmb{X}}_{f,k / k - 1} = \hat{\pmb{X}}_{f / k - 1}$ ，所以式(6.1.63）可以写成：

$$
\begin{array}{r l} & \boldsymbol {S} _ {b, k - 1 / k} - \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \boldsymbol {I} _ {f, k / k - 1} (\boldsymbol {P} _ {f, k / k - 1} - \boldsymbol {P} _ {s, k}) \boldsymbol {I} _ {f, k / k - 1} \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {P} _ {f, k - 1} \boldsymbol {S} _ {b, k - 1 / k} - \\ & \quad \boldsymbol {I} _ {b, k - 1 / k} (\boldsymbol {I} + \boldsymbol {P} _ {f, k - 1} \boldsymbol {I} _ {b, k - 1 / k}) ^ {- 1} \hat {\boldsymbol {X}} _ {f, k - 1} = \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \boldsymbol {I} _ {f, k / k - 1} (\hat {\boldsymbol {X}} _ {s, k} - \hat {\boldsymbol {X}} _ {f, k / k - 1}) \end{array}
$$

根据式 $(6.1.51)$ 和式 $(6.1.52)$ ，上式可写成：

$$
\begin{array}{r l} & \boldsymbol {S} _ {b, k - 1 / k} - \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \boldsymbol {I} _ {f, k / k - 1} (\boldsymbol {P} _ {f, k / k - 1} - \boldsymbol {P} _ {f, k} + \boldsymbol {K} _ {s, k} \boldsymbol {P} _ {f, k + 1 / k} \boldsymbol {K} _ {s, k} ^ {\mathrm{T}} - \boldsymbol {K} _ {s, k} \boldsymbol {P} _ {s, k + 1} \boldsymbol {K} _ {s, k} ^ {\mathrm{T}}) \boldsymbol {K} _ {s, k - 1} ^ {\mathrm{T}} \boldsymbol {S} _ {b, k - 1 / k} - \\ & \boldsymbol {I} _ {b, k - 1 / k} (\boldsymbol {I} + \boldsymbol {P} _ {f, k - 1} \boldsymbol {I} _ {b, k - 1 / k}) ^ {- 1} \hat {\boldsymbol {X}} _ {f, k - 1} = \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \boldsymbol {I} _ {f, k / k - 1} (\hat {\boldsymbol {X}} _ {s, k} - \hat {\boldsymbol {X}} _ {f, k / k - 1}) \end{array}
$$

上式左、右两侧各左乘 $P_{f,k-1}$ ，并根据式(6.1.51)，

$$
\begin{array}{r l} & {\left[ \boldsymbol {P} _ {f, k - 1} - \boldsymbol {P} _ {f, k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \boldsymbol {I} _ {f, k / k - 1} \left(\boldsymbol {P} _ {f, k / k - 1} - \boldsymbol {P} _ {f, k} + \boldsymbol {K} _ {s, k} \boldsymbol {P} _ {f, k + 1 / k} \boldsymbol {K} _ {s, k} ^ {\mathrm{T}} - \boldsymbol {K} _ {s, k} \boldsymbol {P} _ {s, k + 1} \boldsymbol {K} _ {s, k} ^ {\mathrm{T}}\right) \boldsymbol {K} _ {s, k - 1} ^ {\mathrm{T}} \right] \cdot} \\ & {\quad \boldsymbol {S} _ {b, k - 1 / k} - \boldsymbol {P} _ {f, k - 1} \boldsymbol {I} _ {b, k - 1 / k} (\boldsymbol {I} + \boldsymbol {P} _ {f, k - 1} \boldsymbol {I} _ {b, k - 1 / k}) ^ {- 1} \hat {\boldsymbol {X}} _ {f, k - 1} = \boldsymbol {K} _ {s, k - 1} (\hat {\boldsymbol {X}} _ {s, k} - \hat {\boldsymbol {X}} _ {f, k / k - 1})} \end{array}\tag{6.1.64}
$$

根据式 $(6.1.51)$ 及式 $(6.1.52)$ ，式 $(6.1.64)$ 等号左边方括号内的值可写成：

$$
\begin{array}{r l} & {\pmb {P} _ {f, k - 1} - \pmb {K} _ {s, k - 1} (\pmb {P} _ {f, k / k - 1} - \pmb {P} _ {f, k} + \pmb {K} _ {s, k} \pmb {P} _ {f, k + 1 / k} \pmb {K} _ {s, k} ^ {\mathrm{T}} - \pmb {K} _ {s, k} \pmb {P} _ {s, k + 1} \pmb {K} _ {s, k} ^ {\mathrm{T}}) \pmb {K} _ {s, k - 1} ^ {\mathrm{T}} =} \\ & {\qquad \pmb {P} _ {f, k - 1} - \pmb {K} _ {s, k - 1} (\pmb {P} _ {f, k / k - 1} - \pmb {P} _ {f, k} + \pmb {K} _ {s, k} \pmb {P} _ {f, k + 1 / k} \pmb {K} _ {s. k} ^ {\mathrm{T}} - \pmb {P} _ {s, k} + \pmb {P} _ {f, k} - \pmb {K} _ {s, k} \pmb {P} _ {f, k + 1 / k} \pmb {K} _ {s, k} ^ {\mathrm{T}}) \pmb {K} _ {s, k - 1} ^ {\mathrm{T}} =} \\ & {\qquad \pmb {P} _ {f, k - 1} - \pmb {K} _ {s, k - 1} \pmb {P} _ {f, k / k - 1} \pmb {K} _ {s, k - 1} ^ {\mathrm{T}} + \pmb {K} _ {s, k - 1} \pmb {P} _ {s, k} \pmb {K} _ {s, k - 1} ^ {\mathrm{T}} =} \\ & {\qquad \pmb {P} _ {f, k - 1} - \pmb {K} _ {s, k - 1} \pmb {P} _ {f, k / k - 1} \pmb {K} _ {s, k - 1} ^ {\mathrm{T}} + \pmb {{P}} _ {s, k - 1} - \pmb {{P}} _ {f, k - 1} + \pmb {{K}} _ {s, k - 1} \pmb {{P}} _ {f, k / k - 1} \pmb {{K}} _ {s, k - 1} ^ {\mathrm{T}} = \pmb {{P}} _ {s, k - 1}} \end{array}
$$

所以式 $(6.1.64)$ 可写成：

$$
\boldsymbol {P} _ {s, k - 1} \boldsymbol {S} _ {b, k - 1 / k} = \boldsymbol {P} _ {f, k - 1} \boldsymbol {I} _ {b, k - 1 / k} (\boldsymbol {I} + \boldsymbol {P} _ {f, k - 1} \boldsymbol {I} _ {b, k - 1 / k}) ^ {- 1} \hat {\boldsymbol {X}} _ {f, k - 1} = \boldsymbol {K} _ {s, k - 1} (\hat {\boldsymbol {X}} _ {s, k} - \hat {\boldsymbol {X}} _ {f, k / k - 1})\tag{6.1.65}
$$

由式 $(6.1.40)$ 和式 $(6.1.41)$

$$
\begin{array}{r l} \hat {\mathbf {X}} _ {s, k} & = (\mathbf {I} _ {f, k} + \mathbf {I} _ {b, k / k + 1}) ^ {- 1} \mathbf {I} _ {f, k} \hat {\mathbf {X}} _ {f, k} + \mathbf {P} _ {s, k} \mathbf {I} _ {b, k / k + 1} \hat {\mathbf {X}} _ {b, k / k + 1} = \\ & (\mathbf {I} + \mathbf {P} _ {f, k} \mathbf {I} _ {b, k / k + 1}) ^ {- 1} \hat {\mathbf {X}} _ {f, k} + \mathbf {P} _ {s, k} \mathbf {S} _ {b, k / k + 1} \end{array}
$$

$$
\begin{array}{r l} \hat {\mathbf {X}} _ {s, k} - \hat {\mathbf {X}} _ {f, k} & = [ (\mathbf {I} + \mathbf {P} _ {f, k} \mathbf {I} _ {b, k / k + 1}) ^ {- 1} - \mathbf {I} ] \hat {\mathbf {X}} _ {f, k} + \mathbf {P} _ {s, k} \mathbf {S} _ {b, k / k + 1} = \\ & [ \mathbf {I} - (\mathbf {I} + \mathbf {P} _ {f, k} \mathbf {I} _ {b, k / k + 1}) ] (\mathbf {I} + \mathbf {P} _ {f, k} \mathbf {I} _ {b, k / k + 1}) ^ {- 1} \hat {\mathbf {X}} _ {f, k} + \mathbf {P} _ {s, k} \mathbf {S} _ {b, k / k + 1} = \\ & - \mathbf {P} _ {f, k} \mathbf {I} _ {b, k / k + 1} (\mathbf {I} + \mathbf {P} _ {f, k} \mathbf {I} _ {b, k / k + 1}) ^ {- 1} \hat {\mathbf {X}} _ {f, k} + \mathbf {P} _ {s, k} \mathbf {S} _ {b, k / k + 1} \end{array} \tag {6.1.66}
$$

式(6.1.66)的下标 k 换成 k-1，并代入式(6.1.65)，得

即

$$
\begin{array}{r l} \hat {\boldsymbol {X}} _ {s, k - 1} - \hat {\boldsymbol {X}} _ {f, k - 1} & = \boldsymbol {K} _ {s, k - 1} (\hat {\boldsymbol {X}} _ {s, k} - \hat {\boldsymbol {X}} _ {f, k / k - 1}) \\ \hat {\boldsymbol {X}} _ {s, k} & = \hat {\boldsymbol {X}} _ {f, k} + \boldsymbol {K} _ {s, k} (\hat {\boldsymbol {X}} _ {s, k + 1} - \hat {\boldsymbol {X}} _ {f, k + 1 / k}) \end{array}\tag{6.1.67}
$$

(3)RTS平滑算法归纳。根据上述分析，由式(6.1.51)、(6.1.52)和式(6.1.67)，可归纳出固定区间RTS平滑算法如下。

对于给定在 $t_{1}$ 至 $t_{N}$ 区间上的量测量 $Z_{1}, Z_{2}, \cdots, Z_{N}$ ，先按顺序作正向卡尔曼滤波，获得并顺序记录： $\hat{X}_{f,k}; \Phi_{k,k-1}; \hat{X}_{f,k/k-1}; P_{f,k}; P_{f,k/k-1}$ 。 $k = 1, 2, \cdots N$ 。

当 k=N 时,逆序执行 RTS 平滑算法:

$$
\begin{array}{r l} & {\pmb {K} _ {s, k} = \pmb {P} _ {f, k} \pmb {\Phi} _ {k + 1, k} ^ {\mathrm{T}} \pmb {P} _ {f, k + 1 / k} ^ {- 1}} \\ & {\hat {\pmb {X}} _ {s, k} = \hat {\pmb {X}} _ {f, k} + \pmb {K} _ {s, k} (\hat {\pmb {X}} _ {s, k + 1} - \hat {\pmb {X}} _ {f, k + 1 / k})} \\ & {\pmb {P} _ {s, k} = \pmb {P} _ {f, k} - \pmb {K} _ {s, k} (\pmb {P} _ {f, k + 1 / k} - \pmb {P} _ {s, k + 1}) \pmb {K} _ {s, k} ^ {\mathrm{T}}} \\ & {k = N - 1, N - 2, \dots 2, 1, 0} \end{array}
$$

式中 $P_{s,N}=P_{f,N};\hat{X}_{s,N}=\hat{X}_{f,N}$

固定区间平滑利用整个时间区间上的所有量测 $Z_{1}, Z_{2}, \cdots, Z_{k}, \cdots, Z_{N}$ 对区间内每一时刻 $t_{k}$ 上的状态 $X_{k}$ 作最优估计，而滤波只利用 $t_{k}$ 及 $t_{k}$ 时刻以前的量测 $Z_{1}, Z_{2}, \cdots, Z_{k}$ 作最优估计，由于平滑使用的量测比滤波使用的多，所以平滑精度比滤波精度高。但固定区间平滑是一种离线处理方法，只能用在无实时要求的场合，如对试飞数据的处理，根据地面测轨系统对卫星的测量数据计算卫星的入轨参数，根据挂飞数据计算机载导弹捷联惯导传递对准精度等。

RTS算法简洁而有效，但如果计算误差积累使式(6.1.52)右侧第二项超过第一项时，则有可能使 $P_{s,k}$ 成负定阵，这将使平滑结果的合理性得不到保证。为了确保 $P_{s,k}$ 始终非负定，文献[66]提出了对RTS算法改进的算法，即Watanabe算法：

$$
\begin{array}{r l} & {\pmb {S} _ {k} = \pmb {\Phi} _ {k + 1, k} ^ {- \mathrm{T}} \pmb {P} _ {f, k} ^ {- 1} \pmb {\Phi} _ {k + 1, k} ^ {- 1}} \\ & {\pmb {q} _ {k} = \pmb {\Phi} _ {k + 1, k} ^ {- \mathrm{T}} \pmb {P} _ {f, k} ^ {- 1} \hat {\pmb {X}} _ {f, k}} \\ & {\pmb {\Delta} _ {k} = (\pmb {\Gamma} _ {k} ^ {\mathrm{T}} \pmb {S} _ {k} \pmb {\Gamma} _ {k} + \pmb {Q} _ {k} ^ {- 1}) ^ {- 1}} \\ & {\pmb {K} _ {k} = \pmb {S} _ {k} \pmb {\Gamma} _ {k} \pmb {\Delta} _ {k} ^ {\mathrm{T}}} \\ & {\pmb {y} _ {k} = \pmb {\Gamma} _ {k} ^ {\mathrm{T}} \pmb {q} _ {k}} \\ & {\overline {{\pmb {X}}} _ {s, k} = (\pmb {I} - \pmb {\Gamma} _ {k} \pmb {K} _ {k} ^ {\mathrm{T}}) \hat {\pmb {X}} _ {s, k + 1} + \pmb {\Gamma} _ {k} \pmb {\Delta} _ {k} \pmb {y} _ {k}} \\ & {\hat {\pmb {X}} _ {s, k} = \pmb {\Phi} _ {k + 1, k} ^ {- 1} \overline {{\pmb {X}}} _ {s, k}} \\ & {\overline {{\pmb {P}}} _ {s, k} = (\pmb {I} - \pmb {\Gamma} _ {k} \pmb {K} _ {k} ^ {\mathrm{T}}) \pmb {P} _ {s, k + 1} (\pmb {I} - \pmb {\Gamma} _ {k} \pmb {K} _ {k} ^ {\mathrm{T}}) + \pmb {\Gamma} _ {k} \pmb {\Delta} _ {k} \pmb {\Gamma} _ {k} ^ {\mathrm{T}}} \\ & {\pmb {P} _ {s, k} = \pmb {\Phi} _ {k + 1, k} ^ {- 1} \overline {{\pmb {P}}} _ {s, k} \pmb {\Phi} _ {k + 1, k} ^ {- \mathrm{T}}} \\ & {k = N - 1, N - 2, \dots , 2, 1, 0} \end{array}
$$

式中 $P_{s,N}=P_{f,N},\hat{X}_{s,N}=\hat{X}_{f,N}$

## 6.2 非线性系统滤波之一: EKF

## 6.2.1 概述

第二章所讨论的卡尔曼滤波问题都是假设物理系统（动态系统和量测系统）的数学模型是线性的。但是，工程实践中所遇到的物理系统数学模型则往往是非线性的，即系统方程是非线性的，或系统方程和量测方程均是非线性的。例如：飞机和舰船上的惯性导航系统、导弹的制导系统、多普勒导航系统、卫星导航系统以及其他很多工业控制系统等，一般都是非线性系统。

一般的非线性连续系统和离散系统的方程可由以下形式描述：

$$
\dot {\boldsymbol {X}} (t) = \boldsymbol {f} [ \boldsymbol {X} (t), \boldsymbol {w} (t), t ]\tag{6.2.1}
$$

$$
\mathbf {Z} (t) = \mathbf {h} [ \mathbf {X} (t), \nu (t), t ]\tag{6.2.2}
$$

或

$$
\boldsymbol {X} _ {k} = \boldsymbol {f} \left[ \boldsymbol {X} _ {k - 1}, \boldsymbol {W} _ {k - 1}, k - 1 \right]\tag{6.2.3}
$$

$$
\mathbf {Z} _ {k} = \mathbf {h} [ \mathbf {X} _ {k}, \mathbf {V} _ {k}, k ]\tag{6.2.4}
$$

式中， $f[\cdot]$ 是 n 维向量函数，对其自变量而言是非线性的； $h[\cdot]$ 是 m 维向量函数，它对自变量而言也是非线性的； $w(t)$ 或 $\{W_{k-1}\}$ 和 $v(t)$ 或 $\{V_k\}$ 分别是 r 维随机系统动态噪声和 m 维量测系统噪声，初始状态 $X(0)$ 或 $X_0$ 通常是任意值的 n 维随机向量。

如果噪声 $w(t)$ 或 $\{W_{k-1}\}$ 和 $v(t)$ 或 $\{V_k\}$ 的概率分布是任意的，那么系统式(6.2.1)和式(6.2.2)或式(6.2.3)和式(6.2.4)所描述的将是属于非常一般的随机非线性系统。这类系统的最优估计问题的求解是极不方便的。因此，为了使估计问题得到可行的解答，必须对噪声的统计特性给以符合实际而又便于数学处理的假定。本章研究的非线性最优估计问题的随机非线性系统的数学模型是属如下这种类型：

$$
\dot {\boldsymbol {X}} (t) = \boldsymbol {f} [ \boldsymbol {X} (t), t ] + \boldsymbol {G} (t) \boldsymbol {w} (t)\tag{6.2.5}
$$

$$
\mathbf {Z} (t) = \mathbf {h} [ \mathbf {X} (t), t ] + \mathbf {v} (t)\tag{6.2.6}
$$

或

$$
\boldsymbol {X} _ {k} = \boldsymbol {f} [ \boldsymbol {X} _ {k - 1}, k - 1 ] + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {W} _ {k - 1}\tag{6.2.7}
$$

$$
\mathbf {Z} _ {k} = \mathbf {h} [ \mathbf {X} _ {k}, k ] + \mathbf {V} _ {k}\tag{6.2.8}
$$

式中 $w(t)$ 或 $\{\pmb{W}_{k - 1}\}$ 和 $v(t)$ 或 $\{\pmb{V}_k\}$ 均是彼此不相关的零均值白噪声序列，它们与初始状态 $\pmb {X}(0)$ 或 $\pmb{X_0}$ 也不相关，即对于 $t\geqslant t_0$ 或 $k - 1\geqslant 0$ 有

$$
\begin{array}{l} E [ w (t) ] = 0, E [ w (t) \cdot w ^ {\mathrm{T}} (\tau) ] = q (t) \delta (t - \tau) \\ E [ v (t) ] = 0, E [ v (t) \cdot v ^ {\mathrm{T}} (\tau) ] = r (t) \delta (t - \tau) \\ E [ w (t) \cdot v ^ {\mathrm{T}} (\tau) ] = 0, E [ X (0) \cdot w _ {(\tau)} ^ {\mathrm{T}} ] = 0 \quad E [ X (0) V ^ {\mathrm{T}} (\tau) = 0 ] \end{array}
$$

或

$$
\begin{array}{l} {E [ \pmb {W} _ {k - 1} ] = \pmb {0}, E [ \pmb {W} _ {k - 1} \pmb {W} _ {j - 1} ^ {\mathrm{T}} ] = \pmb {Q} _ {k - 1} \delta_ {k - 1, j - 1}} \\ {E [ \pmb {V} _ {k} ] = \pmb {0}, \quad E [ \pmb {V} _ {k} \pmb {V} _ {j} ^ {\mathrm{T}} ] = \pmb {R} _ {k} \delta_ {k, j}} \\ {E [ \pmb {W} _ {k} \pmb {V} _ {j} ^ {\mathrm{T}} ] = \pmb {0}, E [ \pmb {X} _ {0} \pmb {W} _ {k - 1} ^ {\mathrm{T}} ] = \pmb {0}, E [ \pmb {X} _ {0} \pmb {V} _ {k} ^ {\mathrm{T}} ] = \pmb {0}} \end{array}
$$

而初始状态为具有如下均值和方差(强度)阵的随机向量

$$
E [ \mathbf {X} (0) ] = m _ {\mathbf {X} (0)}, \quad \operatorname{Var} [ \mathbf {X} (0) ] = C _ {\mathbf {X} (0)}
$$

或

$$
E [ \boldsymbol {X} _ {0} ] = m _ {\boldsymbol {X} _ {0}}, \operatorname{Var} [ \boldsymbol {X} _ {0} ] = C _ {\boldsymbol {X} _ {0}}
$$

以及 $X(t)$ 或 $X_{k}$ 是 n 维状态向量， $f[X(t),t]$ 或 $f[X_{k-1},k-1]$ 是 n 维非线性向量连续函数或离散函数。 $G(t)$ 或 $\Gamma_{k-1}$ 是 $n \times r$ 矩阵， $h[X(t),t]$ 或 $h[X_{k},k]$ 是 m 维非线性向量连续函数或离散函数。

考虑了最优控制信号 $\boldsymbol{u}(t)$ 或 $U_{k-1}$ 后，式(6.2.5) 和式(6.2.7) 可以改写为

$$
\dot {\boldsymbol {X}} (t) = \boldsymbol {f} [ \boldsymbol {X} (t), t ] + \boldsymbol {B} (t) \boldsymbol {u} (t) + \boldsymbol {G} (t) \boldsymbol {w} (t)\tag{6.2.9a}
$$

或

$$
\boldsymbol {X} _ {k} = \boldsymbol {f} \left[ \boldsymbol {X} _ {k - 1}, k - 1 \right] + \boldsymbol {T} _ {k - 1} \boldsymbol {U} _ {k - 1} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {W} _ {k - 1}\tag{6.2.9b}
$$

卡尔曼滤波基本方程不能直接用来解决非线性系统的滤波问题,但如果系统的非线性较弱,则可对系统方程和量测方程中的非线性函数作泰勒级数展开并仅保留线性项,获得线性模型,此时,卡尔曼滤波基本方程就完全适用了。

设有多元向量函数

$$
\mathbf {Y} = \mathbf {g} (x _ {1}, x _ {2}, \dots , x _ {n})
$$

记

$$
\boldsymbol {X} = \left[ \begin{array}{c} x _ {1} \\ x _ {2} \\ \vdots \\ x _ {n} \end{array} \right], \quad \bar {\boldsymbol {X}} = \left[ \begin{array}{c} \overline {{x}} _ {1} \\ \overline {{x}} _ {2} \\ \vdots \\ \overline {{x}} _ {n} \end{array} \right], \quad \delta \boldsymbol {X} = \left[ \begin{array}{c} \delta x _ {1} \\ \delta x _ {2} \\ \vdots \\ \delta x _ {n} \end{array} \right], \quad \nabla = \left[ \begin{array}{c} \frac {\partial}{\partial x _ {1}} \\ \frac {\partial}{\partial x _ {2}} \\ \vdots \\ \frac {\partial}{\partial x _ {n}} \end{array} \right]
$$

则该多元向量函数的泰勒级数为

$$
\mathbf {Y} = \mathbf {g} (\bar {x} _ {1}, \bar {x} _ {2}, \dots \bar {x} _ {n}) + \sum_ {i = 1} ^ {\infty} \frac {1}{i !} \left(\delta x _ {1} \frac {\partial}{\partial x _ {1}} + \delta x _ {2} \frac {\partial}{\partial x _ {2}} + \dots + \delta x _ {n} \frac {\partial}{\partial x _ {n}}\right) ^ {i} \cdot \mathbf {g} (\bar {x} _ {1}, \bar {x} _ {2}, \dots , \bar {x} _ {n}) =
$$

$$
\mathbf {g} (\bar {\boldsymbol {X}}) + \sum_ {i = 1} ^ {\infty} \frac {1}{i !} (\delta \boldsymbol {X} \cdot \nabla) ^ {i} \mathbf {g} (\bar {\boldsymbol {X}})
$$

若只保留线性项,则

$$
\mathbf {Y} = \mathbf {g} (\bar {\mathbf {X}}) + \left(\delta x _ {1} \frac {\partial \mathbf {g}}{\partial x _ {1}} + \delta x _ {2} \frac {\partial \mathbf {g}}{\partial x _ {2}} + \dots + \delta x _ {n} \frac {\partial \mathbf {g}}{\partial x _ {n}}\right) \Big | _ {\mathbf {x} = \mathbf {X}} =
$$

$$
\mathbf {g} (\bar {\mathbf {X}}) + \left[ \frac {\partial \mathbf {g}}{\partial x _ {1}} \quad \frac {\partial \mathbf {g}}{\partial x _ {2}} \quad \dots \quad \frac {\partial \mathbf {g}}{\partial x _ {n}} \right] \Big | _ {\mathbf {X} = \bar {\mathbf {X}}} \cdot \delta \mathbf {X}
$$

根据附录关于矩阵间的微分关系(A-1)，上式可写成

$$
\mathbf {Y} = \mathbf {g} (\bar {\mathbf {X}}) + \frac {\partial \mathbf {g}}{\partial \mathbf {X} ^ {\mathrm{T}}} \Big | _ {\mathbf {X} = \bar {\mathbf {X}}} \cdot \delta \mathbf {X}
$$

其中

$$
\frac {\partial \mathbf {g}}{\partial \mathbf {X} ^ {\mathrm{T}}} = \left[ \begin{array}{l l l l} \frac {\partial \mathbf {g}}{\partial x _ {1}} & \frac {\partial \mathbf {g}}{\partial x _ {2}} & \dots & \frac {\partial \mathbf {g}}{\partial x _ {n}} \end{array} \right]\tag{6.2.10}
$$

称为非线性函数 g 的雅可比矩阵。

## 6.2.2 按标称状态线性化的卡尔曼滤波方程

1. 围绕标称状态的线性化

当 $w(t)$ 或 $W_{k-1}$ 和 $v(t)$ 或 $V_{k}$ 的噪声恒为零时，系统模型式(6.2.9a)和式(6.2.6)或式(6.2.9b)和式(6.2.8)的解称为非线性方程的理论解，又称“标称轨迹”或“标称状态”。因此,如果把标称轨迹上的值记作 $X^{n}(t)$ 或 $X_{k}^{n}$ 和 $Z^{n}(t)$ 或 $Z_{k}^{n}$ ,则得

$$
\dot {\boldsymbol {X}} ^ {\mathrm{n}} (t) = \boldsymbol {f} [ \boldsymbol {X} ^ {\mathrm{n}} (t), t ] + \boldsymbol {B} (t) \boldsymbol {u} (t)\tag{6.2.11}
$$

$$
\mathbf {Z} ^ {\mathrm{n}} (t) = \mathbf {h} [ \mathbf {X} ^ {\mathrm{n}} (t), t ]
$$

(6.2.12)

或

$$
\boldsymbol {X} _ {k} ^ {\mathrm{n}} = \boldsymbol {f} \left[ \boldsymbol {X} _ {k - 1} ^ {\mathrm{n}}, k - 1 \right] + \boldsymbol {T} _ {k - 1} \boldsymbol {U} _ {k - 1}\tag{6.2.13}
$$

$$
\mathbf {Z} _ {k} ^ {\mathrm{n}} = \mathbf {h} [ \mathbf {X} _ {k} ^ {\mathrm{n}}, k ]\tag{6.2.14}
$$

一般把非线性系统式(6.2.9a)和式(6.2.6)或式(6.2.9b)和式(6.2.8)的真实解称之为“真轨迹”或“真状态”。在真轨迹上的值为 $X(t)$ 或 $X_{k}$ 和 $Z(t)$ 或 $Z_{k}$ 。非线性系统的真轨迹运动与标称轨迹运动的偏差为

$$
\Delta \boldsymbol {X} (t) = \boldsymbol {X} (t) - \boldsymbol {X} ^ {\mathrm{n}} (t)\tag{6.2.15}
$$

$$
\Delta \mathbf {Z} (t) = \mathbf {Z} (t) - \mathbf {Z} ^ {\mathrm{n}} (t)\tag{6.2.16}
$$

或

$$
\Delta X _ {k} = X _ {k} - X _ {k} ^ {\mathrm{n}}\tag{6.2.17}
$$

$$
\Delta \mathbf {Z} _ {k} = \mathbf {Z} _ {k} - \mathbf {Z} _ {k} ^ {\mathrm{n}}\tag{6.2.18}
$$

如果这些偏差都足够小,那么,可以围绕标称状态把 $X(t)$ 和 $Z(t)$ 展开成泰勒级数,并且可取一次近似值得

$$
\dot {\boldsymbol {X}} (t) = \boldsymbol {f} [ \boldsymbol {X} (t), t ] \mid_ {\boldsymbol {X} (t) = \boldsymbol {X} ^ {\mathrm{n}} (t)} + \frac {\partial \boldsymbol {f} [ \boldsymbol {X} (t) , t ]}{\partial \boldsymbol {X} ^ {\mathrm{T}} (t)} \Bigg | _ {\boldsymbol {X} (t) = \boldsymbol {X} ^ {\mathrm{n}} (t)} \Delta \boldsymbol {X} (t) + \boldsymbol {B} (t) \boldsymbol {u} (t) + \boldsymbol {G} (t) \boldsymbol {w} (t)\tag{6.2.19}
$$

$$
\mathbf {Z} (t) = \boldsymbol {h} [ \mathbf {X} (t), t ] \left. \right| _ {\mathbf {X} (t) = \mathbf {X} ^ {\mathrm{n}} (t)} + \frac {\partial \boldsymbol {h} [ \mathbf {X} (t) , t ]}{\partial \mathbf {X} ^ {\mathrm{T}} (t)} \Bigg | _ {\mathbf {X} (t) = \mathbf {X} ^ {\mathrm{n}} (t)} \Delta \mathbf {X} (t) + \boldsymbol {v} (t)\tag{6.2.20}
$$

考虑与式 $(6.2.11)$ 和式 $(6.2.12)$ 的关系后，上面两式可改写成

$$
\dot {\boldsymbol {X}} (t) = \dot {\boldsymbol {X}} ^ {\mathrm{n}} (t) + \boldsymbol {F} ^ {\mathrm{n}} (t) \Delta \boldsymbol {X} (t) + \boldsymbol {G} \boldsymbol {w} (t)\tag{6.2.21}
$$

$$
\mathbf {Z} (t) = \mathbf {Z} ^ {\mathrm{n}} (t) + \mathbf {H} ^ {\mathrm{n}} (t) \Delta \mathbf {X} (t) + \nu (t)\tag{6.2.22}
$$

或

$$
\Delta \dot {\boldsymbol {X}} (t) = \boldsymbol {F} ^ {\mathrm{n}} (t) \Delta \boldsymbol {X} (t) + \boldsymbol {G} (t) \boldsymbol {w} (t)\tag{6.2.23}
$$

$$
\Delta \mathbf {Z} (t) = \mathbf {H} ^ {\mathrm{n}} (t) \Delta \mathbf {X} (t) + \nu (t)\tag{6.2.24}
$$

式中

$$
\begin{array}{r l} \boldsymbol {F} ^ {\mathrm{n}} (t) & = \frac {\partial \boldsymbol {f} [ \boldsymbol {X} (t) , t ]}{\partial \boldsymbol {X} ^ {\mathrm{T}} (t)} \Bigg | _ {\boldsymbol {X} (t) = \boldsymbol {X} ^ {\mathrm{n}} (t)} = \left[ \begin{array}{c c c c} \frac {\partial f _ {1} [ \boldsymbol {X} (t) , t ]}{\partial x _ {1} (t)} & \frac {\partial f _ {1} [ \boldsymbol {X} (t) , t ]}{\partial x _ {2} (t)} & \dots & \frac {\partial f _ {1} [ \boldsymbol {X} (t) , t ]}{\partial x _ {n} (t)} \\ \frac {\partial f _ {2} [ \boldsymbol {X} (t) , t ]}{\partial x _ {1} (t)} & \frac {\partial f _ {2} [ \boldsymbol {X} (t) , t ]}{\partial x _ {2} (t)} & \dots & \frac {\partial f _ {2} [ \boldsymbol {X} (t) , t ]}{\partial x _ {n} (t)} \\ \vdots & \vdots & & \vdots \\ \frac {\partial f _ {n} [ \boldsymbol {X} (t) , t ]}{\partial x _ {1} (t)} & \frac {\partial f _ {n} [ \boldsymbol {X} (t) , t ]}{\partial x _ {2} (t)} & \dots & \frac {\partial f _ {n} [ \boldsymbol {X} (t) , t ]}{\partial x _ {n} (t)} \end{array} \right] _ {\boldsymbol {X} (t) = \boldsymbol {X} ^ {\mathrm{n}} (t)} \\ \boldsymbol {H} ^ {\mathrm{n}} (t) & = \frac {\partial \boldsymbol {h} [ \boldsymbol {X} (t) , t ]}{\partial \boldsymbol {X} ^ {\mathrm{T}} (t)} \Bigg | _ {\boldsymbol {X} (t) = \boldsymbol {X} ^ {\mathrm{n}} (t)} = \end{array}
$$

$$
\left[ \begin{array}{c c c c} \frac {\partial h _ {1} [ \boldsymbol {X} (t) , t ]}{\partial x _ {1} (t)} & \frac {\partial h _ {1} [ \boldsymbol {X} (t) , t ]}{\partial x _ {2} (t)} & \dots & \frac {\partial h _ {1} [ \boldsymbol {X} (t) , t ]}{\partial x _ {n} (t)} \\ \frac {\partial h _ {2} [ \boldsymbol {X} (t) , t ]}{\partial x _ {1} (t)} & \frac {\partial h _ {2} [ \boldsymbol {X} (t) , t ]}{\partial x _ {2} (t)} & \dots & \frac {\partial h _ {2} [ \boldsymbol {X} (t) , t ]}{\partial x _ {n} (t)} \\ \vdots & \vdots & & \vdots \\ \frac {\partial h _ {m} [ \boldsymbol {X} (t) , t ]}{\partial x _ {1} (t)} & \frac {\partial h _ {m} [ \boldsymbol {X} (t) , t ]}{\partial x _ {2} (t)} & \dots & \frac {\partial h _ {m} [ \boldsymbol {X} (t) , t ]}{\partial x _ {n} (t)} \end{array} \right] _ {\boldsymbol {X} (t) = \boldsymbol {X} ^ {n} (t)}
$$

称 $F^{n}(t)$ 和 $H^{n}(t)$ 矩阵为“雅可比矩阵”。式(6.2.23)和式(6.2.24)就是式(6.2.9a)和式(6.2.6)经线性化后得到的“线性干扰方程”。

## 2. 连续型线性化卡尔曼滤波方程

式(6.2.23)和式(6.2.24)的线性干扰方程是一阶线性微分方程,它与第二章介绍的连续型线性系统的模型具有类似的形式。因此,采用第二章连续型卡尔曼滤波基本方程的推导方法,不难导出偏差 $\Delta\mathbf{X}(t)$ 的卡尔曼滤波方程

$$
\Delta \dot {\hat {X}} (t) = F ^ {n} (t) \Delta \hat {X} (t) + K (t) [ \Delta Z (t) - H ^ {n} (t) \Delta \hat {X} (t) ]\tag{6.2.25}
$$

$$
\boldsymbol {K} (t) = \boldsymbol {P} (t) \boldsymbol {H} ^ {\mathrm{nT}} (t) \boldsymbol {r} ^ {- 1} (t)\tag{6.2.26}
$$

$$
\dot {\boldsymbol {P}} (t) = \boldsymbol {P} (t) \boldsymbol {F} ^ {\mathrm{nT}} (t) + \boldsymbol {F} ^ {\mathrm{n}} (t) \boldsymbol {P} (t) - \boldsymbol {P} (t) \boldsymbol {H} ^ {\mathrm{nT}} (t) \boldsymbol {r} ^ {- 1} (t) \boldsymbol {H} ^ {\mathrm{n}} (t) \boldsymbol {P} (t) +
$$

$$
\mathbf {G} (t) \mathbf {q} (t) \mathbf {G} ^ {\mathrm{T}} (t)\tag{6.2.27}
$$

有了状态偏差的最优滤波值 $\Delta\hat{X}(t)$ 后，状态本身的最优滤波值 $\hat{X}(t)$ 就直接下由式得出

$$
\hat {\boldsymbol {X}} (t) = \boldsymbol {X} ^ {\mathrm{n}} (t) + \Delta \hat {\boldsymbol {X}} (t)\tag{6.2.28}
$$

式中状态标称值 $X^n (t)$ 就是非线性系统的状态微分方程

$$
\dot {\boldsymbol {X}} ^ {\mathrm{n}} (t) = \boldsymbol {f} [ \boldsymbol {X} ^ {\mathrm{n}} (t), t ] + \boldsymbol {B} (t) \boldsymbol {u} (t)\tag{6.2.29}
$$

在初始值为 $X^{n}(0)$ 时的解。

式(6.2.25)～式(6.2.29)所组成的方程组称为“连续型线性化卡尔曼滤波方程组”。为了清楚起见，将这些方程归纳如下：

$$
\left\{ \begin{array}{l} \Delta \dot {\hat {X}} (t) = F ^ {n} (t) \Delta \hat {X} (t) + K (t) [ \Delta Z (t) - H ^ {n} (t) \Delta \hat {X} (t) ] \\ K (t) = P (t) H ^ {n T} (t) r ^ {- 1} (t) \\ \dot {P} (t) = P (t) F ^ {n T} (t) + F ^ {n} (t) P (t) - P (t) H ^ {n T} (t) r ^ {- 1} (t) H ^ {n} (t) P (t) + G (t) q (t) G ^ {T} (t) \\ \hat {X} (t) = X ^ {n} (t) + \Delta \hat {X} (t) \\ \dot {X} ^ {n} (t) = f [ X ^ {n} (t), t ] + B (t) u (t) \end{array} \right.
$$

初始值

$$
\Delta \hat {\boldsymbol {X}} (0) = \boldsymbol {0}, \boldsymbol {P} (0) = \boldsymbol {C} _ {\boldsymbol {X} (0)}, \hat {\boldsymbol {X}} (0) = \boldsymbol {X} ^ {\mathrm{n}} (0) = \boldsymbol {m} _ {\boldsymbol {X} (0)}
$$

## 3. 离散型线性化卡尔曼滤波方程

推导离散型线性化卡尔曼滤波方程有两条途径,一条是先进行非线性连续系统的离散化,后线性化;另一条是先进行非线性连续系统线性化,后离散化。这两种离散型系统的线性化卡尔曼滤波方程虽在表达形式上是相同的,但在对有些参数矩阵的计算上有明显区别。

（1）先离散后线性化的卡尔曼滤波方程。与连续情况类似，先离散的非线性系统的线性干扰方程同样可以围绕标称状态，把式(6.2.9b)和式(6.2.8)展开成泰勒级数，并且也取一次近似值，得

$$
\left\{ \begin{array}{l} \mathbf {X} _ {k} = f [ \mathbf {X} _ {k - 1} ^ {\mathrm{n}}, k - 1 ] + \frac {\partial f [ \mathbf {X} _ {k - 1} , k - 1 ]}{\partial \mathbf {X} _ {k - 1} ^ {\mathrm{T}}} \Big | _ {\mathbf {x} _ {k - 1} = \mathbf {x} _ {k - 1} ^ {\mathrm{n}}} \cdot \Delta \mathbf {X} _ {k - 1} + \mathbf {T} _ {k - 1} \mathbf {U} _ {k - 1} + \boldsymbol {\Gamma} _ {k - 1} \mathbf {W} _ {k - 1} \\ \mathbf {Z} _ {k} = h [ \mathbf {X} _ {k} ^ {\mathrm{n}}, k ] + \frac {\partial h [ \mathbf {X} _ {k} , k ]}{\partial \mathbf {X} _ {k} ^ {\mathrm{T}}} \Big | _ {\mathbf {x} _ {k} = \mathbf {x} _ {k} ^ {\mathrm{n}}} \cdot \Delta \mathbf {X} _ {k} + \mathbf {V} _ {k} \end{array} \right.
$$

考虑式 $(6.2.13)$ 和式 $(6.2.14)$ 关系后，上式可改写成

$$
\left\{ \begin{array}{l} \mathbf {X} _ {k} = \mathbf {X} _ {k} ^ {n} + \frac {\partial f [ \mathbf {X} _ {k - 1} , k - 1 ]}{\partial \mathbf {X} _ {k - 1} ^ {\mathrm{T}}} \Big | _ {\mathbf {X} _ {k - 1} = \mathbf {X} _ {k - 1} ^ {\mathrm{n}}} \Delta \mathbf {X} _ {k - 1} + \boldsymbol {\Gamma} _ {k - 1} \mathbf {W} _ {k - 1} \\ \mathbf {Z} _ {k} = \mathbf {Z} _ {k} ^ {\mathrm{n}} + \frac {\partial h [ \mathbf {X} _ {k} , k ]}{\partial \mathbf {X} _ {k} ^ {\mathrm{T}}} \Big | _ {\mathbf {X} _ {k} = \mathbf {X} _ {k} ^ {n}} \Delta \mathbf {X} _ {k} + \mathbf {V} _ {k} \end{array} \right.
$$

将式 $(6.2.17)$ 和式 $(6.2.18)$ 代入上式，即可求得离散型非线性系统的线性摄动方程。

$$
\Delta \boldsymbol {X} _ {k} = \boldsymbol {\Phi} _ {k, k - 1} \Delta \boldsymbol {X} _ {k - 1} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {W} _ {k - 1}\tag{6.2.30}
$$

$$
\Delta \mathbf {Z} _ {k} = \mathbf {H} _ {k} \Delta \mathbf {X} _ {k} + \mathbf {V} _ {k}\tag{6.2.31}
$$

式中 $\Phi_{k,k-1}$ 称为离散型状态偏差转移矩阵(雅可比矩阵)，且有

$$
\boldsymbol {\Phi} _ {k, k - 1} = \frac {\partial f [ \boldsymbol {X} _ {k - 1} , k - 1 ]}{\partial \boldsymbol {X} _ {k - 1} ^ {\mathrm{T}}} \Bigg | _ {\boldsymbol {X} _ {k - 1} = \boldsymbol {X} _ {k - 1} ^ {\mathrm{n}}}\tag{6.2.32}
$$

类似第二章离散型卡尔曼滤波基本方程的推导方法,不难导出偏差 $\Delta X$ 的卡尔曼滤波方程如下:

$$
\Delta \hat {\mathbf {X}} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \Delta \hat {\mathbf {X}} _ {k - 1}\tag{6.2.33}
$$

$$
\boldsymbol {P} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {P} _ {k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {Q} _ {k - 1} \boldsymbol {\Gamma} _ {k - 1} ^ {\mathrm{T}}\tag{6.2.34}
$$

$$
\boldsymbol {K} _ {k} = \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}) ^ {- 1}\tag{6.2.35}
$$

$$
\Delta \hat {\mathbf {X}} _ {k} = \Delta \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {K} _ {k} (\Delta \mathbf {Z} _ {k} - \mathbf {H} _ {k} \Delta \hat {\mathbf {X}} _ {k / k - 1})\tag{6.2.36}
$$

$$
\boldsymbol {P} _ {k} = (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) \boldsymbol {P} _ {k / k - 1} (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) ^ {\mathrm{T}} + \boldsymbol {K} _ {k} \boldsymbol {R} _ {k} \boldsymbol {K} _ {k} ^ {\mathrm{T}}\tag{6.2.37}
$$

于是求得的状态最优滤波值为

$$
\hat {\boldsymbol {X}} _ {k} = \boldsymbol {X} _ {k} ^ {\mathrm{n}} + \Delta \hat {\boldsymbol {X}} _ {k}\tag{6.2.38}
$$

上式中标称状态 $X_{k}^{n}$ 可由式(6.2.13)求得。因原非线性系统是连续型，则 $X_{k}^{n}$ 就是连续型非线性微分方程

$$
\dot {\boldsymbol {X}} ^ {\mathrm{n}} (t) = \boldsymbol {f} [ \boldsymbol {X} ^ {\mathrm{n}} (t), t ] + \boldsymbol {B} (t) \boldsymbol {u} (t)
$$

在初始值为 $X_{k-1}^{n}$ 时的数值解。如果采样周期 T 很短，则可用“欧拉法”求得其解为

$$
\boldsymbol {X} _ {k} ^ {\mathrm{n}} = \boldsymbol {X} _ {k - 1} ^ {\mathrm{n}} + \left\{\boldsymbol {f} \left[ \boldsymbol {X} ^ {\mathrm{n}} \left(t _ {k - 1}\right), t _ {k - 1} \right] + \boldsymbol {B} \left(t _ {k - 1}\right) \boldsymbol {u} \left(t _ {k - 1}\right) \right\} T\tag{6.2.39}
$$

很显然,求解式(6.2.39)比求解式(6.2.13)要方便得多。式(6.2.33)～式(6.2.39)和式(6.2.13)所组成的方程组称为“离散型线性化卡尔曼滤波方程组”。为了清楚起见,归纳如下:

$$
\begin{array}{r l} & {\Delta \hat {X} _ {k / k - 1} = \Phi_ {k, k - 1} \Delta \hat {X} _ {k - 1}} \\ & {P _ {k / k - 1} = \Phi_ {k, k - 1} P _ {k - 1} \Phi_ {k, k - 1} ^ {\mathrm{T}} + \Gamma_ {k - 1} Q _ {k - 1} \Gamma_ {k - 1} ^ {\mathrm{T}}} \\ & {K _ {k} = P _ {k / k - 1} H _ {k} ^ {\mathrm{T}} (H _ {k} P _ {k / k - 1} H _ {k} ^ {\mathrm{T}} + R _ {k}) ^ {- 1}} \\ & {\Delta \hat {X} _ {k} = \Delta \hat {X} _ {k / k - 1} + K _ {k} (\Delta Z _ {k} - H _ {k} \Delta \hat {X} _ {k / k - 1})} \\ & {P _ {k} = (I - K _ {k} H _ {k}) P _ {k / k - 1} (I - K _ {k} H _ {k}) ^ {\mathrm{T}} + K _ {k} R _ {k} K _ {k} ^ {\mathrm{T}}} \\ & {\hat {X} _ {k} = X _ {k} ^ {\mathrm{n}} + \Delta \hat {X} _ {k}} \\ & {X _ {k} ^ {\mathrm{n}} = f [ X _ {k - 1} ^ {\mathrm{n}}, k - 1 ] + T _ {k - 1} U _ {k - 1}} \end{array}
$$

或

$$
\boldsymbol {X} _ {k} ^ {\mathrm{n}} = \boldsymbol {X} _ {k - 1} ^ {\mathrm{n}} + \left\{\boldsymbol {f} \left[ \boldsymbol {X} ^ {\mathrm{n}} \left(t _ {k - 1}\right), t _ {k - 1} \right] + \boldsymbol {B} \left(t _ {k - 1}\right) \boldsymbol {u} \left(t _ {k - 1}\right) \right\} T
$$

初始值

$$
\Delta \hat {X} _ {0} = 0, P _ {0} = C _ {x _ {0}}, \hat {X} _ {0} = X _ {0} ^ {\mathrm{n}} = m _ {x _ {0}}
$$

式中

$$
\begin{array}{l} \boldsymbol {\Phi} _ {k, k - 1} = \frac {\partial f [ \boldsymbol {X} _ {k - 1} , k - 1 ]}{\partial \boldsymbol {X} _ {k - 1} ^ {\mathrm{T}}} \Bigg | _ {\boldsymbol {x} _ {k - 1} = \boldsymbol {x} _ {k - 1} ^ {\mathrm{n}}} = \\ \left[ \begin{array}{c c c c} \frac {\partial f _ {1} [ \boldsymbol {X} _ {k - 1} , k - 1 ]}{\partial x _ {1 , k - 1}} & \frac {\partial f _ {1} [ \boldsymbol {X} _ {k - 1} , k - 1 ]}{\partial x _ {2 , k - 1}} & \dots & \frac {\partial f _ {1} [ \boldsymbol {X} _ {k - 1} , k - 1 ]}{\partial x _ {n , k - 1}} \\ \frac {\partial f _ {2} [ \boldsymbol {X} _ {k - 1} , k - 1 ]}{\partial x _ {1 , k - 1}} & \frac {\partial f _ {2} [ \boldsymbol {X} _ {k - 1} , k - 1 ]}{\partial x _ {2 , k - 1}} & \dots & \frac {\partial f _ {2} [ \boldsymbol {X} _ {k - 1} , k - 1 ]}{\partial x _ {n , k - 1}} \\ \vdots & \vdots & & \vdots \\ \frac {\partial f _ {n} [ \boldsymbol {X} _ {k - 1} , k - 1 ]}{\partial x _ {1 , k - 1}} & \frac {\partial f _ {n} [ \boldsymbol {X} _ {k - 1} , k - 1 ]}{\partial x _ {2 , k - 1}} & \dots & \frac {\partial f _ {n} [ \boldsymbol {X} _ {k - 1} , k - 1 ]}{\partial x _ {n , k - 1}} \end{array} \right] _ {\boldsymbol {x} _ {k - 1} = \boldsymbol {x} _ {k - 1} ^ {\mathrm{n}}} \\ \boldsymbol {H} _ {k} = \frac {\partial h [ \boldsymbol {X} _ {k} , k ]}{\partial \boldsymbol {X} _ {k} ^ {\mathrm{T}}} \Bigg | _ {\boldsymbol {x} _ {k} = \boldsymbol {x} _ {k} ^ {\mathrm{n}}} = \left[ \begin{array}{c c c c} \frac {\partial h _ {1} [ \boldsymbol {X} _ {k} , k ]}{\partial x _ {1 , k}} & \frac {\partial h _ {1} [ \boldsymbol {X} _ {k} , k ]}{\partial x _ {2 , k}} & \dots & \frac {\partial h _ {1} [ \boldsymbol {X} _ {k} , k ]}{\partial x _ {n , k}} \\ \frac {\partial h _ {2} [ \boldsymbol {X} _ {k} , k ]}{\partial x _ {1 , k}} & \frac {\partial h _ {2} [ \boldsymbol {X} _ {k} , k ]}{\partial x _ {2 , k}} & \dots & \frac {\partial h _ {2} [ \boldsymbol {X} _ {k} , k ]}{\partial x _ {n , k}} \\ \vdots & \vdots & & \vdots \\ \frac {\partial h _ {m} [ \boldsymbol {X} _ {k} , k ]}{\partial x _ {1 , k}} & \frac {\partial h _ {m} [ \boldsymbol {X} _ {k} , k ]}{\partial x _ {2 , k}} & \dots & \frac {\partial h _ {m} [ \boldsymbol {X} _ {k} , k ]}{\partial x _ {n , k}} \end{array} \right] _ {\boldsymbol {x} _ {k} = \boldsymbol {x} _ {k} ^ {\mathrm{n}}} \end{array}
$$

式(6.2.39)中的 $X_{k}^{n}$ 也可用一般离散化式(6.2.13)求得。

（2）先线性化后离散的卡尔曼滤波方程。在计算离散后的各系数矩阵时，先线性化后离散的卡尔曼滤波方程的推导比先离散后线性化的卡尔曼滤波方程的推导显得更方便。先线性化后离散，实际上只要对非线性连续系统的线性摄动方程：

$$
\begin{array}{l} \Delta \dot {\mathbf {X}} (t) = \mathbf {F} ^ {\mathrm{n}} (t) \Delta \mathbf {X} (t) + \mathbf {G} (t) \mathbf {w} (t) \\ \Delta \mathbf {Z} (t) = \mathbf {H} ^ {\mathrm{n}} (t) \Delta \mathbf {X} (t) + \mathbf {v} (t) \end{array}
$$

进行离散化就行了。利用基本解阵离散化方法，不难求得离散型线性摄动方程为

$$
\begin{array}{l} \Delta X _ {k} = \Phi_ {k, k - 1} ^ {\mathrm{n}} \Delta X _ {k - 1} + \Gamma_ {k - 1} ^ {\mathrm{n}} W _ {k - 1} \\ \Delta Z _ {k} = H _ {k} ^ {\mathrm{n}} \Delta X _ {k} + V _ {k} \end{array}\tag{6.2.40}
$$

(6.2.41)

式中 $\Phi_{k,k-1}^{n}$ 为线性摄动转移矩阵。当采样周期 T 较小时， $\Phi_{k,k-1}^{n}$ 可用一次近似式表示，即

$$
\Phi_ {k, k - 1} ^ {\mathrm{n}} \approx I + F ^ {\mathrm{n}} (t _ {k - 1}) \cdot T =
$$

$$
\boldsymbol {I} + T \cdot \left[ \begin{array}{c c c c} \frac {\partial f _ {1} [ \boldsymbol {X} (t _ {k - 1}) , t _ {k - 1} ]}{\partial x _ {1} (t _ {k - 1})} & \frac {\partial f _ {1} [ \boldsymbol {X} (t _ {k - 1}) , t _ {k - 1} ]}{\partial x _ {2} (t _ {k - 1})} & \dots & \frac {\partial f _ {1} [ \boldsymbol {X} (t _ {k - 1}) , t _ {k - 1} ]}{\partial x _ {n} (t _ {k - 1})} \\ \frac {\partial f _ {2} [ \boldsymbol {X} (t _ {k - 1}) , t _ {k - 1} ]}{\partial x _ {1} (t _ {k - 1})} & \frac {\partial f _ {2} [ \boldsymbol {X} (t _ {k - 1}) , t _ {k - 1} ]}{\partial x _ {2} (t _ {k - 1})} & \dots & \frac {\partial f _ {2} [ \boldsymbol {X} (t _ {k - 1}) , t _ {k - 1} ]}{\partial x _ {n} (t _ {k - 1})} \\ \vdots & \vdots & & \vdots \\ \frac {\partial f _ {n} [ \boldsymbol {X} (t _ {k - 1}) , t _ {k - 1} ]}{\partial x _ {1} (t _ {n - 1})} & \frac {\partial f _ {n} [ \boldsymbol {X} (t _ {k - 1}) , t _ {k - 1} ]}{\partial x _ {2} (t _ {k - 1})} & \dots & \frac {\partial f _ {n} [ \boldsymbol {X} (t _ {k - 1}) , t _ {k - 1} ]}{\partial x _ {n} (t _ {k - 1})} \end{array} \right] _ {\boldsymbol {X} (t _ {k - 1}) = \boldsymbol {X} ^ {\mathrm{n}} (t _ {k - 1})}
$$

量测阵为

$$
\begin{array}{r l} & {\pmb {H} _ {k} ^ {\mathrm{n}} = \frac {\partial \pmb {h} [ \pmb {X} (t _ {k}) , t _ {k} ]}{\partial \pmb {X} ^ {\mathrm{T}}} \Big | _ {\pmb {X} (t _ {k}) = \pmb {X} ^ {\mathrm{n}} (t _ {k})} =} \\ & {\left[ \begin{array}{c c c c} \frac {\partial h _ {1} [ \pmb {X} (t _ {k}) , t _ {k} ]}{\partial x _ {1} (t _ {k})} & \frac {\partial h _ {1} [ \pmb {X} (t _ {k}) , t _ {k} ]}{\partial x _ {2} (t _ {k})} & \dots & \frac {\partial h _ {1} [ \pmb {X} (t _ {k}) , t _ {k} ]}{\partial x _ {n} (t _ {k})} \\ \frac {\partial h _ {2} [ \pmb {X} (t _ {k}) , t _ {k} ]}{\partial x _ {1} (t _ {k})} & \frac {\partial h _ {2} [ \pmb {X} (t _ {k}) , t _ {k} ]}{\partial x _ {2} (t _ {k})} & \dots & \frac {\partial h _ {2} [ \pmb {X} (t _ {k}) , t _ {k} ]}{\partial x _ {n} (t _ {k})} \\ \vdots & \vdots & & \vdots \\ \frac {\partial h _ {m} [ \pmb {X} (t _ {k}) , t _ {k} ]}{\partial x _ {1} (t _ {k})} & \frac {\partial h _ {m} [ \pmb {X} (t _ {k}) , t _ {k} ]}{\partial x _ {2} (t _ {k})} & \dots & \frac {\partial h _ {m} [ \pmb {X} (t _ {k}) , t _ {k} ]}{\partial x _ {n} (t _ {k})} \end{array} \right] _ {\pmb {X} (t _ {k}) = \pmb {X} ^ {\mathrm{n}} (t _ {k})}} \end{array}
$$

$\Gamma_{k-1}^{n}$ 为系数矩阵，当 $W_{k-1}$ 的方差阵不变时，可以单独计算，即

$$
\boldsymbol {\Gamma} _ {k - 1} ^ {\mathrm{n}} = T \left[ \boldsymbol {I} + \frac {1}{2 !} \boldsymbol {F} ^ {\mathrm{n}} (t _ {k - 1}) T + \frac {1}{3 !} (\boldsymbol {F} ^ {\mathrm{n}} (t _ {k - 1}) T) ^ {2} + \dots \right] \boldsymbol {G} (t _ {k - 1})\tag{6.2.42}
$$

应用第二章推导离散型线性卡尔曼滤波方程的方法,很容易导出离散型线性摄动偏差 $\Delta X$ 的卡尔曼滤波方程如下:

$$
\Delta \hat {X} _ {k / k - 1} = \Phi_ {k, k - 1} ^ {\mathrm{n}} \Delta \hat {X} _ {k - 1}\tag{6.2.43}
$$

$$
\boldsymbol {P} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{n}} \boldsymbol {P} _ {k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{nT}} + \boldsymbol {\Gamma} _ {k - 1} ^ {\mathrm{n}} \boldsymbol {Q} _ {k - 1} \boldsymbol {\Gamma} _ {k - 1} ^ {\mathrm{nT}}\tag{6.2.44}
$$

$$
\mathbf {K} _ {k} = \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{nT}} (\mathbf {H} _ {k} ^ {\mathrm{n}} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{nT}} + \mathbf {R} _ {k}) ^ {- 1}\tag{6.2.45}
$$

$$
\Delta \hat {\mathbf {X}} _ {k} = \Delta \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {K} _ {k} (\Delta \mathbf {Z} _ {k} - \mathbf {H} _ {k} ^ {\mathrm{n}} \Delta \hat {\mathbf {X}} _ {k / k - 1})\tag{6.2.46}
$$

$$
\boldsymbol {P} _ {k} = (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{n}}) \boldsymbol {P} _ {k / k - 1} (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{n}}) ^ {\mathrm{T}} + \boldsymbol {K} _ {k} \boldsymbol {R} _ {k} \boldsymbol {K} _ {k} ^ {\mathrm{T}}\tag{6.2.47}
$$

于是不难求得状态最优滤波值为

$$
\hat {\boldsymbol {X}} _ {k} = \boldsymbol {X} _ {k} ^ {\mathrm{n}} + \Delta \hat {\boldsymbol {X}} _ {k}\tag{6.2.48}
$$

式中标称值 $X_{k}^{n}$ 可用式(6.2.39)计算。很显然，由式(6.2.43)～式(6.2.48)和式(6.2.39)所组成的方程组就是先线性化后离散的卡尔曼滤波方程组，现归纳如下：

$$
\left\{ \begin{array}{l} \Delta \hat {\mathbf {X}} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{n}} \Delta \hat {\mathbf {X}} _ {k - 1} \\ \boldsymbol {P} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{n}} \boldsymbol {P} _ {k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{nT}} + \boldsymbol {\Gamma} _ {k - 1} ^ {\mathrm{n}} \boldsymbol {Q} _ {k - 1} \boldsymbol {\Gamma} _ {k - 1} ^ {\mathrm{nT}} \\ \boldsymbol {K} _ {k} = \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{nT}} (\boldsymbol {H} _ {k} ^ {\mathrm{n}} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{nT}} + \boldsymbol {R} _ {k}) ^ {- 1} \\ \Delta \hat {\mathbf {X}} _ {k} = \Delta \hat {\mathbf {X}} _ {k / k - 1} + \boldsymbol {K} _ {k} (\Delta \boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} ^ {\mathrm{n}} \Delta \hat {\mathbf {X}} _ {k / k - 1}) \\ \boldsymbol {P} _ {k} = (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{n}}) \boldsymbol {P} _ {k / k - 1} (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{n}}) ^ {\mathrm{T}} + \boldsymbol {K} _ {k} \boldsymbol {R} _ {k} \boldsymbol {K} _ {k} ^ {\mathrm{T}} \\ \hat {\mathbf {X}} _ {k} = \boldsymbol {X} _ {k} ^ {\mathrm{n}} + \Delta \hat {\mathbf {X}} _ {k} \\ \boldsymbol {X} _ {k} ^ {\mathrm{n}} = \boldsymbol {X} _ {k - 1} ^ {\mathrm{n}} + \{\boldsymbol {f} [ \boldsymbol {X} ^ {\mathrm{n}} (t _ {k - 1}), t _ {k - 1} ] + \boldsymbol {B} (t _ {k - 1}) \boldsymbol {u} (t _ {k - 1}) \} T \\ \Delta \hat {\mathbf {X}} _ {0} = \mathbf {0}, \quad \boldsymbol {P} _ {0} = \boldsymbol {C} _ {\boldsymbol {X} _ {0}}, \quad \hat {\mathbf {X}} _ {0} = \boldsymbol {X} _ {0} ^ {\mathrm{n}} = m _ {\boldsymbol {X} _ {0}} \end{array} \right.
$$

初始值

4. 对线性化卡尔曼滤波方程的评价

连续型或离散型线性化卡尔曼滤波推导方法均存在着两个严重缺点。其一，因为它是围绕标称状态值 $X^n$ 进行线性化，所以就须要预先知道各个不同时刻的标称状态值。虽然可以预先由标称状态方程式(6.2.11)和式(6.2.13)或式(6.2.39)求出全部数值解，并将它们存放在计算机内，但这样做会占用大量的存储单元，在工程实践中往往难以办到。其二，因为实际系统受各种随机干扰的作用，随着滤波时间的延长，真轨迹与标称轨迹之间的偏差 $\Delta X$ 有可能变得较大，如图6.2.1所示，不能始终保证其“足够小”。因此，破坏了泰勒级数展开式取一次近似的条件。换句话说，线性干扰方程式(6.2.23）和式(6.2.24）或式(6.2.30）和式(6.2.31)不能真实反映实际偏差变化的情况，所以它的最优滤波值 $\Delta \hat{X}(t)$ 或 $\Delta \hat{X}_k$ 也将失去实际意义。因此，在工程实践中很少采用它，而普遍采用“广义卡尔曼滤波”的方法。

![](images/426875b5507d22f0f832df2148bbc9822004ed6152b87084647e7d15d4f26d5d.jpg)  
图 6.2.1 按标称状态线性化的状态轨迹

## 6.2.3 按最优状态估计线性化的卡尔曼滤波方程——广义卡尔曼滤波方程(EKF)

## 1. 围绕最优状态估计的线性化

围绕标称状态 $X_{k}^{n}$ 或 $X^{n}(t)$ 线性化是有严重缺点的，主要在于真轨迹与标称轨迹之间的状态偏差 $\Delta X(t)$ 或 $\Delta X_{k}$ 不能确保其足够小的要求。为此，本节改用另一种近似方法，即采用围绕最优状态估计 $\hat{X}(t)$ 或 $\hat{X}_{k}$ 的线性化方法。现定义真轨迹与标称轨迹间的偏差为

$$
\delta \boldsymbol {X} (t) = \boldsymbol {X} (t) - \hat {\boldsymbol {X}} ^ {\mathrm{n}} (t)\tag{6.2.49}
$$

$$
\delta \mathbf {Z} (t) = \mathbf {Z} (t) - \hat {\mathbf {Z}} ^ {\mathrm{n}} (t)\tag{6.2.50}
$$

或

$$
\delta \boldsymbol {X} _ {k} = \boldsymbol {X} _ {k} - \hat {\boldsymbol {X}} _ {k}\tag{5.2.51}
$$

$$
\delta \mathbf {Z} _ {k} = \mathbf {Z} _ {k} - \hat {\mathbf {Z}} _ {k} ^ {n}\tag{6.2.52}
$$

值得注意的是，式中的标称值 $\hat{\mathbf{X}}^{\mathrm{n}}(t)$ 或 $\hat{\mathbf{X}}_k^{\mathrm{n}}$ 和 $\hat{\mathbf{Z}}^{\mathrm{n}}(t)$ 或 $\hat{\mathbf{Z}}_k^{\mathrm{n}}$ 与式(6.2.11)、式(6.2.12)或式(6.2.13)、式(6.2.14)中的标称值 $X^n (t)$ 或 $X_{k}^{n}$ 和 $Z^n (t)$ 或 $Z_{k}^{n}$ 有着明显的不同。 $\hat{\mathbf{X}}^{\mathrm{n}}(t)$ 或 $\hat{\mathbf{X}}_k^{\mathrm{n}}$ 是非线性系统标称状态微分方程

$$
\hat {\mathbf {X}} ^ {n} (t) = \mathbf {f} [ \hat {\mathbf {X}} ^ {n} (t), t ] + \mathbf {B} (t) \mathbf {u} (t)\tag{6.2.53}
$$

当初始值用初始状态最优估计 $\hat{X}(0)$ 代入时的解，或者初始值用初始状态最优估值 $\hat{X}_{k-1}$ 对式(6.2.53)进行数值求解所得的解。其中 $\hat{X}_k^n$ 就是系统状态 $X_k$ 的一步预测值，即

$$
\hat {\boldsymbol {X}} _ {k / k - 1} = \hat {\boldsymbol {X}} _ {k} ^ {\mathrm{n}}\tag{6.2.54}
$$

当时间从 $t_{k - 1}$ 至 $t$ 变化时，按式(6.2.53）求出的标称值 $\hat{\mathbf{X}}^{\mathrm{n}}(t)$ 轨迹将是一条逐段（按每个采样段）连续的曲线，如图6.2.2所示。由于初始状态最优估计 $\hat{\mathbf{X}}(0)$ 或 $\hat{\mathbf{X}}_{k - 1}$ 比初始标称状态值 $\pmb {X}^{\mathrm{n}}(0)$ 或 $\pmb{X}_{k - 1}^{\mathrm{n}}$ 更接近于真轨迹 $\pmb {X}(t)$ ，所以标称轨迹 $\hat{\mathbf{X}}^{\mathrm{n}}(t)$ 与真轨迹 $\pmb {X}(t)$ 的偏差 $\delta \pmb {X}(t)$ 一般均较原标称轨迹 $\pmb {X}^{\mathrm{n}}(t)$ 与真轨迹 $\pmb {X}(t)$ 的偏差 $\Delta \pmb {X}(t)$ 要小。又由于随时间积累的误差满足偏差 $\delta \pmb {X}(t)$ “足够小”的条件，所以提高了泰勒级数一次近似展开式和系统线性化的精度。 $\hat{\mathbf{Z}}^{\mathrm{n}}(t)$ 或 $\hat{\mathbf{Z}}_k^{\mathrm{n}}$ 也是用状态最优估计 $\hat{\mathbf{X}} (t)$ 或 $\hat{\mathbf{X}}_k$ 计算的标称量测值，其值为

$$
\hat {\mathbf {Z}} ^ {\mathrm{n}} (t) = \mathbf {h} [ \hat {\mathbf {X}} (t), t ]\tag{6.2.55}
$$

或

$$
\hat {\mathbf {Z}} _ {k} ^ {\mathrm{n}} = \mathbf {h} [ \hat {\mathbf {X}} _ {k}, k ]\tag{6.2.56}
$$

![](images/6ef7e965cbd4312ef849bf0109770ced7e8fbc48a5973cd054aca2ade4fba7f3.jpg)  
图 6.2.2 在状态最优估计附近线性化的状态轨迹

式(6.2.56)表明,求 $\hat{Z}_{k}^{n}$ 时须要知道 $\hat{X}_{k}$ ,而求 $\hat{X}_{k}$ 时,却须要知道 $\hat{Z}_{k}^{n}$ ,这显然在递推运算过程中是相互矛盾的。为了解决此矛盾,我们将 $\hat{X}_{k}$ 改用与其十分接近的 $\hat{X}_{k}^{n}$ 值,于是离散型标称量测值可用下面近似式计算

$$
\hat {\mathbf {Z}} _ {k} ^ {\mathrm{n}} = \mathbf {h} [ \hat {\mathbf {X}} _ {k} ^ {\mathrm{n}}, k ]\tag{6.2.57}
$$

也就是

$$
\hat {\mathbf {Z}} _ {k} ^ {\mathrm{n}} = \boldsymbol {h} [ \hat {\mathbf {X}} _ {k / k - 1}, k ]\tag{6.2.58}
$$

因为 $\delta\mathbf{X}(t)$ 足够小, 所以非线性物理系统式(6.2.9) 和式(6.2.6) 在状态最优估计附近展开成泰勒级数, 并取其一次近似值, 得

$$
\begin{array}{r l} \dot {\mathbf {X}} (t) & = f [ \mathbf {X} (t), t ] \left| _ {\mathbf {X} (t) = \hat {\mathbf {X}} (t)} + \frac {\partial f [ \mathbf {X} (t) , t ]}{\partial \mathbf {X} ^ {\mathrm{T}} (t)} \right| _ {\mathbf {X} (t) = \hat {\mathbf {X}} (t)} \cdot [ \mathbf {X} (t) - \hat {\mathbf {X}} (t) ] + \\ \mathbf {B} (t) \mathbf {u} (t) + \mathbf {G} (t) \mathbf {w} (t) & \\ \mathbf {Z} (t) & = h [ \mathbf {X} (t), t ] \left| _ {\mathbf {X} (t) = \hat {\mathbf {X}} (t)} + \frac {\partial h [ \mathbf {X} (t) , t ]}{\partial \mathbf {X} ^ {\mathrm{T}} (t)} \right| _ {\mathbf {X} (t) = \hat {\mathbf {X}} (t)} \cdot [ \mathbf {X} (t) - \hat {\mathbf {X}} (t) ] + v (t) \end{array} \tag {6.2}\tag{6.2.59}
$$

(6.2.60)

考虑标称状态值：

$$
\hat {\boldsymbol {X}} ^ {\mathrm{n}} (t) = \boldsymbol {f} [ \boldsymbol {X} (t), t ] \mid_ {\boldsymbol {X} (t) = \hat {\boldsymbol {X}} (t)} + \boldsymbol {B} (t) \boldsymbol {u} (t)\tag{6.2.61}
$$

$$
\hat {\mathbf {Z}} ^ {\mathrm{n}} (t) = \boldsymbol {h} [ \boldsymbol {X} (t), t ] \mid_ {\boldsymbol {X} (t) = \hat {\boldsymbol {X}} (t)}\tag{6.2.62}
$$

以及式(6.2.49)和式(6.2.50)，则式(6.2.59)和式(6.2.60)可改写成

$$
\begin{array}{l} \delta \dot {\boldsymbol {X}} (t) = \boldsymbol {F} (t) \delta \boldsymbol {X} (t) + \boldsymbol {G} (t) \boldsymbol {w} (t) \\ \delta \boldsymbol {Z} (t) = \boldsymbol {H} (t) \delta \boldsymbol {X} (t) + \boldsymbol {v} (t) \end{array}\tag{6.2.63}
$$

(6.2.64)

式中

$$
\begin{array}{r l} \boldsymbol {F} (t) = & \frac {\partial \boldsymbol {f} [ \boldsymbol {X} (t) , t ]}{\partial \boldsymbol {X} ^ {\mathrm{T}} (t)} \Bigg | _ {\boldsymbol {X} (t) = \hat {\boldsymbol {X}} (t)} = \\ & \left[ \begin{array}{c c c c} \frac {\partial f _ {1} [ \boldsymbol {X} (t) , t ]}{\partial x _ {1} (t)} & \frac {\partial f _ {1} [ \boldsymbol {X} (t) , t ]}{\partial x _ {2} (t)} & \dots & \frac {\partial f _ {1} [ \boldsymbol {X} (t) , t ]}{\partial x _ {n} (t)} \\ \frac {\partial f _ {2} [ \boldsymbol {X} (t) , t ]}{\partial x _ {1} (t)} & \frac {\partial f _ {2} [ \boldsymbol {X} (t) , t ]}{\partial x _ {2} (t)} & \dots & \frac {\partial f _ {2} [ \boldsymbol {X} (t) , t ]}{\partial x _ {n} (t)} \\ \vdots & \vdots & & \vdots \\ \frac {\partial f _ {n} [ \boldsymbol {X} (t) , t ]}{\partial x _ {1} (t)} & \frac {\partial f _ {n} [ \boldsymbol {X} (t) , t ]}{\partial x _ {2} (t)} & \dots & \frac {\partial f _ {n} [ \boldsymbol {X} (t) , t ]}{\partial x _ {n} (t)} \end{array} \right] _ {\boldsymbol {X} (t) = \hat {\boldsymbol {X}} (t)} \end{array}\tag{6.2.65}
$$

$$
\begin{array}{r l} \boldsymbol {H} (t) = & \frac {\partial \boldsymbol {h} [ \boldsymbol {X} (t) , t ]}{\partial \boldsymbol {X} ^ {\mathrm{T}} (t)} \Bigg | _ {\boldsymbol {X} (t) = \hat {\boldsymbol {X}} (t)} = \\ & \left[ \begin{array}{c c c c} \frac {\partial h _ {1} [ \boldsymbol {X} (t) , t ]}{\partial x _ {1} (t)} & \frac {\partial h _ {1} [ \boldsymbol {X} (t) , t ]}{\partial x _ {2} (t)} & \dots & \frac {\partial h _ {1} [ \boldsymbol {X} (t) , t ]}{\partial x _ {n} (t)} \\ \frac {\partial h _ {2} [ \boldsymbol {X} (t) , t ]}{\partial x _ {1} (t)} & \frac {\partial h _ {2} [ \boldsymbol {X} (t) , t ]}{\partial x _ {2} (t)} & \dots & \frac {\partial h _ {2} [ \boldsymbol {X} (t) , t ]}{\partial x _ {n} (t)} \\ \vdots & \vdots & & \vdots \\ \frac {\partial h _ {m} [ \boldsymbol {X} (t) , t ]}{\partial x _ {1} (t)} & \frac {\partial h _ {m} [ \boldsymbol {X} (t) , t ]}{\partial x _ {2} (t)} & \dots & \frac {\partial h _ {m} [ \boldsymbol {X} (t) , t ]}{\partial x _ {n} (t)} \end{array} \right] _ {\boldsymbol {X} (t) = \hat {\boldsymbol {X}} (t)} \end{array}\tag{6.2.66}
$$

式(6.2.63)和式(6.2.64)就是非线性系统式(6.2.9)和式(6.2.8)的连续型线性干扰方程式。

## 2. 连续型非线性广义卡尔曼滤波方程

对连续型非线性广义卡尔曼滤波方程的推导,可采用一种直接的方法,具体来说,就是将泰勒级数展开式(6.2.59)和式(6.2.60)改写成

$$
\dot {\boldsymbol {X}} (t) = \boldsymbol {F} (t) \boldsymbol {X} (t) + \boldsymbol {u} ^ {*} (t) + \boldsymbol {G} (t) \boldsymbol {w} (t)\tag{6.2.67}
$$

$$
\mathbf {Z} (t) = \mathbf {H} (t) \mathbf {X} (t) + \mathbf {Y} (t) + \mathbf {v} (t)\tag{6.2.68}
$$

式中

$$
\boldsymbol {u} ^ {*} (t) = \boldsymbol {f} [ \hat {\boldsymbol {X}} (t), t ] - \boldsymbol {F} (t) \hat {\boldsymbol {X}} (t) + \boldsymbol {B} (t) \boldsymbol {u} (t)\tag{6.2.69}
$$

$$
\mathbf {Y} (t) = \mathbf {h} [ \hat {\mathbf {X}} (t), t ] - \mathbf {H} (t) \hat {\mathbf {X}} (t)\tag{6.2.70}
$$

很显然,式(6.2.67)和式(6.2.68)与第二章介绍的具有确定性外作用的连续系统模型具有相同形式。因此,也就可以由第二章的连续型卡尔曼滤波一般方程的推导方法求得连续系统的广义卡尔曼滤方程

$$
\dot {\hat {\mathbf {X}}} (t) = \mathbf {F} (t) \hat {\mathbf {X}} (t) + \mathbf {u} ^ {*} (t) + \mathbf {K} (t) [ \mathbf {Z} (t) - \mathbf {Y} (t) - \mathbf {H} (t) \hat {\mathbf {X}} (t) ]\tag{6.2.71}
$$

(6.2.72)

$$
\dot {\boldsymbol {P}} (t) = \boldsymbol {P} (t) \boldsymbol {F} ^ {\mathrm{T}} (t) + \boldsymbol {F} (t) \boldsymbol {P} (t) - \boldsymbol {P} (t) \boldsymbol {H} ^ {\mathrm{T}} (t) \boldsymbol {r} ^ {- 1} (t) \boldsymbol {H} (t) \boldsymbol {P} (t) + \boldsymbol {G} (t) \boldsymbol {q} (t) \boldsymbol {G} ^ {\mathrm{T}} (t)\tag{6.2.73}
$$

如果把式(6.2.65)、式(6.2.66)、式(6.2.69)和式(6.2.70)所表示的 $F(t), H(t), u^{*}(t)$ 和 $Y(t)$ 代入式(6.2.71)～式(6.2.73)，则连续型广义卡尔曼滤波方程为如下形式：

$$
\dot {\hat {X}} (t) = f [ \hat {X} (t), t ] + B (t) u (t) + K (t) \{Z (t) - h [ \hat {X} (t), t ] \}\tag{6.2.74}
$$

$$
\boldsymbol {K} (t) = \boldsymbol {P} (t) \left\{\frac {\partial \boldsymbol {h} [ \boldsymbol {X} (t) , t ]}{\partial \boldsymbol {X} ^ {\mathrm{T}} (t)} \right\} \Bigg | _ {\boldsymbol {X} (t) = \hat {\boldsymbol {X}} (t)} ^ {\mathrm{T}} \cdot \boldsymbol {r} ^ {- 1} (t)\tag{6.2.75}
$$

$$
\begin{array}{r l} \dot {\boldsymbol {P}} (t) & = \frac {\partial \boldsymbol {f} [ \boldsymbol {X} (t) , t ]}{\partial \boldsymbol {X} ^ {\mathrm{T}} (t)} \Bigg | _ {\boldsymbol {X} (t) = \hat {\boldsymbol {X}} (t)} \cdot \boldsymbol {P} (t) + \boldsymbol {P} (t) \left\{\frac {\partial \boldsymbol {f} [ \boldsymbol {X} (t) , t ]}{\partial \boldsymbol {X} ^ {\mathrm{T}} (t)} \right\} \Bigg | _ {\boldsymbol {X} (t) = \hat {\boldsymbol {X}} (t)} ^ {\mathrm{T}} - \\ & \quad \boldsymbol {P} (t) \left\{\frac {\partial \boldsymbol {h} [ \boldsymbol {X} (t) , t ]}{\partial \boldsymbol {X} ^ {\mathrm{T}} (t)} \right\} \Bigg | _ {\boldsymbol {X} (t) = \hat {\boldsymbol {X}} (t)} ^ {\mathrm{T}} \cdot \boldsymbol {r} ^ {- 1} (t) \frac {\partial \boldsymbol {h} [ \boldsymbol {X} (t) , t ]}{\partial \boldsymbol {X} ^ {\mathrm{T}} (t)} \Bigg | _ {\boldsymbol {X} (t) = \hat {\boldsymbol {X}} (t)} \cdot \boldsymbol {P} (t) + \\ & \quad \boldsymbol {G} (t) \boldsymbol {q} (t) \boldsymbol {G} ^ {\mathrm{T}} (t) \end{array}\tag{6.2.76}
$$

初始条件为

$$
\hat {\mathbf {X}} (0) = E [ \mathbf {X} (0) ] = m _ {\mathbf {X} (0)}, \quad \mathbf {P} (0) = \mathbf {C} _ {\mathbf {X} (0)}
$$

3. 离散型非线性广义卡尔曼滤波方程

对先线性化后离散的广义卡尔曼滤波方程的推导,采用间接的方法,也就是由下式:

$$
\hat {\boldsymbol {X}} _ {k} = \hat {\boldsymbol {X}} _ {k} ^ {\mathrm{n}} + \delta \hat {\boldsymbol {X}} _ {k}\tag{6.2.77}
$$

间接求解最优滤波值 $\hat{X}_{k}$ 。具体说，直接采用连续系统线性化方程式（6.2.63）和式（6.2.64），即

$$
\begin{array}{r l} \delta \dot {\boldsymbol {X}} (t) & = \boldsymbol {F} (t) \delta \boldsymbol {X} (t) + \boldsymbol {G} (t) \boldsymbol {w} (t) \\ \delta \boldsymbol {Z} (t) & = \boldsymbol {H} (t) \delta \boldsymbol {X} (t) + \boldsymbol {v} (t) \end{array}
$$

对两式分别进行基本解阵离散化得离散型线性干扰方程为

$$
\begin{array}{r} \delta X _ {k} = \Phi_ {k, k - 1} \delta X _ {k - 1} + W _ {k - 1} \\ \delta Z _ {k} = H _ {k} \delta X _ {k} + V _ {k} \end{array}\tag{6.2.78}
$$

(6.2.79)

$T$ 为小量时

$$
\boldsymbol {I} + T \cdot \left[ \begin{array}{c c c c} \frac {\partial f _ {1} [ \boldsymbol {X} (t _ {k - 1}) , t _ {k - 1} ]}{\partial x _ {1} (t _ {k - 1})} & \frac {\partial f _ {1} [ \boldsymbol {X} (t _ {k - 1}) , t _ {k - 1} ]}{\partial x _ {2} (t _ {k - 1})} & \dots & \frac {\partial f _ {1} [ \boldsymbol {X} (t _ {k - 1}) , t _ {k - 1} ]}{\partial x _ {n} (t _ {k - 1})} \\ \frac {\partial f _ {2} [ \boldsymbol {X} (t _ {k - 1}) , t _ {k - 1} ]}{\partial x _ {1} (t _ {k - 1})} & \frac {\partial f _ {2} [ \boldsymbol {X} (t _ {k - 1}) , t _ {k - 1})}{\partial x _ {2} (t _ {k - 1})} & \dots & \frac {\partial f _ {2} [ \boldsymbol {X} (t _ {k - 1}) , t _ {k - 1} ]}{\partial x _ {n} (t _ {k - 1})} \\ \vdots & \vdots & & \vdots \\ \frac {\partial f _ {n} [ \boldsymbol {X} (t _ {k - 1}) , t _ {k - 1} ]}{\partial x _ {1} (t _ {k - 1})} & \frac {\partial f _ {n} [ \boldsymbol {X} (t _ {k - 1}) , t _ {k - 1} ]}{\partial x _ {2} (t _ {k - 1})} & \dots & \frac {\partial f _ {n} [ \boldsymbol {X} (t _ {k - 1}) , t _ {k - 1} ]}{\partial x _ {n} (t _ {k - 1})} \end{array} \right] _ {\boldsymbol {X} (t _ {k - 1}) = \hat {\boldsymbol {X}} _ {k - 1}}
$$

考虑式 $(6.2.54)$ ，则式 $(6.2.79)$ 中的

$$
\begin{array}{r l} \boldsymbol {H} _ {k} & = \frac {\partial \boldsymbol {h} [ \boldsymbol {X} (t _ {k}) , t _ {k} ]}{\partial \boldsymbol {X} ^ {\mathrm{T}} (t _ {k})} \Bigg | _ {\boldsymbol {X} (t _ {k}) = \hat {\boldsymbol {X}} _ {k} ^ {\mathrm{n}} = \hat {\boldsymbol {X}} _ {k / k - 1}} = \\ & \left[ \begin{array}{c c c c} \frac {\partial h _ {1} [ \boldsymbol {X} (t _ {k}) , t _ {k} ]}{\partial x _ {1} (t _ {k})} & \frac {\partial h _ {1} [ \boldsymbol {X} (t _ {k}) , t _ {k} ]}{\partial x _ {2} (t _ {k})} & \dots & \frac {\partial h _ {1} [ \boldsymbol {X} (t _ {k}) , t _ {k} ]}{\partial x _ {n} (t _ {k})} \\ \frac {\partial h _ {2} [ \boldsymbol {X} (t _ {k}) , t _ {k} ]}{\partial x _ {1} (t _ {k})} & \frac {\partial h _ {2} [ \boldsymbol {X} (t _ {k}) , t _ {k} ]}{\partial x _ {2} (t _ {k})} & \dots & \frac {\partial h _ {2} [ \boldsymbol {X} (t _ {k}) , t _ {k} ]}{\partial x _ {n} (t _ {k})} \\ \vdots & \vdots & & \vdots \\ \frac {\partial h _ {m} [ \boldsymbol {X} (t _ {k}) , t _ {k} ]}{\partial x _ {1} (t _ {k})} & \frac {\partial h _ {m} [ \boldsymbol {X} (t _ {k}) , t _ {k} ]}{\partial x _ {2} (t _ {k})} & \dots & \frac {\partial h _ {m} [ \boldsymbol {X} (t _ {k}) , t _ {k} ]}{\partial x _ {n} (t _ {k})} \end{array} \right] _ {\boldsymbol {X} (t _ {k}) = \hat {\boldsymbol {X}} _ {k / k - 1}} \end{array}
$$

等效白噪声序列 $W_{k}$ 的方差阵 $Q_{k}$ 按第二章中的式(2.2.23)、式(2.2.24)、式(2.2.30) 和式(2.2.31) 计算。

在离散型线性干扰方程式(6.2.78)和式(6.2.79)基础上,仿照线性卡尔曼滤波基本方程,不难导出偏差 $\delta X_{k}$ 的卡尔曼滤波方程

$$
\delta \hat {\boldsymbol {X}} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \delta \hat {\boldsymbol {X}} _ {k - 1}\tag{6.2.80}
$$

$$
\delta \hat {\mathbf {X}} _ {k} = \delta \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {K} _ {k} [ \delta \mathbf {Z} _ {k} - \mathbf {H} _ {k} \delta \hat {\mathbf {X}} _ {k / k - 1} ]\tag{6.2.81}
$$

$$
\boldsymbol {K} _ {k} = \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \left[ \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k} \right] ^ {- 1}\tag{6.2.82}
$$

$$
\boldsymbol {P} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {P} _ {k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} + \boldsymbol {Q} _ {k - 1}\tag{6.2.83}
$$

$$
\boldsymbol {P} _ {k} = (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) \boldsymbol {P} _ {k / k - 1} (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) ^ {\mathrm{T}} + \boldsymbol {K} _ {k} \boldsymbol {R} _ {k} \boldsymbol {K} _ {k} ^ {\mathrm{T}}\tag{6.2.84}
$$

式中

$$
\delta \mathbf {Z} _ {k} = \mathbf {Z} _ {k} - \mathbf {h} [ \hat {\mathbf {X}} _ {k} ^ {\mathrm{n}}, k ] = \mathbf {Z} _ {k} - \mathbf {h} [ \hat {\mathbf {X}} _ {k / k - 1}, k ]\tag{6.2.85}
$$

值得注意的是,由于在每次递推计算下一时刻的状态最优估计 $\hat{X}_{k}$ 和标称状态值 $\hat{X}_{k}^{n}$ 时,其初始值均采用状态最优估计的初始值,所以,初始时刻的状态偏差最优估计 $\delta\hat{X}_{k-1}$ 恒等于

零，即

$$
\delta \hat {\boldsymbol {X}} _ {k - 1} = \hat {\boldsymbol {X}} _ {k - 1} - \hat {\boldsymbol {X}} _ {k - 1} ^ {\mathrm{n}} = \mathbf {0}\tag{6.2.86}
$$

从而使状态偏差的一步预测值

$$
\delta \hat {X} _ {k / k - 1} = 0\tag{6.2.87}
$$

将式(6.2.87)代入式(6.2.81)，求得离散型非线性广义卡尔曼滤波方程

$$
\hat {\boldsymbol {X}} _ {k / k - 1} = \hat {\boldsymbol {X}} _ {k} ^ {\mathrm{n}} = \hat {\boldsymbol {X}} _ {k - 1} + \left\{\boldsymbol {f} \left[ \hat {\boldsymbol {X}} _ {k - 1}, t _ {k - 1} \right] + \boldsymbol {B} \left(t _ {k - 1}\right) \boldsymbol {u} \left(t _ {k - 1}\right) \right\} T\tag{6.2.88}
$$

$$
\hat {\mathbf {X}} _ {k} = \hat {\mathbf {X}} _ {k / k - 1} + \delta \hat {\mathbf {X}} _ {k}\tag{6.2.89}
$$

$$
\delta \hat {\boldsymbol {X}} _ {k} = \boldsymbol {K} _ {k} \left\{\boldsymbol {Z} _ {k} - \boldsymbol {h} \left[ \hat {\boldsymbol {X}} _ {k / k - 1}, k \right] \right\}\tag{6.2.90}
$$

$$
\mathbf {K} _ {k} = \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \left[ \mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} + \mathbf {R} _ {k} \right] ^ {- 1}\tag{6.2.91}
$$

$$
\boldsymbol {P} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {P} _ {k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} + \boldsymbol {Q} _ {k - 1}\tag{6.2.92}
$$

$$
\boldsymbol {P} _ {k} = (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) \boldsymbol {P} _ {k / k - 1} (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) ^ {\mathrm{T}} + \boldsymbol {K} _ {k} \boldsymbol {R} _ {k} \boldsymbol {K} _ {k} ^ {\mathrm{T}}\tag{6.2.93}
$$

初始条件

$$
\hat {\boldsymbol {X}} _ {0} = E \left[ \boldsymbol {X} _ {0} \right] = m _ {\boldsymbol {X} _ {0}}, \quad \boldsymbol {P} _ {0} = \boldsymbol {C} _ {\boldsymbol {X} _ {0}}
$$

式 $(6.2.88)$ ～式 $(6.2.93)$ 的计算流程图如图6.2.3所示。

![](images/c36e9d16ee9fe828539c9ccebbfc325685d57def6e3e4f9b342f38e18f649b9a.jpg)  
图 6.2.3 离散型广义卡尔曼滤波方程计算流程图

应该指出，以上几种非线性滤波方法实际上都不是最优的，而是近似最优，故常称这种滤波器为“次优卡尔曼滤波器”。其中“广义卡尔曼滤波器”又称“推广卡尔曼滤波器”，在工程实践中较为普遍地得到应用。

最后，必须注意，先线性化后离散所出现的 $\frac{\partial f[\mathbf{X}(t_{k - 1}), t_{k - 1}]}{\partial\mathbf{X}^{\mathrm{T}}(t_{k - 1})}$ 与先离散后线性化出现的 $\frac{\partial f[\mathbf{X}_{k - 1}, k - 1]}{\partial\mathbf{X}_{k - 1}^{\mathrm{T}}}$ 有本质的区别，决非一回事。

如果系统方程和量测方程已经具有离散化形式：

$$
\begin{array}{r l} \mathbf {X} _ {k + 1} & = f [ \mathbf {X} _ {k}, k ] + \mathbf {B} _ {k} \mathbf {U} _ {k} + \boldsymbol {\Gamma} _ {k} \mathbf {W} _ {k} \\ \mathbf {Z} _ {k + 1} & = h [ \mathbf {X} _ {k + 1}, k + 1 ] + \mathbf {V} _ {k + 1} \end{array}
$$

其中系统噪声和量测噪声均为零均值的白色噪声，且互不相关，则滤波方程推导如下。

假设在 $k + 1$ 时刻已获 $\hat{\mathbf{X}}_k$ ，则对 $f[\mathbf{X}_k,k]$ 和 $\pmb {h}[\mathbf{X}_{k + 1},k + 1]$ 作线性化时，泰勒级数的展开点分别选取 $\hat{\mathbf{X}}_k$ 和 $\hat{\mathbf{X}}_{k + 1 / k}$ ，即

$$
\boldsymbol {X} _ {k + 1} = \boldsymbol {f} [ \hat {\boldsymbol {X}} _ {k}, k ] + \frac {\partial \boldsymbol {f} [ \boldsymbol {X} _ {k} , k ]}{\partial \boldsymbol {X} _ {k} ^ {\mathrm{T}}} \Big | _ {\boldsymbol {X} _ {k} = \hat {\boldsymbol {X}} _ {k}} (\boldsymbol {X} _ {k} - \hat {\boldsymbol {X}} _ {k}) + \boldsymbol {B} _ {k} \boldsymbol {U} _ {k} + \boldsymbol {\Gamma} _ {k} \boldsymbol {W} _ {k}\tag{6.2.94}
$$

$$
\mathbf {Z} _ {k + 1} = h \left[ \hat {\mathbf {X}} _ {k + 1 / k}, k + 1 \right] + \frac {\partial h \left[ \mathbf {X} _ {k + 1} , k + 1 \right]}{\partial \mathbf {X} _ {k + 1} ^ {\mathrm{T}}} \Bigg | _ {\mathbf {X} _ {k + 1} = \hat {\mathbf {X}} _ {k + 1 / k}} \left(\mathbf {X} _ {k + 1} - \hat {\mathbf {X}} _ {k + 1 / k}\right) + \mathbf {V} _ {k + 1}\tag{6.2.95}
$$

记

$$
\boldsymbol {\Phi} _ {k + 1, k} = \frac {\partial f [ \boldsymbol {X} _ {k} , k ]}{\partial \boldsymbol {X} _ {k} ^ {\mathrm{T}}} \Big | _ {\boldsymbol {X} _ {k} = \hat {\boldsymbol {X}} _ {k}}\tag{6.2.96}
$$

$$
\boldsymbol {H} _ {k + 1} = \frac {\partial \boldsymbol {h} [ \boldsymbol {X} _ {k + 1} , k + 1 ]}{\partial \boldsymbol {X} _ {k + 1} ^ {\mathrm{T}}} \Bigg | _ {\boldsymbol {X} _ {k + 1} = \hat {\boldsymbol {x}} _ {k + 1 / k}}\tag{6.2.97}
$$

$$
\boldsymbol {U} _ {k} ^ {*} = \boldsymbol {f} [ \hat {\boldsymbol {X}} _ {k}, k ] + \boldsymbol {B} _ {k} \boldsymbol {U} _ {k} - \frac {\partial \boldsymbol {f} [ \boldsymbol {X} _ {k} , k ]}{\partial \boldsymbol {X} _ {k} ^ {\mathrm{T}}} \Bigg | _ {\boldsymbol {x} _ {k} = \hat {\boldsymbol {x}} _ {k}} \cdot \hat {\boldsymbol {X}} _ {k}
$$

$$
\mathbf {Y} _ {k + 1} = \boldsymbol {h} [ \hat {\mathbf {X}} _ {k + 1 / k}, k + 1 ] - \frac {\partial \boldsymbol {h} [ \mathbf {X} _ {k + 1} , k + 1 ]}{\partial \mathbf {X} _ {k + 1} ^ {\mathrm{T}}} \Big | _ {\mathbf {X} _ {k + 1} = \hat {\mathbf {X}} _ {k + 1 / k}} \cdot \hat {\mathbf {X}} _ {k + 1 / k}
$$

则式 $(6.2.94)$ 和式 $(6.2.95)$ 可写成：

$$
\begin{array}{l} \mathbf {X} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} \mathbf {X} _ {k} + \mathbf {U} _ {k} ^ {*} + \boldsymbol {\Gamma} _ {k} \mathbf {W} _ {k} \\ \mathbf {Z} _ {k + 1} = \mathbf {H} _ {k + 1} \mathbf {X} _ {k + 1} + \mathbf {Y} _ {k + 1} + \mathbf {V} _ {k + 1} \end{array}
$$

根据式(2.2.47)～式(2.2.51)，并考虑到 $S_{k}=0$ ，则有

$$
\hat {\mathbf {X}} _ {k + 1 / k} = \boldsymbol {\Phi} _ {k + 1, k} \hat {\mathbf {X}} _ {k} + \mathbf {U} _ {k} ^ {*} = \boldsymbol {\Phi} _ {k + 1, k} \hat {\mathbf {X}} _ {k} + f [ \hat {\mathbf {X}} _ {k}, k ] + \mathbf {B} _ {k} \mathbf {U} _ {k} - \frac {\partial f [ \mathbf {X} _ {k} , k ]}{\partial \mathbf {X} _ {k} ^ {\mathrm{T}}} \Bigg | _ {\mathbf {X} _ {k} = \hat {\mathbf {X}} _ {k}} \cdot \hat {\mathbf {X}} _ {k} =
$$

$$
f [ \hat {X} _ {k}, k ] + B _ {k} U _ {k}\tag{6.2.98a}
$$

$$
\hat {\mathbf {X}} _ {k + 1} = \hat {\mathbf {X}} _ {k + 1 / k} + \mathbf {K} _ {k + 1} \left[ \mathbf {Z} _ {k + 1} - \mathbf {Y} _ {k + 1} - \mathbf {H} _ {k + 1} \right. \hat {\mathbf {X}} _ {k + 1 / k} ] =
$$

$$
\hat {\mathbf {X}} _ {k + 1 / k} + \mathbf {K} _ {k + 1} \left\{\mathbf {Z} _ {k + 1} - \mathbf {h} [ \hat {\mathbf {X}} _ {k + 1 / k}, k + 1 ] + \frac {\partial \mathbf {h} [ \mathbf {X} _ {k + 1} , k + 1 ]}{\partial \mathbf {X} _ {k + 1} ^ {\mathrm{T}}} \right| _ {\mathbf {X} _ {k + 1} = \hat {\mathbf {x}} _ {k + 1 / k}} \hat {\mathbf {X}} _ {k + 1 / k} - \mathbf {H} _ {k + 1} \hat {\mathbf {X}} _ {k + 1 / k} \Bigg \} =
$$

$$
\hat {\mathbf {X}} _ {k + 1 / k} + \mathbf {K} _ {k + 1} \left\{\mathbf {Z} _ {k + 1} - \mathbf {h} [ \hat {\mathbf {X}} _ {k + 1 / k}, k + 1 ] \right\}\tag{6.2.98b}
$$

$$
\boldsymbol {P} _ {k + 1 / k} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + \boldsymbol {\Gamma} _ {k} \boldsymbol {Q} _ {k} \boldsymbol {\Gamma} _ {k} ^ {\mathrm{T}}\tag{6.2.98c}
$$

$$
\boldsymbol {K} _ {k + 1} = \boldsymbol {P} _ {k + 1 / k} \boldsymbol {H} _ {k + 1} (\boldsymbol {H} _ {k + 1} \boldsymbol {P} _ {k + 1 / k} \boldsymbol {H} _ {k + 1} ^ {\mathrm{T}} + \boldsymbol {R} _ {k + 1}) ^ {- 1}\tag{6.2.98d}
$$

$$
\boldsymbol {P} _ {k + 1} = \left(\boldsymbol {I} - \boldsymbol {K} _ {k + 1} \boldsymbol {H} _ {k + 1}\right) \boldsymbol {P} _ {k + 1 / k} \left(\boldsymbol {I} - \boldsymbol {K} _ {k + 1} \boldsymbol {H} _ {k + 1}\right) ^ {\mathrm{T}} + \boldsymbol {K} _ {k + 1} \boldsymbol {R} _ {k + 1} \boldsymbol {K} _ {k + 1} ^ {\mathrm{T}}\tag{6.2.98e}
$$

其中 $\Phi_{k+1,k}$ 和 $H_{k+1}$ 分别按式(6.2.96)和式(6.2.97)确定。