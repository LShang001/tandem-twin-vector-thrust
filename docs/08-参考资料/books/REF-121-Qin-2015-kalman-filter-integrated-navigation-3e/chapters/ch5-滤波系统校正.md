# 第5章 滤波系统的校正

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

