# 第2章 几种最优估计和卡尔曼滤波基本方程

# 第二章 几种最优估计和卡尔曼滤波基本方程

## 2.1 几种最优估计

所谓估计就是根据测量得出的与状态 $X(t)$ 有关的数据 $Z(t)=h[X(t)]+V(t)$ 解算出 $X(t)$ 的计算值 $\hat{X}(t)$ ，其中随机向量 $V(t)$ 为量测误差， $\hat{X}$ 称为 X 的估计，Z 称为 X 的量测。因为 $\hat{X}(t)$ 是根据 $Z(t)$ 确定的，所以 $\hat{X}(t)$ 是 $Z(t)$ 的函数。若 $\hat{X}$ 是 Z 的线性函数，则 $\hat{X}$ 称作 X 的线性估计。

设在 $\left[t_{0},t_{1}\right]$ 时间段内的量测为Z，相应的估计为 $\hat{X}(t)$ ，则

当 $t = t_{1}$ 时， $\hat{\pmb{X}} (t)$ 称为 $\pmb {X}(t)$ 的估计；

当 $t > t_1$ 时， $\hat{\pmb{X}}(t)$ 称为 $\pmb {X}(t)$ 的预测；

当 $t < t_1$ 时， $\hat{\pmb{X}}(t)$ 称为 $\pmb {X}(t)$ 的平滑。

最优估计是指某一指标函数达到最值时的估计。

若以量测估计 $\hat{Z}$ 的偏差的平方和达到最小为指标，即

$$
(\mathbf {Z} - \hat {\mathbf {Z}}) ^ {\mathrm{T}} (\mathbf {Z} - \hat {\mathbf {Z}}) = \min
$$

则所得估计 $\hat{X}$ 为 X 的最小二乘估计。

若以状态估计 $\hat{X}$ 的均方误差集平均达到最小为指标, 即

$$
E \left[ (\boldsymbol {X} - \hat {\boldsymbol {X}}) ^ {\mathrm{T}} (\boldsymbol {X} - \hat {\boldsymbol {X}}) \right] = \min
$$

则所得估计 $\hat{X}$ 为X的最小方差估计；若 $\hat{X}$ 又是X的线性估计，则 $\hat{X}$ 为X的线性最小方差估计。

也可用估计值出现的概率作为估计指标,这样的估计有极大验后估计、贝叶斯估计和极大似然估计。本节将对上述各种估计作详细介绍。

## 2.1.1 最小二乘估计

1. 一般最小二乘估计

最小二乘估计是高斯(Karl Gauss)在1795年为测定行星轨道而提出的参数估计算法。这种估计的特点是算法简单，不必知道与被估计量及量测量有关的任何统计信息。

设 X 为某一确定性常值向量, 维数为 n 。一般情况下对 X 不能直接测量, 而只能测量到 X 各分量的线性组合。记第 i 次量测 $Z_{i}$ 为

$$
\mathbf {Z} _ {i} = \mathbf {H} _ {i} \mathbf {X} + \mathbf {V} _ {i}\tag{2.1.1}
$$

式中 $Z_{i}$ 为 $m_{i}$ 维向量， $H_{i}$ 和 $V_{i}$ 为第 i 次测量的量测矩阵和随机量测噪声。

若共测量 r 次, 即

$$
\left. \begin{array}{c} \mathbf {Z} _ {1} = \mathbf {H} _ {1} \mathbf {X} + \mathbf {V} _ {1} \\ \mathbf {Z} _ {2} = \mathbf {H} _ {2} \mathbf {X} + \mathbf {V} _ {2} \\ \dots \dots \\ \mathbf {Z} _ {r} = \mathbf {H} _ {r} \mathbf {X} + \mathbf {V} _ {r} \end{array} \right\}\tag{2.1.2}
$$

则由上述诸式可得描述 $r$ 次量测的量测方程

$$
Z = H X + V\tag{2.1.3}
$$

式中， $Z,V$ 为 $\sum_{i = 1}^{r}m_{i} = m$ 维向量， $\pmb{H}$ 为 $m\times n$ 矩阵。

最小二乘估计的指标是:使各次量测 $Z_{i}$ 与由估计 $\hat{X}$ 确定的量测的估计 $\hat{Z}_{i}=H_{i}\hat{X}$ 之差的平方和最小,即

$$
J (\hat {\mathbf {X}}) = (\mathbf {Z} - \mathbf {H} \hat {\mathbf {X}}) ^ {\mathrm{T}} (\mathbf {Z} - \mathbf {H} \hat {\mathbf {X}}) = \min\tag{2.1.4}
$$

而要使上式达到最小,需满足(详细推导见附录 A)

$$
\left. \frac {\partial J}{\partial \boldsymbol {X}} \right| _ {\boldsymbol {X} = \hat {\boldsymbol {X}}} = - 2 \boldsymbol {H} ^ {\mathrm{T}} (\boldsymbol {Z} - \boldsymbol {H} \hat {\boldsymbol {X}}) = \boldsymbol {0}
$$

若 $\pmb{H}$ 具有最大秩 $n$ ，即 $\pmb{H}^{\mathrm{T}}\pmb{H}$ 正定，且 $m = \sum_{i=1}^{r} m_{i} > n$ ，则 $\pmb{X}$ 的最小二乘估计为

$$
\hat {\mathbf {X}} = (\mathbf {H} ^ {\mathrm{T}} \mathbf {H}) ^ {- 1} \mathbf {H} ^ {\mathrm{T}} \mathbf {Z}\tag{2.1.5}
$$

从上式可看出最小二乘估计是一种线性估计。

为了说明最小二乘估计最优的含义,将式(2.1.4)改写成

$$
\begin{array}{r l} J (\hat {\mathbf {X}}) & = \left[ (\mathbf {Z} _ {1} - \mathbf {H} _ {1} \hat {\mathbf {X}}) ^ {\mathrm{T}} (\mathbf {Z} _ {2} - \mathbf {H} _ {2} \hat {\mathbf {X}}) ^ {\mathrm{T}} \dots (\mathbf {Z} _ {r} - \mathbf {H} _ {r} \hat {\mathbf {X}}) ^ {\mathrm{T}} \right] \left[ \begin{array}{c} \mathbf {Z} _ {1} - \mathbf {H} _ {1} \hat {\mathbf {X}} \\ \mathbf {Z} _ {2} - \mathbf {H} _ {2} \hat {\mathbf {X}} \\ \vdots \\ \mathbf {Z} _ {r} - \mathbf {H} _ {r} \hat {\mathbf {X}} \end{array} \right] = \\ & \sum_ {i = 1} ^ {r} (\mathbf {Z} _ {i} - \mathbf {H} _ {i} \hat {\mathbf {X}}) ^ {\mathrm{T}} (\mathbf {Z} _ {i} - \mathbf {H} _ {i} \hat {\mathbf {X}}) = \min \end{array}
$$

这说明,最小二乘估计虽然不能满足式(2.1.2)中的每一个方程,既使每个方程都有偏差,但它使所有方程偏差的平方和达到最小。这实际上兼顾了所有方程的近似程度,使整体误差达到最小,这对抑制测量误差 $V_{1},V_{2},\cdots,V_{r}$ 的影响是有益的。

最小二乘估计具有如下性质：

若量测噪声 V 是均值为零、方差为 R 的随机向量，则

(1) 最小二乘估计是无偏估计, 即

$$
E [ \hat {X} ] = X\tag{2.1.6}
$$

或

$$
E [ \widetilde {X} ] = 0\tag{2.1.7}
$$

式中 $\hat{X} = X - \hat{X}$ 为 $\hat{X}$ 的估计误差。

(2) 最小二乘估计的均方误差阵为

$$
E [ \widetilde {\boldsymbol {X}} \widetilde {\boldsymbol {X}} ^ {\mathrm{T}} ] = (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {H}) ^ {- 1} \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {R} \boldsymbol {H} (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {H}) ^ {- 1}\tag{2.1.8}
$$

证明

① 由式(2.1.5)， $\hat{X}$ 的估计误差为

$$
\tilde {\boldsymbol {X}} = \boldsymbol {X} - \hat {\boldsymbol {X}} = \boldsymbol {X} - (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {H}) ^ {- 1} \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {Z}
$$

由于 $E[\mathbf{V}] = \mathbf{0}$ ，所以

$$
\begin{array}{r l} E [ \widetilde {\mathbf {X}} ] & = E [ (\mathbf {H} ^ {\mathrm{T}} \mathbf {H}) ^ {- 1} (\mathbf {H} ^ {\mathrm{T}} \mathbf {H}) \mathbf {X} - (\mathbf {H} ^ {\mathrm{T}} \mathbf {H}) ^ {- 1} \mathbf {H} ^ {\mathrm{T}} \mathbf {Z} ] = (\mathbf {H} ^ {\mathrm{T}} \mathbf {H}) ^ {- 1} \mathbf {H} ^ {\mathrm{T}} E [ \mathbf {H} \mathbf {X} - \mathbf {Z}) ] = \\ & - (\mathbf {H} ^ {\mathrm{T}} \mathbf {H}) ^ {- 1} \mathbf {H} ^ {\mathrm{T}} E [ \mathbf {V} ] = \mathbf {0} \end{array}
$$

$$
② E [ \widetilde {\mathbf {X}} \widetilde {\mathbf {X}} ^ {\mathrm{T}} ] = (\mathbf {H} ^ {\mathrm{T}} \mathbf {H}) ^ {- 1} \mathbf {H} ^ {\mathrm{T}} E [ \mathbf {V V} ^ {\mathrm{T}} ] \mathbf {H} (\mathbf {H} ^ {\mathrm{T}} \mathbf {H}) ^ {- 1} = (\mathbf {H} ^ {\mathrm{T}} \mathbf {H}) ^ {- 1} \mathbf {H} ^ {\mathrm{T}} \mathbf {R H} (\mathbf {H} ^ {\mathrm{T}} \mathbf {H}) ^ {- 1}
$$

例 2-1 用一台仪器对未知确定性标量 X 作 r 次直接测量，量测值分别为 $Z_{1}, Z_{2}, \cdots, Z_{r}$ ，测量误差的均值为零，方差为 R，求 X 的最小二乘估计 $\hat{X}$ ，并计算估计的均方误差。

解 由题意, r 次直接测量所得的量测方程为

$$
\mathbf {Z} = \mathbf {H} X + \mathbf {V}
$$

式中

$$
\mathbf {Z} = \left[ \begin{array}{l l l l} Z _ {1} & Z _ {2} & \dots & Z _ {r} \end{array} \right] ^ {\mathrm{T}}, \mathbf {H} = \left[ \begin{array}{l l l l} 1 & 1 & \dots & 1 \end{array} \right] ^ {\mathrm{T}}, E [ \mathbf {V V} ^ {\mathrm{T}} ] = R \mathbf {I}
$$

根据式 $(2.1.5)$ ，得

$$
\hat {X} = \frac {1}{r} (Z _ {1} + Z _ {2} + \dots + Z _ {r})
$$

估计的均方误差为

$$
E [ \widetilde {X} ^ {2} ] = \frac {R}{r}
$$

例2-2 设有线性系统满足

$$
\boldsymbol {X} _ {k} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {X} _ {k - 1}
$$

对 $X_{k}$ 的量测为标量, 满足

$$
Z _ {k} = \boldsymbol {h} _ {k} \boldsymbol {X} _ {k} + V _ {k}
$$

式中， $X_{k}$ 为 n 维向量； $V_{k}$ 为随机误差；k 为时刻 $t_{k}$ 。求 $X_{0}$ 基于 $Z_{1}, Z_{2}, \cdots, Z_{m} (m \geqslant n)$ 的最小二乘估计 $\hat{X}_{0/m}$ 。

解 $X_{0}$ 的第 i 次量测为

$$
Z _ {i} = \boldsymbol {h} _ {i} \boldsymbol {X} _ {i} + V _ {i} = \boldsymbol {h} _ {i} \boldsymbol {\Phi} _ {i, 0} \boldsymbol {X} _ {0} + V _ {i} \quad i = 1, 2, \dots , m
$$

$X_0$ 的前 $m$ 次量测为

$$
\mathbf {Z} _ {m} = \mathbf {H} _ {m, 0} \mathbf {X} _ {0} + \mathbf {V} _ {m}
$$

式中

$$
\mathbf {Z} _ {m} = \left[ \begin{array}{c} Z _ {1} \\ Z _ {2} \\ \vdots \\ Z _ {m} \end{array} \right], \mathbf {H} _ {m, 0} = \left[ \begin{array}{c} h _ {1} \boldsymbol {\Phi} _ {1, 0} \\ h _ {2} \boldsymbol {\Phi} _ {2, 0} \\ \vdots \\ h _ {m} \boldsymbol {\Phi} _ {m, 0} \end{array} \right], \mathbf {V} _ {m} = \left[ \begin{array}{c} V _ {1} \\ V _ {2} \\ \vdots \\ V _ {m} \end{array} \right]
$$

根据式(2.1.5)，得 $X_{0}$ 基于 $Z_{1}, Z_{2}, \cdots, Z_{m}$ 的最小二乘估计为

$$
\hat {\mathbf {X}} _ {0 / m} = (\mathbf {H} _ {m, 0} ^ {\mathrm{T}} \mathbf {H} _ {m, 0}) ^ {- 1} \mathbf {H} _ {m, 0} ^ {\mathrm{T}} \mathbf {Z} _ {m}
$$

例2-3 用两台仪器对未知标量 $X$ 各直接测量一次，量测量分别为 $Z_{1}$ 和 $Z_{2}$ ，仪器的测量误差是均值为零，方差分别为 $r$ 和 $4r$ 的随机量，求 $X$ 的最小二乘估计，并计算估计的均方误差。

解 由题意得量测方程

$$
Z = H X + V
$$

式中

$$
\mathbf {Z} = \left[ \begin{array}{l} Z _ {1} \\ Z _ {2} \end{array} \right],   \mathbf {H} = \left[ \begin{array}{l} 1 \\ 1 \end{array} \right],   \mathbf {R} = \left[ \begin{array}{l l} r & 0 \\ 0 & 4 r \end{array} \right]
$$

根据式 $(2.1.5)$ 和式 $(2.1.8)$ ，得

$$
\hat {X} = \frac {1}{2} (Z _ {1} + Z _ {2})
$$

$$
E [ \hat {X} ^ {2} ] = \frac {1}{2} [ 1 1 ] \left[ \begin{array}{l l} r & 0 \\ 0 & 4 r \end{array} \right] \left[ \begin{array}{l} 1 \\ 1 \end{array} \right] \frac {1}{2} = \frac {5}{4} r > r
$$

上式说明,使用精度差一倍的两台仪器同时进行测量,最小二乘估计效果还不如只使用一台精度高的仪器时好。

## 2. 加权最小二乘估计

从例2-3看出，一般最小二乘估计精度不高的原因之一是不分优劣地使用了量测值。如果对不同量测值的质量有所了解，则可用加权的办法分别对待各量测量，精度质量高的权重取得大些，精度质量差的权重取得小些。这就是加权最小二乘估计的思路。加权最小二乘估计 $\hat{X}$ 的求取准则是

$$
J (\hat {\boldsymbol {X}}) = (\boldsymbol {Z} - \boldsymbol {H} \hat {\boldsymbol {X}}) ^ {\mathrm{T}} \boldsymbol {W} (\boldsymbol {Z} - \boldsymbol {H} \hat {\boldsymbol {X}}) = \min\tag{2.1.9}
$$

式中 W 是适当取值的正定加权矩阵。当 W=I 时，式(2.1.9) 就是一般最小二乘准则。

要使式(2.1.9)成立, $\hat{X}$ 应满足

$$
\left. \frac {\partial J (\boldsymbol {X})}{\partial \boldsymbol {X}} \right| _ {\boldsymbol {X} = \hat {\boldsymbol {X}}} = - \boldsymbol {H} ^ {\mathrm{T}} (\boldsymbol {W} + \boldsymbol {W} ^ {\mathrm{T}}) (\boldsymbol {Z} - \boldsymbol {H} \hat {\boldsymbol {X}}) = \mathbf {0}
$$

从中解得

$$
\hat {\mathbf {X}} = \left[ \mathbf {H} ^ {\mathrm{T}} (\mathbf {W} + \mathbf {W} ^ {\mathrm{T}}) \mathbf {H} \right] ^ {- 1} \mathbf {H} ^ {\mathrm{T}} (\mathbf {W} + \mathbf {W} ^ {\mathrm{T}}) \mathbf {Z}
$$

一般情况下,加权阵取成对称阵,即 $W^{T}=W$ ,所以加权最小二乘估计为

$$
\hat {\boldsymbol {X}} = (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {H}) ^ {- 1} \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {Z}\tag{2.1.10}
$$

估计误差为

$$
\begin{array}{r l} \tilde {\boldsymbol {X}} = \boldsymbol {X} - \hat {\boldsymbol {X}} & = (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {H}) ^ {- 1} \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {H} \boldsymbol {X} - (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {H}) ^ {- 1} \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {Z} = \\ & (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {H}) ^ {- 1} \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} (\boldsymbol {H} \boldsymbol {X} - \boldsymbol {Z}) = \\ & - (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {H}) ^ {- 1} \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {V} \end{array}
$$

如果量测误差 V 的均值为零, 方差阵为 R, 则加权最小二乘估计也是无偏估计, 估计的均方误差为

$$
E \left[ \tilde {\boldsymbol {X}} \tilde {\boldsymbol {X}} ^ {\mathrm{T}} \right] = \left(\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {H}\right) ^ {- 1} \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {R} \boldsymbol {W} \boldsymbol {H} \left(\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {H}\right) ^ {- 1}\tag{2.1.11}
$$

如果 $W = R^{-1}$ ，则加权最小二乘估计

$$
\hat {\mathbf {X}} = (\mathbf {H} ^ {\mathrm{T}} \mathbf {R} ^ {- 1} \mathbf {H}) ^ {- 1} \mathbf {H} ^ {\mathrm{T}} \mathbf {R} ^ {- 1} \mathbf {Z}\tag{2.1.12}
$$

又称为马尔柯夫估计。

马尔柯夫估计的均方误差为

$$
E [ \widetilde {\boldsymbol {X}} \widetilde {\boldsymbol {X}} ^ {\mathrm{T}} ] = (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {R} ^ {- 1} \boldsymbol {H}) ^ {- 1}\tag{2.1.13}
$$

马尔柯夫估计的均方误差比任何其他加权最小二乘估计的均方误差都要小,所以是加权最小二乘估计中的最优者。下面证明这一结论。

设 A 和 B 分别为 $n \times m$ 和 $m \times l$ 的矩阵，且 $AA^{T}$ 满秩，则因

$$
\left[ \boldsymbol {B} - \boldsymbol {A} ^ {\mathrm{T}} (\boldsymbol {A A} ^ {\mathrm{T}}) ^ {- 1} \boldsymbol {A B} \right] ^ {\mathrm{T}} \left[ \boldsymbol {B} - \boldsymbol {A} ^ {\mathrm{T}} (\boldsymbol {A A} ^ {\mathrm{T}}) ^ {- 1} \boldsymbol {A B} \right] =
$$

$$
\boldsymbol {B} ^ {\mathrm{T}} \boldsymbol {B} - 2 \boldsymbol {B} ^ {\mathrm{T}} \boldsymbol {A} ^ {\mathrm{T}} (\boldsymbol {A A} ^ {\mathrm{T}}) ^ {- 1} \boldsymbol {A B} + \boldsymbol {B} ^ {\mathrm{T}} \boldsymbol {A} ^ {\mathrm{T}} (\boldsymbol {A A} ^ {\mathrm{T}}) ^ {- 1} \boldsymbol {A A} ^ {\mathrm{T}} (\boldsymbol {A A} ^ {\mathrm{T}}) ^ {- 1} \boldsymbol {A B} =
$$

$$
\boldsymbol {B} ^ {\mathrm{T}} \boldsymbol {B} - (\boldsymbol {A B}) ^ {\mathrm{T}} (\boldsymbol {A A} ^ {\mathrm{T}}) ^ {- 1} (\boldsymbol {A B}) \geqslant \boldsymbol {0}
$$

所以有

$$
\boldsymbol {B} ^ {\mathrm{T}} \boldsymbol {B} \geqslant (\boldsymbol {A B}) ^ {\mathrm{T}} (\boldsymbol {A A} ^ {\mathrm{T}}) ^ {- 1} (\boldsymbol {A B})\tag{2.1.14}
$$

这就是矩阵型的许瓦茨不等式。

由矩阵理论知,正定阵 R 可表示成 $R = S^{T}S$ , 其中 S 为满秩矩阵。令

$$
\mathbf {A} = \mathbf {H} ^ {\mathrm{T}} \mathbf {S} ^ {- 1}\tag{2.1.15a}
$$

$$
\boldsymbol {B} = \boldsymbol {S W H} (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W H}) ^ {- 1}\tag{2.1.15b}
$$

则

$$
\boldsymbol {A} \boldsymbol {B} = \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {S} ^ {- 1} \boldsymbol {S} \boldsymbol {W} \boldsymbol {H} (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {H}) ^ {- 1} = \boldsymbol {I}\tag{2.1.16}
$$

$$
\boldsymbol {B} ^ {\mathrm{T}} \boldsymbol {B} = (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {H}) ^ {- 1} \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {R} \boldsymbol {W} \boldsymbol {H} (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {H}) ^ {- 1}\tag{2.1.17}
$$

根据许瓦茨不等式和式(2.1.16)及式(2.1.15a)，有

$$
\boldsymbol {B} ^ {\mathrm{T}} \boldsymbol {B} \geqslant (\boldsymbol {A B}) ^ {\mathrm{T}} (\boldsymbol {A A} ^ {\mathrm{T}}) ^ {- 1} (\boldsymbol {A B}) = (\boldsymbol {A A} ^ {\mathrm{T}}) ^ {- 1} = (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {R} ^ {- 1} \boldsymbol {H}) ^ {- 1}
$$

式 $(2.1.17)$ 代入上式,得

$$
(\mathbf {H} ^ {\mathrm{T}} \mathbf {W} \mathbf {H}) ^ {- 1} \mathbf {H} ^ {\mathrm{T}} \mathbf {W} \mathbf {R} \mathbf {W} \mathbf {H} (\mathbf {H} ^ {\mathrm{T}} \mathbf {W} \mathbf {H}) ^ {- 1} \geqslant (\mathbf {H} ^ {\mathrm{T}} \mathbf {R} ^ {- 1} \mathbf {H}) ^ {- 1}\tag{2.1.18}
$$

式(2.1.18)说明，只有当 $W = R^{-1}$ 时，估计的均方误差才达到最小，最小值为 $(H^{T}R^{-1}H)^{-1}$ 。

例2-4 对例2-3采用马尔柯夫估计，并求估计的均方误差。

解 取

$$
\boldsymbol {W} = \boldsymbol {R} ^ {- 1} = \left[ \begin{array}{l l} \frac {1}{r} & 0 \\ 0 & \frac {1}{4 r} \end{array} \right]
$$

$$
\hat {X} = ([ 1 1 ] \left[ \begin{array}{l l} \frac {1}{r} & 0 \\ 0 & \frac {1}{4 r} \end{array} \right] [ \begin{array}{l} 1 \\ 1 \end{array} ]) ^ {- 1} [ 1 1 ] \left[ \begin{array}{l l} \frac {1}{r} & 0 \\ 0 & \frac {1}{4 r} \end{array} \right] [ \begin{array}{l} Z _ {1} \\ Z _ {2} \end{array} ] = \frac {4}{5} Z _ {1} + \frac {1}{5} Z _ {2}
$$

$$
E [ \widetilde {X} \widetilde {X} ] = ([ 1 1 ] \left[ \begin{array}{l l} \frac {1}{r} & 0 \\ 0 & \frac {1}{4 r} \end{array} \right] \left[ \begin{array}{l} 1 \\ 1 \end{array} \right]) ^ {- 1} = \frac {4}{5} r <   r
$$

这说明,对精度高的量测值取权重系数 $\frac{4}{5}$ ,对精度低的量测值取权重系数 $\frac{1}{5}$ ,估计精度高于仅用精度高的量测值所得估计的精度。所以增加不同的量测值,并根据其精度质量区别对待利用之,能有效提高估计精度。

## 3. 递推最小二乘估计

从上述诸例可看出,量测值越多,只要处理得合适,最小二乘估计的均方误差就越小。采用批处理实现的最小二乘算法,须存储所有的量测值。若量测值数量十分庞大,则计算机必须具备巨大的存储容量,这显然是不经济的。递推最小二乘估计从每次获得的量测值中提取出被估计量信息,用于修正上一步所得的估计。获得量测的次数越多,修正的次数也越多,估计的精度也越高。下面详细介绍该算法。

设 X 为确定性常值向量, 前 k 次观测积累的量测为 $\bar{Z}_{k}$ , 量测方程为

$$
\overline {{{{\bf Z}}}} _ {k} = \overline {{{{\bf H}}}} _ {k} {\bf X} + \overline {{{{\bf V}}}} _ {k}
$$

式中

$$
\overline {{{\mathbf {Z}}}} _ {k} = \left[ \begin{array}{c} \mathbf {Z} _ {1} \\ \mathbf {Z} _ {2} \\ \vdots \\ \mathbf {Z} _ {k} \end{array} \right], \overline {{{\mathbf {H}}}} _ {k} = \left[ \begin{array}{c} \mathbf {H} _ {1} \\ \mathbf {H} _ {2} \\ \vdots \\ \mathbf {H} _ {k} \end{array} \right], \overline {{{\mathbf {V}}}} _ {k} = \left[ \begin{array}{c} \mathbf {V} _ {1} \\ \mathbf {V} _ {2} \\ \vdots \\ \mathbf {V} _ {k} \end{array} \right]
$$

$Z_{i}$ 为第 i 次量测,量测方程为

$$
\mathbf {Z} _ {i} = \mathbf {H} _ {i} \mathbf {X} + \mathbf {V} _ {i} \quad i = 1, 2, \dots , k
$$

则前 $k + 1$ 次量测为

$$
\overline {{{{\mathbf {Z}}}}} _ {k + 1} = \overline {{{{\mathbf {H}}}}} _ {k + 1} \mathbf {X} + \overline {{{{\mathbf {V}}}}} _ {k + 1}
$$

式中

$$
\overline {{{\mathbf {Z}}}} _ {k + 1} = \left[ \begin{array}{l} \overline {{{\mathbf {Z}}}} _ {k} \\ \mathbf {Z} _ {k + 1} \end{array} \right], \overline {{{\mathbf {H}}}} _ {k + 1} = \left[ \begin{array}{l} \overline {{{\mathbf {H}}}} _ {k} \\ \mathbf {H} _ {k + 1} \end{array} \right], \overline {{{\mathbf {V}}}} _ {k + 1} = \left[ \begin{array}{l} \overline {{{\mathbf {V}}}} _ {k} \\ \mathbf {V} _ {k + 1} \end{array} \right]
$$

$Z_{k+1}$ 为第 $k+1$ 次量测，量测方程为

$$
\mathbf {Z} _ {k + 1} = \mathbf {H} _ {k + 1} \mathbf {X} + \mathbf {V} _ {k + 1}
$$

根据式(2.1.10)，由前 k 次量测确定的加权最小二乘估计为

$$
\hat {\boldsymbol {X}} _ {k} = \left(\overline {{\boldsymbol {H}}} _ {k} ^ {\mathrm{T}} \overline {{\boldsymbol {W}}} _ {k} \overline {{\boldsymbol {H}}} _ {k}\right) ^ {- 1} \overline {{\boldsymbol {H}}} _ {k} ^ {\mathrm{T}} \overline {{\boldsymbol {W}}} _ {k} \overline {{\boldsymbol {Z}}} _ {k}
$$

式中

$$
\overline {{{W}}} _ {k} = \left[ \begin{array}{c c c c} W _ {1} & & & \\ & W _ {2} & & 0 \\ 0 & & \ddots & \\ & & & W _ {k} \end{array} \right]
$$

令

$$
\boldsymbol {P} _ {k} = (\overline {{\boldsymbol {H}}} _ {k} ^ {\mathrm{T}} \overline {{\boldsymbol {W}}} _ {k} \overline {{\boldsymbol {H}}} _ {k}) ^ {- 1}\tag{2.1.19}
$$

则

$$
\hat {\boldsymbol {X}} _ {k} = \boldsymbol {P} _ {k} \overline {{\boldsymbol {H}}} _ {k} ^ {\mathrm{T}} \overline {{\boldsymbol {W}}} _ {k} \overline {{\boldsymbol {Z}}} _ {k}\tag{2.1.20}
$$

由前 $k + 1$ 次量测确定的加权最小二乘估计为

$$
\hat {\boldsymbol {X}} _ {k + 1} = (\overline {{\boldsymbol {H}}} _ {k + 1} ^ {\mathrm{T}} \overline {{\boldsymbol {W}}} _ {k + 1} \overline {{\boldsymbol {H}}} _ {k + 1}) ^ {- 1} \overline {{\boldsymbol {H}}} _ {k + 1} ^ {\mathrm{T}} \overline {{\boldsymbol {W}}} _ {k + 1} \overline {{\boldsymbol {Z}}} _ {k + 1} = \boldsymbol {P} _ {k + 1} [ \overline {{\boldsymbol {H}}} _ {k} ^ {\mathrm{T}} \boldsymbol {H} _ {k + 1} ^ {\mathrm{T}} ] \left[ \begin{array}{c c} \overline {{\boldsymbol {W}}} _ {k} & \boldsymbol {0} \\ \boldsymbol {0} & \boldsymbol {W} _ {k + 1} \end{array} \right] \left[ \begin{array}{c} \overline {{\boldsymbol {Z}}} _ {k} \\ \boldsymbol {Z} _ {k + 1} \end{array} \right] =
$$

$$
\boldsymbol {P} _ {k + 1} \overline {{\boldsymbol {H}}} _ {k} ^ {\mathrm{T}} \overline {{\boldsymbol {W}}} _ {k} \overline {{\boldsymbol {Z}}} _ {k} + \boldsymbol {P} _ {k + 1} \boldsymbol {H} _ {k + 1} ^ {\mathrm{T}} \boldsymbol {W} _ {k + 1} \boldsymbol {Z} _ {k + 1}\tag{2.1.21}
$$

式中

$$
\boldsymbol {P} _ {k + 1} = (\overline {{\boldsymbol {H}}} _ {k + 1} ^ {\mathrm{T}} \overline {{\boldsymbol {W}}} _ {k + 1} \overline {{\boldsymbol {H}}} _ {k + 1}) ^ {- 1} = ([ \overline {{\boldsymbol {H}}} _ {k} ^ {\mathrm{T}} \boldsymbol {H} _ {k + 1} ^ {\mathrm{T}} ] \left[ \begin{array}{c c} \overline {{\boldsymbol {W}}} _ {k} & \boldsymbol {0} \\ \boldsymbol {0} & \boldsymbol {W} _ {k + 1} \end{array} \right] \left[ \begin{array}{c} \overline {{\boldsymbol {H}}} _ {k} \\ \boldsymbol {H} _ {k + 1} \end{array} \right]) ^ {- 1} =
$$

$$
(\overline {{\boldsymbol {H}}} _ {k} ^ {\mathrm{T}} \overline {{\boldsymbol {W}}} _ {k} \overline {{\boldsymbol {H}}} _ {k} + \boldsymbol {H} _ {k + 1} ^ {\mathrm{T}} \boldsymbol {W} _ {k + 1} \boldsymbol {H} _ {k + 1}) ^ {- 1} = (\boldsymbol {P} _ {k} ^ {- 1} + \boldsymbol {H} _ {k + 1} ^ {\mathrm{T}} \boldsymbol {W} _ {k + 1} \boldsymbol {H} _ {k + 1}) ^ {- 1}\tag{2.1.22}
$$

由矩阵反演公式(详细推导见附录 B)

$$
\left(\boldsymbol {A} _ {1 1} - \boldsymbol {A} _ {1 2} \boldsymbol {A} _ {2 2} ^ {- 1} \boldsymbol {A} _ {2 1}\right) ^ {- 1} = \boldsymbol {A} _ {1 1} ^ {- 1} + \boldsymbol {A} _ {1 1} ^ {- 1} \boldsymbol {A} _ {1 2} \left(\boldsymbol {A} _ {2 2} - \boldsymbol {A} _ {2 1} \boldsymbol {A} _ {1 1} ^ {- 1} \boldsymbol {A} _ {1 2}\right) ^ {- 1} \boldsymbol {A} _ {2 1} \boldsymbol {A} _ {1 1} ^ {- 1}\tag{2.1.23}
$$

令 $A_{11}=P_{k}^{-1},A_{12}=-H_{k+1}^{T},A_{22}^{-1}=W_{k+1},A_{21}=H_{k+1}$ ，得

$$
\boldsymbol {P} _ {k + 1} = \boldsymbol {P} _ {k} - \boldsymbol {P} _ {k} \boldsymbol {H} _ {k + 1} ^ {\mathrm{T}} \left(\boldsymbol {W} _ {k + 1} ^ {- 1} + \boldsymbol {H} _ {k + 1} \boldsymbol {P} _ {k} \boldsymbol {H} _ {k + 1} ^ {\mathrm{T}}\right) ^ {- 1} \boldsymbol {H} _ {k + 1} \boldsymbol {P} _ {k}\tag{2.1.24}
$$

再考察式(2.1.21)中的第一项,由式(2.1.20)和式(2.1.22),得

$$
\overline {{{{\boldsymbol {H}}}}} _ {k} ^ {\mathrm{T}} \overline {{{{\boldsymbol {W}}}}} _ {k} \overline {{{{\boldsymbol {Z}}}}} _ {k} = \boldsymbol {P} _ {k} ^ {- 1} \hat {\boldsymbol {X}} _ {k}
$$

$$
\boldsymbol {P} _ {k} ^ {- 1} = \boldsymbol {P} _ {k + 1} ^ {- 1} - \boldsymbol {H} _ {k + 1} ^ {\mathrm{T}} \boldsymbol {W} _ {k + 1} \boldsymbol {H} _ {k + 1}
$$

所以该项为

$$
\begin{array}{r l} & {\pmb {P} _ {k + 1} \overline {{\pmb {H}}} _ {k} ^ {\mathrm{T}} \overline {{\pmb {W}}} _ {k} \overline {{\pmb {Z}}} _ {k} = \pmb {P} _ {k + 1} \pmb {P} _ {k} ^ {- 1} \hat {\pmb {X}} _ {k} = \pmb {P} _ {k + 1} (\pmb {P} _ {k + 1} ^ {- 1} - \pmb {H} _ {k + 1} ^ {\mathrm{T}} \pmb {W} _ {k + 1} \pmb {H} _ {k + 1}) \hat {\pmb {X}} _ {k} =} \\ & {\qquad \hat {\pmb {X}} _ {k} - \pmb {P} _ {k + 1} \pmb {H} _ {k + 1} ^ {\mathrm{T}} \pmb {W} _ {k + 1} \pmb {H} _ {k + 1} \hat {\pmb {X}} _ {k}} \end{array}
$$

因此式 $(2.1.21)$ 成

$$
\begin{array}{r l} \hat {\mathbf {X}} _ {k + 1} & = \hat {\mathbf {X}} _ {k} - \mathbf {P} _ {k + 1} \mathbf {H} _ {k + 1} ^ {\mathrm{T}} \mathbf {W} _ {k + 1} \mathbf {H} _ {k + 1} \hat {\mathbf {X}} _ {k} + \mathbf {P} _ {k + 1} \mathbf {H} _ {k + 1} ^ {\mathrm{T}} \mathbf {W} _ {k + 1} \mathbf {Z} _ {k + 1} = \\ & \quad \hat {\mathbf {X}} _ {k} + \mathbf {P} _ {k + 1} \mathbf {H} _ {k + 1} ^ {\mathrm{T}} \mathbf {W} _ {k + 1} (\mathbf {Z} _ {k + 1} - \mathbf {H} _ {k + 1} \hat {\mathbf {X}} _ {k}) \end{array}\tag{2.1.25}
$$

式(2.1.24)和式(2.1.25)即为递推最小二乘估计的全套算法。

式(2.1.25)说明, $k+1$ 时刻的估计由对k时刻的估计作修正而获得,修正量由对 $k+1$ 时刻的量测的估计误差 $\tilde{Z}_{k+1}=Z_{k+1}-H_{k+1}\hat{X}_{k}$ 经增益阵 $K_{k+1}=P_{k+1}H_{k+1}^{T}W_{k+1}$ 加权后确定,其中 $P_{k+1}$ 由式(2.1.24)确定。

式(2.1.24)和式(2.1.25)确定的算法是递推的,只要给定初始值 $\hat{X}_{0}$ 和 $P_{0}$ ,即可获得X在任意时刻的最小二乘估计。 $\hat{X}_{0}$ 和 $P_{0}$ 的选取可以是任意的,一般可取 $\hat{X}_{0}=0,P_{0}=pI$ ,其中p为很大的正数。由于初值选取盲目,所以递推过程中,刚开始计算时,估计误差跳跃剧烈,随着量测次数的增加,初值影响逐渐消失,估计值逐渐趋于稳定而逼近被估计量。

例2-5 在例2-1中求出 $\hat{X}_k$ 后，试求获得新量测值 $Z_{k + 1}$ 后的递推最小二乘估计。

解 例2-1采用了一般最小二乘算法，这实际上加权阵取作单位阵。根据式(2.1.19)、式(2.1.24)及式(2.1.25)，得

$$
\begin{array}{l} P _ {k} = ([ 1 1 \dots 1 ] \left[ \begin{array}{c} 1 \\ 1 \\ \vdots \\ 1 \end{array} \right]) ^ {- 1} = \frac {1}{k} \\ P _ {k + 1} = \frac {1}{k} - \frac {1}{k} \left(1 + \frac {1}{k}\right) ^ {- 1} \frac {1}{k} = \frac {1}{k + 1} \\ \hat {X} _ {k + 1} = \hat {X} _ {k} + \frac {1}{k + 1} (Z _ {k + 1} - \hat {X} _ {k}) \end{array}
$$

最小二乘估计的最大优点是算法简单,特别是一般最小二乘估计,根本不必知道量测误差的统计信息。但正是这种优点又引起了使用上的局限性,主要体现在如下两点上:

(1) 最小二乘算法只能估计确定性的常值向量,而无法估计随机向量的时间过程。

（2）最小二乘的最优指标只保证了量测的估计均方误差之和最小，而并未确保被估计量的估计误差达到最佳，所以估计精度不高。

## 2.1.2 最小方差估计

1. 最小方差估计与条件均值

设 X 为随机向量, Z 为 X 的量测向量, 即 $\mathbf{Z} = \mathbf{Z}(\mathbf{X}) + \mathbf{V}$ , 求 X 的估计 $\hat{X}$ 就是根据 Z 解算出 X, 显然 $\hat{X}$ 是 Z 的函数, 由于 V 是随机误差, 所以 X 无法从 Z 的函数关系式中直接求取, 而必须按统计意义的最优标准求取。

最小方差估计是使下述指标达到最小的估计 $\hat{X}_{\mathrm{MV}}(Z)$

$$
J = E _ {X, Z} \left\{\left[ X - \hat {X} (Z) \right] ^ {\mathrm{T}} \left[ X - \hat {X} (Z) \right] \right\} | _ {\hat {X} (Z) = \hat {X} _ {\mathrm{MV}} (Z)} = \min\tag{2.1.26}
$$

式中 $X - \hat{X}$ 为估计误差。对于 Z 的某一个实现， $\hat{X}(Z)$ 是与之对应的某一具体样本，所以上式中求均值是对 X 和 Z 同时进行的。

定理2.1

最小方差估计等于量测为某一具体实现条件下的条件均值

$$
\hat {\mathbf {X}} _ {\mathrm{MV}} (\mathbf {Z}) = E [ \mathbf {X} / \mathbf {Z} ]\tag{2.1.27}
$$

下面详细证明这一定理。

证明

$$
\begin{array}{r l} & J = E _ {X, Z} \{\left[ X - \hat {X} _ {\mathrm{MV}} (Z) \right] ^ {\mathrm{T}} [ X - \hat {X} _ {\mathrm{MV}} (Z) ] \} = \\ & \int_ {- \infty} ^ {\infty} \int_ {- \infty} ^ {\infty} [ x - \hat {X} _ {\mathrm{MV}} (z) ] ^ {\mathrm{T}} [ x - \hat {X} _ {\mathrm{MV}} (z) ] p (x, z) \mathrm{d} x \mathrm{d} z \end{array}
$$

式中 $p(x,z)$ 为 X 和 Z 的联合分布密度, 是关于 x 和 z 的标量函数。根据贝叶斯定理:

$$
p (x, z) = p _ {z} (z) \cdot p (x / z)
$$

式中 $p_z(z)$ 为 $\mathbf{Z}$ 的边缘分布， $p(x / z)$ 为 $\mathbf{X}$ 关于 $\mathbf{Z}$ 的条件分布。所以

$$
J = \int_ {- \infty} ^ {\infty} p _ {z} (z) \left\{\int_ {- \infty} ^ {\infty} [ x - \hat {X} _ {\mathrm{MV}} (z) ] ^ {\mathrm{T}} [ x - \hat {X} _ {\mathrm{MV}} (z) ] p (x / z) \mathrm{d} x \right\} \mathrm{d} z\tag{2.1.28}
$$

由于 $\hat{X}_{MV}$ 的选择对 J 的影响只体现在上式的内层积分中，又 $p_{z}(z)$ 恒大于零，所以要使 J 达到最小，只须使上式中的内层积分达到最小即可，即

$$
g = \int_ {- \infty} ^ {\infty} [ \boldsymbol {x} - \hat {\boldsymbol {X}} _ {\mathrm{MV}} (\boldsymbol {z}) ] ^ {\mathrm{T}} [ \boldsymbol {x} - \hat {\boldsymbol {X}} _ {\mathrm{MV}} (\boldsymbol {z}) ] p (\boldsymbol {x} / \boldsymbol {z}) \mathrm{d} \boldsymbol {x} = \min
$$

对 g 作恒等变形, 并注意到: $E[X/z]$ 和 $\hat{X}(z)$ 都是确定量, x 是 X 样本空间内的所有样本, 而 z 是 Z 样本空间内的某一样本, 所以对 x 的积分中, $E[X/z], \hat{X}(z)$ 与 x 无关, 因此

$$
\begin{array}{r l} g & = \int_ {- \infty} ^ {\infty} [ x - E (X / z) + E (X / z) - \hat {X} _ {\mathrm{MV}} (z) ] ^ {\mathrm{T}} \cdot \\ & [ x - E (X / z) + E (X / z) - \hat {X} _ {\mathrm{MV}} (z) ] p (x / z) \mathrm{d} x = \\ & \int_ {- \infty} ^ {\infty} [ x - E (X / z) ] ^ {\mathrm{T}} [ x - E (X / z) ] p (x / z) \mathrm{d} x + \\ & [ E (X / z) - \hat {X} _ {\mathrm{MV}} (z) ] ^ {\mathrm{T}} [ E (X / z) - \hat {X} _ {\mathrm{MV}} (z) ] \int_ {- \infty} ^ {\infty} p (x / z) \mathrm{d} x + \\ & \int_ {- \infty} ^ {\infty} [ x - E (X / z) ] ^ {\mathrm{T}} p (x / z) \mathrm{d} x [ E (X / z) - \hat {X} _ {\mathrm{MV}} (z) ] + \\ & [ E (X / z) - \hat {X} _ {\mathrm{MV}} (z) ] ^ {\mathrm{T}} \int_ {- \infty} ^ {\infty} [ x - E (X / z) ] p (x / z) \mathrm{d} x \end{array}
$$

由于

$$
\begin{array}{r l} & \int_ {- \infty} ^ {\infty} p (x / z) \mathrm{d} x = 1 \\ & \int_ {- \infty} ^ {\infty} [ x - E (X / z) ] p (x / z) \mathrm{d} x = 0 \end{array}
$$

所以

$$
\begin{array}{r l} g = & \int_ {- \infty} ^ {\infty} [ \boldsymbol {x} - E (\boldsymbol {X} / \boldsymbol {z}) ] ^ {\mathrm{T}} [ \boldsymbol {x} - E (\boldsymbol {X} / \boldsymbol {z}) ] p (\boldsymbol {x} / \boldsymbol {z}) \mathrm{d} \boldsymbol {x} + \\ & [ E (\boldsymbol {X} / \boldsymbol {z}) - \hat {\boldsymbol {X}} _ {\mathrm{MV}} (\boldsymbol {z}) ] ^ {\mathrm{T}} [ E (\boldsymbol {X} / \boldsymbol {z}) - \hat {\boldsymbol {X}} _ {\mathrm{MV}} (\boldsymbol {z}) ] \end{array}
$$

上式中，第一项与 $\hat{X}_{\mathrm{MV}}(z)$ 的选取无关，第二项为向量的内积，总是大于或等于零，所以要使 $g$

达到最小,必须使该项内积为零,而要使内积为零,必须使该向量为零,即

$$
\hat {\boldsymbol {X}} _ {\mathrm{MV}} (\boldsymbol {z}) = E [ \boldsymbol {X} / \boldsymbol {z} ]
$$

注意到 z 是式(2.1.28)中的积分变量,积分区间为 Z 的整个样本空间,所以在 Z 的整个样本空间内上式均成立,即

$$
\hat {\mathbf {X}} _ {\mathrm{MV}} (\mathbf {Z}) = E [ \mathbf {X} / \mathbf {Z} ]
$$

2. 最小方差估计的无偏性

定理2.2

最小方差估计是 X 的无偏估计, 即

$$
E [ \mathbf {X} - \hat {\mathbf {X}} _ {\mathrm{MV}} (\mathbf {Z}) ] = \mathbf {0}\tag{2.1.29}
$$

或

$$
E [ \hat {\mathbf {X}} _ {\mathrm{MV}} (\mathbf {Z}) ] = E [ \mathbf {X} ]\tag{2.1.30}
$$

证明

由式(2.1.27)，最小方差估计为 $\hat{\mathbf{X}}_{\mathrm{MV}}(\mathbf{Z}) = E[\mathbf{X} / \mathbf{Z}]$ ，其中求均值是对 $\mathbf{X}$ 求均值，所以 $\hat{\mathbf{X}}_{\mathrm{MV}}(\mathbf{Z})$ 是关于 $\mathbf{Z}$ 的函数，对 $\hat{\mathbf{X}}_{\mathrm{MV}}(\mathbf{Z})$ 求均值实质上是对 $\mathbf{Z}$ 求均值，即

$$
\begin{array}{r l} E [ \hat {\mathbf {X}} _ {\mathrm{MV}} (\mathbf {Z}) ] & = E _ {\mathbf {Z}} [ \hat {\mathbf {X}} _ {\mathrm{MV}} (\mathbf {Z}) ] = E _ {\mathbf {Z}} [ E _ {\mathbf {X}} (\mathbf {X} / \mathbf {Z}) ] = \\ & \int_ {- \infty} ^ {\infty} \left[ \int_ {- \infty} ^ {\infty} x p (\mathbf {x} / \mathbf {z}) \mathrm{d} \mathbf {x} \right] p _ {z} (\mathbf {z}) \mathrm{d} \mathbf {z} \end{array}
$$

应用贝叶斯定理, 得

$$
\begin{array}{r l} E [ \hat {\mathbf {X}} _ {\mathrm{MV}} (\mathbf {Z}) ] & = \int_ {- \infty} ^ {\infty} \int_ {- \infty} ^ {\infty} x p (\mathbf {x}, z) \mathrm{d} z \mathrm{d} \mathbf {x} = \int_ {- \infty} ^ {\infty} x \left[ \int_ {- \infty} ^ {\infty} p (\mathbf {x}, z) \mathrm{d} z \right] \mathrm{d} \mathbf {x} = \\ & \quad \int_ {- \infty} ^ {\infty} x p _ {x} (\mathbf {x}) \mathrm{d} x = E [ \mathbf {X} ] \end{array}
$$

3. 正态随机向量的最小方差估计

定理2.3

若被估计向量 $X_{n\times1}$ 和量测向量 $Z_{m\times1}$ 都服从正态分布，且

$$
\begin{array}{r l} & E [ \mathbf {X} ] = m _ {x}, E [ \mathbf {Z} ] = m _ {z} \\ & \mathrm{Cov} [ \mathbf {X}, \mathbf {Z} ] = E [ (\mathbf {X} - m _ {x}) (\mathbf {Z} - m _ {z}) ^ {\mathrm{T}} ] = C _ {x z} \\ & \mathrm{Var} [ \mathbf {Z} ] = E [ (\mathbf {Z} - m _ {z}) (\mathbf {Z} - m _ {z}) ^ {\mathrm{T}} ] = C _ {z} \end{array}
$$

则 $X$ 的最小方差估计为

$$
\hat {\mathbf {X}} _ {\mathrm{MV}} (\mathbf {Z}) = \mathbf {m} _ {X} + \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} (\mathbf {Z} - \mathbf {m} _ {Z})\tag{2.1.31}
$$

该估计的均方误差为

$$
\operatorname{Var} \left[ \boldsymbol {X} - \hat {\boldsymbol {X}} _ {\mathrm{MV}} (\boldsymbol {Z}) \right] = \boldsymbol {P} = \boldsymbol {C} _ {X} - \boldsymbol {C} _ {X Z} \boldsymbol {C} _ {Z} ^ {- 1} \boldsymbol {C} _ {Z X}\tag{2.1.32}
$$

证明

构造随机向量

$$
\mathbf {Y} = \left[ \begin{array}{c} \mathbf {X} \\ \mathbf {Z} \end{array} \right]
$$

则

$$
E [ \mathbf {Y} ] = \boldsymbol {m} _ {Y} = \left[ \begin{array}{l} \boldsymbol {m} _ {X} \\ \boldsymbol {m} _ {Z} \end{array} \right], \operatorname{Var} [ \mathbf {Y} ] = \boldsymbol {C} _ {Y} = \left[ \begin{array}{l l} \boldsymbol {C} _ {X} & \boldsymbol {C} _ {X Z} \\ \boldsymbol {C} _ {Z X} & \boldsymbol {C} _ {Z} \end{array} \right]
$$

X 和 Z 的联合分布密度为

$$
\begin{array}{r l} p (\mathbf {y}) = & p (\mathbf {x}, \mathbf {z}) = \frac {1}{(2 \pi) ^ {\frac {m + n}{2}} | \mathbf {C} _ {Y} | ^ {\frac {1}{2}}} \exp \left\{- \frac {1}{2} [ \mathbf {x} ^ {\mathrm{T}} - \mathbf {m} _ {X} ^ {\mathrm{T}} \quad \mathbf {z} ^ {\mathrm{T}} - \mathbf {m} _ {Z} ^ {\mathrm{T}} ] \mathbf {C} _ {Y} ^ {- 1} \cdot \right. \\ & \left. [ \mathbf {x} ^ {\mathrm{T}} - \mathbf {m} _ {X} ^ {\mathrm{T}} \quad \mathbf {z} ^ {\mathrm{T}} - \mathbf {m} _ {Z} ^ {\mathrm{T}} ] ^ {\mathrm{T}} \right\} \end{array}\tag{2.1.33}
$$

Z 的边缘分布密度为

$$
p (z) = \frac {1}{(2 \pi) ^ {\frac {m}{2}} | C _ {Z} | ^ {\frac {1}{2}}} \exp \left\{- \frac {1}{2} (z - m _ {Z}) ^ {\mathrm{T}} C _ {Z} ^ {- 1} (z - m _ {Z}) \right\}\tag{2.1.34}
$$

根据贝叶斯定理，X 对 Z 的条件分布密度为

$$
p (x / z) = \frac {p (x , z)}{p (z)}\tag{2.1.35}
$$

由于 X 满足正态分布, 所以从 $p(x/z)$ 中可获得 X 对于 Z 的条件均值 E[X/Z], 也就求得 X 的最小方差估计。下面根据 $p(y)$ , $p(z)$ 和贝叶斯公式求出 $p(x/z)$ , 其中 $C_{Y}^{-1}$ 和 $|C_{Y}|$ 的求解如下:

(1) $C_{Y}^{-1}$ 的求解。由于

$$
\begin{array}{r l} \left[ \begin{array}{c c} I & - C _ {X Z} C _ {Z} ^ {- 1} \\ 0 & I \end{array} \right] & \left[ \begin{array}{c c} C _ {X} & C _ {X Z} \\ C _ {Z X} & C _ {Z} \end{array} \right] \left[ \begin{array}{c c} I & 0 \\ - C _ {Z} ^ {- 1} C _ {X Z} ^ {\mathrm{T}} & I \end{array} \right] = \left[ \begin{array}{c c} C _ {X} - C _ {X Z} C _ {Z} ^ {- 1} C _ {Z X} & 0 \\ 0 & C _ {Z} \end{array} \right] \\ & \left[ \begin{array}{c c} C _ {X} & C _ {X Z} \\ C _ {Z X} & C _ {Z} \end{array} \right] = C _ {Y} \end{array}\tag{2.1.36}
$$

式中

所以对上式两边分别求逆，再左乘 $\begin{bmatrix} I & 0 \\ -C_Z^{-1}C_{XZ}^{\mathrm{T}} & I \end{bmatrix}$ ，右乘 $\begin{bmatrix} I & -C_{XZ}C_Z^{-1} \\ 0 & I \end{bmatrix}$ ，得

$$
\boldsymbol {C} _ {Y} ^ {- 1} = \left[ \begin{array}{c c} \boldsymbol {I} & \boldsymbol {0} \\ - \boldsymbol {C} _ {Z} ^ {- 1} \boldsymbol {C} _ {X Z} ^ {\mathrm{T}} & \boldsymbol {I} \end{array} \right] \left[ \begin{array}{c c} (\boldsymbol {C} _ {X} - \boldsymbol {C} _ {X Z} \boldsymbol {C} _ {Z} ^ {- 1} \boldsymbol {C} _ {Z X}) ^ {- 1} & \boldsymbol {0} \\ \boldsymbol {0} & \boldsymbol {C} _ {Z} ^ {- 1} \end{array} \right] \left[ \begin{array}{c c} \boldsymbol {I} & - \boldsymbol {C} _ {X Z} \boldsymbol {C} _ {Z} ^ {- 1} \\ \boldsymbol {0} & \boldsymbol {I} \end{array} \right]\tag{2.1.37}
$$

(2) $|C_{Y}|$ 的求取。对式(2.1.36)两边分别求取行列式,得

$$
\left| \begin{array}{c c} I & - C _ {X Z} C _ {Z} ^ {- 1} \\ 0 & I \end{array} \right| \cdot | C _ {Y} | \cdot \left| \begin{array}{c c} I & 0 \\ - C _ {Z} ^ {- 1} C _ {X Z} ^ {\mathrm{T}} & I \end{array} \right| = \left| \begin{array}{c c} C _ {X} - C _ {X Z} C _ {Z} ^ {- 1} C _ {Z X} & 0 \\ 0 & C _ {Z} \end{array} \right|
$$

按拉普拉斯法对各行列式作展开,得

$$
\mid C _ {Y} \mid = \mid C _ {X} - C _ {X Z} C _ {Z} ^ {- 1} C _ {Z X} \mid \cdot \mid C _ {Z} \mid\tag{2.1.38}
$$

将式 $(2.1.37)$ 和式 $(2.1.38)$ 代入式 $(2.1.33)$ ，其中指数部分为

$$
\begin{array}{l} E = \left[ \begin{array}{c c} x ^ {\mathrm{T}} - m _ {X} ^ {\mathrm{T}} & z ^ {\mathrm{T}} - m _ {Z} ^ {\mathrm{T}} \end{array} \right] C _ {Y} ^ {- 1} \left[ \begin{array}{c c} x ^ {\mathrm{T}} - m _ {X} ^ {\mathrm{T}} & z ^ {\mathrm{T}} - m _ {Z} ^ {\mathrm{T}} \end{array} \right] ^ {\mathrm{T}} = \\ \left\{\left[ (x - m _ {X}) ^ {\mathrm{T}} (z - m _ {Z}) ^ {\mathrm{T}} \right] \left[ \begin{array}{c c} I & 0 \\ - C _ {Z} ^ {- 1} C _ {X Z} ^ {\mathrm{T}} & I \end{array} \right] \left[ \begin{array}{c c} (C _ {X} - C _ {X Z} C _ {Z} ^ {- 1} C _ {Z X}) ^ {- 1} & 0 \\ 0 & C _ {Z} ^ {- 1} \end{array} \right] \right\}. \\ \left\{\left[ \begin{array}{c c} I & - C _ {X Z} C _ {Z} ^ {- 1} \\ 0 & I \end{array} \right] \left[ \begin{array}{c} x - m _ {X} \\ z - m _ {Z} \end{array} \right] \right\} = \left[ \left\{(x - m _ {X}) ^ {\mathrm{T}} - (z - m _ {Z}) ^ {\mathrm{T}} C _ {Z} ^ {- 1} C _ {X Z} ^ {\mathrm{T}} \right\} \cdot \right. \\ (C _ {X} - C _ {X Z} C _ {Z} ^ {- 1} C _ {Z X}) ^ {- 1} (z - m _ {Z}) ^ {\mathrm{T}} C _ {Z} ^ {- 1} \left[ \begin{array}{c} x - m _ {X} - C _ {X Z} C _ {Z} ^ {- 1} (z - m _ {Z}) \\ z - m _ {Z} \end{array} \right] \end{array}
$$

令 $\overline{m}_X = m_X + C_{XZ}C_Z^{-1}(z - m_Z)$ ，得

$$
\begin{array}{r l} E = & \left[ (x ^ {\mathrm{T}} - \overline {{m}} _ {X} ^ {\mathrm{T}}) (C _ {X} - C _ {X Z} C _ {Z} ^ {- 1} C _ {Z X}) ^ {- 1} \quad (z - m _ {Z}) ^ {\mathrm{T}} C _ {Z} ^ {- 1} \right] \left[ \begin{array}{l} x - \overline {{m}} _ {X} \\ z - m _ {Z} \end{array} \right] = \\ & (x ^ {\mathrm{T}} - \overline {{m}} _ {X} ^ {\mathrm{T}}) (C _ {X} - C _ {X Z} C _ {Z} ^ {- 1} C _ {Z X}) ^ {- 1} (x - \overline {{m}} _ {x}) + (z - m _ {Z}) ^ {\mathrm{T}} C _ {Z} ^ {- 1} (z - m _ {Z}) \end{array}
$$

所以

$$
\begin{array}{r l} p (x, z) = & \frac {1}{(2 \pi) ^ {\frac {m + n}{2}} \left| C _ {X} - C _ {X Z} C _ {Z} ^ {- 1} C _ {Z X} \right| ^ {\frac {1}{2}} \left| C _ {Z} \right| ^ {\frac {1}{2}}} \cdot \\ & \exp \left\{- \frac {1}{2} (x - \overline {{m}} _ {X}) ^ {\mathrm{T}} (C _ {X} - C _ {X Z} C _ {Z} ^ {- 1} C _ {Z X}) ^ {- 1} (x - \overline {{m}} _ {X}) \right\} \cdot \\ & \exp \left\{- \frac {1}{2} (z - m _ {Z}) ^ {\mathrm{T}} C _ {Z} ^ {- 1} \cdot (z - m _ {Z}) \right\} \end{array}\tag{2.1.39}
$$

将式(2.1.34)和式(2.1.39)代入式(2.1.35)，得

$$
\begin{array}{r l} p (\boldsymbol {x} / \boldsymbol {z}) & = \frac {1}{(2 \pi) ^ {\frac {n}{2}} \left| \boldsymbol {C} _ {X} - \boldsymbol {C} _ {X Z} \boldsymbol {C} _ {Z} ^ {- 1} \boldsymbol {C} _ {Z X} \right| ^ {\frac {1}{2}}} \cdot \\ & \exp \left\{- \frac {1}{2} (\boldsymbol {x} - \overline {{\boldsymbol {m}}} _ {X}) ^ {\mathrm{T}} (\boldsymbol {C} _ {X} - \boldsymbol {C} _ {X Z} \boldsymbol {C} _ {Z} ^ {- 1} \boldsymbol {C} _ {Z X}) ^ {- 1} (\boldsymbol {x} - \overline {{\boldsymbol {m}}} _ {X}) \right\} \end{array}\tag{2.1.40}
$$

从上式得 $X$ 关于 $\mathbf{Z}$ 的条件均值为

$$
E [ \mathbf {X} / \mathbf {Z} ] = \overline {{\boldsymbol {m}}} _ {X} = \boldsymbol {m} _ {X} + \boldsymbol {C} _ {X Z} \boldsymbol {C} _ {Z} ^ {- 1} (\mathbf {Z} - \boldsymbol {m} _ {Z})\tag{2.1.41}
$$

X 偏离 $\overline{m}_{x}$ 的方差阵为

$$
\boldsymbol {P} = \boldsymbol {C} _ {X} - \boldsymbol {C} _ {X Z} \boldsymbol {C} _ {Z} ^ {- 1} \boldsymbol {C} _ {Z X}\tag{2.1.42}
$$

根据定理 2.1, 上述两式分别为 X 的最小方差估计和估计的均方误差阵的计算公式。

定理 2.3 说明, 当被估计量 X 和量测量 Z 都服从正态分布时, X 的最小方差估计不必通过对条件概率密度的积分求取, 而只须知道 X 及 Z 的一阶和二阶矩。又从式(2.1.31) 知, $\hat{X}_{\mathrm{MV}}(\mathbf{Z})$ 是关于量测量 Z 的线性函数, 所以 $\hat{X}_{\mathrm{MV}}(\mathbf{Z})$ 是一种线性估计。

若量测量 Z 与被估计量 X 间具有线性关系，则式(2.1.31)和式(2.1.32)中的互协方差阵 $C_{XZ}$ 和 $C_{ZX}$ 可简化成 X 和 Z 各自的一、二阶矩，这给计算带来更大的方便。

$$
\mathbf {Z} = \mathbf {H X} + \mathbf {V}\tag{2.1.43}
$$

式中 V 为量测噪声， $E[V]=0$ ， $\operatorname{Var}[V]=C_{V}$ ， $E[X]=m_{X}$ ， $\operatorname{Var}[X]=C_{X}$ ，X 和 V 互不相关，H 为量测阵，是确定性矩阵。

由式(2.1.43)得

$$
\begin{array}{r l} & {\pmb {m} _ {Z} = \pmb {H m} _ {X}} \\ & {\pmb {C} _ {X Z} = E [ (\pmb {X} - \pmb {m} _ {X}) (\pmb {Z} - \pmb {m} _ {Z}) ^ {\mathrm{T}} ] = \pmb {C} _ {X} \pmb {H} ^ {\mathrm{T}}} \\ & {\pmb {C} _ {Z X} = \pmb {C} _ {X Z} ^ {\mathrm{T}} = \pmb {H C} _ {X}} \\ & {\pmb {C} _ {Z} = E [ (\pmb {Z} - \pmb {m} _ {Z}) (\pmb {Z} - \pmb {m} _ {Z}) ^ {\mathrm{T}} ] = \pmb {H C} _ {X} \pmb {H} ^ {\mathrm{T}} + \pmb {C} _ {V}} \end{array}
$$

将上述诸式代入式(2.1.31)和式(2.1.32)，得

$$
\hat {\mathbf {X}} _ {M V} (\mathbf {Z}) = \mathbf {m} _ {X} + \mathbf {C} _ {X} \mathbf {H} ^ {\mathrm{T}} (\mathbf {H C} _ {X} \mathbf {H} ^ {\mathrm{T}} + \mathbf {C} _ {V}) ^ {- 1} (\mathbf {Z} - \mathbf {H m} _ {X})\tag{2.1.44}
$$

$$
\boldsymbol {P} = \boldsymbol {C} _ {X} - \boldsymbol {C} _ {X} \boldsymbol {H} ^ {\mathrm{T}} (\boldsymbol {H C} _ {X} \boldsymbol {H} ^ {\mathrm{T}} + \boldsymbol {C} _ {V}) ^ {- 1} \boldsymbol {H C} _ {X}\tag{2.1.45}
$$

上述两式的另一种表达式为

$$
\hat {\boldsymbol {X}} _ {\mathrm{MV}} (\boldsymbol {Z}) = \left(\boldsymbol {C} _ {X} ^ {- 1} + \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {C} _ {V} ^ {- 1} \boldsymbol {H}\right) ^ {- 1} \left(\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {C} _ {V} ^ {- 1} \boldsymbol {Z} + \boldsymbol {C} _ {X} ^ {- 1} \boldsymbol {m} _ {X}\right)\tag{2.1.46}
$$

$$
\boldsymbol {P} = (\boldsymbol {C} _ {X} ^ {- 1} + \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {C} _ {V} ^ {- 1} \boldsymbol {H}) ^ {- 1}\tag{2.1.47}
$$

例2-6 设 $X$ 为服从正态分布的随机变量，均值为 $m_X$ ，方差为 $C_X$ ，对 $X$ 用 $m$ 台仪器同时直接测量，测量误差都是服从正态分布的随机变量，均值为零，方差为 $C_V$ ，求 $X$ 的最小方差估计和估计的均方误差。

解 根据题意,量测方程为 $Z=HX+V$

式中 $Z=\begin{bmatrix}Z_{1}\\ Z_{2}\\ \vdots\\ Z_{m}\end{bmatrix},H=\begin{bmatrix}1\\ 1\\ \vdots\\ 1\end{bmatrix},V=\begin{bmatrix}V_{1}\\ V_{2}\\ \vdots\\ V_{m}\end{bmatrix},VarV=C_{V}I$ ，由式(2.1.46)和式(2.1.47)，得

$$
\begin{array}{r l} \hat {X} _ {\mathrm{MV}} & = \left(\frac {1}{C _ {X}} + \frac {m}{C _ {V}}\right) ^ {- 1} \left(\frac {1}{C _ {V}} \sum_ {i = 1} ^ {m} Z _ {i} + \frac {m _ {X}}{C _ {X}}\right) = \\ & m _ {X} + \frac {m C _ {X}}{m C _ {X} + C _ {V}} \left(\frac {1}{m} \sum_ {i = 1} ^ {m} Z _ {i} - m _ {X}\right) \\ P & = \left(\frac {1}{C _ {x}} + \frac {m}{C _ {V}}\right) ^ {- 1} = \frac {C _ {X} C _ {V}}{m C _ {X} + C _ {V}} \end{array}
$$

由于 $P=\mathrm{Var}(X-\hat{X})$ ，它反映了估计误差的分散程度，所以 P 描述了估计的精度，P 越小，估计精度就越高。从 P 的表达式可看出，测量仪器越多，估计精度就越高。

## 2.1.3 极大验后估计

设 X 为随机向量, Z 为 X 的量测, $p(x/z)^{\circ}$ 为 Z=z 条件下 X 的条件概率密度(亦称 X 的验后概率密度)。如果估计值 $\hat{X}_{\mathrm{MA}}(z)$ 使下述指标满足

$$
p (x / z) \mid_ {x = \hat {X} _ {\mathrm{MA}} (z)} = \max\tag{2.1.48}
$$

则 $\hat{X}_{\mathrm{MA}}(z)$ 称为 $X$ 的极大验后估计，其中 $z$ 为 $Z$ 的某一实现。

由于 $\ln p(x/z)$ 与 $p(x/z)$ 在相同的 x 处取得极大值，所以式(2.1.48) 又可改写为

$$
\ln p (x / z) \mid_ {x = \hat {X} _ {\mathrm{MA}} (z)} = \max\tag{2.1.49}
$$

定理2.4

如果 X 和 Z 都服从正态分布, 则 X 的极大验后估计与 X 的最小方差估计相等, 即

$$
\hat {\mathbf {X}} _ {\mathrm{MA}} (\mathbf {Z}) = \hat {\mathbf {X}} _ {\mathrm{MV}} (\mathbf {Z})\tag{2.1.50}
$$

证明 由于 X 和 Z 都服从正态分布, 根据式(2.1.40)～式(2.1.42)得 X 的条件概率密度为

$$
p (\boldsymbol {x} / \boldsymbol {z}) = \frac {1}{(2 \pi) ^ {\frac {n}{2}} | \boldsymbol {P} | ^ {\frac {1}{2}}} \exp \left\{- \frac {1}{2} [ \boldsymbol {x} - \hat {\boldsymbol {X}} _ {\mathrm{MV}} (\boldsymbol {z}) ] ^ {\mathrm{T}} \boldsymbol {P} ^ {- 1} [ \boldsymbol {x} - \hat {\boldsymbol {X}} _ {\mathrm{MV}} (\boldsymbol {z}) ] \right\}
$$

对上式两边求对数, 得

$$
\ln p (\boldsymbol {x} / \boldsymbol {z}) = - \frac {n}{2} \ln (2 \pi) - \frac {1}{2} \ln | \boldsymbol {P} | - \frac {1}{2} [ \boldsymbol {x} - \hat {\boldsymbol {X}} _ {\mathrm{MV}} (\boldsymbol {z}) ] ^ {\mathrm{T}} \cdot \boldsymbol {P} ^ {- 1} [ \boldsymbol {x} - \boldsymbol {X} _ {\mathrm{MV}} (\boldsymbol {z}) ]
$$

要使上式达到最大值,必须有

$$
\left. \frac {\partial \ln p (x / z)}{\partial x} \right| _ {x = \hat {X} _ {\mathrm{MA}} (z)} = - P ^ {- 1} \left[ \hat {X} _ {\mathrm{MA}} (z) - \hat {X} _ {\mathrm{MV}} (z) \right] = 0
$$

所以

$$
\hat {\mathbf {X}} _ {\mathrm{MA}} (\mathbf {Z}) = \hat {\mathbf {X}} _ {\mathrm{MV}} (\mathbf {Z})
$$

必须注意, 只有当 X 和 Z 都服从正态分布时, 式(2.1.50)才成立。如果 X 和 Z 并不都满足正态分布, 则极大验后估计必须根据式(2.1.49)求取。

## 2.1.4 贝叶斯估计

前面介绍的最小方差估计和极大验后估计,实质上都是贝叶斯估计的特殊形式,因此有必

要对贝叶斯估计作介绍。

设 X 为被估计量, Z 是 X 的量测量, $\hat{X}(Z)$ 是根据 Z 给出的对 X 的估计, $\tilde{X} = X - \hat{X}(Z)$ 为估计误差, 如果标量函数

$$
L (\tilde {\boldsymbol {X}}) = L [ \boldsymbol {X} - \hat {\boldsymbol {X}} (\boldsymbol {Z}) ]\tag{2.1.51}
$$

具有如下性质：

(1) 当 $\|\tilde{X}_{2}\|\geqslant\|\tilde{X}_{1}\|$ 时， $L(\tilde{X}_{2})\geqslant L(\tilde{X}_{1})\geqslant0;$

(2) 当 $\|\tilde{X}\|=0$ 时, $L(\tilde{X})=0$ ;

(3) $L(\widetilde{\mathbf{X}}) = L(-\widetilde{\mathbf{X}})$

则称 $L(\tilde{X})$ 为 $\hat{X} (Z)$ 对被估计量 $X$ 的损失函数，也称代价函数，并称其期望值

$$
B (\hat {\mathbf {X}}) = E [ L (\tilde {\mathbf {X}}) ]\tag{2.1.52}
$$

为 $\hat{X}(Z)$ 的贝叶斯风险。其中 $\|\tilde{X}\|$ 为 $\tilde{X}$ 的范数。

将上式中的数学期望写成积分形式

$$
\begin{array}{r l} B (\hat {\boldsymbol {X}}) & = \int_ {- \infty} ^ {\infty} \int_ {- \infty} ^ {\infty} L [ \boldsymbol {x} - \hat {\boldsymbol {X}} (\boldsymbol {z}) ] p (\boldsymbol {x}, \boldsymbol {z}) \mathrm{d} \boldsymbol {x} \mathrm{d} \boldsymbol {z} = \\ & \quad \int_ {- \infty} ^ {\infty} \left\{\int_ {- \infty} ^ {\infty} L [ \boldsymbol {x} - \hat {\boldsymbol {X}} (\boldsymbol {z}) ] p (\boldsymbol {x} / \boldsymbol {z}) \mathrm{d} \boldsymbol {x} \right\} p _ {Z} (\boldsymbol {z}) \mathrm{d} \boldsymbol {z} \end{array}
$$

如果估计量 $\hat{X}_{B}(Z)$ 使贝叶斯风险

$$
B (\hat {\mathbf {X}} _ {B}) = E \left\{L (\mathbf {X} - \hat {\mathbf {X}} (\mathbf {Z}) \right] \} _ {\hat {\mathbf {X}} = \hat {\mathbf {X}} _ {B} (\mathbf {Z})} = \min
$$

则称 $\hat{X}_{B}(Z)$ 为 X 的贝叶斯估计。

显然，当 $L(\tilde{\mathbf{X}}) = \tilde{\mathbf{X}}^{\mathrm{T}}\tilde{\mathbf{X}}$ 时， $\hat{\mathbf{X}}_B(\mathbf{Z})$ 就是 $\mathbf{X}$ 的最小方差估计 $\hat{\mathbf{X}}_{\mathrm{MV}}(\mathbf{Z})$ 。

下面再分析贝叶斯估计与极大验后估计的关系。取估计量 $\hat{X}$ 的损失函数为

$$
L (\boldsymbol {X} - \hat {\boldsymbol {X}}) = \left\{ \begin{array}{l l} 0, & \| \boldsymbol {X} - \hat {\boldsymbol {X}} \| <   \frac {\varepsilon}{2} \\ \frac {1}{\varepsilon}, & \| \boldsymbol {X} - \hat {\boldsymbol {X}} \| \geqslant \frac {\varepsilon}{2} \end{array} \right.\tag{2.1.53}
$$

$\hat{\pmb{X}}$ 的贝叶斯风险为

$$
\begin{array}{l} B (\hat {\mathbf {X}}) = E [ L (\mathbf {X} - \hat {\mathbf {X}}) ] = \int_ {- \infty} ^ {\infty} \left[ \int_ {\| \mathbf {X} - \hat {\mathbf {X}} \| \geqslant \frac {\varepsilon}{2}} \frac {1}{\varepsilon} p (\mathbf {x} / \mathbf {z}) \mathrm{d} \mathbf {x} \right] p _ {Z} (\mathbf {z}) \mathrm{d} \mathbf {z} = \\ \int_ {- \infty} ^ {\infty} \frac {1}{\varepsilon} \left[ 1 - \int_ {\| \mathbf {X} - \hat {\mathbf {X}} \| <   \frac {\varepsilon}{2}} p (\mathbf {x} / \mathbf {z}) \mathrm{d} \mathbf {x} \right] p _ {Z} (\mathbf {z}) \mathrm{d} \mathbf {z} \end{array}
$$

设 $\hat{\boldsymbol{X}}_{B}(\boldsymbol{Z})$ 为 X 的贝叶斯估计,由上式知

$$
B (\hat {\mathbf {X}}) \mid_ {\hat {\mathbf {x}} = \hat {\mathbf {x}} _ {B}} = \min
$$

等价于

$$
\int_ {\| \boldsymbol {x} - \hat {\boldsymbol {X}} \| <   \frac {\varepsilon}{2}} p (\boldsymbol {x} / \boldsymbol {z}) \mathrm{d} \boldsymbol {x} \Big | _ {\hat {\boldsymbol {x}} = \hat {\boldsymbol {x}} _ {B}} = \max
$$

当 $\varepsilon$ 足够小 $(\varepsilon > 0)$ 时，这又相当于要求

$$
p (x / z) \mid_ {x = \hat {X} _ {B}} = \max
$$

此时 $\hat{X}_{B}$ 又是 X 的极大验后估计 $\hat{X}_{MA}$ 。因此，当损失函数为式(2.1.53)，且 $\varepsilon$ 足够小时，X 的贝叶斯估计就是 X 的极大验后估计。

## 2.1.5 极大似然估计

极大验后估计 $\hat{X}_{\mathrm{MA}}$ 是在 $Z = z$ 条件下，使被估计量 $X$ 的条件概率密度 $p(x / z)$ 达到最大的 $x$ 值，而极大似然估计则是使 $Z$ 的条件概率密度 $p(z / x)$ 在量测值 $Z = z$ 处达到最大的 $x$ 值。

设 X 是被估计量, Z 是 X 的量测值, $p(z/x)$ 是 X=x 的条件下, 量测量 Z 的条件概率密度, $p(z/x)$ 称为 X 的似然函数。如果由获得的量测量 Z=z 解算得的估计值 $\hat{X}_{\mathrm{ML}}(Z)$ 使

$$
p (z / x) \mid_ {x = \hat {X} _ {\mathrm{ML}} (Z)} = \max\tag{2.1.54}
$$

则 $\hat{X}_{\mathrm{ML}}(Z)$ 称为 $X$ 的极大似然估计。

与极大验后估计的定义式一样,式(2.1.54)可改写为

$$
\ln p (z / x) \mid_ {x = \hat {X} _ {\mathrm{ML}}} = \max\tag{2.1.55}
$$

式中 $\hat{X}_{\mathrm{ML}}$ 应满足方程

$$
\left. \frac {\partial \ln p (z / x)}{\partial x} \right| _ {x = \hat {X} _ {\mathrm{ML}}} = 0\tag{2.1.56}
$$

式 $(2.1.56)$ 称为似然方程。

与极大验后估计相比，极大似然估计有两个优点：① 确定似然函数 $p(z / x)$ 比确定验后概率密度 $p(x / z)$ 容易些；② 在极大似然估计中，被估计量 $X$ 可以是随机的，也可以是非随机的，适用的范围比极大验后估计广。但是，如果对 $X$ 已具有验前知识，则极大似然估计 $\hat{X}_{\mathrm{ML}}$ 的精度不如极大验后估计 $\hat{X}_{\mathrm{MA}}$ 的精度高，这是因为求取 $\hat{X}_{\mathrm{MA}}$ 时利用了 $X$ 的验前统计信息 $p_X(x)$ ，而 $\hat{X}_{\mathrm{ML}}$ 中并未利用这一信息。只有在不掌握 $X$ 的任何验前知识时，两者的估计精度才相同。下面说明这一点。

定理2.5

如果对 X 没有任何验前知识, 则 X 的极大似然估计与 X 的极大验后估计相同, 即

$$
\hat {\mathbf {X}} _ {\mathrm{ML}} (\mathbf {Z}) = \hat {\mathbf {X}} _ {\mathrm{MA}} (\mathbf {Z})\tag{2.1.57}
$$

证明 根据联合分布与条件分布及边缘分布之间的关系,有

$$
p (\boldsymbol {x} / \boldsymbol {z}) p _ {Z} (\boldsymbol {z}) = p (\boldsymbol {z} / \boldsymbol {x}) p _ {X} (\boldsymbol {x})
$$

即

$$
p (\boldsymbol {x} / \boldsymbol {z}) = \frac {p (\boldsymbol {z} / \boldsymbol {x}) p _ {X} (\boldsymbol {x})}{p _ {Z} (\boldsymbol {z})}
$$

两边取对数,得

$$
\ln p (x / z) = \ln p (z / x) + \ln p _ {x} (x) - \ln p _ {z} (z)\tag{2.1.58}
$$

设 $\hat{X}_{\mathrm{MA}}$ 为 $X$ 的极大验后估计，即

$$
p (x / z) \mid_ {x = \hat {X} _ {\mathrm{MA}}} = \max
$$

则

$$
\left. \frac {\partial \ln p (x / z)}{\partial x} \right| _ {x = \hat {X} _ {\mathrm{MA}}} = 0
$$

将式(2.1.58)代入上式，并注意到 $\ln p_{z}(z)$ 对 x 的偏导数为零，得

$$
\left. \frac {\partial \ln p (z / x)}{\partial x} \right| _ {x = \dot {X} _ {\mathrm{MA}}} + \left. \frac {\mathrm{d} \ln p _ {X} (x)}{\mathrm{d} x} \right| _ {x = \dot {X} _ {\mathrm{MA}}} = 0\tag{2.1.59}
$$

由于假设对 X 没有任何验前知识, 只能认为 X 取得任何值的概率都相等, 于是可将 X 的验前概率密度近似看做方差为无穷大的高斯概率密度

$$
p _ {X} (\boldsymbol {x}) = \frac {1}{\sqrt {(2 \pi) ^ {n} | \boldsymbol {V} _ {X} |}} \exp \left[ - \frac {1}{2} (\boldsymbol {x} - \boldsymbol {\mu} _ {X}) ^ {\mathrm{T}} \boldsymbol {V} _ {X} ^ {- 1} (\boldsymbol {x} - \boldsymbol {\mu} _ {X}) \right]
$$

即

$$
\ln p _ {X} (\boldsymbol {x}) = - \frac {1}{2} \ln [ (2 \pi) ^ {n} | \boldsymbol {V} _ {X} | ] - \frac {1}{2} (\boldsymbol {x} - \boldsymbol {\mu} _ {X}) ^ {\mathrm{T}} \boldsymbol {V} _ {X} ^ {- 1} (\boldsymbol {x} - \boldsymbol {\mu} _ {X})
$$

式中

$$
V _ {X} = \sigma^ {2} I \qquad \sigma \rightarrow \infty
$$

所以

$$
\frac {\mathrm{d} \ln p _ {X} (\boldsymbol {x})}{\mathrm{d} \boldsymbol {x}} = - \boldsymbol {V} _ {X} ^ {- 1} (\boldsymbol {x} - \boldsymbol {\mu} _ {X}) = - \lim _ {\sigma \rightarrow \infty} \frac {1}{\sigma^ {2}} \boldsymbol {I} (\boldsymbol {x} - \boldsymbol {\mu} _ {X}) = \mathbf {0}
$$

因此式(2.1.59)为

$$
\left. \frac {\partial \ln p (z / x)}{\partial x} \right| _ {x = \dot {X} _ {\mathrm{MA}}} = 0\tag{2.1.60}
$$

比较式(2.1.60)和式(2.1.56)知,在缺乏X的任何验前知识时

$$
\hat {\boldsymbol {X}} _ {\mathrm{MA}} = \hat {\boldsymbol {X}} _ {\mathrm{ML}}
$$

例2-7 设 $X$ 为 $n$ 维随机向量，服从 $N(\mu, P)$ 分布， $Z = HX + V$ ，其中 $V$ 为 $m$ 维随机向量，服从 $N(0, R)$ 分布， $X$ 与 $V$ 相互独立，求 $\hat{X}_{\mathrm{ML}}$ 。

解 由于 X 和 V 都服从正态分布, 所以 $Z = HX + V$ 也服从正态分布, X 和 Z 的联合分布 $p(x, z)$ 也是正态的, 根据贝叶斯定理

$$
p (z / x) = \frac {p (x , z)}{p _ {x} (x)}
$$

也是正态的。要获得 $p(z / x)$ ，只须求取 $\mathbf{Z}$ 的条件均值和条件方差。由定理2.1知， $\mathbf{Z}$ 在 $X = x$ 条件下的条件均值即为 $\mathbf{Z}$ 的最小方差估计，根据式(2.1.31)和式(2.1.32)，得 $\mathbf{Z}$ 的条件均值和条件方差为

$$
\begin{array}{r l} & E [ \mathbf {Z} / \mathbf {x} ] = E [ \mathbf {Z} ] + \operatorname{Cov} (\mathbf {Z}, \mathbf {X}) (\operatorname{Var} [ \mathbf {X} ]) ^ {- 1} (\mathbf {x} - E [ \mathbf {X} ]) \\ & \operatorname{Var} [ \mathbf {Z} / \mathbf {x} ] = \operatorname{Var} [ \mathbf {Z} ] - \operatorname{Cov} (\mathbf {Z}, \mathbf {X}) (\operatorname{Var} [ \mathbf {X} ]) ^ {- 1} \operatorname{Cov} (\mathbf {X}, \mathbf {Z}) \end{array}
$$

而

$$
\begin{array}{r l} & E [ X ] = \mu , \quad \mathrm{Var} [ X ] = P \\ & E [ Z ] = H \mu , \quad \mathrm{Var} [ Z ] = H P H ^ {\mathrm{T}} + R \\ & \mathrm{Cov} (X, Z) = P H ^ {\mathrm{T}} \\ & \mathrm{Cov} (Z, X) = [ \mathrm{Cov} (X, Z) ] ^ {\mathrm{T}} = H P \end{array}
$$

所以

$$
\begin{array}{r l} & E [ \mathbf {Z} / x ] = H \boldsymbol {\mu} + H P P ^ {- 1} (x - \boldsymbol {\mu}) = H x \\ & \operatorname{Var} [ \mathbf {Z} / x ] = H P H ^ {\mathrm{T}} + R - H P P ^ {- 1} P H ^ {\mathrm{T}} = R \end{array}
$$

因此似然函数为

$$
p (\mathbf {z} / \mathbf {x}) = \frac {1}{\sqrt {(2 \pi) ^ {m} | \mathbf {R} |}} \exp \left[ - \frac {1}{2} (\mathbf {z} - \mathbf {H x}) ^ {\mathrm{T}} \mathbf {R} ^ {- 1} (\mathbf {z} - \mathbf {H x}) \right]
$$

对数形式为

$$
\ln p (\mathbf {z} / \mathbf {x}) = - \frac {m}{2} \ln (2 \pi) - \frac {1}{2} \ln | \mathbf {R} | - \frac {1}{2} (\mathbf {z} - \mathbf {H x}) ^ {\mathrm{T}} \mathbf {R} ^ {- 1} (\mathbf {z} - \mathbf {H x})
$$

极大似然估计应满足

$$
\left. \frac {\partial \ln p (z / x)}{\partial x} \right| _ {x = \dot {X} _ {\mathrm{ML}}} = 0
$$

即

$$
\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {R} ^ {- 1} (\boldsymbol {z} - \boldsymbol {H} \hat {\boldsymbol {X}} _ {\mathrm{ML}}) = \boldsymbol {0}
$$

所以

$$
\hat {\boldsymbol {X}} _ {\mathrm{ML}} = (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {R} ^ {- 1} \boldsymbol {H}) ^ {- 1} \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {R} ^ {- 1} \boldsymbol {z}
$$

式中 z 是某一量测值。

## 2.1.6 线性最小方差估计

## 1. 线性最小方差估计的求取

由式(2.1.31)知,当被估计量 X 和量测量 Z 都服从正态分布时,X 的最小方差估计 $\hat{X}_{MV}$ 是 Z 的线性函数,求解 $\hat{X}_{MV}$ 时只须知道 X 和 Z 的一、二阶矩。现在反过来问,如果 X 和 Z 并不满足正态分布,而 X 的估计也要表达成 Z 的线性函数,满足的最优指标也是使均方误差最小,X 的估计应怎样求,这一估计与 X 的最小方差估计有何不同。

设 X 为被估计量, Z 是 X 的量测量, 如果 $\hat{X}_{L}(Z)=AZ+b$ 使下述指标满足

$$
E \left[ (\boldsymbol {X} - \hat {\boldsymbol {X}}) ^ {\mathrm{T}} (\boldsymbol {X} - \hat {\boldsymbol {X}}) \right] | _ {\hat {\boldsymbol {X}} = \hat {\boldsymbol {X}} _ {L} (\mathbf {Z})} = \min\tag{2.1.61}
$$

则称 $\hat{X}_L(Z)$ 为 $X$ 在 $Z$ 上的线性最小方差估计，有时用符号 $E^{*}[X / Z]$ 表示之， $*$ 表示与条件均值的区别。

有关量测量 Z 的线性函数有无穷多个,但能使 $\hat{X}$ 具有最小均方误差的线性函数却只有一个,记为

$$
\hat {\mathbf {X}} _ {L} (\mathbf {Z}) = \mathbf {A} ^ {0} \mathbf {Z} + \mathbf {b} ^ {0}
$$

$$
\boldsymbol {A} ^ {0}
$$

$$
\boldsymbol {b} ^ {0}
$$

关系式1

设随机向量 $W = [W_{1} W_{2} \cdots W_{n}]^{T}$ ，则

$$
E [ \boldsymbol {W} ^ {\mathrm{T}} \boldsymbol {W} ] = \operatorname{tr} (\boldsymbol {C} _ {w} + E \boldsymbol {W} \boldsymbol {E} \boldsymbol {W} ^ {\mathrm{T}})
$$

式中 $C_{w}=E\left[(W-EW)(W-EW)^{\mathrm{T}}\right]$ ，tr 表示对矩阵求迹。

证明

$$
\begin{array}{r l} E [ \mathbf {W} ^ {\mathrm{T}} \mathbf {W} ] & = E [ W _ {1} ^ {2} + W _ {2} ^ {2} + \dots + W _ {n} ^ {2} ] = E [ \operatorname{tr} (\mathbf {W W} ^ {\mathrm{T}}) ] = \\ & \quad \operatorname{tr} \{E [ (\mathbf {W} - E \mathbf {W} + E \mathbf {W}) (\mathbf {W} - E \mathbf {W} + E \mathbf {W}) ^ {\mathrm{T}} ] \} = \\ & \quad \operatorname{tr} \{\mathbf {C} _ {w} + E \mathbf {W} E \mathbf {W} ^ {\mathrm{T}} + E [ (\mathbf {W} - E \mathbf {W}) E \mathbf {W} ^ {\mathrm{T}} ] + E [ E \mathbf {W} (\mathbf {W} - E \mathbf {W}) ^ {\mathrm{T}} ] \} = \\ & \quad \operatorname{tr} (\mathbf {C} _ {w} + E \mathbf {W} E \mathbf {W} ^ {\mathrm{T}}) \end{array}
$$

关系式2

设 $Y = X - A^0 Z - b^0$ ， $EX = m_X$ ， $EZ = m_Z$ ，则

$$
\begin{array}{r l} & E \mathbf {Y} = \mathbf {m} _ {X} - \mathbf {A} ^ {0} \mathbf {m} _ {Z} - \mathbf {b} ^ {0} \\ & \mathbf {C} _ {Y} = E [ (\mathbf {Y} - E \mathbf {Y}) (\mathbf {Y} - E \mathbf {Y}) ^ {\mathrm{T}} ] = E \{\left[ (\mathbf {X} - \mathbf {m} _ {X}) - \mathbf {A} ^ {0} (\mathbf {Z} - \mathbf {m} _ {Z}) \right] [ (\mathbf {X} - \mathbf {m} _ {X}) ^ {\mathrm{T}} - \\ & (\mathbf {Z} - \mathbf {m} _ {Z}) ^ {\mathrm{T}} \mathbf {A} ^ {0 \mathrm{T}} ] \} = \mathbf {C} _ {X} + \mathbf {A} ^ {0} \mathbf {C} _ {Z} \mathbf {A} ^ {0 \mathrm{T}} - \mathbf {C} _ {X Z} \mathbf {A} ^ {0 \mathrm{T}} - \mathbf {A} ^ {0} \mathbf {C} _ {Z X} \end{array}
$$

关系式3

设 $L_{(n\times m)}$ , $C_{Z(m\times m)}$ ，其中 $C_{Z}$ 为随机向量 Z 的协方差阵，则 $\mathrm{tr}[LC_{Z}L^{T}]\geqslant0$ ; 若 $\mathrm{tr}[LC_{Z}L^{T}]=0$ ，则 L=0。

证明

$$
\operatorname{tr} \left[ L C _ {Z} L ^ {\mathrm{T}} \right] = \operatorname{tr} \left\{L E \left[ (Z - m _ {Z}) (Z - m _ {Z}) ^ {\mathrm{T}} \right] L ^ {\mathrm{T}} \right\} = \operatorname{tr} \left\{E \left[ [ L (Z - m _ {Z}) ] [ L (Z - m _ {Z}) ] ^ {\mathrm{T}} \right] \right\} =
$$

$$
E \{\operatorname{tr} [ [ L (Z - m _ {z}) ] [ L (Z - m _ {z}) ] ^ {\mathrm{T}} ] \}
$$

令 $L(Z - m_{Z}) = V = [V_{1} V_{2} \cdots V_{n}]^{T}$ ，则

$$
\operatorname{tr} \left[ L C _ {Z} L ^ {\mathrm{T}} \right] = E \left[ \operatorname{tr} \left(V V ^ {\mathrm{T}}\right) \right] = E \left[ V _ {1} ^ {2} + V _ {2} ^ {2} + \dots + V _ {n} ^ {2} \right] \geqslant 0
$$

若 $\operatorname{tr}[LC_Z L^{\mathrm{T}}] = 0$ ，则 $V_{1}^{2} + V_{2}^{2} + \dots + V_{n}^{2} = 0$ ，即

$$
L (Z - m _ {z}) = 0
$$

由于 Z 为随机向量,要使上式恒成立,必须有

$$
L = 0
$$

关系式4

$$
\begin{array}{r l} & (A ^ {0} - C _ {X Z} C _ {Z} ^ {- 1}) C _ {Z} (A ^ {0 \mathrm{T}} - C _ {Z} ^ {- 1} C _ {Z X}) + C _ {X} - C _ {X Z} C _ {Z} ^ {- 1} C _ {Z X} = \\ & A ^ {0} C _ {Z} A ^ {0 \mathrm{T}} - A ^ {0} C _ {Z} C _ {Z} ^ {- 1} C _ {Z X} - C _ {X Z} C _ {Z} ^ {- 1} C _ {Z} A ^ {0 \mathrm{T}} + C _ {X Z} C _ {Z} ^ {- 1} C _ {Z} C _ {Z} ^ {- 1} C _ {Z X} + C _ {X} - C _ {X Z} C _ {Z} ^ {- 1} C _ {Z X} = \\ & C _ {X} + A ^ {0} C _ {Z} A ^ {0 \mathrm{T}} - A ^ {0} C _ {Z X} - C _ {X Z} A ^ {0 \mathrm{T}} \end{array}
$$

利用上述 4 个辅助关系式, 可方便地确定出满足式(2.1.61) 的 $A^{0}$ 和 $b^{0}$ 。

令

则

$$
\begin{array}{r l} & {\mathbf {Y} = \mathbf {X} - \mathbf {A} ^ {0} \mathbf {Z} - \mathbf {b}} \\ & {J = E [ (\mathbf {X} - \mathbf {A} ^ {0} \mathbf {Z} - \mathbf {b} ^ {0}) ^ {\mathrm{T}} (\mathbf {X} - \mathbf {A} ^ {0} \mathbf {Z} - \mathbf {b} ^ {0}) ] = E [ \mathbf {Y} ^ {\mathrm{T}} \mathbf {Y} ]} \end{array}
$$

将关系式 1 代入上式, 得

$$
J = E [ \mathbf {Y} ^ {\mathrm{T}} \mathbf {Y} ] = \operatorname{tr} (\mathbf {C} _ {Y} + E \mathbf {Y} E \mathbf {Y} ^ {\mathrm{T}}) = \operatorname{tr} \mathbf {C} _ {Y} + E \mathbf {Y} ^ {\mathrm{T}} E \mathbf {Y}
$$

将关系式 2 代入上式, 得

$$
J = \operatorname{tr} \left(\boldsymbol {C} _ {X} + \boldsymbol {A} ^ {0} \boldsymbol {C} _ {Z} \boldsymbol {A} ^ {0 \mathrm{T}} - \boldsymbol {C} _ {X Z} \boldsymbol {A} ^ {0 \mathrm{T}} - \boldsymbol {A} ^ {0} \boldsymbol {C} _ {Z X}\right) + \left(\boldsymbol {m} _ {X} - \boldsymbol {A} ^ {0} \boldsymbol {m} _ {Z} - \boldsymbol {b} ^ {0}\right) ^ {\mathrm{T}} \left(\boldsymbol {m} _ {X} - \boldsymbol {A} ^ {0} \boldsymbol {m} _ {Z} - \boldsymbol {b} ^ {0}\right)
$$

利用关系式 4, 上式成

$$
\begin{array}{r l} J & = \mathrm{tr} \big [ (\mathbf {A} ^ {0} - \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1}) \mathbf {C} _ {Z} (\mathbf {A} ^ {0 \mathrm{T}} - \mathbf {C} _ {Z} ^ {- 1} \mathbf {C} _ {Z X}) \big ] + \mathrm{tr} (\mathbf {C} _ {X} - \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} \mathbf {C} _ {Z X}) + \\ & \quad (\mathbf {m} _ {X} - \mathbf {A} ^ {0} \mathbf {m} _ {Z} - \mathbf {b} ^ {0}) ^ {\mathrm{T}} (\mathbf {m} _ {X} - \mathbf {A} ^ {0} \mathbf {m} _ {Z} - \mathbf {b} ^ {0}) \end{array}
$$

上式三项中,第二项与 $A^{0}$ 和 $b^{0}$ 的选择无关。第三项为向量的内积,恒大于等于零,要使J达到最小,必须有

$$
\boldsymbol {m} _ {X} - \boldsymbol {A} ^ {0} \boldsymbol {m} _ {Z} - \boldsymbol {b} ^ {0} = \boldsymbol {0}\tag{2.1.62}
$$

又由关系式 3 知, 第一项也大于等于零, 当该项取最小值零时, 须有

$$
\mathbf {A} ^ {0} - \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} = \mathbf {0}\tag{2.1.63}
$$

由式 $(2.1.62)$ 和式 $(2.1.63)$ ，得

$$
\begin{array}{l} \mathbf {A} ^ {0} = \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} \\ \mathbf {b} ^ {0} = \mathbf {m} _ {X} - \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} \mathbf {m} _ {Z} \end{array}
$$

因此 X 在 Z 上的线性最小方差估计为

$$
\hat {\mathbf {X}} _ {L} (\mathbf {Z}) = \mathbf {m} _ {X} + \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} (\mathbf {Z} - \mathbf {m} _ {Z})\tag{2.1.64}
$$

下面分析线性最小方差估计的均方误差

$$
\begin{array}{r l} \mathbf {P} & = E \left\{\left[ \mathbf {X} - \hat {\mathbf {X}} _ {L} (\mathbf {Z}) \right] \left[ \mathbf {X} - \hat {\mathbf {X}} _ {L} (\mathbf {Z}) \right] ^ {\mathrm{T}} \right\} = \\ & E \left\{\left[ (\mathbf {X} - \mathbf {m} _ {X}) - \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} (\mathbf {Z} - \mathbf {m} _ {Z}) \right] \left[ (\mathbf {X} - \mathbf {m} _ {X}) - \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} (\mathbf {Z} - \mathbf {m} _ {Z}) \right] ^ {\mathrm{T}} \right\} = \\ & \mathbf {C} _ {X} + \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} \mathbf {C} _ {Z} \mathbf {C} _ {Z} ^ {- 1} \mathbf {C} _ {Z X} - \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} \mathbf {C} _ {Z X} - \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} \mathbf {C} _ {Z X} = \\ & \mathbf {C} _ {X} - \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} \mathbf {C} _ {Z X} \end{array}
$$

(2.1.65)

比较式(2.1.31)、式(2.1.32)、式(2.1.64)和式(2.1.65)，知式(2.1.31)与式(2.1.64)具有完全相同的形式，式(2.1.32)与式(2.1.65)也是这样。这说明：如果被估计量X和量测量Z都服从正态分布，则线性最小方差估计与最小方差估计是相同的，即按式(2.1.64)计算得的估计不但在所有的线性估计中精度最优，而且在所有的估计中精度也是最优的。

## 2. 线性最小方差估计的性质

性质1 无偏性

线性最小方差估计是 X 在 Z 上的无偏估计, 即

$$
E [ \mathbf {X} - \hat {\mathbf {X}} _ {l} (\mathbf {Z}) ] = \mathbf {0}
$$

或

$$
E _ {Z} [ \hat {\mathbf {X}} _ {L} (\mathbf {Z}) ] = E \mathbf {X}
$$

根据式 $(2.1.64)$ 对Z求均值即可证明此性质。

性质2 线性1

线性最小方差估计具有线性性质,即若 X 的线性最小方差估计为 $E^{*}[X/Z]$ , 则 $FX + e$ 的线性最小方差估计为

$$
E ^ {*} \left[ (F X + e) / Z \right] = F E ^ {*} \left[ X / Z \right] + e\tag{2.1.66}
$$

式中，F 为确定性矩阵，e 为确定性向量。

证明 令 $FX + e = K$ ，则

$$
\boldsymbol {m} _ {K} = \boldsymbol {F m} _ {X} + \boldsymbol {e}
$$

$$
\mathbf {C} _ {\mathrm{KZ}} = E \left[ (\mathbf {K} - \mathbf {m} _ {\mathrm{K}}) (\mathbf {Z} - \mathbf {m} _ {\mathrm{Z}}) ^ {\mathrm{T}} \right] = E \left[ (\mathbf {F X} + e - \mathbf {F m} _ {\mathrm{X}} - e) (\mathbf {Z} - \mathbf {m} _ {\mathrm{Z}}) ^ {\mathrm{T}} \right] = \mathbf {F C} _ {\mathrm{XZ}}
$$

根据式 $(2.1.64)$ ，得

$$
\begin{array}{r l} & E ^ {*} [ \mathbf {K} / \mathbf {Z} ] = m _ {K} + \mathbf {C} _ {K Z} \mathbf {C} _ {Z} ^ {- 1} (\mathbf {Z} - m _ {Z}) = \mathbf {F} m _ {X} + e + \mathbf {F C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} (\mathbf {Z} - m _ {Z}) = \\ & \qquad \qquad \qquad \qquad \qquad \mathbf {F} [ m _ {X} + \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} (\mathbf {Z} - m _ {Z}) ] + e = \mathbf {F E} ^ {*} [ \mathbf {X} / \mathbf {Z} ] + e \end{array}
$$

性质3 线性2

若 $Y$ 与 $Z$ 互不相关，则

$$
E ^ {*} [ \mathbf {X} / \mathbf {Y}, \mathbf {Z} ] = E ^ {*} [ \mathbf {X} / \mathbf {Y} ] + E ^ {*} [ \mathbf {X} / \mathbf {Z} ] - E \mathbf {X}\tag{2.1.67}
$$

证明 令 $\pmb{T} = \begin{bmatrix} \mathbf{Y}\\ \mathbf{Z} \end{bmatrix}$ ，则

$$
E ^ {*} [ \mathbf {X} / \mathbf {Y}, \mathbf {Z} ] = E ^ {*} [ \mathbf {X} / \mathbf {T} ] = E \mathbf {X} + \mathbf {C} _ {\mathbf {X T}} \mathbf {C} _ {T} ^ {- 1} (\mathbf {T} - E \mathbf {T})
$$

式中

$$
\boldsymbol {C} _ {X T} = E \left\{\left[ \boldsymbol {X} - \boldsymbol {m} _ {X} \right] \left[ \begin{array}{l} \boldsymbol {Y} - \boldsymbol {m} _ {Y} \\ \boldsymbol {Z} - \boldsymbol {m} _ {Z} \end{array} \right] ^ {\mathrm{T}} \right\} = \left[ \begin{array}{l l} \boldsymbol {C} _ {X Y} & \boldsymbol {C} _ {X Z} \end{array} \right]
$$

$$
\boldsymbol {C} _ {T} = E \left\{\left[ \begin{array}{l} \boldsymbol {Y} - \boldsymbol {m} _ {Y} \\ \boldsymbol {Z} - \boldsymbol {m} _ {Z} \end{array} \right] \left[ \begin{array}{l l} (\boldsymbol {Y} - \boldsymbol {m} _ {Y}) ^ {\mathrm{T}} & (\boldsymbol {Z} - \boldsymbol {m} _ {Z}) ^ {\mathrm{T}} \end{array} \right] \right\}
$$

由于 $Y$ 和 $Z$ 互不相关，所以

$$
\begin{array}{r l} & \mathbf {C} _ {T} = \left[ \begin{array}{l l} \mathbf {C} _ {Y} & \mathbf {0} \\ \mathbf {0} & \mathbf {C} _ {Z} \end{array} \right] \\ & E ^ {*} [ \mathbf {X} / \mathbf {Y}, \mathbf {Z} ] = E \mathbf {X} + [ \mathbf {C} _ {X Y} - \mathbf {C} _ {X Z} ] \left[ \begin{array}{l l} \mathbf {C} _ {Y} & \mathbf {0} \\ \mathbf {0} & \mathbf {C} _ {Z} \end{array} \right] ^ {- 1} \left[ \begin{array}{l} \mathbf {Y} - m _ {Y} \\ \mathbf {Z} - m _ {Z} \end{array} \right] = \\ & E \mathbf {X} + C _ {X Y} C _ {Y} ^ {- 1} (\mathbf {Y} - m _ {Y}) + E \mathbf {X} + C _ {X Z} C _ {Z} ^ {- 1} (\mathbf {Z} - m _ {Z}) - E \mathbf {X} = \\ & E ^ {*} [ \mathbf {X} / \mathbf {Y} ] + E ^ {*} [ \mathbf {X} / \mathbf {Z} ] - E \mathbf {X} \end{array}
$$

例2-8 设 $S(t)$ 为零均值标量平稳随机过程，相关函数 $R_{s}(\tau) = E[S(t)S(t + \tau)] =$

$\frac{1}{2}\mathrm{e}^{-|\tau|}$ ，试根据 $S(t)$ 用线性最小方差估计预测 $S(t + T)$ 。

解 根据题意, $S(t)$ 为量测,记为 $Z,S(t+T)$ 为被估计量,记为X。由于 $S(t)$ 为零均值的标量平稳随机过程,所以

$$
C _ {Z} = E [ S (t) S (t) ] = R _ {S} (\tau) \mid_ {\tau = 0} = \frac {1}{2}
$$

$$
C _ {X Z} = E [ S (t + T) S (t) ] = R _ {S} (\tau) \mid_ {\tau = T} = \frac {1}{2} \mathrm{e} ^ {- T}
$$

$$
m _ {X} = 0, m _ {Z} = 0
$$

根据式 $(2.1.64)$ ，得

$$
\hat {S} (t + T) = \frac {1}{2} \mathrm{e} ^ {- T} \times 2 \times S (t) = \mathrm{e} ^ {- T} S (t)
$$

例 2-9 设 $X(t)$ 为标量平稳随机过程, 均值为零, 相关函数为 $R(\tau)$ , 量测量 $Z(t_k) = X(t_k) + V(t_k), k = 1, 2, \cdots, V(t_k)$ 为零均值的白噪声序列, 方差为 $C_V, V(t_k)$ 与 $X(t_i) (i = 1, 2, \cdots, k)$ 不相关, 试用 $Z(t_{k-1})$ 和 $Z(t_k)$ 求线性最小方差估计 $\hat{X}(t_k)$ , 并与单独使用 $Z(t_k)$ 求解 $\hat{X}(t_k)$ 的精度作比较。

$$
\begin{array}{r l} & {\text {解} \quad \mathbf {Z} = \left[ \begin{array}{l} {Z (t _ {k - 1})} \\ {Z (t _ {k})} \end{array} \right] = \left[ \begin{array}{l} {X (t _ {k - 1}) + V (t _ {k - 1})} \\ {X (t _ {k}) + V (t _ {k})} \end{array} \right]} \\ & {\mathbf {C} _ {X Z} = E \{X (t _ {k}) [ X (t _ {k - 1}) + V (t _ {k - 1}) \quad X (t _ {k}) + V (t _ {k}) ] \} = [ R (T) \quad R (0) ]} \\ & {\qquad \mathbf {C} _ {Z} = E \Bigg \{\left[ \begin{array}{l l} {X (t _ {k - 1}) + V (t _ {k - 1})} \\ {X (t _ {k}) + V (t _ {k})} \end{array} \right] [ X (t _ {k - 1}) + V (t _ {k - 1}) \quad X (t _ {k}) + V (t _ {k}) ] \Bigg \} =} \\ & {\qquad \left[ \begin{array}{l l} {R (0) + C _ {V}} & {R (T)} \\ {R (T)} & {R (0) + C _ {V}} \end{array} \right]} \\ & {\qquad T = t _ {k} - t _ {k - 1}, \quad C _ {X} = E [ X ^ {2} (t _ {k}) ] = R (0)} \end{array}
$$

式中

根据式(2.1.64)， $X(t_{k})$ 的估计为

$$
\hat {X} (t _ {k}) = \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} \mathbf {Z} = \left[ \begin{array}{l l} R (T) & R (0) \end{array} \right] \left[ \begin{array}{c c} R (0) + C _ {V} & R (T) \\ R (T) & R (0) + C _ {V} \end{array} \right] ^ {- 1} \mathbf {Z}
$$

根据式 $(2.1.65)$ ，估计的均方误差为

$$
P = C _ {X} - \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} \mathbf {C} _ {Z X} = R (0) - [ R (T) \quad R (0) ] \left[ \begin{array}{c c} R (0) + C _ {V} & R (T) \\ R (T) & R (0) + C _ {V} \end{array} \right] ^ {- 1} \left[ \begin{array}{c} R (T) \\ R (0) \end{array} \right] =
$$

$$
R (0) - \frac {R ^ {2} (T) C _ {V} - R ^ {2} (T) R (0) + R ^ {2} (0) C _ {V} + R ^ {3} (0)}{\left[ R (0) + C _ {V} \right] ^ {2} - R ^ {2} (T)}
$$

若单独使用 $Z(t_{k})$ 来估计 $X(t_{k})$ ，根据式(2.1.65)得估计的均方误差为

$$
P ^ {\prime} = R (0) - \frac {R ^ {2} (0)}{R (0) + C _ {V}}
$$

所以

$$
P ^ {\prime} - P = \frac {R ^ {2} (T) C _ {V} ^ {2}}{[ R (0) + C _ {V} ] \{[ R (0) + C _ {V} ] ^ {2} - R ^ {2} (T) \}}
$$

因为 $R(0) > 0, C_V > 0, R(0) \geqslant R(T)$ ，所以只要 $R(T) \neq 0$ ，即 $X(t_k)$ 与 $X(t_{k-1})$ 之间有依从关系，则上式值总为正值，这说明同时使用 $Z(t_k)$ 和 $Z(t_{k-1})$ 比单独使用 $Z(t_k)$ 估计 $X(t_k)$ 的精度要高。因此尽量多地合理利用量测量能有效提高估计精度。

## 2.1.7 维纳滤波

## 1. 维纳滤波所要解决的问题

设有用信号 $S(t)$ 和干扰信号 $N(t)$ 都是随机信号，现要求从混有干扰的量测量 $Z(t)=S(t)+N(t)$ 中提取出有用信号 $S(t)$ 。由于 $S(t)$ 和 $N(t)$ 都是随机信号，没有确定的频谱，所以不能采用常规滤波的方法来滤除干扰。

设 $S(t)$ 和 $N(t)$ 都是平稳过程，且互相独立，自相关函数及功率谱密度 $R_{S}(\tau), R_{N}(\tau)$ ， $G_{S}(\omega), G_{N}(\omega)$ 都已知。

若无干扰,则滤波系统的输出即为理想输出:

$$
I (t) = \int_ {- \infty} ^ {\infty} h _ {l} (\tau) S (t - \tau) \mathrm{d} \tau\tag{2.1.68}
$$

式中 $h_I(\tau)$ 为理想滤波器的单位脉冲响应，其傅里叶变换 $H(\mathrm{j}\omega) = \mathcal{F}\{h_I(t)\}$ 就是理想滤波器的频率响应。若要求 $I(t) = S(t)$ ，即滤波的情况，则 $h_I(t) = \delta (t), H(\mathrm{j}\omega) = 1$ ；若要求 $I(t) = S(t + T)$ ，即预报的情况，则 $h_I(t) = \delta (t + T), H(\mathrm{j}\omega) = \mathrm{e}^{\mathrm{j}\omega T}$ 。

维纳滤波所要解决的问题是:求出一个物理可实现的线性定常系统的频率特性 $H(\mathrm{j}\omega)$ 或单位脉冲响应 $h(t)$ ，使滤波器的实际输出

$$
y (t) = \int_ {0} ^ {\infty} h (\tau) Z (t - \tau) \mathrm{d} \tau\tag{2.1.69}
$$

![](images/e9b110a10194e5b5d47075cea80eb0ff11d4b2723c83be9aea2a03479dff9fb8.jpg)

与理想输出 $I(t)$ 的误差 $\varepsilon(t)=I(t)-y(t)$ 的均方值达到最小，即

图 2.1.1 维纳滤波示意图

$$
E [ \varepsilon^ {2} (t) ] = E \{[ I (t) - y (t) ] ^ {2} \} = \min\tag{2.1.70}
$$

它可用图 2.1.1 来表示。

## 2. 维纳 - 霍甫方程

由于滤波器的实际输出为 $y(t)$ ，如式(2.1.69)所示，所以相对理想输出 $I(t)$ 的误差为

$$
\varepsilon (t) = I (t) - y (t) = I (t) - \int_ {0} ^ {\infty} h (\tau) Z (t - \tau) \mathrm{d} \tau
$$

平方误差为

$$
\varepsilon^ {2} (t) = I ^ {2} (t) - 2 \int_ {0} ^ {\infty} h (\tau) I (t) Z (t - \tau) \mathrm{d} \tau + \left[ \int_ {0} ^ {\infty} h (\tau) Z (t - \tau) \mathrm{d} \tau \right] \left[ \int_ {0} ^ {\infty} h (\tau_ {1}) Z (t - \tau_ {1}) \mathrm{d} \tau_ {1} \right]
$$

均方误差为

$$
\begin{array}{r l} E [ \varepsilon^ {2} (t) ] & = E [ I ^ {2} (t) ] - 2 \int_ {0} ^ {\infty} h (\tau) E [ I (t) Z (t - \tau) ] \mathrm{d} \tau + \\ & \quad \int_ {0} ^ {\infty} \int_ {0} ^ {\infty} h (\tau) h (\tau_ {1}) E [ Z (t - \tau) Z (t - \tau_ {1}) ] \mathrm{d} \tau \mathrm{d} \tau_ {1} \end{array}
$$

即

$$
\begin{array}{r l} E [ \varepsilon^ {2} (t) ] & = R _ {I} (0) - 2 \int_ {0} ^ {\infty} h (\tau) R _ {I Z} (\tau) \mathrm{d} \tau + \\ & \quad \int_ {0} ^ {\infty} \int_ {0} ^ {\infty} h (\tau) h (\tau_ {1}) R _ {Z} (\tau - \tau_ {1}) \mathrm{d} \tau \mathrm{d} \tau_ {1} \end{array}\tag{2.1.71}
$$

显然 $E[\varepsilon^2 (t)]$ 取决于 $h(t)$ ，为便于说明问题，记 $\overline{\varepsilon^2(h)} = E[\varepsilon^2 (t)]$ 。下面分析要使

$E[\varepsilon^{2}(t)]=\min,h(t)$ 应满足什么条件。

设 $g(t)$ 是任意的一个可微函数，它和 $h(t)$ 一样也满足条件 $g(t)=0(t<0)$ ，这样 $h(t)+\alpha g(t)$ 也是物理上可实现的滤波系统的脉冲响应函数（其中 $\alpha$ 为实变数）。如果 $h(t)$ 使 $\overline{\varepsilon^{2}(h)}=\min$ ，则对任一实数 $\alpha$ 及任一函数 $g(t)$ ，总有 $\overline{\varepsilon^{2}(h+\alpha g)}\geqslant\overline{\varepsilon^{2}(h)}$ ，即函数 $J(\alpha)=\overline{\varepsilon^{2}(h+\alpha g)}$ 在 $\alpha=0$ 处取得极小值。

以 $h(t) + \alpha g(t)$ 代替式(2.1.71)中的 $h(t)$ ，得

$$
\begin{array}{r l} J (\alpha) & = R _ {I} (0) - 2 \int_ {0} ^ {\infty} [ h (\tau) + \alpha g (\tau) ] R _ {I Z} (\tau) \mathrm{d} \tau + \\ & \quad \int_ {0} ^ {\infty} \int_ {0} ^ {\infty} [ h (\tau) + \alpha g (\tau) ] [ h (\tau_ {1}) + \alpha g (\tau_ {1}) ] R _ {Z} (\tau - \tau_ {1}) \mathrm{d} \tau \mathrm{d} \tau_ {1} \end{array}
$$

即

$$
\begin{array}{r l} J (\alpha) & = R _ {I} (0) - 2 \int_ {0} ^ {\infty} h (\tau) R _ {I Z} (\tau) \mathrm{d} \tau + \int_ {0} ^ {\infty} \int_ {0} ^ {\infty} h (\tau) h (\tau_ {1}) R _ {Z} (\tau - \tau_ {1}) \mathrm{d} \tau \mathrm{d} \tau_ {1} - \\ & 2 \alpha \int_ {0} ^ {\infty} g (\tau) R _ {I Z} (\tau) \mathrm{d} \tau + 2 \alpha \int_ {0} ^ {\infty} \int_ {0} ^ {\infty} h (\tau_ {1}) g (\tau) R _ {Z} (\tau - \tau_ {1}) \mathrm{d} \tau \mathrm{d} \tau_ {1} + \\ & \alpha^ {2} \int_ {0} ^ {\infty} \int_ {0} ^ {\infty} g (\tau) g (\tau_ {1}) R _ {Z} (\tau - \tau_ {1}) \mathrm{d} \tau \mathrm{d} \tau_ {1} \end{array}
$$

记上式右侧前三项为 $J(0)$ ，则上式可写为

$$
\begin{array}{r l} J (\alpha) & = J (0) - 2 \alpha \int_ {0} ^ {\infty} g (\tau) [ R _ {I Z} (\tau) - \int_ {0} ^ {\infty} h (\tau_ {1}) R _ {Z} (\tau - \tau_ {1}) \mathrm{d} \tau_ {1} ] \mathrm{d} \tau + \\ & \quad \alpha^ {2} \int_ {0} ^ {\infty} \int_ {0} ^ {\infty} g (\tau) g (\tau_ {1}) R _ {Z} (\tau - \tau_ {1}) \mathrm{d} \tau \mathrm{d} \tau_ {1} \end{array}\tag{2.1.72}
$$

由于 $J(\alpha)$ 在 $\alpha = 0$ 处取得极小值的必要条件是

$$
\frac {\mathrm{d}}{\mathrm{d} \alpha} J (\alpha) \Bigg | _ {\alpha = 0} = 0
$$

所以有

$$
\frac {\mathrm{d}}{\mathrm{d} \alpha} J (\alpha) \Bigg | _ {\alpha = 0} = - 2 \int_ {0} ^ {\infty} g (\tau) \left[ R _ {I Z} (\tau) - \int_ {0} ^ {\infty} h (\tau_ {1}) R _ {Z} (\tau - \tau_ {1}) \mathrm{d} \tau_ {1} \right] \mathrm{d} \tau = 0
$$

又由于 $g(t) = 0 (t < 0)$ ，且 $g(t)$ 为任意函数，因此要使上式成立，相当于对于一切 $\tau \geqslant 0$ ，方括号中的值恒为零。于是得 $\overline{\varepsilon^2(h)} = \min$ 的必要条件为

$$
R _ {I Z} (\tau) - \int_ {0} ^ {\infty} h (\tau_ {1}) R _ {Z} (\tau - \tau_ {1}) \mathrm{d} \tau_ {1} = 0 \qquad \tau \geqslant 0\tag{2.1.73}
$$

事实上，式(2.1.73)也是 $\varepsilon^{2}(h)=\min$ 的充分条件，下面说明之。

设式 $(2.1.73)$ 成立，则式 $(2.1.72)$ 可写成

$$
\begin{array}{r l} J (\alpha) & = J (0) + \alpha^ {2} \int_ {0} ^ {\infty} \int_ {0} ^ {\infty} g (\tau) g (\tau_ {1}) R _ {Z} (\tau - \tau_ {1}) \mathrm{d} \tau \mathrm{d} \tau_ {1} = \\ & J (0) + \alpha^ {2} E \left\{\int_ {0} ^ {\infty} g (\tau) Z (t - \tau) \mathrm{d} \tau \int_ {0} ^ {\infty} g (\tau_ {1}) Z (t - \tau_ {1}) \mathrm{d} \tau_ {1} \right\} = \\ & J (0) + \alpha^ {2} E \left\{\left[ \int_ {0} ^ {\infty} g (\tau) Z (t - \tau) \mathrm{d} \tau \right] ^ {2} \right\} \end{array}
$$

注意到上式的最后一项为非负项,所以有

$$
J (\alpha) \geqslant J (0)
$$

即

$$
\overline {{\varepsilon^ {2} (h + \alpha g)}} \geqslant \overline {{\varepsilon^ {2} (h)}}
$$

这说明,当滤波系统的单位脉冲响应 $h(t)$ 满足式(2.1.73)时,滤波系统的实际输出

$$
y (t) = \int_ {0} ^ {\infty} h (\tau) Z (t - \tau) \mathrm{d} \tau
$$

相对理想输出 $I(t)$ 的均方误差达到最小，因此式(2.1.73) 又是 $\varepsilon^{2}(h)=\min$ 的充分条件。

综上所述， $\varepsilon^2 (h) = \min$ 的充要条件为

$$
R _ {I Z} (\tau) - \int_ {0} ^ {\infty} h (\tau_ {1}) R _ {Z} (\tau - \tau_ {1}) \mathrm{d} \tau_ {1} = 0 \qquad \tau \geqslant 0
$$

该方程称为维纳-霍甫方程, 是关于最佳单位脉冲响应 $h(t)$ 的积分方程, 其中 $R_{IZ}(\tau)$ 和 $R_{Z}(\tau)$ 按下述两式确定:

$$
\begin{array}{r l} & {R _ {I Z} (\tau) = E [ I (t) Z (t - \tau) ] =} \\ & {\qquad E \Big \{\left[ \int_ {- \infty} ^ {\infty} h _ {I} (\lambda) S (t - \lambda) \mathrm{d} \lambda \right] Z (t - \tau) \Big \} =} \\ & {\qquad \int_ {- \infty} ^ {\infty} h _ {I} (\lambda) E \{S (t - \lambda) [ S (t - \tau) + N (t - \tau) ] \} \mathrm{d} \lambda =} \\ & {\qquad \int_ {- \infty} ^ {\infty} h _ {I} (\lambda) [ R _ {S} (\tau - \lambda) + R _ {S N} (\tau - \lambda) ] \mathrm{d} \lambda =} \\ & {\qquad h _ {I} (\tau) * [ R _ {S} (\tau) + R _ {S N} (\tau) ]} \\ & {R _ {Z} (\tau) = E [ Z (t) Z (t - \tau) ] = E \{[ S (t) + N (t) ] [ S (t - \tau) + N (t - \tau) ] \} =} \\ & {\qquad R _ {S} (\tau) + R _ {N} (\tau) + R _ {S N} (\tau) + R _ {N S} (\tau)} \end{array}
$$

若对 $R_{IZ}(\tau)$ 和 $R_Z(\tau)$ 作傅里叶变换，则可得 $I(t)$ 与 $Z(t)$ 的互功率谱 $G_{IZ}(\omega)$ 和 $Z(t)$ 的自功率谱 $G_Z(\omega)$ ，即

$$
G _ {I Z} (\omega) = \mathcal {F} \{R _ {I Z} (\tau) \} = H _ {I} (\mathrm{j} \omega) [ G _ {S} (\omega) + G _ {S N} (\omega) ]\tag{2.1.74}
$$

$$
G _ {Z} (\omega) = \mathcal {F} \{R _ {Z} (\tau) \} = G _ {S} (\omega) + G _ {N} (\omega) + G _ {S N} (\omega) + G _ {N S} (\omega)\tag{2.1.75}
$$

式中， $H_{1}(j\omega)$ 为滤波器的理想频率响应，由滤波器的设计要求确定。

3. 维纳－霍甫方程的求解(设计维纳滤波器的一般方法)

式(2.1.73)中, $\tau\geqslant0$ 是滤波系统的物理可实现条件,正是这一限制给求解式(2.1.73)带来了一定的困难,下面说明这一点。

暂且假设没有 $\tau \geqslant 0$ 的限制，即当 $\tau < 0$ 时式(2.1.73)也成立，即

$$
R _ {I Z} (\tau) = \int_ {- \infty} ^ {\infty} h (\tau_ {1}) R _ {Z} (\tau - \tau_ {1}) \mathrm{d} \tau_ {1} = h (\tau) * R _ {Z} (\tau) \quad (- \infty <   \tau <   \infty)\tag{2.1.76}
$$

对上式两边作傅里叶变换,得

$$
G _ {I Z} (\omega) = H (\mathrm{j} \omega) G _ {Z} (\omega)
$$

所以

$$
\begin{array}{r} H (\mathrm{j} \omega) = \frac {G _ {\mathrm{IZ}} (\omega)}{G _ {\mathrm{Z}} (\omega)} \\ G _ {\mathrm{IZ}} (\omega) = \mathcal {F} \{R _ {\mathrm{IZ}} (\tau) \}, G _ {\mathrm{Z}} (\omega) = \mathcal {F} \{R _ {\mathrm{Z}} (\tau) \} \end{array}\tag{2.1.77}
$$

式中

滤波系统的单位脉冲响应为

$$
h (t) = \mathcal {F} ^ {- 1} \left\{H (\mathrm{j} \omega) \right\} = \frac {1}{2 \pi} \int_ {- \infty} ^ {\infty} \frac {G _ {I Z} (\omega)}{G _ {Z} (\omega)} \mathrm{e} ^ {\mathrm{j} \omega t} \mathrm{d} \omega\tag{2.1.78}
$$

可是，从式(2.1.73)的推导过程知，只有当 $\tau \geqslant 0$ 时式(2.1.73)才成立，而当 $\tau < 0$ 时式(2.1.73)并不成立，所以对式(2.1.76)作傅里叶变换时，积分限只能取 $[0, \infty)$ ，而不能取 $(- \infty, \infty)$ ，即

$$
\begin{array}{r l} & {\int_ {0} ^ {\infty} R _ {I Z} (\tau) \mathrm{e} ^ {- \mathrm{j} \omega \tau} \mathrm{d} \tau = \int_ {0} ^ {\infty} \left[ \int_ {0} ^ {\infty} h (\tau_ {1}) R _ {Z} (\tau - \tau_ {1}) \mathrm{d} \tau_ {1} \right] \mathrm{e} ^ {- \mathrm{j} \omega \tau} \mathrm{d} \tau =} \\ & {\qquad \int_ {0} ^ {\infty} h (\tau_ {1}) \left[ \int_ {0} ^ {\infty} R _ {Z} (\tau - \tau_ {1}) \mathrm{e} ^ {- \mathrm{j} \omega \tau} \mathrm{d} \tau \right] \mathrm{d} \tau_ {1}} \end{array}
$$

上式方括号中是对 $\tau$ 积分， $\tau_{1}$ 可看做常量，令 $\tau - \tau_{1} = \lambda$ ，则 $d\tau = d\lambda, \lambda$ 的变化区间为 $[- \tau_{1}, \infty)$ ，因此有

$$
\int_ {0} ^ {\infty} R _ {I Z} (\tau) \mathrm{e} ^ {- \mathrm{j} \omega \tau} \mathrm{d} \tau = \int_ {0} ^ {\infty} h (\tau_ {1}) \left[ \int_ {- \tau_ {1}} ^ {\infty} R _ {Z} (\lambda) \mathrm{e} ^ {- \mathrm{j} \omega \lambda} \mathrm{d} \lambda \right] \mathrm{e} ^ {- \mathrm{j} \omega \tau_ {1}} \mathrm{d} \tau_ {1}
$$

由于上式方括号中的积分是 $\tau_{1}$ 的函数，上式无法写成谱密度的形式，因此不具备式(2.1.77)和式(2.1.78)确定的解。

式(2.1.73)可采用谱分解的方法求解。在讨论该法之前,先介绍谱分解中要使用的定理,即傅里叶变换的解析性定理。

定理 2.6 （解析性定理）

如果

$$
f (t) \left\{ \begin{array}{l l} = 0, t \geqslant 0 \\ \neq 0, t <   0 \end{array} \right.
$$

且其傅里叶变换 $F(\omega) = \mathcal{F}\{f(t)\}$ 存在，则 $F(\omega)$ 在 $[\omega]$ 的上半平面内解析有界。若 $F(\omega)$ 是 $\omega$ 的有理分式，则 $F(\omega)$ 仅在 $[\omega]$ 的下半平面内有极点，即极点 $\omega_{i} = u_{i} + \mathrm{j}v_{i}$ 中， $v_{i} < 0$ ；反之，若 $F(\omega)$ 在 $[\omega]$ 的上半平面内解析有界，则

$$
f (t) = \mathcal {F} ^ {- 1} \{F (\omega) \} \left\{ \begin{array}{l l} = 0, t \geqslant 0 \\ \neq 0, t <   0 \end{array} \right.
$$

类似地，如果 $f(t)\left\{ \begin{array}{l} \neq 0, (t \geqslant 0) \\ = 0, (t < 0) \end{array} \right.$ ，且 $F(\omega) = \mathcal{F}\{f(t)\}$ 存在，则 $F(\omega)$ 在 $[\omega]$ 的下半平面内解析有界。若 $F(\omega)$ 是 $\omega$ 的有理分式，则 $F(\omega)$ 仅在上半平面内有极点，即极点 $\omega_i = u_i + \mathrm{j}v_i$ 中， $v_i \geqslant 0$ 。反之，若 $F(\omega)$ 在 $[\omega]$ 的下半平面内解析有界，则

$$
f (t) = \mathcal {F} ^ {- 1} \{F (\omega) \} \left\{ \begin{array}{l l} \neq 0, t \geqslant 0 \\ = 0, t <   0 \end{array} \right.
$$

由于篇幅有限,定理2.6将不加证明地应用,详细证明请读者参阅文献[60]。

将式(2.1.73)写成

$$
f (t) = \int_ {0} ^ {\infty} h (\tau) R _ {Z} (t - \tau) \mathrm{d} \tau - R _ {I Z} (t)
$$

显然

$$
f (t) \left\{ \begin{array}{l l} = 0, t \geqslant 0 \\ \neq 0, t <   0 \end{array} \right.
$$

根据定理 2.6, $F(\omega) = \mathcal{F}\{f(t)\}$ 在 $[\omega]$ 的上半平面内解析有界。而

$$
\begin{array}{r l} F (\omega) = \mathcal {F} \{f (t) \} & = \int_ {- \infty} ^ {\infty} \left[ \int_ {0} ^ {\infty} h (\tau) R _ {Z} (t - \tau) \mathrm{d} \tau - R _ {I Z} (t) \right] \mathrm{e} ^ {- \mathrm{j} \omega t} \mathrm{d} t = \\ & \quad \int_ {0} ^ {\infty} h (\tau) \left[ \int_ {- \infty} ^ {\infty} R _ {Z} (t - \tau) \mathrm{e} ^ {- \mathrm{j} \omega t} \mathrm{d} t \right] \mathrm{d} \tau - \int_ {- \infty} ^ {\infty} R _ {I Z} (t) \mathrm{e} ^ {- \mathrm{j} \omega t} \mathrm{d} t \end{array}
$$

在上式的第一项积分中，令 $t - \tau = \lambda$ 。由于 $t: -\infty \to \infty, \tau: 0 \to \infty$ ，所以 $\lambda: -\infty \to \infty$ 。并且由于方括号中对 $t$ 积分时， $\tau$ 为常量，所以 $\mathrm{d}t = \mathrm{d}\lambda$ 。上式又可写成

$$
\begin{array}{r l} F (\omega) & = \int_ {0} ^ {\infty} h (\tau) \left[ \int_ {- \infty} ^ {\infty} R _ {Z} (\lambda) \mathrm{e} ^ {- \mathrm{j} \omega (\lambda + \tau)} \mathrm{d} \lambda \right] \mathrm{d} \tau - G _ {I Z} (\omega) = \\ & \quad \int_ {0} ^ {\infty} h (\tau) \mathrm{e} ^ {- \mathrm{j} \omega \tau} \mathrm{d} \tau \int_ {- \infty} ^ {\infty} R _ {Z} (\lambda) \mathrm{e} ^ {- \mathrm{j} \omega \lambda} \mathrm{d} \lambda - G _ {I Z} (\omega) = \\ & H (\mathrm{j} \omega) G _ {Z} (\omega) - G _ {I Z} (\omega) \end{array}\tag{2.1.79}
$$

为了解出 $H(j\omega)$ ，考虑 $G_{Z}(\omega)$ 可分解成下述形式的特殊情况：

$$
G _ {Z} (\omega) = \Psi_ {\mathrm{下解}} (\omega) \Psi_ {\mathrm{上解}} (\omega)\tag{2.1.80}
$$

式中， $\Psi_{\text{下解}}(\omega)$ 在 $[\omega]$ 的下半平面内解析有界，且没有零极点； $\Psi_{\text{上解}}(\omega)$ 在 $[\omega]$ 的上半平面内解析有界，且没有零极点。

式(2.1.80)在实际系统中一般都能满足,特别是当 $Z(t)$ 为零均值且方差有限的平稳随机过程时, $G_{Z}(\omega)$ 为 $\omega$ 的有理分式, $\Psi_{\text{下解}}(\omega)$ 和 $\Psi_{\text{上解}}(\omega)$ 互为共轭。

式(2.1.79)两边除以 $\Psi_{\text{上解}}(\omega)$ ，得

$$
\frac {F (\omega)}{\Psi_ {\mathrm{上解}} (\omega)} = H (\mathrm{j} \omega) \Psi_ {\mathrm{下解}} (\omega) - \frac {G _ {I Z} (\omega)}{\Psi_ {\mathrm{上解}} (\omega)}\tag{2.1.81}
$$

显然 $\frac{F(\omega)}{\Psi_{\text{上解}}(\omega)}$ 在 $[\omega]$ 的上半平面内解析有界，根据定理2.6，有

$$
\mathcal {F} ^ {- 1} \left\{\frac {F (\omega)}{\Psi_ {\mathrm{上解}} (\omega)} \right\} = \alpha (t) \left\{ \begin{array}{l l} = 0, t \geqslant 0 \\ \neq 0, t <   0 \end{array} \right.
$$

所以当 $t \geqslant 0$ 时

(2.1.82)

因为

$$
\begin{array}{r l} & {\mathcal {F} ^ {- 1} \left\{H (\mathrm{j} \omega) \Psi_ {\text {下解}} (\omega) \right\} = \mathcal {F} ^ {- 1} \left\{\frac {G _ {\mathrm{IZ}} (\omega)}{\Psi_ {\text {上解}} (\omega)} \right\}} \\ & {\qquad h (t) \left\{ \begin{array}{l l} {\neq 0, t \geqslant 0} \\ {= 0, t <   0} \end{array} \right.} \\ & {\qquad H (\mathrm{j} \omega) = \mathcal {F} \{h (t) \}} \end{array}
$$

所以

在 $[\omega]$ 的下半平面内解析有界。由此， $H(\mathrm{j}\omega)\Psi_{\text{下解}}(\omega)$ 在下半平面内解析有界，根据定理2.6

$$
\mathcal {F} ^ {- 1} \left\{H (\mathrm{j} \omega) \Psi_ {\text {下解}} (\omega) \right\} = \beta (t) \left\{ \begin{array}{l l} \neq 0, t \geqslant 0 \\ = 0, t <   0 \end{array} \right.
$$

将式 $(2.1.82)$ 代入上式,得

$$
\beta (t) = \left\{ \begin{array}{l l} \mathcal {F} ^ {- 1} \left\{\frac {G _ {I Z} (\omega)}{\Psi_ {\mathrm{上解}} (\omega)} \right\} = \frac {1}{2 \pi} \int_ {- \infty} ^ {\infty} \frac {G _ {I Z} (\omega)}{\Psi_ {\mathrm{上解}} (\omega)} \mathrm{e} ^ {\mathrm{j} \omega t} \mathrm{d} \omega , t \geqslant 0 \\ 0, t <   0 \end{array} \right.\tag{2.1.83}
$$

所以 $\beta (t)$ 的傅里叶变换为

$$
\begin{array}{r l} & {H (\mathrm{j} \omega) \Psi_ {\mathrm{下解}} (\omega) = \int_ {- \infty} ^ {\infty} \beta (t) \mathrm{e} ^ {- \mathrm{j} \omega t} \mathrm{d} t = \int_ {0} ^ {\infty} \beta (t) \mathrm{e} ^ {- \mathrm{j} \omega t} \mathrm{d} t =} \\ & {\qquad \int_ {0} ^ {\infty} \left[ \frac {1}{2 \pi} \int_ {- \infty} ^ {\infty} \frac {G _ {I Z} (\omega)}{\Psi_ {\mathrm{上解}} (\omega)} \mathrm{e} ^ {\mathrm{j} \omega t} \mathrm{d} \omega \right] \mathrm{e} ^ {- \mathrm{j} \omega t} \mathrm{d} t} \\ & {\qquad H (\mathrm{j} \omega) = \frac {1}{2 \pi \Psi_ {\mathrm{下解}} (\omega)} \int_ {0} ^ {\infty} \left[ \int_ {- \infty} ^ {\infty} \frac {G _ {I Z} (\omega)}{\Psi_ {\mathrm{上解}} (\omega)} \mathrm{e} ^ {\mathrm{j} \omega t} \mathrm{d} \omega \right] \mathrm{e} ^ {- \mathrm{j} \omega t} \mathrm{d} t} \end{array}\tag{2.1.84}
$$

式(2.1.84)就是式(2.1.73)的解,即维纳滤波器频率特性的求解公式,从表达式可看出

运算是十分繁杂的,这也是维纳滤波器设计过程中的困难所在。

综上所述,可归纳出维纳滤波器的一般设计步骤:

(1) 按式(2.1.74) 和式(2.1.75) 计算 $G_{IZ}(\omega)$ 和 $G_{Z}(\omega)$ ;

(2) 按式(2.1.80) 分解 $G_{Z}(\omega)$ ;

(3) 按式(2.1.83)求解 $\beta(t)$ ;

(4) 求出 $\beta(t)$ 的傅里叶变换

$$
B (\mathrm{j} \omega) = \int_ {0} ^ {\infty} \beta (t) \mathrm{e} ^ {- \mathrm{j} \omega t} \mathrm{d} t
$$

(5) 维纳滤波器的频率特性为

$$
H (\mathrm{j} \omega) = \frac {B (\mathrm{j} \omega)}{\Psi_ {\mathrm{下解}} (\omega)}
$$

例2-10 设有用信号 $S(t)$ 的功率谱为 $G_{\mathrm{S}}(\omega) = \frac{1}{1 + \omega^2}$ ，干扰 $N(t)$ 为白噪声过程，功率谱为 $G_{\mathrm{N}}(\omega) = \sigma^2$ ，有用信号和干扰互不相关。要求设计维纳滤波器，该滤波器能抑制干扰 $N(t)$ ，并能对信号 $S$ 作超前 $T$ 的预测，即 $t$ 时刻能预测出 $t + T$ 时刻的值。

解

$$
\begin{array}{r l} & {(1) G _ {Z} (\omega) = G _ {S} (\omega) + G _ {N} (\omega) = \frac {1 + \sigma^ {2} + \sigma^ {2} \omega^ {2}}{1 + \omega^ {2}}} \\ & {\quad H _ {I} (\mathrm{j} \omega) = \mathrm{e} ^ {\mathrm{j} \omega T}} \\ & {\quad G _ {I Z} (\omega) = \frac {\mathrm{e} ^ {\mathrm{j} \omega T}}{1 + \omega^ {2}}} \\ & {(2) G _ {Z} (\omega) = \frac {1 + \sigma^ {2} + \sigma^ {2} \omega^ {2}}{1 + \omega^ {2}} = \frac {\sqrt {1 + \sigma^ {2}} + \mathrm{j} \sigma \omega}{1 + \mathrm{j} \omega} \frac {\sqrt {1 + \sigma^ {2}} - \mathrm{j} \sigma \omega}{1 - \mathrm{j} \omega}} \\ & {\quad \Psi_ {\text {下解}} (\omega) = \frac {\sqrt {1 + \sigma^ {2}} + \mathrm{j} \sigma \omega}{1 + \mathrm{j} \omega}, \quad \Psi_ {\text {上解}} (\omega) = \frac {\sqrt {1 + \sigma^ {2}} - \mathrm{j} \sigma \omega}{1 - \mathrm{j} \omega}} \end{array}
$$

(3) $t \geqslant 0$ 时，

$$
\begin{array}{r l} \beta (t) & = \frac {1}{2 \pi} \int_ {- \infty} ^ {\infty} \frac {\mathrm{e} ^ {\mathrm{j} \omega T}}{(1 + \mathrm{j} \omega) (1 - \mathrm{j} \omega)} \frac {1 - \mathrm{j} \omega}{\sqrt {1 + \sigma^ {2}} - \mathrm{j} \sigma \omega} \mathrm{e} ^ {\mathrm{j} \omega t} \mathrm{d} \omega = \\ & \frac {1}{2 \pi} \int_ {- \infty} ^ {\infty} \frac {1}{(1 + \mathrm{j} \omega) (\sqrt {1 + \sigma^ {2}} - \mathrm{j} \sigma \omega)} \mathrm{e} ^ {\mathrm{j} \omega (t + T)} \mathrm{d} \omega = \\ & \frac {1}{\sigma + \sqrt {1 + \sigma^ {2}}} \frac {1}{2 \pi} \int_ {- \infty} ^ {\infty} \left(\frac {1}{1 + \mathrm{j} \omega} + \frac {\sigma}{\sqrt {1 + \sigma^ {2}} - \mathrm{j} \sigma \omega}\right) \mathrm{e} ^ {\mathrm{j} \omega (t + T)} \mathrm{d} \omega = \\ & \frac {\mathrm{e} ^ {- (t + T)}}{\sigma + \sqrt {1 + \sigma^ {2}}} \end{array}
$$

$t < 0$ 时， $\beta(t) = 0$ 。

$$
B (\mathrm{j} \omega) = \int_ {0} ^ {\infty} \beta (t) \mathrm{e} ^ {- \mathrm{j} \omega t} \mathrm{d} t = \frac {\mathrm{e} ^ {- T}}{\sigma + \sqrt {1 + \sigma^ {2}}} \int_ {0} ^ {\infty} \mathrm{e} ^ {- (1 + \mathrm{j} \omega) t} \mathrm{d} t = \frac {\mathrm{e} ^ {- T}}{\sigma + \sqrt {1 + \sigma^ {2}}} \frac {1}{1 + \mathrm{j} \omega}
$$

(5) 维纳滤波器的频率特性为

$$
H (\mathrm{j} \omega) = \frac {B (\mathrm{j} \omega)}{\Psi_ {\mathrm{下解}} (\omega)} = \frac {\mathrm{e} ^ {- \mathrm{T}}}{\sigma + \sqrt {1 + \sigma^ {2}}} \frac {1}{1 + \mathrm{j} \omega} \frac {1 + \mathrm{j} \omega}{\sqrt {1 + \sigma^ {2}} + \mathrm{j} \sigma \omega} =
$$

$$
\frac {\mathrm{e} ^ {- \mathrm{T}}}{\sigma + \sqrt {1 + \sigma^ {2}}} \frac {1}{\sqrt {1 + \sigma^ {2}} + \mathrm{j} \sigma \omega}
$$

4. 维纳滤波器的伯特－香农(Bode-Shannon)设计法

在式(2.1.84)中,如果 $\frac{G_{IZ}(\omega)}{\Psi_{\text{上解}}(\omega)}$ 可分解为

$$
\frac {G _ {I Z} (\omega)}{\Psi_ {\mathrm{上解}} (\omega)} = \left[ \frac {G _ {I Z} (\omega)}{\Psi_ {\mathrm{上解}} (\omega)} \right] _ {\mathrm{上极}} + \left[ \frac {G _ {I Z} (\omega)}{\Psi_ {\mathrm{上解}} (\omega)} \right] _ {\mathrm{下极}}\tag{2.1.85}
$$

式中， $[\cdot]_{上极}$ 表示只在 $[\omega]$ 的上半平面内有极点； $[\cdot]_{下极}$ 表示只在 $[\omega]$ 的下半平面内有极点。则， $[\cdot]_{上极}$ 在 $[\omega]$ 的下半平面内解析有界； $[\cdot]_{下极}$ 在 $[\omega]$ 的上半平面内解析有界。

$$
\frac {1}{2 \pi} \int_ {- \infty} ^ {\infty} \left[ \frac {G _ {I Z} (\omega)}{\varPsi_ {\mathrm{上解}} (\omega)} \right] _ {\mathrm{上极}} \mathrm{e} ^ {\mathrm{j} \omega t} \mathrm{d} \omega = \xi (t) \left\{ \begin{array}{l l} {{\neq 0, t \geqslant 0}} \\ {{= 0, t <   0}} \end{array} \right.\tag{2.1.86}
$$

$$
\frac {1}{2 \pi} \int_ {- \infty} ^ {\infty} \left[ \frac {G _ {I Z} (\omega)}{\Psi_ {\mathrm{上解}} (\omega)} \right] _ {\mathrm{下极}} \mathrm{e} ^ {\mathrm{j} \omega t} \mathrm{d} \omega = \zeta (t) \left\{ \begin{array}{l l} = 0, t \geqslant 0 \\ \neq 0, t <   0 \end{array} \right.\tag{2.1.87}
$$

因此式(2.1.84)为

$$
\begin{array}{r l} H (\mathrm{j} \omega) & = \frac {1}{\Psi_ {\text {下解}} (\omega)} \int_ {0} ^ {\infty} [ \xi (t) + \zeta (t) ] \mathrm{e} ^ {- \mathrm{j} \omega t} \mathrm{d} t = \frac {1}{\Psi_ {\text {下解}} (\omega)} \int_ {- \infty} ^ {\infty} \xi (t) \mathrm{e} ^ {- \mathrm{j} \omega t} \mathrm{d} t = \\ & \quad \frac {1}{\Psi_ {\text {下解}} (\omega)} \bigg [ \frac {G _ {I Z} (\omega)}{\Psi_ {\text {上解}} (\omega)} \bigg ] _ {\text {上极}} \end{array}\tag{2.1.88}
$$

式(2.1.88)说明,如果 $\frac{G_{IZ}(\omega)}{\Psi_{\text{上解}}(\omega)}$ 是有理分式,即它可分解成仅在上半平面内有极点和仅在下半平面有极点的两部分,则维纳滤波器的频率特性求取方法可以简化。下面具体说明之。

设有用信号 $S(t)$ 和干扰 $N(t)$ 均为平稳随机过程，且互相独立，则

$$
\begin{array}{r l} R _ {Z} (\tau) & = E [ Z (t + \tau) Z (t) ] = E \{[ S (t + \tau) + N (t + \tau) ] [ S (t) + N (t) ] \} = \\ & R _ {S} (\tau) + R _ {N} (\tau) \end{array}\tag{2.1.89}
$$

所以

$$
\begin{array}{r l} & G _ {Z} (\omega) = G _ {S} (\omega) + G _ {N} (\omega) \\ & R _ {I Z} (\tau) = E [ I (t) Z (t - \tau) ] = E \left[ \int_ {- \infty} ^ {\infty} h _ {I} (\lambda) S (t - \lambda) \mathrm{d} \lambda Z (t - \tau) \right] = \\ & \int_ {- \infty} ^ {\infty} h _ {I} (\lambda) \{E [ S (t - \lambda) S (t - \tau) + S (t - \lambda) N (t - \lambda) ] \} \mathrm{d} \lambda = \\ & \int_ {- \infty} ^ {\infty} h _ {I} (\lambda) R _ {S} (\tau - \lambda) \mathrm{d} \lambda = h _ {I} (\tau) * R _ {S} (\tau) \end{array}\tag{2.1.90}
$$

所以

$$
G _ {I Z} (\omega) = H _ {l} (\mathrm{j} \omega) G _ {S} (\omega)
$$

对于滤波问题, $H_{I}(j\omega)=1$

$$
\begin{array}{r} {G _ {I Z} (\omega) = G _ {S} (\omega)} \\ {\Psi_ {\mathrm{上解}} (\omega) \Psi_ {\mathrm{下解}} (\omega) = G _ {Z} (\omega)} \end{array}\tag{2.1.91}
$$

又由于

所以

$$
\Psi_ {\mathrm{上解}} (\omega) = \frac {G _ {Z} (\omega)}{\Psi_ {\mathrm{下解}} (\omega)} = \frac {G _ {S} (\omega) + G _ {N} (\omega)}{\Psi_ {\mathrm{下解}} (\omega)}\tag{2.1.92}
$$

将式(2.1.91)、式(2.1.92)代入式(2.1.88)，得

$$
\begin{array}{r l} H (\mathrm{j} \omega) = & \frac {1}{\Psi_ {\mathrm{下解}} (\omega)} \left[ \frac {G _ {S} (\omega)}{\frac {G _ {S} (\omega) + G _ {N} (\omega)}{\Psi_ {\mathrm{下解}} (\omega)}} \right] _ {\mathrm{上极}} = \\ & \frac {1}{\Psi_ {\mathrm{下解}} (\omega)} \left[ \frac {G _ {S} (\omega)}{G _ {S} (\omega) + G _ {N} (\omega)} \cdot \Psi_ {\mathrm{下解}} (\omega) \right] _ {\mathrm{上极}} \end{array}\tag{2.1.93}
$$

如果能将式(2.1.93)的右边写成关于 $j\omega$ 的表达式，则仅需将 $j\omega$ 换成拉普拉斯算子p即可获得滤波器的传递函数。但 $[\cdot]_{上极}$ 中仅在 $[\omega]$ 的上半平面内有极点的条件又转化成怎样的条件呢？

设

$$
\begin{array}{r l} & {\Psi_ {\mathrm{下解}} (\omega) = H _ {1} (\mathrm{j} \omega)} \\ & {\frac {G _ {S} (\omega)}{G _ {S} (\omega) + G _ {N} (\omega)} = H _ {0} (\mathrm{j} \omega)} \\ & {H _ {1} (\mathrm{j} \omega) H _ {0} (\mathrm{j} \omega) = H _ {2} (\mathrm{j} \omega)} \end{array}
$$

$\left[\frac{G_S(\omega)}{G_S(\omega) + G_N(\omega)} \cdot \Psi_{\text{下解}}(\omega)\right]_{\text{上极}}$ 的任一极点为 $\omega_i = u_i + \mathrm{j}v_i$ 。

由于 $\omega_{i}$ 仅位于 $[\omega]$ 的上半平面内，即 $v_{i} > 0$ ，当以 $p$ 代替 $\mathrm{j}\omega$ 时，[ω]平面内的极点 $\omega_{i} = u_{i} + \mathrm{j}v_{i}$ 就变成[p]平面内的极点 $p_i = \mathrm{j}\omega_i = \mathrm{j}(u_i + \mathrm{j}v_i) = -v_i + \mathrm{j}u_i$ ，而 $-v_{i} < 0$ ，所以位于[ω]上半平面内的极点被变换成位于[p]左半平面内的极点。这样式(2.1.93)改写为

$$
H (p) = \frac {1}{H _ {1} (p)} [ H _ {0} (p) \bullet H _ {1} (p) ] _ {\mathrm{左极}}\tag{2.1.94}
$$

式中 $[\cdot]_{\text{左极}}$ 表示在 $H_0(p)H_1(p)$ 的部分分式展开式中只取极点位于 $[p]$ 平面左侧的部分。

式(2.1.94)确定的 $H(p)$ 就是维纳滤波器的传递函数, 此设计方法称为伯特-香农设计法。

根据上述分析,可归纳出采用伯特-香农法设计维纳滤波器的步骤如下:

(1) 由 $G_{S}(\omega) + G_{N}(\omega) = |H_{1}(j\omega)|^{2}$ 确定出 $H_{1}(j\omega)$ 和 $H_{1}(p)$ ;

(2) 计算 $H_{2}(j\omega)=H_{0}(j\omega)H_{1}(j\omega)$ ，其中

$$
H _ {0} (\mathrm{j} \omega) = \frac {G _ {S} (\omega)}{G _ {S} (\omega) + G _ {N} (\omega)}
$$

根据 $H_{2}(j\omega)$ 确定出 $H_{2}(p)$ ;

(3) 将 $H_{2}(p)$ 分解成部分分式和；

（4）取 $H_{2}(p)$ 部分分式和式中极点仅位于左半平面内（包括虚轴）的所有项的和并记为 $H_{3}(p)$ ;

(5) 维纳滤波器的传递函数为

$$
H (p) = H _ {3} (p) H _ {1} ^ {- 1} (p)
$$

请读者注意,伯特-香农设计法的适用范围仅限于:

（1）有用信号 $S(t)$ 和干扰信号 $N(t)$ 均为零均值的平稳随机过程， $S(t)$ 和 $N(t)$ 互相独立，它们的功率谱均为有理分式；

(2) 适用于干扰的滤除,而不能解决预报和平滑问题。

例2-11 设计具有滤波功能的维纳滤波器，所处理的信号和干扰同例2-10。

解 由于 $G_{S} = \frac{1}{1 + \omega^{2}}, G_{N}(\omega) = \sigma^{2}$ , 有用信号和干扰的功率谱都是有理谱, 且要求滤波器

具有滤波功能,所以可采用伯特-香农法设计维纳滤波器。

$$
(1) G _ {S} (\omega) + G _ {N} (\omega) = \frac {1}{1 + \omega^ {2}} + \sigma^ {2} =
$$

$$
\frac {\sqrt {1 + \sigma^ {2}} + \mathrm{j} \sigma \omega}{1 + \mathrm{j} \omega} \frac {\sqrt {1 + \sigma^ {2}} - \mathrm{j} \sigma \omega}{1 - \mathrm{j} \omega} = H _ {1} (\mathrm{j} \omega) H _ {1} ^ {*} (\mathrm{j} \omega)
$$

$$
H _ {1} (\mathrm{j} \omega) = \frac {\sqrt {1 + \sigma^ {2}} + \mathrm{j} \sigma \omega}{1 + \mathrm{j} \omega}
$$

$$
H _ {1} (p) = \frac {\sqrt {1 + \sigma^ {2}} + \sigma p}{1 + p}
$$

$$
(2) H _ {0} (\mathrm{j} \omega) = \frac {\frac {1}{1 + \omega^ {2}}}{\frac {1}{1 + \omega^ {2}} + \sigma^ {2}} = \frac {1}{1 + \sigma^ {2} + \sigma^ {2} \omega^ {2}}
$$

$$
H _ {2} (\mathrm{j} \omega) = \frac {1}{1 + \sigma^ {2} + \sigma^ {2} \omega^ {2}} \frac {\sqrt {1 + \sigma^ {2}} + \mathrm{j} \sigma \omega}{1 + \mathrm{j} \omega} = \frac {1}{\sqrt {1 + \sigma^ {2}} - \mathrm{j} \sigma \omega} \frac {1}{1 + \mathrm{j} \omega}
$$

$$
H _ {2} (p) = \frac {1}{\sqrt {1 + \sigma^ {2}} - \sigma p} \frac {1}{1 + p}
$$

$$
H _ {2} (p) = \frac {1}{\sqrt {1 + \sigma^ {2}} + \sigma} \left(\frac {\sigma}{\sqrt {1 + \sigma^ {2}} - \sigma p} + \frac {1}{1 + p}\right) \tag {3}
$$

$$
H _ {3} (p) = \frac {1}{\sqrt {1 + \sigma^ {2}} + \sigma} \frac {1}{1 + p}
$$

(5) 维纳滤波器的传递函数为

$$
H (p) = \frac {1}{\sqrt {1 + \sigma^ {2}} + \sigma} \frac {1}{1 + p} \frac {1 + p}{\sqrt {1 + \sigma^ {2}} + \sigma p} = \frac {1}{\sqrt {1 + \sigma^ {2}} + \sigma} \frac {1}{\sqrt {1 + \sigma^ {2}} + \sigma p}
$$

## 2.1.8 几种最优估计的优缺点比较

由于各种估计满足的最优指标不一样,利用的信息不一样,所以适用的对象、达到的精度和计算的复杂性各不一样。

最小二乘估计法适用于对常值向量或随机向量的估计。由于使用的最优指标是使量测估计的精度达到最佳，估计中不必使用与被估计量有关的动态信息与统计信息，甚至连量测误差的统计信息也可不必使用，所以估计精度不高。这种方法的最大优点是算法简单，在对被估计量和量测误差缺乏了解的情况下仍能适用，所以至今仍被大量采用。

最小方差估计是所有估计中估计的均方误差为最小的估计，是所有估计中的最佳者。但这种最优估计只确定出了估计值是被估计量在量测空间上的条件均值这一抽象关系。一般情况下条件均值须通过条件概率密度求取，而条件概率密度的获取本身就非易事，所以按条件均值的一般求法求取最小方差估计是很困难的。

线性最小方差估计是所有线性估计中的最优者，只有当被估计量和量测量都服从正态分布时，线性最小方差估计才与最小方差估计等同，即在所有估计中也是最优的。线性最小方差估计可适用于随机过程的估计，估计过程中只须知道被估计量和量测量的一阶和二阶矩。对于平稳过程，这些一阶和二阶矩都为常值，但对非平稳过程，一阶和二阶矩随时间而变，必须确切知道每一估计时刻的一、二阶矩才能求出估计值，这种要求是十分苛刻的。所以线性最小方差估计适用于平稳过程而难以适用非平稳过程。由例2-9知，估计过程中不同时刻的量测量使用得越多，估计精度就越高，但矩阵求逆的阶数也越高，计算量也越大。

极大验后估计、贝叶斯估计、极大似然估计都与条件概率密度有关，除一些特殊的分布外，如正态分布情况，计算都十分困难。这些估计常用于故障检测和识别的算法中。

维纳滤波是线性最小方差估计的一种。维纳滤波器是一种线性定常系统，适用于对有用信号和干扰信号都是零均值的平稳随机过程的处理。设计维纳滤波器时必须知道有用信号和干扰信号的自功率谱和互功率谱。当功率谱都是有理分式时，可采用伯特-香农设计法求取具有滤波功能的维纳滤波器的传递函数。对于复杂的有用信号和干扰信号，功率谱并非有理谱，此时可将功率谱拟合成有理谱后按伯特-香农法进行设计。与卡尔曼滤波相比，维纳滤波在适用范围、设计方法等方面存在着诸多不足，但对于被估计参量较少的情况，如直升飞机悬停时仅须对高度作估计，结合使用数字滤波技术，维纳滤波仍不失为一种简单而有效的方法[61]。

## 2.2 离散型卡尔曼滤波

2.1节介绍的线性最小方差估计具有一定的实用性，不同时刻的量测信息利用得越多，估计的精度就越高。但这种算法使用了被估计量与量测量的一、二阶矩，对于非平稳过程，必须确切知道这种一、二阶矩的变化规律，这种要求是十分苛刻的，一般是无法办到的。此外，算法中采用对不同时刻的量测值作集中处理的办法，这使计算随着估计过程的推移而逐渐加重。所以线性最小方差估计并不是一种实用的估计算法，特别是对非平稳过程的处理更是困难重重。那么，能否找到一种实用的线性最小方差估计算法，适用于非平稳过程，并与递推最小二乘估计一样，算法采用递推，从量测信息中实时提取出被估计量信息并积存在估计值中呢？答案是肯定的，这种新型算法就是卡尔曼滤波。

1960年由卡尔曼(R.E.Kalman)首次提出的卡尔曼滤波[8]是一种线性最小方差估计。相对2.1节介绍的几种最优估计，卡尔曼滤波具有如下特点：

（1）算法是递推的，且使用状态空间法在时域内设计滤波器，所以卡尔曼滤波适用于对多维随机过程的估计。

（2）采用动力学方程即状态方程描述被估计量的动态变化规律，被估计量的动态统计信息由激励白噪声的统计信息和动力学方程确定。由于激励白噪声是平稳过程，动力学方程已知，所以被估计量既可以是平稳的，也可以是非平稳的，即卡尔曼滤波也适用于非平稳过程。

（3）卡尔曼滤波具有连续型和离散型两类算法，离散型算法可直接在数字计算机上实现。

正由于上述特点，卡尔曼滤波理论一经提出立即受到了工程应用的重视，阿波罗登月飞船和C-5A飞机导航系统的设计是早期应用中的最成功者[13-14]。目前，卡尔曼滤波理论作为一种最重要的最优估计理论被广泛应用于各种领域，组合导航系统的设计是其成功应用中的一个最主要方面。

## 2.2.1 离散型卡尔曼滤波基本方程

设 $t_k$ 时刻的被估计状态 $\mathbf{X}_k$ 受系统噪声序列 $\pmb{W}_{k - 1}$ 驱动，驱动机理由下述状态方程描述

$$
\boldsymbol {X} _ {k} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {X} _ {k - 1} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {W} _ {k - 1}\tag{2.2.1}
$$

对 $X_{k}$ 的量测满足线性关系,量测方程为

$$
\mathbf {Z} _ {k} = \mathbf {H} _ {k} \mathbf {X} _ {k} + \mathbf {V} _ {k}\tag{2.2.2}
$$

式中， $\Phi_{k,k-1}$ 为 $t_{k-1}$ 时刻至 $t_{k}$ 时刻的一步转移阵； $\Gamma_{k-1}$ 为系统噪声驱动阵； $H_{k}$ 为量测阵； $V_{k}$ 为量测噪声序列； $W_{k}$ 为系统激励噪声序列。

同时， $W_{k}$ 和 $V_{k}$ 满足

$$
\left. \begin{array}{r l} E [ \boldsymbol {W} _ {k} ] = \mathbf {0}, & \operatorname{Cov} [ \boldsymbol {W} _ {k}, \boldsymbol {W} _ {j} ] = E [ \boldsymbol {W} _ {k} \boldsymbol {W} _ {j} ^ {\mathrm{T}} ] = Q _ {k} \delta_ {k j} \\ E [ \boldsymbol {V} _ {k} ] = \mathbf {0}, & \operatorname{Cov} [ \boldsymbol {V} _ {k}, \boldsymbol {V} _ {j} ] = E [ \boldsymbol {V} _ {k} \boldsymbol {V} _ {j} ^ {\mathrm{T}} ] = R _ {k} \delta_ {k j} \\ & \operatorname{Cov} [ \boldsymbol {W} _ {k}, \boldsymbol {V} _ {j} ] = E [ \boldsymbol {W} _ {k} \boldsymbol {V} _ {j} ^ {\mathrm{T}} ] = \mathbf {0} \end{array} \right\}\tag{2.2.3}
$$

式中， $Q_{k}$ 为系统噪声序列的方差阵，假设为非负定阵； $R_{k}$ 为量测噪声序列的方差阵，假设为正定阵。

定理2.7

如果被估计状态 $X_{k}$ 满足式(2.2.1)，对 $X_{k}$ 的量测量 $Z_{k}$ 满足式(2.2.2)，系统噪声 $W_{k}$ 和量测噪声 $V_{k}$ 满足式(2.2.3)，系统噪声方差阵 $Q_{k}$ 非负定，量测噪声方差阵 $R_{k}$ 正定， $k$ 时刻的量测为 $Z_{k}$ ，则 $X_{k}$ 的估计 $\hat{X}_{k}$ 按下述方程求解：

状态一步预测

$$
\hat {\boldsymbol {X}} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \hat {\boldsymbol {X}} _ {k - 1}\tag{2.2.4a}
$$

状态估计

$$
\hat {\mathbf {X}} _ {k} = \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {K} _ {k} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1})\tag{2.2.4b}
$$

滤波增益

$$
\boldsymbol {K} _ {k} = \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}) ^ {- 1}\tag{2.2.4c}
$$

或

$$
\boldsymbol {K} _ {k} = \boldsymbol {P} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1}\tag{2.2.4c'}
$$

一步预测均方误差

$$
\boldsymbol {P} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {P} _ {k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {Q} _ {k - 1} \boldsymbol {\Gamma} _ {k - 1} ^ {\mathrm{T}}\tag{2.2.4d}
$$

估计均方误差

$$
\boldsymbol {P} _ {k} = (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) \boldsymbol {P} _ {k / k - 1} (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) ^ {\mathrm{T}} + \boldsymbol {K} _ {k} \boldsymbol {R} _ {k} \boldsymbol {K} _ {k} ^ {\mathrm{T}}\tag{2.2.4e}
$$

或

$$
\boldsymbol {P} _ {k} = (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) \boldsymbol {P} _ {k / k - 1}\tag{2.2.4e'}
$$

或

$$
\boldsymbol {P} _ {k} ^ {- 1} = \boldsymbol {P} _ {k / k - 1} ^ {- 1} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}\tag{2.2.4e \( ^{''} \)}
$$

式(2.2.4)即为离散型卡尔曼滤波基本方程。只要给定初值 $\hat{X}_{0}$ 和 $P_{0}$ ，根据k时刻的量测 $Z_{k}$ ，就可递推计算得k时刻的状态估计 $\hat{X}_{k}(k=1,2,\cdots)$ 。

式(2.2.4)所示算法可用图2.2.1来表示。从图中可明显看出卡尔曼滤波具有两个计算回路：增益计算回路和滤波计算回路。其中增益计算回路是独立计算回路，而滤波计算回路依赖于增益计算回路。

在一个滤波周期内，从卡尔曼滤波在使用系统信息和量测信息的先后次序来看，卡尔曼滤波具有两个明显的信息更新过程：时间更新过程和量测更新过程。式(2.2.4a)说明了根据 $k - 1$ 时刻的状态估计预测 $k$ 时刻状态估计的方法，式(2.2.4d)对这种预测的质量优劣作了定量描述。该两式的计算中仅使用了与系统动态特性有关的信息，如一步转移阵、噪声驱动阵、驱动噪声的方差阵。从时间的推移过程来看，该两式将时间从 $k - 1$ 时刻推进到 $k$ 时刻。所以该两式描述了卡尔曼滤波的时间更新过程。式(2.2.4)的其余诸式用来计算对时间更新值的修正量，该修正量由时间更新的质量优劣 $(P_{k / k - 1})$ 、量测信息的质量优劣 $(R_{k})$ 、量测与状态的关系 $(H_{k})$ 以及具体的量测值 $Z_{k}$ 所确定。所有这些方程围绕一个目的，即正确合理地利用量测 $Z_{k}$ 所以这一过程描述了卡尔曼滤波的量测更新过程。该两过程如图2.2.1所示。

![](images/392e773b04005fe70eb35bdc5f0826c96ef4472dd347ba2bdbacf9bca2e8561e.jpg)  
图2.2.1 卡尔曼滤波的两个计算回路和两个更新过程

## 2.2.2 离散型卡尔曼滤波基本方程的直观推导

为了加深对离散型卡尔曼滤波基本方程的理解, 此处根据直观理解和物理概念推导出式(2.2.4)。

## 1. 一步预测方程推导

一步预测是根据 $k - 1$ 时刻的状态估计预测 $k$ 时刻的状态，即根据 $k - 1$ 个量测 $\mathbf{Z}_1, \mathbf{Z}_2, \mathbf{Z}_{k-1}$ 对 $\mathbf{X}_k$ 作线性最小方差估计

$$
\begin{array}{r l} \hat {\mathbf {X}} _ {k / k - 1} & = E ^ {*} \left[ \mathbf {X} _ {k} / \mathbf {Z} _ {1} \mathbf {Z} _ {2} \dots \mathbf {Z} _ {k - 1} \right] = \\ & E ^ {*} \left[ (\boldsymbol {\Phi} _ {k, k - 1} \mathbf {X} _ {k - 1} + \boldsymbol {\Gamma} _ {k - 1} \mathbf {W} _ {k - 1}) / \mathbf {Z} _ {1} \mathbf {Z} _ {2} \dots \mathbf {Z} _ {k - 1} \right] \end{array}
$$

根据线性最小方差估计的性质(线性1)，有

$$
\hat {\mathbf {X}} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} E ^ {*} \left[ \mathbf {X} _ {k - 1} / \mathbf {Z} _ {1} \mathbf {Z} _ {2} \dots \mathbf {Z} _ {k - 1} \right] + \boldsymbol {\Gamma} _ {k - 1} E ^ {*} \left[ \mathbf {W} _ {k - 1} / \mathbf {Z} _ {1} \mathbf {Z} _ {2} \dots \mathbf {Z} _ {k - 1} \right]
$$

由式(2.2.1)知, $W_{k-1}$ 只影响 $X_{k}$ , 所以 $W_{k-1}$ 与 $Z_{1}Z_{2}\cdots Z_{k-1}$ 不相关, 且 $E[W_{k-1}] = 0$ , 根据式(2.1.64), 有

$$
E ^ {*} \left[ W _ {k - 1} / Z _ {1} Z _ {2} \dots Z _ {k - 1} \right] = 0
$$

而

$$
E ^ {*} \left[ \mathbf {X} _ {k - 1} / \mathbf {Z} _ {1} \mathbf {Z} _ {2} \dots \mathbf {Z} _ {k - 1} \right] = \hat {\mathbf {X}} _ {k - 1}
$$

因此

$$
\hat {\boldsymbol {X}} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \hat {\boldsymbol {X}} _ {k - 1}
$$

## 2. 状态估计方程推导

用一步预测代替真实状态 $X_{k}$ 引起的误差为

$$
\hat {\boldsymbol {X}} _ {k / k - 1} = \boldsymbol {X} _ {k} - \hat {\boldsymbol {X}} _ {k / k - 1}
$$

引起对量测的估计误差为

$$
\widetilde {\mathbf {Z}} _ {k / k - 1} = \mathbf {Z} _ {k} - \hat {\mathbf {Z}} _ {k / k - 1} = \mathbf {H} _ {k} \mathbf {X} _ {k} + \mathbf {V} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1} = \mathbf {H} _ {k} \widetilde {\mathbf {X}} _ {k / k - 1} + \mathbf {V} _ {k}
$$

滤波理论中称 $\tilde{Z}_{k/k-1}$ 为残差(也称新息)。

从上式可看出，残差包含有一步预测误差信息，对 $\widetilde{\mathbf{Z}}_{k / k - 1}$ 作适当的加权处理就能将 $\widetilde{\mathbf{X}}_{k / k - 1}$ 分离出来，用来修正 $\hat{\mathbf{X}}_{k / k - 1}$ 即可得到状态的估计

$$
\hat {\mathbf {X}} _ {k} = \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {K} _ {k} \hat {\mathbf {Z}} _ {k / k - 1} = \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {K} _ {k} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1})
$$

式中， $K_{k}$ 为对残差的加权阵，称为滤波增益阵。

## 3. 滤波增益阵和估计均方误差阵的推导

增益阵的选取准则是使估计的均方误差阵 $P_{k}=E[\widetilde{X}_{k}\widetilde{X}_{k}^{T}]$ 达到最小，其中 $\widetilde{X}_{k}=X_{k}-\widehat{X}_{k}$ 为估计误差。而

$$
\begin{array}{r l} \widetilde {\mathbf {X}} _ {k} & = \mathbf {X} _ {k} - \hat {\mathbf {X}} _ {k} = \mathbf {X} _ {k} - [ \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {K} _ {k} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1}) ] = \\ & \quad \widetilde {\mathbf {X}} _ {k / k - 1} - \mathbf {K} _ {k} (\mathbf {H} _ {k} \widetilde {\mathbf {X}} _ {k / k - 1} + \mathbf {V} _ {k}) = \\ & \quad (\mathbf {I} - \mathbf {K} _ {k} \mathbf {H} _ {k}) \widetilde {\mathbf {X}} _ {k / k - 1} - \mathbf {K} _ {k} \mathbf {V} _ {k} \end{array}
$$

所以

$$
\begin{array}{r l} \mathbf {P} _ {k} & = E [ \widetilde {\mathbf {X}} _ {k} \widetilde {\mathbf {X}} _ {k} ^ {\mathrm{T}} ] = E \{[ (\mathbf {I} - \mathbf {K} _ {k} \mathbf {H} _ {k}) \widetilde {\mathbf {X}} _ {k / k - 1} - \mathbf {K} _ {k} \mathbf {V} _ {k} ] [ (\mathbf {I} - \mathbf {K} _ {k} \mathbf {H} _ {k}) \widetilde {\mathbf {X}} _ {k / k - 1} - \mathbf {K} _ {k} \mathbf {V} _ {k} ] ^ {\mathrm{T}} \} = \\ & (\mathbf {I} - \mathbf {K} _ {k} \mathbf {H} _ {k}) E [ \widetilde {\mathbf {X}} _ {k / k - 1} \widetilde {\mathbf {X}} _ {k / k - 1} ^ {\mathrm{T}} ] (\mathbf {I} - \mathbf {K} _ {k} \mathbf {H} _ {k}) ^ {\mathrm{T}} + \mathbf {K} _ {k} E [ \mathbf {V} _ {k} \mathbf {V} _ {k} ^ {\mathrm{T}} ] \mathbf {K} _ {k} ^ {\mathrm{T}} - \\ & (\mathbf {I} - \mathbf {K} _ {k} \mathbf {H} _ {k}) E [ \widetilde {\mathbf {X}} _ {k / k - 1} \mathbf {V} _ {k} ^ {\mathrm{T}} ] \mathbf {K} _ {k} ^ {\mathrm{T}} - \mathbf {K} _ {k} E [ \mathbf {V} _ {k} \widetilde {\mathbf {X}} _ {k / k - 1} ^ {\mathrm{T}} ] (\mathbf {I} - \mathbf {K} _ {k} \mathbf {H} _ {k}) ^ {\mathrm{T}} \end{array}
$$

由于 $\hat{\mathbf{X}}_{k / k - 1}$ 是根据 $k - 1$ 时刻前的量测对 $k$ 时刻的状态所作的估计，而 $\mathbf{V}_k$ 是 $k$ 时刻的量测噪声，所以 $\mathbf{V}_k$ 与 $\tilde{\mathbf{X}}_{k / k - 1} = \mathbf{X}_k - \hat{\mathbf{X}}_{k / k - 1}$ 不相关，并注意到 $E[\mathbf{V}_k] = \mathbf{0}$ ，因此有

$$
E \left[ \widetilde {\boldsymbol {X}} _ {k / k - 1} \boldsymbol {V} _ {k} ^ {\mathrm{T}} \right] = E \left[ \boldsymbol {V} _ {k} \widetilde {\boldsymbol {X}} _ {k / k - 1} ^ {\mathrm{T}} \right] = \mathbf {0}
$$

代入 $P_{k}$ 的表达式，得

$$
\boldsymbol {P} _ {k} = \left(\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}\right) \boldsymbol {P} _ {k / k - 1} \left(\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}\right) ^ {\mathrm{T}} + \boldsymbol {K} _ {k} \boldsymbol {R} _ {k} \boldsymbol {K} _ {k} ^ {\mathrm{T}}
$$

式中

$$
\begin{array}{r l} & {\pmb {P} _ {k / k - 1} = E [ \widetilde {\pmb {X}} _ {k / k - 1} \widetilde {\pmb {X}} _ {k / k - 1} ^ {\mathrm{T}} ]} \\ & {\pmb {R} _ {k} = E [ \pmb {V} _ {k} \pmb {V} _ {k} ^ {\mathrm{T}} ]} \end{array}
$$

下面根据极值原理从式(2.2.4e)推导出滤波增益阵 $K_{k}$ 。

设 $K_{k}$ 是使估计的均方误差阵达到最小的最佳增益阵，并设该最小均方误差阵为 $P_{k}$ 。显然，若滤波增益阵偏离最佳增益阵的偏离量为 $\delta K_{k}$ ，则由式(2.2.4e)确定的估计的均方误差将偏离最小值 $P_{k}$ 而达到 $P_{k} + \delta P_{k}$ ，且 $\delta P_{k}$ 为非负定阵，即 $\delta P_{k} \geqslant 0$ 。 $K_{k} + \delta K_{k}$ 和 $P_{k} + \delta P_{k}$ 满足的方程为

$$
\begin{array}{r l} \boldsymbol {P} _ {k} + \delta \boldsymbol {P} _ {k} & = [ \boldsymbol {I} - (\boldsymbol {K} _ {k} + \delta \boldsymbol {K} _ {k}) \boldsymbol {H} _ {k} ] \boldsymbol {P} _ {k / k - 1} [ \boldsymbol {I} - (\boldsymbol {K} _ {k} + \delta \boldsymbol {K} _ {k}) \boldsymbol {H} _ {k} ] ^ {\mathrm{T}} + \\ & (\boldsymbol {K} _ {k} + \delta \boldsymbol {K} _ {k}) \boldsymbol {R} _ {k} (\boldsymbol {K} _ {k} + \delta \boldsymbol {K} _ {k}) ^ {\mathrm{T}} \end{array}
$$

其中 $K_{k}$ 和 $P_{k}$ 满足式(2.2.4e)。将式(2.2.4e)代入上式，得

$$
\delta \boldsymbol {P} _ {k} = \boldsymbol {W} + \boldsymbol {W} ^ {\mathrm{T}} + \delta \boldsymbol {K} _ {k} \left(\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}\right) \delta \boldsymbol {K} _ {k} ^ {\mathrm{T}}
$$

式中

$$
\boldsymbol {W} = - \delta \boldsymbol {K} _ {k} \left[ \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} (\boldsymbol {I} - \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {K} _ {k} ^ {\mathrm{T}}) - \boldsymbol {R} _ {k} \boldsymbol {K} _ {k} ^ {\mathrm{T}} \right] =
$$

若取

$$
\begin{array}{r l} & - \delta \mathbf {K} _ {k} \left[ \mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} - (\mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} + \mathbf {R} _ {k}) \mathbf {K} _ {k} ^ {\mathrm{T}} \right] \\ & \quad \mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} - (\mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} + \mathbf {R} _ {k}) \mathbf {K} _ {k} ^ {\mathrm{T}} = \mathbf {0} \end{array}
$$

即

$$
\boldsymbol {K} _ {k} = \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}) ^ {- 1}
$$

则

$$
\begin{array}{r l} & W = 0 \\ \delta P _ {k} = \delta K _ {k} (H _ {k} P _ {k / k - 1} H _ {k} ^ {\mathrm{T}} + R _ {k}) \delta K _ {k} ^ {\mathrm{T}} \end{array}
$$

由于 $R_{k}$ 为正定阵， $P_{k/k-1}$ 至少为非负定阵，所以 $H_{k}P_{k/k-1}H_{k}^{\mathrm{T}}+R_{k}$ 至少为非负定阵。若 $\delta K_{k}$ 为非零阵，则 $\delta P_{k}$ 至少为非负定阵，即 $\delta P_{k}\geqslant0$ 。这说明，若 $K_{k}$ 按式(2.2.4c)确定，则对于相对增益阵 $K_{k}$ 的任何偏离 $\delta K_{k}\neq0$ ，估计的均方误差将产生非负的偏差 $\delta P_{k}$ ，因此 $K_{k}$ 是使估计的均方误差达到最小的最佳增益阵。

## 4. 一步预测均方误差阵的推导

一步预测产生的误差为

$$
\begin{array}{r l} \widetilde {\boldsymbol {X}} _ {k / k - 1} & = \boldsymbol {X} _ {k} - \hat {\boldsymbol {X}} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {X} _ {k - 1} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {W} _ {k - 1} - \boldsymbol {\Phi} _ {k, k - 1} \hat {\boldsymbol {X}} _ {k - 1} = \\ & \quad \boldsymbol {\Phi} _ {k, k - 1} \hat {\boldsymbol {X}} _ {k - 1} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {W} _ {k - 1} \end{array}
$$

所以一步预测均方误差阵为

$$
\begin{array}{r l} \boldsymbol {P} _ {k / k - 1} & = E [ \widetilde {\boldsymbol {X}} _ {k / k - 1} \widetilde {\boldsymbol {X}} _ {k / k - 1} ^ {\mathrm{T}} ] = \\ & E \{[ \boldsymbol {\Phi} _ {k, k - 1} \widetilde {\boldsymbol {X}} _ {k - 1} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {W} _ {k - 1} ] [ \boldsymbol {\Phi} _ {k, k - 1} \widetilde {\boldsymbol {X}} _ {k - 1} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {W} _ {k - 1} ] ^ {\mathrm{T}} \} \end{array}
$$

由于 $\widetilde{\pmb{X}}_{k - 1} = \pmb{X}_{k - 1} - \hat{\pmb{X}}_{k - 1},\pmb{W}_{k - 1}$ 只影响 $X_{k}$ ，不影响 $X_{k - 1}$ ，所以 $W_{k - 1}$ 与 $\widetilde{\pmb{X}}_{k - 1}$ 不相关，又因 $E[\pmb {W}_{k - 1}] = \mathbf{0}$ ，因此

$$
\boldsymbol {P} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {P} _ {k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {Q} _ {k - 1} \boldsymbol {\Gamma} _ {k - 1} ^ {\mathrm{T}}
$$

## 2.2.3 离散型卡尔曼滤波基本方程的正交投影推导

基本方程除利用物理概念经直观推导获得外,还可用严密的数学概念推导获得。介绍这一内容的目的是使读者了解卡尔曼滤波的数学内涵。

1. 正交投影定理

设 X 和 Z 都是随机向量, 如果

$$
E [ \mathbf {X Z} ^ {\mathrm{T}} ] = \mathbf {0}\tag{2.2.5}
$$

则称 X 与 Z 正交。

应注意随机向量间正交、不相关和独立三者的区别。

如果随机向量 X 和 Z 的协方差阵为零, 即

$$
\operatorname{Cov} (\mathbf {X}, \mathbf {Z}) = E \left[ \left(\mathbf {X} - \boldsymbol {m} _ {\mathrm{X}}\right) \left(\mathbf {Z} - \boldsymbol {m} _ {\mathrm{Z}}\right) ^ {\mathrm{T}} \right] = E \left[ \mathbf {X Z} ^ {\mathrm{T}} \right] - \boldsymbol {m} _ {\mathrm{X}} \boldsymbol {m} _ {\mathrm{Z}} ^ {\mathrm{T}} = \mathbf {0}
$$

即

$$
E [ \mathbf {X Z} ^ {\mathrm{T}} ] = \boldsymbol {m} _ {X} \boldsymbol {m} _ {Z} ^ {\mathrm{T}}
$$

(2.2.6)

式中 $m_{X}$ 和 $m_{Z}$ 分别为 X 和 Z 的均值，则称 X 和 Z 不相关。

如果 $X$ 和 $Z$ 的联合分布密度

$$
p (x, z) = p _ {1} (x) \cdot p _ {2} (z)\tag{2.2.7}
$$

式中 $p_{1}(x)$ 和 $p_{2}(z)$ 分别为 X 和 Z 的分布密度，则称 X 和 Z 互相独立。

从上述定义可得出如下结论：

（1）若 X 与 Z 独立，则 X 与 Z 一定不相关。但逆命题一般并不成立，只有当 X 和 Z 都服从正态分布时才成立。

(2) 如果 X 和 Z 的数学期望至少有一个为零, 则不相关与正交等价。

（3）如果 X 和 Z 都服从正态分布，且至少有一个数学期望为零，则不相关、正交和独立三者等价。

如果存在某矩阵 $A^{1}$ 和某向量 $b^{1}$ ，对任意矩阵 A 和任意向量 b 都能使下式成立

$$
E \left\{\left[ \boldsymbol {X} - (\boldsymbol {A} ^ {1} \boldsymbol {Z} + \boldsymbol {b} ^ {1}) \right] (\boldsymbol {A Z} + \boldsymbol {b}) ^ {\mathrm{T}} \right\} = \mathbf {0}\tag{2.2.8}
$$

则称 $A^{1}Z + b^{1}$ 为 X 在 Z 上的正交投影。

式 $(2.2.8)$ 也可改写成如下形式：

$$
\begin{array}{r l} & E \{\left[ \mathbf {X} - (\mathbf {A} ^ {1} \mathbf {Z} + \mathbf {b} ^ {1}) \right] (\mathbf {A Z} + \mathbf {b}) ^ {\mathrm{T}} \} = \\ & E \{\left[ \mathbf {X} - (\mathbf {A} ^ {1} \mathbf {Z} + \mathbf {b} ^ {1}) \right] \mathbf {Z} ^ {\mathrm{T}} \} \mathbf {A} ^ {\mathrm{T}} + E \{\mathbf {X} - (\mathbf {A} ^ {1} \mathbf {Z} + \mathbf {b} ^ {1}) \} \mathbf {b} ^ {\mathrm{T}} = \mathbf {0} \end{array}
$$

由于 A 为任意矩阵, b 为任意向量, 要使上式恒能成立, 须有

$$
\left. \begin{array}{l} E \{\left[ \mathbf {X} - (\mathbf {A} ^ {1} \mathbf {Z} + \mathbf {b} ^ {1}) \right] \mathbf {Z} ^ {\mathrm{T}} \} = \mathbf {0} \\ E [ \mathbf {X} - (\mathbf {A} ^ {1} \mathbf {Z} + \mathbf {b} ^ {1}) ] = \mathbf {0} \end{array} \right\}\tag{2.2.9}
$$

式 $(2.2.9)$ 是正交投影定义的另一种形式。从该式可看出：

（1）正交投影是量测量Z和常值向量 $b^{1}$ 的线性组合，位于Z张成的空间内，该空间也称量测空间。

（2）若用正交投影作为 X 的估计，则估计误差与量测空间正交。

(3) 正交投影是 X 的无偏估计。

根据上述三点,可画出正交投影的几何示意图,如图2.2.2所示。

定理2.8

![](images/2cb20a1c8616edc7b6a00bb5fba81481306503fac3844f6c6e8b8ae5b321782e.jpg)

X 在 Z 上的正交投影即为 X 在 Z 上的线性最小方差估计, 反之亦然, 即

$$
\mathbf {A} ^ {1} \mathbf {Z} + \mathbf {b} ^ {1} = E ^ {*} [ \mathbf {X} / \mathbf {Z} ]
$$

图2.2.2 正交投影的几何示意图

(2.2.10)

证明

(1) 设 $E^{*}[X/Z]$ 是 X 在 Z 上的线性最小方差估计, 根据式(2.1.64), 有

$$
\begin{array}{r l} E \{(\mathbf {X} - E ^ {*} [ \mathbf {X} / \mathbf {Z} ]) \mathbf {Z} ^ {\mathrm{T}} \} & = E \{[ \mathbf {X} - m _ {X} - C _ {X Z} C _ {Z} ^ {- 1} (\mathbf {Z} - m _ {Z}) ] \mathbf {Z} ^ {\mathrm{T}} \} = \\ & E [ \mathbf {X Z} ^ {\mathrm{T}} ] - m _ {X} m _ {Z} ^ {\mathrm{T}} - C _ {X Z} C _ {Z} ^ {- 1} E [ \mathbf {Z Z} ^ {\mathrm{T}} ] + C _ {X Z} C _ {Z} ^ {- 1} m _ {Z} m _ {Z} ^ {\mathrm{T}} \end{array}
$$

由于

$$
E [ \mathbf {X Z} ^ {\mathrm{T}} ] = E \left\{\left[ (\mathbf {X} - \boldsymbol {m} _ {X}) + \boldsymbol {m} _ {X} \right] \left[ (\mathbf {Z} - \boldsymbol {m} _ {Z}) + \boldsymbol {m} _ {Z}\right) ^ {\mathrm{T}} \right\} = \boldsymbol {C} _ {X Z} + \boldsymbol {m} _ {X} \boldsymbol {m} _ {Z} ^ {\mathrm{T}}
$$

同理

$$
E [ \mathbf {Z} \mathbf {Z} ^ {\mathrm{T}} ] = C _ {Z} + m _ {Z} m _ {Z} ^ {\mathrm{T}}
$$

所以

$$
\begin{array}{r l} E \{(\mathbf {X} - E ^ {*} [ \mathbf {X} / \mathbf {Z} ]) \mathbf {Z} ^ {\mathrm{T}} \} & = \mathbf {C} _ {X Z} + m _ {X} \mathbf {m} _ {Z} ^ {\mathrm{T}} - m _ {X} \mathbf {m} _ {Z} ^ {\mathrm{T}} - \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} (\mathbf {C} _ {Z} + m _ {Z} \mathbf {m} _ {Z} ^ {\mathrm{T}}) + \\ & \quad \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} \mathbf {m} _ {Z} \mathbf {m} _ {Z} ^ {\mathrm{T}} = \mathbf {0} \end{array}
$$

此外，由于线性最小方差估计是无偏估计，即

$$
E \{\mathbf {X} - E ^ {*} [ \mathbf {X} / \mathbf {Z} ] \} = \mathbf {0}
$$

根据式(2.2.9)所给定义知, $E^{*}[X/Z]$ 是X在Z上的正交投影。

(2) 设 $A^{1}Z + b^{1}$ 为 X 在 Z 上的正交投影，则根据式(2.2.9)，有

$$
E \{\left[ \mathbf {X} - (\mathbf {A} ^ {\mathrm{I}} \mathbf {Z} + \mathbf {b} ^ {\mathrm{I}}) \right] \mathbf {Z} ^ {\mathrm{T}} \} = \mathbf {0}\tag{1}
$$

$$
E [ \mathbf {X} - (\mathbf {A} ^ {1} \mathbf {Z} + \mathbf {b} ^ {1}) ] = \mathbf {0}\tag{2}
$$

由(1)证明了线性最小方差估计 $E^{*}[X/Z]=A^{0}Z+b^{0}$ 是 X 在 Z 上的正交投影, 即有

$$
E \left\{\left[ \boldsymbol {X} - \left(\boldsymbol {A} ^ {0} \boldsymbol {Z} + \boldsymbol {b} ^ {0}\right) \right] \boldsymbol {Z} ^ {\mathrm{T}} \right\} = \mathbf {0}\tag{3}
$$

$$
E \{\mathbf {X} - (\mathbf {A} ^ {0} \mathbf {Z} + \mathbf {b} ^ {0}) \} = \mathbf {0}\tag{4}
$$

式(1) 减去式(3)，得

$$
\left(\boldsymbol {A} ^ {1} - \boldsymbol {A} ^ {0}\right) E \left[ \boldsymbol {Z} \boldsymbol {Z} ^ {\mathrm{T}} \right] + \left(\boldsymbol {b} ^ {1} - \boldsymbol {b} ^ {0}\right) E \left[ \boldsymbol {Z} ^ {\mathrm{T}} \right] = \mathbf {0}
$$

由于 $E[\mathbf{ZZ}^{\mathrm{T}}] = \mathbf{C}_{\mathrm{Z}} + \mathbf{m}_{\mathrm{Z}}\mathbf{m}_{\mathrm{Z}}^{\mathrm{T}}$ ，因此

$$
\left(\boldsymbol {A} ^ {1} - \boldsymbol {A} ^ {0}\right) \boldsymbol {C} _ {Z} + \left[ \left(\boldsymbol {A} ^ {1} - \boldsymbol {A} ^ {0}\right) m _ {Z} + \boldsymbol {b} ^ {1} - \boldsymbol {b} ^ {0} \right] m _ {Z} ^ {\mathrm{T}} = \mathbf {0}\tag{5}
$$

式(2) 减去式(4)，得

$$
(\mathbf {A} ^ {1} - \mathbf {A} ^ {0}) \mathbf {m} _ {Z} + \mathbf {b} ^ {1} - \mathbf {b} _ {0} = \mathbf {0}\tag{6}
$$

将式(6)代入式(5)，得

$$
(A ^ {1} - A ^ {0}) C _ {Z} = 0
$$

在估计过程中,为了充分利用量测信息,Z 的各分量必须是独立的,即有

$$
\begin{array}{r l} & E \big [ (Z _ {i} - m _ {Z _ {i}}) (Z _ {j} - m _ {Z _ {j}}) \big ] = \sigma_ {i} ^ {2} \delta_ {i j} \quad i = 1, 2, \dots , m, j = 1, 2, \dots , m \\ & \qquad \mathbf {C} _ {Z} = \mathrm{diag} \big [ \sigma_ {1} ^ {2} \sigma_ {2} ^ {2} \dots \sigma_ {m} ^ {2} \big ] \end{array}
$$

即 $C_z$ 满秩，因此有

$$
\mathbf {A} ^ {1} = \mathbf {A} ^ {0}
$$

将此关系式代入式(6)，得

$$
\boldsymbol {b} ^ {1} = \boldsymbol {b} ^ {0}
$$

从而有

$$
\mathbf {A} ^ {1} \mathbf {Z} + \mathbf {b} ^ {1} = \mathbf {A} ^ {0} \mathbf {Z} + \mathbf {b} ^ {0} = E ^ {*} [ \mathbf {X} / \mathbf {Z} ]
$$

这就证明了定理2.8。

2. 更新信息定理

定理 2.9(更新信息定理)

设 $X_{k}$ 为随机向量， $Z_{k}$ 为第 $k$ 次量测， $\overline{Z}_{k-1}$ 和 $\overline{Z}_{k}$ 分别为前 $k-1$ 次量测和前 $k$ 次量测，则

$$
E ^ {*} \left[ \mathbf {X} _ {k} / \overline {{\mathbf {Z}}} _ {k} \right] = E ^ {*} \left[ \mathbf {X} _ {k} / \overline {{\mathbf {Z}}} _ {k - 1} \right] + E \left[ \widetilde {\mathbf {X}} _ {k / k - 1} \widetilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} \right] \cdot \left\{E \left[ \widetilde {\mathbf {Z}} _ {k / k - 1} \widetilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} \right] \right\} ^ {- 1} \widetilde {\mathbf {Z}} _ {k / k - 1}\tag{2.2.11}
$$

即

$$
\hat {\mathbf {X}} _ {k} = \hat {\mathbf {X}} _ {k / k - 1} + E [ \widetilde {\mathbf {X}} _ {k / k - 1} \widetilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} ] \left\{E [ \widetilde {\mathbf {Z}} _ {k / k - 1} \widetilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} ] \right\} ^ {- 1} \widetilde {\mathbf {Z}} _ {k / k - 1}\tag{2.2.12}
$$

式中

$$
\begin{array}{r l} & \widetilde {\mathbf {X}} _ {k / k - 1} = \mathbf {X} _ {k} - E ^ {*} \left[ \mathbf {X} _ {k} / \overline {{\mathbf {Z}}} _ {k - 1} \right] \\ & \widetilde {\mathbf {Z}} _ {k / k - 1} = \mathbf {Z} _ {k} - E ^ {*} \left[ \mathbf {Z} _ {k} / \overline {{\mathbf {Z}}} _ {k - 1} \right] \end{array}
$$

更新信息定理说明只需用 $\widetilde{\mathbf{Z}}_{k / k - 1}$ 对 $\hat{\mathbf{X}}_{k / k - 1}$ 作修正即可获得 $\hat{\mathbf{X}}_k$ ，所以 $\widetilde{\mathbf{Z}}_{k / k - 1}$ 是卡尔曼滤波中的一个重要信息源。由于 $\hat{\mathbf{Z}}_{k / k - 1} = E^{*}\left[\mathbf{Z}_{k} / \overline{\mathbf{Z}}_{k - 1}\right]$ 是前 $k - 1$ 次量测已提供的关于 $\mathbf{Z}_k$ 的信息，而 $\widetilde{\mathbf{Z}}_{k / k - 1}$ 含有第 $k$ 次量测 $\mathbf{Z}_k$ 新提供的信息，所以常将 $\widetilde{\mathbf{Z}}_{k / k - 1}$ 称为新息，这一定理也由此而得名。

证明

$$
\hat {\mathbf {X}} _ {k} = E ^ {*} \left[ \mathbf {X} _ {k} / \overline {{{\mathbf {Z}}}} _ {k} \right] = E ^ {*} \left[ \mathbf {X} _ {k} / \overline {{{\mathbf {Z}}}} _ {k - 1}, \mathbf {Z} _ {k} \right]
$$

而

$$
\mathbf {Z} _ {k} = \mathbf {Z} _ {k} - \hat {\mathbf {Z}} _ {k / k - 1} + \hat {\mathbf {Z}} _ {k / k - 1} = \hat {\mathbf {Z}} _ {k / k - 1} + \hat {\mathbf {Z}} _ {k / k - 1}\tag{1}
$$

式中

即 $\hat{\mathbf{Z}}_{k / k - 1}$ 是关于 $\overline{\mathbf{Z}}_{k - 1}$ 的线性函数，由式(1)可推知， $\mathbf{Z}_k$ 可由 $\overline{\mathbf{Z}}_{k - 1}$ 和 $\widetilde{\mathbf{Z}}_{k / k - 1}$ 线性表示，所以 $\hat{\mathbf{X}}_k$ 可由 $\overline{\mathbf{Z}}_{k - 1}$ 和 $\widetilde{\mathbf{Z}}_{k / k - 1}$ 线性表示

$$
\hat {\mathbf {X}} _ {k} = E ^ {*} \left[ \mathbf {X} _ {k} / \overline {{\mathbf {Z}}} _ {k - 1}, \widetilde {\mathbf {Z}} _ {k / k - 1} \right]\tag{2}
$$

由于 $\widetilde{\mathbf{Z}}_{k / k - 1} = \mathbf{Z}_k - \hat{\mathbf{Z}}_{k / k - 1} = \mathbf{Z}_k - E^*\left[\mathbf{Z}_k / \overline{\mathbf{Z}}_{k - 1}\right]$ ，由定理2.8知 $\widetilde{\mathbf{Z}}_{k / k - 1}$ 与 $\overline{\mathbf{Z}}_{k - 1}$ 正交，即

$$
E [ \widetilde {\mathbf {Z}} _ {k / k - 1} \overline {{\mathbf {Z}}} _ {k - 1} ^ {\mathrm{T}} ] = \mathbf {0}\tag{3}
$$

且

$$
E [ \widetilde {\mathbf {Z}} _ {k / k - 1} ] = \mathbf {0}\tag{4}
$$

根据线性最小方差估计性质3(线性2)，式(2.1.64)及式(3)、式(4)，有

$$
\begin{array}{r l} \hat {\mathbf {X}} _ {k} & = E ^ {*} [ \mathbf {X} _ {k} / \bar {\mathbf {Z}} _ {k - 1}, \tilde {\mathbf {Z}} _ {k / k - 1} ] = \\ & E ^ {*} [ \mathbf {X} _ {k} / \bar {\mathbf {Z}} _ {k - 1} ] + E ^ {*} [ \mathbf {X} _ {k} / \tilde {\mathbf {Z}} _ {k / k - 1} ] - E [ \mathbf {X} _ {k} ] = \\ & \hat {\mathbf {X}} _ {k / k - 1} + E [ \mathbf {X} _ {k} ] + E \{(\mathbf {X} _ {k} - E [ \mathbf {X} _ {k} ]) \tilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} \} \cdot \\ & \{E [ \tilde {\mathbf {Z}} _ {k / k - 1} \tilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} ] \} ^ {- 1} \tilde {\mathbf {Z}} _ {k / k - 1} - E [ \mathbf {X} _ {k} ] = \\ & \hat {\mathbf {X}} _ {k / k - 1} + \{E [ \mathbf {X} _ {k} \tilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} ] - E [ \mathbf {X} _ {k} ] E [ \tilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} ] \} \cdot \\ & \{E [ \tilde {\mathbf {Z}} _ {k / k - 1} \tilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} ] \} ^ {- 1} \tilde {\mathbf {Z}} _ {k / k - 1} = \\ & \hat {\mathbf {X}} _ {k / k - 1} + E [ (\hat {\mathbf {X}} _ {k / k - 1} + \tilde {\mathbf {X}} _ {k / k - 1}) \tilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} ] \cdot \\ & \{E [ \tilde {\mathbf {Z}} _ {k / k - 1} \tilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} ] \} ^ {- 1} \tilde {\mathbf {Z}} _ {k / k - 1} \end{array}
$$

注意到 $\hat{X}_{k/k-1}$ 是确定性向量, 所以

$$
\begin{array}{r l} \hat {\mathbf {X}} _ {k} & = \hat {\mathbf {X}} _ {k / k - 1} + \{\hat {\mathbf {X}} _ {k / k - 1} E [ \widetilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} ] + E [ \widetilde {\mathbf {X}} _ {k / k - 1} \widetilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} ] \} \cdot \{E [ \widetilde {\mathbf {Z}} _ {k / k - 1} \widetilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} ] \} ^ {- 1} \widetilde {\mathbf {Z}} _ {k / k - 1} = \\ & \hat {\mathbf {X}} _ {k / k - 1} + E [ \widetilde {\mathbf {X}} _ {k / k - 1} \widetilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} ] \{E [ \widetilde {\mathbf {Z}} _ {k / k - 1} \widetilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} ] \} ^ {- 1} \widetilde {\mathbf {Z}} _ {k / k - 1} \end{array}
$$

## 3. 基本方程的正交投影推导

下面根据更新信息定理推导卡尔曼滤波的状态估计方程和最佳增益阵。

由于

$$
\begin{array}{r l} \widetilde {\mathbf {X}} _ {k / k - 1} & = \mathbf {X} _ {k} - \hat {\mathbf {X}} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \widetilde {\mathbf {X}} _ {k - 1} + \boldsymbol {\Gamma} _ {k - 1} \mathbf {W} _ {k - 1} \\ \widetilde {\mathbf {Z}} _ {k / k - 1} & = \mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1} = \mathbf {H} _ {k} \widetilde {\mathbf {X}} _ {k / k - 1} + \mathbf {V} _ {k} \end{array}
$$

并注意到 k 时刻的量测噪声不会影响 k-1 时刻前的估计, 所以

$$
\begin{array}{r l} & E [ \widetilde {\mathbf {X}} _ {k / k - 1} \widetilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} ] = E [ \widetilde {\mathbf {X}} _ {k / k - 1} (\mathbf {H} _ {k} \widetilde {\mathbf {X}} _ {k / k - 1} + \mathbf {V} _ {k}) ^ {\mathrm{T}} ] = \\ & \qquad E [ \widetilde {\mathbf {X}} _ {k / k - 1} \widetilde {\mathbf {X}} _ {k / k - 1} ^ {\mathrm{T}} ] \mathbf {H} _ {k} ^ {\mathrm{T}} + E [ \widetilde {\mathbf {X}} _ {k / k - 1} ] E [ \mathbf {V} _ {k} ^ {\mathrm{T}} ] = \\ & \qquad \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \\ & E [ \widetilde {\mathbf {Z}} _ {k / k - 1} \widetilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} ] = E [ (\mathbf {H} _ {k} \widetilde {\mathbf {X}} _ {k / k - 1} + \mathbf {V} _ {k}) (\mathbf {H} _ {k} \widetilde {\mathbf {X}} _ {k / k - 1} + \mathbf {V} _ {k}) ^ {\mathrm{T}} ] = \\ & \qquad \mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} + \mathbf {R} _ {k} \end{array}
$$

将上述关系式代入式 $(2.2.12)$ ，得

$$
\hat {\boldsymbol {X}} _ {k} = \hat {\boldsymbol {X}} _ {k / k - 1} + \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \left(\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}\right) ^ {- 1} \left(\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \hat {\boldsymbol {X}} _ {k / k - 1}\right)
$$

令

$$
\boldsymbol {K} _ {k} = \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}) ^ {- 1}
$$

则

$$
\hat {\mathbf {X}} _ {k} = \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {K} _ {k} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1})
$$

一步预测方程、一步预测的均方误差阵方程和估计的均方误差阵方程的推导与直观推导方法中所述的一样，此处不再重复。

从上述推导过程可看出， $\hat{\mathbf{X}}_k$ 是量测空间 $\overline{\mathbf{Z}}_k$ 上的正交投影，根据定理2.8，正交投影与线性最小方差估计两者等价，所以卡尔曼滤波是 $X_k$ 在 $\overline{\mathbf{Z}}_k$ 上的线性最小方差估计， $\hat{\mathbf{X}}_k$ 利用了 $k$ 时刻及 $k$ 时刻以前的所有量测信息，即按式(2.2.4)确定的估计与按式(2.1.64)确定的估计是一样的，只是算法不同而已。与式(2.1.64)相比，离散型卡尔曼滤波基本方程有如下明显优点：

（1）由于采用了递推算法，不同时刻的量测值不必储存起来，而是经实时处理提炼成被估计状态的信息，随着滤波步数的增加，提取出的信息浓度逐渐增加。

（2）不必了解被估计量和量测量在不同时刻的一、二阶矩，而只须知道驱动被估计量的驱动噪声的统计特性、描述这种驱动作用的系统状态方程及量测噪声的统计特性。驱动噪声和量测噪声都是白噪声，是平稳过程，统计特性不随时间而变，系统的状态方程又是准确已知的，所以卡尔曼滤波能对非平稳的被估计量作估计。

(3) 算法可直接在计算机上实现。

正由于卡尔曼滤波具有其他估计不具备的优点,所以受到工程界的高度重视。

## 2.2.4 离散型卡尔曼滤波基本方程使用要点

1. 滤波初值的选取

卡尔曼滤波是一种递推算法，启动时必须先给定初值 $\hat{X}_{0}$ 和 $P_{0}$ 。

定理2.10

如果选取 $\hat{X}_{0}=m_{X_{0}}$ ，则滤波过程中估计始终无偏，即 $E[\hat{X}_{k}]=E[X_{k}]$ 。

证明

由式(2.2.4b)、式(2.2.1)及式(2.2.2)，得

$$
\begin{array}{r l} \hat {\mathbf {X}} _ {1} & = \boldsymbol {\Phi} _ {1, 0} \hat {\mathbf {X}} _ {0} + \mathbf {K} _ {1} (\mathbf {Z} _ {1} - \mathbf {H} _ {1} \boldsymbol {\Phi} _ {1, 0} \hat {\mathbf {X}} _ {0}) = \\ & \quad \boldsymbol {\Phi} _ {1, 0} \hat {\mathbf {X}} _ {0} + \mathbf {K} _ {1} [ \mathbf {H} _ {1} (\boldsymbol {\Phi} _ {1, 0} \mathbf {X} _ {0} + \boldsymbol {\Gamma} _ {0} \mathbf {W} _ {0}) + \mathbf {V} _ {1} - \mathbf {H} _ {1} \boldsymbol {\Phi} _ {1, 0} \hat {\mathbf {X}} _ {0} ] \end{array}
$$

对上式取均值，并设 $\hat{\pmb{X}}_0 = \pmb{m}_{X_0}$ ，则有

$$
E [ \hat {X} _ {1} ] = \Phi_ {1, 0} m _ {X _ {0}}
$$

而

$$
E [ \boldsymbol {X} _ {1} ] = E [ \boldsymbol {\Phi} _ {1, 0} \boldsymbol {X} _ {0} + \boldsymbol {\Gamma} _ {0} \boldsymbol {W} _ {0} ] = \boldsymbol {\Phi} _ {1, 0} \boldsymbol {m} _ {X _ {0}} = E [ \hat {\boldsymbol {X}} _ {1} ]
$$

同理有

$$
\begin{array}{r l} E [ \hat {X} _ {2} ] & = E [ \Phi_ {2, 1} \hat {X} _ {1} + K _ {2} (Z _ {2} - H _ {2} \Phi_ {2, 1} \hat {X} _ {1}) ] = \\ & E [ \Phi_ {2, 1} \Phi_ {1, 0} m _ {X _ {0}} + K _ {2} H _ {2} \Phi_ {2, 1} (X _ {1} - \hat {X} _ {1}) + K _ {2} V _ {2} + K _ {2} H _ {1} \Gamma_ {1} W _ {1} ] = \\ & \Phi_ {2, 0} m _ {X _ {0}} = E [ X _ {2} ] \end{array}
$$

根据数学归纳法可以证明

$$
E [ \hat {X} _ {k} ] = \Phi_ {k, 0} m _ {X _ {0}} = E [ X _ {k} ]
$$

从证明过程可看出，一步预测 $\hat{\pmb{X}}_{k / k - 1}$ 和 $\hat{\pmb{Z}}_{k / k - 1}$ 也都是无偏的，即

$$
\begin{array}{l} E [ \hat {\mathbf {X}} _ {k / k - 1} ] = E [ \mathbf {X} _ {k} ] \\ E [ \hat {\mathbf {Z}} _ {k / k - 1} ] = E [ \mathbf {Z} _ {k} ] \end{array}
$$

需要指出,如果滤波的起始时刻有量测量 $Z_{0}$ , 则可令

$$
\begin{array}{r l} & \hat {\boldsymbol {X}} _ {0 / - 1} = \boldsymbol {m} _ {X _ {0}} \\ & \boldsymbol {P} _ {0 / - 1} = \boldsymbol {C} _ {X _ {0}} \end{array}
$$

此时 $\hat{X}_{0}$ 就不再是初始值了，而是经过 $Z_{0}$ 修正后的估计。根据式(2.2.4b) 和式(2.2.4c')， $\hat{X}_{0}$ 为

$$
\hat {\boldsymbol {X}} _ {0} = \boldsymbol {m} _ {X _ {0}} + \boldsymbol {C} _ {X _ {0}} \boldsymbol {H} _ {0} ^ {\mathrm{T}} (\boldsymbol {H} _ {0} \boldsymbol {C} _ {X _ {0}} \boldsymbol {H} _ {0} ^ {\mathrm{T}} + \boldsymbol {R} _ {0}) ^ {- 1} (\boldsymbol {Z} _ {0} - \boldsymbol {H} _ {0} \boldsymbol {m} _ {X _ {0}})\tag{2.2.13}
$$

如果并不了解初始状态的统计特性,常令

$$
\hat {X} _ {0} = 0, P _ {0} = \alpha I
$$

或

其中 $\alpha$ 为很大的正数，在此情况下，滤波器不能保证是无偏的。由于 $P_0 \neq C_{X_0}$ ，所以实际的估计均方误差也不一定是最小的， $P_0$ 对滤波的影响将在第四章中讨论。事实上，如果系统是一致完全随机可控和一致完全随机可观测的，则卡尔曼滤波器一定是一致渐近稳定的。随着滤波步数的增加，盲目选取的滤波初值 $\hat{X}_0$ 和 $P_0$ 对滤波值 $\hat{X}_k$ 和 $P_k$ 的影响将逐渐减弱直至消失，估计逐渐趋向无偏，估计的均方误差也逐渐和 $P_0 = C_{X_0}$ 时的结果相一致。因此，在不了解初始状态的统计特性的情况下，一般应将滤波器设计成一致渐近稳定的。第四章将对这一问题作详细探讨。

## 2. 估计均方误差阵的等价形式及选用

式 $(2.2.4)$ 给出了计算估计的均方误差阵的三种等价形式

$$
\boldsymbol {P} _ {k} = (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) \boldsymbol {P} _ {k / k - 1} (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) ^ {\mathrm{T}} + \boldsymbol {K} _ {k} \boldsymbol {P} _ {k} \boldsymbol {K} _ {k} ^ {\mathrm{T}}\tag{2.2.4e}
$$

$$
\boldsymbol {P} _ {k} = (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) \boldsymbol {P} _ {k / k - 1}\tag{2.2.4e'}
$$

$$
\boldsymbol {P} _ {k} ^ {- 1} = \boldsymbol {P} _ {k / k - 1} ^ {- 1} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}\tag{2.2.4e \( ^{''} \)}
$$

它们形式上迥然不同,但本质上具有同解关系,通过恒等变形可逐一推导出这种同解关系。展开式(2.2.4e),得

$$
\begin{array}{r l} & {\pmb {P} _ {k} = (\pmb {I} - \pmb {K} _ {k} \pmb {H} _ {k}) \pmb {P} _ {k / k - 1} (\pmb {I} - \pmb {K} _ {k} \pmb {H} _ {k}) ^ {\mathrm{T}} + \pmb {K} _ {k} \pmb {R} _ {k} \pmb {K} _ {k} ^ {\mathrm{T}} =} \\ & {\qquad \pmb {P} _ {k / k - 1} - \pmb {P} _ {k / k - 1} \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {K} _ {k} ^ {\mathrm{T}} - \pmb {K} _ {k} \pmb {H} _ {k} \pmb {P} _ {k / k - 1} + \pmb {K} _ {k} \pmb {H} _ {k} \pmb {P} _ {k / k - 1} \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {K} _ {k} ^ {\mathrm{T}} + \pmb {K} _ {k} \pmb {R} _ {k} \pmb {K} _ {k} ^ {\mathrm{T}} =} \\ & {\qquad \pmb {P} _ {k / k - 1} - \pmb {P} _ {k / k - 1} \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {K _ {k}} ^ {\mathrm{T}} - \pmb {K} _ {k} \pmb {H} _ {k} \pmb {P} _ {k / k - 1} + \pmb {K} _ {k} (\pmb {H} _ {k} \pmb {P} _ {k / k - 1} \pmb {H} _ {k} ^ {\mathrm{T}} + \pmb {R} _ {k}) \pmb {K _ {k}} ^ {\mathrm{T}}} \end{array}
$$

上式右侧第4项中的 $K_{k}$ 用式(2.2.4c)代入，得

$$
\begin{array}{r l} & {\pmb {P} _ {k} = \pmb {P} _ {k / k - 1} - \pmb {P} _ {k / k - 1} \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {K} _ {k} ^ {\mathrm{T}} - \pmb {K} _ {k} \pmb {H} _ {k} \pmb {P} _ {k / k - 1} + \pmb {P} _ {k / k - 1} \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {K} _ {k} ^ {\mathrm{T}} =} \\ & {\qquad (\pmb {I} - \pmb {K} _ {k} \pmb {H} _ {k}) \pmb {P} _ {k / k - 1}} \end{array}\tag{2.2.4e'}
$$

根据式 $(2.2.4c)$ ，式 $(2.2.4e')$ 又可写成

$$
\boldsymbol {P} _ {k} = \boldsymbol {P} _ {k / k - 1} - \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \left(\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}\right) ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1}
$$

根据矩阵反演公式(2.1.23):

令

$$
\begin{array}{r l} & (A _ {1 1} - A _ {1 2} A _ {2 2} ^ {- 1} A _ {2 1}) ^ {- 1} = A _ {1 1} ^ {- 1} + A _ {1 1} ^ {- 1} A _ {1 2} (A _ {2 2} - A _ {2 1} A _ {1 1} ^ {- 1} A _ {1 2}) ^ {- 1} A _ {2 1} A _ {1 1} ^ {- 1} \\ & \quad P _ {k / k - 1} = A _ {1 1} ^ {- 1}, \quad - H _ {k} ^ {\mathrm{T}} = A _ {1 2}, \quad H _ {k} = A _ {2 1}, \quad R _ {k} = A _ {2 2} \end{array}
$$

则

$$
\boldsymbol {P} _ {k} = (\boldsymbol {P} _ {k / k - 1} ^ {- 1} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}) ^ {- 1}
$$

即

$$
\boldsymbol {P} _ {k} ^ {- 1} = \boldsymbol {P} _ {k / k - 1} ^ {- 1} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}\tag{2.2.4e \( ^{''} \)}
$$

又由式 $(2.2.4e^{\prime\prime})$ 可推导出式 $(2.2.4c)$ 的等价式 $(2.2.4c^{\prime})$ 。

先求解(2.2.4c)式中的 $\left(\boldsymbol{H}_{k}\boldsymbol{P}_{k/k-1}\boldsymbol{H}_{k}^{\mathrm{T}}+\boldsymbol{R}_{k}\right)^{-1}$ ，根据矩阵反演公式(2.1.23)，令 $R_{k}=A_{11}$ ， $H_{k}=-A_{12},P_{k/k-1}=A_{22}^{-1},H_{k}^{\mathrm{T}}=A_{21}$ ，则

$$
\begin{array}{r l} & (\mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} + \mathbf {R} _ {k}) ^ {- 1} = \mathbf {R} _ {k} ^ {- 1} + \mathbf {R} _ {k} ^ {- 1} (- \mathbf {H} _ {k}) [ \mathbf {P} _ {k / k - 1} ^ {- 1} - \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} (- \mathbf {H} _ {k}) ] ^ {- 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} = \\ & \quad \mathbf {R} _ {k} ^ {- 1} - \mathbf {R} _ {k} ^ {- 1} \mathbf {H} _ {k} (\mathbf {P} _ {k / k - 1} ^ {- 1} + \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} \mathbf {H} _ {k}) ^ {- 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} \end{array}
$$

式 $(2.2.4e^{\prime\prime})$ 代入上式，得

$$
\begin{array}{r l} & (\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}) ^ {- 1} = [ \boldsymbol {I} - \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} (\boldsymbol {P} _ {k} ^ {- 1}) ^ {- 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} ] \boldsymbol {R} _ {k} ^ {- 1} = \\ & (\boldsymbol {I} - \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}}) \boldsymbol {R} _ {k} ^ {- 1} \end{array}
$$

上式代入式(2.2.4c)，并根据式(2.2.4e"），得

$$
\begin{array}{r l} \boldsymbol {K} _ {k} & = \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {I} - \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}}) \boldsymbol {R} _ {k} ^ {- 1} = \boldsymbol {P} _ {k / k - 1} (\boldsymbol {P} _ {k} ^ {- 1} - \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}) \boldsymbol {P} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} = \\ & \boldsymbol {P} _ {k / k - 1} \boldsymbol {P} _ {k / k - 1} ^ {- 1} \boldsymbol {P} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} = \boldsymbol {P} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \end{array}\tag{2.2.4c'}
$$

式(2.2.4e)、式(2.2.4e')及式(2.2.4e'')是等价的，滤波过程中应根据实际情况选择使用。式(2.2.4e')形式简单，计算量较小，但计算中的积累误差容易使 $P_{k}$ 失去非负定性甚至对称性。所以实际使用中常用式(2.2.4e)。如果在滤波初始时刻对被估计量的统计特性缺乏了解，选取滤波初值盲目，相应地 $P_{0}$ 选得十分巨大，计算 $P_{1/0}$ 和 $K_{1}$ 就会有溢出的可能，在此情况下，宜采用式(2.2.4e'')，一步预测的均方误差阵也用逆阵来表示。这种逆阵称为信息矩阵，采用信息矩阵表示的滤波方程称为信息滤波方程。关于信息滤波将在第三章中介绍。

## 3. 一步转移阵和等效离散系统噪声方差阵的计算

由卡尔曼滤波的推导知，式(2.2.4)所示基本方程只适用于系统方程和量测方程都是离散型的情况，如式(2.2.1)和式(2.2.2)所示。但实际的物理系统一般都是连续的，动力学特性用连续微分方程描述。所以使用基本方程之前，必须对系统方程和量测方程作离散化处理。

设描述物理系统动力学特性的系统方程为

$$
\dot {\boldsymbol {X}} (t) = \boldsymbol {F} (t) \boldsymbol {X} (t) + \boldsymbol {G} (t) \boldsymbol {w} (t)\tag{2.2.14}
$$

其中系统的驱动源 $w(t)$ 为白噪声过程, 即

$$
\begin{array}{l} E [ \boldsymbol {w} (t) ] = \mathbf {0} \\ E [ \boldsymbol {w} (t) \boldsymbol {w} ^ {\mathrm{T}} (\tau) ] = \boldsymbol {q} \delta (t - \tau) \end{array}
$$

式中 q 为 $w(t)$ 的方差强度阵。

根据线性系统理论,系统方程的离散化形式为

$$
\boldsymbol {X} (t _ {k + 1}) = \boldsymbol {\Phi} (t _ {k + 1}, t _ {k}) \boldsymbol {X} (t _ {k}) + \int_ {t _ {k}} ^ {t _ {k + 1}} \boldsymbol {\Phi} (t _ {k + 1}, \tau) \boldsymbol {G} (\tau) w (\tau) \mathrm{d} \tau\tag{2.2.15}
$$

其中，一步转移阵 $\Phi (t_{k + 1},t_k)$ 满足方程

$$
\left. \begin{array}{l} \dot {\boldsymbol {\Phi}} (t, t _ {k}) = \boldsymbol {F} (t) \boldsymbol {\Phi} (t, t _ {k}) \\ \boldsymbol {\Phi} (t _ {k}, t _ {k}) = \boldsymbol {I} \end{array} \right\}\tag{2.2.16}
$$

求解该方程, 得

$$
\boldsymbol {\Phi} (t _ {k + 1}, t _ {k}) = \exp \int_ {t _ {k}} ^ {t _ {k + 1}} \boldsymbol {F} (t) \mathrm{d} t
$$

当滤波周期 $T(T=t_{k+1}-t_{k})$ 较短时， $F(t)$ 可近似看做常阵，即

$$
\mathbf {F} (t) \approx \mathbf {F} (t _ {k}) \qquad t _ {k} \leqslant t <   t _ {k + 1}
$$

此时

$$
\Phi (t _ {k + 1}, t _ {k}) = \mathrm{e} ^ {T F (t _ {k})}
$$

即

$$
\boldsymbol {\Phi} _ {k + 1, k} = \boldsymbol {I} + T \boldsymbol {F} _ {k} + \frac {T ^ {2}}{2 !} \boldsymbol {F} _ {k} ^ {2} + \frac {T ^ {3}}{3 !} \boldsymbol {F} _ {k} ^ {3} + \dots\tag{2.2.17}
$$

其中 $F_{k}=F(t_{k})$ 。

式 $(2.2.17)$ 就是一步转移阵的实时计算公式。

连续系统的离散化处理还包括对激励白噪声过程 $w(t)$ 的等效离散化处理。

令

$$
\boldsymbol {W} _ {k} = \int_ {t _ {k}} ^ {t _ {k + 1}} \boldsymbol {\Phi} (t _ {k + 1}, \tau) \boldsymbol {G} (\tau) w (\tau) \mathrm{d} \tau\tag{2.2.18}
$$

则式 $(2.2.15)$ 可写成

$$
\boldsymbol {X} (t _ {k + 1}) = \boldsymbol {\Phi} (t _ {k + 1}, t _ {k}) \boldsymbol {X} (t _ {k}) + \boldsymbol {W} _ {k}
$$

简写成

$$
\boldsymbol {X} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {X} _ {k} + \boldsymbol {W} _ {k}\tag{2.2.19}
$$

下面考察由式(2.2.18)定义的 $W_{k}$ 。

$$
E [ \pmb {W} _ {k} ] = \int_ {t _ {k}} ^ {t _ {k + 1}} \pmb {\Phi} (t _ {k + 1}, \tau) \pmb {G} (\tau) E [ \pmb {w} (\tau) ] \mathrm{d} \tau = \mathbf {0}\tag{2.2.20}
$$

$$
\begin{array}{r l} E [ \boldsymbol {W} _ {k} \boldsymbol {W} _ {j} ^ {\mathrm{T}} ] & = E \left[ \int_ {t _ {k}} ^ {t _ {k + 1}} \boldsymbol {\Phi} (t _ {k + 1}, t) \boldsymbol {G} (t) w (t) \mathrm{d} t \cdot \int_ {t _ {j}} ^ {t _ {j + 1}} w ^ {\mathrm{T}} (\tau) \boldsymbol {G} ^ {\mathrm{T}} (\tau) \boldsymbol {\Phi} ^ {\mathrm{T}} (t _ {k + 1}, \tau) \mathrm{d} \tau \right] = \\ & \quad \int_ {t _ {k}} ^ {t _ {k + 1}} \boldsymbol {\Phi} (t _ {k + 1}, t) \boldsymbol {G} (t) \left[ \int_ {t _ {j}} ^ {t _ {j + 1}} E [ w (t) w ^ {\mathrm{T}} (\tau) ] \cdot \boldsymbol {G} ^ {\mathrm{T}} (\tau) \boldsymbol {\Phi} ^ {\mathrm{T}} (t _ {k + 1}, \tau) \mathrm{d} \tau \right] \mathrm{d} t = \\ & \quad \int_ {t _ {k}} ^ {t _ {k + 1}} \boldsymbol {\Phi} (t _ {k + 1}, t) \boldsymbol {G} (t) \left[ \int_ {t _ {j}} ^ {t _ {j + 1}} q \delta (t - \tau) \boldsymbol {G} ^ {\mathrm{T}} (\tau) \boldsymbol {\Phi} (t _ {k + 1}, \tau) \mathrm{d} \tau \right] \mathrm{d} t \end{array}
$$

式中， $\delta(t - \tau)$ 为狄拉克 $\delta$ 函数， $t \in [t_k, t_{k+1}), \tau \in [t_j, t_{j+1})$ 。很明显，如果该两区间不重合，即 $j \neq k$ ，则 $t$ 和 $\tau$ 就不可能有相等的机会，此时积分值恒为零。而当两区间重合时，即 $j = k$ 时

$$
E \left[ \boldsymbol {W} _ {k} \boldsymbol {W} _ {j} ^ {\mathrm{T}} \right] | _ {j = k} = \int_ {t _ {k}} ^ {t _ {k + 1}} \boldsymbol {\Phi} (t _ {k + 1}, t) \boldsymbol {G} (t) \boldsymbol {q} \boldsymbol {G} ^ {\mathrm{T}} (t) \boldsymbol {\Phi} ^ {\mathrm{T}} (t _ {k + 1}, t) \mathrm{d} t
$$

所以

$$
E [ \boldsymbol {W} _ {k} \boldsymbol {W} _ {j} ^ {\mathrm{T}} ] = \boldsymbol {Q} _ {k} \delta_ {k j}\tag{2.2.21}
$$

其中

$$
\delta_ {k j} = \left\{ \begin{array}{l l} 1, & j = k \\ 0, & j \neq k \end{array} \right.
$$

$$
\boldsymbol {Q} _ {k} = \int_ {t _ {k}} ^ {t _ {k + 1}} \boldsymbol {\Phi} (t _ {k + 1}, t) \boldsymbol {G} (t) \boldsymbol {q} \boldsymbol {G} ^ {\mathrm{T}} (t) \boldsymbol {\Phi} ^ {\mathrm{T}} (t _ {k + 1}, t) \mathrm{d} t\tag{2.2.22}
$$

由式(2.2.20)和式(2.2.21)知， $W_{k}$ 为白噪声序列，所以式(2.2.19)描述的等效离散系统满足式(2.2.3)所示离散型卡尔曼滤波基本方程的要求。 $W_{k}$ 的方差阵 $Q_{k}$ 按式(2.2.22)求取。但若直接按式(2.2.22)计算 $Q_{k}$ ，在计算机上执行还存在诸多不便。下面通过对式(2.2.22)作恒等变形，推导出便于在计算机上执行的递推算法。

假设滤波周期 $T(T = t_{k + 1} - t_k)$ 较短， $\mathbf{G}(t)$ 在该周期内变化不大，即有

$$
\mathbf {G} (t) \approx \mathbf {G} _ {k} \qquad t _ {k} \leqslant t <   t _ {k + 1}
$$

式中 $\pmb{G}_k$ 是 $\pmb{G}(t_k)$ 的简写。记

$$
\bar {Q} = G _ {k} q G _ {k} ^ {\mathrm{T}}\tag{2.2.23}
$$

则由式(2.2.17)和式(2.2.22)有

$$
\begin{array} { r l }Q _ { k }&{ } = \int _ { t _ { k } } ^ { t _ { k + 1 } } \left[ I + F _ { k } ( t _ { k + 1 } - t ) + \frac { F _ { k } ^ { 2 } } { 2 ! } ( t _ { k + 1 } - t ) ^ { 2 } + \frac { F _ { k } ^ { 3 } } { 3 ! } ( t _ { k + 1 } - t ) ^ { 3 } + \cdots \right] \cdot\\&{ } \bar { Q } \left[ I + F _ { k } ( t _ { k + 1 } - t ) + \frac { F _ { k } ^ { 2 } } { 2 ! } ( t _ { k + 1 } - t ) ^ { 2 } + \frac { F _ { k } ^ { 3 } } { 3 ! } ( t _ { k + 1 } - t ) ^ { 3 } + \dots \right] ^ { T } d t =\\&{ } \int _ { t _ { k } } ^ { t _ { k + 1 } } \left[ \bar { Q } + \bar { Q } F _ { k } ^ { T } ( t _ { k + 1 } - t ) + \frac { 1 } { 2 ! } \bar { Q } F _ { k } ^ { 2 T } ( t _ { k + 1 } - t ) ^ { 2 } + \right.\\&{ } \frac { 1 } { 3 ! } \bar { Q } F _ { k } ^ { 3 T } ( t _ { k + 1 } - t ) ^ { 3 } + \cdots + F _ { k } \bar { Q } ( t _ { k + 1 } - t ) +\\&{ } F _ { k } \bar { Q } F _ { k } ^ { T } ( t _ { k + 1 } - t ) ^ { 2 } + \frac { 1 } { 2 ! } F _ { k } \bar { Q } F _ { k } ^ { 2 T } ( t _ { k + 1 } - t ) ^ { 3 } + \cdots +\\&{ } \frac { 1 } { 2 ! } F _ { k } ^ { 2 } \bar { Q } ( t _ { k + 1 } - t ) ^ { 2 } + \frac { 1 } { 2 ! } F _ { k } ^ { 2 } \bar { Q } F _ { k } ^ { T } ( t _ { k + 1 } - t ) ^ { 3 } + \cdots +\\&{ } \frac { 1 } { 3 ! } F _ { k } ^ { 3 } \bar { Q } ( t _ { k + 1 } - t ) ^ { 3 } + \frac { 1 } { 3 ! } F _ { k } ^ { 3 } \bar { Q } F _ { k } ^ { T } ( t _ { k + 1 } - t ) ^ { 4 } + \cdots ] d t =\\&{ } \int _ { t _ { k } } ^ { t _ { k + 1 } } \left\{ \bar { Q } + ( \bar { Q } F _ { k } ^ { T } + F _ { k } \bar { Q} ) ( t _ { k + 1 } - t ) + [ \frac { 1 } { 2 ! } ( \bar { Q } F _ { k } ^ { 2 T } + F _ { k } ^ { 2 } \bar { Q } + F _ { k } \bar { Q} F _ { k } ^ { T} ] ( t _ { k + 1 } - t ) ^ { 2 } + [ \frac { 1 } { 3 !} ( \bar { Q } F _ { k } ^ { 3 T } + F _ { k } ^ { 3 } \bar { Q} ) + \right.\\&{ } \left. \frac { 1 } { 2 ! } ( F _ { k } \bar { Q } F _ { k } ^ { 2 T } + F _ { k } ^ { 2 } \bar { Q } F _ { k } ^ { T} ) ] ( t _ { k + 1 } - t ) ^ { 3 } + \cdots \right\} d t =\\&{ } \bar { Q T} + \frac { T ^ { 2 }}{ 2 !} \left[ \frac { 1}{ 1 !} ( \bar { Q F _ { k }} ^ { T } + F _ { k} \bar { Q} ) \right] + \frac { T ^ { 3 }}{ 3 !} \left[ \frac { 1}{ 2 !} ( \bar { Q F _ { k }} ^ { 2 T } + F _ { k} ^ { 2} \bar { Q} ) + \right.\\&{ } \left. \frac { 1}{ 1 !   1 !} F _ { k} \bar { Q F _ { k }} ^ { T} \right] + \frac { T ^ { 4 }}{ 4 !} \left[ \frac { 1}{ 3 !} ( \bar { Q F _ { k }} ^ { 3 T } + F _ { k} ^ { 3} \bar { Q} ) + \right.\\&{ } \left. \right. \frac { 1}{ 2 !   1 !} ( F _ { k} \bar { Q F _ { k }} ^ { 2 T } + F _ { k} ^ { 2} \bar { Q F _ { k }} ^ { T} ] + \frac { T ^ { 5 }}{ 5 !} \left[ \frac { 1}{ 4 !} ( \bar { Q F _ { k }} ^ { 4 T } + F _ { k} ^ { 4} \bar { Q} ) + \right.\\&{ } \left. \right. \frac { 1}{ 3 !   1 !} ( F _ { k} \bar { Q F _ { k }} ^ { 3 T } + F _ { k} ^ { 3} \bar { Q F _ { k }} ^ { T} ) + \frac { 1}{ 2 !   2 !} F _ { k} ^ { 2} \bar { Q F _ { k }} ^ { 2 T} ] + \cdots =\\&{\cal T Q} + \frac {\cal T ^ { 2 }}{2 !} ( \bar {\cal Q} F _ {\mathrm{k}} ^ {\mathrm{T}} + F _ {\mathrm{k}} \bar {\cal Q}) + \frac {\cal T ^ {\mathrm{3}}}{3 !} ( \bar {\cal Q} F _ {\mathrm{k}} ^ {\mathrm{2T}} + F _ {\mathrm{k}} ^ {\mathrm{2}} \bar {\cal Q} +\\&\cal 2 F _ {\mathrm{k}} \bar {\cal Q} F _ {\mathrm{k}} ^ {\mathrm{T}}) + \frac {\cal T ^ {\mathrm{4}}}{4 !} ( \bar {\cal Q} F _ {\mathrm{k}} ^ {\mathrm{3T}} + F _ {\mathrm{k}} ^ {\mathrm{3}} \bar {\cal Q} + {\cal 3 F _ {\mathrm{k}} \bar {\cal Q} F _ {\mathrm{k}} ^ {\mathrm{2T}}} +\\&\cal 3 F _ {\mathrm{k}} ^ {\mathrm{2}} \bar {\cal Q} F _ {\mathrm{k}} ^ {\mathrm{T}}) + \frac {\cal T ^ {\mathrm{5}}}{5 !} ( \bar {\cal Q} F _ {\mathrm{k}} ^ {\mathrm{4T}} + F _ {\mathrm{k}} ^ {\mathrm{4}} \bar {\cal Q} + {\cal 4 F _ {\mathrm{k}} \bar {\cal Q} F _ {\mathrm{k}} ^ {\mathrm{3T}}} +\\&\cal 4 F _ {\mathrm{k}} ^ {\mathrm{3}} \bar {\cal Q} F _ {\mathrm{k}} ^ {\mathrm{T}} + {\cal 6 F _ {\mathrm{k}} ^ {\mathrm{2}} \bar {\cal Q} F _ {\mathrm{k}} ^ {\mathrm{2T}}} ) + \cdots .\end{array}
$$

记

$$
\boldsymbol {M} _ {1} = \bar {\boldsymbol {Q}}\tag{2.2.24}
$$

$$
\boldsymbol {M} _ {2} = \overline {{\boldsymbol {Q}}} \boldsymbol {F} _ {k} ^ {\mathrm{T}} + \boldsymbol {F} _ {k} \overline {{\boldsymbol {Q}}}\tag{2.2.25}
$$

$$
\boldsymbol {M} _ {3} = \overline {{\boldsymbol {Q}}} \boldsymbol {F} _ {k} ^ {2 \mathrm{T}} + \boldsymbol {F} _ {k} ^ {2} \overline {{\boldsymbol {Q}}} + 2 \boldsymbol {F} _ {k} \overline {{\boldsymbol {Q}}} \boldsymbol {F} _ {k} ^ {\mathrm{T}}\tag{2.2.26}
$$

$$
\boldsymbol {M} _ {4} = \overline {{\boldsymbol {Q}}} \boldsymbol {F} _ {k} ^ {3 \mathrm{T}} + \boldsymbol {F} _ {k} ^ {3} \overline {{\boldsymbol {Q}}} + 3 \boldsymbol {F} _ {k} \overline {{\boldsymbol {Q}}} \boldsymbol {F} _ {k} ^ {2 \mathrm{T}} + 3 \boldsymbol {F} _ {k} ^ {2} \overline {{\boldsymbol {Q}}} \boldsymbol {F} _ {k} ^ {\mathrm{T}}\tag{2.2.27}
$$

$$
\boldsymbol {M} _ {5} = \overline {{\boldsymbol {Q}}} \boldsymbol {F} _ {k} ^ {4 \mathrm{T}} + \boldsymbol {F} _ {k} ^ {4} \overline {{\boldsymbol {Q}}} + 4 \boldsymbol {F} _ {k} \overline {{\boldsymbol {Q}}} \boldsymbol {F} _ {k} ^ {3 \mathrm{T}} + 4 \boldsymbol {F} _ {k} ^ {3} \overline {{\boldsymbol {Q}}} \boldsymbol {F} _ {k} ^ {\mathrm{T}} + 6 \boldsymbol {F} _ {k} ^ {2} \overline {{\boldsymbol {Q}}} \boldsymbol {F} _ {k} ^ {2 \mathrm{T}}\tag{2.2.28}
$$

则

$$
Q _ {k} = \frac {T}{1 !} M _ {1} + \frac {T ^ {2}}{2 !} M _ {2} + \frac {T ^ {3}}{3 !} M _ {3} + \frac {T ^ {4}}{4 !} M _ {4} + \frac {T ^ {5}}{5 !} M _ {5} + \dots\tag{2.2.29}
$$

而

$$
\begin{array}{r l} & {\mathbf {F} _ {k} \mathbf {M} _ {1} + (\mathbf {F} _ {k} \mathbf {M} _ {1}) ^ {\mathrm{T}} = \mathbf {F} _ {k} \overline {{\mathbf {Q}}} + \overline {{\mathbf {Q}}} \mathbf {F} _ {k} ^ {\mathrm{T}}} \\ & {\mathbf {F} _ {k} \mathbf {M} _ {2} + (\mathbf {F} _ {k} \mathbf {M} _ {2}) ^ {\mathrm{T}} = \mathbf {F} _ {k} \overline {{\mathbf {Q}}} \mathbf {F} _ {k} ^ {\mathrm{T}} + \mathbf {F} _ {k} ^ {2} \overline {{\mathbf {Q}}} + (\mathbf {F} _ {k} \overline {{\mathbf {Q}}} \mathbf {F} _ {k} ^ {\mathrm{T}} + \mathbf {F} _ {k} ^ {2} \overline {{\mathbf {Q}}}) ^ {\mathrm{T}} =} \\ & {\quad \mathbf {F} _ {k} ^ {2} \overline {{\mathbf {Q}}} + \overline {{\mathbf {Q}}} \mathbf {F} _ {k} ^ {2 \mathrm{T}} + 2 \mathbf {F} _ {k} \overline {{\mathbf {Q}}} \mathbf {F} _ {k} ^ {\mathrm{T}}} \\ & {\mathbf {F} _ {k} \mathbf {M} _ {3} + (\mathbf {F} _ {k} \mathbf {M} _ {3}) ^ {\mathrm{T}} = \mathbf {F} _ {k} \overline {{\mathbf {Q}}} \mathbf {F} _ {k} ^ {2 \mathrm{T}} + \mathbf {F} _ {k} ^ {3} \overline {{\mathbf {Q}}} + 2 \mathbf {F} _ {k} ^ {2} \overline {{\mathbf {Q}}} \mathbf {F} _ {k} ^ {\mathrm{T}} +} \\ & {\quad (\mathbf {F} _ {k} \overline {{\mathbf {Q}}} \mathbf {F} _ {k} ^ {2 \mathrm{T}} + \mathbf {F} _ {k} ^ {3} \overline {{\mathbf {Q}}} + 2 \mathbf {F} _ {k} ^ {2} \overline {{\mathbf {Q}}} \mathbf {F} _ {k} ^ {- 1}) ^ {\mathrm{T}} =} \\ & {\quad \mathbf {F} _ {k} ^ {3} \overline {{\mathbf {Q}}} + \overline {{\mathbf {Q}}} \mathbf {F} _ {k} ^ {3 \mathrm{T}} + 3 \mathbf {F} _ {k} ^ {2} \overline {{\mathbf {Q}}} \mathbf {F} _ {k} ^ {- 1} + 3 \mathbf {F} _ {k} \overline {{\mathbf {Q}}} \mathbf {F} _ {k} ^ {- 2 \mathrm{T}}} \\ & {\mathbf {F} _ {k} \mathbf {M} _ {4} + (\mathbf {F} _ {k} \mathbf {M} _ {4}) ^ {\mathrm{T}} = \mathbf {F} _ {k} \overline {{\mathbf {Q}}} \mathbf {F} _ {k} ^ {- 3 \mathrm{T}} + \mathbf {F} _ {k} ^ {- 4} \overline {{\mathbf {Q}}} + 3 \mathbf {F} _ {k} ^ {- 2} \overline {{\mathbf {Q}}} \mathbf {F} _ {- k} ^ {- 2 \mathrm{T}} + 3 \mathbf {F} _ {- k} ^ {- 3} \overline {{\mathbf {Q}}} \mathbf {F} _ {- k} ^ {- 1}} \\ & {\quad (\mathbf {F} _ {- k} \overline {{\mathbf {Q}}} \mathbf {F} _ {- k} ^ {- 3 \mathrm{T}} + \mathbf {F} _ {- k} ^ {- 4} \overline {{\mathbf {Q}}} + 3 \mathbf {F} _ {- k} ^ {- 2} \overline {{\mathbf {Q}}} \mathbf {F} _ {- k} ^ {- 2 \mathrm{T}} + 3 \mathbf {F} _ {- k} ^ {- 3} \overline {{\mathbf {Q}}} \mathbf {F} _ {- k}) ^ {\mathrm{T}} =} \\ & {\quad \mathbf {F} _ {- k} ^ {- 4} \overline {{\mathbf {Q}}} + \overline {{\mathbf {Q}}} \mathbf {F} _ {- k} ^ {- 4 \mathrm{T}} + 4 \mathbf {F} _ {- k} \overline {{\mathbf {Q}}} \mathbf {F} _ {- k - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1}} \\ & {\dots ..}. \end{array}
$$

上面诸式分别与式(2.2.24)～式(2.2.28)相比较，得

$$
\begin{array}{r l} & {\pmb {M} _ {2} = \pmb {F} _ {k} \pmb {M} _ {1} + (\pmb {F} _ {k} \pmb {M} _ {1}) ^ {\mathrm{T}}} \\ & {\pmb {M} _ {3} = \pmb {F} _ {k} \pmb {M} _ {2} + (\pmb {F} _ {k} \pmb {M} _ {2}) ^ {\mathrm{T}}} \\ & {\pmb {M} _ {4} = \pmb {F} _ {k} \pmb {M} _ {3} + (\pmb {F} _ {k} \pmb {M} _ {3}) ^ {\mathrm{T}}} \\ & {\pmb {M} _ {5} = \pmb {F} _ {k} \pmb {M} _ {4} + (\pmb {F} _ {k} \pmb {M} _ {4}) ^ {\mathrm{T}}} \\ & {\dots \dots} \end{array}
$$

即

$$
\boldsymbol {M} _ {i + 1} = \boldsymbol {F} _ {k} \boldsymbol {M} _ {i} + (\boldsymbol {F} _ {k} \boldsymbol {M} _ {i}) ^ {\mathrm{T}} \quad i = 1, 2, 3, \dots\tag{2.2.30}
$$

式中 $M_{1}$ 按式(2.2.24) 确定。

由式 $(2.2.29)$ 可得

$$
Q _ {k} = \sum_ {i = 1} ^ {\infty} M _ {i} \frac {T ^ {i}}{i !}\tag{2.2.31}
$$

式中 $M_{i}$ 按式(2.2.30)递推计算。

按式(2.2.17)计算一步转移阵和按式(2.2.31)计算等效离散系统噪声方差阵可用图2.2.3所示计算框图表示。

如果滤波周期较长,且系统矩阵 $F(t)$ 随时间变化较剧烈,而 $F(t)$ 在一个滤波周期内能得到 N 个采样值,则计算一步转移阵和等效离散系统噪声方差阵时可利用这些采样值,以简化算法。

设滤波周期为 $T(T = t_{k + 1} - t_k)$ ，在 $[t_k,t_{k + 1})$ 内每隔 $\Delta T = \frac{T}{N}$ 就能得到系统阵的采样值 $\pmb {F}(t_k + i\Delta T)$ ，并记

$$
\boldsymbol {F} _ {k} (i) = \boldsymbol {F} (t _ {k} + i \Delta T) \quad i = 0, 1, 2, \dots , N - 1
$$

则 $t_{k+1}=t_{k}+N\Delta T$ ，并且一步转移阵 $\Phi_{k+1,k}$ 和等效离散系统噪声方程阵 $Q_{k}$ 可按如下方法作简化计算。

![](images/5f0ba5ef7c81422f4f1cdb87598aa59ac80e89fdc076174982e3f0cf0fd01c3b.jpg)  
图 2.2.3 一步转移阵和等效离散系统噪声方差阵计算框图

(1) 一步转移阵的简化算法。由于

$$
\begin{array}{r l} \boldsymbol {\Phi} _ {k + 1, k} & = \boldsymbol {\Phi} [ t _ {k} + N \Delta T, t _ {k} + (N - 1) \Delta T ] \cdot \boldsymbol {\Phi} [ t _ {k} + (N - 1) \Delta T, t _ {k} + (N - 2) \Delta T ] \dots \\ & \boldsymbol {\Phi} [ t _ {k} + \Delta T, t _ {k} ] \end{array}
$$

$\Phi[t_{k}+(i+1)\Delta T,t_{k}+i\Delta T]$ ( $i=0,1,2,\cdots,N-1$ ) 的转移时间间隔为 $\Delta T=\frac{T}{N}$ ，若 N 很大，则 $\Delta T$ 很小，该转移阵的收敛速度远比 $\Phi_{k+1,k}$ 快，级数中关于 $\Delta T$ 的高次项可略去不计，所以

$$
\begin{array}{r l} \boldsymbol {\Phi} _ {k + 1, k} & \approx [ \boldsymbol {I} + \Delta T \boldsymbol {F} _ {k} (N - 1) ] [ \boldsymbol {I} + \Delta T \boldsymbol {F} _ {k} (N - 2) ] \dots \\ & [ \boldsymbol {I} + \Delta T \boldsymbol {F} _ {k} (0) ] = \boldsymbol {I} + \Delta T \sum_ {i = 0} ^ {N - 1} \boldsymbol {F} _ {k} (i) \end{array}\tag{2.2.32}
$$

式中略去了乘积中关于 $\Delta T$ 的高次项。

(2) 等效离散系统噪声方差阵的简化算法。

由式 $(2.2.22)$

$$
Q _ {k} = \int_ {t _ {k}} ^ {t _ {k + 1}} \boldsymbol {\Phi} (t _ {k + 1}, t) \boldsymbol {G} (t) \boldsymbol {q} \boldsymbol {G} ^ {\mathrm{T}} (t) \boldsymbol {\Phi} ^ {\mathrm{T}} (t _ {k + 1}, t) \mathrm{d} t
$$

在时间段 $[t_k, t_{k+1})$ 内，取 $G(t) \approx G(t_k)$ ，并记

$$
\bar {Q} = G (t _ {k}) q G ^ {\mathrm{T}} (t _ {k})
$$

则

$$
Q _ {k} = \int_ {t _ {k}} ^ {t _ {k + 1}} \boldsymbol {\Phi} (t _ {k + 1}, t) \overline {{\boldsymbol {Q}}} \boldsymbol {\Phi} ^ {\mathrm{T}} (t _ {k + 1}, t) \mathrm{d} t = \sum_ {i = 0} ^ {N - 1} \int_ {t _ {k} + i \Delta T} ^ {t _ {k} + (i + 1) \Delta T} \boldsymbol {\Phi} (t _ {k + 1}, t) \overline {{\boldsymbol {Q}}} \boldsymbol {\Phi} ^ {\mathrm{T}} (t _ {k + 1, t}) \mathrm{d} t\tag{2.2.33}
$$

记

$$
\boldsymbol {I} (i) = \int_ {t _ {k} + i \Delta T} ^ {t _ {k} + (i + 1) \Delta T} \boldsymbol {\Phi} (t _ {k + 1}, t) \overline {{\boldsymbol {Q}}} \boldsymbol {\Phi} ^ {\mathrm{T}} (t _ {k + 1}, t) \mathrm{d} t
$$

由于式中

$$
\begin{array}{r l} \boldsymbol {\Phi} (t _ {k + 1}, t) & = \boldsymbol {\Phi} [ t _ {k} + N \Delta T, t _ {k} + (N - 1) \Delta T ] \cdot \boldsymbol {\Phi} [ t _ {k} + (N - 1) \Delta T, t _ {k} + (N - 2) \Delta T ] \dots \\ & \quad \boldsymbol {\Phi} [ t _ {k} + (i + 2) \Delta T, t _ {k} + (i + 1) \Delta T ] \cdot \boldsymbol {\Phi} [ t _ {k} + (i + 1) \Delta T, t ] \approx \\ & [ I + \Delta T F _ {k} (N - 1) ] [ I + \Delta T F _ {k} (N - 2) ] \dots [ I + \Delta T F _ {k} (i + 1) ] \cdot \\ & \{I + [ t _ {k} + (i + 1) \Delta T - t ] F _ {k} (i) \} \end{array}
$$

仅保留式 $(2.2.33)$ 被积函数中关于时间间隔的线性项，则

$$
\begin{array}{r l} \boldsymbol {I} (i) & \approx \int_ {t _ {k + i \Delta T}} ^ {t _ {k + (i + 1) \Delta T}} \{\overline {{Q}} + \Delta T [ F _ {k} (N - 1) + F _ {k} (N - 2) + \dots + \\ & \quad F _ {k} (i + 1) ] \overline {{Q}} + [ t _ {k} + (i + 1) \Delta T - t ] F _ {k} (i) \overline {{Q}} + \\ & \quad \Delta T \overline {{Q}} [ F _ {k} ^ {\mathrm{T}} (N - 1) + F _ {k} ^ {\mathrm{T}} (N - 2) + \dots + F _ {k} ^ {\mathrm{T}} (i + 1) ] + \\ & \quad \overline {{Q}} [ t _ {k} + (i + 1) \Delta T - t ] F _ {k} ^ {\mathrm{T}} (i) \} \mathrm{d} t = \\ & \quad \overline {{Q}} \Delta T + \Delta T ^ {2} \{[ F _ {k} (N - 1) + F _ {k} (N - 2) + \dots + F _ {k} (i + 1) ] \overline {{Q}} + \\ & \quad \overline {{Q}} [ F _ {k} ^ {\mathrm{T}} (N - 1) + F _ {k} ^ {\mathrm{T}} (N - 2) + \dots + F _ {k} ^ {\mathrm{T}} (i + 1) ] \} + \\ & \quad \frac {\Delta T ^ {2}}{2} F _ {k} (i) \overline {{Q}} + \frac {\Delta T ^ {2}}{2} \overline {{Q}} F _ {k} ^ {\mathrm{T}} (i) \qquad i = 0, 1, 2, \dots , N - 1 \end{array}
$$

由上式得

$$
\begin{array}{r l} & {\pmb {I} (0) = \overline {{{\pmb {Q}}}} \Delta T + \Delta T ^ {2} \left\{\left[ \pmb {F} _ {k} (N - 1) + \pmb {F} _ {k} (N - 2) + \dots + \pmb {F} _ {k} (1) + \frac {1}{2} \pmb {F} _ {k} (0) \right] \overline {{{\pmb {Q}}}} + \right.} \\ & {\qquad \left. \overline {{{\pmb {Q}}}} \left[ \pmb {F} _ {k} ^ {\mathrm{T}} (N - 1) + \pmb {F} _ {k} ^ {\mathrm{T}} (N - 2) + \dots + \pmb {F} _ {k} ^ {\mathrm{T}} (1) + \frac {1}{2} \pmb {F} _ {k} ^ {\mathrm{T}} (0) \right] \right\}} \\ & {\pmb {I} (1) = \overline {{{\pmb {Q}}}} \Delta T + \Delta T ^ {2} \left\{\left[ \pmb {F} _ {k} (N - 1) + \pmb {F} _ {k} (N - 2) + \dots + \pmb {F} _ {k} (2) + \frac {1}{2} \pmb {F} _ {k} (1) \right] \overline {{{\pmb {Q}}}} + \right.} \\ & {\qquad \left. \overline {{{\pmb {Q}}}} \left[ \pmb {F} _ {k} ^ {\mathrm{T}} (N - 1) + \pmb {F} _ {k} ^ {\mathrm{T}} (N - 2) + \dots + \pmb {F} _ {k} (\mathrm{2}) + \frac {1}{2} \pmb {F} _ {k} ^ {\mathrm{T}} (1) \right] \right\}} \end{array}
$$

$$
\begin{array}{r l} & {\pmb {I} (N - 2) = \overline {{{\pmb {Q}}}} \Delta T + \Delta \pmb {T} ^ {2} \left\{\left[ \pmb {F} _ {k} (N - 1) + \frac {1}{2} \pmb {F} _ {k} (N - 2) \right] \overline {{{\pmb {Q}}}} + \right.} \\ & {\qquad \left. \overline {{{\pmb {Q}}}} \left[ \pmb {F} _ {k} ^ {\mathrm{T}} (N - 1) + \frac {1}{2} \pmb {F} _ {k} ^ {\mathrm{T}} (N - 2) \right] \right\}} \\ & {\pmb {I} (N - 1) = \overline {{{\pmb {Q}}}} \Delta T + \Delta T ^ {2} \left[ \frac {1}{2} \pmb {F} _ {k} (N - 1) \overline {{{\pmb {Q}}}} + \frac {1}{2} \overline {{{\pmb {Q}}}} \pmb {F} _ {k} ^ {\mathrm{T}} (n - 1) \right]} \end{array}
$$

所以

$$
\begin{array}{r l} & {\pmb {Q} _ {k} = \pmb {I} (0) + \pmb {I} (1) + \dots + \pmb {I} (N - 2) + \pmb {I} (N - 1) = N \Delta T \overline {{{\pmb {Q}}}} +} \\ & {\qquad \Delta T ^ {2} \left\{\left[ \left(N - 1 + \frac {1}{2}\right) \pmb {F} _ {k} (N - 1) + \left(N - 2 + \frac {1}{2}\right) \pmb {F} _ {k} (N - 2) + \dots + \right. \right.} \end{array}
$$

$$
\begin{array}{l} \left(1 + \frac {1}{2}\right) \boldsymbol {F} _ {k} (1) + \frac {1}{2} \boldsymbol {F} _ {k} (0) \Big ] \overline {{\boldsymbol {Q}}} + \overline {{\boldsymbol {Q}}} \Big [ \left(N - 1 + \frac {1}{2}\right) \boldsymbol {F} _ {k} ^ {\mathrm{T}} (N - 1) + \\ \left(N - 1 + \frac {1}{2}\right) \boldsymbol {F} _ {k} ^ {\mathrm{T}} (N - 2) + \dots + \left(1 + \frac {1}{2}\right) \boldsymbol {F} _ {k} ^ {\mathrm{T}} (1) + \frac {1}{2} \boldsymbol {F} _ {k} ^ {\mathrm{T}} (0) \Big ] \Big \} = \\ T \overline {{\boldsymbol {Q}}} + \Delta T ^ {2} \left\{\left[ \sum_ {i = 0} ^ {N - 1} \left(i + \frac {1}{2}\right) \boldsymbol {F} _ {k} (i) \right] \overline {{\boldsymbol {Q}}} + \overline {{\boldsymbol {Q}}} \left[ \sum_ {i = 0} ^ {N - 1} \left(i + \frac {1}{2}\right) \boldsymbol {F} _ {k} ^ {\mathrm{T}} (i) \right] \right\} \end{array}\tag{2.2.34}
$$

考察式(2.2.32)和式(2.2.34)可看出,计算一步转移阵 $\Phi_{k+1,k}$ 所需的矩阵运算仅需作N步矩阵求和,计算等效离散系统噪声方差阵所需的矩阵运算为作N步矩阵求和后作一步矩阵相乘,与式(2.2.31)相比计算量大为减少,这对工程实现十分有利。此外,与式(2.2.17)和式(2.2.31)仅用到 $t_{k}$ 时刻系统的动态信息相比,简化算法中利用了 $[t_{k},t_{k+1})$ 内N次采样获得的系统动态信息,这在一定程度上弥补了级数过早截断引起的精度损失。

## 4. 系统有确定性控制时的滤波基本方程

设系统的状态方程除有白噪声驱动项外,还有确定性驱动项

$$
\boldsymbol {X} _ {k} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {X} _ {k - 1} + \boldsymbol {B} _ {k - 1} \boldsymbol {u} _ {k - 1} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {W} _ {k - 1}
$$

根据线性最小方差估计的性质3，得

$$
\hat {\boldsymbol {X}} _ {k, k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \hat {\boldsymbol {X}} _ {k - 1} + \boldsymbol {B} _ {k - 1} \boldsymbol {u} _ {k - 1}\tag{2.2.35}
$$

所以状态估计方程为

$$
\hat {\boldsymbol {X}} _ {k} = \boldsymbol {\Phi} _ {k, k - 1} \hat {\boldsymbol {X}} _ {k - 1} + \boldsymbol {B} _ {k - 1} \boldsymbol {u} _ {k - 1} + \boldsymbol {K} _ {k} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\Phi} _ {k, k - 1} \hat {\boldsymbol {X}} _ {k - 1} - \boldsymbol {H} _ {k} \boldsymbol {B} _ {k - 1} \boldsymbol {u} _ {k - 1})\tag{2.2.36}
$$

式中 $K_{k}, P_{k/k-1}, P_{k}$ 的计算与式(2.2.4)相同。

## 5. 一步预测基本方程

一步预测基本方程是指利用 $Z_{k}$ 和 $\hat{X}_{k/k-1}$ 递推计算 $\hat{X}_{k+1/k}$ 的全套方程。由式(2.2.4a)和式(2.2.4b)得

$$
\hat {\mathbf {X}} _ {k + 1 / k} = \boldsymbol {\Phi} _ {k + 1, k} \hat {\mathbf {X}} _ {k} = \boldsymbol {\Phi} _ {k + 1, k} \left[ \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {K} _ {k} \left(\mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1}\right) \right]
$$

令

$$
\boldsymbol {K} _ {k} ^ {*} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {K} _ {k}\tag{2.2.37}
$$

则

$$
\hat {\mathbf {X}} _ {k + 1 / k} = \boldsymbol {\Phi} _ {k + 1, k} \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {K} _ {k} ^ {*} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1})\tag{2.2.38}
$$

式中

$$
\boldsymbol {K} _ {k} ^ {*} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}) ^ {- 1}\tag{2.2.39}
$$

根据式(2.2.4d)、式(2.2.4e')和式(2.2.37)，得

$$
\boldsymbol {P} _ {k + 1, k} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + \boldsymbol {\Gamma} _ {k} \boldsymbol {Q} _ {k} \boldsymbol {\Gamma} _ {k} ^ {\mathrm{T}} = \boldsymbol {\Phi} _ {k + 1, k} (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) \boldsymbol {P} _ {k / k - 1} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + \boldsymbol {\Gamma} _ {k} \boldsymbol {Q} _ {k} \boldsymbol {\Gamma} _ {k} ^ {\mathrm{T}} =
$$

$$
\left(\boldsymbol {\Phi} _ {k + 1, k} - \boldsymbol {K} _ {k} ^ {*} \boldsymbol {H} _ {k}\right) \boldsymbol {P} _ {k / k - 1} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + \boldsymbol {\Gamma} _ {k} \boldsymbol {Q} _ {k} \boldsymbol {\Gamma} _ {k} ^ {\mathrm{T}}\tag{2.2.40}
$$

式 $(2.2.38)$ 、式 $(2.2.39)$ 和式 $(2.2.40)$ 即为一步预测基本方程。

例2-12 设有线性定常系统

$$
\begin{array}{r} X _ {k} = \Phi X _ {k - 1} + W _ {k - 1} \\ Z _ {k} = X _ {k} + V _ {k} \end{array}
$$

式中状态变量 $X_{k}$ 与量测 $Z_{k}$ 均为标量， $\Phi$ 为常数。 $\{W_{k}\}$ 和 $\{V_{k}\}$ 为零均值的白噪声序列，分别具有协方差

$$
E [ W _ {k} W _ {j} ] = Q \delta_ {k j}, E [ V _ {k} V _ {j} ] = R \delta_ {k j}
$$

并且 $\{W_k\}, \{V_k\}, X_0$ 三者互不相关。求 $\hat{X}_k$ 的递推方程。

解 根据式 $(2.2.4)$ ，得如下关系式

$$
\hat {X} _ {k / k - 1} = \Phi \hat {X} _ {k - 1}\tag{1}
$$

$$
\hat {X} _ {k} = \hat {X} _ {k / k - 1} + K _ {k} (Z _ {k} - \hat {X} _ {k / k - 1}) = (1 - K _ {k}) \hat {X} _ {k / k - 1} + K _ {k} Z _ {k}\tag{2}
$$

$$
K _ {k} = P _ {k / k - 1} (P _ {k / k - 1} + R) ^ {- 1} = \frac {P _ {k / k - 1}}{P _ {k / k - 1} + R}\tag{3}
$$

$$
P _ {k / k - 1} = \Phi^ {2} P _ {k - 1} + Q\tag{4}
$$

$$
P _ {k} = (1 - K _ {k}) P _ {k / k - 1} = \left(1 - \frac {P _ {k / k - 1}}{P _ {k / k - 1} + R}\right) P _ {k / k - 1} = \frac {R P _ {k / k - 1}}{P _ {k / k - 1} + R} = R K _ {k}\tag{5}
$$

由式(2)知， $K_{k}$ 实际上决定了对量测值 $Z_{k}$ 和上一步估计值 $\hat{X}_{k-1}$ 利用的比例程度。若 $K_{k}$ 增大，则 $Z_{k}$ 的利用权重增大，而 $\hat{X}_{k-1}$ 的利用权重相对降低。又由式(3)和式(4)知， $K_{k}$ 由量测噪声方差 R 和上一步估计的均方误差 $P_{k-1}$ 决定。假设 Q 一定，k 时刻的估计精度较高，由(4)式确定的 $P_{k/k-1}$ 较小，若量测精度很差，即 R 很大，则 $K_{k}$ 很小，结果是对 $Z_{k}$ 的利用权重减小，而对 $\hat{X}_{k-1}$ 的利用权重增大。若 $\hat{X}_{k-1}$ 精度很差，即 $P_{k-1}$ 很大，而量测精度很高，即 R 很小，则 $K_{k}$ 变大，确定 $\hat{X}_{k}$ 时对 $Z_{k}$ 的利用权重增大，而对 $\hat{X}_{k-1}$ 的利用权重相对减小。因此卡尔曼滤波能定量识别各种信息的质量，自动确定对这些信息的利用程度，是一种具有一定智能功能的算法。

例2-13 $\alpha -\beta -\gamma$ 滤波。

设运动体沿某一直线运动，t 时刻的位移、速度、加速度和加加速度分别为 $s(t)$ ， $v(t)$ ， $a(t)$ ， $j(t)$ ， $t_{k}$ 时刻的量测为 $Z_{k}=s_{k}+V_{k}$ ，其中 $s_{k}=s(t_{k})$ ， $k=1,2,3,\cdots$ 。运动体机动运动引起的运动不定性用 $j(t)$ 描述，其中

$$
E [ j (t) ] = 0, \quad E [ j (t) j (\tau) ] = q \delta (t - \tau)
$$

$$
E [ V _ {k} ] = 0, \quad E [ V _ {k} V _ {l} ] = r \delta_ {k l}
$$

量测量的采样周期为 T。求对 $s_{k}, v_{k}, a_{k}$ 的估计。

解 根据 $\dot{s}(t)=v(t)$ , $\dot{v}(t)=a(t)$ , $\dot{a}(t)=j(t)$ , 可列写出状态方程:

$$
\dot {\boldsymbol {X}} (t) = \boldsymbol {F} \boldsymbol {X} (t) + \boldsymbol {G} j (t)
$$

其中

$$
\mathbf {X} (t) = \left[ \begin{array}{l} s (t) \\ v (t) \\ a (t) \end{array} \right], \quad \mathbf {F} = \left[ \begin{array}{l l l} 0 & 1 & 0 \\ 0 & 0 & 1 \\ 0 & 0 & 0 \end{array} \right], \quad \mathbf {G} = \left[ \begin{array}{l} 0 \\ 0 \\ 1 \end{array} \right]
$$

对上式作离散化处理,得

$$
\mathbf {X} _ {k} = \boldsymbol {\Phi} _ {k, k - 1} \mathbf {X} _ {k - 1} + \int_ {t _ {k - 1}} ^ {t _ {k}} \boldsymbol {\Phi} (t _ {k}, \tau) \mathbf {G} j (\tau) \mathrm{d} \tau\tag{1}
$$

其中

$$
\Phi_ {k, k - 1} = I + F T + \frac {T ^ {2}}{2 !} F ^ {2} + \frac {T ^ {3}}{3 !} F ^ {3} + \dots =
$$

$$
\left[ \begin{array}{l l l} 1 & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{array} \right] + \left[ \begin{array}{l l l} 0 & T & 0 \\ 0 & 0 & T \\ 0 & 0 & 0 \end{array} \right] + \left[ \begin{array}{l l l} 0 & 0 & \frac {T ^ {2}}{2 !} \\ 0 & 0 & 0 \\ 0 & 0 & 0 \end{array} \right] = \left[ \begin{array}{l l l} 1 & T & \frac {T ^ {2}}{2} \\ 0 & 1 & T \\ 0 & 0 & 1 \end{array} \right]\tag{2}
$$

$$
\int_ {t _ {k - 1}} ^ {t _ {k}} \pmb {\Phi} (t _ {k}, \tau) \pmb {G} j (\tau) \mathrm{d} \tau \approx \int_ {t _ {k - 1}} ^ {t _ {k}} \pmb {\Phi} (t _ {k}, \tau) \pmb {G} \mathrm{d} \tau \cdot j _ {k - 1} =
$$

$$
\int_ {t _ {k - 1}} ^ {t _ {k}} \left[ \begin{array}{c c c} 1 & t _ {k} - \tau & \frac {(t _ {k} - \tau) ^ {2}}{2} \\ 0 & 1 & t _ {k} - \tau \\ 0 & 0 & 1 \end{array} \right] \left[ \begin{array}{l} 0 \\ 0 \\ 1 \end{array} \right] \mathrm{d} \tau \cdot j _ {k - 1} = \left[ \begin{array}{l} \frac {T ^ {3}}{6} \\ \frac {T ^ {2}}{2} \\ T \end{array} \right] j _ {k - 1}
$$

其中

$$
j _ {k - 1} = \frac {1}{T} \int_ {t _ {k - 1}} ^ {t _ {k}} j (\tau) \mathrm{d} \tau\tag{3}
$$

即 $j(t)$ 在 $[t_{k - 1}, t_k]$ 内的平均值。由式(3)，

$$
E [ j _ {k} ] = \frac {1}{T} \int_ {t _ {k}} ^ {t _ {k + 1}} E [ j (\tau) ] \mathrm{d} \tau = 0
$$

$$
E \left[ j _ {k} j _ {l} \right] = \frac {1}{T ^ {2}} \int_ {t _ {k}} ^ {t _ {k + 1}} \int_ {t _ {l}} ^ {t _ {l + 1}} E \left[ j (t) \cdot j (\tau) \right] \mathrm{d} t \mathrm{d} \tau = \frac {1}{T ^ {2}} \int_ {t _ {k}} ^ {t _ {k + 1}} \int_ {t _ {l}} ^ {t _ {l + 1}} q \delta (t - \tau) \mathrm{d} t \mathrm{d} \tau
$$

显然，只有当 $[t_l, t_{l+1}]$ 和 $[t_k, t_{k+1}]$ 重合时才有可能 $t = \tau$ ，所以当 $l = k$ 时，

$$
E [ j _ {k} ^ {2} ] = \frac {1}{T ^ {2}} \int_ {t _ {k}} ^ {t _ {k + 1}} q \mathrm{d} \tau = \frac {q}{T}
$$

当 $l \neq k$ 时， $E[j_{k}j_{l}] = 0$ 。可见 $E[j_{k}j_{l}] = \frac{q}{T}\delta_{kl}$ ，由式(3)确定的序列为白噪声，其方差为 $Q = \frac{q}{T}$ 。

所以式(1)可写成

$$
\boldsymbol {X} _ {k} = \boldsymbol {\Phi} \boldsymbol {X} _ {k - 1} + \boldsymbol {\Gamma} j _ {k - 1}
$$

此即系统方程,其中

$$
\mathbf {X} _ {k} = \left[ \begin{array}{l} s _ {k} \\ v _ {k} \\ a _ {k} \end{array} \right], \quad \boldsymbol {\Phi} = \left[ \begin{array}{l l l} 1 & T & \frac {T ^ {2}}{2} \\ 0 & 1 & T \\ 0 & 0 & 1 \end{array} \right], \quad \boldsymbol {\Gamma} = \left[ \begin{array}{l} \frac {T ^ {3}}{6} \\ \frac {T ^ {2}}{2} \\ 1 \end{array} \right], \quad Q = \frac {q}{T}
$$

量测方程为

$$
Z _ {k} = s _ {k} + V _ {k} = \mathbf {H X} _ {k} + V _ {k}
$$

其中 $H = \begin{bmatrix} 1 & 0 & 0 \end{bmatrix}$ 。

可见这是定常系统的滤波问题。应用卡尔曼滤波基本方程

$$
\hat {\mathbf {X}} _ {k} = \boldsymbol {\Phi} \hat {\mathbf {X}} _ {k - 1} + \mathbf {K} _ {k} (Z _ {k} - H \boldsymbol {\Phi} \hat {\mathbf {X}} _ {k - 1})\tag{4}
$$

$$
\boldsymbol {P} _ {k / k - 1} = \boldsymbol {\Phi} \boldsymbol {P} _ {k - 1} \boldsymbol {\Phi} ^ {\mathrm{T}} + \boldsymbol {\Gamma} \boldsymbol {Q} \boldsymbol {\Gamma} ^ {\mathrm{T}}\tag{5}
$$

$$
\boldsymbol {P} _ {k} = (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H}) \boldsymbol {P} _ {k / k - 1}\tag{6}
$$

$$
\mathbf {K} _ {k} = \mathbf {P} _ {k} \mathbf {H} ^ {\mathrm{T}} \mathbf {R} ^ {- 1}\tag{7}
$$

式中

$$
Q = \frac {q}{T}, \quad R = r
$$

当滤波达到稳态时， $P_{k}=P_{k-1}=P$ 为定值，此时，由式(6)、(7)和式(5)，得

$$
\boldsymbol {P} = (\boldsymbol {I} - \boldsymbol {P} \boldsymbol {H} ^ {\mathrm{T}} r ^ {- 1} \boldsymbol {H}) (\boldsymbol {\Phi} \boldsymbol {P} \boldsymbol {\Phi} ^ {\mathrm{T}} + \boldsymbol {\Gamma} \frac {q}{T} \boldsymbol {\Gamma} ^ {\mathrm{T}})
$$

这是关于 P 的矩阵代数方程,从中可解出 P,假设解得

$$
\mathbf {P} = \left[ \begin{array}{l l l} P _ {1 1} & P _ {1 2} & P _ {1 3} \\ P _ {2 1} & P _ {2 2} & P _ {2 3} \\ P _ {3 1} & P _ {3 2} & P _ {3 3} \end{array} \right]
$$

代入式(7)，得

$$
\mathbf {K} = \mathbf {P H} ^ {\mathrm{T}} r ^ {- 1} = \left[ \begin{array}{l l l} P _ {1 1} & P _ {1 2} & P _ {1 3} \\ P _ {2 1} & P _ {2 2} & P _ {2 3} \\ P _ {3 1} & P _ {3 2} & P _ {3 3} \end{array} \right] \left[ \begin{array}{l} 1 \\ 0 \\ 0 \end{array} \right] \frac {1}{r} = \left[ \begin{array}{l} \frac {P _ {1 1}}{r} \\ \frac {P _ {2 1}}{r} \\ \frac {P _ {3 1}}{r} \end{array} \right] \stackrel {\text {def}} {=} \left[ \begin{array}{l} \alpha \\ \beta \\ \gamma \end{array} \right]
$$

$$
\hat {\mathbf {X}} _ {k} = \boldsymbol {\Phi} \hat {\mathbf {X}} _ {k - 1} + \left[ \begin{array}{l} \alpha \\ \beta \\ \gamma \end{array} \right] (Z _ {k} - H \boldsymbol {\Phi} \hat {\mathbf {X}} _ {k - 1})
$$

此即为 $\hat{s}_k, \hat{v}_k$ 和 $\hat{a}_k$ 。

在该题中，被估计信息为 $s_{k}, v_{k}, a_{k}$ ，相应的稳态增益为 $\alpha, \beta, \gamma$ ，习惯上称此种滤波为 $\alpha - \beta - \gamma$ 滤波。当被估计量为 $s_{k}$ 和 $v_{k}$ 时，相应的稳态滤波称为 $\alpha - \beta$ 滤波。

例2-14 在例2-12中， $\Phi = Q = R = P_0 = 1$ ，求 $K_{k}$ 和 $P_{k}$ 。

解 根据卡尔曼滤波基本方程, 得

$$
\begin{array}{r l} & P _ {1 / 0} = P _ {0} + Q = 2 \\ & K _ {1} = P _ {1 / 0} (P _ {1 / 0} + R) ^ {- 1} = \frac {2}{3} = 0. 6 7 \\ & P _ {1} = (1 - K _ {1}) P _ {1 / 0} = P _ {1 / 0} (P _ {1 / 0} + R) ^ {- 1} = K _ {1} \end{array}
$$

并计算得

$$
P _ {2} = K _ {2} = \frac {P _ {1} + 1}{P _ {1} + 2} = \frac {5}{8} = 0. 6 2 5
$$

$$
P _ {3} = K _ {3} = \frac {P _ {2} + 1}{P _ {2} + 2} = \frac {1 3}{2 1} = 0. 6 2
$$

$$
P _ {k} = K _ {k} = \frac {P _ {k - 1} + 1}{P _ {k - 1} + 2}
$$

根据上述计算结果绘出的 $P_{k / k - 1}$ 和 $P_{k}(K_{k})$ 在滤波过程中的变化图形，如图2.2.4所示。从图中

![](images/633df0c106fd3cd44256383ec4cd9b8a317c9fb71c4fc4fb03ee1163327493d5.jpg)  
图2.2.4 不同 $k$ 时刻的 $P_{k}(K_{k})$ 和 $P_{k / k - 1}$

可看出，随着滤波步数 k 的增大， $K_{k}$ 和 $P_{k}$ 逐渐减小，并逐渐趋于稳态值，这意味着滤波刚开始时，对状态的估计主要依赖于量测，量测的修正作用不断地改善状态的估计精度，使 $P_{k}$ 逐渐降低。由于估计精度的不断提高，估计值本身所具有信息的可利用程度也在逐渐提高，滤波过程中的时间更新作用逐渐加强，而量测更新作用则逐渐相对减弱。当两者的作用趋于平衡时，滤波达到稳态，此时 $K_{k}$ 和 $P_{k}$ 达到稳态值， $P_{k / k - 1}$ 和 $P_{k}$ 的差别也逐渐减小并趋于一致。

例2-15 设电离层探测器上装有惯性导航系统，在飞行初始阶段用无线电定位测量的方法来实现飞行中导航参数的校正。为分析方便，仅考虑单轴情况，且认为惯导系统的主要误差源是初始条件（位置、速度和加速度）的误差。略去高阶项，则适用于短时间（几分钟）的惯导系统位置误差方程的解为

$$
\delta P (t) = \delta P (0) + \delta v (0) t + \delta a (0) \frac {t ^ {2}}{2}
$$

式中 $\delta P(0)$ , $\delta v(0)$ 和 $\delta a(0)$ 各表示惯导系统在无线电定位信息引入前的位置误差、速度误差和加速度误差。因此系统状态方程为

$$
\dot {\boldsymbol {X}} (t) = \boldsymbol {F} \boldsymbol {X} (t)
$$

式中

$$
\mathbf {X} (t) = \left[ \begin{array}{l} \delta P (t) \\ \delta v (t) \\ \delta a (t) \end{array} \right], \quad \mathbf {F} = \left[ \begin{array}{l l l} 0 & 1 & 0 \\ 0 & 0 & 1 \\ 0 & 0 & 0 \end{array} \right]
$$

量测量 Z 是惯导系统解算得的位置量 $P_{i}$ 与无线电定位系统指示的位置量 $P_{r}$ 之差，即

$$
Z = P _ {i} - P _ {r} = (P + \delta P) - (P + \delta P _ {r}) = \delta P - \delta P _ {r}
$$

式中 $\delta P_{r}$ 为无线电定位误差。

空中校正分别在 t=0,0.5 和 1 分钟时间点上用量测量 Z 对惯导系统进行三次估计，估计出 $\delta\hat{P},\delta\hat{v}$ 和 $\delta\hat{a}$ ，设计该系统的卡尔曼滤波器。

解 根据题意,可画出系统和量测的方块图,如图 2.2.5 所示。

无线电定位系统的定位噪声是跳变剧烈的无线电噪声,可用白噪声来描述之,所以量测方程为

![](images/3e8ae9b649bed751a160729435e0b45c466b3e0c20659eff927315ecbbeb2532.jpg)  
图2.2.5 系统和量测的方块图

$$
Z _ {k} = [ 1 0 0 ] \mathbf {X} _ {k} + V _ {k} = \mathbf {H X} _ {k} + V _ {k}
$$

式中

$$
E [ V _ {k} V _ {j} ] = R \delta_ {k j}, E [ V _ {k} ] = 0
$$

将系统状态方程离散化,系统转移矩阵为

$$
\boldsymbol {\Phi} (T) = \boldsymbol {I} + \boldsymbol {F} T + \frac {T ^ {2}}{2 !} \boldsymbol {F} ^ {2} + \frac {T ^ {3}}{3 !} \boldsymbol {F} ^ {3} + \dots =
$$

$$
\left[ \begin{array}{l l l} 1 & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{array} \right] + \left[ \begin{array}{l l l} 0 & 1 & 0 \\ 0 & 0 & 1 \\ 0 & 0 & 0 \end{array} \right] T + \left[ \begin{array}{l l l} 0 & 0 & 1 \\ 0 & 0 & 0 \\ 0 & 0 & 0 \end{array} \right] \frac {T ^ {2}}{2} = \left[ \begin{array}{l l l} 1 & T & \frac {T ^ {2}}{2} \\ 0 & 1 & T \\ 0 & 0 & 1 \end{array} \right]
$$

设初始时刻状态的均值和方差阵为

$$
\boldsymbol {m} _ {X _ {0}} = \left[ \begin{array}{l} m _ {P _ {0}} \\ m _ {v _ {0}} \\ m _ {a _ {0}} \end{array} \right], \quad \boldsymbol {C} _ {X _ {0}} = \left[ \begin{array}{c c c} C _ {P _ {0}} & 0 & 0 \\ 0 & C _ {v _ {0}} & 0 \\ 0 & 0 & C _ {a _ {0}} \end{array} \right]
$$

在 $k = 0$ 时刻作第一次定位测量和估计，即利用 $Z_{0}$ 去估计 $\pmb{X}_0$ ，并求出 $\pmb{P}_0$ 。令

$$
\hat {\boldsymbol {X}} _ {0 / - 1} = \boldsymbol {m} _ {X _ {0}}, \quad \boldsymbol {P} _ {0 / - 1} = \boldsymbol {C} _ {X _ {0}}
$$

则

$$
\begin{array}{r l} & {\mathbf {K} _ {0} = \mathbf {P} _ {0 / - 1} \mathbf {H} ^ {\mathrm{T}} (\mathbf {H P} _ {0 / - 1} \mathbf {H} ^ {\mathrm{T}} + R) ^ {- 1} = \left[ \begin{array}{c} C _ {P _ {0}} \\ \hline C _ {P _ {0}} + R \\ 0 \\ 0 \end{array} \right]} \\ & {\hat {\mathbf {X}} _ {0} = \hat {\mathbf {X}} _ {0 / - 1} + \mathbf {K} _ {0} (Z _ {0} - \mathbf {H} \hat {\mathbf {X}} _ {0 / - 1})} \end{array}
$$

下面仅从估计的均方误差来看无线电定位量测对估计的作用。

$$
\mathbf {P} _ {0} = \mathbf {P} _ {0 / - 1} - \mathbf {K} _ {0} \mathbf {H P} _ {0 / - 1} = \left[ \begin{array}{c c c} C _ {P _ {0}} - \frac {C _ {P _ {0}} ^ {2}}{C _ {P _ {0}} + R} & 0 & 0 \\ 0 & C _ {v _ {0}} & 0 \\ 0 & 0 & C _ {a _ {0}} \end{array} \right]
$$

因为惯导系统初始位置误差的方差 $C_{P_0}$ 远比无线电定位误差的方差大，即 $C_{P_0} \gg R$ ，所以

$$
P _ {0} ^ {1 1} = \frac {C _ {P _ {0}} R}{C _ {P _ {0}} + R} \approx R
$$

即第一次定位组合后的 $\delta P$ 估计均方误差接近无线电定位误差 R，相当于惯导的定位精度被提高到接近无线电定位的精度。而 $\delta v$ 和 $\delta a$ 未能估计，因估计的均方误差仍为 $C_{v_{0}}$ 和 $C_{u_{0}}$ 。

在第二次定位组合中

$$
\begin{array}{r l} & {\mathbf {P} _ {1 / 0} = \boldsymbol {\Phi} \mathbf {P} _ {0} \boldsymbol {\Phi} ^ {\mathrm{T}} = \left[ \begin{array}{c c c} R + C _ {v _ {0}} T + C _ {a _ {0}} \frac {T ^ {2}}{2} & C _ {v _ {0}} T + C _ {a _ {0}} \frac {T ^ {3}}{2} & C _ {a _ {0}} \frac {T ^ {2}}{2} \\ C _ {v _ {0}} T + C _ {a _ {0}} \frac {T ^ {3}}{2} & C _ {v _ {0}} + C _ {a _ {0}} T ^ {2} & C _ {a _ {0}} T \\ C _ {a _ {0}} \frac {T ^ {2}}{2} & C _ {a _ {0}} T & C _ {a _ {0}} \end{array} \right] \stackrel {\text {def}} {=}} \\ & {\left[ \begin{array}{c c c} P _ {1 / 0} ^ {1 1} & P _ {1 / 0} ^ {1 2} & P _ {1 / 0} ^ {1 3} \\ P _ {1 / 0} ^ {2 1} & P _ {1 / 0} ^ {2 2} & P _ {1 / 0} ^ {2 3} \\ P _ {1 / 0} ^ {3 1} & P _ {1 / 0} ^ {3 2} & P _ {1 / 0} ^ {3 3} \end{array} \right]} \\ & {\mathbf {P} _ {1} = \mathbf {P} _ {1 / 0} - \mathbf {P} _ {1 / 0} \mathbf {H} ^ {\mathrm{T}} (\mathbf {H P} _ {1 / 0} \mathbf {H} ^ {\mathrm{T}} + R) ^ {- 1} \mathbf {H P} _ {1 / 0} =} \\ & {\mathbf {P} _ {1 / 0} - \frac {1}{P _ {1 / 0} ^ {1 1} + R} \left[ \begin{array}{c c c} (P _ {1 / 0} ^ {1 1}) ^ {2} & P _ {1 / 0} ^ {1 1} P _ {1 / 0} ^ {1 2} & P _ {1 / 0} ^ {1 1} P _ {1 / 0} ^ {1 3} \\ P _ {1 / 0} ^ {1 1} P _ {1 / 0} ^ {1 2} & (P _ {1 / 0} ^ {1 2}) ^ {2} & P _ {1 / 0} ^ {1 2} P _ {1 / 0} ^ {1 3} \\ P _ {1 / 0} ^ {1 1} P _ {1 / 0} ^ {1 3} & P _ {1 / 0} ^ {1 2} P _ {1 / 0} ^ {1 3} & (P _ {1 / 0} ^ {1 3}) ^ {2} \end{array} \right]} \end{array}
$$

即

$$
P _ {1} ^ {1 1} = P _ {1 / 0} ^ {1 1} \left(1 - \frac {P _ {1 / 0} ^ {1 1}}{P _ {1 / 0} ^ {1 1} + R}\right) = P _ {1 / 0} ^ {1 1} - \frac {\left(R + C _ {v _ {0}} T + C _ {a _ {0}} \frac {T ^ {2}}{2}\right) ^ {2}}{2 R + C _ {v _ {0}} T + C _ {a _ {0}} \frac {T ^ {2}}{2}}
$$

$$
P _ {1} ^ {2 2} = P _ {1 / 0} ^ {2 2} - \frac {(P _ {1 / 0} ^ {1 2}) ^ {2}}{P _ {1 / 0} ^ {1 1} + R} = P _ {1 / 0} ^ {2 2} - \frac {\left(C _ {v _ {0}} T + C _ {a _ {0}} \frac {T ^ {3}}{2}\right) ^ {2}}{2 R + C _ {v _ {0}} T + C _ {a _ {0}} \frac {T ^ {2}}{2}}
$$

$$
P _ {1} ^ {3 3} = P _ {1 / 0} ^ {3 3} - \frac {(P _ {1 / 0} ^ {1 3}) ^ {2}}{P _ {1 / 0} ^ {1 1} + R} = P _ {1 / 0} ^ {3 3} \left(1 - \frac {C _ {a _ {0}} \frac {T ^ {4}}{4}}{2 R + C _ {v _ {0}} T + C _ {a _ {0}} \frac {T ^ {2}}{2}}\right)
$$

可以看出,在定位时间间隔 T 较短的情况下,第二次定位组合后的估计,仍是 $\delta P$ 下降得较多, $\delta v$ 下降次之, $\delta a$ 下降最少。如果

$$
\begin{array}{l} C _ {P _ {0}} = (1 \mathrm{nmile}) ^ {2}, C _ {v _ {0}} = (7. 3 \mathrm{nmile/h}) ^ {2} \\ C _ {a _ {0}} = 0, \sqrt {R} = 9. 1 \mathrm{m} \end{array}
$$

则包括高阶项的完整方程计算的结果 $\left(\sqrt{P_{k}^{11}}\right)$ 如图2.2.6所示。可以看出，分析结果与图是一致的，其中 $\sqrt{P_{3/2}^{11}}$ 是第三次定位组合后 $\sqrt{P^{11}}$ 的演变情况。

![](images/8761bf8cb91c8ecd3f40a0d1b9da230f25de8fbafa03e96d96c5fa1dc6313878.jpg)  
图 2.2.6 用无线电定位对惯导系统作导航参数校正

## 2.2.5 白噪声条件下离散型卡尔曼滤波方程的一般形式

设系统方程和量测方程的一般形式为

$$
\boldsymbol {X} _ {k} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {X} _ {k - 1} + \boldsymbol {B} _ {k - 1} \boldsymbol {u} _ {k - 1} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {W} _ {k - 1}\tag{2.2.41}
$$

$$
\mathbf {Z} _ {k} = \mathbf {H} _ {k} \mathbf {X} _ {k} + \mathbf {Y} _ {k} + \mathbf {V} _ {k}\tag{2.2.42}
$$

式中， $\{\pmb{u}_k\}$ 和 $\{\mathbf{Y}_k\}$ 都是已知的确定性输入序列， $u_{k}$ 可理解为控制项， $Y_{k}$ 可理解为量测的系统性

误差。其余项的规定同前，但认为同时刻的 $W_{k}$ 与 $\mathbf{V}_k$ 是相关的，即

$$
E \left[ \boldsymbol {W} _ {k} \boldsymbol {V} _ {j} ^ {\mathrm{T}} \right] = \boldsymbol {S} _ {k} \delta_ {k j}
$$

## 1. 滤波一般方程

将式 $(2.2.41)$ 改写为

$$
\boldsymbol {X} _ {k} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {X} _ {k - 1} + \boldsymbol {B} _ {k - 1} \boldsymbol {u} _ {k - 1} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {W} _ {k - 1} + \boldsymbol {J} _ {k - 1} (\boldsymbol {Z} _ {k - 1} - \boldsymbol {H} _ {k - 1} \boldsymbol {X} _ {k - 1} - \boldsymbol {Y} _ {k - 1} - \boldsymbol {V} _ {k - 1})
$$

式中 $J_{k-1}$ 为待定的 $n \times m$ 矩阵。令

$$
\boldsymbol {\Phi} _ {k, k - 1} ^ {*} = \boldsymbol {\Phi} _ {k, k - 1} - \boldsymbol {J} _ {k - 1} \boldsymbol {H} _ {k - 1}\tag{2.2.43}
$$

$$
\boldsymbol {W} _ {k - 1} ^ {*} = \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {W} _ {k - 1} - \boldsymbol {J} _ {k - 1} \boldsymbol {V} _ {k - 1}\tag{2.2.44}
$$

则

$$
\boldsymbol {X} _ {k} = \boldsymbol {\Phi} _ {k, k - 1} ^ {*} \boldsymbol {X} _ {k - 1} + \boldsymbol {B} _ {k - 1} \boldsymbol {u} _ {k - 1} + \boldsymbol {J} _ {k - 1} (\boldsymbol {Z} _ {k - 1} - \boldsymbol {Y} _ {k - 1}) + \boldsymbol {W} _ {k - 1} ^ {*}\tag{2.2.45}
$$

显然式(2.2.45)是式(2.2.41)的恒等变形，其中 $B_{k-1}u_{k-1}+J_{k-1}(Z_{k-1}-Y_{k-1})$ 可看做式(2.2.45)所示系统的控制项。 $\{W_{k}^{*}\}$ 和 $\{V_{k}\}$ 之间的协方差函数阵为

$$
E \left[ \boldsymbol {W} _ {k} ^ {*} \boldsymbol {V} _ {j} ^ {\mathrm{T}} \right] = E \left[ \left(\boldsymbol {\Gamma} _ {k} \boldsymbol {W} _ {k} - \boldsymbol {J} _ {k} \boldsymbol {V} _ {k}\right) \boldsymbol {V} _ {j} ^ {\mathrm{T}} \right] = \left(\boldsymbol {\Gamma} _ {k} \boldsymbol {S} _ {k} - \boldsymbol {J} _ {k} \boldsymbol {R} _ {k}\right) \delta_ {k j}
$$

选择

$$
\boldsymbol {J} _ {k} = \boldsymbol {\Gamma} _ {k} \boldsymbol {S} _ {k} \boldsymbol {R} _ {k} ^ {- 1}\tag{2.2.46}
$$

则

$$
E [ \boldsymbol {W} _ {k} ^ {*} \boldsymbol {V} _ {j} ^ {\mathrm{T}} ] = \mathbf {0}
$$

这说明若按式(2.2.46)选择 $J_{k}$ ，则式(2.2.45)所示系统的系统噪声 $W_{k}^{*}$ 与量测噪声 $V_{k}$ 不相关，这就和2.2.4节中4.的情况完全一样，所以一步预测方程为

$$
\begin{array}{r l} \hat {\mathbf {X}} _ {k / k - 1} & = \boldsymbol {\Phi} _ {k, k - 1} ^ {*} \hat {\mathbf {X}} _ {k - 1} + \mathbf {B} _ {k - 1} \mathbf {u} _ {k - 1} + \mathbf {J} _ {k - 1} (\mathbf {Z} _ {k - 1} - \mathbf {Y} _ {k - 1}) = \\ & \quad \boldsymbol {\Phi} _ {k, k - 1} \hat {\mathbf {X}} _ {k - 1} + \mathbf {B} _ {k - 1} \mathbf {u} _ {k - 1} + \mathbf {J} _ {k - 1} (\mathbf {Z} _ {k - 1} - \mathbf {Y} _ {k - 1} - \mathbf {H} _ {k - 1} \hat {\mathbf {X}} _ {k - 1}) \end{array}\tag{2.2.47}
$$

从上式可看出， $J_{k-1}$ 相当于状态一步预测的增益阵。

$$
\hat {\mathbf {X}} _ {k} = \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {K} _ {k} (\mathbf {Z} _ {k} - \mathbf {Y} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1})\tag{2.2.48}
$$

$$
\boldsymbol {K} _ {k} = \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}) ^ {- 1}\tag{2.2.49}
$$

下面推导 $P_{k/k-1}$ 和 $P_{k}$ 。

式 $(2.2.45)$ 减去式 $(2.2.47)$ ，得

$$
\widetilde {X} _ {k / k - 1} = \Phi_ {k, k - 1} ^ {*} \widetilde {X} _ {k - 1} + W _ {k - 1} ^ {*}
$$

所以

$$
\begin{array}{r l} \boldsymbol {P} _ {k / k - 1} & = E [ \widetilde {\boldsymbol {X}} _ {k / k - 1} \widetilde {\boldsymbol {X}} _ {k / k - 1} ^ {\mathrm{T}} ] = \boldsymbol {\Phi} _ {k, k - 1} ^ {*} \boldsymbol {P} _ {k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {* \mathrm{T}} + \\ & \quad E [ (\boldsymbol {\Gamma} _ {k - 1} \boldsymbol {W} _ {k - 1} - \boldsymbol {J} _ {k - 1} \boldsymbol {V} _ {k - 1}) (\boldsymbol {\Gamma} _ {k - 1} \boldsymbol {W} _ {k - 1} - \boldsymbol {J} _ {k - 1} \boldsymbol {V} _ {k - 1}) ^ {\mathrm{T}} ] = \\ & \quad \boldsymbol {\Phi} _ {k, k - 1} ^ {*} \boldsymbol {P} _ {k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {* \mathrm{T}} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {Q} _ {k - 1} \boldsymbol {\Gamma} _ {k - 1} ^ {\mathrm{T}} + \boldsymbol {J} _ {k - 1} \boldsymbol {R} _ {k - 1} \boldsymbol {J} _ {k - 1} ^ {\mathrm{T}} - \\ & \quad \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {S} _ {k - 1} \boldsymbol {J} _ {k - 1} ^ {\mathrm{T}} - \boldsymbol {J} _ {k - 1} \boldsymbol {S} _ {k - 1} ^ {\mathrm{T}} \boldsymbol {\Gamma} _ {k - 1} ^ {\mathrm{T}} = \\ & \quad (\boldsymbol {\Phi} _ {k, k - 1} - \boldsymbol {J} _ {k - 1} \boldsymbol {H} _ {k - 1}) \boldsymbol {P} _ {k - 1} (\boldsymbol {\Phi} _ {k, k - 1} - \boldsymbol {J} _ {k - 1} \boldsymbol {H} _ {k - 1}) ^ {\mathrm{T}} + \\ & \quad \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {Q} _ {k - 1} \boldsymbol {\Gamma} _ {k - 1} ^ {\mathrm{T}} - \boldsymbol {J} _ {k - 1} \boldsymbol {S} _ {k - 1} ^ {\mathrm{T}} \boldsymbol {\Gamma} _ {k - 1} ^ {\mathrm{T}} \\ & \quad \boldsymbol {P} _ {k} = (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) \boldsymbol {P} _ {k / k - 1} \end{array}\tag{2.2.50}
$$

(2.2.51)

式(2.2.46)～式(2.2.51)就是系统噪声与量测噪声相关条件下离散卡尔曼滤波的一般方程。同样，当初始条件 $\hat{X}_{0}=m_{X0}(P_{0}=C_{X0})$ 时，估计也是无偏的。

2. 一步预测一般方程

将式 $(2.2.47)$ 改写成

$$
\begin{array}{r l} \hat {\mathbf {X}} _ {k + 1 / k} & = \boldsymbol {\Phi} _ {k + 1, k} \hat {\mathbf {X}} _ {k} + \boldsymbol {B} _ {k} \boldsymbol {u} _ {k} + \boldsymbol {J} _ {k} (\mathbf {Z} _ {k} - \mathbf {Y} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k}) = \\ & (\boldsymbol {\Phi} _ {k + 1, k} - \boldsymbol {J} _ {k} \mathbf {H} _ {k}) \hat {\mathbf {X}} _ {k} + \boldsymbol {B} _ {k} \boldsymbol {u} _ {k} + \boldsymbol {J} _ {k} (\mathbf {Z} _ {k} - \mathbf {Y} _ {k}) \end{array}
$$

式 $(2.2.48)$ 代入上式，得

$$
\begin{array}{r l} \hat {\mathbf {X}} _ {k + 1 / k} & = (\boldsymbol {\Phi} _ {k + 1, k} - \boldsymbol {J} _ {k} \boldsymbol {H} _ {k}) [ \hat {\mathbf {X}} _ {k / k - 1} + \boldsymbol {K} _ {k} (\mathbf {Z} _ {k} - \mathbf {Y} _ {k} - \boldsymbol {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1}) ] + \boldsymbol {B} _ {k} \boldsymbol {u} _ {k} + \boldsymbol {J} _ {k} (\mathbf {Z} _ {k} - \mathbf {Y} _ {k}) = \\ & \quad \boldsymbol {\Phi} _ {k + 1, k} \hat {\mathbf {X}} _ {k / k - 1} + \boldsymbol {B} _ {k} \boldsymbol {u} _ {k} + (\boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {K} _ {k} + \boldsymbol {J} _ {k} - \boldsymbol {J} _ {k} \boldsymbol {H} _ {k} \boldsymbol {K} _ {k}) (\mathbf {Z} _ {k} - \mathbf {Y} _ {k} - \boldsymbol {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1}) \end{array}
$$

令

$$
\Phi_ {k + 1, k} \mathbf {K} _ {k} + \mathbf {J} _ {k} - \mathbf {J} _ {k} \mathbf {H} _ {k} \mathbf {K} _ {k} \stackrel {\mathrm{det}} {=} \overline {{\mathbf {K}}} _ {k}\tag{2.2.52}
$$

则

$$
\hat {\mathbf {X}} _ {k + 1 / k} = \boldsymbol {\Phi} _ {k + 1, k} \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {B} _ {k} \mathbf {u} _ {k} + \bar {\mathbf {K}} _ {k} (\mathbf {Z} _ {k} - \mathbf {Y} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1})\tag{2.2.53}
$$

根据式(2.2.46)， $\bar{K}_{k}$ 可写成

$$
\overline {{{\boldsymbol {K}}}} _ {k} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {K} _ {k} + \boldsymbol {\Gamma} _ {k} \boldsymbol {S} _ {k} \boldsymbol {R} _ {k} ^ {- 1} (\boldsymbol {I} - \boldsymbol {H} _ {k} \boldsymbol {K} _ {k})
$$

又因

$$
\begin{array}{r l} \mathbf {I} - \mathbf {H} _ {k} \mathbf {K} _ {k} & = \mathbf {I} - \mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} (\mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} + \mathbf {R} _ {k}) ^ {- 1} = \\ & (\mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} + \mathbf {R} _ {k}) (\mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} + \mathbf {R} _ {k}) ^ {- 1} - \\ & \mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} (\mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} + \mathbf {R} _ {k}) ^ {- 1} = \\ & \mathbf {R} _ {k} (\mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} + \mathbf {R} _ {k}) ^ {- 1} \end{array}
$$

所以

$$
\bar {\boldsymbol {K}} = \left(\boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {\Gamma} _ {k} \boldsymbol {S} _ {k}\right) \left(\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}\right) ^ {- 1}\tag{2.2.54}
$$

下面求预测均方误差阵。将式(2.2.42)代入式(2.2.53)，得

$$
\hat {\mathbf {X}} _ {k + 1 / k} = \boldsymbol {\Phi} _ {k + 1, k} \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {B} _ {k} \mathbf {u} _ {k} + \overline {{\mathbf {K}}} _ {k} \left[ \mathbf {H} _ {k} \left(\mathbf {X} _ {k} - \hat {\mathbf {X}} _ {k / k - 1}\right) + \mathbf {V} _ {k} \right]
$$

所以

$$
\begin{array}{r l} & {\widetilde {X} _ {k + 1 / k} = (\Phi_ {k + 1, k} - \overline {{{K}}} _ {k} H _ {k}) \widetilde {X} _ {k / k - 1} + \Gamma_ {k} W _ {k} - \overline {{{K}}} _ {k} V _ {k}} \\ & {P _ {k + 1 / k} = E [ \widetilde {X} _ {k + 1 / k} \widetilde {X} _ {k + 1 / k} ^ {\mathrm{T}} ] = (\Phi_ {k + 1, k} - \overline {{{K}}} _ {k} H _ {k}) P _ {k / k - 1} (\Phi_ {k + 1, k} - \overline {{{K}}} _ {k} H _ {k}) ^ {\mathrm{T}} +} \\ & {\quad \Gamma_ {k} Q _ {k} \Gamma_ {k} ^ {\mathrm{T}} + \overline {{{K}}} _ {k} R _ {k} \overline {{{K}}} _ {k} ^ {\mathrm{T}} - \Gamma_ {k} S _ {k} \overline {{{K}}} _ {k} ^ {\mathrm{T}} - \overline {{{K}}} _ {k} S _ {k} ^ {\mathrm{T}} \Gamma_ {k} ^ {\mathrm{T}} =} \\ & {\quad \Phi_ {k + 1, k} P _ {k / k - 1} \Phi_ {k + 1, k} ^ {\mathrm{T}} + \Gamma_ {k} Q _ {k} \Gamma_ {k} ^ {\mathrm{T}} - \overline {{{K}}} _ {k} (H _ {k} P _ {k / k - 1} \Phi_ {k + 1, k} ^ {\mathrm{T}} + S _ {k} ^ {\mathrm{T}} \Gamma_ {k} ^ {\mathrm{T}}) -} \\ & {\quad (\Phi_ {k + 1, k} P _ {k / k - 1} H _ {k} ^ {\mathrm{T}} + \Gamma_ {k} S _ {k}) \overline {{{K}}} _ {k} ^ {\mathrm{T}} + \overline {{{K}}} _ {k} (H _ {k} P _ {k / k - 1} H _ {k} ^ {\mathrm{T}} + R _ {k}) \overline {{{K}}} _ {k} ^ {\mathrm{T}}} \end{array}
$$

将式 $(2.2.54)$ 代入上式,经整理,得

$$
\boldsymbol {P} _ {k + 1 / k} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + \boldsymbol {\Gamma} _ {k} \boldsymbol {Q} _ {k} \boldsymbol {\Gamma} _ {k} ^ {\mathrm{T}} - \bar {\boldsymbol {K}} _ {k} (\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + \boldsymbol {S} _ {k} ^ {\mathrm{T}} \boldsymbol {\Gamma} _ {k} ^ {\mathrm{T}})\tag{2.2.55}
$$

式(2.2.53)、式(2.2.54)和式(2.2.55)即为一步预测一般方程。如果 $S_{k}=0$ ，则式(2.2.55)和式(2.2.40)完全一样。

一般方程除了适用于系统和量测有确定性的输入项以及噪声之间相关等情况外，在处理量测中的有色噪声时也要用到，这将在第三章中讨论。

## 2.3 连续型卡尔曼滤波

采用递推算法是离散型卡尔曼滤波的最大优点,算法可由计算机执行,不必存储时间过程中的大量量测数据,因此离散型卡尔曼滤波在工程上得到了广泛的应用。虽然很多物理系统是连续系统,但只要离散化,就能使用离散型卡尔曼滤波方程。

连续型卡尔曼滤波则根据连续时间过程中的量测值,采用求解矩阵微分方程的方法估计系统状态变量的时间连续值,因此算法失去了递推性。连续型卡尔曼滤波是最优估计理论中的一部分,讨论该算法是为了使读者加深对卡尔曼滤波理论的理解。

## 2.3.1 连续型卡尔曼滤波方程的推导

连续型卡尔曼滤波方程可在离散型卡尔曼滤波基本方程的基础上推导出来。基本思路是:将连续系统离散化,应用离散型卡尔曼滤波基本方程和导数概念推导出连续型滤波方程。为叙述方便,先介绍一些基本关系。

1. 连续型白噪声过程和等效白噪声序列之间的关系

设连续系统的系统方程和量测方程分别为

$$
\left. \begin{array}{l} \dot {\boldsymbol {X}} (t) = \boldsymbol {F} (t) \boldsymbol {X} (t) + \boldsymbol {G} (t) \boldsymbol {w} (t) \\ \boldsymbol {Z} (t) = \boldsymbol {H} (t) \boldsymbol {X} (t) + \boldsymbol {v} (t) \end{array} \right\}\tag{2.3.1}
$$

式中

$$
\left. \begin{array}{l l} E [ w (t) ] = 0, & E [ w (t) w ^ {\mathrm{T}} (\tau) ] = q (t) \delta (t - \tau) \\ E [ v (t) ] = 0, & E [ v (t) v ^ {\mathrm{T}} (\tau) ] = r (t) \delta (t - \tau) \end{array} \right\}\tag{2.3.2}
$$

$w(t)$ 和 $v(t)$ 不相关, $q(t)$ 为非负定阵, $r(t)$ 为正定阵。

对式 $(2.3.1)$ 和式 $(2.3.2)$ 作离散化处理,得

$$
\left. \begin{array}{l} \mathbf {X} (t _ {k} + \Delta t) = \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) \mathbf {X} (t _ {k}) + \int_ {t _ {k}} ^ {t _ {k} + \Delta t} \boldsymbol {\Phi} (t _ {k} + \Delta t, \tau) \mathbf {G} (\tau) w (\tau) d \tau \\ \mathbf {Z} (t _ {k} + \Delta t) = \mathbf {H} (t _ {k} + \Delta t) \mathbf {X} (t _ {k} + \Delta t) + \mathbf {V} _ {k} \end{array} \right\}\tag{2.3.3}
$$

式中 $\Phi(t_k + \Delta t, t_k)$ 满足方程

$$
\left. \begin{array}{l} \dot {\boldsymbol {\Phi}} (t, t _ {k}) = \boldsymbol {F} (t) \boldsymbol {\Phi} (t, t _ {k}) \\ \boldsymbol {\Phi} (t _ {k}, t _ {k}) = \boldsymbol {I} \end{array} \right\}\tag{2.3.4}
$$

作如下等效处理

$$
\boldsymbol {W} _ {k} = \frac {1}{\Delta t} \int_ {t _ {k}} ^ {t _ {k} + \Delta t} \boldsymbol {w} (t) \mathrm{d} t\tag{2.3.5}
$$

$$
\mathbf {V} _ {k} = \frac {1}{\Delta t} \int_ {t _ {k}} ^ {t _ {k} + \Delta t} \mathbf {v} (t) \mathrm{d} t\tag{2.3.6}
$$

根据式(2.3.5)对 $W_{k}$ 的定义,有

$$
\begin{array}{r l} E [ \pmb {W} _ {k} \pmb {W} _ {j} ^ {\mathrm{T}} ] = & E \left[ \frac {1}{\Delta t} \int_ {t _ {k}} ^ {t _ {k} + \Delta t} \pmb {w} (t) \mathrm{d} t \cdot \frac {1}{\Delta t} \int_ {t _ {j}} ^ {t _ {j} + \Delta t} \pmb {w} ^ {\mathrm{T}} (\tau) \mathrm{d} \tau \right] = \\ & \frac {1}{\Delta t ^ {2}} \int_ {t _ {k}} ^ {t _ {k} + \Delta t} \int_ {t _ {j}} ^ {t _ {j} + \Delta t} E [ \pmb {w} (t) \pmb {w} ^ {\mathrm{T}} (\tau) ] \mathrm{d} \tau \mathrm{d} t = \\ & \frac {1}{\Delta t ^ {2}} \int_ {t _ {k}} ^ {t _ {k} + \Delta t} \pmb {q} (t) \int_ {t _ {j}} ^ {t _ {j} + \Delta t} \delta (t - \tau) \mathrm{d} \tau \mathrm{d} t \end{array}
$$

由于积分变量 $t \in [t_k, t_k + \Delta t), \tau \in [t_j, t_j + \Delta t)$ ，而当 $t \neq \tau$ 时， $\delta(t - \tau) = 0$ ，所以只有当上述两积分区间重合时，即 $j = k$ 时，上述积分才不为零，所以

$$
E \left[ \boldsymbol {W} _ {k} \boldsymbol {W} _ {j} ^ {\mathrm{T}} \right] = \frac {1}{\Delta t ^ {2}} \int_ {t _ {k}} ^ {t _ {k} + \Delta t} \boldsymbol {q} (t) \int_ {t _ {j}} ^ {t _ {j} + \Delta t} \delta (t - \tau) \mathrm{d} \tau \mathrm{d} t \delta_ {k j} = \frac {1}{\Delta t ^ {2}} \int_ {t _ {k}} ^ {t _ {k} + \Delta t} \boldsymbol {q} (t) \mathrm{d} t \delta_ {k j}
$$

$q(t)$ 是白噪声过程 $w(t)$ 的协方差强度阵, 虽然 $w(t)$ 变化剧烈, 但 $q(t)$ 变化比较缓慢, 对平稳过

程， $q(t)$ 是常阵，根本不随时间而变，所以当 $\Delta t$ 不大时， $q(t)$ 在该区间内可近似看做常阵

$$
E [ \pmb {W} _ {k} \pmb {W} _ {j} ^ {\mathrm{T}} ] \approx \frac {1}{\Delta t ^ {2}} \pmb {q} (t _ {k}) \int_ {t _ {k}} ^ {t _ {k} + \Delta t} 1 \mathrm{d} t \delta_ {k j} = \frac {\pmb {q} (t _ {k})}{\Delta t} \delta_ {k j}\tag{2.3.7}
$$

从式(2.3.7)可看出,白噪声序列 $W_{k}$ 的协方差阵为

$$
Q _ {k} = \frac {\pmb {q} (t _ {k})}{\Delta t}\tag{2.3.8}
$$

同理可得

$$
E [ \mathbf {V} _ {k} \mathbf {V} _ {j} ^ {\mathrm{T}} ] = \mathbf {R} _ {k} \delta_ {k j}
$$

式中

$$
\mathbf {R} _ {k} = \frac {\mathbf {r} (t _ {k})}{\Delta t}\tag{2.3.9}
$$

根据式 $(2.3.5)$ ，式 $(2.3.3)$ 可近似为

$$
\boldsymbol {X} (t _ {k} + \Delta t) \approx \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) \boldsymbol {X} (t _ {k}) + \boldsymbol {\Gamma} (t _ {k} + \Delta t, t _ {k}) \boldsymbol {W} _ {k}\tag{2.3.10}
$$

式中

$$
\Gamma (t _ {k} + \Delta t, t _ {k}) = \int_ {t _ {k}} ^ {t _ {k} + \Delta t} \Phi (t _ {k} + \Delta t, \tau) G (\tau) \mathrm{d} \tau\tag{2.3.11}
$$

$\Delta t$ 越小, 式(2.3.10)的近似度就越好。

2. 连续型卡尔曼滤波基本方程的推导

根据滤波基本方程式(2.2.4)，并考虑式(2.3.8)和式(2.3.9)，得等效离散系统的卡尔曼滤波基本方程

$$
\hat {\mathbf {X}} (t _ {k} + \Delta t / t _ {k}) = \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) \hat {\mathbf {X}} (t _ {k} / t _ {k})\tag{2.3.12}
$$

$$
\hat {\mathbf {X}} (t _ {k} + \Delta t / t _ {k} + \Delta t) = \hat {\mathbf {X}} (t _ {k} + \Delta t / t _ {k}) + \mathbf {K} (t _ {k} + \Delta t) [ \mathbf {Z} (t _ {k} + \Delta t) -
$$

$$
\mathbf {H} (t _ {k} + \Delta t) \hat {\mathbf {X}} (t _ {k} + \Delta t / t _ {k}) ]\tag{2.3.13}
$$

$$
\mathbf {K} (t _ {k} + \Delta t) = \mathbf {P} (t _ {k} + \Delta t / t _ {k} + \Delta t) \mathbf {H} ^ {\mathrm{T}} (t _ {k} + \Delta t) \left[ \frac {\mathbf {r} (t _ {k} + \Delta t)}{\Delta t} \right] ^ {- 1}\tag{2.3.14}
$$

$$
\mathbf {P} (t _ {k} + \Delta t / t _ {k} + \Delta t) = [ \mathbf {I} - \mathbf {K} (t _ {k} + \Delta t) \mathbf {H} (t _ {k} + \Delta t) ] \mathbf {P} (t _ {k} + \Delta t / t _ {k})\tag{2.3.15}
$$

$$
\boldsymbol {P} (t _ {k} + \Delta t / t _ {k}) = \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) \boldsymbol {P} (t _ {k} / t _ {k}) \boldsymbol {\Phi} ^ {\mathrm{T}} (t _ {k} + \Delta t, t _ {k}) +
$$

$$
\boldsymbol {\Gamma} (t _ {k} + \Delta t, t _ {k}) \frac {\boldsymbol {q} (t _ {k})}{\Delta t} \boldsymbol {\Gamma} ^ {\mathrm{T}} (t _ {k} + \Delta t, t _ {k})\tag{2.3.16}
$$

将 $\boldsymbol{\Gamma}(t_{k}+\Delta t,t_{k})$ 和 $\boldsymbol{\Phi}(t_{k}+\Delta t,t_{k})$ 在 $t_{k}$ 处作泰勒级数展开。应用式(2.3.4)所列关系，得

$$
\begin{array}{r l} \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) & = \dot {\boldsymbol {\Phi}} (t _ {k}, t _ {k}) + \dot {\boldsymbol {\Phi}} (t _ {k} + \Delta t, t _ {k}) | _ {\Delta t = 0} \cdot \Delta t + \varepsilon_ {1} (\Delta t) = \\ & \quad I + F (t _ {k}) \Delta t + \varepsilon_ {1} (\Delta t) \end{array}\tag{2.3.17}
$$

式中 $\varepsilon_{1}(\Delta t)$ 为 $\Delta t$ 的高阶无穷小量。

$$
\Gamma (t _ {k} + \Delta t, t _ {k}) = \Gamma (t _ {k}, t _ {k}) + \dot {\Gamma} (t _ {k} + \Delta t, t _ {k}) | _ {\Delta t = 0} \cdot \Delta t + \varepsilon_ {2} (\Delta t)
$$

由式(2.3.11)知

$$
\Gamma (t _ {k}, t _ {k}) = 0
$$

$$
\dot {\Gamma} (t _ {k} + \Delta t, t _ {k}) \mid_ {\Delta t = 0} = \Phi (t _ {k} + \Delta t, t _ {k} + \Delta t) G (t _ {k} + \Delta t) \mid_ {\Delta t = 0} = G (t _ {k})
$$

所以

$$
\boldsymbol {\Gamma} (t _ {k} + \Delta t, t _ {k}) = \mathbf {G} (t _ {k}) \Delta t + \varepsilon_ {2} (\Delta t)\tag{2.3.18}
$$

式中 $\varepsilon_{2}(\Delta t)$ 为 $\Delta t$ 的高阶无穷小。

根据导数定义，并注意到 $\Delta t \rightarrow 0$ 时， $t_{k} = k\Delta t$ 成为连续时间变量，且记为 t，有

$$
\begin{array}{r l} \dot {\hat {X}} (t / t) & = \lim _ {\Delta t \to 0} \frac {\hat {X} (t _ {k} + \Delta t / t _ {k} + \Delta t) - \hat {X} (t _ {k} / t _ {k})}{\Delta t} = \\ & \quad \lim _ {\Delta t \to 0} \frac {1}{\Delta t} [ \Phi (t _ {k} + \Delta t, t _ {k}) - I ] \hat {X} (t _ {k} / t _ {k}) + \\ & \quad \lim _ {\Delta t \to 0} \frac {1}{\Delta t} K (t _ {k} + \Delta t) [ Z (t _ {k} + \Delta t) - H (t _ {k} + \Delta t) \hat {X} (t _ {k} + \Delta t / t _ {k}) ] = \\ & \quad \lim _ {\Delta t \to 0} \frac {1}{\Delta t} [ F (t _ {k}) \Delta t + \varepsilon_ {1} (\Delta t) ] \hat {X} (t _ {k} / t _ {k}) + \\ & \quad \lim _ {\Delta t \to 0} \frac {1}{\Delta t} P (t _ {k} + \Delta t / t _ {k} + \Delta t) H ^ {\mathrm{T}} (t _ {k} + \Delta t) r ^ {- 1} (t _ {k} + \Delta t) \Delta t \cdot \\ & \quad [ Z (t _ {k} + \Delta t) - H (t _ {k} + \Delta t) \hat {X} (t _ {k} + \Delta t / t _ {k}) ] = \\ & \quad F (t) \hat {X} (t / t) + P (t / t) H ^ {\mathrm{T}} (t) r ^ {- 1} (t) [ Z (t) - H (t) \hat {X} (t / t) ] \end{array}
$$

记

$$
\boldsymbol {K} (t) = \boldsymbol {P} (t / t) \boldsymbol {H} ^ {\mathrm{T}} (t) \boldsymbol {r} ^ {- 1} (t)
$$

则

$$
\dot {\hat {\mathbf {X}}} (t / t) = \mathbf {F} (t) \hat {\mathbf {X}} (t / t) + \mathbf {K} (t) [ \mathbf {Z} (t) - \mathbf {H} (t) \hat {\mathbf {X}} (t / t) ]
$$

将式 $(2.3.16)$ 代入式 $(2.3.15)$ ，得

$$
\begin{array}{r l} & {\pmb {P} (t _ {k} + \Delta t / t _ {k} + \Delta t) = [ \pmb {I} - \pmb {K} (t _ {k} + \Delta t) \pmb {H} (t _ {k} + \Delta t) ] [ \pmb {\Phi} (t _ {k} + \Delta t, t _ {k}) \bullet} \\ & {\qquad \pmb {P} (t _ {k} / t _ {k}) \pmb {\Phi} ^ {\mathrm{T}} (t _ {k} + \Delta t, t _ {k}) + \pmb {\Gamma} (t _ {k} + \Delta t, t _ {k}) \frac {\pmb {q} (t _ {k})}{\Delta t} \bullet \pmb {\Gamma} ^ {\mathrm{T}} (t _ {k} + \Delta t, t _ {k}) ] =} \\ & {\qquad [ \pmb {I} - \pmb {P} (t _ {k} + \Delta t / t _ {k} + \Delta t) \pmb {H} ^ {\mathrm{T}} (t _ {k} + \Delta t) \pmb {r} ^ {- 1} (t _ {k} + \Delta t) \bullet} \\ & {\qquad \Delta t \pmb {H} (t _ {k} + \Delta t) ] \{\left[ \pmb {I} + \pmb {F} (t _ {k}) \Delta t + \pmb {\varepsilon} _ {1} (\Delta t) \right] \bullet} \\ & {\qquad \pmb {P} (t _ {k} / t _ {k}) [ \pmb {I} + \pmb {F} (t _ {k}) \Delta t + \pmb {\varepsilon} _ {1} (\Delta t) ] ^ {\mathrm{T}} +} \\ & {\qquad [ \pmb {G} (t _ {k}) \Delta t + \pmb {\varepsilon} _ {2} (\Delta t) ] \frac {\pmb {q} (t _ {k})}{\Delta t} [ \pmb {G} (t _ {k}) \Delta t + \pmb {\varepsilon} _ {2} (\Delta t) ] ^ {\mathrm{T}} \} =} \\ & {\qquad \pmb {P} (t _ {k} / t _ {k}) + \pmb {P} (t _ {k} / t _ {k}) \pmb {F} ^ {\mathrm{T}} (t _ {k}) \Delta t + \pmb {F} (t _ {k}) \pmb {P} (t _ {k} / t _ {k}) \Delta t -} \\ & {\qquad \pmb {P} (t _ {k} + \Delta t / t _ {k} + \Delta t) \pmb {H} ^ {\mathrm{T}} (t _ {k} + \Delta t) \pmb {r} ^ {- 1} (t _ {k} + \Delta t) \Delta t \bullet} \\ & {\qquad \pmb {H} (t _ {k} + \Delta t) \pmb {P} (t _ {k} / t _ {k}) + \pmb {G} (t _ {k}) \pmb {q} (t _ {k}) \pmb {G} ^ {\mathrm{T}} (t _ {k}) \Delta t + \pmb {E} (\Delta t)} \end{array}
$$

式中 $E(\Delta t)$ 是 $\Delta t$ 的高阶小量。所以

$$
\begin{array}{r l} \dot {\mathbf {P}} (t / t) & = \lim _ {\Delta t \to 0} \frac {\mathbf {P} (t _ {k} + \Delta t / t _ {k} + \Delta t) - \mathbf {P} (t _ {k} / t _ {k})}{\Delta t} = \\ & \quad \lim _ {\Delta t \to 0} [ \mathbf {P} (t _ {k} / t _ {k}) \mathbf {F} ^ {\mathrm{T}} (t _ {k}) + \mathbf {F} (t _ {k}) \mathbf {P} (t _ {k} / t _ {k}) - \mathbf {P} (t _ {k} + \Delta t / t _ {k} + \Delta t) \cdot \\ & \quad \mathbf {H} ^ {\mathrm{T}} (t _ {k} + \Delta t) \mathbf {r} ^ {- 1} (t _ {k} + \Delta t) \mathbf {H} (t _ {k} + \Delta t) \mathbf {P} (t _ {k} / t _ {k}) + \\ & \quad \mathbf {G} (t _ {k}) \mathbf {q} (t _ {k}) \mathbf {G} ^ {\mathrm{T}} (t _ {k}) + \frac {\mathbf {E} (\Delta t)}{\Delta t} ] \end{array}
$$

注意到 $\frac{E(\Delta t)}{\Delta t}$ 至少是关于 $\Delta t$ 的一阶多项式，当 $\Delta t \to 0$ 时 $t_k$ 成为连续变量，所以

$$
\begin{array}{r l} \dot {\mathbf {P}} (t / t) & = \mathbf {P} (t / t) \mathbf {F} ^ {\mathrm{T}} (t) + \mathbf {F} (t) \mathbf {P} (t / t) - \mathbf {P} (t / t) \mathbf {H} ^ {\mathrm{T}} (t) \mathbf {r} ^ {- 1} (t) \cdot \\ & \quad \mathbf {H} (t) \mathbf {P} (t / t) + \mathbf {G} (t) \mathbf {q} (t) \mathbf {G} ^ {\mathrm{T}} (t) \end{array}
$$

记 $\hat{\boldsymbol{X}}(t)=\hat{\boldsymbol{X}}(t/t),\boldsymbol{P}(t)=\boldsymbol{P}(t/t)$ ，则上述所推方程可写成

1. 2. 3. 4. 5. 6. 7. 8. 9. 10. 11. 12. 13. 14. 15. 16. 17. 18. 19. 20. 21. 22. 23. 24. 25. 26. 27. 28. 29. 30. 31. 32. 33. 34. 35. 36. 37. 38. 39. 40. 41. 42. 43. 44. 45. 46. 47. 48. 49. 50. 51. 52. 53. 54. 55. 56. 57. 58. 59. 60. 61. 62. 63. 64. 65. 66. 67. 68. 69. 70. 71. 72. 73. 74. 75. 76. 77. 78. 79. 80. 81. 82. 83. 84. 85. 86. 87. 88. 89. 90. 91. 92. 93. 94. 95. 96. 97. 98. 99.

$$
\dot {\hat {\mathbf {X}}} (t) = \mathbf {F} (t) \hat {\mathbf {X}} (t) + \mathbf {K} (t) [ \mathbf {Z} (t) - \mathbf {H} (t) \hat {\mathbf {X}} (t) ]\tag{2.3.19}
$$

$$
\boldsymbol {K} (t) = \boldsymbol {P} (t) \boldsymbol {H} ^ {\mathrm{T}} (t) \boldsymbol {r} ^ {- 1} (t)\tag{2.3.20}
$$

$$
\dot {\boldsymbol {P}} (t) = \boldsymbol {P} (t) \boldsymbol {F} ^ {\mathrm{T}} (t) + \boldsymbol {F} (t) \boldsymbol {P} (t) - \boldsymbol {P} (t) \boldsymbol {H} ^ {\mathrm{T}} (t) \boldsymbol {r} ^ {- 1} (t) \boldsymbol {H} (t) \boldsymbol {P} (t) + \boldsymbol {G} (t) \boldsymbol {q} (t) \boldsymbol {G} ^ {\mathrm{T}} (t)\tag{2.3.21}
$$

式(2.3.19)、式(2.3.20)、式(2.3.21)即为连续型卡尔曼滤波基本方程,这是一组矩阵微分方程组,其中式(2.3.21)是关于 $P(t)$ 的非线性矩阵微分方程,称之为黎卡蒂(Riccati)方程。解这组矩阵微分方程的初值选取如下

$$
\begin{array}{r l} & \hat {\mathbf {X}} (t _ {0}) = E [ \mathbf {X} (t _ {0}) ] \\ & \mathbf {P} (t _ {0}) = \mathrm{Var} [ \mathbf {X} (t _ {0}) ] \end{array}
$$

例2-16 $\alpha -\beta$ 滤波。

设被估计信息 $S(t)$ 是时间的近似线性函数, 即可近似为

$$
S (t) = a + b t
$$

但具有模型误差 $\ddot{S}(t)=w(t)$ , $w(t)$ 为零均值的白噪声过程, 协方差为 $E[w(t)w(\tau)]=q\delta(t-\tau)$ 。 $S(t)$ 的量测值为

$$
Z (t) = S (t) + V (t)
$$

量测噪声 $V(t)$ 也是零均值的白噪声过程，协方差为 $E[V(t)V(\tau)] = r\delta(t - \tau)$ 。求卡尔曼滤波器达到稳态时的增益阵 K。

解 选取状态变量

$$
X _ {1} = S (t), X _ {2} = \dot {S} (t)
$$

则状态方程为

$$
\dot {\boldsymbol {X}} (t) = \boldsymbol {F} \boldsymbol {X} (t) + \boldsymbol {w} (t)
$$

式中

$$
\boldsymbol {F} = \left[ \begin{array}{l l} 0 & 1 \\ 0 & 0 \end{array} \right], \boldsymbol {w} (t) = \left[ \begin{array}{l} 0 \\ \boldsymbol {w} (t) \end{array} \right]
$$

$$
E [ w (t) ] = 0, E [ w (t) w ^ {\mathrm{T}} (\tau) ] = \left[ \begin{array}{l l} 0 & 0 \\ 0 & q \end{array} \right] \delta (t - \tau)
$$

即

$$
\boldsymbol {q} (t) = \left[ \begin{array}{l l} 0 & 0 \\ 0 & q \end{array} \right]
$$

量测方程为

$$
Z (t) = \mathbf {H} \mathbf {X} + V (t)
$$

式中

$$
\boldsymbol {H} = \left[ \begin{array}{l l} 1 & 0 \end{array} \right], \quad E [ V (t) V (\tau) ] = r \delta (t - \tau)
$$

可以证明系统完全可控和完全可观测,所以卡尔曼滤波器是渐近稳定的。当滤波达到稳定时, $\dot{\boldsymbol{P}}(t)=\boldsymbol{0}$ ,所以黎卡蒂方程为

$$
\boldsymbol {F} \boldsymbol {P} + \boldsymbol {P} \boldsymbol {F} ^ {\mathrm{T}} - \boldsymbol {P} \boldsymbol {H} ^ {\mathrm{T}} r ^ {- 1} \boldsymbol {H} \boldsymbol {P} + q = 0
$$

由此可解得滤波达到稳态时的协方差阵为

$$
\boldsymbol {P} = \left[ \begin{array}{l l} P _ {1 1} & P _ {1 2} \\ P _ {2 1} & P _ {2 2} \end{array} \right] = \left[ \begin{array}{c c} \sqrt {2 r \sqrt {q r}} & \sqrt {q r} \\ \sqrt {q r} & \sqrt {2 q \sqrt {q r}} \end{array} \right]
$$

所以稳态增益阵为

$$
\mathbf {K} = \mathbf {P H} ^ {\mathrm{T}} \mathbf {r} ^ {- 1} = \left[ \begin{array}{c} \sqrt {2 \sqrt {q / r}} \\ \sqrt {q / r} \end{array} \right] = \left[ \begin{array}{c} \alpha \\ \beta \end{array} \right]
$$

例2-17 设线性系统的状态方程为

$$
\dot {X} (t) = F X (t) + w (t)
$$

量测方程为

$$
\mathbf {Z} (t) = \mathbf {H} X (t) + \mathbf {V} (t)
$$

式中， $F$ 为常值， $\mathbf{Z} = \begin{bmatrix} Z_1\\ Z_2 \end{bmatrix},\mathbf{H} = \begin{bmatrix} 1\\ 1 \end{bmatrix},\mathbf{V}(t) = \begin{bmatrix} V_1(t)\\ V_2(t) \end{bmatrix},w(t),V_1(t),V_2(t)$ 是零均值互不相关的白噪声过程，且

$$
E [ w (t) w (\tau) ] = q \delta (t - \tau)
$$

$$
E [ \mathbf {V} (t) \mathbf {V} ^ {\mathrm{T}} (\tau) ] = \left[ \begin{array}{l l} r _ {1} & 0 \\ 0 & r _ {2} \end{array} \right] \delta (t - \tau)
$$

求滤波达到稳态时的状态估计方程。

解 滤波达到稳态时

$$
\dot {P} = F P + P F - P \mathbf {H} ^ {\mathrm{T}} \mathbf {r} ^ {- 1} \mathbf {H} P + q = 0
$$

即

$$
F P + F P - P [ 1 1 ] \left[ \begin{array}{l l} r _ {1} & 0 \\ 0 & r _ {2} \end{array} \right] ^ {- 1} \left[ \begin{array}{l} 1 \\ 1 \end{array} \right] P + q = 0
$$

$$
\left(\frac {1}{r _ {1}} + \frac {1}{r _ {2}}\right) P ^ {2} - 2 F P - q = 0
$$

解得

$$
P = \frac {F + \sqrt {F ^ {2} + q \left(\frac {1}{r _ {1}} + \frac {1}{r _ {2}}\right)}}{\frac {1}{r _ {1}} + \frac {1}{r _ {2}}}
$$

$$
\mathbf {K} = P \mathbf {H} ^ {\mathrm{T}} \mathbf {r} ^ {- 1} = P [ 1 1 ] \left[ \begin{array}{l l} \frac {1}{r _ {1}} & 0 \\ 0 & \frac {1}{r _ {2}} \end{array} \right] = \left[ \begin{array}{l l} \frac {P}{r _ {1}} & \frac {P}{r _ {2}} \end{array} \right]
$$

$$
\dot {\hat {X}} (t) = F \hat {X} (t) + \mathbf {K} [ \mathbf {Z} (t) - \mathbf {H} \hat {X} (t) ]
$$

$$
\dot {\hat {X}} (t) = F \hat {X} (t) + \left[ \frac {P}{r _ {1}} \quad \frac {P}{r _ {2}} \right] \left\{\left[ \begin{array}{l} Z _ {1} (t) \\ Z _ {2} (t) \end{array} \right] - \left[ \begin{array}{l} 1 \\ 1 \end{array} \right] \hat {X} (t) \right\} =
$$

$$
\left(F - \frac {P}{r _ {1}} - \frac {P}{r _ {2}}\right) \hat {X} (t) + \frac {P}{r _ {1}} Z _ {1} (t) + \frac {P}{r _ {2}} Z _ {2} (t)
$$

## 2.3.2 黎卡蒂方程的求解

定理2.11

黎卡蒂方程(见式(2.3.21))

$$
\dot {\boldsymbol {P}} (t) = \boldsymbol {P} (t) \boldsymbol {F} ^ {\mathrm{T}} (t) + \boldsymbol {F} () (t) \boldsymbol {P} (t) - \boldsymbol {P} (t) \boldsymbol {H} ^ {\mathrm{T}} (t) \boldsymbol {r} ^ {- 1} (t) \boldsymbol {H} (t) \boldsymbol {P} (t) + \boldsymbol {G} (t) \boldsymbol {q} (t) \boldsymbol {G} ^ {\mathrm{T}} (t)
$$

在 $\boldsymbol{P}(t_{0})=\operatorname{Var}\boldsymbol{X}(t_{0})=\boldsymbol{P}_{0}$ 初始条件下的解为

$$
\boldsymbol {P} (t) = \boldsymbol {Y} (t) \boldsymbol {\Lambda} ^ {- 1} (t)\tag{2.3.22}
$$

式中 $Y(t)$ 和 $\Lambda(t)$ 为线性矩阵微分方程组

$$
\left. \begin{array}{l} \dot {\mathbf {Y}} (t) = \mathbf {F} (t) \mathbf {Y} (t) + \mathbf {q} (t) \boldsymbol {\Lambda} (t) \\ \dot {\boldsymbol {\Lambda}} (t) = \mathbf {H} ^ {\mathrm{T}} (t) \mathbf {r} ^ {- 1} (t) \mathbf {H} (t) \mathbf {Y} (t) - \mathbf {F} ^ {\mathrm{T}} (t) \boldsymbol {\Lambda} (t) \end{array} \right\}\tag{2.3.23}
$$

满足 $Y(t_{0})=P_{0},\Lambda(t_{0})=I$ 初始条件的解。

证明 设

$$
\boldsymbol {Y} (t) = \boldsymbol {P} (t) \boldsymbol {\Lambda} (t)\tag{2.3.24}
$$

式中 $Y(t)$ 和 $\pmb{\Lambda}(t)$ 满足矩阵微分方程组(2.3.23)。下面证明式(2.3.24)中的 $P(t)$ 为黎卡蒂方程式(2.3.21)的解。

对式 $(2.3.24)$ 两边求导数,得

$$
\dot {\mathbf {Y}} (t) = \dot {\mathbf {P}} (t) \boldsymbol {\Lambda} (t) + \mathbf {P} (t) \dot {\boldsymbol {\Lambda}} (t)
$$

将式 $(2.3.23)$ 关系代入上式,得

$$
\boldsymbol {F} (t) \boldsymbol {Y} (t) + \boldsymbol {q} (t) \boldsymbol {\Lambda} (t) = \dot {\boldsymbol {P}} (t) \boldsymbol {\Lambda} (t) + \boldsymbol {P} (t) [ \boldsymbol {H} (t) \boldsymbol {r} ^ {- 1} (t) \boldsymbol {H} (t) \boldsymbol {Y} (t) - \boldsymbol {F} ^ {\mathrm{T}} (t) \boldsymbol {\Lambda} (t) ]
$$

将式 $(2.3.24)$ 代入上式,得

$$
[ \dot {\boldsymbol {P}} (t) - \boldsymbol {F} (t) \boldsymbol {P} (t) - \boldsymbol {P} (t) \boldsymbol {F} ^ {\mathrm{T}} (t) + \boldsymbol {P} (t) \boldsymbol {H} ^ {\mathrm{T}} (t) \boldsymbol {r} ^ {- 1} (t) \boldsymbol {H} (t) \boldsymbol {P} (t) - \boldsymbol {q} (t) ] \boldsymbol {\Lambda} (t) = \boldsymbol {0}\tag{2.3.25}
$$

又由式(2.3.23)和式(2.3.24)，得

又因为

$$
\begin{array}{r l} \dot {\boldsymbol {\Lambda}} (t) & = [ \boldsymbol {H} ^ {\mathrm{T}} (t) \boldsymbol {r} ^ {- 1} (t) \boldsymbol {H} (t) \boldsymbol {P} (t) - \boldsymbol {F} ^ {\mathrm{T}} (t) ] \boldsymbol {\Lambda} (t) \\ & \quad \boldsymbol {\Lambda} (t _ {0}) = \boldsymbol {I} \end{array}
$$

所以

$$
\pmb {\Lambda} (t) = \exp \int_ {t _ {0}} ^ {t} \pmb {A} (\tau) \mathrm{d} \tau
$$

式中

$$
\mathbf {A} (t) = \mathbf {H} ^ {\mathrm{T}} (t) \mathbf {r} ^ {- 1} (t) \mathbf {H} (t) \mathbf {P} (t) - \mathbf {F} ^ {\mathrm{T}} (t)
$$

可见 $\Lambda(t)$ 总是满秩, 因此由式(2.3.25) 得

$$
\dot {\boldsymbol {P}} (t) - \boldsymbol {F} (t) \boldsymbol {P} (t) - \boldsymbol {P} (t) \boldsymbol {F} ^ {\mathrm{T}} (t) + \boldsymbol {P} (t) \boldsymbol {H} ^ {\mathrm{T}} (t) \boldsymbol {r} ^ {- 1} (t) \boldsymbol {H} (t) \boldsymbol {P} (t) - \boldsymbol {q} (t) = \boldsymbol {0}
$$

这是黎卡蒂方程。可见，由式(2.3.22)确定的 $P(t)$ 是黎卡蒂方程的解。

根据上述分析,可归纳出黎卡蒂方程求解的一般步骤。

(1) 将式(2.3.23)写成矩阵形式如下:

$$
\left[ \begin{array}{c} \dot {\boldsymbol {Y}} (t) \\ \dot {\boldsymbol {\Lambda}} (t) \end{array} \right] = \left[ \begin{array}{c c} \boldsymbol {F} (t) & \boldsymbol {q} (t) \\ \boldsymbol {H} ^ {\mathrm{T}} (t) \boldsymbol {r} ^ {- 1} (t) \boldsymbol {H} (t) & - \boldsymbol {F} ^ {\mathrm{T}} (t) \end{array} \right] \left[ \begin{array}{c} \boldsymbol {Y} (t) \\ \boldsymbol {\Lambda} (t) \end{array} \right]\tag{2.3.26}
$$

其中初始条件为

$$
\left[ \begin{array}{c} \mathbf {Y} (t _ {0}) \\ \boldsymbol {\Lambda} (t _ {0}) \end{array} \right] = \left[ \begin{array}{c} \mathbf {P} _ {0} \\ \mathbf {I} \end{array} \right]
$$

(2) 计算式(2.3.26) 所示系统的一步转移阵得

$$
\Theta (t, t _ {0}) = \left[ \begin{array}{l l} \Theta_ {1 1} (t, t _ {0}) & \Theta_ {1 2} (t, t _ {0}) \\ \Theta_ {2 1} (t, t _ {0}) & \Theta_ {2 2} (t, t _ {0}) \end{array} \right]\tag{2.3.27}
$$

(3) 计算式(2.3.26)的解

$$
\left[ \begin{array}{l} \mathbf {Y} (t) \\ \boldsymbol {\Lambda} (t) \end{array} \right] = \left[ \begin{array}{l l} \boldsymbol {\Theta} _ {1 1} & \boldsymbol {\Theta} _ {1 2} \\ \boldsymbol {\Theta} _ {2 1} & \boldsymbol {\Theta} _ {2 2} \end{array} \right] \left[ \begin{array}{l} \boldsymbol {P} _ {0} \\ \boldsymbol {I} \end{array} \right]
$$

即

$$
\left. \begin{array}{l} \mathbf {Y} (t) = \boldsymbol {\Theta} _ {1 1} \mathbf {P} _ {0} + \boldsymbol {\Theta} _ {1 2} \\ \boldsymbol {\Lambda} (t) = \boldsymbol {\Theta} _ {2 1} \mathbf {P} _ {0} + \boldsymbol {\Theta} _ {2 2} \end{array} \right\}\tag{2.3.28}
$$

(4) 计算黎卡蒂方程的解

$$
\boldsymbol {P} (t) = \boldsymbol {Y} (t) \boldsymbol {\Lambda} ^ {- 1} (t)
$$

例2-18 设系统方程和量测方程分别为

$$
\begin{array}{l} {\dot {\mathbf {X}} (t) = \mathbf {F} (t) \mathbf {X} (t)} \\ {\mathbf {Z} (t) = \mathbf {H} (t) \mathbf {X} (t) + \mathbf {v} (t)} \end{array}
$$

式中

$$
E [ \boldsymbol {v} (t) \boldsymbol {v} ^ {\mathrm{T}} (\tau) ] = \boldsymbol {r} (t) \delta (t - \tau)
$$

求连续型卡尔曼滤波中估计的均方误差阵 $P(t)$ 。

解 由于 $q(t) = 0$ ，所以

$$
\begin{array}{r l} & {\left[ \begin{array}{l} \dot {\mathbf {Y}} (t) \\ \dot {\boldsymbol {\Lambda}} (t) \end{array} \right] = \left[ \begin{array}{c c} \mathbf {F} (t) & \mathbf {0} \\ \mathbf {H} ^ {\mathrm{T}} (t) \mathbf {r} ^ {- 1} (t) \mathbf {H} (t) & - \mathbf {F} ^ {\mathrm{T}} (t) \end{array} \right] \left[ \begin{array}{l} \mathbf {Y} (t) \\ \boldsymbol {\Lambda} (t) \end{array} \right]} \\ & {\quad \dot {\mathbf {Y}} (t) = \mathbf {F} (t) \mathbf {Y} (t) \qquad (\mathbf {Y} (t _ {0}) = \mathbf {P} _ {0})} \end{array}\tag{1}
$$

设方程(1)的转移矩阵为 $\Phi(t,t_{0})$ ，则

$$
\mathbf {Y} (t) = \boldsymbol {\Phi} (t, t _ {0}) \mathbf {P} _ {0}\tag{2}
$$

式(2)与式(2.3.28)比较,得

$$
\begin{array}{r l} \Theta_ {1 1} & = \Phi (t, t _ {0}) \\ \Theta_ {1 2} & = 0 \end{array}
$$

又由于

$$
\begin{array}{r l} \dot {\pmb {\Lambda}} (t) & = \pmb {H} ^ {\mathrm{T}} (t) \pmb {r} ^ {- 1} (t) \pmb {H} (t) \pmb {Y} (t) - \pmb {F} ^ {\mathrm{T}} (t) \pmb {\Lambda} (t) = \\ & - \pmb {F} ^ {\mathrm{T}} (t) \pmb {\Lambda} (t) + \pmb {H} ^ {\mathrm{T}} (t) \pmb {r} ^ {- 1} (t) \pmb {H} (t) \pmb {\Phi} (t, t _ {0}) \pmb {P} _ {0} \\ & \pmb {\Lambda} (t _ {0}) = \pmb {I} \end{array}\tag{3}
$$

可见式(3)相应的齐次方程为式(1)所描述系统的伴随系,式(3)的转移矩阵为 $\boldsymbol{\Phi}^{\mathrm{T}}(t_{0},t)$ 。所以

$$
\pmb {\Lambda} (t) = \pmb {\Phi} ^ {\mathrm{T}} (t _ {0}, t) + \int_ {t _ {0}} ^ {t} \pmb {\Phi} ^ {\mathrm{T}} (\tau , t) \pmb {H} ^ {\mathrm{T}} (\tau) \pmb {r} ^ {- 1} (\tau) \pmb {H} (\tau) \pmb {\Phi} (\tau , t _ {0}) \mathrm{d} \tau \pmb {P} _ {0}
$$

又由于

$$
\begin{array}{r l} & \int_ {t _ {0}} ^ {t} \boldsymbol {\Phi} ^ {\mathrm{T}} (\tau , t) \boldsymbol {H} ^ {\mathrm{T}} (\tau) \boldsymbol {r} ^ {- 1} (\tau) \boldsymbol {H} (\tau) \boldsymbol {\Phi} (\tau , t _ {0}) \mathrm{d} \tau = \\ & \int_ {t _ {0}} ^ {t} [ \boldsymbol {\Phi} (\tau , t _ {0}) \boldsymbol {\Phi} (t _ {0}, t) ] ^ {\mathrm{T}} \boldsymbol {H} ^ {\mathrm{T}} (\tau) \boldsymbol {r} ^ {- 1} (\tau) \boldsymbol {H} (\tau) \boldsymbol {\Phi} (\tau , t _ {0}) \mathrm{d} \tau = \end{array}
$$

$$
\boldsymbol {\Phi} ^ {\mathrm{T}} \left(t _ {0}, t\right) \int_ {t _ {0}} ^ {t} \boldsymbol {\Phi} ^ {\mathrm{T}} \left(\tau , t _ {0}\right) \boldsymbol {H} ^ {\mathrm{T}} (\tau) \boldsymbol {r} ^ {- 1} (\tau) \boldsymbol {H} (\tau) \boldsymbol {\Phi} (\tau , t _ {0}) \mathrm{d} \tau
$$

所以

$$
\boldsymbol {\Lambda} (t) = \boldsymbol {\Phi} ^ {\mathrm{T}} (t _ {0}, t) + \boldsymbol {\Phi} ^ {\mathrm{T}} (t _ {0}, t) \int_ {t _ {0}} ^ {t} \boldsymbol {\Phi} ^ {\mathrm{T}} (\tau , t _ {0}) \boldsymbol {H} ^ {\mathrm{T}} (\tau) \boldsymbol {r} ^ {- 1} (\tau) \boldsymbol {H} (\tau) \boldsymbol {\Phi} (\tau , t _ {0}) \mathrm{d} \tau \boldsymbol {P} _ {0}\tag{4}
$$

式(4)与式(2.3.28)相比较,得

$$
\begin{array}{l} \pmb {\Theta} _ {2 1} = \pmb {\Phi} ^ {\mathrm{T}} (t _ {0}, t) \int_ {t _ {0}} ^ {t} \pmb {\Phi} ^ {\mathrm{T}} (\tau , t _ {0}) \pmb {H} ^ {\mathrm{T}} (\tau) \pmb {r} ^ {- 1} (\tau) \pmb {H} (\tau) \pmb {\Phi} (\tau , t _ {0}) \mathrm{d} \tau \\ \pmb {\Theta} _ {2 2} = \pmb {\Phi} ^ {\mathrm{T}} (t _ {0}, t) \end{array}
$$

根据式 $(2.3.22)$ ，估计的均方误差阵为

$$
\boldsymbol {P} (t) = \boldsymbol {Y} (t) \boldsymbol {\Lambda} ^ {- 1} (t) = \boldsymbol {\Theta} _ {1 1} \boldsymbol {P} _ {0} (\boldsymbol {\Theta} _ {2 1} \boldsymbol {P} _ {0} + \boldsymbol {\Theta} _ {2 2}) ^ {- 1}
$$

## 2.3.3 连续型卡尔曼滤波方程的一般形式

设连续系统的系统方程和量测方程的一般形式为

$$
\dot {\boldsymbol {X}} (t) = \boldsymbol {F} (t) \boldsymbol {X} (t) + \boldsymbol {G} (t) \boldsymbol {w} (t) + \boldsymbol {E} (t) \boldsymbol {U} (t)\tag{2.3.29}
$$

$$
\mathbf {Z} (t) = \mathbf {H} (t) \mathbf {X} (t) + \mathbf {Y} (t) + \mathbf {v} (t)\tag{2.3.30}
$$

式中， $Y(t)$ 和 $U(t)$ 为已知的时间函数， $w(t)$ 和 $v(t)$ 的假设除与式(2.3.2)相同外，还假设两者相关，即

$$
E [ \boldsymbol {w} (t) \boldsymbol {v} ^ {\mathrm{T}} (\tau) ] = \boldsymbol {s} (t) \delta (t - \tau)\tag{2.3.31}
$$

对式 $(2.3.29)$ 和式 $(2.3.30)$ 作离散化处理,得

$$
\left. \begin{array}{l} \mathbf {X} (t _ {k} + \Delta t) = \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) \mathbf {X} (t _ {k}) + \boldsymbol {\Gamma} (t _ {k} + \Delta t, t) \mathbf {W} _ {k} + \mathbf {B} (t _ {k} + \Delta t, t _ {k}) \mathbf {U} (t _ {k}) \\ \mathbf {Z} (t _ {k} + \Delta t) = \mathbf {H} (t _ {k} + \Delta t) \mathbf {X} (t _ {k} + \Delta t) + \mathbf {Y} (t _ {k} + \Delta t) + \mathbf {V} _ {k} \end{array} \right\}\tag{2.3.32}
$$

式中 $W_{k}$ 和 $V_{k}$ 是按式(2.3.5)和式(2.3.6)确定的等效白噪声序列，

$$
\begin{array}{r l} & W _ {k} = \frac {1}{\Delta t} \int_ {t _ {k}} ^ {t _ {k} + \Delta t} w (t) \mathrm{d} t \\ & V _ {k} = \frac {1}{\Delta t} \int_ {t _ {k}} ^ {t _ {k} + \Delta t} v (t) \mathrm{d} t \end{array}
$$

根据式 $(2.3.8)$ 和式 $(2.3.9)$

$$
Q _ {k} = \frac {\pmb {q} (t _ {k})}{\Delta t}\tag{2.3.33}
$$

$$
\boldsymbol {R} _ {k} = \frac {\boldsymbol {r} (t _ {k})}{\Delta t}\tag{2.3.34}
$$

仿照式 $(2.3.7)$ 的推导,得

$$
E [ \boldsymbol {W} _ {k} \boldsymbol {V} _ {j} ^ {\mathrm{T}} ] = \frac {\boldsymbol {s} (t _ {k})}{\Delta t} \delta_ {k j}
$$

即

$$
\mathbf {S} _ {k} = \frac {\mathbf {s} (t _ {k})}{\Delta t}\tag{2.3.35}
$$

根据式 $(2.3.17)$ 和式 $(2.3.18)$

$$
\boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) = \boldsymbol {I} + \boldsymbol {F} (t _ {k}) \Delta t + \varepsilon_ {1} (\Delta t)\tag{2.3.36}
$$

$$
\boldsymbol {\Gamma} (t _ {k} + \Delta t, t _ {k}) = \boldsymbol {G} (t _ {k}) \Delta t + \boldsymbol {\varepsilon} _ {2} (\Delta t)\tag{2.3.37}
$$

同理可推得

$$
\boldsymbol {B} (t _ {k} + \Delta t, t _ {k}) = \boldsymbol {E} (t _ {k}) \Delta t + \boldsymbol {\varepsilon} _ {3} (\Delta t)\tag{2.3.38}
$$

又根据式 $(2.2.46)$ ，有

$$
\boldsymbol {J} \left(t _ {k}\right) = \boldsymbol {\Gamma} \left(t _ {k} + \Delta t, t _ {k}\right) \boldsymbol {S} _ {k} \boldsymbol {R} _ {k} ^ {- 1} = \boldsymbol {\Gamma} \left(t _ {k} + \Delta t, t _ {k}\right) \boldsymbol {s} \left(t _ {k}\right) \boldsymbol {r} ^ {- 1} \left(t _ {k}\right)\tag{2.3.39}
$$

因此，按照离散卡尔曼滤波一般方程式(2.2.46)～式(2.2.51)，式(2.3.32)对应的滤波方程为

$$
\begin{array}{r l} & \hat {\mathbf {X}} (t _ {k} + \Delta t / t _ {k}) = \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) \hat {\mathbf {X}} (t _ {k}) + \boldsymbol {B} (t _ {k} + \Delta t, t _ {k}) \boldsymbol {U} (t _ {k}) + \\ & \qquad \boldsymbol {\Gamma} (t _ {k} + \Delta t, t _ {k}) s (t _ {k}) r ^ {- 1} (t _ {k}) [ \mathbf {Z} (t _ {k}) - \mathbf {Y} (t _ {k}) - \mathbf {H} (t _ {k}) \hat {\mathbf {X}} (t _ {k}) ] \\ & \hat {\mathbf {X}} (t _ {k} + \Delta t) = \hat {\mathbf {X}} (t _ {k} + \Delta t / t _ {k}) + \boldsymbol {K} (t _ {k} + \Delta t) [ \mathbf {Z} (t _ {k} + \Delta t) - \mathbf {Y} (t _ {k} + \Delta t) - \\ & \qquad \boldsymbol {H} (t _ {k} + \Delta t) \boldsymbol {X} (t _ {k} + \Delta t / t _ {k}) ] \\ & \boldsymbol {K} (t _ {k} + \Delta t) = \boldsymbol {P} (t _ {k} + \Delta t) \boldsymbol {H} ^ {\mathrm{T}} (t _ {k} + \Delta t) \left[ \frac {\boldsymbol {r} (t _ {k} + \Delta t)}{\Delta t} \right] ^ {- 1} \\ & \boldsymbol {P} (t _ {k} + \Delta t) = [ \boldsymbol {I} - \boldsymbol {K} (t _ {k} + \Delta t) \boldsymbol {H} (t _ {k} + \Delta t) ] \boldsymbol {P} (t _ {k} + \Delta t / t _ {k}) \\ & \boldsymbol {P} (t _ {k} + \Delta t / t _ {k}) = [ \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) - \boldsymbol {J} (t _ {k}) \boldsymbol {H} (t _ {k}) ] \boldsymbol {P} (t _ {k}) [ \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) - \boldsymbol {J} (t _ {k}) \boldsymbol {H} (t _ {k}) ] ^ {\mathrm{T}} + \\ & \qquad \boldsymbol {\Gamma} (t _ {k} + \Delta t, t _ {k})   \frac {\boldsymbol {q} (t _ {k})}{\Delta t} \boldsymbol {\Gamma} ^ {\mathrm{T}} (t _ {k} + \Delta t, t _ {k}) - \boldsymbol {J} (t _ {k} + \Delta t, t _ {k})   \frac {\boldsymbol {s} ^ {\mathrm{T}} (t _ {k})}{\Delta t} \boldsymbol {\Gamma} ^ {\mathrm{T}} (t _ {k} + \Delta t, t _ {k}) \end{array}
$$

所以根据导数定义

$$
\begin{array}{r l} & {\dot {\hat {X}} (t) = \lim _ {\Delta t \to 0} \frac {\hat {X} (t _ {k} + \Delta t) - \hat {X} (t _ {k})}{\Delta t} =} \\ & {\quad \lim _ {\Delta t \to 0} \frac {1}{\Delta t} [ \Phi (t _ {k} + \Delta t, t _ {k}) - I ] \hat {X} (t _ {k}) + \lim _ {\Delta t \to 0} \frac {1}{\Delta t} P (t _ {k} + \Delta t) H ^ {\mathrm{T}} (t _ {k} + \Delta t) \cdot} \\ & {\quad \left[ \frac {r (t _ {k} + \Delta t)}{\Delta t} \right] ^ {- 1} [ Z (t _ {k} + \Delta t) - Y (t _ {k} + \Delta t) - H (t _ {k} + \Delta t) \hat {X} (t _ {k} + \Delta t / t _ {k}) ] +} \\ & {\quad B (t _ {k} + \Delta t, t _ {k}) U (t _ {k}) + I (t _ {k} + \Delta t, t _ {k}) s (t _ {k}) r ^ {- 1} (t _ {k}) \cdot} \\ & {\quad [ Z (t _ {k}) - Y (t _ {k}) - H (t _ {k}) \hat {X} (t _ {k}) ]} \end{array}
$$

将式(2.3.36)、式(2.3.37)、式(2.3.38)代入上式，并注意到 $\pmb{\varepsilon}_1(\Delta t),\pmb{\varepsilon}_2(\Delta t),\pmb{\varepsilon}_3(\Delta t)$ 都是关于 $\Delta t$ 的高阶无穷小，当 $\Delta t\to 0$ 时， $t_k = k\Delta t$ 成为连续变量 $t$ ，所以

$$
\begin{array}{r l} \dot {\hat {X}} (t) & = F (t) \hat {X} (t) + P (t) H ^ {\mathrm{T}} (t) r ^ {- 1} (t) [ Z (t) - Y (t) - H (t) \hat {X} (t) ] + \\ & E (t) U (t) + G (t) s (t) r ^ {- 1} (t) [ Z (t) - Y (t) - H (t) \hat {X} (t) ] = \\ & F (t) \hat {X} (t) + E (t) U (t) + [ P (t) H ^ {\mathrm{T}} (t) + G (t) s (t) ] r ^ {- 1} (t) \cdot \\ & [ Z (t) - Y (t) - H (t) \hat {X} (t) ] \end{array}
$$

令

$$
\bar {\boldsymbol {K}} (t) = \left[ \boldsymbol {P} (t) \boldsymbol {H} ^ {\mathrm{T}} (t) + \boldsymbol {G} (t) \boldsymbol {s} (t) \right] \boldsymbol {r} ^ {- 1} (t)\tag{2.3.40}
$$

则

$$
\begin{array}{r l} & {\dot {\hat {\mathbf {X}}} (t) = \mathbf {F} (t) \hat {\mathbf {X}} (t) + \mathbf {E} (t) \mathbf {U} (t) + \overline {{{\mathbf {K}}}} (t) [ \mathbf {Z} (t) - \mathbf {Y} (t) - \mathbf {H} (t) \hat {\mathbf {X}} (t) ]} \\ & {\mathbf {P} (t _ {k} + \Delta t) = \left\{\mathbf {I} - \mathbf {P} (t _ {k} + \Delta t) \mathbf {H} ^ {\mathrm{T}} (t _ {k} + \Delta t) \left[ \frac {\mathbf {r} (t _ {k} + \Delta t)}{\Delta t} \right] ^ {- 1} \mathbf {H} (t _ {k} + \Delta t) \right\}.} \end{array}\tag{2.3.41}
$$

$$
\begin{array}{l} \{\left[ \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) - \boldsymbol {J} (t _ {k}) \boldsymbol {H} (t _ {k}) \right] \boldsymbol {P} (t _ {k}) \left[ \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) - \boldsymbol {J} (t _ {k}) \boldsymbol {H} (t _ {k}) \right] ^ {\mathrm{T}} + \\ \boldsymbol {\Gamma} (t _ {k} + \Delta t, t _ {k}) \frac {\boldsymbol {q} (t _ {k})}{\Delta t} \boldsymbol {\Gamma} ^ {\mathrm{T}} (t _ {k} + \Delta t, t _ {k}) - \boldsymbol {J} (t _ {k}) \frac {\boldsymbol {s} ^ {\mathrm{T}} (t _ {k})}{\Delta t} \boldsymbol {\Gamma} ^ {\mathrm{T}} (t _ {k} + \Delta t, t _ {k}) \} = \\ \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) \boldsymbol {P} (t _ {k}) \boldsymbol {\Phi} ^ {\mathrm{T}} (t _ {k} + \Delta t, t _ {k}) - \boldsymbol {J} (t _ {k}) \boldsymbol {H} (t _ {k}) \boldsymbol {P} (t _ {k}) [ \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) - \\ \boldsymbol {J} (t _ {k}) \boldsymbol {H} (t _ {k}) ] ^ {\mathrm{T}} - \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) \boldsymbol {P} (t _ {k}) [ \boldsymbol {J} (t _ {k}) \boldsymbol {H} (t _ {k}) ] ^ {\mathrm{T}} - \\ \boldsymbol {P} (t _ {k} + \Delta t) \boldsymbol {H} ^ {\mathrm{T}} (t _ {k} + \Delta t) \left[ \frac {\boldsymbol {r} (t _ {k} + \Delta t)}{\Delta t} \right] ^ {- 1} \boldsymbol {H} (t _ {k} + \Delta t) [ \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) - \\ \boldsymbol {J} (t _ {k}) \boldsymbol {H} (t _ {k}) ] \boldsymbol {P} (t _ {k}) [ \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) - \boldsymbol {J} (t _ {k}) \boldsymbol {H} (t _ {k}) ] ^ {\mathrm{T}} + \\ \left\{\boldsymbol {I} - \boldsymbol {P} (t _ {k} + \Delta t) \boldsymbol {H} ^ {\mathrm{T}} (t _ {k} + \Delta t) \left[ \frac {\boldsymbol {r} (t _ {k} + \Delta t)}{\Delta t} \right] ^ {- 1} \boldsymbol {H} (t _ {k} + \Delta t) \right\}. \\ \{\boldsymbol {\Gamma} (t _ {k} + \Delta t, t _ {k})   \frac {\boldsymbol {q} (t _ {k})}{\Delta t}   \boldsymbol {\Gamma} ^ {\mathrm{T}} (t _ {k} + \Delta t, t _ {k}) - \boldsymbol {J} (t _ {k})   \frac {\boldsymbol {s} ^ {\mathrm{T}} (t _ {k})}{\Delta t}   \boldsymbol {\Gamma} ^ {\mathrm{T}} (t _ {k} + \Delta t, t _ {k}) \} \end{array}
$$

将式 $(2.3.36)$ 、式 $(2.3.37)$ 代入上式，并注意到

$$
\begin{array}{r l} & {\underset {\Delta t \to 0} {\lim} \Phi (t _ {k} + \Delta t, t _ {k}) = I} \\ & {\underset {\Delta t \to 0} {\lim} J (t _ {k}) = \underset {\Delta t \to 0} {\lim} \Gamma (t _ {k} + \Delta t, t _ {k}) s (t _ {k}) r ^ {- 1} (t _ {k}) =} \\ & {\underset {\Delta t \to 0} {\lim} [ \Delta t G (t _ {k}) + \varepsilon_ {2} (\Delta t) ] s (t _ {k}) r ^ {- 1} (t _ {k}) = 0} \end{array}
$$

所以

$$
\begin{array}{r l}\dot {\boldsymbol {P}} (t)&= \lim _ {\Delta t \rightarrow 0} \frac {\boldsymbol {P} (t _ {k} + \Delta t) - \boldsymbol {P} (t _ {k})}{\Delta t} =\\&\lim _ {\Delta t \rightarrow 0} \frac {1}{\Delta t} \left\{\Delta t \boldsymbol {F} (t _ {k}) \boldsymbol {P} (t _ {k}) + \boldsymbol {P} (t _ {k}) \Delta t \boldsymbol {F} ^ {\mathrm{T}} (t _ {k}) - \Delta t \boldsymbol {G} (t _ {k}) \boldsymbol {s} (t _ {k}) \boldsymbol {r} ^ {- 1} (t _ {k}) \cdot \right.\\&\left. \boldsymbol {H} (t _ {k}) \boldsymbol {P} (t _ {k}) [ \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) - \Delta t \boldsymbol {G} (t _ {k}) \boldsymbol {s} (t _ {k}) \boldsymbol {r} ^ {- 1} (t _ {k}) \boldsymbol {H} (t _ {k}) ] ^ {\mathrm{T}} - \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) \cdot \right.\\&\left. \boldsymbol {P} (t _ {k}) [ \Delta t \boldsymbol {G} (t _ {k}) \boldsymbol {s} (t _ {k}) \boldsymbol {r} ^ {- 1} (t _ {k}) \boldsymbol {H} (t _ {k}) ] ^ {\mathrm{T}} - \boldsymbol {P} (t _ {k} + \Delta t) \boldsymbol {H} ^ {\mathrm{T}} (t _ {k} + \Delta t) \Delta t \cdot \right.\\&\left. \boldsymbol {r} ^ {- 1} (t _ {k} + \Delta t) \boldsymbol {H} (t _ {k} + \Delta t) [ \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) - \boldsymbol {J} (t _ {k}) \boldsymbol {H} (t _ {k}) ] \boldsymbol {P} (t _ {k}) \cdot \right.\\&\left[ \right. \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) - \boldsymbol {J} (t _ {k}) \boldsymbol {H} (t _ {k}) ] ^ {\mathrm{T}} \left. \right\} + \lim _ {\Delta t \rightarrow 0} \frac {1}{\Delta t} \left\{\boldsymbol {I} - \boldsymbol {P} (t _ {k} + \Delta t) \boldsymbol {H} ^ {\mathrm{T}} (t _ {k} + \Delta t) \cdot \right.\\&\left. \Delta t r ^ {- 1} (t _ {k} + \Delta t) \boldsymbol {H} (t _ {k} + \Delta t) \right\} \cdot \left\{\Delta t \boldsymbol {G} (t _ {k})   \frac {\boldsymbol {q} (t _ {k})}{\Delta t}   \Delta t \boldsymbol {G} ^ {\mathrm{T}} (t _ {k}) - \Delta t \boldsymbol {G} (t _ {k})   \cdot \right.\\&s (t _ {k}) r ^ {- 1} (t _ {k})   \frac {\boldsymbol {s} ^ {\mathrm{T}} (t _ {k})}{\Delta t}   \Delta t G ^ {\mathrm{T}} (t _ {k})   +   \lim _ {\Delta t \rightarrow 0} (\Delta t)\\&\end{array}
$$

式中 $\varepsilon(\Delta t)$ 是关于 $\Delta t$ 的高阶小量, 所以

$$
\begin{array}{r l} \dot {\mathbf {P}} (t) & = \mathbf {F} (t) \mathbf {P} (t) + \mathbf {P} (t) \mathbf {F} ^ {\mathrm{T}} (t) - \mathbf {G} (t) \mathbf {s} (t) \mathbf {r} ^ {- 1} (t) \mathbf {H} (t) \mathbf {P} (t) - \\ & \quad \mathbf {P} (t) [ \mathbf {G} (t) \mathbf {s} (t) \mathbf {r} ^ {- 1} (t) \mathbf {H} (t) ] ^ {\mathrm{T}} - \mathbf {P} (t) \mathbf {H} ^ {\mathrm{T}} (t) \mathbf {r} ^ {- 1} (t) \mathbf {H} (t) \mathbf {P} (t) + \\ & \quad \mathbf {G} (t) \mathbf {q} (t) \mathbf {G} ^ {\mathrm{T}} (t) - \mathbf {G} (t) \mathbf {s} (t) \mathbf {r} ^ {- 1} (t) \mathbf {s} ^ {\mathrm{T}} (t) \mathbf {G} ^ {\mathrm{T}} (t) \end{array}
$$

由于

$$
\begin{array}{r l} \overline {{{\boldsymbol {K}}}} (t) \boldsymbol {r} (t) \overline {{{\boldsymbol {K}}}} ^ {\mathrm{T}} (t) & = [ \boldsymbol {P} (t) \boldsymbol {H} ^ {\mathrm{T}} (t) + \boldsymbol {G} (t) \boldsymbol {s} (t) ] \boldsymbol {r} ^ {- 1} (t) \boldsymbol {r} (t) \boldsymbol {r} ^ {- 1} (t) \cdot \\ & [ \boldsymbol {H} (t) \boldsymbol {P} (t) + \boldsymbol {s} ^ {\mathrm{T}} (t) \boldsymbol {G} ^ {\mathrm{T}} (t) ] = \\ & \boldsymbol {P} (t) \boldsymbol {H} ^ {\mathrm{T}} (t) \boldsymbol {r} ^ {- 1} (t) \boldsymbol {H} (t) \boldsymbol {P} (t) + \boldsymbol {G} (t) \boldsymbol {s} (t) \boldsymbol {r} ^ {- 1} (t) \boldsymbol {H} (t) \boldsymbol {P} (t) + \\ & \boldsymbol {P} (t) \boldsymbol {H} ^ {\mathrm{T}} (t) \boldsymbol {r} ^ {- 1} (t) \boldsymbol {s} ^ {\mathrm{T}} (t) \boldsymbol {G} ^ {\mathrm{T}} (t) + \boldsymbol {G} (t) \boldsymbol {s} (t) \boldsymbol {r} ^ {- 1} (t) \boldsymbol {s} ^ {\mathrm{T}} (t) \boldsymbol {G} ^ {\mathrm{T}} (t) \end{array}
$$

所以

$$
\dot {\boldsymbol {P}} (t) = \boldsymbol {F} (t) \boldsymbol {P} (t) + \boldsymbol {P} (t) \boldsymbol {F} ^ {\mathrm{T}} (t) + \boldsymbol {G} (t) \boldsymbol {q} (t) \boldsymbol {G} ^ {\mathrm{T}} (t) - \overline {{\boldsymbol {K}}} (t) \boldsymbol {r} (t) \overline {{\boldsymbol {K}}} ^ {\mathrm{T}} (t)\tag{2.3.42}
$$

式(2.3.40)、式(2.3.41)和式(2.3.42)即为连续型卡尔曼滤波的一般方程。

## 习题

2-1 对确定性状态 $X$ 进行了3次量测，量测方程为

$$
Z _ {1} = 3 = [ 1 1 ] X + V _ {1}
$$

$$
Z _ {2} = 1 = [ 0 1 ] \pmb {X} + V _ {2}
$$

$$
Z _ {3} = 2 = [ 1 0 ] X + V _ {3}
$$

已知量测误差为零均值、方差为 r 的白噪声序列，试分别用一般最小二乘和递推最小二乘估计出 X。

2-2 设 $X$ 为标量随机变量，均值为 $m_X$ ，方差为 $C_X$ ，如果用同一台仪器对 $X$ 直接测量 $k$ 次，得量测值 $Z_1, Z_2, \dots, Z_k$ ，用一般最小二乘估计作为 $X$ 的均值的估计，仪器量测误差的均值为零，方差为 $C_V$ ，试求均值的估计及估计的均方误差。

2-3 设某一电压是正态分布的随机变量，均值为 $m_{e}$ ，方差为 $C_{e}$ 。

（1）用两块较好的电压表同时测量得电压 $Z_{1}$ 和 $Z_{2}$ ，量测误差是零均值的正态分布随机变量，方差为 $C_{V}$ ，求最小方差估计 $\hat{E}$ 和均方误差 $P_{e}$ 。

（2）如果用四块较差的电压表同时测量得的电压为 $Z_{1}, Z_{2}, Z_{3}$ 和 $Z_{4}$ ，量测误差也都是零均值的正态分布随机变量，方差都是 $2C_{V}$ ，求最小方差估计 $\hat{E}$ 和均方误差 $P_{e}$ 。

(3) 比较两种方法的精度。

（4）如果用同一块表顺次测量4次，得到的量测值为 $Z_{1}, Z_{2}, Z_{3}$ 和 $Z_{4}$ ，表的测量误差为零均值正态分布的随机变量，方差为 $C_V$ ，试根据4个量测值确定出相应的4个最小方差估计和均方误差，说明所得结果与(1)、(2)情况所得结果的不同意义。

2-4 设有正态分布的标量平稳随机过程 $X(t)$ ，其相关函数为 $R(\tau)$ ，均值为零， $X(t)$ 能够正确测量到，试根据 $X(t)$ 用最小方差估计的方法来预测 $X(t + T)$ 。

2-5 设 $X$ 和 $N$ 为相互独立的正态随机变量，均值分别为 $m_X$ 和 $m_N$ ，方差分别为 $V_X$ 和 $V_N$ ，量测为 $Z = X + N$ ，测量得 $Z = z$ ，求 $X$ 的最小方差估计 $\hat{X}_{\mathrm{MV}}(z)$ 。

2-6 试根据式(2.1.41)证明式(2.1.42)。

2-7 设 $X$ 是服从正态分布的随机变量，均值为 $m$ ，方差为 $\sigma^2$ ，其中 $\sigma^2$ 已知， $m$ 未知，对 $X$ 作 $n$ 次独立量测，得量测值 $X = [X_1 X_2 \cdots X_n]^T$ ，求 $m$ 的极大似然估计 $m_{\mathrm{ML}}(X)$ 。

2-8 证明下述结论：

(1) 若 X 和 Y 为两个相互独立的随机向量，则 X 和 Y 一定不相关。

(2) 若 X 和 Y 是两个服从正态分布且不相关的随机向量，则 X 和 Y 一定相互独立。

2-9 设 $S(t)$ 和 $N(t)$ 分别为有用信号和干扰信号，两者都是平稳过程，且互不相关，功率谱分别为

$$
S _ {S} (\omega) = \frac {1}{\omega^ {2} + a ^ {2}}, \quad S _ {N} (\omega) = \frac {1}{\omega^ {2} + b ^ {2}}
$$

$a > 0, b > 0$ ，量测为 $Z(t) = S(t) + N(t)$ ，求实现滤波的维纳滤波器的传递函数。

2-10 设 $S(t)$ 的功率谱为

$$
S _ {s} (\omega) = \frac {1}{1 + \omega^ {4}}
$$

求实现最佳预测的维纳滤波器的频率特性。

2-11 试比较线性最小方差估计方程与卡尔曼滤波基本方程的异同。

2-12 设 $X$ 和 $Z$ 为随机向量。如果存在一种估计 $E^{**}[X / Z] = A^1 Z$ ，使

$$
E \left[ (\boldsymbol {X} - \boldsymbol {A} ^ {1} \boldsymbol {Z}) ^ {\mathrm{T}} (\boldsymbol {X} - \boldsymbol {A} ^ {1} \boldsymbol {Z}) \right] \leqslant E \left[ (\boldsymbol {X} - \boldsymbol {A} \boldsymbol {Z}) ^ {\mathrm{T}} (\boldsymbol {X} - \boldsymbol {A} \boldsymbol {Z}) \right]
$$

其中 $\mathbf{A}$ 为任一矩阵，证明：

(1) $A^{1}=R_{XZ}R_{Z}^{-1}$ ，其中 $R_{XZ}=E[XZ^{T}]$ ， $R_{Z}=E[ZZ^{T}]$ ;

(2) 估计有偏；

(3) $E\{(\mathbf{X}-E^{**}[\mathbf{X}/\mathbf{Z}])\mathbf{Z}^{\mathrm{T}}\}=0$ 且只要 $E[(\mathbf{X}-\mathbf{A}\mathbf{Z})\mathbf{Z}^{\mathrm{T}}]=0$ ，则 $A=A^{1}$ 。

2-13 设 $X$ 和 $Z$ 为随机向量，证明：

(1) 若 X 与 AZ + b ( $\forall A$ 和 b) 正交，则 $E^{*}[X/Z] = 0$ ;

(2) 若 X 与 Z 正交, 且 $E[X]=0$ , 则 $E^{*}[X/Z]=0$ 。

2-14 试证明，若 $R_{k}, Q_{k}$ 和 $P_{0}$ 都扩大 $\alpha$ 倍，则卡尔曼滤波基本方程中的增益阵 $K_{k}$ 不变。

2-15 设系统和量测方程分别为

$$
\begin{array}{r} X _ {k + 1} = X _ {k} + W _ {k} \\ Z _ {k} = X _ {k} + V _ {k} \end{array}
$$

$X_{k}$ 和 $Z_{k}$ 都是标量， $\{W_{k}\}$ 和 $\{V_{k}\}$ 都是零均值的白噪声序列，且有

$$
\mathrm{Cov} [ W _ {k}, W _ {j} ] = \delta_ {k j}
$$

$$
\mathrm{Cov} [ V _ {k}, V _ {j} ] = \delta_ {k j}
$$

$W_{k}, V_{k}$ 和 $X_{0}$ 三者互不相关， $m_{X0}=0$ ，量测序列为

$$
\{Z _ {i} \} = \{1, - 2, 4, 3, - 1, 1, 1 \}
$$

试按下述3种情况计算 $\hat{X}_{k + 1 / k}$ 和 $P_{k + 1 / k}$

(1) $P_{0} = \infty$ ; (2) $P_{0} = 1$ ; (3) $P_{0} = 0$ 。

2-16 卫星在空间以恒定的角速度旋转,其角位置每隔 T 秒测量一次,即

$$
Z _ {k} = \theta_ {k} + V _ {k}
$$

式中 $\theta_{k}$ 为卫星在 t=kT 时刻的真实角位置， $V_{k}$ 为量测白噪声，并有

$$
E [ V _ {k} ^ {2} ] = (5 ^ {\circ}) ^ {2} \qquad E [ \theta_ {0} ^ {2} ] = (2 0 ^ {\circ}) ^ {2}
$$

$$
E [ \dot {\theta} _ {0} ^ {2} ] = (2 0 ^ {\circ} / \mathrm{s}) ^ {2} \quad E [ \theta_ {0} ] = E [ \dot {\theta} _ {0} ] = E [ \theta_ {0} \dot {\theta} _ {0} ] = 0
$$

为了估计角位置和角速率，试写出系统的状态方程，并计算 $P_{1/0}, K_1, P_1$ 和 $\hat{X}_1$ 。

2-17 计算机仿真练习题:巡航导弹由舰艇发射,发射点距离目标的距离为 540 km,导弹发射后以 300 m/s 的速度直线等速飞向目标。为了保护目标,目标处装备有监视雷达,并用卡尔曼滤波器估计来袭导弹的距离和速度。雷达以 0.1s 的间隔测量导弹的距离,测量误差为零均值的白噪声,均方根为 10 m。设计卡尔曼滤波器时,距离和速度估计初值取为零,相应的误差均方根分别取 500 km 和 350 m/s,来袭导弹飞行状态的随机性用加速度为白噪声来描述,取方差强度 $q=0.01\ m^{2}/s^{3}$ 。

(1) 设计卡尔曼滤波器；

(2) 作滤波效果的仿真计算(仿真时间:0\~1000s,绘出估计误差曲线)。

2-18 设系统方程和量测方程分别为

$$
\begin{array}{r l} \dot {X} (t) & = - X (t) + w (t) \\ Z (t) & = X (t) + v (t) \end{array}
$$

$X(t)$ 和 $Z(t)$ 均为标量，列写出连续型卡尔曼滤波方程，并求出滤波达到稳态时的估计均方误差和最佳增益。其中 $w(t)$ 和 $v(t)$ 是零均值且互不相关的白噪声过程， $q=2\alpha, r=\alpha$ 。

2-19 若定常连续系统噪声方差强度阵 $q$ 和 $r$ 都增加 $\alpha$ 倍，试证明稳态滤波器的增益 $\overline{K}$ 不变。

2-20 设系统方程和量测方程分别为

$$
\begin{array}{r l} \dot {\mathbf {X}} (t) & = \left[ \begin{array}{l l} 0 & 1 \\ - 1 & 0 \end{array} \right] \mathbf {X} (t) + \left[ \begin{array}{l} 0 \\ 1 \end{array} \right] w (t) \\ Z (t) & = [ 1 0 ] \mathbf {X} (t) + v (t) \end{array}
$$

其中 $w(t)$ 和 $v(t)$ 是零均值且互不相关的白噪声过程, q=2, r=1 。列写出连续型卡尔曼滤波方程, 并求出稳态解。

## 2-21 设连续系统状态方程为

$$
\dot {\boldsymbol {X}} (t) = \boldsymbol {F} (t) \boldsymbol {X} (t) + \boldsymbol {E} (t) \boldsymbol {U} (t) + \boldsymbol {G} (t) \boldsymbol {w} (t)
$$

其中 $w(t)$ 是均值为零方差强度为 $q(t)$ 的白噪声过程向量。系统经离散化处理后得

$$
\boldsymbol {X} \left(t _ {k + 1}\right) = \boldsymbol {\Phi} \left(t _ {k + 1}, t _ {k}\right) \boldsymbol {X} \left(t _ {k}\right) + \boldsymbol {B} \left(t _ {k}\right) \boldsymbol {U} \left(t _ {k}\right) + \boldsymbol {\Gamma} \left(t _ {k}\right) \boldsymbol {W} _ {k}
$$

其中

$$
\pmb {\Gamma} (t _ {k}) = \int_ {t _ {k}} ^ {t _ {k + 1}} \pmb {\Phi} (t _ {k + 1}, \tau) \pmb {G} (\tau) \mathrm{d} \tau
$$

$$
\pmb {B} (t _ {k}) = \int_ {t _ {k}} ^ {t _ {k + 1}} \pmb {\Phi} (t _ {k + 1}, \tau) \pmb {E} (\tau) \mathrm{d} \tau
$$

$\Phi(t_{k+1},t_{k})$ 为一步转移阵。证明

$$
\begin{array}{l} \lim _ {T \to 0} \frac {\boldsymbol {\Gamma} (t _ {k})}{T} = \boldsymbol {G} (t) \\ \lim _ {T \to 0} \frac {\boldsymbol {B} (t _ {k})}{T} = \boldsymbol {E} (t) \\ \lim _ {T \to 0} \boldsymbol {\Phi} (t _ {k + 1}, t _ {k}) = \boldsymbol {I} \\ T = t _ {k + 1} - t _ {k}. \end{array}
$$

其中

2-22 设运动体沿某直线运动，t 时刻的位移，速度和加速度分别为 $s(t)$ ， $v(t)$ 和 $a(t)$ 。 $t_{k}$ 时刻的量测为 $Z_{k}=S_{k}+V_{k}$ ，其中 $s_{k}=s(t_{k})$ ， $k=1,2,3,\cdots$ 。运动体机动运动引起的运动不定性用 $a(t)$ 描述，其中

$$
\begin{array}{r l} & E [ a (t) ] = 0, \quad E [ a (t) a (\tau) ] = q \delta (t - \tau) \\ & E [ V _ {k} ] = 0, \quad E [ V _ {k} \quad V _ {l} ] = R \delta_ {k l} \end{array}
$$

量测量的采样周期为 T。列写出 $s_{k}$ 和 $v_{k}$ 的稳态滤波方程。

