# 第6章 卡尔曼滤波的推广

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

例6-1 设某连续非线性系统数学模型为

$$
\left\{ \begin{array}{l} \dot {x} (t) = - x (t) + u (t) + w (t) \\ z (t) = x ^ {3} (t) + v (t) \end{array} \right.
$$

式中 $x(t)$ 和 $z(t)$ 是标量函数， $E[x(0)]=-1, u(t)=5, w(t)$ 和 $v(t)$ 是白噪声过程，即有 $E[w(t)w(\tau)]=q\delta(t-\tau), E[v(t)v(\tau)]=r\delta(t-\tau), P(0)=C_{x_{0}}=0$ 。试求 $x(t)$ 的线性化卡尔曼滤波和广义卡尔曼滤波方程。

解 （1）求 $x(t)$ 的线性化卡尔曼滤波方程。由系统方程可得标称轨迹数学模型为

$$
\left\{ \begin{array}{l} \dot {x} ^ {\mathrm{n}} (t) = - x ^ {\mathrm{n}} (t) + 5 \\ z ^ {\mathrm{n}} (t) = x ^ {\mathrm{n} ^ {3}} (t) \end{array} \right.
$$

式中

于是可直接解得

$$
x ^ {n} (0) = - 1
$$

$$
x ^ {\mathrm{n}} (t) = 5 - 6 \mathrm{e} ^ {- t}
$$

$$
z ^ {n} (t) = (5 - 6 \mathrm{e} ^ {- t}) ^ {3}
$$

并且

$$
f [ x (t), t ] = - x (t), B (t) = 1, h [ x (t), t ] = x ^ {3} (t)
$$

因此可算得

$$
F ^ {\mathrm{n}} (t) = \frac {\partial f [ x (t) , t ]}{\partial x (t)} \Bigg | _ {x (t) = x ^ {\mathrm{n}} (t)} = - 1
$$

$$
H ^ {\mathrm{n}} (t) = \frac {\partial h [ x (t) , t ]}{\partial x (t)} \Bigg | _ {x (t) = x ^ {\mathrm{n}} (t)} = 3 x ^ {2} (t) = 3 (5 - 6 \mathrm{e} ^ {- t}) ^ {2}
$$

再由式(6.2.25)～式(6.2.29)得线性化卡尔曼滤波方程

$$
\begin{array}{r l} \dot {P} (t) & = P (t) F ^ {\mathrm{n}} (t) + F ^ {\mathrm{n}} (t) P (t) - P (t) H ^ {\mathrm{n}} (t) r ^ {- 1} (t) H ^ {\mathrm{n}} (t) P (t) + G (t) q (t) G (t) = \\ & - 2 P (t) - 9 (5 - 6 \mathrm{e} ^ {- t}) ^ {4} r ^ {- 1} P ^ {2} (t) + q \end{array}
$$

$$
\hat {x} (t) = x ^ {\mathrm{n}} (t) + \Delta \hat {x} (t) = (5 - 6 \mathrm{e} ^ {- t}) + \Delta \hat {x} (t)
$$

初始条件：

$$
\hat {x} (0) = E [ x (0) ] = - 1, P (0) = 0
$$

只要求解以上方程组,即可解得 $\hat{x}(t)$ 值。

(2) 求 $x(t)$ 的广义卡尔曼滤波方程。根据本题给出的条件, 得

$$
f [ x (t), t ] \mid_ {x (t) = \hat {x} (t)} = - \hat {x} (t)
$$

$$
\left. \frac {\partial f [ x (t) , t ]}{\partial x (t)} \right| _ {x (t) = \hat {x} (t)} = - 1
$$

$$
h [ x (t), t ] \mid_ {x (t) = \hat {x} (t)} = \hat {x} ^ {3} (t)
$$

$$
\left. \frac {\partial h [ x (t) , t ]}{\partial x (t)} \right| _ {\dot {x} (t) = \dot {x} (t)} = 3 \hat {x} ^ {2} (t)
$$

于是由式(6.2.74)～式(6.2.76)可以得广义卡尔曼滤波方程组

$$
\dot {\hat {x}} (t) = - \hat {x} (t) + 3 \hat {x} ^ {2} (t) r ^ {- 1} P (t) [ Z (t) - \hat {x} ^ {3} (t) ] + 5
$$

$$
\dot {P} (t) = - 2 P (t) - 9 \hat {x} ^ {4} (t) r ^ {- 1} P ^ {2} (t) + q
$$

初始条件：

$$
\hat {x} (0) = - 1, P (0) = 0
$$

求解以上方程组,即可获得状态最优估计 $\hat{x}(t)$ 。

## 6.2.4 二阶广义卡尔曼滤波方程[119]

如果在非线性向量函数的泰勒级数展开式中除保留一次项外,还保留二次项,则相应的滤波算法即为二阶广义卡尔曼滤波,简称二阶 EKF。

设系统方程和量测方程为：

$$
\dot {\boldsymbol {X}} (t) = \boldsymbol {f} [ \boldsymbol {X} (t), t ] + \boldsymbol {G} (t) \boldsymbol {w} (t)\tag{6.2.99}
$$

$$
\mathbf {Z} _ {k} = \mathbf {h} \left[ \mathbf {X} _ {k}, t _ {k} \right] + \mathbf {V} _ {k}\tag{6.2.100}
$$

式中， $w(t)$ 和 $V_{k}$ 均为零均值的高斯白噪声， $w(t)$ 的方差强度阵为 q， $V_{k}$ 的方差阵为 $R_{k}$ 。

由于物理系统一般为连续系统,而量测量一般在离散时间上获取,所以此处假设系统方程具有连续形式,而量测方程具有离散形式。此类系统的最优估计亦称混合滤波。

引入偏微分算子

$$
\boldsymbol {\nabla} = \left[ \frac {\partial}{\partial X _ {1}} \quad \frac {\partial}{\partial X _ {2}} \quad \dots \quad \frac {\partial}{\partial X _ {n}} \right] ^ {T}\tag{6.2.101}
$$

则非线性向量函数在 $\hat{\pmb{X}}(t)$ 上的二阶泰勒级数为

$$
\begin{array}{r l} \boldsymbol {f} [ \boldsymbol {X} (t), t ] & = \boldsymbol {f} [ \hat {\boldsymbol {X}} (t), t ] + \nabla \boldsymbol {f} [ \boldsymbol {X} (t), t ] \Big | _ {\boldsymbol {x} = \hat {\boldsymbol {x}}} [ \boldsymbol {X} (t) - \hat {\boldsymbol {X}} (t) ] + \\ & \frac {1}{2} \left\{\left[ \boldsymbol {X} (t) - \hat {\boldsymbol {X}} (t) \right] \cdot \nabla \right\} ^ {2} \boldsymbol {f} [ \boldsymbol {X} (t), t ] \Big | _ {\boldsymbol {x} = \hat {\boldsymbol {x}}} \end{array}\tag{6.2.101}
$$

式中(·)为向量的点乘。

记(6.2.101)式第三项为 $T_{3}$ ，则 $T_{3}$ 可写成：

$$
\boldsymbol {T} _ {3} = \frac {1}{2} \left\{\left[ \boldsymbol {X} (t) - \hat {\boldsymbol {X}} (t) \right] ^ {\mathrm{T}} \boldsymbol {\nabla} \boldsymbol {\nabla} ^ {\mathrm{T}} [ \boldsymbol {X} (t) - \hat {\boldsymbol {X}} (t) ] \boldsymbol {f} [ \boldsymbol {X} (t), t ] \mid_ {\boldsymbol {X} = \hat {\boldsymbol {X}}} \right\}
$$

记

$$
\begin{array}{r l} \boldsymbol {f} [ \boldsymbol {X} (t), t ] & = \left[ \begin{array}{l l l l} f _ {1} & f _ {2} & \dots & f _ {n} \end{array} \right] ^ {\mathrm{T}} \\ & [ \boldsymbol {X} (t) - \hat {\boldsymbol {X}} (t) ] = \delta \boldsymbol {X} \end{array}
$$

则

$$
\begin{array}{r l} \boldsymbol {T} _ {3} & = \frac {1}{2} \left\{\left[ \boldsymbol {X} (t) - \hat {\boldsymbol {X}} (t) \right] ^ {\mathrm{T}} \nabla \nabla^ {\mathrm{T}} \left[ \boldsymbol {X} (t) - \hat {\boldsymbol {X}} (t) \right] \boldsymbol {f} [ \boldsymbol {X} (t), t ] \Big | _ {\boldsymbol {X} - \hat {\boldsymbol {X}}} \right\} = \\ & \quad \frac {1}{2} \sum_ {i = 1} ^ {n} \delta \boldsymbol {X} ^ {\mathrm{T}} \nabla \nabla^ {\mathrm{T}} \delta \boldsymbol {X} \boldsymbol {e} _ {i} f _ {i} \Big | _ {\boldsymbol {X} = \hat {\boldsymbol {X}}} \end{array}
$$

式中， $e_{i}$ 是第i个元为1而其余元均为0的单位向量。由于 $\delta X^{T}\nabla V^{T}\delta X$ 的维数为1， $f_{i}$ 为标量，微分算子只对f起作用，所以

$$
\boldsymbol {T} _ {3} = \frac {1}{2} \sum_ {i = 1} ^ {n} \boldsymbol {e} _ {i} \delta \boldsymbol {X} ^ {\mathrm{T}} (\boldsymbol {\nabla} \boldsymbol {\nabla} ^ {\mathrm{T}} f _ {i}) \Big | _ {\boldsymbol {X} = \dot {\boldsymbol {X}}} \delta \boldsymbol {X}
$$

式中， $\nabla\nabla^{T}f_{i}$ 为二阶偏导数阵，即

$$
\nabla \nabla^ {\mathrm{T}} f _ {i} \Big | _ {X = \hat {X}} = \left[ \begin{array}{c c c c} \frac {\partial^ {2} f _ {i}}{\partial x _ {1} ^ {2}} & \frac {\partial^ {2} f _ {i}}{\partial x _ {1} \partial x _ {2}} & \dots & \frac {\partial^ {2} f _ {i}}{\partial x _ {1} \partial x _ {n}} \\ \frac {\partial^ {2} f _ {i}}{\partial x _ {2} \partial x _ {1}} & \frac {\partial^ {2} f _ {i}}{\partial x _ {2} ^ {2}} & \dots & \frac {\partial^ {2} f _ {i}}{\partial x _ {2} \partial x _ {n}} \\ \vdots & \vdots & & \vdots \\ \frac {\partial^ {2} f _ {i}}{\partial x _ {n} \partial x _ {1}} & \frac {\partial^ {2} f _ {i}}{\partial x _ {n} \partial x _ {2}} & \dots & \frac {\partial^ {2} f _ {i}}{\partial x _ {n} ^ {2}} \end{array} \right] _ {X = \hat {X}}\tag{6.2.102}
$$

由于

$$
\delta \boldsymbol {X} ^ {\mathrm{T}} (\boldsymbol {\nabla} \boldsymbol {\nabla} ^ {\mathrm{T}} f _ {i}) \delta \boldsymbol {X} = \operatorname{Tr} \left[ (\boldsymbol {\nabla} \boldsymbol {\nabla} ^ {\mathrm{T}} f _ {i}) \delta \boldsymbol {X} \delta \boldsymbol {X} ^ {\mathrm{T}} \right]
$$

式中，Tr 为对矩阵求迹，所以

$$
\boldsymbol {T} _ {3} = \frac {1}{2} \sum_ {i = 1} ^ {n} \boldsymbol {e} _ {i} \operatorname{Tr} \left[ (\boldsymbol {\nabla} \boldsymbol {\nabla} ^ {\mathrm{T}} f _ {i}) \Big | _ {\boldsymbol {X} = \dot {\boldsymbol {X}}} \delta \boldsymbol {X} \delta \boldsymbol {X} ^ {\mathrm{T}} \right]
$$

上式中， $\delta X\delta X^{T}$ 是未知量，而卡尔曼滤波可获得其均方误差阵，即

$$
\boldsymbol {P} = E \left[ \delta \boldsymbol {X} \delta \boldsymbol {X} ^ {\mathrm{T}} \right]
$$

为便于计算, $\delta X\delta X^{T}$ 用其均值P近似,则

$$
\boldsymbol {T} _ {3} = \frac {1}{2} \sum_ {i = 1} ^ {n} \boldsymbol {e} _ {i} \operatorname{Tr} \left[ (\boldsymbol {\nabla} \boldsymbol {\nabla} ^ {\mathrm{T}} f _ {i}) \Big | _ {\boldsymbol {x} = \dot {\boldsymbol {x}}} \boldsymbol {P} \right]
$$

上式代入式(6.2.101)，同样， $\boldsymbol{X}(t)-\hat{\boldsymbol{X}}(t)$ 用其均值近似，则

$$
\boldsymbol {f} [ \boldsymbol {X} (t), t ] = \boldsymbol {f} [ \hat {\boldsymbol {X}} (t), t ] + \frac {1}{2} \sum_ {i = 1} ^ {n} \boldsymbol {e} _ {i} \operatorname{Tr} \left[ (\boldsymbol {\nabla} \boldsymbol {\nabla} ^ {\mathrm{T}} f _ {i}) \Big | _ {\boldsymbol {X} = \hat {\boldsymbol {X}}} \boldsymbol {P} \right]\tag{6.2.103}
$$

1. 二阶 EKF 的时间更新算法

式(6.2.103)代入式(6.2.99)，则由时间更新确定的 $X(t)$ 的估计由下述方程确定：

$$
\dot {\hat {X}} ^ {-} (t) = f [ \hat {X} ^ {-} (t), t ] + \frac {1}{2} \sum_ {i = 1} ^ {n} e _ {i} \operatorname{Tr} \left[ (\boldsymbol {\nabla} \boldsymbol {\nabla} ^ {\mathrm{T}} f _ {i}) \Big | _ {\boldsymbol {x} = \hat {\boldsymbol {x}}} \boldsymbol {P} ^ {-} \right]\tag{6.2.104}
$$

相应的均方误差阵 $P^{-}$ 由下述方程确定：

$$
\dot {\boldsymbol {P}} ^ {-} (t) = \boldsymbol {F} (t) \boldsymbol {P} ^ {-} (t) + \boldsymbol {P} ^ {-} (t) \boldsymbol {F} ^ {\mathrm{T}} (t) + \boldsymbol {G} (t) \boldsymbol {q} \boldsymbol {G} ^ {\mathrm{T}} (t)\tag{6.2.105}
$$

式中 $F(t) = \frac{\partial f(t)}{\partial X^{\mathrm{T}}(t)}\Big|_{X(t) = \hat{X}^{-}(t)}$ ，微分方程采用龙格一库塔算法求解，在 $[t_{k - 1},t_k]$ 滤波周期内，积分初值取 $\hat{X}_{k - 1}$ 和 $P_{k - 1}$ ，由上一步量测更新确定，终值 $\hat{X}^{-}(t_k)$ 和 $P^{-}(t_k)$ 为本次量测更新所用的 $\hat{X}_{k / k - 1}$ 和 $P_{k / k - 1}$ 。

## 2. 二阶 EKF 的量测更新算法

设 $t_{k}$ 时刻的量测为 $Z_{k}$ ，则根据 $Z_{k}$ 和时间更新确定的 $X_{k}$ 的估计为

$$
\hat {\boldsymbol {X}} _ {k} = \hat {\boldsymbol {X}} _ {k / k - 1} + \boldsymbol {K} _ {k} \left[ \boldsymbol {Z} _ {k} - \boldsymbol {h} (\hat {\boldsymbol {X}} _ {k / k - 1}, t _ {k}) \right] - \boldsymbol {C} _ {k}\tag{6.2.106}
$$

式中， $\hat{\mathbf{X}}_{k/k-1} = \hat{\mathbf{X}}^{-}(t_k)$ ，由式(6.2.104)确定。 $\mathbf{C}_k$ 为补偿项，与最佳增益一样，均为待定值。引入 $\mathbf{C}_k$ 的目的是使 $\hat{\mathbf{X}}_k$ 为无偏估计，而确定 $\mathbf{K}_k$ 的最优指标是使 $\hat{\mathbf{X}}_k$ 的均方误差阵 $\mathbf{P}_k$ 的迹达到最小。

$$
\begin{array}{r l} & {\text {记} \widetilde {X} _ {k / k - 1} = X _ {k} - \hat {X} _ {k / k - 1}} \\ & {\widetilde {X} _ {k} = X _ {k} - \hat {X} _ {k}} \end{array}
$$

则根据式 $(6.2.106)$

$$
\begin{array}{r l} \widetilde {\boldsymbol {X}} _ {k} & = \boldsymbol {X} _ {k} - \hat {\boldsymbol {X}} _ {k} + \hat {\boldsymbol {X}} _ {k / k - 1} - \hat {\boldsymbol {X}} _ {k / k - 1} = \widetilde {\boldsymbol {X}} _ {k / k - 1} - \hat {\boldsymbol {X}} _ {k} + \hat {\boldsymbol {X}} _ {k / k - 1} = \\ & \quad \widetilde {\boldsymbol {X}} _ {k / k - 1} - \boldsymbol {K} _ {k} [ \boldsymbol {h} (\boldsymbol {X} _ {k}, t _ {k}) - \boldsymbol {h} (\hat {\boldsymbol {X}} _ {k / k - 1}, t _ {k}) ] - \boldsymbol {K} _ {k} \boldsymbol {V} _ {k} + \boldsymbol {C} _ {k} = \\ & \quad \widetilde {\boldsymbol {X}} _ {k / k - 1} - \boldsymbol {K} _ {k} [ \boldsymbol {h} (\hat {\boldsymbol {X}} _ {k - k - 1}, t _ {k}) + \boldsymbol {H} _ {k} (\boldsymbol {X} _ {k} - \hat {\boldsymbol {X}} _ {k / k - 1}) + \frac {1}{2} \sum_ {j = 1} ^ {m} e _ {j} (\boldsymbol {X} _ {k} - \hat {\boldsymbol {X}} _ {k / k - 1}) ^ {\mathrm{T}} \\ & \quad (\nabla \nabla^ {\mathrm{T}} h _ {j}) \Big | _ {\boldsymbol {X} _ {k} = \hat {\boldsymbol {X}} _ {k / k - 1}} (\boldsymbol {X} _ {k} - \hat {\boldsymbol {X}} _ {k / k - 1}) - \boldsymbol {h} (\hat {\boldsymbol {X}} _ {k / k - 1}, t _ {k}) ] - \boldsymbol {K} _ {k} \boldsymbol {V} _ {k} + \boldsymbol {C} _ {k} = \\ & \quad \widetilde {\boldsymbol {X}} _ {k / k - 1} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k} \widetilde {\boldsymbol {X}} _ {k / k - 1} - \frac {1}{2} \boldsymbol {K} _ {k} \sum_ {j = 1} ^ {m} e _ {j} (\widetilde {\boldsymbol {X}} _ {k / k - 1}) ^ {\mathrm{T}} (\nabla \nabla^ {\mathrm{T}} h _ {j}) \Big | _ {\boldsymbol {X} _ {k} = \hat {\boldsymbol {X}} _ {k / k - 1}} \widetilde {\boldsymbol {X}} _ {k / k - 1} - \boldsymbol {K} _ {k} \boldsymbol {V} _ {k} + \boldsymbol {C} _ {k} \end{array}\tag{6.2.107}
$$

式中， $H_{k}=\frac{\partial h}{\partial X^{T}}\bigg|_{X_{k}=\dot{X}_{k/k-1}},h_{j}$ 为 h 的第 j 个分量。

根据式(6.2.107)，假设 $\hat{\mathbf{X}}_{k / k - 1}$ 为无偏估计，即 $E[\widetilde{\mathbf{X}}_{k / k - 1}] = \mathbf{0}$ ，则要使 $\hat{\mathbf{X}}_k$ 为无偏估计，即 $E[\widetilde{\mathbf{X}}_k] = \mathbf{0}$ 需有

$$
\begin{array}{r l} \mathbf {C} _ {k} & = \frac {1}{2} E \left[ \mathbf {K} _ {k} \sum_ {j = 1} ^ {m} \mathbf {e} _ {j} (\widetilde {\mathbf {X}} _ {k / k - 1}) ^ {\mathrm{T}} (\nabla \nabla^ {\mathrm{T}} h _ {j}) \Big | _ {\mathbf {x} _ {k} = \hat {\mathbf {x}} _ {k / k - 1}} \widetilde {\mathbf {X}} _ {k / k - 1} \right] = \\ & \quad \frac {1}{2} \mathbf {K} _ {k} \sum_ {j = 1} ^ {m} \mathbf {e} _ {j} \operatorname{Tr} \left[ (\nabla \nabla^ {\mathrm{T}} h _ {j}) \Big | _ {\mathbf {x} _ {k} = \hat {\mathbf {x}} _ {k / k - 1}} \mathbf {P} _ {k / k - 1} \right] \end{array}\tag{6.2.108}
$$

式 $(6.2.108)$ 代入式 $(6.2.107)$

$$
\begin{array}{r l} \widetilde {\mathbf {X}} _ {k} & = (\mathbf {I} - \mathbf {K} _ {k} \mathbf {H} _ {k}) \widetilde {\mathbf {X}} _ {k / k - 1} + \frac {1}{2} \mathbf {K} _ {k} \left\{\sum_ {j = 1} ^ {m} \boldsymbol {e} _ {j} \operatorname{Tr} \left[ (\nabla \nabla^ {\mathrm{T}} h _ {j}) \Big | _ {\hat {\mathbf {x}} _ {k / k - 1}} \mathbf {P} _ {k / k - 1} \right] - \right. \\ & \left. \sum_ {j = 1} ^ {m} \boldsymbol {e} _ {j} \operatorname{Tr} \left[ (\nabla \nabla^ {\mathrm{T}} h _ {j}) \Big | _ {\hat {\mathbf {x}} _ {k / k - 1}} (\widetilde {\mathbf {X}} _ {k / k - 1} (\widetilde {\mathbf {X}} _ {k / k - 1}) ^ {\mathrm{T}}) \right] \right\} - \mathbf {K} _ {k} \mathbf {V} _ {k} \end{array}
$$

记

$$
\begin{array}{l} \boldsymbol {A} = \frac {1}{2} \sum_ {j = 1} ^ {m} \boldsymbol {e} _ {j} \operatorname{Tr} \left[ (\boldsymbol {\nabla} \boldsymbol {\nabla} ^ {\mathrm{T}} h _ {j}) \Big | _ {\hat {\mathbf {x}} _ {k / k - 1}} \boldsymbol {P} _ {k / k - 1} \right] - \frac {1}{2} \sum_ {j = 1} ^ {m} \boldsymbol {e} _ {j} \operatorname{Tr} \left[ (\boldsymbol {\nabla} \boldsymbol {\nabla} ^ {\mathrm{T}} h _ {j}) \Big | _ {\hat {\mathbf {x}} _ {k / k - 1}} (\widetilde {\boldsymbol {X}} _ {k / k - 1} (\widetilde {\boldsymbol {X}} _ {k / k - 1}) ^ {\mathrm{T}}) \right] = \\ \frac {1}{2} \sum_ {j = 1} ^ {m} \boldsymbol {e} _ {j} \operatorname{Tr} \left\{(\boldsymbol {\nabla} \boldsymbol {\nabla} ^ {\mathrm{T}} h _ {j}) \Big | _ {\hat {\mathbf {x}} _ {k / k - 1}} [ \boldsymbol {P} _ {k / k - 1} - (\widetilde {\boldsymbol {X}} _ {k / k - 1} (\widetilde {\boldsymbol {X}} _ {k / k - 1}) ^ {\mathrm{T}}) ] \right\} \end{array} \tag {6.2.109}
$$

则

$$
\widetilde {\boldsymbol {X}} _ {k} = (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) \widetilde {\boldsymbol {X}} _ {k / k - 1} + \boldsymbol {K} _ {k} \boldsymbol {A} - \boldsymbol {K} _ {k} \boldsymbol {V} _ {k}\tag{6.2.110}
$$

由于 $E[\widetilde{\mathbf{X}}_{k / k - 1}] = \mathbf{0}, E[\mathbf{V}_k] = \mathbf{0}, \mathbf{V}_k$ 与 $\pmb{A}$ 不相关，所以由式(6.2.110)，

$$
\boldsymbol {P} _ {k} = E \left[ \widetilde {\boldsymbol {X}} _ {k} \left(\widetilde {\boldsymbol {X}} _ {k}\right) ^ {\mathrm{T}} \right] = \left(\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}\right) \boldsymbol {P} _ {k / k - 1} \left(\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}\right) ^ {\mathrm{T}} + \boldsymbol {K} _ {k} \left(\boldsymbol {\Lambda} _ {k} + \boldsymbol {R} _ {k}\right) \boldsymbol {K} _ {k} ^ {\mathrm{T}}\tag{6.2.111}
$$

式中

$$
\pmb {\Lambda} _ {k} = E [ \pmb {A A} ^ {\mathrm{T}} ]\tag{6.2.112}
$$

构造代价函数

$$
J _ {k} = E \left[ \left(\widetilde {\boldsymbol {X}} _ {k}\right) ^ {\mathrm{T}} \mathbf {S} _ {k} \widetilde {\boldsymbol {X}} _ {k} \right] = \operatorname{Tr} \left[ \boldsymbol {S} _ {k} \boldsymbol {P} _ {k} \right]
$$

式中 $S_{k}$ 为任意正定加权阵。

由式(6.2.111)，要使 $J_{k}$ 达到极小，最佳增益应为

$$
\boldsymbol {K} _ {k} = \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k} + \boldsymbol {\Lambda} _ {k}) ^ {- 1}\tag{6.2.113}
$$

利用式 $(6.2.113)$ ，可将式 $(6.2.111)$ 改写成更实用的形式：

$$
\begin{array}{r l} \boldsymbol {P} _ {k} & = \boldsymbol {P} _ {k / k - 1} - \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {K} _ {k} ^ {\mathrm{T}} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} + \boldsymbol {K} _ {k} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {K} _ {k} ^ {\mathrm{T}} + \boldsymbol {K} _ {k} (\boldsymbol {\Lambda} _ {k} + \boldsymbol {R} _ {k}) \boldsymbol {K} _ {k} ^ {\mathrm{T}} = \\ & \quad \boldsymbol {P} _ {k / k - 1} - \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k} + \boldsymbol {\Lambda} _ {k}) ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} - \\ & \quad \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k} + \boldsymbol {\Lambda} _ {k}) ^ {- 1} \boldbf {H} _ {k} \boldsymbol {P} _ {k / k - 1} + \\ & \quad \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k} + \boldsymbol {\Lambda} _ {k}) ^ {- 2} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} = \\ & \quad \boldsymbol {P} _ {k / k - 1} - \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {\mathbf {\Lambda}} + \boldsymbol {\Lambda_ {\mathbf {\Lambda}}}) ^ {- 2} \boldsymbol {H _ {\mathbf {\Lambda}}} \boldsymbol {P _ {\mathbf {\Lambda}}} _ {\mathbf {\Lambda}}. \end{array}\tag{6.2.114}
$$

式中 $\Lambda_{k}$ 按下述分析确定。

根据式 $(6.2.112)$ 和式 $(6.2.109)$ ，

$$
\boldsymbol {\Lambda} _ {k} = \frac {1}{4} E \left(\left\{\sum_ {i = 1} ^ {m} \boldsymbol {e} _ {i} \operatorname{Tr} \left[ (\boldsymbol {\nabla} \boldsymbol {\nabla} ^ {\mathrm{T}} h _ {i}) \right. \mid_ {\hat {\boldsymbol {X}} _ {k / k - 1}} \left(\boldsymbol {P} _ {k / k - 1} - \tilde {\boldsymbol {X}} _ {k / k - 1} \left(\tilde {\boldsymbol {X}} _ {k / k - 1}\right) ^ {\mathrm{T}}\right) \right] \right\}
$$

$$
\left. \right.\left\{ \right.\sum_ {j = 1} ^ {m} \boldsymbol {e} _ {j} \operatorname{Tr} \left[ (\boldsymbol {\nabla} \boldsymbol {\nabla} ^ {\mathrm{T}} h _ {j}) \right.\left. \right| _ {\hat {\boldsymbol {X}} _ {k / k - 1}} \left(\boldsymbol {P} _ {k / k - 1} - \tilde {\boldsymbol {X}} _ {k / k - 1} \left(\tilde {\boldsymbol {X}} _ {k / k - 1}\right) ^ {\mathrm{T}}\right)\left. \right]\left. \right\} ^ {\mathrm{T}}
$$

该式说明 $\Lambda_{k}$ 为 $m \times m$ 的方阵，其中第 i 行第 j 列的元 $(i=1,2,\cdots,m; j=1,2,\cdots,m)$ 为

$$
\begin{array}{r l} \Lambda_ {k} (i, j) = & \frac {1}{4} E \left\{\mathrm{Tr} \left[ (\nabla \nabla^ {\mathrm{T}} h _ {i}) \Big | _ {\hat {\mathbf {X}} _ {k / k - 1}} (\mathbf {P} _ {k / k - 1} - \hat {\mathbf {X}} _ {k / k - 1} (\hat {\mathbf {X}} _ {k / k - 1}) ^ {\mathrm{T}}) \right] \cdot \right. \\ & \left. \mathrm{Tr} \left[ (\nabla \nabla^ {\mathrm{T}} h _ {j}) \Big | _ {\hat {\mathbf {X}} _ {k / k - 1}} (\mathbf {P} _ {k / k - 1} - \hat {\mathbf {X}} _ {k / k - 1} (\hat {\mathbf {X}} _ {k / k - 1}) ^ {\mathrm{T}}) \right] \right\} \end{array}
$$

为简化书写，记

$$
\begin{array}{l} \boldsymbol {D} _ {k, i} = (\nabla \nabla^ {\mathrm{T}} h _ {i}) \Big | _ {\boldsymbol {x} _ {k} = \hat {\boldsymbol {x}} _ {k / k - 1}} \\ \boldsymbol {D} _ {k, j} = (\nabla \nabla^ {\mathrm{T}} h _ {j}) \Big | _ {\boldsymbol {x} _ {k} = \hat {\boldsymbol {x}} _ {k / k - 1}} \end{array}
$$

则

$$
\begin{array}{r l} \Lambda_ {k} (i, j) = & \frac {1}{4} E \{\left[ \mathrm{Tr} (\boldsymbol {D} _ {k, i} \boldsymbol {P} _ {k / k - 1}) - \mathrm{Tr} (\boldsymbol {D} _ {k, i} \widetilde {\boldsymbol {X}} _ {k / k - 1} (\widetilde {\boldsymbol {X}} _ {k / k - 1}) ^ {\mathrm{T}}) \right] \cdot \\ & \left[ \mathrm{Tr} (\boldsymbol {D} _ {k, j} \boldsymbol {P} _ {k / k - 1}) - \mathrm{Tr} (\boldsymbol {D} _ {k, j} \widetilde {\boldsymbol {X}} _ {k / k - 1} (\widetilde {\boldsymbol {X}} _ {k / k - 1}) ^ {\mathrm{T}}) \right] \} = \\ & \frac {1}{4} E \{\mathrm{Tr} [ \boldsymbol {D} _ {k, i} \widetilde {\boldsymbol {X}} _ {k / k - 1} (\widetilde {\boldsymbol {X}} _ {k / k - 1}) ^ {\mathrm{T}} ] \cdot \mathrm{Tr} [ \boldsymbol {D} _ {k, j} \widetilde {\boldsymbol {X}} _ {k / k - 1} (\widetilde {\boldsymbol {X}} _ {k / k - 1}) ^ {\mathrm{T}} ] \} - \\ & \frac {1}{4} \mathrm{Tr} (\boldsymbol {D} _ {k, i} \boldsymbol {P} _ {k / k - 1}) \cdot \mathrm{Tr} (\boldsymbol {D} _ {k, j} \boldsymbol {P} _ {k / k - 1}) \end{array}
$$

由于 $\hat{X}_{k/k-1}$ 为时间更新所得估计值， $\tilde{X}_{k/k-1}$ 为零均值的高斯白噪声。根据文献[139]所列结论：

$$
\begin{array}{l} E \left\{\operatorname{Tr} \left[ \boldsymbol {D} _ {k, i} \widetilde {\boldsymbol {X}} _ {k / k - 1} (\widetilde {\boldsymbol {X}} _ {k / k - 1}) ^ {\mathrm{T}} \right] \operatorname{Tr} \left[ \boldsymbol {D} _ {k, j} \widetilde {\boldsymbol {X}} _ {k / k - 1} (\widetilde {\boldsymbol {X}} _ {k / k - 1}) ^ {\mathrm{T}} \right] \right\} = \\ 2 \operatorname{Tr} \left(\boldsymbol {D} _ {k, i} \boldsymbol {P} _ {k / k - 1} \boldsymbol {D} _ {k, j} \boldsymbol {P} _ {k / k - 1}\right) + \operatorname{Tr} \left(\boldsymbol {D} _ {k, i} \boldsymbol {P} _ {k / k - 1}\right) \operatorname{Tr} \left(\boldsymbol {D} _ {k, j} \boldsymbol {P} _ {k / k - 1}\right) \end{array}
$$

式中， $P_{k / k - 1} = E[\widetilde{\mathbf{X}}_{k / k - 1}(\widetilde{\mathbf{X}}_{k / k - 1})^{\mathrm{T}}]$ 。所以

$$
\Lambda_ {k} (i, j) = \frac {1}{2} \operatorname{Tr} \left[ (\boldsymbol {\nabla} \boldsymbol {\nabla} ^ {\mathrm{T}} h _ {i}) \right. \left| _ {\boldsymbol {x} _ {k} = \dot {\boldsymbol {x}} _ {k / k - 1}} \boldsymbol {P} _ {k / k - 1} (\boldsymbol {\nabla} \boldsymbol {\nabla} ^ {\mathrm{T}} h _ {j}) \right. \left| _ {\boldsymbol {x} _ {k} = \dot {\boldsymbol {x}} _ {k / k - 1}} \boldsymbol {P} _ {k / k - 1} \right]\tag{6.2.115}
$$

根据上述分析,可将二阶广义卡尔曼滤波算法整理如下。

对于具有如下形式的系统方程和量测方程：

$$
\begin{array}{r l} \dot {\boldsymbol {X}} (t) & = f [ \boldsymbol {X} (t), t ] + \boldsymbol {G} (t) w (t) \\ \boldsymbol {Z} _ {k} & = h [ \boldsymbol {X} _ {k}, t _ {k} ] + \boldsymbol {V} _ {k} \end{array}
$$

式中 $w(t)$ 和 $V_{k}$ 均为零均值的高斯白噪声，且该两类噪声不相关，方差强度阵和方差阵分别为 q 和 $R_{k}$ 。则对状态变量作最优估计的二阶 EKF 滤波方程为： $(k=1,2,3,\cdots)$

$$
\begin{array}{l} \dot {\mathbf {X}} ^ {-} (t) = \boldsymbol {f} [ \hat {\mathbf {X}} ^ {-} (t), t ] + \frac {1}{2} \sum_ {i = 1} ^ {n} \boldsymbol {e} _ {i} \operatorname{Tr} \left[ (\boldsymbol {\nabla} \boldsymbol {\nabla} ^ {\mathrm{T}} f _ {i}) \Big | _ {\boldsymbol {x} _ {k} = \hat {\boldsymbol {X}} ^ {-} (t)} \boldsymbol {P} ^ {-} (t) \right] \\ \dot {\boldsymbol {P}} ^ {-} (t) = \boldsymbol {F} (t) \boldsymbol {P} ^ {-} (t) + \boldsymbol {P} ^ {-} (t) \boldsymbol {F} (t) + \boldsymbol {G} (t) \boldsymbol {q} \boldsymbol {G} ^ {\mathrm{T}} (t) \\ \hat {\mathbf {X}} _ {k} = \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {K} _ {k} [ \mathbf {Z} _ {k} - \boldsymbol {h} (\hat {\mathbf {X}} _ {k / k - 1}, t _ {k}) ] - \mathbf {C} _ {k} \\ \mathbf {C} _ {k} = \frac {1}{2} \mathbf {K} _ {k} \sum_ {j = 1} ^ {m} \boldsymbol {e} _ {j} \operatorname{Tr} \left[ (\boldsymbol {\nabla} \boldsymbol {\nabla} ^ {\mathrm{T}} h _ {j}) \Big | _ {\boldsymbol {x} _ {k} = \hat {\boldsymbol {x}} _ {k / k - 1}} \boldsymbol {P} _ {k / k - 1} \right] \\ \mathbf {K} _ {k} = \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k} + \boldsymbol {\Lambda} _ {k}) ^ {- 1} \\ \Lambda_ {k} (i, j) = \frac {1}{2} \operatorname{Tr} \left[ (\boldsymbol {\nabla} \boldsymbol {\nabla} ^ {\mathrm{T}} h _ {i}) \Big | _ {\boldsymbol {x} _ {k} = \hat {\boldsymbol {x}} _ {k / k - 1}} \boldsymbol {P} _ {k / k - 1} (\boldsymbol {\nabla} \boldsymbol {\nabla} ^ {\mathrm{T}} h _ {j}) \Big | _ {\boldsymbol {x} _ {k} = \hat {\boldsymbol {x}} _ {k / k - 1}} \boldsymbol {P} _ {k / k - 1} \right] \\ i = 1, 2, \dots , m; i = 1, 2, \dots , m. \end{array}
$$

$$
\boldsymbol {P} _ {k} = \boldsymbol {P} _ {k / k - 1} - \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \left(\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k} + \boldsymbol {\Lambda} _ {k}\right) ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1}
$$

其中

$$
\boldsymbol {F} (t) = \frac {\partial \boldsymbol {f}}{\partial \boldsymbol {X} ^ {\mathrm{T}} (t)} \Bigg | _ {\boldsymbol {X} (t) = \hat {\boldsymbol {X}} ^ {-} (t)}
$$

微分方程采用龙格－库塔算法联立求解，在 $\left[t_{k-1},t_{k}\right]$ 滤波周期内，积分初值取 $\hat{\mathbf{X}}^{-}(t_{k-1})=\hat{\mathbf{X}}_{k-1},\mathbf{P}^{-}(t_{k-1})=\mathbf{P}_{k-1}$ ，积分终值用于量测更新：

$$
\begin{array}{l} \hat {\mathbf {X}} _ {k / k - 1} = \hat {\mathbf {X}} ^ {-} (t) \Big | _ {t = t _ {k}} \\ \mathbf {P} _ {k / k - 1} = \mathbf {P} ^ {-} (t) \Big | _ {t = t _ {k}} \\ \mathbf {H} _ {k} = \frac {\partial \mathbf {h}}{\partial \mathbf {X} ^ {\mathrm{T}} (t)} \Big | _ {\mathbf {X} (t) = \hat {\mathbf {X}} _ {k / k - 1}} \\ \nabla \nabla^ {\mathrm{T}} f _ {i} = \left[ \begin{array}{c c c c} \frac {\partial^ {2} f _ {i}}{\partial X _ {1} ^ {2}} & \frac {\partial^ {2} f _ {i}}{\partial X _ {1} \partial X _ {2}} & \dots & \frac {\partial^ {2} f _ {i}}{\partial X _ {1} \partial X _ {n}} \\ \frac {\partial^ {2} f _ {i}}{\partial X _ {2} \partial X _ {1}} & \frac {\partial^ {2} f _ {i}}{\partial X _ {2} ^ {2}} & \dots & \frac {\partial^ {2} f _ {i}}{\partial X _ {2} \partial X _ {n}} \\ \vdots & \vdots & & \vdots \\ \frac {\partial^ {2} f _ {i}}{\partial X _ {n} \partial X _ {1}} & \frac {\partial^ {2} f _ {i}}{\partial X _ {n} \partial X _ {2}} & \dots & \frac {\partial^ {2} f _ {i}}{\partial X _ {n} ^ {2}} \end{array} \right] \\ \nabla \nabla^ {\mathrm{T}} h _ {j} = \left[ \begin{array}{c c c c} \frac {\partial^ {2} h _ {j}}{\partial X _ {1} ^ {2}} & \frac {\partial^ {2} h _ {j}}{\partial X _ {1} \partial X _ {2}} & \dots & \frac {\partial^ {2} h _ {j}}{\partial X _ {1} \partial X _ {n}} \\ \frac {\partial^ {2} h _ {j}}{\partial X _ {2} \partial X _ {1}} & \frac {\partial^ {2} h _ {j}}{\partial X _ {2} ^ {2}} & \dots & \frac {\partial^ {2} h _ {j}}{\partial X _ {2} \partial X _ {n}} \\ \vdots & \vdots & & \vdots \\ \frac {\partial^ {2} h _ {j}}{\partial X _ {n} \partial X _ {1}} & \frac {\partial^ {2} h _ {j}}{\partial X _ {n} \partial X _ {2}} & \dots & \frac {\partial^ {2} h _ {j}}{\partial X _ {n} ^ {2}} \end{array} \right] \end{array}
$$

$f_{i}$ 和 $h_{j}$ 分别为 f 和 h 的第 i 个和第 j 个分量； $e_{i}$ 和 $e_{j}$ 分别是第 i 个分量和第 j 个分量为 1 而其余分量均为 0 的单位向量。 $\Lambda_{k}(i,j)$ 为 $\Lambda_{k}$ 的第 i 行第 j 列的元 $(i,j=1,2\cdots,m)$ 。

例6-2 用雷达测距系统确定空气中自由落体距地面的高度，如图6.2.4所示（不考虑风的影响）。图中， $B$ 为自由落体， $S$ 为雷达架设点， $M$ 和 $a$ 分别为 $S$ 点距落点的水平距离和垂直距离。测距误差视为零均值白噪声，方差为 $R$ 。

(1) 列写出系统方程和量测方程；

(2) 计算出系统方程和量测方程的雅可比矩阵以及二阶偏导数矩阵；

![](images/9c95e809fd2d41e8c2242e263c5e76ab2be6395a5e877dfe8f0bfdc99d63f2d7.jpg)

(3) 列写出二阶 EKF 方程, 用于根据雷达测量的距离精确确定出自由落体的高度、速度及弹道系数。

图 6.2.4 雷达测距系统

解 （1）记状态变量 $X_{1}$ 为高度， $X_{2}$ 为速度， $X_{3}$ 为弹道系数，则 $\dot{X}_{1} = -X_{2}$

$$
\dot {X} _ {2} = \rho_ {0} \mathrm{e} ^ {- \frac {X _ {1}}{K}} \cdot \frac {X _ {2} ^ {2} X _ {3}}{2} - g + w _ {1}
$$

$$
\begin{array}{l} \dot {X} _ {3} = w _ {2} \\ Z _ {k} = \sqrt {M ^ {2} + [ X _ {1} (t _ {k}) - a ] ^ {2}} + V _ {k} \end{array}
$$

即

$$
\left[ \begin{array}{l} \dot {X} _ {1} \\ \dot {X} _ {2} \\ \dot {X} _ {3} \end{array} \right] = \left[ \begin{array}{c} - X _ {2} \\ \rho_ {0} \mathrm{e} ^ {- \frac {X _ {1}}{K}} \cdot \frac {X _ {2} ^ {2} X _ {3}}{2} \\ 0 \end{array} \right] - \left[ \begin{array}{l} 0 \\ g \\ 0 \end{array} \right] + \left[ \begin{array}{l} 0 \\ w _ {1} \\ w _ {2} \end{array} \right]
$$

所以

$$
\begin{array}{r l} & f = \left[ \begin{array}{c} - X _ {2} \\ \rho_ {0} \mathrm{e} ^ {- \frac {X _ {1}}{K}} \cdot \frac {X _ {2} ^ {2} X _ {3}}{2} - g \\ 0 \end{array} \right] \\ & h = \sqrt {M ^ {2} + [ X _ {1} (t _ {k}) - a ] ^ {2}} \end{array}
$$

式中，g 为重力加速度，K 为空气密度随高度变化的常数。

(2) 雅可比矩阵为

$$
\mathbf {F} = \frac {\partial \mathbf {f}}{\partial \mathbf {X} ^ {\mathrm{T}}} = \left[ \begin{array}{c c c} 0 & - 1 & 0 \\ F _ {2 1} & F _ {2 2} & F _ {2 3} \\ 0 & 0 & 0 \end{array} \right]
$$

式中

$$
F _ {2 1} = - \rho_ {0} \mathrm{e} ^ {- \frac {X _ {1}}{K}} \cdot \frac {X _ {2} ^ {2} X _ {3}}{2 K}
$$

$$
F _ {2 2} = \rho_ {0} \mathrm{e} ^ {- \frac {X _ {1}}{\kappa}} \cdot X _ {2} X _ {3}
$$

$$
F _ {2 3} = \rho_ {0} \mathrm{e} ^ {- \frac {X _ {1}}{K}} \cdot \frac {X _ {2} ^ {2}}{2}
$$

$$
\mathbf {H} = \frac {\partial h}{\partial \mathbf {X} ^ {\mathrm{T}}} = \left[ (X _ {1} - a) [ M ^ {2} + (X _ {1} - a) ^ {2} ] ^ {- \frac {1}{2}} 0 0 \right]
$$

$$
\nabla \nabla^ {\mathrm{T}} h = \left[ \begin{array}{c c c} \frac {\partial^ {2} h}{\partial X _ {1} ^ {2}} & \frac {\partial^ {2} h}{\partial X _ {1} \partial X _ {2}} & \frac {\partial^ {2} h}{\partial X _ {1} \partial X _ {3}} \\ \frac {\partial^ {2} h}{\partial X _ {2} \partial X _ {1}} & \frac {\partial^ {2} h}{\partial X _ {2} ^ {2}} & \frac {\partial^ {2} h}{\partial X _ {2} \partial X _ {3}} \\ \frac {\partial^ {2} h}{\partial X _ {3} \partial X _ {1}} & \frac {\partial^ {2} h}{\partial X _ {3} \partial X _ {2}} & \frac {\partial^ {2} h}{\partial X _ {3} ^ {2}} \end{array} \right] = \left[ \begin{array}{c c c} \frac {\partial^ {2} h}{\partial X _ {1} ^ {2}} & 0 & 0 \\ 0 & 0 & 0 \\ 0 & 0 & 0 \end{array} \right]
$$

式中

$$
\begin{array}{r l} & \frac {\partial h}{\partial X _ {1}} = \frac {1}{2} \frac {2 (X _ {1} - a)}{\sqrt {M ^ {2} + (X _ {1} - a) ^ {2}}} = \frac {X _ {1} - a}{\sqrt {M ^ {2} + (X _ {1} - a) ^ {2}}} \\ & \frac {\partial^ {2} h}{\partial X _ {1} ^ {2}} = \frac {\partial}{\partial X _ {1}} \left(\frac {\partial h}{\partial X _ {1}}\right) = \\ & \frac {\sqrt {M ^ {2} + (X _ {1} - a) ^ {2}} - (X _ {1} - a) \cdot \frac {1}{2} [ M ^ {2} + (X _ {1} - a) ^ {2} ] ^ {- \frac {1}{2}} \cdot 2 (X _ {1} - a)}{M ^ {2} + (X _ {1} - a) ^ {2}} = \end{array}
$$

$$
\begin{array}{r l} & {\left[ M ^ {2} + (X _ {1} - a) ^ {2} \right] ^ {- \frac {1}{2}} - (X _ {1} - a) ^ {2} \left[ M ^ {2} + (X _ {1} - a) ^ {2} \right] ^ {- \frac {3}{2}}} \\ & {\pmb {\nabla} \pmb {\nabla} ^ {\mathrm{T}} f _ {1} = \left[ \begin{array}{c c c} \frac {\partial^ {2} f _ {1}}{\partial X _ {1} ^ {2}} & \frac {\partial^ {2} f _ {1}}{\partial X _ {1} \partial X _ {2}} & \frac {\partial^ {2} f _ {1}}{\partial X _ {1} \partial X _ {3}} \\ \frac {\partial^ {2} f _ {1}}{\partial X _ {2} \partial X _ {1}} & \frac {\partial^ {2} f _ {1}}{\partial X _ {2} ^ {2}} & \frac {\partial^ {2} f _ {1}}{\partial X _ {2} \partial X _ {3}} \\ \frac {\partial^ {2} f _ {1}}{\partial X _ {3} \partial X _ {1}} & \frac {\partial^ {2} f _ {1}}{\partial X _ {3} \partial X _ {2}} & \frac {\partial^ {2} f _ {1}}{\partial X _ {3} ^ {2}} \end{array} \right] = \left[ \begin{array}{c c c} 0 & 0 & 0 \\ 0 & 0 & 0 \\ 0 & 0 & 0 \end{array} \right]} \\ & {\pmb {\nabla} \pmb {\nabla} ^ {\mathrm{T}} f _ {3} = \pmb {\nabla} \pmb {\nabla} ^ {\mathrm{T}} 0 = \left[ \begin{array}{c c c} 0 & 0 & 0 \\ 0 & 0 & 0 \\ 0 & 0 & 0 \end{array} \right]} \\ & {\pmb {\nabla} \pmb {\nabla} ^ {\mathrm{T}} f _ {2} = \left[ \begin{array}{c c c} \frac {\partial^ {2} f _ {2}}{\partial X _ {1} ^ {2}} & \frac {\partial^ {2} f _ {2}}{\partial X _ {1} \partial X _ {2}} & \frac {\partial^ {2} f _ {2}}{\partial X _ {1} \partial X _ {3}} \\ \frac {\partial^ {2} f _ {2}}{\partial X _ {2} \partial X _ {1}} & \frac {\partial^ {2} f _ {2}}{\partial X _ {2} ^ {2}} & \frac {\partial^ {2} f _ {2}}{\partial X _ {2} \partial X _ {3}} \\ \frac {\partial^ {2} f _ {2}}{\partial X _ {3} \partial X _ {1}} & \frac {\partial^ {2} f _ {2}}{\partial X _ {3} \partial X _ {2}} & \frac {\partial^ {2} f _ {2}}{\partial X _ {3} ^ {2}} \end{array} \right]} \end{array}
$$

由于 $F_{21} = \frac{\partial f_2}{\partial X_1}, F_{22} = \frac{\partial f_2}{\partial X_2}, F_{23} = \frac{\partial f_2}{\partial X_3}$

所以 $\frac{\partial^2f_2}{\partial X_1^2} = \frac{\partial F_{21}}{\partial X_1} = \rho_0\mathrm{e}^{-\frac{X_1}{K}}\cdot \frac{X_2^2X_3}{2K^2}$

$$
\begin{array}{l} \frac {\partial^ {2} f _ {2}}{\partial X _ {1} \partial X _ {2}} = \frac {\partial F _ {2 1}}{\partial X _ {2}} = - \rho_ {0} \mathrm{e} ^ {- \frac {X _ {1}}{K}} \cdot \frac {X _ {2} X _ {3}}{K} \\ \frac {\partial^ {2} f _ {2}}{\partial X _ {1} \partial X _ {3}} = \frac {\partial F _ {2 1}}{\partial X _ {3}} = - \rho_ {0} \mathrm{e} ^ {- \frac {X _ {1}}{K}} \cdot \frac {X _ {2} ^ {2}}{2 K} \\ \frac {\partial^ {2} f _ {2}}{\partial X _ {2} \partial X _ {1}} = \frac {\partial^ {2} f _ {2}}{\partial X _ {1} \partial X _ {2}} = - \rho_ {0} \mathrm{e} ^ {- \frac {X _ {1}}{K}} \cdot \frac {X _ {2} X _ {3}}{K} \\ \frac {\partial^ {2} f _ {2}}{\partial X _ {2} ^ {2}} = \frac {\partial F _ {2 2}}{\partial X _ {2}} = \rho_ {0} \mathrm{e} ^ {- \frac {X _ {1}}{K}} \cdot X _ {3} \\ \frac {\partial^ {2} f _ {2}}{\partial X _ {2} \partial X _ {3}} = \frac {\partial F _ {2 2}}{\partial X _ {3}} = \rho_ {0} \mathrm{e} ^ {- \frac {X _ {1}}{K}} \cdot X _ {2} \\ \frac {\partial^ {2} f _ {2}}{\partial X _ {3} \partial X _ {1}} = \frac {\partial^ {2} f _ {2}}{\partial X _ {1} \partial X _ {3}} = - \rho_ {0} \mathrm{e} ^ {- \frac {X _ {1}}{K}} \cdot \frac {X _ {2} ^ {2}}{2 K} \\ \frac {\partial^ {2} f _ {2}}{\partial X _ {3} \partial X _ {2}} = \frac {\partial^ {2} f _ {2}}{\partial X _ {2} \partial X _ {3}} = \rho_ {0} \mathrm{e} ^ {- \frac {X _ {1}}{K}} \cdot X _ {2} \\ \frac {\partial^ {2} f _ {2}}{\partial X _ {3} ^ {2}} = \frac {\partial^ {2} F _ {2 3}}{\partial X _ {3}} = 0 \end{array}
$$

所以

$$
\boldsymbol {\nabla} \boldsymbol {\nabla} ^ {\mathrm{T}} f _ {2} = \rho_ {0} \mathrm{e} ^ {- \frac {X _ {1}}{K}} \left[ \begin{array}{c c c} \frac {X _ {2} ^ {2} X _ {3}}{2 K ^ {2}} & - \frac {X _ {2} X _ {3}}{K} & - \frac {X _ {2} ^ {2}}{2 K} \\ - \frac {X _ {2} X _ {3}}{K} & X _ {3} & X _ {2} \\ - \frac {X _ {2} ^ {2}}{2 K} & X _ {2} & 0 \end{array} \right]
$$

(3) 二阶 EKF

$$
\text {记}   \pmb {X} (t) = \left[ \begin{array}{l l l} X _ {1} (t) & X _ {2} (t) & X _ {3} (t) \end{array} \right] ^ {\mathrm{T}}, \pmb {f} = \left[ \begin{array}{l l l} f _ {1} & f _ {2} & f _ {3} \end{array} \right] ^ {\mathrm{T}}
$$

则二阶 EKF 为

$$
\dot {\hat {\mathbf {X}}} ^ {-} (t) = \boldsymbol {f} [ \hat {\mathbf {X}} ^ {-} (t), t ] + \frac {1}{2} \mathrm{Tr} [ (\nabla \nabla^ {\mathrm{T}} f _ {2}) \boldsymbol {P} ^ {-} (t) ] \boldsymbol {e} _ {2}
$$

$$
\dot {\boldsymbol {P}} ^ {-} (t) = \boldsymbol {F} (t) \boldsymbol {P} ^ {-} (t) + \boldsymbol {P} ^ {-} (t) \boldsymbol {F} ^ {\mathrm{T}} (t) + \boldsymbol {q}
$$

$$
\hat {\mathbf {X}} _ {k} = \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {K} _ {k} \left[ Z _ {k} - h (\hat {\mathbf {X}} _ {k / k - 1}) \right] - \mathbf {C} _ {k}
$$

$$
\boldsymbol {C} _ {k} = \frac {1}{2} \boldsymbol {K} _ {k} \operatorname{Tr} \left[ (\boldsymbol {\nabla} \boldsymbol {\nabla} ^ {\mathrm{T}} h) \Big | _ {\hat {\mathbf {x}} _ {k / k - 1}} \boldsymbol {P} _ {k / k - 1} \right]
$$

$$
\mathbf {K} _ {k} = \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} (\mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} + R + \Lambda_ {k}) ^ {- 1}
$$

$$
\Lambda_ {k} = \frac {1}{2} \operatorname{Tr} \left[ (\boldsymbol {\nabla} \boldsymbol {\nabla} ^ {\mathrm{T}} h) \left| _ {\hat {\mathbf {x}} _ {k / k - 1}} \boldsymbol {P} _ {k / k - 1} (\boldsymbol {\nabla} \boldsymbol {\nabla} ^ {\mathrm{T}} h) \right| _ {\hat {\mathbf {x}} _ {k / k - 1}} \boldsymbol {P} _ {k / k - 1} \right]
$$

$$
\boldsymbol {P} _ {k} = \boldsymbol {P} _ {k / k - 1} - \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \left(\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + R + \Lambda_ {k}\right) ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1}
$$

其中 $e_{2}=\begin{bmatrix}0&1&0\end{bmatrix}^{T}$

$$
\boldsymbol {F} (t) = \frac {\partial \boldsymbol {f}}{\partial \boldsymbol {X} ^ {\mathrm{T}}} \Big | _ {\dot {\boldsymbol {x}} _ {(t)} ^ {-}} = \left[ \begin{array}{c c c} 0 & - 1 & 0 \\ F _ {2 1} & F _ {2 2} & F _ {2 3} \\ 0 & 0 & 0 \end{array} \right]
$$

$$
\boldsymbol {H} _ {k} = \frac {\partial h}{\partial \boldsymbol {X} ^ {\mathrm{T}}} \Big | _ {\hat {\boldsymbol {x}} _ {k / k - 1}} = \left[ (\hat {X} _ {1 k / k - 1} - a) \left[ M ^ {2} + (\hat {X} _ {1 k / k - 1} - a) ^ {2} \right] ^ {- \frac {1}{2}} \quad 0 \quad 0 \right]
$$

$$
\hat {\boldsymbol {X}} _ {k / k - 1} = \hat {\boldsymbol {X}} ^ {-} (t) \Big | _ {t _ {k}}
$$

$$
\boldsymbol {P} _ {k / k - 1} = \boldsymbol {P} ^ {-} (t) \Big | _ {t _ {k}}
$$

$$
F _ {2 1} = - \rho_ {0} \mathrm{e} ^ {- \frac {\hat {X} _ {1} ^ {-} (t)}{K}} \cdot \frac {(\hat {X} _ {2} ^ {-} (t)) ^ {2} \hat {X} _ {3} ^ {-} (t)}{2 K}
$$

$$
F _ {2 2} = \rho_ {0} \mathrm{e} ^ {- \frac {\hat {X} _ {1} ^ {-} (t)}{K}} \cdot \hat {X} _ {2} ^ {-} (t) \hat {X} _ {3} ^ {-} (t)
$$

$$
F _ {2 3} = \rho_ {0} \mathrm{e} ^ {- \frac {\hat {X} _ {1} ^ {-} (t)}{K}} \cdot \frac {\hat {X} _ {2} ^ {-} (t)}{2}
$$

$$
\left. \left(\boldsymbol {\nabla} \boldsymbol {\nabla} ^ {\mathrm{T}} f _ {2}\right) \right| _ {\hat {\boldsymbol {X}} ^ {-} (t)} = \rho_ {0} \mathrm{e} ^ {- \frac {X _ {1} ^ {-} (t)}{K}} \left[ \begin{array}{c c c} \frac {\left(\hat {X} _ {2} ^ {-} (t)\right) ^ {2} \hat {X} _ {3} ^ {-} (t)}{2 K ^ {2}} & - \frac {\hat {X} _ {2} ^ {-} (t) \hat {X} _ {3} ^ {-} (t)}{K} & - \frac {\left(\hat {X} _ {2} ^ {-} (t)\right) ^ {2}}{2 K} \\ - \frac {\hat {X} _ {2} ^ {-} (t) \hat {X} _ {3} ^ {-} (t)}{K} & \hat {X} _ {3} ^ {-} (t) & \hat {X} _ {2} ^ {-} (t) \\ - \frac {\left(X _ {2} ^ {-} (t)\right) ^ {2}}{2 K} & \hat {X} _ {2} ^ {-} (t) & 0 \end{array} \right]
$$

$$
\left(\boldsymbol {\nabla} \boldsymbol {\nabla} ^ {\mathrm{T}} h\right) \Big | _ {\hat {\mathbf {x}} _ {k / k - 1}} = \left[ \begin{array}{c c c} \frac {\partial^ {2} h}{\partial X _ {1} ^ {2}} & 0 & 0 \\ 0 & 0 & 0 \\ 0 & 0 & 0 \end{array} \right]
$$

$$
\left. \frac {\partial^ {2} h}{\partial X _ {1} ^ {2}} \right| _ {\dot {X} _ {k / k - 1}} = [ M ^ {2} + (X _ {1 k / k - 1} - a) ^ {2} ] ^ {- \frac {1}{2}} - (X _ {1 k / k - 1} - a) ^ {2} [ M ^ {2} + (X _ {1 k / k - 1} - a) ^ {2} ] ^ {- \frac {3}{2}}
$$

## 6.3 非线性系统滤波之二: UKF

卡尔曼滤波是一种基于模型的线性最小方差估计，其标准离散型算法具有如下独特优点：递推计算；使用计算机执行；适用于平稳或非平稳多维随机信号的估计。所以在随机信号处理中，特别是在组合导航设计中得到了广泛应用。但是标准卡尔曼滤波只适用于系统方程和量测方程均为线性的情况。尽管广义卡尔曼滤波 EKF(Extended Kalman Filter) 可解决系统和量测为非线性时的估计，但必须对原系统和量测作泰勒级数展开且仅保留线性项，再用标准卡尔曼滤波算法对线性化后的系统方程和量测方程作处理，所以 EKF 本质上仍然是标准卡尔曼滤波。由于线性化过程中舍弃了二阶以上的高阶项，所以 EKF 只适用于弱非线性对象的估计，被估计对象的非线性越强，引起的估计误差就越大，甚至会引起滤波发散。为了解决强非线性条件下的估计问题，1995 年，S. J. Julier 和 J. K. Uhlmann 提出了 UKF(Unscented Kalman Filter) 算法，用于解决强非线性对象的滤波问题，并由 E. A. Wan 和 R. Vander Merwe 进一步完善。[119,121]

UKF 和标准卡尔曼滤波都属于线性最小方差估计,算法都基于模型,区别在于最佳增益阵的求取方法上。标准卡尔曼滤波确定最佳增益阵时,使用了量测量的先验信息和一步预测均方误差阵,并基于系统和量测均为线性的假设。而 UKF 则根据被估计量和量测量的协方差阵来确定最佳增益阵,协方差阵又根据复现的一倍 $\sigma$ 样本点来计算,这些样本点则根据系统方程和量测方程来确定。所以在计算最佳增益阵的过程中,UKF 并未对系统方程和量测方程提出任何附加条件,算法既适用于线性对象,也适用于非线性对象。但是,UKF 是线性最小方差估计的一种近似形式,而标准卡尔曼滤波是精确的线性最小方差估计。所以只有在非线性条件下,UKF 才能充分显现出其优越性,非线性越强,优越性就越明显,而在线性条件下,标准卡尔曼滤波比 UKF 优越。

## 6.3.1 线性最小方差估计及其近似形式

设 X 为被估计随机向量, 对 X 的量测为随机向量 Z, 则根据式(2.1.64) 和式(2.1.65), X 基于 Z 的线性最小方差估计为

$$
\hat {\boldsymbol {X}} _ {M L} = \boldsymbol {E} \boldsymbol {X} + \boldsymbol {C} _ {X Z} \boldsymbol {C} _ {Z Z} ^ {- 1} (\boldsymbol {Z} - \boldsymbol {E} \boldsymbol {Z})
$$

估计的均方误差阵为

$$
\boldsymbol {P} = \boldsymbol {C} _ {x x} - \boldsymbol {C} _ {x z} \boldsymbol {C} _ {z z} ^ {- 1} \boldsymbol {C} _ {z x}
$$

上述两式适用于 X 和 Z 服从任意分布。当 X 和 Z 都服从正态分布时，所得的估计既是线性估计中的最高精度者，又是所有估计中的最高精度者，即也是最小方差估计。

设系统方程和量测方程具有离散形式

$$
\boldsymbol {X} _ {k} = \boldsymbol {f} (\boldsymbol {X} _ {k - 1}, \boldsymbol {u} _ {k - 1}) + \boldsymbol {W} _ {k - 1}\tag{6.3.1}
$$

$$
\mathbf {Z} _ {k} = \mathbf {h} (\mathbf {X} _ {k}) + \mathbf {V} _ {k}\tag{6.3.2}
$$

式中，f 和 h 为非线性向量函数， $W_{k}$ 和 $V_{k}$ 为不相关的零均值白噪声序列，方差阵分别为 $Q_{k}$ 和 $R_{k}, u_{k-1}$ 为确定性控制项。则 $X_{k}$ 基于 $Z_{o}^{k} = \{Z_{0}, Z_{1}, Z_{2}, \cdots, Z_{k}\}$ 的线性最小方差估计为

$$
\hat {\mathbf {X}} _ {k} = E ^ {*} \left[ \mathbf {X} _ {k} / \mathbf {Z} _ {o} ^ {k} \right] = E ^ {*} \left[ \mathbf {X} _ {k} / \mathbf {Z} _ {o} ^ {k - 1}, \mathbf {Z} _ {k} \right]
$$

根据线性最小方差估计的线性性质,有

$$
\hat {\mathbf {X}} _ {k} = E ^ {*} \left[ \mathbf {X} _ {k} / \mathbf {Z} _ {o} ^ {k - 1} \right] + E ^ {*} \left[ \mathbf {X} _ {k} / \mathbf {Z} _ {k} \right] - E \mathbf {X} _ {k} = \hat {\mathbf {X}} _ {k / k - 1} + E ^ {*} \left[ \mathbf {X} _ {k} / \mathbf {Z} _ {k} \right] - E \mathbf {X} _ {k}\tag{6.3.3}
$$

记 $X = X_{k}, Z = Z_{k}$ , 根据式(2.1.64)

$$
\begin{array}{r l} E ^ {*} \left[ \mathbf {X} _ {k} / \mathbf {Z} _ {k} \right] & = E ^ {*} \left[ \mathbf {X} / \mathbf {Z} \right] = E \mathbf {X} + \mathbf {C} _ {\mathrm{XZ}} \mathbf {C} _ {\mathrm{ZZ}} ^ {- 1} (\mathbf {Z} - E \mathbf {Z}) = \\ & \quad E \mathbf {X} _ {k} + E \big [ (\mathbf {X} _ {k} - E \mathbf {X} _ {k}) (\mathbf {Z} _ {k} - E \mathbf {Z} _ {k}) ^ {\mathrm{T}} \big ] \{E \big [ (\mathbf {Z} _ {k} - E \mathbf {Z} _ {k}) (\mathbf {Z} _ {k} - E \mathbf {Z} _ {k}) ^ {\mathrm{T}} \big ] \} ^ {- 1} (\mathbf {Z} _ {k} - E \mathbf {Z} _ {k}) \approx \\ & \quad E \mathbf {X} _ {k} + E \big [ (\mathbf {X} _ {k} - \hat {\mathbf {X}} _ {k / k - 1}) (\mathbf {Z} _ {k} - \hat {\mathbf {Z}} _ {k / k - 1}) ^ {\mathrm{T}} \big ] \{E \big [ (\mathbf {Z} _ {k} - \\ & \quad \hat {\mathbf {Z}} _ {k / k - 1}) (\mathbf {Z} _ {k} - \hat {\mathbf {Z}} _ {k / k - 1}) ^ {\mathrm{T}} \big ] \} ^ {- 1} (\mathbf {Z} _ {k} - \hat {\mathbf {Z}} _ {k / k - 1}) \end{array}
$$

上式中，均值近似取为条件均值 $EX_{k}\approx \hat{X}_{k / k - 1},EZ_{k}\approx \hat{Z}_{k / k - 1}$

记

$$
\begin{array}{r l} & {\pmb {P} _ {(X Z) _ {k / k - 1}} = E \big [ (\pmb {X} _ {k} - \hat {\pmb {X}} _ {k / k - 1}) (\pmb {Z} _ {k} - \hat {\pmb {Z}} _ {k / k - 1}) ^ {\mathrm{T}} \big ]} \\ & {\pmb {P} _ {(Z Z) _ {k / k - 1}} = E \big [ (\pmb {Z} _ {k} - \hat {\pmb {Z}} _ {k / k - 1}) (\pmb {Z} _ {k} - \hat {\pmb {Z}} _ {k / k - 1}) ^ {\mathrm{T}} \big ]} \\ & {\qquad \pmb {K} _ {k} = \pmb {P} _ {(X Z) _ {k / k - 1}} \pmb {P} _ {(Z Z) _ {k / k - 1}} ^ {- 1}} \end{array}\tag{6.3.4}
$$

则

$$
E ^ {*} \left[ \mathbf {X} _ {k} / \mathbf {Z} _ {k} \right] = E \mathbf {X} _ {k} + \mathbf {K} _ {k} (\mathbf {Z} _ {k} - \hat {\mathbf {Z}} _ {k / k - 1})
$$

将上式代入式 $(6.3.3)$ ，得

$$
\hat {\mathbf {X}} _ {k} = \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {K} _ {k} (\mathbf {Z} _ {k} - \hat {\mathbf {Z}} _ {k / k - 1})\tag{6.3.5}
$$

同理，用条件均值近似均值，即 $EX_{k}\approx\hat{X}_{k/k-1},EZ_{k}\approx\hat{Z}_{k/k-1}$ ，可得式(2.1.65)的近似计算公式：

$$
\boldsymbol {P} _ {k} = \boldsymbol {P} _ {k / k - 1} - \boldsymbol {P} _ {(X Z) _ {k / k - 1}} \boldsymbol {P} _ {(Z Z) _ {k / k - 1}} ^ {- 1} \boldsymbol {P} _ {(Z X) _ {k / k - 1}}
$$

考虑到 $P_{(ZZ)_{k / k - 1}}$ 为对称阵， $P_{(ZX)_{k / k - 1}} = P_{(XZ)_{k / k - 1}}^{\mathrm{T}}$ ，并根据式（6.3.4），得

$$
\begin{array}{r l} \boldsymbol {P} _ {k} & = \boldsymbol {P} _ {k / k - 1} - \boldsymbol {P} _ {(X Z) _ {k / k - 1}} \boldsymbol {P} _ {(Z Z) _ {k / k - 1}} ^ {- 1} \boldsymbol {P} _ {(Z Z) _ {k / k - 1}} \boldsymbol {P} _ {(Z Z) _ {k / k - 1}} ^ {- 1} \boldsymbol {P} _ {(X Z) _ {k / k - 1}} ^ {\mathrm{T}} = \\ & \boldsymbol {P} _ {k / k - 1} - \boldsymbol {K} _ {k} \boldsymbol {P} _ {(Z Z) _ {k / k - 1}} \boldsymbol {K} _ {k} ^ {\mathrm{T}} \end{array}\tag{6.3.6}
$$

式(6.3.4)、式(6.3.5)、式(6.3.6)即为线性最小方差估计的一种近似形式，它们构成了UKF的核心。要确定出增益阵 $K_{k}$ ，并使算法具有递推性，还需用UT变换确定出 $P_{(XZ)_{k/k-1}}$ ， $P_{(ZZ)_{k/k-1}}$ 及一步预测值 $\hat{X}_{k/k-1}$ 。

## 6.3.2 UT变换与UKF算法

## 1. UT 变换

假设 n 维随机向量 X 经 $f(\cdot)$ 非线性变换后形成 m 维随机向量 Y，即

$$
\mathbf {Y} = \mathbf {f} (\mathbf {X})
$$

若已知 X 的均值 $\bar{X}$ 和方差阵 $P_{XX}$ ，则 Y 的均值 $\bar{Y}$ 和方差阵 $P_{YY}$ 可通过 UT 变换求取。具体步骤如下。

步骤1:根据 $\bar{X}$ 和 $P_{XX}$ 复现出 $2n+1$ 个 X 的 1 倍 $\sigma$ 样本点(简称 $\sigma$ 样本点):

$$
\begin{array}{r l} \chi^ {(i)} & = \overline {{X}} \\ \chi^ {(i)} & = \overline {{X}} + \left(\sqrt {(n + \lambda) P _ {X X}}\right) _ {(i)} \quad i = 1, 2, \dots , n \\ \chi^ {(i)} & = \overline {{X}} - \left(\sqrt {(n + \lambda) P _ {X X}}\right) _ {(i - n)} \quad i = n + 1, n + 2, \dots , 2 n \end{array}
$$

式中， $\left(\sqrt{(n+\lambda)P_{XX}}\right)_{(i)}$ 表示矩阵 $(n+\lambda)P_{XX}$ 的下三角分解平方根的第 i 列，为 n 维列向量。
步骤 2: 计算非线性变换产生的样本点:

$$
\mathbf {Y} ^ {(i)} = f [ \chi^ {(i)} ] \quad i = 0, 1, 2, \dots , 2 n
$$

步骤 3: 确定权值

$$
W _ {0} ^ {(m)} = \frac {\lambda}{n + \lambda}\tag{6.3.7a}
$$

$$
W _ {0} ^ {(c)} = \frac {\lambda}{n + \lambda} + 1 - \alpha^ {2} + \beta\tag{6.3.7b}
$$

$$
W _ {i} ^ {(m)} = W _ {i} ^ {(c)} = \frac {1}{2 (n + \lambda)} \quad i = 1, 2, \dots , 2 n\tag{6.3.7c}
$$

上述诸式中，

$$
\lambda = \alpha^ {2} (n + \kappa) - n\tag{6.3.7d}
$$

式中， $\alpha$ 是很小的正数，可取 $10^{-4} \leqslant \alpha \leqslant 1; \kappa = 3 - n; \beta$ 取值与 $X$ 的分布形式有关，对于正态分布， $\beta = 2$ 为最优值。

步骤 4: 确定映射的均值和方差阵

$$
\overline {{{{\mathbf {Y}}}}} \approx \sum_ {i = 0} ^ {2 n} W _ {i} ^ {(m)} \mathbf {Y} ^ {(i)}\tag{6.3.8}
$$

$$
\boldsymbol {P} _ {\mathrm{YY}} \approx \sum_ {i = 0} ^ {2 n} W _ {i} ^ {(c)} \left[ \boldsymbol {Y} ^ {(i)} - \overline {{\boldsymbol {Y}}} \right] \left[ \boldsymbol {Y} ^ {(i)} - \overline {{\boldsymbol {Y}}} \right] ^ {\mathrm{T}}\tag{6.3.9}
$$

## 2. UT 变换的精度分析

(1) 非线性映射的泰勒级数展开。对于非线性对象, EKF 将非线性方程作泰勒级数展开时只保留线性项, 再按线性滤波方法求解被估计量, 所以只适用于非线性较弱的情况。而 UKF 的核心算法 UT 变换所达到的精度, 相当于保留泰勒级数三阶项所达到的精度, 所以适用于强非线性对象的估计。

为叙述方便,将按非线性函数关系 $Y=f(X)$ 确定的映射称为真实映射,由 UT 变换实现 $Y=f(X)$ 的映射称为 UT 映射。

设有非线性向量函数

$$
\mathbf {Y} = \mathbf {f} (x _ {1}, x _ {2}, \dots , x _ {n})
$$

则该函数在 $(\bar{x}_{1},\bar{x}_{2},\cdots,\bar{x}_{n})$ 点的泰勒级数为

$$
\mathbf {Y} = \mathbf {f} (\bar {x} _ {1}, \bar {x} _ {2}, \dots , \bar {x} _ {n}) + \sum_ {i = 1} ^ {\infty} \frac {1}{i !} \left(\delta x _ {1} \frac {\partial}{\partial x _ {1}} + \delta x _ {2} \frac {\partial}{\partial x _ {2}} + \dots + \delta x _ {n} \frac {\partial}{\partial x _ {n}}\right) ^ {i} \mathbf {f} (\bar {x} _ {1}, \bar {x} _ {2}, \dots , \bar {x} _ {n})\tag{6.3.10}
$$

其中 $x_{1}=\bar{x}_{1}+\delta x_{1},x_{2}=\bar{x}_{2}+\delta x_{2},\cdots,x_{n}=\bar{x}_{n}+\delta x_{n}$

记

$$
\boldsymbol {X} = \left[ \begin{array}{c} x _ {1} \\ x _ {2} \\ \vdots \\ x _ {n} \end{array} \right], \quad \overline {{\boldsymbol {X}}} = \left[ \begin{array}{c} \overline {{x}} _ {1} \\ \overline {{x}} _ {2} \\ \vdots \\ \overline {{x}} _ {n} \end{array} \right], \quad \delta \boldsymbol {X} = \left[ \begin{array}{c} \delta x _ {1} \\ \delta x _ {2} \\ \vdots \\ \delta x _ {n} \end{array} \right], \quad \nabla = \left[ \begin{array}{c} \frac {\partial}{\partial x _ {1}} \\ \frac {\partial}{\partial x _ {2}} \\ \vdots \\ \frac {\partial}{\partial x _ {n}} \end{array} \right]\tag{6.3.11}
$$

则式 $(6.3.10)$ 可写成

$$
\mathbf {Y} = \mathbf {f} (\bar {\mathbf {X}}) + \sum_ {i = 1} ^ {\infty} \frac {1}{i !} (\delta \mathbf {X} \cdot \nabla) ^ {i} \mathbf {f} (\bar {\mathbf {X}})\tag{6.3.12}
$$

式中， $\nabla$ 为偏微分算子向量，仅对函数f起作用，而对 $\delta X$ 不起作用，(·)为向量的点乘积。

记算子

$$
\boldsymbol {D} _ {\delta \boldsymbol {X}} ^ {i} \boldsymbol {f} = (\delta \boldsymbol {X} \cdot \nabla) ^ {i} \boldsymbol {f} (\bar {\boldsymbol {X}})\tag{6.3.13}
$$

则

$$
\mathbf {Y} = f (\mathbf {X}) = f (\overline {{\mathbf {X}}}) + \mathbf {D} _ {\delta \mathbf {X}} f + \frac {1}{2} \mathbf {D} _ {\delta \mathbf {X}} ^ {2} f + \frac {1}{3 !} \mathbf {D} _ {\delta \mathbf {X}} ^ {3} f + \frac {1}{4 !} \mathbf {D} _ {\delta \mathbf {X}} ^ {4} f + \dots\tag{6.3.14}
$$

考察上式右侧第二项和第三项。

$$
\begin{array}{r l} \boldsymbol {D} _ {\delta \boldsymbol {X}} \boldsymbol {f} & = (\delta \boldsymbol {X} \cdot \nabla) \boldsymbol {f} (\bar {\boldsymbol {X}}) = \left(\delta x _ {1} \frac {\partial f}{\partial x _ {1}} + \delta x _ {2} \frac {\partial f}{\partial x _ {2}} + \dots + \delta x _ {n} \frac {\partial f}{\partial x _ {n}}\right) _ {\boldsymbol {X} = \bar {\boldsymbol {X}}} = \\ & \left[ \frac {\partial f}{\partial x _ {1}} \frac {\partial f}{\partial x _ {2}} \dots \frac {\partial f}{\partial x _ {n}} \right] \Big | _ {\boldsymbol {X} = \bar {\boldsymbol {X}}} \cdot \delta \boldsymbol {X} \end{array}
$$

根据矩阵间的微分关系式，

$$
\boldsymbol {D} _ {\delta \boldsymbol {X}} \boldsymbol {f} = \frac {\partial \boldsymbol {f}}{\partial \boldsymbol {X} ^ {\mathrm{T}}} \Big | _ {\boldsymbol {X} = \bar {\boldsymbol {X}}} \cdot \delta \boldsymbol {X}\tag{6.3.15}
$$

其中 $\frac{\partial f}{\partial X^{\mathrm{T}}}$ 为 $f$ 的雅可比矩阵，记为

$$
\begin{array}{r l} & {\pmb {F} (\bar {\pmb {X}}) = \frac {\partial \pmb {f}}{\partial \pmb {X} ^ {\mathrm{T}}} \Big | _ {\pmb {x} = \bar {\pmb {x}}}} \\ & {\frac {1}{2} \pmb {D} _ {\delta \pmb {X}} ^ {2} \pmb {f} = \frac {1}{2} \left[ (\delta \pmb {X} \cdot \nabla) (\delta \pmb {X} \cdot \nabla) \right] \pmb {f} (\bar {\pmb {X}}) = \frac {1}{2} \left[ (\delta \pmb {X} ^ {\mathrm{T}} \nabla) ^ {2} \right] \pmb {f} (\bar {\pmb {X}}) =} \\ & {\qquad \frac {1}{2} \left[ (\delta \pmb {X} ^ {\mathrm{T}} \nabla) ^ {\mathrm{T}} (\delta \pmb {X} ^ {\mathrm{T}} \nabla) \right] \pmb {f} (\bar {\pmb {X}}) =} \\ & {\qquad \frac {1}{2} (\nabla^ {\mathrm{T}} \delta \pmb {X} \delta \pmb {X} ^ {\mathrm{T}} \nabla) \pmb {f} (\bar {\pmb {X}})} \end{array}\tag{6.3.16}
$$

所以式 $(6.3.14)$ 可写成

$$
\mathbf {Y} = \mathbf {f} (\mathbf {X}) = \mathbf {f} (\bar {\mathbf {X}}) + \mathbf {F} (\bar {\mathbf {X}}) \delta \mathbf {X} + \frac {1}{2} (\nabla^ {\mathrm{T}} \delta \mathbf {X} \delta \mathbf {X} ^ {\mathrm{T}} \nabla) \mathbf {f} (\bar {\mathbf {X}}) + \frac {1}{3 !} \mathbf {D} _ {\delta x} ^ {3} \mathbf {f} + \frac {1}{4 !} \mathbf {D} _ {\delta x} ^ {4} \mathbf {f} + \dots\tag{6.3.17}
$$

式 $(6.3.11)$ 定义的偏微分算子

$$
\nabla^ {\mathrm{T}} = \left[ \frac {\partial}{\partial x _ {1}} \quad \frac {\partial}{\partial x _ {2}} \quad \dots \quad \frac {\partial}{\partial x _ {n}} \right]
$$

在运算中视为向量,此处以式(6.3.17)中的第三项为例说明其正确性。

$$
\begin{array}{l} \frac {1}{2} (\nabla^ {\mathrm{T}} \delta \mathbf {X} \delta \mathbf {X} ^ {\mathrm{T}} \nabla) f = \frac {1}{2} \left[ \left[ \begin{array}{c c c c} \frac {\partial}{\partial x _ {1}} & \frac {\partial}{\partial x _ {2}} & \dots & \frac {\partial}{\partial x _ {n}} \end{array} \right] \left[ \begin{array}{c} \delta x _ {1} \\ \delta x _ {2} \\ \vdots \\ \delta x _ {n} \end{array} \right] [ \delta x _ {1} & \delta x _ {2} & \dots & \delta x _ {n} ] \left[ \begin{array}{c} \frac {\partial}{\partial x _ {1}} \\ \frac {\partial}{\partial x _ {2}} \\ \vdots \\ \frac {\partial}{\partial x _ {n}} \end{array} \right] \right] f = \\ \frac {1}{2} \left(\frac {\partial}{\partial x _ {1}} \delta x _ {1} + \frac {\partial}{\partial x _ {2}} \delta x _ {2} + \dots + \frac {\partial}{\partial x _ {n}} \delta x _ {n}\right) ^ {2} f \end{array}
$$

该式正是向量多元函数泰勒级数展开式中的二次项。所以在计算式(6.3.13)时,可将 $\nabla$ 视为向量,

$$
\begin{array}{r l} \boldsymbol {D} _ {\delta \boldsymbol {X}} ^ {i} \boldsymbol {f} & = (\delta \boldsymbol {X} \nabla) ^ {i} \boldsymbol {f} (\bar {\boldsymbol {X}}) = \\ & [ (\nabla^ {\mathrm{T}} \delta \boldsymbol {X}) (\nabla^ {\mathrm{T}} \delta \boldsymbol {X}) \dots (\nabla^ {\mathrm{T}} \delta \boldsymbol {X}) ] \boldsymbol {f} (\bar {\boldsymbol {X}}) = \\ & [ (\delta \boldsymbol {X} ^ {\mathrm{T}} \nabla) (\delta \boldsymbol {X} ^ {\mathrm{T}} \nabla) \dots (\delta \boldsymbol {X} ^ {\mathrm{T}} \nabla) ] \boldsymbol {f} (\bar {\boldsymbol {X}}) \end{array}
$$

应用该结论,可计算出 $D_{\delta x}^{2}f$ 的均值为

$$
\begin{array}{r l} E (\pmb {D} _ {\delta \pmb {X}} ^ {2} \pmb {f}) & = E \big [ (\nabla^ {\mathrm{T}} \delta \pmb {X}) (\delta \pmb {X} ^ {\mathrm{T}} \nabla) \big ] \pmb {f} (\bar {\pmb {X}}) = \\ & \big [ \nabla^ {\mathrm{T}} E (\delta \pmb {X} \delta \pmb {X} ^ {\mathrm{T}}) \nabla \big ] \pmb {f} (\bar {\pmb {X}}) = \\ & (\nabla^ {\mathrm{T}} \pmb {P} _ {X X} \nabla) \pmb {f} (\bar {\pmb {X}}) \end{array}\tag{6.3.18}
$$

(2) UT 变换的均值精度分析。设 X 为随机矢量, $\bar{X}$ 为其均值, 则 $\delta X = X - \bar{X}$ 为随机矢量。如果 X 的概率密度相对 $\bar{X}$ 对称, 如正态分布, t 分布等, 则 $\delta X$ 的奇次阶矩为零, 即

$$
E [ D _ {\delta X} ^ {2 i - 1} f ] = 0 \quad i = 1, 2, \dots
$$

所以由式(6.3.17)经非线性变换映射 $Y=f(X)$ 后获得的随机矢量Y即真实映射的均值为

$$
\begin{array}{r l} E [ \mathbf {Y} ] & = E [ \mathbf {f} (\mathbf {X}) ] = \mathbf {f} (\bar {\mathbf {X}}) + \frac {1}{2} \left[ \nabla^ {\mathrm{T}} E (\delta \mathbf {X} \delta \mathbf {X} ^ {\mathrm{T}}) \nabla \right] \mathbf {f} (\bar {\mathbf {X}}) + \\ & E \left[ \frac {1}{4 !} \mathbf {D} _ {\delta \mathbf {X}} ^ {4} \mathbf {f} + \frac {1}{6 !} \mathbf {D} _ {\delta \mathbf {X}} ^ {6} \mathbf {f} + \dots \right] \end{array}
$$

注意到 $P_{XX}=E(\delta X\delta X^{\mathrm{T}})$ 为 X 的自协方差阵，所以由 $Y=f(X)$ 实现的真实映射的均值为：

$$
\overline {{{\mathbf {Y}}}} _ {r} = E [ \mathbf {Y} ] = f (\overline {{{\mathbf {X}}}}) + \frac {1}{2} \left[ \nabla^ {\mathrm{T}} \mathbf {P} _ {x x} \nabla \right] f (\overline {{{\mathbf {X}}}}) + E \left[ \frac {1}{4 !} \mathbf {D} _ {\delta \mathbf {x}} ^ {4} f + \frac {1}{6 !} \mathbf {D} _ {\delta \mathbf {x}} ^ {6} f + \dots \right]\tag{6.3.19}
$$

下面分析采用 UT 变换实现 $Y = f(X)$ 所获得的变换值即 UT 映射 $Y_{UT}$ 的均值。

在 UT 变换中, 自变量 X 的 1 倍 $\sigma$ 再现样本点为

$$
\chi_ {0} = \overline {{{X}}}\tag{6.3.20a}
$$

$$
\chi_ {i} = \overline {{{X}}} + (\sqrt {(n + \lambda) P _ {X X}}) _ {(i)}, \quad i = 1, 2, \dots , n\tag{6.3.20b}
$$

$$
\chi_ {i} = \bar {X} - (\sqrt {(n + \lambda) P _ {X X}}) _ {(i - n)}, \quad i = n + 1, n + 2, \dots , 2 n\tag{6.3.20c}
$$

这些再现样本点的映射样本点为

$$
\mathbf {y} _ {i} = f (\boldsymbol {\chi} _ {i}) \quad i = 0, 1, 2, \dots , 2 n
$$

映射的均值和协方差阵按下式近似计算

$$
\overline {{{\mathbf {Y}}}} _ {U T} = \sum_ {i = 0} ^ {2 n} W _ {i} ^ {(m)} \mathbf {y} _ {i}
$$

$$
\boldsymbol {P} _ {Y Y} = \sum_ {i = 0} ^ {2 n} W _ {i} ^ {(c)} (\mathbf {y} _ {i} - \overline {{\mathbf {Y}}} _ {U T}) (\mathbf {y} _ {i} - \overline {{\mathbf {Y}}} _ {U T}) ^ {\mathrm{T}}
$$

其中权值 W 按式(6.3.7) 计算。

对 $y_{i}$ 在 $\bar{X}$ 点展开, 记

$$
\sigma_ {i} = \left\{ \begin{array}{l l} (\sqrt {(n + \lambda) P _ {X X}}) _ {(i)} & i = 1, 2, \dots , n \\ (- \sqrt {(n + \lambda) P _ {X X}}) _ {(i - n)} & i = n + 1, n + 2, \dots , 2 n \end{array} \right.
$$

则

$$
\mathbf {y} _ {i} = f (\boldsymbol {\chi} _ {i}) = f (\bar {\boldsymbol {X}}) + \boldsymbol {D} _ {\sigma_ {i}} f + \frac {1}{2} \boldsymbol {D} _ {\sigma_ {i}} ^ {2} f + \frac {1}{3 !} \boldsymbol {D} _ {\sigma_ {i}} ^ {3} f + \frac {1}{4 !} \boldsymbol {D} _ {\sigma_ {i}} ^ {4} f + \dots i = 1, 2, \dots , 2 n
$$

$$
\mathbf {y} _ {0} = f (\boldsymbol {\chi} _ {0}) = f (\bar {\boldsymbol {X}})
$$

所以

$$
\begin{array}{r l}\overline {{{\mathbf {Y}}}} _ {U T}&= \frac {\lambda}{n + \lambda} f (\overline {{{\mathbf {X}}}}) + \frac {1}{2 (n + \lambda)} \sum_ {i = 1} ^ {2 n} \left[ f (\overline {{{\mathbf {X}}}}) + D _ {\sigma i} f + \frac {1}{2} D _ {\sigma i} ^ {2} f + \frac {1}{3 !} D _ {\sigma i} ^ {3} f + \frac {1}{4 !} v _ {\sigma i} ^ {4} f + \dots \right] =\\&f (\overline {{{\mathbf {X}}}}) \left[ \frac {\lambda}{n + \lambda} + \frac {1}{2 (n + \lambda)} \sum_ {i = 1} ^ {2 n} 1 \right] + \frac {1}{2 (n + \lambda)} \sum_ {i = 1} ^ {2 n} \left(D _ {\sigma i} f + \frac {1}{2} D _ {\sigma i} ^ {2} f + \frac {1}{3 !} D _ {\sigma i} ^ {3} f + \frac {1}{4 !} D _ {\sigma i} ^ {4} f + \dots\right) =\\&f (\overline {{{\mathbf {X}}}}) + \frac {1}{2 (n + \lambda)} \sum_ {i = 1} ^ {2 n} \left( \right.D _ {\sigma i} f + \frac {1}{2} D _ {\sigma i} ^ {2} f + \frac {1}{3 !} D _ {\sigma i} ^ {3} f + \frac {1}{4 !} D _ {\sigma i} ^ {4} f + \dots\end{array}
$$

由式(6.3.20)知 X 的再现样本点对 $\bar{X}$ 是对称分布的，上式求和后，算子的奇次项互相抵消，如

$$
\begin{array}{l} \sum_ {i = 1} ^ {2 n} D _ {\sigma i} f = \sum_ {i = 1} ^ {2 n} \frac {\partial f}{\partial X ^ {\mathrm{T}}} \Big | _ {X = \bar {X}} \boldsymbol {\sigma} _ {i} = \sum_ {i = 1} ^ {n} \frac {\partial f}{\partial X ^ {\mathrm{T}}} \Big | _ {X = \bar {X}} \boldsymbol {\sigma} _ {i} + \sum_ {i = n + 1} ^ {2 n} \frac {\partial f}{\partial X ^ {\mathrm{T}}} \Big | _ {X = \bar {X}} \boldsymbol {\sigma} _ {i} = \\ \sum_ {i = 1} ^ {n} \frac {\partial f}{\partial X ^ {\mathrm{T}}} _ {X = \bar {X}} (\sqrt {(n + \lambda) P _ {X X}}) _ {(i)} - \sum_ {j = 1} ^ {n} \frac {\partial f}{\partial X ^ {\mathrm{T}}} _ {X = \bar {X}} (\sqrt {(n + \lambda) P _ {X X}}) _ {(j)} = 0 \end{array}
$$

所以

$$
\overline {{{\mathbf {Y}}}} _ {U T} = \boldsymbol {f} (\overline {{{\mathbf {X}}}}) + \frac {1}{2 (n + \lambda)} \sum_ {i = 1} ^ {2 n} \left[ \frac {1}{2} \boldsymbol {D} _ {\sigma i} ^ {2} \boldsymbol {f} + \frac {1}{4 !} \boldsymbol {D} _ {\sigma i} ^ {4} \boldsymbol {f} + \dots \right]\tag{6.3.21}
$$

由式 $(6.3.16)$

$$
\begin{array}{r l} \frac {1}{2 (n + \lambda)} \sum_ {i = 1} ^ {2 n} \frac {1}{2} D _ {\sigma i} ^ {2} f & = \frac {1}{2 (n + \lambda)} \sum_ {i = 1} ^ {2 n} \frac {1}{2} (\nabla^ {\mathrm{T}} \boldsymbol {\sigma} _ {i} \boldsymbol {\sigma} _ {i} ^ {\mathrm{T}} \nabla) f (\bar {\mathbf {X}}) = \\ & \frac {1}{2 (n + \lambda)} \left[ \frac {1}{2} \nabla^ {\mathrm{T}} \sum_ {i = 1} ^ {2 n} (n + \lambda) (\sqrt {\boldsymbol {P} _ {X X}}) _ {(i)} (\sqrt {\boldsymbol {P} _ {X X}}) _ {(i)} ^ {\mathrm{T}} \nabla \right] f (\bar {\mathbf {X}}) = \\ & \frac {1}{2} \left[ \frac {1}{2} \nabla^ {\mathrm{T}} \cdot 2 \sum_ {i = 1} ^ {n} (\sqrt {\boldsymbol {P} _ {X X}}) _ {(i)} (\sqrt {\boldsymbol {P} _ {X X}}) _ {(i)} ^ {\mathrm{T}} \nabla \right] f (\bar {\mathbf {X}}) = \\ & \frac {1}{2} [ \nabla^ {\mathrm{T}} \boldsymbol {P} _ {X X} \nabla ] f (\bar {\mathbf {X}}) \end{array}
$$

上式代入式(6.3.21)，得

$$
\bar {\mathbf {Y}} _ {U T} = \boldsymbol {f} (\bar {\mathbf {X}}) + \frac {1}{2} (\nabla^ {\mathrm{T}} \boldsymbol {P} _ {X X} \nabla) \boldsymbol {f} (\bar {\mathbf {X}}) + \frac {1}{2 (n + \lambda)} \sum_ {i = 1} ^ {2 n} \left(\frac {1}{4 !} \boldsymbol {D} _ {\sigma i} ^ {4} \boldsymbol {f} + \frac {1}{6 !} \boldsymbol {D} _ {\sigma i} ^ {6} \boldsymbol {f} + \dots\right)\tag{6.3.22}
$$

比较式(6.3.19)和式(6.3.22)可看出,采用UT变换计算的映射均值与映射的真实均值仅在泰勒级数的四阶项起不一样,而零阶项至三阶项完全相同。

而如果对非线性函数只作线性化处理,则在对称分布的情况下,映射的均值只含有真实均值的零阶项。

(3) UT 变换的协方差阵分析。由 $Y = f(X)$ 确定的真实映射的协方差分析如下。

$$
\begin{array}{r l} & (\mathbf {P} _ {\mathrm{YY}}) _ {r} = E [ (\mathbf {Y} - \overline {{\mathbf {Y}}} _ {r}) (\mathbf {Y} - \overline {{\mathbf {Y}}} _ {r}) ^ {\mathrm{T}} ] = E [ \mathbf {Y} \mathbf {Y} ^ {\mathrm{T}} ] - E [ \mathbf {Y} \overline {{\mathbf {Y}}} _ {r} ^ {\mathrm{T}} ] - E [ \overline {{\mathbf {Y}}} _ {r} \mathbf {Y} ^ {\mathrm{T}} ] + \overline {{\mathbf {Y}}} _ {r} \overline {{\mathbf {Y}}} _ {r} ^ {\mathrm{T}} = \\ & E [ \mathbf {Y} \mathbf {Y} ^ {\mathrm{T}} ] - \overline {{\mathbf {Y}}} _ {r} \overline {{\mathbf {Y}}} _ {r} ^ {\mathrm{T}} \end{array}
$$

式 $(6.3.14)$ 和式 $(6.3.19)$ 代入上式，

$$
\begin{array}{r l} (P _ {Y Y}) _ {r} = & E \left\{\left[ f (\bar {X}) + D _ {\delta X} f + \frac {1}{2} D _ {\delta X} ^ {2} f + \frac {1}{3 !} D _ {\delta X} ^ {3} f + \frac {1}{4 !} D _ {\delta X} ^ {4} f + \dots \right] \cdot \right. \\ & \left[ f ^ {\mathrm{T}} (\bar {X}) + (D _ {\delta X} f) ^ {\mathrm{T}} + \frac {1}{2} (D _ {\delta X} ^ {2} f) ^ {\mathrm{T}} + \frac {1}{3 !} (D _ {\delta X} ^ {3} f) ^ {\mathrm{T}} + \frac {1}{4 !} (D _ {\delta X} ^ {4} f) ^ {\mathrm{T}} + \dots \right] - \end{array}
$$

$$
\begin{array}{l} \left[ f (\bar {X}) + \frac {1}{2} (\nabla^ {\mathrm{T}} P _ {x x} \nabla) f (\bar {X}) + E \left(\frac {1}{4 !} D _ {\delta X} ^ {4} f + \frac {1}{6 !} D _ {\delta X} ^ {6} f + \dots\right) \right] \cdot \\ \left[ f ^ {\mathrm{T}} (\bar {X}) + \frac {1}{2} [ (\nabla^ {\mathrm{T}} P _ {x x} \nabla) f (\bar {X}) ] ^ {\mathrm{T}} + E \left(\frac {1}{4 !} D _ {\delta X} ^ {4} f + \frac {1}{6 !} D _ {\delta X} ^ {6} f + \dots\right) ^ {\mathrm{T}} \right] \end{array}
$$

考虑到 X 对称分布时, $\delta X$ 的奇次阶矩为零, 并利用式(6.3.15),

$$
\begin{array}{r l} & (\boldsymbol {P} _ {Y Y}) _ {r} = \boldsymbol {F} (\bar {\boldsymbol {X}}) \boldsymbol {P} _ {X X} \boldsymbol {F} ^ {\mathrm{T}} (\bar {\boldsymbol {X}}) - \frac {1}{4} [ (\nabla^ {\mathrm{T}} \boldsymbol {P} _ {X X} \nabla) \boldsymbol {f} (\bar {\boldsymbol {X}}) ] \cdot [ (\nabla^ {\mathrm{T}} \boldsymbol {P} _ {X X} \nabla) \boldsymbol {f} (\bar {\boldsymbol {X}}) ] ^ {\mathrm{T}} + \\ & E \sum_ {i = 2} ^ {\infty} \sum_ {j = 2} ^ {\infty} (\boldsymbol {D} _ {\delta \boldsymbol {X}} ^ {i} \boldsymbol {f}) (\boldsymbol {D} _ {\delta \boldsymbol {X}} ^ {j} \boldsymbol {f}) ^ {\mathrm{T}} - E [ \sum_ {i = 1} ^ {\infty} \sum_ {j = 1} ^ {\infty} \frac {1}{(2 i) ! (2 j) !} (\boldsymbol {D} _ {\delta \boldsymbol {X}} ^ {2 i} \boldsymbol {f}) (\boldsymbol {D} _ {\delta \boldsymbol {X}} ^ {2 j} \boldsymbol {f}) ^ {\mathrm{T}} ] \end{array}\tag{6.3.23}
$$

$Y=f(X)$ 由 UT 变换确定的计算映射的协方差阵分析如下。

$$
\left(\boldsymbol {P} _ {Y Y}\right) _ {U T} = E \left[ \left(\boldsymbol {Y} - \bar {\boldsymbol {Y}} _ {U T}\right) \left(\boldsymbol {Y} - \bar {\boldsymbol {Y}} _ {U T}\right) ^ {\mathrm{T}} \right]
$$

根据式 $(6.3.14)$ 和式 $(6.3.22)$ ，

$$
\begin{array}{l}\mathbf {Y} - \overline {{\mathbf {Y}}} _ {U T} = f (\overline {{\mathbf {X}}}) + D _ {\delta \mathbf {X}} f + \frac {1}{2} D _ {\delta \mathbf {X}} ^ {2} f + \frac {1}{3 !} D _ {\delta \mathbf {X}} ^ {3} f + \dots - f (\overline {{\mathbf {X}}}) -\\\frac {1}{2} (\nabla^ {\mathrm{T}} P _ {X X} \nabla) f (\overline {{\mathbf {X}}}) - \frac {1}{2 (n + \lambda)} \sum_ {i = 1} ^ {2 n} \left(\frac {1}{4 !} D _ {\sigma i} ^ {4} f + \frac {1}{6 !} D _ {\sigma i} ^ {6} f + \dots\right) =\\F (\overline {{\mathbf {X}}}) \delta \mathbf {X} - \frac {1}{2} (\nabla^ {\mathrm{T}} P _ {X X} \nabla) f (\overline {{\mathbf {X}}}) + \frac {1}{2} D _ {\delta \mathbf {X}} ^ {2} f + \frac {1}{3 !} D _ {\delta \mathbf {X}} ^ {3} f + \dots -\\\frac {1}{2 (n + \lambda)} \sum_ {i = 1} ^ {2 n} \left(\frac {1}{4 !} D _ {\sigma i} ^ {4} f + \frac {1}{6 !} D _ {\sigma i} ^ {6} f + \dots\right)\\(P _ {Y Y}) _ {U T} = E \left\{\left[ F (\overline {{\mathbf {X}}}) \delta \mathbf {X} - \frac {1}{2} (\nabla^ {\mathrm{T}} P _ {X X} \nabla) f (\overline {{\mathbf {X}}}) + \frac {1}{2} D _ {\delta X} ^ {2} f + \frac {1}{3 !} D _ {\delta X} ^ {3} f + \dots - \right. \right.\\\left. \frac {1}{2 (n + \lambda)} \sum_ {k = 1} ^ {\infty} \sum_ {i = 1} ^ {2 n} \frac {1}{(2 k) !} D _ {\sigma_ {i}} ^ {2 k} f \right] \cdot \left[ \delta X ^ {\mathrm{T}} F ^ {\mathrm{T}} (\overline {{\mathbf {X}}}) - \frac {1}{2} ((\nabla^ {\mathrm{T}} P _ {X X} \nabla) f (\overline {{\mathbf {X}}})) ^ {\mathrm{T}} + \right.\\\left. \frac {1}{2} (D _ {\delta X} ^ {2} f) ^ {\mathrm{T}} + \frac {1}{3 !} (D _ {\delta X} ^ {3} f) ^ {\mathrm{T}} + \dots - \frac {1}{2 (n + \lambda)} \sum_ {k = 1} ^ {\infty} \sum_ {i = 1} ^ {2 n} \frac {1}{(2 k) !} (D _ {\sigma_ {i}} ^ {2 k} f) ^ {\mathrm{T}} \right]\left. \right\} =\\F (\overline {{\mathbf {X}}}) P _ {X X} F ^ {\mathrm{T}} (\overline {{\mathbf {X}}}) + \frac {1}{4} (\nabla^ {\mathrm{T}} P _ {X X} \nabla) f (\overline {{\mathbf {X}}}) [ (\nabla^ {\mathrm{T}} P _ {X X} \nabla) f (\overline {{\mathbf {X}}}) ] ^ {\mathrm{T}} -\\\frac {1}{4} (\nabla^ {\mathrm{T}} P _ {X X} \nabla) f (\overline {{\mathbf {X}}}) E (D _ {\delta X} ^ {2} f) ^ {\mathrm{T}} - \frac {1}{4} E (D _ {\delta X} ^ {2} f) [ (\nabla^ {\mathrm{T}} P _ {X X} \nabla) f (\overline {{\mathbf {X}}}) ] ^ {\mathrm{T}} +\\\frac {1}{4} E [ (D _ {\delta X} ^ {2} f) (D _ {\delta X} ^ {2} f) ^ {\mathrm{T}} ] + \frac {1}{2 \times 3 !} E [ (D _ {\delta X} ^ {2} f) (D _ {\delta X} ^ {3} f) ^ {\mathrm{T}} ] + \dots .\end{array}
$$

记上式第5项起的和式为 $\Delta_{\Sigma}(\delta X,\sigma)$ ，该和式是泰勒级数展开式中关于 $\delta X$ 和 $\sigma$ 高于四次的项，是高次项和。将式(6.3.18)代入上式，得

$$
\left(\boldsymbol {P} _ {Y Y}\right) _ {U T} = \boldsymbol {F} (\bar {\boldsymbol {X}}) \boldsymbol {P} _ {X X} \boldsymbol {F} ^ {\mathrm{T}} (\bar {\boldsymbol {X}}) - \frac {1}{4} \left(\nabla^ {\mathrm{T}} \boldsymbol {P} _ {X X} \nabla\right) \boldsymbol {f} (\bar {\boldsymbol {X}}) \left[ \left(\nabla^ {\mathrm{T}} \boldsymbol {P} _ {X X} \nabla\right) \boldsymbol {f} (\bar {\boldsymbol {X}}) \right] ^ {\mathrm{T}} + \boldsymbol {\Delta} _ {\sum} (\delta \boldsymbol {X}, \boldsymbol {\sigma})\tag{6.3.24}
$$

比较式 $(6.3.23)$ 和式 $(6.3.24)$ ，可以看出：

由 UT 确定的计算映射的协方差阵与真实映射的协方差阵前两项相同。而当 EKF 仅取线性项时，

$$
\begin{array}{r} \mathbf {Y} _ {L} = f (\bar {\mathbf {X}}) + F (\bar {\mathbf {X}}) \delta \mathbf {X} \\ \bar {\mathbf {Y}} _ {L} = f (\bar {\mathbf {X}}) \end{array}\tag{6.3.25}
$$

$$
\left(\boldsymbol {P} _ {Y Y}\right) _ {L} = E \left[ \left(\boldsymbol {Y} - \bar {\boldsymbol {Y}} _ {L}\right) \left(\boldsymbol {Y} - \bar {\boldsymbol {Y}} _ {L}\right) ^ {\mathrm{T}} \right]
$$

将式(6.3.14)、式(6.3.25)代入上式，并应用式(6.3.15)式，得

$$
\begin{array}{r l} (P _ {Y Y}) _ {L} = & E \left\{\left[ D _ {\delta X} f + \frac {1}{2} D _ {\delta X} ^ {2} f + \frac {1}{3 !} D _ {\delta X} ^ {3} f + \dots \right] \cdot \left[ (D _ {\delta X} f) ^ {\mathrm{T}} + \frac {1}{2} (D _ {\delta X} ^ {2} f) ^ {\mathrm{T}} + \frac {1}{3 !} (D _ {\delta X} ^ {3} f) ^ {\mathrm{T}} + \dots \right] \right\} = \\ & F (\bar {X}) P _ {X X} F ^ {\mathrm{T}} (\bar {X}) + \frac {1}{4} E [ (D _ {\delta X} ^ {2} f) (D _ {\delta X} ^ {2} f) ^ {\mathrm{T}} ] + \frac {1}{(3 !) ^ {2}} E [ (D _ {\delta X} ^ {3} f) (D _ {\delta X} ^ {3} f) ^ {\mathrm{T}} ] + \dots \end{array}
$$

该式说明： $(\boldsymbol{P}_{YY})_{L}$ 至少为 $\boldsymbol{F}(\bar{\boldsymbol{X}})\boldsymbol{P}_{XX}\boldsymbol{F}^{\mathrm{T}}(\bar{\boldsymbol{X}})$ ，而 UT 变换对应的 $(\boldsymbol{P}_{YY})_{UT}$ 不会超过该值。所以，当对象为非线性系统时，UT 变换的精度优于线性化精度，非线性越强，精度的差异越明显。

## 3. UKF 算法

（1）可加性噪声条件下的 UKF 算法。可加性噪声条件下的系统方程和量测方程如式(6.3.1)和式(6.3.2)所示。

由式(6.3.4)知,计算增益的关键是确定出 $P_{(XZ)_{k/k-1}}$ 和 $P_{(ZZ)_{k/k-1}}$ ,该两协方差阵由UT变换完成计算,具体计算步骤如下。

步骤1,选定滤波初值:

$$
\hat {\mathbf {X}} _ {0} = E \mathbf {X} _ {0}\tag{6.3.26a}
$$

$$
\boldsymbol {P} _ {0} = E \left[ \left(\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}\right) \left(\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}\right) ^ {\mathrm{T}} \right]\tag{6.3.26b}
$$

对 $k=1,2,3,\cdots$ ，执行：

步骤 2, 计算 k-1 时刻的 $2n+1$ 个 $\sigma$ 样本点

$$
\tilde {\chi} _ {k - 1} ^ {(0)} = \hat {X} _ {k - 1}\tag{6.3.27a}
$$

$$
\tilde {\chi} _ {k - 1} ^ {(i)} = \hat {X} _ {k - 1} + \gamma (\sqrt {P _ {k - 1}}) _ {(i)} \quad i = 1, 2, \dots , n\tag{6.3.27b}
$$

$$
\tilde {\chi} _ {k - 1} ^ {(i)} = \hat {X} _ {k - 1} - \gamma (\sqrt {P _ {k - 1}}) _ {(i - n)} \quad i = n + 1, n + 2, \dots , 2 n\tag{6.3.27c}
$$

式中， $\gamma=\sqrt{n+\lambda}$ ， $\lambda$ 的确定见式(6.3.7d)。

步骤 3, 计算 k 时刻的一步预测模型值

$$
\boldsymbol {\chi} _ {k / k - 1} ^ {* (i)} = \boldsymbol {f} \left[ \tilde {\boldsymbol {\chi}} _ {k - 1} ^ {(i)}, \boldsymbol {u} _ {k - 1} \right] \quad i = 0, 1, 2, \dots , 2 n\tag{6.3.28a}
$$

$$
\hat {\boldsymbol {X}} _ {k / k - 1} = \sum_ {i = 0} ^ {2 n} W _ {i} ^ {(m)} \boldsymbol {\chi} _ {k / k - 1} ^ {* (i)}\tag{6.3.28b}
$$

$$
\boldsymbol {P} _ {k / k - 1} = \sum_ {i = 0} ^ {2 n} W _ {i} ^ {(c)} \left[ \boldsymbol {\chi} _ {k / k - 1} ^ {* (i)} - \hat {\boldsymbol {X}} _ {k / k - 1} \right] \left[ \boldsymbol {\chi} _ {k / k - 1} ^ {* (i)} - \hat {\boldsymbol {X}} _ {k / k - 1} \right] ^ {\mathrm{T}} + \boldsymbol {Q} _ {k - 1}\tag{6.3.28c}
$$

步骤 4, 计算 k 时刻的一步预测增广样本点

$$
\chi_ {k / k - 1} ^ {(i)} = \chi_ {k / k - 1} ^ {* (i)} \quad i = 0, 1, 2, \dots , 2 n\tag{6.3.29a}
$$

$$
\chi_ {k / k - 1} ^ {(i)} = \chi_ {k / k - 1} ^ {(0)} + \gamma (\sqrt {Q _ {k - 1}}) _ {(i - 2 n)} \quad i = 2 n + 1, 2 n + 2, \dots , 3 n\tag{6.3.29b}
$$

$$
\chi_ {k / k - 1} ^ {(i)} = \chi_ {k / k - 1} ^ {(0)} - \gamma (\sqrt {Q _ {k - 1}}) _ {(i - 3 n)} \quad i = 3 n + 1, 3 n + 2, \dots , 4 n\tag{6.3.29c}
$$

$$
\mathbb {Z} _ {k / k - 1} ^ {(i)} = \boldsymbol {h} \left[ \chi_ {k / k - 1} ^ {(i)} \right] \quad i = 0, 1, 2, 3, \dots , 4 n\tag{6.3.29d}
$$

$$
\hat {\mathbf {Z}} _ {k / k - 1} = \sum_ {i = 0} ^ {4 n} W _ {i} ^ {(m)} \mathbb {Z} _ {k / k - 1} ^ {(i)}\tag{6.3.29e}
$$

式中， $W_{i}^{(m)}=\frac{1}{2(2n+\lambda)},i=1,2,3,\cdots,4n;W_{0}^{(m)}=\frac{\lambda}{2n+\lambda}$ 。

步骤 5, 计算 $P_{(XZ)_{k/k-1}}$ , $P_{(ZZ)_{k/k-1}}$

$$
\boldsymbol {P} _ {(X Z) _ {k / k - 1}} = \sum_ {i = 0} ^ {4 n} W _ {i} ^ {(c)} \left[ \boldsymbol {\chi} _ {k / k - 1} ^ {(i)} - \hat {\boldsymbol {X}} _ {k / k - 1} \right] \left[ \mathbb {Z} _ {k / k - 1} ^ {(i)} - \hat {\mathbf {Z}} _ {k / k - 1} \right] ^ {\mathrm{T}}\tag{6.3.30a}
$$

$$
\boldsymbol {P} _ {(Z Z) _ {k / k - 1}} = \sum_ {i = 0} ^ {4 n} W _ {i} ^ {(c)} \left[ \mathbb {Z} _ {k / k - 1} ^ {(i)} - \hat {\mathbf {Z}} _ {k / k - 1} \right] \left[ \mathbb {Z} _ {k / k - 1} ^ {(i)} - \hat {\mathbf {Z}} _ {k / k - 1} \right] ^ {\mathrm{T}} + \boldsymbol {R} _ {k}\tag{6.3.30b}
$$

式中， $W_{i}^{(c)}=\frac{1}{2(2n+\lambda)},i=1,2,3,\cdots,4n;W_{0}^{(c)}=\frac{\lambda}{2n+\lambda}+1-\alpha^{2}+\beta.$

步骤 6, 计算增益矩阵

$$
\boldsymbol {K} _ {k} = \boldsymbol {P} _ {(X Z) _ {k / k - 1}} \boldsymbol {P} _ {(Z Z) _ {k / k - 1}} ^ {- 1}\tag{6.3.31a}
$$

步骤 7, 计算滤波值

$$
\hat {\mathbf {X}} _ {k} = \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {K} _ {k} [ \mathbf {Z} _ {k} - \hat {\mathbf {Z}} _ {k / k - 1} ]\tag{6.3.31b}
$$

$$
\boldsymbol {P} _ {k} = \boldsymbol {P} _ {k / k - 1} - \boldsymbol {K} _ {k} \boldsymbol {P} _ {(Z Z) _ {k / k - 1}} \boldsymbol {K} _ {k} ^ {\mathrm{T}}\tag{6.3.31c}
$$

如果在计算步骤 4 和 5 中对一步预测样本点不作增广, 则只需将步骤 4 和步骤 5 改写成如下形式:

步骤 4', 计算 k 时刻的一步预测样本点,

$$
\chi_ {k / k - 1} ^ {(0)} = \hat {\mathbf {X}} _ {k / k - 1}\tag{6.3.32a}
$$

$$
\chi_ {k / k - 1} ^ {(i)} = \hat {\mathbf {X}} _ {k / k - 1} + \gamma (\sqrt {\mathbf {P} _ {k / k - 1}}) _ {(i)} \quad i = 1, 2, \dots , n\tag{6.3.32b}
$$

$$
\chi_ {k / k - 1} ^ {(i)} = \hat {\mathbf {X}} _ {k / k - 1} - \gamma (\sqrt {\mathbf {P} _ {k / k - 1}}) _ {(i - n)} \quad i = n + 1, n + 2, \dots , 2 n\tag{6.3.32c}
$$

步骤 5', 计算 $P_{(XZ)_{k/k-1}}, P_{(ZZ)_{k/k-1}}$

$$
\boldsymbol {P} _ {(X Z) _ {k / k - 1}} = \sum_ {i = 0} ^ {2 n} W _ {i} ^ {(c)} \left[ \boldsymbol {\chi} _ {k / k - 1} ^ {(i)} - \hat {\boldsymbol {X}} _ {k / k - 1} \right] \left[ \mathbb {Z} _ {k / k - 1} ^ {(i)} - \hat {\mathbf {Z}} _ {k / k - 1} \right] ^ {\mathrm{T}}\tag{6.3.33a}
$$

$$
\boldsymbol {P} _ {(Z Z) _ {k / k - 1}} = \sum_ {i = 0} ^ {2 n} W _ {i} ^ {(c)} \left[ \mathbb {Z} _ {k / k - 1} ^ {(i)} - \dot {\boldsymbol {L}} _ {k / k - 1} \right] \left[ \mathbb {Z} _ {k / k - 1} ^ {(i)} - \hat {\boldsymbol {Z}} _ {k / k - 1} \right] ^ {\mathrm{T}} + \boldsymbol {R} _ {k}\tag{6.3.33b}
$$

式中

$$
\mathbb {Z} _ {k / k - 1} ^ {(i)} = \boldsymbol {h} [ \chi_ {k / k - 1} ^ {(i)} ], \quad i = 0, 1, 2, \dots , 2 n\tag{6.3.34a}
$$

$$
\hat {\mathbf {Z}} _ {k / k - 1} = \sum_ {i = 0} ^ {2 n} W _ {i} ^ {(m)} \mathbb {Z} _ {k / k - 1} ^ {(i)}\tag{6.3.34b}
$$

$W_{i}^{(m)}$ 和 $W_{i}^{(c)}$ 按式(6.3.7)计算。

按步骤 4' 和 5' 计算的一步预测样本点比按步骤 4 和 5 计算的少一半, 这对降低计算量是有利的, 但丢弃了原复现样本所保持的奇阶矩信息。

(2) 非可加性噪声条件下的 UKF 算法。设系统方程和量测方程具有一般形式：

$$
\boldsymbol {X} _ {k} = \boldsymbol {f} (\boldsymbol {X} _ {k - 1}, \boldsymbol {u} _ {k - 1}, \boldsymbol {W} _ {k - 1})\tag{6.3.35}
$$

$$
\mathbf {Z} _ {k} = \pmb {h} (\mathbf {X} _ {k}, \mathbf {V} _ {k})\tag{6.3.36}
$$

式中， $W_{k}$ 和 $V_{k}$ 为零均值不相关的高斯白噪声，方差阵分别为 $Q_{k}$ 和 $R_{k}$ 。设 $X_{k}, W_{k}, V_{k}$ 的维数分别为 n, q, p。

记增广状态

$$
\mathbf {X} _ {k} ^ {a} = \left[ \begin{array}{l} \mathbf {X} _ {k} \\ \mathbf {W} _ {k} \\ \mathbf {V} _ {k} \end{array} \right], \quad \mathbf {P} _ {k} ^ {a} = \left[ \begin{array}{c c c} \mathbf {P} _ {k} & \mathbf {0} & \mathbf {0} \\ \mathbf {0} & Q _ {k} & \mathbf {0} \\ \mathbf {0} & \mathbf {0} & R _ {k} \end{array} \right]
$$

则执行 UKF 滤波计算的步骤如下。

步骤 1, 确定滤波初值

由于 $E[\pmb{W}_k] = \mathbf{0}, E[\pmb{V}_k] = \mathbf{0}, E[\pmb{W}_k\pmb{W}_k^{\mathrm{T}}] = Q_k, E[\pmb{V}_k\pmb{V}_k^{\mathrm{T}}] = R_k$ ，所以滤波初值取为

$$
\hat {\mathbf {X}} _ {0} ^ {a} = \left[ \begin{array}{c} \hat {\mathbf {X}} _ {0} \\ \mathbf {0} \\ \mathbf {0} \end{array} \right], \quad \mathbf {P} _ {0} ^ {a} = \left[ \begin{array}{c c c} \mathbf {P} _ {0} & \mathbf {0} & \mathbf {0} \\ \mathbf {0} & Q _ {0} & \mathbf {0} \\ \mathbf {0} & \mathbf {0} & R _ {0} \end{array} \right]
$$

式中， $\hat{X}_{0}=E[X_{0}],P_{0}=E[(X_{0}-EX_{0})(X_{0}-EX_{0})^{\mathrm{T}}]$ 。

$$
k = 1, 2, 3, \dots
$$

步骤 2, 计算 1 倍 $\sigma$ 样本点

$$
\chi_ {k - 1} ^ {a (0)} = \hat {\mathbf {X}} _ {k - 1} ^ {a}\tag{6.3.37a}
$$

$$
\chi_ {k - 1} ^ {a (i)} = \hat {\mathbf {X}} _ {k - 1} ^ {a} + \gamma (\sqrt {\mathbf {P} _ {k - 1} ^ {a}}) _ {(i)} \quad i = 1, 2, \dots , L\tag{6.3.37b}
$$

$$
\chi_ {k - 1} ^ {a (i)} = \hat {\mathbf {X}} _ {k - 1} ^ {a} - \gamma (\sqrt {\mathbf {P} _ {k - 1} ^ {a}}) _ {(i - L)} \quad i = L + 1, L + 2, \dots , 2 L\tag{6.3.37c}
$$

其中

$$
L = n + q + p
$$

$$
\gamma = \sqrt {L + \lambda}
$$

$$
\chi_ {k - 1} ^ {a (i)} = \left[ \begin{array}{l} \chi_ {k - 1} ^ {X (i)} \\ \chi_ {k - 1} ^ {W (i)} \\ \chi_ {k - 1} ^ {V (i)} \end{array} \right], \quad i = 1, 2, \dots , 2 L
$$

$$
\boldsymbol {P} _ {k - 1} ^ {a} = \left[ \begin{array}{c c c} \boldsymbol {P} _ {k - 1} & \boldsymbol {0} & \boldsymbol {0} \\ \boldsymbol {0} & Q _ {k - 1} & \boldsymbol {0} \\ \boldsymbol {0} & \boldsymbol {0} & \boldsymbol {R} _ {k - 1} \end{array} \right]
$$

步骤 3: 时间更新:

$$
\chi_ {k / k - 1} ^ {X (i)} = f \left[ \chi_ {k - 1} ^ {X (i)}, u _ {k - 1}, \chi_ {k - 1} ^ {W (i)} \right], \quad i = 0, 1, 2, \dots , 2 L\tag{6.3.38a}
$$

$$
\hat {\mathbf {X}} _ {k / k - 1} = \sum_ {i = 0} ^ {2 L} W _ {i} ^ {(m)} \chi_ {k / k - 1} ^ {\mathbf {X} (i)}\tag{6.3.38b}
$$

$$
\boldsymbol {P} _ {k / k - 1} = \sum_ {i = 0} ^ {2 L} W _ {i} ^ {(c)} \left(\chi_ {k / k - 1} ^ {X (i)} - \hat {\boldsymbol {X}} _ {k / k - 1}\right) \left(\chi_ {k / k - 1} ^ {X (i)} - \hat {\boldsymbol {X}} _ {k / k - 1}\right) ^ {\mathrm{T}}\tag{6.3.38c}
$$

$$
\mathbb {Z} _ {k / k - 1} ^ {(i)} = h \left[ \chi_ {k / k - 1} ^ {\boldsymbol {X} (i)}, \chi_ {k / k - 1} ^ {\boldsymbol {V} (i)} \right] i = 0, 1, 2, 3, \dots , 2 L\tag{6.3.38d}
$$

$$
\hat {\mathbf {Z}} _ {k / k - 1} = \sum_ {i = 0} ^ {2 L} W _ {i} ^ {(m)} \mathbb {Z} _ {k / k - 1} ^ {(i)}\tag{6.3.38e}
$$

其中

$$
W _ {0} ^ {(m)} = \frac {\lambda}{L + \lambda}
$$

$$
W _ {0} ^ {(c)} = \frac {\lambda}{L + \lambda} + 1 - \alpha^ {2} + \beta
$$

$$
W _ {i} ^ {(m)} = W _ {i} ^ {(c)} = \frac {1}{2 (L + \lambda)} i = 1, 2, \dots , 2 L
$$

步骤 4: 量测更新:

$$
\boldsymbol {P} _ {(Z Z) _ {k / k - 1}} = \sum_ {i = 0} ^ {2 L} W _ {i} ^ {(c)} \left[ \mathbb {Z} _ {k / k - 1} ^ {(i)} - \hat {\mathbf {Z}} _ {k / k - 1} \right] \left[ \mathbb {Z} _ {k / k - 1} ^ {(i)} - \hat {\mathbf {Z}} _ {k / k - 1} \right] ^ {\mathrm{T}}
$$

(6.3.39a)

$$
\boldsymbol {P} _ {(X Z) _ {k / k - 1}} = \sum_ {i = 0} ^ {2 L} W _ {i} ^ {(c)} \left[ \chi_ {k / k - 1} ^ {\boldsymbol {X} (i)} - \hat {\boldsymbol {X}} _ {k / k - 1} \right] \left[ \mathbb {Z} _ {k / k - 1} ^ {(i)} - \hat {\mathbf {Z}} _ {k / k - 1} \right] ^ {\mathrm{T}}\tag{6.3.39b}
$$

$$
\boldsymbol {K} _ {k} = \boldsymbol {P} _ {(X Z) _ {k / k - 1}} \boldsymbol {P} _ {(Z Z) _ {k / k - 1}} ^ {- 1}\tag{6.3.39c}
$$

$$
\hat {\mathbf {X}} _ {k} = \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {K} _ {k} [ \mathbf {Z} _ {k} - \hat {\mathbf {Z}} _ {k / k - 1} ]\tag{6.3.39d}
$$

$$
\boldsymbol {P} _ {k} = \boldsymbol {P} _ {k / k - 1} - \boldsymbol {K} _ {k} \boldsymbol {P} _ {(Z Z) _ {k / k - 1}} \boldsymbol {K} _ {k} ^ {\mathrm{T}}\tag{6.3.39e}
$$

## 4. UKF 的基本特点

从上述分析可得出如下结论：

(1) UKF 本质上是线性最小方差估计的一种近似算法。近似体现在: 用随机量的条件均值代替均值, 使用被估计量和量测量的再现样本点计算两者的自协方差阵和互协方差阵。

(2) 若系统噪声、量测噪声和被估计量的初值都服从正态分布, 则 UKF 又是最小方差估计的一种近似算法。

（3）由于被估计量和量测量的再现样本点可直接根据系统方程和量测方程计算，所以UKF不必对系统方程和量测方程作线性化处理，与EKF相比，其优点是：避免了雅可比矩阵的求解；EKF只是非线性对象的一阶近似，二阶EKF是非线性对象的二阶近似，而UKF至少达到二阶近似，所以当系统方程和量测方程呈现强非线性时，EKF的线性化处理将引起严重的截断误差，导致滤波发散，而UKF无此问题，所以UKF能适用于强非线性条件下的估计，EKF只适用于弱非线性对象的估计。

（4）由于 UKF 是线性最小方差估计的一种近似算法，而标准卡尔曼滤波是精确算法，所以，在非线性系统的估计中宜采用 UKF，非线性越强，算法的优势就越明显；若系统的非线性较弱，则可采用 EKF 算法；而在线性系统估计中应采用标准卡尔曼滤波算法。

顺便对 UKF 中的 Unscented 的理解提出浅识供参考。在 UT 变换中需选定三个参数： $\alpha$ ， $\beta$ ， $\kappa$ 。尽管很多文献给出了选定这些参数的特定方法，如当状态变量服从正态分布时，选择 $\beta=2$ ，但这些参数的最佳值完全取决于所研究对象的具体特殊情况，其中的定量关系至今还未被完全理解和研究透。所以从这点上看，Unscented 具有无踪和无线索的含意。

## 6.4 非线性系统滤波之三: 粒子滤波 $^{[119,121]}$

粒子滤波直接根据概率密度计算条件均值，即最小方差估计，其中概率密度由 EKF 或 UKF 近似确定，k 时刻的估计值 $\hat{X}_{k}$ 由众多不同分布的样本值（粒子）加权平均确定，而计算每个粒子必须完成一次 EKF 或 UKF 计算，所以粒子滤波适用于系统和量测为非线性条件下的估计，且估计精度高于单独采用 EKF 或 UKF 时的精度，但计算量远高于 EKF 和 UKF。

粒子滤波有诸多别称，如：序列重要性采样算法SIS(Sequential Importance Sampling)[122]、自举滤波（Bootstrap Filtering)[123]、凝聚算法（Condensation Algorithm)[124,125]、交互粒子近似法(Interacting Particle Approximations)[126]、蒙特-卡洛滤波(Monte-Carlo Filtering)[127]、序贯蒙特-卡洛滤波(Sequential Monte-Carlo(SMC) Filtering)等[128,129]。粒子滤波的思想早在1940年已由Metropolies和维纳提出[130]，当时由于受计算条件的限制而并未得到广泛重视。20世纪80年代后，随着计算机技术的快速发展，粒子滤波计算成为可能，并逐渐受到重视。本节将对粒子滤波算法作详细推导，并介绍结合UKF算法的粒子滤波，即UPF。

## 6.4.1 条件均值的近似求解

设 $X_{0}^{k}=\{X_{0},X_{1},\cdots,X_{k}\}$ 为 $t_{0}$ 至 $t_{k}$ 时刻的系统状态， $Z_{0}^{k}=\{Z_{0},Z_{1},\cdots,Z_{k}\}$ 为 $t_{0}$ 至 $t_{k}$ 时刻的量测量，则根据式(2.1.27)，基于 $Z_{0}^{k}$ 对 $f(X_{0}^{k})$ 的最小方差估计，即条件均值为

$$
\hat {f} (\mathbf {X} _ {0} ^ {k}) = E [ \mathbf {f} (\mathbf {X} _ {0} ^ {k}) / \mathbf {Z} _ {0} ^ {k} ] = \int f (\mathbf {X} _ {0} ^ {k}) p (\mathbf {X} _ {0} ^ {k} / \mathbf {Z} _ {0} ^ {k}) \mathrm{d} \mathbf {X} _ {0} ^ {k}\tag{6.4.1}
$$

式中 $p(X_{0}^{k}/Z_{0}^{k})$ 为 $X_{0}^{k}$ 的条件概率密度。

根据贝叶斯公式和全概率公式,有

$$
p (\mathbf {X} _ {0} ^ {k} / \mathbf {Z} _ {0} ^ {k}) = \frac {p (\mathbf {X} _ {0} ^ {k}) p (\mathbf {Z} _ {0} ^ {k} / \mathbf {X} _ {0} ^ {k})}{p (\mathbf {Z} _ {0} ^ {k})}\tag{6.4.2}
$$

直接按式(6.4.1)求解 $f(X_{0}^{k})$ 的条件均值是十分困难的。为了绕过这一障碍，引入条件概率密度的推荐形式 $q(X_{0}^{k}/Z_{0}^{k})$ ，为叙述方便，称此推荐形式为推荐密度，为待定值。

将式 $(6.4.2)$ 代入式 $(6.4.1)$ ，并考虑其推荐密度，得

$$
\hat {f} (\mathbf {X} _ {0} ^ {k}) = \int f (\mathbf {X} _ {0} ^ {k}) \frac {p (\mathbf {X} _ {0} ^ {k}) p (\mathbf {Z} _ {0} ^ {k} / \mathbf {X} _ {0} ^ {k})}{p (\mathbf {Z} _ {0} ^ {k}) q (\mathbf {X} _ {0} ^ {k} / \mathbf {Z} _ {0} ^ {k})} q (\mathbf {X} _ {0} ^ {k} / \mathbf {Z} _ {0} ^ {k}) \mathrm{d} \mathbf {X} _ {0} ^ {k}
$$

记

$$
w _ {k} = \frac {p (\mathbf {X} _ {0} ^ {k}) p (\mathbf {Z} _ {0} ^ {k} / \mathbf {X} _ {0} ^ {k})}{q (\mathbf {X} _ {0} ^ {k} / \mathbf {Z} _ {0} ^ {k})}\tag{6.4.3}
$$

并称之为非归一化权重系数,则

$$
\begin{array}{l} \hat {f} (\mathbf {X} _ {0} ^ {k}) = \frac {1}{p (\mathbf {Z} _ {0} ^ {k})} \int f (\mathbf {X} _ {0} ^ {k}) w _ {k} q (\mathbf {X} _ {0} ^ {k} / \mathbf {Z} _ {0} ^ {k}) \mathrm{d} \mathbf {X} _ {0} ^ {k} = \frac {\int f (\mathbf {X} _ {0} ^ {k}) w _ {k} q (\mathbf {X} _ {0} ^ {k} / \mathbf {Z} _ {0} ^ {k}) \mathrm{d} \mathbf {X} _ {0} ^ {k}}{\int p (\mathbf {Z} _ {0} ^ {k} / \mathbf {X} _ {0} ^ {k}) p (\mathbf {X} _ {0} ^ {k}) \frac {q (\mathbf {X} _ {0} ^ {k} / \mathbf {Z} _ {0} ^ {k})}{q (\mathbf {X} _ {0} ^ {k} / \mathbf {Z} _ {0} ^ {k})} \mathrm{d} \mathbf {X} _ {0} ^ {k}} = \\ \frac {\int f (\mathbf {X} _ {0} ^ {k}) w _ {k} q (\mathbf {X} _ {0} ^ {k} / \mathbf {Z} _ {0} ^ {k}) \mathrm{d} \mathbf {X} _ {0} ^ {k}}{\int w _ {k} q (\mathbf {X} _ {0} ^ {k} / \mathbf {Z} _ {0} ^ {k}) \mathrm{d} \mathbf {X} _ {0} ^ {k}} = \frac {E _ {q} (\bullet / \mathbf {Z} _ {0} ^ {k}) [ w _ {k} f (\mathbf {X} _ {0} ^ {k}) ]}{E _ {q} (\bullet / \mathbf {Z} _ {0} ^ {k}) (w _ {k})} \end{array}\tag{6.4.4}
$$

式(6.4.4)中， $E_{q}(\cdot/\mathbf{Z}_{0}^{k})[\omega_{k}f(\mathbf{X}_{0}^{k})]$ 是随机量 $w_{k}f(\mathbf{X}_{0}^{k})$ 在推荐密度条件下求取的数学期望， $E_{q}(\cdot/\mathbf{Z}_{0}^{k})(\omega_{k})$ 是 $w_{k}$ 在推荐密度条件下求取的数学期望。

式(6.4.4)说明: $f(\mathbf{X}_{0}^{k})$ 基于 $Z_{0}^{k}$ 的条件均值可用推荐密度来求取,这就避开了对 $X_{0}^{k}$ 条件概率密度的求取要求,而推荐密度可以通过UKF来确定,这将在6.4.4小节中介绍。此外, $w_{k}$ 可用递推方法求取。

考察式(6.4.3)，由于不同时刻的量测相互独立， $Z_{0},Z_{1},\cdots,Z_{k-1}$ 与 $X_{k}$ 无关，而与 $X_{0}^{k-1}$ 有关， $Z_{k}$ 与 $X_{k}$ 有关，而与 $X_{0}^{k-1}$ 无关，所以

$$
p \left(\mathbf {Z} _ {0} ^ {k} / \mathbf {X} _ {0} ^ {k}\right) = p \left[ \left(\mathbf {Z} _ {0} ^ {k - 1}, \mathbf {Z} _ {k}\right) / \left(\mathbf {X} _ {0} ^ {k - 1}, \mathbf {X} _ {k}\right) \right] = p \left(\mathbf {Z} _ {0} ^ {k - 1} / \mathbf {X} _ {0} ^ {k - 1}\right) \cdot p \left(\mathbf {Z} _ {k} / \mathbf {X} _ {k}\right)\tag{6.4.5}
$$

又根据联合分布密度、边缘分布密度和条件分布密度间的关系，

$$
p \left(\boldsymbol {X} _ {0} ^ {k}\right) = p \left(\boldsymbol {X} _ {0} ^ {k - 1}, \boldsymbol {X} _ {k}\right) = p \left(\boldsymbol {X} _ {0} ^ {k - 1}\right) p \left(\boldsymbol {X} _ {k} / \boldsymbol {X} _ {0} ^ {k - 1}\right)
$$

假设系统状态为马尔柯夫链，即 $t_k$ 时刻的系统状态 $\mathbf{X}_k$ 仅与 $t_{k - 1}$ 时刻的状态 $\mathbf{X}_{k - 1}$ 有关，而与 $t_{k - 1}$ 时刻前的状态无关。则

$$
p (\mathbf {X} _ {0} ^ {k}) = p (\mathbf {X} _ {0} ^ {k - 1}) p (\mathbf {X} _ {k} / \mathbf {X} _ {k - 1})\tag{6.4.6}
$$

再考察推荐密度,根据联合分布、边缘分布和条件分布间的关系,且考虑到 $Z_{k}$ 与 $X_{0}, X_{1}$ ,

$\cdots,X_{k-1}$ 无关,所以有

$$
\begin{array}{r l} & q (\mathbf {X} _ {0} ^ {k} / \mathbf {Z} _ {0} ^ {k}) = q [ (\mathbf {X} _ {0} ^ {k - 1}, \mathbf {X} _ {k}) / \mathbf {Z} _ {0} ^ {k} ] = q (\mathbf {X} _ {0} ^ {k - 1} / \mathbf {Z} _ {0} ^ {k}) q [ \mathbf {X} _ {k} / (\mathbf {X} _ {0} ^ {k - 1}, \mathbf {Z} _ {0} ^ {k}) ] = \\ & \qquad q (\mathbf {X} _ {0} ^ {k - 1} / \mathbf {Z} _ {0} ^ {k - 1}) q [ \mathbf {X} _ {k} / (\mathbf {X} _ {0} ^ {k - 1}, \mathbf {Z} _ {0} ^ {k}) ] \end{array}\tag{6.4.7}
$$

将式 $(6.4.5)$ 、式 $(6.4.6)$ 、式 $(6.4.7)$ 代入式 $(6.4.3)$ ，得

$$
\begin{array}{r l} \mathcal {w} _ {k} & = \frac {p (\mathbf {X} _ {0} ^ {k}) p (\mathbf {Z} _ {0} ^ {k} / \mathbf {X} _ {0} ^ {k})}{q (\mathbf {X} _ {0} ^ {k} / \mathbf {Z} _ {0} ^ {k})} = \frac {p (\mathbf {X} _ {0} ^ {k - 1}) p (\mathbf {Z} _ {0} ^ {k - 1} / \mathbf {X} _ {0} ^ {k - 1}) p (\mathbf {X} _ {k} / \mathbf {X} _ {k - 1}) p (\mathbf {Z} _ {k} / \mathbf {X} _ {k})}{q (\mathbf {X} _ {0} ^ {k - 1} / \mathbf {Z} _ {0} ^ {k - 1}) q [ \mathbf {X} _ {k} / (\mathbf {X} _ {0} ^ {k - 1} , \mathbf {Z} _ {0} ^ {k}) ]} = \\ & \quad \mathcal {w} _ {k - 1} \frac {p (\mathbf {X} _ {k} / \mathbf {X} _ {k - 1}) p (\mathbf {Z} _ {k} / \mathbf {X} _ {k})}{q [ \mathbf {X} _ {k} / (\mathbf {X} _ {0} ^ {k - 1} , \mathbf {Z} _ {0} ^ {k}) ]} \end{array}\tag{6.4.8}
$$

该式即为非归一化权重系数的递推公式，其中 $p(\mathbf{X}_k / \mathbf{X}_{k - 1})$ 由系统方程确定， $p(\mathbf{Z}_k / \mathbf{X}_k)$ 由量测方程确定， $\omega_0$ 按式(6.4.3）确定。

推荐密度确定后,可按该密度函数计算出 N 组状态复现样本簇:

$$
\chi_ {0} ^ {k} (i) = \left\{\chi_ {0} ^ {(i)}, \chi_ {1} ^ {(i)}, \dots , \chi_ {k} ^ {(i)} \right\} \quad i = 1, 2, \dots , N
$$

式中 $\chi_{k}^{(i)}$ 为按密度函数生成的分布样本点，此处形象化地称之为粒子。式(6.4.8)中的状态量 $X_{k}, X_{k-1}, X_{0}^{k-1}$ 可用这些粒子来近似，从而可近似计算出非归一化权重系数，再按式(6.4.4)计算出估计值。

$$
w _ {k} ^ {(i)} = w _ {k - 1} ^ {(i)} \frac {p \left[ \chi_ {k} ^ {(i)} / \chi_ {k - 1} ^ {(i)} \right] p \left[ Z _ {k} / \chi_ {k} ^ {(i)} \right]}{q \left[ \chi_ {k} ^ {(i)} / (\chi_ {0} ^ {k} (i) , Z _ {0} ^ {k}) \right]}\tag{6.4.9}
$$

$$
\hat {f} (\mathbf {X} _ {0} ^ {k}) \approx \frac {N ^ {- 1} \sum_ {i = 1} ^ {N} f (\chi_ {0} ^ {k} (i)) w _ {k} ^ {(i)}}{N ^ {- 1} \sum_ {i = 1} ^ {N} w _ {k} ^ {(i)}} = \sum_ {i = 1} ^ {N} f (\chi_ {0} ^ {k} (i)) \widetilde {w} _ {k} ^ {(i)}\tag{6.4.10}
$$

其中

$$
\widetilde {w} _ {k} ^ {(i)} = \frac {w _ {k} ^ {(i)}}{\sum_ {j = 1} ^ {N} w _ {k} ^ {(j)}}\tag{6.4.11}
$$

为归一化权重系数。很明显：

$$
\sum_ {i = 1} ^ {N} \widetilde {\omega} _ {k} ^ {(i)} = 1\tag{6.4.12}
$$

如果取 $f(X_{0}^{k})=X_{k}$ ，则以上讨论的问题即为对 $X_{k}$ 的估计问题。

按式(6.4.10)计算估计值实质上是对复现的分布样本点即粒子上的函数值作加权平均，该算法称为SIS(Sequential Importance Sampling)算法。

## 6.4.2 二次采样

SIS算法存在致命缺陷:按式(6.4.8)和式(6.4.11)计算的权重系数的方差随时间而增加,若干步递推后,某些粒子的权重系数趋于1,而另一些粒子的权重系数则趋于0,一大批分布样本点会被丢弃,这种现象称为粒子退化。为避免粒子退化现象,Efron,Rubin和Smith等提出了二次采样算法SIR(Sampling-Importance Resampling) $^{[140-142]}$ 。具体为:

步骤1: 在第i次二次采样过程中 $(i=1,2,\cdots,N,N$ 为粒子总数 $)$ ，首先生成在 $[0,1]$ 区间内均匀分布的随机数 $r_{i}$ ;

步骤 2: 计算归一化权重系数的累加值, 每次累加只顺序增加一项归一化权重系数, 每次

所得累加结果与 $r_i$ 比较，若

$$
\sum_ {m = 1} ^ {j - 1} \widetilde {\omega} _ {k} ^ {(m)} <   r _ {i}
$$

$$
\sum_ {m = 1} ^ {j} \widetilde {\omega} _ {k} ^ {(m)} \geqslant r _ {i}
$$

则该次二次采样结果为

$$
\chi_ {k} ^ {+ (i)} = \chi_ {k} ^ {(j)} \quad (i, j = 1, 2, \dots , N)
$$

式中 $\chi_{k}^{(j)}$ 为二次采样前的第 j 个原始粒子， $\chi_{k}^{+(i)}$ 为二次采样所得第 i 个更新粒子。

图 6.4.1 为粒子数 N=10 的 SIR 二次采样示意图。图中，横轴所标 1,2,…,10 为原始粒子序号，纵轴为归一化权重系数的累加结果，数值范围为 0 至 1，实线长度代表相应序号粒子的归一化权重系数。假设在执行第 9 次采样时所生成的均匀分布随机数 $r_{9}=0.3$ ，由于 $\widetilde{w}_{k}^{(1)}+\widetilde{w}_{k}^{(2)}+\widetilde{w}_{k}^{(3)}<r_{9},\widetilde{w}_{k}^{(1)}+\widetilde{w}_{k}^{(2)}+\widetilde{w}_{k}^{(3)}+\widetilde{w}_{k}^{(4)}>r_{9}$ ，所以 $\chi_{k}^{+(9)}=\chi_{k}^{(4)}$ 。

![](images/3a2cd3c908c839b24e679f44be013cd5b6ca5af7f3a69031950f6cb72f457f8a.jpg)  
图6.4.1 SIR二次采样

二次采样实质上将原始粒子映射成均匀分布的更新粒子,每个更新粒子的概率密度相同,所以权重系数均为 $\frac{1}{N}$ 。

由于 $\widetilde{w}_k^{(i)}$ 大小不一，所以原始粒子被二次采样选作更新粒子的次数也不一样，例如图6.4.1中 $\widetilde{w}_k^{(4)}$ 很大，而 $\widetilde{w}_k^{(6)}$ 很小，则 $\chi_k^{(4)}$ 可能多次被选作更新粒子，而 $\chi_k^{(6)}$ 可能一次也没被选中，这样还会有一批原始粒子被白白丢弃。针对此问题，MacKay和Higuchi提出了残差二次采样算法(Residual Resampling)[143,144]，该算法分两步执行：

第一步，对原始粒子 $\chi_{k}^{(i)}$ 以 $N_{k_{1}}^{(i)}$ 为重复次数进行采样，其中

$$
N _ {k _ {1}} ^ {(i)} = [ \widetilde {N w} _ {k _ {1}} ^ {(i)} ]\tag{6.4.13}
$$

[·]表示取整, $\widetilde{w}_{k_{1}}^{(i)}$ 按式(6.4.9)式和式(6.4.11)确定。如此采样后获得第一部分二次采样更新粒子,粒子个数为

$$
N _ {k _ {1}} = \sum_ {i = 1} ^ {N} N _ {k _ {1}} ^ {(i)}\tag{6.4.14}
$$

第二步,由于取整过程中舍弃了小数部分,所以要获得 N 个二次采样更新粒子,还需补充 $N_{k_{2}} = N - N_{k_{1}}$ 个粒子。具体方法如下。

## (1) 计算新的权重系数

$$
\widetilde {w} _ {k _ {2}} ^ {(i)} = N _ {k _ {2}} ^ {- 1} \left[ \widetilde {w} _ {k _ {1}} ^ {(i)} N - N _ {k _ {1}} ^ {(i)} \right] \quad i = 1, 2, \dots , N
$$

(2) 根据 $\widetilde{w}_{k_{2}}^{(i)}$ 按 SIR 算法对原始粒子再作二次采样，确定出第二部分二次采样更新粒子。记在该次采样中对原始粒子 $\chi_{k}^{(i)}$ 的重复采集次数为 $N_{k_{2}}^{(i)}$ ，则在获得两部分更新粒子过程中对 $\chi_{k}^{(i)}$ 的总采集次数为 $N_{k}^{(i)} = N_{k_{1}}^{(i)} + N_{k_{2}}^{(i)}$ ，第二部分中总的更新粒子数为 $N_{k_{2}} = \sum_{i=1}^{N} N_{k_{2}}^{(i)}$ 。

（3）将第一部分和第二部分二次采样粒子合并在一起，即为二次采样粒子全体，各粒子的新权重系数都为 $\frac{1}{N}$ 。

## 6.4.3 粒子滤波一般形式

由 6.4.1 节和 6.4.2 节可综合出一般形式粒子滤波的执行步骤如下。

步骤 1, 确定初值。

根据初始状态 $X_{0}$ 的先验概率密度 $p(X_{0})$ 生成粒子初始值 $\chi_{0}^{(i)}(i=1,2,\cdots,N)$ 。

对于 $k=1,2,3,\cdots$ 执行：

步骤2，选定推荐概率密度 $q[X_{k}/(X_{0}^{k}(i),Z_{0}^{k})]$ ，并根据此推荐密度生成k时刻的粒子 $\chi_{k}^{(i)}$ ， $i=1,2,\cdots,N$ ，作为二次采样的原始粒子。

计算权重系数

$$
\begin{array}{r l} w _ {k} ^ {(i)} = w _ {k - 1} ^ {(i)} & \frac {p [ \mathbf {Z} _ {k} / \boldsymbol {\chi} _ {k} ^ {(i)} ] p [ \boldsymbol {\chi} _ {k} ^ {(i)} / \boldsymbol {\chi} _ {k - 1} ^ {(i)} ]}{q [ \boldsymbol {\chi} _ {k} ^ {(i)} / (\boldsymbol {\chi} _ {0} ^ {k} (i) , \mathbf {Z} _ {0} ^ {k}) ]} \\ w _ {0} ^ {(i)} = p (\boldsymbol {\chi} _ {0} ^ {(i)}) \\ \widetilde {w} _ {k} ^ {(i)} = w _ {k} ^ {(i)} \left[ \sum_ {j = 1} ^ {N} w _ {k} ^ {(j)} \right] ^ {- 1}, & i = 1, 2, \dots , N \end{array}\tag{6.4.15}
$$

(6.4.16)

步骤 3, 采用 SIR 法或残差二次采样法, 对原始粒子 $\chi_{k}^{(i)} (i=1,2,\cdots,N)$ 作二次采样, 生成二次采样更新粒子 $\chi_{k}^{+(j)} (j=1,2,\cdots,N)$ , 每个粒子的权重系数为 $\frac{1}{N}$ 。

步骤 4, 根据二次采样粒子计算滤波值

$$
\hat {\boldsymbol {X}} _ {k} = \frac {1}{N} \sum_ {j = 1} ^ {N} \boldsymbol {\chi} _ {k} ^ {+ (j)}\tag{6.4.17}
$$

## 6.4.4 基于UKF的粒子滤波——UPF

粒子滤波的核心是合理选择推荐概率密度,推荐密度选择得与真实密度越接近,滤波效果就越好,反之则越差,甚至发散。如果粒子滤波与UKF结合起来,推荐密度由UKF来确定,则既可解决粒子退化的问题,又能使粒子更新时获得量测量的最新验后信息,有利于粒子移向似然比高的区域。结合 UKF 的粒子滤波称为 UPF。

设系统方程和量测方程分别为

$$
\boldsymbol {X} _ {k} = \boldsymbol {f} (\boldsymbol {X} _ {k - 1}, \boldsymbol {u} _ {k - 1}) + \boldsymbol {W} _ {k - 1}\tag{6.4.18}
$$

$$
\mathbf {Z} _ {k} = \mathbf {h} (\mathbf {X} _ {k}) + \mathbf {V} _ {k}\tag{6.4.19}
$$

式中 $W_{k}$ 和 $V_{k}$ 为互不相关的高斯分布白噪声，均值为零，方差阵分别为 $Q_{k}$ 和 $R_{k}$ ，则 UPF 执行步骤如下。

步骤 1, 确定初值。

设 $X_{0}$ 的先验概率密度为 $p(X_{0})$ ，方差阵为 $P_{0}$ 。根据 $p(X_{0})$ 生成粒子初值 $\chi_{0}^{(i)}$ ，并选取 $w_{0}^{(i)} = p(\chi_{0}^{(i)}), i = 1, 2, \cdots, N$ 。为简化计算，将每个粒子的分布密度近似取正态分布 $N(\overline{\chi}_{0}^{(i)}, P_{0}^{i})$ ，其中 $\overline{\chi}_{0}^{(i)} = \chi_{0}^{(i)}, P_{0}^{i} = P_{0}$ 。

对于 $i=1,2,\cdots,N,k=1,2,3,\cdots$ 执行：

步骤 2, UKF 滤波计算。

(1) 计算 $\sigma$ 样本点

$$
\chi_ {(0) k - 1} ^ {(i)} = \overline {{\chi}} _ {k - 1} ^ {(i)}\tag{6.4.20a}
$$

$$
\chi_ {(j) k - 1} ^ {(i)} = \overline {{\chi}} _ {k - 1} ^ {(i)} + \gamma (\sqrt {\boldsymbol {P} _ {k - 1} ^ {(i)}}) _ {(j)} \quad j = 1, 2, \dots , n\tag{6.4.20b}
$$

$$
\chi_ {(j) k - 1} ^ {(i)} = \bar {\chi} _ {k - 1} ^ {(i)} - \gamma (\sqrt {\mathbf {P} _ {k - 1} ^ {(i)}}) _ {(j - n)} \quad j = n + 1, n + 2, \dots , 2 n\tag{6.4.20c}
$$

式中， $\gamma=\sqrt{n+\lambda}$ ， $\lambda$ 按式(6.3.7d)确定，n 为状态维数。

(2) 时间更新

$$
\chi_ {(j) k / k - 1} ^ {(i)} = f \left(\chi_ {(j) k - 1} ^ {(i)}, u _ {k - 1}\right)\tag{6.4.21}
$$

$$
\overline {{{{\boldsymbol {\chi}}}}} _ {k / k - 1} ^ {(i)} = \sum_ {j = 0} ^ {2 n} W _ {j} ^ {(m)} \boldsymbol {\chi} _ {(j) k / k - 1} ^ {(i)}\tag{6.4.22}
$$

$$
\boldsymbol {P} _ {k / k - 1} ^ {(i)} = \sum_ {j = 0} ^ {2 n} W _ {j} ^ {(c)} \left(\boldsymbol {\chi} _ {(j) k / k - 1} ^ {(i)} - \overline {{\boldsymbol {\chi}}} _ {k / k - 1} ^ {(i)}\right) \left(\boldsymbol {\chi} _ {(j) k / k - 1} ^ {(i)} - \overline {{\boldsymbol {\chi}}} _ {k / k - 1} ^ {(i)}\right) ^ {\mathrm{T}} + \boldsymbol {Q} _ {k - 1}\tag{6.4.23}
$$

$$
\mathbf {Z} _ {(j) k / k - 1} ^ {(i)} = \boldsymbol {h} (\boldsymbol {\chi} _ {(j) k / k - 1} ^ {(i)})\tag{6.4.24}
$$

$$
\overline {{{{\mathbf {Z}}}}} _ {k / k - 1} ^ {(i)} = \sum_ {j = 0} ^ {2 n} W _ {j} ^ {(m)} \mathbf {Z} _ {(j) k / k - 1} ^ {(i)}\tag{6.4.25}
$$

式中

$$
W _ {0} ^ {(m)} = \frac {\lambda}{n + \lambda}\tag{6.4.26a}
$$

$$
W _ {0} ^ {(c)} = \frac {\lambda}{n + \lambda} + 1 - \alpha^ {2} + \beta\tag{6.4.26b}
$$

$$
W _ {j} ^ {(m)} = W _ {j} ^ {(c)} = \frac {1}{2 (n + \lambda)} \quad j = 1, 2, \dots , 2 n\tag{6.4.26c}
$$

$\lambda=\alpha^{2}(n+\kappa)-n,10^{-4}\leqslant\alpha\leqslant1,\kappa=3-n$ ，对正态分布 $\beta=2$ 。

(3) 量测更新

$$
\boldsymbol {P} _ {(Z Z) k / k - 1} ^ {(i)} = \sum_ {j = 0} ^ {2 n} W _ {j} ^ {(c)} (\boldsymbol {Z} _ {(j) k / k - 1} ^ {(i)} - \overline {{\boldsymbol {Z}}} _ {k / k - 1} ^ {(i)}) (\boldsymbol {Z} _ {(j) k / k - 1} ^ {(i)} - \overline {{\boldsymbol {Z}}} _ {k / k - 1} ^ {(i)}) ^ {\mathrm{T}} + \boldsymbol {R} _ {k}\tag{6.4.27}
$$

$$
\boldsymbol {P} _ {(X Z) k / k - 1} ^ {(i)} = \sum_ {j = 0} ^ {2 n} W _ {j} ^ {(c)} (\boldsymbol {\chi} _ {(j) k / k - 1} ^ {(i)} - \overline {{\boldsymbol {\chi}}} _ {k / k - 1} ^ {(i)}) (\boldsymbol {Z} _ {(j) k / k - 1} ^ {(i)} - \overline {{\boldsymbol {Z}}} _ {k / k - 1} ^ {(i)}) ^ {\mathrm{T}}
$$

(6.4.28)

$$
\boldsymbol {K} _ {k} ^ {(i)} = \boldsymbol {P} _ {(X Z) k / k - 1} ^ {(i)} (\boldsymbol {P} _ {(Z Z) k / k - 1} ^ {(i)}) ^ {- 1}\tag{6.4.29}
$$

$$
\overline {{{{\boldsymbol {\chi}}}}} _ {k} ^ {(i)} = \overline {{{{\boldsymbol {\chi}}}}} _ {k / k - 1} ^ {(i)} + \mathbf {K} _ {k} ^ {(i)} \left(\mathbf {Z} _ {k} - \overline {{{{\mathbf {Z}}}}} _ {k / k - 1} ^ {(i)}\right)\tag{6.4.30}
$$

$$
\boldsymbol {P} _ {k} ^ {(i)} = \boldsymbol {P} _ {k / k - 1} ^ {(i)} - \boldsymbol {K} _ {k} ^ {(i)} \boldsymbol {P} _ {(Z Z) k / k - 1} ^ {(i)} (\boldsymbol {K} _ {k} ^ {(i)}) ^ {\mathrm{T}}\tag{6.4.31}
$$

步骤 3, 选取推荐密度

$$
q \left(\boldsymbol {X} _ {k} ^ {(i)} / \left(\boldsymbol {X} _ {0} ^ {k} (i), \boldsymbol {Z} _ {0} ^ {k}\right)\right) \approx N \left(\overline {{{\chi}}} _ {k} ^ {(i)}, \boldsymbol {P} _ {k} ^ {(i)}\right)\tag{6.4.32}
$$

并由该推荐密度生成粒子 $\chi_{k}^{(i)}$ ，作为二次采样的原始粒子。

步骤 4, 计算权重系数

$$
w _ {k} ^ {(i)} = w _ {k - 1} ^ {(i)} \frac {p \left(\mathbf {Z} _ {k} / \boldsymbol {\chi} _ {k} ^ {(i)}\right) p \left(\boldsymbol {\chi} _ {k} ^ {(i)} / \boldsymbol {\chi} _ {k - 1} ^ {(i)}\right)}{p \left(\boldsymbol {\chi} _ {k} ^ {(i)}\right)}\tag{6.4.33}
$$

式中， $p(\boldsymbol{\chi}_{k}^{(i)})$ 根据概率密度 $N(\overline{\boldsymbol{\chi}}_{k}^{(i)},\boldsymbol{P}_{k}^{(i)})$ 计算， $p(\boldsymbol{\chi}_{k}^{(i)}/\boldsymbol{\chi}_{k-1}^{(i)})$ 根据概率密度 $N(f(\overline{\boldsymbol{\chi}}_{k-1}^{(i)},\boldsymbol{u}_{k-1}),\boldsymbol{Q}_{k-1})$ 计算， $p(\mathbf{Z}_{k}/\boldsymbol{\chi}_{k}^{(i)})$ 根据概率密度 $N(\boldsymbol{h}(\overline{\boldsymbol{\chi}}_{k}^{(i)}),\boldsymbol{R}_{k})$ 计算。

步骤5,采用SIR法或残差二次采样法对原始粒子 $\chi_{k}^{(i)}(i=1,2,\cdots,N)$ 作二次采样,生成二次采样更新粒子 $\chi_{k}^{+(j)}(j=1,2,\cdots,N)$ ,每个二次采样更新粒子的权重系数均为 $\frac{1}{N}$ 。

步骤 6, 根据二次采样更新粒子计算滤波值

$$
\hat {\mathbf {X}} _ {k} = \frac {1}{N} \sum_ {j = 1} ^ {N} \boldsymbol {\chi} _ {k} ^ {+ (j)}\tag{6.4.34}
$$

在 UPF 算法中, 每计算一个更新粒子 $\chi_{k}^{+(j)}$ , 就要执行一次 UKF 计算, 若粒子滤波中有 N 个更新粒子参与加权平均以获得 $\hat{X}_{k}$ (见式(6.4.34)), 则每作一次 UPF 滤波需要执行 N 次 UKF 计算, 计算量是十分巨大的。

按式(6.4.15)计算粒子滤波权重系数时,需要知道条件密度 $p(\mathbf{Z}_{k}/\boldsymbol{\chi}_{k}^{(i)})$ 和 $p(\boldsymbol{\chi}_{k}^{(i)}/\boldsymbol{\chi}_{k-1}^{(i)})$ 。在 UPF 中,这些量按如下方法确定。

根据式(6.4.18)和式(6.4.19)，由于系统噪声和量测噪声都是可加性高斯白噪声，所以有

$$
p \left(\boldsymbol {X} _ {k} / \boldsymbol {X} _ {k - 1}\right) = N \left(\boldsymbol {f} \left(\bar {\boldsymbol {X}} _ {k - 1}, \boldsymbol {u} _ {k - 1}\right), \boldsymbol {Q} _ {k - 1}\right)
$$

即该条件密度服从正态分布，均值为 $f(\overline{X}_{k - 1},\pmb {u}_{k - 1})$ ，方差阵为 $Q_{k - 1}$ 。

同理有

$$
p \left(\mathbf {Z} _ {k} / \mathbf {X} _ {k}\right) = N \left(\boldsymbol {h} \left(\bar {\mathbf {X}} _ {k}\right), \boldsymbol {R} _ {k}\right)
$$

因此， $p(\chi_{k}^{(i)}/\chi_{k-1}^{(i)})$ 按概率密度 $N(f(\chi_{k-1}^{(i)},u_{k-1}),Q_{k-1})$ 计算； $p(Z_{k}/\chi_{k}^{(i)})$ 根据概率密度 $N(h(\overline{\chi}_{k}^{(i)}),R_{k})$ 计算。这就是 UPF 步骤 4 中用到的关系。

上述诸式中， $N$ 表示正态分布， $\pmb{\chi}_k^{(i)}$ 服从的分布为 $N(\overline{\pmb{\chi}}_k^{(i)},\pmb{P}_k^{(i)})$ ， $\mathbf{Z}_k$ 为 $k$ 时刻的量测。

粒子滤波是基于贝叶斯估计导出的。只有在对象为非线性且噪声不满足高斯分布的条件下，粒子滤波在估计精度上的优势才能充分显现出来，并且其计算量比UKF的计算量还要大。与UKF相比，粒子滤波也是通过非线性变换求取样本点的，再用这些样本点合成被估计状态的均值和方差阵。但UKF基于特定算法来选择这些样本点即求取加权系数，而粒子滤波对样本点的选择是随机的，所需的样本点个数也远多于UKF。此外，UKF的估计误差不会收敛于零，而粒子滤波会，条件是使用的粒子数趋于无穷多。粒子滤波使用的粒子数越多，估计精度就越高，当然要付出巨大计算量的代价。

对于被处理对象为非线性且噪声并不服从高斯分布的情况,粒子滤波的估计精度最高,计算量最大,UKF次之,EKF再次之。但对于被处理对象为线性且噪声服从高斯分布的情况,离散型卡尔曼滤波基本方程在估计精度和计算量上都具有优势,应该作为首选算法。因为在此情况下,即使采用粒子滤波或 UKF,估计精度也不可能有所提高,而付出的高额计算量徒劳无益。因此,在选用粒子滤波时,必须对适用对象、估计精度及计算量三者作权衡。

## 习题

## 6-1 设有连续系统数学模型

$$
\left\{ \begin{array}{l} \dot {\mathbf {X}} (t) = \mathbf {f} [ \mathbf {X} (t), t ] + \mathbf {B} (t) \mathbf {u} (t) + \mathbf {G} (t) \mathbf {w} (t) \\ \mathbf {Z} (t) = \mathbf {h} [ \mathbf {X} (t), t ] + \mathbf {v} (t) \end{array} \right.
$$

试对该系统先离散后线性化，并推导离散型线性化卡尔曼滤波方程和广义卡尔曼滤波方程。

## 6-2 设有一维离散系统

$$
\left\{ \begin{array}{l} x _ {k} = a x _ {k - 1} + w _ {k - 1} \\ z _ {k} = x _ {k} + v _ {k} \end{array} \right.
$$

其中 $\{w_{k-1}\}$ 和 $\{v_{k}\}$ 是互不相关的，并且是与初始状态 $x_{0}$ 不相关的零均值白噪声序列。它们的方差分别为 $Q_{k-1}$ 和 $R_{k}$ 。a为未知参数。试通过量测值 $z_{k}$ 同时进行滤波并估计参数a。

6-3 计算机仿真练习题:地球卫星的平面轨道模型可描述为

$$
\begin{array}{r l} \ddot {r} & = r \dot {\theta} ^ {2} - \frac {G M}{r ^ {2}} + w \\ \ddot {\theta} & = - \frac {2 \dot {\theta} \dot {r}}{r} \end{array}
$$

其中向径 r 为卫星至地心的距离，角距 $\theta$ 为向径 r 与节线（轨道平面与赤道平面的交线）间的夹角，引力常数 $G=6.6742\times10^{-11}m^{3}/kg/s^{2}$ ，地球质量 $M=5.98\times10^{24}kg, w$ 为高斯型白噪声干扰，均值为 0，方差强度为 $10^{-6}m^{2}/s^{3}$ ，该干扰由空间碎片、高空大气阻力、卫星气密部件漏气等因素引起。假设地面测轨系统每隔 15 秒测得卫星的向径 r 和角距 $\theta$ ，测量误差均为高斯型白噪声，均方根分别为 100m 和 0.1rad。

按如下要求完成仿真计算。仿真过程中，取卫星过升交点（向径与节线重合）时为时间起点 $(t=0)$ ，假设此时卫星高度为200km，线速度为7.9km/s，地球近似视为圆球，半径R=6371km。

(1) 根据轨道模型, 计算 1 小时内卫星在干扰项 w 的一个样本激励下, $r, \dot{r}, \theta, \dot{\theta}$ 的真值;

(2) 根据所得量测值, 分别用 EKF 和二阶 EKF 估计出 $r, \dot{r}, \theta, \dot{\theta}$ , 比较计算两种滤波算法的滤波误差并绘出相应曲线。

