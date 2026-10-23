# 第7章 容错组合导航的设计理论

# 第七章 容错组合导航的设计理论

## 7.1 概述

现代科学技术已能给航行体提供了多种导航设备，如惯性导航系统、全球定位系统（GPS）、多普勒导航雷达、奥米加（Omega）导航系统、罗兰系统（Long Range，简写为Loran）、塔康系统（Tactical Air Navigation，简写为Tacan），还有天文导航系统、地形辅助系统等。这些导航设备都各有优缺点，精度和成本也大不相同。惯性导航（以下简称“惯导”）系统一般作为主要导航设备，因为它是一种自主式导航系统，并能输出多种导航参数（位置、速度、航向、姿态等）。在初始条件正确的情况下，惯导系统的短期精度较高。惯导系统的缺点是，它的误差是随时间积累的。如果要提高惯导系统的长期精度，就须要提高惯性器件的精度和初始对准的精度，这必将大大提高成本。多普勒导航雷达可测量航行体的地速和偏流角。它和惯导系统一样，同属于航位推算（Dead Reckoning）式导航，从地速去推算位置，因此位置误差也是积累的。GPS是一种先进的导航设备，它的定位精度极高，也可给出速度信号和姿态信号。但GPS有下面的缺点：GPS导航不是自主式导航，主动权不在我们手中，卫星信号可能被人为地故意加上干扰；GPS星座在24小时内对地球的覆盖不够完善，有时候收不到所需的4颗卫星的信号；GPS的数据更新率太低（一般一秒一次），不能满足实时控制的要求；另外，卫星信号可能被遮蔽（如高山，飞机机翼遮蔽等）。其他无线电定位导航设备如罗兰-C、塔康等其定位精度较低，但其误差不随时间增长。

将惯导系统与其他导航系统适当地组合起来,可取长补短,大大提高导航精度。组合导航中的惯导系统的精度可比单独使用惯导系统时要求的精度低,因此可大大降低惯导系统的成本。

组合导航系统还可提高系统的任务可靠性和容错性能。因为组合导航中有余度的导航信息，如组合适当，则可利用余度信息检测出某导航子系统的故障，将此失效的子系统隔离掉，并将剩下的正常的子系统重新组合（系统重构），就可继续完成导航任务。组合导航系统还可协助惯导系统进行空中对准和校准，从而提高飞机的快速反应能力。

组合导航已有很长的历史。20世纪60年代以前，组合方法一般采用频率滤波或经典自动控制中的校正方法。20世纪60年代以后，一般采用卡尔曼滤波技术最优地组合各个导航系统的信息，估计导航系统的误差状态，再用误差状态的最优估计值去校正系统以提高组合系统的导航精度。

利用卡尔曼滤波技术对组合导航系统进行最优组合有两种途径:一种是集中式卡尔曼滤波,另一种是分散化卡尔曼滤波。集中式卡尔曼滤波是利用一个卡尔曼滤波器来集中地处理所有导航子系统的信息。集中式卡尔曼滤波虽然在理论上可给出误差状态的最优估计,但它存在下述缺点。

（1）集中式卡尔曼滤波器的状态维数高，因而计算负担重，不利于滤波的实时运行。这是因为集中式卡尔曼滤波器包含了各个子系统的误差状态，例如除了公共的三个位置误差、三个速度误差、三个姿态误差之外，还有惯导系统中陀螺和加速度计的误差状态、GPS的误差状态、多普勒导航雷达的误差状态以及其他子系统的误差状态等。状态的高维数会带来所谓“维数灾难”，使计算负担急剧增加。降维滤波又会损失滤波精度，甚至带来滤波发散。

（2）集中式卡尔曼滤波器的容错性能差，不利于故障诊断。这是因为任一导航子系统的故障在集中式滤波器中会污染其他状态，使组合系统输出的导航信息不可靠。

分散化滤波已发展了20多年。1971年Pearson $^{[107]}$ 就提出了动态分解的概念和状态估计两级结构。以后，Speyer $^{[34]}$ ，Willsky $^{[36]}$ ，Bierman $^{[40]}$ ，Kerr $^{[39]}$ 和Carlson $^{[42]}$ 等都对分散化滤波技术作出了贡献。在众多的分散化滤波方法中，Carlson提出的联邦滤波器（Federated Filter），由于设计的灵活性、计算量小、容错性能好而受到了重视。现在，联邦滤波器已被美国空军的容错导航系统“公共卡尔曼滤波器”计划选为基本算法 $^{[43]}$ 。

## 7.2 联邦滤波器算法原理

在本节中,将先介绍一种简单的分散化滤波算法。这种算法将作为一个比较基础,由此可以更清楚地了解联邦滤波器的特点和所要解决的问题。接着将介绍各子滤波估计不相关时全局滤波的融合算法和各子滤波估计相关时全局滤波的融合算法,最后介绍联邦滤波器的几种结构形式。

## 7.2.1 联邦滤波要解决的问题

为了说清楚联邦滤波的特点和所要解决的问题,下面介绍一种比较简单的分散化滤波方法,它的局部滤波和全局滤波都是最优的 $^{[108]}$ 。

假定系统的状态方程和量测方程为

$$
\left. \begin{array}{l} \mathbf {X} _ {k} = \boldsymbol {\Phi} _ {k, k - 1} \mathbf {X} _ {k - 1} + \mathbf {W} _ {k - 1} \\ \mathbf {Z} _ {k} = \mathbf {H} _ {k} \mathbf {X} _ {k} + \mathbf {V} _ {k} \end{array} \right\}\tag{7.2.1}
$$

式中 $W_{k}$ 的协方差阵为 $Q_{k}, V_{k}$ 的协方差阵为 $R_{k}$ 。

子系统的状态方程和量测方程为

$$
\left. \begin{array}{l} \mathbf {X} _ {i k} = \boldsymbol {\Phi} _ {k, k - 1} ^ {i} \mathbf {X} _ {i, k - 1} + \mathbf {W} _ {i, k - 1} \\ \mathbf {Z} _ {i k} = \mathbf {A} _ {i k} \mathbf {X} _ {i k} + \mathbf {V} _ {i k} \quad i = 1, 2, \dots , N \end{array} \right\}\tag{7.2.2}
$$

式(7.2.2)中 $W_{ik}$ 的协方差阵为 $Q_{ik}, V_{ik}$ 的协方差阵为 $R_{ik}$ 。

$$
\mathbf {Z} _ {k} = \left[ \begin{array}{l l l l} \mathbf {Z} _ {1 k} ^ {\mathrm{T}} & \mathbf {Z} _ {2 k} ^ {\mathrm{T}} & \dots & \mathbf {Z} _ {N k} ^ {\mathrm{T}} \end{array} \right] ^ {\mathrm{T}}\tag{7.2.3}
$$

式 $(7.2.3)$ 表示总系统利用了所有子系统的量测信息。

假设各子系统的量测值是相互独立的，且假设子系统状态 $X_{ik}$ 是总系统状态 $X_{k}$ 的一部分，即有

$$
\left. \begin{array}{l} \boldsymbol {X} _ {i k} = \boldsymbol {M} _ {i} \boldsymbol {X} _ {k} \\ \boldsymbol {H} _ {i k} = \boldsymbol {A} _ {i k} \boldsymbol {M} _ {i} \end{array} \right\}\tag{7.2.4}
$$

则集中滤波可用各子系统的量测值表示为

$$
\left. \begin{array}{l} \hat {\mathbf {X}} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \hat {\mathbf {X}} _ {k - 1} \\ \mathbf {P} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \mathbf {P} _ {k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} + \boldsymbol {Q} _ {k - 1} \\ \hat {\mathbf {X}} _ {k} = \hat {\mathbf {X}} _ {k / k - 1} + \sum_ {i} \mathbf {K} _ {i k} (\mathbf {Z} _ {i k} - \mathbf {H} _ {i k} \hat {\mathbf {X}} _ {k / k - 1}) \\ \mathbf {K} _ {i k} = \mathbf {P} _ {k / k - 1} \mathbf {H} _ {i k} ^ {\mathrm{T}} (\mathbf {H} _ {i k} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {i k} ^ {\mathrm{T}} + \mathbf {R} _ {i k}) ^ {- 1} \\ \mathbf {P} _ {k} = (\mathbf {I} - \sum_ {i} \mathbf {K} _ {i k} \mathbf {H} _ {i k}) \mathbf {P} _ {k / k - 1} \end{array} \right\}\tag{7.2.5}
$$

也可用信息滤波形式改写上面的量测更新方程,得到

$$
\left. \begin{array}{l} \hat {\mathbf {X}} _ {k} = \mathbf {P} _ {k} \mathbf {P} _ {k / k - 1} ^ {- 1} \hat {\mathbf {X}} _ {k / k - 1} + \sum_ {i} \mathbf {P} _ {k} \mathbf {H} _ {i k} ^ {\mathrm{T}} \mathbf {R} _ {i k} ^ {- 1} \mathbf {Z} _ {i k} \\ \mathbf {P} _ {k} ^ {- 1} = \mathbf {P} _ {k / k - 1} ^ {- 1} + \sum_ {i} \mathbf {H} _ {i k} ^ {\mathrm{T}} \mathbf {R} _ {i k} ^ {- 1} \mathbf {H} _ {i k} \end{array} \right\}\tag{7.2.6}
$$

子系统的局部滤波方程则可表示为

$$
\left. \begin{array}{l} \hat {\boldsymbol {X}} _ {i k} = \boldsymbol {P} _ {i k} \boldsymbol {P} _ {i, k / k - 1} ^ {- 1} \hat {\boldsymbol {X}} _ {i, k / k - 1} + \boldsymbol {P} _ {i k} \boldsymbol {A} _ {i k} ^ {\mathrm{T}} \boldsymbol {R} _ {i k} ^ {- 1} \boldsymbol {Z} _ {i k} \\ \boldsymbol {P} _ {i k} ^ {- 1} = \boldsymbol {P} _ {i, k / k - 1} ^ {- 1} + \boldsymbol {A} _ {i k} ^ {\mathrm{T}} \boldsymbol {R} _ {i k} ^ {- 1} \boldsymbol {A} _ {i k} \end{array} \right\}\tag{7.2.7}
$$

下面来推导全局滤波 $\hat{X}_{k}, P_{k}$ 与局部滤波 $\hat{X}_{ik}, P_{ik}$ 的关系。

将式 $(7.2.4)$ 中的第二式代入式 $(7.2.6)$ 中的第一式，得

$$
\hat {\mathbf {X}} _ {k} = \mathbf {P} _ {k} \mathbf {P} _ {k / k - 1} ^ {- 1} \hat {\mathbf {X}} _ {k / k - 1} + \sum_ {i} \mathbf {P} _ {k} \mathbf {M} _ {i} ^ {\mathrm{T}} \mathbf {A} _ {i k} ^ {\mathrm{T}} \mathbf {R} _ {i k} ^ {- 1} \mathbf {Z} _ {i k}
$$

由式(7.2.7)中的第一式求出 $A_{ik}^{T}R_{ik}^{-1}Z_{ik}$ 后，代入上式，可得

$$
\hat {\boldsymbol {X}} _ {k} = \boldsymbol {P} _ {k} \boldsymbol {P} _ {k / k - 1} ^ {- 1} \hat {\boldsymbol {X}} _ {k / k - 1} + \sum_ {i} \boldsymbol {P} _ {k} \boldsymbol {M} _ {i} ^ {\mathrm{T}} \boldsymbol {P} _ {i k} ^ {- 1} \hat {\boldsymbol {X}} _ {i k} - \sum_ {i} \boldsymbol {P} _ {k} \boldsymbol {M} _ {i} ^ {\mathrm{T}} \boldsymbol {P} _ {i, k / k - 1} ^ {- 1} \hat {\boldsymbol {X}} _ {i, k / k - 1}\tag{7.2.8}
$$

类似地，由式(7.2.7)中的第二式求出 $A_{ik}^{T}R_{ik}^{-1}A_{ik}$ 再代入式(7.2.6)中的第二式，可求得

$$
\boldsymbol {P} _ {k} ^ {- 1} = \boldsymbol {P} _ {k / k - 1} ^ {- 1} + \sum_ {i} \boldsymbol {M} _ {i} ^ {\mathrm{T}} \boldsymbol {P} _ {i k} ^ {- 1} \boldsymbol {M} _ {i} - \sum_ {i} \boldsymbol {M} _ {i} ^ {\mathrm{T}} \boldsymbol {P} _ {i, k / k - 1} ^ {- 1} \boldsymbol {M} _ {i}\tag{7.2.9}
$$

由式(7.2.8)和式(7.2.9)可知,全局滤波的量测更新可用局部滤波来表示。但全局滤波的时间更新(即求预报值 $\hat{X}_{k/k-1}$ 和 $P_{k/k-1}$ )仍需用全局滤波方程式(7.2.5)。上面得到的全局滤波是最优的,局部滤波相对于子系统来讲也是最优的,而且局部滤波器的运行是并行的。该分散化滤波方案曾被用于GPS/INS组合的研究。上述分散化滤波方法有下面的缺点:

（1）全局滤波的合成算法式(7.2.8)和式(7.2.9)还是比较复杂的，不仅用到了子滤波器的滤波值和协方差，还用到了它们的预报值，而且全局滤波的时间更新要用全局滤波方程。

(2) 算法基于各测量值是不相关的假设。

Carlson 提出的联邦滤波器 $^{[42]}$ 是一种崭新的分散化滤波方法,有人甚至把它从分散化滤波中独立出来 $^{[109]}$ ,作为一类滤波方法。联邦滤波致力于解决以下几个问题:

（1）滤波器的容错性能要好。当一个或几个导航子系统出现故障时，要能容易地检测和分离故障，并能很快地将剩下的正常的导航子系统重新组合起来（重构）以继续给出所需的滤波解。

(2) 滤波的精度要高。

（3）由局部滤波到全局滤波的合成(也可叫融合)算法要简单,计算量小,数据通讯少,以利于算法的实时执行。

上述的几个性能要求相互是有矛盾的。例如，要容错性能好，有时就要牺牲一些精度。为了解决这几个性能要求，联邦滤波中用了“信息分配”原则。通过将系统中的信息进行不同的分配,可以在这几个性能要求中获得最佳的折中,以满足不同的使用要求。至于什么是信息,如何进行分配,这些问题将在7.2.4节中进行阐述。

## 7.2.2 各子滤波器的估计不相关时的融合算法

首先考虑两个局部滤波器 $(N=2)$ 的情况。局部状态估计为 $\hat{X}_{1}$ 和 $\hat{X}_{2}$ ，相应的估计误差方差阵为 $P_{11}$ 和 $P_{22}$ 。考虑融合后的全局状态估计 $\hat{X}_{g}$ 为局部状态估计的线性组合，即

$$
\hat {\boldsymbol {X}} _ {\mathrm{g}} = \boldsymbol {W} _ {1} \hat {\boldsymbol {X}} _ {1} + \boldsymbol {W} _ {2} \hat {\boldsymbol {X}} _ {2}\tag{7.2.10}
$$

其中 $W_{1}$ 和 $W_{2}$ 是待定的加权阵。

全局估计 $\hat{X}_{g}$ 应满足以下两个条件：

(1) 若 $\hat{X}_{1}$ 和 $\hat{X}_{2}$ 为无偏估计，则 $\hat{X}_{g}$ 也应是无偏估计，即

$$
E \left\{\left(\boldsymbol {X} - \hat {\boldsymbol {X}} _ {g}\right) \right\} = \mathbf {0}
$$

式中 X 为真实状态。

(2) $\hat{X}_{g}$ 的估计误差协方差阵最小，即 $P_{g}=E\left\{(X-\hat{X}_{g})(X-\hat{X}_{g})^{T}\right\}$ 最小。

由条件(1)可得

即

$$
\begin{array}{r l} & E \{\mathbf {X} - \hat {\mathbf {X}} _ {\mathrm{g}} \} = E \{\mathbf {X} - \mathbf {W} _ {1} \hat {\mathbf {X}} _ {1} - \mathbf {W} _ {2} \hat {\mathbf {X}} _ {2} \} = \mathbf {0} \\ & \{\mathbf {I} - \mathbf {W} _ {1} - \mathbf {W} _ {2} \} E \{\mathbf {X} \} + \mathbf {W} _ {1} E \{\mathbf {X} - \hat {\mathbf {X}} _ {1} \} + \mathbf {W} _ {2} E \{\mathbf {X} - \hat {\mathbf {X}} _ {2} \} = \mathbf {0} \end{array}
$$

由于 $\hat{X}_1$ 和 $\hat{X}_2$ 为最优无偏估计，因此有

$$
\pmb {I} - \pmb {W} _ {1} - \pmb {W} _ {2} = \pmb {0} \text {或} \pmb {W} _ {1} = \pmb {I} - \pmb {W} _ {2}\tag{7.2.11}
$$

将式 $(7.2.11)$ 代入式 $(7.2.10)$ 得

$$
\hat {\boldsymbol {X}} _ {\mathrm{g}} = \hat {\boldsymbol {X}} _ {1} + \boldsymbol {W} _ {2} (\hat {\boldsymbol {X}} _ {2} - \hat {\boldsymbol {X}} _ {1})\tag{7.2.12}
$$

和

$$
\boldsymbol {X} - \hat {\boldsymbol {X}} _ {\mathrm{g}} = (\boldsymbol {I} - \boldsymbol {W} _ {2}) (\boldsymbol {X} - \hat {\boldsymbol {X}} _ {1}) + \boldsymbol {W} _ {2} (\boldsymbol {X} - \hat {\boldsymbol {X}} _ {2})
$$

于是 $P_{g}=E\left\{(X-\hat{X}_{g})(X-\hat{X}_{g})^{T}\right\}=$

$$
\boldsymbol {P} _ {1 1} - \boldsymbol {W} _ {2} (\boldsymbol {P} _ {1 1} - \boldsymbol {P} _ {1 2}) ^ {\mathrm{T}} - (\boldsymbol {P} _ {1 1} - \boldsymbol {P} _ {1 2}) \boldsymbol {W} _ {2} ^ {\mathrm{T}} + \boldsymbol {W} _ {2} (\boldsymbol {P} _ {1 1} - \boldsymbol {P} _ {1 2} - \boldsymbol {P} _ {2 1} + \boldsymbol {P} _ {2 2}) \boldsymbol {W} _ {2} ^ {\mathrm{T}}\tag{7.2.13}
$$

式中

$$
\boldsymbol {P} _ {1 1} = E \left\{\left(\boldsymbol {X} - \hat {\boldsymbol {X}} _ {1}\right) \left(\boldsymbol {X} - \hat {\boldsymbol {X}} _ {1}\right) ^ {\mathrm{T}} \right\}, \boldsymbol {P} _ {2 2} = E \left\{\left(\boldsymbol {X} - \hat {\boldsymbol {X}} _ {2}\right) \left(\boldsymbol {X} - \hat {\boldsymbol {X}} _ {2}\right) ^ {\mathrm{T}} \right\}
$$

$$
\boldsymbol {P} _ {1 2} = E \left\{\left(\boldsymbol {X} - \hat {\boldsymbol {X}} _ {1}\right) \left(\boldsymbol {X} - \hat {\boldsymbol {X}} _ {2}\right) ^ {\mathrm{T}} \right\}, \boldsymbol {P} _ {2 1} = \boldsymbol {P} _ {1 2} ^ {\mathrm{T}}
$$

现在来选择 $W_{2}$ ，使 $P_{g}$ 为最小。这等价于使 $trP_{g}$ 为最小。

利用公式

$$
\frac {\partial \mathrm{tr} (\mathbf {A X})}{\partial \mathbf {X}} = \mathbf {A} ^ {\mathrm{T}}, \frac {\partial \mathrm{tr} (\mathbf {A X} ^ {\mathrm{T}})}{\partial \mathbf {X}} = \mathbf {A}, \frac {\partial \mathrm{tr} (\mathbf {X B X} ^ {\mathrm{T}})}{\partial \mathbf {X}} = 2 \mathbf {X B} \quad (\mathbf {B} \text {为对称阵})
$$

由式(7.2.13)可得

$$
\frac {\partial \mathrm{tr} P _ {\mathrm{g}}}{\partial W _ {2}} = - (P _ {1 1} - P _ {1 2}) - (P _ {1 1} - P _ {1 2}) + 2 W _ {2} (P _ {1 1} - P _ {1 2} - P _ {2 1} + P _ {2 2}) = 0
$$

由此求出

$$
\boldsymbol {W} _ {2} = (\boldsymbol {P} _ {1 1} - \boldsymbol {P} _ {1 2}) (\boldsymbol {P} _ {1 1} + \boldsymbol {P} _ {2 2} - \boldsymbol {P} _ {1 2} - \boldsymbol {P} _ {2 1}) ^ {- 1}\tag{7.2.14}
$$

将式 $(7.2.14)$ 代入式 $(7.2.13)$ 和式 $(7.2.12)$ 得

$$
\boldsymbol {P} _ {\mathrm{g}} = \boldsymbol {P} _ {1 1} - (\boldsymbol {P} _ {1 1} - \boldsymbol {P} _ {1 2}) (\boldsymbol {P} _ {1 1} + \boldsymbol {P} _ {2 2} - \boldsymbol {P} _ {2 1} - \boldsymbol {P} _ {1 2}) ^ {- 1} (\boldsymbol {P} _ {1 1} - \boldsymbol {P} _ {1 2}) ^ {\mathrm{T}}\tag{7.2.15}
$$

$$
\hat {\boldsymbol {X}} _ {\mathrm{g}} = \hat {\boldsymbol {X}} _ {1} + (\boldsymbol {P} _ {1 1} - \boldsymbol {P} _ {1 2}) (\boldsymbol {P} _ {1 1} + \boldsymbol {P} _ {2 2} - \boldsymbol {P} _ {1 2} - \boldsymbol {P} _ {2 1}) ^ {- 1} (\hat {\boldsymbol {X}} _ {2} - \hat {\boldsymbol {X}} _ {1})\tag{7.2.16}
$$

下面证明 $P_{g} < P_{11}$ 和 $P_{g} < P_{22}$ 。

注意到

$$
E \{(\widetilde {X} _ {1} - \widetilde {X} _ {2}) (\widetilde {X} _ {1} - \widetilde {X} _ {2}) ^ {\mathrm{T}} \} = P _ {1 1} + P _ {2 2} - P _ {1 2} - P _ {2 1} \geqslant 0
$$

式中

$$
\widetilde {X} _ {1} \stackrel {\mathrm{def}} {=} X - \hat {X} _ {1}, \quad \widetilde {X} _ {2} \stackrel {\mathrm{def}} {=} X - \hat {X} _ {2}
$$

若不考虑 $\hat{X}_1 = \hat{X}_2$ 情况（这时只有一个估计值，不需融合），于是

$$
\begin{array}{r l} & (\boldsymbol {P} _ {1 1} + \boldsymbol {P} _ {2 2} - \boldsymbol {P} _ {1 2} - \boldsymbol {P} _ {2 1}) ^ {- 1} > \mathbf {0} \\ & (\boldsymbol {P} _ {1 1} - \boldsymbol {P} _ {1 2}) (\boldsymbol {P} _ {1 1} + \boldsymbol {P} _ {2 2} - \boldsymbol {P} _ {1 2} - \boldsymbol {P} _ {2 1}) ^ {- 1} (\boldsymbol {P} _ {1 1} - \boldsymbol {P} _ {1 2}) ^ {\mathrm{T}} > \mathbf {0} \end{array}
$$

由式(7.2.15)可导出 $P_{g}<P_{11}$

同理可推得 $P_{g} < P_{22}$ 。这说明全局估计优于局部估计。

若 $\hat{X}_1$ 和 $\hat{X}_2$ 是不相关的，即有

$$
\mathbf {P} _ {1 2} = \mathbf {P} _ {2 1} = \mathbf {0}
$$

则式 $(7.2.15)$ 和式 $(7.2.16)$ 可简化为

$$
\hat {\mathbf {X}} _ {\mathrm{g}} = \left(\mathbf {P} _ {1 1} ^ {- 1} + \mathbf {P} _ {2 2} ^ {- 1}\right) ^ {- 1} \left(\mathbf {P} _ {1 1} ^ {- 1} \hat {\mathbf {X}} _ {1} + \mathbf {P} _ {2 2} ^ {- 1} \hat {\mathbf {X}} _ {2}\right)\tag{7.2.17}
$$

$$
\boldsymbol {P} _ {\mathrm{g}} = (\boldsymbol {P} _ {1 1} ^ {- 1} + \boldsymbol {P} _ {2 2} ^ {- 1}) ^ {- 1}\tag{7.2.18}
$$

利用数学归纳法很容易将上面的结果推广到有 N 个局部估计的情况。

定理7.1

若有 N 个局部状态估计 $\hat{X}_{1}, \hat{X}_{2}, \cdots, \hat{X}_{N}$ 和相应的估计误差协方差阵 $P_{11}, P_{22}, \cdots, P_{NN}$ ，且各局部估计互不相关，即 $P_{ij} = 0 (i \neq j)$ ，则全局最优估计可表示为

$$
\hat {\mathbf {X}} _ {\mathrm{g}} = \mathbf {P} _ {\mathrm{g}} \sum_ {i = 1} ^ {N} \mathbf {P} _ {i i} ^ {- 1} \hat {\mathbf {X}} _ {i}\tag{7.2.19}
$$

$$
\mathbf {P} _ {\mathrm{g}} = (\sum_ {i = 1} ^ {N} \mathbf {P} _ {i i} ^ {- 1}) ^ {- 1}\tag{7.2.20}
$$

式 $(7.2.19)$ 和式 $(7.2.20)$ 也可根据马尔柯夫估计推导出。

设联邦滤波器有 $N(N>2)$ 个子滤波器，子滤波器 i 的输出为

$$
\hat {\boldsymbol {X}} _ {i} = \boldsymbol {X} + \tilde {\boldsymbol {X}} _ {i} \quad i = 1, 2, \dots , N
$$

式中，X 为各子滤波器的公共状态，维数为 n, $\widetilde{X}_{i}$ 为子滤波器 i 的估计误差，若子滤波器工作正常，则 $\widetilde{X}_{i}$ 为白噪声。

根据 $N$ 个子滤波器的输出可列写关于 $\pmb{X}$ 的量测方程

式中

$$
\mathbf {Z} = \mathbf {H X} + \mathbf {V}
$$

$$
\mathbf {Z} = \left[ \begin{array}{c} \hat {\mathbf {X}} _ {1} \\ \hat {\mathbf {X}} _ {2} \\ \vdots \\ \hat {\mathbf {X}} _ {N} \end{array} \right], \quad \mathbf {H} = \left[ \begin{array}{c} \mathbf {I} _ {n \times n} \\ \mathbf {I} _ {n \times n} \\ \vdots \\ \mathbf {I} _ {n \times n} \end{array} \right], \quad \mathbf {V} = \left[ \begin{array}{c} \widetilde {\mathbf {X}} _ {1} \\ \widetilde {\mathbf {X}} _ {2} \\ \vdots \\ \widetilde {\mathbf {X}} _ {N} \end{array} \right]
$$

假设各子滤波器工作正常,且估计误差互不相关,则有

$$
\begin{array}{r l} & E [ \mathbf {V} ] = \mathbf {0} \\ & \mathbf {R} = E [ \mathbf {V V ^ {\mathrm{T}}} ] = \mathrm{diag} [ \mathbf {P} _ {1 1} \mathbf {P} _ {2 2} \dots \mathbf {P} _ {N N} ] \end{array}
$$

其中 $P_{ii}=E\left[\widetilde{X}_{i}\widetilde{X}_{i}^{\mathrm{T}}\right]$ ，即子滤波器 i 的估计误差方差阵。

根据式(2.1.12)和式(2.1.13)，公共状态X的马尔柯夫估计为

$$
\begin{array}{r l} & {\pmb {P} _ {g} = (\pmb {H} ^ {\mathrm{T}} \pmb {R} ^ {- 1} \pmb {H}) ^ {- 1} = \left(\sum_ {i = 1} ^ {N} \pmb {P} _ {i i} ^ {- 1}\right) ^ {- 1}} \\ & {\hat {\pmb {X}} _ {g} = (\pmb {H} ^ {\mathrm{T}} \pmb {R} ^ {- 1} \pmb {H}) ^ {- 1} \pmb {H} ^ {\mathrm{T}} \pmb {R} ^ {- 1} \pmb {Z} = \left(\sum_ {i = 1} ^ {N} \pmb {P} _ {i i} ^ {- 1}\right) ^ {- 1} \sum_ {i = 1} ^ {N} \pmb {P} _ {i i} ^ {- 1} \hat {\pmb {X}} _ {i} = \pmb {P} _ {g} \sum_ {i = 1} ^ {N} \pmb {P} _ {i i} ^ {- 1} \hat {\pmb {X}} _ {i}} \end{array}
$$

上面结果的物理意义是很明显的。若 $\hat{X}_i$ 的估计精度差，即 $P_{ii}$ 大，那么它在全局估计中的贡献 $P_{ii}^{-1}\hat{X}_i$ 就比较小。这个结果非常简单明了，但其条件是各局部估计应是不相关的。在一般情况下，这个条件是不满足的，即各局部估计是相关的。联邦滤波器的设计就是针对这种情况，对滤波过程进行适当的改造，使得局部估计实际上不相关，于是上述定理就可应用了。

## 7.2.3 各子滤波器的估计相关时的融合算法

假设各子滤波器的状态估计可表示为

$$
\hat {\boldsymbol {X}} _ {i} = \left[ \begin{array}{c} \hat {\boldsymbol {X}} _ {c i} \\ \hat {\boldsymbol {X}} _ {b i} \end{array} \right]\tag{7.2.21}
$$

式中， $\hat{X}_{ci}$ 是各子滤波器的公共状态 $X_{c}$ 的估计，如导航位置、速度、姿态等的误差状态的估计； $\hat{X}_{bi}$ 则是第 $i$ 个滤波器专有的状态的估计，如GPS的误差状态的估计。我们只对公共状态的估计进行融合以得到全局估计。

## 1. 信息分配原则与全局最优估计

联邦滤波器是一种两级滤波结果，如图7.2.1所示。图中公共参考系统一般是惯导系统，它的输出 $X_{k}$ 一方面直接给主滤波器，另一方面它可以输给各子滤波器（局部滤波器）作为量测值。各子系统的输出只给相应的子滤波器。各子滤波器的局部估计值 $\hat{X}_{i}$ （公共状态）及其协方差阵 $P_{i}$ 送入主滤波器和主滤波器的估计值一起进行融合以得到全局最优估计。此外，从图中还可以看到，由子滤波器与主滤波器合成的全局估计值 $\hat{X}_{g}$ 及其相应的协方差阵 $P_{g}$ 被放大为 $\beta_{i}^{-1}P_{g}(\beta_{i}\leqslant1)$ 后再反馈到子滤波器（图中用虚线表示），以重置子滤波器的估计值，即

$$
\hat {\boldsymbol {X}} _ {i} = \hat {\boldsymbol {X}} _ {\mathrm{g}}, \quad \boldsymbol {P} _ {i i} = \beta_ {i} ^ {- 1} \boldsymbol {P} _ {\mathrm{g}}\tag{7.2.22}
$$

同时主滤波器预报误差的协方差阵也可重置为全局协方差阵的 $\beta_{m}^{-1}$ 倍，即为 $\beta_{m}^{-1}P_{g}(\beta_{m}\leqslant1)$ 。这种反馈的结构是联邦滤波器区别于一般分散化滤波器的特点。 $\beta_{i}(i=1,2,\cdots,N,m)$ 称为“信息分配系数”。 $\beta_{i}$ 是根据“信息分配”原则来确定的，不同的 $\beta_{i}$ 值可以获得联邦滤波器的不同结构和不同的特性（即容错性、精度和计算量）。

![](images/406dc68de0d1a3afb04f279016ad1bd35d010c8436275ce29428c0a3a59524d1.jpg)  
图 7.2.1 联邦滤波器的一般结构

什么是“信息分配”原则呢？首先来说明什么是信息。系统中有两类信息：

（1）状态运动方程的信息。卡尔曼滤波器要利用状态方程的信息，而递推最小二乘估计则只用测量信息而不用系统状态运动方程的信息。因此，理论上卡尔曼滤波将给出更精确的估计和预测。状态方程的信息量是与状态方程中过程噪声的方差（或协方差阵）成反比的。过程噪声越弱，状态方程就越精确。因此，状态方程的信息量可以用过程噪声协方差阵的逆，即 $Q^{-1}$ 来表示。此外，状态初值的信息，也是状态方程的信息。初值的信息量可用初值估计的协方差阵的逆，即 $P^{-1}(0)$ 来表示。

（2）量测方程的信息。量测方程的信息量可用量测噪声协方差阵的逆，即 $R^{-1}$ 来表示。当状态方程、量测方程及 $P(0), Q, R$ 选定后，状态估计 $\hat{X}$ 及估计误差P也就完全确定了，而状态估计的信息量可用 $P^{-1}$ 来表示。对公共状态来讲，它所对应的过程噪声包含在所有的子滤波器及主滤波器中。因此，过程噪声的信息量存在重复使用的问题。各子滤波器的量测方程中只包含了对应子系统的噪声，如INS/GPS局部滤波器的量测噪声只包含GPS的量测噪声，INS/Doppler局部滤波器只包含Doppler雷达的噪声（可见文献[110]中所列的方程）。于是，可以认为各局部滤波器的量测信息是自然地分割的，不存在重复使用的问题。

假设将过程噪声总的信息量 $Q^{-1}$ 分配到各局部滤波器和主滤波器中去，即

$$
Q ^ {- 1} = \sum_ {i = 1} ^ {N} Q _ {i} ^ {- 1} + Q _ {m} ^ {- 1}\tag{7.2.23}
$$

而

$$
Q _ {i} = \beta_ {i} ^ {- 1} Q\tag{7.2.24}
$$

故

$$
Q ^ {- 1} = \sum_ {i = 1} ^ {N} \beta_ {i} Q ^ {- 1} + \beta_ {m} Q ^ {- 1}\tag{7.2.25}
$$

根据“信息守恒”原理，由上式可知

$$
\sum_ {i = 1} ^ {N} \beta_ {i} + \beta_ {m} = 1\tag{7.2.26}
$$

状态估计初始信息 $P^{-1}(0)$ 也可按上述方法分配。假设状态估计的信息也可同样分配，得

$$
\boldsymbol {P} ^ {- 1} = \boldsymbol {P} _ {1} ^ {- 1} + \boldsymbol {P} _ {2} ^ {- 1} + \dots + \boldsymbol {P} _ {N} ^ {- 1} + \boldsymbol {P} _ {m} ^ {- 1} = \sum_ {i = 1} ^ {N} \beta_ {i} \boldsymbol {P} ^ {- 1} + \beta_ {m} \boldsymbol {P} ^ {- 1}\tag{7.2.27}
$$

注意到在上面状态估计信息的分配中，已假定各子滤波器的局部估计是不相关的，即 $P_{ij}=0(i\neq j)$ 。在这个不相关的假设条件下，就可应用式(7.2.19)和式(7.2.20)来获得全局估计了。为了使 $P_{ij}$ 永远等于零，则要对滤波过程进行改造，先构造一个增广系统，它的状态向量是由N个局部滤波器的子系统和主滤波器的子系统的状态重叠而成的，即

$$
\begin{array}{r l} & \boldsymbol {X} = \left[ \begin{array}{c} \boldsymbol {X} _ {1} \\ \vdots \\ \boldsymbol {X} _ {N} \end{array} \right] \\ & \overline {{N}} = N + 1 \end{array}
$$

式中

(7.2.28)

每个子系统的状态向量 $X_{i}$ 又可表示为

$$
\boldsymbol {X} _ {i} = \left[ \begin{array}{c} \boldsymbol {X} _ {c} \\ \boldsymbol {X} _ {b i} \end{array} \right]\tag{7.2.29}
$$

式中， $X_{c}$ 是公共状态向量， $X_{bi}$ 是第 i 个子系统的误差状态。

这个增广系统的状态向量中含有余度的状态——公共状态，但这并不影响理论分析。增

广系统的状态方程为

$$
\left[ \begin{array}{c} \boldsymbol {X} _ {1} \\ \vdots \\ \boldsymbol {X} _ {\bar {N}} \end{array} \right] _ {k + 1} = \left[ \begin{array}{c c c} \boldsymbol {\Phi} _ {1 1} & & \\ & \ddots & \\ & & \boldsymbol {\Phi} _ {\bar {N} \bar {N}} \end{array} \right] \left[ \begin{array}{c} \boldsymbol {X} _ {1} \\ \vdots \\ \boldsymbol {X} _ {\bar {N}} \end{array} \right] _ {k} + \left[ \begin{array}{c} \boldsymbol {G} _ {1} \\ \vdots \\ \boldsymbol {G} _ {\bar {N}} \end{array} \right] \boldsymbol {W} _ {k}\tag{7.2.30}
$$

$$
\boldsymbol {E} [ \boldsymbol {W} _ {k} \boldsymbol {W} _ {k} ^ {\mathrm{T}} ] = \boldsymbol {Q}\tag{7.2.31}
$$

第 $i$ 个子系统的量测方程为

$$
\mathbf {Z} _ {i} = \mathbf {H} _ {i} \mathbf {X} _ {i} + \mathbf {V} _ {i}\tag{7.2.32}
$$

令

$$
\boldsymbol {H} = \left[ \begin{array}{l l l l l l} \mathbf {0} & \mathbf {0} & \dots & \boldsymbol {H} _ {i} & \dots & \mathbf {0} \end{array} \right]\tag{7.2.33}
$$

则 $Z_{i}$ 可用增广系统的状态表示为

$$
\mathbf {Z} _ {i} = \mathbf {H X} + \mathbf {V} _ {i}\tag{7.2.34}
$$

增广系统总体滤波的协方差阵一般可表示为

$$
\boldsymbol {P} = \left[ \begin{array}{c c c} \boldsymbol {P} _ {1 1} & \dots & \boldsymbol {P} _ {1 \bar {N}} \\ \vdots & & \vdots \\ \boldsymbol {P} _ {\bar {N} 1} & \dots & \boldsymbol {P} _ {\bar {N} \bar {N}} \end{array} \right]\tag{7.2.35}
$$

式中， $P_{ji}$ 表示局部滤波之间的相关性。现证明当 $P_{ji}(0)=0$ 时，增广系统滤波的量测更新和时间更新可分解为各局部滤波器的独立的量测更新和时间更新，它们之间没有交联。

为使符号书写简便,令

$$
\left. \begin{array}{l} \hat {\boldsymbol {X}} (+) = \hat {\boldsymbol {X}} _ {k}, \hat {\boldsymbol {X}} (-) = \hat {\boldsymbol {X}} _ {k / k - 1} \\ \boldsymbol {P} (+) = \boldsymbol {P} _ {k}, \boldsymbol {P} (-) = \boldsymbol {P} _ {k / k - 1}, \boldsymbol {Z} = \boldsymbol {Z} _ {k} \end{array} \right\}\tag{7.2.36}
$$

于是，当只用 $Z_{i}$ 一个量测值时（这时 $\mathbf{Z} = \mathbf{Z}_i, \mathbf{R} = \mathbf{R}_i$ ），有

$$
\mathbf {A} = \mathbf {H P} (-) \mathbf {H} ^ {\mathrm{T}} + \mathbf {R} = \mathbf {A} _ {i}\tag{7.2.37}
$$

式中

$$
\mathbf {A} _ {i} = \mathbf {H} _ {i} \mathbf {P} _ {i i} (-) \mathbf {H} _ {i} ^ {\mathrm{T}} + \mathbf {R} _ {i}\tag{7.2.38}
$$

集中滤波的量测更新为

$$
\hat {\boldsymbol {X}} (+) = \hat {\boldsymbol {X}} (-) + \boldsymbol {P} (-) \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {A} ^ {- 1} \left[ \boldsymbol {Z} _ {i} - \boldsymbol {H} \hat {\boldsymbol {X}} (-) \right]\tag{7.2.39}
$$

考虑第 i 个量测对第 j 个估计的更新,由上式取出第 j 个估计

$$
\hat {\mathbf {X}} _ {j} (+) = \hat {\mathbf {X}} _ {j} (-) + \mathbf {P} _ {j i} (-) \mathbf {H} _ {i} ^ {\mathrm{T}} \mathbf {A} _ {i} ^ {- 1} [ \mathbf {Z} _ {i} - \mathbf {H} _ {i} \hat {\mathbf {X}} _ {i} (-) ]\tag{7.2.40}
$$

集中滤波协方差的量测更新为

$$
\boldsymbol {P} (+) = \boldsymbol {P} (-) - \boldsymbol {P} (-) \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {A} ^ {- 1} \boldsymbol {H} \boldsymbol {P} (-)\tag{7.2.41}
$$

取出第 $jl$ 个元

$$
\boldsymbol {P} _ {j l} (+) = \boldsymbol {P} _ {j l} (-) - \boldsymbol {P} _ {j i} (-) \boldsymbol {H} _ {i} ^ {\mathrm{T}} \boldsymbol {A} _ {i} ^ {- 1} \boldsymbol {H} _ {i} \boldsymbol {P} _ {i l} (-)\tag{7.2.42}
$$

（1）当 $l = i, j \neq i$ 时，由式(7.2.42)可知，由 $P_{ji}(-) = 0$ 可推出 $P_{ji}(+) = 0$ ，即相关项 $P_{ji}$ 的预报值若为零，则由量测更新所得滤波值仍为零。当 $P_{ji}(-) = 0$ 时，由式(7.2.40)还可得 $\hat{X}_j(+) = \hat{X}_j(-)$ ，即第 $i$ 个量测值不会引起 $\hat{X}_j$ 的量测更新。

（2）当 $l = i, j = i$ 时，由式(7.2.40)和式(7.2.42)可见， $\hat{\pmb{X}}_i$ 和 $\pmb{P}_{ii}$ 的量测更新都是只与第 $i$ 个量测值有关。

（3）当 $l \neq i, j \neq i$ 时，只要 $\pmb{P}_{ji}(-) = \mathbf{0}$ ，由式(7.2.42)有 $\pmb{P}_{jl}(+) = \pmb{P}_{jl}(-)$ 。即第 $i$ 个量测不会对其他局部滤波之间的相关项（暂时假定存在这样的相关）进行量测更新。

上述讨论说明，只要 $P_{ji}(-) = 0$ ，集中滤波的量测更新可分解为各局部滤波器的量测更新。

## 2. 联邦滤波的时间更新

先考虑集中滤波的时间更新,由状态方程式(7.2.30)可得

$$
\begin{array}{r l} \left[ \begin{array}{c c c} P _ {1 1} & \dots & P _ {1 \bar {N}} \\ \vdots & & \vdots \\ P _ {\bar {N} 1} & \dots & P _ {\bar {N} \bar {N}} \end{array} \right] & = \left[ \begin{array}{c c c} \Phi_ {1 1} & & \\ & \ddots & \\ & & \Phi_ {\bar {N} \bar {N}} \end{array} \right] \left[ \begin{array}{c c c} P _ {1 1} ^ {\prime} & \dots & P _ {1 \bar {N}} ^ {\prime} \\ \vdots & & \vdots \\ P _ {\bar {N} 1} ^ {\prime} & \dots & P _ {\bar {N} \bar {N}} ^ {\prime} \end{array} \right] \left[ \begin{array}{c c c} \Phi_ {1 1} ^ {\mathrm{T}} & & \\ & \ddots & \\ & & \Phi_ {\bar {N} \bar {N}} ^ {\mathrm{T}} \end{array} \right] + \\ & \left[ \begin{array}{c} G _ {1} \\ \vdots \\ G _ {\bar {N}} \end{array} \right] Q [ G _ {1} ^ {\mathrm{T}} & \dots & G _ {\bar {N}} ^ {\mathrm{T}} ] \end{array}\tag{7.2.43}
$$

由式 $(7.2.43)$ 可得

$$
\boldsymbol {P} _ {j i} = \boldsymbol {\Phi} _ {j j} \boldsymbol {P} _ {j i} ^ {\prime} \boldsymbol {\Phi} _ {i i} ^ {\mathrm{T}} + \boldsymbol {G} _ {j} \boldsymbol {Q} \boldsymbol {G} _ {i} ^ {\mathrm{T}}\tag{7.2.44}
$$

$$
\boldsymbol {P} _ {i i} \stackrel {\text { def }} {=} \boldsymbol {P} _ {i i} (k / k - 1), \boldsymbol {P} _ {i i} ^ {\prime} \stackrel {\text { def }} {=} \boldsymbol {P} _ {i i} (k - 1), \boldsymbol {P} _ {j i} \stackrel {\text { def }} {=} \boldsymbol {P} _ {j i} (k / k - 1), \boldsymbol {P} _ {j i} ^ {\prime} \stackrel {\text { def }} {=} \boldsymbol {P} _ {j i} (k - 1)
$$

由于公共状态的公共噪声 $Q$ 的存在，即使 $P_{ji}^{\prime} = 0$ ，也不会有 $P_{ji} = 0$ 。也就是说，时间更新将引入各子滤波器估计的相关。现在用所谓“方差上界”技术来消除时间更新引入的相关。先将式(7.2.43)中的过程噪声项改写为

$$
\left[ \begin{array}{c} \boldsymbol {G} _ {1} \\ \vdots \\ \boldsymbol {G} _ {\bar {N}} \end{array} \right] Q \left[ \begin{array}{l l l} \boldsymbol {G} _ {1} ^ {\mathrm{T}} & \dots & \boldsymbol {G} _ {\bar {N}} ^ {\mathrm{T}} \end{array} \right] = \left[ \begin{array}{c c c} \boldsymbol {G} _ {1} & & \\ & \ddots & \\ & & \boldsymbol {G} _ {\bar {N}} \end{array} \right] \left[ \begin{array}{c c c} Q & \dots & Q \\ \vdots & & \vdots \\ Q & \dots & Q \end{array} \right] \left[ \begin{array}{c c c} \boldsymbol {G} _ {1} ^ {\mathrm{T}} & & \\ & \ddots & \\ & & \boldsymbol {G} _ {\bar {N}} ^ {\mathrm{T}} \end{array} \right]\tag{7.2.45}
$$

由矩阵理论可知，式(7.2.45)右端由 $Q$ 组成的 $\overline{N} \times \overline{N}$ 矩阵有以下的上界：

$$
\begin{array}{r l} & {\left[ \begin{array}{c c c} Q & \dots & Q \\ \vdots & & \vdots \\ Q & \dots & Q \end{array} \right] \leqslant \left[ \begin{array}{c c c} \gamma_ {1} Q & \dots & 0 \\ \vdots & & \vdots \\ 0 & \dots & \gamma_ {N} Q \end{array} \right]} \\ & {\frac {1}{\gamma_ {1}} + \dots + \frac {1}{\gamma_ {N}} = 1 \quad 0 \leqslant \frac {1}{\gamma_ {i}} \leqslant 1} \end{array}\tag{7.2.46}
$$

(7.2.47)

式 $(7.2.46)$ 右端的上界矩阵与左端的原矩阵之差为半正定的。再由式 $(7.2.43)$ 可得

$$
\begin{array}{r l} \left[ \begin{array}{c c c} P _ {1 1} & \dots & P _ {1 \bar {N}} \\ \vdots & & \vdots \\ P _ {N 1} & \dots & P _ {N N} \end{array} \right] & \leqslant \left[ \begin{array}{c c c} \Phi_ {1 1} & & \\ & \ddots & \\ & & \Phi_ {\bar {N} \bar {N}} \end{array} \right] \left[ \begin{array}{c c c} P _ {1 1} ^ {\prime} & \dots & P _ {1 \bar {N}} ^ {\prime} \\ \vdots & & \vdots \\ P _ {N 1} ^ {\prime} & \dots & P _ {N N} ^ {\prime} \end{array} \right] \left[ \begin{array}{c c c} \Phi_ {1 1} ^ {\mathrm{T}} & & \\ & \ddots & \\ & & \Phi_ {\bar {N} \bar {N}} ^ {\mathrm{T}} \end{array} \right] + \\ & \left[ \begin{array}{c c c} G _ {1} & & \\ & \ddots & \\ & & G _ {\bar {N}} \end{array} \right] \left[ \begin{array}{c c c} \gamma_ {1} Q & & \\ & \ddots & \\ & & \gamma_ {\bar {N}} Q \end{array} \right] \left[ \begin{array}{c c c} G _ {1} ^ {\mathrm{T}} & & \\ & \ddots & \\ & & G _ {N} ^ {\mathrm{T}} \end{array} \right] \end{array}\tag{7.2.48}
$$

在式 $(7.2.48)$ 中取等号,即放大协方差(得到比较保守的结果),可得分离的时间更新

$$
\boldsymbol {P} _ {i i} = \boldsymbol {\Phi} _ {i i} \boldsymbol {P} _ {i i} ^ {\prime} \boldsymbol {\Phi} _ {i i} ^ {\mathrm{T}} + \gamma_ {i} \boldsymbol {G} _ {i} Q \boldsymbol {G} _ {i} ^ {\mathrm{T}}\tag{7.2.49}
$$

$$
\boldsymbol {P} _ {j i} = \boldsymbol {\Phi} _ {j j} \boldsymbol {P} _ {j i} ^ {\prime} \boldsymbol {\Phi} _ {i i} ^ {\mathrm{T}} = 0, \quad \boldsymbol {P} _ {j i} ^ {\prime} = 0\tag{7.2.50}
$$

式(7.2.50)说明, 只要 $P_{ji}^{\prime}=P_{ji}(k-1)=0$ , 就有 $P_{ji}=P_{ji}(k/k-1)=0$ 。这就是说, 时间更新也是在各子滤波器中独立进行的, 没有子滤波器之间的关联。

由 $P_{ji}(0)=0$ ，从式(7.2.50)可推出 $P_{ji}(1/0)=0$ ，由式(7.2.42)可推出 $P_{ji}(1/1)=0$ ，再由式(7.2.50)可推出 $P_{ji}(2/1)=0$ ，如此循环可得出 $P_{ji}(k/k-1)=0$ 和 $P_{ji}(k)=0$ 。而初始协方差阵也可设置上界，即

$$
\left[ \begin{array}{c c c} \boldsymbol {P} _ {1 1} (0) & \dots & \boldsymbol {P} _ {1 \bar {N}} (0) \\ \vdots & & \vdots \\ \boldsymbol {P} _ {\bar {N} 1} (0) & \dots & \boldsymbol {P} _ {\bar {N} \bar {N}} (0) \end{array} \right] \leqslant \left[ \begin{array}{c c c} \gamma_ {1} \boldsymbol {P} _ {1 1} (0) & & \\ & \ddots & \\ & & \gamma_ {\bar {N}} \boldsymbol {P} _ {\bar {N} \bar {N}} (0) \end{array} \right]\tag{7.2.51}
$$

式(7.2.51)右端无相关项,也就是说,将各子滤波器自身的初始方差阵放大些就可忽略各自滤波器初始方差之间的相关项。当然,这样也得到了保守的(次优的)局部滤波结果。

总而言之，采用方差上界技术后各子滤波器的量测更新和时间更新都可以独立进行，也就是各子滤波器的估计是不相关的。这样，就可应用7.2.2节中的最优合成定理来融合局部估计以获得全局估计。现在的问题是，采用了方差上界技术后，局部估计是次优的，合成后的全局估计相对集中滤波是否也变成次优的了。回答是否定的，合成后的全局估计是最优的。简单说明如下：采用方差上界技术后，由式(7.2.49)，子滤波器的过程噪声方差 $Q$ 被放大为 $\gamma_{i}Q$ 或反过来说，子滤波器只分配到原过程信息量 $Q^{-1}$ 的一部分，即 $\gamma_{i}^{-1}Q^{-1}$ ，当然子滤波器的估计是次优的。但信息分配是根据信息守恒原理在各子滤波器和主滤波器之间分配的，即满足

$$
\sum_ {i = 1} ^ {N} \gamma_ {i} ^ {- 1} Q ^ {- 1} + \gamma_ {m} ^ {- 1} Q ^ {- 1} = Q ^ {- 1}\tag{7.2.52}
$$

这样，在合成过程中信息量又被恢复到原来的值，所以合成后的估计将是最优的。

如果第 i 个子滤波器的初值协方差 $P_{i}(0)$ 被放大为 $\gamma_{i}P_{i}(0)$ ，过程噪声 Q 被放大为 $\gamma_{i}Q$ ，则由滤波协方差的时间更新式（7.2.49）可知，预报协方差 $P_{i}(1/0)$ 也放大了 $\gamma_{i}$ 倍。由于我们只考虑对公共状态估计的融合问题，因此，如果公共状态的最优预报协方差阵为 $P_{g}(-)$ ，现在第 i 个子滤波器的预报协方差阵变为 $P_{i}(-)=\gamma_{i}P_{g}(-)$ 。设用融合算法式（7.2.20）的结果为 $P^{-1}$ ，则

$$
\begin{array}{r l} \boldsymbol {P} ^ {- 1} (-) & = \boldsymbol {P} _ {1} ^ {- 1} (-) + \dots + \boldsymbol {P} _ {N} ^ {- 1} (-) + \boldsymbol {P} _ {m} ^ {- 1} (-) = \\ & \gamma_ {1} ^ {- 1} \boldsymbol {P} _ {\mathrm{g}} ^ {- 1} (-) + \dots + \gamma_ {N} ^ {- 1} \boldsymbol {P} _ {\mathrm{g}} ^ {- 1} (-) + \gamma_ {m} ^ {- 1} \boldsymbol {P} _ {\mathrm{g}} ^ {- 1} (-) = \\ & (\gamma_ {1} ^ {- 1} + \dots + \gamma_ {N} ^ {- 1} + \gamma_ {m} ^ {- 1}) \boldsymbol {P} _ {\mathrm{g}} ^ {- 1} (-) = \boldsymbol {P} _ {\mathrm{g}} ^ {- 1} (-) \end{array}\tag{7.2.53}
$$

这说明用上面的融合算法和信息分配原则,合成后的预报方差是最优的(这里暂时只对 $P_{i}(1/0)$ 的合成的最优性作了证明,下面将对任意k来证明)。

## 3. 联邦滤波的量测更新

如果量测更新后的滤波协方差也增加 $\gamma_{i}$ 倍，则由初始估计协方差和过程噪声协方差增加 $\gamma_{i}$ 倍可推出预报协方差 $P_{i}(k / k - 1)$ 和 $P_{i}(k)$ 都增加 $\gamma_{i}$ 倍，对任何 $k$ 成立。由量测更新方程式(7.2.42）可得（令 $P_{i} = P_{ii}$ ）

$$
\boldsymbol {P} _ {i} (+) = \boldsymbol {P} _ {i} (-) - \boldsymbol {P} _ {i} (-) \boldsymbol {H} _ {i} ^ {\mathrm{T}} \boldsymbol {A} _ {i} ^ {- 1} \boldsymbol {H} _ {i} \boldsymbol {P} _ {i} (-)\tag{7.2.54}
$$

式中当 $P_{i}$ (一)增加 $\gamma_{i}$ 倍时, $P_{i}(+)$ 并不增加 $\gamma_{i}$ 倍。为解决此问题,在联邦滤波方案中采用全局滤波来重置局部滤波值及滤波协方差,即有

$$
\hat {\boldsymbol {X}} _ {i} (+) = \hat {\boldsymbol {X}} _ {\mathrm{g}} (+)\tag{7.2.55}
$$

$$
\boldsymbol {P} _ {i} = \gamma_ {i} \boldsymbol {P} _ {\mathrm{g}} (+)\tag{7.2.56}
$$

重置后的滤波协方差 $P_{i}(+)$ 是 $P_{\mathrm{g}}(+)$ 的 $\gamma_{i}$ 倍，由式(7.2.49)又可推出下一步预报协方差 $P_{i}(-)$ 是 $P_{\mathrm{g}}(-)$ 的 $\gamma_{i}$ 倍。于是式(7.2.53）所示的预报协方差的最优融合在任何时刻都成立。剩下的问题是 $P_{\mathrm{g}}(+)$ 如何获得。式(7.2.54）所示的局部最优滤波协方差也可写成

$$
\boldsymbol {P} _ {i} ^ {- 1} (+) = \boldsymbol {P} _ {i} ^ {- 1} (-) + \boldsymbol {H} _ {i} ^ {\mathrm{T}} \boldsymbol {R} _ {i} ^ {- 1} \boldsymbol {H} _ {i}\tag{7.2.57}
$$

将子滤波器及主滤波器的协方差阵之逆合成，即

$$
\begin{array}{r l} & {\pmb {P} _ {m} ^ {- 1} (-) + \sum_ {i = 1} ^ {N} \pmb {P} _ {i} ^ {- 1} (+) = \pmb {P} _ {m} ^ {- 1} (-) + \sum_ {i = 1} ^ {N} (\pmb {P} _ {i} ^ {- 1} (-) + \pmb {H} _ {i} ^ {\mathrm{T}} \pmb {R} _ {i} ^ {- 1} \pmb {H} _ {i}) =} \\ & {\qquad \qquad \qquad \qquad \qquad \qquad \qquad \qquad \qquad \qquad \pmb {P} _ {\mathrm{g}} ^ {- 1} (-) + \sum_ {i = 1} ^ {N} \pmb {H} _ {i} ^ {\mathrm{T}} \pmb {R} _ {i} ^ {- 1} \pmb {H} _ {i} = \pmb {P} _ {\mathrm{g}} ^ {- 1} (+)} \end{array}\tag{7.2.58}
$$

式中采用了 $P_{m}^{-1}$ （一），这是因为主滤波器的量测更新就是靠子滤波器的量测更新来进行的，如果再进行量测更新，就会有量测信息的重复使用问题。式（7.2.58）揭示了全局滤波是组合了各子滤波器的独立量测信息（由 $R_{i}^{-1}$ 来表示）来进行最优量测更新的。

采用信息分配原则后,局部滤波虽是次优的,但合成后的全局滤波却是最优的。如果融合的周期长于局部滤波周期,即经过几次局部滤波后才进行一次融合,那么全局估计也会变成次优的。

## 4. 联邦滤波器的设计步骤

根据上面的理论分析可得出下面几点联邦滤波器的设计技巧：

（1）用方差上界技术使各子滤波器初始估计协方差阵互不相关。

(2) 用方差上界技术使各子滤波器的过程噪声协方差阵互不相关。

(3) 增广系统转移矩阵无子系统间的交联项。

（4）局部量测更新不会引起子滤波器估计的相关。

服从信息分配原则的各独立的局部估计可按式(7.2.19)和式(7.2.20)合成以得到全局最优估计。

联邦滤波器的设计步骤可归纳如下：

（1）将子滤波器和主滤波器的初始估计协方差阵设置为组合系统初始值的 $\gamma_{i}(i=1,2,\cdots,N,m)$ 倍。 $\gamma_{i}$ 满足信息守恒原则式(7.2.47)。

（2）将子滤波器和主滤波器的过程噪声协方差阵设置为组合系统过程噪声协方差阵的 $\gamma_{i}$ 倍。

(3) 各子滤波器处理自己的量测信息, 获得局部估计。

（4）在得到各子滤波器的局部估计和主滤波器的估计后按式(7.2.19)和式(7.2.20)进行最优合成。

（5）用全局滤波解来重置各子滤波器和主滤波器的滤波值和协方差阵。

## 7.2.4 联邦滤波器的结构与性能分析

## 1. 联邦滤波器的6种结构

在设计联邦滤波器时，信息分配系数的确定是至关重要的。不同的值会有不同的联邦滤波的结构和特性（容错性、最优性、计算量等）。若令 $\beta_{i}=1/\gamma_{i}(i=1,\cdots,N,m)$ ，则6种不同的设计结构可表达如下：

(1) 第 1 类结构 $(\beta_{m}=1,\beta_{i}=0,$ “零化式”重置 $)$ 。这类结构如图 7.2.2 所示。

这时主滤波器分配到全部(状态运动方程)信息,由于子滤波器的过程噪声协方差阵为无穷,子滤波器状态方程已没有信息,所以子滤波器实际上不需用状态方程而只用量测方程来进行最小二乘估计。将这些估计值输给主滤波器作量测值。这时主滤波器的工作频率可低于子滤波的工作频率,因为局部滤波器的量测数据已经过最小二乘估计而平滑。由于工作频率低,主滤波器可用高阶状态方程,其中包含精确的INS模型。实际上这时子滤波器起了“数据压缩”的作用。由于 $\beta_{i}^{-1}Q\rightarrow\infty$ ,子滤波器的预报值 $\hat{X}_{i}(k/k-1)$ 的协方差阵 $P_{i}(k/k-1)$ 趋于无穷,所以不能通过新息 $\left[Z_{i}(k)-H_{i}\hat{X}_{i}(k/k-1)\right]$ 来检测k时刻子系统i的输出 $Z_{i}(k)$ 的故障。相反,主滤波器拥有了全部状态运动信息,状态方程还可用高阶精确模型,因此 $\hat{X}_{m}(k/k-1)$ 的协方差阵 $P_{m}(k/k-1)$ 很小,便于用主滤波器的新息来检测子系统 $(i=1\sim N)$ 的故障。

![](images/31488c475eab4e7815038b8771a9ca833f79a80e55242a2d035d6950b458d31a.jpg)  
图7.2.2 联邦滤波器结构1

此外，由于子滤波器状态信息只被重置到零（“零化”式重置），这样就减少了主滤波器到子滤波器的数据传输，因此数据通讯量下降。各子滤波器协方差被重置为无穷，因此不须时间更新计算，计算变得简单。图7.2.2中虚线表示简单的零化重置(zero reset)。

(2) 第 2 类结构 $\left(\beta_{m}=\beta_{i}=1/(N+1)\right.$ ，有重置）。这类结构如图 7.2.3 所示。

![](images/e0fedc62d300642ebf998f857d5cae97d68075d7df52abdc535ad960acba7e60.jpg)  
图7.2.3 联邦滤波器结构2

这时，信息在主滤波器和各子滤波器之间平均分配。融合后的全局滤波精度高，局部滤波因为有全局滤波的反馈重置，其精度也提高了。用全局滤波和局部滤波的新息都可以更好地进行故障检测。在某个子系统故障被隔离后，其他良好的局部滤波器的估计值作为替代值的能力也提高了，但重置使得局部滤波受全局滤波的反馈影响。这样，一个子系统的故障可以通过全局滤波的反馈重置而使具有良好子系统的局部滤波也受到污染，于是容错性能下降。故障隔离后,局部滤波器要重新初始化,于是要经过一段过渡时间后其滤波值才能使用,这样故障恢复能力就下降了。这种设计中,主滤波器的模型阶次可高些,特别可采用更精确的INS模型。

（3）第3类结构 $(\beta_{m} = 0, \beta_{i} = 1 / N$ ，有重置）。这类结构如图7.2.4所示。

这时主滤波器状态方程无信息分配，也就是 $\beta_{m}^{-1}Q$ 为无穷，不需用主滤波器进行滤波，所以主滤波器的估计值就取为全局估计，即

$$
\hat {\boldsymbol {X}} _ {m} = \hat {\boldsymbol {X}} _ {\mathrm{g}} = \boldsymbol {P} _ {\mathrm{g}} (\boldsymbol {P} _ {1} ^ {- 1} \hat {\boldsymbol {X}} _ {1} + \dots + \boldsymbol {P} _ {N} ^ {- 1} \hat {\boldsymbol {X}} _ {N})
$$

由重置带来的问题与第2类结构相同。

![](images/7f6807dbfa8a3ad3c25e702fbf660266ae92ee3f8aa7da973030aefc3c0a4649.jpg)  
图7.2.4 联邦滤波器结构3

（4）第4类结构 $(\beta_{m} = 0,\beta_{i} = 1 / N$ ，无重置）。这类结构如图7.2.5所示。

![](images/43aa0d0cb9b48c3b239318faaef194868cf970bf7fbfa8e1ee1f73cf1005b71d.jpg)  
图 7.2.5 联邦滤波器结构 4

这种设计与第 3 类相比只是没有重置,所以各局部滤波器独立滤波,没有反馈重置带来的相互影响,这就提供了最高的容错性能。但由于没有全局最优估计的重置,所以局部估计的精度不高。

以上 4 类结构都采用了信息分配原则,因此多个局部滤波器都要根据不同的信息分配方法来重新设计,1988 年发表的文献[42]称它们为下一代组合导航的设计方案。对于当前应用的组合导航,其组合滤波器已设计好了,如果保留这个子滤波器不变,再加上一级主滤波器就可以构成所谓“串级”(cascaded)滤波器,它们也属于联邦滤波器的一类。为什么要在原有的单个的子滤波器上再加一级主滤波器呢?其理由如下:

1）主滤波器使用的 INS 模型可以比已有的子滤波器所用的 INS 模型更精确，模型阶次可以更高。

2）可以加入新的子系统，用主滤波器来处理它们的新量测信息。

为了具体起见,假定我们已有 INS/GPS 滤波器,再加上一个新的子系统 2,如电光地标系统。可以构成下面两类的设计结构。

（5）第5类结构 $(\beta_{m} = 1, \beta_{1} = 0$ ，有重置）。这类结构如图7.2.6所示。

![](images/df17962ae8265facd9ce70660c463233242dd16080d421eeff7c6bb31aded71e.jpg)  
图 7.2.6 联邦滤波器结构 5

这类结构中的主滤波器包含所有状态信息，而子滤波器的信息阵在每次融合后被重置为零，然后子滤波器再重新启动，简称为零化启动（restart）。局部滤波器起到数据压缩的作用。在融合周期内，局部滤波器积累 GPS 数据将它“压缩”，融合周期一到就将它们输给主滤波器。主滤波器的估计的量测更新用子系统 2 的数据。

(6) 第 6 类结构 $\left(\beta_{m}=\beta_{1}=0.5\right)$ ，有重置。这类结构如图 7.2.7 所示。

![](images/55bc8b6cffd2d9a76d91fa175042e232b13ce71746de3dcf3b1861e279debd25.jpg)  
图7.2.7 联邦滤波器结构6

这类设计中 $\beta_{m}=\beta_{1}=0.5$ ，在每次融合得到全局估计后，子滤波器的信息被重置到全局信息的一半，或其协方差阵重置为全局估计协方差阵的1倍，即 $P_{1}^{-1}=\frac{1}{2}P_{g}^{-1},P_{m}^{-1}=\frac{1}{2}P_{g}^{-1}$ 。这个重置过程在文献[42]中称为信息再分(rescale)。主滤波器的量测更新用新子系统的数据。

## 2. 信息融合对非公共状态的影响

我们知道，子滤波器和主滤波器的状态向量都包含公共状态 $X_{c}$ 和各自的子系统的误差状态 $X_{bi}(i=1,2,\cdots,N,m)$ ，只有对公共状态才能进行信息融合以获得全局估计。各子系统的误差状态由各自的子滤波器来估计。但公共状态和子系统的误差状态是有交联的。局部滤波器的协方差阵可写为

$$
\boldsymbol {P} _ {i} = \left[ \begin{array}{c c} \boldsymbol {P} _ {c _ {i}} & \boldsymbol {P} _ {c _ {i} b _ {i}} \\ \boldsymbol {P} _ {b _ {i} c _ {i}} & \boldsymbol {P} _ {b _ {i}} \end{array} \right]
$$

式中 $P_{c_{i}b_{i}}$ 和 $P_{b_{i}c_{i}}$ 就是公共状态和子系统误差状态的交联项。在联邦滤波时由于信息分配和主滤波器对子滤波器的重置，公共状态的协方差阵 $P_{c_{i}}$ 会发生变化，例如公共状态估计精度提高， $P_{c_{i}}$ 下降。这样，通过状态间的耦合影响， $P_{b_{i}}$ 也将下降，即子系统的误差的估计也会有一些改善。

## 3. 联邦滤波器容错性能分析

在容错组合导航的设计中,联邦滤波器起到了关键的作用,因此有必要对它的各种结构的容错性能作更深入的分析,并与集中滤波器的容错性能进行比较。所谓“容错”,它包含了故障检测、故障隔离和故障恢复(简称为 FDIR)。

（1）集中滤波器的容错性能分析。集中滤波器(CF)用一个滤波器来处理所有的子系统的量测信息,它的容错性能可分析如下:

## 1）利用测量值残差（即新息） $r_{k}$

$$
\boldsymbol {r} _ {k} = \mathbf {Z} _ {k} - \boldsymbol {H} \hat {\mathbf {X}} _ {k / k - 1} = \mathbf {Z} _ {k} - \hat {\mathbf {Z}} _ {k}
$$

可以较好地检测和隔离某些子系统的突变故障(硬故障)。因为 $\hat{X}_{k/k-1}$ 包含了 k 以前的量测值 $Z_{i}(i<k)$ 的信息，当无故障时， $H\hat{X}_{k/k-1}=\hat{Z}_{k}$ 是对 $Z_{k}$ 的最好的预报估计，所以 $r_{k}$ 应很小（理论上为零均值白噪声）。当 $Z_{k}$ 发生突变故障时， $r_{k}$ 也会发生突变，据此就可以检测和隔离子系统故障。

2）新息检验对软故障的检测不是很有效的。这是因为软故障是逐渐发展的，开始时故障很小，不易被检测出来。未被检测的故障将污染 $\hat{\mathbf{X}}_{k / k - 1}$ ，使得 $\hat{\mathbf{X}}_{k / k - 1}$ “跟踪”故障，减少了 $\hat{\mathbf{Z}}_k$ 与 $Z_{k}$ 的差异，这时 $r_k$ 不会发生大的变化，因此故障检测效果不好。

3）集中滤波器的故障恢复能力不强。因为在故障子系统被隔离后，已被故障污染的滤波解必须重新恢复正常。这就需要重新初始化已隔离了故障子系统后的集中滤波器，重新利用无故障的子系统的新的信息（系统重构），于是必须经过一段过渡过程后，系统的滤波解才恢复正常。这样，系统不能立即恢复正常，即故障恢复能力差。

（2）联邦滤波器容错性能分析。相对于集中滤波器来讲，联邦滤波器的容错性能要强得多。它具有以下优点：

1）因为融合周期可以长于子滤波的周期，于是在融合之前，软故障可以有较长的时间去发展到可被主滤波器检测的程度。

2）子滤波器自身的子系统误差状态是分开估计的。这些子系统的误差状态在子滤波周期内不会受其他子系统的故障影响，只有在较长的融合周期之后才会有影响。

3）某一子系统的故障被检测和隔离后，其他正常的子滤波器的解仍存在（只要没有重置发生），于是利用这些正常的子滤波器的解经过简单的融合算法可立即得到全局解，因此故障恢复的能力很强。

4）主滤波器可以使用一个比子滤波器甚至比集中滤波器更精确的 INS 模型，这样检测 INS 故障的能力就提高了。

下面再归纳一下前面提到的 6 类联邦滤波结构的容错性能：

第 1 类结构( $\beta_{m}=1,\beta_{i}=0$ ,“零化式”重置)

这种结构的子滤波器的故障检测和隔离(FDI)能力很差。这是因为子滤波器状态信息分配为零，协方差趋于无穷。而主滤波器拥有未发生故障前的全部信息，因此FDI能力强，它还有对INS故障的强的FDI能力。这种结构故障恢复(FR)能力中等，在有故障子系统的数据被主滤波器使用后，全局解将受到污染。虽然这时其他正常的子滤波器未被污染，但它们却只起最小二乘估计作用，它们的解不具有“长记忆”特性，不能外推使用。因此，不能用它们来使主滤波器迅速地进行故障恢复。主滤波器在故障子系统隔离后必须重新初始化，经过一段过渡时间后才能从故障中恢复。

第 2 类结构( $\beta_{m}=\beta_{i}=1/(N+1)$ ，有重置)

这种结构的子滤波器的 FDI 能力较好。主滤波器的 FDI 能力中等。在一个子系统发生故障后，主滤波器将受到它的污染，再通过重置使其他子滤波器污染，故在故障子系统被隔离后，主子滤波器都要重新初始化，FR 能力与第 1 种结构以及集中滤波器相同。

第3类结构 $(\beta_{m} = 0, \beta_{i} = 1 / N$ ，有重置）

容错性能基本上与第2种结构一样。但 $\beta_{m}=0$ ，主滤波器FDI能力差。

第 4 类结构( $\beta_{m}=0,\beta_{i}=1/N$ ,无重置)

由于无重置,各子滤波器不会互相影响,因此这种结构的容错性能最好。

第 5 类结构( $\beta_{m}=1,\beta_{1}=0$ ,有重置)

子滤波器的 FDI 能力差。主滤波器的 FDI 能力强，但 FR 能力差（因为要重新初始化）。

第 6 类结构( $\beta_{m}=\beta_{1}=0.5$ ,有重置)

主、子滤波器的 FDI 能力中等。同样，主滤波器的 FR 能力差。

总之，如果不将融合后的全局状态估计和协方差阵去反馈重置子滤波器，那么就不会产生子滤波器的交叉污染，子滤波器的精度虽下降，但联邦滤波器的容错性能却大大提高了。

## 7.3 系统级故障检测与隔离的原理与方法

组合导航系统的容错设计是提高组合导航系统任务可靠性的重要途径。容错设计的出发点是从系统的整体设计上来提高其可靠性，而不是去提高每一个元部件的基本可靠性。容错设计的主要方法是使系统具有自监控的功能，通过监控系统的运行状态，实时地检测并隔离故障部件，进而采取必要措施，切换掉故障部件，将正常的部件重新组合起来（系统重构），从而使整个系统在内部有故障的情况下仍能正常工作或降低性能安全地工作。

故障检测与隔离(FDI)是一项专门的技术,它已有很大的发展。现有的 FDI 方法可以分为以下几大类:

(1) 基于硬件余度的方法。即采用多套相同的硬件(如传感器)，检查它们输出的一致性

来实现 FDI。

（2）基于解析余度的方法。解析余度是基于数学模型的方法。例如不同功能的传感器所测量的变量是不同的，但它们的输出之间可能存在一定的解析关系，这些关系可以用静态的或动态的数学模型来描述，如测量方程和状态方程。利用这种关系可以进行 FDI。解析余度方法又可主要分为参数估计法和状态估计法。导航系统的 FDI 常采用状态估计法，主要是卡尔曼滤波方法。

（3）基于人工智能的方法。如专家系统、模糊决策、神经元网络等。

Kerr $^{[39]}$ 曾指出导航系统 FDI 所要考虑的特殊因素和现有 FDI 方法应用于导航系统时的局限性。

导航系统 FDI 所要考虑的特殊因素有：

(1) 软故障的类型及其严重性。

(2) 故障的可检测性问题。

(3) 软故障的累积时间。

(4) 不同类型故障的可区分程度。

(5) 故障恢复的难易程度。

现有 FDI 方法应用于导航系统的局限性主要有：

(1) 现有方法一般针对时不变系统,而导航系统通常要用时变线性的误差方程来描述。

(2) 现有方法一般针对低阶系统,而导航系统的误差模型一般很复杂。

（3）现有方法常假设系统模型与卡尔曼滤波模型一致，而导航系统常使用降阶滤波器。在无故障情况下，滤波器的残差也不是白噪声。

（4）现有方法常针对大的信噪比情况，而导航系统的信噪比却不大。

在容错组合导航系统中，必须实时地确定各子滤波器处理的量测信息的有效性，以便决定用哪些局部状态估计来计算整体状态估计。这就要求在子滤波器的设计中，应配备实时的故障检测及隔离算法。一旦检测到故障就必须对故障进行隔离，最后通过系统信息重构使整体系统不致因故障而失效。这实际上是一个动态系统的故障检测及隔离问题。早在70年代初，人们就开始研究有关动态系统故障检测及隔离的问题，至今已有许多故障检测及隔离方法。针对组合导航系统的特点，这里将采用一种 $\chi^{2}$ 检验法来确定系统量测信息的有效性。该方法并不确定造成故障的具体原因，而仅仅是实时地确定一组量测值的有效性，因而它十分适用于系统级的故障检测及隔离。

在子系统的故障被检测与隔离后,我们可进一步对其内部的元器件进行故障检测和隔离。前面已提到惯导系统在组合导航中常作为参考系统,它的可靠性更为重要,所以在7.4节中将要讨论惯性器件的故障检测和隔离原理,至于其他器件如GPS,多普勒等子导航系统的器件就不讨论了。

## 7.3.1 系统级的故障检测与隔离

考虑带故障的离散系统模型：

$$
\left. \begin{array}{l} \boldsymbol {X} _ {k} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {X} _ {k - 1} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {W} _ {k - 1} \\ \boldsymbol {Z} _ {k} = \boldsymbol {H} _ {k} \boldsymbol {X} _ {k} + \boldsymbol {V} _ {k} + f _ {k, \varphi} \boldsymbol {\gamma} \end{array} \right\}\tag{7.3.1}
$$

式中， $Z_{k}\in R^{m}$ 是系统的量测值； $X_{k}\in R^{n}$ 是系统状态； $\Phi_{k,k-1}\in R^{n\times n}$ 是系统状态的一步转移矩

阵； $\Gamma_{k-1}\in R^{n\times r}$ 是系统噪声矩阵； $W_{k}\in R^{r}$ 和 $V_{k}\in R^{m}$ 是相互独立的高斯白噪声序列，且有

$$
\begin{array}{l l} E \{\boldsymbol {W} _ {k} \} = \mathbf {0}, & E \{\boldsymbol {W} _ {k} \boldsymbol {W} _ {j} ^ {\mathrm{T}} \} = Q _ {k} \delta_ {k j} \\ E \{\boldsymbol {V} _ {k} \} = \mathbf {0}, & E \{\boldsymbol {V} _ {k} \boldsymbol {V} _ {j} ^ {\mathrm{T}} \} = R _ {k} \delta_ {k j} \end{array}
$$

式中， $\delta_{kj}$ 为克朗尼克 $\delta$ 函数； $\gamma$ 是随机向量，它表示故障的大小； $f_{k,\varphi}$ 是分段函数

$$
f _ {k, \varphi} = \left\{ \begin{array}{l l} 1, & k \geqslant \varphi \\ 0, & k <   \varphi \end{array} \right.
$$

式中 $\varphi$ 是故障发生的时间。

初始状态 $X_{0}$ 是独立于噪声 $W_{k}$ 和 $V_{k}$ 的高斯随机向量，且

$$
E \{\mathbf {X} _ {0} \} = \mathbf {X} ^ {0}, \quad E \{\mathbf {X} _ {0} \mathbf {X} _ {0} ^ {\mathrm{T}} \} = \mathbf {P} ^ {0}
$$

1. 状态 $\chi^2$ 检验法

状态 $\chi^2$ 检验法利用两个状态估计的差异： $\hat{\mathbf{X}}_k$ 是由量测值 $\mathbf{Z}_k$ 经卡尔曼滤波得到的； $\hat{\mathbf{X}}_k^S$ 则是由所谓“状态递推器”或“影子滤波器”用先验信息递推计算而得的。前者和量测信息有关，因而会受到子系统故障的影响，而后者和量测信息无关，因而不受故障的影响。利用两者之间的这种差异便可以对故障进行检测和隔离。两个状态估计 $\hat{\mathbf{X}}_k$ 和 $\hat{\mathbf{X}}_k^S$ 可用下列公式计算：

$$
\left. \begin{array}{l} \hat {\mathbf {X}} _ {k} = [ \mathbf {I} - \mathbf {K} _ {k} \mathbf {H} _ {k} ] \boldsymbol {\Phi} _ {k, k - 1} \hat {\mathbf {X}} _ {k - 1} + \mathbf {K} _ {k} \mathbf {Z} _ {k} \\ \hat {\mathbf {X}} _ {0} = \mathbf {X} ^ {0} \\ \mathbf {P} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \mathbf {P} _ {k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} + \boldsymbol {\Gamma} _ {k - 1} \mathbf {Q} _ {k - 1} \boldsymbol {\Gamma} _ {k - 1} ^ {\mathrm{T}} \\ \mathbf {P} _ {k} = [ \mathbf {I} - \mathbf {K} _ {k} \mathbf {H} _ {k} ] \mathbf {P} _ {k / k - 1} \\ \mathbf {P} _ {0} = \mathbf {P} ^ {0} \\ \mathbf {K} _ {k} = \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} [ \mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} + \mathbf {R} _ {k} ] ^ {- 1} \\ \hat {\mathbf {X}} _ {k} ^ {\mathrm{S}} = \boldsymbol {\Phi} _ {k, k - 1} \hat {\mathbf {X}} _ {k - 1} ^ {\mathrm{S}} \\ \hat {\mathbf {X}} _ {0} ^ {\mathrm{S}} = \mathbf {X} ^ {0} \\ \mathbf {P} _ {k} ^ {\mathrm{S}} = \boldsymbol {\Phi} _ {k, k - 1} \mathbf {P} _ {k - 1} ^ {\mathrm{S}} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} + \boldsymbol {\Gamma} _ {k - 1} \mathbf {Q} _ {k - 1} \boldsymbol {\Gamma} _ {k - 1} ^ {\mathrm{T}} \\ \mathbf {P} _ {0} ^ {\mathrm{S}} = \mathbf {P} ^ {0} \end{array} \right\}\tag{7.3.2}
$$

(7.3.3)

由于 $X^{0}$ 是高斯随机向量，故 $X_{k}, \hat{X}_{k}, \hat{X}_{k}^{S}$ 均为高斯随机向量。定义估计误差 $e_{k}^{K}$ 和 $e_{k}^{S}$ 为

$$
\left. \begin{array}{l} \boldsymbol {e} _ {k} ^ {K} = \boldsymbol {X} _ {k} - \hat {\boldsymbol {X}} _ {k} \\ \boldsymbol {e} _ {k} ^ {S} = \boldsymbol {X} _ {k} - \hat {\boldsymbol {X}} _ {k} ^ {S} \end{array} \right\}\tag{7.3.4}
$$

并定义

$$
\boldsymbol {\beta} _ {k} = \boldsymbol {e} _ {k} ^ {K} - \boldsymbol {e} _ {k} ^ {S} = \hat {\boldsymbol {X}} _ {k} ^ {S} - \hat {\boldsymbol {X}} _ {k}\tag{7.3.5}
$$

$\pmb{\beta}_{k}$ 的方差为

$$
\begin{array}{r l} \mathbf {T} _ {k} & = E \{\boldsymbol {\beta} _ {k} \boldsymbol {\beta} _ {k} ^ {\mathrm{T}} \} = E \{\boldsymbol {e} _ {k} ^ {K} (\boldsymbol {e} _ {k} ^ {K}) ^ {\mathrm{T}} - \boldsymbol {e} _ {k} ^ {K} (\boldsymbol {e} _ {k} ^ {S}) ^ {\mathrm{T}} - \boldsymbol {e} _ {k} ^ {S} (\boldsymbol {e} _ {k} ^ {K}) ^ {\mathrm{T}} + \boldsymbol {e} _ {k} ^ {S} (\boldsymbol {e} _ {k} ^ {S}) ^ {\mathrm{T}} \} = \\ & \quad \boldsymbol {P} _ {k} + \boldsymbol {P} _ {k} ^ {S} - \boldsymbol {P} _ {k} ^ {K S} - (\boldsymbol {P} _ {k} ^ {K S}) ^ {\mathrm{T}} \end{array}\tag{7.3.6}
$$

由于 $\beta_{k}$ 是高斯随机向量 $e_{k}^{K}$ 和 $e_{k}^{S}$ 的线性组合，所以它也是高斯随机向量，且其均值为零，方差为 $T_{k}$ 。

当系统发生故障时，由于估计 $\hat{\pmb{X}}_k^S$ 和量测值 $Z_{k}$ 无关，所以仍是无偏估计，即 $E\{\pmb{e}_k^S\} = \mathbf{0}$ ；而估计 $\hat{\pmb{X}}_k$ 因受故障影响变成了有偏估计，即 $E\{\pmb{e}_k^K\} \neq \mathbf{0}$ 。由式(7.3.5)可知 $E\{\pmb{\beta}_k\} \neq \mathbf{0}$ ，即 $\pmb{\beta}_k$ 的均值不为零。因此，通过对 $\pmb{\beta}_k$ 均值的检验可确定系统是否发生了故障。

根据 7.5 节中的故障检测的统计原理, 对 $\beta_{k}$ 作以下二元假设:

$H_{0}$ ——无故障

$H_{1}$ ——有故障

$$
\begin{array}{r} E \{\pmb {\beta} _ {k} \} = \mathbf {0}, E \{\pmb {\beta} _ {k} \pmb {\beta} _ {k} ^ {\mathrm{T}} \} = \pmb {T} _ {k} \\ E \{\pmb {\beta} _ {k} \} = \pmb {\mu}, E \{\pmb {\beta} _ {k} - \pmb {\mu} \} (\pmb {\beta} _ {k} - \pmb {\mu}) ^ {\mathrm{T}} \} = \pmb {T} _ {k} \end{array}
$$

由于 $\beta_{k} \in R^{n}$ 是高斯随机向量，故有以下条件概率密度函数：

$$
p _ {r} \left(\boldsymbol {\beta} / H _ {0}\right) = \frac {1}{\sqrt {2 \pi} | \boldsymbol {T} _ {k} | ^ {\frac {1}{2}}} \exp \left[ - \frac {1}{2} \boldsymbol {\beta} _ {k} ^ {\mathrm{T}} \boldsymbol {T} _ {k} ^ {- 1} \boldsymbol {\beta} _ {k} \right]\tag{7.3.7}
$$

$$
p _ {r} (\boldsymbol {\beta} / H _ {1}) = \frac {1}{\sqrt {2 \pi} | \boldsymbol {T} _ {k} | ^ {\frac {1}{2}}} \exp \left[ - \frac {1}{2} (\boldsymbol {\beta} _ {k} - \boldsymbol {\mu}) ^ {\mathrm{T}} \boldsymbol {T} _ {k} ^ {- 1} (\boldsymbol {\beta} _ {k} - \boldsymbol {\mu}) \right]\tag{7.3.8}
$$

式(7.3.7)，式(7.3.8)中 $|\cdot|$ 表示行列式值。

$p_{r}(\boldsymbol{\beta}/H_{0})$ 和 $p_{r}(\boldsymbol{\beta}/H_{1})$ 的对数似然比为

$$
\Lambda_ {k} = \ln \frac {p _ {r} (\boldsymbol {\beta} / H _ {0})}{p _ {r} (\boldsymbol {\beta} / H _ {1})} = \frac {1}{2} \{\boldsymbol {\beta} _ {k} ^ {\mathrm{T}} \boldsymbol {T} _ {k} ^ {- 1} \boldsymbol {\beta} _ {k} - (\boldsymbol {\beta} _ {k} - \boldsymbol {\mu}) ^ {\mathrm{T}} \boldsymbol {T} _ {k} ^ {- 1} (\boldsymbol {\beta} _ {k} - \boldsymbol {\mu}) \}\tag{7.3.9}
$$

由于式(7.3.9)中的 $\mu$ 是未知的，故用其极大似然估计 $\hat{\mu}$ 代替。求 $\hat{\mu}$ 使 $\Lambda_{k}$ 达到极大，得

$$
\hat {\boldsymbol {\mu}} _ {k} = \boldsymbol {\beta} _ {k}\tag{7.3.10}
$$

将式 $(7.3.10)$ 代入式 $(7.3.9)$ ，并不考虑系数1/2，便得到以下故障检测函数：

$$
\lambda_ {k} = \boldsymbol {\beta} _ {k} ^ {\mathrm{T}} \boldsymbol {T} _ {k} ^ {- 1} \boldsymbol {\beta} _ {k}\tag{7.3.11}
$$

可以证明 $\lambda_{k}$ 服从自由度为 n 的 $\chi^{2}$ 分布，即 $\lambda_{k} \sim \chi^{2}(n)$ 。

定义: 设 $X_{1}, X_{2}, \cdots, X_{n}$ 互相独立, 且都具有分布 $N(0,1)$ , 则随机变量 $\sum_{i=1}^{n} X_{i}^{2}$ 的分布称为具有 n 个自由度的中心 $\chi^{2}$ 分布, 并记为 $\gamma \sim \chi^{2}(n)$ 。 $\chi^{2}(n)$ 的概率密度函数为

$$
\chi^ {2} (\lambda , n) = \left\{ \begin{array}{l l} \left[ 2 ^ {n / 2} \Gamma \left(\frac {n}{2}\right) \right] ^ {- 1} \lambda^ {\frac {n}{2} - 1} \mathrm{e} ^ {- \frac {\lambda}{2}}, & \lambda > 0 \\ 0, & \lambda \leqslant 0 \end{array} \right.\tag{7.3.12}
$$

式中 $n$ 为自由度， $\Gamma \left(\frac{n}{2}\right)$ 为 $\Gamma$ 函数，即

$$
\Gamma \left(\frac {n}{2}\right) = \int_ {0} ^ {\infty} \lambda^ {\frac {n}{2} - 1} \mathrm{e} ^ {- \lambda} \mathrm{d} \lambda
$$

中心 $\chi^{2}(\lambda,n)$ 的图形如图 7.3.1 所示。

![](images/f900d2502eec47d2d22ea018c54134446d7d1baec62fa399524597ed596e2be0.jpg)  
图7.3.1 $\chi^2 (\lambda ,n)$ 的概率密度函数

现在来证明 $\lambda_{k} = \pmb{\beta}_{k}^{\mathrm{T}}\pmb{T}_{k}^{-1}\pmb{\beta}_{k}$ 服从 $\chi^2 (n)$ 分布。 $\pmb{T}_k$ 为协方差阵，故是正定的，于是 $\pmb{T}_k^{-1}$ 也是正

定的,因此有分解

$$
\mathbf {T} _ {k} ^ {- 1} = \mathbf {A} ^ {\mathrm{T}} \mathbf {A}
$$

式中 A 为非奇异。则

$$
\lambda_ {k} = \boldsymbol {\beta} _ {k} ^ {\mathrm{T}} \boldsymbol {T} _ {k} ^ {- 1} \boldsymbol {\beta} _ {k} = (\boldsymbol {A} \boldsymbol {\beta} _ {k}) ^ {\mathrm{T}} (\boldsymbol {A} \boldsymbol {\beta} _ {k}) = \boldsymbol {\alpha} ^ {\mathrm{T}} \boldsymbol {\alpha} = \boldsymbol {\alpha} _ {1} ^ {2} + \boldsymbol {\alpha} _ {2} ^ {2} + \dots + \boldsymbol {\alpha} _ {n} ^ {2}
$$

要证明 $\alpha_{i}$ 与 $\alpha_{j}$ 独立，这只要证明协方差阵 $E[\alpha \alpha^{\mathrm{T}}]$ 为对角阵即可。因为

$$
E [ \alpha \alpha^ {\mathrm{T}} ] = E [ (\mathbf {A} \boldsymbol {\beta} _ {k}) (\mathbf {A} \boldsymbol {\beta} _ {k}) ^ {\mathrm{T}} ] = \mathbf {A} E [ \boldsymbol {\beta} _ {k} \boldsymbol {\beta} _ {k} ^ {\mathrm{T}} ] \mathbf {A} ^ {\mathrm{T}} = \mathbf {A} \mathbf {T} _ {k} \mathbf {A} ^ {\mathrm{T}} = \mathbf {A} [ \mathbf {A} ^ {- 1} (\mathbf {A} ^ {\mathrm{T}}) ^ {- 1} ] \mathbf {A} ^ {\mathrm{T}} = \mathbf {I}
$$

根据上面的定义,即有 $\lambda_{k} \sim \chi^{2}(n)$ 。

故障判定准则为

$$
\left\{ \begin{array}{l l} {\text {若} \lambda_ {k} > T _ {D},} & {\text {判定有故障}} \\ {\text {若} \lambda_ {k} \leqslant T _ {D},} & {\text {判定无故障}} \end{array} \right.
$$

其中 $T_{D}$ 是预先设置的门限，它决定了故障检测的性能。由奈曼-皮尔逊准则可知（见7.5节），当限定误警率 $P_{f} = \alpha$ 时，则由 $P_{f} = P[\lambda_{k} > T_{D} / H_{0}] = \alpha$ 解出的门限 $T_{D}$ 可使漏检率 $P[\lambda_k \leqslant T_D / H_1]$ 达到最小，因而 $T_{D}$ 可由误警率 $P_{f}$ 确定。

误检率 $P_{f}=\alpha$ 由图 7.3.1 中门限之外的曲线下的面积给定，即

$$
P _ {f} = \int_ {T _ {D}} ^ {\infty} \chi^ {2} (\lambda , n) \mathrm{d} \lambda = 1 - \int_ {0} ^ {T _ {D}} \chi^ {2} (\lambda , n) \mathrm{d} \lambda\tag{7.3.13}
$$

给定 $P_{f} = \alpha$ ，就可由 $\chi^2$ 分布求出门限值 $T_{D}$ 。 $\chi^2$ 分布的部分内容见表7.3.1。

表 7.3.1 ${\chi }^{2}$ 分布的门限值 ${T}_{D}$

<table><tr><td> $\alpha$ \n</td><td>1</td><td>2</td><td>3</td></tr><tr><td> $10^{-1}$ </td><td>2.71</td><td>4.61</td><td>6.25</td></tr><tr><td> $10^{-2}$ </td><td>6.63</td><td>9.21</td><td>11.34</td></tr><tr><td> $10^{-3}$ </td><td>10.83</td><td>13.82</td><td>16.27</td></tr><tr><td> $10^{-4}$ </td><td>15.14</td><td>18.42</td><td>21.11</td></tr><tr><td> $10^{-5}$ </td><td>19.51</td><td>23.03</td><td>25.90</td></tr><tr><td> $10^{-6}$ </td><td>23.93</td><td>27.63</td><td>30.66</td></tr><tr><td> $10^{-7}$ </td><td>28.37</td><td>32.24</td><td>35.41</td></tr><tr><td> $10^{-8}$ </td><td>32.84</td><td>36.84</td><td>40.13</td></tr><tr><td> $10^{-9}$ </td><td>37.32</td><td>41.45</td><td>44.84</td></tr></table>

例如，当 $n = 3, P_{f} = \alpha = 10^{-3}$ 时， $T_{D} = 16.27$ 。

为了计算检测函数 $\lambda_{k}$ ，要求 $T_{k}$ 是已知的，而由式(7.3.6)可知，这就要求计算两个估计误差 $e_{k}^{K}$ 和 $e_{k}^{S}$ 的相关协方差阵 $P_{k}^{KS}$ 。由于两个状态估计 $\hat{X}_{k}$ 和 $\hat{X}_{k}^{S}$ 具有相同的初始条件并受到同样

的噪声干扰，故该相关协方差一般不为零。

由式(7.3.1)～式(7.3.3)的第一式可得以下估计误差的表达式

$$
\left. \begin{array}{l} e _ {k} ^ {K} = [ I - K _ {k} H _ {k} ] \Phi_ {k, k - 1} e _ {k - 1} ^ {K} + [ I - K _ {k} H _ {k} ] \Gamma_ {k - 1} W _ {k - 1} - K _ {k} V _ {k} ] \\ e _ {k} ^ {S} = \Phi_ {k, k - 1} e _ {k - 1} ^ {S} + \Gamma_ {k - 1} W _ {k - 1} \end{array} \right\}\tag{7.3.14}
$$

因 $V_{k}$ 与 $W_{k-1}$ 不相关，且它们分别与 $e_{k-1}^{S}$ 和 $e_{k-1}^{K}$ 不相关，可得

$$
\begin{array}{r l} \boldsymbol {P} _ {k} ^ {K S} & = E \{\boldsymbol {e} _ {k} ^ {K} (\boldsymbol {e} _ {k} ^ {S}) ^ {\mathrm{T}} \} = [ \boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k} ] \boldsymbol {\Phi} _ {k, k - 1} E \{\boldsymbol {e} _ {k - 1} ^ {K} (\boldsymbol {e} _ {k - 1} ^ {S}) ^ {\mathrm{T}} \} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} + \\ & [ \boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k} ] \boldsymbol {\Gamma} _ {k - 1} E \{\boldsymbol {W} _ {k - 1} \boldsymbol {W} _ {k - 1} ^ {\mathrm{T}} \} \boldsymbol {\Gamma} _ {k - 1} ^ {\mathrm{T}} \end{array}
$$

即

$$
\boldsymbol {P} _ {k} ^ {K S} = \left[ \boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k} \right] \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {P} _ {k - 1} ^ {K S} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} + \left[ \boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k} \right] \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {Q} _ {k - 1} \boldsymbol {\Gamma} _ {k - 1} ^ {\mathrm{T}}\tag{7.3.15}
$$

又由式(7.3.2)的第3,4式可得

$$
\boldsymbol {P} _ {k} = \left[ \boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k} \right] \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {P} _ {k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} + \left[ \boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k} \right] \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {Q} _ {k - 1} \boldsymbol {\Gamma} _ {k - 1} ^ {\mathrm{T}}\tag{7.3.16}
$$

故若取初值 $P_0 = P_0^{KS} = P^0$ ，则有

$$
\boldsymbol {P} _ {k} = \boldsymbol {P} _ {k} ^ {K S}\tag{7.3.17}
$$

将式 $(7.3.17)$ 代入式 $(7.3.6)$ 得

$$
\boldsymbol {T} _ {k} = \boldsymbol {P} _ {k} ^ {S} - \boldsymbol {P} _ {k}\tag{7.3.18}
$$

用一个状态递推器的 $\chi^2$ 检验的缺点是：

在一般卡尔曼滤波器中，初值误差、系统噪声和建模误差的影响将会由量测更新来克服，使系统滤波精度逐渐提高。但在“状态递推器”中没有量测更新，所以这些误差将使状态递推值越来越偏离真实值，因此无故障时 $\beta_{k}$ 的值也越来越大，其方差 $T_{k}$ 也逐渐增大，这就降低了故障检测的灵敏度。文献[111]中提出了用两个“状态递推器”的方法。两个状态递推器交替工作，一个作故障检测用时，另一个被卡尔曼滤波器的输出所校正，下一个周期两者的作用反过来。这样就可保持状态递推器的误差不逐渐扩大。

## 2. 残差 $\chi^{2}$ 检验法

上一小节介绍的 $\chi^{2}$ 检验法要求计算两个状态估计 $\hat{X}_{k}$ 和 $\hat{X}_{k}^{S}$ ，这无疑将增加机载计算机的负担。为了克服这一缺点，以下给出一种用残差 $\chi^{2}$ 检验对系统的故障进行检测和隔离。

由前面的介绍得知,每个局部滤波器均为卡尔曼滤波器,其残差为

$$
\boldsymbol {r} _ {k} = \mathbf {Z} _ {k} - \boldsymbol {H} _ {k} \hat {\boldsymbol {X}} _ {k / k - 1}\tag{7.3.19}
$$

式中预报值 $\hat{\pmb{X}}_{k / k - 1}$ 为

$$
\hat {\boldsymbol {X}} _ {k / k - 1} = \boldsymbol {\Phi} _ {k / k - 1} \hat {\boldsymbol {X}} _ {k - 1}\tag{7.3.20}
$$

可以证明,当无故障发生时卡尔曼滤波器的残差 $r_{k}$ 是零均值高斯白噪声,而其方差为

$$
\mathbf {A} _ {k} = \mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} + \mathbf {R} _ {k}\tag{7.3.21}
$$

当系统发生故障时, 残差 $r_{k}$ 的均值就不再为零了。因此, 通过对残差 $r_{k}$ 的均值的检验亦可确定系统是否发生了故障。

同前面一样，对 $r_{k}$ 可作以下二元假设：

$H_{0}$ ，无故障

$H_{1}$ ，有故障

$$
\begin{array}{r} {E \{\boldsymbol {r} _ {k} \} = \mathbf {0}, E \{\boldsymbol {r} _ {k} \boldsymbol {r} _ {k} ^ {\mathrm{T}} \} = \boldsymbol {A} _ {k}} \\ {E \{\boldsymbol {r} _ {k} \} = \boldsymbol {\mu}, E \{\boldsymbol {r} _ {k} - \boldsymbol {\mu} ] [ \boldsymbol {r} _ {k} - \boldsymbol {\mu} ] ^ {\mathrm{T}} \} = \boldsymbol {A} _ {k}} \end{array}
$$

同理可推出故障检测函数

$$
\lambda_ {k} = \boldsymbol {r} _ {k} ^ {\mathrm{T}} \boldsymbol {A} _ {k} ^ {- 1} \boldsymbol {r} _ {k}\tag{7.3.22}
$$

式中 $\lambda_{k}$ 是服从自由度为 m 的 $\chi^{2}$ 分布，即 $\lambda_{k} \sim \chi^{2}(m)$ 。m 为测量 $Z_{k}$ 的维数。

故障判定准则为

$$
\left\{ \begin{array}{l l} {\text {若}   \lambda_ {k} > T _ {D},} & {\text {判定有故障}} \\ {\text {若}   \lambda_ {k} \leqslant T _ {D},} & {\text {判定无故障}} \end{array} \right.\tag{7.3.23}
$$

其中预先设置的门限 $T_{D}$ 同样可由误警率 $P_{f}$ 确定。

由于对每组量测信息都设计了一个局部滤波器,因而,在局部滤波器中加入上述故障检测及隔离算法即可确定失效的量测信息及相应的错误局部状态估计。用剩余的正确局部状态估计按7.2节中介绍的联邦滤波算法,可计算出可靠的系统状态估计。

残差 $\chi^{2}$ 检验对软故障的检测不十分有效。因为软故障开始很小，不易检测出来，有故障的输出将影响预报值 $\hat{X}_{k/k-1}$ ，使它“跟踪”故障输出，残差 $r_{k}$ 一直保持比较小，因此难于用 $r_{k}$ 来发现软故障。

## 7.3.2 组合导航系统容错设计

现以一个例子介绍容错组合导航系统的设计方法的应用(详见文献[112])。

设某组合导航系统包括三种导航系统：捷联惯导系统(SINS)、全球定位系统(GPS)和多普勒(Doppler)导航系统。考虑以SINS作为主导航系统并分别和GPS以及Doppler进行组合，得到两个子系统：SINS/GPS和SINS/Doppler。现为此SINS/GPS/Doppler组合导航系统设计一个容错方案。

首先假定此组合导航系统的误差状态方程为

量测方程为

$$
\begin{array}{r l} & {\dot {\mathbf {X}} (t) = \mathbf {F} (t) \mathbf {X} (t) + \mathbf {G} (t) \mathbf {W} (t)} \\ & {\quad \mathbf {Z} (t) = \mathbf {H} (t) \mathbf {X} (t) + \mathbf {V} (t)} \\ & {\mathbf {Z} _ {i} (t) = \mathbf {H} _ {k} (t) \mathbf {X} (t) + \mathbf {V} _ {i} (t) \quad i = 1, 2} \end{array}
$$

量测方程可分为两部分： $\mathbf{Z}_{i}(t) = \mathbf{H}_{k}(t) \mathbf{X}(t) + \mathbf{V}_{i}(t) \quad i = 1, 2$

式中， $Z_{1}(t)$ 是由子系统 SINS/GPS 得到的量测值， $Z_{2}(t)$ 是由子系统 SINS/Doppler 得到的量测值。 $Z_{1}(t)$ 可表示为

$$
\mathbf {Z} _ {1} (t) = \left[ \begin{array}{c} \boldsymbol {\rho} _ {\mathrm{G}} - \boldsymbol {\rho} _ {1} \\ \dot {\boldsymbol {\rho}} _ {\mathrm{G}} - \boldsymbol {\rho} _ {1} \end{array} \right] = \mathbf {H} _ {1} (t) \mathbf {X} (t) + \mathbf {V} _ {1} (t)
$$

式中， $\rho_{G}$ 和 $\dot{\rho}_{G}$ 为 GPS 得到的伪距和伪距率；而 $\rho_{I}$ 和 $\dot{\rho}_{I}$ 是 SINS 给出的飞行器到选定的 4 颗导航星的伪距和伪距率的估算值。 $Z_{2}(t)$ 可表示为

$$
\mathbf {Z} _ {2} (t) = \left[ \mathbf {v} _ {\mathrm{I}} - \mathbf {v} _ {\mathrm{D}} \right] = \mathbf {H} _ {2} (t) \mathbf {X} (t) + \mathbf {V} _ {2} (t)
$$

式中， $v_{1}$ 为 SINS 提供的飞行器速度， $v_{D}$ 为 Doppler 雷达测量到的地速。

将上述连续方程离散化,可得

$$
\begin{array}{l} \mathbf {X} _ {k} = \boldsymbol {\Phi} _ {k, k - 1} \mathbf {X} _ {k - 1} + \boldsymbol {\Gamma} _ {k - 1} \mathbf {W} _ {k - 1} \\ \mathbf {Z} _ {k} = \mathbf {H} _ {k} \mathbf {X} _ {k} + \mathbf {V} _ {k} \end{array}
$$

它们可写成以下子系统方程形式：

$$
\left\{ \begin{array}{l} \boldsymbol {X} _ {i k} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {X} _ {i k - 1} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {W} _ {k - 1} \\ \boldsymbol {Z} _ {i k} = \boldsymbol {H} _ {i k} \boldsymbol {X} _ {k} + \boldsymbol {V} _ {i k} \quad i = 1, 2, \end{array} \right.
$$

式中， $X_{ik}\in R^{n},Z_{1k}\in R^{8},Z_{2k}\in R^{2};W_{k}$ 和 $V_{k}$ 是相互独立的高斯白噪声，且有

$$
\boldsymbol {E} \left\{\boldsymbol {V} _ {k} \boldsymbol {V} _ {j} ^ {\mathrm{T}} \right\} = \boldsymbol {R} _ {k} \delta_ {k j}, \quad \boldsymbol {R} _ {k} = \operatorname{diag} \left\{\boldsymbol {R} _ {1 k}, \boldsymbol {R} _ {2 k} \right\}
$$

在设计的容错方案中,采用余度传感器结构来提高 SINS 的可靠性,并用 7.4 节中介绍的容错方案来保证组合导航系统中的 SINS 是可靠的。因此,只要通过子系统的故障检测,即可确定是何种导航系统发生了故障。

导航系统故障判定准则为：

若子系统 SINS/GPS 故障, 则判定导航系统 GPS 故障;

若子系统 SINS/Doppler 故障, 则判定导航系统 Doppler 故障。

针对两个子系统即SINS/GPS和SINS/Doppler设计两个局部滤波器 $(\beta_{1} = \beta_{2} = 1 / 2)$ ，由局部滤波器可获得组合导航系统误差状态的局部估计值。在每个局部滤波器中均采用前两节中介绍的故障检测及隔离方法，以确定子系统是否发生了故障。设计一个主滤波器 $(\beta_{m} = 0)$ ，利用7.2节中介绍的综合算法，将来自局部滤波器的局部状态估计综合起来，就得到了组合导航系统误差状态的全局估计值，然后利用这些估计值进行反馈校正。

容错系统的结构如图 7.3.2 所示。由图中可以看出，该容错系统包括：余度传感器结构的容错（SINS 容错算法），子系统的故障检测及隔离，容错综合算法。局部滤波器中的故障检测及隔离算法可确定子系统的故障，进而判定失效的导航系统。一旦确定了某个子系统发生了故障，则由相应局部滤波器得到的状态估计是不正确的，因而不将其输入到主滤波器。此时，主滤波器仅利用另一局部滤波器的状态估计值给出系统误差状态的估计值，同时对失效的导航系统进行故障修复或通道切换。一旦失效的导航系统修复完毕，则由相应的子系统输出驱动的局部滤波器又能获得正确的局部状态估计值，将其输入到主滤波器，则可使组合导航系统重新为用户提供可靠的、精确的导航信息。

设由两个局部滤波器得到的误差状态估计及估计误差方差分别为 $\hat{X}_{1}, P_{1}$ 和 $\hat{X}_{2}, P_{2}$ ，则当系统没有故障时，整体误差状态估计值为

式中

$$
\begin{array}{r l} & {\hat {X} _ {\mathrm{g}} = P _ {\mathrm{g}} (P _ {1} ^ {- 1} \hat {X} _ {1} + P _ {2} ^ {- 1} \hat {X} _ {2})} \\ & {\quad P _ {\mathrm{g}} = (P _ {1} ^ {- 1} + P _ {2} ^ {- 1}) ^ {- 1}} \end{array}
$$

若子系统 SINS/GPS 失效(对应于局部滤波器一), 则 $\hat{X}_{1}$ 是不正确的, 因而就不将其输入到主滤波器, 此时, 系统误差状态的整体估计为

$$
\hat {X} _ {\mathrm{g}} = \hat {X} _ {2}
$$

而失效子系统的输出可由下式估计：

$$
\hat {\mathbf {Z}} _ {1} = \mathbf {H} _ {1} \hat {\mathbf {X}} _ {\mathrm{g}}
$$

同理,当子系统 SINS/Doppler 失效时,系统误差状态的整体估计为

$$
\hat {X} _ {\mathrm{g}} = \hat {X} _ {1}
$$

而失效子系统的输出估计值为 $\hat{Z}_{2}=H_{2}\hat{X}_{g}$ 。

由此可以看出,按本章介绍的方法设计的容错组合导航系统即使在某些导航系统失效时,也能提供可靠的、精确的导航信息。

![](images/17450715c242c7a1e2648f00d5b7ef1d3aa1f1e21d3b726cf059d82e6c5b0793.jpg)  
图 7.3.2 SINS/GPS/Doppler 容错系统结构

## 7.4 惯性器件的故障检测与隔离原理

## 7.4.1 概述

前面已提到，在容错组合导航系统中惯导系统常作为参考系统，其可靠性是至关重要的。因此，对惯导系统本身常采用余度技术和容错设计以提高它的可靠性。对平台惯导系统来讲，只能采用两套或多套平台系统，再将它们安置于飞机内不同部位，以提高战斗中的生存能力。对捷联惯导系统来讲，可以在元器件级采用余度技术。构成余度惯性组件的器件数目要在4个以上，这样就有1个以上的余度。惯性器件的配置方式有两种，即测量轴正交配置与非正交（斜置）配置。从可靠性角度来看，为达到相同的容错性能，斜置式配置所需的器件数目要少于正交配置所需数目。本节只讨论捷联式惯性组件中器件的故障检测和隔离原理。

在惯性组件中给定器件数目后，器件的配置方案的好坏可用器件的测量误差对沿参考正交系轴的角速度（或比力）估值的影响和故障检测的能力来评价。对此可建立一个指标函数，优化这个函数就可确定器件的配置角度。这方面的内容可参考文献[113,114]，这里只给出一些结果。

惯性器件的输出方程为

$$
m = H \omega + \varepsilon\tag{7.4.1}
$$

式中： $m \in R^{n}$ 是惯性器件输出； $\omega \in R^{3}$ 是机体的三维角速率；H 为 $n \times 3$ 的配置矩阵。惯性器件的配置方案不同，H 阵的形式也不同。表 7.4.1 给出单轴惯性器件的配置矩阵 H。它们相应的配置结构由图 7.4.1～图 7.4.5 所示，图中 $h_{i}(i=1,2,\cdots,6)$ 表示测量轴的方向。

表 7.4.1 单轴惯性器件的配置矩阵 $H$

<table><tr><td>n</td><td colspan="101">配置矩阵H</td></tr><tr><td>3</td><td colspan="100">H= [1 0 0]</td><td><img src="images/20e9eccd36bf081e68456912ead936933e2ab6b9657787c63af7cdd0efde953f.jpg"/>图7.4.1 三单轴仪表正交配置</td></tr><tr><td rowspan="5">4</td><td colspan="100">H= [1 0 0 s]T</td><td></td></tr><tr><td colspan="88">H= [1 0 0 s]T</td><td><img src="images/5101920838dc5e2ba48fb699cd703aff97a01c037e171034b4a2f8b12171e797.jpg"/>图7.4.2 四单轴仪表正交斜置配置</td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td></tr><tr><td colspan="82">H= [s -s -s s]T</td><td><img src="images/caf8753726d8204393b8708e810dfae9a8368fb99afd7a36f49f72a9eaa68db1.jpg"/>图7.4.3 四单轴仪表对称配置</td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td></tr><tr><td colspan="80">H= [s -s -s s]T</td><td>图7.4.3 四单轴仪表对称配置</td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td></tr><tr><td colspan="71">H= [s -s -s s]T</td><td>图7.4.3 四单轴仪表对称配置</td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td></tr></table>

续表

<table><tr><td>n</td><td>配置矩阵H</td><td>配置结构图</td></tr><tr><td>5</td><td> $s = \sin\alpha$  $c = \cos\alpha$  $H = \begin{bmatrix} s & sc & -cb & -cb & sc \\ 0 & ss & sb & -sb & -ss \\ c & c & c & c & c \end{bmatrix}^{\mathrm{T}}$  $sc = \sin\alpha \cos b$  $ss = \sin\alpha \sin b$  $cb = \sin\alpha \cos(b/2)$  $sb = \sin\alpha \sin(b/2)$  $\alpha = \arcsin\sqrt{2/3}, b = 72^\circ$ </td><td><img src="images/2ef46d6d2d7dcf77834472567d24ebcf5c374f0191a7af01dfffc3fefdd34b19.jpg"/>图7.4.4五单轴仪表对称配置</td></tr><tr><td>6</td><td> $H = \begin{bmatrix} s & -s & c & c & 0 & 0 \\ 0 & 0 & s & -s & c & c \\ c & c & 0 & 0 & s & -s \end{bmatrix}^{\mathrm{T}}$  $s = \sin\alpha$  $c = \cos\alpha$  $2\alpha = 63^\circ 26'5.8''$ </td><td><img src="images/de88560a1ed2bffca860ddd880391a9055e06e219e4b6b72f843ea601f8f5bdd.jpg"/>图7.4.5六单轴仪表对称配置</td></tr></table>

表 7.4.2 给出了双轴惯性器件的 H 阵。它们相应的配置结构如图 7.4.6～图 7.4.12 所示。图中， $h_{is}$ 表示自旋轴； $h_{ij}$ 表示测量轴， $i(i=1\sim n)$ 表示惯性器件序号， $j(j=1,2)$ 表示两个测量轴。

表 7.4.2 双轴惯性器件的配置矩阵 $H$

<table><tr><td>n</td><td colspan="9">配置矩阵H</td><td>配置结构图</td></tr><tr><td>2</td><td colspan="8"> $H = \begin{bmatrix} 1 & 0 & 0 & c \\ 0 & 1 & c & 0 \\ 0 & 0 & c & c \end{bmatrix}^{\mathrm{T}}$ </td><td> $c = \sqrt{2}/2$ </td><td><img src="images/aae47ea044552e47372775970f21aaddd9d2b873875c521f33b427127117f26e.jpg"/>图7.4.6二双轴仪表正交配置</td></tr><tr><td>3</td><td colspan="8"> $H = \begin{bmatrix} c & c & 0 & 0 & -c & c \\ -c & c & c & c & 0 & 0 \\ 0 & 0 & -c & c & c & c \end{bmatrix}^{\mathrm{T}}$ </td><td> $c = \sqrt{2}/2$ </td><td><img src="images/fe5c60af98ec89e5150df15cdf356774d700fcd35afef6f157d47218cabf4624.jpg"/>图74.7三双轴仪表对称配置</td></tr></table>

续表

<table><tr><td>n</td><td>配置矩阵H</td><td>配置结构图</td></tr><tr><td rowspan="3">4</td><td> $\begin{array}{c} \mathbf{H}=\begin{bmatrix}a & b & -b & -a & -a & -b & b & a \\b & a & a & b & -b & -a & -a & -b \\c & c & c & c & c & c & c & c\end{bmatrix}^{\mathrm{T}} \\c=-1/\sqrt{3}\\a=-(1-c)/2\\b=-(1+c)/2\end{array}$ </td><td><img src="images/edc63e4beb01dd64a7a9fe7db6b98f8fb78143d2d5675114dc9749ff44047f49.jpg"/>图7.4.8 四双轴仪表对称配置</td></tr><tr><td> $\begin{array}{c} \mathbf{H}=\begin{bmatrix}-a & a & 0 & 0 & -b & b & b & -b \\0 & 0 & a & -a & b & -b & b & -b \\a & a & a & a & a & a & a & a\end{bmatrix}^{\mathrm{T}}\\a=1/\sqrt{2}\\b=1/2\end{array}$ </td><td><img src="images/d510b1e0c9e35d3d2b311876512dd6f3aa4ebe6369b36c3aeddaf3142fa47f29.jpg"/>图7.4.9 四双轴仪表共面配置</td></tr><tr><td> $\begin{bmatrix}0 & -1/\sqrt{2} & 1 & 0 & 0 & 1 & 0 & 0 \\0 & -1/\sqrt{2} & 0 & 1 & 0 & 0 & 0 & 1 \\1 & 0 & 0 & 0 & 1 & 0 & 1 & 0\end{bmatrix}^{\mathrm{T}}$ </td><td><img src="images/b517a32f01280c7d275b24d0475ed8185f4a7fd8aa482a70e383880dbe9adb3a.jpg"/><img src="images/f57e4f26bf4dfe876c888bd07d33c397b8da53b0380aa6613f4b70f9b073d46b.jpg"/>图7.4.10 四双轴仪表正交配置自旋轴与测量轴方向</td></tr><tr><td>5</td><td> $H = \begin{bmatrix} 1 & 0 & 0 & 0 & 0 & 1 & 1 & 0 & 0 & a \\ 0 & 1 & 1 & 0 & 0 & 0 & 0 & a & 1 & 0 \\ 0 & 0 & 0 & 1 & 1 & 0 & 0 & a & 0 & a \end{bmatrix}^{T}$  $a = 1/\sqrt{2}$ </td><td><img src="images/0c0448d83c06a04b8780ed00f71fe54791f993d89e712e2a7d44f823f156b9da.jpg"/><img src="images/26cae6c85c8113cc3be21f3943d0095b68aa1ac5ddd9b766e8f65665b8074fbc.jpg"/>图7.4.11五双轴仪表正交配置自旋轴与测量轴方向</td></tr><tr><td>6</td><td> $H = \begin{bmatrix} 1 & 0 & 0 & 0 & 0 & 1 & 1 & 0 & 0 & a & 0 & a \\ 0 & 1 & 1 & 0 & 0 & 0 & 0 & a & 1 & 0 & 0 & a \\ 0 & 0 & 0 & 1 & 1 & 0 & 0 & a & 0 & a & 1 & 0 \end{bmatrix}^{T}$  $a = 1/\sqrt{2}$ </td><td><img src="images/e532ed2857e118c277f6e5e940c24931b7c07d4b2728f89996f1d324b1f04c6d.jpg"/><img src="images/6a1c7b0bef1ea80ad8b1c30b629f8dabe143dd9c0ed249912626a82e1ceb90bc.jpg"/>图7.4.12六双轴仪表正交配置自旋轴与测量轴方向</td></tr></table>

这里以一组6个单自由度陀螺为对象介绍最佳配置方案(这个方案已被Honeywell公司采用并被用于波音777的惯性系统中 $^{[115]}$ )。一组6个单自由度陀螺的最佳配置,如图7.4.5所示。6个陀螺的敏感轴沿十二面体的6个平面的法向定位。这种结构有一个独特的对称性,即所有的陀螺敏感轴都彼此相距一个球面角 $2\alpha=63^{\circ}26'5.8''$ 。每一对陀螺的敏感轴位于参考正交坐标系的一个正交平面内，并与正交轴之间的夹角为 $\alpha$ 。

令被测空间角速度向量为

$$
\boldsymbol {\omega} = \left[ \begin{array}{l l l} \omega_ {X} & \omega_ {Y} & \omega_ {Z} \end{array} \right] ^ {\mathrm{T}}
$$

则由图7.4.5所示的空间几何关系可得6个陀螺所反映出的测量值分别为

$$
\left. \begin{array}{l} m _ {1} = \omega_ {X} \sin \alpha + \omega_ {Z} \cos \alpha \\ m _ {2} = - \omega_ {X} \sin \alpha + \omega_ {Z} \cos \alpha \\ m _ {3} = \omega_ {X} \cos \alpha + \omega_ {Y} \sin \alpha \\ m _ {4} = \omega_ {X} \cos \alpha - \omega_ {Y} \sin \alpha \\ m _ {5} = \omega_ {Y} \cos \alpha + \omega_ {Z} \sin \alpha \\ m _ {6} = \omega_ {Y} \cos \alpha - \omega_ {Z} \sin \alpha \end{array} \right\}\tag{7.4.2}
$$

式 $(7.4.2)$ 可写成

$$
m = H \omega\tag{7.4.3}
$$

式中

$$
\boldsymbol {m} = \left[ \begin{array}{l l l l l l} m _ {1} & m _ {2} & m _ {3} & m _ {4} & m _ {5} & m _ {6} \end{array} \right] ^ {\mathrm{T}}\tag{7.4.4}
$$

$$
\boldsymbol {H} = \left[ \begin{array}{c c c c c c} \sin \alpha & - \sin \alpha & \cos \alpha & \cos \alpha & 0 & 0 \\ 0 & 0 & \sin \alpha & - \sin \alpha & \cos \alpha & \cos \alpha \\ \cos \alpha & \cos \alpha & 0 & 0 & \sin \alpha & - \sin \alpha \end{array} \right] ^ {\mathrm{T}}\tag{7.4.5}
$$

由式(7.4.2)可以看出,每个陀螺可同时测量沿两个机体轴的姿态角速率,而从任意3个陀螺的测量值都可以得到沿3个坐标轴的全姿态信息。由此可见,只要保证3个陀螺正常工作,该余度结构就能提供正确的三轴全姿态信息。

现以这种 6 个单自由度陀螺结构为对象,介绍两种用于余度传感器结构的容错方案。一种适用于检测硬故障,另一种适用于检测软故障。这两种方案同样可用于其他余度传感器结构。

## 7.4.2 惯性器件硬故障检测——直接比较法

## 1. 奇偶方程

我们知道,三维欧氏空间中任意4个向量 $X_{1},X_{2},X_{3},X_{4}$ 之间必存在线性相关关系,即有

$$
a \mathbf {X} _ {1} + b \mathbf {X} _ {2} + c \mathbf {X} _ {3} + d \mathbf {X} _ {4} = \mathbf {0}\tag{7.4.6}
$$

式中 $a, b, c, d$ 是不全为零的实常数。因此，在余度惯性组件中，沿任意4个方向的陀螺（加速度计）的测量值也有这种线性相关的关系。以图7.4.5的正12面体上6轴斜置配置为例，忽略测量误差，令 $s = \sin \alpha, c = \cos \alpha$ ，则由式(7.4.2）的前4式可得

$$
(m _ {1} - m _ {2}) c - (m _ {3} + m _ {4}) s = 0\tag{7.4.7}
$$

上式表明,如果4个陀螺的测量值满足上面的关系,则说明这4个陀螺无故障,否则,当式(7.4.7)不成立时,说明4个陀螺中至少有1个出了故障。这样,故障检测问题便可转换为对线性相关方程式(7.4.7)的逻辑判定问题。这种通过比较各个陀螺测量值来检测故障的方法又称为奇偶检测法,式(7.4.7)的线性相关方程又称为奇偶方程。

类似地,我们可以对6个陀螺中的任意4个的组合列出奇偶方程,其结果列于表7.4.3中。由于测量误差的存在,奇偶方程的左端不是严格为零,用“≈”号代替“=”号。

表 7.4.3 6 个单自由度陀螺系统的奇偶方程

<table><tr><td>序号</td><td>陀螺组合</td><td>奇偶检测方程</td></tr><tr><td>1</td><td>1234</td><td> $(m_1 - m_2)c - (m_3 + m_4)s \approx 0$ </td></tr><tr><td>2</td><td>1235</td><td> $(m_2 + m_3)c - (m_1 + m_5)s \approx 0$ </td></tr><tr><td>3</td><td>1236</td><td> $(m_3 - m_1)c - (m_2 - m_6)s \approx 0$ </td></tr><tr><td>4</td><td>1245</td><td> $(m_4 - m_1)c - (m_2 + m_5)s \approx 0$ </td></tr><tr><td>5</td><td>1246</td><td> $(m_2 + m_4)c - (m_1 - m_6)s \approx 0$ </td></tr><tr><td>6</td><td>1256</td><td> $(m_5 - m_6)c - (m_1 + m_2)s \approx 0$ </td></tr><tr><td>7</td><td>1345</td><td> $(m_4 + m_5)c - (m_1 + m_3)s \approx 0$ </td></tr><tr><td>8</td><td>1346</td><td> $(m_6 - m_3)c + (m_1 + m_4)s \approx 0$ </td></tr><tr><td>9</td><td>1356</td><td> $(m_1 + m_6)c + (m_3 + m_5)s \approx 0$ </td></tr><tr><td>10</td><td>1456</td><td> $(m_5 - m_1)c + (m_4 - m_6)s \approx 0$ </td></tr><tr><td>11</td><td>2345</td><td> $(m_5 - m_3)c + (m_4 - m_2)s \approx 0$ </td></tr><tr><td>12</td><td>2346</td><td> $(m_5 + m_4)c + (m_2 - m_3)s \approx 0$ </td></tr><tr><td>13</td><td>2356</td><td> $(m_2 - m_5)c + (m_3 + m_6)s \approx 0$ </td></tr><tr><td>14</td><td>2456</td><td> $(m_2 + m_6)c + (m_4 - m_5)s \approx 0$ </td></tr><tr><td>15</td><td>3456</td><td> $(m_4 - m_3)c + (m_5 + m_6)s \approx 0$ </td></tr></table>

注：表中 $c = \cos\alpha = 0.85065, s = \sin\alpha = 0.52574$ 。

## 2. 故障检测与隔离

如果把上述每个奇偶检测方程等式右端设置严格二进制的量 $K_{i}(i=1\sim15)$ ，即当第 $i$ 个奇偶方程满足时， $K_{i}=0$ ，反之， $K_{i}=1$ 。根据 $K_{i}$ 的值可建立故障隔离的真值表，见表 7.4.4。如果所有的陀螺工作都正常，则 $K_{1}\sim K_{15}$ 均为零，系统重构的程序元为 $P_{0}$ 。若 1 号陀螺失效，则与 1 号陀螺的量测值 $m_{1}$ 有关的奇偶检测方程均不成立， $K_{1}\sim K_{10}$ 均等于 1；而其他与 $m_{1}$ 无关的奇偶方程 $K_{11}\sim K_{15}$ 均等于零，系统重构的程序元为 $P_{1}$ ，从而可将 1 号陀螺进行故障隔离。其他 1 个或 2 个陀螺失效的情况可依此类推，并示于表 7.4.4 中。若 3 个或 3 个以上的陀螺失效，奇偶检测方程全部为 1，在这种情况下只能进行故障检测，而不能进行故障隔离。

现介绍一种代码方式来对陀螺的故障进行隔离。该方法较真值表简单直观。其主要思想是：用一个六位码来对应由6个陀螺中的4个测量值构成的奇偶检测方程。如第i号陀螺的测量值 $m_{i}(i=1\sim6)$ 出现在奇偶检测方程中，则对应于该奇偶检测方程的六位码的第i位取值为1，否则为0。按此规则可得到15个奇偶检测方程所对应的代码表，见表7.4.5所列。当某些陀螺失效时，在15个奇偶方程中含有失效陀螺测量值的方程就不成立了。通过检测剩余的奇偶方程所对应的六位码各占的值，便可确定究竟是哪些陀螺失效。检测方法是首先将剩余的奇偶方程所对应的六位码分别相加，然后检查所得的各位值。既然失效陀螺的测量值并不会出现在剩余的奇偶方程中，因而在相应代码中的对应位数皆为零。因此，若剩余陀螺代码和的第i位数值为零，则说明第i号陀螺失效了。下面举例说明。

说明： $P_{0} \sim P_{21}$ 能进行故障检测与识别， $P_{22}$ 只能进行故障检测。

表 7.4.4 6 个单自由度陀螺系统故障检测真值表

<table><tr><td rowspan="2">故障陀螺</td><td colspan="15">奇偶检测值</td><td rowspan="2">系统重构程序元</td></tr><tr><td> $K_1$ </td><td> $K_2$ </td><td> $K_3$ </td><td> $K_4$ </td><td> $K_5$ </td><td> $K_6$ </td><td> $K_7$ </td><td> $K_8$ </td><td> $K_9$ </td><td> $K_{10}$ </td><td> $K_{11}$ </td><td> $K_{12}$ </td><td> $K_{13}$ </td><td> $K_{14}$ </td><td> $K_{15}$ </td></tr><tr><td>没有</td><td>0</td><td>0</td><td>0</td><td>0</td><td>0</td><td>0</td><td>0</td><td>0</td><td>0</td><td>0</td><td>0</td><td>0</td><td>0</td><td>0</td><td>0</td><td> $P_0$ </td></tr><tr><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>0</td><td>0</td><td>0</td><td>0</td><td>0</td><td> $P_1$ </td></tr><tr><td>2</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>0</td><td>0</td><td>0</td><td>0</td><td>1</td><td>1</td><td>1</td><td>1</td><td>0</td><td> $P_2$ </td></tr><tr><td>3</td><td>1</td><td>1</td><td>1</td><td>0</td><td>0</td><td>0</td><td>1</td><td>1</td><td>1</td><td>0</td><td>1</td><td>1</td><td>1</td><td>0</td><td>1</td><td> $P_3$ </td></tr><tr><td>4</td><td>1</td><td>0</td><td>0</td><td>1</td><td>1</td><td>0</td><td>1</td><td>1</td><td>0</td><td>1</td><td>1</td><td>1</td><td>0</td><td>1</td><td>1</td><td> $P_4$ </td></tr><tr><td>5</td><td>0</td><td>1</td><td>0</td><td>1</td><td>0</td><td>1</td><td>1</td><td>0</td><td>1</td><td>1</td><td>1</td><td>0</td><td>1</td><td>1</td><td>1</td><td> $P_5$ </td></tr><tr><td>6</td><td>0</td><td>0</td><td>1</td><td>0</td><td>1</td><td>1</td><td>0</td><td>1</td><td>1</td><td>1</td><td>0</td><td>1</td><td>1</td><td>1</td><td>1</td><td> $P_6$ </td></tr><tr><td>1,2</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>0</td><td> $P_7$ </td></tr><tr><td>1,3</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>0</td><td>1</td><td> $P_8$ </td></tr><tr><td>1,4</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>0</td><td>1</td><td>1</td><td> $P_9$ </td></tr><tr><td>1,5</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>0</td><td>1</td><td>1</td><td>1</td><td> $P_{10}$ </td></tr><tr><td>1,6</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>0</td><td>1</td><td>1</td><td>1</td><td>1</td><td> $P_{11}$ </td></tr><tr><td>2,3</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>0</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td> $P_{12}$ </td></tr><tr><td>2,4</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>0</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td> $P_{13}$ </td></tr><tr><td>2,5</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>0</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td> $P_{14}$ </td></tr><tr><td>2,6</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>0</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td> $P_{15}$ </td></tr><tr><td>3,4</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>0</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td> $P_{16}$ </td></tr><tr><td>3,5</td><td>1</td><td>1</td><td>1</td><td>1</td><td>0</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td> $P_{17}$ </td></tr><tr><td>3,6</td><td>1</td><td>1</td><td>1</td><td>0</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td> $P_{18}$ </td></tr><tr><td>4,5</td><td>1</td><td>1</td><td>0</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td> $P_{19}$ </td></tr><tr><td>4,6</td><td>1</td><td>0</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td> $P_{20}$ </td></tr><tr><td>5,6</td><td>0</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td> $P_{21}$ </td></tr><tr><td>3个或3个以上</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td>1</td><td> $P_{22}$ </td></tr></table>

表 7.4.5 对应于奇偶检测方程的代码表

<table><tr><td>奇偶检测方程序号</td><td>代码</td><td>奇偶检测方程序号</td><td>代码</td></tr><tr><td>1(1234)</td><td>111100</td><td>9(1356)</td><td>101011</td></tr><tr><td>2(1235)</td><td>111010</td><td>10(1456)</td><td>100111</td></tr><tr><td>3(1236)</td><td>111001</td><td>11(2345)</td><td>011110</td></tr><tr><td>4(1245)</td><td>110110</td><td>12(2346)</td><td>011101</td></tr><tr><td>5(1246)</td><td>110101</td><td>13(2356)</td><td>011011</td></tr><tr><td>6(1256)</td><td>110011</td><td>14(2456)</td><td>010111</td></tr><tr><td>7(1345)</td><td>101110</td><td>15(3456)</td><td>001111</td></tr><tr><td>8(1346)</td><td>101101</td><td></td><td></td></tr></table>

例 7-1 利用代码和的方法判断:① 1 号陀螺失效;② 1 号陀螺与 3 号陀螺失效,这两种故障情况。

解 ① 1 号陀螺失效。由表 7.4.5 可知, 剩余有效奇偶方程的序号及其对应的六位码分别为

$$
\begin{array}{r l r} & \text {序号} & \text {六位码} \\ & 1 1 & 0 1 1 1 1 0 \\ & 1 2 & 0 1 1 1 0 1 \\ & 1 3 & 0 1 1 0 1 1 \\ & 1 4 & 0 1 0 1 1 1 \\ & 1 5 & + 0 0 1 1 1 1 \\ & & \hline & 0 4 4 4 4 4 \end{array}
$$

六位码之和的第1位为零，说明1号陀螺失效。

② 1号陀螺与3号陀螺失效。由表7.4.5可知，剩余有效奇偶方程的序号及其对应的六位码分别为

<table><tr><td>序号</td><td>六位码</td></tr><tr><td>14</td><td>010111</td></tr></table>

此时的六位码之和的第1位和第3位为零，说明1号陀螺和3号陀螺失效。

由于 3 个陀螺失效后, 所有的奇偶检测方程均不成立, 故该方案只能检测到第 3 个陀螺失效而无法确定究竟是哪些陀螺失效。

一旦确定失效陀螺后，就必须重新组织剩余的正常陀螺以提供正确的量测信息，即进行系统重构，并由剩余的有效奇偶检测方程对剩余的陀螺故障进行检测和隔离。表7.4.6和表 7.4.7 分别列出了 1 个陀螺失效后和 2 个陀螺失效后所有剩余的陀螺组合情况(每一组合对应于一个奇偶检测方程)。

表 7.4.6 1 个陀螺失效后的剩余奇偶检测方程

<table><tr><td>失效陀螺号方程名</td><td>1</td><td>2</td><td>3</td><td>4</td><td>5</td><td>6</td></tr><tr><td rowspan="5">奇偶检测方程</td><td>2345</td><td>1345</td><td>1245</td><td>1235</td><td>1234</td><td>1234</td></tr><tr><td>2346</td><td>1346</td><td>1246</td><td>1236</td><td>1236</td><td>1235</td></tr><tr><td>2356</td><td>1356</td><td>1256</td><td>1256</td><td>1246</td><td>1245</td></tr><tr><td>2456</td><td>1456</td><td>1456</td><td>1356</td><td>1346</td><td>1345</td></tr><tr><td>3456</td><td>3456</td><td>2456</td><td>2356</td><td>2346</td><td>2345</td></tr></table>

表 7.4.7 2 个陀螺失效后的剩余奇偶检测方程

<table><tr><td>失效陀螺</td><td>1,2</td><td>1,3</td><td>1,4</td><td>1,5</td><td>1,6</td><td>2,3</td><td>2,4</td><td>2,5</td><td>2,6</td><td>3,4</td><td>3,5</td><td>3,6</td><td>4,5</td><td>4,6</td><td>5,6</td></tr><tr><td>奇偶检测方程</td><td>3456</td><td>2456</td><td>2356</td><td>2346</td><td>2345</td><td>1456</td><td>1356</td><td>1346</td><td>1345</td><td>1256</td><td>1246</td><td>1245</td><td>1236</td><td>1235</td><td>1234</td></tr></table>

将表 7.4.6 和表 7.4.7 存入计算机中, 表中的每一列对应一个容错模块。陀螺发生故障后, 只须将管理程序切换到相应的模块, 就可对该 6 个单自由度陀螺的余度结构进行容错管理。

这种 6 个单自由度陀螺系统除了具有高可靠性外, 还可以利用多余度传感器提供的重复量测值通过数据处理方法来减少单个陀螺测量误差的影响。当陀螺无故障时, 利用最小二乘原理, 由式(7.4.1) 可给出角速度的最佳估计值

$$
\hat {\boldsymbol {\omega}} = (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {H}) ^ {- 1} \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {m}\tag{7.4.8}
$$

当某个陀螺发生故障时，应进行系统重构，将失效的测量方程从式(7.4.1)中去掉，然后用最小二乘原理计算 $\hat{\omega}$ 。

由此可知,多余度传感器的最佳配置不仅能显著提高系统的可靠性,还可以通过最小二乘数据处理技术提高系统的测量精度,充分显示出多余度捷联系统的优越性。

## 7.4.3 惯性器件的软故障检测——广义似然比法

## 1. 广义似然比检测

假定余度传感器的配置有 m 个传感器, 其量测方程中仅含噪声干扰作用, 即

$$
\mathbf {Z} = \mathbf {H X} + \varepsilon\tag{7.4.9}
$$

式中， $X \in R^{n}$ 是待测的导航状态（加速度或角速率）； $Z \in R^{m}$ 是 m 个传感器的量测值（ $m \geqslant n$ ）；H 是传感器配置的几何矩阵； $\varepsilon$ 是高斯白噪声。

定义如下奇偶向量：

$$
p = V Z\tag{7.4.10}
$$

式中，p 为奇偶向量，V 是待定的满秩矩阵。显然， $p=VHX+V\varepsilon$ 。为使其独立于待测状态 X 而仅与噪声或可能的故障有关，应使

$$
V H = 0\tag{7.4.11}
$$

将式 $(7.4.11)$ 代入式 $(7.4.10)$ ，得

$$
p = V \varepsilon\tag{7.4.12}
$$

传感器无故障时,奇偶向量仅是噪声的函数。而当传感器发生偏差故障时,量测方程变为

$$
\mathbf {Z} = \mathbf {H X} + \mathbf {b} _ {f} + \varepsilon\tag{7.4.13}
$$

式中 $b_{f}$ 是故障向量，其对应于有故障的传感器的元不为零，其他元均为零。同理可得

$$
\boldsymbol {p} = \boldsymbol {V} \boldsymbol {\varepsilon} + \boldsymbol {V} \boldsymbol {b} _ {f}\tag{7.4.14}
$$

这时奇偶向量不仅与噪声有关，还与故障有关。正是由于奇偶向量在有故障和无故障情况下表现出来的不一致性，为故障检测提供了基础。

下面介绍基于广义似然比检测(GLT)的故障检测及隔离(FDI)方法。假定高斯随机向量 $\varepsilon$ 有以下统计特性：

$$
E \{\boldsymbol {\varepsilon} \} = 0, \quad E \{\boldsymbol {\varepsilon} \boldsymbol {\varepsilon} ^ {\mathrm{T}} \} = \boldsymbol {\sigma} ^ {2} \boldsymbol {I}\tag{7.4.15}
$$

由式(7.4.12)和式(7.4.14)可知,奇偶向量p在无故障假设 $H_{0}$ 及有故障假设 $H_{1}$ 情况下的统计特性为

$$
H _ {0}: \quad E \{\boldsymbol {p} \} = \mathbf {0}, \quad E \{\boldsymbol {p p} ^ {\mathrm{T}} \} = \boldsymbol {\sigma} ^ {2} \boldsymbol {V V} ^ {\mathrm{T}}\tag{7.4.16}
$$

$$
H _ {1}: \quad E \{\boldsymbol {p} \} = \boldsymbol {\mu}, \quad E \{(\boldsymbol {p} - \boldsymbol {\mu}) (\boldsymbol {p} - \boldsymbol {\mu}) ^ {\mathrm{T}} \} = \sigma^ {2} \mathbf {V V} ^ {\mathrm{T}}\tag{7.4.17}
$$

式中

$$
\boldsymbol {\mu} = \boldsymbol {V b} _ {f}\tag{7.4.18}
$$

为确定某种假设的合理性,定义如下对数似然比:

$$
\Lambda (\boldsymbol {p}) = \ln \frac {p _ {r} (\boldsymbol {p} / H _ {1})}{p _ {r} (\boldsymbol {p} / H _ {0})}\tag{7.4.19}
$$

式中 $p_{r}(\cdot/\cdot)$ 表示正态条件概率密度函数。由式(7.4.16) 和式(7.4.17) 可得

$$
\Lambda (\boldsymbol {p}) = \frac {1}{2} \left[ \boldsymbol {p} ^ {\mathrm{T}} \left(\boldsymbol {V V} ^ {\mathrm{T}}\right) ^ {- 1} \boldsymbol {p} - (\boldsymbol {p} - \boldsymbol {\mu}) ^ {\mathrm{T}} \left(\boldsymbol {V V} ^ {\mathrm{T}}\right) ^ {- 1} (\boldsymbol {p} - \boldsymbol {\mu}) \right]\tag{7.4.20}
$$

其中 $\mu$ 是未知向量。由式(7.4.20) 可求得其极大似然估计为

$$
\hat {\boldsymbol {\mu}} = \boldsymbol {p}\tag{7.4.21}
$$

将式 $(7.4.21)$ 代入式 $(7.4.20)$ 得

$$
\Lambda (\boldsymbol {p}) = \frac {1}{2} [ \boldsymbol {p} ^ {\mathrm{T}} (\boldsymbol {V V} ^ {\mathrm{T}}) ^ {- 1} \boldsymbol {p} ]\tag{7.4.22}
$$

则故障检测判决函数 $DF_{D}$ 可定义为

$$
D F _ {\mathrm{D}} = \boldsymbol {p} ^ {\mathrm{T}} (\boldsymbol {V V} ^ {\mathrm{T}}) ^ {- 1} \boldsymbol {p}\tag{7.4.23}
$$

故障判决准则为

$$
\left\{ \begin{array}{l l} {\text {若} D F _ {\mathrm{D}} > T _ {\mathrm{D}},} & {\text {判定有故障}} \\ {\text {若} D F _ {\mathrm{D}} \leqslant T _ {\mathrm{D}},} & {\text {判定无故障}} \end{array} \right.\tag{7.4.24}
$$

式中 $T_{\mathrm{D}}$ 是预先设定的门限。由式(7.4.12)可知无故障时 $\pmb{p}$ 为高斯随机向量，若 $\pmb{p} \in \mathbb{R}^k$ ，则 $DF_{\mathrm{D}} \sim \chi^{2}(k)$ 。当给定误警率 $P_{f} = P_{r}[DF_{\mathrm{D}} > T_{\mathrm{D}} / H_{0}] (P_{r}[\cdot]$ 代表概率）时，可由表7.3.1确定 $T_{\mathrm{D}}$ 。检测到故障后就必须进行故障隔离，为此要检验以下 $m$ 个假设：

$H_{i}$ 表示第 i 个传感器发生故障， $i=1,2,\cdots,m$ 。假定第 j 个传感器发生故障，则式(7.4.13)中的故障向量可写成

$$
\boldsymbol {b} _ {f} = \boldsymbol {e} _ {j} f\tag{7.4.25}
$$

式中， $e_{j}$ 是单位向量，它的第 j 个元为 1，其他元为零；f 为故障大小。由式(7.4.18)得

$$
\boldsymbol {\mu} = \mathbf {V b} _ {f} = \mathbf {V e} _ {j} f = f \mathbf {v} _ {j}\tag{7.4.26}
$$

式中 $v_{j}$ 是矩阵 V 的第 j 列。

在假设 $H_{j}$ 的情况下, 奇偶向量 p 的统计特性为

$$
H _ {j}: \quad E \{\boldsymbol {p} \} = f \boldsymbol {v} _ {j}, \quad E \{(\boldsymbol {p} - f \boldsymbol {v} _ {j}) (\boldsymbol {p} - f \boldsymbol {v} _ {j}) ^ {\mathrm{T}} \} = \sigma^ {2} V V ^ {\mathrm{T}}\tag{7.4.27}
$$

此时有关奇偶向量 $p$ 的似然函数为

$$
p _ {r} (\boldsymbol {p} / H _ {j}) = K \exp \left\{- \frac {1}{2 \sigma^ {2}} (\boldsymbol {p} - f \boldsymbol {v} _ {j}) ^ {\mathrm{T}} (\boldsymbol {V V} ^ {\mathrm{T}}) ^ {- 1} (\boldsymbol {p} - f \boldsymbol {v} _ {j}) \right\}\tag{7.4.28}
$$

式中， $K$ 为常数， $f$ 是待估计的故障量大小（标量）。由式(7.4.28)求得 $f$ 的极大似然估计为

$$
\hat {f} = \frac {\boldsymbol {p} ^ {\mathrm{T}} (\boldsymbol {V V} ^ {\mathrm{T}}) ^ {- 1} \boldsymbol {v} _ {j}}{\boldsymbol {v} _ {j} ^ {\mathrm{T}} (\boldsymbol {V V} ^ {\mathrm{T}}) ^ {- 1} \boldsymbol {v} _ {j}}\tag{7.4.29}
$$

将 $\hat{f}$ 代入式(7.4.28)并取对数，再略去一些常数，可得到以下故障判决函数：

$$
D F _ {I _ {j}} = \frac {\left[ \boldsymbol {p} ^ {\mathrm{T}} (\boldsymbol {V V} ^ {\mathrm{T}}) ^ {- 1} \boldsymbol {v} _ {j} \right] ^ {2}}{\boldsymbol {v} _ {j} ^ {\mathrm{T}} (\boldsymbol {V V} ^ {\mathrm{T}}) ^ {- 1} \boldsymbol {v} _ {j}}\tag{7.4.30}
$$

若 $DF_{I_{k}} = \max_{j} DF_{I_{j}} \quad (j = 1, 2, \cdots, m)$ ，则表明第 k 个传感器极有可能已发生软故障。

## 2. 奇偶向量的求取

上述故障检测及隔离算法要求 V 矩阵是已知的，现在介绍 V 矩阵的选择方法。由式(7.4.11)可知，V 矩阵位于 H 的左零空间，因而 V 的秩为 $(m-n)$ ，即 V 至多有 $(m-n)$ 个线性无关行。由此可以选定 V 矩阵的维数为 $(m-n)\times m$ ，于是得到的奇偶方程的个数是 $(m-n)$ 。待定的 V 矩阵有 $(m^{2}-nm)$ 个元，而式(7.4.11)只有 $(nm-n^{2})$ 个等式，故 V 矩阵仍有 $(m-n)^{2}$ 个元可以自由选择。为了简化判决函数，不妨对 V 矩阵增加以下约束，即

$$
\boldsymbol {V} \boldsymbol {V} ^ {\mathrm{T}} = \boldsymbol {I}\tag{7.4.31}
$$

式(7.4.31) 含 $(m-n)^{2}$ 个等式,由于 $VV^{T}$ 是对称阵,故这些等式并非完全是线性无关的。为了完全确定V矩阵,Potter等建议将V矩阵选择为具有正对角元的上三角矩阵,然后通过正交化运算便可完全确定V矩阵。详细算法见文献[116]。

$$
\left\{ \begin{array}{l} \boldsymbol {W} = \boldsymbol {I} - \boldsymbol {H} (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {H}) ^ {- 1} \boldsymbol {H} ^ {\mathrm{T}} = [ w (i, j) ] \\ v ^ {2} (1, 1) = w (1, 1) \\ v (1, j) = w (1, j) / v (1, 1) \quad j = 2, \dots m \\ v ^ {2} (i, i) = w (i, i) - \sum_ {k = 1} ^ {i - 1} v ^ {2} (k, i) \quad i = 2, \dots , m - n \\ v (i, j) = [ w (i, j) - \sum_ {k = 1} ^ {i - 1} v (k, i) v (k, j) ] / v (i, i) \quad i = 2, \dots , m - n; j = i + 1, \dots , m \\ v (i, j) = 0 \quad i = 2, \dots , m - n; j = 1, \dots , i - 1 \end{array} \right.
$$

由上述算法选定 V 矩阵后, 判决函数式(7.4.23) 和式(7.4.30) 可简化为

$$
\left. \begin{array}{l} D F _ {\mathrm{D}} = \boldsymbol {p} ^ {\mathrm{T}} \boldsymbol {p} \\ D F _ {l _ {j}} = \frac {(\boldsymbol {p} ^ {\mathrm{T}} \boldsymbol {v} _ {j}) ^ {2}}{\boldsymbol {v} _ {j} ^ {\mathrm{T}} \boldsymbol {v} _ {j}} \end{array} \right\}\tag{7.4.32}
$$

若 $v_{j}^{T}v_{j}$ 和 j 无关，则式(7.4.32) 可进一步简化为

$$
D F _ {I _ {j}} = (\boldsymbol {P} ^ {\mathrm{T}} \boldsymbol {v} _ {j}) ^ {2}\tag{7.4.33}
$$

这一特性定义为故障的一致可检测性,一般对称配置的传感器几何结构都具有此特性。

## 3. 奇偶向量的补偿

在前面的推导中曾假定量测值中没有传感器误差,但实际情况并非如此。一般情况下,传感器中含有刻度系数误差、安装误差以及常值偏差,这时无故障情况下的量测方程为

$$
\mathbf {Z} = (\mathbf {I} + \mathbf {H} _ {\mathrm{se}}) [ \mathbf {H} + \mathbf {H} _ {\mathrm{me}}) \mathbf {X} + \mathbf {b} _ {1} + \boldsymbol {\varepsilon} _ {1} ]\tag{7.4.34}
$$

式中， $H_{se} \in R^{m \times m}$ 是和刻度系数有关的矩阵； $H_{me} \in R^{m \times n}$ 是和安装误差有关的矩阵； $b_{1}$ 是常值偏置； $\varepsilon_{1}$ 是高斯白噪声。

考虑误差阵 $H_{se}$ 和 $H_{me}$ 的元数量级都较小，故认为 $H_{se}H_{me} \approx 0$ ，则式(7.4.34)变为

$$
\mathbf {Z} = (\mathbf {H} + \mathbf {H} _ {\mathrm{me}} + \mathbf {H} _ {\mathrm{se}} \mathbf {H}) \mathbf {X} + (\mathbf {I} + \mathbf {H} _ {\mathrm{se}}) (\mathbf {b} _ {1} + \boldsymbol {\varepsilon} _ {1})\tag{7.4.35}
$$

令

$$
\left. \begin{array}{l} \boldsymbol {b} = (\boldsymbol {I} + \boldsymbol {H} _ {\mathrm{se}}) \boldsymbol {b} _ {1} \\ \boldsymbol {\varepsilon} = (\boldsymbol {I} + \boldsymbol {H} _ {\mathrm{se}}) \boldsymbol {\varepsilon} _ {1} \\ \boldsymbol {H} _ {m} = \boldsymbol {H} _ {\mathrm{me}} + \boldsymbol {H} _ {\mathrm{se}} \boldsymbol {H} \end{array} \right\}\tag{7.4.36}
$$

则式 $(7.4.35)$ 可写成

$$
\mathbf {Z} = (\mathbf {H} + \mathbf {H} _ {m}) \mathbf {X} + \mathbf {b} + \boldsymbol {\varepsilon}\tag{7.4.37}
$$

再由式(7.4.10)和式(7.4.11)可得到如下奇偶方程：

$$
\boldsymbol {p} = \mathbf {V H} _ {m} \boldsymbol {X} + \mathbf {V b} + \mathbf {V} \varepsilon\tag{7.4.38}
$$

这时奇偶向量 p 是真实状态 X、正常的常值偏置以及测量噪声的函数。当飞行器作机动飞行时，奇偶向量各元的数值将增大，以致使故障判决函数有可能超过给定的门限造成误警。曾有人提出采用随飞行器运动规律变化的动态门限来解决这个问题，但该门限会设置过大，以致无法检测动态时的软故障（即缓慢变化或较小的突变故障）。此外，常值偏置也将降低软故障检测能力。

为了改进 FDI 性能,应试图估计出传感器的所有误差以便得到比较精确的量测方程,使这些误差不再影响奇偶向量。然而,并非所有的误差都是可观的,在此采用估计误差的线性组合方法。这些误差项的线性组合一旦被估计出来,就可排除其对奇偶向量的影响,提高 FDI 的性能。

在无故障发生时，式 $(7.4.38)$ 可改写为

$$
p = A X + b ^ {\prime} + V \varepsilon\tag{7.4.39}
$$

式中 $A=VH_{m}, b'=Vb$ 。当 n=3 时，待估计量是 $(m-3)\times3$ 维矩阵 A 的元和 $b'$ 向量的 $(m-3)$ 个元。将 A 按如下形式排列成一向量：

$$
\boldsymbol {a} = \left[ \begin{array}{l l l l l l} a _ {1 1} & a _ {1 2} & a _ {1 3} & a _ {2 1} & \dots & a _ {l 3} \end{array} \right] ^ {\mathrm{T}}\tag{7.4.40}
$$

式中 $a_{ij}$ 为矩阵 $\mathbf{A}$ 的元， $l = m - 3$ 。因此式(7.4.39)右端的第一项可写成

$$
A X = C a\tag{7.4.41}
$$

式中 C 是 $l \times 3l$ 的矩阵, 定义为

$$
\boldsymbol {C} = \left[ \begin{array}{c c c c} \boldsymbol {X} ^ {\mathrm{T}} & 0 0 0 & \dots & 0 0 0 \\ 0 0 0 & \boldsymbol {X} ^ {\mathrm{T}} & \dots & 0 0 0 \\ \vdots & \vdots & & \vdots \\ 0 0 0 & 0 0 0 & \dots & \boldsymbol {X} ^ {\mathrm{T}} \end{array} \right]\tag{7.4.42}
$$

则式 $(7.4.39)$ 可表示为

$$
p = C a + b ^ {\prime} + V \varepsilon\tag{7.4.43}
$$

式(7.4.43)可看做为量测方程。但式中C是未知的，由于C和真实状态X有关，可用X的估计值

$$
\hat {\boldsymbol {X}} = (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {H}) ^ {- 1} \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {Z}\tag{7.4.44}
$$

代入 C 的表达式得出 C 的估计值 $\hat{C}$ ，将 $\hat{C}$ 代入式 (7.4.43) 得

$$
p = \hat {C} a + b ^ {\prime} + \varepsilon^ {\prime}\tag{7.4.45}
$$

将待估计量组合成一个增广向量 $e$

$$
e = \left[ \begin{array}{c} a \\ b ^ {\prime} \end{array} \right]\tag{7.4.46}
$$

则式 $(7.4.45)$ 可写成

式中

$$
\begin{array}{r} {p = \hat {M} e + \varepsilon^ {\prime}} \\ {\hat {M} = [ \hat {C} I ], \quad \varepsilon^ {\prime} = V \varepsilon} \end{array}\tag{7.4.47}
$$

可将式(7.4.47)看成量测方程,量测值为 p,状态为 e,量测噪声为 $\varepsilon^{\prime}$ 。若 $\varepsilon$ 为白噪声,则 $\varepsilon^{\prime}$ 也为白噪声。要估计 e,还需要 e 的状态方程。但决定误差 e 的状态方程是比较困难的,在文献[117]中假定 e 满足下面的离散马尔可夫过程

$$
\boldsymbol {e} _ {k + 1} = \boldsymbol {\Phi} _ {k} \boldsymbol {e} _ {k} = \boldsymbol {\omega} _ {k}\tag{7.4.48}
$$

式中， $\Phi_{k}$ 为状态转移矩阵， $\omega_{k}$ 为白噪声，方差设为 $Q_{k}, \Phi_{k}$ 和 $Q_{k}$ 应根据具体情况而定。此式可包含一类模型，如随机常数和随机游动模型。当 $a$ 和 $b'$ 都是随机常数时，可令 $\Phi_{k}$ 为单位阵， $\omega_{k}$ 为零。在文献[117]中的仿真用式(7.4.48)的一阶马尔可夫模型，并设相关时间为50分钟。根据状态方程式(7.4.48)和量测方程式(7.4.47)就可应用卡尔曼滤波估计 $e_{k}$ 。在求得估计值 $\hat{e}_{k}$ 后，可用下面方法来补偿奇偶向量

$$
p ^ {*} = p - \hat {M} \hat {e} = \varepsilon^ {\prime} = V \varepsilon\tag{7.4.49}
$$

即经补偿后的奇偶向量 $p^{*}$ 只与噪声有关而与状态 X 无关。

当故障发生时

$$
\boldsymbol {p} ^ {*} = \boldsymbol {V b} _ {f} + \boldsymbol {V \varepsilon}\tag{7.4.50}
$$

此时,故障检测和隔离函数分别为

$$
\left. \begin{array}{l} D F _ {D} ^ {*} = (\boldsymbol {p} ^ {*}) ^ {\mathrm{T}} \boldsymbol {p} ^ {*} \\ D F _ {1 j} ^ {*} = [ (\boldsymbol {p} ^ {*}) ^ {\mathrm{T}} \boldsymbol {v} _ {j} ] ^ {2} \end{array} \right\}\tag{7.4.51}
$$

由于经补偿后的奇偶向量基本上不受器件安装误差和自身误差的影响,因此用它进行故障检测和隔离时对故障比较灵敏,可以检测出软故障。

## 4. 广义似然比检测方法举例

下面以 6 个单自由度陀螺结构为例介绍广义似然比检测容错方案。

对一组6个单自由度陀螺结构，由式(7.4.5)可知其量测矩阵为 $(2\alpha = 63^{\circ}26'5.8'')$

$$
\boldsymbol {H} = \left[ \begin{array}{c c c c c c} 0. 5 2 5 7 3 & - 0. 5 2 5 7 3 & 0. 8 5 0 6 5 & 0. 8 5 0 6 5 & 0 & 0 \\ 0 & 0 & 0. 5 2 5 7 3 & - 0. 5 2 5 7 3 & 0. 8 5 0 6 5 & 0. 8 5 0 6 5 \\ 0. 8 5 0 6 5 & 0. 8 5 0 6 5 & 0 & 0 & 0. 5 2 5 7 3 & - 0. 5 2 5 7 3 \end{array} \right]
$$

由 Potter 的算法可得到相应的 V 矩阵为

$$
\mathbf {V} = \left[ \begin{array}{c c c c c c} 0. 7 0 7 1 1 & - 0. 3 1 6 2 3 & - 0. 3 1 6 2 3 & - 0. 3 1 6 2 3 & - 0. 3 1 6 2 3 & 0. 3 1 6 2 3 \\ 0 & 0. 6 3 2 4 6 & 0. 1 9 5 4 4 & 0. 1 9 5 4 4 & - 0. 5 1 1 6 7 & 0. 5 1 1 6 7 \\ 0 & 0 & 0. 6 0 1 5 0 & - 0. 6 0 1 5 0 & - 0. 3 7 1 7 5 & - 0. 3 7 1 7 5 \end{array} \right]
$$

当只有陀螺 $i(i=1,2,\cdots,6)$ 失效时，剩余的正确量测方程为

$$
\mathbf {Z} ^ {i} = \mathbf {H} ^ {i} \mathbf {X} + \varepsilon^ {i}\tag{7.4.52}
$$

矩阵上标 i 表示是由原矩阵去掉第 i 行后形成的矩阵。此时，相应的 $H^{i} \in R^{5 \times 3}$ 。相应的 $V^{i} \in R^{2 \times 5}$ 。 $V^{i}$ 由 Potter 的算法求得。

若接着陀螺 $j(j=1,2,\cdots,6;j\neq i)$ 失效，则剩余的正确量测方程为

$$
\mathbf {Z} ^ {i j} = \mathbf {H} ^ {i j} \mathbf {X} + \boldsymbol {\varepsilon} ^ {i j}\tag{7.4.53}
$$

矩阵上标 $ij$ 表示是由原矩阵去掉第 $i, j$ 行后形成的矩阵。此时 $\pmb{H}^{ij} \in \mathbf{R}^{4 \times 3}$ ，相应的 $\pmb{V}^{ij} \in \mathbf{R}^{1 \times 4}$ 。同样 $\pmb{V}^{ij}$ 也由 Potter 的算法求得。

在有 3 个陀螺失效后, 奇偶方程就不存在了。这说明对 6 个单自由度陀螺的余度结构, 该容错方案不能检测第 4 个陀螺故障。当 n=3 时, m 个传感器只能检测到 $(m-3)$ 个故障。

表7.4.8和表7.4.9分别给出了1个陀螺和2个陀螺失效后各种情况的V矩阵。表中的每个V矩阵都应对应有一个检测和隔离模块，它们都可预先存入计算机内。一旦某个陀螺失效，管理程序就切换到相应的检测及隔离模块，由此即可完成对该余度传感器结构的容错管理。

表 7.4.8 1 个陀螺失效后各种情况的 $V$ 矩阵

<table><tr><td>失效陀螺号\矩阵</td><td colspan="5"> $V(2 \times 5)$ </td></tr><tr><td rowspan="2">1</td><td>0.623 46</td><td>0.195 44</td><td>0.195 44</td><td>-0.511 67</td><td>0.511 67</td></tr><tr><td>0</td><td>0.601 50</td><td>-0.601 50</td><td>-0.371 57</td><td>-0.371 75</td></tr><tr><td rowspan="2">2</td><td>0.623 46</td><td>-0.195 44</td><td>-0.195 44</td><td>-0.511 67</td><td>0.511 67</td></tr><tr><td>0</td><td>0.601 50</td><td>-0.601 50</td><td>-0.371 57</td><td>-0.371 75</td></tr><tr><td rowspan="2">3</td><td>0.623 46</td><td>-0.195 44</td><td>-0.511 67</td><td>-0.511 67</td><td>0.195 44</td></tr><tr><td>0</td><td>0.601 50</td><td>0.371 57</td><td>-0.371 57</td><td>0.601 50</td></tr><tr><td rowspan="2">4</td><td>0.623 46</td><td>-0.195 44</td><td>-0.511 67</td><td>-0.195 44</td><td>0.511 67</td></tr><tr><td>0</td><td>0.601 50</td><td>0.371 75</td><td>-0.601 50</td><td>0.371 57</td></tr><tr><td rowspan="2">5</td><td>0.623 46</td><td>-0.511 67</td><td>-0.511 67</td><td>-0.195 44</td><td>0.195 44</td></tr><tr><td>0</td><td>0.371 75</td><td>-0.371 75</td><td>0.601 50</td><td>0.601 50</td></tr><tr><td rowspan="2">6</td><td>0.623 46</td><td>-0.511 67</td><td>-0.195 44</td><td>-0.511 67</td><td>-0.195 44</td></tr><tr><td>0</td><td>0.371 75</td><td>0.601 50</td><td>-0.371 57</td><td>-0.601 50</td></tr></table>

## 5. 两种检测方案的比较

直接比较量测值的检测方案(以下简称方案1)和广义似然比检测方案(以下简称方案2)各有其优缺点。

方案 1 的物理概念比较明显,且不要求对量测噪声作任何假设(如方案 2 中的高斯零均值为假设)。但是当余度传感器结构中只有 4 个正常的传感器工作时,方案 1 只能检测到故障而无法识别故障(因为此时所有奇偶检测方程均不成立),并且,方案1不能排除传感器误差对FDI性能的影响,因而它不具备检测软故障的能力。

表 7.4.9 2 个陀螺失效后各种情况的 V 矩阵

<table><tr><td>失效陀螺号\矩阵</td><td colspan="4"> $V(1 \times 4)$ </td></tr><tr><td>1,2</td><td>0.601 50</td><td>-0.601 50</td><td>-0.371 75</td><td>-0.371 75</td></tr><tr><td>1,3</td><td>0.601 50</td><td>0.371 75</td><td>-0.371 75</td><td>0.601 50</td></tr><tr><td>1,4</td><td>0.601 50</td><td>0.371 75</td><td>-0.601 50</td><td>0.371 75</td></tr><tr><td>1,5</td><td>0.371 75</td><td>-0.371 75</td><td>0.601 50</td><td>0.601 50</td></tr><tr><td>1,6</td><td>0.371 75</td><td>0.601 50</td><td>-0.371 75</td><td>-0.601 50</td></tr><tr><td>2,3</td><td>0.371 75</td><td>-0.371 75</td><td>-0.601 50</td><td>0.371 75</td></tr><tr><td>2,4</td><td>0.601 50</td><td>-0.371 75</td><td>-0.371 75</td><td>0.601 50</td></tr><tr><td>2,5</td><td>0.601 50</td><td>-0.601 50</td><td>0.371 75</td><td>0.601 50</td></tr><tr><td>2,6</td><td>0.371 75</td><td>0.371 75</td><td>-0.601 50</td><td>-0.601 50</td></tr><tr><td>3,4</td><td>0.371 75</td><td>0.371 75</td><td>-0.601 50</td><td>0.601 50</td></tr><tr><td>3,5</td><td>0.371 75</td><td>-0.601 50</td><td>-0.601 50</td><td>-0.371 75</td></tr><tr><td>3,6</td><td>0.601 50</td><td>-0.371 75</td><td>-0.601 50</td><td>-0.371 75</td></tr><tr><td>4,5</td><td>0.601 50</td><td>-0.371 75</td><td>-0.601 50</td><td>0.371 75</td></tr><tr><td>4,6</td><td>0.371 75</td><td>-0.601 50</td><td>-0.601 50</td><td>0.371 75</td></tr><tr><td>5,6</td><td>0.601 50</td><td>-0.601 50</td><td>-0.371 75</td><td>-0.371 75</td></tr></table>

方案 2 虽然对噪声作了一定的假设,但这种假设在大多数情况下是合理的。由于故障检测及隔离运算是针对同样的奇偶方程,因而能检测到故障就能识别故障。此时,由于方案 2 可对奇偶向量进行补偿,以排除传感器误差对它的影响,因而它具备较强的软故障检测能力。由此可见,方案 2 具有更广泛的适用性。

以上介绍的两种容错方案均没有考虑 2 个以上的传感器同时发生故障的情况。一般都认为这种情况出现的概率极小，故不予考虑。但实际上并不能排除其发生的可能性，因而有必要开展这方面的研究工作。

## 7.5 故障的统计检测原理与风险分析

如何判断系统是否发生了故障(检测问题)和哪个元部件发生了故障(隔离或识别问题),需要根据系统的某些观测信息作出判决。而信息常常混杂有干扰,它是随机的,基于随机量所作的判决属于统计判决或统计检测的范畴。统计检测理论已渗透到诊断、通讯、导航、控制、雷达、地震预报和模式识别等领域,已成为这些学科的一门基础理论。

统计检测可归结为假设检验问题。例如有故障和无故障可作为两种假设，判断哪个假设为真，即是二元假设检验问题。有时也要用到多元假设检验，即判断多个假设中哪一个为真。

例如，有 m 个传感器，假设其中任一个有故障，就构成 m 元假设检验。还有一种复合假设检验，其中表征假设的参数可以在一个范围内变化。例如，故障的幅度和故障发生的时间都可作为表征假设的参数。

## 7.5.1 二元假设检验

设对某事物(元部件、系统等)的状态有两种假设 $H_{0}$ 和 $H_{1}$ ，现要根据 $(0,T)$ 时间内的观测量 $\mathbf{Z}(t)$ 判决 $H_{0}$ 或 $H_{1}$ 为真。当观测量为离散数据时，根据 $\mathbf{Z}(k)(k=1,2,\cdots,n)$ 判决 $H_{0}$ 或 $H_{1}$ 为真。这里观测数据(样本)的数目为 n, n > 1 称为多样本检验，n = 1 称为单样本检验。

在故障检测中， $H_{0}$ 表示无故障， $H_{1}$ 表示有故障。例如， $H_{0}$ 表示观测数据的平均值为零， $H_{1}$ 表示观测数据的平均值不为零。有 4 种可能性：

(1) $H_{0}$ 为真, 判 $H_{1}$ 为真, 这称为误检, 其概率写成 $P_{F}$ ;

(2) $H_{1}$ 为真, 判 $H_{0}$ 为真, 这称为漏检, 其概率写成 $P_{M}$ ;

(3) $H_{0}$ 为真,判 $H_{0}$ 为真,这称为无误检,其概率写成 $1-P_{F}$ ;

(4) $H_{1}$ 为真, 判 $H_{1}$ 为真, 这称为正确检测, 其概率为 $P_{D}=1-P_{M}$ 。

误检概率可定义为

$$
P _ {\mathrm{F}} \stackrel {\mathrm{def}} {=} P (\text {判} H _ {1} \text {真} / H _ {0} \text {真})\tag{7.5.1}
$$

漏检概率可定义为

$$
P _ {\mathrm{M}} \stackrel {\mathrm{def}} {=} P (\text {判} H _ {0} \text {真} / H _ {1} \text {真})\tag{7.5.2}
$$

设观测值 z 构成的观测空间为 Z，将 Z 划分为两个互不相交的子空间 $Z_{0}$ 和 $Z_{1}$

$$
\mathbf {Z} = \mathbf {Z} _ {0} + \mathbf {Z} _ {1}
$$

判决规则是：当 $z \in \mathbf{Z}_0$ 时，判 $H_0$ 为真；当 $z \in \mathbf{Z}_1$ 时，判 $H_1$ 为真，这可用图7.5.1来表示。

设观测值 $z$ 在 $H_0$ 或 $H_{1}$ 为真时的条件概率密度 $p(z / H_0)$ 和 $p(z / H_1)$ 已知，于是结合上面的判决区域可得

$$
P _ {\mathrm{F}} = \int_ {z _ {1}} p (z / H _ {0}) \mathrm{d} z\tag{7.5.3}
$$

$$
P _ {\mathrm{M}} = \int_ {z _ {0}} p (z / H _ {1}) \mathrm{d} z = 1 - \int_ {z _ {1}} p (z / H _ {1}) \mathrm{d} z = 1 - P _ {\mathrm{D}}\tag{7.5.4}
$$

当 z 为标量时， $P_{F}$ 和 $P_{M}$ 可用图 7.5.2 中的阴影来表示。图中 $Z_{T}$ 是观测量的门限值。

![](images/8373e832d52d82170c78f0ad574efa950cbf6b5d2bd694b9b0c06db4072cda1e.jpg)  
图 7.5.1 二元假设检验的判决区域

![](images/fe9e79deb3dc7179c5204347997146a251415e59746c0465b936ede8da79fddb.jpg)  
图7.5.2 $P_{\mathrm{F}}$ 和 $P_{\mathrm{M}}$ 的图示

二元假设检验的判决准则应能产生尽量大的检测概率 $P_{\mathrm{D}}$ 和尽量小的误检概率 $P_{\mathrm{F}}$ ，但这两者是有矛盾的。例如，由式(7.5.3)和式(7.5.4)可知，若取 $Z_{1} = Z$ 则有 $P_{\mathrm{D}} = 1$ ，即有故障时不会漏检,但同时有 $P_{F}=1$ , 即无故障时都误检为有故障。因此, 判决准则的选取应使 $P_{D}$ 和 $P_{F}$ 都获得满意的值, 达到适当的折中。在统计检测理论中已发展了多种判决准则, 下面介绍几种常用的准则。

## 1. 最小误差概率准则

设 $P(H_{0})$ , $P(H_{1})$ 是 $H_{0}$ , $H_{1}$ 分别为真时的先验概率, 对于二元假设检验有

$$
P (H _ {0}) + P (H _ {1}) = 1\tag{7.5.5}
$$

总误差 $P_{e}$ 包含两部分: 出现 $H_{0}$ 时判 $H_{1}$ 为真的错误和出现 $H_{1}$ 时判 $H_{0}$ 为真的错误, 即

$$
P _ {\mathrm{e}} = P (H _ {0}) P _ {\mathrm{F}} + P (H _ {1}) P _ {\mathrm{M}}\tag{7.5.6}
$$

将式 $(7.5.3)$ 和式 $(7.5.4)$ 代入上式，得

$$
\begin{array}{r l} & P _ {\mathrm{e}} = P (H _ {0}) \int_ {z _ {1}} p (z / H _ {0}) \mathrm{d} z + P (H _ {1}) \int_ {z _ {0}} p (z / H _ {1}) \mathrm{d} z = \\ & \quad P (H _ {1}) + \int_ {z _ {1}} \left[ P (H _ {0}) p (z / H _ {0}) - P (H _ {1}) p (z / H _ {1}) \right] \mathrm{d} z \end{array}\tag{7.5.7}
$$

上式第二项与判决区的划分有关,我们应划分判决区使得第二项积分号内的值在 $Z_{1}$ 区内为负(使 $P_{e}$ 减小),而在 $Z_{0}$ 区内则应为正,于是判决准则为

$$
P \left(H _ {0}\right) p \left(\mathbf {z} / H _ {0}\right) - P \left(H _ {1}\right) p \left(\mathbf {z} / H _ {1}\right) \left\{ \begin{array}{l l} > 0, & H _ {0} \\ <   0, & H _ {1} \end{array} \right.\tag{7.5.8}
$$

上式的意义为: 当左端的值大于零时判 $H_{0}$ 为真, 小于零时判 $H_{1}$ 为真。式(7.5.8) 可写成

$$
L (z) \stackrel {\mathrm{def}} {=} \frac {p (z / H _ {1})}{p (z / H _ {0})} \left\{ \begin{array}{l l} > \frac {P (H _ {0})}{P (H _ {1})} \stackrel {\mathrm{def}} {=} T, & H _ {1} \\ <   \frac {P (H _ {0})}{P (H _ {1})} \stackrel {\mathrm{def}} {=} T, & H _ {0} \end{array} \right.\tag{7.5.9}
$$

式中 L 称为判决函数。它是一个似然比，即两个条件概率密度之比。似然比是统计检测理论中一个十分重要的量。T 称为似然比门限，它的值随所用的判决准则的不同而不同。式(7.5.9) 即最小错误概率判决准则。显然一维观测量门限值 $Z_{T}$ 满足

$$
L (Z _ {T}) = \frac {P (H _ {0})}{P (H _ {1})}
$$

(7.5.10)

## 2. 贝叶斯准则(最小风险准则)

在实际问题中,不同类型的错误决策造成的后果是不同的。例如,漏检可能造成机毁人亡,而误检只是造成虚惊而已。因此,对不同类型的错误应给予不同的代价因子。设

$C_{01}$ —— $H_{1}$ 为真判为 $H_{0}$ （漏检）的代价因子；

$C_{10}$ —— $H_{0}$ 为真判为 $H_{1}$ （误检）的代价因子；

$C_{00}$ —— $H_{0}$ 为真判为 $H_{0}$ 的代价因子；

$C_{11}$ —— $H_{1}$ 为真判为 $H_{1}$ 的代价因子。

于是贝叶斯风险(代价函数)R为

$$
R = C _ {0 1} P (H _ {1}) P _ {\mathrm{M}} + C _ {1 0} P (H _ {0}) P _ {\mathrm{F}} + C _ {0 0} P (H _ {0}) (1 - P _ {\mathrm{F}}) + C _ {1 1} P (H _ {1}) (1 - P _ {\mathrm{M}})\tag{7.5.11}
$$

一般情况下，令 $C_{00}=C_{11}=0$ ，即认为正确判断不产生风险，于是贝叶斯风险简化为

$$
R = C _ {0 1} P (H _ {1}) P _ {\mathrm{M}} + C _ {1 0} P (H _ {0}) P _ {\mathrm{F}} =
$$

$$
C _ {1 0} P (H _ {0}) \int_ {\mathbf {z} _ {1}} p (\mathbf {z} / H _ {0}) \mathrm{d} \mathbf {z} + C _ {0 1} P (H _ {1}) \int_ {\mathbf {z} _ {0}} p (\mathbf {z} / H _ {1}) \mathrm{d} \mathbf {z} =
$$

$$
C _ {0 1} P (H _ {1}) + \int_ {\mathbf {z} _ {1}} \left[ C _ {1 0} P (H _ {0}) p (\mathbf {z} / H _ {0}) - C _ {0 1} P (H _ {1}) p (\mathbf {z} / H _ {1}) \right] \mathrm{d} \mathbf {z}
$$

同理,为使风险 R 最小,应使上式右端积分号内的值在 $Z_{1}$ 区内为负,于是得出下面的贝叶斯判决准则

$$
L (z) \stackrel {\mathrm{def}} {=} \frac {p (z / H _ {1})}{p (z / H _ {0})} \left\{ \begin{array}{l l} > \frac {P (H _ {0}) C _ {1 0}}{P (H _ {1}) C _ {0 1}} \stackrel {\mathrm{def}} {=} T, & H _ {1} \\ <   \frac {P (H _ {0}) C _ {1 0}}{P (H _ {1}) C _ {0 1}} \stackrel {\mathrm{def}} {=} T, & H _ {0} \end{array} \right.\tag{7.5.12}
$$

由式(7.5.12)与式(7.5.9)可知，贝叶斯准则与最小错误概率准则的不同处仅在于门限 $T$ 的值不同。当 $C_{01} > C_{10}$ 时，门限将减小，从而使漏检概率减小。

## 3. 最大后验概率准则

后验概率 $P(H_0 / z)$ 和 $P(H_{1} / z)$ 分别是在给定观测量 $\mathbf{z}$ 的条件下 $H_0$ 和 $H_{1}$ 为真的概率。最大后验概率比准则为

$$
\frac {P (H _ {1} / z)}{P (H _ {0} / z)} \left\{ \begin{array}{l l} > 1, & H _ {1} \\ <   1, & H _ {0} \end{array} \right.\tag{7.5.13}
$$

上式的物理意义很明确，在给定观测量后， $H_{1}$ 为真的条件概率大于 $H_{0}$ 为真的条件概率时就判 $H_{1}$ 为真。利用贝叶斯公式，得

$$
\left. \begin{array}{l} P (H _ {1} / z) p (z) = p (z / H _ {1}) P (H _ {1}) \\ P (H _ {0} / z) p (z) = p (z / H _ {0}) P (H _ {0}) \end{array} \right\}\tag{7.5.14}
$$

式中 $p(z)$ 是全概率密度函数，且

$$
p (z) = p (z / H _ {1}) P (H _ {1}) + p (z / H _ {0}) P (H _ {0})\tag{7.5.15}
$$

将式 $(7.5.14)$ 代入式 $(7.5.13)$ ，得

$$
\frac {P \left(H _ {1} / z\right)}{P \left(H _ {0} / z\right)} = \frac {p \left(z / H _ {1}\right) P \left(H _ {1}\right)}{p \left(z / H _ {0}\right) P \left(H _ {0}\right)} \left\{ \begin{array}{l l} > 1, & H _ {1} \\ <   1, & H _ {0} \end{array} \right.
$$

即

$$
L (z) = \frac {p (z / H _ {1})}{p (z / H _ {0})} \left\{ \begin{array}{l l} > \frac {P (H _ {0})}{P (H _ {1})}, & H _ {1} \\ <   \frac {P (H _ {0})}{P (H _ {1})}, & H _ {0} \end{array} \right.\tag{7.5.16}
$$

于是，最大后验概率准则的判决公式与最小错误概率准则的判决公式是相同的。

4. 奈曼-皮尔逊(Neyman-Pearson)准则

在某些情况下，先验概率 $P(H_{0})$ ， $P(H_{1})$ 和代价因子 $C_{01}$ ， $C_{10}$ ， $C_{00}$ ， $C_{11}$ 都不知道，这时可采用奈曼-皮尔逊准则。它假定对一类错误概率要加以限制，在此约束条件下使另一类错误概率最小。在故障检测中，通常约束误检概率，例如

$$
P _ {\mathrm{F}} = \alpha\tag{7.5.17}
$$

在此条件下使漏检概率 $P_{M}$ 最小。

应用拉格朗日乘子 $\lambda > 0$ ，构造目标函数

$$
\begin{array}{r l} & J = P _ {\mathrm{M}} + \lambda (P _ {\mathrm{F}} - \alpha) = \int_ {z _ {0}} p (z / H _ {1}) \mathrm{d} z + \lambda [ \int_ {z _ {1}} p (z / H _ {0}) \mathrm{d} z - \alpha ] = \\ & \quad \lambda (1 - \alpha) + \int_ {z _ {0}} [ p (z / H _ {1}) - \lambda p (z / H _ {0}) ] \mathrm{d} z \end{array}\tag{7.5.18}
$$

因为 $\lambda > 0, \alpha < 1$ ，上式第一项为正，要使 J 最小，必须使积分号内为负，即在 $Z_{0}$ 区内应有

$$
\lambda p (z / H _ {0}) > p (z / H _ {1})
$$

或

$$
L (z) = \frac {p (z / H _ {1})}{p (z / H _ {0})} \left\{ \begin{array}{l l} > \lambda , & H _ {1} \\ <   \lambda , & H _ {0} \end{array} \right.\tag{7.5.19}
$$

上式即奈曼-皮尔逊判决准则。 $\lambda$ 为似然比门限，它的确定方法如下：

设似然比 $L(z)$ 在 $H_0$ 为真的条件下的概率密度为 $p(L / H_0)$ ，则由

$$
\int_ {\lambda} ^ {\infty} p (L / H _ {0}) \mathrm{d} L = P _ {\mathrm{F}} = \alpha\tag{7.5.20}
$$

就可解出 $\lambda$ 。但求 $p(L / H_0)$ 比较麻烦，一维情况下我们可先算出观测量的门限值 $Z_{T}$ ，再来求 $\lambda$ 的值。由

$$
\int_ {Z _ {T}} ^ {\infty} p (z / H _ {0}) \mathrm{d} z = \alpha\tag{7.5.21}
$$

求出 $Z_{T}$ 的值，再由式(7.5.19)可求得

$$
\lambda = \frac {p (Z _ {T} / H _ {1})}{p (Z _ {T} / H _ {0})} = L (Z _ {T})\tag{7.5.22}
$$

例7-2 设某个具有正态分布的随机变量，它的方差 $\sigma^2 = 1$ 。当 $H_0$ 为真时，均值 $m = 0$ ；当 $H_{1}$ 为真时， $m = 1$ 。用奈曼-皮尔逊准则决定检测门限和 $P_{\mathrm{M}}$ ，设 $P_{\mathrm{F}} = 0.1$ 。

解

$$
p (z / H _ {0}) = \frac {1}{\sqrt {2 \pi}} \exp \left[ - \frac {z ^ {2}}{2} \right]
$$

$$
p (z / H _ {1}) = \frac {1}{\sqrt {2 \pi}} \exp \left[ - \frac {(z - 1) ^ {2}}{2} \right]
$$

$$
L (z) = \frac {p (z / H _ {1})}{p (z / H _ {0})} = \exp \left(z - \frac {1}{2}\right)
$$

由奈曼-皮尔逊判决准则得

$$
\exp \left(z - \frac {1}{2}\right) \left\{ \begin{array}{l l} > \lambda , & H _ {1} \\ <   \lambda , & H _ {0} \end{array} \right.
$$

对上式取对数得

$$
\text { 若 } \quad z > \frac {1}{2} + \ln \lambda \stackrel {\mathrm{def}} {=} Z _ {T}, H _ {1} \text { 为真; }
$$

$$
\text { 若 } \quad z <   \frac {1}{2} + \ln \lambda \stackrel {\mathrm{def}} {=} Z _ {T}, H _ {0} \text {   为真。   }
$$

因为

$$
P _ {\mathrm{F}} = \int_ {z _ {T}} ^ {\infty} p (z / H _ {0}) \mathrm{d} z = \int_ {z _ {T}} ^ {\infty} \frac {1}{\sqrt {2 \pi}} \exp \left[ - \frac {z ^ {2}}{2} \right] \mathrm{d} z = 0. 1
$$

由上式查正态分布积分表可得 $z_{T}=1.29$ 。

$$
\lambda = \exp \left(z _ {T} - \frac {1}{2}\right) = \exp \left(1. 2 9 - \frac {1}{2}\right) = 2. 2
$$

$$
P _ {\mathrm{M}} = \int_ {- \infty} ^ {z _ {T}} p [ z / H _ {1} ] \mathrm{d} z = \int_ {- \infty} ^ {1. 2 9} \frac {1}{\sqrt {2 \pi}} \exp \left[ - \frac {(z - 1) ^ {2}}{2} \right] \mathrm{d} z = 0. 3 8 6
$$

$$
P _ {\mathrm{D}} = 1 - P _ {\mathrm{M}} = 0. 6 1 4
$$

检测概率 $P_{D}$ 的值是比较低的。若要提高 $P_{D}$ ，可放松对 $P_{F}$ 的要求，用较大的 $\alpha$ 值，这时 $Z_{T}$ 可变小，从而提高 $P_{D}$ 。

除了最小错误概率准则、贝叶斯准则、最大后验准则、奈曼-皮尔逊准则等几种二元假设的检验准则外，还有一种极小极大准则，这里就不介绍了。我们可看到，尽管各种准则的出发点不同，但最后都可归结为将似然比与一个门限进行比较，以判决 $H_{0}$ 或 $H_{1}$ 为真，而门限的值则随准则的不同而不同。

## 5. 多样本与单样本检验

前面讨论的各种准则都可用多样本或单样本进行检验。这里将介绍用多样本进行检验的表达形式并对比多样本检验与单样本检验的性能。

设某个具有正态分布特征的随机变量有 N 个独立的观测值 $z_{i}(i=1,2,\cdots,N)$ ，它们的方差都为 $\sigma^{2}$ 。当 $H_{0}$ 为真时， $z_{i}$ 的均值为零；当 $H_{1}$ 为真时， $z_{i}$ 的均值为 m。显然

$$
p (z _ {i} / H _ {0}) = \frac {1}{\sqrt {2 \pi} \sigma} \exp \left(- \frac {z _ {i} ^ {2}}{2 \sigma^ {2}}\right)\tag{7.5.23}
$$

$$
p (z _ {i} / H _ {1}) = \frac {1}{\sqrt {2 \pi} \sigma} \exp \left[ \left(- \frac {(z _ {i} - m) ^ {2}}{2 \sigma^ {2}}\right) \right]\tag{7.5.24}
$$

$N$ 个独立的观测值 $z_{i}$ 构成观测向量

$$
\mathbf {z} = \left[ \begin{array}{c c c c} z _ {1} & z _ {2} & \dots & z _ {N} \end{array} \right] ^ {\mathrm{T}}\tag{7.5.25}
$$

由于观测值的独立性,可得

$$
p (z / H _ {0}) = \prod_ {i = 1} ^ {N} \frac {1}{\sqrt {2 \pi} \sigma} \exp \left(\frac {- z _ {i} ^ {2}}{2 \sigma^ {2}}\right)\tag{7.5.26}
$$

$$
p \left(\mathbf {z} / H _ {1}\right) = \prod_ {i = 1} ^ {N} \frac {1}{\sqrt {2 \pi} \sigma} \exp \left[ \left(\frac {- (z _ {i} - m) ^ {2}}{2 \sigma^ {2}}\right) \right]\tag{7.5.27}
$$

$$
\ln L (z) = \ln \frac {p (z / H _ {1})}{p (z / H _ {0})} = \frac {m}{\sigma^ {2}} \sum_ {i = 1} ^ {N} z _ {i} - \frac {N m ^ {2}}{2 \sigma^ {2}} \left\{ \begin{array}{l l} > \ln T, & H _ {1} \\ <   \ln T, & H _ {0} \end{array} \right.\tag{7.5.28}
$$

令

$$
Y = \frac {\sum_ {i = 1} ^ {N} z _ {i}}{N}\tag{7.5.29}
$$

由式(7.5.28)可得

$$
\left. \begin{array}{l} Y > \frac {\sigma^ {2}}{N m} \ln T + \frac {m}{2} \stackrel {\mathrm{def}} {=} Y _ {T}, H _ {1} \\ Y <   \frac {\sigma^ {2}}{N m} \ln T + \frac {m}{2} \stackrel {\mathrm{def}} {=} Y _ {T}, H _ {0} \end{array} \right\}\tag{7.5.30}
$$

上式表明，在作判决时只要将 Y 与新门限 $Y_{T}$ 比较即可，因此 Y 是一个“充分统计量”。所谓充分统计量是指观测值变换后的一个标量值，它包含了作统计决策所需的全部信息。充分统计量是概率统计学中一个重要的概念。

由概率论知,有限个相互独立的正态分布的随机变量之和仍服从正态分布,因此Y服从正态分布。它的均值 $m_{y}$ 和方差 $\sigma_{y}^{2}$ 可计算如下:

当 $H_0$ 为真时，

$$
m _ {y} = 0; \quad \sigma_ {y} ^ {2} = \frac {\sigma^ {2}}{N}
$$

当 $H_{1}$ 为真时，

$$
m _ {y} = m; \quad \sigma_ {y} ^ {2} = \frac {\sigma^ {2}}{N}
$$

故

$$
p (y / H _ {0}) = \frac {\sqrt {N}}{\sqrt {2 \pi} \sigma} \exp \left(- \frac {N y ^ {2}}{2 \sigma^ {2}}\right)
$$

$$
p (y / H _ {1}) = \frac {\sqrt {N}}{\sqrt {2 \pi} \sigma} \exp \left[ - \frac {N (y - m) ^ {2}}{2 \sigma^ {2}} \right]
$$

$$
P _ {\mathrm{F}} = \int_ {Y _ {T}} ^ {\infty} p (y / H _ {0}) \mathrm{d} y = \int_ {Y _ {T}} ^ {\infty} \frac {\sqrt {N}}{\sqrt {2 \pi} \sigma} \exp \left(- \frac {N y ^ {2}}{2 \sigma^ {2}}\right) \mathrm{d} y
$$

作变量置换

$$
x = \frac {\sqrt {N}}{\sigma} y
$$

得

$$
P _ {\mathrm{F}} = \frac {1}{\sqrt {2 \pi}} \int_ {X _ {T}} ^ {\infty} \exp \left(- \frac {x ^ {2}}{2}\right) \mathrm{d} x = 1 - \Phi (X _ {T})\tag{7.5.31}
$$

式中

$$
\left. \begin{array}{l} X _ {T} = \frac {\sqrt {N}}{\sigma} Y _ {T} \\ \Phi (X _ {T}) = \frac {1}{\sqrt {2 \pi}} \int_ {- \infty} ^ {X _ {T}} \exp \left(- \frac {x ^ {2}}{2}\right) \mathrm{d} x \end{array} \right\}\tag{7.5.32}
$$

$$
P _ {\mathrm{M}} = \int_ {- \infty} ^ {Y _ {T}} p (y / H _ {1}) \mathrm{d} y = \int_ {- \infty} ^ {Y _ {T}} \frac {\sqrt {N}}{\sqrt {2 \pi} \sigma} \exp \left[ - \frac {N (y - m) ^ {2}}{2 \sigma^ {2}} \right] \mathrm{d} y
$$

作置换

$$
x ^ {\prime} = \frac {\sqrt {N}}{\sigma} (y - m)
$$

$$
P _ {\mathrm{M}} = \frac {1}{\sqrt {2 \pi}} \int_ {- \infty} ^ {X _ {T}} \exp \left(- \frac {x ^ {' 2}}{2}\right) \mathrm{d} x ^ {'} = \Phi (X _ {T} ^ {'})\tag{7.5.33}
$$

式中

$$
X _ {T} ^ {\prime} = \frac {\sqrt {N}}{\sigma} (Y _ {T} - m)\tag{7.5.34}
$$

查阅正态分布积分表就可算出 $P_{F}$ 和 $P_{M}$ 。

设 $m=1,\sigma=1,T=1$ ，我们来比较 N=2 和 N=1 时的检验质量。先由式(7.5.32)和式(7.5.34)算得门限 $X_{T}$ 和 $X_{T}^{\prime}$ 的值，再由式(7.5.31)和式(7.5.33)算得 $P_{F}$ 和 $P_{M}$ 值。结果如下：

$$
Y _ {T} = 1 / 2
$$

$$
N = 1, \quad X _ {T} = 1 / 2,   X _ {T} ^ {\prime} = - 1 / 2 \qquad P _ {\mathrm{F}} = 0. 3 1, \quad P _ {\mathrm{M}} = 0. 3 1
$$

$$
N = 2, \quad X _ {T} = 1 / \sqrt {2}, \quad X _ {T} ^ {\prime} = - 1 / \sqrt {2} \quad P _ {\mathrm{F}} = 0. 2 4, P _ {\mathrm{M}} = 0. 2 4
$$

可见 N=2 时的 $P_{F}$ 和 $P_{M}$ 的值均比 N=1 时的值小，即多样本的检验质量较高。而单样本检验的优点是存储量小，检验延迟小。

## 7.5.2 多元假设检验

在很多情况下系统可能有多种状态,要检测系统属于哪一种状态就要用多元假设检验。

例如，系统中有 M-1 个传感器，则可有 M 个状态，即所有传感器无故障和 M-1 个传感器中任一个发生故障（假定无两个以上传感器同时故障）。现假设

$H_{0}$ ——所有传感器无故障；

$H_{1}$ ——第一个传感器有故障；

$H_{M-1}$ —— 第 M-1 个传感器有故障。

设 $H_{0}, H_{1}, \cdots, H_{M-1}$ 的先验概率分别为 $P(H_{0}), P(H_{1}), \cdots, P(H_{M-1})$ 则有

$$
\sum_ {i = 0} ^ {M - 1} P (H _ {i}) = 1\tag{7.5.35}
$$

现在的问题是要根据观察量 z 的取值来判决哪个假设为真。z 可以是单样本也可以是多样本。将观测空间 Z 合理地划分出 M 个互不相交的区域（如图 7.5.3 所示）：

$$
\mathbf {Z} = \mathbf {Z} _ {0} + \mathbf {Z} _ {1} + \dots + \mathbf {Z} _ {M - 1}\tag{7.5.36}
$$

![](images/897cc655908ab1137d8169713a15abfc08031fbff0e18441fcbc4b85a0659708.jpg)  
图 7.5.3 多元假设检验的判决区域

贝叶斯风险为

$$
R = \sum_ {i = 0} ^ {M - 1} \sum_ {j = 0} ^ {M - 1} C _ {i j} P (H _ {j}) P (D _ {i} / H _ {j})\tag{7.5.37}
$$

式中 $P(D_{i}/H_{j})$ 表示在 $H_{j}$ 为真的条件下判决 $H_{i}$ 为真的概率。下面来确定多元假设检验的贝叶斯准则。

## 定理7.2

贝叶斯风险为最小的判决等价于下面的判决：

$$
\lambda_ {i} (z) = \sum_ {j = 0} ^ {M - 1} C _ {i j} P (H _ {j}) p (z / H _ {j}) = \min _ {k = 0, 1, \dots , M - 1} \lambda_ {k} (z) \rightarrow \text {判} H _ {i} \text {成立}\tag{7.5.38}
$$

即计算 $\lambda_{k}(z)$ ( $k=0,1,\cdots,M-1$ )，当 $\lambda_{i}(z)$ 最小时就判 $H_{i}$ 为真。

证明

因为

所以

$$
\begin{aligned} P(D_{i} / H_{j}) & = \int_{\mathbf{z}_{i}}p(\mathbf{z} / H_{j})\mathrm{d}\mathbf{z}\\ R & = \sum_{i = 0}^{M - 1}\sum_{j = 0}^{M - 1}C_{ij}P(H_{J})\int_{\mathbf{z}_{i}}p(\mathbf{z} / H_{j})\mathrm{d}\mathbf{z} = \\ & \sum_{i = 0}^{M - 1}C_{ii}P(H_{i})\int_{\mathbf{z}_{i}}p(\mathbf{z} / H_{i})\mathrm{d}\mathbf{z} + \sum_{i = 0}^{M - 1}\sum_{\substack{j = 0\\ j\neq i}}^{M - 1}C_{ij}P(H_{j})\int_{\mathbf{z}_{i}}p(\mathbf{z} / H_{j})\mathrm{d}\mathbf{z} \end{aligned}\tag{7.5.39}
$$

由于

$$
\mathbf {Z} _ {i} = \mathbf {Z} - \sum_ {\substack {j = 0 \\ j \neq i}} ^ {M - 1} \mathbf {Z} _ {j}\tag{7.5.40}
$$

和

故

$$
\begin{aligned} & \int_{\mathbf{z}}p(\mathbf{z} / H_{i})\mathrm{d}\mathbf{z} = 1\\ & R = \sum_{i = 0}^{M - 1}C_{ii}P(H_{i}) + \sum_{i = 0}^{M - 1}\int_{\mathbf{z}_{i}}\sum_{\substack{j = 0\\ j\neq i}}^{M - 1}P(H_{j})(C_{ij} - C_{jj})p(\mathbf{z} / H_{j})\mathrm{d}\mathbf{z} \end{aligned}
$$

上式第一项与判决区划分无关，故 R 最小等价于第二项的被积函数在 $Z_{i}$ 区域内为最小，即

$$
I _ {i} (\mathbf {z}) = \sum_ {\substack {j = 0 \\ j \neq i}} ^ {M - 1} P (H _ {j}) (C _ {i j} - C _ {j j}) p (\mathbf {z} / H _ {j}) = \sum_ {j = 0} ^ {M - 1} P (H _ {j}) (C _ {i j} - C _ {j j}) p (\mathbf {z} / H _ {j}) = \min\tag{7.5.41}
$$

式中包含 $C_{jj}$ 的项对所有 $I_{i}$ 都一样，故上式成立，即

$$
\lambda_ {i} (z) = \sum_ {j = 0} ^ {M - 1} C _ {i j} P (H _ {j}) p (z / H _ {j}) = \lambda_ {k} (z) _ {\min} \quad k = 0, 1, \dots , M - 1\tag{7.5.42}
$$

于是式(7.5.38)成立。作为对上式的一个解释，现将该式除以 $p(z)$ ，并用 $R_{i}(z)$ 表示，即

$$
R _ {i} (z) = \frac {\sum_ {j = 0} ^ {M - 1} C _ {i j} p (z / H _ {j}) P (H _ {j})}{p (z)} = \sum_ {j = 0} ^ {M - 1} C _ {i j} P (H _ {j} / z) = R _ {k} (z) _ {\min}   k = 0, 1, \dots , M - 1 \quad \text {判} H _ {i} \text {成立}\tag{7.5.43}
$$

由于 $R_{i}(z)$ 是给定观测量 $\mathbf{z}$ 之后选择 $H_{i}$ 所付出的代价，所以判决规则的意义是比较直观的，即在给定 $\mathbf{Z}$ 的条件下，哪个 $H_{i}$ 带来的代价小就判决哪个 $H_{i}$ 成立。例如 $M - 1 = 3$ 时，若

$R_0 < R_1, R_0 < R_2 \to$ 判 $H_0$ 真；

$R_{1} < R_{0}, R_{1} < R_{2} \rightarrow$ 判 $H_{1}$ 真；

$$
R _ {2} <   R _ {0},   R _ {2} <   R _ {1} \rightarrow \text { 判 } H _ {2} \text { 真。 }
$$

且

$$
\left\{ \begin{array}{l} C _ {i j} = 1, i \neq j \\ C _ {i j} = 0, i = j \end{array} \right.
$$

则由式(7.5.43)得

或

$$
\begin{aligned} R_{i}(z) = & \sum_{\substack{j = 0\\ j\neq i}}^{M - 1}P(H_{j} / z) = \sum_{j = 0}^{M - 1}P(H_{j} / z) - P(H_{i} / z) = \\ & 1 - P(H_{i} / z) = R_{k}(z)_{\min}\quad k = 0,1,\dots ,M - 1\quad \to \text{判} H_{i}\text{真}\\ & P(H_{i} / z) = P(H_{k} / z)_{\max}\quad k = 0,1,\dots ,M - 1\quad \to \text{判} H_{i}\text{真} \end{aligned}\tag{7.5.44}
$$

此式表明，在上面确定的代价因子的条件下，贝叶斯准则等价于最大后验概率准则。哪个后验概率大就判决哪个假设 $H_{i}$ 成立。

## 7.5.3 复合假设检验

二元和多元假设的前提是事物状态只有有限种可能,但很多情况下事物的状态依赖于某些参数,而参数可以在一个范围内变化。例如,传感器发生偏置故障而故障幅度是一个未知数,它可以在一个范围内变化。再如,故障发生时刻也是未知的,并可在一个范围内变化。含有未知参数的假设检验称为复合假设检验,以区别于参数确定的情况。

设 $\theta$ 为未知参数, 它可以是随机向量且具有概率密度函数, 也可以是非随机的。若 $\theta$ 是非随机变量, 则可先用极大似然估计得出 $\theta$ 的估计值, 设 $H_{0}$ 假设下估值为 $\hat{\theta}_{0}, H_{1}$ 假设下为 $\hat{\theta}_{1}$ 。将估计值代入似然比表达式就可得出所谓广义似然比检验 GLR(Generalized Likelihood Ratio):

$$
L (z) = \frac {p (z / H _ {1} , \hat {\boldsymbol {\theta}} _ {1})}{p (z / H _ {0} , \hat {\boldsymbol {\theta}} _ {0})} = \frac {\underset {\boldsymbol {\theta}} {\max} p (z / H _ {1} , \boldsymbol {\theta})}{\underset {\boldsymbol {\theta}} {\max} p (z / H _ {0} , \boldsymbol {\theta})} \left\{ \begin{array}{l l} > T, & H _ {1} \\ <   T, & H _ {0} \end{array} \right.\tag{7.5.45}
$$

例7-3 在 $H_0$ 假设下观测数据 $z$ 服从均值为零，方差为 $\sigma^2$ 正态分布，在 $H_{1}$ 假设下 $z$ 服从均值为 $m$ 、方差为 $\sigma^2$ 的正态分布。其中 $m$ 可在某范围内取任意值。 $H_{1}$ 含未知参数 $m$ ，因此是复合假设， $H_0$ 则是简单假设。

$$
p (z / H _ {0}) = \frac {1}{\sqrt {2 \pi} \sigma} \exp \left(- \frac {z ^ {2}}{2 \sigma^ {2}}\right)\tag{7.5.46}
$$

$$
p (z / H _ {1}, m) = \frac {1}{\sqrt {2 \pi} \sigma} \exp \left[ - \frac {(z - m) ^ {2}}{2 \sigma^ {2}} \right] \qquad - \infty <   m <   \infty\tag{7.5.47}
$$

由 $\frac{\partial p(z / H_1,m)}{\partial m} = 0$ ，可得估计值 $\hat{m} = z$ ，将此代入似然比式中可得判决准则为

$$
L (z) = \exp \left(\frac {z ^ {2}}{2 \sigma^ {2}}\right) \left\{ \begin{array}{l l} > T, & H _ {1} \\ <   T, & H _ {0} \end{array} \right.\tag{7.5.48}
$$

取对数,可得

$$
\left. \begin{array}{l} z ^ {2} > 2 \sigma^ {2} \ln T, H _ {1} \\ z ^ {2} <   2 \sigma^ {2} \ln T, H _ {0} \end{array} \right\}\tag{7.5.49}
$$

或

$$
\left. \begin{array}{l} | z | > \sigma \sqrt {2 \ln T} \stackrel {\text {def}} {=} Z _ {T}, H _ {1} \\ | z | <   \sigma \sqrt {2 \ln T} \stackrel {\text {def}} {=} Z _ {T}, H _ {0} \end{array} \right\}\tag{7.5.50}
$$

式 $(7.5.50)$ 的门限可由给定误检率 $P_{F}$ 由正态分布决定。

若 $\theta$ 是随机向量，且具有概率密度函数 $p(\boldsymbol{\theta}/H_{0})$ 和 $p(\boldsymbol{\theta}/H_{1})$ 。注意到

$$
p (z / \theta , H _ {1}) p (\theta / H _ {1}) = p (z, \theta / H _ {1})\tag{7.5.51}
$$

$$
\int_ {\Theta} p (z / \theta , H _ {1}) p (\theta / H _ {1}) \mathrm{d} \theta = p (z / H _ {1})\tag{7.5.52}
$$

式中 $\Theta$ 是 $\theta$ 的变化域，则似然比可写成

$$
L (z) = \frac {p (z / H _ {1})}{p (z / H _ {0})} = \frac {\int_ {\Theta} p (z / \theta , H _ {1}) p (\theta / H _ {1}) \mathrm{d} \theta}{\int_ {\Theta} p (z / \theta , H _ {0}) p (\theta / H _ {0}) \mathrm{d} \theta}\tag{7.5.53}
$$

根据似然比可作检验

$$
L (z) \left\{ \begin{array}{l l} > T, & H _ {1} \\ <   T, & H _ {0} \end{array} \right.
$$

似然比门限可由前面讨论的各种准则来决定。这样就将问题化为简单的二元假设问题了。

上面我们讨论了二元假设情况下的复合假设检验问题,对多元假设情况下的复合假设检验也可作类似的讨论。

对动态系统来讲,其输出数据是不断更新的。因此在多样本检测中要采用滑动数据窗检验。设开始时数据窗从 $k-N+1$ 到 k,有 N 个样本,下一时刻数据窗从 $k-N+2$ 到 $k+1$ ,仍为 N 个样本。可以证明,故障发生在数据窗前的漏检概率小于发生在数据窗内的漏检概率,且故障时刻越靠近窗户终端,则漏检概率越大。滑动数据窗检测的具体方法在此就不再叙述了。

