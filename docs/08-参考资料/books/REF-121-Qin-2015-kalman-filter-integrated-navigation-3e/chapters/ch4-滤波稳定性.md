# 第4章 卡尔曼滤波稳定性介绍

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

